/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 * MHU Doorbell + Shared SRAM1 heap test: A32 (responder) <-> M55-HE/HP
 *
 * Linux counterpart of the Zephyr responder
 * (samples/drivers/ipm/ipm_arm_mhu_doorbell/src/a32_to_m55_responder.c).
 * The initiator (m55_to_a32_initiator.c) can run on either M55 core. A32
 * listens on both the HE and HP doorbells for the selected MHU instance
 * and replies on whichever one rang.
 *
 * Protocol:
 *   - A struct sys_heap and the memory it manages both live in shared
 *     SRAM1. The M55 initiator owns the heap (alloc and free); A32 only
 *     reads/writes the data block whose physical address the M55 passes
 *     over the doorbell.
 *   - The initiator allocates a block, writes a payload (magic MAGIC_HE,
 *     on both HE and HP), and rings the doorbell with the block address.
 *   - A32 reads/validates the block, writes its response into the same
 *     block (magic MAGIC_A32, payload echoed), and rings the doorbell
 *     back with the block address. The initiator then frees the block.
 *
 * Cache coherency:
 *   A32 maps the shared SRAM via /dev/mem with O_SYNC (non-cacheable),
 *   so no explicit cache maintenance is required on the A32 side. The
 *   initiator cleans its D-cache after writing and invalidates before
 *   reading.
 *
 * Shared memory layout (struct shared_msg), matching mhu_doorbell_msg.h:
 *   [0x00] magic      - 0xCAFE0E00 from the initiator, 0xA32F055E by A32
 *   [0x04] msg_id     - incrementing message counter
 *   [0x08] data_len   - number of payload bytes (max 240)
 *   [0x0C] data[240]  - payload
 *   [0xFC] checksum   - sum of payload bytes
 *
 * Test flow (per iteration):
 *   1. A32 waits for a doorbell from either M55 core. The value is the
 *      physical address of the initiator's heap block.
 *   2. A32 reads and validates the block via the non-cacheable mapping.
 *   3. A32 writes its response into the same block.
 *   4. A32 rings the doorbell back to that core with the block address.
 *
 * Usage:
 *   ./a32_to_m55_doorbell_responder [--mhu1]
 *   Default uses MHU0. Pass --mhu1 to use MHU1.
 */
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stddef.h>
#include <dirent.h>
#include <poll.h>

struct rpmsg_endpoint_info {
    char name[32];
    uint32_t src;
    uint32_t dst;
};

#define RPMSG_CREATE_EPT_IOCTL _IOW(0xb5, 0x1, struct rpmsg_endpoint_info)
#define RPMSG_DESTROY_EPT_IOCTL _IO(0xb5, 0x2)

#define ITERATIONS       25
#define MAX_ERRORS       10
#define RX_TIMEOUT_MS    5000  /* After the first doorbell; first wait is forever */
#define NPEERS           2

/* RPMSG Device Paths */
#define RPMSG_CTRL0_DEV "/dev/rpmsg_ctrl0"

/* Shared SRAM1 cross-core heap for M55 <-> A32 communication.
 * Must match mhu_doorbell_heap.h (SHARED_HEAP_HEA32_*). HE and HP initiators
 * use the same heap for a given MHU instance (one peer at a time). A32 does
 * not touch the Zephyr heap metadata. The control-block base is page-aligned
 * and the whole span fits in one page, so one /dev/mem mapping covers every
 * block.
 *   MHU0: ctrl 0x027DD000, mem 0x027DD100, span 0x800
 *   MHU1: ctrl 0x027DE000, mem 0x027DE100, span 0x800
 */
#define SHARED_HEAP_BASE_MHU0  0x027DD000UL
#define SHARED_HEAP_BASE_MHU1  0x027DE000UL
#define SHARED_HEAP_SPAN       0x800UL

/* MAGIC_HE is the initiator's wire magic on both cores (see
 * m55_to_a32_initiator.c). MAGIC_HP belongs to the HE<->HP test, not this one.
 */
#define MAGIC_HE         0xCAFE0E00   /* Written by the M55 initiator */
#define MAGIC_A32        0xA32F055E   /* Written by A32 */
#define MAX_PAYLOAD      240

#define MAP_SIZE         4096UL
#define MAP_MASK         (MAP_SIZE - 1)

struct shared_msg {
    uint32_t magic;
    uint32_t msg_id;
    uint32_t data_len;
    uint8_t  data[MAX_PAYLOAD];
    uint32_t checksum;
};

_Static_assert(offsetof(struct shared_msg, checksum) == 0xFC,
               "shared_msg layout must match mhu_doorbell_msg.h");
_Static_assert(sizeof(struct shared_msg) == 0x100,
               "shared_msg size must match mhu_doorbell_msg.h");
_Static_assert((SHARED_HEAP_BASE_MHU0 & (MAP_SIZE - 1)) == 0,
               "MHU0 heap base must be page-aligned");
_Static_assert((SHARED_HEAP_BASE_MHU1 & (MAP_SIZE - 1)) == 0,
               "MHU1 heap base must be page-aligned");
_Static_assert(SHARED_HEAP_SPAN <= MAP_SIZE,
               "heap span must fit in the single mapped page");

/*
 * Doorbell endpoints from the APSS "arm,client" node (ensemble-ex.dtsi).
 * Each name is channel window 0 of that MHU (data-transfer, one word), which
 * is what ipm_send(..., ch_id 0, 4 bytes) on the M55 writes.
 *
 *   MHU0 HP: txdb0 / rxdb0   (&mbox_m55_hp_mhu0_tx / _rx)
 *   MHU1 HP: txdb1 / rxdb1
 *   MHU0 HE: txdb2 / rxdb2   (&mbox_m55_he_mhu0_tx / _rx)
 *   MHU1 HE: txdb3 / rxdb3
 */
struct peer_chan {
    const char *name;
    const char *tx_ept;
    const char *rx_ept;
    int fd_tx;
    int fd_rx;
};

int fd_ctrl = -1;
struct peer_chan peers[NPEERS] = {
    { "M55-HE", NULL, NULL, -1, -1 },
    { "M55-HP", NULL, NULL, -1, -1 },
};
int mem_fd = -1;
void *map_base = MAP_FAILED;

volatile sig_atomic_t stop_flag = 0;

static uint32_t calc_checksum(const uint8_t *buf, uint32_t len)
{
    uint32_t sum = 0;

    for (uint32_t i = 0; i < len; i++) {
        sum += buf[i];
    }
    return sum;
}

static int find_rpmsg_dev(const char *ept_name, char *dev_path, size_t path_len)
{
    DIR *dir;
    struct dirent *entry;
    char sysfs_name_path[sizeof("/sys/class/rpmsg//name") + 256];
    char ept_name_buf[64];
    int sysfs_fd;
    ssize_t bytes_read;

    dir = opendir("/sys/class/rpmsg");
    if (!dir)
        return -1;

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_name[0] == '.')
            continue;

        snprintf(sysfs_name_path, sizeof(sysfs_name_path),
                 "/sys/class/rpmsg/%.255s/name", entry->d_name);

        sysfs_fd = open(sysfs_name_path, O_RDONLY);
        if (sysfs_fd < 0)
            continue;

        bytes_read = read(sysfs_fd, ept_name_buf, sizeof(ept_name_buf) - 1);
        close(sysfs_fd);

        if (bytes_read <= 0)
            continue;

        if (ept_name_buf[bytes_read - 1] == '\n')
            bytes_read--;
        ept_name_buf[bytes_read] = '\0';

        if (strcmp(ept_name_buf, ept_name) == 0) {
            snprintf(dev_path, path_len, "/dev/%.*s",
                     (int)(path_len - sizeof("/dev/")), entry->d_name);
            closedir(dir);
            return 0;
        }
    }

    closedir(dir);
    return -1;
}

static void sigint_handler(int sig_num)
{
    (void)sig_num;
    stop_flag = 1;
}

static int open_endpoint(const char *ept_name)
{
    struct rpmsg_endpoint_info info;
    char dev_path[64];
    int fd;

    memset(&info, 0, sizeof(info));
    strncpy(info.name, ept_name, sizeof(info.name) - 1);
    info.src = 0xFFFFFFFF;
    info.dst = 0xFFFFFFFF;

    printf("Creating endpoint '%s'...\n", info.name);
    if (ioctl(fd_ctrl, RPMSG_CREATE_EPT_IOCTL, &info) == -1) {
        printf("ERROR: IOCTL create '%s' failed: %s\n", ept_name,
               strerror(errno));
        return -1;
    }

    if (find_rpmsg_dev(info.name, dev_path, sizeof(dev_path)) != 0) {
        printf("ERROR: Cannot find rpmsg device for '%s'\n", ept_name);
        return -1;
    }

    fd = open(dev_path, O_RDWR);
    if (fd == -1) {
        printf("ERROR: Cannot open %s: %s\n", dev_path, strerror(errno));
        return -1;
    }

    printf("Opened %s: %s\n", ept_name, dev_path);
    return fd;
}

static void cleanup(void)
{
    if (map_base != MAP_FAILED) {
        munmap(map_base, MAP_SIZE);
        map_base = MAP_FAILED;
    }
    if (mem_fd != -1) {
        close(mem_fd);
        mem_fd = -1;
    }
    for (int i = 0; i < NPEERS; i++) {
        if (peers[i].fd_tx != -1) {
            ioctl(peers[i].fd_tx, RPMSG_DESTROY_EPT_IOCTL);
            close(peers[i].fd_tx);
            peers[i].fd_tx = -1;
        }
        if (peers[i].fd_rx != -1) {
            ioctl(peers[i].fd_rx, RPMSG_DESTROY_EPT_IOCTL);
            close(peers[i].fd_rx);
            peers[i].fd_rx = -1;
        }
    }
    if (fd_ctrl != -1) {
        close(fd_ctrl);
        fd_ctrl = -1;
    }
    printf("\nClosed files, exiting\n");
}

int main(int argc, char *argv[])
{
    int use_mhu1 = 0;

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--mhu1") == 0)
            use_mhu1 = 1;
    }

    unsigned long heap_base = use_mhu1 ? SHARED_HEAP_BASE_MHU1
                                       : SHARED_HEAP_BASE_MHU0;

    /* HE first, then HP. Same HEA32 heap; reply goes back on the peer that rang. */
    if (use_mhu1) {
        peers[0].tx_ept = "txdb3";
        peers[0].rx_ept = "rxdb3";
        peers[1].tx_ept = "txdb1";
        peers[1].rx_ept = "rxdb1";
    } else {
        peers[0].tx_ept = "txdb2";
        peers[0].rx_ept = "rxdb2";
        peers[1].tx_ept = "txdb0";
        peers[1].rx_ept = "rxdb0";
    }

    printf("==========================================\n");
    printf("A32 <-> M55-HE/HP : %s Doorbell + Shared SRAM1 heap (responder)\n",
           use_mhu1 ? "MHU1" : "MHU0");
    printf("  Shared heap ctrl: 0x%08lx (span 0x%lx)\n", heap_base,
           SHARED_HEAP_SPAN);
    printf("==========================================\n");

    /* Map shared SRAM via /dev/mem (O_SYNC => non-cacheable).
     * The doorbell carries a physical block address; translate it into this
     * mapping via (heap_virt + (phys - heap_base)), same as the Zephyr
     * responder's (heap_virt + (rx_addr - SHARED_HEAP_CTRL)).
     */
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) {
        printf("ERROR: Cannot open /dev/mem: %s\n", strerror(errno));
        cleanup();
        return -1;
    }

    map_base = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                    mem_fd, heap_base & ~MAP_MASK);
    if (map_base == MAP_FAILED) {
        printf("ERROR: mmap failed: %s\n", strerror(errno));
        cleanup();
        return -1;
    }
    uint8_t *heap_virt = (uint8_t *)map_base + (heap_base & MAP_MASK);

    /* Open rpmsg control device */
    fd_ctrl = open(RPMSG_CTRL0_DEV, O_RDWR);
    if (fd_ctrl == -1) {
        printf("ERROR: Cannot open %s: %s\n", RPMSG_CTRL0_DEV, strerror(errno));
        cleanup();
        return -1;
    }

    for (int i = 0; i < NPEERS; i++) {
        peers[i].fd_tx = open_endpoint(peers[i].tx_ept);
        if (peers[i].fd_tx == -1) {
            cleanup();
            return -1;
        }
        peers[i].fd_rx = open_endpoint(peers[i].rx_ept);
        if (peers[i].fd_rx == -1) {
            cleanup();
            return -1;
        }
    }

    signal(SIGINT, sigint_handler);

    printf("\nWaiting for M55-HE/HP doorbells...\n\n");

    int completed = 0;
    int errors = 0;

    for (int iter = 0; iter < ITERATIONS && !stop_flag; iter++) {
        struct pollfd pfds[NPEERS];

        for (int i = 0; i < NPEERS; i++) {
            pfds[i].fd = peers[i].fd_rx;
            pfds[i].events = POLLIN;
            pfds[i].revents = 0;
        }

        /* First doorbell: wait for the M55 to boot. Later ones time out. */
        int timeout = (iter == 0) ? -1 : RX_TIMEOUT_MS;
        int poll_ret = poll(pfds, NPEERS, timeout);

        if (poll_ret == 0) {
            printf("ERROR: Timeout waiting for doorbell (%d ms)\n",
                   RX_TIMEOUT_MS);
            break;
        }
        if (poll_ret < 0) {
            if (errno == EINTR && stop_flag)
                break;
            printf("ERROR: poll failed: %s\n", strerror(errno));
            errors++;
            if (errors >= MAX_ERRORS)
                break;
            continue;
        }

        int peer = -1;

        for (int i = 0; i < NPEERS; i++) {
            if (pfds[i].revents & (POLLERR | POLLHUP | POLLNVAL)) {
                printf("ERROR: %s RX endpoint error (revents 0x%x)\n",
                       peers[i].name, pfds[i].revents);
                errors++;
                peer = -2;
                break;
            }
            if (pfds[i].revents & POLLIN) {
                peer = i;
                break;
            }
        }
        if (peer == -2) {
            if (errors >= MAX_ERRORS)
                break;
            continue;
        }
        if (peer < 0)
            continue;

        uint32_t rx_addr;
        int read_bytes = read(peers[peer].fd_rx, &rx_addr, sizeof(rx_addr));

        if (read_bytes < (int)sizeof(rx_addr)) {
            if (read_bytes < 0 && errno == EINTR && stop_flag)
                break;
            printf("ERROR: Read from %s failed (got %d bytes): %s\n",
                   peers[peer].name, read_bytes, strerror(errno));
            errors++;
            if (errors >= MAX_ERRORS)
                break;
            continue;
        }

        printf("A32: Doorbell RX from %s addr=0x%08x\n",
               peers[peer].name, rx_addr);

        if (rx_addr < heap_base ||
            rx_addr + sizeof(struct shared_msg) > heap_base + SHARED_HEAP_SPAN) {
            printf("[%d] ERROR: doorbell addr 0x%08x outside heap "
                   "[0x%08lx, 0x%08lx)\n", iter, rx_addr, heap_base,
                   heap_base + SHARED_HEAP_SPAN);
            errors++;
            if (errors >= MAX_ERRORS)
                break;
            continue;
        }

        volatile struct shared_msg *msg = (volatile struct shared_msg *)
                (heap_virt + (rx_addr - heap_base));

        if (msg->magic != MAGIC_HE) {
            printf("[%d] ERROR: Bad magic 0x%08x (expected 0x%08x)\n",
                   iter, msg->magic, MAGIC_HE);
            errors++;
            if (errors >= MAX_ERRORS)
                break;
            continue;
        }

        uint32_t rx_len = msg->data_len;

        if (rx_len > MAX_PAYLOAD) {
            printf("[%d] ERROR: data_len %u exceeds MAX_PAYLOAD\n",
                   iter, rx_len);
            errors++;
            if (errors >= MAX_ERRORS)
                break;
            continue;
        }

        uint32_t computed = calc_checksum((const uint8_t *)msg->data,
                                          rx_len);
        printf("[%d] %s->A32: block @ 0x%08x msg_id=%u len=%u "
               "cksum=0x%x/0x%x %s\n",
               iter, peers[peer].name, rx_addr, msg->msg_id, rx_len,
               computed, msg->checksum,
               (computed == msg->checksum) ? "PASS" : "FAIL");

        if (computed != msg->checksum) {
            errors++;
            if (errors >= MAX_ERRORS)
                break;
            continue;
        }

        /* --- Write the response into the same block (echo payload) --- */
        msg->magic = MAGIC_A32;
        /* msg_id, data[] and data_len are the M55 payload echoed back. */
        msg->checksum = calc_checksum((const uint8_t *)msg->data, rx_len);

        /* --- Ring doorbell back to the initiator with the block address --- */
        uint32_t tx_addr = rx_addr;
        ssize_t wrote = write(peers[peer].fd_tx, &tx_addr, sizeof(tx_addr));

        if (wrote != (ssize_t)sizeof(tx_addr)) {
            printf("[%d] ERROR: Write (doorbell) to %s failed: %s\n",
                   iter, peers[peer].name, strerror(errno));
            errors++;
            if (errors >= MAX_ERRORS)
                break;
            continue;
        }

        printf("[%d] A32->%s: block @ 0x%08x msg_id=%u len=%u "
               "cksum=0x%x Complete\n\n",
               iter, peers[peer].name, rx_addr, msg->msg_id, rx_len,
               msg->checksum);
        completed++;
    }

    printf("Done: %d/%d passed, %d failed\n", completed, ITERATIONS, errors);
    cleanup();
    return (completed + errors == ITERATIONS) ? 0 : -1;
}
