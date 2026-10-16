/* Copyright (C) 2026 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <dirent.h>

struct rpmsg_endpoint_info {
    char name[32];
    uint32_t src;
    uint32_t dst;
};

#define ITERATIONS 10
#define RPMSG_CREATE_EPT_IOCTL _IOW(0xb5, 0x1, struct rpmsg_endpoint_info)
#define RPMSG_DESTROY_EPT_IOCTL _IO(0xb5, 0x2)
#define RESPONSE_MSG 0xC0FFEE00

/* RPMSG Device Paths */
#define RPMSG_CTRL0_DEV         "/dev/rpmsg_ctrl0"

/*
 * Endpoints for A32 <-> M55 HE (MHU0)
 * txdb2 -> &mbox_m55_he_mhu0_tx
 * rxdb2 -> &mbox_m55_he_mhu0_rx
 */
struct rpmsg_endpoint_info ept_tx_info = {"txdb2", 0xFFFFFFFF, 0xFFFFFFFF};
struct rpmsg_endpoint_info ept_rx_info = {"rxdb2", 0xFFFFFFFF, 0xFFFFFFFF};

int fd_ctrl = -1;
int fd_tx = -1;
int fd_rx = -1;

volatile sig_atomic_t stop_flag = 0;

/**
 * find_rpmsg_dev() - Discover /dev/rpmsgX node by endpoint name via sysfs
 * @ept_name: Endpoint name to match (e.g. "txdb2")
 * @dev_path: Buffer to store discovered device path (e.g. "/dev/rpmsg2")
 * @path_len: Size of dev_path buffer
 *
 * Scans /sys/class/rpmsg/rpmsgX/name to find the device node whose
 * endpoint name matches @ept_name.
 *
 * Return: 0 on success, -1 if no matching device found
 */
static int find_rpmsg_dev(const char *ept_name, char *dev_path, size_t path_len)
{
    DIR *dir;
    struct dirent *entry;
    char sysfs_name_path[256];
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
                 "/sys/class/rpmsg/%.233s/name", entry->d_name);

        sysfs_fd = open(sysfs_name_path, O_RDONLY);
        if (sysfs_fd < 0)
            continue;

        bytes_read = read(sysfs_fd, ept_name_buf, sizeof(ept_name_buf) - 1);
        close(sysfs_fd);

        if (bytes_read <= 0)
            continue;

        /* Remove trailing newline */
        if (ept_name_buf[bytes_read - 1] == '\n')
            bytes_read--;
        ept_name_buf[bytes_read] = '\0';

        if (strcmp(ept_name_buf, ept_name) == 0) {
            snprintf(dev_path, path_len, "/dev/%.58s", entry->d_name);
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

static void cleanup(void)
{
    if (fd_tx != -1) {
        ioctl(fd_tx, RPMSG_DESTROY_EPT_IOCTL);
        close(fd_tx);
        fd_tx = -1;
    }
    if (fd_rx != -1) {
        ioctl(fd_rx, RPMSG_DESTROY_EPT_IOCTL);
        close(fd_rx);
        fd_rx = -1;
    }
    if (fd_ctrl != -1) {
        close(fd_ctrl);
        fd_ctrl = -1;
    }
    printf("\nClosed files, exiting\n");
}

int main(int argc, char *argv[])
{
    int ioctl_ret;
    char rpmsg_dev_path[64];

    printf("==========================================\n");
    printf("A32 Responder <-> M55 HE MHU0 Test\n");
    printf("==========================================\n");

    /* Open control device */
    fd_ctrl = open(RPMSG_CTRL0_DEV, O_RDWR);
    if (fd_ctrl == -1) {
        printf("ERROR: Cannot open %s: %s\n", RPMSG_CTRL0_DEV, strerror(errno));
        return -1;
    }

    /* Create TX endpoint (txdb2) */
    printf("Creating TX endpoint '%s'...\n", ept_tx_info.name);
    ioctl_ret = ioctl(fd_ctrl, RPMSG_CREATE_EPT_IOCTL, &ept_tx_info);
    if (ioctl_ret == -1) {
        printf("ERROR: TX IOCTL failed: %s\n", strerror(errno));
        cleanup();
        return -1;
    }

    /* Create RX endpoint (rxdb2) */
    printf("Creating RX endpoint '%s'...\n", ept_rx_info.name);
    ioctl_ret = ioctl(fd_ctrl, RPMSG_CREATE_EPT_IOCTL, &ept_rx_info);
    if (ioctl_ret == -1) {
        printf("ERROR: RX IOCTL failed: %s\n", strerror(errno));
        cleanup();
        return -1;
    }

    /*
     * Discover TX device via sysfs by matching endpoint name.
     * This avoids opening the wrong /dev/rpmsgX when multiple
     * rpmsg devices exist.
     */
    if (find_rpmsg_dev(ept_tx_info.name, rpmsg_dev_path, sizeof(rpmsg_dev_path)) == 0) {
        fd_tx = open(rpmsg_dev_path, O_RDWR);
        printf("Opened TX: %s (matched '%s')\n", rpmsg_dev_path, ept_tx_info.name);
    } else {
        printf("ERROR: Cannot find rpmsg device for TX endpoint '%s'\n",
               ept_tx_info.name);
        cleanup();
        return -1;
    }

    if (fd_tx == -1) {
        printf("ERROR: Cannot open TX rpmsg device: %s\n", strerror(errno));
        cleanup();
        return -1;
    }

    /* Discover RX device via sysfs */
    if (find_rpmsg_dev(ept_rx_info.name, rpmsg_dev_path, sizeof(rpmsg_dev_path)) == 0) {
        fd_rx = open(rpmsg_dev_path, O_RDWR);
        printf("Opened RX: %s (matched '%s')\n\n", rpmsg_dev_path, ept_rx_info.name);
    } else {
        printf("ERROR: Cannot find rpmsg device for RX endpoint '%s'\n",
               ept_rx_info.name);
        cleanup();
        return -1;
    }

    if (fd_rx == -1) {
        printf("ERROR: Cannot open RX rpmsg device: %s\n", strerror(errno));
        cleanup();
        return -1;
    }

    /* Register signal handler — only sets a flag (async-signal-safe) */
    signal(SIGINT, sigint_handler);

    printf("Waiting for messages...\n\n");

    /* Main loop: wait for message, then send reply (like for_rtss_hp_mhu) */
    int iterator;
    uint32_t rx_word;
    uint32_t tx_payload;
    int read_bytes;

    for (iterator = 0; iterator < ITERATIONS && !stop_flag; iterator++) {
        /* Wait for incoming message (blocking read) */
        read_bytes = read(fd_rx, &rx_word, sizeof(rx_word));
        if (read_bytes < (int)sizeof(rx_word)) {
            printf("ERROR: Read failed (got %d bytes): %s\n",
                   read_bytes, strerror(errno));
            break;
        }
        printf("A32: Received on Ch0 = 0x%x\n", rx_word);
        /* Send reply */
        tx_payload = RESPONSE_MSG;
        printf("A32: Sending 0x%x on Ch0\n", tx_payload);
        if (write(fd_tx, &tx_payload, sizeof(tx_payload)) == -1) {
            printf("ERROR: Write failed: %s\n", strerror(errno));
            break;
        }

        printf("Iteration %d complete\n\n", iterator + 1);
    }

    cleanup();
    return 0;
}
