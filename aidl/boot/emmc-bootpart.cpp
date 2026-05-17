//
// SPDX-FileCopyrightText: The LineageOS Project
// SPDX-License-Identifier: Apache-2.0
//

#include <android-base/logging.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "emmc-bootpart.h"

#define CMD_ARG(b3, b2, b1, b0) ((b3 << 24) | (b2 << 16) | (b1 << 8) | b0)

namespace aidl::android::hardware::boot {
static int mmc_read_ext_csd(int fd, uint8_t* ext_csd) {
    struct mmc_ioc_cmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    memset(ext_csd, 0, 512);

    cmd.blocks = 1;
    cmd.blksz = 512;
    cmd.opcode = MMC_SEND_EXT_CSD;
    cmd.flags = MMC_CMD_ADTC | MMC_RSP_R1;
    mmc_ioc_cmd_set_data(cmd, ext_csd);

    if (ioctl(fd, MMC_IOC_CMD, &cmd) < 0) {
        LOG(ERROR) << "Failed to read ext csd: " << errno;
        return errno;
    }
    return 0;
}

static inline uint8_t mmc_get_bootpart(uint8_t* ext_csd) {
    return (ext_csd[EXT_CSD_PART_CONFIG] >> 3) & 0x07;
}

static int mmc_switch_bootpart(int fd, uint8_t* ext_csd, uint8_t bootpart) {
    uint8_t arg = (ext_csd[EXT_CSD_PART_CONFIG] & ~(0x38)) | (bootpart << 3);
    struct mmc_ioc_cmd cmd;
    memset(&cmd, 0, sizeof(cmd));

    cmd.opcode = MMC_SWITCH;
    cmd.arg = CMD_ARG(MMC_SWITCH_MODE_WRITE_BYTE, EXT_CSD_PART_CONFIG, arg, EXT_CSD_CMD_SET_NORMAL);
    cmd.flags = MMC_CMD_AC | MMC_RSP_R1B;
    mmc_ioc_cmd_set_data(cmd, ext_csd);

    if (ioctl(fd, MMC_IOC_CMD, &cmd) < 0) {
        LOG(ERROR) << "MMC_IOC_CMD failed: " << errno;
        return errno;
    }

    return 0;
}

int emmc_set_boot_part(int32_t in_slot) {
    uint8_t ext_csd[512];
    uint8_t target_bootpart = (in_slot == 1) ? 2 : 1;
    int fd, ret;

    fd = open(MMCBLK0_DEV, O_RDWR);
    if (fd < 0) {
        LOG(ERROR) << "Failed to open " << MMCBLK0_DEV ": " << errno;
        return errno;
    }

    ret = mmc_read_ext_csd(fd, ext_csd);
    if (ret < 0) goto out;

    if (mmc_get_bootpart(ext_csd) == target_bootpart) {
        ret = 0;
        goto out;
    }

    ret = mmc_switch_bootpart(fd, ext_csd, target_bootpart);
out:
    close(fd);
    return ret;
}

};  // namespace aidl::android::hardware::boot
