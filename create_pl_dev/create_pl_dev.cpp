/*
 * SPDX-FileCopyrightText: 2024 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "create_pl_dev"

#include <android-base/unique_fd.h>
#include <errno.h>
#include <fcntl.h>
#include <libdm/dm.h>
#include <log/log.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define BLOCK_SIZE 512

#define EMMC_HSZ 0x800
#define UFS_HSZ 0x1000

#define COMBO_HEADER_SIZE 4
#define UFS_HEADER_SIZE 3

#define UFS_HEADER "UFS"
#define EMMC_HEADER "EMMC"
#define COMBO_HEADER "COMB"

using namespace android::dm;

struct pl_device {
    const char* dm_name;
    const char* dev;
};

static struct pl_device pl_devices[] = {
        {"preloader_raw_a", "/dev/block/sda"},
        {"preloader_raw_b", "/dev/block/sdb"},
        {"preloader_raw_a", "/dev/block/mmcblk0boot0"},
        {"preloader_raw_b", "/dev/block/mmcblk0boot1"},
};

static void create_dm_device(const char* name, const char* dev, int start, int count) {
    DeviceMapper& dm = DeviceMapper::Instance();
    DmTable table;
    std::unique_ptr<DmTarget> target;
    std::string path;

    target = std::make_unique<DmTargetLinear>(0, count, dev, start);
    if (!table.AddTarget(std::move(target))) {
        ALOGE("Failed to add target for %s.", name);
        return;
    }

    if (!dm.CreateDevice(name, table, &path, std::chrono::milliseconds(500))) {
        ALOGE("Failed to create device %s.", name);
        return;
    }

    ALOGI("Created DM device %s at %s.", name, path.c_str());
}

int main() {
    int fd, size, count, start;
    char header[COMBO_HEADER_SIZE];

    for (int i = 0; i < sizeof(pl_devices) / sizeof(pl_device); i++) {
        pl_device* device = &pl_devices[i];

        if (access(device->dev, F_OK) == -1) {
            ALOGE("Device %s not found.", device->dev);
            continue;
        }

        fd = open(device->dev, O_RDONLY);
        if (fd == -1) {
            ALOGE("Failed to open %s: %s.", device->dev, strerror(errno));
            continue;
        }

        size = lseek(fd, 0, SEEK_END);
        if (size == -1) {
            ALOGE("Failed to seek %s: %s.", device->dev, strerror(errno));
            close(fd);
            continue;
        }

        count = size / BLOCK_SIZE;

        if (lseek(fd, 0, SEEK_SET) == -1) {
            ALOGE("Failed to seek %s: %s.", device->dev, strerror(errno));
            close(fd);
            continue;
        }

        if (read(fd, header, COMBO_HEADER_SIZE) != COMBO_HEADER_SIZE) {
            ALOGE("Failed to read %s: %s.", device->dev, strerror(errno));
            close(fd);
            continue;
        }

        close(fd);

        if (strncmp(header, UFS_HEADER, UFS_HEADER_SIZE) == 0 ||
            strncmp(header, COMBO_HEADER, COMBO_HEADER_SIZE) == 0) {
            start = UFS_HSZ / BLOCK_SIZE;
        } else if (strncmp(header, EMMC_HEADER, COMBO_HEADER_SIZE) == 0) {
            start = EMMC_HSZ / BLOCK_SIZE;
        } else {
            ALOGE("Unknown header %s for %s.", header, device->dev);
            continue;
        }

        count -= start;

        create_dm_device(device->dm_name, device->dev, start, count);
    }

    return 0;
}
