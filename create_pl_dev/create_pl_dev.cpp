/*
 * SPDX-FileCopyrightText: 2024 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "create_pl_dev"

#include <android-base/unique_fd.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <libdm/dm.h>
#include <log/log.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <memory>
#include <string>

#define BLOCK_SIZE 512

#define EMMC_HSZ 0x800
#define UFS_HSZ 0x1000

#define COMBO_HEADER_SIZE 4
#define UFS_HEADER_SIZE 3

#define UFS_HEADER "UFS"
#define EMMC_HEADER "EMMC"
#define COMBO_HEADER "COMB"

using android::base::unique_fd;
using namespace android::dm;

struct pl_device {
    const char* dm_name;
    const char* dev;
    int lun;
};

static struct pl_device pl_devices[] = {
        {"preloader_raw_a", "/dev/block/sda", 0},
        {"preloader_raw_b", "/dev/block/sdb", 1},
        {"preloader_raw_a", "/dev/block/mmcblk0boot0", -1},
        {"preloader_raw_b", "/dev/block/mmcblk0boot1", -1},
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

static int get_header_size(const char* dev) {
    char header[COMBO_HEADER_SIZE];

    unique_fd fd(open(dev, O_RDONLY));
    if (fd == -1) {
        ALOGE("Failed to open %s: %s.", dev, strerror(errno));
        return 0;
    }

    if (read(fd, header, COMBO_HEADER_SIZE) != COMBO_HEADER_SIZE) {
        ALOGE("Failed to read %s: %s.", dev, strerror(errno));
        return 0;
    }

    if (strncmp(header, UFS_HEADER, UFS_HEADER_SIZE) == 0 ||
        strncmp(header, COMBO_HEADER, COMBO_HEADER_SIZE) == 0) {
        return UFS_HSZ;
    }

    if (strncmp(header, EMMC_HEADER, COMBO_HEADER_SIZE) == 0) {
        return EMMC_HSZ;
    }

    ALOGE("Unknown header %.4s for %s.", header, dev);
    return 0;
}

static std::string get_lun_device(int lun) {
    std::string path = "/sys/bus/scsi/devices/0:0:0:" + std::to_string(lun) + "/block";
    std::unique_ptr<DIR, decltype(&closedir)> dir(opendir(path.c_str()), closedir);
    if (!dir) {
        return "";
    }

    struct dirent* entry;
    while ((entry = readdir(dir.get())) != nullptr) {
        if (entry->d_name[0] == '.') {
            continue;
        }

        std::string dev = std::string("/dev/block/") + entry->d_name;
        if (access(dev.c_str(), F_OK) == 0 && get_header_size(dev.c_str()) != 0) {
            return dev;
        }
    }

    return "";
}

int main() {
    for (int i = 0; i < sizeof(pl_devices) / sizeof(pl_device); i++) {
        pl_device* device = &pl_devices[i];
        std::string dev = device->dev;

        if (device->lun >= 0) {
            std::string lun_dev = get_lun_device(device->lun);
            if (!lun_dev.empty()) {
                dev = lun_dev;
            }
        }

        if (access(dev.c_str(), F_OK) == -1) {
            ALOGE("Device %s not found.", dev.c_str());
            continue;
        }

        int hsz = get_header_size(dev.c_str());
        if (hsz == 0) {
            continue;
        }

        unique_fd fd(open(dev.c_str(), O_RDONLY));
        if (fd == -1) {
            ALOGE("Failed to open %s: %s.", dev.c_str(), strerror(errno));
            continue;
        }

        off_t size = lseek(fd, 0, SEEK_END);
        if (size == -1) {
            ALOGE("Failed to seek %s: %s.", dev.c_str(), strerror(errno));
            continue;
        }

        create_dm_device(device->dm_name, dev.c_str(), hsz / BLOCK_SIZE, (size - hsz) / BLOCK_SIZE);
    }

    return 0;
}
