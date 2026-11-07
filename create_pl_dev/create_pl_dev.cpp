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
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

#define BLOCK_SIZE 512

#define EMMC_HSZ 0x800
#define UFS_HSZ 0x1000

#define COMBO_HEADER_SIZE 4
#define UFS_HEADER_SIZE 3

#define UFS_HEADER "UFS"
#define EMMC_HEADER "EMMC"
#define COMBO_HEADER "COMB"

#define BRLYT_OFFSET 0x200
#define BRLYT_ID "BRLYT"
#define BRLYT_DESC_OFFSET 0x214
#define BRLYT_DESC_COUNT 8
#define BL_EXIST_MAGIC 0x42424242

#define BL_TYPE_PRELOADER 0x1
#define BL_TYPE_ROT_BOOT_FW 0xb
#define BL_TYPE_ROT_BOOT_FMC 0xd

using android::base::unique_fd;
using namespace android::dm;

struct bl_descriptor {
    uint32_t magic;
    uint8_t dev;
    uint8_t reserved;
    uint16_t type;
    uint32_t begin;
    uint32_t boundary;
    uint32_t attr;
} __attribute__((packed));

struct pl_device {
    const char* suffix;
    const char* dev;
    int lun;
};

static struct pl_device pl_devices[] = {
        {"_a", "/dev/block/sda", 0},
        {"_b", "/dev/block/sdb", 1},
        {"_a", "/dev/block/mmcblk0boot0", -1},
        {"_b", "/dev/block/mmcblk0boot1", -1},
};

static void create_dm_device(const std::string& name, const char* dev, uint64_t start,
                             uint64_t count) {
    DeviceMapper& dm = DeviceMapper::Instance();
    DmTable table;
    std::unique_ptr<DmTarget> target;
    std::string path;

    target = std::make_unique<DmTargetLinear>(0, count, dev, start);
    if (!table.AddTarget(std::move(target))) {
        ALOGE("Failed to add target for %s.", name.c_str());
        return;
    }

    if (!dm.CreateDevice(name, table, &path, std::chrono::milliseconds(500))) {
        ALOGE("Failed to create device %s.", name.c_str());
        return;
    }

    ALOGI("Created DM device %s at %s.", name.c_str(), path.c_str());
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

static const char* get_region_name(uint16_t type) {
    switch (type) {
        case BL_TYPE_PRELOADER:
            return "preloader_raw";
        case BL_TYPE_ROT_BOOT_FMC:
            return "rot_boot_fmc";
        case BL_TYPE_ROT_BOOT_FW:
            return "rot_boot_fw";
        default:
            return nullptr;
    }
}

static bool create_region_devices(int fd, const char* dev, const char* suffix, uint64_t size) {
    char id[sizeof(BRLYT_ID)];
    bl_descriptor descs[BRLYT_DESC_COUNT];
    std::vector<bl_descriptor> regions;
    bool has_preloader = false;
    bool has_rot_boot = false;

    if (pread(fd, id, sizeof(id), BRLYT_OFFSET) != sizeof(id) ||
        memcmp(id, BRLYT_ID, sizeof(id)) != 0) {
        return false;
    }

    if (pread(fd, descs, sizeof(descs), BRLYT_DESC_OFFSET) != sizeof(descs)) {
        return false;
    }

    for (const auto& desc : descs) {
        if (desc.magic != BL_EXIST_MAGIC) {
            continue;
        }

        has_preloader |= desc.type == BL_TYPE_PRELOADER;
        has_rot_boot |= desc.type == BL_TYPE_ROT_BOOT_FMC;
        regions.push_back(desc);
    }

    if (!has_preloader || !has_rot_boot) {
        return false;
    }

    for (const auto& region : regions) {
        const char* name = get_region_name(region.type);
        if (name == nullptr) {
            continue;
        }

        uint64_t begin = region.begin;
        uint64_t end = size;
        for (const auto& other : regions) {
            if (other.begin > begin && other.begin < end) {
                end = other.begin;
            }
        }

        if (begin >= end || begin % BLOCK_SIZE || end % BLOCK_SIZE) {
            ALOGE("Invalid %s region on %s.", name, dev);
            continue;
        }

        create_dm_device(std::string(name) + suffix, dev, begin / BLOCK_SIZE,
                         (end - begin) / BLOCK_SIZE);
    }

    return true;
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

        if (create_region_devices(fd, dev.c_str(), device->suffix, size)) {
            continue;
        }

        create_dm_device(std::string("preloader_raw") + device->suffix, dev.c_str(),
                         hsz / BLOCK_SIZE, (size - hsz) / BLOCK_SIZE);
    }

    return 0;
}
