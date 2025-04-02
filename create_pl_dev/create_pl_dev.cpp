/*
 * SPDX-FileCopyrightText: 2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "create_pl_dev"

#include <android-base/unique_fd.h>
#include <android-base/logging.h>
#include <libdm/dm.h>
#include <map>
#include <fstream>
#include <cstring>
#include <iostream>
#include <string>
#include <cerrno>
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

std::map<const char*, const char*> pl_devices = {
    {"preloader_raw_a", "/dev/block/sda"},
    {"preloader_raw_b", "/dev/block/sdb"},
    {"preloader_raw_a", "/dev/block/mmcblk0boot0"},
    {"preloader_raw_b", "/dev/block/mmcblk0boot1"}
};

void create_dm_device(const char* name, const char* dev, int start, int count) {
    DeviceMapper& dm = DeviceMapper::Instance();
    DmTable table;
    std::unique_ptr<DmTarget> target;
    std::string path;

    target = std::make_unique<DmTargetLinear>(0, count, dev, start);
    if (!table.AddTarget(std::move(target))) {
        LOG(ERROR) << "Failed to add target for " << name;
        return;
    }

    if (!dm.CreateDevice(name, table, &path, std::chrono::milliseconds(500))) {
        LOG(ERROR) << "Failed to create device " << name;
        return;
    }

    LOG(INFO) << "Created DM device " << name << " at " << path;
}

int main() {
    std::ifstream file;
    std::streampos size;
    int start, count;
    char header[COMBO_HEADER_SIZE];

    for (const auto& [dm_name, dev] : pl_devices) {
        if (access(dev, F_OK) == -1) {
            LOG(ERROR) << "Device " << dev << " not found.";
            continue;
        }

        file.open(dev, std::ios::binary | std::ios::ate);
        if (!file) {
            LOG(ERROR) << "Failed to open " << dev << ": " << strerror(errno);
            continue;
        }

        std::streampos size = file.tellg();
        if (size == -1) {
            LOG(ERROR) << "Failed to seek " << dev << ": " << strerror(errno);
            continue;
        }

        count = size / BLOCK_SIZE;

        file.seekg(0, std::ios::beg);
        if (!file) {
            LOG(ERROR) << "Failed to seek " << dev << ": " << strerror(errno);
            continue;
        }

        file.read(header, COMBO_HEADER_SIZE);
        if (file.gcount() != COMBO_HEADER_SIZE) {
            LOG(ERROR) << "Failed to read " << dev << ": " << strerror(errno);
            file.close();
            continue;
        }

        file.close();

        if (std::memcmp(header, UFS_HEADER, UFS_HEADER_SIZE) == 0
            || std::memcmp(header, COMBO_HEADER, COMBO_HEADER_SIZE) == 0) {
            start = UFS_HSZ / BLOCK_SIZE;
        } else if (std::memcmp(header, EMMC_HEADER, COMBO_HEADER_SIZE) == 0) {
            start = EMMC_HSZ / BLOCK_SIZE;
        } else {
            LOG(ERROR) << "Unknown header " << header << " for " << dev;
            continue;
        }

        count -= start;
        create_dm_device(dm_name, dev, start, count);
    }

    return 0;
}
