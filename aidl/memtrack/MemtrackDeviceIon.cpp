/*
 * SPDX-FileCopyrightText: 2025-2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MemtrackDeviceIon.h"

#include <android-base/logging.h>

#include <fstream>

namespace aidl {
namespace android {
namespace hardware {
namespace memtrack {

bool MemtrackDeviceIon::getMemory(int pid, MemtrackRecord& record) {
    std::ifstream ifs(mPath);
    std::string line, client_name;
    unsigned int client_pid;
    int64_t client_size;

    if (!ifs.is_open()) {
        return false;
    }

    while (std::getline(ifs, line)) {
        std::istringstream iss(line);
        iss >> client_name >> client_pid >> client_size;

        if (client_pid == pid) {
            LOG(DEBUG) << "Accounting memory allocated by PID " << pid << ": " << client_size;
            record.sizeInBytes += client_size;
        }
    }

    LOG(DEBUG) << "Total memory allocated by PID " << pid << ": " << record.sizeInBytes;

    record.flags = MemtrackRecord::FLAG_SMAPS_UNACCOUNTED | MemtrackRecord::FLAG_SHARED |
                   MemtrackRecord::FLAG_SYSTEM | MemtrackRecord::FLAG_NONSECURE;
    return true;
}

}  // namespace memtrack
}  // namespace hardware
}  // namespace android
}  // namespace aidl
