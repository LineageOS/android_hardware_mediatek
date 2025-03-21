/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MemtrackDeviceDmaHeap.h"

#include <android-base/logging.h>

#include <fstream>
#include <sstream>

namespace aidl {
namespace android {
namespace hardware {
namespace memtrack {

bool MemtrackDeviceDmaHeap::getMemory(int pid, MemtrackRecord& record) {
    std::ofstream ofs(mPath);

    if (!ofs.is_open()) {
        return false;
    }
    ofs << "pid:" << pid << "\n";
    ofs.close();

    std::ifstream ifs(mPath);
    std::string line;
    unsigned int client_pid;
    int64_t client_pss;
    int64_t client_size;

    if (!ifs.is_open()) {
        return false;
    }

    while (std::getline(ifs, line)) {
        if (line.length() >= 3 && line.substr(0, 3) == "---") {
            break;
        }

        std::istringstream iss(line);
        if (iss >> client_pid >> client_pss >> client_size) {
            if (client_pid == pid) {
                LOG(DEBUG) << "Accounting memory allocated by PID " << pid << ": "
                           << client_size * 1024;
                record.sizeInBytes += client_size * 1024;
                break;
            }
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
