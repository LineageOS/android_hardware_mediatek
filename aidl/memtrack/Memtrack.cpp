/*
 * SPDX-FileCopyrightText: 2020 The Android Open Source Project
 * SPDX-FileCopyrightText: 2025-2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Memtrack.h"
#include "MemtrackDeviceIon.h"
#include "MemtrackDeviceMali.h"
#include "MemtrackDevicePvr.h"

#include <android-base/logging.h>

#include <fstream>

namespace aidl {
namespace android {
namespace hardware {
namespace memtrack {

bool MemtrackDevice::initializeDevice() {
    for (std::string& path : mMemPaths) {
        std::ifstream ifs(path);
        if (ifs.is_open()) {
            mPath = path;
            return true;
        }
    }
    return false;
}

Memtrack::Memtrack() {
    addDevice<MemtrackDeviceIon>(MemtrackType::GRAPHICS);
    addDevice<MemtrackDeviceMali>(MemtrackType::GL);
    addDevice<MemtrackDevicePvr>(MemtrackType::GL);
}

ndk::ScopedAStatus Memtrack::getMemory(int pid, MemtrackType type,
                                       std::vector<MemtrackRecord>* _aidl_return) {
    MemtrackRecord record = {.flags = 0, .sizeInBytes = 0};

    if (pid < 0) {
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));
    }

    _aidl_return->clear();

    if (type != MemtrackType::OTHER && type != MemtrackType::GL && type != MemtrackType::GRAPHICS &&
        type != MemtrackType::MULTIMEDIA && type != MemtrackType::CAMERA) {
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
    }

    auto it = mDevices.find(type);
    if (it == mDevices.end()) {
        return ndk::ScopedAStatus::ok();
    }

    if (!it->second->getMemory(pid, record)) {
        LOG(ERROR) << "Failed to get memory for pid " << pid;
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));
    }

    _aidl_return->push_back(record);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Memtrack::getGpuDeviceInfo(std::vector<DeviceInfo>* _aidl_return) {
    _aidl_return->clear();
    DeviceInfo dev_info = {.id = 0, .name = "default_gpu"};
    _aidl_return->emplace_back(dev_info);
    return ndk::ScopedAStatus::ok();
}

}  // namespace memtrack
}  // namespace hardware
}  // namespace android
}  // namespace aidl
