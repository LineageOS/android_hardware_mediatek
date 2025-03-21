/*
 * SPDX-FileCopyrightText: 2020 The Android Open Source Project
 * SPDX-FileCopyrightText: 2025-2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/android/hardware/memtrack/BnMemtrack.h>
#include <aidl/android/hardware/memtrack/DeviceInfo.h>
#include <aidl/android/hardware/memtrack/MemtrackRecord.h>
#include <aidl/android/hardware/memtrack/MemtrackType.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace aidl {
namespace android {
namespace hardware {
namespace memtrack {

static constexpr int kMaxDevices = 5;

class MemtrackDevice {
  public:
    MemtrackDevice(std::vector<std::string> mem_paths) : mMemPaths(mem_paths) {}
    virtual ~MemtrackDevice() = default;

    bool initializeDevice();

    virtual bool getMemory(int pid, MemtrackRecord& record) = 0;

    const std::string& getPath() const { return mPath; }

  protected:
    std::string mPath;

  private:
    std::vector<std::string> mMemPaths;
};

class Memtrack : public BnMemtrack {
  public:
    Memtrack();
    ndk::ScopedAStatus getMemory(int pid, MemtrackType type,
                                 std::vector<MemtrackRecord>* _aidl_return) override;

    ndk::ScopedAStatus getGpuDeviceInfo(std::vector<DeviceInfo>* _aidl_return) override;

  protected:
    template <class MemtrackDeviceType>
    void addDevice(MemtrackType type) {
        int intType = static_cast<int>(type);

        if (mDevices[intType] != nullptr) return;

        std::unique_ptr<MemtrackDeviceType> device = std::make_unique<MemtrackDeviceType>();
        if (device->initializeDevice()) {
            mDevices[intType] = std::move(device);
        } else {
            device.reset();
        }
    }

  private:
    std::array<std::unique_ptr<MemtrackDevice>, kMaxDevices> mDevices;
};

}  // namespace memtrack
}  // namespace hardware
}  // namespace android
}  // namespace aidl
