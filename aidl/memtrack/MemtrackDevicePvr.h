/*
 * SPDX-FileCopyrightText: 2025-2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Memtrack.h"

namespace aidl {
namespace android {
namespace hardware {
namespace memtrack {

class MemtrackDevicePvr : public MemtrackDevice {
  public:
    MemtrackDevicePvr()
        : MemtrackDevice({
                  "/proc/pvr/memtrack_stats",
          }) {}

    bool getMemory(int pid, MemtrackRecord& record) override;
};

}  // namespace memtrack
}  // namespace hardware
}  // namespace android
}  // namespace aidl
