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

class MemtrackDeviceMali : public MemtrackDevice {
  public:
    MemtrackDeviceMali()
        : MemtrackDevice({
                  "/proc/mtk_mali/gpu_memory",
                  "/proc/mali/memory_usage",
          }) {}

    bool getMemory(int pid, MemtrackRecord& record) override;
};

}  // namespace memtrack
}  // namespace hardware
}  // namespace android
}  // namespace aidl
