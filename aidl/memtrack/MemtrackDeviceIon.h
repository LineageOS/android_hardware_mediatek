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

class MemtrackDeviceIon : public MemtrackDevice {
  public:
    MemtrackDeviceIon()
        : MemtrackDevice({
                  "/proc/ion/clients/clients_summary",
                  "/sys/kernel/debug/ion/clients/clients_summary",
          }) {}

    bool getMemory(int pid, MemtrackRecord& record) override;
};

}  // namespace memtrack
}  // namespace hardware
}  // namespace android
}  // namespace aidl
