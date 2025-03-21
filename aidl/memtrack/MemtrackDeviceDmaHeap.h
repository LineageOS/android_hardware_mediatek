/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "Memtrack.h"

namespace aidl {
namespace android {
namespace hardware {
namespace memtrack {

class MemtrackDeviceDmaHeap : public MemtrackDevice {
  public:
    MemtrackDeviceDmaHeap()
        : MemtrackDevice({
                  "/proc/dma_heap/rss_pid",
          }) {}

    bool getMemory(int pid, MemtrackRecord& record) override;
};

}  // namespace memtrack
}  // namespace hardware
}  // namespace android
}  // namespace aidl
