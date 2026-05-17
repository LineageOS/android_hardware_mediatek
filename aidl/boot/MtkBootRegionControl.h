//
// SPDX-FileCopyrightText: 2022 The Android Open Source Project
// SPDX-FileCopyrightText: The LineageOS Project
// SPDX-License-Identifier: Apache-2.0
//

#pragma once

#include <stdint.h>
#include <memory>

namespace aidl::android::hardware::boot {

class MtkBootRegionControl final {
  public:
    MtkBootRegionControl();
    ~MtkBootRegionControl() = default;

    MtkBootRegionControl(const MtkBootRegionControl&) = delete;
    MtkBootRegionControl& operator=(const MtkBootRegionControl&) = delete;

    bool SetActiveBootSlot(int32_t in_slot);

  private:
    int (*mSetBootPartCb)(int32_t in_slot);
};

}  // namespace aidl::android::hardware::boot
