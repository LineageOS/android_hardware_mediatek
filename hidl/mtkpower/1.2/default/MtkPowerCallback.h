/*
 * Copyright (C) 2022 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <vendor/mediatek/hardware/mtkpower/1.2/IMtkPowerCallback.h>
#include <hidl/MQDescriptor.h>
#include <hidl/Status.h>

namespace vendor::mediatek::hardware::mtkpower::implementation {

using ::android::hardware::hidl_array;
using ::android::hardware::hidl_memory;
using ::android::hardware::hidl_string;
using ::android::hardware::hidl_vec;
using ::android::hardware::Return;
using ::android::hardware::Void;
using ::android::sp;

struct MtkPowerCallback : public V1_2::IMtkPowerCallback {
    // Methods from ::vendor::mediatek::hardware::mtkpower::V1_1::IMtkPowerCallback follow.
    Return<void> mtkPowerHint(int32_t hint, int32_t duration) override;
    Return<void> notifyAppState(const hidl_string& pack, const hidl_string& act, int32_t pid, int32_t state, int32_t uid) override;

    // Methods from ::vendor::mediatek::hardware::mtkpower::V1_2::IMtkPowerCallback follow.
    Return<void> notifyScnUpdate(int32_t hint, int32_t data) override;
};

}  // namespace vendor::mediatek::hardware::mtkpower::implementation
