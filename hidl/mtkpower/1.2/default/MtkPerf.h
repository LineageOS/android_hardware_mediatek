/*
 * Copyright (C) 2022 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <vendor/mediatek/hardware/mtkpower/1.2/IMtkPerf.h>
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

struct MtkPerf : public V1_2::IMtkPerf {
    // Methods from ::vendor::mediatek::hardware::mtkpower::V1_0::IMtkPerf follow.
    Return<int32_t> perfLockAcquire(int32_t pl_handle, uint32_t duration, const hidl_vec<int32_t>& boostsList, int32_t reserved) override;
    Return<void> perfLockRelease(int32_t pl_handle, int32_t reserved) override;

    // Methods from ::vendor::mediatek::hardware::mtkpower::V1_1::IMtkPerf follow.
    Return<int32_t> perfCusLockHint(int32_t hint, uint32_t duration) override;

    // Methods from ::vendor::mediatek::hardware::mtkpower::V1_2::IMtkPerf follow.
    Return<int32_t> perfLockReleaseSync(int32_t pl_handle, int32_t reserved) override;
};

}  // namespace vendor::mediatek::hardware::mtkpower::implementation
