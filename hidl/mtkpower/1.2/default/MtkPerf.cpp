/*
 * Copyright (C) 2022 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vendor.mediatek.hardware.mtkpower@1.2-service.stub"

#include <android-base/logging.h>

#include "MtkPerf.h"

namespace vendor::mediatek::hardware::mtkpower::implementation {

// Methods from ::vendor::mediatek::hardware::mtkpower::V1_0::IMtkPerf follow.
Return<int32_t> MtkPerf::perfLockAcquire(int32_t pl_handle, uint32_t duration, const hidl_vec<int32_t>& /* boostsList */, int32_t reserved) {
    LOG(INFO) << "perfLockAcquire pl_handle: " << pl_handle
              << " duration: " << duration << " reserved: "
              << reserved;
    return pl_handle;
}

Return<void> MtkPerf::perfLockRelease(int32_t pl_handle, int32_t reserved) {
    LOG(INFO) << "perfLockRelease pl_handle: " << pl_handle
              << " reserved: " << reserved;
    return Void();
}


// Methods from ::vendor::mediatek::hardware::mtkpower::V1_1::IMtkPerf follow.
Return<int32_t> MtkPerf::perfCusLockHint(int32_t hint, uint32_t duration) {
    LOG(INFO) << "perfCusLockHint hint: " << hint
              << " duraton: " << duration;
    return 233;
}


// Methods from ::vendor::mediatek::hardware::mtkpower::V1_2::IMtkPerf follow.
Return<int32_t> MtkPerf::perfLockReleaseSync(int32_t pl_handle, int32_t reserved) {
    LOG(INFO) << "perfLockReleaseSync pl_handle: " << pl_handle
              << " reserved: " << reserved;
    return 0;
}

}  // namespace vendor::mediatek::hardware::mtkpower::implementation
