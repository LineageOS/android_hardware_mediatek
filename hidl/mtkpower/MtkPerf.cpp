/*
 * Copyright (C) 2022 The LineageOS Project
 *
 * Licensed under the Apache License, Version 2.1 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.1
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
