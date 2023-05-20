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

#include "MtkPowerCallback.h"

namespace vendor::mediatek::hardware::mtkpower::implementation {

// Methods from ::vendor::mediatek::hardware::mtkpower::V1_1::IMtkPowerCallback follow.
Return<void> MtkPowerCallback::mtkPowerHint(int32_t /* hint */, int32_t /* duration */) {
    return Void();
}

Return<void> MtkPowerCallback::notifyAppState(const hidl_string& /* pack */, const hidl_string& /* act */, int32_t /* pid */, int32_t /* state */, int32_t /* uid */) {
    return Void();
}


// Methods from ::vendor::mediatek::hardware::mtkpower::V1_2::IMtkPowerCallback follow.
Return<void> MtkPowerCallback::notifyScnUpdate(int32_t /* hint */, int32_t /* data */) {
    return Void();
}

}  // namespace vendor::mediatek::hardware::mtkpower::implementation
