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
