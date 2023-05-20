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

#include "MtkPower.h"

namespace vendor::mediatek::hardware::mtkpower::implementation {

// Methods from ::vendor::mediatek::hardware::mtkpower::V1_0::IMtkPower follow.
Return<void> MtkPower::mtkCusPowerHint(int32_t hint, int32_t data) {
    LOG(INFO) << "mtkCusPowerHint hint: " << hint
              << " data: " << data;
    return Void();
}

Return<void> MtkPower::mtkPowerHint(int32_t hint, int32_t data) {
    LOG(INFO) << "mtkPowerHint hint: " << hint
              << " data: " << data;
    return Void();
}

Return<void> MtkPower::notifyAppState(const hidl_string& pack, const hidl_string& act, int32_t pid, int32_t state, int32_t uid) {
    LOG(INFO) << "notifyAppState pack: " << pack
              << " act: " << act << " pid: " << pid
              << " state: " << state << " uid: " << uid;
    return Void();
}

Return<int32_t> MtkPower::querySysInfo(int32_t cmd, int32_t param) {
    LOG(INFO) << "querySysInfo cmd: " << cmd
              << " param: " << param;
    return 0;
}

Return<int32_t> MtkPower::setSysInfo(int32_t type, const hidl_string& data) {
    LOG(INFO) << "setSysInfo type: " << type
              << " data: " << data;
    return 0;
}

Return<int32_t> MtkPower::setSysInfoAsync(int32_t type, const hidl_string& data) {
    LOG(INFO) << "setSysInfoAsync type: " << type
              << " data: " << data;
    return 0;
}


// Methods from ::vendor::mediatek::hardware::mtkpower::V1_1::IMtkPower follow.
Return<int32_t> MtkPower::setMtkPowerCallback(const sp<::vendor::mediatek::hardware::mtkpower::V1_1::IMtkPowerCallback>& /* callback */) {
    LOG(WARNING) << "setMtkPowerCallback";
    return 0;
}


// Methods from ::vendor::mediatek::hardware::mtkpower::V1_2::IMtkPower follow.
Return<int32_t> MtkPower::setMtkScnUpdateCallback(int32_t /* hint */, const sp<::vendor::mediatek::hardware::mtkpower::V1_2::IMtkPowerCallback>& /* callback */) {
    LOG(WARNING) << "setMtkScnUpdateCallback";
    return 0;
}

}  // namespace vendor::mediatek::hardware::mtkpower::implementation
