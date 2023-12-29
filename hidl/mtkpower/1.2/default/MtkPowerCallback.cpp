/*
 * Copyright (C) 2022 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
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
