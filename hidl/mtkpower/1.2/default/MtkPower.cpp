/*
 * Copyright (C) 2022 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vendor.mediatek.hardware.mtkpower@1.2-service.stub"

#include <android-base/logging.h>
#include <aidl/android/hardware/power/IPower.h>
#include <aidl/android/hardware/power/Mode.h>
#include <android/binder_manager.h>

#include "MtkPower.h"

static std::shared_ptr<aidl::android::hardware::power::IPower> gAidlPowerHal;
static const std::string kInstance =
    std::string(aidl::android::hardware::power::IPower::descriptor) + "/default";

namespace vendor::mediatek::hardware::mtkpower::implementation {

bool MtkPower::getAidlPowerHal(void) {
    if (!gAidlPowerHal) {
        ndk::SpAIBinder pwBinder = ndk::SpAIBinder(
            AServiceManager_getService(kInstance.c_str()));
        gAidlPowerHal = aidl::android::hardware::power::IPower::fromBinder(pwBinder);
    }

    return !!gAidlPowerHal;
}

MtkPower::MtkPower() {
    if (!getAidlPowerHal()) {
        LOG(ERROR) << "Can't get AIDL Power HAL!";
    } else {
        LOG(INFO) << "Connected to power AIDL HAL";
    }
}

// Methods from ::vendor::mediatek::hardware::mtkpower::V1_0::IMtkPower follow.
Return<void> MtkPower::mtkCusPowerHint(int32_t hint, int32_t data) {
    LOG(INFO) << "mtkCusPowerHint hint: " << hint
              << " data: " << data;
    return Void();
}

Return<void> MtkPower::mtkPowerHint(int32_t hint, int32_t data) {
    // Forward AUDIO_POWER hints to libperfmgr
    if (hint == MTKPOWER_HINT_AUDIO_POWER) {
        // Enable the mode if data is non-zero
        bool enabled = data != 0;
        LOG(INFO) << "mtkPowerhint hint: MTKPOWER_HINT_AUDIO_POWER data: " << data << " enabled: " << enabled;
        if (getAidlPowerHal()) {
            gAidlPowerHal->setMode(
                aidl::android::hardware::power::Mode::AUDIO_STREAMING_LOW_LATENCY,
                enabled
            );
        } else {
            LOG(ERROR) << "mtkPowerHint: Can't get AIDL Power HAL!";
        }
    } else {
        LOG(INFO) << "mtkPowerHint hint: " << hint
                  << " data: " << data;
    }
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

Return<void> MtkPower::setSysInfoAsync(int32_t type, const hidl_string& data) {
    LOG(INFO) << "setSysInfoAsync type: " << type
              << " data: " << data;
    return Void();
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
