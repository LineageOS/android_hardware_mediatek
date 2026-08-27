/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vendor.mediatek.hardware.mtkpower-service.stub"

#include <aidl/android/hardware/power/IPower.h>
#include <aidl/android/hardware/power/Mode.h>
#include <android-base/logging.h>
#include <android/binder_manager.h>

#include "MtkPowerService.h"

static std::shared_ptr<aidl::android::hardware::power::IPower> gAidlPowerHal;
static const std::string kInstance =
        std::string(aidl::android::hardware::power::IPower::descriptor) + "/default";

namespace aidl {
namespace vendor {
namespace mediatek {
namespace hardware {
namespace mtkpower {

bool MtkPowerService::getAidlPowerHal(void) {
    if (!gAidlPowerHal) {
        ndk::SpAIBinder pwBinder = ndk::SpAIBinder(AServiceManager_getService(kInstance.c_str()));
        gAidlPowerHal = aidl::android::hardware::power::IPower::fromBinder(pwBinder);
    }

    return !!gAidlPowerHal;
}

MtkPowerService::MtkPowerService() {
    if (!getAidlPowerHal()) {
        LOG(ERROR) << "Can't get AIDL Power HAL!";
    } else {
        LOG(INFO) << "Connected to power AIDL HAL";
    }
}

ndk::ScopedAStatus MtkPowerService::perfLockAcquire(int handle, int duration,
                                                    const std::vector<int>& /* boostList */,
                                                    int pid, int reserved, int* _aidl_return) {
    LOG(INFO) << __func__ << ": handle=" << handle << ", duration=" << duration << ", pid=" << pid
              << ", reserved=" << reserved;
    *_aidl_return = 1;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MtkPowerService::perfCusLockHint(int hint, int duration, int pid,
                                                    int* _aidl_return) {
    LOG(INFO) << __func__ << ": hint=" << hint << ", duration=" << duration << ", pid=" << pid;
    *_aidl_return = 1;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MtkPowerService::perfLockRelease(int handle, int reserved) {
    LOG(INFO) << __func__ << ": handle=" << handle << ", reserved=" << reserved;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MtkPowerService::perfLockReleaseSync(int handle, int reserved,
                                                        int* _aidl_return) {
    LOG(INFO) << __func__ << ": handle=" << handle << ", reserved=" << reserved;
    *_aidl_return = 1;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MtkPowerService::mtkPowerHint(int hint, int data) {
    // Forward MTKPOWER_HINT_AUDIO hints to libperfmgr
    switch (hint) {
        case MTKPOWER_HINT_AUDIO_LATENCY_DL:
        case MTKPOWER_HINT_AUDIO_LATENCY_UL:
        case MTKPOWER_HINT_AUDIO_POWER_DL:
        case MTKPOWER_HINT_AUDIO_POWER_UL:
        case MTKPOWER_HINT_AUDIO_HAL_OPEN: {
            // Enable the mode if data is non-zero
            bool enabled = data != 0;
            LOG(INFO) << __func__ << ": hint=" << hint << ", data=" << data
                      << ", enabled=" << enabled;
            if (getAidlPowerHal()) {
                gAidlPowerHal->setMode(
                        aidl::android::hardware::power::Mode::AUDIO_STREAMING_LOW_LATENCY, enabled);
            } else {
                LOG(ERROR) << __func__ << ": Can't get AIDL Power HAL!";
            }
            break;
        }
        default: {
            LOG(INFO) << __func__ << ": hint=" << hint << ", data=" << data;
            break;
        }
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MtkPowerService::mtkCusPowerHint(int hint, int data) {
    LOG(INFO) << __func__ << ": hint=" << hint << ", data=" << data;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MtkPowerService::querySysInfo(int cmd, int param, int* _aidl_return) {
    LOG(INFO) << __func__ << ": cmd=" << cmd << ", param=" << param;
    *_aidl_return = 1;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MtkPowerService::setSysInfo(int type, const std::string& data,
                                               int* _aidl_return) {
    LOG(INFO) << __func__ << ": type=" << type << ", data=" << data;
    *_aidl_return = 1;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MtkPowerService::setSysInfoAsync(int type, const std::string& data) {
    LOG(INFO) << __func__ << ": type=" << type << ", data=" << data;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MtkPowerService::setMtkPowerCallback(
        const std::shared_ptr<
                ::aidl::vendor::mediatek::hardware::mtkpower::IMtkPowerCallback>& /* callback */,
        int* _aidl_return) {
    LOG(WARNING) << __func__;
    *_aidl_return = 1;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MtkPowerService::setMtkScnUpdateCallback(
        const std::shared_ptr<
                ::aidl::vendor::mediatek::hardware::mtkpower::IMtkPowerCallback>& /* callback */,
        int* _aidl_return) {
    LOG(WARNING) << __func__;
    *_aidl_return = 1;
    return ndk::ScopedAStatus::ok();
}

}  // namespace mtkpower
}  // namespace hardware
}  // namespace mediatek
}  // namespace vendor
}  // namespace aidl
