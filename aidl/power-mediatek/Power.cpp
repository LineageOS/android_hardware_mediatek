/*
 * SPDX-FileCopyrightText: 2020 The Android Open Source Project
 * SPDX-FileCopyrightText: 2023 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <dlfcn.h>
#include <time.h>

#include "Power.h"

#include <android-base/logging.h>
#include <android-base/properties.h>

#ifdef TAP_TO_WAKE_NODE
#include <android-base/file.h>
#endif

using android::base::GetIntProperty;

#define DLSYM_GET_FUNCTION(func_ptr, handle, func_name)          \
    func_ptr = (typeof(func_ptr))dlsym(handle, #func_name);      \
    if (func_ptr == NULL) {                                      \
        LOG(ERROR) << "Could not locate symbol " #func_name "."; \
        abort();                                                 \
    }

namespace aidl {
namespace android {
namespace hardware {
namespace power {
namespace impl {
namespace mediatek {

#ifdef MODE_EXT
extern bool isDeviceSpecificModeSupported(Mode type, bool* _aidl_return);
extern bool setDeviceSpecificMode(Mode type, bool enabled);
#endif

Power::Power() {
    mTouchBoostDuration = GetIntProperty(kTouchBoostDurationProperty, kDefaultTouchBoostDuration);
    if (mTouchBoostDuration < 0 || mTouchBoostDuration > kMaxTouchBoostDuration) {
        LOG(ERROR) << "Invalid touch boost duration: " << mTouchBoostDuration;
        mTouchBoostDuration = kDefaultTouchBoostDuration;
    }

    mPerf = (struct libpowerhal_t*)calloc(1, sizeof(struct libpowerhal_t));
    if (!mPerf) {
        LOG(ERROR) << "Could not allocate memory for libpowerhal struct.";
        abort();
    }

    mPerf->perfLib = dlopen(POWERHAL_LIB_NAME, RTLD_LAZY);
    if (!mPerf->perfLib) {
        LOG(ERROR) << "Could not dlopen " << POWERHAL_LIB_NAME;
        abort();
    }

    DLSYM_GET_FUNCTION(mPerf->Init, mPerf->perfLib, libpowerhal_Init);
    DLSYM_GET_FUNCTION(mPerf->CusLockHint, mPerf->perfLib, libpowerhal_CusLockHint);
    DLSYM_GET_FUNCTION(mPerf->LockRel, mPerf->perfLib, libpowerhal_LockRel);
    DLSYM_GET_FUNCTION(mPerf->UserScnDisableAll, mPerf->perfLib, libpowerhal_UserScnDisableAll);
    DLSYM_GET_FUNCTION(mPerf->UserScnRestoreAll, mPerf->perfLib, libpowerhal_UserScnRestoreAll);

    mPerf->Init(1);
}

Power::~Power() {}

long long Power::calcTimespanUs(struct timespec start, struct timespec end) {
    long long diff_in_us = 0;
    diff_in_us += (end.tv_sec - start.tv_sec) * USINSEC;
    diff_in_us += (end.tv_nsec - start.tv_nsec) / NSINUS;
    return diff_in_us;
}

void Power::handleInteractionHint(int32_t targetDuration) {
    struct timespec currentInteractionTime;
    long long elapsedTime;
    int32_t durationMs = kMinInteractiveDuration;

    // if targetDuration is 0, we perform a touch boost instead.
    if (targetDuration == 0) {
        if (mTouchBoostDuration == 0) {
            return;
        }
        mPerf->CusLockHint(MTKPOWER_HINT_APP_TOUCH, mTouchBoostDuration, getpid());
        return;
    }

    if (targetDuration > durationMs) {
        durationMs = (targetDuration > kMaxInteractiveDuration) ? kMaxInteractiveDuration
                                                                : targetDuration;
    }

    clock_gettime(CLOCK_MONOTONIC, &currentInteractionTime);
    elapsedTime = calcTimespanUs(mPreviousInteractionTime, currentInteractionTime);

    // don't hint if it's been less than 250ms since last boost
    // also detect if we're doing anything resembling a fling
    // support additional boosting in case of flings
    if (elapsedTime < 250000 && durationMs <= 750) {
        return;
    }

    if (durationMs <= mPreviousInteractionDuration) {
        // don't hint if previous hint's duration covers this hint's duration
        if (elapsedTime <= (mPreviousInteractionDuration - durationMs)) {
            return;
        }
    }

    mPreviousInteractionTime = currentInteractionTime;
    mPreviousInteractionDuration = durationMs;

    // most vendors don't implement the AOSP interaction hint in
    // powerscntbl.xml, but MTKPOWER_HINT_UX_SCROLLING is guaranteed to
    // be implemented on vendors newer than S.
    int interactionHandle = mPerf->CusLockHint(MTKPOWER_HINT_UX_SCROLLING, durationMs, getpid());

    if (mPreviousInteractionHandle > 0) {
        mPerf->LockRel(mPreviousInteractionHandle);
    }
    mPreviousInteractionHandle = interactionHandle;
}

ndk::ScopedAStatus Power::setMode(Mode type, bool enabled) {
    LOG(VERBOSE) << "Power setMode: " << static_cast<int32_t>(type) << " to: " << enabled;

#ifdef MODE_EXT
    if (setDeviceSpecificMode(type, enabled)) {
        return ndk::ScopedAStatus::ok();
    }
#endif
    switch (type) {
#ifdef TAP_TO_WAKE_NODE
        case Mode::DOUBLE_TAP_TO_WAKE: {
            ::android::base::WriteStringToFile(enabled ? "1" : "0", TAP_TO_WAKE_NODE, true);
            break;
        }
#endif
        case Mode::LAUNCH: {
            if (mLaunchHandle > 0) {
                mPerf->LockRel(mLaunchHandle);
                mLaunchHandle = 0;
            }

            if (enabled) {
                mLaunchHandle =
                        mPerf->CusLockHint(MTKPOWER_HINT_LAUNCH, kLaunchBoostDuration, getpid());
            }
            break;
        }
        case Mode::INTERACTIVE: {
            if (enabled) {
                // Device is now in an interactive state,
                // resume all previously performing hints.
                mPerf->UserScnRestoreAll();
            } else {
                // Device is entering a non interactive state,
                // disable all hints to save power.
                mPerf->UserScnDisableAll();
            }
            break;
        }
        default:
            break;
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Power::isModeSupported(Mode type, bool* _aidl_return) {
    LOG(INFO) << "Power isModeSupported: " << static_cast<int32_t>(type);

#ifdef MODE_EXT
    if (isDeviceSpecificModeSupported(type, _aidl_return)) {
        return ndk::ScopedAStatus::ok();
    }
#endif

    switch (type) {
#ifdef TAP_TO_WAKE_NODE
        case Mode::DOUBLE_TAP_TO_WAKE:
#endif
        case Mode::LAUNCH:
        case Mode::INTERACTIVE:
            *_aidl_return = true;
            break;
        default:
            break;
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Power::setBoost(Boost type, int32_t durationMs) {
    if (type == Boost::INTERACTION) {
        LOG(VERBOSE) << "Power setBoost INTERACTION for: " << durationMs << "ms";
        handleInteractionHint(durationMs);
        return ndk::ScopedAStatus::ok();
    }

    LOG(ERROR) << "Power unknown boost type: " << static_cast<int32_t>(type);
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Power::isBoostSupported(Boost type, bool* _aidl_return) {
    LOG(INFO) << "Power isBoostSupported: " << static_cast<int32_t>(type);

    *_aidl_return = (type == Boost::INTERACTION);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Power::createHintSession(int32_t, int32_t, const std::vector<int32_t>&, int64_t,
                                            std::shared_ptr<IPowerHintSession>* _aidl_return) {
    *_aidl_return = nullptr;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus Power::getHintSessionPreferredRate(int64_t* outNanoseconds) {
    *outNanoseconds = -1;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

}  // namespace mediatek
}  // namespace impl
}  // namespace power
}  // namespace hardware
}  // namespace android
}  // namespace aidl
