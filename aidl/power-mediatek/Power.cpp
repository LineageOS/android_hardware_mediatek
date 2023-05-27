/*
 * Copyright (C) 2020 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <dlfcn.h>

#include "Power.h"

#include <android-base/logging.h>

#ifdef TAP_TO_WAKE_NODE
#include <android-base/file.h>
#endif

#define POWERHAL_LIB_NAME "libpowerhal.so"
#define LIBPOWERHAL_INIT "libpowerhal_Init"
#define LIBPOWERHAL_CUSLOCKHINT "libpowerhal_CusLockHint"
#define LIBPOWERHAL_LOCKREL "libpowerhal_LockRel"
#define LIBPOWERHAL_USERSCNDISABLEALL "libpowerhal_UserScnDisableAll"
#define LIBPOWERHAL_USERSCNRESTOREALL "libpowerhal_UserScnRestoreAll"

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

const std::vector<Boost> SUPPORTED_BOOSTS {
    Boost::INTERACTION,
};

const std::vector<Mode> SUPPORTED_MODES {
#ifdef TAP_TO_WAKE_NODE
    Mode::DOUBLE_TAP_TO_WAKE,
#endif
    Mode::LOW_POWER,
    Mode::SUSTAINED_PERFORMANCE,
    Mode::LAUNCH,
    Mode::EXPENSIVE_RENDERING,
    Mode::INTERACTIVE,
};

Power::Power() {
    powerHandle = dlopen(POWERHAL_LIB_NAME, RTLD_LAZY);
    if (!powerHandle) {
        LOG(ERROR) << "Could not dlopen " << POWERHAL_LIB_NAME;
        abort();
    }

    libpowerhal_Init =
        reinterpret_cast<libpowerhal_Init_handle>(dlsym(powerHandle, LIBPOWERHAL_INIT));

    if (!libpowerhal_Init) {
        LOG(ERROR) << "Could not locate symbol " << LIBPOWERHAL_INIT;
        abort();
    }

    libpowerhal_CusLockHint =
        reinterpret_cast<libpowerhal_CusLockHint_handle>(dlsym(powerHandle, LIBPOWERHAL_CUSLOCKHINT));

    if (!libpowerhal_CusLockHint) {
        LOG(ERROR) << "Could not locate symbol " << LIBPOWERHAL_CUSLOCKHINT;
        abort();
    }

    libpowerhal_LockRel =
        reinterpret_cast<libpowerhal_LockRel_handle>(dlsym(powerHandle, LIBPOWERHAL_LOCKREL));

    if (!libpowerhal_LockRel) {
        LOG(ERROR) << "Could not locate symbol " << LIBPOWERHAL_LOCKREL;
        abort();
    }

    libpowerhal_UserScnDisableAll =
         reinterpret_cast<libpowerhal_UserScnDisableAll_handle>(dlsym(powerHandle, LIBPOWERHAL_USERSCNDISABLEALL));

    if (!libpowerhal_UserScnDisableAll) {
        LOG(ERROR) << "Could not locate symbol " << LIBPOWERHAL_USERSCNDISABLEALL;
        abort();
    }

    libpowerhal_UserScnRestoreAll =
        reinterpret_cast<libpowerhal_UserScnRestoreAll_handle>(dlsym(powerHandle, LIBPOWERHAL_USERSCNRESTOREALL));

    if (!libpowerhal_UserScnRestoreAll) {
        LOG(ERROR) << "Could not locate symbol " << LIBPOWERHAL_USERSCNRESTOREALL;
        abort();
    }

    mLowPowerEnabled = 0;
    libpowerhal_Init(1);
}

Power::~Power() { }

ndk::ScopedAStatus Power::setMode(Mode type, bool enabled) {
    LOG(VERBOSE) << "Power setMode: " << static_cast<int32_t>(type) << " to: " << enabled;

#ifdef MODE_EXT
    if (setDeviceSpecificMode(type, enabled)) {
        return ndk::ScopedAStatus::ok();
    }
#endif
    switch (type) {
#ifdef TAP_TO_WAKE_NODE
        case Mode::DOUBLE_TAP_TO_WAKE:
        {
            ::android::base::WriteStringToFile(enabled ? "1" : "0", TAP_TO_WAKE_NODE, true);
            break;
        }
#endif
        case Mode::LAUNCH:
        {
            if (mLowPowerEnabled && !mLaunchHandle)
                break;

            if (mLaunchHandle != 0) {
                libpowerhal_LockRel(mLaunchHandle);
                mLaunchHandle = 0;
            }

            if (enabled && !mLowPowerEnabled)
                mLaunchHandle = libpowerhal_CusLockHint(11, 30000, getpid());

            break;
        }
        case Mode::INTERACTIVE:
        {
            if (enabled)
                /* Device now in interactive state,
                   restore all currently held hints. */
                libpowerhal_UserScnRestoreAll();
            else
                /* Device entering non interactive state,
                   disable all hints to save power. */
                libpowerhal_UserScnDisableAll();
            break;
        }
        case Mode::EXPENSIVE_RENDERING:
        {
            if (mLowPowerEnabled && !mExpensiveRenderingHandle)
                break;

            if (mExpensiveRenderingHandle != 0) {
                libpowerhal_LockRel(mExpensiveRenderingHandle);
                mExpensiveRenderingHandle = 0;
            }

            if (enabled && !mLowPowerEnabled)
                mExpensiveRenderingHandle = libpowerhal_CusLockHint(12, 0, getpid());

            break;
        }
        case Mode::SUSTAINED_PERFORMANCE:
        {
            if (mLowPowerEnabled && !mSustainedPerformanceHandle)
                break;

            if (mSustainedPerformanceHandle != 0) {
                libpowerhal_LockRel(mSustainedPerformanceHandle);
                mSustainedPerformanceHandle = 0;
            }

            if (enabled && !mLowPowerEnabled)
                mSustainedPerformanceHandle = libpowerhal_CusLockHint(8, 0, getpid());

            break;
        }
        case Mode::LOW_POWER:
            mLowPowerEnabled = enabled;
            break;
        default:
            break;
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Power::isModeSupported(Mode type, bool* _aidl_return) {
#ifdef MODE_EXT
    if (isDeviceSpecificModeSupported(type, _aidl_return)) {
        return ndk::ScopedAStatus::ok();
    }
#endif

    LOG(INFO) << "Power isModeSupported: " << static_cast<int32_t>(type);
    *_aidl_return = std::find(SUPPORTED_MODES.begin(), SUPPORTED_MODES.end(), type) != SUPPORTED_MODES.end();

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Power::setBoost(Boost type, int32_t durationMs) {
    int32_t mediatek_type = 0;

    if (mLowPowerEnabled) {
        LOG(VERBOSE) << "Will not perform boosts in LOW_POWER";
        return ndk::ScopedAStatus::ok();
    }

    switch (type) {
        case Boost::INTERACTION:
            LOG(VERBOSE) << "Power setBoost INTERACTION for: " << durationMs << "ms";
            if (durationMs < 1)
                durationMs = 80;
            mediatek_type = 0; // INTERACTION
            break;
        default:
            LOG(ERROR) << "Power unknown boost type: " << static_cast<int32_t>(type);
            return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    libpowerhal_CusLockHint(mediatek_type, durationMs, getpid());
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Power::isBoostSupported(Boost type, bool* _aidl_return) {
    LOG(INFO) << "Power isBoostSupported: " << static_cast<int32_t>(type);
    *_aidl_return = std::find(SUPPORTED_BOOSTS.begin(), SUPPORTED_BOOSTS.end(), type) != SUPPORTED_BOOSTS.end();

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
