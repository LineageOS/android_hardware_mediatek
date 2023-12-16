/*
 * SPDX-FileCopyrightText: 2020 The Android Open Source Project
 * SPDX-FileCopyrightText: 2023 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "types.h"

#include <aidl/android/hardware/power/BnPower.h>

namespace aidl {
namespace android {
namespace hardware {
namespace power {
namespace impl {
namespace mediatek {

const std::string kTouchBoostDurationProperty = "persist.vendor.powerhal.touchboost_duration";
const int32_t kDefaultTouchBoostDuration = 1; /* ms */
const int32_t kLaunchBoostDuration = 30000;   /* ms */
const int32_t kMaxInteractiveDuration = 5000; /* ms */
const int32_t kMinInteractiveDuration = 400;  /* ms */
const int32_t kMaxTouchBoostDuration = 1000;  /* ms */

class Power : public BnPower {
  public:
    Power();
    ~Power();
    ndk::ScopedAStatus setMode(Mode type, bool enabled) override;
    ndk::ScopedAStatus isModeSupported(Mode type, bool* _aidl_return) override;
    ndk::ScopedAStatus setBoost(Boost type, int32_t durationMs) override;
    ndk::ScopedAStatus isBoostSupported(Boost type, bool* _aidl_return) override;
    ndk::ScopedAStatus createHintSession(int32_t tgid, int32_t uid,
                                         const std::vector<int32_t>& threadIds,
                                         int64_t durationNanos,
                                         std::shared_ptr<IPowerHintSession>* _aidl_return) override;
    ndk::ScopedAStatus getHintSessionPreferredRate(int64_t* outNanoseconds) override;

  private:
    static long long calcTimespanUs(struct timespec start, struct timespec end);
    void handleInteractionHint(int32_t targetDuration);

    libpowerhal_t* mPerf;

    struct timespec mPreviousInteractionTime;
    int32_t mTouchBoostDuration;
    int32_t mPreviousInteractionDuration;
    int32_t mPreviousInteractionHandle;
    int32_t mLaunchHandle;
};

}  // namespace mediatek
}  // namespace impl
}  // namespace power
}  // namespace hardware
}  // namespace android
}  // namespace aidl
