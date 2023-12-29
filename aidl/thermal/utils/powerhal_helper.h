/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/android/hardware/power/IPower.h>
#include <aidl/android/hardware/thermal/ThrottlingSeverity.h>
#include <aidl/google/hardware/power/extension/pixel/IPowerExt.h>

#include <queue>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace aidl {
namespace android {
namespace hardware {
namespace thermal {
namespace implementation {

using ::aidl::android::hardware::power::IPower;
using ::aidl::google::hardware::power::extension::pixel::IPowerExt;

using CdevRequestStatus = std::unordered_map<std::string, int>;

class PowerHalService {
  public:
    PowerHalService();
    ~PowerHalService() = default;
    bool connect();
    bool isAidlPowerHalExist() { return power_hal_aidl_exist_; }
    bool isModeSupported(const std::string &type, const ThrottlingSeverity &t);
    bool isPowerHalConnected() { return power_hal_aidl_ != nullptr; }
    bool isPowerHalExtConnected() { return power_hal_ext_aidl_ != nullptr; }
    void setMode(const std::string &type, const ThrottlingSeverity &t, const bool &enable,
                 const bool error_on_exit = false);

  private:
    bool power_hal_aidl_exist_;
    std::shared_ptr<IPower> power_hal_aidl_;
    std::shared_ptr<IPowerExt> power_hal_ext_aidl_;
    std::mutex lock_;
};

}  // namespace implementation
}  // namespace thermal
}  // namespace hardware
}  // namespace android
}  // namespace aidl
