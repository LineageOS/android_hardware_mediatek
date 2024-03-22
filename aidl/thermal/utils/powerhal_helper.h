/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/android/hardware/power/IPower.h>
#include <aidl/android/hardware/thermal/IThermal.h>
#include <aidl/android/hardware/thermal/ThrottlingSeverity.h>
#include <aidl/google/hardware/power/extension/pixel/IPowerExt.h>
#include <utils/Trace.h>

#include <queue>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "thermal_info.h"

namespace aidl {
namespace android {
namespace hardware {
namespace thermal {
namespace implementation {

using ::aidl::android::hardware::power::IPower;
using ::aidl::google::hardware::power::extension::pixel::IPowerExt;

using CdevRequestStatus = std::unordered_map<std::string, int>;

struct PowerHintstatus {
    std::unordered_map<ThrottlingSeverity, ThrottlingSeverity> hint_severity_map;
    ThrottlingSeverity prev_hint_severity;
};

class PowerHalService {
  public:
    PowerHalService();
    ~PowerHalService() = default;
    bool connect();
    void reconnect();
    bool isAidlPowerHalExist() { return power_hal_aidl_exist_; }
    bool isModeSupported(const std::string &type, const ThrottlingSeverity &t);
    bool isPowerHalConnected() { return power_hal_aidl_ != nullptr; }
    bool isPowerHalExtConnected() { return power_hal_ext_aidl_ != nullptr; }
    void setMode(const std::string &type, const ThrottlingSeverity &t, const bool &enable,
                 const bool error_on_exit = false);
    void updateSupportedPowerHints(
            const std::unordered_map<std::string, SensorInfo> &sensor_info_map_);
    void sendPowerExtHint(const Temperature &t);

  private:
    ndk::ScopedAIBinder_DeathRecipient power_hal_ext_aidl_death_recipient_;
    static void onPowerHalExtAidlBinderDied(void *cookie) {
        if (cookie) {
            auto *e = static_cast<PowerHalService *>(cookie);
            {
                std::lock_guard<std::mutex> lock(e->lock_);
                e->power_hal_aidl_ = nullptr;
                e->power_hal_ext_aidl_ = nullptr;
            }
            e->reconnect();
        }
    }

    bool power_hal_aidl_exist_;
    std::shared_ptr<IPower> power_hal_aidl_;
    std::shared_ptr<IPowerExt> power_hal_ext_aidl_;
    std::mutex lock_;
    std::unordered_map<std::string, PowerHintstatus> supported_powerhint_map_;
    mutable std::shared_mutex powerhint_status_mutex_;
};

}  // namespace implementation
}  // namespace thermal
}  // namespace hardware
}  // namespace android
}  // namespace aidl
