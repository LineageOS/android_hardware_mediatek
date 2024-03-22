/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define ATRACE_TAG (ATRACE_TAG_THERMAL | ATRACE_TAG_HAL)

#include "powerhal_helper.h"

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/stringprintf.h>
#include <android-base/strings.h>
#include <android/binder_manager.h>

#include <iterator>
#include <set>
#include <sstream>
#include <thread>
#include <vector>

#include "thermal_throttling.h"

namespace aidl {
namespace android {
namespace hardware {
namespace thermal {
namespace implementation {

using ::android::base::StringPrintf;

PowerHalService::PowerHalService()
    : power_hal_aidl_exist_(true), power_hal_aidl_(nullptr), power_hal_ext_aidl_(nullptr) {
    connect();
}

bool PowerHalService::connect() {
    std::lock_guard<std::mutex> lock(lock_);

    if (!power_hal_aidl_exist_) {
        return false;
    }

    if (power_hal_aidl_ && power_hal_ext_aidl_) {
        return true;
    }

    const std::string kInstance = std::string(IPower::descriptor) + "/default";
    ndk::SpAIBinder power_binder =
            ndk::SpAIBinder(AServiceManager_waitForService(kInstance.c_str()));
    ndk::SpAIBinder ext_power_binder;

    if (power_binder.get() == nullptr) {
        LOG(ERROR) << "Cannot get Power Hal Binder";
        power_hal_aidl_exist_ = false;
        return false;
    }

    power_hal_aidl_ = IPower::fromBinder(power_binder);

    if (power_hal_aidl_ == nullptr) {
        power_hal_aidl_exist_ = false;
        LOG(ERROR) << "Cannot get Power Hal AIDL" << kInstance.c_str();
        return false;
    }

    if (STATUS_OK != AIBinder_getExtension(power_binder.get(), ext_power_binder.getR()) ||
        ext_power_binder.get() == nullptr) {
        LOG(ERROR) << "Cannot get Power Hal Extension Binder";
        power_hal_aidl_exist_ = false;
        return false;
    }

    power_hal_ext_aidl_ = IPowerExt::fromBinder(ext_power_binder);
    if (power_hal_ext_aidl_ == nullptr) {
        LOG(ERROR) << "Cannot get Power Hal Extension AIDL";
        power_hal_aidl_exist_ = false;
    }

    if (power_hal_ext_aidl_death_recipient_.get() == nullptr) {
        power_hal_ext_aidl_death_recipient_ = ndk::ScopedAIBinder_DeathRecipient(
                AIBinder_DeathRecipient_new(onPowerHalExtAidlBinderDied));
    }

    auto linked = AIBinder_linkToDeath(power_hal_ext_aidl_->asBinder().get(),
                                       power_hal_ext_aidl_death_recipient_.get(), this);

    if (linked != STATUS_OK) {
        LOG(ERROR) << "Failed to register power_hal_ext death recipient";
    }

    return true;
}

void PowerHalService::reconnect() {
    ATRACE_CALL();
    if (!connect()) {
        LOG(ERROR) << " Failed to reconnect power_hal_ext";
        return;
    }

    LOG(INFO) << "Resend the power hints when power_hal_ext is reconnected";
    std::lock_guard<std::shared_mutex> _lock(powerhint_status_mutex_);
    for (const auto &[sensor_name, supported_powerhint] : supported_powerhint_map_) {
        std::stringstream log_buf;
        for (const auto &severity : ::ndk::enum_range<ThrottlingSeverity>()) {
            bool mode = severity <= supported_powerhint.prev_hint_severity;
            setMode(sensor_name, severity, mode);
            log_buf << toString(severity).c_str() << ":" << mode << " ";
        }

        LOG(INFO) << sensor_name << " send powerhint: " << log_buf.str();
        log_buf.clear();
    }
    return;
}

void PowerHalService::updateSupportedPowerHints(
        const std::unordered_map<std::string, SensorInfo> &sensor_info_map_) {
    for (auto const &name_status_pair : sensor_info_map_) {
        if (!(name_status_pair.second.send_powerhint)) {
            continue;
        }
        ThrottlingSeverity current_severity = ThrottlingSeverity::NONE;
        for (const auto &severity : ::ndk::enum_range<ThrottlingSeverity>()) {
            if (severity == ThrottlingSeverity::NONE) {
                supported_powerhint_map_[name_status_pair.first]
                        .hint_severity_map[ThrottlingSeverity::NONE] = ThrottlingSeverity::NONE;
                continue;
            }

            bool isSupported = false;
            ndk::ScopedAStatus isSupportedResult;

            if (power_hal_ext_aidl_ != nullptr) {
                isSupported = isModeSupported(name_status_pair.first, severity);
            }
            if (isSupported)
                current_severity = severity;
            supported_powerhint_map_[name_status_pair.first].hint_severity_map[severity] =
                    current_severity;
        }
    }
}

void PowerHalService::sendPowerExtHint(const Temperature &t) {
    ATRACE_CALL();
    std::lock_guard<std::shared_mutex> _lock(powerhint_status_mutex_);
    ThrottlingSeverity prev_hint_severity = supported_powerhint_map_[t.name].prev_hint_severity;
    ThrottlingSeverity current_hint_severity =
            supported_powerhint_map_[t.name].hint_severity_map[t.throttlingStatus];
    std::stringstream log_buf;

    if (!power_hal_aidl_exist_) {
        LOG(ERROR) << "power_hal_aidl is not exist";
        return;
    }

    if (prev_hint_severity == current_hint_severity) {
        return;
    }

    for (const auto &severity : ::ndk::enum_range<ThrottlingSeverity>()) {
        if (severity != supported_powerhint_map_[t.name].hint_severity_map[severity]) {
            continue;
        }
        bool mode = severity <= current_hint_severity;
        setMode(t.name, severity, mode);
        log_buf << toString(severity).c_str() << ":" << mode << " ";
    }

    LOG(INFO) << t.name << " send powerhint: " << log_buf.str();

    supported_powerhint_map_[t.name].prev_hint_severity = current_hint_severity;
}

bool PowerHalService::isModeSupported(const std::string &type, const ThrottlingSeverity &t) {
    bool isSupported = false;
    if (!connect()) {
        return false;
    }
    std::string power_hint = StringPrintf("THERMAL_%s_%s", type.c_str(), toString(t).c_str());
    lock_.lock();
    if (!power_hal_ext_aidl_->isModeSupported(power_hint, &isSupported).isOk()) {
        LOG(ERROR) << "Fail to check supported mode, Hint: " << power_hint;
        power_hal_ext_aidl_ = nullptr;
        power_hal_aidl_ = nullptr;
        lock_.unlock();
        return false;
    }
    lock_.unlock();
    return isSupported;
}

void PowerHalService::setMode(const std::string &type, const ThrottlingSeverity &t,
                              const bool &enable, const bool error_on_exit) {
    if (!connect()) {
        return;
    }

    std::string power_hint = StringPrintf("THERMAL_%s_%s", type.c_str(), toString(t).c_str());
    lock_.lock();
    if (!power_hal_ext_aidl_->setMode(power_hint, enable).isOk()) {
        LOG(ERROR) << "Fail to set mode, Hint: " << power_hint;
        power_hal_ext_aidl_ = nullptr;
        power_hal_aidl_ = nullptr;
        lock_.unlock();
        if (!error_on_exit) {
            setMode(type, t, enable, true);
        }
        return;
    }
    lock_.unlock();
}

}  // namespace implementation
}  // namespace thermal
}  // namespace hardware
}  // namespace android
}  // namespace aidl
