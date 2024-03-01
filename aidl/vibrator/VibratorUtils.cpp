/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Vibrator.h"

#include <android-base/logging.h>
#include <android-base/properties.h>
#include <fstream>

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

ndk::ScopedAStatus Vibrator::setNode(const std::string path, const int32_t value) {
    std::ofstream file(path);

    if (!file.is_open()) {
        LOG(ERROR) << "Failed to write " << value << " to " << path;
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));
    }

    file << value << std::endl;

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::activate(const int32_t timeoutMs) {
    ndk::ScopedAStatus status;

    /* timeoutMs under 1 = turn off vibrator */
    if (timeoutMs < 1) {
        return off();
    }

    status = setNode(kVibratorState, 1);
    if (!status.isOk()) return status;

    status = setNode(kVibratorDuration, timeoutMs);
    if (!status.isOk()) return status;

    status = setNode(kVibratorActivate, 1);
    if (!status.isOk()) return status;

    return ndk::ScopedAStatus::ok();
}

#ifdef VIBRATOR_SUPPORTS_EFFECTS
bool Vibrator::exists(const std::string path) {
    std::ofstream file(path);
    return file.is_open();
}

int Vibrator::getNode(const std::string path, const int fallback) {
    std::ifstream file(path);
    int value;

    if (!file.is_open()) {
        LOG(ERROR) << "failed to read from " << path.c_str();
        return fallback;
    }

    file >> value;
    return value;
}
#endif

int Vibrator::getIntProperty(const std::string& key, const int fallback) {
    return ::android::base::GetIntProperty(kVibratorPropPrefix + key, fallback);
}

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
