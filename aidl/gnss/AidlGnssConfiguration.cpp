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

#define LOG_TAG "AidlGnssConfigurationAidl"

#include "AidlGnssConfiguration.h"
#include <log/log.h>

namespace aidl::android::hardware::gnss {

AidlGnssConfiguration::AidlGnssConfiguration(
        const GnssConfigurationInterface_ext* halConfigurationIface) :
        mGnssHalConfigIface(halConfigurationIface) {
}
AidlGnssConfiguration::~AidlGnssConfiguration() {
}

ndk::ScopedAStatus AidlGnssConfiguration::setSuplVersion(int version)  {
    if (mGnssHalConfigIface == nullptr) {
        ALOGE("%s: GNSS Configuration interface is not available.", __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    std::string config = "SUPL_VER=" + std::to_string(version) + "\n";
    mGnssHalConfigIface->configuration_update(config.c_str(), config.size());
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnssConfiguration::setSuplMode(int mode)  {
    if (mGnssHalConfigIface == nullptr) {
        ALOGE("%s: GNSS Configuration interface is not available.", __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    std::string config = "SUPL_MODE=" + std::to_string(mode) + "\n";
    mGnssHalConfigIface->configuration_update(config.c_str(), config.size());
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnssConfiguration::setLppProfile(int lppProfile) {
    if (mGnssHalConfigIface == nullptr) {
        ALOGE("%s: GNSS Configuration interface is not available.", __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    std::string config = "LPP_PROFILE=" + std::to_string(lppProfile) + "\n";
    mGnssHalConfigIface->configuration_update(config.c_str(), config.size());
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnssConfiguration::setGlonassPositioningProtocol(int protocol) {
    if (mGnssHalConfigIface == nullptr) {
        ALOGE("%s: GNSS Configuration interface is not available.", __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    std::string config = "A_GLONASS_POS_PROTOCOL_SELECT=" +
            std::to_string(protocol) + "\n";
    mGnssHalConfigIface->configuration_update(config.c_str(), config.size());
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnssConfiguration::setEmergencySuplPdn(bool enabled) {
    if (mGnssHalConfigIface == nullptr) {
        ALOGE("%s: GNSS Configuration interface is not available.", __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    std::string config = "USE_EMERGENCY_PDN_FOR_EMERGENCY_SUPL=" + std::to_string(enabled ? 1 : 0)
            + "\n";
    mGnssHalConfigIface->configuration_update(config.c_str(), config.size());
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnssConfiguration::setEsExtensionSec(int emergencyExtensionSeconds) {
    if (mGnssHalConfigIface == nullptr) {
        ALOGE("%s: GNSS Configuration interface is not available.", __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    ALOGD("setEsExtensionSec emergencyExtensionSeconds: %d", emergencyExtensionSeconds);
    mGnssHalConfigIface->set_es_extension_sec(emergencyExtensionSeconds);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnssConfiguration::setBlocklist(const vector<BlocklistedSource>& sourceList) {
    ALOGD("GnssConfiguration::setBlocklist");

    if (mGnssHalConfigIface == nullptr) {
        ALOGE("%s: GNSS Configuration interface is not available.", __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    size_t listSize = sourceList.size();
    long long blackSvid[GNSS_CONSTELLATION_SIZE];  /// V2_0::GnssConstellationType size is 8
    memset(blackSvid, 0x00, sizeof(long long)*GNSS_CONSTELLATION_SIZE);

    for (size_t i = 0; i < listSize; i++) {
        int idx = (int) sourceList[i].constellation;
        if (idx >= GNSS_CONSTELLATION_SIZE) {
            ALOGE("%s: GnssConstellation type is out of boundary.", __func__);
            continue;
        } else if (sourceList[i].svid == 0) { /// all sv are blocked
            blackSvid[idx] = (long long)(((long long)0xFFFFFFFFL << 32) | 0xFFFFFFFFL);
        } else if (idx == GNSS_CONSTELLATION_QZSS) {
            blackSvid[idx] |= (long long)((long long)0x01L << (sourceList[i].svid - 193)); /// QZSS is strated from 193
        } else {
            blackSvid[idx] |= (long long)((long long)0x01L << (sourceList[i].svid - 1));
        }
    }

    mGnssHalConfigIface->set_black_list(blackSvid, GNSS_CONSTELLATION_SIZE);
    return ndk::ScopedAStatus::ok();
}

}  // namespace aidl::android::hardware::gnss
