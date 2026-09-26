/*
 * Copyright (C) 2022 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vendor.mediatek.hardware.mtkpower@1.2-service.stub"

#include <aidl/android/hardware/power/IPower.h>
#include <aidl/android/hardware/power/Mode.h>
#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/parseint.h>
#include <android-base/strings.h>
#include <android/binder_manager.h>

#include <algorithm>
#include <filesystem>

#include "MtkPower.h"

using android::base::ParseInt;
using android::base::ReadFileToString;
using android::base::StartsWith;
using android::base::Tokenize;
using android::base::Trim;

static std::shared_ptr<aidl::android::hardware::power::IPower> gAidlPowerHal;
static const std::string kInstance =
        std::string(aidl::android::hardware::power::IPower::descriptor) + "/default";
static const std::string kCpufreqPath = "/sys/devices/system/cpu/cpufreq";

static int readInt(const std::string& path) {
    std::string buf;
    int value = -1;
    if (ReadFileToString(path, &buf)) {
        ParseInt(Trim(buf), &value);
    }
    return value;
}

namespace vendor::mediatek::hardware::mtkpower::implementation {

bool MtkPower::getAidlPowerHal(void) {
    if (!gAidlPowerHal) {
        ndk::SpAIBinder pwBinder = ndk::SpAIBinder(AServiceManager_getService(kInstance.c_str()));
        gAidlPowerHal = aidl::android::hardware::power::IPower::fromBinder(pwBinder);
    }

    return !!gAidlPowerHal;
}

void MtkPower::loadClusters(void) {
    std::vector<std::pair<int, ClusterInfo>> policies;
    std::error_code ec;

    for (const auto& entry : std::filesystem::directory_iterator(kCpufreqPath, ec)) {
        const std::string name = entry.path().filename();
        int id;
        if (!StartsWith(name, "policy") || !ParseInt(name.substr(6), &id)) {
            continue;
        }

        std::string cpus;
        ReadFileToString(entry.path() / "related_cpus", &cpus);

        ClusterInfo info;
        info.cpuNum = Tokenize(cpus, " \n").size();
        info.freqMin = readInt(entry.path() / "cpuinfo_min_freq");
        info.freqMax = readInt(entry.path() / "cpuinfo_max_freq");
        policies.emplace_back(id, info);
    }

    std::sort(policies.begin(), policies.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    for (const auto& [id, info] : policies) {
        mClusters.push_back(info);
    }
}

MtkPower::MtkPower() {
    loadClusters();

    if (!getAidlPowerHal()) {
        LOG(ERROR) << "Can't get AIDL Power HAL!";
    } else {
        LOG(INFO) << "Connected to power AIDL HAL";
    }
}

// Methods from ::vendor::mediatek::hardware::mtkpower::V1_0::IMtkPower follow.
Return<void> MtkPower::mtkCusPowerHint(int32_t hint, int32_t data) {
    LOG(INFO) << "mtkCusPowerHint hint: " << hint << " data: " << data;
    return Void();
}

Return<void> MtkPower::mtkPowerHint(int32_t hint, int32_t data) {
    // Forward MTKPOWER_HINT_AUDIO hints to libperfmgr
    switch (hint) {
        case MTKPOWER_HINT_AUDIO_LATENCY_DL:
        case MTKPOWER_HINT_AUDIO_LATENCY_UL:
        case MTKPOWER_HINT_AUDIO_POWER_DL:
        case MTKPOWER_HINT_AUDIO_POWER_UL:
        case MTKPOWER_HINT_AUDIO_POWER: {
            // Enable the mode if data is non-zero
            bool enabled = data != 0;
            LOG(INFO) << "mtkPowerhint hint: " << hint << " data: " << data
                      << " enabled: " << enabled;
            if (getAidlPowerHal()) {
                gAidlPowerHal->setMode(
                        aidl::android::hardware::power::Mode::AUDIO_STREAMING_LOW_LATENCY, enabled);
            } else {
                LOG(ERROR) << "mtkPowerHint: Can't get AIDL Power HAL!";
            }
            break;
        }
        default: {
            LOG(INFO) << "mtkPowerHint hint: " << hint << " data: " << data;
            break;
        }
    }
    return Void();
}

Return<void> MtkPower::notifyAppState(const hidl_string& pack, const hidl_string& act, int32_t pid,
                                      int32_t state, int32_t uid) {
    LOG(INFO) << "notifyAppState pack: " << pack << " act: " << act << " pid: " << pid
              << " state: " << state << " uid: " << uid;
    return Void();
}

Return<int32_t> MtkPower::querySysInfo(int32_t cmd, int32_t param) {
    bool validCluster = param >= 0 && param < static_cast<int>(mClusters.size());
    int32_t ret;

    switch (cmd) {
        case MTKPOWER_CMD_GET_CLUSTER_NUM:
            ret = mClusters.size();
            break;
        case MTKPOWER_CMD_GET_CLUSTER_CPU_NUM:
            ret = validCluster ? mClusters[param].cpuNum : -1;
            break;
        case MTKPOWER_CMD_GET_CLUSTER_CPU_FREQ_MIN:
            ret = validCluster ? mClusters[param].freqMin : -1;
            break;
        case MTKPOWER_CMD_GET_CLUSTER_CPU_FREQ_MAX:
            ret = validCluster ? mClusters[param].freqMax : -1;
            break;
        default:
            ret = 0;
            break;
    }

    LOG(DEBUG) << "querySysInfo cmd: " << cmd << " param: " << param << " ret: " << ret;
    return ret;
}

Return<int32_t> MtkPower::setSysInfo(int32_t type, const hidl_string& data) {
    LOG(INFO) << "setSysInfo type: " << type << " data: " << data;
    return 0;
}

Return<void> MtkPower::setSysInfoAsync(int32_t type, const hidl_string& data) {
    LOG(INFO) << "setSysInfoAsync type: " << type << " data: " << data;
    return Void();
}

// Methods from ::vendor::mediatek::hardware::mtkpower::V1_1::IMtkPower follow.
Return<int32_t> MtkPower::setMtkPowerCallback(
        const sp<::vendor::mediatek::hardware::mtkpower::V1_1::IMtkPowerCallback>& /* callback */) {
    LOG(WARNING) << "setMtkPowerCallback";
    return 0;
}

// Methods from ::vendor::mediatek::hardware::mtkpower::V1_2::IMtkPower follow.
Return<int32_t> MtkPower::setMtkScnUpdateCallback(
        int32_t /* hint */,
        const sp<::vendor::mediatek::hardware::mtkpower::V1_2::IMtkPowerCallback>& /* callback */) {
    LOG(WARNING) << "setMtkScnUpdateCallback";
    return 0;
}

}  // namespace vendor::mediatek::hardware::mtkpower::implementation
