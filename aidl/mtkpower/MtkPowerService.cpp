/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vendor.mediatek.hardware.mtkpower-service.stub"

#include <aidl/android/hardware/power/IPower.h>
#include <aidl/android/hardware/power/Mode.h>
#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/parseint.h>
#include <android-base/strings.h>
#include <android/binder_manager.h>

#include <algorithm>
#include <filesystem>

#include "MtkPowerService.h"

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

void MtkPowerService::loadClusters(void) {
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

MtkPowerService::MtkPowerService() {
    loadClusters();

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

void MtkPowerService::forwardAudioHint(int hint, int data) {
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
}

ndk::ScopedAStatus MtkPowerService::mtkPowerHint(int hint, int data) {
    forwardAudioHint(hint, data);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MtkPowerService::mtkCusPowerHint(int hint, int data) {
    forwardAudioHint(hint, data);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MtkPowerService::querySysInfo(int cmd, int param, int* _aidl_return) {
    bool validCluster = param >= 0 && param < static_cast<int>(mClusters.size());

    switch (cmd) {
        case MTKPOWER_CMD_GET_CLUSTER_NUM:
            *_aidl_return = mClusters.size();
            break;
        case MTKPOWER_CMD_GET_CLUSTER_CPU_NUM:
            *_aidl_return = validCluster ? mClusters[param].cpuNum : -1;
            break;
        case MTKPOWER_CMD_GET_CLUSTER_CPU_FREQ_MIN:
            *_aidl_return = validCluster ? mClusters[param].freqMin : -1;
            break;
        case MTKPOWER_CMD_GET_CLUSTER_CPU_FREQ_MAX:
            *_aidl_return = validCluster ? mClusters[param].freqMax : -1;
            break;
        default:
            *_aidl_return = 1;
            break;
    }

    LOG(DEBUG) << __func__ << ": cmd=" << cmd << ", param=" << param << ", ret=" << *_aidl_return;
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
        int /* scn */,
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
