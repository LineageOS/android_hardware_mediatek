/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/vendor/mediatek/hardware/mtkpower/BnMtkPowerService.h>

#define MTKPOWER_HINT_AUDIO_LATENCY_DL 36
#define MTKPOWER_HINT_AUDIO_LATENCY_UL 37
#define MTKPOWER_HINT_AUDIO_POWER_DL 38
#define MTKPOWER_HINT_AUDIO_POWER_UL 44
#define MTKPOWER_HINT_AUDIO_HAL_OPEN 55

namespace aidl {
namespace vendor {
namespace mediatek {
namespace hardware {
namespace mtkpower {

class MtkPowerService : public BnMtkPowerService {
  public:
    MtkPowerService();
    bool getAidlPowerHal(void);
    ndk::ScopedAStatus perfLockAcquire(int hdl, int duration, const std::vector<int>& boostList,
                                       int pid, int reserved, int* _aidl_return) override;
    ndk::ScopedAStatus perfCusLockHint(int hint, int duration, int pid, int* _aidl_return) override;
    ndk::ScopedAStatus perfLockRelease(int hdl, int reserved) override;
    ndk::ScopedAStatus perfLockReleaseSync(int hdl, int reserved, int* _aidl_return) override;
    ndk::ScopedAStatus mtkPowerHint(int hint, int data) override;
    ndk::ScopedAStatus mtkCusPowerHint(int hint, int data) override;
    ndk::ScopedAStatus querySysInfo(int cmd, int param, int* _aidl_return) override;
    ndk::ScopedAStatus setSysInfo(int type, const std::string& data, int* _aidl_return) override;
    ndk::ScopedAStatus setSysInfoAsync(int type, const std::string& data) override;
    ndk::ScopedAStatus setMtkPowerCallback(
            const std::shared_ptr<::aidl::vendor::mediatek::hardware::mtkpower::IMtkPowerCallback>&
                    callback,
            int* _aidl_return) override;
    ndk::ScopedAStatus setMtkScnUpdateCallback(
            int scn,
            const std::shared_ptr<::aidl::vendor::mediatek::hardware::mtkpower::IMtkPowerCallback>&
                    callback,
            int* _aidl_return) override;
};

}  // namespace mtkpower
}  // namespace hardware
}  // namespace mediatek
}  // namespace vendor
}  // namespace aidl
