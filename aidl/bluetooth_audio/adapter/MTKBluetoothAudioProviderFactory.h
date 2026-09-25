/*
 * Copyright (C) 2022 The Android Open Source Project
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

#pragma once

#include <BluetoothAudioProviderFactory.h>
#include <aidl/android/hardware/bluetooth/audio/BnBluetoothAudioProviderFactory.h>
#include <vendor/mediatek/hardware/bluetooth/audio/2.1/IBluetoothAudioProvidersFactory.h>
#include <vendor/mediatek/hardware/bluetooth/audio/2.2/IBluetoothAudioProvidersFactory.h>

namespace vendor {
namespace mediatek {
namespace hardware {
namespace bluetooth {
namespace audio {
namespace adapter {
using namespace ::aidl::android::hardware::bluetooth::audio;

class MTKBluetoothAudioProviderFactory
    : public ::aidl::android::hardware::bluetooth::audio::BnBluetoothAudioProviderFactory {
    using BluetoothAudioProviderFactorySw =
            ::aidl::android::hardware::bluetooth::audio::BluetoothAudioProviderFactory;
    using BluetoothAudioProviderFactory_2_1 =
            ::vendor::mediatek::hardware::bluetooth::audio::V2_1::IBluetoothAudioProvidersFactory;
    using BluetoothAudioProviderFactory_2_2 =
            ::vendor::mediatek::hardware::bluetooth::audio::V2_2::IBluetoothAudioProvidersFactory;
    BluetoothAudioProviderFactorySw factory_sw;
    android::sp<BluetoothAudioProviderFactory_2_1> factory_2_1;
    android::sp<BluetoothAudioProviderFactory_2_2> factory_2_2;
    bool factoriesLoaded;
    void loadFactoriesIfNeeded();

  public:
    MTKBluetoothAudioProviderFactory();

    ndk::ScopedAStatus openProvider(
            const SessionType session_type,
            std::shared_ptr<IBluetoothAudioProvider>* _aidl_return) override;

    ndk::ScopedAStatus getProviderCapabilities(
            const SessionType session_type, std::vector<AudioCapabilities>* _aidl_return) override;

    ndk::ScopedAStatus getProviderInfo(SessionType in_sessionType,
                                       std::optional<ProviderInfo>* _aidl_return) override;
};

}  // namespace adapter
}  // namespace audio
}  // namespace bluetooth
}  // namespace hardware
}  // namespace mediatek
}  // namespace vendor
