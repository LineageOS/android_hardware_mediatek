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

#include <aidl/android/hardware/bluetooth/audio/IBluetoothAudioPort.h>
#include <android/hardware/audio/common/5.0/types.h>
#include <vendor/mediatek/hardware/bluetooth/audio/2.1/IBluetoothAudioPort.h>

namespace vendor {
namespace mediatek {
namespace hardware {
namespace bluetooth {
namespace audio {
namespace adapter {

using IBluetoothAudioPort_2_1 =
        ::vendor::mediatek::hardware::bluetooth::audio::V2_1::IBluetoothAudioPort;
using SourceMetadata_5_0 = ::android::hardware::audio::common::V5_0::SourceMetadata;
using namespace ::aidl::android::hardware::bluetooth::audio;
using IBluetoothAudioPort_AIDL = ::aidl::android::hardware::bluetooth::audio::IBluetoothAudioPort;
using ::android::hardware::Return;

class HidlToAidlMiddleware_2_1 : public IBluetoothAudioPort_2_1 {
    std::shared_ptr<IBluetoothAudioPort_AIDL> port;

  public:
    HidlToAidlMiddleware_2_1(const std::shared_ptr<IBluetoothAudioPort_AIDL>& port);

    Return<void> startStream() override;

    Return<void> stopStream() override;

    Return<void> suspendStream() override;

    Return<void> getPresentationPosition(getPresentationPosition_cb _hidl_cb);

    Return<void> updateMetadata(const SourceMetadata_5_0& sourceMetadata);

    Return<void> enterGameMode(uint8_t enter) override;
};

}  // namespace adapter
}  // namespace audio
}  // namespace bluetooth
}  // namespace hardware
}  // namespace mediatek
}  // namespace vendor
