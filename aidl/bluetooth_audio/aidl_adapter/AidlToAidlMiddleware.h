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
#include <aidl/vendor/mediatek/hardware/bluetooth/audio/BnBluetoothAudioPort.h>

namespace vendor {
namespace mediatek {
namespace hardware {
namespace bluetooth {
namespace audio {
namespace adapter {

using ::aidl::android::hardware::audio::common::SinkMetadata;
using ::aidl::android::hardware::audio::common::SourceMetadata;
using namespace ::aidl::android::hardware::bluetooth::audio;
namespace mtk = ::aidl::vendor::mediatek::hardware::bluetooth::audio;

class AidlToAidlMiddleware : public mtk::BnBluetoothAudioPort {
  public:
    AidlToAidlMiddleware(
            const std::shared_ptr<::aidl::android::hardware::bluetooth::audio::IBluetoothAudioPort>&
                    port);

    ndk::ScopedAStatus startStream(bool is_low_latency) override;

    ndk::ScopedAStatus suspendStream() override;

    ndk::ScopedAStatus stopStream() override;

    ndk::ScopedAStatus getPresentationPosition(mtk::PresentationPosition* _aidl_return) override;

    ndk::ScopedAStatus updateSourceMetadata(const SourceMetadata& source_metadata) override;

    ndk::ScopedAStatus updateSinkMetadata(const SinkMetadata& sink_metadata) override;

    ndk::ScopedAStatus setLatencyMode(mtk::LatencyMode latency_mode) override;

    ndk::ScopedAStatus enterGameMode(int8_t enter) override;

  private:
    std::shared_ptr<::aidl::android::hardware::bluetooth::audio::IBluetoothAudioPort> port;
    ndk::SpAIBinder createBinder() override;
};

}  // namespace adapter
}  // namespace audio
}  // namespace bluetooth
}  // namespace hardware
}  // namespace mediatek
}  // namespace vendor
