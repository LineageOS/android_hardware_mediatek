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

#define LOG_TAG "BtAudioNakahara"

#include <aidl/android/hardware/audio/common/SourceMetadata.h>
#include <aidl/android/hardware/bluetooth/audio/AudioConfiguration.h>
#include <aidl/android/hardware/bluetooth/audio/BluetoothAudioStatus.h>
#include <aidl/android/hardware/bluetooth/audio/SessionType.h>
#include <android-base/logging.h>

#include <functional>
#include <unordered_map>

#include "HidlToAidlMiddleware_2_1.h"

namespace vendor {
namespace mediatek {
namespace hardware {
namespace bluetooth {
namespace audio {
namespace adapter {

using ::aidl::android::hardware::audio::common::PlaybackTrackMetadata;
using ::aidl::android::hardware::audio::common::SourceMetadata;
using ::aidl::android::media::audio::common::AudioContentType;
using ::aidl::android::media::audio::common::AudioUsage;
using ::android::hardware::Void;
using HidlStatus = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::Status;

HidlToAidlMiddleware_2_1::HidlToAidlMiddleware_2_1(
        const std::shared_ptr<IBluetoothAudioPort_AIDL>& port)
    : port(port) {}

Return<void> HidlToAidlMiddleware_2_1::startStream() {
    port->startStream(false);
    return Void();
}

Return<void> HidlToAidlMiddleware_2_1::stopStream() {
    port->stopStream();
    return Void();
}

Return<void> HidlToAidlMiddleware_2_1::suspendStream() {
    port->suspendStream();
    return Void();
}

Return<void> HidlToAidlMiddleware_2_1::getPresentationPosition(
        getPresentationPosition_cb _hidl_cb) {
    PresentationPosition presentation_position;
    auto ret_val = port->getPresentationPosition(&presentation_position);
    if (ret_val.isOk()) {
        _hidl_cb(HidlStatus::SUCCESS, presentation_position.remoteDeviceAudioDelayNanos,
                 presentation_position.transmittedOctets,
                 {.tvSec = static_cast<uint64_t>(
                          presentation_position.transmittedOctetsTimestamp.tvSec),
                  .tvNSec = static_cast<uint64_t>(
                          presentation_position.transmittedOctetsTimestamp.tvNSec)});
    } else {
        _hidl_cb(HidlStatus::FAILURE, 0, 0, {.tvSec = 0, .tvNSec = 0});
    }
    return Void();
}

Return<void> HidlToAidlMiddleware_2_1::updateMetadata(const SourceMetadata_5_0& sourceMetadata) {
    std::vector<PlaybackTrackMetadata> metadata_vec;
    metadata_vec.reserve(sourceMetadata.tracks.size());
    for (const auto& metadata : sourceMetadata.tracks) {
        metadata_vec.push_back({
                .usage = static_cast<AudioUsage>(metadata.usage),
                .contentType = static_cast<AudioContentType>(metadata.contentType),
                .gain = metadata.gain,
        });
    }
    port->updateSourceMetadata(SourceMetadata{metadata_vec});
    return Void();
}

Return<void> HidlToAidlMiddleware_2_1::enterGameMode(uint8_t enter) {
    /* TODO: Implement */
    LOG(FATAL) << "Unsupported method enterGameMode called with argument: " << enter;
    return Void();
}

}  // namespace adapter
}  // namespace audio
}  // namespace bluetooth
}  // namespace hardware
}  // namespace mediatek
}  // namespace vendor
