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

#include <aidl/android/hardware/bluetooth/audio/BnBluetoothAudioProvider.h>
#include <aidl/vendor/mediatek/hardware/bluetooth/audio/IBluetoothAudioProvider.h>

using ::aidl::android::hardware::common::fmq::MQDescriptor;
using ::aidl::android::hardware::common::fmq::SynchronizedReadWrite;

using DataMQDesc = MQDescriptor<int8_t, SynchronizedReadWrite>;

namespace vendor {
namespace mediatek {
namespace hardware {
namespace bluetooth {
namespace audio {
namespace adapter {
using namespace ::aidl::android::hardware::bluetooth::audio;
namespace mtk = ::aidl::vendor::mediatek::hardware::bluetooth::audio;

class MTKBluetoothAudioProvider : public BnBluetoothAudioProvider {
    std::shared_ptr<mtk::IBluetoothAudioProvider> provider;

  public:
    MTKBluetoothAudioProvider(std::shared_ptr<mtk::IBluetoothAudioProvider> provider);
    ndk::ScopedAStatus startSession(const std::shared_ptr<IBluetoothAudioPort>& host_if,
                                    const AudioConfiguration& audio_config,
                                    const std::vector<LatencyMode>& latency_modes,
                                    DataMQDesc* _aidl_return);
    ndk::ScopedAStatus endSession();
    ndk::ScopedAStatus streamStarted(BluetoothAudioStatus status);
    ndk::ScopedAStatus streamSuspended(BluetoothAudioStatus status);
    ndk::ScopedAStatus updateAudioConfiguration(const AudioConfiguration& audio_config);
    ndk::ScopedAStatus setLowLatencyModeAllowed(bool allowed);
    ndk::ScopedAStatus setCodecPriority(const CodecId& in_codecId, int32_t in_priority) override;
    ndk::ScopedAStatus getLeAudioAseConfiguration(
            const std::optional<
                    std::vector<std::optional<IBluetoothAudioProvider::LeAudioDeviceCapabilities>>>&
                    in_remoteSinkAudioCapabilities,
            const std::optional<
                    std::vector<std::optional<IBluetoothAudioProvider::LeAudioDeviceCapabilities>>>&
                    in_remoteSourceAudioCapabilities,
            const std::vector<IBluetoothAudioProvider::LeAudioConfigurationRequirement>&
                    in_requirements,
            std::vector<IBluetoothAudioProvider::LeAudioAseConfigurationSetting>* _aidl_return)
            override;
    ndk::ScopedAStatus getLeAudioAseQosConfiguration(
            const IBluetoothAudioProvider::LeAudioAseQosConfigurationRequirement& in_qosRequirement,
            IBluetoothAudioProvider::LeAudioAseQosConfigurationPair* _aidl_return) override;
    ndk::ScopedAStatus getLeAudioAseDatapathConfiguration(
            const std::optional<IBluetoothAudioProvider::StreamConfig>& in_sinkConfig,
            const std::optional<IBluetoothAudioProvider::StreamConfig>& in_sourceConfig,
            IBluetoothAudioProvider::LeAudioDataPathConfigurationPair* _aidl_return) override;
    ndk::ScopedAStatus onSinkAseMetadataChanged(
            IBluetoothAudioProvider::AseState in_state, int32_t cigId, int32_t cisId,
            const std::optional<std::vector<std::optional<MetadataLtv>>>& in_metadata) override;
    ndk::ScopedAStatus onSourceAseMetadataChanged(
            IBluetoothAudioProvider::AseState in_state, int32_t cigId, int32_t cisId,
            const std::optional<std::vector<std::optional<MetadataLtv>>>& in_metadata) override;
    ndk::ScopedAStatus getLeAudioBroadcastConfiguration(
            const std::optional<
                    std::vector<std::optional<IBluetoothAudioProvider::LeAudioDeviceCapabilities>>>&
                    in_remoteSinkAudioCapabilities,
            const IBluetoothAudioProvider::LeAudioBroadcastConfigurationRequirement& in_requirement,
            IBluetoothAudioProvider::LeAudioBroadcastConfigurationSetting* _aidl_return) override;
    ndk::ScopedAStatus getLeAudioBroadcastDatapathConfiguration(
            const AudioContext& in_context,
            const std::vector<LeAudioBroadcastConfiguration::BroadcastStreamMap>& in_streamMap,
            IBluetoothAudioProvider::LeAudioDataPathConfiguration* _aidl_return) override;

    ndk::ScopedAStatus parseA2dpConfiguration(const CodecId& codec_id,
                                              const std::vector<uint8_t>& configuration,
                                              CodecParameters* codec_parameters,
                                              A2dpStatus* _aidl_return);
    ndk::ScopedAStatus getA2dpConfiguration(
            const std::vector<A2dpRemoteCapabilities>& remote_a2dp_capabilities,
            const A2dpConfigurationHint& hint, std::optional<A2dpConfiguration>* _aidl_return);

  protected:
    std::shared_ptr<IBluetoothAudioPort> stack_iface_;
    ::ndk::ScopedAIBinder_DeathRecipient death_recipient_;
};

}  // namespace adapter
}  // namespace audio
}  // namespace bluetooth
}  // namespace hardware
}  // namespace mediatek
}  // namespace vendor
