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

#define LOG_TAG "BTAudioProviderStub"

#include <android-base/logging.h>
#include <ranges>

#include "AidlToAidlMiddleware.h"
#include "MTKBluetoothAudioProvider.h"

namespace vendor {
namespace mediatek {
namespace hardware {
namespace bluetooth {
namespace audio {
namespace adapter {

inline mtk::ChannelMode to_channel_mode_mtk(const ChannelMode& channelMode) {
    switch (channelMode) {
        case ChannelMode::UNKNOWN:
            return mtk::ChannelMode::UNKNOWN;
        case ChannelMode::MONO:
            return mtk::ChannelMode::MONO;
        case ChannelMode::STEREO:
            return mtk::ChannelMode::STEREO;
        case ChannelMode::DUALMONO:
            return mtk::ChannelMode::UNKNOWN;
    }
}

inline mtk::PcmConfiguration to_pcm_configuration_mtk(const PcmConfiguration& configuration) {
    return mtk::PcmConfiguration{
            .sampleRateHz = configuration.sampleRateHz,
            .channelMode = to_channel_mode_mtk(configuration.channelMode),
            .bitsPerSample = configuration.bitsPerSample,
            .dataIntervalUs = configuration.dataIntervalUs,
            /* TODO: should this be set based on dataIntervalUs being smaller than 20000? */
            .isLowLatencyEnabled = mtk::LowLatencyEnabled::Disabled,
    };
}

inline mtk::SbcChannelMode to_sbc_channel_mode_mtk(const SbcChannelMode& channelMode) {
    switch (channelMode) {
        case SbcChannelMode::UNKNOWN:
            return mtk::SbcChannelMode::UNKNOWN;
        case SbcChannelMode::JOINT_STEREO:
            return mtk::SbcChannelMode::JOINT_STEREO;
        case SbcChannelMode::STEREO:
            return mtk::SbcChannelMode::STEREO;
        case SbcChannelMode::DUAL:
            return mtk::SbcChannelMode::DUAL;
        case SbcChannelMode::MONO:
            return mtk::SbcChannelMode::MONO;
    }
}

inline mtk::SbcAllocMethod to_sbc_alloc_method_mtk(const SbcAllocMethod& allocMethod) {
    switch (allocMethod) {
        case SbcAllocMethod::ALLOC_MD_S:
            return mtk::SbcAllocMethod::ALLOC_MD_S;
        case SbcAllocMethod::ALLOC_MD_L:
            return mtk::SbcAllocMethod::ALLOC_MD_L;
    }
}

inline mtk::SbcConfiguration to_sbc_configuration_mtk(const SbcConfiguration& configuration) {
    return mtk::SbcConfiguration{
            .sampleRateHz = configuration.sampleRateHz,
            .channelMode = to_sbc_channel_mode_mtk(configuration.channelMode),
            .blockLength = configuration.blockLength,
            .numSubbands = configuration.numSubbands,
            .allocMethod = to_sbc_alloc_method_mtk(configuration.allocMethod),
            .bitsPerSample = configuration.bitsPerSample,
            .minBitpool = configuration.minBitpool,
            .maxBitpool = configuration.maxBitpool,
    };
}

inline mtk::AacObjectType to_aac_object_type_mtk(const AacObjectType& objectType) {
    switch (objectType) {
        case AacObjectType::MPEG2_LC:
            return mtk::AacObjectType::MPEG2_LC;
        case AacObjectType::MPEG4_LC:
            return mtk::AacObjectType::MPEG4_LC;
        case AacObjectType::MPEG4_LTP:
            return mtk::AacObjectType::MPEG4_LTP;
        case AacObjectType::MPEG4_SCALABLE:
            return mtk::AacObjectType::MPEG4_SCALABLE;
    }
}

inline mtk::AacConfiguration to_aac_configuration_mtk(const AacConfiguration& configuration) {
    return mtk::AacConfiguration{
            .objectType = to_aac_object_type_mtk(configuration.objectType),
            .sampleRateHz = configuration.sampleRateHz,
            .channelMode = to_channel_mode_mtk(configuration.channelMode),
            .variableBitRateEnabled = configuration.variableBitRateEnabled,
            .bitsPerSample = configuration.bitsPerSample,
    };
}

inline mtk::LdacChannelMode to_ldac_channel_mode_mtk(const LdacChannelMode& channelMode) {
    switch (channelMode) {
        case LdacChannelMode::UNKNOWN:
            return mtk::LdacChannelMode::UNKNOWN;
        case LdacChannelMode::STEREO:
            return mtk::LdacChannelMode::STEREO;
        case LdacChannelMode::DUAL:
            return mtk::LdacChannelMode::DUAL;
        case LdacChannelMode::MONO:
            return mtk::LdacChannelMode::MONO;
    }
}

inline mtk::LdacQualityIndex to_ldac_quality_index_mtk(const LdacQualityIndex& qualityIndex) {
    switch (qualityIndex) {
        case LdacQualityIndex::HIGH:
            return mtk::LdacQualityIndex::HIGH;
        case LdacQualityIndex::MID:
            return mtk::LdacQualityIndex::MID;
        case LdacQualityIndex::LOW:
            return mtk::LdacQualityIndex::LOW;
        case LdacQualityIndex::ABR:
            return mtk::LdacQualityIndex::ABR;
    }
}

inline mtk::LdacConfiguration to_ldac_configuration_mtk(const LdacConfiguration& configuration) {
    return mtk::LdacConfiguration{
            .sampleRateHz = configuration.sampleRateHz,
            .channelMode = to_ldac_channel_mode_mtk(configuration.channelMode),
            .qualityIndex = to_ldac_quality_index_mtk(configuration.qualityIndex),
            .bitsPerSample = configuration.bitsPerSample,
    };
}

inline mtk::AptxConfiguration to_aptx_configuration_mtk(const AptxConfiguration& configuration) {
    return mtk::AptxConfiguration{
            .sampleRateHz = configuration.sampleRateHz,
            .channelMode = to_channel_mode_mtk(configuration.channelMode),
            .bitsPerSample = configuration.bitsPerSample,
    };
}

inline mtk::AptxAdaptiveChannelMode to_aptx_adaptive_channel_mode_mtk(
        const AptxAdaptiveChannelMode& channelMode) {
    switch (channelMode) {
        case AptxAdaptiveChannelMode::JOINT_STEREO:
            return mtk::AptxAdaptiveChannelMode::JOINT_STEREO;
        case AptxAdaptiveChannelMode::MONO:
            return mtk::AptxAdaptiveChannelMode::MONO;
        case AptxAdaptiveChannelMode::DUAL_MONO:
            return mtk::AptxAdaptiveChannelMode::DUAL_MONO;
        case AptxAdaptiveChannelMode::TWS_STEREO:
            return mtk::AptxAdaptiveChannelMode::TWS_STEREO;
        case AptxAdaptiveChannelMode::UNKNOWN:
            return mtk::AptxAdaptiveChannelMode::UNKNOWN;
    }
}

inline mtk::AptxMode to_aptx_mode_mtk(const AptxMode& mode) {
    switch (mode) {
        case AptxMode::UNKNOWN:
            return mtk::AptxMode::UNKNOWN;
        case AptxMode::HIGH_QUALITY:
            return mtk::AptxMode::HIGH_QUALITY;
        case AptxMode::LOW_LATENCY:
            return mtk::AptxMode::LOW_LATENCY;
        case AptxMode::ULTRA_LOW_LATENCY:
            return mtk::AptxMode::ULTRA_LOW_LATENCY;
    }
}

inline mtk::AptxAdaptiveInputMode to_aptx_adaptive_input_mode_mtk(
        const AptxAdaptiveInputMode& inputMode) {
    switch (inputMode) {
        case AptxAdaptiveInputMode::STEREO:
            return mtk::AptxAdaptiveInputMode::STEREO;
        case AptxAdaptiveInputMode::DUAL_MONO:
            return mtk::AptxAdaptiveInputMode::DUAL_MONO;
    }
}

inline mtk::AptxAdaptiveConfiguration to_aptx_adaptive_configuration_mtk(
        const AptxAdaptiveConfiguration& configuration) {
    return mtk::AptxAdaptiveConfiguration{
            .sampleRateHz = configuration.sampleRateHz,
            .channelMode = to_aptx_adaptive_channel_mode_mtk(configuration.channelMode),
            .bitsPerSample = configuration.bitsPerSample,
            .aptxMode = to_aptx_mode_mtk(configuration.aptxMode),
            .sinkBufferingMs =
                    mtk::AptxSinkBuffering{
                            .minLowLatency = configuration.sinkBufferingMs.minLowLatency,
                            .maxLowLatency = configuration.sinkBufferingMs.maxLowLatency,
                            .minHighQuality = configuration.sinkBufferingMs.minHighQuality,
                            .maxHighQuality = configuration.sinkBufferingMs.maxHighQuality,
                            .minTws = configuration.sinkBufferingMs.minTws,
                            .maxTws = configuration.sinkBufferingMs.maxTws,
                    },
            .ttp =
                    mtk::AptxAdaptiveTimeToPlay{
                            .lowLowLatency = configuration.ttp.lowLowLatency,
                            .highLowLatency = configuration.ttp.highLowLatency,
                            .lowHighQuality = configuration.ttp.lowHighQuality,
                            .highHighQuality = configuration.ttp.highHighQuality,
                            .lowTws = configuration.ttp.lowTws,
                            .highTws = configuration.ttp.highTws,
                    },
            .inputMode = to_aptx_adaptive_input_mode_mtk(configuration.inputMode),
            .inputFadeDurationMs = configuration.inputFadeDurationMs,
            .aptxAdaptiveConfigStream = configuration.aptxAdaptiveConfigStream,
    };
}

inline mtk::Lc3Configuration to_lc3_configuration_mtk(const Lc3Configuration& configuration) {
    return mtk::Lc3Configuration{
            .pcmBitDepth = configuration.pcmBitDepth,
            .samplingFrequencyHz = configuration.samplingFrequencyHz,
            .frameDurationUs = configuration.frameDurationUs,
            .octetsPerFrame = configuration.octetsPerFrame,
            .blocksPerSdu = configuration.blocksPerSdu,
            .channelMode = to_channel_mode_mtk(configuration.channelMode),
    };
}

inline mtk::CodecConfiguration::VendorConfiguration to_codec_configuration_vendor_configuration_mtk(
        const CodecConfiguration::VendorConfiguration& configuration) {
    return mtk::CodecConfiguration::VendorConfiguration{
            .vendorId = configuration.vendorId,
            .codecId = configuration.codecId,
            .codecConfig = configuration.codecConfig,
    };
}

inline mtk::CodecConfiguration::CodecSpecific to_codec_configuration_codec_specific_mtk(
        const CodecConfiguration::CodecSpecific& configuration) {
    switch (configuration.getTag()) {
        case CodecConfiguration::CodecSpecific::sbcConfig:
            return mtk::CodecConfiguration::CodecSpecific::make<
                    mtk::CodecConfiguration::CodecSpecific::Tag::sbcConfig>(
                    to_sbc_configuration_mtk(
                            configuration
                                    .get<CodecConfiguration::CodecSpecific::Tag::sbcConfig>()));
        case CodecConfiguration::CodecSpecific::aacConfig:
            return mtk::CodecConfiguration::CodecSpecific::make<
                    mtk::CodecConfiguration::CodecSpecific::Tag::aacConfig>(
                    to_aac_configuration_mtk(
                            configuration
                                    .get<CodecConfiguration::CodecSpecific::Tag::aacConfig>()));
        case CodecConfiguration::CodecSpecific::ldacConfig:
            return mtk::CodecConfiguration::CodecSpecific::make<
                    mtk::CodecConfiguration::CodecSpecific::Tag::ldacConfig>(
                    to_ldac_configuration_mtk(
                            configuration
                                    .get<CodecConfiguration::CodecSpecific::Tag::ldacConfig>()));
        case CodecConfiguration::CodecSpecific::aptxConfig:
            return mtk::CodecConfiguration::CodecSpecific::make<
                    mtk::CodecConfiguration::CodecSpecific::Tag::aptxConfig>(
                    to_aptx_configuration_mtk(
                            configuration
                                    .get<CodecConfiguration::CodecSpecific::Tag::aptxConfig>()));
        case CodecConfiguration::CodecSpecific::aptxAdaptiveConfig:
            return mtk::CodecConfiguration::CodecSpecific::make<
                    mtk::CodecConfiguration::CodecSpecific::Tag::aptxAdaptiveConfig>(
                    to_aptx_adaptive_configuration_mtk(
                            configuration.get<
                                    CodecConfiguration::CodecSpecific::Tag::aptxAdaptiveConfig>()));
        case CodecConfiguration::CodecSpecific::lc3Config:
            return mtk::CodecConfiguration::CodecSpecific::make<
                    mtk::CodecConfiguration::CodecSpecific::Tag::lc3Config>(
                    to_lc3_configuration_mtk(
                            configuration
                                    .get<CodecConfiguration::CodecSpecific::Tag::lc3Config>()));
        case CodecConfiguration::CodecSpecific::vendorConfig:
            return mtk::CodecConfiguration::CodecSpecific::make<
                    mtk::CodecConfiguration::CodecSpecific::Tag::vendorConfig>(
                    to_codec_configuration_vendor_configuration_mtk(
                            configuration
                                    .get<CodecConfiguration::CodecSpecific::Tag::vendorConfig>()));
        case CodecConfiguration::CodecSpecific::opusConfig:  // TODO: Implement this
            return mtk::CodecConfiguration::CodecSpecific::make<
                    mtk::CodecConfiguration::CodecSpecific::Tag::vendorConfig>(
                    mtk::CodecConfiguration::VendorConfiguration{});
    }
}

inline mtk::CodecType to_codec_type_mtk(const CodecType& codecType) {
    switch (codecType) {
        case CodecType::UNKNOWN:
            return mtk::CodecType::UNKNOWN;
        case CodecType::SBC:
            return mtk::CodecType::SBC;
        case CodecType::AAC:
            return mtk::CodecType::AAC;
        case CodecType::APTX:
            return mtk::CodecType::APTX;
        case CodecType::APTX_HD:
            return mtk::CodecType::APTX_HD;
        case CodecType::LDAC:
            return mtk::CodecType::LDAC;
        case CodecType::LC3:
            return mtk::CodecType::LC3;
        case CodecType::VENDOR:
            return mtk::CodecType::VENDOR;
        case CodecType::APTX_ADAPTIVE:
            return mtk::CodecType::APTX_ADAPTIVE;
        case CodecType::OPUS:
            return mtk::CodecType::UNKNOWN;
        case CodecType::APTX_ADAPTIVE_LE:
            return mtk::CodecType::UNKNOWN;
        case CodecType::APTX_ADAPTIVE_LEX:
            return mtk::CodecType::UNKNOWN;
    }
}

inline mtk::CodecConfiguration to_codec_configuration_mtk(const CodecConfiguration& configuration) {
    return mtk::CodecConfiguration{
            .codecType = to_codec_type_mtk(configuration.codecType),
            .encodedAudioBitrate = configuration.encodedAudioBitrate,
            .peerMtu = configuration.peerMtu,
            .isScmstEnabled = configuration.isScmstEnabled,
            .config = to_codec_configuration_codec_specific_mtk(configuration.config),
    };
}

inline mtk::LeAudioCodecConfiguration to_le_audio_codec_configuration_mtk(
        const LeAudioCodecConfiguration& configuration) {
    switch (configuration.getTag()) {
        case LeAudioCodecConfiguration::Tag::lc3Config:
            return mtk::LeAudioCodecConfiguration::make<
                    mtk::LeAudioCodecConfiguration::Tag::lc3Config>(to_lc3_configuration_mtk(
                    configuration.get<LeAudioCodecConfiguration::Tag::lc3Config>()));
        case LeAudioCodecConfiguration::Tag::vendorConfig:
            return mtk::LeAudioCodecConfiguration::make<
                    mtk::LeAudioCodecConfiguration::Tag::vendorConfig>(
                    mtk::LeAudioCodecConfiguration::VendorConfiguration{
                            .extension =
                                    configuration
                                            .get<LeAudioCodecConfiguration::Tag::vendorConfig>()
                                            .extension});
        case LeAudioCodecConfiguration::Tag::aptxAdaptiveLeConfig:  // TODO: Implement this
            return mtk::LeAudioCodecConfiguration::make<
                    mtk::LeAudioCodecConfiguration::Tag::vendorConfig>(
                    mtk::LeAudioCodecConfiguration::VendorConfiguration{});
        case LeAudioCodecConfiguration::Tag::opusConfig:  // TODO: Implement this
            return mtk::LeAudioCodecConfiguration::make<
                    mtk::LeAudioCodecConfiguration::Tag::vendorConfig>(
                    mtk::LeAudioCodecConfiguration::VendorConfiguration{});
    }
}

inline mtk::LeAudioConfiguration to_le_audio_configuration_mtk(
        const LeAudioConfiguration& configuration) {
    return mtk::LeAudioConfiguration{
            .codecType = to_codec_type_mtk(configuration.codecType),
            .streamMap = configuration.streamMap | std::views::transform([](auto streamMap) {
                             return mtk::LeAudioConfiguration::StreamMap{
                                     .streamHandle = streamMap.streamHandle,
                                     .audioChannelAllocation = streamMap.audioChannelAllocation,
                             };
                         }) |
                         std::ranges::to<std::vector>(),
            .peerDelayUs = configuration.peerDelayUs,
            .leAudioCodecConfig =
                    to_le_audio_codec_configuration_mtk(configuration.leAudioCodecConfig),
    };
}

inline mtk::LeAudioBroadcastConfiguration to_le_audio_broadcast_configuration_mtk(
        const LeAudioBroadcastConfiguration& configuration) {
    return mtk::LeAudioBroadcastConfiguration{
            .codecType = to_codec_type_mtk(configuration.codecType),
            .streamMap = configuration.streamMap | std::views::transform([](auto streamMap) {
                             return mtk::LeAudioBroadcastConfiguration::BroadcastStreamMap{
                                     .streamHandle = streamMap.streamHandle,
                                     .audioChannelAllocation = streamMap.audioChannelAllocation,
                                     .leAudioCodecConfig = to_le_audio_codec_configuration_mtk(
                                             streamMap.leAudioCodecConfig),
                             };
                         }) |
                         std::ranges::to<std::vector>(),
    };
}

inline mtk::AudioConfiguration to_mtk_audio_configuration(const AudioConfiguration& configuration) {
    switch (configuration.getTag()) {
        case AudioConfiguration::Tag::pcmConfig:
            return mtk::AudioConfiguration::make<mtk::AudioConfiguration::Tag::pcmConfig>(
                    to_pcm_configuration_mtk(
                            configuration.get<AudioConfiguration::Tag::pcmConfig>()));
        case AudioConfiguration::Tag::a2dpConfig:
            return mtk::AudioConfiguration::make<mtk::AudioConfiguration::Tag::a2dpConfig>(
                    to_codec_configuration_mtk(
                            configuration.get<AudioConfiguration::Tag::a2dpConfig>()));
        case AudioConfiguration::Tag::leAudioConfig:
            return mtk::AudioConfiguration::make<mtk::AudioConfiguration::Tag::leAudioConfig>(
                    to_le_audio_configuration_mtk(
                            configuration.get<AudioConfiguration::Tag::leAudioConfig>()));
        case AudioConfiguration::Tag::leAudioBroadcastConfig:
            return mtk::AudioConfiguration::make<
                    mtk::AudioConfiguration::Tag::leAudioBroadcastConfig>(
                    to_le_audio_broadcast_configuration_mtk(
                            configuration.get<AudioConfiguration::Tag::leAudioBroadcastConfig>()));
        case AudioConfiguration::Tag::hfpConfig:  // TODO: Implement this
            return mtk::AudioConfiguration::make<mtk::AudioConfiguration::Tag::a2dpConfig>(
                    mtk::CodecConfiguration{});
        case AudioConfiguration::Tag::a2dp:  // TODO: Implement this
            return mtk::AudioConfiguration::make<mtk::AudioConfiguration::Tag::a2dpConfig>(
                    mtk::CodecConfiguration{});
    }
}

inline mtk::BluetoothAudioStatus to_mtk_status(const BluetoothAudioStatus& status) {
    switch (status) {
        case BluetoothAudioStatus::UNKNOWN:
            return mtk::BluetoothAudioStatus::UNKNOWN;
        case BluetoothAudioStatus::SUCCESS:
            return mtk::BluetoothAudioStatus::SUCCESS;
        case BluetoothAudioStatus::UNSUPPORTED_CODEC_CONFIGURATION:
            return mtk::BluetoothAudioStatus::UNSUPPORTED_CODEC_CONFIGURATION;
        case BluetoothAudioStatus::FAILURE:
            return mtk::BluetoothAudioStatus::FAILURE;
        case BluetoothAudioStatus::RECONFIGURATION:
            return mtk::BluetoothAudioStatus::RECONFIGURATION;
    }
}

struct BluetoothAudioProviderContext {
    std::shared_ptr<MTKBluetoothAudioProvider> provider;
};

static void binderUnlinkedCallbackAidl(void* cookie) {
    LOG(INFO) << __func__;
    BluetoothAudioProviderContext* ctx = static_cast<BluetoothAudioProviderContext*>(cookie);
    delete ctx;
}

static void binderDiedCallbackAidl(void* cookie) {
    LOG(INFO) << __func__;
    BluetoothAudioProviderContext* ctx = static_cast<BluetoothAudioProviderContext*>(cookie);
    CHECK_NE(ctx, nullptr);

    ctx->provider->endSession();
}

MTKBluetoothAudioProvider::MTKBluetoothAudioProvider(
        std::shared_ptr<mtk::IBluetoothAudioProvider> provider)
    : provider(provider) {
    death_recipient_ = ::ndk::ScopedAIBinder_DeathRecipient(
            AIBinder_DeathRecipient_new(binderDiedCallbackAidl));
    AIBinder_DeathRecipient_setOnUnlinked(death_recipient_.get(), binderUnlinkedCallbackAidl);
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::startSession(
        const std::shared_ptr<IBluetoothAudioPort>& host_if, const AudioConfiguration& audio_config,
        const std::vector<LatencyMode>& latencyModes, DataMQDesc* _aidl_return) {
    if (host_if == nullptr) {
        *_aidl_return = DataMQDesc();
        LOG(ERROR) << __func__ << " Illegal argument";
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }
    auto stack_if = ::ndk::SharedRefBase::make<AidlToAidlMiddleware>(host_if);
    auto retval = provider->startSession(
            stack_if, to_mtk_audio_configuration(audio_config),
            latencyModes | std::views::transform([](const LatencyMode& latencyMode) {
                switch (latencyMode) {
                    case LatencyMode::UNKNOWN:
                        return mtk::LatencyMode::UNKNOWN;
                    case LatencyMode::LOW_LATENCY:
                        return mtk::LatencyMode::LOW_LATENCY;
                    case LatencyMode::FREE:
                        return mtk::LatencyMode::FREE;
                    case LatencyMode::DYNAMIC_SPATIAL_AUDIO_SOFTWARE:
                        return mtk::LatencyMode::LOW_LATENCY;  // TODO: is this ok?
                    case LatencyMode::DYNAMIC_SPATIAL_AUDIO_HARDWARE:
                        return mtk::LatencyMode::LOW_LATENCY;  // TODO: is this ok?
                }
            }) | std::ranges::to<std::vector>(),
            _aidl_return);
    if (retval.isOk()) {
        stack_iface_ = host_if;
        BluetoothAudioProviderContext* cookie =
                new BluetoothAudioProviderContext{ref<MTKBluetoothAudioProvider>()};
        AIBinder_linkToDeath(stack_iface_->asBinder().get(), death_recipient_.get(), cookie);
        return ndk::ScopedAStatus::ok();
    } else {
        return retval;
    }
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::endSession() {
    if (stack_iface_ != nullptr) {
        AIBinder_unlinkToDeath(stack_iface_->asBinder().get(), death_recipient_.get(), this);
        stack_iface_ = nullptr;
    } else {
        LOG(INFO) << __func__ << " - has NO session";
    }
    return provider->endSession();
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::streamStarted(BluetoothAudioStatus status) {
    return provider->streamStarted(to_mtk_status(status));
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::streamSuspended(BluetoothAudioStatus status) {
    return provider->streamSuspended(to_mtk_status(status));
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::updateAudioConfiguration(
        const AudioConfiguration& audio_config) {
    return provider->updateAudioConfiguration(to_mtk_audio_configuration(audio_config));
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::setLowLatencyModeAllowed(bool allowed) {
    /* TODO: maybe we want to call enterGameMode somewhere...? */
    return provider->setLowLatencyModeAllowed(allowed);
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::parseA2dpConfiguration(
        [[maybe_unused]] const CodecId& codec_id,
        [[maybe_unused]] const std::vector<uint8_t>& configuration,
        [[maybe_unused]] CodecParameters* codec_parameters,
        [[maybe_unused]] A2dpStatus* _aidl_return) {
    LOG(INFO) << __func__ << " - is illegal";
    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::getA2dpConfiguration(
        [[maybe_unused]] const std::vector<A2dpRemoteCapabilities>& remote_a2dp_capabilities,
        [[maybe_unused]] const A2dpConfigurationHint& hint,
        [[maybe_unused]] std::optional<A2dpConfiguration>* _aidl_return) {
    LOG(INFO) << __func__ << " - is illegal";

    return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::setCodecPriority(
        const ::aidl::android::hardware::bluetooth::audio::CodecId& in_codecId,
        int32_t in_priority) {
    (void)in_codecId;
    (void)in_priority;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
};

ndk::ScopedAStatus MTKBluetoothAudioProvider::getLeAudioAseConfiguration(
        const std::optional<std::vector<
                std::optional<::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::
                                      LeAudioDeviceCapabilities>>>& in_remoteSinkAudioCapabilities,
        const std::optional<std::vector<std::optional<
                ::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::
                        LeAudioDeviceCapabilities>>>& in_remoteSourceAudioCapabilities,
        const std::vector<::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::
                                  LeAudioConfigurationRequirement>& in_requirements,
        std::vector<::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::
                            LeAudioAseConfigurationSetting>* _aidl_return) {
    (void)in_remoteSinkAudioCapabilities;
    (void)in_remoteSourceAudioCapabilities;
    (void)in_requirements;
    (void)_aidl_return;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
};

ndk::ScopedAStatus MTKBluetoothAudioProvider::getLeAudioAseQosConfiguration(
        const ::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::
                LeAudioAseQosConfigurationRequirement& in_qosRequirement,
        ::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::
                LeAudioAseQosConfigurationPair* _aidl_return) {
    (void)in_qosRequirement;
    (void)_aidl_return;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
};

ndk::ScopedAStatus MTKBluetoothAudioProvider::getLeAudioAseDatapathConfiguration(
        const std::optional<
                ::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::StreamConfig>&
                in_sinkConfig,
        const std::optional<
                ::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::StreamConfig>&
                in_sourceConfig,
        ::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::
                LeAudioDataPathConfigurationPair* _aidl_return) {
    (void)in_sinkConfig;
    (void)in_sourceConfig;
    (void)_aidl_return;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::onSinkAseMetadataChanged(
        ::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::AseState in_state,
        int32_t cigId, int32_t cisId,
        const std::optional<std::vector<
                std::optional<::aidl::android::hardware::bluetooth::audio::MetadataLtv>>>&
                in_metadata) {
    (void)in_state;
    (void)cigId;
    (void)cisId;
    (void)in_metadata;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
};

ndk::ScopedAStatus MTKBluetoothAudioProvider::onSourceAseMetadataChanged(
        ::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::AseState in_state,
        int32_t cigId, int32_t cisId,
        const std::optional<std::vector<
                std::optional<::aidl::android::hardware::bluetooth::audio::MetadataLtv>>>&
                in_metadata) {
    (void)in_state;
    (void)cigId;
    (void)cisId;
    (void)in_metadata;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
};

ndk::ScopedAStatus MTKBluetoothAudioProvider::getLeAudioBroadcastConfiguration(
        const std::optional<std::vector<
                std::optional<::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::
                                      LeAudioDeviceCapabilities>>>& in_remoteSinkAudioCapabilities,
        const ::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::
                LeAudioBroadcastConfigurationRequirement& in_requirement,
        ::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::
                LeAudioBroadcastConfigurationSetting* _aidl_return) {
    (void)in_remoteSinkAudioCapabilities;
    (void)in_requirement;
    (void)_aidl_return;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
};

ndk::ScopedAStatus MTKBluetoothAudioProvider::getLeAudioBroadcastDatapathConfiguration(
        const ::aidl::android::hardware::bluetooth::audio::AudioContext& in_context,
        const std::vector<::aidl::android::hardware::bluetooth::audio::
                                  LeAudioBroadcastConfiguration::BroadcastStreamMap>& in_streamMap,
        ::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::
                LeAudioDataPathConfiguration* _aidl_return) {
    (void)in_context;
    (void)in_streamMap;
    (void)_aidl_return;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::getLeAudioAseCodecConfiguredParameters(
        const std::optional<std::vector<std::optional<
                ::aidl::android::hardware::bluetooth::audio::LeAudioAseConfiguration>>>&
                in_sinkAseConfiguration,
        const std::optional<std::vector<std::optional<
                ::aidl::android::hardware::bluetooth::audio::LeAudioAseConfiguration>>>&
                in_sourceAseConfiguration,
        std::optional<::aidl::android::hardware::bluetooth::audio::IBluetoothAudioProvider::
                              LeAudioAseCodecConfiguredResponse>* _aidl_return) {
    (void)in_sinkAseConfiguration;
    (void)in_sourceAseConfiguration;
    (void)_aidl_return;
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

}  // namespace adapter
}  // namespace audio
}  // namespace bluetooth
}  // namespace hardware
}  // namespace mediatek
}  // namespace vendor
