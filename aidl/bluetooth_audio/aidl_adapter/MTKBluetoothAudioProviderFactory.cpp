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

#include <ranges>
#include <android/binder_manager.h>

#include "MTKBluetoothAudioProvider.h"
#include "MTKBluetoothAudioProviderFactory.h"

namespace vendor {
namespace mediatek {
namespace hardware {
namespace bluetooth {
namespace audio {
namespace adapter {

const static std::unordered_map<SessionType, mtk::SessionType> session_type_aidl_to_mtk_map{
        {SessionType::A2DP_SOFTWARE_ENCODING_DATAPATH,
         mtk::SessionType::A2DP_SOFTWARE_ENCODING_DATAPATH},
        {SessionType::A2DP_HARDWARE_OFFLOAD_ENCODING_DATAPATH,
         mtk::SessionType::A2DP_HARDWARE_OFFLOAD_ENCODING_DATAPATH},
        {SessionType::HEARING_AID_SOFTWARE_ENCODING_DATAPATH,
         mtk::SessionType::HEARING_AID_SOFTWARE_ENCODING_DATAPATH},
        {SessionType::LE_AUDIO_SOFTWARE_ENCODING_DATAPATH,
         mtk::SessionType::LE_AUDIO_SOFTWARE_ENCODING_DATAPATH},
        {SessionType::LE_AUDIO_SOFTWARE_DECODING_DATAPATH,
         mtk::SessionType::LE_AUDIO_SOFTWARE_DECODING_DATAPATH},
        {SessionType::LE_AUDIO_HARDWARE_OFFLOAD_ENCODING_DATAPATH,
         mtk::SessionType::LE_AUDIO_HARDWARE_OFFLOAD_ENCODING_DATAPATH},
        {SessionType::LE_AUDIO_HARDWARE_OFFLOAD_DECODING_DATAPATH,
         mtk::SessionType::LE_AUDIO_HARDWARE_OFFLOAD_DECODING_DATAPATH},
        {SessionType::LE_AUDIO_BROADCAST_SOFTWARE_ENCODING_DATAPATH,
         mtk::SessionType::LE_AUDIO_BROADCAST_SOFTWARE_ENCODING_DATAPATH},
        {SessionType::LE_AUDIO_BROADCAST_HARDWARE_OFFLOAD_ENCODING_DATAPATH,
         mtk::SessionType::LE_AUDIO_BROADCAST_HARDWARE_OFFLOAD_ENCODING_DATAPATH},
        {SessionType::A2DP_SOFTWARE_DECODING_DATAPATH,
         mtk::SessionType::A2DP_SOFTWARE_DECODING_DATAPATH},
        {SessionType::A2DP_HARDWARE_OFFLOAD_DECODING_DATAPATH,
         mtk::SessionType::A2DP_HARDWARE_OFFLOAD_DECODING_DATAPATH},
};

inline mtk::SessionType to_session_type_mtk(const SessionType& session_type_aidl) {
    auto it = session_type_aidl_to_mtk_map.find(session_type_aidl);
    if (it != session_type_aidl_to_mtk_map.end()) return it->second;
    return mtk::SessionType::UNKNOWN;
}

inline bool isHardwareSessionType(const SessionType& session_type_aidl) {
    switch (session_type_aidl) {
        case SessionType::A2DP_HARDWARE_OFFLOAD_ENCODING_DATAPATH:
        case SessionType::LE_AUDIO_HARDWARE_OFFLOAD_ENCODING_DATAPATH:
        case SessionType::LE_AUDIO_HARDWARE_OFFLOAD_DECODING_DATAPATH:
        case SessionType::LE_AUDIO_BROADCAST_HARDWARE_OFFLOAD_ENCODING_DATAPATH:
        case SessionType::HFP_HARDWARE_OFFLOAD_DATAPATH:
        case SessionType::A2DP_HARDWARE_OFFLOAD_DECODING_DATAPATH:
            return true;
        case SessionType::A2DP_SOFTWARE_ENCODING_DATAPATH:
        case SessionType::HEARING_AID_SOFTWARE_ENCODING_DATAPATH:
        case SessionType::LE_AUDIO_SOFTWARE_ENCODING_DATAPATH:
        case SessionType::LE_AUDIO_SOFTWARE_DECODING_DATAPATH:
        case SessionType::LE_AUDIO_BROADCAST_SOFTWARE_ENCODING_DATAPATH:
        case SessionType::A2DP_SOFTWARE_DECODING_DATAPATH:
        case SessionType::HFP_SOFTWARE_ENCODING_DATAPATH:
        case SessionType::HFP_SOFTWARE_DECODING_DATAPATH:
        case SessionType::UNKNOWN:
            return false;
    }
}

inline ChannelMode from_channel_mode_mtk(const mtk::ChannelMode& channelMode) {
    switch (channelMode) {
        case mtk::ChannelMode::UNKNOWN: return ChannelMode::UNKNOWN;
        case mtk::ChannelMode::MONO: return ChannelMode::MONO;
        case mtk::ChannelMode::STEREO: return ChannelMode::STEREO;
    }
}

inline PcmCapabilities from_pcm_capabilities_mtk(const mtk::PcmCapabilities& capabilities) {
    return PcmCapabilities{
        .sampleRateHz = capabilities.sampleRateHz,
        .channelMode = capabilities.channelMode | std::views::transform(from_channel_mode_mtk) | std::ranges::to<std::vector>(),
        .bitsPerSample = capabilities.bitsPerSample,
        .dataIntervalUs = capabilities.dataIntervalUs,
    };
}

inline SbcChannelMode from_sbc_channel_mode_mtk(const mtk::SbcChannelMode& channelMode) {
    switch (channelMode) {
        case mtk::SbcChannelMode::UNKNOWN: return SbcChannelMode::UNKNOWN;
        case mtk::SbcChannelMode::JOINT_STEREO: return SbcChannelMode::JOINT_STEREO;
        case mtk::SbcChannelMode::STEREO: return SbcChannelMode::STEREO;
        case mtk::SbcChannelMode::DUAL: return SbcChannelMode::DUAL;
        case mtk::SbcChannelMode::MONO: return SbcChannelMode::MONO;
    }
}

inline SbcAllocMethod from_sbc_alloc_method_mtk(const mtk::SbcAllocMethod& allocMethod) {
    switch (allocMethod) {
        case mtk::SbcAllocMethod::ALLOC_MD_S: return SbcAllocMethod::ALLOC_MD_S;
        case mtk::SbcAllocMethod::ALLOC_MD_L: return SbcAllocMethod::ALLOC_MD_L;
    }
}

inline SbcCapabilities from_sbc_capabilities_mtk(const mtk::SbcCapabilities& capabilities) {
    return SbcCapabilities{
        .sampleRateHz = capabilities.sampleRateHz,
        .channelMode = capabilities.channelMode | std::views::transform(from_sbc_channel_mode_mtk) | std::ranges::to<std::vector>(),
        .blockLength = capabilities.blockLength,
        .numSubbands = capabilities.numSubbands,
        .allocMethod = capabilities.allocMethod | std::views::transform(from_sbc_alloc_method_mtk) | std::ranges::to<std::vector>(),
        .bitsPerSample = capabilities.bitsPerSample,
        .minBitpool = capabilities.minBitpool,
        .maxBitpool = capabilities.maxBitpool,
    };
}

inline AacObjectType from_aac_object_type_mtk(const mtk::AacObjectType& objectType) {
    switch (objectType) {
        case mtk::AacObjectType::MPEG2_LC: return AacObjectType::MPEG2_LC;
        case mtk::AacObjectType::MPEG4_LC: return AacObjectType::MPEG4_LC;
        case mtk::AacObjectType::MPEG4_LTP: return AacObjectType::MPEG4_LTP;
        case mtk::AacObjectType::MPEG4_SCALABLE: return AacObjectType::MPEG4_SCALABLE;
    }
}

inline AacCapabilities from_aac_capabilities_mtk(const mtk::AacCapabilities& capabilities) {
    return AacCapabilities{
        .objectType = capabilities.objectType | std::views::transform(from_aac_object_type_mtk) | std::ranges::to<std::vector>(),
        .sampleRateHz = capabilities.sampleRateHz,
        .channelMode = capabilities.channelMode | std::views::transform(from_channel_mode_mtk) | std::ranges::to<std::vector>(),
        .variableBitRateSupported = capabilities.variableBitRateSupported,
        .bitsPerSample = capabilities.bitsPerSample,
    };
}

inline LdacChannelMode from_ldac_channel_mode_mtk(const mtk::LdacChannelMode& channelMode) {
    switch (channelMode) {
        case mtk::LdacChannelMode::UNKNOWN: return LdacChannelMode::UNKNOWN;
        case mtk::LdacChannelMode::STEREO: return LdacChannelMode::STEREO;
        case mtk::LdacChannelMode::DUAL: return LdacChannelMode::DUAL;
        case mtk::LdacChannelMode::MONO: return LdacChannelMode::MONO;
    }
}

inline LdacQualityIndex from_ldac_quality_index_mtk(const mtk::LdacQualityIndex& qualityIndex) {
    switch (qualityIndex) {
        case mtk::LdacQualityIndex::HIGH: return LdacQualityIndex::HIGH;
        case mtk::LdacQualityIndex::MID: return LdacQualityIndex::MID;
        case mtk::LdacQualityIndex::LOW: return LdacQualityIndex::LOW;
        case mtk::LdacQualityIndex::ABR: return LdacQualityIndex::ABR;
    }
}

inline LdacCapabilities from_ldac_capabilities_mtk(const mtk::LdacCapabilities& capabilities) {
    return LdacCapabilities{
        .sampleRateHz = capabilities.sampleRateHz,
        .channelMode = capabilities.channelMode | std::views::transform(from_ldac_channel_mode_mtk) | std::ranges::to<std::vector>(),
        .qualityIndex = capabilities.qualityIndex | std::views::transform(from_ldac_quality_index_mtk) | std::ranges::to<std::vector>(),
        .bitsPerSample = capabilities.bitsPerSample,
    };
}

inline AptxCapabilities from_aptx_capabilities_mtk(const mtk::AptxCapabilities& capabilities) {
    return AptxCapabilities{
        .sampleRateHz = capabilities.sampleRateHz,
        .channelMode = capabilities.channelMode | std::views::transform(from_channel_mode_mtk) | std::ranges::to<std::vector>(),
        .bitsPerSample = capabilities.bitsPerSample,
    };
}

inline AptxAdaptiveChannelMode from_aptx_adaptive_channel_mode_mtk(const mtk::AptxAdaptiveChannelMode& channelMode) {
    switch (channelMode) {
        case mtk::AptxAdaptiveChannelMode::JOINT_STEREO: return AptxAdaptiveChannelMode::JOINT_STEREO;
        case mtk::AptxAdaptiveChannelMode::MONO: return AptxAdaptiveChannelMode::MONO;
        case mtk::AptxAdaptiveChannelMode::DUAL_MONO: return AptxAdaptiveChannelMode::DUAL_MONO;
        case mtk::AptxAdaptiveChannelMode::TWS_STEREO: return AptxAdaptiveChannelMode::TWS_STEREO;
        case mtk::AptxAdaptiveChannelMode::UNKNOWN: return AptxAdaptiveChannelMode::UNKNOWN;
    }
}

inline AptxMode from_aptx_mode_mtk(const mtk::AptxMode& mode) {
    switch (mode) {
        case mtk::AptxMode::UNKNOWN: return AptxMode::UNKNOWN;
        case mtk::AptxMode::HIGH_QUALITY: return AptxMode::HIGH_QUALITY;
        case mtk::AptxMode::LOW_LATENCY: return AptxMode::LOW_LATENCY;
        case mtk::AptxMode::ULTRA_LOW_LATENCY: return AptxMode::ULTRA_LOW_LATENCY;
    }
}

inline AptxAdaptiveInputMode from_aptx_adaptive_input_mode_mtk(const mtk::AptxAdaptiveInputMode& inputMode) {
    switch (inputMode) {
        case mtk::AptxAdaptiveInputMode::STEREO: return AptxAdaptiveInputMode::STEREO;
        case mtk::AptxAdaptiveInputMode::DUAL_MONO: return AptxAdaptiveInputMode::DUAL_MONO;
    }
}

inline AptxAdaptiveCapabilities from_aptx_adaptive_capabilities_mtk(const mtk::AptxAdaptiveCapabilities& capabilities) {
    return AptxAdaptiveCapabilities{
        .sampleRateHz = capabilities.sampleRateHz,
        .channelMode = capabilities.channelMode | std::views::transform(from_aptx_adaptive_channel_mode_mtk) | std::ranges::to<std::vector>(),
        .bitsPerSample = capabilities.bitsPerSample,
        .aptxMode = capabilities.aptxMode | std::views::transform(from_aptx_mode_mtk) | std::ranges::to<std::vector>(),
        .sinkBufferingMs = AptxSinkBuffering{
            .minLowLatency = capabilities.sinkBufferingMs.minLowLatency,
            .maxLowLatency = capabilities.sinkBufferingMs.maxLowLatency,
            .minHighQuality = capabilities.sinkBufferingMs.minHighQuality,
            .maxHighQuality = capabilities.sinkBufferingMs.maxHighQuality,
            .minTws = capabilities.sinkBufferingMs.minTws,
            .maxTws = capabilities.sinkBufferingMs.maxTws,
        },
        .ttp = AptxAdaptiveTimeToPlay{
            .lowLowLatency = capabilities.ttp.lowLowLatency,
            .highLowLatency = capabilities.ttp.highLowLatency,
            .lowHighQuality = capabilities.ttp.lowHighQuality,
            .highHighQuality = capabilities.ttp.highHighQuality,
            .lowTws = capabilities.ttp.lowTws,
            .highTws = capabilities.ttp.highTws,
        },
        .inputMode = from_aptx_adaptive_input_mode_mtk(capabilities.inputMode),
        .inputFadeDurationMs = capabilities.inputFadeDurationMs,
        .aptxAdaptiveConfigStream = capabilities.aptxAdaptiveConfigStream,
    };
}

inline Lc3Capabilities from_lc3_capabilities_mtk(const mtk::Lc3Capabilities& capabilities) {
    return Lc3Capabilities{
        .pcmBitDepth = capabilities.pcmBitDepth,
        .samplingFrequencyHz = capabilities.samplingFrequencyHz,
        .frameDurationUs = capabilities.frameDurationUs,
        .octetsPerFrame = capabilities.octetsPerFrame,
        .blocksPerSdu = capabilities.blocksPerSdu,
        .channelMode = capabilities.channelMode | std::views::transform(from_channel_mode_mtk) | std::ranges::to<std::vector>(),
    };
}

inline CodecCapabilities::VendorCapabilities from_lhdcv5_capabilities_mtk(
        const mtk::Lhdcv5Capabilities& capabilities) {
    return CodecCapabilities::VendorCapabilities{};  // TODO: Implement this
}

inline CodecCapabilities::Capabilities from_codec_capabilities_capabilities_mtk(
        const mtk::CodecCapabilities::Capabilities& capabilities) {
    switch (capabilities.getTag()) {
        case mtk::CodecCapabilities::Capabilities::sbcCapabilities:
            return CodecCapabilities::Capabilities::make<CodecCapabilities::Capabilities::Tag::sbcCapabilities>(
                    from_sbc_capabilities_mtk(capabilities.get<mtk::CodecCapabilities::Capabilities::Tag::sbcCapabilities>()));
        case mtk::CodecCapabilities::Capabilities::aacCapabilities:
            return CodecCapabilities::Capabilities::make<CodecCapabilities::Capabilities::Tag::aacCapabilities>(
                    from_aac_capabilities_mtk(capabilities.get<mtk::CodecCapabilities::Capabilities::Tag::aacCapabilities>()));
        case mtk::CodecCapabilities::Capabilities::ldacCapabilities:
            return CodecCapabilities::Capabilities::make<CodecCapabilities::Capabilities::Tag::ldacCapabilities>(
                    from_ldac_capabilities_mtk(capabilities.get<mtk::CodecCapabilities::Capabilities::Tag::ldacCapabilities>()));
        case mtk::CodecCapabilities::Capabilities::aptxCapabilities:
            return CodecCapabilities::Capabilities::make<CodecCapabilities::Capabilities::Tag::aptxCapabilities>(
                    from_aptx_capabilities_mtk(capabilities.get<mtk::CodecCapabilities::Capabilities::Tag::aptxCapabilities>()));
        case mtk::CodecCapabilities::Capabilities::aptxAdaptiveCapabilities:
            return CodecCapabilities::Capabilities::make<CodecCapabilities::Capabilities::Tag::aptxAdaptiveCapabilities>(
                    from_aptx_adaptive_capabilities_mtk(capabilities.get<mtk::CodecCapabilities::Capabilities::Tag::aptxAdaptiveCapabilities>()));
        case mtk::CodecCapabilities::Capabilities::lc3Capabilities:
            return CodecCapabilities::Capabilities::make<CodecCapabilities::Capabilities::Tag::lc3Capabilities>(
                    from_lc3_capabilities_mtk(capabilities.get<mtk::CodecCapabilities::Capabilities::Tag::lc3Capabilities>()));
        case mtk::CodecCapabilities::Capabilities::vendorCapabilities:
            return CodecCapabilities::Capabilities::make<CodecCapabilities::Capabilities::Tag::vendorCapabilities>(
                CodecCapabilities::VendorCapabilities{.extension = capabilities.get<mtk::CodecCapabilities::Capabilities::Tag::vendorCapabilities>().extension});
        case mtk::CodecCapabilities::Capabilities::lhdcv5Capabilities:
            return CodecCapabilities::Capabilities::make<CodecCapabilities::Capabilities::Tag::vendorCapabilities>(
                    from_lhdcv5_capabilities_mtk(capabilities.get<mtk::CodecCapabilities::Capabilities::Tag::lhdcv5Capabilities>()));
    }
}

inline CodecType from_codec_type_mtk(const mtk::CodecType& codecType) {
    switch (codecType) {
        case mtk::CodecType::UNKNOWN: return CodecType::UNKNOWN;
        case mtk::CodecType::SBC: return CodecType::SBC;
        case mtk::CodecType::AAC: return CodecType::AAC;
        case mtk::CodecType::APTX: return CodecType::APTX;
        case mtk::CodecType::APTX_HD: return CodecType::APTX_HD;
        case mtk::CodecType::LDAC: return CodecType::LDAC;
        case mtk::CodecType::LC3: return CodecType::LC3;
        case mtk::CodecType::VENDOR: return CodecType::VENDOR;
        case mtk::CodecType::APTX_ADAPTIVE: return CodecType::APTX_ADAPTIVE;
        case mtk::CodecType::LHDCV3: return CodecType::VENDOR;
        case mtk::CodecType::LHDCV2: return CodecType::VENDOR;
        case mtk::CodecType::LHDCV5: return CodecType::VENDOR;
    }
}

inline CodecCapabilities from_codec_capabilities_mtk(const mtk::CodecCapabilities& capabilities) {
    return CodecCapabilities{
            .codecType = from_codec_type_mtk(capabilities.codecType),
            .capabilities = from_codec_capabilities_capabilities_mtk(capabilities.capabilities)};
}

inline AudioLocation from_audio_location_mtk(const mtk::AudioLocation& audioLocation) {
    switch (audioLocation) {
        case mtk::AudioLocation::UNKNOWN: return AudioLocation::UNKNOWN;
        case mtk::AudioLocation::FRONT_LEFT: return AudioLocation::FRONT_LEFT;
        case mtk::AudioLocation::FRONT_RIGHT: return AudioLocation::FRONT_RIGHT;
    }
}

inline UnicastCapability from_unicast_capability_mtk(const mtk::UnicastCapability& capabilities) {
    return UnicastCapability{
        .codecType = from_codec_type_mtk(capabilities.codecType),
        .supportedChannel = from_audio_location_mtk(capabilities.supportedChannel),
        .deviceCount = capabilities.deviceCount,
        .channelCountPerDevice = capabilities.channelCountPerDevice,
        .leAudioCodecCapabilities = capabilities.leAudioCodecCapabilities.getTag() == mtk::UnicastCapability::LeAudioCodecCapabilities::Tag::lc3Capabilities
         ? UnicastCapability::LeAudioCodecCapabilities::make<UnicastCapability::LeAudioCodecCapabilities::Tag::lc3Capabilities>(
                from_lc3_capabilities_mtk(capabilities.leAudioCodecCapabilities.get<mtk::UnicastCapability::LeAudioCodecCapabilities::Tag::lc3Capabilities>()))
         : UnicastCapability::LeAudioCodecCapabilities::make<UnicastCapability::LeAudioCodecCapabilities::Tag::vendorCapabillities>(
                UnicastCapability::VendorCapabilities{.extension = capabilities.leAudioCodecCapabilities.get<mtk::UnicastCapability::LeAudioCodecCapabilities::Tag::vendorCapabillities>().extension}),
    };
}

inline BroadcastCapability from_broadcast_capability_mtk(const mtk::BroadcastCapability& capabilities) {
    return BroadcastCapability{
        .codecType = from_codec_type_mtk(capabilities.codecType),
        .supportedChannel = from_audio_location_mtk(capabilities.supportedChannel),
        .channelCountPerStream = capabilities.channelCountPerStream,
        .leAudioCodecCapabilities = capabilities.leAudioCodecCapabilities.getTag() == mtk::BroadcastCapability::LeAudioCodecCapabilities::Tag::lc3Capabilities
         ? BroadcastCapability::LeAudioCodecCapabilities::make<BroadcastCapability::LeAudioCodecCapabilities::Tag::lc3Capabilities>(
                capabilities.leAudioCodecCapabilities.get<mtk::BroadcastCapability::LeAudioCodecCapabilities::Tag::lc3Capabilities>().transform([](auto list){
                    return list | std::views::transform([](auto lc3Capabilities){
                        return lc3Capabilities.transform(from_lc3_capabilities_mtk);
                    }) | std::ranges::to<std::vector>();
                }))
         : BroadcastCapability::LeAudioCodecCapabilities::make<BroadcastCapability::LeAudioCodecCapabilities::Tag::vendorCapabillities>(
             capabilities.leAudioCodecCapabilities.get<mtk::BroadcastCapability::LeAudioCodecCapabilities::Tag::vendorCapabillities>().transform([](auto list){
                 return list | std::views::transform([](auto vendorCapabilitiesOptional){
                    return vendorCapabilitiesOptional.transform([](auto vendorCapabilities){
                         return BroadcastCapability::VendorCapabilities{.extension = vendorCapabilities.extension};
                     });
                 }) | std::ranges::to<std::vector>();
             })),
    };
}

inline LeAudioCodecCapabilitiesSetting from_le_audio_codec_capabilities_mtk(const mtk::LeAudioCodecCapabilitiesSetting& capabilities) {
    return LeAudioCodecCapabilitiesSetting{
        .unicastEncodeCapability = from_unicast_capability_mtk(capabilities.unicastEncodeCapability),
        .unicastDecodeCapability = from_unicast_capability_mtk(capabilities.unicastDecodeCapability),
        .broadcastCapability = from_broadcast_capability_mtk(capabilities.broadcastCapability),
    };
}

inline AudioCapabilities from_audio_capabilities_mtk(const mtk::AudioCapabilities& capabilities) {
    switch (capabilities.getTag()) {
        case mtk::AudioCapabilities::pcmCapabilities:
            return AudioCapabilities::make<AudioCapabilities::Tag::pcmCapabilities>(
                    from_pcm_capabilities_mtk(capabilities.get<mtk::AudioCapabilities::Tag::pcmCapabilities>()));
        case mtk::AudioCapabilities::a2dpCapabilities:
            return AudioCapabilities::make<AudioCapabilities::Tag::a2dpCapabilities>(
                    from_codec_capabilities_mtk(capabilities.get<mtk::AudioCapabilities::Tag::a2dpCapabilities>()));
        case mtk::AudioCapabilities::leAudioCapabilities:
            return AudioCapabilities::make<AudioCapabilities::Tag::leAudioCapabilities>(
                    from_le_audio_codec_capabilities_mtk(capabilities.get<mtk::AudioCapabilities::Tag::leAudioCapabilities>()));
    }
}

void MTKBluetoothAudioProviderFactory::loadFactoryIfNeeded() {
    if (factory_hw == nullptr) {
        factory_hw = mtk::IBluetoothAudioProviderFactory::fromBinder(ndk::SpAIBinder(
            AServiceManager_waitForService((std::string() + mtk::IBluetoothAudioProviderFactory::descriptor + "/default").c_str())));
    }
}

MTKBluetoothAudioProviderFactory::MTKBluetoothAudioProviderFactory() {}

ndk::ScopedAStatus MTKBluetoothAudioProviderFactory::openProvider(
        const SessionType session_type, std::shared_ptr<IBluetoothAudioProvider>* _aidl_return) {
    if (!isHardwareSessionType(session_type)) {
        return factory_sw.openProvider(session_type, _aidl_return);
    }

    loadFactoryIfNeeded();
    std::shared_ptr<mtk::IBluetoothAudioProvider> mtk_return;
    auto retval = factory_hw->openProvider(to_session_type_mtk(session_type), &mtk_return);
    if (retval.isOk()) {
        *_aidl_return = ::ndk::SharedRefBase::make<MTKBluetoothAudioProvider>(mtk_return);
    }
    return retval;
}

ndk::ScopedAStatus MTKBluetoothAudioProviderFactory::getProviderCapabilities(
        const SessionType session_type, std::vector<AudioCapabilities>* _aidl_return) {
    if (!isHardwareSessionType(session_type)) {
        return factory_sw.getProviderCapabilities(session_type, _aidl_return);
    }

    loadFactoryIfNeeded();
    std::vector<mtk::AudioCapabilities> mtk_return;
    auto retval = factory_hw->getProviderCapabilities(to_session_type_mtk(session_type), &mtk_return);
    if (retval.isOk()) {
        _aidl_return->resize(mtk_return.size());
        for (int i = 0; i < mtk_return.size(); i++) {
            _aidl_return->at(i) = from_audio_capabilities_mtk(mtk_return[i]);
        }
    }
    return retval;
}

ndk::ScopedAStatus MTKBluetoothAudioProviderFactory::getProviderInfo(
        SessionType session_type, std::optional<ProviderInfo>* _aidl_return) {
    if (!isHardwareSessionType(session_type)) {
        return factory_sw.getProviderInfo(session_type, _aidl_return);
    }

    *_aidl_return = std::nullopt;
    return ndk::ScopedAStatus::fromStatus(STATUS_UNKNOWN_TRANSACTION);
}

}  // namespace adapter
}  // namespace audio
}  // namespace bluetooth
}  // namespace hardware
}  // namespace mediatek
}  // namespace vendor
