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

#define LOG_TAG "BTAudioProviderFactoryAIDL"

#include <android-base/logging.h>
#include <future>

#include "MTKBluetoothAudioProvider.h"
#include "MTKBluetoothAudioProviderFactory.h"

namespace vendor {
namespace mediatek {
namespace hardware {
namespace bluetooth {
namespace audio {
namespace adapter {

using ::android::hardware::hidl_vec;
using SessionType_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SessionType;
using SessionType_2_2 = ::vendor::mediatek::hardware::bluetooth::audio::V2_2::SessionType;
using SampleRate_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SampleRate;
using SampleRate_2_2 = ::vendor::mediatek::hardware::bluetooth::audio::V2_2::SampleRate;
using CodecType_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::CodecType;
using CodecType_2_2 = ::vendor::mediatek::hardware::bluetooth::audio::V2_2::CodecType;
using ChannelMode_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::ChannelMode;
using BitsPerSample_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::BitsPerSample;
using PcmParameters_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::PcmParameters;
using PcmParameters_2_2 = ::vendor::mediatek::hardware::bluetooth::audio::V2_2::PcmParameters;
using CodecCapabilities_2_1 =
        ::vendor::mediatek::hardware::bluetooth::audio::V2_1::CodecCapabilities;
using LeAudioCodecCapabilities_2_2 =
        ::vendor::mediatek::hardware::bluetooth::audio::V2_2::LeAudioCodecCapabilities;
using SbcParameters_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SbcParameters;
using AacParameters_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::AacParameters;
using AacVariableBitRate_2_1 =
        ::vendor::mediatek::hardware::bluetooth::audio::V2_1::AacVariableBitRate;
using LdacParameters_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::LdacParameters;
using AptxParameters_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::AptxParameters;
using LhdcParameters_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::LhdcParameters;
using SbcChannelMode_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SbcChannelMode;
using SbcBlockLength_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SbcBlockLength;
using SbcNumSubbands_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SbcNumSubbands;
using SbcAllocMethod_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SbcAllocMethod;
using AacObjectType_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::AacObjectType;
using LdacQualityIndex_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::LdacQualityIndex;
using LdacChannelMode_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::LdacChannelMode;
using AudioCapabilities_2_1 =
        ::vendor::mediatek::hardware::bluetooth::audio::V2_1::AudioCapabilities;
using AudioCapabilities_2_2 =
        ::vendor::mediatek::hardware::bluetooth::audio::V2_2::AudioCapabilities;
using HidlStatus = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::Status;
const static std::unordered_map<SessionType, SessionType_2_1> session_type_aidl_to_2_1_map{
        {SessionType::A2DP_SOFTWARE_ENCODING_DATAPATH,
         SessionType_2_1::A2DP_SOFTWARE_ENCODING_DATAPATH},
        {SessionType::A2DP_HARDWARE_OFFLOAD_ENCODING_DATAPATH,
         SessionType_2_1::A2DP_HARDWARE_OFFLOAD_DATAPATH},
        {SessionType::HEARING_AID_SOFTWARE_ENCODING_DATAPATH,
         SessionType_2_1::HEARING_AID_SOFTWARE_ENCODING_DATAPATH},
};

inline SessionType_2_1 to_session_type_2_1(const SessionType& session_type_aidl) {
    auto it = session_type_aidl_to_2_1_map.find(session_type_aidl);
    if (it != session_type_aidl_to_2_1_map.end()) return it->second;
    return SessionType_2_1::UNKNOWN;
}

const static std::unordered_map<SessionType, SessionType_2_2> session_type_aidl_to_2_2_map{
        {SessionType::LE_AUDIO_SOFTWARE_ENCODING_DATAPATH,
         SessionType_2_2::LE_AUDIO_SOFTWARE_ENCODING_DATAPATH},
        {SessionType::LE_AUDIO_SOFTWARE_DECODING_DATAPATH,
         SessionType_2_2::LE_AUDIO_SOFTWARE_DECODED_DATAPATH},
        {SessionType::LE_AUDIO_HARDWARE_OFFLOAD_ENCODING_DATAPATH,
         SessionType_2_2::LE_AUDIO_HARDWARE_OFFLOAD_ENCODING_DATAPATH},
        {SessionType::LE_AUDIO_HARDWARE_OFFLOAD_DECODING_DATAPATH,
         SessionType_2_2::LE_AUDIO_HARDWARE_OFFLOAD_DECODING_DATAPATH},
};

inline SessionType_2_2 to_session_type_2_2(const SessionType& session_type_aidl) {
    auto it = session_type_aidl_to_2_2_map.find(session_type_aidl);
    if (it != session_type_aidl_to_2_2_map.end()) return it->second;
    return static_cast<SessionType_2_2>(to_session_type_2_1(session_type_aidl));
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

const static std::unordered_map<SampleRate_2_2, int32_t> sample_rate_hidl_2_2_map{
        {SampleRate_2_2::RATE_44100, 44100},   {SampleRate_2_2::RATE_48000, 48000},
        {SampleRate_2_2::RATE_88200, 88200},   {SampleRate_2_2::RATE_96000, 96000},
        {SampleRate_2_2::RATE_176400, 176400}, {SampleRate_2_2::RATE_192000, 192000},
        {SampleRate_2_2::RATE_16000, 16000},   {SampleRate_2_2::RATE_24000, 24000},
        {SampleRate_2_2::RATE_8000, 8000},     {SampleRate_2_2::RATE_32000, 32000},
};

const static std::unordered_map<CodecType_2_2, CodecType> codec_type_hidl_2_2_map{
        {CodecType_2_2::UNKNOWN, CodecType::UNKNOWN}, {CodecType_2_2::SBC, CodecType::SBC},
        {CodecType_2_2::AAC, CodecType::AAC},         {CodecType_2_2::APTX, CodecType::APTX},
        {CodecType_2_2::APTX_HD, CodecType::APTX_HD}, {CodecType_2_2::LDAC, CodecType::LDAC},
        {CodecType_2_2::LC3, CodecType::LC3},
};

const static std::unordered_map<ChannelMode_2_1, ChannelMode> channel_mode_hidl_map{
        {ChannelMode_2_1::UNKNOWN, ChannelMode::UNKNOWN},
        {ChannelMode_2_1::MONO, ChannelMode::MONO},
        {ChannelMode_2_1::STEREO, ChannelMode::STEREO},
};

const static std::unordered_map<BitsPerSample_2_1, uint8_t> bits_per_sample_hidl_map{
        {BitsPerSample_2_1::BITS_16, 16},
        {BitsPerSample_2_1::BITS_24, 24},
        {BitsPerSample_2_1::BITS_32, 32},
};

inline std::vector<int32_t> from_sample_rate_2_2(const SampleRate_2_2& sampleRate) {
    std::vector<int32_t> out;
    for (const auto& [bits, value] : sample_rate_hidl_2_2_map) {
        if (sampleRate & bits) {
            out.push_back(value);
        }
    }
    return out;
}

inline std::vector<int32_t> from_sample_rate_2_1(const SampleRate_2_1& sampleRate) {
    return from_sample_rate_2_2(static_cast<SampleRate_2_2>(sampleRate));
}

inline CodecType from_codec_type_2_2(const CodecType_2_2& codecType) {
    auto it = codec_type_hidl_2_2_map.find(codecType);
    if (it != codec_type_hidl_2_2_map.end()) return it->second;
    return CodecType::UNKNOWN;
}

inline CodecType from_codec_type_2_1(const CodecType_2_1& codecType) {
    return from_codec_type_2_2(static_cast<CodecType_2_2>(codecType));
}

inline std::vector<ChannelMode> from_channel_mode_2_1(const ChannelMode_2_1& channelMode) {
    std::vector<ChannelMode> out;
    for (const auto& [bits, value] : channel_mode_hidl_map) {
        if (channelMode & bits) {
            out.push_back(value);
        }
    }
    return out;
}

inline std::vector<uint8_t> from_bits_per_sample_2_1(const BitsPerSample_2_1& bitsPerSample) {
    std::vector<uint8_t> out;
    for (const auto& [bits, value] : bits_per_sample_hidl_map) {
        if (bitsPerSample & bits) {
            out.push_back(value);
        }
    }
    return out;
}

inline PcmCapabilities from_pcm_parameters_2_1(const PcmParameters_2_1& capabilities) {
    return PcmCapabilities{
            .sampleRateHz = from_sample_rate_2_1(capabilities.sampleRate),
            .channelMode = from_channel_mode_2_1(capabilities.channelMode),
            .bitsPerSample = from_bits_per_sample_2_1(capabilities.bitsPerSample),
            .dataIntervalUs = {20000},  // A2DP frame duration
    };
}

inline PcmCapabilities from_pcm_parameters_2_2(const PcmParameters_2_2& capabilities) {
    return PcmCapabilities{
            .sampleRateHz = from_sample_rate_2_2(capabilities.sampleRate),
            .channelMode = from_channel_mode_2_1(capabilities.channelMode),
            .bitsPerSample = from_bits_per_sample_2_1(capabilities.bitsPerSample),
            .dataIntervalUs = {static_cast<int>(capabilities.dataIntervalUs)},
    };
}

const static std::unordered_map<SbcChannelMode_2_1, SbcChannelMode> sbc_channel_mode_hidl_2_1_map{
        {SbcChannelMode_2_1::UNKNOWN, SbcChannelMode::UNKNOWN},
        {SbcChannelMode_2_1::JOINT_STEREO, SbcChannelMode::JOINT_STEREO},
        {SbcChannelMode_2_1::STEREO, SbcChannelMode::STEREO},
        {SbcChannelMode_2_1::DUAL, SbcChannelMode::DUAL},
        {SbcChannelMode_2_1::MONO, SbcChannelMode::MONO},
};

const static std::unordered_map<SbcBlockLength_2_1, uint8_t> sbc_block_length_hidl_map{
        {SbcBlockLength_2_1::BLOCKS_4, 4},
        {SbcBlockLength_2_1::BLOCKS_8, 8},
        {SbcBlockLength_2_1::BLOCKS_12, 12},
        {SbcBlockLength_2_1::BLOCKS_16, 16},
};

const static std::unordered_map<SbcNumSubbands_2_1, uint8_t> sbc_subbands_hidl_map{
        {SbcNumSubbands_2_1::SUBBAND_4, 4},
        {SbcNumSubbands_2_1::SUBBAND_8, 8},
};

const static std::unordered_map<SbcAllocMethod_2_1, SbcAllocMethod> sbc_alloc_method_hidl_map{
        {SbcAllocMethod_2_1::ALLOC_MD_S, SbcAllocMethod::ALLOC_MD_S},
        {SbcAllocMethod_2_1::ALLOC_MD_L, SbcAllocMethod::ALLOC_MD_L},
};

inline std::vector<SbcChannelMode> from_sbc_channel_mode_2_1(
        const SbcChannelMode_2_1& channelMode) {
    std::vector<SbcChannelMode> out;
    for (const auto& [bits, value] : sbc_channel_mode_hidl_2_1_map) {
        if (channelMode & bits) {
            out.push_back(value);
        }
    }
    return out;
}

inline std::vector<uint8_t> from_sbc_block_length_2_1(const SbcBlockLength_2_1& blockLength) {
    std::vector<uint8_t> out;
    for (const auto& [bits, value] : sbc_block_length_hidl_map) {
        if (blockLength & bits) {
            out.push_back(value);
        }
    }
    return out;
}

inline std::vector<uint8_t> from_sbc_num_subbands_2_1(const SbcNumSubbands_2_1& numSubbands) {
    std::vector<uint8_t> out;
    for (const auto& [bits, value] : sbc_subbands_hidl_map) {
        if (numSubbands & bits) {
            out.push_back(value);
        }
    }
    return out;
}

inline std::vector<SbcAllocMethod> from_sbc_alloc_method_2_1(
        const SbcAllocMethod_2_1& allocMethod) {
    std::vector<SbcAllocMethod> out;
    for (const auto& [bits, value] : sbc_alloc_method_hidl_map) {
        if (allocMethod & bits) {
            out.push_back(value);
        }
    }
    return out;
}

inline SbcCapabilities from_sbc_parameters_2_1(const SbcParameters_2_1& parameters) {
    return SbcCapabilities{
            .sampleRateHz = from_sample_rate_2_1(parameters.sampleRate),
            .channelMode = from_sbc_channel_mode_2_1(parameters.channelMode),
            .blockLength = from_sbc_block_length_2_1(parameters.blockLength),
            .numSubbands = from_sbc_num_subbands_2_1(parameters.numSubbands),
            .allocMethod = from_sbc_alloc_method_2_1(parameters.allocMethod),
            .bitsPerSample = from_bits_per_sample_2_1(parameters.bitsPerSample),
            .minBitpool = parameters.minBitpool,
            .maxBitpool = parameters.maxBitpool,
    };
}

const static std::unordered_map<AacObjectType_2_1, AacObjectType> aac_object_type_hidl_map{
        {AacObjectType_2_1::MPEG2_LC, AacObjectType::MPEG2_LC},
        {AacObjectType_2_1::MPEG4_LC, AacObjectType::MPEG4_LC},
        {AacObjectType_2_1::MPEG4_LTP, AacObjectType::MPEG4_LTP},
        {AacObjectType_2_1::MPEG4_SCALABLE, AacObjectType::MPEG4_SCALABLE},
};

inline std::vector<AacObjectType> from_aac_object_type_2_1(const AacObjectType_2_1& objectType) {
    std::vector<AacObjectType> out;
    for (const auto& [bits, value] : aac_object_type_hidl_map) {
        if (objectType & bits) {
            out.push_back(value);
        }
    }
    return out;
}

inline AacCapabilities from_aac_parameters_2_1(const AacParameters_2_1& parameters) {
    return AacCapabilities{
            .objectType = from_aac_object_type_2_1(parameters.objectType),
            .sampleRateHz = from_sample_rate_2_1(parameters.sampleRate),
            .channelMode = from_channel_mode_2_1(parameters.channelMode),
            .variableBitRateSupported =
                    parameters.variableBitRateEnabled == AacVariableBitRate_2_1::ENABLED,
            .bitsPerSample = from_bits_per_sample_2_1(parameters.bitsPerSample),
            .adaptiveBitRateSupported = false,
    };
}

const static std::unordered_map<LdacChannelMode_2_1, LdacChannelMode> ldac_channel_mode_hidl_map{
        {LdacChannelMode_2_1::UNKNOWN, LdacChannelMode::UNKNOWN},
        {LdacChannelMode_2_1::STEREO, LdacChannelMode::STEREO},
        {LdacChannelMode_2_1::DUAL, LdacChannelMode::DUAL},
        {LdacChannelMode_2_1::MONO, LdacChannelMode::MONO},
};

inline std::vector<LdacChannelMode> from_ldac_channel_mode_2_1(
        const LdacChannelMode_2_1& channelMode) {
    std::vector<LdacChannelMode> out;
    for (const auto& [bits, value] : ldac_channel_mode_hidl_map) {
        if (channelMode & bits) {
            out.push_back(value);
        }
    }
    return out;
}

inline LdacCapabilities from_ldac_parameters_2_1(const LdacParameters_2_1& parameters) {
    return LdacCapabilities{
            .sampleRateHz = from_sample_rate_2_1(parameters.sampleRate),
            .channelMode = from_ldac_channel_mode_2_1(parameters.channelMode),
            // according to types.hal, all qualities must be supported, and hence this isn't a
            // bitfield in HIDL
            .qualityIndex = {LdacQualityIndex::HIGH, LdacQualityIndex::MID, LdacQualityIndex::LOW,
                             LdacQualityIndex::ABR},
            .bitsPerSample = from_bits_per_sample_2_1(parameters.bitsPerSample),
    };
}

inline AptxCapabilities from_aptx_parameters_2_1(const AptxParameters_2_1& parameters) {
    return AptxCapabilities{
            .sampleRateHz = from_sample_rate_2_1(parameters.sampleRate),
            .channelMode = from_channel_mode_2_1(parameters.channelMode),
            .bitsPerSample = from_bits_per_sample_2_1(parameters.bitsPerSample),
    };
}

inline CodecCapabilities::VendorCapabilities from_lhdc_parameters_2_1(
        const LhdcParameters_2_1& parameters) {
    return CodecCapabilities::VendorCapabilities{};  // TODO: Implement this
}

inline CodecCapabilities::Capabilities from_codec_capabilities_capabilities_2_1(
        const CodecCapabilities_2_1::Capabilities& capabilities) {
    switch (capabilities.getDiscriminator()) {
        case CodecCapabilities_2_1::Capabilities::hidl_discriminator::sbcCapabilities:
            return CodecCapabilities::Capabilities::make<
                    CodecCapabilities::Capabilities::Tag::sbcCapabilities>(
                    from_sbc_parameters_2_1(capabilities.sbcCapabilities()));
        case CodecCapabilities_2_1::Capabilities::hidl_discriminator::aacCapabilities:
            return CodecCapabilities::Capabilities::make<
                    CodecCapabilities::Capabilities::Tag::aacCapabilities>(
                    from_aac_parameters_2_1(capabilities.aacCapabilities()));
        case CodecCapabilities_2_1::Capabilities::hidl_discriminator::ldacCapabilities:
            return CodecCapabilities::Capabilities::make<
                    CodecCapabilities::Capabilities::Tag::ldacCapabilities>(
                    from_ldac_parameters_2_1(capabilities.ldacCapabilities()));
        case CodecCapabilities_2_1::Capabilities::hidl_discriminator::aptxCapabilities:
            return CodecCapabilities::Capabilities::make<
                    CodecCapabilities::Capabilities::Tag::aptxCapabilities>(
                    from_aptx_parameters_2_1(capabilities.aptxCapabilities()));
        case CodecCapabilities_2_1::Capabilities::hidl_discriminator::lhdcCapabilities:
            return CodecCapabilities::Capabilities::make<
                    CodecCapabilities::Capabilities::Tag::vendorCapabilities>(
                    from_lhdc_parameters_2_1(capabilities.lhdcCapabilities()));
    }
}

inline CodecCapabilities from_codec_capabilities_2_1(const CodecCapabilities_2_1& capabilities) {
    return CodecCapabilities{
            .codecType = from_codec_type_2_1(capabilities.codecType),
            .capabilities = from_codec_capabilities_capabilities_2_1(capabilities.capabilities)};
}

inline LeAudioCodecCapabilitiesSetting from_le_audio_codec_capabilities_2_2(
        const LeAudioCodecCapabilities_2_2& capabilities) {
    // refer to getProviderCapabilities comment below
    LOG(FATAL) << "LE Audio Codec capabilities shouldn't be queried from MTK 2.2 HIDL";
    return LeAudioCodecCapabilitiesSetting{};
}

inline AudioCapabilities from_audio_capabilities_2_1(const AudioCapabilities_2_1& capabilities) {
    switch (capabilities.getDiscriminator()) {
        case AudioCapabilities_2_1::hidl_discriminator::pcmCapabilities:
            return AudioCapabilities::make<AudioCapabilities::Tag::pcmCapabilities>(
                    from_pcm_parameters_2_1(capabilities.pcmCapabilities()));
        case AudioCapabilities_2_1::hidl_discriminator::codecCapabilities:
            return AudioCapabilities::make<AudioCapabilities::Tag::a2dpCapabilities>(
                    from_codec_capabilities_2_1(capabilities.codecCapabilities()));
    }
}

inline AudioCapabilities from_audio_capabilities_2_2(const AudioCapabilities_2_2& capabilities) {
    switch (capabilities.getDiscriminator()) {
        case AudioCapabilities_2_2::hidl_discriminator::pcmCapabilities:
            return AudioCapabilities::make<AudioCapabilities::Tag::pcmCapabilities>(
                    from_pcm_parameters_2_2(capabilities.pcmCapabilities()));
        case AudioCapabilities_2_2::hidl_discriminator::codecCapabilities:
            return AudioCapabilities::make<AudioCapabilities::Tag::a2dpCapabilities>(
                    from_codec_capabilities_2_1(capabilities.codecCapabilities()));
        case AudioCapabilities_2_2::hidl_discriminator::leAudioCapabilities:
            return AudioCapabilities::make<AudioCapabilities::Tag::leAudioCapabilities>(
                    from_le_audio_codec_capabilities_2_2(capabilities.leAudioCapabilities()));
    }
}

void MTKBluetoothAudioProviderFactory::loadFactoriesIfNeeded() {
    if (factory_2_1 == nullptr) {
        factory_2_1 = BluetoothAudioProviderFactory_2_1::getService("default");
        factory_2_2 = BluetoothAudioProviderFactory_2_2::castFrom(factory_2_1);
    }
}

MTKBluetoothAudioProviderFactory::MTKBluetoothAudioProviderFactory() {}

ndk::ScopedAStatus MTKBluetoothAudioProviderFactory::openProvider(
        const SessionType session_type, std::shared_ptr<IBluetoothAudioProvider>* _aidl_return) {
    if (!isHardwareSessionType(session_type)) {
        return factory_sw.openProvider(session_type, _aidl_return);
    }
    loadFactoriesIfNeeded();
    HidlStatus provider_status;
    std::shared_ptr<IBluetoothAudioProvider> provider = nullptr;
    std::promise<void> hidl_openProvider_promise;
    auto hidl_openProvider_future = hidl_openProvider_promise.get_future();
    if (factory_2_2 != nullptr) {
        factory_2_2->openProvider_2_1(
                to_session_type_2_2(session_type),
                [&provider_status, &hidl_openProvider_promise, &provider](
                        HidlStatus status,
                        const android::sp<IBluetoothAudioProvider_2_2>& provider_hidl) {
                    provider_status = status;
                    if (status == HidlStatus::SUCCESS) {
                        provider = ::ndk::SharedRefBase::make<MTKBluetoothAudioProvider>(
                                provider_hidl);
                    }
                    hidl_openProvider_promise.set_value();
                });
    } else {
        factory_2_1->openProvider(
                to_session_type_2_1(session_type),
                [&provider_status, &hidl_openProvider_promise, &provider](
                        HidlStatus status,
                        const android::sp<IBluetoothAudioProvider_2_1>& provider_hidl) {
                    provider_status = status;
                    if (status == HidlStatus::SUCCESS) {
                        provider = ::ndk::SharedRefBase::make<MTKBluetoothAudioProvider>(
                                provider_hidl);
                    }
                    hidl_openProvider_promise.set_value();
                });
    }
    hidl_openProvider_future.get();
    if (provider_status == HidlStatus::SUCCESS) {
        *_aidl_return = provider;
        return ndk::ScopedAStatus::ok();
    } else if (provider_status == HidlStatus::UNSUPPORTED_CODEC_CONFIGURATION) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    } else {
        return ndk::ScopedAStatus::fromExceptionCode(EX_SERVICE_SPECIFIC);
    }
}

ndk::ScopedAStatus MTKBluetoothAudioProviderFactory::getProviderCapabilities(
        const SessionType session_type, std::vector<AudioCapabilities>* _aidl_return) {
    if (session_type != SessionType::A2DP_HARDWARE_OFFLOAD_ENCODING_DATAPATH &&
        session_type != SessionType::A2DP_HARDWARE_OFFLOAD_DECODING_DATAPATH) {
        // Do get HFP/LEA offload codecs (anything newer than A2DP) from sw factory as
        // well as they can be configured using XML, instead of being hardcoded like
        // A2DP defaults are. AOSP capability AIDL is a poor match for the MTK LEA HIDL,
        // so these must be configured seperately if intended to be supported.
        return factory_sw.getProviderCapabilities(session_type, _aidl_return);
    }
    loadFactoriesIfNeeded();
    std::shared_ptr<IBluetoothAudioProvider> provider = nullptr;
    std::promise<void> hidl_openProvider_promise;
    auto hidl_openProvider_future = hidl_openProvider_promise.get_future();
    if (factory_2_2 != nullptr) {
        factory_2_2->getProviderCapabilities_2_1(
                to_session_type_2_2(session_type),
                [&hidl_openProvider_promise,
                 &_aidl_return](hidl_vec<AudioCapabilities_2_2> codec_capabilities_hidl) {
                    _aidl_return->resize(codec_capabilities_hidl.size());
                    for (int i = 0; i < codec_capabilities_hidl.size(); i++) {
                        _aidl_return->at(i) =
                                from_audio_capabilities_2_2(codec_capabilities_hidl[i]);
                    }
                    hidl_openProvider_promise.set_value();
                });
    } else {
        factory_2_1->getProviderCapabilities(
                to_session_type_2_1(session_type),
                [&hidl_openProvider_promise,
                 &_aidl_return](hidl_vec<AudioCapabilities_2_1> codec_capabilities_hidl) {
                    _aidl_return->resize(codec_capabilities_hidl.size());
                    for (int i = 0; i < codec_capabilities_hidl.size(); i++) {
                        _aidl_return->at(i) =
                                from_audio_capabilities_2_1(codec_capabilities_hidl[i]);
                    }
                    hidl_openProvider_promise.set_value();
                });
    }
    hidl_openProvider_future.get();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MTKBluetoothAudioProviderFactory::getProviderInfo(
        SessionType session_type, std::optional<ProviderInfo>* _aidl_return) {
    if (isHardwareSessionType(session_type)) {
        *_aidl_return = std::nullopt;

        // Implementing getProviderInfo equates supporting
        // A2dp codec extensibility.
        // A2dp Codec extensibility cannot be enabled until the following
        // requirements are fulfilled.
        //
        //  1. The Bluetooth controller must support the HCI Requirements
        //     v1.04 or later, and must support the vendor HCI command
        //     A2DP Offload Start (v2), A2DP Offload Stop (v2) as indicated
        //     by the field a2dp_offload_v2 of the vendor capabilities.
        //
        //  2. The implementation of the provider must be completed with
        //     DSP configuration for streaming.
        return ndk::ScopedAStatus::fromStatus(STATUS_UNKNOWN_TRANSACTION);
    }

    return factory_sw.getProviderInfo(session_type, _aidl_return);
}

}  // namespace adapter
}  // namespace audio
}  // namespace bluetooth
}  // namespace hardware
}  // namespace mediatek
}  // namespace vendor
