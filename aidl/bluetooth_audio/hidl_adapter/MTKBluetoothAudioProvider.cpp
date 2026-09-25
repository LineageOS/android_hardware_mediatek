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
#include <fmq/ConvertMQDescriptors.h>
#include <future>

#include "HidlToAidlMiddleware_2_1.h"
#include "MTKBluetoothAudioProvider.h"

namespace vendor {
namespace mediatek {
namespace hardware {
namespace bluetooth {
namespace audio {
namespace adapter {
using namespace ::aidl::android::hardware::bluetooth::audio;
using ::android::AidlMessageQueue;

using AudioConfig_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::AudioConfiguration;
using HidlStatus = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::Status;
using PcmConfig_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::PcmParameters;
using SampleRate_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SampleRate;
using ChannelMode_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::ChannelMode;
using BitsPerSample_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::BitsPerSample;
using CodecConfig_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::CodecConfiguration;
using CodecType_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::CodecType;
using SbcConfig_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SbcParameters;
using AacConfig_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::AacParameters;
using LdacConfig_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::LdacParameters;
using AptxConfig_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::AptxParameters;
using SbcAllocMethod_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SbcAllocMethod;
using SbcBlockLength_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SbcBlockLength;
using SbcChannelMode_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SbcChannelMode;
using SbcNumSubbands_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::SbcNumSubbands;
using AacObjectType_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::AacObjectType;
using AacVarBitRate_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::AacVariableBitRate;
using LdacChannelMode_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::LdacChannelMode;
using LdacQualityIndex_2_1 = ::vendor::mediatek::hardware::bluetooth::audio::V2_1::LdacQualityIndex;
using LowLatencyEnabled_2_1 =
        ::vendor::mediatek::hardware::bluetooth::audio::V2_1::LowLatencyEnabled;

using AudioConfig_2_2 = ::vendor::mediatek::hardware::bluetooth::audio::V2_2::AudioConfiguration;
using CodecType_2_2 = ::vendor::mediatek::hardware::bluetooth::audio::V2_2::CodecType;
using PcmConfig_2_2 = ::vendor::mediatek::hardware::bluetooth::audio::V2_2::PcmParameters;
using SampleRate_2_2 = ::vendor::mediatek::hardware::bluetooth::audio::V2_2::SampleRate;
using Lc3CodecConfig_2_2 =
        ::vendor::mediatek::hardware::bluetooth::audio::V2_2::LeAudioCodecConfiguration;
using Lc3Config_2_2 = ::vendor::mediatek::hardware::bluetooth::audio::V2_2::Lc3Parameters;
using Lc3FrameDuration_2_2 = ::vendor::mediatek::hardware::bluetooth::audio::V2_2::Lc3FrameDuration;
using PlcMethod_2_2 = ::vendor::mediatek::hardware::bluetooth::audio::V2_2::PlcMethod;

const static std::unordered_map<int32_t, SampleRate_2_2> sample_rate_to_hidl_2_2_map{
        {44100, SampleRate_2_2::RATE_44100},   {48000, SampleRate_2_2::RATE_48000},
        {88200, SampleRate_2_2::RATE_88200},   {96000, SampleRate_2_2::RATE_96000},
        {176400, SampleRate_2_2::RATE_176400}, {192000, SampleRate_2_2::RATE_192000},
        {16000, SampleRate_2_2::RATE_16000},   {24000, SampleRate_2_2::RATE_24000},
        {8000, SampleRate_2_2::RATE_8000},     {32000, SampleRate_2_2::RATE_32000},
};

const static std::unordered_map<CodecType, CodecType_2_1> codec_type_to_hidl_2_1_map{
        {CodecType::UNKNOWN, CodecType_2_1::UNKNOWN}, {CodecType::SBC, CodecType_2_1::SBC},
        {CodecType::AAC, CodecType_2_1::AAC},         {CodecType::APTX, CodecType_2_1::APTX},
        {CodecType::APTX_HD, CodecType_2_1::APTX_HD}, {CodecType::LDAC, CodecType_2_1::LDAC},
        {CodecType::LC3, CodecType_2_1::UNKNOWN},
};

const static std::unordered_map<SbcChannelMode, SbcChannelMode_2_1>
        sbc_channel_mode_to_hidl_2_1_map{
                {SbcChannelMode::UNKNOWN, SbcChannelMode_2_1::UNKNOWN},
                {SbcChannelMode::JOINT_STEREO, SbcChannelMode_2_1::JOINT_STEREO},
                {SbcChannelMode::STEREO, SbcChannelMode_2_1::STEREO},
                {SbcChannelMode::DUAL, SbcChannelMode_2_1::DUAL},
                {SbcChannelMode::MONO, SbcChannelMode_2_1::MONO},
        };

const static std::unordered_map<int8_t, SbcBlockLength_2_1> sbc_block_length_to_hidl_map{
        {4, SbcBlockLength_2_1::BLOCKS_4},
        {8, SbcBlockLength_2_1::BLOCKS_8},
        {12, SbcBlockLength_2_1::BLOCKS_12},
        {16, SbcBlockLength_2_1::BLOCKS_16},
};

const static std::unordered_map<int8_t, SbcNumSubbands_2_1> sbc_subbands_to_hidl_map{
        {4, SbcNumSubbands_2_1::SUBBAND_4},
        {8, SbcNumSubbands_2_1::SUBBAND_8},
};

const static std::unordered_map<SbcAllocMethod, SbcAllocMethod_2_1> sbc_alloc_method_to_hidl_map{
        {SbcAllocMethod::ALLOC_MD_S, SbcAllocMethod_2_1::ALLOC_MD_S},
        {SbcAllocMethod::ALLOC_MD_L, SbcAllocMethod_2_1::ALLOC_MD_L},
};

const static std::unordered_map<AacObjectType, AacObjectType_2_1> aac_object_type_to_hidl_map{
        {AacObjectType::MPEG2_LC, AacObjectType_2_1::MPEG2_LC},
        {AacObjectType::MPEG4_LC, AacObjectType_2_1::MPEG4_LC},
        {AacObjectType::MPEG4_LTP, AacObjectType_2_1::MPEG4_LTP},
        {AacObjectType::MPEG4_SCALABLE, AacObjectType_2_1::MPEG4_SCALABLE},
};

const static std::unordered_map<LdacChannelMode, LdacChannelMode_2_1> ldac_channel_mode_to_hidl_map{
        {LdacChannelMode::UNKNOWN, LdacChannelMode_2_1::UNKNOWN},
        {LdacChannelMode::STEREO, LdacChannelMode_2_1::STEREO},
        {LdacChannelMode::DUAL, LdacChannelMode_2_1::DUAL},
        {LdacChannelMode::MONO, LdacChannelMode_2_1::MONO},
};

const static std::unordered_map<LdacQualityIndex, LdacQualityIndex_2_1> ldac_qindex_to_hidl_map{
        {LdacQualityIndex::HIGH, LdacQualityIndex_2_1::QUALITY_HIGH},
        {LdacQualityIndex::MID, LdacQualityIndex_2_1::QUALITY_MID},
        {LdacQualityIndex::LOW, LdacQualityIndex_2_1::QUALITY_LOW},
        {LdacQualityIndex::ABR, LdacQualityIndex_2_1::QUALITY_ABR},
};

inline SampleRate_2_2 to_hidl_sample_rate_2_2(const int32_t sample_rate_hz) {
    auto it = sample_rate_to_hidl_2_2_map.find(sample_rate_hz);
    if (it != sample_rate_to_hidl_2_2_map.end()) return it->second;
    return SampleRate_2_2::RATE_UNKNOWN;
}

inline SampleRate_2_1 to_hidl_sample_rate_2_1(const int32_t sample_rate_hz) {
    auto it = sample_rate_to_hidl_2_2_map.find(sample_rate_hz);
    if (it != sample_rate_to_hidl_2_2_map.end()) return static_cast<SampleRate_2_1>(it->second);
    return SampleRate_2_1::RATE_UNKNOWN;
}

inline BitsPerSample_2_1 to_hidl_bits_per_sample(const int8_t bit_per_sample) {
    switch (bit_per_sample) {
        case 16:
            return BitsPerSample_2_1::BITS_16;
        case 24:
            return BitsPerSample_2_1::BITS_24;
        case 32:
            return BitsPerSample_2_1::BITS_32;
        default:
            return BitsPerSample_2_1::BITS_UNKNOWN;
    }
}

inline ChannelMode_2_1 to_hidl_channel_mode(const ChannelMode channel_mode) {
    switch (channel_mode) {
        case ChannelMode::MONO:
            return ChannelMode_2_1::MONO;
        case ChannelMode::STEREO:
            return ChannelMode_2_1::STEREO;
        default:
            return ChannelMode_2_1::UNKNOWN;
    }
}

inline PcmConfig_2_1 to_hidl_pcm_config_2_1(const PcmConfiguration& pcm_config) {
    PcmConfig_2_1 hidl_pcm_config;
    hidl_pcm_config.sampleRate = to_hidl_sample_rate_2_1(pcm_config.sampleRateHz);
    hidl_pcm_config.channelMode = to_hidl_channel_mode(pcm_config.channelMode);
    hidl_pcm_config.bitsPerSample = to_hidl_bits_per_sample(pcm_config.bitsPerSample);
    /* TODO: should this be set based on dataIntervalUs being smaller than 20000? */
    hidl_pcm_config.isLowLatencyEnabled = LowLatencyEnabled_2_1::Disabled;
    return hidl_pcm_config;
}

inline CodecType_2_1 to_hidl_codec_type_2_1(const CodecType codec_type) {
    auto it = codec_type_to_hidl_2_1_map.find(codec_type);
    if (it != codec_type_to_hidl_2_1_map.end()) return it->second;
    return CodecType_2_1::UNKNOWN;
}

inline CodecType_2_2 to_hidl_codec_type_2_2(const CodecType codec_type) {
    if (codec_type == CodecType::LC3) return CodecType_2_2::LC3;
    return static_cast<CodecType_2_2>(to_hidl_codec_type_2_1(codec_type));
}

inline SbcConfig_2_1 to_hidl_sbc_config(const SbcConfiguration sbc_config) {
    SbcConfig_2_1 hidl_sbc_config;
    hidl_sbc_config.minBitpool = sbc_config.minBitpool;
    hidl_sbc_config.maxBitpool = sbc_config.maxBitpool;
    hidl_sbc_config.sampleRate = to_hidl_sample_rate_2_1(sbc_config.sampleRateHz);
    hidl_sbc_config.bitsPerSample = to_hidl_bits_per_sample(sbc_config.bitsPerSample);
    if (sbc_channel_mode_to_hidl_2_1_map.find(sbc_config.channelMode) !=
        sbc_channel_mode_to_hidl_2_1_map.end()) {
        hidl_sbc_config.channelMode = sbc_channel_mode_to_hidl_2_1_map.at(sbc_config.channelMode);
    }
    if (sbc_block_length_to_hidl_map.find(sbc_config.blockLength) !=
        sbc_block_length_to_hidl_map.end()) {
        hidl_sbc_config.blockLength = sbc_block_length_to_hidl_map.at(sbc_config.blockLength);
    }
    if (sbc_subbands_to_hidl_map.find(sbc_config.numSubbands) != sbc_subbands_to_hidl_map.end()) {
        hidl_sbc_config.numSubbands = sbc_subbands_to_hidl_map.at(sbc_config.numSubbands);
    }
    if (sbc_alloc_method_to_hidl_map.find(sbc_config.allocMethod) !=
        sbc_alloc_method_to_hidl_map.end()) {
        hidl_sbc_config.allocMethod = sbc_alloc_method_to_hidl_map.at(sbc_config.allocMethod);
    }
    return hidl_sbc_config;
}

inline AacConfig_2_1 to_hidl_aac_config(const AacConfiguration aac_config) {
    AacConfig_2_1 hidl_aac_config;
    hidl_aac_config.sampleRate = to_hidl_sample_rate_2_1(aac_config.sampleRateHz);
    hidl_aac_config.bitsPerSample = to_hidl_bits_per_sample(aac_config.bitsPerSample);
    hidl_aac_config.channelMode = to_hidl_channel_mode(aac_config.channelMode);
    if (aac_object_type_to_hidl_map.find(aac_config.objectType) !=
        aac_object_type_to_hidl_map.end()) {
        hidl_aac_config.objectType = aac_object_type_to_hidl_map.at(aac_config.objectType);
    }
    hidl_aac_config.variableBitRateEnabled = aac_config.variableBitRateEnabled
                                                     ? AacVarBitRate_2_1::ENABLED
                                                     : AacVarBitRate_2_1::DISABLED;
    return hidl_aac_config;
}

inline LdacConfig_2_1 to_hidl_ldac_config(const LdacConfiguration ldac_config) {
    LdacConfig_2_1 hidl_ldac_config;
    hidl_ldac_config.sampleRate = to_hidl_sample_rate_2_1(ldac_config.sampleRateHz);
    hidl_ldac_config.bitsPerSample = to_hidl_bits_per_sample(ldac_config.bitsPerSample);
    if (ldac_channel_mode_to_hidl_map.find(ldac_config.channelMode) !=
        ldac_channel_mode_to_hidl_map.end()) {
        hidl_ldac_config.channelMode = ldac_channel_mode_to_hidl_map.at(ldac_config.channelMode);
    }
    if (ldac_qindex_to_hidl_map.find(ldac_config.qualityIndex) != ldac_qindex_to_hidl_map.end()) {
        hidl_ldac_config.qualityIndex = ldac_qindex_to_hidl_map.at(ldac_config.qualityIndex);
    }
    return hidl_ldac_config;
}

inline AptxConfig_2_1 to_hidl_aptx_config(const AptxConfiguration aptx_config) {
    AptxConfig_2_1 hidl_aptx_config;
    hidl_aptx_config.sampleRate = to_hidl_sample_rate_2_1(aptx_config.sampleRateHz);
    hidl_aptx_config.bitsPerSample = to_hidl_bits_per_sample(aptx_config.bitsPerSample);
    hidl_aptx_config.channelMode = to_hidl_channel_mode(aptx_config.channelMode);
    return hidl_aptx_config;
}

inline CodecConfig_2_1 to_hidl_codec_config_2_1(const CodecConfiguration& codec_config) {
    CodecConfig_2_1 hidl_codec_config;
    hidl_codec_config.codecType = to_hidl_codec_type_2_1(codec_config.codecType);
    hidl_codec_config.encodedAudioBitrate = static_cast<uint32_t>(codec_config.encodedAudioBitrate);
    hidl_codec_config.peerMtu = static_cast<uint32_t>(codec_config.peerMtu);
    hidl_codec_config.isScmstEnabled = codec_config.isScmstEnabled;
    switch (codec_config.config.getTag()) {
        case CodecConfiguration::CodecSpecific::sbcConfig:
            hidl_codec_config.config.sbcConfig(to_hidl_sbc_config(
                    codec_config.config.get<CodecConfiguration::CodecSpecific::sbcConfig>()));
            break;
        case CodecConfiguration::CodecSpecific::aacConfig:
            hidl_codec_config.config.aacConfig(to_hidl_aac_config(
                    codec_config.config.get<CodecConfiguration::CodecSpecific::aacConfig>()));
            break;
        case CodecConfiguration::CodecSpecific::ldacConfig:
            hidl_codec_config.config.ldacConfig(to_hidl_ldac_config(
                    codec_config.config.get<CodecConfiguration::CodecSpecific::ldacConfig>()));
            break;
        case CodecConfiguration::CodecSpecific::aptxConfig:
            hidl_codec_config.config.aptxConfig(to_hidl_aptx_config(
                    codec_config.config.get<CodecConfiguration::CodecSpecific::aptxConfig>()));
            break;
        default:
            break;
    }
    return hidl_codec_config;
}

inline AudioConfig_2_1 to_hidl_audio_config_2_1(const AudioConfiguration& audio_config) {
    AudioConfig_2_1 hidl_audio_config;
    if (audio_config.getTag() == AudioConfiguration::pcmConfig) {
        hidl_audio_config.pcmConfig(
                to_hidl_pcm_config_2_1(audio_config.get<AudioConfiguration::pcmConfig>()));
    } else if (audio_config.getTag() == AudioConfiguration::a2dpConfig) {
        hidl_audio_config.codecConfig(
                to_hidl_codec_config_2_1(audio_config.get<AudioConfiguration::a2dpConfig>()));
    }
    return hidl_audio_config;
}

inline PcmConfig_2_2 to_hidl_pcm_config_2_2(const PcmConfiguration& pcm_config) {
    PcmConfig_2_2 hidl_pcm_config;
    hidl_pcm_config.sampleRate = to_hidl_sample_rate_2_2(pcm_config.sampleRateHz);
    hidl_pcm_config.channelMode = to_hidl_channel_mode(pcm_config.channelMode);
    hidl_pcm_config.bitsPerSample = to_hidl_bits_per_sample(pcm_config.bitsPerSample);
    hidl_pcm_config.dataIntervalUs = static_cast<uint32_t>(pcm_config.dataIntervalUs);
    return hidl_pcm_config;
}

inline Lc3Config_2_2 to_hidl_lc3_config_2_2(const Lc3Configuration& lc3_config) {
    Lc3Config_2_2 hidl_lc3_config;
    hidl_lc3_config.pcmBitDepth = to_hidl_bits_per_sample(lc3_config.pcmBitDepth);
    hidl_lc3_config.samplingFrequency = to_hidl_sample_rate_2_2(lc3_config.samplingFrequencyHz);
    if (lc3_config.samplingFrequencyHz == 10000)
        hidl_lc3_config.frameDuration = Lc3FrameDuration_2_2::DURATION_10000US;
    else if (lc3_config.samplingFrequencyHz == 7500)
        hidl_lc3_config.frameDuration = Lc3FrameDuration_2_2::DURATION_7500US;
    hidl_lc3_config.octetsPerFrame = static_cast<uint32_t>(lc3_config.octetsPerFrame);
    hidl_lc3_config.blocksPerSdu = static_cast<uint32_t>(lc3_config.blocksPerSdu);
    return hidl_lc3_config;
}

inline Lc3CodecConfig_2_2 to_hidl_leaudio_config_2_2(const LeAudioConfiguration& unicast_config) {
    Lc3CodecConfig_2_2 hidl_lc3_codec_config = {
            .audioChannelAllocation = 0,
            .encodedAudioBitrate = 0,                   // TODO: What should this be set to?
            .plc_method = PlcMethod_2_2::STANDARD_PLC,  // TODO: What should this be set to?
            .le_audio_type = 0,                         // TODO: What should this be set to?
    };
    hidl_lc3_codec_config.codecType = to_hidl_codec_type_2_2(unicast_config.codecType);
    if (unicast_config.leAudioCodecConfig.getTag() == LeAudioCodecConfiguration::lc3Config) {
        LOG(FATAL) << __func__ << ": unexpected codec type(vendor?)";
    }
    auto& le_codec_config =
            unicast_config.leAudioCodecConfig.get<LeAudioCodecConfiguration::lc3Config>();

    hidl_lc3_codec_config.lc3Config = to_hidl_lc3_config_2_2(le_codec_config);

    for (const auto& map : unicast_config.streamMap) {
        hidl_lc3_codec_config.audioChannelAllocation |= map.audioChannelAllocation;
    }
    return hidl_lc3_codec_config;
}

inline Lc3CodecConfig_2_2 to_hidl_leaudio_broadcast_config_2_2(
        const LeAudioBroadcastConfiguration& broadcast_config) {
    Lc3CodecConfig_2_2 hidl_lc3_codec_config = {
            .audioChannelAllocation = 0,
            .encodedAudioBitrate = 0,                   // TODO: What should this be set to?
            .plc_method = PlcMethod_2_2::STANDARD_PLC,  // TODO: What should this be set to?
            .le_audio_type = 0,                         // TODO: What should this be set to?
    };
    hidl_lc3_codec_config.codecType = to_hidl_codec_type_2_2(broadcast_config.codecType);
    // NOTE: Broadcast is not officially supported in HIDL
    if (broadcast_config.streamMap.empty()) {
        return hidl_lc3_codec_config;
    }
    if (broadcast_config.streamMap[0].leAudioCodecConfig.getTag() !=
        LeAudioCodecConfiguration::lc3Config) {
        LOG(FATAL) << __func__ << ": unexpected codec type(vendor?)";
    }
    auto& le_codec_config = broadcast_config.streamMap[0]
                                    .leAudioCodecConfig.get<LeAudioCodecConfiguration::lc3Config>();
    hidl_lc3_codec_config.lc3Config = to_hidl_lc3_config_2_2(le_codec_config);

    for (const auto& map : broadcast_config.streamMap) {
        hidl_lc3_codec_config.audioChannelAllocation |= map.audioChannelAllocation;
    }
    return hidl_lc3_codec_config;
}

inline AudioConfig_2_2 to_hidl_audio_config_2_2(const AudioConfiguration& audio_config) {
    AudioConfig_2_2 hidl_audio_config;
    switch (audio_config.getTag()) {
        case AudioConfiguration::pcmConfig:
            hidl_audio_config.pcmConfig(
                    to_hidl_pcm_config_2_2(audio_config.get<AudioConfiguration::pcmConfig>()));
            break;
        case AudioConfiguration::a2dpConfig:
            hidl_audio_config.codecConfig(
                    to_hidl_codec_config_2_1(audio_config.get<AudioConfiguration::a2dpConfig>()));
            break;
        case AudioConfiguration::a2dp:
            break;
        case AudioConfiguration::leAudioConfig:
            hidl_audio_config.leAudioCodecConfig(to_hidl_leaudio_config_2_2(
                    audio_config.get<AudioConfiguration::leAudioConfig>()));
            break;
        case AudioConfiguration::leAudioBroadcastConfig:
            hidl_audio_config.leAudioCodecConfig(to_hidl_leaudio_broadcast_config_2_2(
                    audio_config.get<AudioConfiguration::leAudioBroadcastConfig>()));
            break;
        default:
            LOG(FATAL) << __func__ << ": unexpected AudioConfiguration";
    }
    return hidl_audio_config;
}

inline HidlStatus to_hidl_status(const BluetoothAudioStatus& status) {
    switch (status) {
        case BluetoothAudioStatus::SUCCESS:
            return HidlStatus::SUCCESS;
        case BluetoothAudioStatus::UNSUPPORTED_CODEC_CONFIGURATION:
            return HidlStatus::UNSUPPORTED_CODEC_CONFIGURATION;
        default:
            return HidlStatus::FAILURE;
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
        android::sp<IBluetoothAudioProvider_2_1> provider)
    : provider_2_1(provider), provider_2_2(IBluetoothAudioProvider_2_2::castFrom(provider)) {
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
    HidlStatus session_status;

    std::promise<void> hidl_startSession_promise;
    auto hidl_startSession_future = hidl_startSession_promise.get_future();
    auto hidl_cb = [&session_status, &hidl_startSession_promise, &_aidl_return](
                           HidlStatus status,
                           const ::android::hardware::MQDescriptorSync<unsigned char>& dataMQ) {
        session_status = status;
        *_aidl_return = DataMQDesc();
        if (status == HidlStatus::SUCCESS && dataMQ.isHandleValid()) {
            android::unsafeHidlToAidlMQDescriptor<unsigned char, int8_t, SynchronizedReadWrite>(
                    dataMQ, _aidl_return);
        }
        hidl_startSession_promise.set_value();
    };
    auto stack_if = android::sp<HidlToAidlMiddleware_2_1>::make(host_if);
    if (provider_2_2 != nullptr) {
        provider_2_2->startSession_2_1(stack_if, to_hidl_audio_config_2_2(audio_config), hidl_cb);
    } else {
        provider_2_1->startSession(stack_if, to_hidl_audio_config_2_1(audio_config), hidl_cb);
    }
    hidl_startSession_future.get();
    if (session_status == HidlStatus::SUCCESS) {
        stack_iface_ = host_if;
        BluetoothAudioProviderContext* cookie =
                new BluetoothAudioProviderContext{ref<MTKBluetoothAudioProvider>()};
        AIBinder_linkToDeath(stack_iface_->asBinder().get(), death_recipient_.get(), cookie);
        return ndk::ScopedAStatus::ok();
    } else if (session_status == HidlStatus::UNSUPPORTED_CODEC_CONFIGURATION) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    } else {
        return ndk::ScopedAStatus::fromExceptionCode(EX_SERVICE_SPECIFIC);
    }
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::endSession() {
    if (stack_iface_ != nullptr) {
        AIBinder_unlinkToDeath(stack_iface_->asBinder().get(), death_recipient_.get(), this);
        stack_iface_ = nullptr;
    } else {
        LOG(INFO) << __func__ << " - has NO session";
    }
    provider_2_1->endSession();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::streamStarted(BluetoothAudioStatus status) {
    provider_2_1->streamStarted(to_hidl_status(status));
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::streamSuspended(BluetoothAudioStatus status) {
    provider_2_1->streamSuspended(to_hidl_status(status));
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::updateAudioConfiguration(
        const AudioConfiguration& audio_config) {
    // AOSP implementation of HIDL adapter will not notify HAL of this,
    // so we don't need to do anything here.
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus MTKBluetoothAudioProvider::setLowLatencyModeAllowed(bool allowed) {
    // AOSP implementation of HIDL adapter will not notify HAL of this,
    // so we don't need to do anything here.
    /* TODO: but maybe we _want_ to do something here? we do have enterGameMode... */
    return ndk::ScopedAStatus::ok();
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
