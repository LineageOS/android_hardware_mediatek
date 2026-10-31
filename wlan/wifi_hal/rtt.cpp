/*
 * Copyright (C) 2017 The Android Open Source Project
 *
 * Portions copyright (C) 2023 Broadcom Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#include <linux/rtnetlink.h>
#include <netpacket/packet.h>
#include <linux/filter.h>
#include <linux/errqueue.h>

#include <linux/pkt_sched.h>
#include <netlink/object-api.h>
#include <netlink/netlink.h>
#include <netlink/socket.h>

#include "nl80211_copy.h"

#include "sync.h"

#define LOG_TAG  "WifiHAL"

#include <log/log.h>
#include <utils/String8.h>

#include <hardware_legacy/wifi_hal.h>
#include "common.h"
#include "cpp_bindings.h"

using namespace android;
#define RTT_RESULT_V4_SIZE (sizeof(wifi_rtt_result_v4))
#define RTT_RESULT_V3_SIZE (sizeof(wifi_rtt_result_v3))
#define RTT_RESULT_V2_SIZE (sizeof(wifi_rtt_result_v2))
#define RTT_RESULT_V1_SIZE (sizeof(wifi_rtt_result))
#define UNSPECIFIED -1 // wifi HAL common definition for unspecified value
/* Loglevel */
#define RTT_DEBUG(x)
#define RTT_INFO(x) ALOGI x

typedef enum {

    RTT_SUBCMD_SET_CONFIG = ANDROID_NL80211_SUBCMD_RTT_RANGE_START,
    RTT_SUBCMD_CANCEL_CONFIG,
    RTT_SUBCMD_GETCAPABILITY,
    RTT_SUBCMD_GETAVAILCHANNEL,
    RTT_SUBCMD_SET_RESPONDER,
    RTT_SUBCMD_CANCEL_RESPONDER,
} RTT_SUB_COMMAND;

typedef enum {
    RTT_ATTRIBUTE_TARGET_INVALID            = 0,
    RTT_ATTRIBUTE_TARGET_CNT                = 1,
    RTT_ATTRIBUTE_TARGET_INFO               = 2,
    RTT_ATTRIBUTE_TARGET_MAC                = 3,
    RTT_ATTRIBUTE_TARGET_TYPE               = 4,
    RTT_ATTRIBUTE_TARGET_PEER               = 5,
    RTT_ATTRIBUTE_TARGET_CHAN               = 6,
    RTT_ATTRIBUTE_TARGET_PERIOD             = 7,
    RTT_ATTRIBUTE_TARGET_NUM_BURST          = 8,
    RTT_ATTRIBUTE_TARGET_NUM_FTM_BURST      = 9,
    RTT_ATTRIBUTE_TARGET_NUM_RETRY_FTM      = 10,
    RTT_ATTRIBUTE_TARGET_NUM_RETRY_FTMR     = 11,
    RTT_ATTRIBUTE_TARGET_LCI                = 12,
    RTT_ATTRIBUTE_TARGET_LCR                = 13,
    RTT_ATTRIBUTE_TARGET_BURST_DURATION     = 14,
    RTT_ATTRIBUTE_TARGET_PREAMBLE           = 15,
    RTT_ATTRIBUTE_TARGET_BW                 = 16,
    RTT_ATTRIBUTE_TARGET_NTB_MIN_MEAS_TIME  = 17,
    RTT_ATTRIBUTE_TARGET_NTB_MAX_MEAS_TIME  = 18,
    /* Add Attributes related to the event */
    RTT_ATTRIBUTE_RESULTS_COMPLETE          = 30,
    RTT_ATTRIBUTE_RESULTS_PER_TARGET        = 31,
    RTT_ATTRIBUTE_RESULT_CNT                = 32,
    RTT_ATTRIBUTE_RESULT                    = 33,
    RTT_ATTRIBUTE_RESUTL_DETAIL             = 34,
    RTT_ATTRIBUTE_RESULT_FREQ               = 35,
    RTT_ATTRIBUTE_RESULT_BW                 = 36,
    RTT_ATTRIBUTE_RESULT_I2R_TX_LTF_RPT_CNT = 37,
    RTT_ATTRIBUTE_RESULT_R2I_TX_LTF_RPT_CNT = 38,
    RTT_ATTRIBUTE_RESULT_NTB_MIN_MEAS_TIME  = 39,
    RTT_ATTRIBUTE_RESULT_NTB_MAX_MEAS_TIME  = 40,

    /* Security */
    RTT_ATTRIBUTE_TARGET_PROTECTED_FRM_REQD = 41,
    RTT_ATTRIBUTE_TARGET_KEY_LIFE_TIME      = 42,
    RTT_ATTRIBUTE_TARGET_RTT_AKM            = 43,
    RTT_ATTRIBUTE_TARGET_SEC_LTF_REQD       = 44,
    RTT_ATTRIBUTE_TARGET_KEY_PASSPHRASE     = 45,
    RTT_ATTRIBUTE_TARGET_KEY_PASSPHRASE_LEN = 46,
    RTT_ATTRIBUTE_TARGET_CIPHER_TYPE        = 47,
    RTT_ATTRIBUTE_RESULT_NTB_I2R_STS        = 48,
    RTT_ATTRIBUTE_RESULT_NTB_R2I_STS        = 49,
    RTT_ATTRIBUTE_RESULT_RNG_PROT_ENABLED   = 50,
    RTT_ATTRIBUTE_RESULT_SLTF_ENABLED       = 51,
    RTT_ATTRIBUTE_RESULT_RTT_AKM            = 52,
    RTT_ATTRIBUTE_RESULT_CIPHER_TYPE        = 53,
    RTT_ATTRIBUTE_RESULT_SLTF_PROTO_VER	    = 54,
    /* Add any new RTT_ATTRIBUTE prior to RTT_ATTRIBUTE_MAX */
    RTT_ATTRIBUTE_MAX
} RTT_ATTRIBUTE;
typedef struct strmap_entry {
    int			id;
    String8		text;
} strmap_entry_t;
struct dot11_rm_ie {
    u8 id;
    u8 len;
    u8 token;
    u8 mode;
    u8 type;
} __attribute__ ((packed));
typedef struct dot11_rm_ie dot11_rm_ie_t;
#define DOT11_HDR_LEN 2
#define DOT11_RM_IE_LEN       5
#define DOT11_MNG_MEASURE_REQUEST_ID		38	/* 11H MeasurementRequest */
#define DOT11_MNG_MEASURE_REPORT_ID		39	/* 11H MeasurementResponse */
#define DOT11_MEASURE_TYPE_LCI		8   /* d11 measurement LCI type */
#define DOT11_MEASURE_TYPE_CIVICLOC	11  /* d11 measurement location civic */

static const strmap_entry_t err_info[] = {
    {RTT_STATUS_SUCCESS, String8("Success")},
    {RTT_STATUS_FAILURE, String8("Failure")},
    {RTT_STATUS_FAIL_NO_RSP, String8("No reponse")},
    {RTT_STATUS_FAIL_INVALID_TS, String8("Invalid Timestamp")},
    {RTT_STATUS_FAIL_PROTOCOL, String8("Protocol error")},
    {RTT_STATUS_FAIL_REJECTED, String8("Rejected")},
    {RTT_STATUS_FAIL_NOT_SCHEDULED_YET, String8("not scheduled")},
    {RTT_STATUS_FAIL_SCHEDULE,  String8("schedule failed")},
    {RTT_STATUS_FAIL_TM_TIMEOUT, String8("timeout")},
    {RTT_STATUS_FAIL_AP_ON_DIFF_CHANNEL, String8("AP is on difference channel")},
    {RTT_STATUS_FAIL_NO_CAPABILITY, String8("no capability")},
    {RTT_STATUS_FAIL_BUSY_TRY_LATER, String8("busy and try later")},
    {RTT_STATUS_ABORTED, String8("aborted")}
};

static const char*
get_err_info(int status)
{
    int i;
    const strmap_entry_t *p_entry;
    int num_entries = sizeof(err_info)/ sizeof(err_info[0]);
    /* scan thru the table till end */
    p_entry = err_info;
    for (i = 0; i < (int) num_entries; i++)
    {
        if (p_entry->id == status)
            return p_entry->text.c_str();
        p_entry++;		/* next entry */
    }
    return "unknown error";			/* not found */
}

class GetRttCapabilitiesCommand : public WifiCommand
{
    wifi_rtt_capabilities_v4 *mCapabilities;

public:
    GetRttCapabilitiesCommand(wifi_interface_handle iface, wifi_rtt_capabilities_v4 *capabilities)
        : WifiCommand("GetRttCapabilitiesCommand", iface, 0), mCapabilities(capabilities)
    {
        memset(mCapabilities, 0, sizeof(*mCapabilities));
    }

    virtual int create() {
        ALOGD("Creating message to get rtt capabilities; iface = %d", mIfaceInfo->id);

        int ret = mMsg.create(GOOGLE_OUI, RTT_SUBCMD_GETCAPABILITY);
        if (ret < 0) {
            return ret;
        }

        return ret;
    }

protected:
    virtual int handleResponse(WifiEvent& reply) {
        rtt_capabilities_mc_az_t SrcCapabilities;
        rtt_capabilities_t mcSrcCapabilities;
        wifi_rtt_capabilities_v4 DestCapabilities;

        ALOGD("In GetRttCapabilitiesCommand::handleResponse");

        if ((reply.get_cmd() != NL80211_CMD_VENDOR) || (reply.get_vendor_data() == NULL) ||
                (reply.get_vendor_data_len() < sizeof(rtt_capabilities_t))) {
            ALOGD("Ignoring reply with cmd = %d "
                    "min expected len %d, mc_az rtt capa size %d,"
                    "  mc rtt capa size %d\n",
                    reply.get_cmd(), reply.get_vendor_data_len(),
                    static_cast<int>(sizeof(rtt_capabilities_mc_az_t)),
                    static_cast<int>(sizeof(rtt_capabilities_t)));
            return NL_SKIP;
        }

        int id = reply.get_vendor_id();
        int subcmd = reply.get_vendor_subcmd();
        void *data = reply.get_vendor_data();
        int len = reply.get_vendor_data_len();

        ALOGD("Id = %0x, subcmd = %d, len = %d,"
                " min expected len = %d, max exp len %d\n",
                id, subcmd, len, static_cast<int>(sizeof(rtt_capabilities_t)),
                static_cast<int>(sizeof(rtt_capabilities_mc_az_t)));

        memset(&SrcCapabilities, 0, sizeof(SrcCapabilities));
        memset(&mcSrcCapabilities, 0, sizeof(mcSrcCapabilities));
        memset(&DestCapabilities, 0, sizeof(DestCapabilities));

        if (len == sizeof(mcSrcCapabilities)) {
            memcpy(&mcSrcCapabilities, data, len);
            DestCapabilities.rtt_capab_v3.rtt_capab.rtt_one_sided_supported =
                    mcSrcCapabilities.rtt_one_sided_supported;

            DestCapabilities.rtt_capab_v3.rtt_capab.rtt_ftm_supported =
                    mcSrcCapabilities.rtt_ftm_supported;

            DestCapabilities.rtt_capab_v3.rtt_capab.lci_support =
                    mcSrcCapabilities.lci_support;

            DestCapabilities.rtt_capab_v3.rtt_capab.lcr_support =
                    mcSrcCapabilities.lcr_support;

            DestCapabilities.rtt_capab_v3.rtt_capab.preamble_support =
                    mcSrcCapabilities.preamble_support;

            DestCapabilities.rtt_capab_v3.rtt_capab.bw_support =
                    mcSrcCapabilities.bw_support;

            DestCapabilities.rtt_capab_v3.rtt_capab.responder_supported = 0;

            DestCapabilities.rtt_capab_v3.rtt_capab.mc_version = 0;

            DestCapabilities.rtt_capab_v3.az_preamble_support = 0;

            DestCapabilities.rtt_capab_v3.az_bw_support = 0;

            DestCapabilities.rtt_capab_v3.ntb_initiator_supported = 0;

            DestCapabilities.rtt_capab_v3.ntb_responder_supported = 0;

            DestCapabilities.secure_he_ltf_supported = 0;

            DestCapabilities.ranging_fame_protection_supported = 0;

            DestCapabilities.supported_akms = WPA_KEY_MGMT_NONE;

            DestCapabilities.supported_cipher_suites = WPA_CIPHER_NONE;
        } else {
            memcpy(&SrcCapabilities, data, min((int)sizeof(rtt_capabilities_mc_az_t), len));

            DestCapabilities.rtt_capab_v3.rtt_capab.rtt_one_sided_supported =
                    SrcCapabilities.rtt_capab.rtt_one_sided_supported;

            DestCapabilities.rtt_capab_v3.rtt_capab.rtt_ftm_supported =
                    SrcCapabilities.rtt_capab.rtt_ftm_supported;

            DestCapabilities.rtt_capab_v3.rtt_capab.lci_support =
                    SrcCapabilities.rtt_capab.lci_support;

            DestCapabilities.rtt_capab_v3.rtt_capab.lcr_support =
                    SrcCapabilities.rtt_capab.lcr_support;

            DestCapabilities.rtt_capab_v3.rtt_capab.preamble_support =
                    SrcCapabilities.rtt_capab.preamble_support;

            DestCapabilities.rtt_capab_v3.rtt_capab.bw_support =
                    SrcCapabilities.rtt_capab.bw_support;

            DestCapabilities.rtt_capab_v3.rtt_capab.responder_supported = 0;

            DestCapabilities.rtt_capab_v3.rtt_capab.mc_version = 0;

            DestCapabilities.rtt_capab_v3.az_preamble_support =
                     (rtt_cap_preamble_type_t)SrcCapabilities.az_preamble_support;

            DestCapabilities.rtt_capab_v3.az_bw_support =
                    SrcCapabilities.az_bw_support;

            DestCapabilities.rtt_capab_v3.ntb_initiator_supported =
                    SrcCapabilities.ntb_initiator_supported;

            DestCapabilities.rtt_capab_v3.ntb_responder_supported =
                    SrcCapabilities.ntb_responder_supported;

            DestCapabilities.secure_he_ltf_supported =
                    SrcCapabilities.secure_he_ltf_supported;

            DestCapabilities.ranging_fame_protection_supported =
                    SrcCapabilities.protected_rtt_frm_supported;

            DestCapabilities.supported_akms =
                    (wifi_rtt_akm)SrcCapabilities.supported_akms;

            DestCapabilities.supported_cipher_suites =
                    (wifi_rtt_cipher_suite)SrcCapabilities.supported_cipher_suites;
        }

        memcpy(mCapabilities, &DestCapabilities, sizeof(DestCapabilities));
        return NL_OK;
    }
};


class GetRttResponderInfoCommand : public WifiCommand
{
    wifi_rtt_responder* mResponderInfo;
public:
    GetRttResponderInfoCommand(wifi_interface_handle iface, wifi_rtt_responder *responderInfo)
        : WifiCommand("GetRttResponderInfoCommand", iface, 0), mResponderInfo(responderInfo)
    {
        memset(mResponderInfo, 0 , sizeof(*mResponderInfo));

    }

    virtual int create() {
        ALOGD("Creating message to get responder info ; iface = %d", mIfaceInfo->id);

        int ret = mMsg.create(GOOGLE_OUI, RTT_SUBCMD_GETAVAILCHANNEL);
        if (ret < 0) {
            return ret;
        }

        return ret;
    }

protected:
    virtual int handleResponse(WifiEvent& reply) {

        ALOGD("In GetRttResponderInfoCommand::handleResponse");

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        int id = reply.get_vendor_id();
        int subcmd = reply.get_vendor_subcmd();

        void *data = reply.get_vendor_data();
        int len = reply.get_vendor_data_len();

        ALOGD("Id = %0x, subcmd = %d, len = %d, expected len = %d", id, subcmd, len,
                static_cast<int>(sizeof(*mResponderInfo)));

        memcpy(mResponderInfo, data, min(len, (int) sizeof(*mResponderInfo)));

        return NL_OK;
    }
};


class EnableResponderCommand : public WifiCommand
{
    wifi_channel_info  mChannelInfo;
    wifi_rtt_responder* mResponderInfo;
    unsigned int m_max_duration_sec;
public:
    EnableResponderCommand(wifi_interface_handle iface, int id, wifi_channel_info channel_hint,
            unsigned max_duration_seconds, wifi_rtt_responder *responderInfo)
            : WifiCommand("EnableResponderCommand", iface, 0), mChannelInfo(channel_hint),
            mResponderInfo(responderInfo), m_max_duration_sec(max_duration_seconds)
    {
        memset(mResponderInfo, 0, sizeof(*mResponderInfo));
        memset(&mChannelInfo, 0, sizeof(mChannelInfo));
        m_max_duration_sec = 0;
    }

    virtual int create() {
        ALOGD("Creating message to set responder ; iface = %d", mIfaceInfo->id);

        int ret = mMsg.create(GOOGLE_OUI, RTT_SUBCMD_SET_RESPONDER);
        if (ret < 0) {
            return ret;
        }

        return ret;
    }

protected:
    virtual int handleResponse(WifiEvent& reply) {

        ALOGD("In EnableResponderCommand::handleResponse");

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        int id = reply.get_vendor_id();
        int subcmd = reply.get_vendor_subcmd();

        void *data = reply.get_vendor_data();
        int len = reply.get_vendor_data_len();

        ALOGD("Id = %0x, subcmd = %d, len = %d, expected len = %d", id, subcmd, len,
                static_cast<int>(sizeof(*mResponderInfo)));

        memcpy(mResponderInfo, data, min(len, (int) sizeof(*mResponderInfo)));

        return NL_OK;
    }
};


class CancelResponderCommand : public WifiCommand
{

public:
    CancelResponderCommand(wifi_interface_handle iface, int id)
        : WifiCommand("CancelResponderCommand", iface, 0)/*, mChannelInfo(channel)*/
    {

    }

    virtual int create() {
        ALOGD("Creating message to cancel responder ; iface = %d", mIfaceInfo->id);

        int ret = mMsg.create(GOOGLE_OUI, RTT_SUBCMD_CANCEL_RESPONDER);
        if (ret < 0) {
            return ret;
        }

        return ret;
    }

protected:
    virtual int handleResponse(WifiEvent& reply) {
        /* Nothing to do on response! */
        return NL_SKIP;
    }

};

class RttCommand : public WifiCommand
{
    unsigned numRttParams;
    int mCompleted;
    int currentIdx = 0;
    int totalCnt = 0;
    static const int MAX_RESULTS = 1024;
    wifi_rtt_result *rttResultsV1[MAX_RESULTS];
    wifi_rtt_result_v4 *rttResultsV4[MAX_RESULTS];
    wifi_rtt_config *rttParamsV1;
    wifi_rtt_config_v3 *rttParamsV3;
    wifi_rtt_config_v4 *rttParamsV4;
    wifi_rtt_event_handler_v4 rttHandlerV4;
    int nextidx = 0;
    wifi_channel channel = 0;
    wifi_rtt_bw bw;
    int result_size = 0;
    int opt_result_size = 0;
    u8 i2r_tx_ltf_repetition_count = 0;
    u8 r2i_tx_ltf_repetition_count = 0;
    u32 ntb_min_measurement_time = 0;
    u32 ntb_max_measurement_time = 0;
    wifi_rtt_akm base_akm = WPA_KEY_MGMT_NONE;
    wifi_rtt_cipher_suite cipher_suite = WPA_CIPHER_NONE;
    int secure_he_ltf_protocol_version = 0;
    bool is_ranging_protection_enabled = 0;
    bool is_secure_he_ltf_enabled = 0;
    u8 num_tx_sts = 0, num_rx_sts = 0;

public:
    RttCommand(wifi_interface_handle iface, int id, unsigned num_rtt_config,
            wifi_rtt_config_v4 rtt_config[], wifi_rtt_event_handler_v4 handler)
        : WifiCommand("RttCommand", iface, id), numRttParams(num_rtt_config), rttParamsV4(rtt_config),
        rttHandlerV4(handler)
    {
        memset(rttResultsV1, 0, sizeof(rttResultsV1));
        memset(rttResultsV4, 0, sizeof(rttResultsV4));
        currentIdx = 0;
        mCompleted = 0;
        totalCnt = 0;
        channel = 0;
        result_size = 0;
        opt_result_size = 0;
        channel = 0;
        result_size = 0;
        opt_result_size = 0;
    }

    RttCommand(wifi_interface_handle iface, int id)
        : WifiCommand("RttCommand", iface, id)
    {
        currentIdx = 0;
        mCompleted = 0;
        totalCnt = 0;
        numRttParams = 0;
        memset(rttResultsV1, 0, sizeof(rttResultsV1));
        memset(rttResultsV4, 0, sizeof(rttResultsV4));
        rttParamsV1 = NULL;
        rttParamsV3 = NULL;
        rttParamsV4 = NULL;
        rttHandlerV4.on_rtt_results_v4 = NULL;
        channel = 0;
        result_size = 0;
        opt_result_size = 0;
    }

    int createSetupRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, RTT_SUBCMD_SET_CONFIG);
        if (result < 0) {
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u8(RTT_ATTRIBUTE_TARGET_CNT, numRttParams);
        if (result < 0) {
            return result;
        }
        nlattr *rtt_config = request.attr_start(RTT_ATTRIBUTE_TARGET_INFO);
        for (unsigned i = 0; i < numRttParams; i++) {
            nlattr *attr2 = request.attr_start(i);
            if (attr2 == NULL) {
                return WIFI_ERROR_OUT_OF_MEMORY;
            }

            rttParamsV3 = &rttParamsV4[i].rtt_config;
            rttParamsV1 = &rttParamsV3->rtt_config;
            result = request.put_addr(RTT_ATTRIBUTE_TARGET_MAC, rttParamsV1->addr);
            if (result < 0) {
                return result;
            }

            result = request.put_u8(RTT_ATTRIBUTE_TARGET_TYPE, rttParamsV1->type);
            if (result < 0) {
                return result;
            }

            result = request.put_u8(RTT_ATTRIBUTE_TARGET_PEER, rttParamsV1->peer);
            if (result < 0) {
                return result;
            }

            result = request.put(RTT_ATTRIBUTE_TARGET_CHAN, &rttParamsV1->channel,
                    sizeof(wifi_channel_info));
            if (result < 0) {
                return result;
            }

            result = request.put_u32(RTT_ATTRIBUTE_TARGET_NUM_BURST, rttParamsV1->num_burst);
            if (result < 0) {
                return result;
            }

            ALOGI("num_frames_per_burst %d\n", rttParamsV1->num_frames_per_burst);
            result = request.put_u32(RTT_ATTRIBUTE_TARGET_NUM_FTM_BURST,
                    rttParamsV1->num_frames_per_burst);
            if (result < 0) {
                return result;
            }

            result = request.put_u32(RTT_ATTRIBUTE_TARGET_NUM_RETRY_FTM,
                    rttParamsV1->num_retries_per_rtt_frame);
            if (result < 0) {
                return result;
            }

            result = request.put_u32(RTT_ATTRIBUTE_TARGET_NUM_RETRY_FTMR,
                    rttParamsV1->num_retries_per_ftmr);
            if (result < 0) {
                return result;
            }

            result = request.put_u32(RTT_ATTRIBUTE_TARGET_PERIOD,
                    rttParamsV1->burst_period);
            if (result < 0) {
                return result;
            }

            result = request.put_u32(RTT_ATTRIBUTE_TARGET_BURST_DURATION,
                    rttParamsV1->burst_duration);
            if (result < 0) {
                return result;
            }

            result = request.put_u8(RTT_ATTRIBUTE_TARGET_LCI, rttParamsV1->LCI_request);
            if (result < 0) {
                return result;
            }

            result = request.put_u8(RTT_ATTRIBUTE_TARGET_LCR, rttParamsV1->LCR_request);
            if (result < 0) {
                return result;
            }

            result = request.put_u8(RTT_ATTRIBUTE_TARGET_BW, rttParamsV1->bw);
            if (result < 0) {
                return result;
            }

            result = request.put_u8(RTT_ATTRIBUTE_TARGET_PREAMBLE, rttParamsV1->preamble);
            if (result < 0) {
                return result;
            }

            /* Below params are applicable for only 11az ranging */
            if ((rttParamsV1->type == RTT_TYPE_2_SIDED_11AZ_NTB) ||
                    (rttParamsV1->type == RTT_TYPE_2_SIDED_11AZ_NTB_SECURE)) {
                result = request.put_u32(RTT_ATTRIBUTE_TARGET_NTB_MIN_MEAS_TIME,
                        rttParamsV3->ntb_min_measurement_time);
                if (result < 0) {
                    return result;
                }

                result = request.put_u32(RTT_ATTRIBUTE_TARGET_NTB_MAX_MEAS_TIME,
                        rttParamsV3->ntb_max_measurement_time);
                if (result < 0) {
                    return result;
                }
            }

            /* Below params are applicable for only 11az secure ranging */
            if (rttParamsV1->type == RTT_TYPE_2_SIDED_11AZ_NTB_SECURE) {
                result = request.put_u16(RTT_ATTRIBUTE_TARGET_RTT_AKM,
                        rttParamsV4[i].rtt_secure_config.pasn_config.base_akm);
                if (result < 0) {
                    return result;
                }

                result = request.put_u16(RTT_ATTRIBUTE_TARGET_CIPHER_TYPE,
                        rttParamsV4[i].rtt_secure_config.pasn_config.pairwise_cipher_suite);
                if (result < 0) {
                    return result;
                }

                if (rttParamsV4[i].rtt_secure_config.pasn_config.passphrase_len) {
                    if (rttParamsV4[i].rtt_secure_config.pasn_config.passphrase_len >
                            RTT_SECURITY_MAX_PASSPHRASE_LEN) {
                        ALOGE("%s: Invalid passphrase len = %d\n", __func__,
                                rttParamsV4[i].rtt_secure_config.pasn_config.passphrase_len);
                        return WIFI_ERROR_INVALID_ARGS;
                    }
                    result = request.put_u32(RTT_ATTRIBUTE_TARGET_KEY_PASSPHRASE_LEN,
                            rttParamsV4[i].rtt_secure_config.pasn_config.passphrase_len);
                    if (result < 0) {
                        ALOGE("%s: Failed to fill passphrase len, result = %d\n", __func__, result);
                        return result;
                    }

                    result = request.put(RTT_ATTRIBUTE_TARGET_KEY_PASSPHRASE,
                            (void *)rttParamsV4[i].rtt_secure_config.pasn_config.passphrase,
                             rttParamsV4[i].rtt_secure_config.pasn_config.passphrase_len);
                    if (result < 0) {
                        ALOGE("%s: Failed to fill passphrase, result = %d\n", __func__, result);
                        return result;
                    }
                }

                result = request.put_u8(RTT_ATTRIBUTE_TARGET_SEC_LTF_REQD,
                        rttParamsV4[i].rtt_secure_config.enable_secure_he_ltf);
                if (result < 0) {
                    ALOGE("%s: Failed to fill enab_sec_ltf val, result = %d\n", __func__, result);
                    return result;
                }

                result = request.put_u8(RTT_ATTRIBUTE_TARGET_PROTECTED_FRM_REQD,
                        rttParamsV4[i].rtt_secure_config.enable_ranging_frame_protection);
                if (result < 0) {
                    ALOGE("%s: Failed to fill enab_range_prot val, result= %d\n", __func__, result);
                    return result;
                }
            }

            request.attr_end(attr2);
        }

        request.attr_end(rtt_config);
        request.attr_end(data);
        return WIFI_SUCCESS;
    }

    int createTeardownRequest(WifiRequest& request, unsigned num_devices, mac_addr addr[]) {
        int result = request.create(GOOGLE_OUI, RTT_SUBCMD_CANCEL_CONFIG);
        if (result < 0) {
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u8(RTT_ATTRIBUTE_TARGET_CNT, num_devices);

        if (result < 0) {
            return result;
        }
        for(unsigned i = 0; i < num_devices; i++) {
            result = request.put_addr(RTT_ATTRIBUTE_TARGET_MAC, addr[i]);
            if (result < 0) {
                return result;
            }
        }
        request.attr_end(data);
        return result;
    }

    int start() {
        ALOGD("Setting RTT configuration");
        WifiRequest request(familyId(), ifaceId());
        int result = createSetupRequest(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to create setup request; result = %d", result);
            return result;
        }

        registerVendorHandler(GOOGLE_OUI, RTT_EVENT_COMPLETE);

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            unregisterVendorHandler(GOOGLE_OUI, RTT_EVENT_COMPLETE);
            ALOGE("failed to configure RTT setup; result = %d", result);
            return result;
        }

        ALOGI("Successfully started RTT operation");
        return result;
    }

    virtual int cancel() {
        ALOGD("Stopping RTT");

        WifiRequest request(familyId(), ifaceId());
        int result = createTeardownRequest(request, 0, NULL);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to create stop request; result = %d", result);
        } else {
            result = requestResponse(request);
            if (result != WIFI_SUCCESS) {
                ALOGE("failed to stop scan; result = %d", result);
            }
        }

        unregisterVendorHandler(GOOGLE_OUI, RTT_EVENT_COMPLETE);
        ALOGD("Stopped RTT");
        return WIFI_SUCCESS;
    }

    int cancel_specific(unsigned num_devices, mac_addr addr[]) {
        ALOGE("Stopping RTT");

        WifiRequest request(familyId(), ifaceId());
        int result = createTeardownRequest(request, num_devices, addr);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to create stop request; result = %d", result);
        } else {
            result = requestResponse(request);
            if (result != WIFI_SUCCESS) {
                ALOGE("failed to stop RTT; result = %d", result);
            }
        }

        unregisterVendorHandler(GOOGLE_OUI, RTT_EVENT_COMPLETE);
        return WIFI_SUCCESS;
    }

    virtual int handleResponse(WifiEvent& reply) {
        /* Nothing to do on response! */
        return NL_SKIP;
    }

    virtual int handleEvent(WifiEvent& event) {
        ALOGI("Got an RTT event");
        nlattr *vendor_data = event.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = event.get_vendor_data_len();
        int result_cnt = 0;
        wifi_rtt_result *rtt_result_v1_ptr = NULL;
        wifi_rtt_result_v3 *rtt_result_v3_ptr = NULL;
        wifi_rtt_result_v2 *rtt_result_v2_ptr = NULL;
        wifi_rtt_result_v4 *rtt_result_v4_ptr = NULL;

        if (vendor_data == NULL || len == 0) {
            ALOGI("No rtt results found");
            return NL_STOP;
        }

        for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
            if (it.get_type() == RTT_ATTRIBUTE_RESULTS_COMPLETE) {
                mCompleted = it.get_u32();
                RTT_DEBUG(("Completed flag : %d\n", mCompleted));
            } else if (it.get_type() == RTT_ATTRIBUTE_RESULTS_PER_TARGET) {
                result_cnt = 0;
                mac_addr bssid;
                for (nl_iterator it2(it.get()); it2.has_next(); it2.next()) {
                    if (it2.get_type() == RTT_ATTRIBUTE_TARGET_MAC) {
                        memcpy(bssid, it2.get_data(), sizeof(mac_addr));
                        RTT_DEBUG(("target mac : %02x:%02x:%02x:%02x:%02x:%02x\n",
                                bssid[0], bssid[1], bssid[2], bssid[3],
                                bssid[4], bssid[5]));
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_FREQ) {
                        channel = it2.get_u32();
                        if (rtt_result_v2_ptr) {
                            if (!channel) {
                                rtt_result_v2_ptr->frequency = UNSPECIFIED;
                            } else {
                                rtt_result_v2_ptr->frequency = channel;
                            }
                            RTT_DEBUG(("rtt_resultV2 : \n\tchannel :%d",
                                    rtt_result_v2_ptr->frequency));
                        } else {
                            ALOGE("Not allocated to copy freq, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_BW) {
                        bw = (wifi_rtt_bw)it2.get_u32();
                        if (rtt_result_v2_ptr) {
                            rtt_result_v2_ptr->packet_bw = bw;
                            RTT_DEBUG(("rtt_resultV2 : \n\tpacket_bw :%d",
                                    rtt_result_v2_ptr->packet_bw));
                        } else {
                            ALOGE("Not allocated to copy bw, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_CNT) {
                        result_cnt = it2.get_u32();
                        RTT_DEBUG(("result_cnt : %d\n", result_cnt));
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_I2R_TX_LTF_RPT_CNT) {
                        i2r_tx_ltf_repetition_count = it2.get_u8();
                        RTT_DEBUG(("i2r_tx_ltf_repetition_count: %d\n",
                                i2r_tx_ltf_repetition_count));
                        if (rtt_result_v3_ptr) {
                            rtt_result_v3_ptr->i2r_tx_ltf_repetition_count =
                                    i2r_tx_ltf_repetition_count;
                        } else {
                            ALOGE("Not allocated to copy i2r, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_R2I_TX_LTF_RPT_CNT) {
                        r2i_tx_ltf_repetition_count = it2.get_u8();
                        RTT_DEBUG(("r2i_tx_ltf_repetition_count: %d\n",
                                r2i_tx_ltf_repetition_count));
                        if (rtt_result_v3_ptr) {
                            rtt_result_v3_ptr->r2i_tx_ltf_repetition_count =
                                    r2i_tx_ltf_repetition_count;
                        } else {
                            ALOGE("Not allocated to copy r2i, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_NTB_MIN_MEAS_TIME) {
                        ntb_min_measurement_time = it2.get_u32();
                        RTT_DEBUG(("ntb_min_measurement_time: %d\n", ntb_min_measurement_time));
                        if (rtt_result_v3_ptr) {
                            rtt_result_v3_ptr->ntb_min_measurement_time =
                                    ntb_min_measurement_time;
                        } else {
                            ALOGE("Not allocated to copy min meas time, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_NTB_MAX_MEAS_TIME) {
                        ntb_max_measurement_time = it2.get_u32();
                        RTT_DEBUG(("ntb_max_measurement_time: %d\n", ntb_max_measurement_time));
                        if (rtt_result_v3_ptr) {
                            rtt_result_v3_ptr->ntb_max_measurement_time =
                                    ntb_max_measurement_time;
                        } else {
                            ALOGE("Not allocated to copy max meas time, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_NTB_I2R_STS) {
                        num_tx_sts = it2.get_u8();
                        RTT_DEBUG(("num_tx_sts: %d\n", num_tx_sts));
                        if (rtt_result_v3_ptr) {
                            rtt_result_v3_ptr->num_tx_sts = num_tx_sts;
                        } else {
                            ALOGE("Not allocated to copy i2r sts, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_NTB_R2I_STS) {
                        num_rx_sts = it2.get_u8();
                        RTT_DEBUG(("num_rx_sts: %d\n", num_rx_sts));
                        if (rtt_result_v3_ptr) {
                            rtt_result_v3_ptr->num_rx_sts = num_rx_sts;
                        } else {
                            ALOGE("Not allocated to copy r2i sts, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_RNG_PROT_ENABLED) {
                        is_ranging_protection_enabled = it2.get_u8();
                        RTT_DEBUG(("ranging_enab %d, currentIdx %d\n",
                                is_ranging_protection_enabled, currentIdx));
                        if (rtt_result_v4_ptr) {
                            rtt_result_v4_ptr->is_ranging_protection_enabled =
                                    is_ranging_protection_enabled;
                        } else {
                            ALOGE("Not allocated to copy ranging_enab, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_SLTF_ENABLED) {
                        is_secure_he_ltf_enabled = it2.get_u8();
                        RTT_DEBUG(("ltf_enab %d, currentIdx %d\n",
                              is_secure_he_ltf_enabled, currentIdx));
                        if (rtt_result_v4_ptr) {
                            rtt_result_v4_ptr->is_secure_he_ltf_enabled =
                                    is_secure_he_ltf_enabled;
                        } else {
                            ALOGE("Not allocated to copy sltf enab, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_RTT_AKM) {
                        base_akm = (wifi_rtt_akm)it2.get_u16();
                        RTT_DEBUG(("rtt_resultv4 : \n\t base_akm: %d", base_akm));
                        if (rtt_result_v4_ptr) {
                            rtt_result_v4_ptr->base_akm = base_akm;
                        } else {
                            ALOGE("Not allocated to copy akm, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_CIPHER_TYPE) {
                        cipher_suite = (wifi_rtt_cipher_suite)it2.get_u16();
                        RTT_DEBUG(("rtt_resultv4 : \n\t cipher_suite: %d", cipher_suite));
                        if (rtt_result_v4_ptr) {
                            rtt_result_v4_ptr->cipher_suite = cipher_suite;
                        } else {
                            ALOGE("Not allocated to copy cipher, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT_SLTF_PROTO_VER) {
                        secure_he_ltf_protocol_version = it2.get_u32();
                        RTT_DEBUG(("rtt_resultv4 : \n\t secure_he_ltf_protocol_version: %d",
                                secure_he_ltf_protocol_version));
                        if (rtt_result_v4_ptr) {
                            rtt_result_v4_ptr->secure_he_ltf_protocol_version =
                                    secure_he_ltf_protocol_version;
                        } else {
                            ALOGE("Not allocated to copy secure ltf, currentIdx %d\n", currentIdx);
                            break;
                        }
                    } else if (it2.get_type() == RTT_ATTRIBUTE_RESULT) {
                        u8 *data = nullptr;
                        uint32_t result_len = 0;
                        dot11_rm_ie_t *ele_1 = nullptr;
                        size_t data_size = 0;
                        size_t opt_result_size = 0;
                        int ret;

                        data = (u8 *)it2.get_data();  // Use it2, not it, for current attribute
                        result_len = it2.get_len();

                        if (!data || result_len < RTT_RESULT_V1_SIZE) {
                            ALOGE("%s: null or insufficient data (len=%u)\n", __func__, result_len);
                            ret = WIFI_ERROR_INVALID_ARGS;
                            goto error;
                        }

                        data_size = RTT_RESULT_V1_SIZE;

                        if (result_len == RTT_RESULT_V1_SIZE) {
                            ALOGI("rtt result with no optional data!\n");
                        } else if (result_len > RTT_RESULT_V1_SIZE) {
                            size_t offset = RTT_RESULT_V1_SIZE;
                            while (offset + sizeof(dot11_rm_ie_t) <= result_len) {
                                ele_1 = (dot11_rm_ie_t *)(data + offset);
                                size_t ie_total_len = (size_t)ele_1->len + DOT11_HDR_LEN;

                                if (offset + ie_total_len > result_len) {
                                    ALOGE("%s: Optional element overruns buffer"
                                          " (offset=%zu, ie_len=%zu, result_len=%u)\n",
                                          __func__, offset, ie_total_len, result_len);
                                    ret = WIFI_ERROR_INVALID_ARGS;
                                    goto error;
                                }

                                if (ele_1->id == DOT11_MNG_MEASURE_REPORT_ID) {
                                    if (ele_1->type == DOT11_MEASURE_TYPE_LCI ||
                                        ele_1->type == DOT11_MEASURE_TYPE_CIVICLOC) {
                                        data_size += ie_total_len;
                                        offset += ie_total_len;
                                    } else {
                                        ALOGE("%s: Corrupt data! Unknown measure type %d (len %u)\n",
                                              __func__, ele_1->type, result_len);
                                        ret = WIFI_ERROR_INVALID_ARGS;
                                        goto error;
                                    }
                                } else {
                                    ALOGE("%s: Unknown element ID, stop parsing the opt data\n",
                                          __func__);
                                    break;
                                }
                            }
                        } else {
                            ALOGE("%s: Unexpected data length! Exit. len %u\n", __func__, result_len);
                            ret = WIFI_ERROR_INVALID_ARGS;
                            goto error;
                        }

                        currentIdx = nextidx;
                        rttResultsV1[currentIdx] = (wifi_rtt_result *)malloc(data_size);
                        rtt_result_v1_ptr = rttResultsV1[currentIdx];
                        if (!rtt_result_v1_ptr) {
                            mCompleted = 1;
                            ALOGE("failed to allocate the wifi_result_v1\n");
                            break;
                        }

                        /* Copy the entire RTT result including optional data */
                        memcpy(rtt_result_v1_ptr, data, data_size);

                        /* Handle optional LCI/LCR elements */
                        size_t remaining = data_size - RTT_RESULT_V1_SIZE;
                        if (remaining > 0) {
                            ele_1 = (dot11_rm_ie_t *)((char *)rtt_result_v1_ptr + RTT_RESULT_V1_SIZE);
                            dot11_rm_ie_t *ele_2 = nullptr;

                            if (ele_1->id == DOT11_MNG_MEASURE_REPORT_ID) {
                                size_t ie1_size = ele_1->len + DOT11_HDR_LEN;
                                if (ele_1->type == DOT11_MEASURE_TYPE_LCI) {
                                    rtt_result_v1_ptr->LCI = (wifi_information_element *)ele_1;
                                    remaining -= ie1_size;
                                    opt_result_size += ie1_size;

                                    if (remaining > 0) {
                                        ele_2 = (dot11_rm_ie_t *)((char *)ele_1 + ie1_size);
                                        if ((ele_2->id == DOT11_MNG_MEASURE_REPORT_ID) &&
                                            (ele_2->type == DOT11_MEASURE_TYPE_CIVICLOC)) {
                                            rtt_result_v1_ptr->LCR = (wifi_information_element *)ele_2;
                                            opt_result_size += ele_2->len + DOT11_HDR_LEN;
                                        }
                                    }
                                } else if (ele_1->type == DOT11_MEASURE_TYPE_CIVICLOC) {
                                    rtt_result_v1_ptr->LCR = (wifi_information_element *)ele_1;
                                    remaining -= ie1_size;
                                    opt_result_size += ie1_size;

                                    if (remaining > 0) {
                                        ele_2 = (dot11_rm_ie_t *)((char *)ele_1 + ie1_size);
                                        if ((ele_2->id == DOT11_MNG_MEASURE_REPORT_ID) &&
                                            (ele_2->type == DOT11_MEASURE_TYPE_LCI)) {
                                            rtt_result_v1_ptr->LCI = (wifi_information_element *)ele_2;
                                            opt_result_size += ele_2->len + DOT11_HDR_LEN;
                                        }
                                    }
                                }
                            }
                        }

                        /* Allocate and populate v4 struct including optional data */
                        rttResultsV4[currentIdx] = (wifi_rtt_result_v4 *)malloc(RTT_RESULT_V4_SIZE + opt_result_size);
                        rtt_result_v4_ptr = rttResultsV4[currentIdx];
                        if (!rtt_result_v4_ptr) {
                            ALOGE("failed to allocate the rtt_result v4\n");
                            break;
                        }

                        rtt_result_v3_ptr = &rtt_result_v4_ptr->rtt_result_v3;
                        rtt_result_v2_ptr = &rtt_result_v3_ptr->rtt_result;

                        /* Copy v1 data into v2 */
                        memcpy(&rtt_result_v2_ptr->rtt_result, rtt_result_v1_ptr, RTT_RESULT_V1_SIZE + opt_result_size);

                        if (!channel) {
                            rtt_result_v2_ptr->frequency = UNSPECIFIED;
                        }

                        totalCnt++;
                        nextidx = currentIdx + 1;
                    }
                }
            }
        }

        if (mCompleted) {
            unregisterVendorHandler(GOOGLE_OUI, RTT_EVENT_COMPLETE);
            {
                if (*rttHandlerV4.on_rtt_results_v4) {
                    RTT_DEBUG(("Current Id: %d: retrieved rtt_resultv4 :\n"
                            " burst_num : %d, measurement_number : %d,\n"
                            " success_number : %d, number_per_burst_peer : %d, status : %s,\n"
                            " retry_after_duration : %d type : %d rssi : %d dbm, rssi_spread : %d dbm,\n"
                            " tx_rate : %d Kbps, rx_rate : %d Kbps, rtt : %lu pss, rtt_sd : %lu ps,\n"
                            " rtt_spread : %lu ps, distance : %d mm, distance_sd_mm : %d mm, distance_spread_mm : %d mm,"
                            " timestamp : %lu, burst_duration : %d ms, freq : %d,\n"
                            " packet_bw : %d, negotiated_burst_num : %d\n",
                            currentIdx,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.burst_num,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.measurement_number,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.success_number,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.number_per_burst_peer,
                            get_err_info(rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.status),
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.retry_after_duration,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.type,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.rssi,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.rssi_spread,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.tx_rate.bitrate * 100,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.rx_rate.bitrate * 100,
                            (unsigned long)rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.rtt,
                            (unsigned long)rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.rtt_sd,
                            (unsigned long)rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.rtt_spread,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.distance_mm,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.distance_sd_mm,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.distance_spread_mm,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.ts,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.burst_duration,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.frequency,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.packet_bw,
                            rttResultsV4[currentIdx]->rtt_result_v3.rtt_result.rtt_result.negotiated_burst_num));
                    RTT_DEBUG(("is_ranging_protection_enabled == %d",
                           rttResultsV4[currentIdx]->is_ranging_protection_enabled));
                    RTT_DEBUG(("is_secure_he_ltf_enabled == %d",
                           rttResultsV4[currentIdx]->is_secure_he_ltf_enabled));
                    RTT_DEBUG(("base_akm == %d", rttResultsV4[currentIdx]->base_akm));
                    RTT_DEBUG(("cipher_suite == %d", rttResultsV4[currentIdx]->cipher_suite));
                    (*rttHandlerV4.on_rtt_results_v4)(id(), totalCnt, rttResultsV4);
                }
            }

            for (int i = 0; i < currentIdx; i++) {
                free(rttResultsV1[i]);
                rttResultsV1[i] = NULL;

                free(rttResultsV4[i]);
                rttResultsV4[i] = NULL;
            }
            totalCnt = currentIdx = nextidx = 0;
            WifiCommand *cmd = wifi_unregister_cmd(wifiHandle(), id());
            if (cmd)
                cmd->releaseRef();
        }
error:
        return NL_SKIP;
    }
};

/* API to request RTT measurement */
wifi_error wifi_rtt_range_request_v4(wifi_request_id id, wifi_interface_handle iface,
        unsigned num_rtt_config, wifi_rtt_config_v4 rtt_config[],
        wifi_rtt_event_handler_v4 handler)
{
    if (iface == NULL) {
        ALOGE("wifi_rtt_range_request_v3: NULL iface pointer provided."
                " Exit.");
        return WIFI_ERROR_INVALID_ARGS;
    }

    wifi_handle handle = getWifiHandle(iface);
    if (handle == NULL) {
        ALOGE("wifi_rtt_range_request_v3: NULL handle pointer provided."
            " Exit.");
        return WIFI_ERROR_INVALID_ARGS;
    }

    ALOGI("Rtt range_request; id = %d", id);
    RttCommand *cmd = new RttCommand(iface, id, num_rtt_config, rtt_config, handler);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = wifi_register_cmd(handle, id, cmd);
    if (result != WIFI_SUCCESS) {
        cmd->releaseRef();
        return result;
    }
    result = (wifi_error)cmd->start();
    if (result != WIFI_SUCCESS) {
        wifi_unregister_cmd(handle, id);
        cmd->releaseRef();
        return result;
    }
    return result;
}

/* API to cancel RTT measurements */
wifi_error wifi_rtt_range_cancel(wifi_request_id id,  wifi_interface_handle iface,
        unsigned num_devices, mac_addr addr[])
{
   if (iface == NULL) {
	ALOGE("wifi_rtt_range_cancel: NULL iface pointer provided."
		" Exit.");
	return WIFI_ERROR_INVALID_ARGS;
   }

    wifi_handle handle = getWifiHandle(iface);
    if (handle == NULL) {
	ALOGE("wifi_rtt_range_cancel: NULL handle pointer provided."
		" Exit.");
	return WIFI_ERROR_INVALID_ARGS;
    }

    ALOGI("Rtt range_cancel_request; id = %d", id);
    RttCommand *cmd = new RttCommand(iface, id);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    cmd->cancel_specific(num_devices, addr);
    wifi_unregister_cmd(handle, id);
    cmd->releaseRef();
    return WIFI_SUCCESS;
}

/* API to get RTT capability */
wifi_error wifi_get_rtt_capabilities_v4(wifi_interface_handle iface,
        wifi_rtt_capabilities_v4 *capabilities)
{
    if (iface == NULL) {
        ALOGE("wifi_get_rtt_capabilities_v4: NULL iface pointer provided."
                " Exit.");
        return WIFI_ERROR_INVALID_ARGS;
    }

    if (capabilities == NULL) {
        ALOGE("wifi_get_rtt_capabilities_v4: NULL capabilities pointer provided."
                " Exit.");
        return WIFI_ERROR_INVALID_ARGS;
    }

    GetRttCapabilitiesCommand command(iface, capabilities);
    return (wifi_error) command.requestResponse();
}

/* API to get the responder information */
wifi_error wifi_rtt_get_responder_info(wifi_interface_handle iface,
        wifi_rtt_responder* responderInfo)
{
    if (iface == NULL) {
	ALOGE("wifi_rtt_get_responder_info: NULL iface pointer provided."
		" Exit.");
	return WIFI_ERROR_INVALID_ARGS;
    }

    GetRttResponderInfoCommand command(iface, responderInfo);
    return (wifi_error) command.requestResponse();

}

/**
 * Enable RTT responder mode.
 * channel_hint - hint of the channel information where RTT responder should be enabled on.
 * max_duration_seconds - timeout of responder mode.
 * wifi_rtt_responder - information for RTT responder e.g. channel used and preamble supported.
 */
wifi_error wifi_enable_responder(wifi_request_id id, wifi_interface_handle iface,
                                wifi_channel_info channel_hint, unsigned max_duration_seconds,
                                wifi_rtt_responder* responderInfo)
{
    EnableResponderCommand command(iface, id, channel_hint, max_duration_seconds, responderInfo);
    return (wifi_error) command.requestResponse();
}

/**
 * Disable RTT responder mode.
 */
wifi_error wifi_disable_responder(wifi_request_id id, wifi_interface_handle iface)
{
    CancelResponderCommand command(iface, id);
    return (wifi_error) command.requestResponse();
}

