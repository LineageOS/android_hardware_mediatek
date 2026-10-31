/*
 * Copyright (C) 2017 The Android Open Source Project
 *
 * Portions copyright (C) 2024 Broadcom Limited
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
#include <inttypes.h>
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
#include <netlink/handlers.h>

#include "sync.h"

#define LOG_TAG  "WifiHAL"

#include <utils/Log.h>

#include <hardware_legacy/wifi_hal.h>
#include "common.h"
#include "cpp_bindings.h"

static const char *TwtCmdToString(int cmd);
static void EventGetAttributeData(u8 sub_event_type, nlattr *vendor_data);
static const char *TwtEventToString(int cmd);
int session_id;

#define C2S(x)  case x: return #x;
#define TWT_MAC_INVALID_TRANSID 0xFFFF
#define TWT_CONFIG_ID_AUTO      0xFF

/* Struct for table which has event and cmd type */
typedef struct cmd_type_lookup {
    int event_type;
    int cmd_type;
} cmd_type_lookup_t;

cmd_type_lookup_t cmd_type_lookup_tbl[] = {
    {TWT_SESSION_SETUP_CREATE, TWT_SESSION_SETUP_REQUEST},
    {TWT_SESSION_SETUP_UPDATE, TWT_SESSION_UPDATE_REQUEST},
    {TWT_SESSION_TEARDOWN, TWT_SESSION_TEAR_DOWN_REQUEST},
    {TWT_SESSION_STATS, TWT_SESSION_GET_STATS},
    {TWT_SESSION_SUSPEND, TWT_SESSION_SUSPEND_REQUEST},
    {TWT_SESSION_RESUME, TWT_SESSION_RESUME_REQUEST}
};

typedef struct _twt_hal_info {
    void *twt_handle;
    void *twt_feature_request;
    wifi_request_id request_id;
    TwtRequestType cmd_type;
} twt_hal_info_t;

twt_hal_info_t twt_info;

#define TWT_HANDLE(twt_info)           ((twt_info).twt_handle)
#define GET_TWT_HANDLE(twt_info)       ((TwtHandle *)twt_info.twt_handle)
#define SET_TWT_DATA(id, type)         ((twt_info.cmd_type = type) && (twt_info.request_id = id))

#define WIFI_IS_TWT_REQ_SUPPORT        ((1u << 0u))
#define WIFI_IS_TWT_RESP_SUPPORT       ((1u << 1u))
#define WIFI_IS_TWT_BROADCAST_SUPPORT  ((1u << 2u))
#define WIFI_IS_TWT_FLEX_SUPPORT       ((1u << 3u))
#define WIFI_MIN_WAKE_DUR_MICROS       ((1u << 4u))
#define WIFI_MAX_WAKE_DUR_MICROS       ((1u << 5u))
#define WIFI_MIN_WAKE_INRVL_MICROS     ((1u << 6u))
#define WIFI_MAX_WAKE_iNRVL_MICROS     ((1u << 7u))

/* To be deprecated */
#define WL_TWT_CAP_FLAGS_REQ_SUPPORT    (1u << 0u)
#define WL_TWT_CAP_FLAGS_RESP_SUPPORT   (1u << 1u)
#define WL_TWT_CAP_FLAGS_BTWT_SUPPORT   (1u << 2u)
#define WL_TWT_CAP_FLAGS_FLEX_SUPPORT   (1u << 3u)

class TwtHandle
{
    public:
        wifi_twt_events mEvents;
        TwtHandle(wifi_handle handle, wifi_twt_events events):mEvents(events)
    {}

};

static const char *TwtCmdToString(int cmd)
{
    switch (cmd) {
        C2S(TWT_GET_CAPABILITIES);
        C2S(TWT_SESSION_SETUP_REQUEST);
        C2S(TWT_SESSION_UPDATE_REQUEST);
        C2S(TWT_SESSION_SUSPEND_REQUEST);
        C2S(TWT_SESSION_RESUME_REQUEST);
        C2S(TWT_SESSION_TEAR_DOWN_REQUEST);
        C2S(TWT_SESSION_GET_STATS);
        C2S(TWT_SESSION_CLEAR_STATS);
        default:
            return "UNKNOWN_TWT_CMD";
    }
}

static const char *TwtEventToString(int sub_event_type)
{
    switch (sub_event_type) {
        C2S(TWT_SESSION_FAILURE);
        C2S(TWT_SESSION_SETUP_CREATE);
        C2S(TWT_SESSION_SETUP_UPDATE);
        C2S(TWT_SESSION_TEARDOWN);
        C2S(TWT_SESSION_STATS);
        C2S(TWT_SESSION_SUSPEND);
        C2S(TWT_SESSION_RESUME);
        default:
            return "UNKNOWN_TWT_EVENT";
    }
}

static bool is_twt_sub_event(int sub_event_type)
{
    bool is_twt_event = false;
    switch (sub_event_type) {
        case TWT_SESSION_FAILURE:
        case TWT_SESSION_SETUP_CREATE:
        case TWT_SESSION_SETUP_UPDATE:
        case TWT_SESSION_TEARDOWN:
        case TWT_SESSION_STATS:
        case TWT_SESSION_SUSPEND:
        case TWT_SESSION_RESUME:
            is_twt_event = true;
    }
    return is_twt_event;
}

/* Return cmd type matching the event type */
static int cmd_type_lookup(int event_type) {
    for (u8 i = 0; i < ARRAYSIZE(cmd_type_lookup_tbl); i++) {
        if (event_type == cmd_type_lookup_tbl[i].event_type) {
            return cmd_type_lookup_tbl[i].cmd_type;
        }
    }
    ALOGE("Lookup for cmd type with event_type = %s failed\n",
                TwtEventToString(event_type));
    return -1;
}

void EventGetAttributeData(u8 sub_event_type, nlattr *vendor_data)
{
    u8 attr_type = 0;
    wifi_twt_error_code error_code;
    TwtHandle *twt_handle = GET_TWT_HANDLE(twt_info);
    wifi_request_id RequestId = 0;

    if (!get_halutil_mode()) {
        TwtRequestType cmd_type = (TwtRequestType)cmd_type_lookup(sub_event_type);

        if (twt_handle == NULL) {
            ALOGE("twt callback handle is null, skip processing the event data !!\n");
            goto fail;
        }

        ALOGI("EventGetAttributeData: event: %s, cmd: %s!!\n",
            TwtEventToString(sub_event_type), TwtCmdToString(cmd_type));

        if ((sub_event_type == TWT_SESSION_FAILURE) || (cmd_type == twt_info.cmd_type)) {
            RequestId = twt_info.request_id;
            ALOGE("Retrieved RequestId %d\n", RequestId);
        } else {
            ALOGE("Unexpected event_type %d!!\n", cmd_type);
            goto fail;
        }
    }

    switch (sub_event_type) {
        case TWT_SESSION_FAILURE: {
            for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
                attr_type = it.get_type();
                switch (attr_type) {
                    case TWT_ATTRIBUTE_SUB_EVENT:
                        if (sub_event_type != it.get_u8()) {
                            ALOGE("Non matching attributes: Skip\n");
                            goto fail;
                        }
                        break;
                    case TWT_ATTRIBUTE_ERROR_CODE:
                        error_code = (wifi_twt_error_code)it.get_u8();
                        ALOGD("error code = %u\n", error_code);
                        break;
                    default:
                        ALOGE("Unknown attr_type: %d\n", attr_type);
                        goto fail;
                }
            }

            twt_handle->mEvents.on_twt_failure(RequestId, error_code);
            ALOGI("Notified on_twt_failure: Id %d\n", RequestId);
            break;
        }
        case TWT_SESSION_SETUP_CREATE:
        case TWT_SESSION_SETUP_UPDATE: {
            wifi_twt_session session;

            memset(&session, 0, sizeof(wifi_twt_session));

            for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
                attr_type = it.get_type();
                switch (attr_type) {
                    case TWT_ATTRIBUTE_SUB_EVENT:
                        if (sub_event_type != it.get_u8()) {
                            ALOGE("Non matching attributes: Skip\n");
                            goto fail;
                        }
                        break;
                    case TWT_ATTRIBUTE_SESSION_ID:
                        session.session_id = it.get_u32();
                        ALOGI("session_id = %d\n", session.session_id);
                        break;
                    case TWT_ATTRIBUTE_MLO_LINK_ID:
                        session.mlo_link_id = it.get_u8();
                        ALOGI("mlo_link_id = %d\n", session.mlo_link_id);
                        break;
                    case TWT_ATTRIBUTE_WAKE_DUR_MICROS:
                        session.wake_duration_micros = it.get_u32();
                        ALOGI("wake_duration_micros = %d\n",
                                session.wake_duration_micros);
                        break;
                    case TWT_ATTRIBUTE_WAKE_INTERVAL_MICROS:
                        session.wake_interval_micros = it.get_u32();
                        ALOGI("wake_interval_micros = %" PRIu64 "\n",
                                session.wake_interval_micros);
                        break;
                    case TWT_ATTRIBUTE_NEG_TYPE:
                        session.negotiation_type = (wifi_twt_negotiation_type)it.get_u8();
                        ALOGI("neg type = %u\n", static_cast<uint32_t>(session.negotiation_type));
                        break;
                    case TWT_ATTRIBUTE_IS_TRIGGER_ENABLED:
                        session.is_trigger_enabled = it.get_u8();
                        ALOGI("is_trigger_enabled = %d\n", session.is_trigger_enabled);
                        break;
                    case TWT_ATTRIBUTE_IS_ANNOUNCED:
                        session.is_announced = it.get_u8();
                        ALOGI("is_announced = %d\n", session.is_announced);
                        break;
                    case TWT_ATTRIBUTE_IS_IMPLICIT:
                        session.is_implicit = it.get_u8();
                        ALOGI("is_implicit = %d\n", session.is_implicit);
                        break;
                    case TWT_ATTRIBUTE_IS_PROTECTED:
                        session.is_protected = it.get_u8();
                        ALOGI("is_protected = %d\n", session.is_protected);
                        break;
                    case TWT_ATTRIBUTE_IS_UPDATABLE:
                        session.is_updatable = it.get_u8();
                        ALOGI("is_updatable = %d\n", session.is_updatable);
                        break;
                    case TWT_ATTRIBUTE_IS_SUSPENDABLE:
                        session.is_suspendable = it.get_u8();
                        ALOGI("is_suspendable = %d\n", session.is_suspendable);
                        break;
                    case TWT_ATTRIBUTE_IS_RESP_PM_MODE_ENABLED:
                        session.is_responder_pm_mode_enabled = it.get_u8();
                        ALOGI("is_responder_pm_mode_enabled = %d\n",
                                session.is_responder_pm_mode_enabled);
                        break;
                    default:
                        ALOGE("Unknown attr_type: %d\n", attr_type);
                        goto fail;
                }
            }

            if (session.session_id != TWT_CONFIG_ID_AUTO) {
                if (sub_event_type == TWT_SESSION_SETUP_CREATE) {
                    twt_handle->mEvents.on_twt_session_create(RequestId, session);
                    ALOGI("Notified on_twt_session_create: Id %d\n", RequestId);
                } else if (sub_event_type == TWT_SESSION_SETUP_UPDATE) {
                    twt_handle->mEvents.on_twt_session_update(RequestId, session);
                    ALOGI("Notified on_twt_session_update: Id %d\n", RequestId);
                } else {
                    ALOGE("Unexpected event_type %d!!\n", sub_event_type);
                }
            } else {
                ALOGE("Unexpected session_id!!\n");
            }

            break;
        }

        case TWT_SESSION_SUSPEND:
        case TWT_SESSION_RESUME: {
            for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
                attr_type = it.get_type();
                switch (attr_type) {
                    case TWT_ATTRIBUTE_SUB_EVENT:
                        if (sub_event_type != it.get_u8()) {
                            ALOGE("Non matching attributes: Skip\n");
                            goto fail;
                        }
                        break;
                    case TWT_ATTRIBUTE_SESSION_ID:
                        session_id = it.get_u32();
                        ALOGI("session_id = %d\n", session_id);
                        break;
                    default:
                        ALOGE("Unknown attr_type: %d\n", attr_type);
                        goto fail;
                }
            }

            if (session_id != TWT_CONFIG_ID_AUTO) {
                if (sub_event_type == TWT_SESSION_SUSPEND) {
                    twt_handle->mEvents.on_twt_session_suspend(RequestId, session_id);
                    ALOGI("Notified on_twt_session_suspend: Id %d\n", RequestId);
                } else if (sub_event_type == TWT_SESSION_RESUME) {
                    twt_handle->mEvents.on_twt_session_resume(RequestId, session_id);
                    ALOGI("Notified on_twt_session_resume: Id %d\n", RequestId);
                } else {
                    ALOGE("Unexpected event_type %d!!\n", sub_event_type);
                }
            } else {
                ALOGE("Unexpected session_id!!\n");
            }
            break;
        }
        case TWT_SESSION_TEARDOWN: {
            wifi_twt_teardown_reason_code reason_code;

            for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
                attr_type = it.get_type();
                switch (attr_type) {
                    case TWT_ATTRIBUTE_SUB_EVENT:
                        if (sub_event_type != it.get_u8()) {
                            ALOGE("Non matching attributes: Skip\n");
                            goto fail;
                        }
                        break;
                    case TWT_ATTRIBUTE_SESSION_ID:
                        session_id = it.get_u32();
                        ALOGI("session_id = %d\n", session_id);
                        break;
                    case TWT_ATTRIBUTE_REASON_CODE:
                        reason_code = (wifi_twt_teardown_reason_code)it.get_u8();
                        ALOGI("reason code = %u\n", reason_code);
                        break;
                    default:
                        ALOGE("Unknown attr_type: %d\n", attr_type);
                        goto fail;
                }
            }

            if (session_id != TWT_CONFIG_ID_AUTO) {
                twt_handle->mEvents.on_twt_session_teardown(RequestId,
                        session_id, reason_code);
                ALOGI("Notified on_twt_session_teardown: Id %d\n", RequestId);
            } else {
                ALOGE("Unexpected session_id!!\n");
            }

            break;
        }
        case TWT_SESSION_STATS: {
            wifi_twt_session_stats stats;

            memset(&stats, 0, sizeof(wifi_twt_session_stats));

            for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
                attr_type = it.get_type();
                switch (attr_type) {
                    case TWT_ATTRIBUTE_SUB_EVENT:
                        if (sub_event_type != it.get_u8()) {
                            ALOGE("Non matching attributes: Skip\n");
                            goto fail;
                        }
                        break;
                    case TWT_ATTRIBUTE_SESSION_ID:
                        session_id = it.get_u32();
                        ALOGI("session_id = %d\n", session_id);
                        break;
                    case TWT_ATTRIBUTE_AVG_PKT_NUM_TX:
                        stats.avg_pkt_num_tx = it.get_u32();
                        ALOGI("avg_pkt_num_tx = %u\n", stats.avg_pkt_num_tx);
                        break;
                    case TWT_ATTRIBUTE_AVG_PKT_NUM_RX:
                        stats.avg_pkt_num_rx = it.get_u32();
                        ALOGI("avg_pkt_num_rx = %u\n", stats.avg_pkt_num_rx);
                        break;
                    case TWT_ATTRIBUTE_AVG_TX_PKT_SIZE:
                        stats.avg_tx_pkt_size = it.get_u32();
                        ALOGI("avg_tx_pkt_size = %u\n", stats.avg_tx_pkt_size);
                        break;
                    case TWT_ATTRIBUTE_AVG_RX_PKT_SIZE:
                        stats.avg_rx_pkt_size = it.get_u32();
                        ALOGI("avg_rx_pkt_size = %u\n", stats.avg_rx_pkt_size);
                        break;
                    case TWT_ATTRIBUTE_AVG_EOSP_DUR_US:
                        stats.avg_eosp_dur_us = it.get_u32();
                        ALOGI("avg_eosp_dur_us = %u\n", stats.avg_eosp_dur_us);
                        break;
                    case TWT_ATTRIBUTE_EOSP_COUNT:
                        stats.eosp_count = it.get_u32();
                        ALOGI("eosp_count = %u\n", stats.eosp_count);
                        break;
                    default:
                        ALOGE("Unknown attr_type: %d\n", attr_type);
                        goto fail;
                }
            }

            if (session_id != TWT_CONFIG_ID_AUTO) {
                twt_handle->mEvents.on_twt_session_stats(RequestId,
                        session_id, stats);
                ALOGI("Notified on_twt_session_stats: Id %d\n", RequestId);
            } else {
                ALOGE("Unexpected session_id!!\n");
            }

            break;
        }
        default:
            ALOGE("Unknown event_type: %d\n", sub_event_type);
            break;
    }

    fail:
        return;
}

void HandleTwtEvent(nlattr *vendor_data) {
    u8 sub_event_type = 0;
    u8 event_type = 0;

    for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
        event_type = it.get_type();
        if (event_type == TWT_ATTRIBUTE_SUB_EVENT) {
            sub_event_type = it.get_u8();
            ALOGI("%s: Event %s: (%d)\n",
                    __func__, TwtEventToString(sub_event_type), sub_event_type);
            if (is_twt_sub_event(sub_event_type)) {
                EventGetAttributeData(sub_event_type, vendor_data);
            }
        }
    }
    return;
}

class TwtEventCap : public WifiCommand
{
    transaction_id mId;
public:
    TwtEventCap(wifi_interface_handle iface, int id)
        : WifiCommand("TwtCommand", iface, id)
    {
        mId = id;
    }

    int start()
    {
        registerTwtVendorEvents();
        return WIFI_SUCCESS;
    }

    int handleResponse(WifiEvent& reply) {
        return NL_SKIP;
    }

    void registerTwtVendorEvents()
    {
        registerVendorHandler(BRCM_OUI, BRCM_VENDOR_EVENT_TWT);
    }

    void unregisterTwtVendorEvents()
    {
        unregisterVendorHandler(BRCM_OUI, BRCM_VENDOR_EVENT_TWT);
    }

    int handleEvent(WifiEvent& event) {
        u16 attr_type;
        wifi_twt_error_code error_code;
        u8 sub_event_type = 0;
        TwtEventType twt_event;
        int session_id = 0;

        ALOGI("In TwtEventCap::handleEvent\n");

        nlattr *vendor_data = event.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = event.get_vendor_data_len();
        int event_id = event.get_vendor_subcmd();

        if (!vendor_data || len == 0) {
            ALOGE("No event data found");
            return NL_SKIP;
        }

        switch (event_id) {
            case BRCM_VENDOR_EVENT_TWT: {
                HandleTwtEvent(vendor_data);
                break;
            }
            default:
                break;
        }
        return NL_SKIP;
    }
};

/* To see event prints in console */
wifi_error twt_event_check_request(int id, wifi_interface_handle iface)
{
    TwtEventCap *cmd = new TwtEventCap(iface, id);
    if (cmd == NULL) {
        return WIFI_ERROR_NOT_SUPPORTED;
    }
    return (wifi_error)cmd->start();
}

static void twt_parse_cap_report(nlattr *vendor_data, wifi_twt_capabilities *mCapabilities)
{
    for (nl_iterator it2(vendor_data); it2.has_next(); it2.next()) {
        if (it2.get_type() == TWT_ATTRIBUTE_IS_REQUESTOR_SUPPORTED) {
            mCapabilities->is_twt_requester_supported = it2.get_u8();
        } else if (it2.get_type() == TWT_ATTRIBUTE_IS_RESPONDER_SUPPORTED) {
            mCapabilities->is_twt_responder_supported = it2.get_u8();
        } else if (it2.get_type() == TWT_ATTRIBUTE_IS_BROADCAST_SUPPORTED) {
            mCapabilities->is_broadcast_twt_supported = it2.get_u8();
        } else if (it2.get_type() == TWT_ATTRIBUTE_IS_FLEXIBLE_SUPPORTED) {
            mCapabilities->is_flexible_twt_supported = it2.get_u8();
        } else if (it2.get_type() == TWT_ATTRIBUTE_MIN_WAKE_DURATION_US) {
            mCapabilities->min_wake_duration_micros = it2.get_u32();
        } else if (it2.get_type() == TWT_ATTRIBUTE_MAX_WAKE_DURATION_US) {
            mCapabilities->max_wake_duration_micros = it2.get_u32();
        } else if (it2.get_type() == TWT_ATTRIBUTE_MIN_WAKE_INTERVAL_US) {
            mCapabilities->min_wake_interval_micros = it2.get_u32();
        } else if (it2.get_type() == TWT_ATTRIBUTE_MAX_WAKE_INTERVAL_US) {
            mCapabilities->max_wake_interval_micros = it2.get_u32();
        } else {
             ALOGW("Ignoring invalid attribute type = %d, size = %d",
                     it2.get_type(), it2.get_len());
        }
    }
    return;
}
////////////////////////////////////////////////////////////////////////////////
class TwtFeatureRequest : public WifiCommand
{
    wifi_twt_request *reqContext;
    TwtRequestType mType;
    wifi_request_id mId = 0;
    int mSessionId;
    wifi_twt_capabilities *mCapabilities;

public:
    /* Constructor for register event callback */
    TwtFeatureRequest(wifi_interface_handle iface, wifi_request_id id,
        TwtRequestType cmdType)
        : WifiCommand("TwtFeatureRequest", iface, id),
        mType(cmdType)
    {
    }

    TwtFeatureRequest(wifi_interface_handle iface, wifi_twt_capabilities *capabilities,
        TwtRequestType cmdType)
        : WifiCommand("TwtFeatureRequest", iface, 0), mCapabilities(capabilities),
        mType(cmdType)
    {
        memset(mCapabilities, 0, sizeof(*mCapabilities));
    }

    /* Constructor for session_setup */
    TwtFeatureRequest(wifi_interface_handle iface, wifi_request_id id,
        wifi_twt_request *params, TwtRequestType cmdType)
        : WifiCommand("TwtFeatureRequest", iface, id),
        reqContext(params), mType(cmdType)
    {
        setId(id);
    }

    /* Constructor for session_update */
    TwtFeatureRequest(wifi_interface_handle iface, wifi_request_id id,
        int session_id, wifi_twt_request *params, TwtRequestType cmdType)
        : WifiCommand("TwtFeatureRequest", iface, id),
        mSessionId(session_id), reqContext(params), mType(cmdType)
    {
        setId(id);
        mSessionId = session_id;
    }

    /* Constructor for session suspend, resume, teardown, get_stats, clear_stats */
    TwtFeatureRequest(wifi_interface_handle iface, wifi_request_id id,
        int session_id, TwtRequestType cmdType)
        : WifiCommand("TwtFeatureRequest", iface, id),
        mSessionId(session_id), mType(cmdType)
    {
        setId(id);
        mSessionId = session_id;
    }

    ~TwtFeatureRequest() {
        ALOGE("TwtFeatureRequest destroyed\n");
    }

    void setId(transaction_id id) {
        if (id != TWT_MAC_INVALID_TRANSID) {
            mId = id;
        }
    }

    transaction_id getId() {
        return mId;
    }

    void setType(TwtRequestType type ) {
        mType = type;
    }

    int getSessionId() {
        return mSessionId;
    }

    int createRequest(WifiRequest& request)
    {
        ALOGI("TWT CMD: %s, Id %d\n", TwtCmdToString(mType), mId);
        if (mType == TWT_GET_CAPABILITIES) {
            return TwtSessionGetCap(request);
        } else if (mType == TWT_SESSION_SETUP_REQUEST) {
            return TwtSessionSetup(request, (wifi_twt_request *)reqContext);
        } else if (mType == TWT_SESSION_UPDATE_REQUEST) {
            return TwtSessionUpdate(request, mSessionId, (wifi_twt_request *)reqContext);
        } else if (mType == TWT_SESSION_SUSPEND_REQUEST) {
            return TwtSessionSuspend(request, mSessionId);
        } else if (mType == TWT_SESSION_RESUME_REQUEST) {
            return TwtSessionResume(request, mSessionId);
        } else if (mType == TWT_SESSION_TEAR_DOWN_REQUEST) {
            return TwtSessionTearDown(request, mSessionId);
        } else if (mType == TWT_SESSION_GET_STATS) {
            return TwtSessionGetStats(request, mSessionId);
        } else if (mType == TWT_SESSION_CLEAR_STATS) {
            return TwtSessionClearStats(request, mSessionId);
        } else {
            ALOGE("%s: Unknown TWT request: %d\n", __func__, mType);
            return WIFI_ERROR_UNKNOWN;
        }

        return WIFI_SUCCESS;
    }

    int TwtSessionGetCap(WifiRequest& request) {
        ALOGD("Creating message to get twt capabilities; iface\n");

        int result = request.create(GOOGLE_OUI, TWT_SUBCMD_GETCAPABILITY);
        if (result < 0) {
            ALOGE("Failed to send the twt cap cmd, err = %d\n", result);
        } else {
            ALOGD("Success to send twt cap cmd, err = %d\n", result);
        }
        return result;
    }

    int TwtSessionSetup(WifiRequest& request, wifi_twt_request *mParams)
    {
        int result = request.create(GOOGLE_OUI, TWT_SUBCMD_SESSION_SETUP_REQUEST);
        if (result < 0) {
            ALOGE("%s Failed to create request, result = %d\n", __func__, result);
            return result;
        }

        /* If handle is 0xFFFF, then update instance_id in response of this request
         * otherwise, update not needed
         */
        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        if (mParams->mlo_link_id) {
            result = request.put_s8(TWT_ATTRIBUTE_MLO_LINK_ID, mParams->mlo_link_id);
            if (result < 0) {
                ALOGE("%s: Failed to fill mlo link id = %d, result = %d\n",
                        __func__, mParams->mlo_link_id, result);
                return result;
            }
        }

        if (mParams->min_wake_duration_micros) {
            result = request.put_u32(TWT_ATTRIBUTE_MIN_WAKE_DURATION_US,
                    mParams->min_wake_duration_micros);
            if (result < 0) {
                ALOGE("%s: Failed to fill min_wake_duration_micros = %d, result = %d\n",
                        __func__, mParams->min_wake_duration_micros, result);
                return result;
            }
        }

        if (mParams->max_wake_duration_micros) {
            result = request.put_u32(TWT_ATTRIBUTE_MAX_WAKE_DURATION_US,
                    mParams->max_wake_duration_micros);
            if (result < 0) {
                ALOGE("%s: Failed to fill max_wake_duration_micros = %d, result = %d\n",
                        __func__, mParams->max_wake_duration_micros, result);
                return result;
            }
         }

         if (mParams->min_wake_interval_micros) {
            result = request.put_u32(TWT_ATTRIBUTE_MIN_WAKE_INTERVAL_US,
                    mParams->min_wake_interval_micros);
            if (result < 0) {
                ALOGE("%s: Failed to fill min_wake_interval_micros = %" PRIu64 ", result = %d\n",
                        __func__, mParams->min_wake_interval_micros, result);
                return result;
            }
        }

        if (mParams->max_wake_interval_micros) {
            result = request.put_u32(TWT_ATTRIBUTE_MAX_WAKE_INTERVAL_US,
                    mParams->max_wake_interval_micros);
            if (result < 0) {
                ALOGE("%s: Failed to fill max_wake_interval_micros = %" PRIu64 ", result = %d\n",
                        __func__, mParams->max_wake_interval_micros, result);
                return result;
            }
        }

        request.attr_end(data);

        ALOGI("Returning successfully\n");
        return result;
    }

    int TwtSessionUpdate(WifiRequest& request, int mSessionId, wifi_twt_request *mParams)
    {
        int result = request.create(GOOGLE_OUI, TWT_SUBCMD_SESSION_UPDATE_REQUEST);
        if (result < 0) {
            ALOGE("%s: Failed to create twt_update request, result = %d\n",
                    __func__, result);
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(TWT_ATTRIBUTE_SESSION_ID, mSessionId);
        if (result < 0) {
            ALOGE("%s: Failed to fill mSessionId = %d, result = %d\n",
                    __func__, mSessionId, result);
            return result;
        }

        if (mParams->mlo_link_id) {
            result = request.put_s8(TWT_ATTRIBUTE_MLO_LINK_ID, mParams->mlo_link_id);
            if (result < 0) {
                ALOGE("%s: Failed to fill mlo link id = %d, result = %d\n",
                        __func__, mParams->mlo_link_id, result);
                return result;
           }
        }

        if (mParams->min_wake_duration_micros) {
            result = request.put_u32(TWT_ATTRIBUTE_MIN_WAKE_DURATION_US,
                    mParams->min_wake_duration_micros);
            if (result < 0) {
                ALOGE("%s: Failed to fill min_wake_duration_micros = %d, result = %d\n",
                        __func__, mParams->min_wake_duration_micros, result);
                return result;
            }
        }

        if (mParams->max_wake_duration_micros) {
            result = request.put_u32(TWT_ATTRIBUTE_MAX_WAKE_DURATION_US,
                    mParams->max_wake_duration_micros);
            if (result < 0) {
                ALOGE("%s: Failed to fill max_wake_duration_micros = %d, result = %d\n",
                        __func__, mParams->max_wake_duration_micros, result);
                return result;
            }
        }

        if (mParams->min_wake_interval_micros) {
            result = request.put_u32(TWT_ATTRIBUTE_MIN_WAKE_INTERVAL_US,
                    mParams->min_wake_interval_micros);
            if (result < 0) {
                ALOGE("%s: Failed to fill min_wake_interval_micros = %" PRIu64 ", result = %d\n",
                        __func__, mParams->min_wake_interval_micros, result);
                return result;
            }
        }

        if (mParams->max_wake_interval_micros) {
            result = request.put_u32(TWT_ATTRIBUTE_MAX_WAKE_INTERVAL_US,
                    mParams->max_wake_interval_micros);
            if (result < 0) {
                ALOGE("%s: Failed to fill max_wake_interval_micros = %" PRIu64 ", result = %d\n",
                        __func__, mParams->max_wake_interval_micros, result);
                return result;
            }
        }

        request.attr_end(data);

        ALOGI("TwtSessionUpdate: Returning successfully\n");

        return WIFI_SUCCESS;
    }

    int TwtSessionTearDown(WifiRequest& request, int mSessionId)
    {
        int result = request.create(GOOGLE_OUI, TWT_SUBCMD_SESSION_TEAR_DOWN_REQUEST);
        if (result < 0) {
            ALOGE("%s: Failed to create request, result = %d\n", __func__, result);
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(TWT_ATTRIBUTE_SESSION_ID, mSessionId);
        if (result < 0) {
            ALOGE("%s: Failed to fill mSessionId = %d, result = %d\n",
                    __func__, mSessionId, result);
            return result;
        }
        request.attr_end(data);
        return WIFI_SUCCESS;
    }

    int TwtSessionSuspend(WifiRequest& request, int mSessionId)
    {
        int result = request.create(GOOGLE_OUI, TWT_SUBCMD_SESSION_SUSPEND_REQUEST);
        if (result < 0) {
            ALOGE("%s: Failed to create session suspend request, result = %d\n",
                    __func__, result);
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(TWT_ATTRIBUTE_SESSION_ID, mSessionId);
        if (result < 0) {
            ALOGE("%s: Failed to fill mSessionId = %d, result = %d\n",
                    __func__, mSessionId, result);
            return result;
        }
        request.attr_end(data);
        return WIFI_SUCCESS;
    }

    int TwtSessionResume(WifiRequest& request, int mSessionId)
    {
        int result = request.create(GOOGLE_OUI, TWT_SUBCMD_SESSION_RESUME_REQUEST);
        if (result < 0) {
            ALOGE("%s: Failed to create session resume request, result = %d\n",
                    __func__, result);
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(TWT_ATTRIBUTE_SESSION_ID, mSessionId);
        if (result < 0) {
            ALOGE("%s: Failed to fill mSessionId = %d, result = %d\n",
                    __func__, mSessionId, result);
            return result;
        }

        request.attr_end(data);
        return WIFI_SUCCESS;
    }

    int TwtSessionGetStats(WifiRequest& request, int mSessionId)
    {
        int result = request.create(GOOGLE_OUI, TWT_SUBCMD_SESSION_GETSTATS);
        if (result < 0) {
            ALOGE("%s: Failed to create session get stats request, result = %d\n",
                    __func__, result);
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(TWT_ATTRIBUTE_SESSION_ID, mSessionId);
        if (result < 0) {
            ALOGE("%s: Failed to fill mSessionId = %d, result = %d\n",
                    __func__, mSessionId, result);
            return result;
        }
        request.attr_end(data);
        return WIFI_SUCCESS;
    }

    int TwtSessionClearStats(WifiRequest& request, int mSessionId)
    {
        int result = request.create(GOOGLE_OUI, TWT_SUBCMD_SESSION_CLR_STATS);
        if (result < 0) {
            ALOGE("%s: Failed to create session clear stats request, result = %d\n",
                    __func__, result);
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(TWT_ATTRIBUTE_SESSION_ID, mSessionId);
        if (result < 0) {
            ALOGE("%s: Failed to fill mSessionId = %d, result = %d\n",
                    __func__, mSessionId, result);
            return result;
        }

        request.attr_end(data);
        return WIFI_SUCCESS;
    }

    int open()
    {
        int result = 0;
        WifiRequest request(familyId(), ifaceId());
        result = createRequest(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("%s: failed to create setup request; result = %d", __func__, result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("%s: failed to configure setup; result = %d", __func__, result);
            return result;
        }

        request.destroy();
        return WIFI_SUCCESS;
    }

    void registerTwtVendorEvents()
    {
        registerVendorHandler(BRCM_OUI, BRCM_VENDOR_EVENT_TWT);
    }

    void unregisterTwtVendorEvents()
    {
        unregisterVendorHandler(BRCM_OUI, BRCM_VENDOR_EVENT_TWT);
    }

protected:
    virtual int handleResponse(WifiEvent& reply) {

        ALOGI("In TwtFeatureRequest::handleResponse\n");

        wifi_error ret = WIFI_SUCCESS;

        if (reply.get_cmd() != NL80211_CMD_VENDOR || reply.get_vendor_data() == NULL) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = reply.get_vendor_data_len();

        if (vendor_data == NULL || len == 0) {
            ALOGE("no vendor data in twt cmd response; ignoring it");
            return NL_SKIP;
        }

        for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
            if (it.get_type() == TWT_ATTRIBUTE_WIFI_ERROR) {
                ret = (wifi_error)it.get_s8();
            } else if ((mType == TWT_GET_CAPABILITIES) && (it.get_type() == TWT_ATTRIBUTE_CAP)) {
                twt_parse_cap_report(it.get(), mCapabilities);
            } else {
                ALOGW("Ignoring invalid attribute type = %d, size = %d",
                        it.get_type(), it.get_len());
            }
        }

        return NL_SKIP;
    }

    int handleEvent(WifiEvent& event) {
        u16 attr_type;
        u8 sub_event_type = 0;
        TwtEventType twt_event;
        wifi_twt_error_code error_code;
        int session_id = 0;

        ALOGI("In TwtFeatureRequest::handleEvent\n");

        nlattr *vendor_data = event.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = event.get_vendor_data_len();
        int event_id = event.get_vendor_subcmd();

        if (!vendor_data || len == 0) {
            ALOGE("No event data found");
            return NL_SKIP;
        }

        switch (event_id) {
            case BRCM_VENDOR_EVENT_TWT: {
                HandleTwtEvent(vendor_data);
                break;
            }
            default:
                ALOGE("Unknown event: %d\n", event_id);
                break;
        }
        return NL_SKIP;
    }
};

void twt_deinit_handler()
{
    if (twt_info.twt_feature_request) {
        /* register for Twt vendor events with info mac class*/
        TwtFeatureRequest *cmd_event =
                (TwtFeatureRequest*)(twt_info.twt_feature_request);
        cmd_event->unregisterTwtVendorEvents();
        delete (TwtFeatureRequest*)twt_info.twt_feature_request;
        twt_info.twt_feature_request = NULL;
    }
    if (TWT_HANDLE(twt_info)) {
        delete GET_TWT_HANDLE(twt_info);
        TWT_HANDLE(twt_info) = NULL;
    }
    ALOGI("wifi twt internal clean up done");
    return;
}

wifi_error wifi_twt_register_events(wifi_interface_handle iface,
        wifi_twt_events handlers)
{
    wifi_handle handle = getWifiHandle(iface);
    if (TWT_HANDLE(twt_info)) {
        /* cleanup and re-register */
        twt_deinit_handler();
    }
    memset(&twt_info, 0, sizeof(twt_info));
    TWT_HANDLE(twt_info) = new TwtHandle(handle, handlers);
    twt_info.twt_feature_request =
            (void*)new TwtFeatureRequest(iface, 0, TWT_LAST);
    NULL_CHECK_RETURN(twt_info.twt_feature_request,
            "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    TwtFeatureRequest *cmd_event = (TwtFeatureRequest*)(twt_info.twt_feature_request);
    cmd_event->registerTwtVendorEvents();
    return WIFI_SUCCESS;
}

/* API to get TWT capability */
wifi_error wifi_twt_get_capabilities(wifi_interface_handle iface,
        wifi_twt_capabilities* capabilities)
{
    wifi_error ret = WIFI_SUCCESS;
    TwtFeatureRequest *cmd;
    TwtRequestType cmdType = TWT_GET_CAPABILITIES;
    memset(capabilities, 0, sizeof(wifi_twt_capabilities));

    if (iface == NULL) {
        ALOGE("wifi_twt_get_capability: NULL iface pointer provided."
                " Exit.");
        return WIFI_ERROR_INVALID_ARGS;
    }

    if (capabilities == NULL) {
        ALOGE("wifi_twt_get_capability: NULL capabilities pointer provided."
                " Exit.");
        return WIFI_ERROR_INVALID_ARGS;
    }

    cmd = new TwtFeatureRequest(iface, capabilities, cmdType);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);

    ret = (wifi_error)cmd->open();
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s : failed in create twt_cap req, error = %d\n", __func__, ret);
        ret = WIFI_ERROR_NOT_SUPPORTED;
    }
    cmd->releaseRef();
    return ret;
}

wifi_error wifi_twt_session_setup(wifi_request_id id, wifi_interface_handle iface,
        wifi_twt_request request)
{
    wifi_error ret = WIFI_SUCCESS;
    TwtFeatureRequest *cmd;
    TwtRequestType cmdType = TWT_SESSION_SETUP_REQUEST;

    SET_TWT_DATA(id, cmdType);

    cmd = new TwtFeatureRequest(iface, id, &request, cmdType);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);

    cmd->setId(id);
    ret = (wifi_error)cmd->open();
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s : failed in create twt_setup req, error = %d\n", __func__, ret);
    }
    cmd->releaseRef();
    return ret;
}

wifi_error wifi_twt_session_update(wifi_request_id id, wifi_interface_handle iface,
        int session_id, wifi_twt_request request)
{
    wifi_error ret = WIFI_SUCCESS;
    TwtFeatureRequest *cmd;
    TwtRequestType cmdType = TWT_SESSION_UPDATE_REQUEST;

    SET_TWT_DATA(id, cmdType);

    cmd = new TwtFeatureRequest(iface, id, session_id, &request, cmdType);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);

    cmd->setId(id);
    ret = (wifi_error)cmd->open();
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s : failed in create twt_update req, error = %d\n", __func__, ret);
    }
    cmd->releaseRef();
    return ret;
}

wifi_error wifi_twt_session_suspend(wifi_request_id id, wifi_interface_handle iface,
        int session_id)
{
    wifi_error ret = WIFI_SUCCESS;
    TwtFeatureRequest *cmd;
    TwtRequestType cmdType = TWT_SESSION_SUSPEND_REQUEST;

    SET_TWT_DATA(id, cmdType);

    cmd = new TwtFeatureRequest(iface, id, session_id, cmdType);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);

    cmd->setId(id);
    ret = (wifi_error)cmd->open();
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s : failed in create twt_suspend req, error = %d\n", __func__, ret);
    }
    cmd->releaseRef();
    return ret;
}

wifi_error wifi_twt_session_resume(wifi_request_id id, wifi_interface_handle iface,
        int session_id)
{
    wifi_error ret = WIFI_SUCCESS;
    TwtFeatureRequest *cmd;
    TwtRequestType cmdType = TWT_SESSION_RESUME_REQUEST;

    SET_TWT_DATA(id, cmdType);

    cmd = new TwtFeatureRequest(iface, id, session_id, cmdType);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);

    cmd->setId(id);
    ret = (wifi_error)cmd->open();
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s : failed in create twt_resume req, error = %d\n", __func__, ret);
    }
    cmd->releaseRef();
    return ret;
}

wifi_error wifi_twt_session_teardown(wifi_request_id id, wifi_interface_handle iface,
        int session_id)
{
    wifi_error ret = WIFI_SUCCESS;
    TwtFeatureRequest *cmd;
    TwtRequestType cmdType = TWT_SESSION_TEAR_DOWN_REQUEST;

    SET_TWT_DATA(id, cmdType);

    cmd = new TwtFeatureRequest(iface, id, session_id, cmdType);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);

    cmd->setId(id);
    ret = (wifi_error)cmd->open();
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s : failed in create twt_teardown req, error = %d\n", __func__, ret);
    }
    cmd->releaseRef();
    return ret;
}

wifi_error wifi_twt_session_get_stats(wifi_request_id id, wifi_interface_handle iface,
        int session_id)
{
    wifi_error ret = WIFI_SUCCESS;
    TwtFeatureRequest *cmd;
    TwtRequestType cmdType = TWT_SESSION_GET_STATS;

    SET_TWT_DATA(id, cmdType);

    cmd = new TwtFeatureRequest(iface, id, session_id, cmdType);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);

    cmd->setId(id);
    ret = (wifi_error)cmd->open();
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s : failed to create twt_get_stats req, error = %d\n", __func__, ret);
    }
    cmd->releaseRef();
    return ret;
}

