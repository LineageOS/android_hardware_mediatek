/*
 * Copyright (C) 2017 The Android Open Source Project
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

#define LOG_TAG "WifiHAL"

#include "nan.h"
#include <errno.h>
#include <log/log.h>
#include "sync.h"

/* Debug dump of NAN request/response blobs (verbose level only). */
static void hexdump(const void* buf, uint16_t len) {
    const uint8_t* bytes = (const uint8_t*)buf;
    char line[3 * 16 + 1];

    for (uint16_t off = 0; off < len; off += 16) {
        int n = 0;
        for (uint16_t i = off; i < len && i < off + 16; i++) {
            n += snprintf(line + n, sizeof(line) - n, "%02x ", bytes[i]);
        }
        ALOGV("HEXDUMP %p+%04x | %s", buf, off, line);
    }
}

#ifdef __GNUC__
#define PRINTF_FORMAT(a, b) __attribute__((format(printf, (a), (b))))
#define STRUCT_PACKED __attribute__((packed))
#else
#define PRINTF_FORMAT(a, b)
#define STRUCT_PACKED
#endif

#define OUT_OF_BAND_SERVICE_INSTANCE_ID 0

// Singleton Static Instance
NanCommand* NanCommand::mNanCommandInstance = NULL;

// Implementation of the functions exposed in nan.h
wifi_error nan_register_handler(wifi_interface_handle iface, NanCallbackHandler handlers) {
    // Obtain the singleton instance
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    wifi_handle wifiHandle = getWifiHandle(iface);

    nanCommand = NanCommand::instance(wifiHandle);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->setCallbackHandler(handlers);
    return ret;
}

wifi_error nan_get_version(wifi_handle handle, NanVersion* version) {
    *version = (NAN_MAJOR_VERSION << 16 | NAN_MINOR_VERSION << 8 | NAN_MICRO_VERSION);
    return WIFI_SUCCESS;
}

/*  Function to send enable request to the wifi driver.*/
wifi_error nan_enable_request(transaction_id id, wifi_interface_handle iface,
                              NanEnableRequest* msg) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    /* validate parameters */
    if (msg->support_2dot4g_val == 0) {
        /* Must support 2.4G */
        return WIFI_ERROR_NOT_SUPPORTED;
    }

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanEnable(id, msg);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanEnable Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

/*  Function to send disable request to the wifi driver.*/
wifi_error nan_disable_request(transaction_id id, wifi_interface_handle iface) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanDisable(id);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanDisable Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

/*  Function to send publish request to the wifi driver.*/
wifi_error nan_publish_request(transaction_id id, wifi_interface_handle iface,
                               NanPublishRequest* msg) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanPublish(id, msg);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanPublish Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

/*  Function to send publish cancel to the wifi driver.*/
wifi_error nan_publish_cancel_request(transaction_id id, wifi_interface_handle iface,
                                      NanPublishCancelRequest* msg) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanPublishCancel(id, msg);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanPublishCancel Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

/*  Function to send Subscribe request to the wifi driver.*/
wifi_error nan_subscribe_request(transaction_id id, wifi_interface_handle iface,
                                 NanSubscribeRequest* msg) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanSubscribe(id, msg);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanSubscribe Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

/*  Function to cancel subscribe to the wifi driver.*/
wifi_error nan_subscribe_cancel_request(transaction_id id, wifi_interface_handle iface,
                                        NanSubscribeCancelRequest* msg) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanSubscribeCancel(id, msg);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanSubscribeCancel Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

/*  Function to send NAN follow up request to the wifi driver.*/
wifi_error nan_transmit_followup_request(transaction_id id, wifi_interface_handle iface,
                                         NanTransmitFollowupRequest* msg) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanTransmitFollowup(id, msg);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanTransmitFollowup Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

/*  Function to send NAN statistics request to the wifi driver.*/
wifi_error nan_stats_request(transaction_id id, wifi_interface_handle iface, NanStatsRequest* msg) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanStats(id, msg);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanStats Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

/*  Function to send NAN configuration request to the wifi driver.*/
wifi_error nan_config_request(transaction_id id, wifi_interface_handle iface,
                              NanConfigRequest* msg) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    /* validate parameters */
    if ((msg->config_rssi_proximity != 1 /* sigma */) && (msg->sid_beacon != 1 /* FWK */)) {
        return WIFI_ERROR_NOT_SUPPORTED;
    }

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanConfig(id, msg);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanConfig Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

/*  Function to send NAN request to the wifi driver.*/
wifi_error nan_tca_request(transaction_id id, wifi_interface_handle iface, NanTCARequest* msg) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanTCA(id, msg);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanTCA Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

/*  Function to send NAN Beacon sdf payload to the wifi driver.
    This instructs the Discovery Engine to begin publishing the
    received payload in any Beacon or Service Discovery Frame
    transmitted*/
wifi_error nan_beacon_sdf_payload_request(transaction_id id, wifi_interface_handle iface,
                                          NanBeaconSdfPayloadRequest* msg) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanBeaconSdfPayload(id, msg);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanBeaconSdfPayload Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

wifi_error nan_get_sta_parameter(transaction_id id, wifi_interface_handle iface,
                                 NanStaParameter* msg) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    wifi_handle wifiHandle = getWifiHandle(iface);

    nanCommand = NanCommand::instance(wifiHandle);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->getNanStaParameter(iface, msg);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: getNanStaParameter Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

cleanup:
    return ret;
}

/*  Function to get NAN capabilities */
wifi_error nan_get_capabilities(transaction_id id, wifi_interface_handle iface) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanCapabilities(id);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanCapabilities Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

/*  Function to get NAN capabilities */
wifi_error nan_debug_command_config(transaction_id id, wifi_interface_handle iface,
                                    NanDebugParams debug, int debug_msg_length) {
    wifi_error ret;
    NanCommand* nanCommand = NULL;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    if (debug_msg_length <= 0) {
        ALOGE("%s: Invalid debug message length = %d", __FUNCTION__, debug_msg_length);
        return WIFI_ERROR_UNKNOWN;
    }

    nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NAN);
    if (nanCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nanCommand->mapKernelErrortoWifiHalError(nanCommand->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = nanCommand->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = nanCommand->putNanDebugCommand(debug, debug_msg_length);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanDebugCommand Error:%d", __FUNCTION__, ret);
        goto cleanup;
    }

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

wifi_error nan_initialize_vendor_cmd(wifi_interface_handle iface, NanCommand** nanCommand) {
    wifi_error ret;
    interface_info* ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    if (nanCommand == NULL) {
        ALOGE("%s: Error nanCommand NULL", __FUNCTION__);
        return WIFI_ERROR_INVALID_ARGS;
    }

    *nanCommand = new NanCommand(wifiHandle, 0, MTK_OUI, MTK_SUBCMD_NDP);
    if (*nanCommand == NULL) {
        ALOGE("%s: Object creation failed", __FUNCTION__);
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    /* Create the message */
    ret = (*nanCommand)->mapKernelErrortoWifiHalError((*nanCommand)->create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    ret = (*nanCommand)->set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    return WIFI_SUCCESS;

cleanup:
    delete *nanCommand;
    return ret;
}

wifi_error nan_data_interface_create(transaction_id id, wifi_interface_handle iface,
                                     char* iface_name) {
    ALOGV("NAN_DP_INTERFACE_CREATE");
    wifi_error ret;
    struct nlattr* nlData;
    NanCommand* nanCommand = NULL;

    if (iface_name == NULL) {
        ALOGE("%s: Invalid Nan Data Interface Name. \n", __FUNCTION__);
        return WIFI_ERROR_INVALID_ARGS;
    }

    ret = nan_initialize_vendor_cmd(iface, &nanCommand);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: Initialization failed", __FUNCTION__);
        return ret;
    }

    /* Add the vendor specific attributes for the NL command. */
    nlData = nanCommand->attr_start(NL80211_ATTR_VENDOR_DATA);
    if (!nlData) goto cleanup;

    if (nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
                            MTK_WLAN_VENDOR_ATTR_NDP_INTERFACE_CREATE) ||
        nanCommand->put_u16(MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID, id) ||
        nanCommand->put_string(MTK_WLAN_VENDOR_ATTR_NDP_IFACE_STR, iface_name)) {
        goto cleanup;
    }

    nanCommand->attr_end(nlData);

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

wifi_error nan_data_interface_delete(transaction_id id, wifi_interface_handle iface,
                                     char* iface_name) {
    ALOGV("NAN_DP_INTERFACE_DELETE");
    wifi_error ret;
    struct nlattr* nlData;
    NanCommand* nanCommand = NULL;

    if (iface_name == NULL) {
        ALOGE("%s: Invalid Nan Data Interface Name. \n", __FUNCTION__);
        return WIFI_ERROR_INVALID_ARGS;
    }
    ret = nan_initialize_vendor_cmd(iface, &nanCommand);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: Initialization failed", __FUNCTION__);
        return ret;
    }

    /* Add the vendor specific attributes for the NL command. */
    nlData = nanCommand->attr_start(NL80211_ATTR_VENDOR_DATA);
    if (!nlData) goto cleanup;

    if (nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
                            MTK_WLAN_VENDOR_ATTR_NDP_INTERFACE_DELETE) ||
        nanCommand->put_u16(MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID, id) ||
        nanCommand->put_string(MTK_WLAN_VENDOR_ATTR_NDP_IFACE_STR, iface_name)) {
        goto cleanup;
    }

    nanCommand->attr_end(nlData);

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

wifi_error nan_data_request_initiator(transaction_id id, wifi_interface_handle iface,
                                      NanDataPathInitiatorRequest* msg) {
    ALOGV("NAN_DP_REQUEST_INITIATOR");
    wifi_error ret;
    struct nlattr *nlData, *nlCfgSecurity, *nlCfgQos;
    NanCommand* nanCommand = NULL;

    if (msg == NULL) return WIFI_ERROR_INVALID_ARGS;

    ret = nan_initialize_vendor_cmd(iface, &nanCommand);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: Initialization failed", __FUNCTION__);
        return ret;
    }

    if ((msg->cipher_type != NAN_CIPHER_SUITE_SHARED_KEY_NONE) &&
        (msg->key_info.body.pmk_info.pmk_len == 0) &&
        (msg->key_info.body.passphrase_info.passphrase_len == 0)) {
        ALOGE("%s: Failed-Initiator req, missing pmk and passphrase", __FUNCTION__);
        return WIFI_ERROR_INVALID_ARGS;
    }

    if ((msg->cipher_type != NAN_CIPHER_SUITE_SHARED_KEY_NONE) &&
        (msg->requestor_instance_id == OUT_OF_BAND_SERVICE_INSTANCE_ID) &&
        (msg->service_name_len == 0)) {
        ALOGE("%s: Failed-Initiator req, missing service name for out of band request",
              __FUNCTION__);
        return WIFI_ERROR_INVALID_ARGS;
    }

    /* Add the vendor specific attributes for the NL command. */
    nlData = nanCommand->attr_start(NL80211_ATTR_VENDOR_DATA);
    if (!nlData) goto cleanup;

    if (nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
                            MTK_WLAN_VENDOR_ATTR_NDP_INITIATOR_REQUEST) ||
        nanCommand->put_u16(MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID, id) ||
        nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_INSTANCE_ID,
                            msg->requestor_instance_id) ||
        nanCommand->put_bytes(MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR,
                              (char*)msg->peer_disc_mac_addr, NAN_MAC_ADDR_LEN) ||
        nanCommand->put_string(MTK_WLAN_VENDOR_ATTR_NDP_IFACE_STR, msg->ndp_iface)) {
        goto cleanup;
    }

    if (msg->channel_request_type != NAN_DP_CHANNEL_NOT_REQUESTED) {
        if (nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_CONFIG,
                                msg->channel_request_type) ||
            nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL, msg->channel))
            goto cleanup;
    }

    if (msg->app_info.ndp_app_info_len != 0) {
        if (nanCommand->put_bytes(MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO,
                                  (char*)msg->app_info.ndp_app_info,
                                  msg->app_info.ndp_app_info_len)) {
            goto cleanup;
        }
    }
    if (msg->ndp_cfg.security_cfg == NAN_DP_CONFIG_SECURITY) {
        nlCfgSecurity = nanCommand->attr_start(MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_SECURITY);
        if (!nlCfgSecurity) goto cleanup;

        if (nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_SECURITY_TYPE, 0)) {
            goto cleanup;
        }
        nanCommand->attr_end(nlCfgSecurity);
    }
    if (msg->ndp_cfg.qos_cfg == NAN_DP_CONFIG_QOS) {
        nlCfgQos = nanCommand->attr_start(MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_QOS);
        if (!nlCfgQos) goto cleanup;
        /* TBD Qos Info */
        nanCommand->attr_end(nlCfgQos);
    }
    if (msg->cipher_type != NAN_CIPHER_SUITE_SHARED_KEY_NONE) {
        if (nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_CSID, msg->cipher_type)) goto cleanup;
    }
    if (msg->key_info.key_type == NAN_SECURITY_KEY_INPUT_PMK &&
        msg->key_info.body.pmk_info.pmk_len == NAN_PMK_INFO_LEN) {
        if (nanCommand->put_bytes(MTK_WLAN_VENDOR_ATTR_NDP_PMK,
                                  (char*)msg->key_info.body.pmk_info.pmk,
                                  msg->key_info.body.pmk_info.pmk_len))
            goto cleanup;
    } else if (msg->key_info.key_type == NAN_SECURITY_KEY_INPUT_PASSPHRASE &&
               msg->key_info.body.passphrase_info.passphrase_len >=
                       NAN_SECURITY_MIN_PASSPHRASE_LEN &&
               msg->key_info.body.passphrase_info.passphrase_len <=
                       NAN_SECURITY_MAX_PASSPHRASE_LEN) {
        if (nanCommand->put_bytes(MTK_WLAN_VENDOR_ATTR_NDP_PASSPHRASE,
                                  (char*)msg->key_info.body.passphrase_info.passphrase,
                                  msg->key_info.body.passphrase_info.passphrase_len))
            goto cleanup;
    }
    if (msg->service_name_len) {
        if (nanCommand->put_bytes(MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_NAME, (char*)msg->service_name,
                                  msg->service_name_len))
            goto cleanup;
    }
    nanCommand->attr_end(nlData);

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

wifi_error nan_data_indication_response(transaction_id id, wifi_interface_handle iface,
                                        NanDataPathIndicationResponse* msg) {
    ALOGV("NAN_DP_INDICATION_RESPONSE");
    wifi_error ret;
    struct nlattr *nlData, *nlCfgSecurity, *nlCfgQos;
    NanCommand* nanCommand = NULL;

    if (msg == NULL) return WIFI_ERROR_INVALID_ARGS;

    ret = nan_initialize_vendor_cmd(iface, &nanCommand);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: Initialization failed", __FUNCTION__);
        return ret;
    }

    if ((msg->cipher_type != NAN_CIPHER_SUITE_SHARED_KEY_NONE) &&
        (msg->key_info.body.pmk_info.pmk_len == 0) &&
        (msg->key_info.body.passphrase_info.passphrase_len == 0)) {
        ALOGE("%s: Failed-Initiator req, missing pmk and passphrase", __FUNCTION__);
        return WIFI_ERROR_INVALID_ARGS;
    }

    /* Add the vendor specific attributes for the NL command. */
    nlData = nanCommand->attr_start(NL80211_ATTR_VENDOR_DATA);
    if (!nlData) goto cleanup;

    if (nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
                            MTK_WLAN_VENDOR_ATTR_NDP_RESPONDER_REQUEST) ||
        nanCommand->put_u16(MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID, id) ||
        nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID, msg->ndp_instance_id) ||
        nanCommand->put_string(MTK_WLAN_VENDOR_ATTR_NDP_IFACE_STR, msg->ndp_iface) ||
        nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_RESPONSE_CODE, msg->rsp_code)) {
        goto cleanup;
    }
    if (msg->app_info.ndp_app_info_len != 0) {
        if (nanCommand->put_bytes(MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO,
                                  (char*)msg->app_info.ndp_app_info,
                                  msg->app_info.ndp_app_info_len)) {
            goto cleanup;
        }
    }
    if (msg->ndp_cfg.security_cfg == NAN_DP_CONFIG_SECURITY) {
        nlCfgSecurity = nanCommand->attr_start(MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_SECURITY);
        if (!nlCfgSecurity) goto cleanup;
        /* Setting value to 0 for now */
        if (nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_SECURITY_TYPE, 0)) {
            goto cleanup;
        }
        nanCommand->attr_end(nlCfgSecurity);
    }
    if (msg->ndp_cfg.qos_cfg == NAN_DP_CONFIG_QOS) {
        nlCfgQos = nanCommand->attr_start(MTK_WLAN_VENDOR_ATTR_NDP_CONFIG_QOS);
        if (!nlCfgQos) goto cleanup;

        /* TBD Qos Info */
        nanCommand->attr_end(nlCfgQos);
    }
    if (msg->cipher_type != NAN_CIPHER_SUITE_SHARED_KEY_NONE) {
        if (nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_CSID, msg->cipher_type)) goto cleanup;
    }
    if (msg->key_info.key_type == NAN_SECURITY_KEY_INPUT_PMK &&
        msg->key_info.body.pmk_info.pmk_len == NAN_PMK_INFO_LEN) {
        if (nanCommand->put_bytes(MTK_WLAN_VENDOR_ATTR_NDP_PMK,
                                  (char*)msg->key_info.body.pmk_info.pmk,
                                  msg->key_info.body.pmk_info.pmk_len))
            goto cleanup;
    } else if (msg->key_info.key_type == NAN_SECURITY_KEY_INPUT_PASSPHRASE &&
               msg->key_info.body.passphrase_info.passphrase_len >=
                       NAN_SECURITY_MIN_PASSPHRASE_LEN &&
               msg->key_info.body.passphrase_info.passphrase_len <=
                       NAN_SECURITY_MAX_PASSPHRASE_LEN) {
        if (nanCommand->put_bytes(MTK_WLAN_VENDOR_ATTR_NDP_PASSPHRASE,
                                  (char*)msg->key_info.body.passphrase_info.passphrase,
                                  msg->key_info.body.passphrase_info.passphrase_len))
            goto cleanup;
    }

    if (msg->service_name_len) {
        if (nanCommand->put_bytes(MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_NAME, (char*)msg->service_name,
                                  msg->service_name_len))
            goto cleanup;
    }
    nanCommand->attr_end(nlData);

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

wifi_error nan_data_end(transaction_id id, wifi_interface_handle iface,
                        NanDataPathEndRequest* msg) {
    wifi_error ret;
    ALOGV("NAN_DP_END");
    struct nlattr* nlData;
    NanCommand* nanCommand = NULL;

    if (msg == NULL) return WIFI_ERROR_INVALID_ARGS;

    ret = nan_initialize_vendor_cmd(iface, &nanCommand);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: Initialization failed", __FUNCTION__);
        return ret;
    }

    /* Add the vendor specific attributes for the NL command. */
    nlData = nanCommand->attr_start(NL80211_ATTR_VENDOR_DATA);
    if (!nlData) goto cleanup;

    if (nanCommand->put_u32(MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD,
                            MTK_WLAN_VENDOR_ATTR_NDP_END_REQUEST) ||
        nanCommand->put_u16(MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID, id) ||
        nanCommand->put_bytes(MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY,
                              (char*)msg->ndp_instance_id, msg->num_ndp_instances * sizeof(u32))) {
        goto cleanup;
    }
    nanCommand->attr_end(nlData);

    ret = nanCommand->requestEvent();
    if (ret != WIFI_SUCCESS) ALOGE("%s: requestEvent Error:%d", __FUNCTION__, ret);

cleanup:
    delete nanCommand;
    return ret;
}

// Implementation related to nan class common functions
// Constructor
// Making the constructor private since this class is a singleton
NanCommand::NanCommand(wifi_handle handle, int id, u32 vendor_id, u32 subcmd)
    : WifiVendorCommand(handle, id, vendor_id, subcmd) {
    memset(&mHandler, 0, sizeof(mHandler));
    mNanVendorEvent = NULL;
    mNanDataLen = 0;
    mStaParam = NULL;
}

NanCommand* NanCommand::instance(wifi_handle handle) {
    if (handle == NULL) {
        ALOGE("Handle is invalid");
        return NULL;
    }
    if (mNanCommandInstance == NULL) {
        mNanCommandInstance = new NanCommand(handle, 0, MTK_OUI, MTK_SUBCMD_NAN);
        ALOGV("NanCommand %p created", mNanCommandInstance);
        return mNanCommandInstance;
    } else {
        if (handle != getWifiHandle(mNanCommandInstance->mInfo)) {
            /* upper layer must have cleaned up the handle and reinitialized,
               so we need to update the same */
            ALOGI("Handle different, update the handle");
            mNanCommandInstance->mInfo = (hal_info*)handle;
        }
    }
    ALOGV("NanCommand %p created already", mNanCommandInstance);
    return mNanCommandInstance;
}

void NanCommand::cleanup() {
    // free the VendorData
    if (mVendorData) {
        free(mVendorData);
    }
    mVendorData = NULL;
    // cleanup the mMsg
    mMsg.destroy();
}

void NanCommand::destroyInstance() {
    NanCommand* cmd = mNanCommandInstance;

    if (cmd == NULL) {
        return;
    }
    mNanCommandInstance = NULL;
    cmd->unregisterVendorHandler(cmd->mVendor_id, MTK_SUBCMD_NAN);
    cmd->unregisterVendorHandler(cmd->mVendor_id, MTK_SUBCMD_NDP);
    cmd->cleanup();
    /* the event dispatcher may still hold a reference; the last release deletes the command */
    cmd->releaseRef();
}

NanCommand::~NanCommand() {
    ALOGV("NanCommand %p destroyed", this);
}

int NanCommand::handleResponse(WifiEvent& reply) {
    return NL_SKIP;
}

wifi_error NanCommand::setCallbackHandler(NanCallbackHandler nHandler) {
    wifi_error res;
    mHandler = nHandler;
    res = mapKernelErrortoWifiHalError(registerVendorHandler(mVendor_id, MTK_SUBCMD_NAN));
    if (res != WIFI_SUCCESS) {
        // error case should not happen print log
        ALOGE("%s: Unable to register Vendor Handler Vendor Id=0x%x"
              "subcmd=MTK_SUBCMD_NAN",
              __FUNCTION__, mVendor_id);
        return res;
    }

    res = mapKernelErrortoWifiHalError(registerVendorHandler(mVendor_id, MTK_SUBCMD_NDP));
    if (res != WIFI_SUCCESS) {
        // error case should not happen print log
        ALOGE("%s: Unable to register Vendor Handler Vendor Id=0x%x"
              "subcmd=MTK_SUBCMD_NDP",
              __FUNCTION__, mVendor_id);
        return res;
    }
    return res;
}

/* This function implements creation of Vendor command */
int NanCommand::create() {
    int ret = mMsg.create(NL80211_CMD_VENDOR);
    if (ret != WIFI_SUCCESS) goto out;

    /* Insert the oui in the msg */
    ret = mMsg.put_u32(NL80211_ATTR_VENDOR_ID, mVendor_id);
    if (ret != WIFI_SUCCESS) goto out;
    /* Insert the subcmd in the msg */
    ret = mMsg.put_u32(NL80211_ATTR_VENDOR_SUBCMD, mSubcmd);

out:
    if (ret != WIFI_SUCCESS) mMsg.destroy();
    return ret;
}

// This function will be the main handler for incoming event
// MTK_SUBCMD_NAN
// Call the appropriate callback handler after parsing the vendor data.
int NanCommand::handleEvent(WifiEvent& event) {
    WifiVendorCommand::handleEvent(event);
    ALOGV("%s: Subcmd=%u Vendor data len received:%d", __FUNCTION__, mSubcmd, mDataLen);
    hexdump(mVendorData, mDataLen);

    if (mSubcmd == MTK_SUBCMD_NAN) {
        // Parse the vendordata and get the NAN attribute
        struct nlattr* tb_vendor[MTK_WLAN_VENDOR_ATTR_MAX + 1];
        nla_parse(tb_vendor, MTK_WLAN_VENDOR_ATTR_MAX, (struct nlattr*)mVendorData, mDataLen, NULL);
        // Populating the mNanVendorEvent and mNanDataLen to point to NAN data.
        mNanVendorEvent = (char*)nla_data(tb_vendor[MTK_WLAN_VENDOR_ATTR_NAN]);
        mNanDataLen = nla_len(tb_vendor[MTK_WLAN_VENDOR_ATTR_NAN]);

        if (isNanResponse()) {
            // handleNanResponse will parse the data and call
            // the response callback handler with the populated
            // NanResponseMsg
            handleNanResponse();
        } else {
            // handleNanIndication will parse the data and call
            // the corresponding Indication callback handler
            // with the corresponding populated Indication event
            handleNanIndication();
        }
    } else if (mSubcmd == MTK_SUBCMD_NDP) {
        // Parse the vendordata and get the NAN attribute
        u32 ndpCmdType;
        struct nlattr* tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_MAX + 1];
        nla_parse(tb_vendor, MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_MAX, (struct nlattr*)mVendorData,
                  mDataLen, NULL);

        if (tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD]) {
            ndpCmdType = nla_get_u32(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_SUBCMD]);
            ALOGV("%s: NDP Cmd Type : val 0x%x", __FUNCTION__, ndpCmdType);
            switch (ndpCmdType) {
                case MTK_WLAN_VENDOR_ATTR_NDP_INTERFACE_CREATE:
                    handleNdpResponse(NAN_DP_INTERFACE_CREATE, tb_vendor);
                    break;
                case MTK_WLAN_VENDOR_ATTR_NDP_INTERFACE_DELETE:
                    handleNdpResponse(NAN_DP_INTERFACE_DELETE, tb_vendor);
                    break;
                case MTK_WLAN_VENDOR_ATTR_NDP_INITIATOR_RESPONSE:
                    handleNdpResponse(NAN_DP_INITIATOR_RESPONSE, tb_vendor);
                    break;
                case MTK_WLAN_VENDOR_ATTR_NDP_RESPONDER_RESPONSE:
                    handleNdpResponse(NAN_DP_RESPONDER_RESPONSE, tb_vendor);
                    break;
                case MTK_WLAN_VENDOR_ATTR_NDP_END_RESPONSE:
                    handleNdpResponse(NAN_DP_END, tb_vendor);
                    break;
                case MTK_WLAN_VENDOR_ATTR_NDP_REQUEST_IND:
                case MTK_WLAN_VENDOR_ATTR_NDP_CONFIRM_IND:
                case MTK_WLAN_VENDOR_ATTR_NDP_END_IND:
                case MTK_WLAN_VENDOR_ATTR_NDP_SCHEDULE_UPDATE_IND:
                    handleNdpIndication(ndpCmdType, tb_vendor);
                    break;
                default:
                    ALOGE("%s: Invalid NDP subcmd response received %d", __FUNCTION__, ndpCmdType);
            }
        }
    } else {
        // error case should not happen print log
        ALOGE("%s: Wrong NAN subcmd received %d", __FUNCTION__, mSubcmd);
    }
    mNanVendorEvent = NULL;
    return NL_SKIP;
}

/*Helper function to Write and Read TLV called in indication as well as request */
u16 NANTLV_WriteTlv(pNanTlv pInTlv, u8* pOutTlv) {
    u16 writeLen = 0;
    u16 i;

    if (!pInTlv) {
        ALOGE("NULL pInTlv");
        return writeLen;
    }

    if (!pOutTlv) {
        ALOGE("NULL pOutTlv");
        return writeLen;
    }

    *pOutTlv++ = pInTlv->type & 0xFF;
    *pOutTlv++ = (pInTlv->type & 0xFF00) >> 8;
    writeLen += 2;

    ALOGV("WRITE TLV type %u, writeLen %u", pInTlv->type, writeLen);

    *pOutTlv++ = pInTlv->length & 0xFF;
    *pOutTlv++ = (pInTlv->length & 0xFF00) >> 8;
    writeLen += 2;

    ALOGV("WRITE TLV length %u, writeLen %u", pInTlv->length, writeLen);

    for (i = 0; i < pInTlv->length; ++i) {
        *pOutTlv++ = pInTlv->value[i];
    }

    writeLen += pInTlv->length;
    ALOGV("WRITE TLV value, writeLen %u", writeLen);
    return writeLen;
}

u16 NANTLV_ReadTlv(u8* pInTlv, pNanTlv pOutTlv) {
    u16 readLen = 0;

    if (!pInTlv) {
        ALOGE("NULL pInTlv");
        return readLen;
    }

    if (!pOutTlv) {
        ALOGE("NULL pOutTlv");
        return readLen;
    }

    pOutTlv->type = *pInTlv++;
    pOutTlv->type |= *pInTlv++ << 8;
    readLen += 2;

    ALOGV("READ TLV type %u, readLen %u", pOutTlv->type, readLen);

    pOutTlv->length = *pInTlv++;
    pOutTlv->length |= *pInTlv++ << 8;
    readLen += 2;

    ALOGV("READ TLV length %u, readLen %u", pOutTlv->length, readLen);

    if (pOutTlv->length) {
        pOutTlv->value = pInTlv;
        readLen += pOutTlv->length;
    } else {
        pOutTlv->value = NULL;
    }

    ALOGV("READ TLV  readLen %u", readLen);
    return readLen;
}

u8* addTlv(u16 type, u16 length, const u8* value, u8* pOutTlv) {
    NanTlv nanTlv;
    u16 len;

    nanTlv.type = type;
    nanTlv.length = length;
    nanTlv.value = (u8*)value;

    len = NANTLV_WriteTlv(&nanTlv, pOutTlv);
    return (pOutTlv + len);
}

wifi_error NanCommand::putNanEnable(transaction_id id, const NanEnableRequest* pReq) {
    wifi_error ret;
    ALOGV("NAN_ENABLE");
    size_t message_len = NAN_MAX_ENABLE_REQ_SIZE;

    if (pReq == NULL) {
        cleanup();
        return WIFI_ERROR_INVALID_ARGS;
    }

    message_len +=
            (pReq->config_support_5g ? (SIZEOF_TLV_HDR + sizeof(pReq->support_5g_val)) : 0) +
            (pReq->config_sid_beacon ? (SIZEOF_TLV_HDR + sizeof(pReq->sid_beacon_val)) : 0) +
            (pReq->config_2dot4g_rssi_close ? (SIZEOF_TLV_HDR + sizeof(pReq->rssi_close_2dot4g_val))
                                            : 0) +
            (pReq->config_2dot4g_rssi_middle
                     ? (SIZEOF_TLV_HDR + sizeof(pReq->rssi_middle_2dot4g_val))
                     : 0) +
            (pReq->config_hop_count_limit ? (SIZEOF_TLV_HDR + sizeof(pReq->hop_count_limit_val))
                                          : 0) +
            (pReq->config_2dot4g_support ? (SIZEOF_TLV_HDR + sizeof(pReq->support_2dot4g_val))
                                         : 0) +
            (pReq->config_2dot4g_beacons ? (SIZEOF_TLV_HDR + sizeof(pReq->beacon_2dot4g_val)) : 0) +
            (pReq->config_2dot4g_sdf ? (SIZEOF_TLV_HDR + sizeof(pReq->sdf_2dot4g_val)) : 0) +
            (pReq->config_5g_beacons ? (SIZEOF_TLV_HDR + sizeof(pReq->beacon_5g_val)) : 0) +
            (pReq->config_5g_sdf ? (SIZEOF_TLV_HDR + sizeof(pReq->sdf_5g_val)) : 0) +
            (pReq->config_5g_rssi_close ? (SIZEOF_TLV_HDR + sizeof(pReq->rssi_close_5g_val)) : 0) +
            (pReq->config_5g_rssi_middle ? (SIZEOF_TLV_HDR + sizeof(pReq->rssi_middle_5g_val))
                                         : 0) +
            (pReq->config_2dot4g_rssi_proximity
                     ? (SIZEOF_TLV_HDR + sizeof(pReq->rssi_proximity_2dot4g_val))
                     : 0) +
            (pReq->config_5g_rssi_close_proximity
                     ? (SIZEOF_TLV_HDR + sizeof(pReq->rssi_close_proximity_5g_val))
                     : 0) +
            (pReq->config_rssi_window_size ? (SIZEOF_TLV_HDR + sizeof(pReq->rssi_window_size_val))
                                           : 0) +
            (pReq->config_oui ? (SIZEOF_TLV_HDR + sizeof(pReq->oui_val)) : 0) +
            (pReq->config_intf_addr ? (SIZEOF_TLV_HDR + sizeof(pReq->intf_addr_val)) : 0) +
            (pReq->config_cluster_attribute_val
                     ? (SIZEOF_TLV_HDR + sizeof(pReq->config_cluster_attribute_val))
                     : 0) +
            (pReq->config_scan_params ? NAN_MAX_SOCIAL_CHANNELS * (SIZEOF_TLV_HDR + sizeof(u32))
                                      : 0) +
            (pReq->config_random_factor_force
                     ? (SIZEOF_TLV_HDR + sizeof(pReq->random_factor_force_val))
                     : 0) +
            (pReq->config_hop_count_force ? (SIZEOF_TLV_HDR + sizeof(pReq->hop_count_force_val))
                                          : 0) +
            (pReq->config_24g_channel ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_5g_channel ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_dw.config_2dot4g_dw_band ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_dw.config_5g_dw_band ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_disc_mac_addr_randomization ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (
                    /* Always include cfg discovery indication TLV */
                    SIZEOF_TLV_HDR + sizeof(u32)) +
            (pReq->config_subscribe_sid_beacon
                     ? (SIZEOF_TLV_HDR + sizeof(pReq->subscribe_sid_beacon_val))
                     : 0) +
            (pReq->config_discovery_beacon_int ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_nss ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_enable_ranging ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_dw_early_termination ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_ndpe_attr ? (SIZEOF_TLV_HDR + sizeof(NanDevCapAttrCap)) : 0);

    pNanEnableReqMsg pFwReq = (pNanEnableReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ALOGV("Message Len %zu", message_len);
    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_ENABLE_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    pFwReq->fwHeader.transactionId = id;

    u8* tlvs = pFwReq->ptlv;

    /* Write the TLVs to the message. */

    tlvs = addTlv(NAN_TLV_TYPE_CLUSTER_ID_LOW, sizeof(pReq->cluster_low),
                  (const u8*)&pReq->cluster_low, tlvs);
    tlvs = addTlv(NAN_TLV_TYPE_CLUSTER_ID_HIGH, sizeof(pReq->cluster_high),
                  (const u8*)&pReq->cluster_high, tlvs);
    tlvs = addTlv(NAN_TLV_TYPE_MASTER_PREFERENCE, sizeof(pReq->master_pref),
                  (const u8*)&pReq->master_pref, tlvs);
    if (pReq->config_support_5g) {
        tlvs = addTlv(NAN_TLV_TYPE_5G_SUPPORT, sizeof(pReq->support_5g_val),
                      (const u8*)&pReq->support_5g_val, tlvs);
    }
    if (pReq->config_sid_beacon) {
        tlvs = addTlv(NAN_TLV_TYPE_SID_BEACON, sizeof(pReq->sid_beacon_val),
                      (const u8*)&pReq->sid_beacon_val, tlvs);
    }
    if (pReq->config_2dot4g_rssi_close) {
        tlvs = addTlv(NAN_TLV_TYPE_24G_RSSI_CLOSE, sizeof(pReq->rssi_close_2dot4g_val),
                      (const u8*)&pReq->rssi_close_2dot4g_val, tlvs);
    }
    if (pReq->config_2dot4g_rssi_middle) {
        tlvs = addTlv(NAN_TLV_TYPE_24G_RSSI_MIDDLE, sizeof(pReq->rssi_middle_2dot4g_val),
                      (const u8*)&pReq->rssi_middle_2dot4g_val, tlvs);
    }
    if (pReq->config_hop_count_limit) {
        tlvs = addTlv(NAN_TLV_TYPE_HOP_COUNT_LIMIT, sizeof(pReq->hop_count_limit_val),
                      (const u8*)&pReq->hop_count_limit_val, tlvs);
    }
    if (pReq->config_2dot4g_support) {
        tlvs = addTlv(NAN_TLV_TYPE_24G_SUPPORT, sizeof(pReq->support_2dot4g_val),
                      (const u8*)&pReq->support_2dot4g_val, tlvs);
    }
    if (pReq->config_2dot4g_beacons) {
        tlvs = addTlv(NAN_TLV_TYPE_24G_BEACON, sizeof(pReq->beacon_2dot4g_val),
                      (const u8*)&pReq->beacon_2dot4g_val, tlvs);
    }
    if (pReq->config_2dot4g_sdf) {
        tlvs = addTlv(NAN_TLV_TYPE_24G_SDF, sizeof(pReq->sdf_2dot4g_val),
                      (const u8*)&pReq->sdf_2dot4g_val, tlvs);
    }
    if (pReq->config_5g_beacons) {
        tlvs = addTlv(NAN_TLV_TYPE_5G_BEACON, sizeof(pReq->beacon_5g_val),
                      (const u8*)&pReq->beacon_5g_val, tlvs);
    }
    if (pReq->config_5g_sdf) {
        tlvs = addTlv(NAN_TLV_TYPE_5G_SDF, sizeof(pReq->sdf_5g_val), (const u8*)&pReq->sdf_5g_val,
                      tlvs);
    }
    if (pReq->config_2dot4g_rssi_proximity) {
        tlvs = addTlv(NAN_TLV_TYPE_24G_RSSI_CLOSE_PROXIMITY,
                      sizeof(pReq->rssi_proximity_2dot4g_val),
                      (const u8*)&pReq->rssi_proximity_2dot4g_val, tlvs);
    }
    /* Add the support of sending 5G RSSI values */
    if (pReq->config_5g_rssi_close) {
        tlvs = addTlv(NAN_TLV_TYPE_5G_RSSI_CLOSE, sizeof(pReq->rssi_close_5g_val),
                      (const u8*)&pReq->rssi_close_5g_val, tlvs);
    }
    if (pReq->config_5g_rssi_middle) {
        tlvs = addTlv(NAN_TLV_TYPE_5G_RSSI_MIDDLE, sizeof(pReq->rssi_middle_5g_val),
                      (const u8*)&pReq->rssi_middle_5g_val, tlvs);
    }
    if (pReq->config_5g_rssi_close_proximity) {
        tlvs = addTlv(NAN_TLV_TYPE_5G_RSSI_CLOSE_PROXIMITY,
                      sizeof(pReq->rssi_close_proximity_5g_val),
                      (const u8*)&pReq->rssi_close_proximity_5g_val, tlvs);
    }
    if (pReq->config_rssi_window_size) {
        tlvs = addTlv(NAN_TLV_TYPE_RSSI_AVERAGING_WINDOW_SIZE, sizeof(pReq->rssi_window_size_val),
                      (const u8*)&pReq->rssi_window_size_val, tlvs);
    }
    if (pReq->config_oui) {
        tlvs = addTlv(NAN_TLV_TYPE_CLUSTER_OUI_NETWORK_ID, sizeof(pReq->oui_val),
                      (const u8*)&pReq->oui_val, tlvs);
    }
    if (pReq->config_intf_addr) {
        tlvs = addTlv(NAN_TLV_TYPE_SOURCE_MAC_ADDRESS, sizeof(pReq->intf_addr_val),
                      (const u8*)&pReq->intf_addr_val[0], tlvs);
    }
    if (pReq->config_cluster_attribute_val) {
        tlvs = addTlv(NAN_TLV_TYPE_CLUSTER_ATTRIBUTE_IN_SDF,
                      sizeof(pReq->config_cluster_attribute_val),
                      (const u8*)&pReq->config_cluster_attribute_val, tlvs);
    }
    if (pReq->config_scan_params) {
        u32 socialChannelParamVal[NAN_MAX_SOCIAL_CHANNELS];
        /* Fill the social channel param */
        fillNanSocialChannelParamVal(&pReq->scan_params_val, socialChannelParamVal);
        int i;
        for (i = 0; i < NAN_MAX_SOCIAL_CHANNELS; i++) {
            tlvs = addTlv(NAN_TLV_TYPE_SOCIAL_CHANNEL_SCAN_PARAMS, sizeof(socialChannelParamVal[i]),
                          (const u8*)&socialChannelParamVal[i], tlvs);
        }
    }
    if (pReq->config_random_factor_force) {
        tlvs = addTlv(NAN_TLV_TYPE_RANDOM_FACTOR_FORCE, sizeof(pReq->random_factor_force_val),
                      (const u8*)&pReq->random_factor_force_val, tlvs);
    }
    if (pReq->config_hop_count_force) {
        tlvs = addTlv(NAN_TLV_TYPE_HOP_COUNT_FORCE, sizeof(pReq->hop_count_force_val),
                      (const u8*)&pReq->hop_count_force_val, tlvs);
    }
    if (pReq->config_24g_channel) {
        tlvs = addTlv(NAN_TLV_TYPE_24G_CHANNEL, sizeof(u32), (const u8*)&pReq->channel_24g_val,
                      tlvs);
    }
    if (pReq->config_5g_channel) {
        tlvs = addTlv(NAN_TLV_TYPE_5G_CHANNEL, sizeof(u32), (const u8*)&pReq->channel_5g_val, tlvs);
    }
    if (pReq->config_dw.config_2dot4g_dw_band) {
        tlvs = addTlv(NAN_TLV_TYPE_2G_COMMITTED_DW, sizeof(pReq->config_dw.dw_2dot4g_interval_val),
                      (const u8*)&pReq->config_dw.dw_2dot4g_interval_val, tlvs);
    }
    if (pReq->config_dw.config_5g_dw_band) {
        tlvs = addTlv(NAN_TLV_TYPE_5G_COMMITTED_DW, sizeof(pReq->config_dw.dw_5g_interval_val),
                      (const u8*)&pReq->config_dw.dw_5g_interval_val, tlvs);
    }
    if (pReq->config_disc_mac_addr_randomization) {
        tlvs = addTlv(NAN_TLV_TYPE_DISC_MAC_ADDR_RANDOM_INTERVAL, sizeof(u32),
                      (const u8*)&pReq->disc_mac_addr_rand_interval_sec, tlvs);
    }

    u32 config_discovery_indications;
    config_discovery_indications = (u32)pReq->discovery_indication_cfg;
    tlvs = addTlv(NAN_TLV_TYPE_CONFIG_DISCOVERY_INDICATIONS, sizeof(u32),
                  (const u8*)&config_discovery_indications, tlvs);

    if (pReq->config_subscribe_sid_beacon) {
        tlvs = addTlv(NAN_TLV_TYPE_SUBSCRIBE_SID_BEACON, sizeof(pReq->subscribe_sid_beacon_val),
                      (const u8*)&pReq->subscribe_sid_beacon_val, tlvs);
    }
    if (pReq->config_discovery_beacon_int) {
        tlvs = addTlv(NAN_TLV_TYPE_DB_INTERVAL, sizeof(u32),
                      (const u8*)&pReq->discovery_beacon_interval, tlvs);
    }
    if (pReq->config_nss) {
        tlvs = addTlv(NAN_TLV_TYPE_TX_RX_CHAINS, sizeof(u32), (const u8*)&pReq->nss, tlvs);
    }
    if (pReq->config_enable_ranging) {
        tlvs = addTlv(NAN_TLV_TYPE_ENABLE_DEVICE_RANGING, sizeof(u32),
                      (const u8*)&pReq->enable_ranging, tlvs);
    }
    if (pReq->config_dw_early_termination) {
        tlvs = addTlv(NAN_TLV_TYPE_DW_EARLY_TERMINATION, sizeof(u32),
                      (const u8*)&pReq->enable_dw_termination, tlvs);
    }
    if (pReq->config_ndpe_attr) {
        NanDevCapAttrCap nanDevCapAttr;
        memset(&nanDevCapAttr, 0, sizeof(nanDevCapAttr));
        nanDevCapAttr.ndpe_attr_supp = pReq->use_ndpe_attr;
        tlvs = addTlv(NAN_TLV_TYPE_DEV_CAP_ATTR_CAPABILITY, sizeof(NanDevCapAttrCap),
                      (const u8*)&nanDevCapAttr, tlvs);
    }
    mVendorData = (char*)pFwReq;
    mDataLen = message_len;

    // Insert the vendor specific data
    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

wifi_error NanCommand::putNanDisable(transaction_id id) {
    wifi_error ret;
    ALOGV("NAN_DISABLE");
    size_t message_len = sizeof(NanDisableReqMsg);

    pNanDisableReqMsg pFwReq = (pNanDisableReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ALOGV("Message Len %zu", message_len);
    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_DISABLE_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    pFwReq->fwHeader.transactionId = id;

    mVendorData = (char*)pFwReq;
    mDataLen = message_len;

    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

wifi_error NanCommand::putNanConfig(transaction_id id, const NanConfigRequest* pReq) {
    wifi_error ret;
    ALOGV("NAN_CONFIG");
    size_t message_len = 0;
    int idx = 0;

    if (pReq == NULL || pReq->num_config_discovery_attr > NAN_MAX_POSTDISCOVERY_LEN) {
        cleanup();
        return WIFI_ERROR_INVALID_ARGS;
    }

    message_len = sizeof(NanMsgHeader);

    message_len +=
            (pReq->config_sid_beacon ? (SIZEOF_TLV_HDR + sizeof(pReq->sid_beacon)) : 0) +
            (pReq->config_master_pref ? (SIZEOF_TLV_HDR + sizeof(pReq->master_pref)) : 0) +
            (pReq->config_rssi_proximity ? (SIZEOF_TLV_HDR + sizeof(pReq->rssi_proximity)) : 0) +
            (pReq->config_5g_rssi_close_proximity
                     ? (SIZEOF_TLV_HDR + sizeof(pReq->rssi_close_proximity_5g_val))
                     : 0) +
            (pReq->config_rssi_window_size ? (SIZEOF_TLV_HDR + sizeof(pReq->rssi_window_size_val))
                                           : 0) +
            (pReq->config_cluster_attribute_val
                     ? (SIZEOF_TLV_HDR + sizeof(pReq->config_cluster_attribute_val))
                     : 0) +
            (pReq->config_scan_params ? NAN_MAX_SOCIAL_CHANNELS * (SIZEOF_TLV_HDR + sizeof(u32))
                                      : 0) +
            (pReq->config_random_factor_force
                     ? (SIZEOF_TLV_HDR + sizeof(pReq->random_factor_force_val))
                     : 0) +
            (pReq->config_hop_count_force ? (SIZEOF_TLV_HDR + sizeof(pReq->hop_count_force_val))
                                          : 0) +
            (pReq->config_conn_capability ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_dw.config_2dot4g_dw_band ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_dw.config_5g_dw_band ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_disc_mac_addr_randomization ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_subscribe_sid_beacon
                     ? (SIZEOF_TLV_HDR + sizeof(pReq->subscribe_sid_beacon_val))
                     : 0) +
            (
                    /* Always include cfg discovery indication TLV */
                    SIZEOF_TLV_HDR + sizeof(u32)) +
            (pReq->config_discovery_beacon_int ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_nss ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_enable_ranging ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_dw_early_termination ? (SIZEOF_TLV_HDR + sizeof(u32)) : 0) +
            (pReq->config_ndpe_attr ? (SIZEOF_TLV_HDR + sizeof(NanDevCapAttrCap)) : 0);

    if (pReq->num_config_discovery_attr) {
        for (idx = 0; idx < pReq->num_config_discovery_attr; idx++) {
            message_len += SIZEOF_TLV_HDR +
                           calcNanTransmitPostDiscoverySize(&pReq->discovery_attr_val[idx]);
        }
    }

    if (pReq->config_fam && calcNanFurtherAvailabilityMapSize(&pReq->fam_val)) {
        message_len += (SIZEOF_TLV_HDR + calcNanFurtherAvailabilityMapSize(&pReq->fam_val));
    }

    pNanConfigurationReqMsg pFwReq = (pNanConfigurationReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ALOGV("Message Len %zu", message_len);
    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_CONFIGURATION_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    pFwReq->fwHeader.transactionId = id;

    u8* tlvs = pFwReq->ptlv;
    if (pReq->config_sid_beacon) {
        tlvs = addTlv(NAN_TLV_TYPE_SID_BEACON, sizeof(pReq->sid_beacon),
                      (const u8*)&pReq->sid_beacon, tlvs);
    }
    if (pReq->config_master_pref) {
        tlvs = addTlv(NAN_TLV_TYPE_MASTER_PREFERENCE, sizeof(pReq->master_pref),
                      (const u8*)&pReq->master_pref, tlvs);
    }
    if (pReq->config_rssi_window_size) {
        tlvs = addTlv(NAN_TLV_TYPE_RSSI_AVERAGING_WINDOW_SIZE, sizeof(pReq->rssi_window_size_val),
                      (const u8*)&pReq->rssi_window_size_val, tlvs);
    }
    if (pReq->config_rssi_proximity) {
        tlvs = addTlv(NAN_TLV_TYPE_24G_RSSI_CLOSE_PROXIMITY, sizeof(pReq->rssi_proximity),
                      (const u8*)&pReq->rssi_proximity, tlvs);
    }
    if (pReq->config_5g_rssi_close_proximity) {
        tlvs = addTlv(NAN_TLV_TYPE_5G_RSSI_CLOSE_PROXIMITY,
                      sizeof(pReq->rssi_close_proximity_5g_val),
                      (const u8*)&pReq->rssi_close_proximity_5g_val, tlvs);
    }
    if (pReq->config_cluster_attribute_val) {
        tlvs = addTlv(NAN_TLV_TYPE_CLUSTER_ATTRIBUTE_IN_SDF,
                      sizeof(pReq->config_cluster_attribute_val),
                      (const u8*)&pReq->config_cluster_attribute_val, tlvs);
    }
    if (pReq->config_scan_params) {
        u32 socialChannelParamVal[NAN_MAX_SOCIAL_CHANNELS];
        /* Fill the social channel param */
        fillNanSocialChannelParamVal(&pReq->scan_params_val, socialChannelParamVal);
        int i;
        for (i = 0; i < NAN_MAX_SOCIAL_CHANNELS; i++) {
            tlvs = addTlv(NAN_TLV_TYPE_SOCIAL_CHANNEL_SCAN_PARAMS, sizeof(socialChannelParamVal[i]),
                          (const u8*)&socialChannelParamVal[i], tlvs);
        }
    }
    if (pReq->config_random_factor_force) {
        tlvs = addTlv(NAN_TLV_TYPE_RANDOM_FACTOR_FORCE, sizeof(pReq->random_factor_force_val),
                      (const u8*)&pReq->random_factor_force_val, tlvs);
    }
    if (pReq->config_hop_count_force) {
        tlvs = addTlv(NAN_TLV_TYPE_HOP_COUNT_FORCE, sizeof(pReq->hop_count_force_val),
                      (const u8*)&pReq->hop_count_force_val, tlvs);
    }
    if (pReq->config_conn_capability) {
        u32 val = getNanTransmitPostConnectivityCapabilityVal(&pReq->conn_capability_val);
        tlvs = addTlv(NAN_TLV_TYPE_POST_NAN_CONNECTIVITY_CAPABILITIES_TRANSMIT, sizeof(val),
                      (const u8*)&val, tlvs);
    }
    if (pReq->num_config_discovery_attr) {
        for (idx = 0; idx < pReq->num_config_discovery_attr; idx++) {
            fillNanTransmitPostDiscoveryVal(&pReq->discovery_attr_val[idx],
                                            (u8*)(tlvs + SIZEOF_TLV_HDR));
            tlvs = addTlv(NAN_TLV_TYPE_POST_NAN_DISCOVERY_ATTRIBUTE_TRANSMIT,
                          calcNanTransmitPostDiscoverySize(&pReq->discovery_attr_val[idx]),
                          (const u8*)(tlvs + SIZEOF_TLV_HDR), tlvs);
        }
    }
    if (pReq->config_fam && calcNanFurtherAvailabilityMapSize(&pReq->fam_val)) {
        fillNanFurtherAvailabilityMapVal(&pReq->fam_val, (u8*)(tlvs + SIZEOF_TLV_HDR));
        tlvs = addTlv(NAN_TLV_TYPE_FURTHER_AVAILABILITY_MAP,
                      calcNanFurtherAvailabilityMapSize(&pReq->fam_val),
                      (const u8*)(tlvs + SIZEOF_TLV_HDR), tlvs);
    }
    if (pReq->config_dw.config_2dot4g_dw_band) {
        tlvs = addTlv(NAN_TLV_TYPE_2G_COMMITTED_DW, sizeof(pReq->config_dw.dw_2dot4g_interval_val),
                      (const u8*)&pReq->config_dw.dw_2dot4g_interval_val, tlvs);
    }
    if (pReq->config_dw.config_5g_dw_band) {
        tlvs = addTlv(NAN_TLV_TYPE_5G_COMMITTED_DW, sizeof(pReq->config_dw.dw_5g_interval_val),
                      (const u8*)&pReq->config_dw.dw_5g_interval_val, tlvs);
    }
    if (pReq->config_disc_mac_addr_randomization) {
        tlvs = addTlv(NAN_TLV_TYPE_DISC_MAC_ADDR_RANDOM_INTERVAL, sizeof(u32),
                      (const u8*)&pReq->disc_mac_addr_rand_interval_sec, tlvs);
    }
    if (pReq->config_subscribe_sid_beacon) {
        tlvs = addTlv(NAN_TLV_TYPE_SUBSCRIBE_SID_BEACON, sizeof(pReq->subscribe_sid_beacon_val),
                      (const u8*)&pReq->subscribe_sid_beacon_val, tlvs);
    }
    if (pReq->config_discovery_beacon_int) {
        tlvs = addTlv(NAN_TLV_TYPE_DB_INTERVAL, sizeof(u32),
                      (const u8*)&pReq->discovery_beacon_interval, tlvs);
    }

    u32 config_discovery_indications;
    config_discovery_indications = (u32)(pReq->discovery_indication_cfg);
    /* Always include the discovery cfg TLV as there is no cfg flag */
    tlvs = addTlv(NAN_TLV_TYPE_CONFIG_DISCOVERY_INDICATIONS, sizeof(u32),
                  (const u8*)&config_discovery_indications, tlvs);
    if (pReq->config_nss) {
        tlvs = addTlv(NAN_TLV_TYPE_TX_RX_CHAINS, sizeof(u32), (const u8*)&pReq->nss, tlvs);
    }
    if (pReq->config_enable_ranging) {
        tlvs = addTlv(NAN_TLV_TYPE_ENABLE_DEVICE_RANGING, sizeof(u32),
                      (const u8*)&pReq->enable_ranging, tlvs);
    }
    if (pReq->config_dw_early_termination) {
        tlvs = addTlv(NAN_TLV_TYPE_DW_EARLY_TERMINATION, sizeof(u32),
                      (const u8*)&pReq->enable_dw_termination, tlvs);
    }
    if (pReq->config_ndpe_attr) {
        NanDevCapAttrCap nanDevCapAttr;
        memset(&nanDevCapAttr, 0, sizeof(nanDevCapAttr));
        nanDevCapAttr.ndpe_attr_supp = pReq->use_ndpe_attr;
        tlvs = addTlv(NAN_TLV_TYPE_DEV_CAP_ATTR_CAPABILITY, sizeof(NanDevCapAttrCap),
                      (const u8*)&nanDevCapAttr, tlvs);
    }

    mVendorData = (char*)pFwReq;
    mDataLen = message_len;

    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

wifi_error NanCommand::putNanPublish(transaction_id id, const NanPublishRequest* pReq) {
    wifi_error ret;
    ALOGV("NAN_PUBLISH");
    if (pReq == NULL) {
        cleanup();
        return WIFI_ERROR_INVALID_ARGS;
    }

    size_t message_len =
            sizeof(NanMsgHeader) + sizeof(NanPublishServiceReqParams) +
            (pReq->service_name_len ? SIZEOF_TLV_HDR + pReq->service_name_len : 0) +
            (pReq->service_specific_info_len ? SIZEOF_TLV_HDR + pReq->service_specific_info_len
                                             : 0) +
            (pReq->rx_match_filter_len ? SIZEOF_TLV_HDR + pReq->rx_match_filter_len : 0) +
            (pReq->tx_match_filter_len ? SIZEOF_TLV_HDR + pReq->tx_match_filter_len : 0) +
            (SIZEOF_TLV_HDR + sizeof(NanServiceAcceptPolicy)) +
            (pReq->cipher_type ? SIZEOF_TLV_HDR + sizeof(NanCsidType) : 0) +
            ((pReq->sdea_params.config_nan_data_path || pReq->sdea_params.security_cfg ||
              pReq->sdea_params.ranging_state || pReq->sdea_params.range_report ||
              pReq->sdea_params.qos_cfg)
                     ? SIZEOF_TLV_HDR + sizeof(NanFWSdeaCtrlParams)
                     : 0) +
            ((pReq->ranging_cfg.ranging_interval_msec ||
              pReq->ranging_cfg.config_ranging_indications ||
              pReq->ranging_cfg.distance_ingress_mm || pReq->ranging_cfg.distance_egress_mm)
                     ? SIZEOF_TLV_HDR + sizeof(NanFWRangeConfigParams)
                     : 0) +
            ((pReq->range_response_cfg.publish_id ||
              (pReq->range_response_cfg.ranging_response >= NAN_RANGE_REQUEST_ACCEPT))
                     ? SIZEOF_TLV_HDR + sizeof(NanFWRangeReqMsg)
                     : 0) +
            (pReq->sdea_service_specific_info_len
                     ? SIZEOF_TLV_HDR + pReq->sdea_service_specific_info_len
                     : 0);

    if ((pReq->key_info.key_type == NAN_SECURITY_KEY_INPUT_PMK) &&
        (pReq->key_info.body.pmk_info.pmk_len == NAN_PMK_INFO_LEN))
        message_len += SIZEOF_TLV_HDR + NAN_PMK_INFO_LEN;
    else if ((pReq->key_info.key_type == NAN_SECURITY_KEY_INPUT_PASSPHRASE) &&
             (pReq->key_info.body.passphrase_info.passphrase_len >=
              NAN_SECURITY_MIN_PASSPHRASE_LEN) &&
             (pReq->key_info.body.passphrase_info.passphrase_len <=
              NAN_SECURITY_MAX_PASSPHRASE_LEN))
        message_len += SIZEOF_TLV_HDR + pReq->key_info.body.passphrase_info.passphrase_len;

    pNanPublishServiceReqMsg pFwReq = (pNanPublishServiceReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ALOGV("Message Len %zu", message_len);
    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_PUBLISH_SERVICE_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    if (pReq->publish_id == 0) {
        pFwReq->fwHeader.handle = 0xFFFF;
    } else {
        pFwReq->fwHeader.handle = pReq->publish_id;
    }
    pFwReq->fwHeader.transactionId = id;

    pFwReq->publishServiceReqParams.ttl = pReq->ttl;
    pFwReq->publishServiceReqParams.period = pReq->period;
    pFwReq->publishServiceReqParams.replyIndFlag = (pReq->recv_indication_cfg & BIT_3) ? 0 : 1;
    pFwReq->publishServiceReqParams.publishType = pReq->publish_type;
    pFwReq->publishServiceReqParams.txType = pReq->tx_type;

    pFwReq->publishServiceReqParams.rssiThresholdFlag = pReq->rssi_threshold_flag;
    pFwReq->publishServiceReqParams.matchAlg = pReq->publish_match_indicator;
    pFwReq->publishServiceReqParams.count = pReq->publish_count;
    pFwReq->publishServiceReqParams.connmap = pReq->connmap;
    pFwReq->publishServiceReqParams.pubTerminatedIndDisableFlag =
            (pReq->recv_indication_cfg & BIT_0) ? 1 : 0;
    pFwReq->publishServiceReqParams.pubMatchExpiredIndDisableFlag =
            (pReq->recv_indication_cfg & BIT_1) ? 1 : 0;
    pFwReq->publishServiceReqParams.followupRxIndDisableFlag =
            (pReq->recv_indication_cfg & BIT_2) ? 1 : 0;

    pFwReq->publishServiceReqParams.reserved2 = 0;

    u8* tlvs = pFwReq->ptlv;
    if (pReq->service_name_len) {
        tlvs = addTlv(NAN_TLV_TYPE_SERVICE_NAME, pReq->service_name_len,
                      (const u8*)&pReq->service_name[0], tlvs);
    }
    if (pReq->service_specific_info_len) {
        tlvs = addTlv(NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO, pReq->service_specific_info_len,
                      (const u8*)&pReq->service_specific_info[0], tlvs);
    }
    if (pReq->rx_match_filter_len) {
        tlvs = addTlv(NAN_TLV_TYPE_RX_MATCH_FILTER, pReq->rx_match_filter_len,
                      (const u8*)&pReq->rx_match_filter[0], tlvs);
    }
    if (pReq->tx_match_filter_len) {
        tlvs = addTlv(NAN_TLV_TYPE_TX_MATCH_FILTER, pReq->tx_match_filter_len,
                      (const u8*)&pReq->tx_match_filter[0], tlvs);
    }

    /* Pass the Accept policy always */
    tlvs = addTlv(NAN_TLV_TYPE_NAN_SERVICE_ACCEPT_POLICY, sizeof(NanServiceAcceptPolicy),
                  (const u8*)&pReq->service_responder_policy, tlvs);

    if (pReq->cipher_type) {
        NanCsidType pNanCsidType;
        pNanCsidType.csid_type = pReq->cipher_type;
        tlvs = addTlv(NAN_TLV_TYPE_NAN_CSID, sizeof(NanCsidType), (const u8*)&pNanCsidType, tlvs);
    }

    if ((pReq->key_info.key_type == NAN_SECURITY_KEY_INPUT_PMK) &&
        (pReq->key_info.body.pmk_info.pmk_len == NAN_PMK_INFO_LEN)) {
        tlvs = addTlv(NAN_TLV_TYPE_NAN_PMK, pReq->key_info.body.pmk_info.pmk_len,
                      (const u8*)&pReq->key_info.body.pmk_info.pmk[0], tlvs);
    } else if ((pReq->key_info.key_type == NAN_SECURITY_KEY_INPUT_PASSPHRASE) &&
               (pReq->key_info.body.passphrase_info.passphrase_len >=
                NAN_SECURITY_MIN_PASSPHRASE_LEN) &&
               (pReq->key_info.body.passphrase_info.passphrase_len <=
                NAN_SECURITY_MAX_PASSPHRASE_LEN)) {
        tlvs = addTlv(NAN_TLV_TYPE_NAN_PASSPHRASE,
                      pReq->key_info.body.passphrase_info.passphrase_len,
                      (const u8*)&pReq->key_info.body.passphrase_info.passphrase[0], tlvs);
    }

    if (pReq->sdea_params.config_nan_data_path || pReq->sdea_params.security_cfg ||
        pReq->sdea_params.ranging_state || pReq->sdea_params.range_report ||
        pReq->sdea_params.qos_cfg) {
        NanFWSdeaCtrlParams pNanFWSdeaCtrlParams;
        memset(&pNanFWSdeaCtrlParams, 0, sizeof(NanFWSdeaCtrlParams));

        if (pReq->sdea_params.config_nan_data_path) {
            pNanFWSdeaCtrlParams.data_path_required = 1;
            pNanFWSdeaCtrlParams.data_path_type = (pReq->sdea_params.ndp_type & BIT_0)
                                                          ? NAN_DATA_PATH_MULTICAST_MSG
                                                          : NAN_DATA_PATH_UNICAST_MSG;
        }
        if (pReq->sdea_params.security_cfg) {
            pNanFWSdeaCtrlParams.security_required = pReq->sdea_params.security_cfg;
        }
        if (pReq->sdea_params.ranging_state) {
            pNanFWSdeaCtrlParams.ranging_required = pReq->sdea_params.ranging_state;
        }
        if (pReq->sdea_params.range_report) {
            pNanFWSdeaCtrlParams.range_report =
                    (((pReq->sdea_params.range_report & NAN_ENABLE_RANGE_REPORT) >> 1) ? 1 : 0);
        }
        if (pReq->sdea_params.qos_cfg) {
            pNanFWSdeaCtrlParams.qos_required = pReq->sdea_params.qos_cfg;
        }
        tlvs = addTlv(NAN_TLV_TYPE_SDEA_CTRL_PARAMS, sizeof(NanFWSdeaCtrlParams),
                      (const u8*)&pNanFWSdeaCtrlParams, tlvs);
    }

    if (pReq->ranging_cfg.ranging_interval_msec || pReq->ranging_cfg.config_ranging_indications ||
        pReq->ranging_cfg.distance_ingress_mm || pReq->ranging_cfg.distance_egress_mm) {
        NanFWRangeConfigParams pNanFWRangingCfg;

        memset(&pNanFWRangingCfg, 0, sizeof(NanFWRangeConfigParams));
        pNanFWRangingCfg.range_interval = pReq->ranging_cfg.ranging_interval_msec;

        pNanFWRangingCfg.ranging_indication_event = pReq->ranging_cfg.config_ranging_indications;
        if (pReq->ranging_cfg.config_ranging_indications & NAN_RANGING_INDICATE_INGRESS_MET_MASK)
            pNanFWRangingCfg.geo_fence_threshold.inner_threshold =
                    pReq->ranging_cfg.distance_ingress_mm;
        if (pReq->ranging_cfg.config_ranging_indications & NAN_RANGING_INDICATE_EGRESS_MET_MASK)
            pNanFWRangingCfg.geo_fence_threshold.outer_threshold =
                    pReq->ranging_cfg.distance_egress_mm;
        tlvs = addTlv(NAN_TLV_TYPE_NAN_RANGING_CFG, sizeof(NanFWRangeConfigParams),
                      (const u8*)&pNanFWRangingCfg, tlvs);
    }

    if (pReq->sdea_service_specific_info_len) {
        tlvs = addTlv(NAN_TLV_TYPE_SDEA_SERVICE_SPECIFIC_INFO, pReq->sdea_service_specific_info_len,
                      (const u8*)&pReq->sdea_service_specific_info[0], tlvs);
    }

    if (pReq->range_response_cfg.publish_id ||
        (pReq->range_response_cfg.ranging_response >= NAN_RANGE_REQUEST_ACCEPT)) {
        NanFWRangeReqMsg pNanFWRangeReqMsg;
        memset(&pNanFWRangeReqMsg, 0, sizeof(NanFWRangeReqMsg));
        pNanFWRangeReqMsg.range_id = (u16)pReq->range_response_cfg.publish_id;
        CHAR_ARRAY_TO_MAC_ADDR(pReq->range_response_cfg.peer_addr,
                               pNanFWRangeReqMsg.range_mac_addr);
        pNanFWRangeReqMsg.ranging_accept =
                ((pReq->range_response_cfg.ranging_response == NAN_RANGE_REQUEST_ACCEPT) ? 1 : 0);
        pNanFWRangeReqMsg.ranging_reject =
                ((pReq->range_response_cfg.ranging_response == NAN_RANGE_REQUEST_REJECT) ? 1 : 0);
        pNanFWRangeReqMsg.ranging_cancel =
                ((pReq->range_response_cfg.ranging_response == NAN_RANGE_REQUEST_CANCEL) ? 1 : 0);
        tlvs = addTlv(NAN_TLV_TYPE_NAN20_RANGING_REQUEST, sizeof(NanFWRangeReqMsg),
                      (const u8*)&pNanFWRangeReqMsg, tlvs);
    }

    mVendorData = (char*)pFwReq;
    mDataLen = message_len;

    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

wifi_error NanCommand::putNanPublishCancel(transaction_id id, const NanPublishCancelRequest* pReq) {
    wifi_error ret;
    ALOGV("NAN_PUBLISH_CANCEL");
    if (pReq == NULL) {
        cleanup();
        return WIFI_ERROR_INVALID_ARGS;
    }
    size_t message_len = sizeof(NanPublishServiceCancelReqMsg);

    pNanPublishServiceCancelReqMsg pFwReq = (pNanPublishServiceCancelReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ALOGV("Message Len %zu", message_len);
    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_PUBLISH_SERVICE_CANCEL_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    pFwReq->fwHeader.handle = pReq->publish_id;
    pFwReq->fwHeader.transactionId = id;

    mVendorData = (char*)pFwReq;
    mDataLen = message_len;

    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

wifi_error NanCommand::putNanSubscribe(transaction_id id, const NanSubscribeRequest* pReq) {
    wifi_error ret;

    ALOGV("NAN_SUBSCRIBE");
    if (pReq == NULL) {
        cleanup();
        return WIFI_ERROR_INVALID_ARGS;
    }

    size_t message_len =
            sizeof(NanMsgHeader) + sizeof(NanSubscribeServiceReqParams) +
            (pReq->service_name_len ? SIZEOF_TLV_HDR + pReq->service_name_len : 0) +
            (pReq->service_specific_info_len ? SIZEOF_TLV_HDR + pReq->service_specific_info_len
                                             : 0) +
            (pReq->rx_match_filter_len ? SIZEOF_TLV_HDR + pReq->rx_match_filter_len : 0) +
            (pReq->tx_match_filter_len ? SIZEOF_TLV_HDR + pReq->tx_match_filter_len : 0) +
            (pReq->cipher_type ? SIZEOF_TLV_HDR + sizeof(NanCsidType) : 0) +
            ((pReq->sdea_params.config_nan_data_path || pReq->sdea_params.security_cfg ||
              pReq->sdea_params.ranging_state || pReq->sdea_params.range_report ||
              pReq->sdea_params.qos_cfg)
                     ? SIZEOF_TLV_HDR + sizeof(NanFWSdeaCtrlParams)
                     : 0) +
            ((pReq->ranging_cfg.ranging_interval_msec ||
              pReq->ranging_cfg.config_ranging_indications ||
              pReq->ranging_cfg.distance_ingress_mm || pReq->ranging_cfg.distance_egress_mm)
                     ? SIZEOF_TLV_HDR + sizeof(NanFWRangeConfigParams)
                     : 0) +
            ((pReq->range_response_cfg.requestor_instance_id ||
              pReq->range_response_cfg.ranging_response)
                     ? SIZEOF_TLV_HDR + sizeof(NanFWRangeReqMsg)
                     : 0) +
            (pReq->sdea_service_specific_info_len
                     ? SIZEOF_TLV_HDR + pReq->sdea_service_specific_info_len
                     : 0);

    message_len += (pReq->num_intf_addr_present * (SIZEOF_TLV_HDR + NAN_MAC_ADDR_LEN));

    if ((pReq->key_info.key_type == NAN_SECURITY_KEY_INPUT_PMK) &&
        (pReq->key_info.body.pmk_info.pmk_len == NAN_PMK_INFO_LEN))
        message_len += SIZEOF_TLV_HDR + NAN_PMK_INFO_LEN;
    else if ((pReq->key_info.key_type == NAN_SECURITY_KEY_INPUT_PASSPHRASE) &&
             (pReq->key_info.body.passphrase_info.passphrase_len >=
              NAN_SECURITY_MIN_PASSPHRASE_LEN) &&
             (pReq->key_info.body.passphrase_info.passphrase_len <=
              NAN_SECURITY_MAX_PASSPHRASE_LEN))
        message_len += SIZEOF_TLV_HDR + pReq->key_info.body.passphrase_info.passphrase_len;

    pNanSubscribeServiceReqMsg pFwReq = (pNanSubscribeServiceReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ALOGV("Message Len %zu", message_len);
    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_SUBSCRIBE_SERVICE_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    if (pReq->subscribe_id == 0) {
        pFwReq->fwHeader.handle = 0xFFFF;
    } else {
        pFwReq->fwHeader.handle = pReq->subscribe_id;
    }
    pFwReq->fwHeader.transactionId = id;

    pFwReq->subscribeServiceReqParams.ttl = pReq->ttl;
    pFwReq->subscribeServiceReqParams.period = pReq->period;
    pFwReq->subscribeServiceReqParams.subscribeType = pReq->subscribe_type;
    pFwReq->subscribeServiceReqParams.srfAttr = pReq->serviceResponseFilter;
    pFwReq->subscribeServiceReqParams.srfInclude = pReq->serviceResponseInclude;
    pFwReq->subscribeServiceReqParams.srfSend = pReq->useServiceResponseFilter;
    pFwReq->subscribeServiceReqParams.ssiRequired = pReq->ssiRequiredForMatchIndication;
    pFwReq->subscribeServiceReqParams.matchAlg = pReq->subscribe_match_indicator;
    pFwReq->subscribeServiceReqParams.count = pReq->subscribe_count;
    pFwReq->subscribeServiceReqParams.rssiThresholdFlag = pReq->rssi_threshold_flag;
    pFwReq->subscribeServiceReqParams.subTerminatedIndDisableFlag =
            (pReq->recv_indication_cfg & BIT_0) ? 1 : 0;
    pFwReq->subscribeServiceReqParams.subMatchExpiredIndDisableFlag =
            (pReq->recv_indication_cfg & BIT_1) ? 1 : 0;
    pFwReq->subscribeServiceReqParams.followupRxIndDisableFlag =
            (pReq->recv_indication_cfg & BIT_2) ? 1 : 0;
    pFwReq->subscribeServiceReqParams.connmap = pReq->connmap;
    pFwReq->subscribeServiceReqParams.reserved = 0;

    u8* tlvs = pFwReq->ptlv;
    if (pReq->service_name_len) {
        tlvs = addTlv(NAN_TLV_TYPE_SERVICE_NAME, pReq->service_name_len,
                      (const u8*)&pReq->service_name[0], tlvs);
    }
    if (pReq->service_specific_info_len) {
        tlvs = addTlv(NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO, pReq->service_specific_info_len,
                      (const u8*)&pReq->service_specific_info[0], tlvs);
    }
    if (pReq->rx_match_filter_len) {
        tlvs = addTlv(NAN_TLV_TYPE_RX_MATCH_FILTER, pReq->rx_match_filter_len,
                      (const u8*)&pReq->rx_match_filter[0], tlvs);
    }
    if (pReq->tx_match_filter_len) {
        tlvs = addTlv(NAN_TLV_TYPE_TX_MATCH_FILTER, pReq->tx_match_filter_len,
                      (const u8*)&pReq->tx_match_filter[0], tlvs);
    }

    int i = 0;
    for (i = 0; i < pReq->num_intf_addr_present; i++) {
        tlvs = addTlv(NAN_TLV_TYPE_MAC_ADDRESS, NAN_MAC_ADDR_LEN, (const u8*)&pReq->intf_addr[i][0],
                      tlvs);
    }

    if (pReq->cipher_type) {
        NanCsidType pNanCsidType;
        pNanCsidType.csid_type = pReq->cipher_type;
        tlvs = addTlv(NAN_TLV_TYPE_NAN_CSID, sizeof(NanCsidType), (const u8*)&pNanCsidType, tlvs);
    }

    if ((pReq->key_info.key_type == NAN_SECURITY_KEY_INPUT_PMK) &&
        (pReq->key_info.body.pmk_info.pmk_len == NAN_PMK_INFO_LEN)) {
        tlvs = addTlv(NAN_TLV_TYPE_NAN_PMK, pReq->key_info.body.pmk_info.pmk_len,
                      (const u8*)&pReq->key_info.body.pmk_info.pmk[0], tlvs);
    } else if ((pReq->key_info.key_type == NAN_SECURITY_KEY_INPUT_PASSPHRASE) &&
               (pReq->key_info.body.passphrase_info.passphrase_len >=
                NAN_SECURITY_MIN_PASSPHRASE_LEN) &&
               (pReq->key_info.body.passphrase_info.passphrase_len <=
                NAN_SECURITY_MAX_PASSPHRASE_LEN)) {
        tlvs = addTlv(NAN_TLV_TYPE_NAN_PASSPHRASE,
                      pReq->key_info.body.passphrase_info.passphrase_len,
                      (const u8*)&pReq->key_info.body.passphrase_info.passphrase[0], tlvs);
    }

    if (pReq->sdea_params.config_nan_data_path || pReq->sdea_params.security_cfg ||
        pReq->sdea_params.ranging_state || pReq->sdea_params.range_report ||
        pReq->sdea_params.qos_cfg) {
        NanFWSdeaCtrlParams pNanFWSdeaCtrlParams;
        memset(&pNanFWSdeaCtrlParams, 0, sizeof(NanFWSdeaCtrlParams));

        if (pReq->sdea_params.config_nan_data_path) {
            pNanFWSdeaCtrlParams.data_path_required = 1;
            pNanFWSdeaCtrlParams.data_path_type = (pReq->sdea_params.ndp_type & BIT_0)
                                                          ? NAN_DATA_PATH_MULTICAST_MSG
                                                          : NAN_DATA_PATH_UNICAST_MSG;
        }
        if (pReq->sdea_params.security_cfg) {
            pNanFWSdeaCtrlParams.security_required = pReq->sdea_params.security_cfg;
        }
        if (pReq->sdea_params.ranging_state) {
            pNanFWSdeaCtrlParams.ranging_required = pReq->sdea_params.ranging_state;
        }
        if (pReq->sdea_params.range_report) {
            pNanFWSdeaCtrlParams.range_report =
                    ((pReq->sdea_params.range_report & NAN_ENABLE_RANGE_REPORT >> 1) ? 1 : 0);
        }
        if (pReq->sdea_params.qos_cfg) {
            pNanFWSdeaCtrlParams.qos_required = pReq->sdea_params.qos_cfg;
        }
        tlvs = addTlv(NAN_TLV_TYPE_SDEA_CTRL_PARAMS, sizeof(NanFWSdeaCtrlParams),
                      (const u8*)&pNanFWSdeaCtrlParams, tlvs);
    }

    if (pReq->ranging_cfg.ranging_interval_msec || pReq->ranging_cfg.config_ranging_indications ||
        pReq->ranging_cfg.distance_ingress_mm || pReq->ranging_cfg.distance_egress_mm) {
        NanFWRangeConfigParams pNanFWRangingCfg;
        memset(&pNanFWRangingCfg, 0, sizeof(NanFWRangeConfigParams));
        pNanFWRangingCfg.range_interval = pReq->ranging_cfg.ranging_interval_msec;

        pNanFWRangingCfg.ranging_indication_event = pReq->ranging_cfg.config_ranging_indications;
        if (pReq->ranging_cfg.config_ranging_indications & NAN_RANGING_INDICATE_INGRESS_MET_MASK)
            pNanFWRangingCfg.geo_fence_threshold.inner_threshold =
                    pReq->ranging_cfg.distance_ingress_mm;
        if (pReq->ranging_cfg.config_ranging_indications & NAN_RANGING_INDICATE_EGRESS_MET_MASK)
            pNanFWRangingCfg.geo_fence_threshold.outer_threshold =
                    pReq->ranging_cfg.distance_egress_mm;
        tlvs = addTlv(NAN_TLV_TYPE_NAN_RANGING_CFG, sizeof(NanFWRangeConfigParams),
                      (const u8*)&pNanFWRangingCfg, tlvs);
    }

    if (pReq->sdea_service_specific_info_len) {
        tlvs = addTlv(NAN_TLV_TYPE_SDEA_SERVICE_SPECIFIC_INFO, pReq->sdea_service_specific_info_len,
                      (const u8*)&pReq->sdea_service_specific_info[0], tlvs);
    }

    if (pReq->range_response_cfg.requestor_instance_id ||
        (pReq->range_response_cfg.ranging_response >= NAN_RANGE_REQUEST_ACCEPT)) {
        NanFWRangeReqMsg pNanFWRangeReqMsg;
        memset(&pNanFWRangeReqMsg, 0, sizeof(NanFWRangeReqMsg));
        pNanFWRangeReqMsg.range_id = pReq->range_response_cfg.requestor_instance_id;
        memcpy(&pNanFWRangeReqMsg.range_mac_addr, &pReq->range_response_cfg.peer_addr,
               NAN_MAC_ADDR_LEN);
        pNanFWRangeReqMsg.ranging_accept =
                ((pReq->range_response_cfg.ranging_response == NAN_RANGE_REQUEST_ACCEPT) ? 1 : 0);
        pNanFWRangeReqMsg.ranging_reject =
                ((pReq->range_response_cfg.ranging_response == NAN_RANGE_REQUEST_REJECT) ? 1 : 0);
        pNanFWRangeReqMsg.ranging_cancel =
                ((pReq->range_response_cfg.ranging_response == NAN_RANGE_REQUEST_CANCEL) ? 1 : 0);
        tlvs = addTlv(NAN_TLV_TYPE_NAN20_RANGING_REQUEST, sizeof(NanFWRangeReqMsg),
                      (const u8*)&pNanFWRangeReqMsg, tlvs);
    }

    mVendorData = (char*)pFwReq;
    mDataLen = message_len;
    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

wifi_error NanCommand::putNanSubscribeCancel(transaction_id id,
                                             const NanSubscribeCancelRequest* pReq) {
    wifi_error ret;
    ALOGV("NAN_SUBSCRIBE_CANCEL");
    if (pReq == NULL) {
        cleanup();
        return WIFI_ERROR_INVALID_ARGS;
    }
    size_t message_len = sizeof(NanSubscribeServiceCancelReqMsg);

    pNanSubscribeServiceCancelReqMsg pFwReq = (pNanSubscribeServiceCancelReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ALOGV("Message Len %zu", message_len);
    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_SUBSCRIBE_SERVICE_CANCEL_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    pFwReq->fwHeader.handle = pReq->subscribe_id;
    pFwReq->fwHeader.transactionId = id;

    mVendorData = (char*)pFwReq;
    mDataLen = message_len;
    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

wifi_error NanCommand::putNanTransmitFollowup(transaction_id id,
                                              const NanTransmitFollowupRequest* pReq) {
    wifi_error ret;
    ALOGV("TRANSMIT_FOLLOWUP");
    if (pReq == NULL) {
        cleanup();
        return WIFI_ERROR_INVALID_ARGS;
    }

    size_t message_len =
            sizeof(NanMsgHeader) + sizeof(NanTransmitFollowupReqParams) +
            (pReq->service_specific_info_len ? SIZEOF_TLV_HDR + pReq->service_specific_info_len
                                             : 0) +
            (pReq->sdea_service_specific_info_len
                     ? SIZEOF_TLV_HDR + pReq->sdea_service_specific_info_len
                     : 0);

    /* Mac address needs to be added in TLV */
    message_len += (SIZEOF_TLV_HDR + sizeof(pReq->addr));

    pNanTransmitFollowupReqMsg pFwReq = (pNanTransmitFollowupReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ALOGV("Message Len %zu", message_len);
    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_TRANSMIT_FOLLOWUP_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    pFwReq->fwHeader.handle = pReq->publish_subscribe_id;
    pFwReq->fwHeader.transactionId = id;

    pFwReq->transmitFollowupReqParams.matchHandle = pReq->requestor_instance_id;
    if (pReq->priority != NAN_TX_PRIORITY_HIGH) {
        pFwReq->transmitFollowupReqParams.priority = 1;
    } else {
        pFwReq->transmitFollowupReqParams.priority = 2;
    }
    pFwReq->transmitFollowupReqParams.window = pReq->dw_or_faw;
    pFwReq->transmitFollowupReqParams.followupTxRspDisableFlag =
            (pReq->recv_indication_cfg & BIT_0) ? 1 : 0;
    pFwReq->transmitFollowupReqParams.reserved = 0;

    u8* tlvs = pFwReq->ptlv;

    /* Mac address needs to be added in TLV */
    tlvs = addTlv(NAN_TLV_TYPE_MAC_ADDRESS, sizeof(pReq->addr), (const u8*)&pReq->addr[0], tlvs);
    u16 tlv_type = NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO;

    if (pReq->service_specific_info_len) {
        tlvs = addTlv(tlv_type, pReq->service_specific_info_len,
                      (const u8*)&pReq->service_specific_info[0], tlvs);
    }

    if (pReq->sdea_service_specific_info_len) {
        tlvs = addTlv(NAN_TLV_TYPE_SDEA_SERVICE_SPECIFIC_INFO, pReq->sdea_service_specific_info_len,
                      (const u8*)&pReq->sdea_service_specific_info[0], tlvs);
    }

    mVendorData = (char*)pFwReq;
    mDataLen = message_len;

    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

wifi_error NanCommand::putNanStats(transaction_id id, const NanStatsRequest* pReq) {
    wifi_error ret;
    ALOGV("NAN_STATS");
    if (pReq == NULL) {
        cleanup();
        return WIFI_ERROR_INVALID_ARGS;
    }
    size_t message_len = sizeof(NanStatsReqMsg);

    pNanStatsReqMsg pFwReq = (pNanStatsReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ALOGV("Message Len %zu", message_len);
    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_STATS_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    pFwReq->fwHeader.transactionId = id;

    pFwReq->statsReqParams.statsType = pReq->stats_type;
    pFwReq->statsReqParams.clear = pReq->clear;
    pFwReq->statsReqParams.reserved = 0;

    mVendorData = (char*)pFwReq;
    mDataLen = message_len;

    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

wifi_error NanCommand::putNanTCA(transaction_id id, const NanTCARequest* pReq) {
    wifi_error ret;
    ALOGV("NAN_TCA");
    if (pReq == NULL) {
        cleanup();
        return WIFI_ERROR_INVALID_ARGS;
    }
    size_t message_len = sizeof(NanTcaReqMsg);

    message_len += (SIZEOF_TLV_HDR + 2 * sizeof(u32));
    pNanTcaReqMsg pFwReq = (pNanTcaReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ALOGV("Message Len %zu", message_len);
    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_TCA_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    pFwReq->fwHeader.transactionId = id;

    u32 tcaReqParams[2];
    memset(tcaReqParams, 0, sizeof(tcaReqParams));
    tcaReqParams[0] = (pReq->rising_direction_evt_flag & 0x01);
    tcaReqParams[0] |= (pReq->falling_direction_evt_flag & 0x01) << 1;
    tcaReqParams[0] |= (pReq->clear & 0x01) << 2;
    tcaReqParams[1] = pReq->threshold;

    u8* tlvs = pFwReq->ptlv;

    if (pReq->tca_type == NAN_TCA_ID_CLUSTER_SIZE) {
        tlvs = addTlv(NAN_TLV_TYPE_CLUSTER_SIZE_REQ, sizeof(tcaReqParams),
                      (const u8*)&tcaReqParams[0], tlvs);
    } else {
        ALOGE("%s: Unrecognized tca_type:%u", __FUNCTION__, pReq->tca_type);
        cleanup();
        return WIFI_ERROR_INVALID_ARGS;
    }

    mVendorData = (char*)pFwReq;
    mDataLen = message_len;

    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

wifi_error NanCommand::putNanBeaconSdfPayload(transaction_id id,
                                              const NanBeaconSdfPayloadRequest* pReq) {
    wifi_error ret;
    ALOGV("NAN_BEACON_SDF_PAYLAOD");
    if (pReq == NULL) {
        cleanup();
        return WIFI_ERROR_INVALID_ARGS;
    }
    size_t message_len = sizeof(NanMsgHeader) + SIZEOF_TLV_HDR + sizeof(u32) + pReq->vsa.vsa_len;

    pNanBeaconSdfPayloadReqMsg pFwReq = (pNanBeaconSdfPayloadReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ALOGV("Message Len %zu", message_len);
    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_BEACON_SDF_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    pFwReq->fwHeader.transactionId = id;

    /* Construct First 4 bytes of NanBeaconSdfPayloadReqMsg */
    u32 temp = 0;
    temp = pReq->vsa.payload_transmit_flag & 0x01;
    temp |= (pReq->vsa.tx_in_discovery_beacon & 0x01) << 1;
    temp |= (pReq->vsa.tx_in_sync_beacon & 0x01) << 2;
    temp |= (pReq->vsa.tx_in_service_discovery & 0x01) << 3;
    temp |= (pReq->vsa.vendor_oui & 0x00FFFFFF) << 8;

    int tlv_len = sizeof(u32) + pReq->vsa.vsa_len;
    u8* tempBuf = (u8*)malloc(tlv_len);
    if (tempBuf == NULL) {
        ALOGE("%s: Malloc failed", __func__);
        free(pFwReq);
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }
    memset(tempBuf, 0, tlv_len);
    memcpy(tempBuf, &temp, sizeof(u32));
    memcpy((tempBuf + sizeof(u32)), pReq->vsa.vsa, pReq->vsa.vsa_len);

    u8* tlvs = pFwReq->ptlv;

    /* Write the TLVs to the message. */
    tlvs = addTlv(NAN_TLV_TYPE_VENDOR_SPECIFIC_ATTRIBUTE_TRANSMIT, tlv_len, (const u8*)tempBuf,
                  tlvs);
    free(tempBuf);

    mVendorData = (char*)pFwReq;
    mDataLen = message_len;

    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

// callback handlers registered for nl message send
static int error_handler_nan(struct sockaddr_nl* nla, struct nlmsgerr* err, void* arg) {
    struct sockaddr_nl* tmp;
    int* ret = (int*)arg;
    tmp = nla;
    *ret = err->error;
    ALOGE("%s: Error code:%d (%s)", __func__, *ret, strerror(-(*ret)));
    return NL_STOP;
}

// callback handlers registered for nl message send
static int ack_handler_nan(struct nl_msg* msg, void* arg) {
    int* ret = (int*)arg;
    struct nl_msg* a;

    ALOGE("%s: called", __func__);
    a = msg;
    *ret = 0;
    return NL_STOP;
}

// callback handlers registered for nl message send
static int finish_handler_nan(struct nl_msg* msg, void* arg) {
    int* ret = (int*)arg;
    struct nl_msg* a;

    ALOGE("%s: called", __func__);
    a = msg;
    *ret = 0;
    return NL_SKIP;
}

// Override base class requestEvent and implement little differently here
// This will send the request message
// We dont wait for any response back in case of Nan as it is asynchronous
// thus no wait for condition.
wifi_error NanCommand::requestEvent() {
    wifi_error res;
    int status;
    struct nl_cb* cb;

    cb = nl_cb_alloc(NL_CB_DEFAULT);
    if (!cb) {
        ALOGE("%s: Callback allocation failed", __func__);
        res = WIFI_ERROR_OUT_OF_MEMORY;
        goto out;
    }

    if (!mInfo->cmd_sock) {
        ALOGE("%s: Command socket is null", __func__);
        res = WIFI_ERROR_OUT_OF_MEMORY;
        goto out;
    }

    /* send message */
    ALOGV("%s:Handle:%p Socket Value:%p", __func__, mInfo, mInfo->cmd_sock);
    status = nl_send_auto_complete(mInfo->cmd_sock, mMsg.getMessage());
    if (status < 0) {
        res = mapKernelErrortoWifiHalError(status);
        goto out;
    }

    status = 1;

    nl_cb_err(cb, NL_CB_CUSTOM, error_handler_nan, &status);
    nl_cb_set(cb, NL_CB_FINISH, NL_CB_CUSTOM, finish_handler_nan, &status);
    nl_cb_set(cb, NL_CB_ACK, NL_CB_CUSTOM, ack_handler_nan, &status);

    // err is populated as part of finish_handler
    while (status > 0) nl_recvmsgs(mInfo->cmd_sock, cb);

    res = mapKernelErrortoWifiHalError(status);
out:
    nl_cb_put(cb);
    // free the VendorData
    if (mVendorData) {
        free(mVendorData);
    }
    mVendorData = NULL;
    // cleanup the mMsg
    mMsg.destroy();
    return res;
}

int NanCommand::calcNanTransmitPostDiscoverySize(const NanTransmitPostDiscovery* pPostDiscovery) {
    /* Fixed size of u32 for Conn Type, Device Role and R flag + Dur + Rsvd*/
    int ret = sizeof(u32);
    /* size of availability interval bit map is 4 bytes */
    ret += sizeof(u32);
    /* size of mac address is 6 bytes*/
    ret += (SIZEOF_TLV_HDR + NAN_MAC_ADDR_LEN);
    if (pPostDiscovery && pPostDiscovery->type == NAN_CONN_WLAN_MESH) {
        /* size of WLAN_MESH_ID  */
        ret += (SIZEOF_TLV_HDR + pPostDiscovery->mesh_id_len);
    }
    if (pPostDiscovery && pPostDiscovery->type == NAN_CONN_WLAN_INFRA) {
        /* size of Infrastructure ssid  */
        ret += (SIZEOF_TLV_HDR + pPostDiscovery->infrastructure_ssid_len);
    }
    ALOGV("%s:size:%d", __func__, ret);
    return ret;
}

void NanCommand::fillNanSocialChannelParamVal(const NanSocialChannelScanParams* pScanParams,
                                              u32* pChannelParamArr) {
    int i;
    if (pChannelParamArr) {
        memset(pChannelParamArr, 0, NAN_MAX_SOCIAL_CHANNELS * sizeof(u32));
        for (i = 0; i < NAN_MAX_SOCIAL_CHANNELS; i++) {
            pChannelParamArr[i] = pScanParams->scan_period[i] << 16;
            pChannelParamArr[i] |= pScanParams->dwell_time[i] << 8;
        }
        pChannelParamArr[NAN_CHANNEL_24G_BAND] |= 6;
        pChannelParamArr[NAN_CHANNEL_5G_BAND_LOW] |= 44;
        pChannelParamArr[NAN_CHANNEL_5G_BAND_HIGH] |= 149;
        ALOGV("%s: Filled SocialChannelParamVal", __func__);
        hexdump((char*)pChannelParamArr, NAN_MAX_SOCIAL_CHANNELS * sizeof(u32));
    }
    return;
}

u32 NanCommand::getNanTransmitPostConnectivityCapabilityVal(
        const NanTransmitPostConnectivityCapability* pCapab) {
    u32 ret = 0;
    ret |= (pCapab->payload_transmit_flag ? 1 : 0) << 16;
    ret |= (pCapab->is_mesh_supported ? 1 : 0) << 5;
    ret |= (pCapab->is_ibss_supported ? 1 : 0) << 4;
    ret |= (pCapab->wlan_infra_field ? 1 : 0) << 3;
    ret |= (pCapab->is_tdls_supported ? 1 : 0) << 2;
    ret |= (pCapab->is_wfds_supported ? 1 : 0) << 1;
    ret |= (pCapab->is_wfd_supported ? 1 : 0);
    ALOGV("%s: val:%d", __func__, ret);
    return ret;
}

void NanCommand::fillNanTransmitPostDiscoveryVal(const NanTransmitPostDiscovery* pTxDisc,
                                                 u8* pOutValue) {
    if (pTxDisc && pOutValue) {
        u8* tlvs = &pOutValue[8];
        pOutValue[0] = pTxDisc->type;
        pOutValue[1] = pTxDisc->role;
        pOutValue[2] = (pTxDisc->transmit_freq ? 1 : 0);
        pOutValue[2] |= ((pTxDisc->duration & 0x03) << 1);
        memcpy(&pOutValue[4], &pTxDisc->avail_interval_bitmap,
               sizeof(pTxDisc->avail_interval_bitmap));
        tlvs = addTlv(NAN_TLV_TYPE_MAC_ADDRESS, NAN_MAC_ADDR_LEN, (const u8*)&pTxDisc->addr[0],
                      tlvs);
        if (pTxDisc->type == NAN_CONN_WLAN_MESH) {
            tlvs = addTlv(NAN_TLV_TYPE_WLAN_MESH_ID, pTxDisc->mesh_id_len,
                          (const u8*)&pTxDisc->mesh_id[0], tlvs);
        }
        if (pTxDisc->type == NAN_CONN_WLAN_INFRA) {
            tlvs = addTlv(NAN_TLV_TYPE_WLAN_INFRA_SSID, pTxDisc->infrastructure_ssid_len,
                          (const u8*)&pTxDisc->infrastructure_ssid_val[0], tlvs);
        }
        ALOGV("%s: Filled TransmitPostDiscoveryVal", __func__);
        hexdump((char*)pOutValue, calcNanTransmitPostDiscoverySize(pTxDisc));
    }

    return;
}

void NanCommand::fillNanFurtherAvailabilityMapVal(const NanFurtherAvailabilityMap* pFam,
                                                  u8* pOutValue) {
    int idx = 0;

    if (pFam && pOutValue) {
        u32 famsize = calcNanFurtherAvailabilityMapSize(pFam);
        pNanFurtherAvailabilityMapAttrTlv pFwReq = (pNanFurtherAvailabilityMapAttrTlv)pOutValue;

        memset(pOutValue, 0, famsize);
        pFwReq->numChan = pFam->numchans;
        for (idx = 0; idx < pFam->numchans; idx++) {
            const NanFurtherAvailabilityChannel* pFamChan = &pFam->famchan[idx];
            pNanFurtherAvailabilityChan pFwFamChan =
                    (pNanFurtherAvailabilityChan)((u8*)&pFwReq->pFaChan[0] +
                                                  (idx * sizeof(NanFurtherAvailabilityChan)));

            pFwFamChan->entryCtrl.availIntDuration = pFamChan->entry_control;
            pFwFamChan->entryCtrl.mapId = pFamChan->mapid;
            pFwFamChan->opClass = pFamChan->class_val;
            pFwFamChan->channel = pFamChan->channel;
            memcpy(&pFwFamChan->availIntBitmap, &pFamChan->avail_interval_bitmap,
                   sizeof(pFwFamChan->availIntBitmap));
        }
        ALOGV("%s: Filled FurtherAvailabilityMapVal", __func__);
        hexdump((char*)pOutValue, famsize);
    }
    return;
}

int NanCommand::calcNanFurtherAvailabilityMapSize(const NanFurtherAvailabilityMap* pFam) {
    int ret = 0;
    if (pFam && pFam->numchans && pFam->numchans <= NAN_MAX_FAM_CHANNELS) {
        /* Fixed size of u8 for numchans*/
        ret = sizeof(u8);
        /* numchans * sizeof(FamChannels) */
        ret += (pFam->numchans * sizeof(NanFurtherAvailabilityChan));
    }
    ALOGV("%s:size:%d", __func__, ret);
    return ret;
}

wifi_error NanCommand::putNanCapabilities(transaction_id id) {
    wifi_error ret;
    ALOGV("NAN_CAPABILITIES");
    size_t message_len = sizeof(NanCapabilitiesReqMsg);

    pNanCapabilitiesReqMsg pFwReq = (pNanCapabilitiesReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_CAPABILITIES_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    pFwReq->fwHeader.transactionId = id;

    mVendorData = (char*)pFwReq;
    mDataLen = message_len;

    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

wifi_error NanCommand::putNanDebugCommand(NanDebugParams debug, int debug_msg_length) {
    wifi_error ret;
    ALOGV("NAN_AVAILABILITY_DEBUG");
    size_t message_len = sizeof(NanTestModeReqMsg);

    message_len += (SIZEOF_TLV_HDR + debug_msg_length);
    pNanTestModeReqMsg pFwReq = (pNanTestModeReqMsg)malloc(message_len);
    if (pFwReq == NULL) {
        cleanup();
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ALOGV("Message Len %zu\n", message_len);
    ALOGV("%s: Debug Command Type = 0x%x \n", __func__, debug.cmd);
    ALOGV("%s: ** Debug Command Data Start **", __func__);
    hexdump(debug.debug_cmd_data, debug_msg_length);
    ALOGV("%s: ** Debug Command Data End **", __func__);

    memset(pFwReq, 0, message_len);
    pFwReq->fwHeader.msgVersion = (u16)NAN_MSG_VERSION1;
    pFwReq->fwHeader.msgId = NAN_MSG_ID_TESTMODE_REQ;
    pFwReq->fwHeader.msgLen = message_len;
    pFwReq->fwHeader.transactionId = 0;

    u8* tlvs = pFwReq->ptlv;
    tlvs = addTlv(NAN_TLV_TYPE_TESTMODE_GENERIC_CMD, debug_msg_length, (const u8*)&debug, tlvs);

    mVendorData = (char*)pFwReq;
    mDataLen = message_len;

    /* Write the TLVs to the message. */
    ret = put_bytes(NL80211_ATTR_VENDOR_DATA, mVendorData, mDataLen);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: put_bytes Error:%d", __func__, ret);
        cleanup();
        return ret;
    }
    hexdump(mVendorData, mDataLen);
    return ret;
}

// Function which calls the necessaryIndication callback
// based on the indication type
int NanCommand::handleNanIndication() {
    // Based on the message_id in the header determine the Indication type
    // and call the necessary callback handler
    u16 msg_id;
    int res = 0;

    msg_id = getIndicationType();

    ALOGV("handleNanIndication msg_id:%u", msg_id);
    switch (msg_id) {
        case NAN_INDICATION_PUBLISH_REPLIED:
            NanPublishRepliedInd publishRepliedInd;
            memset(&publishRepliedInd, 0, sizeof(publishRepliedInd));
            res = getNanPublishReplied(&publishRepliedInd);
            if (!res && mHandler.EventPublishReplied) {
                (*mHandler.EventPublishReplied)(&publishRepliedInd);
            }
            break;

        case NAN_INDICATION_PUBLISH_TERMINATED:
            NanPublishTerminatedInd publishTerminatedInd;
            memset(&publishTerminatedInd, 0, sizeof(publishTerminatedInd));
            res = getNanPublishTerminated(&publishTerminatedInd);
            if (!res && mHandler.EventPublishTerminated) {
                (*mHandler.EventPublishTerminated)(&publishTerminatedInd);
            }
            break;

        case NAN_INDICATION_MATCH:
            NanMatchInd matchInd;
            memset(&matchInd, 0, sizeof(matchInd));
            res = getNanMatch(&matchInd);
            if (!res && mHandler.EventMatch) {
                (*mHandler.EventMatch)(&matchInd);
            }
            break;

        case NAN_INDICATION_MATCH_EXPIRED:
            NanMatchExpiredInd matchExpiredInd;
            memset(&matchExpiredInd, 0, sizeof(matchExpiredInd));
            res = getNanMatchExpired(&matchExpiredInd);
            if (!res && mHandler.EventMatchExpired) {
                (*mHandler.EventMatchExpired)(&matchExpiredInd);
            }
            break;

        case NAN_INDICATION_SUBSCRIBE_TERMINATED:
            NanSubscribeTerminatedInd subscribeTerminatedInd;
            memset(&subscribeTerminatedInd, 0, sizeof(subscribeTerminatedInd));
            res = getNanSubscribeTerminated(&subscribeTerminatedInd);
            if (!res && mHandler.EventSubscribeTerminated) {
                (*mHandler.EventSubscribeTerminated)(&subscribeTerminatedInd);
            }
            break;

        case NAN_INDICATION_DE_EVENT:
            NanDiscEngEventInd discEngEventInd;
            memset(&discEngEventInd, 0, sizeof(discEngEventInd));
            res = getNanDiscEngEvent(&discEngEventInd);
            if (!res && mHandler.EventDiscEngEvent) {
                (*mHandler.EventDiscEngEvent)(&discEngEventInd);
            }
            break;

        case NAN_INDICATION_FOLLOWUP:
            NanFollowupInd followupInd;
            memset(&followupInd, 0, sizeof(followupInd));
            res = getNanFollowup(&followupInd);
            if (!res && mHandler.EventFollowup) {
                (*mHandler.EventFollowup)(&followupInd);
            }
            break;

        case NAN_INDICATION_DISABLED:
            NanDisabledInd disabledInd;
            memset(&disabledInd, 0, sizeof(disabledInd));
            res = getNanDisabled(&disabledInd);
            if (!res && mHandler.EventDisabled) {
                (*mHandler.EventDisabled)(&disabledInd);
            }
            break;

        case NAN_INDICATION_TCA:
            NanTCAInd tcaInd;
            memset(&tcaInd, 0, sizeof(tcaInd));
            res = getNanTca(&tcaInd);
            if (!res && mHandler.EventTca) {
                (*mHandler.EventTca)(&tcaInd);
            }
            break;

        case NAN_INDICATION_BEACON_SDF_PAYLOAD:
            NanBeaconSdfPayloadInd beaconSdfPayloadInd;
            memset(&beaconSdfPayloadInd, 0, sizeof(beaconSdfPayloadInd));
            res = getNanBeaconSdfPayload(&beaconSdfPayloadInd);
            if (!res && mHandler.EventBeaconSdfPayload) {
                (*mHandler.EventBeaconSdfPayload)(&beaconSdfPayloadInd);
            }
            break;

        case NAN_INDICATION_SELF_TRANSMIT_FOLLOWUP:
            NanTransmitFollowupInd transmitFollowupInd;
            memset(&transmitFollowupInd, 0, sizeof(NanTransmitFollowupInd));
            res = getNanTransmitFollowupInd(&transmitFollowupInd);
            if (!res && mHandler.EventTransmitFollowup) {
                (*mHandler.EventTransmitFollowup)(&transmitFollowupInd);
            }
            break;

        case NAN_INDICATION_RANGING_REQUEST_RECEIVED:
            NanRangeRequestInd rangeRequestInd;
            memset(&rangeRequestInd, 0, sizeof(NanRangeRequestInd));
            res = getNanRangeRequestReceivedInd(&rangeRequestInd);
            if (!res && mHandler.EventRangeRequest) {
                (*mHandler.EventRangeRequest)(&rangeRequestInd);
            }
            break;

        case NAN_INDICATION_RANGING_RESULT:
            NanRangeReportInd rangeReportInd;
            memset(&rangeReportInd, 0, sizeof(NanRangeReportInd));
            res = getNanRangeReportInd(&rangeReportInd);
            if (!res && mHandler.EventRangeReport) {
                (*mHandler.EventRangeReport)(&rangeReportInd);
            }
            break;

        default:
            ALOGE("handleNanIndication error invalid msg_id:%u", msg_id);
            res = (int)WIFI_ERROR_INVALID_REQUEST_ID;
            break;
    }
    return res;
}

// Function which will return the Nan Indication type based on
// the initial few bytes of mNanVendorEvent
NanIndicationType NanCommand::getIndicationType() {
    if (mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid argument mNanVendorEvent:%p", __func__, mNanVendorEvent);
        return NAN_INDICATION_UNKNOWN;
    }

    NanMsgHeader* pHeader = (NanMsgHeader*)mNanVendorEvent;

    switch (pHeader->msgId) {
        case NAN_MSG_ID_PUBLISH_REPLIED_IND:
            return NAN_INDICATION_PUBLISH_REPLIED;
        case NAN_MSG_ID_PUBLISH_TERMINATED_IND:
            return NAN_INDICATION_PUBLISH_TERMINATED;
        case NAN_MSG_ID_MATCH_IND:
            return NAN_INDICATION_MATCH;
        case NAN_MSG_ID_MATCH_EXPIRED_IND:
            return NAN_INDICATION_MATCH_EXPIRED;
        case NAN_MSG_ID_FOLLOWUP_IND:
            return NAN_INDICATION_FOLLOWUP;
        case NAN_MSG_ID_SUBSCRIBE_TERMINATED_IND:
            return NAN_INDICATION_SUBSCRIBE_TERMINATED;
        case NAN_MSG_ID_DE_EVENT_IND:
            return NAN_INDICATION_DE_EVENT;
        case NAN_MSG_ID_DISABLE_IND:
            return NAN_INDICATION_DISABLED;
        case NAN_MSG_ID_TCA_IND:
            return NAN_INDICATION_TCA;
        case NAN_MSG_ID_BEACON_SDF_IND:
            return NAN_INDICATION_BEACON_SDF_PAYLOAD;
        case NAN_MSG_ID_SELF_TRANSMIT_FOLLOWUP_IND:
            return NAN_INDICATION_SELF_TRANSMIT_FOLLOWUP;
        case NAN_MSG_ID_RANGING_REQUEST_RECEVD_IND:
            return NAN_INDICATION_RANGING_REQUEST_RECEIVED;
        case NAN_MSG_ID_RANGING_RESULT_IND:
            return NAN_INDICATION_RANGING_RESULT;
        default:
            return NAN_INDICATION_UNKNOWN;
    }
}

int NanCommand::getNanPublishReplied(NanPublishRepliedInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanPublishRepliedIndMsg pRsp = (pNanPublishRepliedIndMsg)mNanVendorEvent;
    event->requestor_instance_id = pRsp->publishRepliedIndParams.matchHandle;

    event->rssi_value = 0;
    u8* pInputTlv = pRsp->ptlv;
    NanTlv outputTlv;
    u16 readLen = 0;
    int remainingLen = (mNanDataLen - (sizeof(NanMsgHeader)));

    if (remainingLen <= 0) {
        ALOGI("%s: No TLV's present", __func__);
        return WIFI_SUCCESS;
    }
    while ((remainingLen > 0) && (0 != (readLen = NANTLV_ReadTlv(pInputTlv, &outputTlv)))) {
        switch (outputTlv.type) {
            case NAN_TLV_TYPE_MAC_ADDRESS:
                if (outputTlv.length > sizeof(event->addr)) {
                    outputTlv.length = sizeof(event->addr);
                }
                memcpy(event->addr, outputTlv.value, outputTlv.length);
                break;
            case NAN_TLV_TYPE_RECEIVED_RSSI_VALUE:
                if (outputTlv.length > sizeof(event->rssi_value)) {
                    outputTlv.length = sizeof(event->rssi_value);
                }
                memcpy(&event->rssi_value, outputTlv.value, outputTlv.length);
                break;
            default:
                ALOGI("Unknown TLV type skipped");
                break;
        }
        remainingLen -= readLen;
        pInputTlv += readLen;
        memset(&outputTlv, 0, sizeof(outputTlv));
    }
    return WIFI_SUCCESS;
}

int NanCommand::getNanPublishTerminated(NanPublishTerminatedInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanPublishTerminatedIndMsg pRsp = (pNanPublishTerminatedIndMsg)mNanVendorEvent;
    event->publish_id = pRsp->fwHeader.handle;
    NanErrorTranslation((NanInternalStatusType)pRsp->reason, 0, (void*)event, false);
    return WIFI_SUCCESS;
}

int NanCommand::getNanMatch(NanMatchInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanMatchIndMsg pRsp = (pNanMatchIndMsg)mNanVendorEvent;
    event->publish_subscribe_id = pRsp->fwHeader.handle;
    event->requestor_instance_id = pRsp->matchIndParams.matchHandle;
    event->match_occured_flag = pRsp->matchIndParams.matchOccuredFlag;
    event->out_of_resource_flag = pRsp->matchIndParams.outOfResourceFlag;

    u8* pInputTlv = pRsp->ptlv;
    NanTlv outputTlv;
    u16 readLen = 0;
    int remainingLen = (mNanDataLen - (sizeof(NanMsgHeader) + sizeof(NanMatchIndParams)));
    int ret = 0, idx = 0;

    // Has SDF match filter and service specific info TLV
    if (remainingLen <= 0) {
        ALOGV("%s: No TLV's present", __func__);
        return WIFI_SUCCESS;
    }
    ALOGV("%s: TLV remaining Len:%d", __func__, remainingLen);
    while ((remainingLen > 0) && (0 != (readLen = NANTLV_ReadTlv(pInputTlv, &outputTlv)))) {
        ALOGV("%s: Remaining Len:%d readLen:%d type:%d length:%d", __func__, remainingLen, readLen,
              outputTlv.type, outputTlv.length);
        switch (outputTlv.type) {
            case NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO:
                if (outputTlv.length > NAN_MAX_SERVICE_NAME_LEN) {
                    outputTlv.length = NAN_MAX_SERVICE_NAME_LEN;
                }
                event->service_specific_info_len = outputTlv.length;
                memcpy(event->service_specific_info, outputTlv.value, outputTlv.length);
                break;
            case NAN_TLV_TYPE_SDF_MATCH_FILTER:
                if (outputTlv.length > NAN_MAX_MATCH_FILTER_LEN) {
                    outputTlv.length = NAN_MAX_MATCH_FILTER_LEN;
                }
                event->sdf_match_filter_len = outputTlv.length;
                memcpy(event->sdf_match_filter, outputTlv.value, outputTlv.length);
                break;
            case NAN_TLV_TYPE_MAC_ADDRESS:
                if (outputTlv.length > sizeof(event->addr)) {
                    outputTlv.length = sizeof(event->addr);
                }
                memcpy(event->addr, outputTlv.value, outputTlv.length);
                break;
            case NAN_TLV_TYPE_RECEIVED_RSSI_VALUE:
                if (outputTlv.length > sizeof(event->rssi_value)) {
                    outputTlv.length = sizeof(event->rssi_value);
                }
                memcpy(&event->rssi_value, outputTlv.value, outputTlv.length);
                break;
            case NAN_TLV_TYPE_POST_NAN_CONNECTIVITY_CAPABILITIES_RECEIVE:
                if (outputTlv.length != sizeof(u32)) {
                    ALOGE("NAN_TLV_TYPE_POST_NAN_CONNECTIVITY_CAPABILITIES_RECEIVE"
                          "Incorrect size:%d expecting %zu",
                          outputTlv.length, sizeof(u32));
                    break;
                }
                event->is_conn_capability_valid = 1;
                /* Populate conn_capability from received TLV */
                getNanReceivePostConnectivityCapabilityVal(outputTlv.value,
                                                           &event->conn_capability);
                break;
            case NAN_TLV_TYPE_POST_NAN_DISCOVERY_ATTRIBUTE_RECEIVE:
                /* Populate receive discovery attribute from
                   received TLV */
                idx = event->num_rx_discovery_attr;
                if (idx < 0 || idx >= NAN_MAX_POSTDISCOVERY_LEN) {
                    ALOGE("NAN_TLV_TYPE_POST_NAN_DISCOVERY_ATTRIBUTE_RECEIVE"
                          " Incorrect index:%d >= %d",
                          idx, NAN_MAX_POSTDISCOVERY_LEN);
                    break;
                }
                ret = getNanReceivePostDiscoveryVal(outputTlv.value, outputTlv.length,
                                                    &event->discovery_attr[idx]);
                if (ret == 0) {
                    event->num_rx_discovery_attr++;
                } else {
                    ALOGE("NAN_TLV_TYPE_POST_NAN_DISCOVERY_ATTRIBUTE_RECEIVE"
                          "Incorrect");
                }
                break;
            case NAN_TLV_TYPE_FURTHER_AVAILABILITY_MAP:
                /* Populate further availability bitmap from
                   received TLV */
                ret = getNanFurtherAvailabilityMap(outputTlv.value, outputTlv.length,
                                                   &event->num_chans, &event->famchan[0]);
                if (ret < 0)
                    ALOGE("NAN_TLV_TYPE_FURTHER_AVAILABILITY_MAP"
                          "Incorrect");
                break;
            case NAN_TLV_TYPE_CLUSTER_ATTRIBUTE:
                if (outputTlv.length > sizeof(event->cluster_attribute)) {
                    outputTlv.length = sizeof(event->cluster_attribute);
                }
                memcpy(event->cluster_attribute, outputTlv.value, outputTlv.length);
                event->cluster_attribute_len = outputTlv.length;
                break;
            case NAN_TLV_TYPE_NAN_CSID:
                if (outputTlv.length > sizeof(event->peer_cipher_type)) {
                    outputTlv.length = sizeof(event->peer_cipher_type);
                }
                memcpy(&event->peer_cipher_type, outputTlv.value, outputTlv.length);
                break;
            case NAN_TLV_TYPE_NAN_SCID:
                if (outputTlv.length > sizeof(event->scid)) {
                    outputTlv.length = sizeof(event->scid);
                }
                event->scid_len = outputTlv.length;
                memcpy(event->scid, outputTlv.value, outputTlv.length);
                break;
            case NAN_TLV_TYPE_SDEA_CTRL_PARAMS:
                if (outputTlv.length != sizeof(u32)) {
                    ALOGE("NAN_TLV_TYPE_SDEA_CTRL_PARAMS"
                          "Incorrect size:%d expecting %zu",
                          outputTlv.length, sizeof(u32));
                    break;
                }
                getNanReceiveSdeaCtrlParams(outputTlv.value, &event->peer_sdea_params);
                break;
            case NAN_TLV_TYPE_NAN20_RANGING_RESULT:
                if (outputTlv.length > sizeof(event->range_info)) {
                    outputTlv.length = sizeof(event->range_info);
                }
                memcpy(&event->range_info, outputTlv.value, outputTlv.length);
                break;
            case NAN_TLV_TYPE_SDEA_SERVICE_SPECIFIC_INFO:
                if (outputTlv.length > NAN_MAX_SDEA_SERVICE_SPECIFIC_INFO_LEN) {
                    outputTlv.length = NAN_MAX_SDEA_SERVICE_SPECIFIC_INFO_LEN;
                }
                event->sdea_service_specific_info_len = outputTlv.length;
                memcpy(event->sdea_service_specific_info, outputTlv.value, outputTlv.length);
                break;
            default:
                ALOGV("Unknown TLV type skipped");
                break;
        }
        remainingLen -= readLen;
        pInputTlv += readLen;
        memset(&outputTlv, 0, sizeof(outputTlv));
    }
    return WIFI_SUCCESS;
}

int NanCommand::getNanMatchExpired(NanMatchExpiredInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanMatchExpiredIndMsg pRsp = (pNanMatchExpiredIndMsg)mNanVendorEvent;
    event->publish_subscribe_id = pRsp->fwHeader.handle;
    event->requestor_instance_id = pRsp->matchExpiredIndParams.matchHandle;
    return WIFI_SUCCESS;
}

int NanCommand::getNanSubscribeTerminated(NanSubscribeTerminatedInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanSubscribeTerminatedIndMsg pRsp = (pNanSubscribeTerminatedIndMsg)mNanVendorEvent;
    event->subscribe_id = pRsp->fwHeader.handle;
    NanErrorTranslation((NanInternalStatusType)pRsp->reason, 0, (void*)event, false);
    return WIFI_SUCCESS;
}

int NanCommand::getNanFollowup(NanFollowupInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanFollowupIndMsg pRsp = (pNanFollowupIndMsg)mNanVendorEvent;
    event->publish_subscribe_id = pRsp->fwHeader.handle;
    event->requestor_instance_id = pRsp->followupIndParams.matchHandle;
    event->dw_or_faw = pRsp->followupIndParams.window;

    u8* pInputTlv = pRsp->ptlv;
    NanTlv outputTlv;
    u16 readLen = 0;
    int remainingLen = (mNanDataLen - (sizeof(NanMsgHeader) + sizeof(NanFollowupIndParams)));

    // Has service specific info and extended service specific info TLV
    if (remainingLen <= 0) {
        ALOGV("%s: No TLV's present", __func__);
        return WIFI_SUCCESS;
    }
    ALOGV("%s: TLV remaining Len:%d", __func__, remainingLen);
    while ((remainingLen > 0) && (0 != (readLen = NANTLV_ReadTlv(pInputTlv, &outputTlv)))) {
        ALOGV("%s: Remaining Len:%d readLen:%d type:%d length:%d", __func__, remainingLen, readLen,
              outputTlv.type, outputTlv.length);
        switch (outputTlv.type) {
            case NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO:
            case NAN_TLV_TYPE_EXT_SERVICE_SPECIFIC_INFO:
                if (outputTlv.length > NAN_MAX_SERVICE_SPECIFIC_INFO_LEN) {
                    outputTlv.length = NAN_MAX_SERVICE_SPECIFIC_INFO_LEN;
                }
                event->service_specific_info_len = outputTlv.length;
                memcpy(event->service_specific_info, outputTlv.value, outputTlv.length);
                break;
            case NAN_TLV_TYPE_MAC_ADDRESS:
                if (outputTlv.length > sizeof(event->addr)) {
                    outputTlv.length = sizeof(event->addr);
                }
                memcpy(event->addr, outputTlv.value, outputTlv.length);
                break;
            case NAN_TLV_TYPE_SDEA_SERVICE_SPECIFIC_INFO:
                if (outputTlv.length > NAN_MAX_SDEA_SERVICE_SPECIFIC_INFO_LEN) {
                    outputTlv.length = NAN_MAX_SDEA_SERVICE_SPECIFIC_INFO_LEN;
                }
                event->sdea_service_specific_info_len = outputTlv.length;
                memcpy(event->sdea_service_specific_info, outputTlv.value, outputTlv.length);
                break;
            default:
                ALOGV("Unknown TLV type skipped");
                break;
        }
        remainingLen -= readLen;
        pInputTlv += readLen;
        memset(&outputTlv, 0, sizeof(outputTlv));
    }
    return WIFI_SUCCESS;
}

int NanCommand::getNanDiscEngEvent(NanDiscEngEventInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanEventIndMsg pRsp = (pNanEventIndMsg)mNanVendorEvent;
    memset(&event->data, 0, sizeof(event->data));

    u8* pInputTlv = pRsp->ptlv;
    NanTlv outputTlv;
    u16 readLen = 0;
    int remainingLen = (mNanDataLen - (sizeof(NanMsgHeader)));

    // Has Self-STA Mac TLV
    if (remainingLen <= 0) {
        ALOGE("%s: No TLV's present", __func__);
        return WIFI_SUCCESS;
    }

    ALOGV("%s: TLV remaining Len:%d", __func__, remainingLen);
    while ((remainingLen > 0) && (0 != (readLen = NANTLV_ReadTlv(pInputTlv, &outputTlv)))) {
        ALOGV("%s: Remaining Len:%d readLen:%d type:%d length:%d", __func__, remainingLen, readLen,
              outputTlv.type, outputTlv.length);
        switch (outputTlv.type) {
            case NAN_TLV_TYPE_EVENT_SELF_STATION_MAC_ADDRESS:
                if (outputTlv.length > NAN_MAC_ADDR_LEN) {
                    ALOGV("%s: Reading only first %d bytes of TLV", __func__, NAN_MAC_ADDR_LEN);
                    outputTlv.length = NAN_MAC_ADDR_LEN;
                }
                memcpy(event->data.mac_addr.addr, outputTlv.value, outputTlv.length);
                event->event_type = NAN_EVENT_ID_DISC_MAC_ADDR;
                break;
            case NAN_TLV_TYPE_EVENT_STARTED_CLUSTER:
                if (outputTlv.length > NAN_MAC_ADDR_LEN) {
                    ALOGV("%s: Reading only first %d bytes of TLV", __func__, NAN_MAC_ADDR_LEN);
                    outputTlv.length = NAN_MAC_ADDR_LEN;
                }
                memcpy(event->data.cluster.addr, outputTlv.value, outputTlv.length);
                event->event_type = NAN_EVENT_ID_STARTED_CLUSTER;
                break;
            case NAN_TLV_TYPE_EVENT_JOINED_CLUSTER:
                if (outputTlv.length > NAN_MAC_ADDR_LEN) {
                    ALOGV("%s: Reading only first %d bytes of TLV", __func__, NAN_MAC_ADDR_LEN);
                    outputTlv.length = NAN_MAC_ADDR_LEN;
                }
                memcpy(event->data.cluster.addr, outputTlv.value, outputTlv.length);
                event->event_type = NAN_EVENT_ID_JOINED_CLUSTER;
                break;
            default:
                ALOGV("Unhandled TLV type:%d", outputTlv.type);
                break;
        }
        remainingLen -= readLen;
        pInputTlv += readLen;
        memset(&outputTlv, 0, sizeof(outputTlv));
    }
    return WIFI_SUCCESS;
}

int NanCommand::getNanDisabled(NanDisabledInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanDisableIndMsg pRsp = (pNanDisableIndMsg)mNanVendorEvent;
    NanErrorTranslation((NanInternalStatusType)pRsp->reason, 0, (void*)event, false);
    return WIFI_SUCCESS;
}

int NanCommand::getNanTca(NanTCAInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanTcaIndMsg pRsp = (pNanTcaIndMsg)mNanVendorEvent;
    memset(&event->data, 0, sizeof(event->data));

    u8* pInputTlv = pRsp->ptlv;
    NanTlv outputTlv;
    u16 readLen = 0;

    int remainingLen = (mNanDataLen - (sizeof(NanMsgHeader)));

    // Has NAN_TCA_ID_CLUSTER_SIZE
    if (remainingLen <= 0) {
        ALOGE("%s: No TLV's present", __func__);
        return WIFI_SUCCESS;
    }

    ALOGV("%s: TLV remaining Len:%d", __func__, remainingLen);
    while ((remainingLen > 0) && (0 != (readLen = NANTLV_ReadTlv(pInputTlv, &outputTlv)))) {
        ALOGV("%s: Remaining Len:%d readLen:%d type:%d length:%d", __func__, remainingLen, readLen,
              outputTlv.type, outputTlv.length);
        switch (outputTlv.type) {
            case NAN_TLV_TYPE_CLUSTER_SIZE_RSP:
                if (outputTlv.length != 2 * sizeof(u32)) {
                    ALOGE("%s: Wrong length %d in Tca Indication expecting %zu bytes", __func__,
                          outputTlv.length, 2 * sizeof(u32));
                    break;
                }
                event->rising_direction_evt_flag = outputTlv.value[0] & 0x01;
                event->falling_direction_evt_flag = (outputTlv.value[0] & 0x02) >> 1;
                memcpy(&(event->data.cluster.cluster_size), &outputTlv.value[4],
                       sizeof(event->data.cluster.cluster_size));
                event->tca_type = NAN_TCA_ID_CLUSTER_SIZE;
                break;
            default:
                ALOGV("Unhandled TLV type:%d", outputTlv.type);
                break;
        }
        remainingLen -= readLen;
        pInputTlv += readLen;
        memset(&outputTlv, 0, sizeof(outputTlv));
    }
    return WIFI_SUCCESS;
}

int NanCommand::getNanBeaconSdfPayload(NanBeaconSdfPayloadInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanBeaconSdfPayloadIndMsg pRsp = (pNanBeaconSdfPayloadIndMsg)mNanVendorEvent;
    memset(&event->data, 0, sizeof(event->data));

    u8* pInputTlv = pRsp->ptlv;
    NanTlv outputTlv;
    u16 readLen = 0;
    int remainingLen = (mNanDataLen - (sizeof(NanMsgHeader)));

    // Has Mac address
    if (remainingLen <= 0) {
        ALOGV("%s: No TLV's present", __func__);
        return WIFI_SUCCESS;
    }

    ALOGV("%s: TLV remaining Len:%d", __func__, remainingLen);
    while ((remainingLen > 0) && (0 != (readLen = NANTLV_ReadTlv(pInputTlv, &outputTlv)))) {
        ALOGV("%s: Remaining Len:%d readLen:%d type:%d length:%d", __func__, remainingLen, readLen,
              outputTlv.type, outputTlv.length);
        switch (outputTlv.type) {
            case NAN_TLV_TYPE_MAC_ADDRESS:
                if (outputTlv.length > sizeof(event->addr)) {
                    outputTlv.length = sizeof(event->addr);
                }
                memcpy(event->addr, outputTlv.value, outputTlv.length);
                break;

            case NAN_TLV_TYPE_VENDOR_SPECIFIC_ATTRIBUTE_RECEIVE: {
                NanReceiveVendorSpecificAttribute* recvVsaattr = &event->vsa;
                if (outputTlv.length < sizeof(u32)) {
                    ALOGE("NAN_TLV_TYPE_VENDOR_SPECIFIC_ATTRIBUTE_RECEIVE"
                          "Incorrect length:%d",
                          outputTlv.length);
                    break;
                }
                event->is_vsa_received = 1;
                recvVsaattr->vsa_received_on = (outputTlv.value[0] >> 1) & 0x07;
                memcpy(&recvVsaattr->vendor_oui, &outputTlv.value[1], 3);
                recvVsaattr->attr_len = outputTlv.length - 4;
                if (recvVsaattr->attr_len > NAN_MAX_VSA_DATA_LEN) {
                    recvVsaattr->attr_len = NAN_MAX_VSA_DATA_LEN;
                }
                if (recvVsaattr->attr_len) {
                    memcpy(recvVsaattr->vsa, &outputTlv.value[4], recvVsaattr->attr_len);
                }
                break;
            }

            case NAN_TLV_TYPE_BEACON_SDF_PAYLOAD_RECEIVE:
                event->is_beacon_sdf_payload_received = 1;
                event->data.frame_len = outputTlv.length;
                if (event->data.frame_len > NAN_MAX_FRAME_DATA_LEN) {
                    event->data.frame_len = NAN_MAX_FRAME_DATA_LEN;
                }
                memcpy(&event->data.frame_data, &outputTlv.value[0], event->data.frame_len);
                break;

            default:
                ALOGV("Unhandled TLV Type:%d", outputTlv.type);
                break;
        }
        remainingLen -= readLen;
        pInputTlv += readLen;
        memset(&outputTlv, 0, sizeof(outputTlv));
    }
    return WIFI_SUCCESS;
}

void NanCommand::getNanReceivePostConnectivityCapabilityVal(
        const u8* pInValue, NanReceivePostConnectivityCapability* pRxCapab) {
    if (pInValue && pRxCapab) {
        pRxCapab->is_mesh_supported = (pInValue[0] & (0x01 << 5));
        pRxCapab->is_ibss_supported = (pInValue[0] & (0x01 << 4));
        pRxCapab->wlan_infra_field = (pInValue[0] & (0x01 << 3));
        pRxCapab->is_tdls_supported = (pInValue[0] & (0x01 << 2));
        pRxCapab->is_wfds_supported = (pInValue[0] & (0x01 << 1));
        pRxCapab->is_wfd_supported = pInValue[0] & 0x01;
    }
}

void NanCommand::getNanReceiveSdeaCtrlParams(const u8* pInValue,
                                             NanSdeaCtrlParams* pPeerSdeaParams) {
    if (pInValue && pPeerSdeaParams) {
        pPeerSdeaParams->security_cfg =
                (NanDataPathSecurityCfgStatus)((pInValue[0] & BIT_6) ? NAN_DP_CONFIG_SECURITY
                                                                     : NAN_DP_CONFIG_NO_SECURITY);
        pPeerSdeaParams->ranging_state =
                (NanRangingState)((pInValue[0] & BIT_7) ? NAN_RANGING_ENABLE : NAN_RANGING_DISABLE);
#if 0
        pPeerSdeaParams->enable_ranging_limit =
                         (NanRangingLimitState)((pInValue[0] & BIT_8) ?
                          NAN_RANGING_LIMIT_ENABLE : NAN_RANGING_LIMIT_DISABLE);
#endif
    }
    return;
}

int NanCommand::getNanReceivePostDiscoveryVal(const u8* pInValue, u32 length,
                                              NanReceivePostDiscovery* pRxDisc) {
    int ret = 0;

    if (length <= 8 || pInValue == NULL) {
        ALOGE("%s: Invalid Arg TLV Len %d < 4", __func__, length);
        return -1;
    }

    pRxDisc->type = (NanConnectionType)pInValue[0];
    pRxDisc->role = (NanDeviceRole)pInValue[1];
    pRxDisc->duration = (NanAvailDuration)(pInValue[2] & 0x03);
    pRxDisc->mapid = ((pInValue[2] >> 2) & 0x0F);
    memcpy(&pRxDisc->avail_interval_bitmap, &pInValue[4], sizeof(pRxDisc->avail_interval_bitmap));

    u8* pInputTlv = (u8*)&pInValue[8];
    NanTlv outputTlv;
    u16 readLen = 0;
    int remainingLen = (length - 8);

    // Has Mac address
    if (remainingLen <= 0) {
        ALOGE("%s: No TLV's present", __func__);
        return -1;
    }

    ALOGV("%s: TLV remaining Len:%d", __func__, remainingLen);
    while ((remainingLen > 0) && (0 != (readLen = NANTLV_ReadTlv(pInputTlv, &outputTlv)))) {
        ALOGV("%s: Remaining Len:%d readLen:%d type:%d length:%d", __func__, remainingLen, readLen,
              outputTlv.type, outputTlv.length);
        switch (outputTlv.type) {
            case NAN_TLV_TYPE_MAC_ADDRESS:
                if (outputTlv.length > sizeof(pRxDisc->addr)) {
                    outputTlv.length = sizeof(pRxDisc->addr);
                }
                memcpy(pRxDisc->addr, outputTlv.value, outputTlv.length);
                break;
            case NAN_TLV_TYPE_WLAN_MESH_ID:
                if (outputTlv.length > sizeof(pRxDisc->mesh_id)) {
                    outputTlv.length = sizeof(pRxDisc->mesh_id);
                }
                memcpy(pRxDisc->mesh_id, outputTlv.value, outputTlv.length);
                pRxDisc->mesh_id_len = outputTlv.length;
                break;
            case NAN_TLV_TYPE_WLAN_INFRA_SSID:
                if (outputTlv.length > sizeof(pRxDisc->infrastructure_ssid_val)) {
                    outputTlv.length = sizeof(pRxDisc->infrastructure_ssid_val);
                }
                memcpy(pRxDisc->infrastructure_ssid_val, outputTlv.value, outputTlv.length);
                pRxDisc->infrastructure_ssid_len = outputTlv.length;
                break;
            default:
                ALOGV("Unhandled TLV Type:%d", outputTlv.type);
                break;
        }
        remainingLen -= readLen;
        pInputTlv += readLen;
        memset(&outputTlv, 0, sizeof(outputTlv));
    }
    return ret;
}

int NanCommand::getNanFurtherAvailabilityMap(const u8* pInValue, u32 length, u8* num_chans,
                                             NanFurtherAvailabilityChannel* pFac) {
    int idx = 0;

    if ((length == 0) || pInValue == NULL) {
        ALOGE("%s: Invalid Arg TLV Len %d or pInValue NULL", __func__, length);
        return -1;
    }

    *num_chans = pInValue[0];
    if (*num_chans > NAN_MAX_FAM_CHANNELS) {
        ALOGE("%s: Unable to accommodate numchans %d", __func__, *num_chans);
        return -1;
    }

    if (length < (sizeof(u8) + (*num_chans * sizeof(NanFurtherAvailabilityChan)))) {
        ALOGE("%s: Invalid TLV Length", __func__);
        return -1;
    }

    for (idx = 0; idx < *num_chans; idx++) {
        pNanFurtherAvailabilityChan pRsp =
                (pNanFurtherAvailabilityChan)((u8*)&pInValue[1] +
                                              (idx * sizeof(NanFurtherAvailabilityChan)));

        pFac->entry_control = (NanAvailDuration)(pRsp->entryCtrl.availIntDuration);
        pFac->mapid = pRsp->entryCtrl.mapId;
        pFac->class_val = pRsp->opClass;
        pFac->channel = pRsp->channel;
        memcpy(&pFac->avail_interval_bitmap, &pRsp->availIntBitmap,
               sizeof(pFac->avail_interval_bitmap));
        pFac++;
    }
    return 0;
}

wifi_error NanCommand::getNanStaParameter(wifi_interface_handle iface, NanStaParameter* pRsp) {
    wifi_error ret = WIFI_ERROR_NONE;
    transaction_id id = 1;
    interface_info* ifaceInfo = getIfaceInfo(iface);

    ret = mapKernelErrortoWifiHalError(create());
    if (ret != WIFI_SUCCESS) goto cleanup;

    /* Set the interface Id of the message. */
    ret = set_iface_id(ifaceInfo->name);
    if (ret != WIFI_SUCCESS) goto cleanup;

    /*
       Construct NL message to get the sync stats parameter
       which has all the parameter required by staparameter.
    */
    NanStatsRequest syncStats;
    memset(&syncStats, 0, sizeof(syncStats));
    syncStats.stats_type = NAN_STATS_ID_DE_TIMING_SYNC;
    syncStats.clear = 0;

    mStaParam = pRsp;
    ret = putNanStats(id, &syncStats);
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: putNanStats Error:%d", __func__, ret);
        goto cleanup;
    }
    ret = requestEvent();
    if (ret != 0) {
        ALOGE("%s: requestEvent Error:%d", __func__, ret);
        goto cleanup;
    }

    struct timespec abstime;
    abstime.tv_sec = 4;
    abstime.tv_nsec = 0;
    ret = mapKernelErrortoWifiHalError(mCondition.wait(abstime));
    if (ret == WIFI_ERROR_TIMED_OUT) {
        ALOGE("%s: Time out happened.", __func__);
        goto cleanup;
    }
    ALOGV("%s: NanStaparameter Master_pref:%x,"
          " Random_factor:%x, hop_count:%x "
          " beacon_transmit_time:%d"
          " ndp_channel_freq:%d",
          __func__, pRsp->master_pref, pRsp->random_factor, pRsp->hop_count,
          pRsp->beacon_transmit_time, pRsp->ndp_channel_freq);
cleanup:
    mStaParam = NULL;
    return ret;
}

int NanCommand::getNanTransmitFollowupInd(NanTransmitFollowupInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanSelfTransmitFollowupIndMsg pRsp = (pNanSelfTransmitFollowupIndMsg)mNanVendorEvent;
    event->id = pRsp->fwHeader.transactionId;
    NanErrorTranslation((NanInternalStatusType)pRsp->reason, 0, (void*)event, false);
    return WIFI_SUCCESS;
}

// Function which calls the necessaryIndication callback
// based on the indication type
int NanCommand::handleNdpIndication(u32 ndpCmdType, struct nlattr** tb_vendor) {
    // Based on the message_id in the header determine the Indication type
    // and call the necessary callback handler
    int res = 0;

    ALOGI("handleNdpIndication msg_id:%u", ndpCmdType);
    switch (ndpCmdType) {
        case MTK_WLAN_VENDOR_ATTR_NDP_REQUEST_IND:
            NanDataPathRequestInd ndpRequestInd;
            memset(&ndpRequestInd, 0, sizeof(ndpRequestInd));

            res = getNdpRequest(tb_vendor, &ndpRequestInd);
            if (!res && mHandler.EventDataRequest) {
                (*mHandler.EventDataRequest)(&ndpRequestInd);
            }
            break;

        case MTK_WLAN_VENDOR_ATTR_NDP_CONFIRM_IND:
            NanDataPathConfirmInd ndpConfirmInd;
            memset(&ndpConfirmInd, 0, sizeof(ndpConfirmInd));

            res = getNdpConfirm(tb_vendor, &ndpConfirmInd);
            if (!res && mHandler.EventDataConfirm) {
                (*mHandler.EventDataConfirm)(&ndpConfirmInd);
            }
            break;

        case MTK_WLAN_VENDOR_ATTR_NDP_END_IND: {
            NanDataPathEndInd* ndpEndInd = NULL;
            u8 num_ndp_ids = 0;

            if (!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY]) {
                ALOGE("%s: MTK_WLAN_VENDOR_ATTR_NDP not found", __FUNCTION__);
                return WIFI_ERROR_INVALID_ARGS;
            }

            num_ndp_ids = (u8)(nla_len(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY]) /
                               sizeof(u32));
            ALOGD("%s: NDP Num Instance Ids : val %d", __FUNCTION__, num_ndp_ids);

            if (num_ndp_ids) {
                ndpEndInd = (NanDataPathEndInd*)malloc(sizeof(NanDataPathEndInd) +
                                                       (sizeof(u32) * num_ndp_ids));
                if (!ndpEndInd) {
                    ALOGE("%s: ndp_instance_id malloc Failed", __FUNCTION__);
                    return WIFI_ERROR_OUT_OF_MEMORY;
                }
                ndpEndInd->num_ndp_instances = num_ndp_ids;
                nla_memcpy(ndpEndInd->ndp_instance_id,
                           tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY],
                           sizeof(u32) * ndpEndInd->num_ndp_instances);
            }
            if (mHandler.EventDataEnd) {
                (*mHandler.EventDataEnd)(ndpEndInd);
            }
            free(ndpEndInd);
            break;
        }

        case MTK_WLAN_VENDOR_ATTR_NDP_SCHEDULE_UPDATE_IND: {
            NanDataPathScheduleUpdateInd* pNdpScheduleUpdateInd;
            u32 num_channels = 0, num_ndp_ids = 0;

            if ((!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR]) ||
                (!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_SCHEDULE_UPDATE_REASON]) ||
                (!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY])) {
                ALOGE("%s: MTK_WLAN_VENDOR_ATTR_NDP not found", __FUNCTION__);
                return WIFI_ERROR_INVALID_ARGS;
            }
            if (tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_NUM_CHANNELS]) {
                num_channels = nla_get_u32(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_NUM_CHANNELS]);
                ALOGD("%s: num_channels = %d", __FUNCTION__, num_channels);
                if ((num_channels > NAN_MAX_CHANNEL_INFO_SUPPORTED) &&
                    (!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_INFO])) {
                    ALOGE("%s: MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_INFO not found", __FUNCTION__);
                    return WIFI_ERROR_INVALID_ARGS;
                }
            }
            num_ndp_ids = (u8)(nla_len(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY]) /
                               sizeof(u32));
            ALOGD("%s: NDP Num Instance Ids : val %d", __FUNCTION__, num_ndp_ids);

            pNdpScheduleUpdateInd = (NanDataPathScheduleUpdateInd*)malloc(
                    sizeof(NanDataPathScheduleUpdateInd) + (sizeof(u32) * num_ndp_ids));
            if (!pNdpScheduleUpdateInd) {
                ALOGE("%s: NdpScheduleUpdate malloc Failed", __FUNCTION__);
                return WIFI_ERROR_OUT_OF_MEMORY;
            }
            pNdpScheduleUpdateInd->num_channels = num_channels;
            pNdpScheduleUpdateInd->num_ndp_instances = num_ndp_ids;

            res = getNdpScheduleUpdate(tb_vendor, pNdpScheduleUpdateInd);
            if (!res && mHandler.EventScheduleUpdate) {
                (*mHandler.EventScheduleUpdate)(pNdpScheduleUpdateInd);
            }
            free(pNdpScheduleUpdateInd);
            break;
        }
        default:
            ALOGE("handleNdpIndication error invalid ndpCmdType:%u", ndpCmdType);
            res = (int)WIFI_ERROR_INVALID_REQUEST_ID;
            break;
    }
    return res;
}

int NanCommand::getNdpRequest(struct nlattr** tb_vendor, NanDataPathRequestInd* event) {
    u32 len = 0;

    if (event == NULL || tb_vendor == NULL) {
        ALOGE("%s: Invalid input argument event:%p tb_vendor:%p", __FUNCTION__, event, tb_vendor);
        return WIFI_ERROR_INVALID_ARGS;
    }
    if ((!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_INSTANCE_ID]) ||
        (!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR]) ||
        (!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID])) {
        ALOGE("%s: MTK_WLAN_VENDOR_ATTR_NDP not found", __FUNCTION__);
        return WIFI_ERROR_INVALID_ARGS;
    }

    event->service_instance_id =
            nla_get_u16(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_SERVICE_INSTANCE_ID]);
    ALOGD("%s: Service Instance id : val %d", __FUNCTION__, event->service_instance_id);

    len = nla_len(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR]);
    len = ((sizeof(event->peer_disc_mac_addr) <= len) ? sizeof(event->peer_disc_mac_addr) : len);
    memcpy(&event->peer_disc_mac_addr[0],
           nla_data(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR]), len);

    event->ndp_instance_id = nla_get_u32(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID]);
    ALOGD("%s: Ndp Instance id: %d", __FUNCTION__, event->ndp_instance_id);
    if (tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]) {
        len = nla_len(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]);
        len = ((sizeof(event->app_info.ndp_app_info) <= len) ? sizeof(event->app_info.ndp_app_info)
                                                             : len);
        memcpy(&event->app_info.ndp_app_info[0],
               nla_data(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]), len);
        event->app_info.ndp_app_info_len = len;
    } else {
        ALOGD("%s: NDP App Info not present", __FUNCTION__);
    }
    return WIFI_SUCCESS;
}

int NanCommand::getNdpConfirm(struct nlattr** tb_vendor, NanDataPathConfirmInd* event) {
    u32 len = 0;
    NanInternalStatusType drv_reason_code;
    struct nlattr* chInfo;
    NanChannelInfo* pChInfo;
    int rem;
    u32 i = 0;

    if (event == NULL || tb_vendor == NULL) {
        ALOGE("%s: Invalid input argument event:%p tb_vendor:%p", __FUNCTION__, event, tb_vendor);
        return WIFI_ERROR_INVALID_ARGS;
    }
    if ((!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID]) ||
        (!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_NDI_MAC_ADDR]) ||
        (!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_RESPONSE_CODE])) {
        ALOGE("%s: MTK_WLAN_VENDOR_ATTR_NDP not found", __FUNCTION__);
        return WIFI_ERROR_INVALID_ARGS;
    }

    event->ndp_instance_id = nla_get_u16(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID]);
    ALOGD("%s: Service Instance id : val %d", __FUNCTION__, event->ndp_instance_id);

    len = nla_len(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_NDI_MAC_ADDR]);
    len = ((sizeof(event->peer_ndi_mac_addr) <= len) ? sizeof(event->peer_ndi_mac_addr) : len);
    memcpy(&event->peer_ndi_mac_addr[0], nla_data(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_NDI_MAC_ADDR]),
           len);

    event->rsp_code =
            (NanDataPathResponseCode)nla_get_u32(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_RESPONSE_CODE]);
    ALOGD("%s: Response code %d", __FUNCTION__, event->rsp_code);

    if (tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]) {
        len = nla_len(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]);
        len = ((sizeof(event->app_info.ndp_app_info) <= len) ? sizeof(event->app_info.ndp_app_info)
                                                             : len);
        memcpy(&event->app_info.ndp_app_info[0],
               nla_data(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_APP_INFO]), len);
        event->app_info.ndp_app_info_len = len;
    } else {
        ALOGD("%s: NDP App Info not present", __FUNCTION__);
    }
    drv_reason_code = (NanInternalStatusType)nla_get_u32(
            tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_DRV_RETURN_VALUE]);
    ALOGD("%s: Drv reason code %d", __FUNCTION__, drv_reason_code);
    switch (drv_reason_code) {
        case NDP_I_MGMT_FRAME_REQUEST_FAILED:
        case NDP_I_MGMT_FRAME_RESPONSE_FAILED:
        case NDP_I_MGMT_FRAME_CONFIRM_FAILED:
        case NDP_I_MGMT_FRAME_SECURITY_INSTALL_FAILED:
            event->reason_code = NAN_STATUS_PROTOCOL_FAILURE;
            break;
        default:
            event->reason_code = (NanStatusType)drv_reason_code;
            break;
    }
    ALOGD("%s: Reason code %d", __FUNCTION__, event->reason_code);

    if (tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_NUM_CHANNELS]) {
        event->num_channels = nla_get_u32(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_NUM_CHANNELS]);
        ALOGD("%s: num_channels = %d", __FUNCTION__, event->num_channels);
        if ((event->num_channels > NAN_MAX_CHANNEL_INFO_SUPPORTED) &&
            (!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_INFO])) {
            ALOGE("%s: MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_INFO not found", __FUNCTION__);
            return WIFI_ERROR_INVALID_ARGS;
        }
    }

    if (event->num_channels != 0) {
        for (chInfo = (struct nlattr*)nla_data(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_INFO]),
            rem = nla_len(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_INFO]);
             (i < NAN_MAX_CHANNEL_INFO_SUPPORTED && nla_ok(chInfo, rem));
             chInfo = nla_next(chInfo, &(rem))) {
            struct nlattr* tb2[MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_MAX + 1];

            pChInfo =
                    (NanChannelInfo*)((u8*)event->channel_info + (i++ * (sizeof(NanChannelInfo))));
            nla_parse(tb2, MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_MAX, (struct nlattr*)nla_data(chInfo),
                      nla_len(chInfo), NULL);

            if (!tb2[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL]) {
                ALOGE("%s: MTK_WLAN_VENDOR_ATTR_CHANNEL not found", __FUNCTION__);
                return WIFI_ERROR_INVALID_ARGS;
            }
            pChInfo->channel = nla_get_u32(tb2[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL]);
            ALOGD("%s: Channel = %d", __FUNCTION__, pChInfo->channel);

            if (!tb2[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_WIDTH]) {
                ALOGE("%s: MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_WIDTH not found", __FUNCTION__);
                return WIFI_ERROR_INVALID_ARGS;
            }
            pChInfo->bandwidth = nla_get_u32(tb2[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_WIDTH]);
            ALOGD("%s: Channel BW = %d", __FUNCTION__, pChInfo->bandwidth);

            if (!tb2[MTK_WLAN_VENDOR_ATTR_NDP_NSS]) {
                ALOGE("%s: MTK_WLAN_VENDOR_ATTR_NDP_NSS not found", __FUNCTION__);
                return WIFI_ERROR_INVALID_ARGS;
            }
            pChInfo->nss = nla_get_u32(tb2[MTK_WLAN_VENDOR_ATTR_NDP_NSS]);
            ALOGD("%s: No. Spatial Stream = %d", __FUNCTION__, pChInfo->nss);
        }
    }
    return WIFI_SUCCESS;
}

int NanCommand::getNdpScheduleUpdate(struct nlattr** tb_vendor,
                                     NanDataPathScheduleUpdateInd* event) {
    u32 len = 0;
    struct nlattr* chInfo;
    NanChannelInfo* pChInfo;
    int rem;
    u32 i = 0;

    len = nla_len(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR]);
    len = ((sizeof(event->peer_mac_addr) <= len) ? sizeof(event->peer_mac_addr) : len);
    memcpy(&event->peer_mac_addr[0],
           nla_data(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR]), len);

    event->schedule_update_reason_code =
            nla_get_u32(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_SCHEDULE_UPDATE_REASON]);
    ALOGD("%s: Reason code %d", __FUNCTION__, event->schedule_update_reason_code);

    if (event->num_channels != 0) {
        for (chInfo = (struct nlattr*)nla_data(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_INFO]),
            rem = nla_len(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_INFO]);
             (i < NAN_MAX_CHANNEL_INFO_SUPPORTED && nla_ok(chInfo, rem));
             chInfo = nla_next(chInfo, &(rem))) {
            struct nlattr* tb2[MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_MAX + 1];

            pChInfo =
                    (NanChannelInfo*)((u8*)event->channel_info + (i++ * (sizeof(NanChannelInfo))));
            nla_parse(tb2, MTK_WLAN_VENDOR_ATTR_NDP_PARAMS_MAX, (struct nlattr*)nla_data(chInfo),
                      nla_len(chInfo), NULL);

            if (!tb2[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL]) {
                ALOGE("%s: MTK_WLAN_VENDOR_ATTR_CHANNEL not found", __FUNCTION__);
                return WIFI_ERROR_INVALID_ARGS;
            }
            pChInfo->channel = nla_get_u32(tb2[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL]);
            ALOGD("%s: Channel = %d", __FUNCTION__, pChInfo->channel);

            if (!tb2[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_WIDTH]) {
                ALOGE("%s: MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_WIDTH not found", __FUNCTION__);
                return WIFI_ERROR_INVALID_ARGS;
            }
            pChInfo->bandwidth = nla_get_u32(tb2[MTK_WLAN_VENDOR_ATTR_NDP_CHANNEL_WIDTH]);
            ALOGD("%s: Channel BW = %d", __FUNCTION__, pChInfo->bandwidth);

            if (!tb2[MTK_WLAN_VENDOR_ATTR_NDP_NSS]) {
                ALOGE("%s: MTK_WLAN_VENDOR_ATTR_NDP_NSS not found", __FUNCTION__);
                return WIFI_ERROR_INVALID_ARGS;
            }
            pChInfo->nss = nla_get_u32(tb2[MTK_WLAN_VENDOR_ATTR_NDP_NSS]);
            ALOGD("%s: No. Spatial Stream = %d", __FUNCTION__, pChInfo->nss);
        }
    }

    if (event->num_ndp_instances) {
        nla_memcpy(event->ndp_instance_id, tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY],
                   sizeof(u32) * event->num_ndp_instances);
    }
    return WIFI_SUCCESS;
}

int NanCommand::getNanRangeRequestReceivedInd(NanRangeRequestInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanFWRangeReqRecvdInd pRsp = (pNanFWRangeReqRecvdInd)mNanVendorEvent;

    u8* pInputTlv = pRsp->ptlv;
    NanTlv outputTlv;
    u16 readLen = 0;

    int remainingLen = (mNanDataLen - (sizeof(NanMsgHeader)));

    if (remainingLen <= 0) {
        ALOGE("%s: No TLV's present", __func__);
        return WIFI_SUCCESS;
    }

    ALOGV("%s: TLV remaining Len:%d", __func__, remainingLen);
    while ((remainingLen > 0) && (0 != (readLen = NANTLV_ReadTlv(pInputTlv, &outputTlv)))) {
        ALOGV("%s: Remaining Len:%d readLen:%d type:%d length:%d", __func__, remainingLen, readLen,
              outputTlv.type, outputTlv.length);
        switch (outputTlv.type) {
            case NAN_TLV_TYPE_NAN20_RANGING_REQUEST_RECEIVED:
                NanFWRangeReqRecvdMsg fwRangeReqRecvd;
                if (outputTlv.length > sizeof(fwRangeReqRecvd)) {
                    outputTlv.length = sizeof(fwRangeReqRecvd);
                }
                memcpy(&fwRangeReqRecvd, outputTlv.value, outputTlv.length);
                FW_MAC_ADDR_TO_CHAR_ARRAY(fwRangeReqRecvd.range_mac_addr,
                                          event->range_req_intf_addr);
                event->publish_id = fwRangeReqRecvd.range_id;
                break;
            default:
                ALOGV("Unhandled TLV type:%d", outputTlv.type);
                break;
        }
        remainingLen -= readLen;
        pInputTlv += readLen;
        memset(&outputTlv, 0, sizeof(outputTlv));
    }
    return WIFI_SUCCESS;
}

int NanCommand::getNanRangeReportInd(NanRangeReportInd* event) {
    if (event == NULL || mNanVendorEvent == NULL) {
        ALOGE("%s: Invalid input argument event:%p mNanVendorEvent:%p", __func__, event,
              mNanVendorEvent);
        return WIFI_ERROR_INVALID_ARGS;
    }

    pNanFWRangeReportInd pRsp = (pNanFWRangeReportInd)mNanVendorEvent;

    u8* pInputTlv = pRsp->ptlv;
    NanTlv outputTlv;
    u16 readLen = 0;

    int remainingLen = (mNanDataLen - (sizeof(NanMsgHeader)));

    if (remainingLen <= 0) {
        ALOGE("%s: No TLV's present", __func__);
        return WIFI_SUCCESS;
    }

    ALOGV("%s: TLV remaining Len:%d", __func__, remainingLen);
    while ((remainingLen > 0) && (0 != (readLen = NANTLV_ReadTlv(pInputTlv, &outputTlv)))) {
        ALOGV("%s: Remaining Len:%d readLen:%d type:%d length:%d", __func__, remainingLen, readLen,
              outputTlv.type, outputTlv.length);
        switch (outputTlv.type) {
            case NAN_TLV_TYPE_MAC_ADDRESS:
                if (outputTlv.length > NAN_MAC_ADDR_LEN) {
                    outputTlv.length = NAN_MAC_ADDR_LEN;
                }
                memcpy(event->range_req_intf_addr, outputTlv.value, outputTlv.length);
                break;

            case NAN_TLV_TYPE_NAN20_RANGING_RESULT:
                NanFWRangeReportParams range_params;
                if (outputTlv.length > sizeof(NanFWRangeReportParams)) {
                    outputTlv.length = sizeof(NanFWRangeReportParams);
                }
                memcpy(&range_params, outputTlv.value, outputTlv.length);
                event->range_measurement_mm = range_params.range_measurement;
                event->publish_id = range_params.publish_id;
                //          event->event_type = range_params.event_type;
                break;
            default:
                ALOGV("Unhandled TLV type:%d", outputTlv.type);
                break;
        }
        remainingLen -= readLen;
        pInputTlv += readLen;
        memset(&outputTlv, 0, sizeof(outputTlv));
    }
    return WIFI_SUCCESS;
}

int NanCommand::isNanResponse() {
    if (mNanVendorEvent == NULL) {
        ALOGE("NULL check failed");
        return WIFI_ERROR_INVALID_ARGS;
    }

    NanMsgHeader* pHeader = (NanMsgHeader*)mNanVendorEvent;

    switch (pHeader->msgId) {
        case NAN_MSG_ID_ERROR_RSP:
        case NAN_MSG_ID_CONFIGURATION_RSP:
        case NAN_MSG_ID_PUBLISH_SERVICE_CANCEL_RSP:
        case NAN_MSG_ID_PUBLISH_SERVICE_RSP:
        case NAN_MSG_ID_SUBSCRIBE_SERVICE_RSP:
        case NAN_MSG_ID_SUBSCRIBE_SERVICE_CANCEL_RSP:
        case NAN_MSG_ID_TRANSMIT_FOLLOWUP_RSP:
        case NAN_MSG_ID_STATS_RSP:
        case NAN_MSG_ID_ENABLE_RSP:
        case NAN_MSG_ID_DISABLE_RSP:
        case NAN_MSG_ID_TCA_RSP:
        case NAN_MSG_ID_BEACON_SDF_RSP:
        case NAN_MSG_ID_CAPABILITIES_RSP:
        case NAN_MSG_ID_TESTMODE_RSP:
            return 1;
        default:
            return 0;
    }
}

struct verboseTlv {
    NanTlvType tlvType;
    char strTlv[NAN_ERROR_STR_LEN];
};

struct verboseTlv tlvToStr[] = {
        {NAN_TLV_TYPE_SDF_MATCH_FILTER, " SDF match filter"},
        {NAN_TLV_TYPE_TX_MATCH_FILTER, " Tx match filter"},
        {NAN_TLV_TYPE_RX_MATCH_FILTER, " Rx match filter"},
        {NAN_TLV_TYPE_SERVICE_SPECIFIC_INFO, " Service specific info"},
        {NAN_TLV_TYPE_EXT_SERVICE_SPECIFIC_INFO, " Extended Service specific info"},
        {NAN_TLV_TYPE_VENDOR_SPECIFIC_ATTRIBUTE_TRANSMIT, " Vendor specific attribute transmit"},
        {NAN_TLV_TYPE_VENDOR_SPECIFIC_ATTRIBUTE_RECEIVE, " Vendor specific attribute receive"},
        {NAN_TLV_TYPE_POST_NAN_CONNECTIVITY_CAPABILITIES_RECEIVE,
         " Post Nan connectivity capability receive"},
        {NAN_TLV_TYPE_POST_NAN_DISCOVERY_ATTRIBUTE_RECEIVE,
         " Post Nan discovery attribute receive"},
        {NAN_TLV_TYPE_BEACON_SDF_PAYLOAD_RECEIVE, " Beacon SDF payload receive"},

        /* Configuration types */
        {NAN_TLV_TYPE_CONFIG_FIRST, " Config first"},
        {NAN_TLV_TYPE_24G_SUPPORT, " 2.4G support"},
        {NAN_TLV_TYPE_24G_BEACON, " 2.4G beacon"},
        {NAN_TLV_TYPE_24G_SDF, " 2.4G SDF"},
        {NAN_TLV_TYPE_24G_RSSI_CLOSE, " 2.4G RSSI close"},
        {NAN_TLV_TYPE_24G_RSSI_MIDDLE, " 2.4G RSSI middle"},
        {NAN_TLV_TYPE_24G_RSSI_CLOSE_PROXIMITY, " 2.4G RSSI close proximity"},
        {NAN_TLV_TYPE_5G_SUPPORT, " 5G support"},
        {NAN_TLV_TYPE_5G_BEACON, " 5G beacon"},
        {NAN_TLV_TYPE_5G_SDF, " 5G SDF"},
        {NAN_TLV_TYPE_5G_RSSI_CLOSE, " 5G RSSI close"},
        {NAN_TLV_TYPE_5G_RSSI_MIDDLE, " 5G RSSI middle"},
        {NAN_TLV_TYPE_5G_RSSI_CLOSE_PROXIMITY, " 5G RSSI close proximity"},
        {NAN_TLV_TYPE_SID_BEACON, " SID beacon"},
        {NAN_TLV_TYPE_HOP_COUNT_LIMIT, " Hop count limit"},
        {NAN_TLV_TYPE_MASTER_PREFERENCE, " Master preference"},
        {NAN_TLV_TYPE_CLUSTER_ID_LOW, " Cluster ID low"},
        {NAN_TLV_TYPE_CLUSTER_ID_HIGH, " Cluster ID high"},
        {NAN_TLV_TYPE_RSSI_AVERAGING_WINDOW_SIZE, " RSSI averaging window size"},
        {NAN_TLV_TYPE_CLUSTER_OUI_NETWORK_ID, " Cluster OUI network ID"},
        {NAN_TLV_TYPE_SOURCE_MAC_ADDRESS, " Source MAC address"},
        {NAN_TLV_TYPE_CLUSTER_ATTRIBUTE_IN_SDF, " Cluster attribute in SDF"},
        {NAN_TLV_TYPE_SOCIAL_CHANNEL_SCAN_PARAMS, " Social channel scan params"},
        {NAN_TLV_TYPE_DEBUGGING_FLAGS, " Debugging flags"},
        {NAN_TLV_TYPE_POST_NAN_CONNECTIVITY_CAPABILITIES_TRANSMIT,
         " Post nan connectivity capabilities transmit"},
        {NAN_TLV_TYPE_POST_NAN_DISCOVERY_ATTRIBUTE_TRANSMIT,
         " Post nan discovery attribute transmit"},
        {NAN_TLV_TYPE_FURTHER_AVAILABILITY_MAP, " Further availability map"},
        {NAN_TLV_TYPE_HOP_COUNT_FORCE, " Hop count force"},
        {NAN_TLV_TYPE_RANDOM_FACTOR_FORCE, " Random factor force"},
        {NAN_TLV_TYPE_RANDOM_UPDATE_TIME, " Random update time"},
        {NAN_TLV_TYPE_EARLY_WAKEUP, " Early wakeup"},
        {NAN_TLV_TYPE_PERIODIC_SCAN_INTERVAL, " Periodic scan interval"},
        {NAN_TLV_TYPE_DW_INTERVAL, " DW interval"},
        {NAN_TLV_TYPE_DB_INTERVAL, " DB interval"},
        {NAN_TLV_TYPE_FURTHER_AVAILABILITY, " Further availability"},
        {NAN_TLV_TYPE_24G_CHANNEL, " 2.4G channel"},
        {NAN_TLV_TYPE_5G_CHANNEL, " 5G channel"},
        {NAN_TLV_TYPE_CONFIG_LAST, " Config last"},

        /* Attributes types */
        {NAN_TLV_TYPE_ATTRS_FIRST, " Attributes first"},
        {NAN_TLV_TYPE_AVAILABILITY_INTERVALS_MAP, " Availability intervals map"},
        {NAN_TLV_TYPE_WLAN_MESH_ID, " WLAN mesh ID"},
        {NAN_TLV_TYPE_MAC_ADDRESS, " MAC address"},
        {NAN_TLV_TYPE_RECEIVED_RSSI_VALUE, " Received RSSI value"},
        {NAN_TLV_TYPE_CLUSTER_ATTRIBUTE, " Cluster attribute"},
        {NAN_TLV_TYPE_WLAN_INFRA_SSID, " WLAN infra SSID"},
        {NAN_TLV_TYPE_ATTRS_LAST, " Attributes last"},

        /* Events Type */
        {NAN_TLV_TYPE_EVENTS_FIRST, " Events first"},
        {NAN_TLV_TYPE_EVENT_SELF_STATION_MAC_ADDRESS, " Event Self station MAC address"},
        {NAN_TLV_TYPE_EVENT_STARTED_CLUSTER, " Event started cluster"},
        {NAN_TLV_TYPE_EVENT_JOINED_CLUSTER, " Event joined cluster"},
        {NAN_TLV_TYPE_EVENT_CLUSTER_SCAN_RESULTS, " Event cluster scan results"},
        {NAN_TLV_TYPE_FAW_MEM_AVAIL, " FAW memory availability"},
        {NAN_TLV_TYPE_EVENTS_LAST, " Events last"},

        /* TCA types */
        {NAN_TLV_TYPE_TCA_FIRST, " TCA-Threshold Crossing Alert first"},
        {NAN_TLV_TYPE_CLUSTER_SIZE_REQ, " Cluster size request"},
        {NAN_TLV_TYPE_CLUSTER_SIZE_RSP, " Cluster size response"},
        {NAN_TLV_TYPE_TCA_LAST, " TCA last"},

        /* Statistics types */
        {NAN_TLV_TYPE_STATS_FIRST, " Stats first"},
        {NAN_TLV_TYPE_DE_PUBLISH_STATS, " Discovery engine publish stats"},
        {NAN_TLV_TYPE_DE_SUBSCRIBE_STATS, " Discovery engine subscribe stats"},
        {NAN_TLV_TYPE_DE_MAC_STATS, " Discovery engine MAC stats"},
        {NAN_TLV_TYPE_DE_TIMING_SYNC_STATS, " Discovery engine timing sync stats"},
        {NAN_TLV_TYPE_DE_DW_STATS, " Discovery engine DW stats"},
        {NAN_TLV_TYPE_DE_STATS, " Discovery engine stats"},
        {NAN_TLV_TYPE_STATS_LAST, " Stats last"},

        {NAN_TLV_TYPE_LAST, " Last"}};

struct errorCode {
    NanStatusType frameworkError;
    NanInternalStatusType firmwareError;
    char nan_error[NAN_ERROR_STR_LEN];
};

struct errorCode errorCodeTranslation[] = {
        {NAN_STATUS_SUCCESS, NAN_I_STATUS_SUCCESS, "NAN status success"},

        {NAN_STATUS_INTERNAL_FAILURE, NAN_I_STATUS_DE_FAILURE, "NAN Discovery engine failure"},

        {NAN_STATUS_INVALID_PUBLISH_SUBSCRIBE_ID, NAN_I_STATUS_INVALID_HANDLE,
         "Invalid Publish/Subscribe ID"},

        {NAN_STATUS_NO_RESOURCE_AVAILABLE, NAN_I_STATUS_NO_SPACE_AVAILABLE, "No space available"},

        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_PUBLISH_TYPE,
         "Invalid Publish type, can be 0 or 1"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_TX_TYPE, "Invalid Tx type"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_MSG_VERSION,
         "Invalid internal message version"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_MSG_LEN, "Invalid message length"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_MSG_ID, "Invalid message ID"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_MATCH_ALGORITHM,
         "Invalid matching algorithm, can be 0(match once), 1(match continuous) or 2(match never)"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_TLV_LEN, "Invalid TLV length"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_TLV_TYPE, "Invalid TLV type"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_MISSING_TLV_TYPE, "Missing TLV type"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_TOTAL_TLVS_LEN, "Invalid total TLV length"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_TLV_VALUE, "Invalid TLV value"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_TX_PRIORITY, "Invalid Tx priority"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_CONNECTION_MAP, "Invalid connection map"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_THRESHOLD_CROSSING_ALERT_ID,
         "Invalid TCA-Threshold Crossing Alert ID"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_STATS_ID, "Invalid STATS ID"},

        {NAN_STATUS_PROTOCOL_FAILURE, NAN_I_STATUS_TX_FAIL, "Tx Fail"},

        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_RSSI_CLOSE_VALUE,
         "Invalid RSSI close value range is 20dbm to 60dbm"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_RSSI_MIDDLE_VALUE,
         "Invalid RSSI middle value range is 20dbm to 75dbm"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_HOP_COUNT_LIMIT,
         "Invalid hop count limit, max hop count limit is 5"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_HIGH_CLUSTER_ID_VALUE,
         "Invalid cluster ID value. Please set the cluster id high greater than the cluster id "
         "low"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_BACKGROUND_SCAN_PERIOD,
         "Invalid background scan period. The range is 10 to 30 milliseconds"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_SCAN_CHANNEL,
         "Invalid scan channel. Only valid channels are the NAN social channels"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_POST_NAN_CONNECTIVITY_CAPABILITIES_BITMAP,
         "Invalid post nan connectivity bitmap"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_FURTHER_AVAILABILITY_MAP_NUMCHAN_VALUE,
         "Invalid further availability map number of channel value"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_FURTHER_AVAILABILITY_MAP_DURATION_VALUE,
         "Invalid further availability map duration value"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_FURTHER_AVAILABILITY_MAP_CLASS_VALUE,
         "Invalid further availability map class value"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_FURTHER_AVAILABILITY_MAP_CHANNEL_VALUE,
         "Invalid further availability map channel value"},
        {NAN_STATUS_INVALID_PARAM,
         NAN_I_STATUS_INVALID_FURTHER_AVAILABILITY_MAP_AVAILABILITY_INTERVAL_BITMAP_VALUE,
         "Invalid further availability map availability interval bitmap value"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_FURTHER_AVAILABILITY_MAP_MAP_ID,
         "Invalid further availability map map ID"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_POST_NAN_DISCOVERY_CONN_TYPE_VALUE,
         "Invalid post nan discovery connection type value"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_POST_NAN_DISCOVERY_DEVICE_ROLE_VALUE,
         "Invalid post nan discovery device role value"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_POST_NAN_DISCOVERY_DURATION_VALUE,
         "Invalid post nan discovery duration value"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_POST_NAN_DISCOVERY_BITMAP_VALUE,
         "Invalid post nan discovery bitmap value"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_MISSING_FUTHER_AVAILABILITY_MAP,
         "Missing further availability map"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_BAND_CONFIG_FLAGS,
         "Invalid band configuration flags"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_RANDOM_FACTOR_UPDATE_TIME_VALUE,
         "Invalid random factor update time value"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_ONGOING_SCAN_PERIOD,
         "Invalid ongoing scan period"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_DW_INTERVAL_VALUE,
         "Invalid DW interval value"},
        {NAN_STATUS_INVALID_PARAM, NAN_I_STATUS_INVALID_DB_INTERVAL_VALUE,
         "Invalid DB interval value"},

        {NAN_STATUS_SUCCESS, NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_TIMEOUT,
         "Terminated Reason: Timeout"},
        {NAN_STATUS_SUCCESS, NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_USER_REQUEST,
         "Terminated Reason: User Request"},
        {NAN_STATUS_SUCCESS, NAN_I_PUBLISH_SUBSCRIBE_TERMINATED_REASON_COUNT_REACHED,
         "Terminated Reason: Count Reached"},

        {NAN_STATUS_INVALID_REQUESTOR_INSTANCE_ID, NAN_I_STATUS_INVALID_REQUESTER_INSTANCE_ID,
         "Invalid match handle"},
        {NAN_STATUS_NAN_NOT_ALLOWED, NAN_I_STATUS_NAN_NOT_ALLOWED, "Nan not allowed"},
        {NAN_STATUS_NO_OTA_ACK, NAN_I_STATUS_NO_OTA_ACK, "No OTA ack"},
        {NAN_STATUS_ALREADY_ENABLED, NAN_I_STATUS_NAN_ALREADY_ENABLED, "NAN is Already enabled"},
        {NAN_STATUS_FOLLOWUP_QUEUE_FULL, NAN_I_STATUS_FOLLOWUP_QUEUE_FULL, "Follow-up queue full"},

        {NAN_STATUS_UNSUPPORTED_CONCURRENCY_NAN_DISABLED, NDP_I_UNSUPPORTED_CONCURRENCY,
         "Unsupported Concurrency"},

        {NAN_STATUS_INTERNAL_FAILURE, NDP_I_NAN_DATA_IFACE_CREATE_FAILED,
         "NAN data interface create failed"},
        {NAN_STATUS_INTERNAL_FAILURE, NDP_I_NAN_DATA_IFACE_DELETE_FAILED,
         "NAN data interface delete failed"},
        {NAN_STATUS_INTERNAL_FAILURE, NDP_I_DATA_INITIATOR_REQUEST_FAILED,
         "NAN data initiator request failed"},
        {NAN_STATUS_INTERNAL_FAILURE, NDP_I_DATA_RESPONDER_REQUEST_FAILED,
         "NAN data responder request failed"},

        {NAN_STATUS_INVALID_NDP_ID, NDP_I_INVALID_NDP_INSTANCE_ID, "Invalid NDP instance ID"},

        {NAN_STATUS_INVALID_PARAM, NDP_I_INVALID_RESPONSE_CODE, "Invalid response code"},
        {NAN_STATUS_INVALID_PARAM, NDP_I_INVALID_APP_INFO_LEN, "Invalid app info length"},

        {NAN_STATUS_PROTOCOL_FAILURE, NDP_I_MGMT_FRAME_REQUEST_FAILED,
         "Management frame request failed"},
        {NAN_STATUS_PROTOCOL_FAILURE, NDP_I_MGMT_FRAME_RESPONSE_FAILED,
         "Management frame response failed"},
        {NAN_STATUS_PROTOCOL_FAILURE, NDP_I_MGMT_FRAME_CONFIRM_FAILED,
         "Management frame confirm failed"},

        {NAN_STATUS_INTERNAL_FAILURE, NDP_I_END_FAILED, "NDP end failed"},

        {NAN_STATUS_PROTOCOL_FAILURE, NDP_I_MGMT_FRAME_END_REQUEST_FAILED,
         "Management frame end request failed"},

        {NAN_STATUS_INTERNAL_FAILURE, NDP_I_VENDOR_SPECIFIC_ERROR, "Vendor specific error"}};

void NanCommand::NanErrorTranslation(NanInternalStatusType firmwareErrorRecvd, u32 valueRcvd,
                                     void* pResponse, bool is_ndp_rsp) {
    int i = 0, j = 0;
    u16 msg_id; /* Based on the message_id in the header determine the Indication type */
    NanResponseMsg* pRsp;
    NanPublishTerminatedInd* pRspInd;
    NanDisabledInd* pRspdInd;
    char tlvInfo[NAN_ERROR_STR_LEN];
    tlvInfo[0] = '\0';

    if ((is_ndp_rsp == true) || isNanResponse()) {
        pRsp = (NanResponseMsg*)pResponse;
        for (i = 0; i < (int)(sizeof(errorCodeTranslation) / sizeof(errorCode)); i++) {
            if (errorCodeTranslation[i].firmwareError == firmwareErrorRecvd) {
                pRsp->status = errorCodeTranslation[i].frameworkError;
                strlcpy(pRsp->nan_error, errorCodeTranslation[i].nan_error, NAN_ERROR_STR_LEN);
                if (NAN_I_STATUS_INVALID_TLV_TYPE == firmwareErrorRecvd) {
                    for (j = 0; j < (int)(sizeof(tlvToStr) / sizeof(verboseTlv)); j++) {
                        if (tlvToStr[j].tlvType == valueRcvd) {
                            strlcpy(tlvInfo, tlvToStr[i].strTlv, NAN_ERROR_STR_LEN);
                            break;
                        }
                    }
                }
                strlcat(pRsp->nan_error, tlvInfo, sizeof(pRsp->nan_error));
                break;
            }
        }
        if (i == (int)(sizeof(errorCodeTranslation) / sizeof(errorCode))) {
            pRsp->status = NAN_STATUS_INTERNAL_FAILURE;
            strlcpy(pRsp->nan_error, "NAN Discovery engine failure", NAN_ERROR_STR_LEN);
        }
        ALOGD("%s: Status: %d Error Info[value %d]: %s", __FUNCTION__, pRsp->status, valueRcvd,
              pRsp->nan_error);
    } else {
        msg_id = getIndicationType();

        switch (msg_id) {
            case NAN_INDICATION_PUBLISH_TERMINATED:
            case NAN_INDICATION_SUBSCRIBE_TERMINATED:
            case NAN_INDICATION_SELF_TRANSMIT_FOLLOWUP:
                pRspInd = (NanPublishTerminatedInd*)pResponse;
                for (i = 0; i < (int)(sizeof(errorCodeTranslation) / sizeof(errorCode)); i++) {
                    if (errorCodeTranslation[i].firmwareError == firmwareErrorRecvd) {
                        pRspInd->reason = errorCodeTranslation[i].frameworkError;
                        strlcpy(pRspInd->nan_reason, errorCodeTranslation[i].nan_error,
                                NAN_ERROR_STR_LEN);
                        break;
                    }
                }
                if (i == (int)(sizeof(errorCodeTranslation) / sizeof(errorCode))) {
                    pRspInd->reason = NAN_STATUS_INTERNAL_FAILURE;
                    strlcpy(pRspInd->nan_reason, "NAN Discovery engine failure", NAN_ERROR_STR_LEN);
                }
                ALOGD("%s: Status: %d Error Info[value %d]: %s", __FUNCTION__, pRspInd->reason,
                      valueRcvd, pRspInd->nan_reason);
                break;
            case NAN_INDICATION_DISABLED:
                pRspdInd = (NanDisabledInd*)pResponse;
                for (i = 0; i < (int)(sizeof(errorCodeTranslation) / sizeof(errorCode)); i++) {
                    if (errorCodeTranslation[i].firmwareError == firmwareErrorRecvd) {
                        pRspdInd->reason = errorCodeTranslation[i].frameworkError;
                        strlcpy(pRspdInd->nan_reason, errorCodeTranslation[i].nan_error,
                                NAN_ERROR_STR_LEN);
                        break;
                    }
                }
                if (i == (int)(sizeof(errorCodeTranslation) / sizeof(errorCode))) {
                    pRspdInd->reason = NAN_STATUS_INTERNAL_FAILURE;
                    strlcpy(pRspdInd->nan_reason, "NAN Discovery engine failure",
                            NAN_ERROR_STR_LEN);
                }
                ALOGD("%s: Status: %d Error Info[value %d]: %s", __FUNCTION__, pRspdInd->reason,
                      valueRcvd, pRspdInd->nan_reason);
                break;
        }
    }
}

int NanCommand::getNanResponse(transaction_id* id, NanResponseMsg* pRsp) {
    if (mNanVendorEvent == NULL || pRsp == NULL) {
        ALOGE("NULL check failed");
        return WIFI_ERROR_INVALID_ARGS;
    }

    NanMsgHeader* pHeader = (NanMsgHeader*)mNanVendorEvent;

    switch (pHeader->msgId) {
        case NAN_MSG_ID_ERROR_RSP: {
            pNanErrorRspMsg pFwRsp = (pNanErrorRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->status, pFwRsp->value, pRsp, false);
            pRsp->response_type = NAN_RESPONSE_ERROR;
            break;
        }
        case NAN_MSG_ID_CONFIGURATION_RSP: {
            pNanConfigurationRspMsg pFwRsp = (pNanConfigurationRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->status, pFwRsp->value, pRsp, false);
            pRsp->response_type = NAN_RESPONSE_CONFIG;
        } break;
        case NAN_MSG_ID_PUBLISH_SERVICE_CANCEL_RSP: {
            pNanPublishServiceCancelRspMsg pFwRsp = (pNanPublishServiceCancelRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->status, pFwRsp->value, pRsp, false);
            pRsp->response_type = NAN_RESPONSE_PUBLISH_CANCEL;
            pRsp->body.publish_response.publish_id = pFwRsp->fwHeader.handle;
            break;
        }
        case NAN_MSG_ID_PUBLISH_SERVICE_RSP: {
            pNanPublishServiceRspMsg pFwRsp = (pNanPublishServiceRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->status, pFwRsp->value, pRsp, false);
            pRsp->response_type = NAN_RESPONSE_PUBLISH;
            pRsp->body.publish_response.publish_id = pFwRsp->fwHeader.handle;
            break;
        }
        case NAN_MSG_ID_SUBSCRIBE_SERVICE_RSP: {
            pNanSubscribeServiceRspMsg pFwRsp = (pNanSubscribeServiceRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->status, pFwRsp->value, pRsp, false);
            pRsp->response_type = NAN_RESPONSE_SUBSCRIBE;
            pRsp->body.subscribe_response.subscribe_id = pFwRsp->fwHeader.handle;
        } break;
        case NAN_MSG_ID_SUBSCRIBE_SERVICE_CANCEL_RSP: {
            pNanSubscribeServiceCancelRspMsg pFwRsp =
                    (pNanSubscribeServiceCancelRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->status, pFwRsp->value, pRsp, false);
            pRsp->response_type = NAN_RESPONSE_SUBSCRIBE_CANCEL;
            pRsp->body.subscribe_response.subscribe_id = pFwRsp->fwHeader.handle;
            break;
        }
        case NAN_MSG_ID_TRANSMIT_FOLLOWUP_RSP: {
            pNanTransmitFollowupRspMsg pFwRsp = (pNanTransmitFollowupRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->status, pFwRsp->value, pRsp, false);
            pRsp->response_type = NAN_RESPONSE_TRANSMIT_FOLLOWUP;
            break;
        }
        case NAN_MSG_ID_STATS_RSP: {
            pNanStatsRspMsg pFwRsp = (pNanStatsRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->statsRspParams.status,
                                pFwRsp->statsRspParams.value, pRsp, false);
            pRsp->response_type = NAN_RESPONSE_STATS;
            pRsp->body.stats_response.stats_type = (NanStatsType)pFwRsp->statsRspParams.statsType;
            ALOGV("%s: stats_type:%d", __func__, pRsp->body.stats_response.stats_type);
            u8* pInputTlv = pFwRsp->ptlv;
            NanTlv outputTlv;
            memset(&outputTlv, 0, sizeof(outputTlv));
            u16 readLen = 0;
            int remainingLen = (mNanDataLen - (sizeof(NanMsgHeader) + sizeof(NanStatsRspParams)));
            if (remainingLen > 0) {
                readLen = NANTLV_ReadTlv(pInputTlv, &outputTlv);
                ALOGV("%s: Remaining Len:%d readLen:%d type:%d length:%d", __func__, remainingLen,
                      readLen, outputTlv.type, outputTlv.length);
                if (outputTlv.length <= sizeof(pRsp->body.stats_response.data)) {
                    handleNanStatsResponse(pRsp->body.stats_response.stats_type,
                                           (char*)outputTlv.value, &pRsp->body.stats_response,
                                           outputTlv.length);
                }
            } else
                ALOGV("%s: No TLV's present", __func__);
            break;
        }
        case NAN_MSG_ID_ENABLE_RSP: {
            pNanEnableRspMsg pFwRsp = (pNanEnableRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->status, pFwRsp->value, pRsp, false);
            pRsp->response_type = NAN_RESPONSE_ENABLED;
            break;
        }
        case NAN_MSG_ID_DISABLE_RSP: {
            pNanDisableRspMsg pFwRsp = (pNanDisableRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->status, 0, pRsp, false);
            pRsp->response_type = NAN_RESPONSE_DISABLED;
            break;
        }
        case NAN_MSG_ID_TCA_RSP: {
            pNanTcaRspMsg pFwRsp = (pNanTcaRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->status, pFwRsp->value, pRsp, false);
            pRsp->response_type = NAN_RESPONSE_TCA;
            break;
        }
        case NAN_MSG_ID_BEACON_SDF_RSP: {
            pNanBeaconSdfPayloadRspMsg pFwRsp = (pNanBeaconSdfPayloadRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->status, 0, pRsp, false);
            pRsp->response_type = NAN_RESPONSE_BEACON_SDF_PAYLOAD;
            break;
        }
        case NAN_MSG_ID_CAPABILITIES_RSP: {
            pNanCapabilitiesRspMsg pFwRsp = (pNanCapabilitiesRspMsg)mNanVendorEvent;
            *id = (transaction_id)pFwRsp->fwHeader.transactionId;
            NanErrorTranslation((NanInternalStatusType)pFwRsp->status, pFwRsp->value, pRsp, false);
            pRsp->response_type = NAN_GET_CAPABILITIES;
            pRsp->body.nan_capabilities.max_concurrent_nan_clusters =
                    pFwRsp->max_concurrent_nan_clusters;
            pRsp->body.nan_capabilities.max_publishes = pFwRsp->max_publishes;
            pRsp->body.nan_capabilities.max_subscribes = pFwRsp->max_subscribes;
            pRsp->body.nan_capabilities.max_service_name_len = pFwRsp->max_service_name_len;
            pRsp->body.nan_capabilities.max_match_filter_len = pFwRsp->max_match_filter_len;
            pRsp->body.nan_capabilities.max_total_match_filter_len =
                    pFwRsp->max_total_match_filter_len;
            pRsp->body.nan_capabilities.max_service_specific_info_len =
                    pFwRsp->max_service_specific_info_len;
            pRsp->body.nan_capabilities.max_vsa_data_len = pFwRsp->max_vsa_data_len;
            pRsp->body.nan_capabilities.max_mesh_data_len = pFwRsp->max_mesh_data_len;
            pRsp->body.nan_capabilities.max_ndi_interfaces = pFwRsp->max_ndi_interfaces;
            pRsp->body.nan_capabilities.max_ndp_sessions = pFwRsp->max_ndp_sessions;
            pRsp->body.nan_capabilities.max_app_info_len = pFwRsp->max_app_info_len;
            pRsp->body.nan_capabilities.max_queued_transmit_followup_msgs =
                    pFwRsp->max_queued_transmit_followup_msgs;
            pRsp->body.nan_capabilities.ndp_supported_bands = pFwRsp->ndp_supported_bands;
            pRsp->body.nan_capabilities.cipher_suites_supported = pFwRsp->cipher_suites_supported;
            pRsp->body.nan_capabilities.max_scid_len = pFwRsp->max_scid_len;
            pRsp->body.nan_capabilities.is_ndp_security_supported =
                    pFwRsp->is_ndp_security_supported;
            pRsp->body.nan_capabilities.max_sdea_service_specific_info_len =
                    pFwRsp->max_sdea_service_specific_info_len;
            pRsp->body.nan_capabilities.max_subscribe_address = pFwRsp->max_subscribe_address;
            pRsp->body.nan_capabilities.ndpe_attr_supported = pFwRsp->ndpe_attr_supported;
            break;
        }
        default:
            return -1;
    }
    return 0;
}

int NanCommand::handleNanResponse() {
    // parse the data and call
    // the response callback handler with the populated
    // NanResponseMsg
    NanResponseMsg rsp_data;
    int ret;
    transaction_id id;

    ALOGV("handleNanResponse called %p", this);
    memset(&rsp_data, 0, sizeof(rsp_data));
    // get the rsp_data
    ret = getNanResponse(&id, &rsp_data);

    ALOGI("handleNanResponse ret:%d status:%u value:%s response_type:%u", ret, rsp_data.status,
          rsp_data.nan_error, rsp_data.response_type);
    if (ret == 0 && (rsp_data.response_type == NAN_RESPONSE_STATS) && (mStaParam != NULL) &&
        (rsp_data.body.stats_response.stats_type == NAN_STATS_ID_DE_TIMING_SYNC)) {
        /*
           Fill the staParam with appropriate values and return from here.
           No need to call NotifyResponse as the request is for getting the
           STA response
        */
        NanSyncStats* pSyncStats = &rsp_data.body.stats_response.data.sync_stats;
        mStaParam->master_rank = pSyncStats->myRank;
        mStaParam->master_pref = (pSyncStats->myRank & 0xFF00000000000000) >> 56;
        mStaParam->random_factor = (pSyncStats->myRank & 0x00FF000000000000) >> 48;
        mStaParam->hop_count = pSyncStats->currAmHopCount;
        mStaParam->beacon_transmit_time = pSyncStats->currAmBTT;
        mStaParam->ndp_channel_freq = pSyncStats->ndpChannelFreq;

        ALOGI("%s:0x%02x master_pref 0x%02x random_factor 0x%02x hop_count %u Channel", __func__,
              mStaParam->master_pref, mStaParam->random_factor, mStaParam->hop_count,
              mStaParam->ndp_channel_freq);

        return ret;
    }
    // Call the NotifyResponse Handler
    if (ret == 0 && mHandler.NotifyResponse) {
        (*mHandler.NotifyResponse)(id, &rsp_data);
    }
    return ret;
}

void NanCommand::handleNanStatsResponse(NanStatsType stats_type, char* rspBuf,
                                        NanStatsResponse* pRsp, u32 message_len) {
    if (stats_type == NAN_STATS_ID_DE_PUBLISH) {
        NanPublishStats publish_stats;
        if (message_len != sizeof(NanPublishStats)) {
            ALOGE("%s: stats_type = %d invalid stats length = %u expected length = %zu\n", __func__,
                  stats_type, message_len, sizeof(NanPublishStats));
            return;
        }
        FwNanPublishStats* pPubStats = (FwNanPublishStats*)rspBuf;

        publish_stats.validPublishServiceReqMsgs = pPubStats->validPublishServiceReqMsgs;
        publish_stats.validPublishServiceRspMsgs = pPubStats->validPublishServiceRspMsgs;
        publish_stats.validPublishServiceCancelReqMsgs =
                pPubStats->validPublishServiceCancelReqMsgs;
        publish_stats.validPublishServiceCancelRspMsgs =
                pPubStats->validPublishServiceCancelRspMsgs;
        publish_stats.validPublishRepliedIndMsgs = pPubStats->validPublishRepliedIndMsgs;
        publish_stats.validPublishTerminatedIndMsgs = pPubStats->validPublishTerminatedIndMsgs;
        publish_stats.validActiveSubscribes = pPubStats->validActiveSubscribes;
        publish_stats.validMatches = pPubStats->validMatches;
        publish_stats.validFollowups = pPubStats->validFollowups;
        publish_stats.invalidPublishServiceReqMsgs = pPubStats->invalidPublishServiceReqMsgs;
        publish_stats.invalidPublishServiceCancelReqMsgs =
                pPubStats->invalidPublishServiceCancelReqMsgs;
        publish_stats.invalidActiveSubscribes = pPubStats->invalidActiveSubscribes;
        publish_stats.invalidMatches = pPubStats->invalidMatches;
        publish_stats.invalidFollowups = pPubStats->invalidFollowups;
        publish_stats.publishCount = pPubStats->publishCount;
        publish_stats.publishNewMatchCount = pPubStats->publishNewMatchCount;
        publish_stats.pubsubGlobalNewMatchCount = pPubStats->pubsubGlobalNewMatchCount;
        memcpy(&pRsp->data.publish_stats, &publish_stats, sizeof(NanPublishStats));
    } else if (stats_type == NAN_STATS_ID_DE_SUBSCRIBE) {
        NanSubscribeStats sub_stats;
        if (message_len != sizeof(NanSubscribeStats)) {
            ALOGE("%s: stats_type = %d invalid stats length = %u expected length = %zu\n", __func__,
                  stats_type, message_len, sizeof(NanSubscribeStats));
            return;
        }
        FwNanSubscribeStats* pSubStats = (FwNanSubscribeStats*)rspBuf;

        sub_stats.validSubscribeServiceReqMsgs = pSubStats->validSubscribeServiceReqMsgs;
        sub_stats.validSubscribeServiceRspMsgs = pSubStats->validSubscribeServiceRspMsgs;
        sub_stats.validSubscribeServiceCancelReqMsgs =
                pSubStats->validSubscribeServiceCancelReqMsgs;
        sub_stats.validSubscribeServiceCancelRspMsgs =
                pSubStats->validSubscribeServiceCancelRspMsgs;
        sub_stats.validSubscribeTerminatedIndMsgs = pSubStats->validSubscribeTerminatedIndMsgs;
        sub_stats.validSubscribeMatchIndMsgs = pSubStats->validSubscribeMatchIndMsgs;
        sub_stats.validSubscribeUnmatchIndMsgs = pSubStats->validSubscribeUnmatchIndMsgs;
        sub_stats.validSolicitedPublishes = pSubStats->validSolicitedPublishes;
        sub_stats.validMatches = pSubStats->validMatches;
        sub_stats.validFollowups = pSubStats->validFollowups;
        sub_stats.invalidSubscribeServiceReqMsgs = pSubStats->invalidSubscribeServiceReqMsgs;
        sub_stats.invalidSubscribeServiceCancelReqMsgs =
                pSubStats->invalidSubscribeServiceCancelReqMsgs;
        sub_stats.invalidSubscribeFollowupReqMsgs = pSubStats->invalidSubscribeFollowupReqMsgs;
        sub_stats.invalidSolicitedPublishes = pSubStats->invalidSolicitedPublishes;
        sub_stats.invalidMatches = pSubStats->invalidMatches;
        sub_stats.invalidFollowups = pSubStats->invalidFollowups;
        sub_stats.subscribeCount = pSubStats->subscribeCount;
        sub_stats.bloomFilterIndex = pSubStats->bloomFilterIndex;
        sub_stats.subscribeNewMatchCount = pSubStats->subscribeNewMatchCount;
        sub_stats.pubsubGlobalNewMatchCount = pSubStats->pubsubGlobalNewMatchCount;
        memcpy(&pRsp->data.subscribe_stats, &sub_stats, sizeof(NanSubscribeStats));
    } else if (stats_type == NAN_STATS_ID_DE_DW) {
        NanDWStats dw_stats;
        if (message_len != sizeof(NanDWStats)) {
            ALOGE("%s: stats_type = %d invalid stats length = %u expected length = %zu\n", __func__,
                  stats_type, message_len, sizeof(NanDWStats));
            return;
        }
        FwNanMacStats* pMacStats = (FwNanMacStats*)rspBuf;

        dw_stats.validFrames = pMacStats->validFrames;
        dw_stats.validActionFrames = pMacStats->validActionFrames;
        dw_stats.validBeaconFrames = pMacStats->validBeaconFrames;
        dw_stats.ignoredActionFrames = pMacStats->ignoredActionFrames;
        dw_stats.ignoredBeaconFrames = pMacStats->ignoredBeaconFrames;
        dw_stats.invalidFrames = pMacStats->invalidFrames;
        dw_stats.invalidActionFrames = pMacStats->invalidActionFrames;
        dw_stats.invalidBeaconFrames = pMacStats->invalidBeaconFrames;
        dw_stats.invalidMacHeaders = pMacStats->invalidMacHeaders;
        dw_stats.invalidPafHeaders = pMacStats->invalidPafHeaders;
        dw_stats.nonNanBeaconFrames = pMacStats->nonNanBeaconFrames;
        dw_stats.earlyActionFrames = pMacStats->earlyActionFrames;
        dw_stats.inDwActionFrames = pMacStats->inDwActionFrames;
        dw_stats.lateActionFrames = pMacStats->lateActionFrames;
        dw_stats.framesQueued = pMacStats->framesQueued;
        dw_stats.totalTRSpUpdates = pMacStats->totalTRSpUpdates;
        dw_stats.completeByTRSp = pMacStats->completeByTRSp;
        dw_stats.completeByTp75DW = pMacStats->completeByTp75DW;
        dw_stats.completeByTendDW = pMacStats->completeByTendDW;
        dw_stats.lateActionFramesTx = pMacStats->lateActionFramesTx;
        memcpy(&pRsp->data.dw_stats, &dw_stats, sizeof(NanDWStats));
    } else if (stats_type == NAN_STATS_ID_DE_MAC) {
        NanMacStats mac_stats;
        if (message_len != sizeof(NanMacStats)) {
            ALOGE("%s: stats_type = %d invalid stats length = %u expected length = %zu\n", __func__,
                  stats_type, message_len, sizeof(NanMacStats));
            return;
        }
        FwNanMacStats* pMacStats = (FwNanMacStats*)rspBuf;

        mac_stats.validFrames = pMacStats->validFrames;
        mac_stats.validActionFrames = pMacStats->validActionFrames;
        mac_stats.validBeaconFrames = pMacStats->validBeaconFrames;
        mac_stats.ignoredActionFrames = pMacStats->ignoredActionFrames;
        mac_stats.ignoredBeaconFrames = pMacStats->ignoredBeaconFrames;
        mac_stats.invalidFrames = pMacStats->invalidFrames;
        mac_stats.invalidActionFrames = pMacStats->invalidActionFrames;
        mac_stats.invalidBeaconFrames = pMacStats->invalidBeaconFrames;
        mac_stats.invalidMacHeaders = pMacStats->invalidMacHeaders;
        mac_stats.invalidPafHeaders = pMacStats->invalidPafHeaders;
        mac_stats.nonNanBeaconFrames = pMacStats->nonNanBeaconFrames;
        mac_stats.earlyActionFrames = pMacStats->earlyActionFrames;
        mac_stats.inDwActionFrames = pMacStats->inDwActionFrames;
        mac_stats.lateActionFrames = pMacStats->lateActionFrames;
        mac_stats.framesQueued = pMacStats->framesQueued;
        mac_stats.totalTRSpUpdates = pMacStats->totalTRSpUpdates;
        mac_stats.completeByTRSp = pMacStats->completeByTRSp;
        mac_stats.completeByTp75DW = pMacStats->completeByTp75DW;
        mac_stats.completeByTendDW = pMacStats->completeByTendDW;
        mac_stats.lateActionFramesTx = pMacStats->lateActionFramesTx;
        mac_stats.twIncreases = pMacStats->twIncreases;
        mac_stats.twDecreases = pMacStats->twDecreases;
        mac_stats.twChanges = pMacStats->twChanges;
        mac_stats.twHighwater = pMacStats->twHighwater;
        mac_stats.bloomFilterIndex = pMacStats->bloomFilterIndex;
        memcpy(&pRsp->data.mac_stats, &mac_stats, sizeof(NanMacStats));
    } else if (stats_type == NAN_STATS_ID_DE_TIMING_SYNC) {
        NanSyncStats sync_stats;
        if (message_len != sizeof(NanSyncStats)) {
            ALOGE("%s: stats_type = %d invalid stats length = %u expected length = %zu\n", __func__,
                  stats_type, message_len, sizeof(NanSyncStats));
            return;
        }
        FwNanSyncStats* pSyncStats = (FwNanSyncStats*)rspBuf;

        sync_stats.currTsf = pSyncStats->currTsf;
        sync_stats.myRank = pSyncStats->myRank;
        sync_stats.currAmRank = pSyncStats->currAmRank;
        sync_stats.lastAmRank = pSyncStats->lastAmRank;
        sync_stats.currAmBTT = pSyncStats->currAmBTT;
        sync_stats.lastAmBTT = pSyncStats->lastAmBTT;
        sync_stats.currAmHopCount = pSyncStats->currAmHopCount;
        sync_stats.currRole = pSyncStats->currRole;
        sync_stats.currClusterId = pSyncStats->currClusterId;

        sync_stats.timeSpentInCurrRole = pSyncStats->timeSpentInCurrRole;
        sync_stats.totalTimeSpentAsMaster = pSyncStats->totalTimeSpentAsMaster;
        sync_stats.totalTimeSpentAsNonMasterSync = pSyncStats->totalTimeSpentAsNonMasterSync;
        sync_stats.totalTimeSpentAsNonMasterNonSync = pSyncStats->totalTimeSpentAsNonMasterNonSync;
        sync_stats.transitionsToAnchorMaster = pSyncStats->transitionsToAnchorMaster;
        sync_stats.transitionsToMaster = pSyncStats->transitionsToMaster;
        sync_stats.transitionsToNonMasterSync = pSyncStats->transitionsToNonMasterSync;
        sync_stats.transitionsToNonMasterNonSync = pSyncStats->transitionsToNonMasterNonSync;
        sync_stats.amrUpdateCount = pSyncStats->amrUpdateCount;
        sync_stats.amrUpdateRankChangedCount = pSyncStats->amrUpdateRankChangedCount;
        sync_stats.amrUpdateBTTChangedCount = pSyncStats->amrUpdateBTTChangedCount;
        sync_stats.amrUpdateHcChangedCount = pSyncStats->amrUpdateHcChangedCount;
        sync_stats.amrUpdateNewDeviceCount = pSyncStats->amrUpdateNewDeviceCount;
        sync_stats.amrExpireCount = pSyncStats->amrExpireCount;
        sync_stats.mergeCount = pSyncStats->mergeCount;
        sync_stats.beaconsAboveHcLimit = pSyncStats->beaconsAboveHcLimit;
        sync_stats.beaconsBelowRssiThresh = pSyncStats->beaconsBelowRssiThresh;
        sync_stats.beaconsIgnoredNoSpace = pSyncStats->beaconsIgnoredNoSpace;
        sync_stats.beaconsForOurCluster = pSyncStats->beaconsForOtherCluster;
        sync_stats.beaconsForOtherCluster = pSyncStats->beaconsForOtherCluster;
        sync_stats.beaconCancelRequests = pSyncStats->beaconCancelRequests;
        sync_stats.beaconCancelFailures = pSyncStats->beaconCancelFailures;
        sync_stats.beaconUpdateRequests = pSyncStats->beaconUpdateRequests;
        sync_stats.beaconUpdateFailures = pSyncStats->beaconUpdateFailures;
        sync_stats.syncBeaconTxAttempts = pSyncStats->syncBeaconTxAttempts;
        sync_stats.syncBeaconTxFailures = pSyncStats->syncBeaconTxFailures;
        sync_stats.discBeaconTxAttempts = pSyncStats->discBeaconTxAttempts;
        sync_stats.discBeaconTxFailures = pSyncStats->discBeaconTxFailures;
        sync_stats.amHopCountExpireCount = pSyncStats->amHopCountExpireCount;
        sync_stats.ndpChannelFreq = pSyncStats->ndpChannelFreq;
        sync_stats.ndpChannelFreq2 = pSyncStats->ndpChannelFreq2;
        sync_stats.schedUpdateChannelFreq = pSyncStats->schedUpdateChannelFreq;
        memcpy(&pRsp->data.sync_stats, &sync_stats, sizeof(NanSyncStats));
    } else if (stats_type == NAN_STATS_ID_DE) {
        NanDeStats de_stats;
        if (message_len != sizeof(NanDeStats)) {
            ALOGE("%s: stats_type = %d invalid stats length = %u expected length = %zu\n", __func__,
                  stats_type, message_len, sizeof(NanDeStats));
            return;
        }
        FwNanDeStats* pDeStats = (FwNanDeStats*)rspBuf;

        de_stats.validErrorRspMsgs = pDeStats->validErrorRspMsgs;
        de_stats.validTransmitFollowupReqMsgs = pDeStats->validTransmitFollowupReqMsgs;
        de_stats.validTransmitFollowupRspMsgs = pDeStats->validTransmitFollowupRspMsgs;
        de_stats.validFollowupIndMsgs = pDeStats->validFollowupIndMsgs;
        de_stats.validConfigurationReqMsgs = pDeStats->validConfigurationReqMsgs;
        de_stats.validConfigurationRspMsgs = pDeStats->validConfigurationRspMsgs;
        de_stats.validStatsReqMsgs = pDeStats->validStatsReqMsgs;
        de_stats.validStatsRspMsgs = pDeStats->validStatsRspMsgs;
        de_stats.validEnableReqMsgs = pDeStats->validEnableReqMsgs;
        de_stats.validEnableRspMsgs = pDeStats->validEnableRspMsgs;
        de_stats.validDisableReqMsgs = pDeStats->validDisableReqMsgs;
        de_stats.validDisableRspMsgs = pDeStats->validDisableRspMsgs;
        de_stats.validDisableIndMsgs = pDeStats->validDisableIndMsgs;
        de_stats.validEventIndMsgs = pDeStats->validEventIndMsgs;
        de_stats.validTcaReqMsgs = pDeStats->validTcaReqMsgs;
        de_stats.validTcaRspMsgs = pDeStats->validTcaRspMsgs;
        de_stats.validTcaIndMsgs = pDeStats->validTcaIndMsgs;
        de_stats.invalidTransmitFollowupReqMsgs = pDeStats->invalidTransmitFollowupReqMsgs;
        de_stats.invalidConfigurationReqMsgs = pDeStats->invalidConfigurationReqMsgs;
        de_stats.invalidStatsReqMsgs = pDeStats->invalidStatsReqMsgs;
        de_stats.invalidEnableReqMsgs = pDeStats->invalidEnableReqMsgs;
        de_stats.invalidDisableReqMsgs = pDeStats->invalidDisableReqMsgs;
        de_stats.invalidTcaReqMsgs = pDeStats->invalidTcaReqMsgs;
        memcpy(&pRsp->data.de_stats, &de_stats, sizeof(NanDeStats));
    } else {
        ALOGE("Unknown stats_type:%d\n", stats_type);
    }
}

int NanCommand::handleNdpResponse(NanResponseType ndpCmdType, struct nlattr** tb_vendor) {
    // parse the data and call
    // the response callback handler with the populated
    // NanResponseMsg
    NanResponseMsg rsp_data;
    transaction_id id;

    memset(&rsp_data, 0, sizeof(rsp_data));

    if ((!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID]) ||
        (!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE]) ||
        (!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_DRV_RETURN_VALUE])) {
        ALOGE("%s: MTK_WLAN_VENDOR_ATTR_NDP not found", __FUNCTION__);
        return WIFI_ERROR_INVALID_ARGS;
    }

    id = nla_get_u16(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID]);
    ALOGD("%s: Transaction id : val %d", __FUNCTION__, id);

    NanErrorTranslation((NanInternalStatusType)nla_get_u32(
                                tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE]),
                        nla_get_u32(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_DRV_RETURN_VALUE]),
                        &rsp_data, true);
    rsp_data.response_type = ndpCmdType;

    if (ndpCmdType == NAN_DP_INITIATOR_RESPONSE) {
        if (!tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID]) {
            ALOGE("%s: MTK_WLAN_VENDOR_ATTR_NDP not found", __FUNCTION__);
            return WIFI_ERROR_INVALID_ARGS;
        }
        rsp_data.body.data_request_response.ndp_instance_id =
                nla_get_u32(tb_vendor[MTK_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID]);
    }
    // Call the NotifyResponse Handler
    if (mHandler.NotifyResponse) {
        (*mHandler.NotifyResponse)(id, &rsp_data);
    }
    return WIFI_SUCCESS;
}

/* Called from wifi_cleanup(): drop the NAN singleton and its vendor event handlers. */
wifi_error nan_deinit_handler() {
    NanCommand::destroyInstance();
    return WIFI_SUCCESS;
}
