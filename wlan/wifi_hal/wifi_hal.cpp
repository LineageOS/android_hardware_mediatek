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
#include <sys/socket.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#include <linux/rtnetlink.h>
#include <netpacket/packet.h>
#include <linux/filter.h>
#include <linux/errqueue.h>
#include <errno.h>

#include <linux/pkt_sched.h>
#include <netlink/object-api.h>
#include <netlink/netlink.h>
#include <netlink/socket.h>
#include <netlink/attr.h>
#include <netlink/handlers.h>
#include <netlink/msg.h>

#include <dirent.h>
#include <net/if.h>

#include <sys/types.h>
#include <unistd.h>

#include "sync.h"

#define LOG_TAG  "WifiHAL"
#include <log/log.h>

#include <hardware_legacy/wifi_hal.h>
#include <hardware_legacy/rtt.h>
#include "common.h"
#include "cpp_bindings.h"
#include "brcm_version.h"
#include <stdio.h>
#include <string>
#include <vector>
/*
 BUGBUG: normally, libnl allocates ports for all connections it makes; but
 being a static library, it doesn't really know how many other netlink connections
 are made by the same process, if connections come from different shared libraries.
 These port assignments exist to solve that problem - temporarily. We need to fix
 libnl to try and allocate ports across the entire process.
 */

#define WIFI_HAL_CMD_SOCK_PORT       644
#define WIFI_HAL_DEBUG_CMD_SOCK_PORT 646
#define WIFI_HAL_EVENT_SOCK_PORT     645
#define MAX_VIRTUAL_IFACES           6
#define WIFI_HAL_EVENT_BUFFER_NOT_AVAILABLE 105

/*
 * Defines for wifi_wait_for_driver_ready()
 * Specify durations between polls and max wait time
 */
#define POLL_DRIVER_DURATION_US (100000)
#define POLL_DRIVER_MAX_TIME_MS (10000)
#define EVENT_BUF_SIZE 2048
#define C2S(x)  case x: return #x;

static int internal_no_seq_check(nl_msg *msg, void *arg);
static int internal_valid_message_handler(nl_msg *msg, void *arg);
static int wifi_get_multicast_id(wifi_handle handle, const char *name, const char *group);
static int wifi_add_membership(wifi_handle handle, const char *group);
static wifi_error wifi_init_interfaces(wifi_handle handle);
static wifi_error wifi_start_rssi_monitoring(wifi_request_id id, wifi_interface_handle
                        iface, s8 max_rssi, s8 min_rssi, wifi_rssi_event_handler eh);
static wifi_error wifi_stop_rssi_monitoring(wifi_request_id id, wifi_interface_handle iface);
static wifi_error wifi_set_packet_filter(wifi_interface_handle handle,
                            const u8 *program, u32 len);
static wifi_error wifi_read_packet_filter(wifi_interface_handle handle, u32 src_offset,
                            u8 *host_dst, u32 length);
static wifi_error wifi_get_packet_filter_capabilities(wifi_interface_handle handle,
                u32 *version, u32 *max_len);
static wifi_error wifi_configure_nd_offload(wifi_interface_handle iface, u8 enable);
static wifi_error wifi_get_usable_channels(wifi_handle handle, u32 band_mask, u32 iface_mode_mask,
		u32 filter_mask, u32 max_size, u32* size, wifi_usable_channel* channels);
static wifi_error wifi_get_supported_radio_combinations_matrix(wifi_handle handle,
		u32 max_size, u32* size, wifi_radio_combination_matrix *radio_combination_matrix);

static void wifi_cleanup_dynamic_ifaces(wifi_handle handle);
static wifi_error wifi_enable_tx_power_limits(wifi_interface_handle iface,
        bool isEnable);
wifi_error wifi_get_cached_scan_results(wifi_interface_handle iface,
    wifi_cached_scan_result_handler handler);
wifi_error wifi_enable_sta_channel_for_peer_network(wifi_handle handle,
    u32 channelCategoryEnableFlag);
wifi_error wifi_nan_suspend_request(transaction_id id,
    wifi_interface_handle iface, NanSuspendRequest* msg);
wifi_error wifi_nan_resume_request(transaction_id id,
    wifi_interface_handle iface, NanResumeRequest* msg);


typedef enum wifi_attr {
    ANDR_WIFI_ATTRIBUTE_INVALID                    = 0,
    ANDR_WIFI_ATTRIBUTE_NUM_FEATURE_SET            = 1,
    ANDR_WIFI_ATTRIBUTE_FEATURE_SET                = 2,
    ANDR_WIFI_ATTRIBUTE_PNO_RANDOM_MAC_OUI         = 3,
    ANDR_WIFI_ATTRIBUTE_NODFS_SET                  = 4,
    ANDR_WIFI_ATTRIBUTE_COUNTRY                    = 5,
    ANDR_WIFI_ATTRIBUTE_ND_OFFLOAD_VALUE           = 6,
    ANDR_WIFI_ATTRIBUTE_TCPACK_SUP_VALUE           = 7,
    ANDR_WIFI_ATTRIBUTE_LATENCY_MODE               = 8,
    ANDR_WIFI_ATTRIBUTE_RANDOM_MAC                 = 9,
    ANDR_WIFI_ATTRIBUTE_TX_POWER_SCENARIO          = 10,
    ANDR_WIFI_ATTRIBUTE_THERMAL_MITIGATION         = 11,
    ANDR_WIFI_ATTRIBUTE_THERMAL_COMPLETION_WINDOW  = 12,
    ANDR_WIFI_ATTRIBUTE_VOIP_MODE                  = 13,
    ANDR_WIFI_ATTRIBUTE_DTIM_MULTIPLIER            = 14,
    ANDR_WIFI_ATTRIBUTE_CHAN_POLICY                = 15,
     // Add more attribute here
    ANDR_WIFI_ATTRIBUTE_MAX
} wifi_attr_t;

enum wifi_radio_combo_attributes {
    ANDR_WIFI_ATTRIBUTE_RADIO_COMBO_INVALID    = 0,
    ANDR_WIFI_ATTRIBUTE_RADIO_COMBO_MATRIX     = 1,
    // Add more attribute here
    ANDR_WIFI_ATTRIBUTE_RADIO_COMBO_MAX
};

enum wifi_rssi_monitor_attr {
    RSSI_MONITOR_ATTRIBUTE_INVALID	= 0,
    RSSI_MONITOR_ATTRIBUTE_MAX_RSSI	= 1,
    RSSI_MONITOR_ATTRIBUTE_MIN_RSSI	= 2,
    RSSI_MONITOR_ATTRIBUTE_START	= 3,
    // Add more attribute here
    RSSI_MONITOR_ATTRIBUTE_MAX
};

enum wifi_apf_attr {
    APF_ATTRIBUTE_VERSION,
    APF_ATTRIBUTE_MAX_LEN,
    APF_ATTRIBUTE_PROGRAM,
    APF_ATTRIBUTE_PROGRAM_LEN
};

enum apf_request_type {
    GET_APF_CAPABILITIES,
    SET_APF_PROGRAM,
    READ_APF_PROGRAM
};

enum wifi_dscp_attr {
    DSCP_ATTRIBUTE_INVALID	= 0,
    DSCP_ATTRIBUTE_START	= 1,
    DSCP_ATTRIBUTE_END		= 2,
    DSCP_ATTRIBUTE_AC		= 3,
    /* Add more attributes here */
    DSCP_ATTRIBUTE_MAX
};

enum wifi_dscp_request_type {
    SET_DSCP_TABLE,
    RESET_DSCP_TABLE
};

enum wifi_chavoid_attr {
    CHAVOID_ATTRIBUTE_INVALID   = 0,
    CHAVOID_ATTRIBUTE_CNT       = 1,
    CHAVOID_ATTRIBUTE_CONFIG    = 2,
    CHAVOID_ATTRIBUTE_BAND      = 3,
    CHAVOID_ATTRIBUTE_CHANNEL   = 4,
    CHAVOID_ATTRIBUTE_PWRCAP    = 5,
    CHAVOID_ATTRIBUTE_MANDATORY = 6,
    /* Add more attributes here */
    CHAVOID_ATTRIBUTE_MAX
};

enum wifi_usable_channel_attributes {
    USABLECHAN_ATTRIBUTE_INVALID    = 0,
    USABLECHAN_ATTRIBUTE_BAND       = 1,
    USABLECHAN_ATTRIBUTE_IFACE      = 2,
    USABLECHAN_ATTRIBUTE_FILTER     = 3,
    USABLECHAN_ATTRIBUTE_MAX_SIZE   = 4,
    USABLECHAN_ATTRIBUTE_SIZE       = 5,
    USABLECHAN_ATTRIBUTE_CHANNELS   = 6,
    USABLECHAN_ATTRIBUTE_MAX
};

enum wifi_multista_attr {
    MULTISTA_ATTRIBUTE_PRIM_CONN_IFACE,
    MULTISTA_ATTRIBUTE_USE_CASE,
    /* Add more attributes here */
    MULTISTA_ATTRIBUTE_MAX
};

enum multista_request_type {
    SET_PRIMARY_CONNECTION,
    SET_USE_CASE
};

enum wifi_tx_power_limits {
    TX_POWER_CAP_ATTRIBUTE_INVALID    = 0,
    TX_POWER_CAP_ENABLE_ATTRIBUTE     = 1,
    /* Add more attributes here */
    TX_POWER_ATTRIBUTE_MAX
};

/* Initialize/Cleanup */

void wifi_socket_set_local_port(struct nl_sock *sock, uint32_t port)
{
    uint32_t pid = getpid() & 0x3FFFFF;
    nl_socket_set_local_port(sock, pid + (port << 22));
}

static nl_sock * wifi_create_nl_socket(int port)
{
    // ALOGI("Creating socket");
    struct nl_sock *sock = nl_socket_alloc();
    if (sock == NULL) {
        ALOGE("Could not create handle");
        return NULL;
    }

    wifi_socket_set_local_port(sock, port);

    if (nl_connect(sock, NETLINK_GENERIC)) {
        ALOGE("Could not connect handle");
        nl_socket_free(sock);
        return NULL;
    }
    return sock;
}

static const char *IfaceTypeToString(wifi_interface_type iface_type)
{
    switch (iface_type) {
        C2S(WIFI_INTERFACE_TYPE_STA)
        C2S(WIFI_INTERFACE_TYPE_AP)
        C2S(WIFI_INTERFACE_TYPE_P2P)
        C2S(WIFI_INTERFACE_TYPE_NAN)
    default:
        return "UNKNOWN_WIFI_INTERFACE_TYPE";
    }
}

/*initialize function pointer table with Broadcom HHAL API*/
wifi_error init_wifi_vendor_hal_func_table(wifi_hal_fn *fn)
{
    if (fn == NULL) {
        return WIFI_ERROR_UNKNOWN;
    }
    fn->wifi_initialize = wifi_initialize;
    fn->wifi_wait_for_driver_ready = wifi_wait_for_driver_ready;
    fn->wifi_cleanup = wifi_cleanup;
    fn->wifi_event_loop = wifi_event_loop;
    fn->wifi_get_supported_feature_set = wifi_get_supported_feature_set;
    fn->wifi_get_concurrency_matrix = wifi_get_concurrency_matrix;
    fn->wifi_set_scanning_mac_oui =  wifi_set_scanning_mac_oui;
    fn->wifi_get_ifaces = wifi_get_ifaces;
    fn->wifi_get_iface_name = wifi_get_iface_name;
    fn->wifi_start_gscan = wifi_start_gscan;
    fn->wifi_stop_gscan = wifi_stop_gscan;
    fn->wifi_get_cached_gscan_results = wifi_get_cached_gscan_results;
    fn->wifi_set_bssid_hotlist = wifi_set_bssid_hotlist;
    fn->wifi_reset_bssid_hotlist = wifi_reset_bssid_hotlist;
    fn->wifi_set_significant_change_handler = wifi_set_significant_change_handler;
    fn->wifi_reset_significant_change_handler = wifi_reset_significant_change_handler;
    fn->wifi_get_gscan_capabilities = wifi_get_gscan_capabilities;
    fn->wifi_get_link_stats = wifi_get_link_stats;
    fn->wifi_set_link_stats = wifi_set_link_stats;
    fn->wifi_clear_link_stats = wifi_clear_link_stats;
    fn->wifi_get_valid_channels = wifi_get_valid_channels;
    fn->wifi_rtt_range_request_v4 = wifi_rtt_range_request_v4;
    fn->wifi_rtt_range_cancel = wifi_rtt_range_cancel;
    fn->wifi_get_rtt_capabilities_v4 = wifi_get_rtt_capabilities_v4;
    fn->wifi_rtt_get_responder_info = wifi_rtt_get_responder_info;
    fn->wifi_enable_responder = wifi_enable_responder;
    fn->wifi_disable_responder = wifi_disable_responder;
    fn->wifi_set_nodfs_flag = wifi_set_nodfs_flag;
    fn->wifi_start_logging = wifi_start_logging;
    fn->wifi_set_epno_list = wifi_set_epno_list;
    fn->wifi_reset_epno_list = wifi_reset_epno_list;
    fn->wifi_set_country_code = wifi_set_country_code;
    fn->wifi_get_firmware_memory_dump = wifi_get_firmware_memory_dump;
    fn->wifi_set_log_handler = wifi_set_log_handler;
    fn->wifi_reset_log_handler = wifi_reset_log_handler;
    fn->wifi_set_alert_handler = wifi_set_alert_handler;
    fn->wifi_reset_alert_handler = wifi_reset_alert_handler;
    fn->wifi_get_firmware_version = wifi_get_firmware_version;
    fn->wifi_get_ring_buffers_status = wifi_get_ring_buffers_status;
    fn->wifi_get_logger_supported_feature_set = wifi_get_logger_supported_feature_set;
    fn->wifi_get_ring_data = wifi_get_ring_data;
    fn->wifi_get_driver_version = wifi_get_driver_version;
    fn->wifi_start_rssi_monitoring = wifi_start_rssi_monitoring;
    fn->wifi_stop_rssi_monitoring = wifi_stop_rssi_monitoring;
    fn->wifi_configure_nd_offload = wifi_configure_nd_offload;
    fn->wifi_start_sending_offloaded_packet = wifi_start_sending_offloaded_packet;
    fn->wifi_stop_sending_offloaded_packet = wifi_stop_sending_offloaded_packet;
    fn->wifi_start_pkt_fate_monitoring = wifi_start_pkt_fate_monitoring;
    fn->wifi_get_tx_pkt_fates = wifi_get_tx_pkt_fates;
    fn->wifi_get_rx_pkt_fates = wifi_get_rx_pkt_fates;
    fn->wifi_get_packet_filter_capabilities = wifi_get_packet_filter_capabilities;
    fn->wifi_get_wake_reason_stats = wifi_get_wake_reason_stats;
    fn->wifi_set_packet_filter = wifi_set_packet_filter;
    fn->wifi_enable_firmware_roaming = wifi_enable_firmware_roaming;
    fn->wifi_get_roaming_capabilities = wifi_get_roaming_capabilities;
    fn->wifi_configure_roaming = wifi_configure_roaming;
    fn->wifi_nan_register_handler = nan_register_handler;
    fn->wifi_nan_enable_request = nan_enable_request;
    fn->wifi_nan_disable_request = nan_disable_request;
    fn->wifi_nan_publish_request = nan_publish_request;
    fn->wifi_nan_publish_cancel_request = nan_publish_cancel_request;
    fn->wifi_nan_subscribe_request = nan_subscribe_request;
    fn->wifi_nan_subscribe_cancel_request = nan_subscribe_cancel_request;
    fn->wifi_nan_transmit_followup_request = nan_transmit_followup_request;
    fn->wifi_nan_stats_request = nan_stats_request;
    fn->wifi_nan_config_request = nan_config_request;
    fn->wifi_nan_tca_request = nan_tca_request;
    fn->wifi_nan_beacon_sdf_payload_request = nan_beacon_sdf_payload_request;
    fn->wifi_nan_get_version = nan_get_version;
    fn->wifi_nan_get_capabilities = nan_get_capabilities;
    fn->wifi_nan_data_interface_create = nan_data_interface_create;
    fn->wifi_nan_data_interface_delete = nan_data_interface_delete;
    fn->wifi_nan_data_request_initiator = nan_data_request_initiator;
    fn->wifi_nan_data_indication_response = nan_data_indication_response;
    fn->wifi_nan_data_end = nan_data_end;
    fn->wifi_set_latency_mode = wifi_set_latency_mode;
    fn->wifi_select_tx_power_scenario = wifi_select_tx_power_scenario;
    fn->wifi_reset_tx_power_scenario = wifi_reset_tx_power_scenario;
    fn->wifi_read_packet_filter = wifi_read_packet_filter;
    fn->wifi_set_subsystem_restart_handler = wifi_set_subsystem_restart_handler;
    fn->wifi_set_thermal_mitigation_mode = wifi_set_thermal_mitigation_mode;
    fn->wifi_map_dscp_access_category = wifi_map_dscp_access_category;
    fn->wifi_reset_dscp_mapping = wifi_reset_dscp_mapping;
    fn->wifi_virtual_interface_create = wifi_virtual_interface_create;
    fn->wifi_virtual_interface_delete = wifi_virtual_interface_delete;
    fn->wifi_set_coex_unsafe_channels = wifi_set_coex_unsafe_channels;
    fn->wifi_twt_get_capabilities = wifi_twt_get_capabilities;
    fn->wifi_twt_register_events = wifi_twt_register_events;
    fn->wifi_twt_session_setup = wifi_twt_session_setup;
    fn->wifi_twt_session_update = wifi_twt_session_update;
    fn->wifi_twt_session_suspend = wifi_twt_session_suspend;
    fn->wifi_twt_session_resume = wifi_twt_session_resume;
    fn->wifi_twt_session_teardown = wifi_twt_session_teardown;
    fn->wifi_twt_session_get_stats = wifi_twt_session_get_stats;
    fn->wifi_multi_sta_set_primary_connection = wifi_multi_sta_set_primary_connection;
    fn->wifi_multi_sta_set_use_case = wifi_multi_sta_set_use_case;
    fn->wifi_set_voip_mode = wifi_set_voip_mode;
    fn->wifi_set_dtim_config = wifi_set_dtim_config;
    fn->wifi_get_usable_channels = wifi_get_usable_channels;
    fn->wifi_trigger_subsystem_restart = wifi_trigger_subsystem_restart;
    fn->wifi_get_supported_radio_combinations_matrix = wifi_get_supported_radio_combinations_matrix;
    fn->wifi_nan_rtt_chre_enable_request = nan_chre_enable_request;
    fn->wifi_nan_rtt_chre_disable_request = nan_chre_disable_request;
    fn->wifi_chre_register_handler = nan_chre_register_handler;
    fn->wifi_enable_tx_power_limits = wifi_enable_tx_power_limits;
    fn->wifi_get_cached_scan_results = wifi_get_cached_scan_results;
    fn->wifi_enable_sta_channel_for_peer_network = wifi_enable_sta_channel_for_peer_network;
    fn->wifi_nan_suspend_request = wifi_nan_suspend_request;
    fn->wifi_nan_resume_request = wifi_nan_resume_request;
    fn->wifi_nan_pairing_request = nan_pairing_request;
    fn->wifi_nan_pairing_indication_response = nan_pairing_indication_response;
    fn->wifi_nan_bootstrapping_request = nan_bootstrapping_request;
    fn->wifi_nan_bootstrapping_indication_response = nan_bootstrapping_indication_response;
    fn->wifi_nan_pairing_end = nan_pairing_end;
    return WIFI_SUCCESS;
}
#ifdef GOOGLE_WIFI_FW_CONFIG_VERSION_C_WRAPPER
#include <google_wifi_firmware_config_version_info.h>

static void
wifi_check_valid_ota_version(wifi_interface_handle handle)
{
    bool valid = false;
    int32_t default_ver = get_google_default_vendor_wifi_config_version();
    int32_t ota_ver = get_google_ota_updated_wifi_config_version();
    ALOGE("default_ver %d, ota_ver %d", default_ver, ota_ver);

    if (ota_ver > default_ver) {
        valid = verify_google_ota_updated_wifi_config_integrity();
    }

    if (valid) {
        ALOGE("Valid config files of OTA.");
        wifi_hal_ota_update(handle, ota_ver);
    }
    else {
        ALOGE("Do not valid config files of OTA.");
    }
}
#endif // GOOGLE_WIFI_FW_CONFIG_VERSION_C_WRAPPER

hal_info *halInfo = NULL;
wifi_error wifi_pre_initialize(void)
{
    srand(getpid());

    int numIfaceHandles = 0;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;
    wifi_error result = WIFI_SUCCESS;
    wifi_handle handle;
    struct nl_sock *debug_cmd_sock = NULL;

    ALOGE("wifi_pre_initialize");
    ALOGE("--- HAL version: %s ---\n", HAL_VERSION);
    halInfo = (hal_info *)malloc(sizeof(hal_info));
    if (halInfo == NULL) {
        ALOGE("Could not allocate hal_info");
        return WIFI_ERROR_UNKNOWN;
    }

    memset(halInfo, 0, sizeof(*halInfo));

    ALOGI("Creating socket");
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, halInfo->cleanup_socks) == -1) {
        ALOGE("Could not create cleanup sockets");
        free(halInfo);
        return WIFI_ERROR_UNKNOWN;
    }

    struct nl_sock *cmd_sock = wifi_create_nl_socket(WIFI_HAL_CMD_SOCK_PORT);
    if (cmd_sock == NULL) {
        ALOGE("Could not create handle");
        free(halInfo);
        return WIFI_ERROR_UNKNOWN;
    }

    /* Set the socket buffer size */
    if (nl_socket_set_buffer_size(cmd_sock, (256*1024), 0) < 0) {
        ALOGE("Could not set size for cmd_sock: %s",
               strerror(errno));
    } else {
        ALOGV("nl_socket_set_buffer_size successful for cmd_sock");
    }

    debug_cmd_sock = wifi_create_nl_socket(WIFI_HAL_DEBUG_CMD_SOCK_PORT);
    if (debug_cmd_sock == NULL) {
        ALOGE("Could not create debug_cmd_sock");
        free(halInfo);
        return WIFI_ERROR_UNKNOWN;
    }

    if (nl_socket_set_buffer_size(debug_cmd_sock, (256*1024), 0) < 0) {
        ALOGE("Could not set size for debug_cmd_sock: %s", strerror(errno));
    } else {
        ALOGV("nl_socket_set_buffer_size successful for debug_cmd_sock");
    }

    struct nl_sock *event_sock = wifi_create_nl_socket(WIFI_HAL_EVENT_SOCK_PORT);
    if (event_sock == NULL) {
        ALOGE("Could not create handle");
        nl_socket_free(cmd_sock);
        nl_socket_free(debug_cmd_sock);
        free(halInfo);
        return WIFI_ERROR_UNKNOWN;
    }

    /* Set the socket buffer size */
    if (nl_socket_set_buffer_size(event_sock, (4*1024*1024), 0) < 0) {
        ALOGE("Could not set size for event_sock: %s",
               strerror(errno));
    } else {
        ALOGV("nl_socket_set_buffer_size successful for event_sock");
    }

    struct nl_cb *cb = nl_socket_get_cb(event_sock);
    if (cb == NULL) {
        ALOGE("Could not create handle");
        nl_socket_free(cmd_sock);
        nl_socket_free(debug_cmd_sock);
        nl_socket_free(event_sock);
        free(halInfo);
        return WIFI_ERROR_UNKNOWN;
    }

    nl_cb_set(cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM, internal_no_seq_check, halInfo);
    nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, internal_valid_message_handler, halInfo);
    nl_cb_put(cb);

    halInfo->cmd_sock = cmd_sock;
    halInfo->event_sock = event_sock;
    halInfo->debug_cmd_sock = debug_cmd_sock;
    halInfo->clean_up = false;
    halInfo->in_event_loop = false;

    halInfo->event_cb = (cb_info *)malloc(sizeof(cb_info) * DEFAULT_EVENT_CB_SIZE);
    halInfo->alloc_event_cb = DEFAULT_EVENT_CB_SIZE;
    halInfo->num_event_cb = 0;

    halInfo->cmd = (cmd_info *)malloc(sizeof(cmd_info) * DEFAULT_CMD_SIZE);
    halInfo->alloc_cmd = DEFAULT_CMD_SIZE;
    halInfo->num_cmd = 0;

    halInfo->nl80211_family_id = genl_ctrl_resolve(cmd_sock, "nl80211");
    if (halInfo->nl80211_family_id < 0) {
        ALOGE("Could not resolve nl80211 familty id");
        nl_socket_free(cmd_sock);
        nl_socket_free(debug_cmd_sock);
        nl_socket_free(event_sock);
        free(halInfo);
        return WIFI_ERROR_UNKNOWN;
    }

    int debug_nl80211_family_id = genl_ctrl_resolve(debug_cmd_sock, "nl80211");
    if (debug_nl80211_family_id < 0) {
        ALOGE("Could not resolve nl80211 family id on debug_cmd_sock");
        nl_socket_free(cmd_sock);
        nl_socket_free(event_sock);
        nl_socket_free(debug_cmd_sock);
        free(halInfo);
        return WIFI_ERROR_UNKNOWN;
    }
    halInfo->debug_nl80211_family_id = debug_nl80211_family_id;

    pthread_mutex_init(&halInfo->cb_lock, NULL);
    InitResponseLock();

    handle = (wifi_handle) halInfo;

    if (wifi_init_interfaces(handle) != WIFI_SUCCESS) {
        ALOGE("No wifi interface found");
        nl_socket_free(cmd_sock);
        nl_socket_free(debug_cmd_sock);
        nl_socket_free(event_sock);
        pthread_mutex_destroy(&halInfo->cb_lock);
        free(halInfo);
        return WIFI_ERROR_NOT_AVAILABLE;
    }

    if ((wifi_add_membership(handle, "scan") < 0) ||
            (wifi_add_membership(handle, "mlme")  < 0) ||
            (wifi_add_membership(handle, "regulatory") < 0) ||
            (wifi_add_membership(handle, "vendor") < 0)) {
        ALOGE("Add membership failed");
        nl_socket_free(cmd_sock);
        nl_socket_free(debug_cmd_sock);
        nl_socket_free(event_sock);
        pthread_mutex_destroy(&halInfo->cb_lock);
        free(halInfo);
        return WIFI_ERROR_NOT_AVAILABLE;
    }

    ALOGI("Initialized Wifi HAL Successfully; vendor cmd = %d", NL80211_CMD_VENDOR);
    wlan0Handle = wifi_get_wlan_interface((wifi_handle)halInfo, ifaceHandles, numIfaceHandles);

    if (wlan0Handle != NULL) {
        ALOGE("Calling preInit");
        if (!get_halutil_mode()) {
#ifdef GOOGLE_WIFI_FW_CONFIG_VERSION_C_WRAPPER
            (void) wifi_check_valid_ota_version(wlan0Handle);
#endif // GOOGLE_WIFI_FW_CONFIG_VERSION_C_WRAPPER
            result = wifi_hal_preInit(wlan0Handle);
            if (result != WIFI_SUCCESS) {
                ALOGE("wifi_hal_preInit failed");
            }
        }
    }

    return WIFI_SUCCESS;
}

wifi_error wifi_initialize(wifi_handle *handle)
{

    int numIfaceHandles = 0;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;
    wifi_error result = WIFI_SUCCESS;

    ALOGE("wifi_initialize");

    if (halInfo == NULL) {
        result = wifi_pre_initialize();
        if (result != WIFI_SUCCESS) {
            ALOGE("wifi_initialize wifi_pre_initialize failed");
            return result;
        } else {
            ALOGE("wifi_initialize wifi_pre_initialize succeeded");
        }
    }

    *handle = (wifi_handle) halInfo;
    wlan0Handle = wifi_get_wlan_interface((wifi_handle)halInfo, ifaceHandles, numIfaceHandles);

    if (wlan0Handle != NULL) {
        ALOGE("Calling Hal_init");
        if (!get_halutil_mode()) {
            result = wifi_start_hal(wlan0Handle);
            if (result != WIFI_SUCCESS) {
                ALOGE("wifi_start_hal failed");
            }
        }
    } else {
        ALOGI("Not Calling set alert handler as global_iface is NULL");
        return WIFI_ERROR_UNKNOWN;
    }
    return WIFI_SUCCESS;
}

wifi_error wifi_wait_for_driver_ready(void)
{
    // This function will wait to make sure basic client netdev is created
    // Function times out after 10 seconds
    int count = (POLL_DRIVER_MAX_TIME_MS * 1000) / POLL_DRIVER_DURATION_US;
    FILE *fd;
    wifi_error status = WIFI_SUCCESS;

    do {
        if ((fd = fopen("/sys/class/net/wlan0", "r")) != NULL) {
            fclose(fd);
            status = wifi_pre_initialize();
            return status;
        }
        usleep(POLL_DRIVER_DURATION_US);
    } while(--count > 0);

    ALOGE("Timed out waiting on Driver ready ... ");
    return WIFI_ERROR_TIMED_OUT;
}

static int wifi_add_membership(wifi_handle handle, const char *group)
{
    hal_info *info = getHalInfo(handle);

    int id = wifi_get_multicast_id(handle, "nl80211", group);
    if (id < 0) {
        ALOGE("Could not find group %s", group);
        return id;
    }

    int ret = nl_socket_add_membership(info->event_sock, id);
    if (ret < 0) {
        ALOGE("Could not add membership to group %s", group);
    }

    // ALOGI("Successfully added membership for group %s", group);
    return ret;
}

static void internal_cleaned_up_handler(wifi_handle handle)
{
    hal_info *info = getHalInfo(handle);

    ALOGI("internal clean up");

    if (info->cmd_sock != 0) {
        ALOGI("cmd_sock non null. clean up");
        close(info->cleanup_socks[0]);
        info->cleanup_socks[0] = -1;
        close(info->cleanup_socks[1]);
        info->cleanup_socks[1] = -1;
        nl_socket_free(info->cmd_sock);
        nl_socket_free(info->event_sock);
        info->cmd_sock = NULL;
        info->event_sock = NULL;
    }

    if (info->debug_cmd_sock != 0) {
        ALOGI("debug_cmd_sock non null. clean up");
        nl_socket_free(info->debug_cmd_sock);
        info->debug_cmd_sock = NULL;
    }

    if (info->interfaces) {
        for (int i = 0; i < info->num_interfaces; i++) {
            free(info->interfaces[i]);
        }
        free(info->interfaces);
    }

    DestroyResponseLock();
    pthread_mutex_destroy(&info->cb_lock);
    free(info);
    info = NULL;

    ALOGI("Internal cleanup completed");
}

void wifi_internal_module_cleanup()
{
    nan_deinit_handler();
    twt_deinit_handler();
}

void wifi_cleanup(wifi_handle handle, wifi_cleaned_up_handler cleaned_up_handler)
{
    if (!handle) {
        ALOGE("Handle is null");
        return;
    }

    hal_info *info = getHalInfo(handle);
    wifi_error result;

    int numIfaceHandles = 0;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;

    wlan0Handle = wifi_get_wlan_interface((wifi_handle) info, ifaceHandles, numIfaceHandles);

    if (wlan0Handle != NULL) {
        ALOGE("Calling hal cleanup");
        if (!get_halutil_mode()) {
            wifi_cleanup_dynamic_ifaces(handle);
            ALOGI("Cleaned dynamic virtual ifaces\n");
            result = wifi_stop_hal(wlan0Handle);
            if (result != WIFI_SUCCESS) {
                ALOGE("wifi_stop_hal failed");
            }
            ALOGI("wifi_stop_hal success");
        }

    } else {
        ALOGE("Not cleaning up hal as global_iface is NULL");
    }

    /* calling internal modules or cleanup */
    wifi_internal_module_cleanup();
    pthread_mutex_lock(&info->cb_lock);

    int bad_commands = 0;

    ALOGI("event_cb callbacks left: %d ", info->num_event_cb);
    for (int i = 0; i < info->num_event_cb; i++) {
        ALOGI("event_cb cleanup. index:%d", i);
        cb_info *cbi = &(info->event_cb[i]);
        if (!cbi) {
            ALOGE("cbi null for index %d", i);
            continue;
        }
        ALOGI("event_cb cleanup. vendor cmd:%d sub_cmd:%d", cbi->vendor_id, cbi->vendor_subcmd);
        WifiCommand *cmd = (WifiCommand *)cbi->cb_arg;
        if (cmd != NULL) {
            ALOGI("Command left in event_cb %p", cmd);
        }
    }

    ALOGI("Check bad commands: num_cmd:%d bad_commands:%d", info->num_cmd, bad_commands);
    while (info->num_cmd > bad_commands) {
        int num_cmd = info->num_cmd;
        cmd_info *cmdi = &(info->cmd[bad_commands]);
        WifiCommand *cmd = cmdi->cmd;
        if (cmd != NULL) {
            ALOGI("Cancelling command %p: %s", cmd, cmd->getType());
            pthread_mutex_unlock(&info->cb_lock);
            cmd->cancel();
            if (num_cmd == info->num_cmd) {
                ALOGI("Cancelling command %p: %s did not work",
                    cmd, (cmd ? cmd->getType(): ""));
                bad_commands++;
            }
            /* release reference added when command is saved */
            cmd->releaseRef();
            pthread_mutex_lock(&info->cb_lock);
        }
    }

    for (int i = 0; i < info->num_event_cb; i++) {
        cb_info *cbi = &(info->event_cb[i]);
        if (!cbi) {
            ALOGE("cbi null for index %d", i);
            continue;
        }
        WifiCommand *cmd = (WifiCommand *)cbi->cb_arg;
        ALOGE("Leaked command %p", cmd);
    }
    pthread_mutex_unlock(&info->cb_lock);

    /* global func ptr be invalidated and will not call any command from legacy hal */
    if (cleaned_up_handler) {
        ALOGI("cleaned_up_handler to invalidates func ptr");
        cleaned_up_handler(handle);
    } else {
        ALOGI("cleaned up handler is null");
    }

    info->clean_up = true;
    if (info && info->cleanup_socks[0] != -1) {
        if (TEMP_FAILURE_RETRY(write(info->cleanup_socks[0], "Exit", 4)) < 1) {
	    // As a fallback set the cleanup flag to TRUE
	    ALOGE("could not write to the cleanup socket");
	}
    }
    ALOGE("wifi_clean_up done");
}

static int internal_pollin_handler(wifi_handle handle)
{
    hal_info *info = getHalInfo(handle);
    struct nl_cb *cb = nl_socket_get_cb(info->event_sock);
    int res = nl_recvmsgs(info->event_sock, cb);
    if (res) {
        ALOGE("nl_recvmsgs returned %d", res);
    }
    nl_cb_put(cb);
    return res;
}

/* Run event handler */
void wifi_event_loop(wifi_handle handle)
{
    hal_info *info = getHalInfo(handle);
    if (info->in_event_loop) {
        return;
    } else {
        info->in_event_loop = true;
    }

    pollfd pfd[2];
    memset(&pfd[0], 0, sizeof(pollfd) * 2);

    pfd[0].fd = nl_socket_get_fd(info->event_sock);
    pfd[0].events = POLLIN;
    pfd[1].fd = info->cleanup_socks[1];
    pfd[1].events = POLLIN;

    char buf[2048];
    /* TODO: Add support for timeouts */

    do {
        int timeout = -1;                   /* Infinite timeout */
        pfd[0].revents = 0;
        pfd[1].revents = 0;
        // ALOGI("Polling socket");
        int result = TEMP_FAILURE_RETRY(poll(pfd, 2, timeout));
        if (result < 0) {
            // ALOGE("Error polling socket");
        } else if (pfd[0].revents & POLLERR) {
            ALOGE("POLL Error; error no = %d (%s)", errno, strerror(errno));
            if (errno == WIFI_HAL_EVENT_BUFFER_NOT_AVAILABLE) {
                ALOGE("Poll again, No buffer space");
                internal_pollin_handler(handle);
            } else {
                ssize_t result2 = TEMP_FAILURE_RETRY(read(pfd[0].fd, buf, sizeof(buf)));
                ALOGE("Read after POLL returned %zd, error no = %d (%s)", result2,
                        errno, strerror(errno));
            }
        } else if (pfd[0].revents & POLLHUP) {
            ALOGE("Remote side hung up");
            break;
        } else if (pfd[0].revents & POLLIN && !info->clean_up) {
            // ALOGI("Found some events!!!");
            internal_pollin_handler(handle);
        } else if (pfd[1].revents & POLLIN) {
            ALOGI("Got a signal to exit!!!");
        } else {
            ALOGE("Unknown event - %0x, %0x", pfd[0].revents, pfd[1].revents);
        }
    } while (!info->clean_up);

    internal_cleaned_up_handler(handle);
    ALOGE("Exit %s", __FUNCTION__);
}

///////////////////////////////////////////////////////////////////////////////////////

static int internal_no_seq_check(struct nl_msg *msg, void *arg)
{
    return NL_OK;
}

static int internal_valid_message_handler(nl_msg *msg, void *arg)
{
    // ALOGI("got an event");

    wifi_handle handle = (wifi_handle)arg;
    hal_info *info = getHalInfo(handle);

    WifiEvent event(msg);
    int res = event.parse();
    if (res < 0) {
        ALOGE("Failed to parse event: %d", res);
        return NL_SKIP;
    }

    int cmd = event.get_cmd();
    uint32_t vendor_id = 0;
    int subcmd = 0;

    if (cmd == NL80211_CMD_VENDOR) {
        vendor_id = event.get_u32(NL80211_ATTR_VENDOR_ID);
        subcmd = event.get_u32(NL80211_ATTR_VENDOR_SUBCMD);
        ALOGV("event received %s, vendor_id = 0x%0x, subcmd = 0x%0x",
                event.get_cmdString(), vendor_id, subcmd);
    } else {
        // ALOGV("event received %s", event.get_cmdString());
    }

    // ALOGV("event received %s, vendor_id = 0x%0x", event.get_cmdString(), vendor_id);
    // event.log();

    pthread_mutex_lock(&info->cb_lock);

    for (int i = 0; i < info->num_event_cb; i++) {
        if (cmd == info->event_cb[i].nl_cmd) {
            if (cmd == NL80211_CMD_VENDOR
                && ((vendor_id != info->event_cb[i].vendor_id)
                || (subcmd != info->event_cb[i].vendor_subcmd)))
            {
                /* event for a different vendor, ignore it */
                continue;
            }

            cb_info *cbi = &(info->event_cb[i]);
            nl_recvmsg_msg_cb_t cb_func = cbi->cb_func;
            void *cb_arg = cbi->cb_arg;
            WifiCommand *cmd = (WifiCommand *)cbi->cb_arg;
            if (cmd != NULL) {
                cmd->addRef();
            }
            pthread_mutex_unlock(&info->cb_lock);
            if (cb_func)
                (*cb_func)(msg, cb_arg);
            if (cmd != NULL) {
                cmd->releaseRef();
            }

            return NL_OK;
        }
    }

    pthread_mutex_unlock(&info->cb_lock);
    return NL_OK;
}

///////////////////////////////////////////////////////////////////////////////////////

class GetMulticastIdCommand : public WifiCommand
{
private:
    const char *mName;
    const char *mGroup;
    int   mId;
public:
    GetMulticastIdCommand(wifi_handle handle, const char *name, const char *group)
        : WifiCommand("GetMulticastIdCommand", handle, 0)
    {
        mName = name;
        mGroup = group;
        mId = -1;
    }

    int getId() {
        return mId;
    }

    virtual int create() {
        int nlctrlFamily = genl_ctrl_resolve(mInfo->cmd_sock, "nlctrl");
        // ALOGI("ctrl family = %d", nlctrlFamily);
        int ret = mMsg.create(nlctrlFamily, CTRL_CMD_GETFAMILY, 0, 0);
        if (ret < 0) {
            return ret;
        }
        ret = mMsg.put_string(CTRL_ATTR_FAMILY_NAME, mName);
        return ret;
    }

    virtual int handleResponse(WifiEvent& reply) {

        // ALOGI("handling reponse in %s", __func__);

        struct nlattr **tb = reply.attributes();
        struct nlattr *mcgrp = NULL;
        int i;

        if (!tb[CTRL_ATTR_MCAST_GROUPS]) {
            ALOGI("No multicast groups found");
            return NL_SKIP;
        } else {
            // ALOGI("Multicast groups attr size = %d", nla_len(tb[CTRL_ATTR_MCAST_GROUPS]));
        }

        for_each_attr(mcgrp, tb[CTRL_ATTR_MCAST_GROUPS], i) {

            // ALOGI("Processing group");
            struct nlattr *tb2[CTRL_ATTR_MCAST_GRP_MAX + 1];
            nla_parse(tb2, CTRL_ATTR_MCAST_GRP_MAX, (nlattr *)nla_data(mcgrp),
                nla_len(mcgrp), NULL);
            if (!tb2[CTRL_ATTR_MCAST_GRP_NAME] || !tb2[CTRL_ATTR_MCAST_GRP_ID]) {
                continue;
            }

            char *grpName = (char *)nla_data(tb2[CTRL_ATTR_MCAST_GRP_NAME]);
            int grpNameLen = nla_len(tb2[CTRL_ATTR_MCAST_GRP_NAME]);

            // ALOGI("Found group name %s", grpName);

            if (strncmp(grpName, mGroup, grpNameLen) != 0)
                continue;

            mId = nla_get_u32(tb2[CTRL_ATTR_MCAST_GRP_ID]);
            break;
        }

        return NL_SKIP;
    }

};

class SetPnoMacAddrOuiCommand : public WifiCommand {

private:
    byte *mOui;
    feature_set *fset;
    feature_set *feature_matrix;
    int *fm_size;
    int set_size_max;
public:
    SetPnoMacAddrOuiCommand(wifi_interface_handle handle, oui scan_oui)
        : WifiCommand("SetPnoMacAddrOuiCommand", handle, 0)
    {
        mOui = scan_oui;
        fset = NULL;
        feature_matrix = NULL;
        fm_size = NULL;
        set_size_max = 0;
    }

    int createRequest(WifiRequest& request, int subcmd, byte *scan_oui) {
        int result = request.create(GOOGLE_OUI, subcmd);
        if (result < 0) {
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put(ANDR_WIFI_ATTRIBUTE_PNO_RANDOM_MAC_OUI, scan_oui, DOT11_OUI_LEN);
        if (result < 0) {
            return result;
        }

        request.attr_end(data);
        return WIFI_SUCCESS;

    }

    int start() {
        ALOGD("Sending mac address OUI");
        WifiRequest request(familyId(), ifaceId());
        int result = createRequest(request, WIFI_SUBCMD_SET_PNO_RANDOM_MAC_OUI, mOui);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to create request; result = %d", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to set scanning mac OUI; result = %d", result);
        }

        return result;
    }
protected:
    virtual int handleResponse(WifiEvent& reply) {
         ALOGD("Request complete!");
        /* Nothing to do on response! */
        return NL_SKIP;
    }
};

class SetNodfsCommand : public WifiCommand {

private:
    u32 mNoDfs;
public:
    SetNodfsCommand(wifi_interface_handle handle, u32 nodfs)
        : WifiCommand("SetNodfsCommand", handle, 0) {
        mNoDfs = nodfs;
    }
    virtual int create() {
        int ret;

        ret = mMsg.create(GOOGLE_OUI, WIFI_SUBCMD_NODFS_SET);
        if (ret < 0) {
            ALOGE("Can't create message to send to driver - %d", ret);
            return ret;
        }

        nlattr *data = mMsg.attr_start(NL80211_ATTR_VENDOR_DATA);
        ret = mMsg.put_u32(ANDR_WIFI_ATTRIBUTE_NODFS_SET, mNoDfs);
        if (ret < 0) {
             return ret;
        }

        mMsg.attr_end(data);
        return WIFI_SUCCESS;
    }
};

class SetCountryCodeCommand : public WifiCommand {
private:
    const char *mCountryCode;
public:
    SetCountryCodeCommand(wifi_interface_handle handle, const char *country_code)
        : WifiCommand("SetCountryCodeCommand", handle, 0) {
        mCountryCode = country_code;
        }
    virtual int create() {
        int ret;

        ret = mMsg.create(GOOGLE_OUI, WIFI_SUBCMD_SET_COUNTRY_CODE);
        if (ret < 0) {
             ALOGE("Can't create message to send to driver - %d", ret);
             return ret;
        }

        nlattr *data = mMsg.attr_start(NL80211_ATTR_VENDOR_DATA);
        ret = mMsg.put_string(ANDR_WIFI_ATTRIBUTE_COUNTRY, mCountryCode);
        if (ret < 0) {
            return ret;
        }

        mMsg.attr_end(data);
        return WIFI_SUCCESS;

    }
};

class SetRSSIMonitorCommand : public WifiCommand {
private:
    s8 mMax_rssi;
    s8 mMin_rssi;
    wifi_rssi_event_handler mHandler;
public:
    SetRSSIMonitorCommand(wifi_request_id id, wifi_interface_handle handle,
                s8 max_rssi, s8 min_rssi, wifi_rssi_event_handler eh)
        : WifiCommand("SetRSSIMonitorCommand", handle, id), mMax_rssi(max_rssi), mMin_rssi
        (min_rssi), mHandler(eh)
        {
            ALOGI("SetRSSIMonitorCommand %p created", this);
        }

   virtual ~SetRSSIMonitorCommand() {
        /*
         * Mostly, this call will be no effect. However, this could be valid
         * when object destroy without calling unregisterVendorHandler().
         * This is added to protect hal crash due to use-after-free.
         */
        ALOGI("Try to remove event handler if exist, vendor 0x%0x, subcmd 0x%x",
            GOOGLE_OUI, GOOGLE_RSSI_MONITOR_EVENT);
        unregisterVendorHandler(GOOGLE_OUI, GOOGLE_RSSI_MONITOR_EVENT);
        ALOGI("SetRSSIMonitorCommand %p destroyed", this);
   }

   virtual void addRef() {
        int refs = __sync_add_and_fetch(&mRefs, 1);
        ALOGI("addRef: WifiCommand %p has %d references", this, refs);
   }

   virtual void releaseRef() {
        int refs = __sync_sub_and_fetch(&mRefs, 1);
        if (refs == 0) {
            ALOGI("releaseRef: WifiCommand %p has deleted", this);
            delete this;
        } else {
            ALOGI("releaseRef: WifiCommand %p has %d references", this, refs);
        }
   }

   int createRequest(WifiRequest& request, int enable) {
        int result = request.create(GOOGLE_OUI, WIFI_SUBCMD_SET_RSSI_MONITOR);
        if (result < 0) {
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(RSSI_MONITOR_ATTRIBUTE_MAX_RSSI, (enable ? mMax_rssi: 0));
        if (result < 0) {
            return result;
        }
        ALOGD("create request");
        result = request.put_u32(RSSI_MONITOR_ATTRIBUTE_MIN_RSSI, (enable? mMin_rssi: 0));
        if (result < 0) {
            return result;
        }
        result = request.put_u32(RSSI_MONITOR_ATTRIBUTE_START, enable);
        if (result < 0) {
            return result;
        }
        request.attr_end(data);
        return result;
    }

    int start() {
        WifiRequest request(familyId(), ifaceId());
        int result = createRequest(request, 1);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to create request; result = %d", result);
            return result;
        }

        registerVendorHandler(GOOGLE_OUI, GOOGLE_RSSI_MONITOR_EVENT);
        ALOGI("Register GOOGLE_RSSI_MONITOR_EVENT handler");

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            unregisterVendorHandler(GOOGLE_OUI, GOOGLE_RSSI_MONITOR_EVENT);
            ALOGE("Failed to set RSSI Monitor, result = %d", result);
            return result;
        }

        ALOGI("Successfully set RSSI monitoring");
        return result;
    }

    virtual int cancel() {
        unregisterVendorHandler(GOOGLE_OUI, GOOGLE_RSSI_MONITOR_EVENT);

        WifiRequest request(familyId(), ifaceId());
        int result = createRequest(request, 0);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to create request; result = %d", result);
        } else {
            result = requestResponse(request);
            if (result != WIFI_SUCCESS) {
                ALOGE("failed to stop RSSI monitoring = %d", result);
            }
        }
        return WIFI_SUCCESS;
    }

    virtual int handleResponse(WifiEvent& reply) {
        /* Nothing to do on response! */
        return NL_SKIP;
    }

   virtual int handleEvent(WifiEvent& event) {
        ALOGI("Got a RSSI monitor event");

        nlattr *vendor_data = event.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = event.get_vendor_data_len();

        if (vendor_data == NULL || len == 0) {
            ALOGI("RSSI monitor: No data");
            return NL_SKIP;
        }
        /* driver<->HAL event structure */
        #define RSSI_MONITOR_EVT_VERSION   1
        typedef struct {
            u8 version;
            s8 cur_rssi;
            mac_addr BSSID;
        } rssi_monitor_evt;

        rssi_monitor_evt *data = (rssi_monitor_evt *)event.get_vendor_data();

        if (data->version != RSSI_MONITOR_EVT_VERSION) {
            ALOGI("Event version mismatch %d, expected %d", data->version, RSSI_MONITOR_EVT_VERSION);
            return NL_SKIP;
        }

        if (*mHandler.on_rssi_threshold_breached) {
            (*mHandler.on_rssi_threshold_breached)(id(), data->BSSID, data->cur_rssi);
        } else {
            ALOGW("No RSSI monitor handler registered");
        }

        return NL_SKIP;
    }

};

class AndroidPktFilterCommand : public WifiCommand {
    private:
        const u8* mProgram;
        u8* mReadProgram;
        u32 mProgramLen;
        u32* mVersion;
        u32* mMaxLen;
        int mReqType;
    public:
        AndroidPktFilterCommand(wifi_interface_handle handle,
                u32* version, u32* max_len)
            : WifiCommand("AndroidPktFilterCommand", handle, 0),
                    mVersion(version), mMaxLen(max_len),
                    mReqType(GET_APF_CAPABILITIES)
        {
            mProgram = NULL;
            mProgramLen = 0;
            mReadProgram = NULL;
        }

        AndroidPktFilterCommand(wifi_interface_handle handle,
                const u8* program, u32 len)
            : WifiCommand("AndroidPktFilterCommand", handle, 0),
                    mProgram(program), mProgramLen(len),
                    mReqType(SET_APF_PROGRAM)
        {
            mVersion = NULL;
            mMaxLen = NULL;
            mReadProgram = NULL;
        }

        AndroidPktFilterCommand(wifi_interface_handle handle,
            u8* host_dst, u32 length)
            : WifiCommand("AndroidPktFilterCommand", handle, 0),
                mReadProgram(host_dst), mProgramLen(length),
                mReqType(READ_APF_PROGRAM)
        {
            mProgram = NULL;
            mVersion = NULL;
            mMaxLen = NULL;
        }

    int createRequest(WifiRequest& request) {
        if (mReqType == SET_APF_PROGRAM) {
            ALOGI("%s: APF set program request\n", __FUNCTION__);
            return createSetPktFilterRequest(request);
        } else if (mReqType == GET_APF_CAPABILITIES) {
            ALOGI("%s: APF get capabilities request\n", __FUNCTION__);
	    return createGetPktFilterCapabilitesRequest(request);
        } else if (mReqType == READ_APF_PROGRAM) {
            ALOGI("%s: APF read packet filter request\n", __FUNCTION__);
            return createReadPktFilterRequest(request);
        } else {
            ALOGE("%s Unknown APF request\n", __FUNCTION__);
            return WIFI_ERROR_NOT_SUPPORTED;
        }
        return WIFI_SUCCESS;
    }

    int createSetPktFilterRequest(WifiRequest& request) {
        u8 *program = new u8[mProgramLen];
        int result;

        NULL_CHECK_RETURN(program, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);

        ALOGI("mProgramLen : %d\n", mProgramLen);
        if (mProgramLen > NL_MSG_DEFAULT_LEN) {
            result = request.create_custom_len(GOOGLE_OUI, APF_SUBCMD_SET_FILTER,
                    NL_MSG_MAX_LEN);
        } else {
            result = request.create(GOOGLE_OUI, APF_SUBCMD_SET_FILTER);
        }
        if (result < 0) {
            ALOGE("Failed to create cmd: %d, err %d\n", APF_SUBCMD_SET_FILTER, result);
            delete[] program;
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(APF_ATTRIBUTE_PROGRAM_LEN, mProgramLen);
        if (result < 0) {
            ALOGE("Failed to put the program_len %d, err %d\n", mProgramLen, result);
            goto exit;
        }
        memcpy(program, mProgram, mProgramLen);
        result = request.put(APF_ATTRIBUTE_PROGRAM, program, mProgramLen);
        if (result < 0) {
            ALOGE("Failed to copy program_ptr %d, err %d\n", mProgramLen, result);
            goto exit;
        }
exit:   request.attr_end(data);
        delete[] program;
        return result;
    }

    int createGetPktFilterCapabilitesRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, APF_SUBCMD_GET_CAPABILITIES);
        if (result < 0) {
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        request.attr_end(data);
        return result;
    }

    int createReadPktFilterRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, APF_SUBCMD_READ_FILTER);
            if (result < 0) {
                return result;
            }
            nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
            request.attr_end(data);
            return result;
    }

    int start() {
        WifiRequest request(familyId(), ifaceId());
        int result = createRequest(request);
        if (result < 0) {
            ALOGI("CreateRequest failed for APF, result = %d", result);
            return result;
        }
        result = requestResponse(request);
        if (result < 0) {
            ALOGI("Request Response failed for APF, result = %d", result);
            return result;
        }
        return result;
    }

    int cancel() {
        return WIFI_SUCCESS;
    }

    int handleResponse(WifiEvent& reply) {
        ALOGE("In SetAPFCommand::handleResponse mReqType %d\n", mReqType);

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        int id = reply.get_vendor_id();
        int subcmd = reply.get_vendor_subcmd();

        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = reply.get_vendor_data_len();

        ALOGI("Id = %0x, subcmd = %d, len = %d", id, subcmd, len);
        if (vendor_data == NULL || len == 0) {
            ALOGE("no vendor data in SetAPFCommand response; ignoring it");
            return NL_SKIP;
        }
        if (mReqType == SET_APF_PROGRAM) {
            ALOGE("Response received for set packet filter command\n");
        } else if (mReqType == GET_APF_CAPABILITIES) {
            *mVersion = 0;
            *mMaxLen = 0;
            ALOGD("Response received for get packet filter capabilities command\n");
            for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
                if (it.get_type() == APF_ATTRIBUTE_VERSION) {
                    *mVersion = it.get_u32();
                    ALOGI("APF version is %d\n", *mVersion);
                } else if (it.get_type() == APF_ATTRIBUTE_MAX_LEN) {
                    *mMaxLen = it.get_u32();
                    ALOGI("APF max len is %d\n", *mMaxLen);
                } else {
                    ALOGE("Ignoring invalid attribute type = %d, size = %d",
                            it.get_type(), it.get_len());
                }
            }
        } else if (mReqType == READ_APF_PROGRAM) {
            ALOGD("Read packet filter, mProgramLen = %d\n", mProgramLen);
            for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
                if (it.get_type() == APF_ATTRIBUTE_PROGRAM) {
                    u8 *buffer = NULL;
                    buffer = (u8 *)it.get_data();
                    memcpy(mReadProgram, buffer, mProgramLen);
                } else if (it.get_type() == APF_ATTRIBUTE_PROGRAM_LEN) {
                    int apf_length = it.get_u32();
                    ALOGD("apf program length = %d\n", apf_length);
                }
            }
        }
        return NL_OK;
    }

    int handleEvent(WifiEvent& event) {
        /* No Event to receive for APF commands */
        return NL_SKIP;
    }
};

class SetNdoffloadCommand : public WifiCommand {

private:
    u8 mEnable;
public:
    SetNdoffloadCommand(wifi_interface_handle handle, u8 enable)
        : WifiCommand("SetNdoffloadCommand", handle, 0) {
        mEnable = enable;
    }
    virtual int create() {
        int ret;

        ret = mMsg.create(GOOGLE_OUI, WIFI_SUBCMD_CONFIG_ND_OFFLOAD);
        if (ret < 0) {
            ALOGE("Can't create message to send to driver - %d", ret);
            return ret;
        }

        nlattr *data = mMsg.attr_start(NL80211_ATTR_VENDOR_DATA);
        ret = mMsg.put_u8(ANDR_WIFI_ATTRIBUTE_ND_OFFLOAD_VALUE, mEnable);
        if (ret < 0) {
             return ret;
        }

        mMsg.attr_end(data);
        return WIFI_SUCCESS;
    }
};

class GetFeatureSetCommand : public WifiCommand {

private:
    int feature_type;
    feature_set *fset;
    feature_set *feature_matrix;
    int *fm_size;
    int set_size_max;
public:
    GetFeatureSetCommand(wifi_interface_handle handle, int feature, feature_set *set,
         feature_set set_matrix[], int *size, int max_size)
        : WifiCommand("GetFeatureSetCommand", handle, 0) {
        feature_type = feature;
        fset = set;
        feature_matrix = set_matrix;
        fm_size = size;
        set_size_max = max_size;
    }

    virtual int create() {
        int ret;

        if(feature_type == ANDR_WIFI_ATTRIBUTE_NUM_FEATURE_SET) {
            ret = mMsg.create(GOOGLE_OUI, WIFI_SUBCMD_GET_FEATURE_SET);
        } else if (feature_type == ANDR_WIFI_ATTRIBUTE_FEATURE_SET) {
            ret = mMsg.create(GOOGLE_OUI, WIFI_SUBCMD_GET_FEATURE_SET_MATRIX);
        } else {
            ALOGE("Unknown feature type %d", feature_type);
            return -1;
        }

        if (ret < 0) {
            ALOGE("Can't create message to send to driver - %d", ret);
        }

        return ret;
    }

protected:
    virtual int handleResponse(WifiEvent& reply) {

        ALOGV("In GetFeatureSetCommand::handleResponse");

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        int id = reply.get_vendor_id();
        int subcmd = reply.get_vendor_subcmd();

        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = reply.get_vendor_data_len();

        ALOGV("Id = %0x, subcmd = %d, len = %d", id, subcmd, len);
        if (vendor_data == NULL || len == 0) {
            ALOGE("no vendor data in GetFeatureSetCommand response; ignoring it");
            return NL_SKIP;
        }
        if(feature_type == ANDR_WIFI_ATTRIBUTE_NUM_FEATURE_SET) {
            void *data = reply.get_vendor_data();
            if(!fset) {
                ALOGE("Buffers pointers not set");
                return NL_SKIP;
            }
            memcpy(fset, data, min(len, (int) sizeof(*fset)));
        } else {
            int num_features_set = 0;
            int i = 0;

            if(!feature_matrix || !fm_size) {
                ALOGE("Buffers pointers not set");
                return NL_SKIP;
            }

            for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
                if (it.get_type() == ANDR_WIFI_ATTRIBUTE_NUM_FEATURE_SET) {
                    num_features_set = it.get_u32();
                    ALOGV("Got feature list with %d concurrent sets", num_features_set);
                    if(set_size_max && (num_features_set > set_size_max))
                        num_features_set = set_size_max;
                    *fm_size = num_features_set;
                } else if ((it.get_type() == ANDR_WIFI_ATTRIBUTE_FEATURE_SET) &&
                             i < num_features_set) {
                    feature_matrix[i] = it.get_u32();
                    i++;
                } else {
                    ALOGW("Ignoring invalid attribute type = %d, size = %d",
                            it.get_type(), it.get_len());
                }
            }

        }
        return NL_OK;
    }

};
/////////////////////////////////////////////////////////////////////
class GetRadioComboCommand : public WifiCommand {
private:
    wifi_radio_combination_matrix *rcmatrix;
    u32* rc_size;
    u32 set_size_max;
    int ret = 0;

public:
    GetRadioComboCommand(wifi_interface_handle handle, u32 max_size, u32* size,
        wifi_radio_combination_matrix *radio_combination_matrix)
        : WifiCommand("GetRadioComboCommand", handle, 0), rcmatrix(radio_combination_matrix),
        rc_size(size), set_size_max(max_size)
    {
    }

    virtual int createRequest(WifiRequest& mMsg) {
        ret = mMsg.create(GOOGLE_OUI, WIFI_SUBCMD_GET_RADIO_COMBO_MATRIX);
        if (ret < 0) {
            ALOGE("Can't create message to send to driver - %d", ret);
        }
        nlattr *data = mMsg.attr_start(NL80211_ATTR_VENDOR_DATA);
        mMsg.attr_end(data);

        return ret;
    }

    int start() {
        WifiRequest request(familyId(), ifaceId());
        ret = createRequest(request);
        if (ret < 0) {
            ALOGE("Request failed for radio_combo_matrix, result = %d", ret);
            return ret;
        }
        ret = requestResponse(request);
        if (ret < 0) {
            ALOGE("Request Response failed for radio_combo_matrix, result = %d", ret);
            return ret;
        }
        ALOGD("Done! %s", __FUNCTION__);
        return ret;
    }

protected:
   virtual int handleResponse(WifiEvent& reply) {
        ALOGD("In GetRadioComboCommand::handleResponse");
        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        int id = reply.get_vendor_id();
        int subcmd = reply.get_vendor_subcmd();

        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = reply.get_vendor_data_len();

        ALOGV("Id = %0x, subcmd = %d, len = %d", id, subcmd, len);
        if (vendor_data == NULL || len == 0) {
            ALOGE("no vendor data in GetRadioComboCommand response; ignoring it");
            return NL_SKIP;
        }

        for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
            if (it.get_type() == ANDR_WIFI_ATTRIBUTE_RADIO_COMBO_MATRIX) {
                void *data = it.get_data();
                *rc_size = it.get_len();
                if (!data || !*rc_size) {
                    ALOGE("Buffers pointers not set");
                    return NL_SKIP;
                }
                if (set_size_max < *rc_size) {
                    ALOGE("Unexpected buffers size");
                    return NL_SKIP;
                }
                memcpy(rcmatrix, data, min(len, *rc_size));
            } else {
                ALOGW("Ignoring invalid attribute type = %d, size = %d",
                        it.get_type(), it.get_len());
            }
        }

        ALOGD("GetRadioComboCommand::Success");
        return NL_OK;
    }
};
/////////////////////////////////////////////////////////////////////

class SetLatencyModeCommand : public WifiCommand {
private:
    u32 mLatencyMode;
public:
    SetLatencyModeCommand(wifi_interface_handle handle, u32 LatencyMode)
        : WifiCommand("SetLatencyModeCommand", handle, 0) {
            mLatencyMode = LatencyMode;
    }

    virtual int create() {
        int ret;

        /* Check for invalid latency Mode */
        if ((mLatencyMode != WIFI_LATENCY_MODE_NORMAL) &&
            (mLatencyMode != WIFI_LATENCY_MODE_LOW)) {
            ALOGE("SetLatencyModeCommand: Invalid mode: %d", mLatencyMode);
            return WIFI_ERROR_UNKNOWN;
        }

        ret = mMsg.create(GOOGLE_OUI, WIFI_SUBCMD_SET_LATENCY_MODE);
        if (ret < 0) {
            ALOGE("Can't create message to send to driver - %d", ret);
            return ret;
        }

        nlattr *data = mMsg.attr_start(NL80211_ATTR_VENDOR_DATA);
        ret = mMsg.put_u32(ANDR_WIFI_ATTRIBUTE_LATENCY_MODE, mLatencyMode);
        if (ret < 0) {
            return ret;
        }

        mMsg.attr_end(data);
        return WIFI_SUCCESS;
    }
};
static int wifi_get_multicast_id(wifi_handle handle, const char *name, const char *group)
{
    GetMulticastIdCommand cmd(handle, name, group);
    int res = cmd.requestResponse();
    if (res < 0)
        return res;
    else
        return cmd.getId();
}

/////////////////////////////////////////////////////////////////////////

static bool is_wifi_interface(const char *name)
{
    if (strncmp(name, "wlan", 4) != 0 && strncmp(name, "swlan", 5) != 0 &&
        strncmp(name, "p2p", 3) != 0 && strncmp(name, "aware", 5) != 0) {
        /* not a wifi interface; ignore it */
        return false;
    } else {
        return true;
    }
}

static int get_interface(const char *name, interface_info *info)
{
    int size = 0;
    size = strlcpy(info->name, name, sizeof(info->name));
    if (size >= sizeof(info->name)) {
        return WIFI_ERROR_OUT_OF_MEMORY;
    }
    info->id = if_nametoindex(name);
    // ALOGI("found an interface : %s, id = %d", name, info->id);
    return WIFI_SUCCESS;
}

wifi_error wifi_init_interfaces(wifi_handle handle)
{
    hal_info *info = (hal_info *)handle;

    struct dirent *de;

    DIR *d = opendir("/sys/class/net");
    if (d == 0)
        return WIFI_ERROR_UNKNOWN;

    int n = 0;
    while ((de = readdir(d))) {
        if (de->d_name[0] == '.')
            continue;
        if (is_wifi_interface(de->d_name) ) {
            n++;
        }
    }

    closedir(d);

    if (n == 0)
        return WIFI_ERROR_NOT_AVAILABLE;

    d = opendir("/sys/class/net");
    if (d == 0)
        return WIFI_ERROR_UNKNOWN;

    /* Have place holder for 3 virtual interfaces */
    n += MAX_VIRTUAL_IFACES;
    info->interfaces = (interface_info **)calloc(n, sizeof(interface_info *) * n);
    if (!info->interfaces) {
        info->num_interfaces = 0;
        closedir(d);
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    int i = 0;
    while ((de = readdir(d))) {
        if (de->d_name[0] == '.')
            continue;
        if (is_wifi_interface(de->d_name)) {
            interface_info *ifinfo = (interface_info *)malloc(sizeof(interface_info));
            if (!ifinfo) {
                if (info->interfaces) {
                    for (int j = 0; j < i; j++) {
                        free(info->interfaces[j]);
                    }
                }
                free(info->interfaces);
                info->num_interfaces = 0;
                closedir(d);
                return WIFI_ERROR_OUT_OF_MEMORY;
            }
            memset(ifinfo, 0, sizeof(interface_info));
            if (get_interface(de->d_name, ifinfo) != WIFI_SUCCESS) {
                free(ifinfo);
                continue;
            }
            /* Mark as static iface */
            ifinfo->is_virtual = false;
            ifinfo->handle = handle;
            info->interfaces[i] = ifinfo;
            i++;
        }
    }

    closedir(d);

    info->num_interfaces = i;
    info->max_num_interfaces = n;
    return WIFI_SUCCESS;
}

wifi_error wifi_get_ifaces(wifi_handle handle, int *num, wifi_interface_handle **interfaces)
{
    hal_info *info = (hal_info *)handle;

    *interfaces = (wifi_interface_handle *)info->interfaces;
    *num = info->num_interfaces;

    return WIFI_SUCCESS;
}

wifi_error wifi_add_iface_hal_info(wifi_handle handle, const char* ifname)
{
    hal_info *info = NULL;
    int i = 0;

    info = (hal_info *)handle;
    if (info == NULL) {
        ALOGE("Could not find info\n");
        return WIFI_ERROR_UNKNOWN;
    }

    ALOGI("%s: add interface_info for iface: %s\n", __FUNCTION__, ifname);
    if (info->num_interfaces == MAX_VIRTUAL_IFACES) {
        ALOGE("No space. max limit reached for virtual interfaces %d\n", info->num_interfaces);
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    interface_info *ifinfo = (interface_info *)malloc(sizeof(interface_info));
    if (!ifinfo) {
        free(info->interfaces);
        info->num_interfaces = 0;
        return WIFI_ERROR_OUT_OF_MEMORY;
    }

    ifinfo->handle = handle;
    while (i < info->max_num_interfaces) {
        if (info->interfaces[i] == NULL) {
            if (get_interface(ifname, ifinfo) != WIFI_SUCCESS) {
                continue;
            }
            ifinfo->is_virtual = true;
            info->interfaces[i] = ifinfo;
            info->num_interfaces++;
            ALOGI("%s: Added iface: %s at the index %d\n", __FUNCTION__, ifname, i);
            break;
        }
        i++;
    }
    return WIFI_SUCCESS;
}

wifi_error wifi_clear_iface_hal_info(wifi_handle handle, const char* ifname)
{
    hal_info *info = (hal_info *)handle;
    int i = 0;

    ALOGI("%s: clear hal info for iface: %s\n", __FUNCTION__, ifname);
    while (i < info->max_num_interfaces) {
        if ((info->interfaces[i] != NULL) &&
            strncmp(info->interfaces[i]->name, ifname,
            sizeof(info->interfaces[i]->name)) == 0) {
            memset(info->interfaces[i], 0, sizeof(info->interfaces[i]));
            free(info->interfaces[i]);
            info->interfaces[i] = NULL;
            info->num_interfaces--;
            ALOGI("%s: Cleared the index = %d for iface: %s\n", __FUNCTION__, i, ifname);
            break;
        }
        i++;
    }
    if (i < info->num_interfaces) {
        for (int j = i; j < info->num_interfaces; j++) {
            info->interfaces[j] = info->interfaces[j+1];
        }
        info->interfaces[info->num_interfaces] = NULL;
    }
    return WIFI_SUCCESS;
}

wifi_interface_handle wifi_get_wlan_interface(wifi_handle info, wifi_interface_handle *ifaceHandles, int numIfaceHandles)
{
    char buf[EVENT_BUF_SIZE];
    wifi_interface_handle wlan0Handle;
    wifi_error res = wifi_get_ifaces((wifi_handle)info, &numIfaceHandles, &ifaceHandles);
    if (res < 0) {
        return NULL;
    }
    for (int i = 0; i < numIfaceHandles; i++) {
        if (wifi_get_iface_name(ifaceHandles[i], buf, sizeof(buf)) == WIFI_SUCCESS) {
            if (strncmp(buf, "wlan0", 5) == 0) {
                ALOGI("found interface %s\n", buf);
                wlan0Handle = ifaceHandles[i];
                return wlan0Handle;
            }
        }
    }
    return NULL;
}
wifi_error wifi_get_iface_name(wifi_interface_handle handle, char *name, size_t size)
{
    interface_info *info = (interface_info *)handle;
    strncpy(name, info->name, (IFNAMSIZ));
    name[IFNAMSIZ - 1] = '\0';
    return WIFI_SUCCESS;
}

wifi_interface_handle wifi_get_iface_handle(wifi_handle handle, char *name)
{
    char buf[EVENT_BUF_SIZE];
    wifi_interface_handle *ifaceHandles;
    int numIfaceHandles;
    wifi_interface_handle ifHandle;

    wifi_error res = wifi_get_ifaces((wifi_handle)handle, &numIfaceHandles, &ifaceHandles);
    if (res < 0) {
        return NULL;
    }
    for (int i = 0; i < numIfaceHandles; i++) {
        if (wifi_get_iface_name(ifaceHandles[i], buf, sizeof(buf)) == WIFI_SUCCESS) {
            if (strcmp(buf, name) == 0) {
                ALOGI("found interface %s\n", buf);
                ifHandle = ifaceHandles[i];
                return ifHandle;
            }
        }
    }
    return NULL;
}

wifi_error wifi_get_supported_feature_set(wifi_interface_handle handle, feature_set *set)
{
    GetFeatureSetCommand command(handle, ANDR_WIFI_ATTRIBUTE_NUM_FEATURE_SET, set, NULL, NULL, 1);
    return (wifi_error) command.requestResponse();
}

wifi_error wifi_get_concurrency_matrix(wifi_interface_handle handle, int set_size_max,
       feature_set set[], int *set_size)
{
    GetFeatureSetCommand command(handle, ANDR_WIFI_ATTRIBUTE_FEATURE_SET, NULL,
            set, set_size, set_size_max);
    return (wifi_error) command.requestResponse();
}

wifi_error wifi_get_supported_radio_combinations_matrix(wifi_handle handle,
    u32 max_size, u32* size, wifi_radio_combination_matrix *radio_combination_matrix)
{
    int numIfaceHandles = 0;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;

    wlan0Handle = wifi_get_wlan_interface((wifi_handle)handle, ifaceHandles, numIfaceHandles);
    GetRadioComboCommand *cmd = new GetRadioComboCommand(wlan0Handle, max_size,
        size, radio_combination_matrix);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = (wifi_error)cmd->start();
    if (result == WIFI_SUCCESS) {
        ALOGD("Get radio combo matrix success");
    } else {
        ALOGE("Get radio combo matrix failed\n");
    }
    cmd->releaseRef();
    return result;
}

wifi_error wifi_set_scanning_mac_oui(wifi_interface_handle handle, oui scan_oui)
{
    SetPnoMacAddrOuiCommand command(handle, scan_oui);
    return (wifi_error)command.start();

}

wifi_error wifi_set_nodfs_flag(wifi_interface_handle handle, u32 nodfs)
{
    SetNodfsCommand command(handle, nodfs);
    return (wifi_error) command.requestResponse();
}

wifi_error wifi_set_country_code(wifi_interface_handle handle, const char *country_code)
{
    SetCountryCodeCommand command(handle, country_code);
    return (wifi_error) command.requestResponse();
}

static wifi_error wifi_start_rssi_monitoring(wifi_request_id id, wifi_interface_handle
                        iface, s8 max_rssi, s8 min_rssi, wifi_rssi_event_handler eh)
{
    ALOGI("Starting RSSI monitor %d", id);
    wifi_handle handle = getWifiHandle(iface);
    SetRSSIMonitorCommand *cmd = new SetRSSIMonitorCommand(id, iface, max_rssi, min_rssi, eh);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = wifi_register_cmd(handle, id, cmd);
    if (result != WIFI_SUCCESS) {
        ALOGI("wifi_register_cmd() is failed %d", id);
        cmd->releaseRef();
        return result;
    }
    result = (wifi_error)cmd->start();
    if (result != WIFI_SUCCESS) {
        ALOGI("start() is failed %d", id);
        wifi_unregister_cmd(handle, id);
        cmd->releaseRef();
        return result;
    }
    return result;
}

static wifi_error wifi_stop_rssi_monitoring(wifi_request_id id, wifi_interface_handle iface)
{
    ALOGI("Stopping RSSI monitor %d", id);

    if (id == -1) {
        wifi_rssi_event_handler handler;
        s8 max_rssi = 0, min_rssi = 0;
        memset(&handler, 0, sizeof(handler));
        SetRSSIMonitorCommand *cmd = new SetRSSIMonitorCommand(id, iface,
                                                    max_rssi, min_rssi, handler);
        NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
        cmd->cancel();
        cmd->releaseRef();
        return WIFI_SUCCESS;
    }
    return wifi_cancel_cmd(id, iface);
}

static wifi_error wifi_get_packet_filter_capabilities(wifi_interface_handle handle,
        u32 *version, u32 *max_len)
{
    ALOGD("Getting APF capabilities, halHandle = %p\n", handle);
    AndroidPktFilterCommand *cmd = new AndroidPktFilterCommand(handle, version, max_len);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = (wifi_error)cmd->start();
    if (result == WIFI_SUCCESS) {
        ALOGD("Getting APF capability, version = %d, max_len = %d\n", *version, *max_len);
    }
    cmd->releaseRef();
    return result;
}

static wifi_error wifi_set_packet_filter(wifi_interface_handle handle,
        const u8 *program, u32 len)
{
    char iface_name[IFNAMSIZ];

    ALOGE("Setting APF program, halHandle = %p\n", handle);
    if (wifi_get_iface_name(handle, iface_name, sizeof(iface_name)) != WIFI_SUCCESS) {
       ALOGE("%s : Invalid interface handle\n", __func__);
       return WIFI_ERROR_INVALID_ARGS;
    }
    ALOGE("Set apf filter for iface = %s\n", iface_name);

    AndroidPktFilterCommand *cmd = new AndroidPktFilterCommand(handle, program, len);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = (wifi_error)cmd->start();
    cmd->releaseRef();
    return result;
}

static wifi_error wifi_read_packet_filter(wifi_interface_handle handle,
    u32 src_offset, u8 *host_dst, u32 length)
{
    ALOGD("Read APF program, halHandle = %p, length = %d\n", handle, length);
    AndroidPktFilterCommand *cmd = new AndroidPktFilterCommand(handle, host_dst, length);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = (wifi_error)cmd->start();
    if (result == WIFI_SUCCESS) {
        ALOGI("Read APF program success\n");
    }
    cmd->releaseRef();
    return result;
}
static wifi_error wifi_configure_nd_offload(wifi_interface_handle handle, u8 enable)
{
    SetNdoffloadCommand command(handle, enable);
    return (wifi_error) command.requestResponse();
}
wifi_error wifi_set_latency_mode(wifi_interface_handle handle, wifi_latency_mode mode)
{
    ALOGD("Setting Wifi Latency mode, halHandle = %p LatencyMode = %d\n", handle, mode);
    SetLatencyModeCommand command(handle, mode);
    return (wifi_error) command.requestResponse();
}

/////////////////////////////////////////////////////////////////////////
class TxPowerScenario : public WifiCommand {
    wifi_power_scenario mScenario;
public:
    // constructor for tx power scenario setting
    TxPowerScenario(wifi_interface_handle handle, wifi_power_scenario scenario)
    : WifiCommand("TxPowerScenario", handle, 0), mScenario(scenario)
    {
        mScenario = scenario;
    }

    // constructor for tx power scenario resetting
    TxPowerScenario(wifi_interface_handle handle)
    : WifiCommand("TxPowerScenario", handle, 0)
    {
        mScenario = WIFI_POWER_SCENARIO_DEFAULT;
    }

    int createRequest(WifiRequest& request, int subcmd, wifi_power_scenario mScenario) {
        int result = request.create(GOOGLE_OUI, subcmd);
        if (result < 0) {
            return result;
        }

        if ((mScenario <= WIFI_POWER_SCENARIO_INVALID) ||
           (mScenario > WIFI_POWER_SCENARIO_ON_BODY_VIDEO_RECORDING)) {
            ALOGE("Unsupported tx power value:%d\n", mScenario);
            return WIFI_ERROR_NOT_SUPPORTED;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_s8(ANDR_WIFI_ATTRIBUTE_TX_POWER_SCENARIO, mScenario);
        if (result < 0) {
            ALOGE("Failed to put tx power scenario request; result = %d", result);
            return result;
        }
        request.attr_end(data);
        return WIFI_SUCCESS;
    }

    int start(wifi_power_scenario mScenario) {
        WifiRequest request(familyId(), ifaceId());
        int result = createRequest(request, WIFI_SUBCMD_TX_POWER_SCENARIO, mScenario);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to create request; result = %d", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to send tx power scenario; result = %d", result);
        }
        return result;
    }
protected:
    virtual int handleResponse(WifiEvent& reply) {
        ALOGD("Request complete!");
        /* Nothing to do on response! */
        return NL_SKIP;
    }
};


wifi_error wifi_select_tx_power_scenario(wifi_interface_handle handle, wifi_power_scenario scenario)
{
    ALOGE("wifi_select_tx_power_scenario");
    TxPowerScenario command(handle);
    return (wifi_error)command.start(scenario);
}

wifi_error wifi_reset_tx_power_scenario(wifi_interface_handle handle)
{
    wifi_power_scenario scenario = WIFI_POWER_SCENARIO_DEFAULT;
    ALOGE("wifi_reset_tx_power_scenario");
    TxPowerScenario command(handle);
    return (wifi_error)command.start(scenario);
}

/////////////////////////////////////////////////////////////////////////////

class ThermalMitigation : public WifiCommand {
private:
    wifi_thermal_mode mMitigation;
    u32 mCompletionWindow;
public:
    // constructor for thermal mitigation setting
    ThermalMitigation(wifi_interface_handle handle,
		    wifi_thermal_mode mitigation, u32 completion_window)
    : WifiCommand("ThermalMitigation", handle, 0)
    {
        mMitigation = mitigation;
		mCompletionWindow = completion_window;
    }

    int createRequest(WifiRequest& request, int subcmd,
		    wifi_thermal_mode mitigation, u32 completion_window) {
        int result = request.create(GOOGLE_OUI, subcmd);
        if (result < 0) {
            return result;
        }

        if ((mitigation < WIFI_MITIGATION_NONE) ||
           (mitigation > WIFI_MITIGATION_EMERGENCY)) {
            ALOGE("Unsupported tx mitigation value:%d\n", mitigation);
            return WIFI_ERROR_NOT_SUPPORTED;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_s8(ANDR_WIFI_ATTRIBUTE_THERMAL_MITIGATION, mitigation);
        if (result < 0) {
            ALOGE("Failed to put tx power scenario request; result = %d", result);
            return result;
        }
        result = request.put_u32(ANDR_WIFI_ATTRIBUTE_THERMAL_COMPLETION_WINDOW,
		       	completion_window);
        if (result < 0) {
            ALOGE("Failed to put tx power scenario request; result = %d", result);
            return result;
        }

        request.attr_end(data);
        return WIFI_SUCCESS;
    }

    int start() {
        WifiRequest request(familyId(), ifaceId());
        int result = createRequest(request, WIFI_SUBCMD_THERMAL_MITIGATION, mMitigation,
		       	mCompletionWindow);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to create request; result = %d", result);
            return result;
        }

        ALOGD("try to get resp; mitigation=%d, delay=%d", mMitigation, mCompletionWindow);
        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to send thermal mitigation; result = %d", result);
        }
        return result;
    }
protected:
    virtual int handleResponse(WifiEvent& reply) {
        ALOGD("Request complete!");
        /* Nothing to do on response! */
        return NL_SKIP;
    }
};

wifi_error wifi_set_thermal_mitigation_mode(wifi_handle handle,
                                            wifi_thermal_mode mode,
                                            u32 completion_window)
{
    int numIfaceHandles = 0;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;

    wlan0Handle = wifi_get_wlan_interface((wifi_handle)handle, ifaceHandles, numIfaceHandles);
    ThermalMitigation command(wlan0Handle, mode, completion_window);
    return (wifi_error)command.start();
}

/////////////////////////////////////////////////////////////////////////////

class ChAvoidCommand : public WifiCommand {
    private:
        u32 mNumParams;
        wifi_coex_unsafe_channel *chavoidParams;
        u32 mMandatory;

    public:
        ChAvoidCommand(wifi_interface_handle handle,
            u32 num, wifi_coex_unsafe_channel channels[], u32 mandatory)
            : WifiCommand("ChAvoidCommand", handle, 0),
                mNumParams(num), chavoidParams(channels), mMandatory(mandatory)
        {
        }

    int createRequest(WifiRequest& request) {
        return createSetChAvoidRequest(request);
    }

    int createSetChAvoidRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, CHAVOID_SUBCMD_SET_CONFIG);
        if (result < 0) {
            ALOGE("%s : Failed to create SUBCMD\n", __func__);
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(CHAVOID_ATTRIBUTE_CNT, mNumParams);
        if (result < 0) {
            ALOGE("%s : Failed to set cound\n", __func__);
            return result;
        }
        result = request.put_u32(CHAVOID_ATTRIBUTE_MANDATORY, mMandatory);
        if (result < 0) {
            ALOGE("%s : Failed to set mandatory cap\n", __func__);
            return result;
        }

        nlattr *chavoid_config = request.attr_start(CHAVOID_ATTRIBUTE_CONFIG);
        for (int i = 0; i< mNumParams; i++) {
            nlattr *item = request.attr_start(i);
            if (item == NULL) {
                ALOGE("%s : Failed to alloc item\n", __func__);
                return WIFI_ERROR_OUT_OF_MEMORY;
            }
            result = request.put_u32(CHAVOID_ATTRIBUTE_BAND, chavoidParams[i].band);
            if (result < 0) {
                ALOGE("%s : Failed to set band\n", __func__);
                return result;
            }
            result = request.put_u32(CHAVOID_ATTRIBUTE_CHANNEL, chavoidParams[i].channel);
            if (result < 0) {
                ALOGE("%s : Failed to set channel\n", __func__);
                return result;
            }
            result = request.put_u32(CHAVOID_ATTRIBUTE_PWRCAP, chavoidParams[i].power_cap_dbm);
            if (result < 0) {
                ALOGE("%s : Failed to set power cap\n", __func__);
                return result;
            }
            request.attr_end(item);
        }
        request.attr_end(chavoid_config);
        request.attr_end(data);
        return WIFI_SUCCESS;
    }

    int start() {
        WifiRequest request(familyId(), ifaceId());
        int result = createRequest(request);
        if (result < 0) {
            return result;
        }
        result = requestResponse(request);
        if (result < 0) {
            ALOGI("Request Response failed for ChAvoid, result = %d", result);
            return result;
        }
        return result;
    }

    int handleResponse(WifiEvent& reply) {
        ALOGD("In ChAvoidCommand::handleResponse");

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = reply.get_vendor_data_len();

        if (vendor_data == NULL || len == 0) {
            ALOGE("no vendor data in ChAvoidCommand response; ignoring it");
            return NL_SKIP;
        }
        ALOGD("Response received for ChAvoid command\n");
        return NL_OK;
    }

    int handleEvent(WifiEvent& event) {
        /* No Event to receive for ChAvoid commands */
        ALOGD("ChAvoid command %s\n", __FUNCTION__);
        return NL_SKIP;
    }
};

wifi_error wifi_set_coex_unsafe_channels(wifi_handle handle,
        u32 num, wifi_coex_unsafe_channel channels[], u32 mandatory)
{
    int numIfaceHandles = 0;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;

    wlan0Handle = wifi_get_wlan_interface((wifi_handle)handle, ifaceHandles, numIfaceHandles);

    ChAvoidCommand *cmd = new ChAvoidCommand(wlan0Handle, num, channels, mandatory);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = (wifi_error)cmd->start();
    if (result == WIFI_SUCCESS) {
        ALOGI("Setting Channel Avoidance success\n");
    } else {
        ALOGE("Setting Channel Avoidance failed\n");
    }
    cmd->releaseRef();
    return result;
}

/////////////////////////////////////////////////////////////////////////////

class DscpCommand : public WifiCommand {
    private:
	u32 mStart;
	u32 mEnd;
	u32 mAc;
        int mReqType;
    public:
        DscpCommand(wifi_interface_handle handle,
                u32 start, u32 end, u32 ac)
            : WifiCommand("DscpCommand", handle, 0),
                    mStart(start), mEnd(end), mAc(ac),
                    mReqType(SET_DSCP_TABLE)
        {
        }

        DscpCommand(wifi_interface_handle handle)
            : WifiCommand("DscpCommand", handle, 0),
                    mReqType(RESET_DSCP_TABLE)
        {
            mStart = 0;
            mEnd = 0;
            mAc = 0;
        }

    int createRequest(WifiRequest& request) {
        if (mReqType == SET_DSCP_TABLE) {
            ALOGI("\n%s: DSCP set table request\n", __FUNCTION__);
            return createSetDscpRequest(request);
        } else if (mReqType == RESET_DSCP_TABLE) {
            ALOGI("\n%s: DSCP reset table request\n", __FUNCTION__);
            return createResetDscpRequest(request);
        } else {
            ALOGE("\n%s Unknown DSCP request\n", __FUNCTION__);
            return WIFI_ERROR_NOT_SUPPORTED;
        }
        return WIFI_SUCCESS;
    }

    int createSetDscpRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, DSCP_SUBCMD_SET_TABLE);
        if (result < 0) {
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(DSCP_ATTRIBUTE_START, mStart);
        if (result < 0) {
            goto exit;
        }
        result = request.put_u32(DSCP_ATTRIBUTE_END, mEnd);
        if (result < 0) {
            goto exit;
        }
        result = request.put_u32(DSCP_ATTRIBUTE_AC, mAc);
        if (result < 0) {
            goto exit;
        }
        request.attr_end(data);
exit:
        return result;
    }

    int createResetDscpRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, DSCP_SUBCMD_RESET_TABLE);
        if (result < 0) {
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        request.attr_end(data);
        return result;
    }

    int start() {
        WifiRequest request(familyId(), ifaceId());
        int result = createRequest(request);
        if (result < 0) {
            return result;
        }
        result = requestResponse(request);
        if (result < 0) {
            ALOGI("Request Response failed for DSCP, result = %d", result);
            return result;
        }
        return result;
    }

    int handleResponse(WifiEvent& reply) {
        ALOGD("In DscpCommand::handleResponse");

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = reply.get_vendor_data_len();

        if (vendor_data == NULL || len == 0) {
            ALOGE("no vendor data in DscpCommand response; ignoring it");
            return NL_SKIP;
        }
        if( mReqType == SET_DSCP_TABLE) {
            ALOGD("Response received for Set DSCP command\n");
        } else if (mReqType == RESET_DSCP_TABLE) {
            ALOGD("Response received for Reset DSCP command\n");
        }
        return NL_OK;
    }

    int handleEvent(WifiEvent& event) {
        /* No Event to receive for DSCP commands */
        ALOGD("DSCP command %s\n", __FUNCTION__);
        return NL_SKIP;
    }
};

wifi_error wifi_map_dscp_access_category(wifi_handle handle,
        u32 start, u32 end, u32 ac)
{
    int numIfaceHandles = 0;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;

    wlan0Handle = wifi_get_wlan_interface((wifi_handle)handle, ifaceHandles, numIfaceHandles);

    DscpCommand *cmd = new DscpCommand(wlan0Handle, start, end, ac);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = (wifi_error)cmd->start();
    if (result == WIFI_SUCCESS) {
        ALOGI("Mapping DSCP table success\n");
    } else {
        ALOGE("Mapping DSCP table fail\n");
    }
    cmd->releaseRef();
    return result;
}

wifi_error wifi_reset_dscp_mapping(wifi_handle handle)
{
    int numIfaceHandles = 0;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;

    wlan0Handle = wifi_get_wlan_interface((wifi_handle)handle, ifaceHandles, numIfaceHandles);

    DscpCommand *cmd = new DscpCommand(wlan0Handle);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = (wifi_error)cmd->start();
    if (result == WIFI_SUCCESS) {
        ALOGI("Resetting DSCP table success\n");
    } else {
        ALOGE("Resetting DSCP table fail\n");
    }
    cmd->releaseRef();
    return result;
}

class VirtualIfaceConfig : public WifiCommand {
    const char *mIfname;
    nl80211_iftype mType;
    u32 mwlan0_id;

public:
    VirtualIfaceConfig(wifi_interface_handle handle, const char* ifname,
        nl80211_iftype iface_type, u32 wlan0_id)
    : WifiCommand("VirtualIfaceConfig", handle, 0), mIfname(ifname), mType(iface_type),
        mwlan0_id(wlan0_id)
    {
        mIfname = ifname;
        mType = iface_type;
        mwlan0_id = wlan0_id;
    }
    int createRequest(WifiRequest& request, const char* ifname,
        nl80211_iftype iface_type, u32 wlan0_id) {
        ALOGD("add ifname = %s, iface_type = %d, wlan0_id = %d",
        ifname, iface_type, wlan0_id);

        int result = request.create(NL80211_CMD_NEW_INTERFACE);
        if (result < 0) {
            ALOGE("failed to create NL80211_CMD_NEW_INTERFACE; result = %d", result);
            return result;
        }

        result = request.put_u32(NL80211_ATTR_IFINDEX, wlan0_id);
        if (result < 0) {
            ALOGE("failed to put NL80211_ATTR_IFINDEX; result = %d", result);
            return result;
        }

        result = request.put_string(NL80211_ATTR_IFNAME, ifname);
        if (result < 0) {
            ALOGE("failed to put NL80211_ATTR_IFNAME = %s; result = %d", ifname, result);
            return result;
        }

        result = request.put_u32(NL80211_ATTR_IFTYPE, iface_type);
        if (result < 0) {
            ALOGE("failed to put NL80211_ATTR_IFTYPE = %d; result = %d", iface_type, result);
            return result;
        }
        return WIFI_SUCCESS;
    }

    int deleteRequest(WifiRequest& request, const char* ifname) {
        ALOGD("delete ifname = %s\n", ifname);
        int result = request.create(NL80211_CMD_DEL_INTERFACE);
        if (result < 0) {
            ALOGE("failed to create NL80211_CMD_DEL_INTERFACE; result = %d", result);
            return result;
        }
        result = request.put_u32(NL80211_ATTR_IFINDEX, if_nametoindex(ifname));
        if (result < 0) {
            ALOGE("failed to put NL80211_ATTR_IFINDEX = %d; result = %d",
                if_nametoindex(ifname), result);
            return result;
        }
        result = request.put_string(NL80211_ATTR_IFNAME, ifname);
        if (result < 0) {
            ALOGE("failed to put NL80211_ATTR_IFNAME = %s; result = %d", ifname, result);
            return result;
        }
        return WIFI_SUCCESS;
    }

    int createIface() {
        ALOGD("Creating virtual interface");
        WifiRequest request(familyId(), ifaceId());
        int result = createRequest(request, mIfname, mType, mwlan0_id);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to create virtual iface request; result = %d\n", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to get the virtual iface create response; result = %d\n", result);
            return result;
        }
        ALOGD("Created virtual interface");
        return WIFI_SUCCESS;
    }

    int deleteIface() {
        ALOGD("Deleting virtual interface");
        WifiRequest request(familyId(), ifaceId());
        int result = deleteRequest(request, mIfname);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to create virtual iface delete request; result = %d\n", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to get response of delete virtual interface; result = %d\n", result);
            return result;
        }
        ALOGD("Deleted virtual interface");
        return WIFI_SUCCESS;
    }
protected:
    virtual int handleResponse(WifiEvent& reply) {
         ALOGD("Request complete!");
        /* Nothing to do on response! */
        return NL_SKIP;
    }
};

static std::vector<std::string> added_ifaces;

static void wifi_cleanup_dynamic_ifaces(wifi_handle handle)
{
    int len = added_ifaces.size();
    ALOGI("%s: virtual iface size %d\n", __FUNCTION__, len);
    while (len--) {
        wifi_virtual_interface_delete(handle, added_ifaces.front().c_str());
        ALOGI("%s: deleted virtual iface %s\n",
            __FUNCTION__, added_ifaces.front().c_str());
    }
    added_ifaces.clear();
}

wifi_error wifi_virtual_interface_create(wifi_handle handle, const char* ifname,
        wifi_interface_type iface_type)
{
    int numIfaceHandles = 0;
    wifi_error ret = WIFI_SUCCESS;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;
    nl80211_iftype type = NL80211_IFTYPE_UNSPECIFIED;
    u32 wlan0_id = if_nametoindex("wlan0");

    if (!handle || !wlan0_id) {
        ALOGE("%s: Error wifi_handle NULL or wlan0 not present\n", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    /* Do not create interface if already exist. */
    if (if_nametoindex(ifname)) {
        ALOGD("%s: if_nametoindex(%s) = %d already exists, skip create \n",
            __FUNCTION__, ifname, if_nametoindex(ifname));
        return WIFI_SUCCESS;
    }

    ALOGD("%s: ifname name = %s, type = %s\n", __FUNCTION__, ifname,
        IfaceTypeToString(iface_type));

    switch (iface_type) {
        case WIFI_INTERFACE_TYPE_STA:
            type = NL80211_IFTYPE_STATION;
            break;
        case WIFI_INTERFACE_TYPE_AP:
            type = NL80211_IFTYPE_AP;
            break;
        case WIFI_INTERFACE_TYPE_P2P:
            type = NL80211_IFTYPE_P2P_DEVICE;
            break;
        case WIFI_INTERFACE_TYPE_NAN:
            ALOGE("%s: Unsupported interface type %u\n", __FUNCTION__, iface_type);
            return WIFI_ERROR_NOT_SUPPORTED;
        default:
            ALOGE("%s: Wrong interface type %u\n", __FUNCTION__, iface_type);
            return WIFI_ERROR_UNKNOWN;
    }

    wlan0Handle = wifi_get_wlan_interface((wifi_handle)handle, ifaceHandles, numIfaceHandles);
    VirtualIfaceConfig command(wlan0Handle, ifname, type, wlan0_id);

    ret = (wifi_error)command.createIface();
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: Iface add Error:%d", __FUNCTION__,ret);
        return ret;
    }
    /* Update dynamic interface list */
    added_ifaces.push_back(std::string(ifname));
    ret = wifi_add_iface_hal_info((wifi_handle)handle, ifname);
    return ret;
}

wifi_error wifi_virtual_interface_delete(wifi_handle handle, const char* ifname)
{
    int numIfaceHandles = 0;
    int i = 0;
    wifi_error ret = WIFI_SUCCESS;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;
    hal_info *info = (hal_info *)handle;
    u32 wlan0_id = if_nametoindex("wlan0");

    if (!handle || !wlan0_id) {
        ALOGE("%s: Error wifi_handle NULL or wlan0 not present\n", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    /* Check if interface delete requested on valid interface */
    while (i < info->max_num_interfaces) {
        if (info->interfaces[i] != NULL &&
            strncmp(info->interfaces[i]->name,
            ifname, sizeof(info->interfaces[i]->name)) == 0) {
            if (!get_halutil_mode()) {
                if (info->interfaces[i]->is_virtual == false) {
                    ALOGI("%s: %s is static iface, skip delete\n",
                        __FUNCTION__, ifname);
                        return WIFI_SUCCESS;
                }
            } else {
                ALOGI("%s: %s delete iface", __FUNCTION__, ifname);
                break;
            }
        }
        i++;
    }

    ALOGI("%s: iface name=%s\n", __FUNCTION__, ifname);
    wlan0Handle = wifi_get_wlan_interface((wifi_handle)handle, ifaceHandles, numIfaceHandles);
    VirtualIfaceConfig command(wlan0Handle, ifname, (nl80211_iftype)0, 0);
    ret = (wifi_error)command.deleteIface();
    if (ret != WIFI_SUCCESS) {
        ALOGE("%s: Iface delete Error:%d", __FUNCTION__,ret);
        /* If lower layer returns an error, hang event is expected to do wifi reset
         * and hal state needs to be cleared irrespectivily. Hence fall-through.
         */
    }
    /* Update dynamic interface list */
    added_ifaces.erase(std::remove(added_ifaces.begin(), added_ifaces.end(), std::string(ifname)),
        added_ifaces.end());
    ret = wifi_clear_iface_hal_info((wifi_handle)handle, ifname);
    return ret;
}
/////////////////////////////////////////////////////////////////////////////

class MultiStaConfig : public WifiCommand {
    wifi_multi_sta_use_case mUseCase;
    int mReqType;

public:
    MultiStaConfig(wifi_interface_handle handle)
    : WifiCommand("MultiStaConfig", handle, 0), mReqType(SET_PRIMARY_CONNECTION)
    {
        mUseCase = WIFI_DUAL_STA_TRANSIENT_PREFER_PRIMARY;
    }
    MultiStaConfig(wifi_interface_handle handle, wifi_multi_sta_use_case use_case)
    : WifiCommand("MultiStaConfig", handle, 0), mUseCase(use_case), mReqType(SET_USE_CASE)
    {
        mUseCase = use_case;
    }

    int createRequest(WifiRequest& request) {
        if (mReqType == SET_PRIMARY_CONNECTION) {
            ALOGI("\n%s: MultiSta set primary connection\n", __FUNCTION__);
            return createSetPrimaryConnectionRequest(request);
        } else if (mReqType == SET_USE_CASE) {
            ALOGI("\n%s: MultiSta set use case\n", __FUNCTION__);
            return createSetUsecaseRequest(request);
        } else {
            ALOGE("\n%s Unknown MultiSta request\n", __FUNCTION__);
            return WIFI_ERROR_NOT_SUPPORTED;
        }
        return WIFI_SUCCESS;
    }

    int createSetPrimaryConnectionRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, WIFI_SUBCMD_SET_MULTISTA_PRIMARY_CONNECTION);
        if (result < 0) {
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        request.attr_end(data);

        return result;
    }

    int createSetUsecaseRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, WIFI_SUBCMD_SET_MULTISTA_USE_CASE);
        if (result < 0) {
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(MULTISTA_ATTRIBUTE_USE_CASE, mUseCase);
        if (result < 0) {
            ALOGE("failed to put MULTISTA_ATTRIBUTE_USE_CASE = %d; result = %d", mUseCase, result);
            goto exit;
        }

exit:   request.attr_end(data);
        return result;
    }

    int start() {
        WifiRequest request(familyId(), ifaceId());
        int result = createRequest(request);
        if (result < 0) {
            return result;
        }
        result = requestResponse(request);
        if (result < 0) {
            ALOGI("Request Response failed for MultiSta, result = %d", result);
            return result;
        }
        ALOGI("Done!");
        return result;
    }
protected:
    virtual int handleResponse(WifiEvent& reply) {
        ALOGD("Request complete!");
        /* Nothing to do on response! */
        return NL_SKIP;
    }
};

wifi_error wifi_multi_sta_set_primary_connection(wifi_handle handle, wifi_interface_handle iface)
{
    wifi_error ret = WIFI_SUCCESS;
    char buf[IFNAMSIZ];

    if (!handle || !iface) {
        ALOGE("%s: Error wifi_handle NULL or invalid wifi interface handle\n", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    if (wifi_get_iface_name(iface, buf, sizeof(buf)) != WIFI_SUCCESS) {
        ALOGE("%s : Invalid interface handle\n", __func__);
        return WIFI_ERROR_INVALID_ARGS;
    }

    ALOGD("Setting Multista primary connection for iface = %s\n", buf);

    MultiStaConfig *cmd = new MultiStaConfig(iface);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    ret = (wifi_error)cmd->start();
    cmd->releaseRef();
    return ret;
}

wifi_error wifi_multi_sta_set_use_case(wifi_handle handle, wifi_multi_sta_use_case use_case)
{
    int numIfaceHandles = 0;
    wifi_error ret = WIFI_SUCCESS;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;

    if (!handle) {
        ALOGE("%s: Error wifi_handle NULL\n", __FUNCTION__);
        return WIFI_ERROR_UNKNOWN;
    }

    if (!(use_case >= WIFI_DUAL_STA_TRANSIENT_PREFER_PRIMARY &&
	    use_case <= WIFI_DUAL_STA_NON_TRANSIENT_UNBIASED)) {
        ALOGE("%s: Invalid  multi_sta usecase %d\n", __FUNCTION__, use_case);
        return WIFI_ERROR_INVALID_ARGS;
    }

    wlan0Handle = wifi_get_wlan_interface((wifi_handle)handle, ifaceHandles, numIfaceHandles);
    ALOGD("Setting Multista usecase = %d\n", use_case);
    MultiStaConfig *cmd = new MultiStaConfig(wlan0Handle, use_case);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    ret = (wifi_error)cmd->start();
    cmd->releaseRef();
    return ret;
}
/////////////////////////////////////////////////////////////////////////////////////////////////

class SetVoipModeCommand : public WifiCommand {

private:
    wifi_voip_mode mMode;
public:
    SetVoipModeCommand(wifi_interface_handle handle, wifi_voip_mode mode)
        : WifiCommand("SetVoipModeCommand", handle, 0) {
        mMode = mode;
    }
    virtual int create() {
        int ret;

        ret = mMsg.create(GOOGLE_OUI, WIFI_SUBCMD_CONFIG_VOIP_MODE);
        if (ret < 0) {
            ALOGE("Can't create message to send to driver - %d", ret);
            return ret;
        }

        nlattr *data = mMsg.attr_start(NL80211_ATTR_VENDOR_DATA);
        ret = mMsg.put_u32(ANDR_WIFI_ATTRIBUTE_VOIP_MODE, mMode);
        ALOGE("mMode - %d", mMode);
        if (ret < 0) {
             ALOGE("Failed to set voip mode %d\n", mMode);
             return ret;
        }

        ALOGI("Successfully configured voip mode %d\n", mMode);
        mMsg.attr_end(data);
        return WIFI_SUCCESS;
    }
};

wifi_error wifi_set_voip_mode(wifi_interface_handle handle, wifi_voip_mode mode)
{
    ALOGD("Setting VOIP mode, halHandle = %p Mode = %d\n", handle, mode);
    SetVoipModeCommand command(handle, mode);
    return (wifi_error) command.requestResponse();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
class SetDtimConfigCommand : public WifiCommand {

private:
    uint32_t multiplier;
public:
    SetDtimConfigCommand(wifi_interface_handle handle, u32 dtim_multiplier)
        : WifiCommand("SetDtimConfigCommand", handle, 0) {
        multiplier = dtim_multiplier;
    }
    virtual int create() {
        int ret;

        ret = mMsg.create(GOOGLE_OUI, WIFI_SUBCMD_SET_DTIM_CONFIG);
        if (ret < 0) {
            ALOGE("Can't create message to send to driver - %d", ret);
            return ret;
        }

        nlattr *data = mMsg.attr_start(NL80211_ATTR_VENDOR_DATA);
        ret = mMsg.put_u32(ANDR_WIFI_ATTRIBUTE_DTIM_MULTIPLIER, multiplier);
        if (ret < 0) {
             ALOGE("Failed to set dtim mutiplier %d\n", multiplier);
             return ret;
        }

        ALOGI("Successfully configured dtim multiplier %d\n", multiplier);
        mMsg.attr_end(data);
        return WIFI_SUCCESS;
    }
};

wifi_error wifi_set_dtim_config(wifi_interface_handle handle, u32 multiplier)
{
    ALOGD("Setting DTIM config , halHandle = %p Multiplier = %d\n", handle, multiplier);
    SetDtimConfigCommand command(handle, multiplier);
    return (wifi_error) command.requestResponse();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
class UsableChannelCommand : public WifiCommand {

private:
    u32 mBandMask;
    u32 mIfaceModeMask;
    u32 mFilterMask;
    u32 mMaxSize;
    u32* mSize;
    wifi_usable_channel* mChannels;

public:
    UsableChannelCommand(wifi_interface_handle handle, u32 band_mask, u32 iface_mode_mask,
                   u32 filter_mask, u32 max_size, u32* size, wifi_usable_channel* channels)
        : WifiCommand("UsableChannelCommand", handle, 0),
                       mBandMask(band_mask), mIfaceModeMask(iface_mode_mask),
                       mFilterMask(filter_mask), mMaxSize(max_size),
		       mSize(size), mChannels(channels)
    {
    }

    int createRequest(WifiRequest& request) {
        createUsableChannelRequest(request);
        return WIFI_SUCCESS;
    }

    int createUsableChannelRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, WIFI_SUBCMD_USABLE_CHANNEL);
        if (result < 0) {
            ALOGE("Failed to create UsableChannel request; result = %d", result);
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);

        result = request.put_u32(USABLECHAN_ATTRIBUTE_BAND, mBandMask);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to put log level; result = %d", result);
            return result;
        }
        result = request.put_u32(USABLECHAN_ATTRIBUTE_IFACE, mIfaceModeMask);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to put ring flags; result = %d", result);
            return result;
        }
        result = request.put_u32(USABLECHAN_ATTRIBUTE_FILTER, mFilterMask);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to put usablechan filter; result = %d", result);
            return result;
        }
        result = request.put_u32(USABLECHAN_ATTRIBUTE_MAX_SIZE, mMaxSize);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to put usablechan max_size; result = %d", result);
            return result;
        }
        request.attr_end(data);

        return WIFI_SUCCESS;
    }

    int start() {
        WifiRequest request(familyId(), ifaceId());
        int result = createRequest(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to create request; result = %d", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to set scanning mac OUI; result = %d", result);
        }

        return result;
    }

    virtual int handleResponse(WifiEvent& reply) {
        ALOGD("In DebugCommand::handleResponse");

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = reply.get_vendor_data_len();
        wifi_usable_channel *channels(mChannels);

        if (vendor_data == NULL || len == 0) {
            ALOGE("No Debug data found");
            return NL_SKIP;
        }

        nl_iterator it(vendor_data);
        if (it.get_type() == USABLECHAN_ATTRIBUTE_SIZE) {
            *mSize = it.get_u32();
        } else {
            ALOGE("Unknown attribute: %d expecting %d",
                it.get_type(), USABLECHAN_ATTRIBUTE_SIZE);
            return NL_SKIP;
        }

        it.next();
        if (it.get_type() == USABLECHAN_ATTRIBUTE_CHANNELS) {
            memcpy(channels, it.get_data(), sizeof(wifi_usable_channel) * *mSize);
        } else {
            ALOGE("Unknown attribute: %d expecting %d",
                it.get_type(), USABLECHAN_ATTRIBUTE_SIZE);
            return NL_SKIP;
        }

        return NL_OK;
    }

    virtual int handleEvent(WifiEvent& event) {
        /* NO events! */
        return NL_SKIP;
    }
};

wifi_error wifi_get_usable_channels(wifi_handle handle, u32 band_mask, u32 iface_mode_mask,
		u32 filter_mask, u32 max_size, u32* size, wifi_usable_channel* channels)
{
    int numIfaceHandles = 0;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;

    wlan0Handle = wifi_get_wlan_interface((wifi_handle)handle, ifaceHandles, numIfaceHandles);
    UsableChannelCommand command(wlan0Handle, band_mask, iface_mode_mask,
                                    filter_mask, max_size, size, channels);
    return (wifi_error)command.start();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
class EnableTxPowerLimit : public WifiCommand {
private:
    bool mEnableTxLimits;
public:
    EnableTxPowerLimit(wifi_interface_handle handle, bool enable_tx_pwr_limits)
        : WifiCommand("EnableTxPowerLimit", handle, 0)
    {
        mEnableTxLimits = enable_tx_pwr_limits;
    }

    virtual int create() {
        int ret;

        ret = mMsg.create(GOOGLE_OUI, WIFI_SUBCMD_ENABLE_TX_POWER_LIMIT);
        if (ret < 0) {
            ALOGE("Can't create message to send to driver - %d", ret);
            return ret;
        }

        nlattr *data = mMsg.attr_start(NL80211_ATTR_VENDOR_DATA);
        ret = mMsg.put_u8(TX_POWER_CAP_ENABLE_ATTRIBUTE, mEnableTxLimits);
        if (ret < 0) {
             ALOGE("Failed to put enable tx power limit param %d\n", mEnableTxLimits);
             return ret;
        }
        mMsg.attr_end(data);
        return WIFI_SUCCESS;
    }
};

wifi_error wifi_enable_tx_power_limits(wifi_interface_handle handle, bool isEnable)
{
    ALOGD("Configuring the tx power limits , halHandle = %p\n", handle);

    EnableTxPowerLimit command(handle, isEnable);
    return (wifi_error) command.requestResponse();
}

//////////////////////////////////////////////////////
class EnableStaChannel : public WifiCommand {

private:
    u32 mChannelCatEnabFlag;
public:
    EnableStaChannel(wifi_interface_handle handle, u32 channelCategoryEnableFlag)
        : WifiCommand("EnableStaChannel", handle, 0) {
        mChannelCatEnabFlag = channelCategoryEnableFlag;
    }
    virtual int create() {
        int ret;

        ret = mMsg.create(GOOGLE_OUI, WIFI_SUBCMD_CHANNEL_POLICY);
        if (ret < 0) {
            ALOGE("Can't create message to send to driver - %d", ret);
            return ret;
        }

        nlattr *data = mMsg.attr_start(NL80211_ATTR_VENDOR_DATA);
        ret = mMsg.put_u32(ANDR_WIFI_ATTRIBUTE_CHAN_POLICY, mChannelCatEnabFlag);
        if (ret < 0) {
             return ret;
        }

        mMsg.attr_end(data);
        return WIFI_SUCCESS;
    }
};

/* enable or disable the feature of allowing current STA-connected
 * channel for WFA GO, SAP and Wi-Fi Aware when the regulatory allows.
 */
wifi_error wifi_enable_sta_channel_for_peer_network(wifi_handle handle,
        u32 channelCategoryEnableFlag) {
    int numIfaceHandles = 0;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;

    wlan0Handle = wifi_get_wlan_interface((wifi_handle)handle, ifaceHandles, numIfaceHandles);
    EnableStaChannel command(wlan0Handle, channelCategoryEnableFlag);
    return (wifi_error) command.requestResponse();
}
