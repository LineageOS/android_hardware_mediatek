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

#include <hardware_legacy/wifi_hal.h>

#ifndef __WIFI_HAL_COMMON_H__
#define __WIFI_HAL_COMMON_H__

#define LOG_TAG  "WifiHAL"

#include <log/log.h>
#include "nl80211_copy.h"
#include "sync.h"
#include <unistd.h>

#define ARRAYSIZE(a)            (u8)(sizeof(a) / sizeof(a[0]))
#define SOCKET_BUFFER_SIZE      (32768U)
#define RECV_BUF_SIZE           (4096)
#define DEFAULT_EVENT_CB_SIZE   (64)
#define DEFAULT_CMD_SIZE        (64)
#define DOT11_OUI_LEN             3
#define DOT11_MAX_SSID_LEN        32
#define MAX_CACHED_SCAN_RESULT    32

#define ETHERTYPE_IP            0x0800          /* IP */
#define ETHERTYPE_IPV6          0x86dd          /* IP protocol version 6 */
#define MAX_PROBE_RESP_IE_LEN      2048
/*
 Vendor OUI - This is a unique identifier that identifies organization. Lets
 code Android specific functions with Google OUI; although vendors can do more
 with their own OUI's as well.
 */

const uint32_t GOOGLE_OUI = 0x001A11;
const uint32_t BRCM_OUI =  0x001018;
/* TODO: define vendor OUI here */

#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"

#define NMR2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5], (a)[6], (a)[7]
#define NMRSTR "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x"
#define NAN_MAX_SVC_INFO_LEN            255u
#define NAN_MASTER_RANK_LEN             8u
#define NAN_SCID_INFO_LEN               16u

#define NAN_MAX_NDP_PEER                8u
#define NAN_MAX_NDP_COUNT_SIZE          NAN_MAX_NDP_PEER * sizeof(NanDataPathId)
#define SAR_CONFIG_SCENARIO_COUNT       100
#define MAX_NUM_RADIOS                  3u
#define MAX_CMD_RESP_BUF_LEN            8192u
#define MAX_MLO_LINK                    4u
/* For STA, peer would be only one - AP.*/
#define NUM_PEER_AP                     1u
/* 11n/HT:   OFDM(12) + HT(16) rates = 28 (MCS0 ~ MCS15)
 * 11ac/VHT: OFDM(12) + VHT(12) x 2 nss = 36 (MCS0 ~ MCS11)
 * 11ax/HE:  OFDM(12) + HE(12) x 2 nss = 36 (MCS0 ~ MCS11)
 * 11be/EHT:  OFDM(12) + EHT(16) x 2 nss = 44 (MCS0 ~ MCS15)
 */
#define MAX_NUM_RATE                        44u
#define NUM_RATE_NON_BE                 36u
#define NUM_RATE_VHT                    28u
#define NUM_RATE_HT                     12u

#define NL_MSG_MAX_LEN                  5120u

/* nl_msg->nm_nlh->nlmsg_len is added by data len of the attributes
 * NL80211_ATTR_VENDOR_ID, NL80211_ATTR_VENDOR_SUBCMD,
 * NL80211_ATTR_IFINDEX, APF_PROGRAM_LEN by 56u
 * To keep the additioanl room and aligned,
 * keeping the overhead of 128u
 */
#define NL_MSG_HDR_OVERHEAD_LEN		128u
#define NL_MSG_DEFAULT_LEN		(getpagesize() - NL_MSG_HDR_OVERHEAD_LEN)

#define NAN_MAX_PAIRING_CNT             8u
#define NAN_MAX_COOKIE_LEN              255u
#define NAN_DEFAULT_RX_CHAINS_SUPPORTED 2u
#define NAN_SEC_CIPHER_SUITE_CAP_ENAB_GTK_IGTK_BIGTK    (1u << 2u)
#define NAN_SEC_ALGO_NCS_GK_CCM_128	5u     /* CCMP 128 */
#define NAN_SEC_ALGO_NCS_GK_GCM_256	6u     /* GCMP 256 */

/*
 This enum defines ranges for various commands; commands themselves
 can be defined in respective feature headers; i.e. find gscan command
 definitions in gscan.cpp
 */

typedef int (*nl_recvmsg_msg_cb_t)(struct nl_msg *msg, void *arg);
typedef enum {
    /* don't use 0 as a valid subcommand */
    VENDOR_NL80211_SUBCMD_UNSPECIFIED,

    /* define all vendor startup commands between 0x0 and 0x0FFF */
    VENDOR_NL80211_SUBCMD_RANGE_START = 0x0001,
    VENDOR_NL80211_SUBCMD_RANGE_END   = 0x0FFF,

    /* define all GScan related commands between 0x1000 and 0x10FF */
    ANDROID_NL80211_SUBCMD_GSCAN_RANGE_START = 0x1000,
    ANDROID_NL80211_SUBCMD_GSCAN_RANGE_END   = 0x10FF,

    /* define all NearbyDiscovery related commands between 0x1100 and 0x11FF */
    ANDROID_NL80211_SUBCMD_NBD_RANGE_START = 0x1100,
    ANDROID_NL80211_SUBCMD_NBD_RANGE_END   = 0x11FF,

    /* define all RTT related commands between 0x1100 and 0x11FF */
    ANDROID_NL80211_SUBCMD_RTT_RANGE_START = 0x1100,
    ANDROID_NL80211_SUBCMD_RTT_RANGE_END   = 0x11FF,

    ANDROID_NL80211_SUBCMD_LSTATS_RANGE_START = 0x1200,
    ANDROID_NL80211_SUBCMD_LSTATS_RANGE_END   = 0x12FF,

    /* define all Logger related commands between 0x1400 and 0x14FF */
    ANDROID_NL80211_SUBCMD_DEBUG_RANGE_START = 0x1400,
    ANDROID_NL80211_SUBCMD_DEBUG_RANGE_END   = 0x14FF,

    /* define all wifi offload related commands between 0x1600 and 0x16FF */
    ANDROID_NL80211_SUBCMD_WIFI_OFFLOAD_RANGE_START = 0x1600,
    ANDROID_NL80211_SUBCMD_WIFI_OFFLOAD_RANGE_END   = 0x16FF,

    /* define all NAN related commands between 0x1700 and 0x17FF */
    ANDROID_NL80211_SUBCMD_NAN_RANGE_START = 0x1700,
    ANDROID_NL80211_SUBCMD_NAN_RANGE_END   = 0x17FF,

    /* define all Android Packet Filter related commands between 0x1800 and 0x18FF */
    ANDROID_NL80211_SUBCMD_PKT_FILTER_RANGE_START = 0x1800,
    ANDROID_NL80211_SUBCMD_PKT_FILTER_RANGE_END   = 0x18FF,

    /* define all tx power related commands between 0x1900 and 0x1910 */
    ANDROID_NL80211_SUBCMD_TX_POWER_RANGE_START = 0x1900,
    ANDROID_NL80211_SUBCMD_TX_POWER_RANGE_END	 = 0x1910,

    /* define all thermal mode related commands between 0x1920 and 0x192F */
    ANDROID_NL80211_SUBCMD_MITIGATION_RANGE_START = 0x1920,
    ANDROID_NL80211_SUBCMD_MITIGATION_RANGE_END   = 0x192F,

    /* define all DSCP related commands between 0x2000 and 0x20FF */
    ANDROID_NL80211_SUBCMD_DSCP_RANGE_START =	0x2000,
    ANDROID_NL80211_SUBCMD_DSCP_RANGE_END   =	0x20FF,

    /* define all Channel Avoidance related commands between 0x2100 and 0x211F */
    ANDROID_NL80211_SUBCMD_CHAVOID_RANGE_START =    0x2100,
    ANDROID_NL80211_SUBCMD_CHAVOID_RANGE_END   =    0x211F,

    /* define all OTA Download related commands between 0x2120 and 0x212F */
    ANDROID_NL80211_SUBCMD_OTA_DOWNLOAD_START	= 0x2120,
    ANDROID_NL80211_SUBCMD_OTA_DOWNLOAD_END	= 0x212F,

    /* define all VOIP mode config related commands between 0x2130 and 0x213F */
    ANDROID_NL80211_SUBCMD_VIOP_MODE_START =	    0x2130,
    ANDROID_NL80211_SUBCMD_VIOP_MODE_END =	    0x213F,

    /* define all TWT related commands between 0x2140 and 0x214F */
    ANDROID_NL80211_SUBCMD_TWT_START =              0x2140,
    ANDROID_NL80211_SUBCMD_TWT_END =                0x214F,

    /* define all Usable Channel related commands between 0x2150 and 0x215F */
    ANDROID_NL80211_SUBCMD_USABLE_CHANNEL_START =   0x2150,
    ANDROID_NL80211_SUBCMD_USABLE_CHANNEL_END =     0x215F,

    /* define all init/deinit related commands between 0x2160 and 0x216F */
    ANDROID_NL80211_SUBCMD_INIT_DEINIT_RANGE_START = 0x2160,
    ANDROID_NL80211_SUBCMD_INIT_DEINIT_RANGE_END   = 0x216F,

   /* define scan related commands between 0x2170 and 0x2175 */
    ANDROID_NL80211_SUBCMD_SCAN_START = 0x2170,
    ANDROID_NL80211_SUBCMD_SCAN_END = 0x2175,

   /* This is reserved for future usage */

} ANDROID_VENDOR_SUB_COMMAND;

typedef enum {

    GSCAN_SUBCMD_GET_CAPABILITIES = ANDROID_NL80211_SUBCMD_GSCAN_RANGE_START,

    GSCAN_SUBCMD_SET_CONFIG,                            /* 0x1001 */

    GSCAN_SUBCMD_SET_SCAN_CONFIG,                       /* 0x1002 */
    GSCAN_SUBCMD_ENABLE_GSCAN,                          /* 0x1003 */
    GSCAN_SUBCMD_GET_SCAN_RESULTS,                      /* 0x1004 */
    GSCAN_SUBCMD_SCAN_RESULTS,                          /* 0x1005 */

    GSCAN_SUBCMD_SET_HOTLIST,                           /* 0x1006 */

    GSCAN_SUBCMD_SET_SIGNIFICANT_CHANGE_CONFIG,         /* 0x1007 */
    GSCAN_SUBCMD_ENABLE_FULL_SCAN_RESULTS,              /* 0x1008 */
    GSCAN_SUBCMD_GET_CHANNEL_LIST,                       /* 0x1009 */

    WIFI_SUBCMD_GET_FEATURE_SET,                         /* 0x100A */
    WIFI_SUBCMD_GET_FEATURE_SET_MATRIX,                  /* 0x100B */
    WIFI_SUBCMD_SET_PNO_RANDOM_MAC_OUI,                  /* 0x100C */
    WIFI_SUBCMD_NODFS_SET,                               /* 0x100D */
    WIFI_SUBCMD_SET_COUNTRY_CODE,                        /* 0x100E */
    /* Add more sub commands here */
    GSCAN_SUBCMD_SET_EPNO_SSID,                          /* 0x100F */

    WIFI_SUBCMD_SET_SSID_WHITE_LIST,                     /* 0x1010 */
    WIFI_SUBCMD_SET_ROAM_PARAMS,                         /* 0x1011 */
    WIFI_SUBCMD_ENABLE_LAZY_ROAM,                        /* 0x1012 */
    WIFI_SUBCMD_SET_BSSID_PREF,                          /* 0x1013 */
    WIFI_SUBCMD_SET_BSSID_BLACKLIST,                     /* 0x1014 */

    GSCAN_SUBCMD_ANQPO_CONFIG,                           /* 0x1015 */
    WIFI_SUBCMD_SET_RSSI_MONITOR,                        /* 0x1016 */
    WIFI_SUBCMD_CONFIG_ND_OFFLOAD,                       /* 0x1017 */
    WIFI_SUBCMD_CONFIG_TCPACK_SUP,                       /* 0x1018 */
    WIFI_SUBCMD_FW_ROAM_POLICY,                          /* 0x1019 */
    WIFI_SUBCMD_ROAM_CAPABILITY,                         /* 0x101a */
    WIFI_SUBCMD_SET_LATENCY_MODE,                        /* 0x101b */
    WIFI_SUBCMD_SET_MULTISTA_PRIMARY_CONNECTION,         /* 0x101c */
    WIFI_SUBCMD_SET_MULTISTA_USE_CASE,                   /* 0x101d */
    WIFI_SUBCMD_SET_DTIM_CONFIG,                         /* 0x101e */
    WIFI_SUBCMD_CHANNEL_POLICY,                         /* 0x101f */

    GSCAN_SUBCMD_MAX,

    /* NAN related */
    NAN_SUBCMD_ENABLE = ANDROID_NL80211_SUBCMD_NAN_RANGE_START,
    NAN_SUBCMD_DISABLE,                                 /* 0x1701 */
    NAN_SUBCMD_PUBLISH,                                 /* 0x1702 */
    NAN_SUBCMD_SUBSCRIBE,                               /* 0x1703 */
    NAN_SUBCMD_PUBLISH_CANCEL,                          /* 0x1704 */
    NAN_SUBCMD_SUBSCRIBE_CANCEL,                        /* 0x1705 */
    NAN_SUBCMD_TRANSMIT_FOLLOWUP,                       /* 0x1706 */
    NAN_SUBCMD_CONFIG,                                  /* 0x1707 */
    NAN_SUBCMD_TCA,                                     /* 0x1708 */
    NAN_SUBCMD_STATS,                                   /* 0x1709 */
    NAN_SUBCMD_GET_CAPABILITIES,                        /* 0x170A */
    NAN_SUBCMD_DATA_PATH_IFACE_CREATE,                  /* 0x170B */
    NAN_SUBCMD_DATA_PATH_IFACE_DELETE,                  /* 0x170C */
    NAN_SUBCMD_DATA_PATH_REQUEST,                       /* 0x170D */
    NAN_SUBCMD_DATA_PATH_RESPONSE,                      /* 0x170E */
    NAN_SUBCMD_DATA_PATH_END,                           /* 0x170F */
    NAN_SUBCMD_DATA_PATH_SEC_INFO,                      /* 0x1710 */
    NAN_SUBCMD_VERSION_INFO,                            /* 0x1711 */
    NAN_SUBCMD_ENABLE_MERGE,                            /* 0x1712 */
    NAN_SUBCMD_SUSPEND,                                 /* 0x1713 */
    NAN_SUBCMD_RESUME,                                  /* 0x1714 */
    NAN_SUBCMD_PAIRING_REQUEST,                         /* 0x1715 */
    NAN_SUBCMD_PAIRING_RESPONSE,                        /* 0x1716 */
    NAN_SUBCMD_PAIRING_END,                             /* 0x1717 */
    NAN_SUBCMD_BOOTSTRAPPING_REQUEST,                   /* 0x1718 */
    NAN_SUBCMD_BOOTSTRAPPING_RESPONSE,                  /* 0x1719 */
    APF_SUBCMD_GET_CAPABILITIES = ANDROID_NL80211_SUBCMD_PKT_FILTER_RANGE_START,
    APF_SUBCMD_SET_FILTER,
    APF_SUBCMD_READ_FILTER,
    WIFI_SUBCMD_TX_POWER_SCENARIO = ANDROID_NL80211_SUBCMD_TX_POWER_RANGE_START,
    WIFI_SUBCMD_THERMAL_MITIGATION = ANDROID_NL80211_SUBCMD_MITIGATION_RANGE_START,
    DSCP_SUBCMD_SET_TABLE = ANDROID_NL80211_SUBCMD_DSCP_RANGE_START,
    DSCP_SUBCMD_RESET_TABLE,                            /* 0x2001 */
    CHAVOID_SUBCMD_SET_CONFIG = ANDROID_NL80211_SUBCMD_CHAVOID_RANGE_START,

    TWT_SUBCMD_GETCAPABILITY	= ANDROID_NL80211_SUBCMD_TWT_START,
    TWT_SUBCMD_SESSION_SETUP_REQUEST,
    TWT_SUBCMD_SESSION_TEAR_DOWN_REQUEST,
    TWT_SUBCMD_SESSION_UPDATE_REQUEST,
    TWT_SUBCMD_SESSION_SUSPEND_REQUEST,
    TWT_SUBCMD_SESSION_RESUME_REQUEST,
    TWT_SUBCMD_SESSION_GETSTATS,
    TWT_SUBCMD_SESSION_CLR_STATS,

    WIFI_SUBCMD_CONFIG_VOIP_MODE = ANDROID_NL80211_SUBCMD_VIOP_MODE_START,

    WIFI_SUBCMD_GET_OTA_CURRUNT_INFO = ANDROID_NL80211_SUBCMD_OTA_DOWNLOAD_START,
    WIFI_SUBCMD_OTA_UPDATE,
    WIFI_SUBCMD_USABLE_CHANNEL = ANDROID_NL80211_SUBCMD_USABLE_CHANNEL_START,
    WIFI_SUBCMD_TRIGGER_SSR = ANDROID_NL80211_SUBCMD_INIT_DEINIT_RANGE_START,
    WIFI_SUBCMD_GET_RADIO_COMBO_MATRIX,
    WIFI_SUBCMD_ENABLE_TX_POWER_LIMIT,
    WIFI_SUBCMD_GET_CACHED_SCAN_RESULTS = ANDROID_NL80211_SUBCMD_SCAN_START,
} WIFI_SUB_COMMAND;

typedef enum {
    BRCM_RESERVED1                          = 0,
    BRCM_RESERVED2                          = 1,
    GSCAN_EVENT_SIGNIFICANT_CHANGE_RESULTS	= 2,
    GSCAN_EVENT_HOTLIST_RESULTS_FOUND       = 3,
    GSCAN_EVENT_SCAN_RESULTS_AVAILABLE      = 4,
    GSCAN_EVENT_FULL_SCAN_RESULTS           = 5,
    RTT_EVENT_COMPLETE                      = 6,
    GSCAN_EVENT_COMPLETE_SCAN               = 7,
    GSCAN_EVENT_HOTLIST_RESULTS_LOST        = 8,
    GSCAN_EVENT_EPNO_EVENT                  = 9,
    GOOGLE_DEBUG_RING_EVENT                 = 10,
    GOOGLE_DEBUG_MEM_DUMP_EVENT             = 11,
    GSCAN_EVENT_ANQPO_HOTSPOT_MATCH         = 12,
    GOOGLE_RSSI_MONITOR_EVENT               = 13,
    GOOGLE_MKEEP_ALIVE                      = 14,

    /*
     * BRCM specific events should be placed after the Generic events
     * in order to match between the DHD and HAL
     */
    NAN_EVENT_ENABLED                      = 15,
    NAN_EVENT_DISABLED                     = 16,
    NAN_EVENT_SUBSCRIBE_MATCH              = 17,
    NAN_EVENT_PUBLISH_REPLIED_IND          = 18,
    NAN_EVENT_PUBLISH_TERMINATED           = 19,
    NAN_EVENT_SUBSCRIBE_TERMINATED         = 20,
    NAN_EVENT_DE_EVENT                     = 21,
    NAN_EVENT_FOLLOWUP                     = 22,
    NAN_EVENT_TRANSMIT_FOLLOWUP_IND        = 23,
    NAN_EVENT_DATA_REQUEST                 = 24,
    NAN_EVENT_DATA_CONFIRMATION            = 25,
    NAN_EVENT_DATA_END                     = 26,
    NAN_EVENT_BEACON                       = 27,
    NAN_EVENT_SDF                          = 28,
    NAN_EVENT_TCA                          = 29,
    NAN_EVENT_SUBSCRIBE_UNMATCH            = 30,
    NAN_EVENT_UNKNOWN                      = 31,
    BRCM_VENDOR_EVENT_HANGED               = 33,
 
    ROAM_EVENT_START,
    GOOGLE_FILE_DUMP_EVENT                 = 37,
    NAN_ASYNC_RESPONSE_DISABLED            = 40,
    BRCM_VENDOR_EVENT_TWT                  = 43,
    BRCM_TPUT_DUMP_EVENT                   = 44,
    NAN_EVENT_MATCH_EXPIRY                 = 45,
    NAN_EVENT_SUSPENSION_STATUS            = 49,
    NAN_EVENT_PAIRING_REQUEST              = 50,
    NAN_EVENT_PAIRING_CONFIRMATION         = 51,
    NAN_EVENT_PAIRING_END                  = 52,
    NAN_EVENT_BOOTSTRAPPING_REQUEST        = 53,
    NAN_EVENT_BOOTSTRAPPING_CONFIRMATION   = 54
    /* Add more events from here */
} WIFI_EVENT;

typedef void (*wifi_internal_event_handler) (wifi_handle handle, int events);

class WifiCommand;

typedef struct {
    int nl_cmd;
    uint32_t vendor_id;
    int vendor_subcmd;
    nl_recvmsg_msg_cb_t cb_func;
    void *cb_arg;
} cb_info;

typedef struct {
    wifi_request_id id;
    WifiCommand *cmd;
} cmd_info;

typedef struct {
    wifi_handle handle;                             // handle to wifi data
    char name[IFNAMSIZ+1];                          // interface name + trailing null
    int  id;                                        // id to use when talking to driver
    bool is_virtual;                                // mark each iface as virtual or static
} interface_info;

typedef enum {
    NAN_STATE_DISABLED = 0,
    NAN_STATE_AP = 1,
    NAN_STATE_CHRE = 2,
} nan_enab_state_t;

typedef struct {

    struct nl_sock *cmd_sock;                       // command socket object
    struct nl_sock *debug_cmd_sock;                 // command socket object
    struct nl_sock *event_sock;                     // event socket object
    int nl80211_family_id;                          // family id for 80211 driver
    int debug_nl80211_family_id;                    // family id for 80211 driver
    int cleanup_socks[2];                           // sockets used to implement wifi_cleanup

    bool in_event_loop;                             // Indicates that event loop is active
    bool clean_up;                                  // Indication to exit since cleanup has started

    wifi_internal_event_handler event_handler;      // default event handler
    wifi_cleaned_up_handler cleaned_up_handler;     // socket cleaned up handler

    cb_info *event_cb;                              // event callbacks
    int num_event_cb;                               // number of event callbacks
    int alloc_event_cb;                             // number of allocated callback objects
    pthread_mutex_t cb_lock;                        // mutex for the event_cb access

    cmd_info *cmd;                                  // Outstanding commands
    int num_cmd;                                    // number of commands
    int alloc_cmd;                                  // number of commands allocated

    interface_info **interfaces;                    // array of interfaces
    int num_interfaces;                             // number of interfaces
    int max_num_interfaces;                         // max number of interfaces
    wifi_subsystem_restart_handler restart_handler; // trigger sub system handler

    wifi_chre_handler chre_nan_cb;                  // chre CB for nan status
    nan_enab_state_t nan_state;                     // Nan enable state
    bool nan_pairing_supported;                     // Nan Pairing capability
    bool nan_suspend_supported;                     // Nan suspend/resume capability

    // add other details
} hal_info;

#define PNO_SSID_FOUND  0x1
#define PNO_SSID_LOST    0x2

typedef struct wifi_pno_result {
    unsigned char ssid[DOT11_MAX_SSID_LEN];
    unsigned char ssid_len;
    signed char rssi;
    u16 channel;
    u16 flags;
    mac_addr  bssid;
} wifi_pno_result_t;

typedef struct wifi_gscan_result {
    u64 ts;                           // Time of discovery
    u8 ssid[DOT11_MAX_SSID_LEN+1];    // null terminated
    mac_addr bssid;                   // BSSID
    u32 channel;                      // channel frequency in MHz
    s32 rssi;                         // in db
    u64 rtt;                          // in nanoseconds
    u64 rtt_sd;                       // standard deviation in rtt
    u16 beacon_period;                // units are Kusec
    u16 capability;                   // Capability information
    u32 pad;
} wifi_gscan_result_t;

typedef struct wifi_gscan_full_result {
    wifi_gscan_result_t fixed;
    u32 scan_ch_bucket;              // scan chbucket bitmask
    u32 ie_length;                   // byte length of Information Elements
    u8  ie_data[1];                  // IE data to follow
} wifi_gscan_full_result_t;

wifi_error wifi_twt_session_setup(wifi_request_id id, wifi_interface_handle iface,
        wifi_twt_request request);
wifi_error wifi_twt_session_update(wifi_request_id id, wifi_interface_handle iface,
        int session_id, wifi_twt_request request);
wifi_error wifi_twt_session_suspend(wifi_request_id id, wifi_interface_handle iface,
        int session_id);
wifi_error wifi_twt_session_resume(wifi_request_id id, wifi_interface_handle iface,
        int session_id);
wifi_error wifi_twt_session_teardown(wifi_request_id id, wifi_interface_handle iface,
        int session_id);
wifi_error wifi_twt_session_get_stats(wifi_request_id id, wifi_interface_handle iface,
        int session_id);
wifi_error wifi_twt_get_capabilities(wifi_interface_handle iface,
        wifi_twt_capabilities* capabilities);
wifi_error wifi_twt_register_events(wifi_interface_handle iface, wifi_twt_events events);

void twt_deinit_handler();

typedef enum {
    TWT_EVENT_INVALID            = 0,
    TWT_SESSION_FAILURE          = 1,
    TWT_SESSION_SETUP_CREATE     = 2,
    TWT_SESSION_SETUP_UPDATE     = 3,
    TWT_SESSION_TEARDOWN         = 4,
    TWT_SESSION_STATS            = 5,
    TWT_SESSION_SUSPEND          = 6,
    TWT_SESSION_RESUME           = 7,
    TWT_EVENT_LAST
} TwtEventType;

typedef enum {
    TWT_INVALID                     = 0,
    TWT_GET_CAPABILITIES            = 1,
    TWT_SESSION_SETUP_REQUEST       = 2,
    TWT_SESSION_UPDATE_REQUEST	    = 3,
    TWT_SESSION_SUSPEND_REQUEST	    = 4,
    TWT_SESSION_RESUME_REQUEST	    = 5,
    TWT_SESSION_TEAR_DOWN_REQUEST   = 6,
    TWT_SESSION_GET_STATS           = 7,
    TWT_SESSION_CLEAR_STATS         = 8,
    TWT_LAST
} TwtRequestType;

typedef enum {
    TWT_ATTRIBUTE_INVALID                 = 0,
    TWT_ATTRIBUTE_SESSION_ID              = 1,
    TWT_ATTRIBUTE_MLO_LINK_ID             = 2,
    TWT_ATTRIBUTE_WAKE_DUR_MICROS         = 3,
    TWT_ATTRIBUTE_WAKE_INTERVAL_MICROS    = 4,
    TWT_ATTRIBUTE_NEG_TYPE                = 5,
    TWT_ATTRIBUTE_IS_TRIGGER_ENABLED      = 6,
    TWT_ATTRIBUTE_IS_ANNOUNCED            = 7,
    TWT_ATTRIBUTE_IS_IMPLICIT             = 8,
    TWT_ATTRIBUTE_IS_PROTECTED            = 9,
    TWT_ATTRIBUTE_IS_UPDATABLE            = 10,
    TWT_ATTRIBUTE_IS_SUSPENDABLE          = 11,
    TWT_ATTRIBUTE_IS_RESP_PM_MODE_ENABLED = 12,
    TWT_ATTRIBUTE_REASON_CODE             = 13,
    TWT_ATTRIBUTE_AVG_PKT_NUM_TX          = 14,
    TWT_ATTRIBUTE_AVG_PKT_NUM_RX          = 15,
    TWT_ATTRIBUTE_AVG_TX_PKT_SIZE         = 16,
    TWT_ATTRIBUTE_AVG_RX_PKT_SIZE         = 17,
    TWT_ATTRIBUTE_AVG_EOSP_DUR_US         = 18,
    TWT_ATTRIBUTE_EOSP_COUNT              = 19,
    TWT_ATTRIBUTE_MIN_WAKE_DURATION_US    = 20,
    TWT_ATTRIBUTE_MAX_WAKE_DURATION_US    = 21,
    TWT_ATTRIBUTE_MIN_WAKE_INTERVAL_US    = 22,
    TWT_ATTRIBUTE_MAX_WAKE_INTERVAL_US    = 23,
    TWT_ATTRIBUTE_ERROR_CODE              = 24,
    TWT_ATTRIBUTE_SUB_EVENT               = 25,
    TWT_ATTRIBUTE_CAP                     = 26,
    TWT_ATTRIBUTE_IS_REQUESTOR_SUPPORTED  = 27,
    TWT_ATTRIBUTE_IS_RESPONDER_SUPPORTED  = 28,
    TWT_ATTRIBUTE_IS_BROADCAST_SUPPORTED  = 29,
    TWT_ATTRIBUTE_IS_FLEXIBLE_SUPPORTED	  = 30,
    TWT_ATTRIBUTE_WIFI_ERROR              = 31,
    TWT_ATTRIBUTE_MAX
} TWT_ATTRIBUTE;

wifi_error wifi_register_handler(wifi_handle handle, int cmd, nl_recvmsg_msg_cb_t func, void *arg);
wifi_error wifi_register_vendor_handler(wifi_handle handle,
            uint32_t id, int subcmd, nl_recvmsg_msg_cb_t func, void *arg);

void wifi_unregister_handler(wifi_handle handle, int cmd);
void wifi_unregister_vendor_handler_without_lock(wifi_handle handle, uint32_t id, int subcmd);
void wifi_unregister_vendor_handler(wifi_handle handle, uint32_t id, int subcmd);

wifi_error wifi_register_cmd(wifi_handle handle, int id, WifiCommand *cmd);
WifiCommand *wifi_unregister_cmd(wifi_handle handle, int id);
WifiCommand *wifi_get_cmd(wifi_handle handle, int id);
void wifi_unregister_cmd(wifi_handle handle, WifiCommand *cmd);

interface_info *getIfaceInfo(wifi_interface_handle);
wifi_handle getWifiHandle(wifi_interface_handle handle);
hal_info *getHalInfo(wifi_handle handle);
hal_info *getHalInfo(wifi_interface_handle handle);
wifi_handle getWifiHandle(hal_info *info);
wifi_interface_handle getIfaceHandle(interface_info *info);
wifi_error wifi_cancel_cmd(wifi_request_id id, wifi_interface_handle iface);
wifi_error wifi_get_cancel_cmd(wifi_request_id id, wifi_interface_handle iface);
wifi_error nan_deinit_handler();
wifi_error wifi_start_hal(wifi_interface_handle iface);
wifi_error wifi_stop_hal(wifi_interface_handle iface);
wifi_interface_handle wifi_get_wlan_interface(wifi_handle info,
	    wifi_interface_handle *ifaceHandles, int numIfaceHandles);
#ifdef RING_DUMP
wifi_error wifi_start_ring_dump(wifi_interface_handle iface,
            wifi_ring_buffer_data_handler ring_handle);
wifi_error wifi_stop_ring_dump(wifi_interface_handle iface);
#endif /* RING_DUMP */
wifi_error wifi_hal_ota_update(wifi_interface_handle iface, uint32_t ota_version);
wifi_error wifi_hal_preInit(wifi_interface_handle iface);
/* API to get wake reason statistics */
wifi_error wifi_get_wake_reason_stats(wifi_interface_handle handle,
        WLAN_DRIVER_WAKE_REASON_CNT *wifi_wake_reason_cnt);
wifi_error wifi_virtual_interface_create(wifi_handle handle, const char* ifname,
        wifi_interface_type iface_type);
wifi_error wifi_virtual_interface_delete(wifi_handle handle, const char* ifname);
wifi_error wifi_set_coex_unsafe_channels(wifi_handle handle, u32 num_channels,
                                         wifi_coex_unsafe_channel channels[], u32 restrictions);
wifi_error wifi_set_voip_mode(wifi_interface_handle handle, wifi_voip_mode mode);
wifi_error wifi_set_dtim_config(wifi_interface_handle handle, u32 multiplier);
void set_hautil_mode(bool halutil_mode);
bool get_halutil_mode();

wifi_error wifi_trigger_subsystem_restart(wifi_handle handle);

/**@brief nan_chre_enable_request
 *        Request from CHRE to enable NAN
 * @param transaction id:
 * @param wifi_interface_handle:
 * @param NanEnableRequest:
 * @return Synchronous wifi_error
 */
wifi_error nan_chre_enable_request(transaction_id id,
                        wifi_interface_handle iface,
                        NanEnableRequest* msg);

/**@brief nan_chre_disable_request
 *        Request from CHRE to disable NAN
 * @param transaction id:
 * @param wifi_interface_handle:
 * @return Synchronous wifi_error
 */
wifi_error nan_chre_disable_request(transaction_id id,
                        wifi_interface_handle iface);

/**@brief nan_chre_register_handler
 *        Register chre handler to handle NAN status
 * @param wifi_interface_handle:
 * @param wifi_chre_handler:
 * @return Synchronous wifi_error
 */
wifi_error nan_chre_register_handler(wifi_interface_handle iface,
                        wifi_chre_handler handler);

// some common macros

void prhex(const char *msg, u8 *buf, u32 nbytes);
#define min(x, y)       ((x) < (y) ? (x) : (y))
#define max(x, y)       ((x) > (y) ? (x) : (y))

#define NULL_CHECK_RETURN(ptr, str, ret) \
    do { \
        if (!(ptr)) { \
            ALOGE("%s(): null pointer - #ptr (%s)\n", __FUNCTION__, str); \
            return ret; \
        } \
    } while (0)

/* RTT Capabilities */
typedef struct rtt_capabilities {
    u8 rtt_one_sided_supported;  /* if 1-sided rtt data collection is supported */
    u8 rtt_ftm_supported;        /* if ftm rtt data collection is supported */
    u8 lci_support;              /* location configuration information */
    u8 lcr_support;              /* Civic Location */
    u8 preamble_support;         /* bit mask indicate what preamble is supported */
    u8 bw_support;               /* bit mask indicate what BW is supported */
    u8 PAD[2];
} rtt_capabilities_t;

typedef u16 rtt_cap_preamble_type_t;
typedef u16 base_akm_type_t;
typedef u16 rtt_cipher_suite_t;
/* RTT Capabilities v2 (11az support) */
typedef struct rtt_capabilities_mc_az {
    struct rtt_capabilities rtt_capab;
    /* 11AZ support */
    /* Bitmask of preamble supported by the 11az initiator */
    rtt_cap_preamble_type_t az_preamble_support;
    /* bitmask of BW supported by 11az initiator */
    u8 az_bw_support;
    /* if 11az non-TB initiator is supported */
    u8 ntb_initiator_supported;
    /* if 11az non-TB responder is supported */
    u8 ntb_responder_supported;
    /* if 11az secure ltf is supported */
    u8 secure_he_ltf_supported;
    /* if 11az protected ranging frame is supported */
    u8 protected_rtt_frm_supported;
    /* Supported AKM for secure ranging */
    base_akm_type_t supported_akms;
    /* Supported cipher type for secure ranging */
    rtt_cipher_suite_t supported_cipher_suites;
} rtt_capabilities_mc_az_t;
#endif

