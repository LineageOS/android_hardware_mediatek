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
#include <unistd.h>
#include <cutils/properties.h>


#include "nl80211_copy.h"
#include "sync.h"

#define LOG_TAG  "WifiHAL"

#include <log/log.h>

#include <hardware_legacy/wifi_hal.h>
#include "common.h"
#include "cpp_bindings.h"
#include <sys/stat.h>
#include "brcm_version.h"
#define WIFI_HAL_EVENT_SOCK_PORT     645

typedef enum {
    LOGGER_START_LOGGING = ANDROID_NL80211_SUBCMD_DEBUG_RANGE_START,
    LOGGER_TRIGGER_MEM_DUMP,
    LOGGER_GET_MEM_DUMP,
    LOGGER_GET_VER,
    LOGGER_GET_RING_STATUS,
    LOGGER_GET_RING_DATA,
    LOGGER_GET_FEATURE,
    LOGGER_RESET_LOGGING,
    LOGGER_TRIGGER_DRIVER_MEM_DUMP,
    LOGGER_GET_DRIVER_MEM_DUMP,
    LOGGER_START_PKT_FATE_MONITORING,
    LOGGER_GET_TX_PKT_FATES,
    LOGGER_GET_RX_PKT_FATES,
    LOGGER_GET_WAKE_REASON_STATS,
    LOGGER_DEBUG_GET_DUMP,
    LOGGER_FILE_DUMP_DONE_IND,
    LOGGER_SET_HAL_START,
    LOGGER_HAL_STOP,
    LOGGER_SET_HAL_PID,
    LOGGER_SET_TPUT_DEBUG_DUMP_CMD,
    LOGGER_GET_BUF_RING_MAP
} DEBUG_SUB_COMMAND;

#define MAX_NV_FILE 4
#define MAX_SKU_NAME_LEN 5
#define OTA_PATH "/data/vendor/firmware/wifi/"
#define OTA_CLM_FILE "bcmdhd_clm.blob"
#define OTA_TXCAP_BLOB_FILE "bcmdhd_txcap.blob"
#define OTA_NVRAM_FILE "bcmdhd.cal"
#define HW_DEV_PROP "ro.revision"
#define HW_SKU_PROP "ro.boot.hardware.sku"

typedef enum {
    NVRAM,
    CLM_BLOB,
    TXCAP_BLOB
} OTA_TYPE;

char ota_nvram_ext[10];
typedef struct ota_info_buf {
    u32 ota_clm_len;
    const void *ota_clm_buf[1];
    u32 ota_nvram_len;
    const void *ota_nvram_buf[1];
    u32 ota_txcap_len;
    const void *ota_txcap_buf[1];
} ota_info_buf_t;
u32 applied_ota_version = 0;

typedef enum {
    LOGGER_ATTRIBUTE_INVALID			= 0,
    LOGGER_ATTRIBUTE_DRIVER_VER			= 1,
    LOGGER_ATTRIBUTE_FW_VER			= 2,
    LOGGER_ATTRIBUTE_RING_ID			= 3,
    LOGGER_ATTRIBUTE_RING_NAME			= 4,
    LOGGER_ATTRIBUTE_RING_FLAGS			= 5,
    LOGGER_ATTRIBUTE_LOG_LEVEL			= 6,
    LOGGER_ATTRIBUTE_LOG_TIME_INTVAL		= 7,
    LOGGER_ATTRIBUTE_LOG_MIN_DATA_SIZE		= 8,
    LOGGER_ATTRIBUTE_FW_DUMP_LEN		= 9,
    LOGGER_ATTRIBUTE_FW_DUMP_DATA		= 10,
    LOGGER_ATTRIBUTE_FW_ERR_CODE		= 11,
    LOGGER_ATTRIBUTE_RING_DATA			= 12,
    LOGGER_ATTRIBUTE_RING_STATUS		= 13,
    LOGGER_ATTRIBUTE_RING_NUM			= 14,
    LOGGER_ATTRIBUTE_DRIVER_DUMP_LEN		= 15,
    LOGGER_ATTRIBUTE_DRIVER_DUMP_DATA		= 16,
    LOGGER_ATTRIBUTE_PKT_FATE_NUM		= 17,
    LOGGER_ATTRIBUTE_PKT_FATE_DATA		= 18,
    LOGGER_ATTRIBUTE_HANG_REASON		= 19,
    LOGGER_ATTRIBUTE_BUF_RING_NUM		= 20,
    LOGGER_ATTRIBUTE_BUF_RING_MAP		= 21,
    /* Add new attributes just above this */
    LOGGER_ATTRIBUTE_MAX
} LOGGER_ATTRIBUTE;

typedef enum {
    DEBUG_OFF = 0,
    DEBUG_NORMAL,
    DEBUG_VERBOSE,
    DEBUG_VERY,
    DEBUG_VERY_VERY,
} LOGGER_LEVEL;

typedef enum {
    GET_FW_VER,
    GET_DRV_VER,
    GET_RING_DATA,
    GET_RING_STATUS,
    GET_FEATURE,
    START_RING_LOG,
    GET_BUF_RING_MAP,
} GetCmdType;

typedef enum {
    PACKET_MONITOR_START,
    TX_PACKET_FATE,
    RX_PACKET_FATE,
} PktFateReqType;

enum wake_stat_attributes {
    WAKE_STAT_ATTRIBUTE_INVALID,
    WAKE_STAT_ATTRIBUTE_TOTAL,
    WAKE_STAT_ATTRIBUTE_WAKE,
    WAKE_STAT_ATTRIBUTE_COUNT,
    WAKE_STAT_ATTRIBUTE_CMD_COUNT_USED,
    WAKE_STAT_ATTRIBUTE_TOTAL_DRIVER_FW,
    WAKE_STAT_ATTRIBUTE_DRIVER_FW_WAKE,
    WAKE_STAT_ATTRIBUTE_DRIVER_FW_COUNT,
    WAKE_STAT_ATTRIBUTE_DRIVER_FW_COUNT_USED,
    WAKE_STAT_ATTRIBUTE_TOTAL_RX_DATA_WAKE,
    WAKE_STAT_ATTRIBUTE_RX_UNICAST_COUNT,
    WAKE_STAT_ATTRIBUTE_RX_MULTICAST_COUNT,
    WAKE_STAT_ATTRIBUTE_RX_BROADCAST_COUNT,
    WAKE_STAT_ATTRIBUTE_RX_ICMP_PKT,
    WAKE_STAT_ATTRIBUTE_RX_ICMP6_PKT,
    WAKE_STAT_ATTRIBUTE_RX_ICMP6_RA,
    WAKE_STAT_ATTRIBUTE_RX_ICMP6_NA,
    WAKE_STAT_ATTRIBUTE_RX_ICMP6_NS,
    WAKE_STAT_ATTRIBUTE_IPV4_RX_MULTICAST_ADD_CNT,
    WAKE_STAT_ATTRIBUTE_IPV6_RX_MULTICAST_ADD_CNT,
    WAKE_STAT_ATTRIBUTE_OTHER_RX_MULTICAST_ADD_CNT,
    WAKE_STAT_ATTRIBUTE_RX_MULTICAST_PKT_INFO,
    WAKE_STAT_ATTRIBUTE_MAX
};

typedef enum {
    SET_HAL_START_ATTRIBUTE_DEINIT = 0x0001,
    SET_HAL_START_ATTRIBUTE_PRE_INIT = 0x0002,
    SET_HAL_START_ATTRIBUTE_EVENT_SOCK_PID = 0x0003
} SET_HAL_START_ATTRIBUTE;

typedef enum {
    OTA_DOWNLOAD_CLM_LENGTH_ATTR    = 0x0001,
    OTA_DOWNLOAD_CLM_ATTR           = 0x0002,
    OTA_DOWNLOAD_NVRAM_LENGTH_ATTR  = 0x0003,
    OTA_DOWNLOAD_NVRAM_ATTR         = 0x0004,
    OTA_SET_FORCE_REG_ON            = 0x0005,
    OTA_CUR_NVRAM_EXT_ATTR          = 0x0006,
    OTA_DOWNLOAD_TXCAP_BLOB_LENGTH_ATTR    = 0x0007,
    OTA_DOWNLOAD_TXCAP_BLOB_ATTR           = 0x0008,
} OTA_DOWNLOAD_ATTRIBUTE;

#define C2S(x)  case x: return #x;
static const char *DebugDumpToString(int mType);
static const char *DebugDumpToString(int mType)
{
    switch (mType) {
        C2S(GET_FW_VER)
        C2S(GET_DRV_VER)
        C2S(GET_RING_DATA)
        C2S(GET_RING_STATUS)
        C2S(GET_FEATURE)
        C2S(START_RING_LOG)
        C2S(GET_BUF_RING_MAP)
        default:
            return "DUMP_TYPE_INVALID";
    }
}

#define HAL_START_REQUEST_ID 2
#define HAL_RESTART_ID 3
#define FILE_NAME_LEN 256
#define RING_NAME_LEN 32
#if defined(RING_DUMP)
/* Loglevel */
#define DUMP_DEBUG(x)
#define DUMP_INFO(x) ALOGI x
#define FILE_DUMP_REQUEST_ID 2

static const char *EWP_EventAttrToString(int len_attr);
static const char *EWP_CmdAttrToString(int data_attr);

typedef struct buf_data {
    u32 ver; /* version of struct */
    u32 len; /* Total len */
    /* size of each buffer in case of split buffers (0 - single buffer). */
    u32 buf_threshold;
    const void *data_buf[1]; /* array of user space buffer pointers.*/
} buf_data_t;

/* Attributes associated with GOOGLE_FILE_DUMP_EVENT */
typedef enum {
    DUMP_LEN_ATTR_INVALID                       = 0,
    DUMP_LEN_ATTR_MEMDUMP                       = 1,
    DUMP_LEN_ATTR_SSSR_C0_BEFORE                = 2,
    DUMP_LEN_ATTR_SSSR_C0_AFTER                 = 3,
    DUMP_LEN_ATTR_SSSR_C1_BEFORE                = 4,
    DUMP_LEN_ATTR_SSSR_C1_AFTER                 = 5,
    DUMP_LEN_ATTR_SSSR_C2_BEFORE                = 6,
    DUMP_LEN_ATTR_SSSR_C2_AFTER                 = 7,
    DUMP_LEN_ATTR_SSSR_DIG_BEFORE               = 8,
    DUMP_LEN_ATTR_SSSR_DIG_AFTER                = 9,
    DUMP_LEN_ATTR_TIMESTAMP                     = 10,
    DUMP_LEN_ATTR_GENERAL_LOG                   = 11,
    DUMP_LEN_ATTR_ECNTRS                        = 12,
    DUMP_LEN_ATTR_SPECIAL_LOG                   = 13,
    DUMP_LEN_ATTR_DHD_DUMP                      = 14,
    DUMP_LEN_ATTR_EXT_TRAP                      = 15,
    DUMP_LEN_ATTR_HEALTH_CHK                    = 16,
    DUMP_LEN_ATTR_PRESERVE_LOG                  = 17,
    DUMP_LEN_ATTR_COOKIE                        = 18,
    DUMP_LEN_ATTR_FLOWRING_DUMP                 = 19,
    DUMP_LEN_ATTR_PKTLOG                        = 20,
    DUMP_LEN_ATTR_PKTLOG_DEBUG                  = 21,
    DUMP_FILENAME_ATTR_DEBUG_DUMP               = 22,
    DUMP_FILENAME_ATTR_MEM_DUMP                 = 23,
    DUMP_FILENAME_ATTR_SSSR_CORE_0_BEFORE_DUMP  = 24,
    DUMP_FILENAME_ATTR_SSSR_CORE_0_AFTER_DUMP   = 25,
    DUMP_FILENAME_ATTR_SSSR_CORE_1_BEFORE_DUMP  = 26,
    DUMP_FILENAME_ATTR_SSSR_CORE_1_AFTER_DUMP   = 27,
    DUMP_FILENAME_ATTR_SSSR_CORE_2_BEFORE_DUMP  = 28,
    DUMP_FILENAME_ATTR_SSSR_CORE_2_AFTER_DUMP   = 29,
    DUMP_FILENAME_ATTR_SSSR_DIG_BEFORE_DUMP     = 30,
    DUMP_FILENAME_ATTR_SSSR_DIG_AFTER_DUMP      = 31,
    DUMP_FILENAME_ATTR_PKTLOG_DUMP              = 32,
    DUMP_FILENAME_ATTR_PKTLOG_DEBUG_DUMP        = 33,
    DUMP_LEN_ATTR_STATUS_LOG                    = 34,
    DUMP_LEN_ATTR_AXI_ERROR                     = 35,
    DUMP_FILENAME_ATTR_AXI_ERROR_DUMP           = 36,
    DUMP_LEN_ATTR_RTT_LOG                       = 37,
    DUMP_LEN_ATTR_SDTC_ETB_DUMP                 = 38,
    DUMP_FILENAME_ATTR_SDTC_ETB_DUMP            = 39,
    DUMP_LEN_ATTR_PKTID_MAP_LOG                 = 40,
    DUMP_LEN_ATTR_PKTID_UNMAP_LOG               = 41,
    DUMP_LEN_ATTR_EWP_HW_INIT_LOG               = 42,
    DUMP_LEN_ATTR_EWP_HW_MOD_DUMP               = 43,
    DUMP_LEN_ATTR_EWP_HW_REG_DUMP               = 44,
    DUMP_LEN_ATTR_WRAPPER_REG_DUMP              = 45,
    /*  Please add new attributes from here to sync up old DHD */
    DUMP_EVENT_ATTR_MAX                         = 46,
} EWP_DUMP_EVENT_ATTRIBUTE;

/* Attributes associated with DEBUG_GET_DUMP_BUF */
typedef enum {
    DUMP_BUF_ATTR_INVALID               = 0,
    DUMP_BUF_ATTR_MEMDUMP               = 1,
    DUMP_BUF_ATTR_SSSR_C0_BEFORE        = 2,
    DUMP_BUF_ATTR_SSSR_C0_AFTER         = 3,
    DUMP_BUF_ATTR_SSSR_C1_BEFORE        = 4,
    DUMP_BUF_ATTR_SSSR_C1_AFTER         = 5,
    DUMP_BUF_ATTR_SSSR_C2_BEFORE        = 6,
    DUMP_BUF_ATTR_SSSR_C2_AFTER         = 7,
    DUMP_BUF_ATTR_SSSR_DIG_BEFORE       = 8,
    DUMP_BUF_ATTR_SSSR_DIG_AFTER        = 9,
    DUMP_BUF_ATTR_TIMESTAMP             = 10,
    DUMP_BUF_ATTR_GENERAL_LOG           = 11,
    DUMP_BUF_ATTR_ECNTRS                = 12,
    DUMP_BUF_ATTR_SPECIAL_LOG           = 13,
    DUMP_BUF_ATTR_DHD_DUMP              = 14,
    DUMP_BUF_ATTR_EXT_TRAP              = 15,
    DUMP_BUF_ATTR_HEALTH_CHK            = 16,
    DUMP_BUF_ATTR_PRESERVE_LOG          = 17,
    DUMP_BUF_ATTR_COOKIE                = 18,
    DUMP_BUF_ATTR_FLOWRING_DUMP         = 19,
    DUMP_BUF_ATTR_PKTLOG                = 20,
    DUMP_BUF_ATTR_PKTLOG_DEBUG          = 21,
    DUMP_BUF_ATTR_STATUS_LOG            = 22,
    DUMP_BUF_ATTR_AXI_ERROR             = 23,
    DUMP_BUF_ATTR_RTT_LOG               = 24,
    DUMP_BUF_ATTR_SDTC_ETB_DUMP         = 25,
    DUMP_BUF_ATTR_PKTID_MAP_LOG         = 26,
    DUMP_BUF_ATTR_PKTID_UNMAP_LOG       = 27,
    DUMP_BUF_ATTR_EWP_HW_INIT_LOG       = 28,
    DUMP_BUF_ATTR_EWP_HW_MOD_DUMP       = 29,
    DUMP_BUF_ATTR_EWP_HW_REG_DUMP       = 30,
    DUMP_BUF_ATTR_WRAPPER_REG_DUMP      = 31,
    /*  Please add new attributes from here to sync up old DHD */
    DUMP_BUF_ATTR_MAX                   = 32,
} EWP_DUMP_CMD_ATTRIBUTE;

typedef enum {
    DUMP_TYPE_MEM_DUMP                  = 0,
    DUMP_TYPE_DEBUG_DUMP                = 1,
    DUMP_TYPE_SSSR_CORE0_BEF_DUMP       = 2,
    DUMP_TYPE_SSSR_CORE0_AFT_DUMP       = 3,
    DUMP_TYPE_SSSR_CORE1_BEF_DUMP       = 4,
    DUMP_TYPE_SSSR_CORE1_AFT_DUMP       = 5,
    DUMP_TYPE_SSSR_CORE2_BEF_DUMP       = 6,
    DUMP_TYPE_SSSR_CORE2_AFT_DUMP       = 7,
    DUMP_TYPE_SSSR_DIG_BEF_DUMP         = 8,
    DUMP_TYPE_SSSR_DIG_AFT_DUMP         = 9,
    DUMP_TYPE_PKTLOG_DUMP               = 10,
    DUMP_TYPE_PKTLOG_DEBUG_DUMP         = 11,
    DUMP_TYPE_AXI_ERROR_DUMP            = 12,
    DUMP_TYPE_D2H_MINI_DUMP             = 13,
    DUMP_TYPE_SDTC_ETB_DUMP             = 14,
    /*  Please add new attributes from here to sync up old DHD */
    DUMP_TYPE_MAX                       = 15,
} EWP_DUMP_TYPE;

/* Struct for table which has len_attr, data_attr and dump file type attr */
typedef struct logger_attr_entry {
    u8 attr_type; /* Type of attribute */
    u8 buf_attr; /* Buffer associated with the attribute */
    u8 dump_type; /* Each attribute will be linked to a dump type */
} logger_attr_entry_t;

logger_attr_entry_t attr_lookup_tbl[] = {
    /* Mem Dump Block */
    {DUMP_FILENAME_ATTR_MEM_DUMP, 0, DUMP_TYPE_MEM_DUMP},
    {DUMP_LEN_ATTR_MEMDUMP, DUMP_BUF_ATTR_MEMDUMP, DUMP_TYPE_MEM_DUMP},
    /* SSSR Dump Block */
    {DUMP_FILENAME_ATTR_SSSR_CORE_0_BEFORE_DUMP, 0, DUMP_TYPE_SSSR_CORE0_BEF_DUMP},
    {DUMP_LEN_ATTR_SSSR_C0_BEFORE, DUMP_BUF_ATTR_SSSR_C0_BEFORE, DUMP_TYPE_SSSR_CORE0_BEF_DUMP},

    {DUMP_FILENAME_ATTR_SSSR_CORE_0_AFTER_DUMP, 0, DUMP_TYPE_SSSR_CORE0_AFT_DUMP},
    {DUMP_LEN_ATTR_SSSR_C0_AFTER, DUMP_BUF_ATTR_SSSR_C0_AFTER, DUMP_TYPE_SSSR_CORE0_AFT_DUMP},

    {DUMP_FILENAME_ATTR_SSSR_CORE_1_BEFORE_DUMP, 0, DUMP_TYPE_SSSR_CORE1_BEF_DUMP},
    {DUMP_LEN_ATTR_SSSR_C1_BEFORE, DUMP_BUF_ATTR_SSSR_C1_BEFORE, DUMP_TYPE_SSSR_CORE1_BEF_DUMP},

    {DUMP_FILENAME_ATTR_SSSR_CORE_1_AFTER_DUMP, 0, DUMP_TYPE_SSSR_CORE1_AFT_DUMP},
    {DUMP_LEN_ATTR_SSSR_C1_AFTER, DUMP_BUF_ATTR_SSSR_C1_AFTER, DUMP_TYPE_SSSR_CORE1_AFT_DUMP},

    {DUMP_FILENAME_ATTR_SSSR_CORE_2_BEFORE_DUMP, 0, DUMP_TYPE_SSSR_CORE2_BEF_DUMP},
    {DUMP_LEN_ATTR_SSSR_C2_BEFORE, DUMP_BUF_ATTR_SSSR_C2_BEFORE, DUMP_TYPE_SSSR_CORE2_BEF_DUMP},

    {DUMP_FILENAME_ATTR_SSSR_CORE_2_AFTER_DUMP, 0, DUMP_TYPE_SSSR_CORE2_AFT_DUMP},
    {DUMP_LEN_ATTR_SSSR_C2_AFTER, DUMP_BUF_ATTR_SSSR_C2_AFTER, DUMP_TYPE_SSSR_CORE2_AFT_DUMP},

    {DUMP_FILENAME_ATTR_SSSR_DIG_BEFORE_DUMP, 0, DUMP_TYPE_SSSR_DIG_BEF_DUMP},
    {DUMP_LEN_ATTR_SSSR_DIG_BEFORE, DUMP_BUF_ATTR_SSSR_DIG_BEFORE, DUMP_TYPE_SSSR_DIG_BEF_DUMP},

    {DUMP_FILENAME_ATTR_SSSR_DIG_AFTER_DUMP, 0, DUMP_TYPE_SSSR_DIG_AFT_DUMP},
    {DUMP_LEN_ATTR_SSSR_DIG_AFTER, DUMP_BUF_ATTR_SSSR_DIG_AFTER, DUMP_TYPE_SSSR_DIG_AFT_DUMP},

    /* Debug Dump Block */
    {DUMP_FILENAME_ATTR_DEBUG_DUMP, 0, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_TIMESTAMP, DUMP_BUF_ATTR_TIMESTAMP, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_GENERAL_LOG, DUMP_BUF_ATTR_GENERAL_LOG, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_ECNTRS, DUMP_BUF_ATTR_ECNTRS, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_SPECIAL_LOG, DUMP_BUF_ATTR_SPECIAL_LOG, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_DHD_DUMP, DUMP_BUF_ATTR_DHD_DUMP, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_EXT_TRAP, DUMP_BUF_ATTR_EXT_TRAP, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_HEALTH_CHK, DUMP_BUF_ATTR_HEALTH_CHK, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_PRESERVE_LOG, DUMP_BUF_ATTR_PRESERVE_LOG, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_COOKIE, DUMP_BUF_ATTR_COOKIE, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_FLOWRING_DUMP, DUMP_BUF_ATTR_FLOWRING_DUMP, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_STATUS_LOG, DUMP_BUF_ATTR_STATUS_LOG, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_RTT_LOG, DUMP_BUF_ATTR_RTT_LOG, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_PKTID_MAP_LOG, DUMP_BUF_ATTR_PKTID_MAP_LOG, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_PKTID_UNMAP_LOG, DUMP_BUF_ATTR_PKTID_UNMAP_LOG, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_EWP_HW_INIT_LOG, DUMP_BUF_ATTR_EWP_HW_INIT_LOG, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_EWP_HW_MOD_DUMP, DUMP_BUF_ATTR_EWP_HW_MOD_DUMP, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_EWP_HW_REG_DUMP, DUMP_BUF_ATTR_EWP_HW_REG_DUMP, DUMP_TYPE_DEBUG_DUMP},
    {DUMP_LEN_ATTR_WRAPPER_REG_DUMP, DUMP_BUF_ATTR_WRAPPER_REG_DUMP, DUMP_TYPE_DEBUG_DUMP},

    /* PKT log dump block */
    {DUMP_FILENAME_ATTR_PKTLOG_DUMP, 0, DUMP_TYPE_PKTLOG_DUMP},
    {DUMP_LEN_ATTR_PKTLOG, DUMP_BUF_ATTR_PKTLOG, DUMP_TYPE_PKTLOG_DUMP},
    {DUMP_FILENAME_ATTR_PKTLOG_DEBUG_DUMP, 0, DUMP_TYPE_PKTLOG_DEBUG_DUMP},
    {DUMP_LEN_ATTR_PKTLOG_DEBUG, DUMP_BUF_ATTR_PKTLOG_DEBUG, DUMP_TYPE_PKTLOG_DEBUG_DUMP},
    /* AXI error log dump block */
    {DUMP_FILENAME_ATTR_AXI_ERROR_DUMP, 0, DUMP_TYPE_AXI_ERROR_DUMP},
    {DUMP_LEN_ATTR_AXI_ERROR, DUMP_BUF_ATTR_AXI_ERROR, DUMP_TYPE_AXI_ERROR_DUMP},
    /* SDTC etb log dump block */
    {DUMP_FILENAME_ATTR_SDTC_ETB_DUMP, 0, DUMP_TYPE_SDTC_ETB_DUMP},
    {DUMP_LEN_ATTR_SDTC_ETB_DUMP, DUMP_BUF_ATTR_SDTC_ETB_DUMP, DUMP_TYPE_SDTC_ETB_DUMP},
    {DUMP_EVENT_ATTR_MAX, 0, 0},
};

static const char *EWP_EventAttrToString(int len_attr)
{
    switch (len_attr) {
        C2S(DUMP_LEN_ATTR_MEMDUMP)
        C2S(DUMP_LEN_ATTR_SSSR_C0_BEFORE)
        C2S(DUMP_LEN_ATTR_SSSR_C0_AFTER)
        C2S(DUMP_LEN_ATTR_SSSR_C1_BEFORE)
        C2S(DUMP_LEN_ATTR_SSSR_C1_AFTER)
        C2S(DUMP_LEN_ATTR_SSSR_C2_BEFORE)
        C2S(DUMP_LEN_ATTR_SSSR_C2_AFTER)
        C2S(DUMP_LEN_ATTR_SSSR_DIG_BEFORE)
        C2S(DUMP_LEN_ATTR_SSSR_DIG_AFTER)
        C2S(DUMP_LEN_ATTR_TIMESTAMP)
        C2S(DUMP_LEN_ATTR_GENERAL_LOG)
        C2S(DUMP_LEN_ATTR_ECNTRS)
        C2S(DUMP_LEN_ATTR_SPECIAL_LOG)
        C2S(DUMP_LEN_ATTR_DHD_DUMP)
        C2S(DUMP_LEN_ATTR_EXT_TRAP)
        C2S(DUMP_LEN_ATTR_HEALTH_CHK)
        C2S(DUMP_LEN_ATTR_PRESERVE_LOG)
        C2S(DUMP_LEN_ATTR_COOKIE)
        C2S(DUMP_LEN_ATTR_FLOWRING_DUMP)
        C2S(DUMP_LEN_ATTR_PKTLOG)
        C2S(DUMP_LEN_ATTR_PKTLOG_DEBUG)
        C2S(DUMP_LEN_ATTR_STATUS_LOG)
        C2S(DUMP_FILENAME_ATTR_DEBUG_DUMP)
        C2S(DUMP_FILENAME_ATTR_MEM_DUMP)
        C2S(DUMP_FILENAME_ATTR_SSSR_CORE_0_BEFORE_DUMP)
        C2S(DUMP_FILENAME_ATTR_SSSR_CORE_0_AFTER_DUMP)
        C2S(DUMP_FILENAME_ATTR_SSSR_CORE_1_BEFORE_DUMP)
        C2S(DUMP_FILENAME_ATTR_SSSR_CORE_1_AFTER_DUMP)
        C2S(DUMP_FILENAME_ATTR_SSSR_CORE_2_BEFORE_DUMP)
        C2S(DUMP_FILENAME_ATTR_SSSR_CORE_2_AFTER_DUMP)
        C2S(DUMP_FILENAME_ATTR_SSSR_DIG_BEFORE_DUMP)
        C2S(DUMP_FILENAME_ATTR_SSSR_DIG_AFTER_DUMP)
        C2S(DUMP_FILENAME_ATTR_PKTLOG_DUMP)
        C2S(DUMP_FILENAME_ATTR_PKTLOG_DEBUG_DUMP)
        C2S(DUMP_LEN_ATTR_AXI_ERROR)
        C2S(DUMP_FILENAME_ATTR_AXI_ERROR_DUMP)
        C2S(DUMP_LEN_ATTR_RTT_LOG)
        C2S(DUMP_FILENAME_ATTR_SDTC_ETB_DUMP)
        C2S(DUMP_LEN_ATTR_SDTC_ETB_DUMP)
        C2S(DUMP_LEN_ATTR_PKTID_MAP_LOG)
        C2S(DUMP_LEN_ATTR_PKTID_UNMAP_LOG)
        C2S(DUMP_LEN_ATTR_EWP_HW_INIT_LOG)
        C2S(DUMP_LEN_ATTR_EWP_HW_MOD_DUMP)
        C2S(DUMP_LEN_ATTR_EWP_HW_REG_DUMP)
        C2S(DUMP_LEN_ATTR_WRAPPER_REG_DUMP)
        default:
            return "DUMP_LEN_ATTR_INVALID";
    }
}

static const char *EWP_CmdAttrToString(int attr)
{
    switch (attr) {
        C2S(DUMP_BUF_ATTR_MEMDUMP)
        C2S(DUMP_BUF_ATTR_SSSR_C0_BEFORE)
        C2S(DUMP_BUF_ATTR_SSSR_C0_AFTER)
        C2S(DUMP_BUF_ATTR_SSSR_C1_BEFORE)
        C2S(DUMP_BUF_ATTR_SSSR_C1_AFTER)
        C2S(DUMP_BUF_ATTR_SSSR_C2_BEFORE)
        C2S(DUMP_BUF_ATTR_SSSR_C2_AFTER)
        C2S(DUMP_BUF_ATTR_SSSR_DIG_BEFORE)
        C2S(DUMP_BUF_ATTR_SSSR_DIG_AFTER)
        C2S(DUMP_BUF_ATTR_TIMESTAMP)
        C2S(DUMP_BUF_ATTR_GENERAL_LOG)
        C2S(DUMP_BUF_ATTR_ECNTRS)
        C2S(DUMP_BUF_ATTR_SPECIAL_LOG)
        C2S(DUMP_BUF_ATTR_DHD_DUMP)
        C2S(DUMP_BUF_ATTR_EXT_TRAP)
        C2S(DUMP_BUF_ATTR_HEALTH_CHK)
        C2S(DUMP_BUF_ATTR_PRESERVE_LOG)
        C2S(DUMP_BUF_ATTR_COOKIE)
        C2S(DUMP_BUF_ATTR_FLOWRING_DUMP)
        C2S(DUMP_BUF_ATTR_PKTLOG)
        C2S(DUMP_BUF_ATTR_PKTLOG_DEBUG)
        C2S(DUMP_BUF_ATTR_STATUS_LOG)
        C2S(DUMP_BUF_ATTR_AXI_ERROR)
        C2S(DUMP_BUF_ATTR_RTT_LOG)
        C2S(DUMP_BUF_ATTR_SDTC_ETB_DUMP)
        C2S(DUMP_BUF_ATTR_PKTID_MAP_LOG)
        C2S(DUMP_BUF_ATTR_PKTID_UNMAP_LOG)
        C2S(DUMP_BUF_ATTR_EWP_HW_INIT_LOG)
        C2S(DUMP_BUF_ATTR_EWP_HW_MOD_DUMP)
        C2S(DUMP_BUF_ATTR_EWP_HW_REG_DUMP)
        C2S(DUMP_BUF_ATTR_WRAPPER_REG_DUMP)
        default:
            return "DUMP_BUF_ATTR_INVALID";
    }
}

/* Return index for matching buffer attribute */
static int logger_attr_buffer_lookup(u8 attr) {
    for (u8 i = 0; i < ARRAYSIZE(attr_lookup_tbl); i++) {
        if (attr == attr_lookup_tbl[i].buf_attr) {
            return i;
        }
    }
    ALOGE("Lookup for buf attr = %s failed\n",
    EWP_CmdAttrToString(attr));
    return -1;
}

/* Return index matching the length attribute */
static int logger_attr_lookup(u8 attr) {
    for (u8 i = 0; i < ARRAYSIZE(attr_lookup_tbl); i++) {
        if (attr == attr_lookup_tbl[i].attr_type) {
            return i;
        }
    }
    ALOGE("Lookup for len attr = %s failed\n",
        EWP_EventAttrToString(attr));
    return -1;
}
#endif /* RING_DUMP */

#define DBGRING_NAME_MAX 32  //Copy from legacy hal
typedef struct wifi_buf_ring_map_entry {
    uint32_t type;
    uint32_t ring_id;
    char ring_name[DBGRING_NAME_MAX];
} wifi_buf_ring_map_entry_t;

typedef struct {
    char hw_id[PROPERTY_VALUE_MAX];
    char sku[MAX_SKU_NAME_LEN];
} sku_info_t;

sku_info_t sku_table[] = {
    // BCM4389
    { {"G9S9B"}, {"MMW"} },
    { {"G8V0U"}, {"MMW"} },
    { {"GFQM1"}, {"MMW"} },
    { {"GB62Z"}, {"MMW"} },
    { {"GE2AE"}, {"MMW"} },
    { {"GQML3"}, {"MMW"} },
    { {"GB7N6"}, {"ROW"} },
    { {"GLU0G"}, {"ROW"} },
    { {"GNA8F"}, {"ROW"} },
    { {"GX7AS"}, {"ROW"} },
    { {"GP4BC"}, {"ROW"} },
    { {"GVU6C"}, {"ROW"} },
    { {"GR1YH"}, {"JPN"} },
    { {"GF5KQ"}, {"JPN"} },
    { {"GPQ72"}, {"JPN"} },
    { {"GB17L"}, {"JPN"} },
    { {"GFE4J"}, {"JPN"} },
    { {"G03Z5"}, {"JPN"} },
    // BCM4398
    { {"GKWS6"}, {"MMW"} },
    { {"G1MNW"}, {"MMW"} },
    { {"GPJ41"}, {"ROW"} },
    { {"GC3VE"}, {"ROW"} },
    { {"GE9DP"}, {"JPN"} },
    { {"GZPF0"}, {"JPN"} },
    { {"G1AZG"}, {"EU"} },
    { {"G9BQD"}, {"NA"} },
    // BCM4383
    { {"G8HHN"}, {"MMW"} },
    { {"G6GPR"}, {"ROW"} },
    { {"G576D"}, {"JPN"} },
    { {"GKV4X"}, {"NA"} }
};
///////////////////////////////////////////////////////////////////////////////
class DebugCommand : public WifiCommand
{
    char *mBuff;
    int *mBuffSize;
    u32 *mNumRings;
    wifi_ring_buffer_status *mStatus;
    u32 *mNumMaps;
    wifi_buf_ring_map_entry_t *mMaps;
    unsigned int *mSupport;
    u32 mVerboseLevel;
    u32 mFlags;
    u32 mMaxIntervalSec;
    u32 mMinDataSize;
    char *mRingName;
    GetCmdType mType;

public:

    // constructor for get version
    DebugCommand(wifi_interface_handle iface, char *buffer, int *buffer_size,
            GetCmdType cmdType)
        : WifiCommand("DebugCommand", iface, 0), mBuff(buffer), mBuffSize(buffer_size), mType
        (cmdType)
    {
        mNumRings =  NULL;
        mStatus = NULL;
        mSupport = NULL;
        mVerboseLevel = 0;
        mFlags = 0;
        mMaxIntervalSec = 0;
        mMinDataSize = 0;
        mRingName = NULL;
        memset(mBuff, 0, *mBuffSize);
        mNumMaps = NULL;
        mMaps = NULL;
    }

    // constructor for ring data
    DebugCommand(wifi_interface_handle iface, char *ring_name, GetCmdType cmdType)
        : WifiCommand("DebugCommand", iface, 0), mRingName(ring_name), mType(cmdType)
    {
        mBuff = NULL;
        mBuffSize = NULL;
        mNumRings =  NULL;
        mStatus = NULL;
        mSupport = NULL;
        mVerboseLevel = 0;
        mFlags = 0;
        mMaxIntervalSec = 0;
        mMinDataSize = 0;
        mNumMaps = NULL;
        mMaps = NULL;
    }

    // constructor for ring status
    DebugCommand(wifi_interface_handle iface, u32 *num_rings,
            wifi_ring_buffer_status *status, GetCmdType cmdType)
        : WifiCommand("DebugCommand", iface, 0), mNumRings(num_rings), mStatus(status), mType(cmdType)
    {
        mBuff = NULL;
        mBuffSize = NULL;
        mSupport = NULL;
        mVerboseLevel = 0;
        mFlags = 0;
        mMaxIntervalSec = 0;
        mMinDataSize = 0;
        mRingName = NULL;
        memset(mStatus, 0, sizeof(wifi_ring_buffer_status) * (*mNumRings));
        mNumMaps = NULL;
        mMaps = NULL;
    }

    // constructor for feature set
    DebugCommand(wifi_interface_handle iface, unsigned int *support, GetCmdType cmdType)
        : WifiCommand("DebugCommand", iface, 0), mSupport(support), mType(cmdType)
    {
        mBuff = NULL;
        mBuffSize = NULL;
        mNumRings =  NULL;
        mStatus = NULL;
        mVerboseLevel = 0;
        mFlags = 0;
        mMaxIntervalSec = 0;
        mMinDataSize = 0;
        mRingName = NULL;
        mMaps = NULL;
        mNumMaps = NULL;
    }

    // constructor for buf ring map
    DebugCommand(wifi_interface_handle iface, u32 *num_maps,
            wifi_buf_ring_map_entry_t *map, GetCmdType cmdType)
        : WifiCommand("DebugCommand", iface, 0), mNumMaps(num_maps), mMaps(map), mType(cmdType)
    {
        memset(mMaps, 0, sizeof(wifi_buf_ring_map_entry_t) * (*mNumMaps));
        mBuff = NULL;
        mBuffSize = NULL;
        mNumRings =  NULL;
        mStatus = NULL;
        mVerboseLevel = 0;
        mFlags = 0;
        mMaxIntervalSec = 0;
        mMinDataSize = 0;
        mRingName = NULL;
        mSupport = NULL;
    }

    // constructor for ring params
    DebugCommand(wifi_interface_handle iface, u32 verbose_level, u32 flags,
            u32 max_interval_sec, u32 min_data_size, char *ring_name, GetCmdType cmdType)
        : WifiCommand("DebugCommand", iface, 0), mVerboseLevel(verbose_level), mFlags(flags),
        mMaxIntervalSec(max_interval_sec), mMinDataSize(min_data_size),
        mRingName(ring_name), mType(cmdType)
    {
        mBuff = NULL;
        mBuffSize = NULL;
        mNumRings =  NULL;
        mStatus = NULL;
        mSupport = NULL;
        mMaps = NULL;
        mNumMaps = NULL;
    }

    int createRingRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, LOGGER_START_LOGGING);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to create start ring logger request; result = %d", result);
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);

        result = request.put_u32(LOGGER_ATTRIBUTE_LOG_LEVEL, mVerboseLevel);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to put log level; result = %d", result);
            return result;
        }
        result = request.put_u32(LOGGER_ATTRIBUTE_RING_FLAGS, mFlags);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to put ring flags; result = %d", result);
            return result;
        }
        result = request.put_u32(LOGGER_ATTRIBUTE_LOG_TIME_INTVAL, mMaxIntervalSec);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to put log time interval; result = %d", result);
            return result;
        }
        result = request.put_u32(LOGGER_ATTRIBUTE_LOG_MIN_DATA_SIZE, mMinDataSize);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to put min data size; result = %d", result);
            return result;
        }
        result = request.put_string(LOGGER_ATTRIBUTE_RING_NAME, mRingName);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to put ringbuffer name; result = %d", result);
            return result;
        }
        request.attr_end(data);

        return WIFI_SUCCESS;
    }

    int createRequest(WifiRequest &request) {
        int result;

        ALOGI("CreateRequest mType = %s", DebugDumpToString(mType));
        switch (mType) {
            case GET_FW_VER:
            {
                result = request.create(GOOGLE_OUI, LOGGER_GET_VER);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to create get fw version request; result = %d", result);
                    return result;
                }

                nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);

                // Driver expecting only attribute type, passing mbuff as data with
                // length 0 to avoid undefined state
                result = request.put(LOGGER_ATTRIBUTE_FW_VER, mBuff, 0);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to put get fw version request; result = %d", result);
                    return result;
                }
                request.attr_end(data);
                break;
            }

            case GET_DRV_VER:
            {
                result = request.create(GOOGLE_OUI, LOGGER_GET_VER);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to create get drv version request; result = %d", result);
                    return result;
                }

                nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);

                // Driver expecting only attribute type, passing mbuff as data with
                // length 0 to avoid undefined state
                result = request.put(LOGGER_ATTRIBUTE_DRIVER_VER, mBuff, 0);

                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to put get drv version request; result = %d", result);
                    return result;
                }
                request.attr_end(data);
                break;
            }

            case GET_RING_DATA:
            {
                result = request.create(GOOGLE_OUI, LOGGER_GET_RING_DATA);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to create get ring data request; result = %d", result);
                    return result;
                }

                nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
                result = request.put_string(LOGGER_ATTRIBUTE_RING_NAME, mRingName);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to put ring data request; result = %d", result);
                    return result;
                }
                ALOGI("Success to put ring data request for RingName %s", mRingName);
                request.attr_end(data);
                break;
            }

            case GET_RING_STATUS:
            {
                result = request.create(GOOGLE_OUI, LOGGER_GET_RING_STATUS);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to create get ring status request; result = %d", result);
                    return result;
                }
                break;
            }

            case GET_FEATURE:
            {
                result = request.create(GOOGLE_OUI, LOGGER_GET_FEATURE);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to create get feature request; result = %d", result);
                    return result;
                }
                break;
            }

            case GET_BUF_RING_MAP:
            {
                result = request.create(GOOGLE_OUI, LOGGER_GET_BUF_RING_MAP);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to create get ring status request; result = %d", result);
                    return result;
                }
                break;
            }

            case START_RING_LOG:
                result = createRingRequest(request);
                break;

            default:
                ALOGE("Unknown Debug command");
                result = WIFI_ERROR_UNKNOWN;
        }
        return result;
    }

    int start() {
        ALOGD("Start debug command: mType %s", DebugDumpToString(mType));
        WifiRequest request(familyId(), ifaceId());
        int result = createRequest(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to create debug request; result = %d", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to register debug response; result = %d", result);
        }
        return result;
    }

    virtual int handleResponse(WifiEvent& reply) {
        ALOGD("In DebugCommand::handleResponse, mType:%s\n", DebugDumpToString(mType));

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        switch (mType) {
            case GET_DRV_VER:
            case GET_FW_VER:
            {
                void *data = reply.get_vendor_data();
                int len = reply.get_vendor_data_len();

                ALOGD("len = %d, expected len = %d", len, *mBuffSize);
                memcpy(mBuff, data, min(len, *mBuffSize));
                if (*mBuffSize < len)
                    return NL_SKIP;
                *mBuffSize = len;
                break;
            }

            case START_RING_LOG:
            case GET_RING_DATA:
                break;

            case GET_RING_STATUS:
            {
                nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
                int len = reply.get_vendor_data_len();
                wifi_ring_buffer_status *status(mStatus);

                if (vendor_data == NULL || len == 0) {
                    ALOGE("No Debug data found");
                    return NL_SKIP;
                }

                nl_iterator it(vendor_data);
                if (it.get_type() == LOGGER_ATTRIBUTE_RING_NUM) {
                    unsigned int num_rings = it.get_u32();
                    if (*mNumRings < num_rings) {
                        ALOGE("Not enough status buffers provided, available: %d required: %d",
                                *mNumRings, num_rings);
                    } else {
                        *mNumRings = num_rings;
                    }
                } else {
                    ALOGE("Unknown attribute: %d expecting %d",
                            it.get_type(), LOGGER_ATTRIBUTE_RING_NUM);
                    return NL_SKIP;
                }

                it.next();
                for (unsigned int i = 0; it.has_next() && i < *mNumRings; it.next()) {
                    if (it.get_type() == LOGGER_ATTRIBUTE_RING_STATUS) {
                        const int status_size = sizeof(wifi_ring_buffer_status);
                        if (it.get_len() > status_size) {
                            ALOGE("ring status unexpected len = %d, dest len = %d",
                                it.get_len(), status_size);
                            return NL_SKIP;
                        } else {
                            memcpy(status, it.get_data(), status_size);
                            i++;
                            status++;
                        }
                    } else {
                        ALOGW("Ignoring invalid attribute type = %d, size = %d",
                                it.get_type(), it.get_len());
                    }
                }
                break;
            }

            case GET_FEATURE:
            {
                void *data = reply.get_vendor_data();
                int len = reply.get_vendor_data_len();

                ALOGD("len = %d, expected len = %d", len, static_cast<int>(sizeof(unsigned int)));
                memcpy(mSupport, data, sizeof(unsigned int));
                break;
            }

            case GET_BUF_RING_MAP:
            {
                nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
                int len = reply.get_vendor_data_len();
                wifi_buf_ring_map_entry_t *map(mMaps);

                if (vendor_data == NULL || len == 0) {
                    ALOGE("No Debug data found");
                    return NL_SKIP;
                }

                nl_iterator it(vendor_data);
                if (it.get_type() == LOGGER_ATTRIBUTE_BUF_RING_NUM) {
                    unsigned int num_maps = it.get_u32();
                    if (*mNumMaps < num_maps) {
                        ALOGE("Not enough status buffers provided, available: %d required: %d",
                            *mNumMaps, num_maps);
                    } else {
                        *mNumMaps = num_maps;
                    }
                } else {
                    ALOGE("Unknown attribute: %d expecting %d",
                        it.get_type(), LOGGER_ATTRIBUTE_BUF_RING_NUM);
                    return NL_SKIP;
                }

                it.next();
                for (unsigned int i = 0; it.has_next() && i < *mNumMaps; it.next()) {
                    if (it.get_type() == LOGGER_ATTRIBUTE_BUF_RING_MAP) {
                        const int entry_size = sizeof(wifi_buf_ring_map_entry_t);
                        if (it.get_len() > entry_size) {
                            ALOGE("GET_BUF_RING_MAP: unexpected len = %d, dest len = %d",
                                it.get_len(), entry_size);
                            return NL_SKIP;
                        } else {
                            memcpy(map, it.get_data(), entry_size);
                        }
                        i++;
                        map++;
                    } else {
                        ALOGW("Ignoring invalid attribute type = %d, size = %d",
                            it.get_type(), it.get_len());
                    }
                }
                break;
            }

            default:
                ALOGW("Unknown Debug command");
        }
        return NL_OK;
    }

    virtual int handleEvent(WifiEvent& event) {
        /* NO events! */
        return NL_SKIP;
    }
};

/* API to collect a firmware version string */
wifi_error wifi_get_firmware_version(wifi_interface_handle iface, char *buffer,
        int buffer_size)
{
    if (buffer && (buffer_size > 0)) {
        DebugCommand *cmd = new DebugCommand(iface, buffer, &buffer_size, GET_FW_VER);
        NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
        wifi_error result = (wifi_error)cmd->start();
        cmd->releaseRef();
        return result;
    } else {
        ALOGE("FW version buffer NULL");
        return  WIFI_ERROR_INVALID_ARGS;
    }
}

/* API to collect a driver version string */
wifi_error wifi_get_driver_version(wifi_interface_handle iface, char *buffer, int buffer_size)
{
    if (buffer && (buffer_size > 0)) {
        DebugCommand *cmd = new DebugCommand(iface, buffer, &buffer_size, GET_DRV_VER);
        NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
        wifi_error result = (wifi_error)cmd->start();
        cmd->releaseRef();
        return result;
    } else {
        ALOGE("Driver version buffer NULL");
        return  WIFI_ERROR_INVALID_ARGS;
    }
}

/* API to collect driver records */
wifi_error wifi_get_ring_data(wifi_interface_handle iface, char *ring_name)
{
    DebugCommand *cmd = new DebugCommand(iface, ring_name, GET_RING_DATA);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = (wifi_error)cmd->start();
    cmd->releaseRef();
    return result;
}

/* API to get the status of all ring buffers supported by driver */
wifi_error wifi_get_ring_buffers_status(wifi_interface_handle iface,
        u32 *num_rings, wifi_ring_buffer_status *status)
{
    if (status && num_rings) {
        DebugCommand *cmd = new DebugCommand(iface, num_rings, status, GET_RING_STATUS);
        NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
        wifi_error result = (wifi_error)cmd->start();
        cmd->releaseRef();
        return result;
    } else {
        ALOGE("Ring status buffer NULL");
        return  WIFI_ERROR_INVALID_ARGS;
    }
}

/* API to get supportable feature */
wifi_error wifi_get_logger_supported_feature_set(wifi_interface_handle iface,
        unsigned int *support)
{
    if (support) {
        DebugCommand *cmd = new DebugCommand(iface, support, GET_FEATURE);
        NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
        wifi_error result = (wifi_error)cmd->start();
        cmd->releaseRef();
        return result;
    } else {
        ALOGE("Get support buffer NULL");
        return  WIFI_ERROR_INVALID_ARGS;
    }
}

wifi_error wifi_start_logging(wifi_interface_handle iface, u32 verbose_level,
        u32 flags, u32 max_interval_sec, u32 min_data_size, char *ring_name)
{
    if (ring_name) {
        ALOGE("Ring name: level:%d sec:%d ring_name:%s",
                verbose_level, max_interval_sec, ring_name);
        DebugCommand *cmd = new DebugCommand(iface, verbose_level, flags, max_interval_sec,
                    min_data_size, ring_name, START_RING_LOG);
        NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
        wifi_error result = (wifi_error)cmd->start();
        cmd->releaseRef();
        return result;
    } else {
        ALOGE("Ring name NULL");
        return  WIFI_ERROR_INVALID_ARGS;
    }
}

typedef struct {
    u32 magic;
    int num_entries;
} __attribute__((packed)) wifi_ring_buffer_entry_pack;

#define WIFI_RING_BUFFER_PACK_MAGIC 0xDBAADBAA

///////////////////////////////////////////////////////////////////////////////
class SetLogHandler : public WifiCommand
{
    wifi_ring_buffer_data_handler mHandler;

public:
    SetLogHandler(wifi_interface_handle iface, int id, wifi_ring_buffer_data_handler handler)
        : WifiCommand("SetLogHandler", iface, id), mHandler(handler)
    { }

    int start() {
        ALOGV("Register loghandler");
        int result;
        uint32_t event_sock_pid = getpid() + (WIFI_HAL_EVENT_SOCK_PORT << 22);

        WifiRequest request(familyId(), ifaceId());

        /* set hal event socket port to driver */
        result = request.create(GOOGLE_OUI, LOGGER_SET_HAL_PID);
        if (result != WIFI_SUCCESS) {
            ALOGV("Failed to set Hal preInit; result = %d", result);
            return result;
        }
        registerVendorHandler(GOOGLE_OUI, GOOGLE_DEBUG_RING_EVENT);

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(SET_HAL_START_ATTRIBUTE_EVENT_SOCK_PID, event_sock_pid);
        if (result != WIFI_SUCCESS) {
            unregisterVendorHandler(GOOGLE_OUI, GOOGLE_DEBUG_RING_EVENT);
            ALOGV("Hal preInit Failed to put pic = %d", result);
            return result;
        }

        if (result != WIFI_SUCCESS) {
            unregisterVendorHandler(GOOGLE_OUI, GOOGLE_DEBUG_RING_EVENT);
            ALOGV("Hal preInit Failed to put pid= %d", result);
            return result;
        }

        request.attr_end(data);

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            unregisterVendorHandler(GOOGLE_OUI, GOOGLE_DEBUG_RING_EVENT);
            ALOGE("Failed to register set Hal preInit; result = %d", result);
            return result;
        }
        return result;
    }

    virtual int cancel() {
        /* Send a command to driver to stop generating logging events */
        ALOGV("Clear loghandler");

        /* unregister event handler */
        unregisterVendorHandler(GOOGLE_OUI, GOOGLE_DEBUG_RING_EVENT);
        wifi_unregister_cmd(wifiHandle(), id());

        WifiRequest request(familyId(), ifaceId());
        int result = request.create(GOOGLE_OUI, LOGGER_RESET_LOGGING);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to create reset request; result = %d", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("failed to request reset; result = %d", result);
            return result;
        }

        ALOGD("Success to clear loghandler");
        return WIFI_SUCCESS;
    }

    virtual int handleEvent(WifiEvent& event) {
        char *buffer = NULL;
        int buffer_size = 0;

        // ALOGD("In SetLogHandler::handleEvent");
        nlattr *vendor_data = event.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = event.get_vendor_data_len();
        int event_id = event.get_vendor_subcmd();
        // ALOGI("Got Logger event: %d", event_id);

        if (vendor_data == NULL || len == 0) {
            ALOGE("No Debug data found");
            return NL_SKIP;
        }

        if (event_id == GOOGLE_DEBUG_RING_EVENT) {
            wifi_ring_buffer_status status;
            memset(&status, 0, sizeof(status));

            for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
                if (it.get_type() == LOGGER_ATTRIBUTE_RING_STATUS) {
                    const int status_size = sizeof(wifi_ring_buffer_status);
                    if (it.get_len() > status_size) {
                        ALOGE("SetLogHandler: ring status unexpected len = %d, dest len = %d",
                           it.get_len(), status_size);
                        return NL_SKIP;
                    } else {
                        memcpy(&status, it.get_data(), status_size);
                    }
                } else if (it.get_type() == LOGGER_ATTRIBUTE_RING_DATA) {
                    buffer_size = it.get_len();
                    buffer = (char *)it.get_data();
                    ALOGV("SetLogHandler: ring data size = %d", buffer_size);
                } else {
                    ALOGW("Ignoring invalid attribute type = %d, size = %d",
                            it.get_type(), it.get_len());
                }
            }

            // ALOGI("Retrieved Debug data");
            if (mHandler.on_ring_buffer_data) {
                /* Skip msg header. Retrieved log */
                char *pBuff;
                int num_entries;
                int cur_off = 0;
                wifi_ring_buffer_entry_pack *pack_hdr =
                    (wifi_ring_buffer_entry_pack *)buffer;
                wifi_ring_buffer_entry *entry_hdr =
                    (wifi_ring_buffer_entry *)(buffer + sizeof(*pack_hdr));
                cur_off += sizeof(*pack_hdr);

                if (pack_hdr->magic != WIFI_RING_BUFFER_PACK_MAGIC) {
                    ALOGE("SetLogHandler: magic code is not matched "
                        "magic:%u ring_name:%s\n", pack_hdr->magic, status.name);
                    return NL_SKIP;
                }

                num_entries = pack_hdr->num_entries;

                while (num_entries > 0) {
                    /* Check for accesses that exceed the total buffer size */
                    if (cur_off + sizeof(*entry_hdr) + entry_hdr->entry_size > buffer_size) {
                        ALOGE("SetLogHandler: detected invalid access "
                            "num_entries:%d cur_num:%d buffer_size:%d cur_off:%d "
                            "hdrsize:%d entry_size:%d ring_name:%s\n",
                            pack_hdr->num_entries, num_entries, buffer_size, cur_off,
                            static_cast<int>(sizeof(*entry_hdr)), entry_hdr->entry_size, status.name);
                        return NL_SKIP;
                    }

                    /* Copy buffer without hdr to the ringbuffer in LegacyHAL */
                    pBuff = (char *)entry_hdr + sizeof(*entry_hdr);
                    (*mHandler.on_ring_buffer_data)((char *)status.name, pBuff,
                        entry_hdr->entry_size, &status);

                    cur_off += sizeof(*entry_hdr) + entry_hdr->entry_size;

                    /* jump to next entry_hdr */
                    entry_hdr = (wifi_ring_buffer_entry *)((char *)entry_hdr + sizeof(*entry_hdr) + entry_hdr->entry_size);

                    num_entries--;
                }

            }
        } else {
            ALOGE("Unknown Event");
            return NL_SKIP;
        }
        return NL_OK;
    }
};

wifi_error wifi_set_log_handler(wifi_request_id id, wifi_interface_handle iface,
        wifi_ring_buffer_data_handler handler)
{
    wifi_handle handle = getWifiHandle(iface);
    ALOGE("Loghandler start, handle = %p", handle);

    SetLogHandler *cmd = new SetLogHandler(iface, id, handler);
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

#ifdef RING_DUMP
    wifi_start_ring_dump(iface, handler);
#endif /* RING_DUMP */
    return result;
}

wifi_error wifi_reset_log_handler(wifi_request_id id, wifi_interface_handle iface)
{
    wifi_handle handle = getWifiHandle(iface);
    ALOGE("Loghandler reset, wifi_request_id = %d, handle = %p", id, handle);

#ifdef RING_DUMP
    wifi_stop_ring_dump(iface);
#endif /* RING_DUMP */

    if (id == -1) {
        wifi_ring_buffer_data_handler handler;
        memset(&handler, 0, sizeof(handler));

        SetLogHandler *cmd = new SetLogHandler(iface, id, handler);
        NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
        cmd->cancel();
        cmd->releaseRef();

        return WIFI_SUCCESS;
    }

    return wifi_get_cancel_cmd(id, iface);
}

///////////////////////////////////////////////////////////////////////////////
class SetAlertHandler : public WifiCommand
{
    wifi_alert_handler mHandler;
    int mBuffSize;
    char *mBuff;
    int mErrCode;

public:
    SetAlertHandler(wifi_interface_handle iface, int id, wifi_alert_handler handler)
        : WifiCommand("SetAlertHandler", iface, id), mHandler(handler), mBuffSize(0), mBuff(NULL),
            mErrCode(0)
    { }

    int start() {
        ALOGV("Start Alerting");
        registerVendorHandler(GOOGLE_OUI, GOOGLE_DEBUG_MEM_DUMP_EVENT);
        return WIFI_SUCCESS;
    }

    virtual int cancel() {
        ALOGV("Clear alerthandler");

        /* unregister alert handler */
        unregisterVendorHandler(GOOGLE_OUI, GOOGLE_DEBUG_MEM_DUMP_EVENT);
        wifi_unregister_cmd(wifiHandle(), id());
        ALOGD("Success to clear alerthandler");
        return WIFI_SUCCESS;
    }

    virtual int handleResponse(WifiEvent& reply) {
        ALOGD("In SetAlertHandler::handleResponse");

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = reply.get_vendor_data_len();

        ALOGD("len = %d", len);
        if (vendor_data == NULL || len == 0) {
            ALOGE("no vendor data in memory dump response; ignoring it");
            return NL_SKIP;
        }

        for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
            if (it.get_type() == LOGGER_ATTRIBUTE_FW_DUMP_DATA) {
                ALOGI("Initiating alert callback");
                if (mHandler.on_alert) {
                    (*mHandler.on_alert)(id(), mBuff, mBuffSize, mErrCode);
                }
                if (mBuff) {
                    free(mBuff);
                    mBuff = NULL;
                }
            }
        }
        return NL_OK;
    }

    virtual int handleEvent(WifiEvent& event) {
        char *buffer = NULL;
        int buffer_size = 0;
        bool is_err_alert = false;

        nlattr *vendor_data = event.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = event.get_vendor_data_len();
        int event_id = event.get_vendor_subcmd();
        ALOGI("Got event: %d", event_id);

        if (vendor_data == NULL || len == 0) {
            ALOGE("No Debug data found");
            return NL_SKIP;
        }

        if (event_id == GOOGLE_DEBUG_MEM_DUMP_EVENT) {
            for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
                if (it.get_type() == LOGGER_ATTRIBUTE_FW_DUMP_LEN) {
                    mBuffSize = it.get_u32();
                } else if (it.get_type() == LOGGER_ATTRIBUTE_RING_DATA) {
                    buffer_size = it.get_len();
                    buffer = (char *)it.get_data();
                } else if (it.get_type() == LOGGER_ATTRIBUTE_FW_ERR_CODE) {
                    /* Error code is for error alert event only */
                    mErrCode = it.get_u32();
                    is_err_alert = true;
                } else {
                    ALOGW("Ignoring invalid attribute type = %d, size = %d",
                            it.get_type(), it.get_len());
                }
            }

            if (is_err_alert) {
                mBuffSize = sizeof(mErrCode);
                if (mBuff) free(mBuff);
                mBuff = (char *)malloc(mBuffSize);
                if (!mBuff) {
                  ALOGE("Buffer allocation failed");
                  return NL_SKIP;
                }
                memcpy(mBuff, (char *)&mErrCode, mBuffSize);
                ALOGI("Initiating alert callback");
                if (mHandler.on_alert) {
                  (*mHandler.on_alert)(id(), mBuff, mBuffSize, mErrCode);
                }
                if (mBuff) {
                  free(mBuff);
                  mBuff = NULL;
                }
                mBuffSize = 0;
                return NL_OK;
            }

            if (mBuffSize) {
                ALOGD("dump size: %d meta data size: %d", mBuffSize, buffer_size);
                if (mBuff) free(mBuff);
                mBuff = (char *)malloc(mBuffSize + buffer_size);
                if (!mBuff) {
                    ALOGE("Buffer allocation failed");
                    return NL_SKIP;
                }
                memcpy(mBuff, buffer, buffer_size);

                WifiRequest request(familyId(), ifaceId());
                int result = request.create(GOOGLE_OUI, LOGGER_GET_MEM_DUMP);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to create get memory dump request; result = %d", result);
                    free(mBuff);
                    return NL_SKIP;
                }
                nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
                result = request.put_u32(LOGGER_ATTRIBUTE_FW_DUMP_LEN, mBuffSize);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to put get memory dump request; result = %d", result);
                    return result;
                }

                result = request.put_u64(LOGGER_ATTRIBUTE_FW_DUMP_DATA,
                         (uint64_t)(mBuff+buffer_size));
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to put get memory dump request; result = %d", result);
                    return result;
                }

                request.attr_end(data);
                mBuffSize += buffer_size;

                result = requestResponse(request);

                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to register get momory dump response; result = %d", result);
                }
            } else {
                ALOGE("dump event missing dump length attribute");
                return NL_SKIP;
            }
        }
        return NL_OK;
    }
};

class SetRestartHandler : public WifiCommand
{
    wifi_subsystem_restart_handler mHandler;
    char *mBuff;
public:
    SetRestartHandler(wifi_handle handle, wifi_request_id id, wifi_subsystem_restart_handler handler)
        : WifiCommand("SetRestartHandler", handle, id), mHandler(handler), mBuff(NULL)
    { }
    int start() {
        ALOGI("Start Restart Handler handler");
        registerVendorHandler(BRCM_OUI, BRCM_VENDOR_EVENT_HANGED);
        return WIFI_SUCCESS;
    }
    virtual int cancel() {
        ALOGI("Clear Restart Handler");

        /* unregister alert handler */
        unregisterVendorHandler(BRCM_OUI, BRCM_VENDOR_EVENT_HANGED);
        wifi_unregister_cmd(wifiHandle(), id());
        ALOGI("Success to clear restarthandler");
        return WIFI_SUCCESS;
    }

    virtual int handleResponse(WifiEvent& reply) {
        /* Nothing to do on response! */
        return NL_OK;
    }

    virtual int handleEvent(WifiEvent& event) {
        nlattr *vendor_data = event.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = event.get_vendor_data_len();
        int event_id = event.get_vendor_subcmd();
        ALOGI("Got event: %d", event_id);

        if (vendor_data == NULL || len == 0) {
            ALOGE("No Debug data found");
            return NL_SKIP;
        }
        if (event_id == BRCM_VENDOR_EVENT_HANGED) {
            for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
                if (it.get_type() == LOGGER_ATTRIBUTE_HANG_REASON) {
                    mBuff = (char *)it.get_data();
                } else {
                    ALOGI("Ignoring invalid attribute type = %d, size = %d",
                            it.get_type(), it.get_len());
                }
            }

            if (*mHandler.on_subsystem_restart) {
                (*mHandler.on_subsystem_restart)(mBuff);
                ALOGI("Hang event received. Trigger SSR handler:%p",
                    mHandler.on_subsystem_restart);
            } else {
                ALOGI("No Restart handler registered");
            }
        }
        return NL_OK;
    }
};

///////////////////////////////////////////////////////////////////////////////
class SubSystemRestart : public WifiCommand
{
    public:
    SubSystemRestart(wifi_interface_handle iface)
        : WifiCommand("SubSystemRestart", iface, 0)
    { }

    int createRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, WIFI_SUBCMD_TRIGGER_SSR);
        if (result < 0) {
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);

        request.attr_end(data);
        return WIFI_SUCCESS;
    }

    int create() {
        WifiRequest request(familyId(), ifaceId());

        int result = createRequest(request);
        if (result < 0) {
            ALOGE("Failed to create ssr request result = %d\n", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to register ssr response; result = %d\n", result);
        }
        return result;
    }

    protected:
    int handleResponse(WifiEvent& reply) {
        /* Nothing to do on response! */
        return NL_OK;
    }

    int handleEvent(WifiEvent& event) {
        /* NO events to handle here! */
        return NL_SKIP;
    }

};
///////////////////////////////////////////////////////////////////////////////
class HalInit : public WifiCommand
{
    int mErrCode;

    public:
    HalInit(wifi_interface_handle iface, int id)
        : WifiCommand("HalInit", iface, id), mErrCode(0)
    { }

    int start() {
        ALOGE("Start Set Hal");
        WifiRequest request(familyId(), ifaceId());

        int result = request.create(GOOGLE_OUI, LOGGER_SET_HAL_START);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to set hal start; result = %d", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to register set hal start response; result = %d", result);
        }
        return result;
    }


    virtual int cancel() {
        ALOGE("Cancel: Stop Hal");
        WifiRequest request(familyId(), ifaceId());

        int result = request.create(GOOGLE_OUI, LOGGER_HAL_STOP);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to stop hal ; result = %d", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to register set hal start response; result = %d", result);
        }
        wifi_unregister_cmd(wifiHandle(), id());
        ALOGE("Stop HAL Successfully Completed, mErrCode = %d\n", mErrCode);
        return result;
    }

    int preInit() {
        ALOGE("Hal preInit");
        WifiRequest request(familyId(), ifaceId());

        int result = request.create(GOOGLE_OUI, LOGGER_SET_HAL_START);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to set Hal preInit; result = %d", result);
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_string(SET_HAL_START_ATTRIBUTE_PRE_INIT, (char *)HAL_VERSION);
        if (result != WIFI_SUCCESS) {
            ALOGE("Hal preInit Failed to put data= %d", result);
            return result;
        }
        request.attr_end(data);

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to register set Hal preInit; result = %d", result);
        }
        return result;
    }

    virtual int handleResponse(WifiEvent& reply) {
        ALOGE("In SetHalStarted::handleResponse");

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }
        return NL_OK;
    }

    virtual int handleEvent(WifiEvent& event) {
        /* NO events! */
        return NL_SKIP;
    }
};

#ifdef RING_DUMP
///////////////////////////////////////////////////////////////////////////////
class RingDump : public WifiCommand
{
    int mLargestBuffSize;
    char *mBuff;
    int mErrCode;
    int mNumMaps;
    wifi_buf_ring_map_entry_t *mMap;
    int attr_type_len[DUMP_EVENT_ATTR_MAX];
    char *ring_name[DUMP_BUF_ATTR_MAX];
    wifi_ring_buffer_data_handler mHandle;

public:
    RingDump(wifi_interface_handle iface, int id, int num_maps, wifi_buf_ring_map_entry_t *map,
        wifi_ring_buffer_data_handler ring_handle)
        : WifiCommand("RingDump", iface, id), mLargestBuffSize(0), mBuff(NULL),
        mErrCode(0), mNumMaps(num_maps), mMap(map), mHandle(ring_handle)
    {
        memset(attr_type_len, 0, sizeof(attr_type_len));
        for (int i = 0; i < DUMP_BUF_ATTR_MAX; i++) {
            ring_name[i] = NULL;
        }
    }
    RingDump(wifi_interface_handle iface, int id)
        : WifiCommand("RingDump", iface, id), mLargestBuffSize(0), mBuff(NULL),
        mErrCode(0)
    {
        mMap = NULL;
        mNumMaps = 0;
        memset(&mHandle, 0, sizeof(wifi_ring_buffer_data_handler));
        for (int i = 0; i < DUMP_BUF_ATTR_MAX; i++) {
            ring_name[i] = NULL;
        }
    }

    int start() {
        DUMP_INFO(("Start Ring Dump Map_cnt %d, register the event\n", mNumMaps));
        registerVendorHandler(GOOGLE_OUI, GOOGLE_FILE_DUMP_EVENT);

        //Set ringname to buf hashmap
        for (int i = 0; i < mNumMaps; i++) {
            int type = mMap[i].type;
            ring_name[type] = (char *)malloc(DBGRING_NAME_MAX);
            memset(ring_name[type], 0, DBGRING_NAME_MAX);
            memcpy(ring_name[type], mMap[i].ring_name, strlen(mMap[i].ring_name));
            DUMP_DEBUG(("Set ringname Buf:%s Ringname:%s len:%lu",
                EWP_CmdAttrToString(type), ring_name[type], strlen(mMap[i].ring_name)));
        }
        return WIFI_SUCCESS;
    }

    virtual int freeup() {
        DUMP_DEBUG(("freeup:Enter\n"));
        if (mBuff) {
            free(mBuff);
            mBuff = NULL;
            DUMP_INFO(("freed allocated memory and the handler\n"));
        }
        return WIFI_SUCCESS;
    }

    virtual int cancel() {
        /* unregister file dump handler */
        unregisterVendorHandler(GOOGLE_OUI, GOOGLE_FILE_DUMP_EVENT);
        wifi_unregister_cmd(wifiHandle(), id());

        /* Free up the ring names allocated */
        for (u8 i = 0; i < DUMP_BUF_ATTR_MAX; i++) {
            if (ring_name[i]) {
                free(ring_name[i]);
                ring_name[i] = NULL;
            }
        }
        memset(&mHandle, 0, sizeof(wifi_ring_buffer_data_handler));

        DUMP_INFO(("Stop Ring Dump Successfully Completed, mErrCode = %d\n", mErrCode));
        return WIFI_SUCCESS;
    }

    virtual int handleResponse(WifiEvent& reply) {
        DUMP_DEBUG(("RingDump::handleResponse\n"));
        int buf_attr = DUMP_BUF_ATTR_INVALID;
        int len_attr = DUMP_LEN_ATTR_INVALID;
        int index = -1;

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = reply.get_vendor_data_len();

        if (vendor_data == NULL || len == 0) {
            ALOGE("no vendor data in memory dump response; ignoring it");
            return NL_SKIP;
        }

        for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
            buf_attr = it.get_type();
            switch (buf_attr) {
                case DUMP_BUF_ATTR_TIMESTAMP:
                case DUMP_BUF_ATTR_ECNTRS:
                case DUMP_BUF_ATTR_DHD_DUMP:
                case DUMP_BUF_ATTR_EXT_TRAP:
                case DUMP_BUF_ATTR_HEALTH_CHK:
                case DUMP_BUF_ATTR_COOKIE:
                case DUMP_BUF_ATTR_FLOWRING_DUMP:
                case DUMP_BUF_ATTR_STATUS_LOG:
                case DUMP_BUF_ATTR_RTT_LOG:
                case DUMP_BUF_ATTR_PKTID_MAP_LOG:
                case DUMP_BUF_ATTR_PKTID_UNMAP_LOG: {
                    if (it.get_u32()) {
                        ALOGE("Copying data to userspace failed for buf attr = %s, status = %d\n",
                            EWP_CmdAttrToString(buf_attr), it.get_u32());
                        return WIFI_ERROR_UNKNOWN;
                    }
                    index = logger_attr_buffer_lookup(buf_attr);
                    if (index == -1) {
                        ALOGE("Invalid index. buf attr = %s\n", EWP_CmdAttrToString(buf_attr));
                        return WIFI_ERROR_UNKNOWN;
                    }
                    len_attr = attr_lookup_tbl[index].attr_type;
                    if (len_attr == DUMP_EVENT_ATTR_MAX) {
                        ALOGE("Invalid len attr = %s\n", EWP_EventAttrToString(len_attr));
                        return WIFI_ERROR_UNKNOWN;
                    }
                    if (!mBuff || attr_type_len[len_attr] <= 0) {
                        ALOGE("buff is empty for attr = %s\n", EWP_CmdAttrToString(buf_attr));
                        return WIFI_ERROR_UNKNOWN;
                    }

                    if (!ring_name[buf_attr]) {
                        ALOGE("Not allocated buf attr = %s\n", EWP_CmdAttrToString(buf_attr));
                        return WIFI_ERROR_UNKNOWN;
                    }
                    DUMP_INFO(("RingDump:: buf_attr:%s size = %d ring_name:%s\n",
                        EWP_CmdAttrToString(buf_attr), attr_type_len[len_attr],
                        ring_name[buf_attr]));
                    if (mHandle.on_ring_buffer_data) {
                        /* on_ring_buffer_data callback requires status memory
                         * so should pass status memory */
                        wifi_ring_buffer_status status;
                        memset(&status, 0, sizeof(status));
                        /* Skip msg header. Retrieved log */
                        (*mHandle.on_ring_buffer_data)(ring_name[buf_attr], mBuff,
                            attr_type_len[len_attr], &status);
                    } else {
                        DUMP_INFO(("RingDump: Retrieved Invalid mHandle\n"));
                        break;
                    }
                    ALOGI("Notified RingDump data of buf attr = %s\n", EWP_CmdAttrToString(buf_attr));
                    if (mBuff) {
                        memset(mBuff, 0, mLargestBuffSize);
                    }
                    break;
                }
                default: {
                    DUMP_DEBUG(("Ignoring invalid attribute buf_attr = %d, size = %d",
                        buf_attr, it.get_len()));
                    break;
                }
            }
        }
        return NL_OK;
    }

    virtual int request_logger_dump(WifiRequest& request,
            buf_data_t *buf, int len_attr) {

        int result = 0;
        int buf_attr = DUMP_BUF_ATTR_INVALID;
        int index = -1;
        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);

        index = logger_attr_lookup(len_attr);
        if (index == -1) {
            ALOGE("Invalid index\n");
            return WIFI_ERROR_UNKNOWN;
        }
        buf_attr = attr_lookup_tbl[index].buf_attr;

        if (buf_attr != DUMP_BUF_ATTR_INVALID) {
            result = request.put(buf_attr, buf, sizeof(buf_data_t));
            if (result != WIFI_SUCCESS) {
                ALOGE("Failed to put get memory dump request; result = %d", result);
                return result;
            }
        } else {
            ALOGE("Invalid buf attr = %s, index = %d\n",
                EWP_CmdAttrToString(buf_attr), index);
            return WIFI_ERROR_UNKNOWN;
        }
        DUMP_INFO(("Trigger get ring dump for buf attr = %s\n",
                EWP_CmdAttrToString(buf_attr)));

        request.attr_end(data);
        return result;
    }

    virtual int handleEvent(WifiEvent& event) {
        mLargestBuffSize = 0;
        mBuff = NULL;
        memset(attr_type_len, 0, sizeof(attr_type_len));
        u8 i = 0;
        int result = 0;
        int mActualBuffSize = 0;
        int index = -1;

        nlattr *vendor_data = event.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = event.get_vendor_data_len();
        int event_id = event.get_vendor_subcmd();
        int req_attr_cnt = 0;
        int req_attr[DUMP_EVENT_ATTR_MAX];
        int buf_attr = DUMP_BUF_ATTR_INVALID;

        if (vendor_data == NULL || len == 0) {
            ALOGE("No Debug data found");
            return NL_SKIP;
        }
        DUMP_INFO(("Ring Dump handler. Got event: %d", event_id));

        buf_data_t buf;

        memset(&buf, 0, sizeof(buf_data_t));
        buf.ver = 0;
        buf.buf_threshold = 0;

        if (event_id == GOOGLE_FILE_DUMP_EVENT) {
            for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
                int attr = it.get_type();
                switch (attr) {
                    case DUMP_LEN_ATTR_TIMESTAMP:
                    case DUMP_LEN_ATTR_ECNTRS:
                    case DUMP_LEN_ATTR_DHD_DUMP:
                    case DUMP_LEN_ATTR_EXT_TRAP:
                    case DUMP_LEN_ATTR_HEALTH_CHK:
                    case DUMP_LEN_ATTR_COOKIE:
                    case DUMP_LEN_ATTR_FLOWRING_DUMP:
                    case DUMP_LEN_ATTR_STATUS_LOG:
                    case DUMP_LEN_ATTR_RTT_LOG:
                    case DUMP_LEN_ATTR_PKTID_MAP_LOG:
                    case DUMP_LEN_ATTR_PKTID_UNMAP_LOG: {
                        mActualBuffSize = it.get_u32();
                        DUMP_DEBUG(("len attr %s, len %d\n",
                            EWP_EventAttrToString(attr), mActualBuffSize));
                        if (mActualBuffSize > mLargestBuffSize) {
                            mLargestBuffSize = mActualBuffSize;
                        }
                        attr_type_len[attr] = mActualBuffSize;

                        /* Store the order in which attributes are received
                         * so that file dump can be done in the same order
                         */
                        req_attr[req_attr_cnt++] = attr;
                        break;
                    }
                    default: {
                        ALOGE("RingDump: Ignoring invalid attribute type = %d, size = %d",
                            attr, it.get_len());
                            break;
                        }
                }
            }
            /* Allocation for the largest buffer size to use it recursively for other buf attr. */
            if (mLargestBuffSize) {
                DUMP_INFO(("Max dump size: %d", mLargestBuffSize));
                mBuff = (char *)malloc(mLargestBuffSize);
                if (!mBuff) {
                    ALOGE("Buffer allocation failed");
                    return NL_SKIP;
                }
                memset(mBuff, 0, mLargestBuffSize);
            }

            WifiRequest request(familyId(), ifaceId());
            result = request.create(GOOGLE_OUI, LOGGER_DEBUG_GET_DUMP);
            if (result != WIFI_SUCCESS) {
                ALOGE("Failed to create get memory dump request; result = %d", result);
                freeup();
                goto exit;
            }

            /* Requesting logger dump for each received attr */
            for (i = 0; i < req_attr_cnt; i++) {
                int attr = req_attr[i];

                if (attr_type_len[attr] == 0) {
                    continue;
                }

                index = logger_attr_lookup(attr);
                if (index == -1) {
                    ALOGE("Invalid index\n");
                    return WIFI_ERROR_UNKNOWN;
                }

                buf_attr = attr_lookup_tbl[index].buf_attr;
                if (!ring_name[buf_attr]) {
                    ALOGE("Failed to find ringname index:%d buf_attr:%d", index, buf_attr);
                    continue;
                }

                buf.len = attr_type_len[attr];
                buf.data_buf[0] = mBuff;
                DUMP_DEBUG(("buf len = %d, buf ptr= %p for attr = %s\n",
                    buf.len, buf.data_buf[0], EWP_EventAttrToString(attr)));
                result = request_logger_dump(request, &buf, attr);
                if (result != WIFI_SUCCESS) {
                    /* Proceeding further for other attributes */
                    ALOGE("Failed to request the ring dump for attr = %s; result = %d",
                        EWP_EventAttrToString(attr), result);
                    continue;
                }
                result = requestDebugResponse(request);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to register get ring dump response for attr = %s; result = %d",
                        EWP_EventAttrToString(attr), result);
                        /* Proceeding further for other attributes */
                        continue;
                }
                ALOGI("Success to send up the ring dump response for attr = %s; result = %d",
                    EWP_EventAttrToString(attr), result);
            }

            ALOGI("RingDump response is sent, proceed to done indication");
            WifiRequest request2(familyId(), ifaceId());
            result = request2.create(GOOGLE_OUI, LOGGER_FILE_DUMP_DONE_IND);
            if (result != WIFI_SUCCESS) {
                ALOGE("Failed to indicate FILE_DUMP_DONE_IND; result = %d", result);
                freeup();
                goto exit;
            }
            result = requestResponse(request2);
            if (result != WIFI_SUCCESS) {
                ALOGE("Failed to register file done ind response; result = %d", result);
            }
            freeup();
        } else {
            ALOGE("dump event missing dump length attribute");
            return NL_SKIP;
        }
    exit:
        if (result != WIFI_SUCCESS) {
            return NL_SKIP;
        }
        return NL_OK;
    }
};
///////////////////////////////////////////////////////////////////////////////
#endif /* RING_DUMP */

wifi_error wifi_start_hal(wifi_interface_handle iface)
{
    wifi_handle handle = getWifiHandle(iface);
    ALOGV("HAL INIT start, handle = %p", handle);

    HalInit *cmd = new HalInit(iface, HAL_START_REQUEST_ID);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = wifi_register_cmd(handle, HAL_START_REQUEST_ID, cmd);
    if (result != WIFI_SUCCESS) {
        cmd->releaseRef();
        return result;
    }
    result = (wifi_error)cmd->start();
    if (result != WIFI_SUCCESS) {
        wifi_unregister_cmd(handle, HAL_START_REQUEST_ID);
        cmd->releaseRef();
        return result;
    }
    return result;
}

wifi_error wifi_hal_preInit(wifi_interface_handle iface)
{
    wifi_handle handle = getWifiHandle(iface);
    ALOGV("wifi_hal_preInit, handle = %p", handle);

    HalInit *cmd = new HalInit(iface, HAL_START_REQUEST_ID);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = wifi_register_cmd(handle, HAL_START_REQUEST_ID, cmd);
    if (result != WIFI_SUCCESS) {
        cmd->releaseRef();
        return result;
    }
    result = (wifi_error)cmd->preInit();
    if (result != WIFI_SUCCESS) {
        wifi_unregister_cmd(handle, HAL_START_REQUEST_ID);
        cmd->releaseRef();
        return result;
    }
    return result;
}

#ifdef RING_DUMP
wifi_error wifi_start_ring_dump(wifi_interface_handle iface,
    wifi_ring_buffer_data_handler ring_handle)
{
    wifi_handle handle = getWifiHandle(iface);
    DUMP_INFO(("start ring dump, handle = %p, ring_handle = %p", handle, ring_handle.on_ring_buffer_data));
    wifi_buf_ring_map_entry_t map[DUMP_BUF_ATTR_MAX];
    unsigned int num_maps = DUMP_BUF_ATTR_MAX;
    wifi_error result;

    /* Get mapping table from driver */
    DebugCommand *debug_cmd = new DebugCommand(iface, &num_maps, map, GET_BUF_RING_MAP);
    NULL_CHECK_RETURN(debug_cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    result = (wifi_error)debug_cmd->start();
    debug_cmd->releaseRef();

    /* Set ringname to corresponding buf attr */
    DUMP_INFO(("Cached ring_handle: %p for RingDump", ring_handle.on_ring_buffer_data));
    RingDump *cmd = new RingDump(iface, FILE_DUMP_REQUEST_ID, num_maps, map, ring_handle);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    result = wifi_register_cmd(handle, FILE_DUMP_REQUEST_ID, cmd);
    if (result != WIFI_SUCCESS) {
        cmd->releaseRef();
        return result;
    }

    result = (wifi_error)cmd->start();
    if (result != WIFI_SUCCESS) {
        wifi_unregister_cmd(handle, FILE_DUMP_REQUEST_ID);
        cmd->releaseRef();
        return result;
    }
    return result;
}

wifi_error wifi_stop_ring_dump(wifi_interface_handle iface)
{
    RingDump *cmd = new RingDump(iface, FILE_DUMP_REQUEST_ID);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    DUMP_INFO(("stop ring dump"));
    cmd->cancel();
    cmd->releaseRef();
    return WIFI_SUCCESS;
}
#endif /* RING_DUMP */

wifi_error wifi_stop_hal(wifi_interface_handle iface)
{
    HalInit *cmd = new HalInit(iface, HAL_START_REQUEST_ID);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    cmd->cancel();
    cmd->releaseRef();
    return WIFI_SUCCESS;
}


wifi_error wifi_set_subsystem_restart_handler(wifi_handle handle,
                                              wifi_subsystem_restart_handler handler)
{
    hal_info *info = NULL;

    info = (hal_info *)handle;
    if (info == NULL) {
        ALOGE("Could not find hal info\n");
        return WIFI_ERROR_UNKNOWN;
    }

    SetRestartHandler *cmd = new SetRestartHandler(handle, HAL_RESTART_ID, handler);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = wifi_register_cmd(handle, HAL_RESTART_ID, cmd);
    if (result != WIFI_SUCCESS) {
        cmd->releaseRef();
        return result;
    }

    result = (wifi_error)cmd->start();
    if (result != WIFI_SUCCESS) {
        wifi_unregister_cmd(handle, HAL_RESTART_ID);
        cmd->releaseRef();
        return result;
    }

    /* Cache the handler to use it for trigger subsystem restart */
    ALOGI("Register SSR handler");
    info->restart_handler = handler;
    return result;
}

wifi_error wifi_trigger_subsystem_restart(wifi_handle handle)
{
    wifi_error result = WIFI_SUCCESS;
    hal_info *info = NULL;
    char error_str[20];
    SubSystemRestart *cmd = NULL;
    wifi_interface_handle *ifaceHandles = NULL;
    wifi_interface_handle wlan0Handle;
    int numIfaceHandles = 0;

    info = (hal_info *)handle;
    if (handle == NULL || info == NULL) {
        ALOGE("Could not find hal info\n");
        result = WIFI_ERROR_UNKNOWN;
        goto exit;
    }

    ALOGI("Trigger subsystem restart\n");

    wlan0Handle = wifi_get_wlan_interface((wifi_handle)handle, ifaceHandles, numIfaceHandles);

    cmd = new SubSystemRestart(wlan0Handle);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);

    result = (wifi_error)cmd->create();
    if (result != WIFI_SUCCESS) {
        cmd->releaseRef();
        strncpy(error_str, "WIFI_ERROR_UNKNOWN", sizeof(error_str));
        ALOGE("Failed to create SSR");
        goto exit;
    }

    strncpy(error_str, "WIFI_SUCCESS", sizeof(error_str));

exit:
    if (info->restart_handler.on_subsystem_restart) {
        ALOGI("Trigger ssr handler registered handler:%p",
            info->restart_handler.on_subsystem_restart);
        (info->restart_handler.on_subsystem_restart)(error_str);
    } else {
        ALOGI("No trigger ssr handler registered");
    }

    return result;
}

wifi_error wifi_set_alert_handler(wifi_request_id id, wifi_interface_handle iface,
        wifi_alert_handler handler)
{
    wifi_handle handle = getWifiHandle(iface);
    ALOGV("Alerthandler start, handle = %p", handle);

    SetAlertHandler *cmd = new SetAlertHandler(iface, id, handler);
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

wifi_error wifi_reset_alert_handler(wifi_request_id id, wifi_interface_handle iface)
{
    wifi_handle handle = getWifiHandle(iface);
    ALOGV("Alerthandler reset, wifi_request_id = %d, handle = %p", id, handle);

    if (id == -1) {
        wifi_alert_handler handler;
        memset(&handler, 0, sizeof(handler));

        SetAlertHandler *cmd = new SetAlertHandler(iface, id, handler);
        NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
        cmd->cancel();
        cmd->releaseRef();
        return WIFI_SUCCESS;
    }

    return wifi_get_cancel_cmd(id, iface);
}

///////////////////////////////////////////////////////////////////////////////
class MemoryDumpCommand: public WifiCommand
{
    wifi_firmware_memory_dump_handler mHandler;
    int mBuffSize;
    char *mBuff;

public:
    MemoryDumpCommand(wifi_interface_handle iface, wifi_firmware_memory_dump_handler handler)
        : WifiCommand("MemoryDumpCommand", iface, 0), mHandler(handler), mBuffSize(0), mBuff(NULL)
    { }

    int start() {
        ALOGD("Start memory dump command");
        WifiRequest request(familyId(), ifaceId());

        int result = request.create(GOOGLE_OUI, LOGGER_TRIGGER_MEM_DUMP);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to create trigger fw memory dump request; result = %d", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to register trigger memory dump response; result = %d", result);
        }
        return result;
    }

    virtual int handleResponse(WifiEvent& reply) {
        ALOGD("In MemoryDumpCommand::handleResponse");

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = reply.get_vendor_data_len();

        ALOGD("len = %d", len);
        if (vendor_data == NULL || len == 0) {
            ALOGE("no vendor data in memory dump response; ignoring it");
            return NL_SKIP;
        }

        for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
            if (it.get_type() == LOGGER_ATTRIBUTE_FW_DUMP_LEN) {
                mBuffSize = it.get_u32();

                if (mBuff)
                    free(mBuff);
                mBuff = (char *)malloc(mBuffSize);
                if (!mBuff) {
                    ALOGE("Buffer allocation failed");
                    return NL_SKIP;
                }
                WifiRequest request(familyId(), ifaceId());
                int result = request.create(GOOGLE_OUI, LOGGER_GET_MEM_DUMP);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to create get memory dump request; result = %d", result);
                    free(mBuff);
                    return NL_SKIP;
                }

                nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
                result = request.put_u32(LOGGER_ATTRIBUTE_FW_DUMP_LEN, mBuffSize);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to put get memory dump request; result = %d", result);
                    return result;
                }

                result = request.put_u64(LOGGER_ATTRIBUTE_FW_DUMP_DATA, (uint64_t)mBuff);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to put get memory dump request; result = %d", result);
                    return result;
                }
                request.attr_end(data);

                result = requestResponse(request);
                if (result != WIFI_SUCCESS) {
                    ALOGE("Failed to register get momory dump response; result = %d", result);
                }
            } else if (it.get_type() == LOGGER_ATTRIBUTE_FW_DUMP_DATA) {
                ALOGI("Initiating memory dump callback");
                if (mHandler.on_firmware_memory_dump) {
                    (*mHandler.on_firmware_memory_dump)(mBuff, mBuffSize);
                }
                if (mBuff) {
                    free(mBuff);
                    mBuff = NULL;
                }
            } else {
                ALOGW("Ignoring invalid attribute type = %d, size = %d",
                        it.get_type(), it.get_len());
            }
        }
        return NL_OK;
    }

    virtual int handleEvent(WifiEvent& event) {
        /* NO events! */
        return NL_SKIP;
    }
};

/* API to collect a firmware memory dump for a given iface */
wifi_error wifi_get_firmware_memory_dump( wifi_interface_handle iface,
        wifi_firmware_memory_dump_handler handler)
{
    MemoryDumpCommand *cmd = new MemoryDumpCommand(iface, handler);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = (wifi_error)cmd->start();
    cmd->releaseRef();
    return result;
}

class PacketFateCommand: public WifiCommand
{
    void *mReportBufs;
    size_t mNoReqFates;
    size_t *mNoProvidedFates;
    PktFateReqType mReqType;

public:
    PacketFateCommand(wifi_interface_handle handle)
        : WifiCommand("PacketFateCommand", handle, 0), mReqType(PACKET_MONITOR_START)
    {
        mReportBufs = NULL;
        mNoReqFates = 0;
        mNoProvidedFates = NULL;
    }

    PacketFateCommand(wifi_interface_handle handle, wifi_tx_report *tx_report_bufs,
            size_t n_requested_fates, size_t *n_provided_fates)
        : WifiCommand("PacketFateCommand", handle, 0), mReportBufs(tx_report_bufs),
                  mNoReqFates(n_requested_fates), mNoProvidedFates(n_provided_fates),
                  mReqType(TX_PACKET_FATE)
    { }

    PacketFateCommand(wifi_interface_handle handle, wifi_rx_report *rx_report_bufs,
            size_t n_requested_fates, size_t *n_provided_fates)
        : WifiCommand("PacketFateCommand", handle, 0), mReportBufs(rx_report_bufs),
                  mNoReqFates(n_requested_fates), mNoProvidedFates(n_provided_fates),
                  mReqType(RX_PACKET_FATE)
    { }

    int createRequest(WifiRequest& request) {
        if (mReqType == TX_PACKET_FATE) {
            ALOGD("%s Get Tx packet fate request\n", __FUNCTION__);
            return createTxPktFateRequest(request);
        } else if (mReqType == RX_PACKET_FATE) {
            ALOGD("%s Get Rx packet fate request\n", __FUNCTION__);
            return createRxPktFateRequest(request);
        } else if (mReqType == PACKET_MONITOR_START) {
            ALOGD("%s Monitor packet fate request\n", __FUNCTION__);
            return createMonitorPktFateRequest(request);
        } else {
            ALOGE("%s Unknown packet fate request\n", __FUNCTION__);
            return WIFI_ERROR_NOT_SUPPORTED;
        }
        return WIFI_SUCCESS;
    }

    int createMonitorPktFateRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, LOGGER_START_PKT_FATE_MONITORING);
        if (result < 0) {
            ALOGE("Failed to create monitorPktFate result:%d\n", result);
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        request.attr_end(data);
        return result;
    }

    int createTxPktFateRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, LOGGER_GET_TX_PKT_FATES);
        if (result < 0) {
            ALOGE("Failed to create TxPktFate result:%d\n", result);
            return result;
        }

        memset(mReportBufs, 0, (mNoReqFates * sizeof(wifi_tx_report)));
        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(LOGGER_ATTRIBUTE_PKT_FATE_NUM, mNoReqFates);
        if (result < 0) {
            ALOGE("Failed to set TxPktFate num result:%d mNoReqFates:%zu\n", result, mNoReqFates);
            return result;
        }
        result = request.put_u64(LOGGER_ATTRIBUTE_PKT_FATE_DATA, (uint64_t)mReportBufs);
        if (result < 0) {
            ALOGE("Failed to set TxPktFate buf result:%d\n", result);
            return result;
        }
        request.attr_end(data);
        return result;
    }

    int createRxPktFateRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, LOGGER_GET_RX_PKT_FATES);
        if (result < 0) {
            ALOGE("Failed to create RxPktFate result:%d\n", result);
            return result;
        }

        memset(mReportBufs, 0, (mNoReqFates * sizeof(wifi_rx_report)));
        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);
        result = request.put_u32(LOGGER_ATTRIBUTE_PKT_FATE_NUM, mNoReqFates);
        if (result < 0) {
            ALOGE("Failed to set RxPktFate num result:%d mNoReqFates:%zu\n", result, mNoReqFates);
            return result;
        }
        result = request.put_u64(LOGGER_ATTRIBUTE_PKT_FATE_DATA, (uint64_t)mReportBufs);
        if (result < 0) {
            ALOGE("Failed to set RxPktFate buf result:%d\n", result);
            return result;
        }
        request.attr_end(data);
        return result;
    }

    int start() {
        ALOGD("Start get packet fate command\n");
        WifiRequest request(familyId(), ifaceId());

        int result = createRequest(request);
        if (result < 0) {
            ALOGE("Failed to create get pkt fate request; result = %d\n", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to register get pkt fate response; result = %d\n", result);
        }
        return result;
    }

    int handleResponse(WifiEvent& reply) {
        ALOGD("In GetPktFateCommand::handleResponse\n");

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGI("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        int id = reply.get_vendor_id();
        int subcmd = reply.get_vendor_subcmd();
        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = reply.get_vendor_data_len();

        ALOGI("Id = %0x, subcmd = %d, len = %d", id, subcmd, len);

        if (mReqType == TX_PACKET_FATE) {
            ALOGI("Response recieved for get TX pkt fate command\n");
        } else if (mReqType == RX_PACKET_FATE) {
            ALOGI("Response recieved for get RX pkt fate command\n");
        } else if (mReqType == PACKET_MONITOR_START) {
            ALOGI("Response recieved for monitor pkt fate command\n");
            return NL_OK;
        } else {
            ALOGE("Response recieved for unknown pkt fate command\n");
            return NL_SKIP;
        }

        if (vendor_data == NULL || len == 0) {
            ALOGE("no vendor data in GetPktFateCommand response; ignoring it\n");
            return NL_SKIP;
        }

        for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
            if (it.get_type() == LOGGER_ATTRIBUTE_PKT_FATE_NUM) {
                *mNoProvidedFates = it.get_u32();
                ALOGI("No: of pkt fates provided is %zu\n", *mNoProvidedFates);
            } else {
                ALOGE("Ignoring invalid attribute type = %d, size = %d\n",
                        it.get_type(), it.get_len());
            }
        }

        return NL_OK;
    }

    int handleEvent(WifiEvent& event) {
        /* NO events to handle here! */
        return NL_SKIP;
    }
};

class GetWakeReasonCountCommand : public WifiCommand {
    WLAN_DRIVER_WAKE_REASON_CNT *mWakeReasonCnt;
    void *mCmdEventWakeCount;
    public:
    GetWakeReasonCountCommand(wifi_interface_handle handle,
        WLAN_DRIVER_WAKE_REASON_CNT *wlanDriverWakeReasonCount) :
        WifiCommand("GetWakeReasonCountCommand", handle, 0),
        mWakeReasonCnt(wlanDriverWakeReasonCount)
    {
        mCmdEventWakeCount = mWakeReasonCnt->cmd_event_wake_cnt;
    }

    int createRequest(WifiRequest& request) {
        int result = request.create(GOOGLE_OUI, LOGGER_GET_WAKE_REASON_STATS);
        if (result < 0) {
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);

        request.attr_end(data);
        return WIFI_SUCCESS;
    }

    int start() {
        ALOGD("Start get wake stats command\n");
        WifiRequest request(familyId(), ifaceId());

        int result = createRequest(request);
        if (result < 0) {
            ALOGE("Failed to create request result = %d\n", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to register wake stats  response; result = %d\n", result);
        }
        return result;
    }

    protected:
    int handleResponse(WifiEvent& reply) {
        ALOGE("In GetWakeReasonCountCommand::handleResponse");

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
            ALOGE("no vendor data in GetGetWakeReasonCountCommand response; ignoring it");
            return NL_SKIP;
        }

        for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
            switch (it.get_type()) {
                case WAKE_STAT_ATTRIBUTE_TOTAL_DRIVER_FW:
                    mWakeReasonCnt->total_driver_fw_local_wake =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_TOTAL:
                    mWakeReasonCnt->total_cmd_event_wake =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_CMD_COUNT_USED:
                    mWakeReasonCnt->cmd_event_wake_cnt_used =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_WAKE:
                    memcpy(mCmdEventWakeCount, it.get_data(),
                            (mWakeReasonCnt->cmd_event_wake_cnt_used * sizeof(int)));
                    break;
                case WAKE_STAT_ATTRIBUTE_TOTAL_RX_DATA_WAKE:
                    mWakeReasonCnt->total_rx_data_wake =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_RX_UNICAST_COUNT:
                    mWakeReasonCnt->rx_wake_details.rx_unicast_cnt =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_RX_MULTICAST_COUNT:
                    mWakeReasonCnt->rx_wake_details.rx_multicast_cnt =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_RX_BROADCAST_COUNT:
                    mWakeReasonCnt->rx_wake_details.rx_broadcast_cnt =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_RX_ICMP_PKT:
                    mWakeReasonCnt->rx_wake_pkt_classification_info.icmp_pkt =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_RX_ICMP6_PKT:
                    mWakeReasonCnt->rx_wake_pkt_classification_info.icmp6_pkt =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_RX_ICMP6_RA:
                    mWakeReasonCnt->rx_wake_pkt_classification_info.icmp6_ra =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_RX_ICMP6_NA:
                    mWakeReasonCnt->rx_wake_pkt_classification_info.icmp6_na =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_RX_ICMP6_NS:
                    mWakeReasonCnt->rx_wake_pkt_classification_info.icmp6_ns =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_IPV4_RX_MULTICAST_ADD_CNT:
                    mWakeReasonCnt->rx_multicast_wake_pkt_info.ipv4_rx_multicast_addr_cnt =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_IPV6_RX_MULTICAST_ADD_CNT:
                    mWakeReasonCnt->rx_multicast_wake_pkt_info.ipv6_rx_multicast_addr_cnt =
                        it.get_u32();
                    break;
                case WAKE_STAT_ATTRIBUTE_OTHER_RX_MULTICAST_ADD_CNT:
                    mWakeReasonCnt->rx_multicast_wake_pkt_info.other_rx_multicast_addr_cnt =
                        it.get_u32();
                    break;
                default:
                    break;
            }

        }
        return NL_OK;
    }
};

wifi_error wifi_start_pkt_fate_monitoring(wifi_interface_handle handle)
{
    PacketFateCommand *cmd = new PacketFateCommand(handle);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = (wifi_error)cmd->start();
    cmd->releaseRef();
    return result;
}

wifi_error wifi_get_tx_pkt_fates(wifi_interface_handle handle,
        wifi_tx_report *tx_report_bufs, size_t n_requested_fates,
        size_t *n_provided_fates)
{
    PacketFateCommand *cmd = new PacketFateCommand(handle, tx_report_bufs,
                n_requested_fates, n_provided_fates);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = (wifi_error)cmd->start();
    cmd->releaseRef();
    return result;
}

wifi_error wifi_get_rx_pkt_fates(wifi_interface_handle handle,
        wifi_rx_report *rx_report_bufs, size_t n_requested_fates,
        size_t *n_provided_fates)
{
    PacketFateCommand *cmd = new PacketFateCommand(handle, rx_report_bufs,
                n_requested_fates, n_provided_fates);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);
    wifi_error result = (wifi_error)cmd->start();
    cmd->releaseRef();
    return result;
}

wifi_error wifi_get_wake_reason_stats(wifi_interface_handle handle,
        WLAN_DRIVER_WAKE_REASON_CNT *wifi_wake_reason_cnt)
{
    GetWakeReasonCountCommand *cmd =
        new GetWakeReasonCountCommand(handle, wifi_wake_reason_cnt);
    wifi_error result = (wifi_error)cmd->start();
    cmd->releaseRef();
    return result;
}

///////////////////////////////////////////////////////////////////////////////
class OtaUpdateCommand : public WifiCommand
{
    int mErrCode;

    public:
    OtaUpdateCommand(wifi_interface_handle iface)
        : WifiCommand("OtaUpdateCommand", iface, 0), mErrCode(0)
    { }

    int start() {
        ALOGE("Start OtaUpdateCommand");
        WifiRequest request(familyId(), ifaceId());

        int result = request.create(GOOGLE_OUI, WIFI_SUBCMD_GET_OTA_CURRUNT_INFO);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to set hal start; result = %d", result);
            return result;
        }

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to register set hal start response; result = %d", result);
        }
        return result;
    }

    int otaDownload(ota_info_buf_t* buf, uint32_t ota_version) {
        u32 force_reg_on = false;
        WifiRequest request(familyId(), ifaceId());
        int result = request.create(GOOGLE_OUI, WIFI_SUBCMD_OTA_UPDATE);

        ALOGE("Download the OTA configuration");
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to set Hal preInit; result = %d", result);
            return result;
        }

        nlattr *data = request.attr_start(NL80211_ATTR_VENDOR_DATA);

        result = request.put_u32(OTA_DOWNLOAD_CLM_LENGTH_ATTR, buf->ota_clm_len);
        if (result != WIFI_SUCCESS) {
            ALOGE("otaDownload Failed to put data= %d", result);
            return result;
        }

        result = request.put(OTA_DOWNLOAD_CLM_ATTR, buf->ota_clm_buf, sizeof(*buf->ota_clm_buf));
        if (result != WIFI_SUCCESS) {
            ALOGE("otaDownload Failed to put data= %d", result);
            return result;
        }

        result = request.put_u32(OTA_DOWNLOAD_NVRAM_LENGTH_ATTR, buf->ota_nvram_len);
        if (result != WIFI_SUCCESS) {
            ALOGE("otaDownload Failed to put data= %d", result);
            return result;
        }

        result = request.put(OTA_DOWNLOAD_NVRAM_ATTR,
                buf->ota_nvram_buf, sizeof(*buf->ota_nvram_buf));
        if (result != WIFI_SUCCESS) {
            ALOGE("otaDownload Failed to put data= %d", result);
            return result;
        }

        if (applied_ota_version != ota_version) {
            force_reg_on = true;
            applied_ota_version = ota_version;
        }
        result = request.put_u32(OTA_SET_FORCE_REG_ON, force_reg_on);
        if (result != WIFI_SUCCESS) {
            ALOGE("otaDownload Failed to put data= %d", result);
            return result;
        }

        result = request.put_u32(OTA_DOWNLOAD_TXCAP_BLOB_LENGTH_ATTR, buf->ota_txcap_len);
        if (result != WIFI_SUCCESS) {
            ALOGE("otaDownload Failed to put data= %d", result);
            return result;
        }

        result = request.put(OTA_DOWNLOAD_TXCAP_BLOB_ATTR,
            buf->ota_txcap_buf, sizeof(*buf->ota_txcap_buf));
        if (result != WIFI_SUCCESS) {
            ALOGE("otaDownload Failed to put data= %d", result);
            return result;
        }

        request.attr_end(data);

        result = requestResponse(request);
        if (result != WIFI_SUCCESS) {
            ALOGE("Failed to register set otaDownload; result = %d, %d\n", result, mErrCode);
        }

        return result;
    }

    virtual int handleResponse(WifiEvent& reply) {
        ALOGD("In OtaUpdateCommand::handleResponse");

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
            ALOGE("no vendor data in GetPktFateCommand response; ignoring it\n");
            return NL_SKIP;
        }

        for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
            switch (it.get_type()) {
                case OTA_CUR_NVRAM_EXT_ATTR:
                    strncpy(ota_nvram_ext, (char*)it.get_string(), it.get_len());
                    ALOGI("Current Nvram ext [%s]\n", ota_nvram_ext);
                    break;
                default:
                    ALOGE("Ignoring invalid attribute type = %d, size = %d\n",
                            it.get_type(), it.get_len());
                    break;
            }
        }
        return NL_OK;
    }

    virtual int handleEvent(WifiEvent& event) {
        /* NO events! */
        return NL_SKIP;
    }
};

wifi_error read_ota_file(char* file, char** buffer, uint32_t* size)
{
    FILE* fp = NULL;
    int file_size, count;
    char* buf;

    fp = fopen(file, "r");
    if (fp == NULL) {
        ALOGI("File [%s] doesn't exist.", file);
        return WIFI_ERROR_NOT_AVAILABLE;
    }

    fseek(fp, 0, SEEK_END);
    file_size = ftell(fp);
    if (file_size == -1L) {
        ALOGI("File [%s] size error not valid", file);
        fclose(fp);
        return WIFI_ERROR_NOT_AVAILABLE;
    }

    buf = (char *)malloc(file_size + 1);
    if (buf == NULL) {
        fclose(fp);
        return WIFI_ERROR_UNKNOWN;
    }
    memset(buf, 0, file_size + 1);
    fseek(fp, 0, SEEK_SET);
    count = fread(buf, file_size, 1, fp);
    if (!count) {
        ALOGE("fread fail.");
        free(buf);
        fclose(fp);
        return WIFI_ERROR_UNKNOWN;
    }

    *buffer = (char*) buf;
    *size = file_size;
    fclose(fp);
    return WIFI_SUCCESS;
}

wifi_error check_multiple_nvram_clm(uint32_t type, char* hw_revision, char* hw_sku,
        char** buffer, uint32_t* buffer_len)
{
    char file_name[MAX_NV_FILE][FILE_NAME_LEN];
    char nvram_clmblob_default_file[FILE_NAME_LEN] = {0,};
    wifi_error result = WIFI_SUCCESS;

    if (type == CLM_BLOB) {
        sprintf(nvram_clmblob_default_file, "%s%s", OTA_PATH, OTA_CLM_FILE);
    }
    else if (type == NVRAM) {
        sprintf(nvram_clmblob_default_file, "%s%s", OTA_PATH, OTA_NVRAM_FILE);
    }
    else if (type == TXCAP_BLOB) {
        sprintf(nvram_clmblob_default_file, "%s%s", OTA_PATH, OTA_TXCAP_BLOB_FILE);
    }

    for (unsigned int i = 0; i < MAX_NV_FILE; i++) {
        memset(file_name[i], 0, FILE_NAME_LEN);
    }

    sprintf(file_name[0], "%s_%s_%s", nvram_clmblob_default_file, hw_revision, hw_sku);
    sprintf(file_name[1], "%s_%s", nvram_clmblob_default_file, hw_revision);
    sprintf(file_name[2], "%s_%s", nvram_clmblob_default_file, hw_sku);
    sprintf(file_name[3], "%s", nvram_clmblob_default_file);

    for (unsigned int i = 0; i < MAX_NV_FILE; i++) {
        result = read_ota_file(file_name[i], buffer, buffer_len);
        if (result == WIFI_SUCCESS) {
            ALOGI("[OTA] %s PATH %s", type == NVRAM ? "NVRAM" : "CLM", file_name[i]);
            break;
        }
    }
    return result;
}

wifi_error wifi_hal_ota_update(wifi_interface_handle iface, uint32_t ota_version)
{
    wifi_handle handle = getWifiHandle(iface);
    wifi_error result = WIFI_SUCCESS;
    ota_info_buf_t buf;
    char *buffer_nvram = NULL;
    char *buffer_clm = NULL;
    char *buffer_txcap_blob = NULL;
    char prop_revision_buf[PROPERTY_VALUE_MAX] = {0,};
    char prop_sku_buf[PROPERTY_VALUE_MAX] = {0,};
    char sku_name[MAX_SKU_NAME_LEN] = {0,};

    OtaUpdateCommand *cmd = new OtaUpdateCommand(iface);
    NULL_CHECK_RETURN(cmd, "memory allocation failure", WIFI_ERROR_OUT_OF_MEMORY);

    ALOGD("wifi_hal_ota_update, handle = %p, ota_version %d\n", handle, ota_version);

    result = (wifi_error)cmd->start();
    if (result != WIFI_SUCCESS) {
        cmd->releaseRef();
        return result;
    }

    property_get(HW_DEV_PROP, prop_revision_buf, NULL);
    property_get(HW_SKU_PROP, prop_sku_buf, NULL);

    strncpy(sku_name, "NA", MAX_SKU_NAME_LEN);
    for (int i = 0; i < ARRAYSIZE(sku_table); i ++) {
        if (strcmp(prop_sku_buf, sku_table[i].hw_id) == 0) {
            strncpy(sku_name, sku_table[i].sku, MAX_SKU_NAME_LEN);
            break;
        }
    }
    ALOGD("prop_sku_buf is %s, sku_name is %s", prop_sku_buf, sku_name);

    check_multiple_nvram_clm(CLM_BLOB, prop_revision_buf, sku_name, &buffer_clm, &buf.ota_clm_len);
    if (buffer_clm == NULL) {
        ALOGE("buffer_clm is null");
        goto exit;
    }
    buf.ota_clm_buf[0] = buffer_clm;

    check_multiple_nvram_clm(TXCAP_BLOB, prop_revision_buf, sku_name,
        &buffer_txcap_blob, &buf.ota_txcap_len);
    if (buffer_txcap_blob == NULL) {
        ALOGE("buffer_txcap_blob is null");
        goto exit;
    }
    buf.ota_txcap_buf[0] = buffer_txcap_blob;

    check_multiple_nvram_clm(NVRAM, prop_revision_buf, sku_name,
            &buffer_nvram, &buf.ota_nvram_len);
    if (buffer_nvram == NULL) {
        ALOGE("buffer_nvram is null");
        goto exit;
    }
    buf.ota_nvram_buf[0] = buffer_nvram;
    cmd->otaDownload(&buf, ota_version);

exit:
    if (buffer_clm != NULL) {
        free(buffer_clm);
    }
    if (buffer_nvram != NULL) {
        free(buffer_nvram);
    }
    if (buffer_txcap_blob != NULL) {
        free(buffer_txcap_blob);
    }

    cmd->releaseRef();

    return result;
}
