/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _MTK_DRIVER_NL80211_H_
#define _MTK_DRIVER_NL80211_H_

#include <linux/wireless.h>

#define OUI_MTK 0x000CE7

enum mtk_nl80211_vendor_subcmd {
    MTK_SUBCMD_STRING_CMD = 0x2454,
};

enum mtk_string_attribute {
    STRING_ATTRIBUTE_INVALID = 0,
    STRING_ATTRIBUTE_DATA = 1,
    STRING_ATTRIBUTE_MAX = 2
};

struct wpa_driver_cmd_reply_s {
    u32 reply_buf_size;
    u32 reply_len;
    union _reply_buf {
        char* ptr;
        u64 data;
    } reply_buf;
};

struct wpa_driver_test_mode_info {
    u32 index;
    u32 buflen;
};

struct wpa_driver_p2p_sigma_params {
    struct wpa_driver_test_mode_info hdr;
    u32 idx;
    u32 value;
};

/* SW CMD */
struct wpa_driver_sw_cmd_params {
    struct wpa_driver_test_mode_info hdr;
    u8 set;
    u32 adr;
    u32 data;
};

struct wpa_driver_suspendmode_params {
    struct wpa_driver_test_mode_info hdr;
    u8 suspend;
};

enum nl80211_testmode_params {
    /* Old test mode command id, compatible with exist testmode command */
    NL80211_TESTMODE_SW_CMD = 1,

    /* New test mode command id, should greater than TESTMODE_CMD_ID_NEW_BEGIN */
    NL80211_TESTMODE_SUSPEND = 101,
};

#endif
