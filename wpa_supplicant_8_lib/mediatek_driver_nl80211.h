/*
 * SPDX-FileCopyrightText: 2024 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _MTK_DRIVER_NL80211_H_
#define _MTK_DRIVER_NL80211_H_

#include <linux/wireless.h>

#ifndef BITS
/* Eddie */
/* bits range: for example BITS(16,23) = 0xFF0000
 *   ==>  (BIT(m)-1)   = 0x0000FFFF     ~(BIT(m)-1)   => 0xFFFF0000
 *   ==>  (BIT(n+1)-1) = 0x00FFFFFF
 */
#define BITS(m, n) (~(BIT(m) - 1) & ((BIT(n) - 1) | BIT(n)))
#endif /* BIT */

#define OUI_MTK 0x000CE7

#define BUG_REPORT_NUM 47

#define IW_ENCODE_ALG_SMS4 0x20

#define IW_ETH_ALEN (6)

extern void nl80211_vendor_event_mtk(struct wpa_driver_nl80211_data*, u32, u8*, size_t);

enum nl80211_testmode_sta_link_statistics_attr {
    __NL80211_TESTMODE_STA_STATISTICS_INVALID = 0,
    NL80211_TESTMODE_STA_STATISTICS_VERSION,
    NL80211_TESTMODE_STA_STATISTICS_MAC,
    NL80211_TESTMODE_STA_STATISTICS_LINK_SCORE,
    NL80211_TESTMODE_STA_STATISTICS_FLAG,
    NL80211_TESTMODE_STA_STATISTICS_PER,
    NL80211_TESTMODE_STA_STATISTICS_RSSI,
    NL80211_TESTMODE_STA_STATISTICS_PHY_MODE,
    NL80211_TESTMODE_STA_STATISTICS_TX_RATE,
    NL80211_TESTMODE_STA_STATISTICS_TOTAL_CNT,
    NL80211_TESTMODE_STA_STATISTICS_THRESHOLD_CNT,
    NL80211_TESTMODE_STA_STATISTICS_AVG_PROCESS_TIME,
    NL80211_TESTMODE_STA_STATISTICS_MAX_PROCESS_TIME,
    NL80211_TESTMODE_STA_STATISTICS_AVG_HIF_PROCESS_TIME,
    NL80211_TESTMODE_STA_STATISTICS_MAX_HIF_PROCESS_TIME,
    NL80211_TESTMODE_STA_STATISTICS_FAIL_CNT,
    NL80211_TESTMODE_STA_STATISTICS_TIMEOUT_CNT,
    NL80211_TESTMODE_STA_STATISTICS_AVG_AIR_TIME,
    NL80211_TESTMODE_STA_STATISTICS_TC_EMPTY_CNT_ARRAY,
    NL80211_TESTMODE_STA_STATISTICS_TC_QUE_LEN_ARRAY,
    NL80211_TESTMODE_STA_STATISTICS_TC_AVG_QUE_LEN_ARRAY,
    NL80211_TESTMODE_STA_STATISTICS_TC_CUR_QUE_LEN_ARRAY,
    NL80211_TESTMODE_STA_STATISTICS_ENQUEUE,
    NL80211_TESTMODE_STA_STATISTICS_STA_ENQUEUE,
    NL80211_TESTMODE_STA_STATISTICS_DEQUEUE,
    NL80211_TESTMODE_STA_STATISTICS_STA_DEQUEUE,
    NL80211_TESTMODE_STA_STATISTICS_RB_ARRAY,
    NL80211_TESTMODE_STA_STATISTICS_NO_TC_ARRAY,
    NL80211_TESTMODE_STA_STATISTICS_TC_USED_ARRAY,
    NL80211_TESTMODE_STA_STATISTICS_TC_WANTED_ARRAY,
    NL80211_TESTMODE_STA_STATISTICS_IRQ_ISR_CNT,
    NL80211_TESTMODE_STA_STATISTICS_IRQ_ISR_PASS_CNT,
    NL80211_TESTMODE_STA_STATISTICS_IRQ_TASK_CNT,
    NL80211_TESTMODE_STA_STATISTICS_IRQ_AB_CNT,
    NL80211_TESTMODE_STA_STATISTICS_IRQ_SW_CNT,
    NL80211_TESTMODE_STA_STATISTICS_IRQ_TX_CNT,
    NL80211_TESTMODE_STA_STATISTICS_IRQ_RX_CNT,
    NL80211_TESTMODE_STA_STATISTICS_RESERVED_ARRAY,
    NL80211_TESTMODE_STA_STATISTICS_NUM,
    NL80211_TESTMODE_STA_STATISTICS_MAX = NL80211_TESTMODE_STA_STATISTICS_NUM - 1
};

enum nl80211_testmode_link_detect_attr {
    NL80211_TESTMODE_LINK_INVALID = 0,
    NL80211_TESTMODE_LINK_TX_FAIL_CNT,
    NL80211_TESTMODE_LINK_TX_RETRY_CNT,
    NL80211_TESTMODE_LINK_TX_MULTI_RETRY_CNT,
    NL80211_TESTMODE_LINK_ACK_FAIL_CNT,
    NL80211_TESTMODE_LINK_FCS_ERR_CNT,
    NL80211_TESTMODE_LINK_TX_OK_CNT,
    NL80211_TESTMODE_LINK_RX_OK_CNT,
    NL80211_TESTMODE_LINK_RST_REASON,
    NL80211_TESTMODE_LINK_RST_TIME,
    NL80211_TESTMODE_LINK_ROAM_FAIL_TIMES,
    NL80211_TESTMODE_LINK_ROAM_FAIL_TIME,
    NL80211_TESTMODE_LINK_TX_DONE_DELAY_IS_ARP,
    NL80211_TESTMODE_LINK_ARRIVE_DRV_TICK,
    NL80211_TESTMODE_LINK_ENQUE_TICK,
    NL80211_TESTMODE_LINK_DEQUE_TICK,
    NL80211_TESTMODE_LINK_LEAVE_DRV_TICK,
    NL80211_TESTMODE_LINK_CURR_TICK,
    NL80211_TESTMODE_LINK_CURR_TIME,
    NL80211_TESTMODE_LINK_DETECT_NUM,
    NL80211_TESTMODE_LINK_DETECT_MAX = NL80211_TESTMODE_LINK_DETECT_NUM - 1
};

typedef enum _ENUM_TRAFFIC_CLASS_INDEX_T {
    TC0_INDEX = 0,
    TC1_INDEX,
    TC2_INDEX,
    TC3_INDEX,
    TC_DATA_NUM,
    TC4_INDEX = TC_DATA_NUM,
    TC5_INDEX,
    TC_NUM
} ENUM_TRAFFIC_CLASS_INDEX_T;

struct wpa_driver_sta_statistics_s {
    u8 version;
    u8 addr[ETH_ALEN];
    u32 flag;
    u32 link_score;
    u8 per;
    int rssi;
    u32 phy_mode;
    double tx_rate;
    u32 tx_total_cnt;
    u32 enqueue_total_cnt;
    u32 dequeue_total_cnt;
    u32 enqueue_sta_total_cnt;
    u32 dequeue_sta_total_cnt;
    u32 tx_exc_threshold_cnt;
    u32 tx_avg_process_time;
    u32 tx_max_process_time;
    u32 tx_avg_hif_process_time;
    u32 tx_max_hif_process_time;
    u32 tx_fail_cnt;
    u32 tx_timeout_cnt;
    u32 tx_avg_air_time;
    u32 tc_buf_full_cnt[TC_DATA_NUM];
    u32 tc_que_len[TC_DATA_NUM];
    u32 tc_back_count[TC_DATA_NUM];
    u32 dequeue_no_tc_res[TC_DATA_NUM];
    u32 tc_wanted_res[TC_DATA_NUM];
    u32 tc_used_res[TC_DATA_NUM];
    u32 isr_cnt;
    u32 isr_pass_cnt;
    u32 isr_task_cnt;
    u32 isr_ab_cnt;
    u32 isr_sw_cnt;
    u32 isr_tx_cnt;
    u32 isr_rx_cnt;
    u32 tc_avg_que_len[TC_DATA_NUM];
    u32 tc_cur_que_len[TC_DATA_NUM];
    u8 reserved[32];
};

struct wpa_driver_sta_link_detect_s {
    u64 tx_fail_cnt;
    u64 tx_retry_cnt;
    u64 tx_multi_retry_cnt;
    u64 ack_fail_cnt;
    u64 fcs_err_cnt;
    u64 tx_ok_cnt;
    u64 rx_ok_cnt;
    u32 rst_reason;
    u64 rst_time;
    u32 roam_fail_times;
    u64 roam_fail_time;
    u8 tx_done_delay_is_arp;
    u32 arrive_drv_tick;
    u32 en_que_tick;
    u32 de_que_tick;
    u32 leave_drv_tick;
    u32 curr_tick;
    u64 curr_time;
    u32 bug_report[BUG_REPORT_NUM];
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

struct wpa_driver_cmd_reply_params {
    struct wpa_driver_test_mode_info hdr;
    struct wpa_driver_cmd_reply_s reply_info;
};

struct wpa_driver_testmode_params {
    struct wpa_driver_test_mode_info hdr;
    u8* buf;
};

struct wpa_driver_get_sta_statistics_params {
    struct wpa_driver_test_mode_info hdr;
    u32 version;
    u32 flag;
    u8 addr[ETH_ALEN];
    u8* buf;
};

struct wpa_driver_p2p_sigma_params {
    struct wpa_driver_test_mode_info hdr;
    u32 idx;
    u32 value;
};

struct wpa_driver_get_sta_link_detect_params {
    struct wpa_driver_test_mode_info hdr;
    u8* buf;
};

struct wpa_driver_hotspot_params {
    struct wpa_driver_test_mode_info hdr;
    u8 blocked;
    u8 bssid[ETH_ALEN];
};

struct wpa_driver_hotspot_set_config_params {
    struct wpa_driver_test_mode_info hdr;
    u32 index;
    u32 value;
};

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

struct wpa_pmkid_entry {
    u8 bssid[IW_ETH_ALEN];
    u8 sta[IW_ETH_ALEN];
    u8 pmkid[IW_PMKID_LEN];
    u8 addremove;
};
struct wpa_driver_pmkid_entry {
    struct wpa_driver_test_mode_info hdr;
    u8 bssid[IW_ETH_ALEN];
    u8 sta[IW_ETH_ALEN];
    u8 pmkid[IW_PMKID_LEN];
    u8 addremove;
};

struct iw_encode_exts {
    u32 ext_flags;
    u8 tx_seq[IW_ENCODE_SEQ_MAX_SIZE];
    u8 rx_seq[IW_ENCODE_SEQ_MAX_SIZE];
    u8 addr[ETH_ALEN];
    u16 alg;
    u16 key_len;
    u8 key[32];
};

struct wpa_driver_rx_filter_params {
    struct wpa_driver_test_mode_info hdr;
    u32 Ipv4FilterHigh;
    u32 Ipv4FilterLow;
    u32 Ipv6FilterHigh;
    u32 Ipv6FilterLow;
    u32 SnapFilterHigh;
    u32 SnapFilterLow;
};

struct wpa_driver_wapi_key_params {
    struct wpa_driver_test_mode_info hdr;
    u8 key_index;
    u8 key_len;
    struct iw_encode_exts extparams;
};

struct wpa_driver_wfd_data_s {
    struct wpa_driver_test_mode_info hdr;
    u32 WfdCmdType;
    u8 WfdEnable;
    u8 WfdCoupleSinkStatus;
    u8 WfdSessionAvailable;
    u8 WfdSigmaMode;
    u16 WfdDevInfo;
    u16 WfdControlPort;
    u16 WfdMaximumTp;
    u16 WfdExtendCap;
    u8 WfdCoupleSinkAddress[ETH_ALEN];
    u8 WfdAssociatedBssid[ETH_ALEN];
    u8 WfdVideoIp[4];
    u8 WfdAudioIp[4];
    u16 WfdVideoPort;
    u16 WfdAudioPort;
    u32 WfdFlag;
    u32 WfdPolicy;
    u32 WfdState;
    u8 WfdSessionInformationIE[24 * 8];
    u16 WfdSessionInformationIELen;
    u8 Reverved1[2];
    u8 WfdPrimarySinkMac[ETH_ALEN];
    u8 WfdSecondarySinkMac[ETH_ALEN];
    u32 WfdAdvancedFlag;
    u8 WfdLocalIp[4];
    u16 WfdLifetimeAc2;
    u16 WfdLifetimeAc3;
    u16 WfdCounterThreshold;
    u8 Reverved2[54];
    u8 Reverved3[64];
    u8 Reverved4[64];
} wfd_data;

struct wpa_driver_set_beamplus_params {
    struct wpa_driver_test_mode_info hdr;
    u32 value;
};

enum nl80211_testmode_params {
    NL80211_TESTMODE_SW_CMD = 1,
    NL80211_TESTMODE_WAPI = 2,
    NL80211_TESTMODE_HS20 = 3,
    NL80211_TESTMODE_POORLINK = 4,
    NL80211_TESTMODE_STATISTICS = 0x10,
    NL80211_TESTMODE_LINK_DETECT = 0x20,
    NL80211_TESTMODE_HS_SET_CONFIG = 51,
    NL80211_TESTMODE_NEW_BEGIN = 100,
    NL80211_TESTMODE_SUSPEND = 101,
    NL80211_TESTMODE_STR_CMD = 102,
    NL80211_TESTMODE_RXFILTER = 103,
    NL80211_TESTMODE_UPDATE_STA_PMKID = 1000
};

enum mtk_nl80211_vendor_subcmds {
    WIFI_EVENT_DRIVER_ERROR = 8,
    WIFI_EVENT_GENERIC_RESPONSE = 10,
};

enum mtk_wlan_vendor_attr_driver_error {
    MTK_WALN_VENDOR_ATTR_DRIVER_ERROR_DATA_STALL_NOTICE = 0,
    MTK_WALN_VENDOR_ATTR_DRIVER_ERROR_AFTER_LAST,
    MTK_WALN_VENDOR_ATTR_DRIVER_ERROR_MAX = MTK_WALN_VENDOR_ATTR_DRIVER_ERROR_AFTER_LAST - 1
};

#endif
