/*
 * Copyright (C) 2025 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "includes.h"
#include "netlink/genl/genl.h"

#include "common.h"
#include "driver_nl80211.h"
#include "linux_ioctl.h"
#include "wpa_supplicant_i.h"
#include "config.h"
#ifdef ANDROID
#include "android_drv.h"
#endif

#include "mediatek_driver_nl80211.h"
#include "driver_i.h"
#include "p2p/p2p_i.h"

#define PRIV_CMD_SIZE 512
#define MAX_DRIVER_CMD_LEN 512
#define MAX_ARGS 32
#define MAX_RESPONSE_LEN 2048
#define MAX_CONSECUTIVE_ERRORS 5

typedef struct android_wifi_priv_cmd {
    char buf[PRIV_CMD_SIZE];
    int used_len;
    int total_len;
} android_wifi_priv_cmd;

/* Track consecutive errors to detect driver hang */
static int error_count = 0;

/**
 * struct driver_cmd_context - Context for driver commands
 *
 * This structure provides a cleaner way to handle driver commands
 * by bundling all related data together.
 */
struct driver_cmd_context {
    struct i802_bss *bss;
    struct wpa_driver_nl80211_data *drv;
    struct wpa_supplicant *wpa_s;
    struct hostapd_data *hapd;
    char *cmd;
    size_t cmd_len;
    char *buf;
    size_t buf_len;
    android_wifi_priv_cmd priv_cmd;
    struct ifreq ifr;
};

/**
 * init_driver_cmd_context - Initialize the driver command context
 * @ctx: Driver command context to initialize
 * @priv: Private BSS data
 * @cmd: Command string
 * @buf: Buffer for response
 * @buf_len: Length of buffer
 *
 * Returns: 0 on success, -1 on failure
 */
static int init_driver_cmd_context(struct driver_cmd_context *ctx,
                                  void *priv, char *cmd, char *buf, size_t buf_len)
{
    if (!ctx || !priv || !cmd || !buf)
        return -1;

    os_memset(ctx, 0, sizeof(*ctx));

    ctx->bss = priv;
    ctx->drv = ctx->bss->drv;
    ctx->cmd = cmd;
    ctx->cmd_len = os_strlen(cmd);
    ctx->buf = buf;
    ctx->buf_len = buf_len;

    if (!ctx->drv || !ctx->drv->ctx) {
        wpa_printf(MSG_ERROR, "MediaTek: Invalid driver context");
        return -1;
    }

    /* Determine if we're in station or AP mode */
    if (ctx->drv->nlmode == NL80211_IFTYPE_AP) {
        ctx->hapd = ctx->drv->ctx;
    } else {
        ctx->wpa_s = ctx->drv->ctx;
    }

    /* Initialize private command structure */
    os_memset(&ctx->priv_cmd, 0, sizeof(ctx->priv_cmd));
    os_memset(&ctx->ifr, 0, sizeof(ctx->ifr));

    return 0;
}

/**
 * skip_white_space - Skip leading whitespace characters in a string
 * @cmd: Input command string to process
 *
 * Returns: Pointer to first non-space character in the string
 */
char *skip_white_space(char *cmd)
{
	char *pos = cmd;

	while (*pos == ' ')
		pos++;

	return pos;
}

/**
 * tokenize_cmd - Parse command into arguments
 * @cmd: Command string
 * @argv: Array to store argument pointers
 * @max_args: Maximum number of arguments
 *
 * Returns: Number of arguments parsed
 */
static int tokenize_cmd(char *cmd, char *argv[], int max_args)
{
    int argc = 0;
    char *pos, *start = cmd;

    while (argc < max_args) {
        /* Skip leading whitespace */
        start = skip_white_space(start);
        if (*start == '\0')
            break;

        /* Record argument start */
        argv[argc++] = start;

        /* Find end of argument */
        pos = start;
        while (*pos && *pos != ' ' && *pos != '\t' && *pos != '\n')
            pos++;

        if (*pos == '\0')
            break;

        /* Null-terminate and advance to next argument */
        *pos++ = '\0';
        start = pos;
    }

    return argc;
}

/**
 * track_driver_error - Track driver errors to detect potential hangs
 */
static void track_driver_error(void)
{
    error_count++;
    if (error_count > MAX_CONSECUTIVE_ERRORS) {
        wpa_printf(MSG_WARNING, "MediaTek: Detected potential driver hang - reached max consecutive errors");
        error_count = 0; /* Reset counter to avoid log spam */
    }
}

/**
 * reset_error_counter - Reset the error counter on successful operations
 */
static void reset_error_counter(void)
{
    error_count = 0;
}

/**
 * mediatek_send_command - Send command to driver via ioctl
 * @ctx: Driver command context
 * @cmd: Command string
 *
 * Returns: 0 on success, negative on failure
 */
static int mediatek_send_command(struct driver_cmd_context *ctx, const char *cmd)
{
    int ret;

    if (ctx->cmd_len >= PRIV_CMD_SIZE) {
        wpa_printf(MSG_ERROR, "MediaTek: Command too long: %s", cmd);
        return -E2BIG;
    }

    os_strlcpy(ctx->ifr.ifr_name, ctx->bss->ifname, IFNAMSIZ);
    os_memcpy(ctx->priv_cmd.buf, cmd, ctx->cmd_len + 1);
    ctx->priv_cmd.used_len = ctx->cmd_len + 1;
    ctx->priv_cmd.total_len = PRIV_CMD_SIZE;
    ctx->ifr.ifr_data = &ctx->priv_cmd;

    ret = ioctl(ctx->drv->global->ioctl_sock, SIOCDEVPRIVATE + 1, &ctx->ifr);
    if (ret < 0) {
        wpa_printf(MSG_ERROR, "MediaTek: Private command failed: %s, ret=%d, errno=%d(%s)",
                 cmd, ret, errno, strerror(errno));
        track_driver_error();
        return ret;
    }

    reset_error_counter();
    return 0;
}

/* Station statistics handler for nl80211 testmode response */
static int station_stats_handler(struct nl_msg *msg, void *arg)
{
    struct nlattr *tb[NL80211_ATTR_MAX + 1] = {0};
    struct genlmsghdr *gnlh = nlmsg_data(nlmsg_hdr(msg));
    struct nlattr *sinfo[NL80211_TESTMODE_STA_STATISTICS_NUM] = {0};
    struct wpa_driver_sta_statistics_s *sta_stats = arg;

    static struct nla_policy stats_policy[NL80211_TESTMODE_STA_STATISTICS_NUM] = {
        [NL80211_TESTMODE_STA_STATISTICS_VERSION] = { .type = NLA_U8 },
        [NL80211_TESTMODE_STA_STATISTICS_MAC] = { .type = NLA_UNSPEC },
        [NL80211_TESTMODE_STA_STATISTICS_LINK_SCORE] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_FLAG] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_PER] = { .type = NLA_U8 },
        [NL80211_TESTMODE_STA_STATISTICS_RSSI] = { .type = NLA_U8 },
        [NL80211_TESTMODE_STA_STATISTICS_PHY_MODE] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_TX_RATE] = { .type = NLA_U16 },
        [NL80211_TESTMODE_STA_STATISTICS_FAIL_CNT] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_TIMEOUT_CNT] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_AVG_AIR_TIME] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_TOTAL_CNT] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_THRESHOLD_CNT] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_AVG_PROCESS_TIME] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_MAX_PROCESS_TIME] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_AVG_HIF_PROCESS_TIME] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_MAX_HIF_PROCESS_TIME] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_TC_EMPTY_CNT_ARRAY] = { .type = NLA_UNSPEC },
        [NL80211_TESTMODE_STA_STATISTICS_TC_QUE_LEN_ARRAY] = { .type = NLA_UNSPEC },
        [NL80211_TESTMODE_STA_STATISTICS_TC_AVG_QUE_LEN_ARRAY] = { .type = NLA_UNSPEC },
        [NL80211_TESTMODE_STA_STATISTICS_TC_CUR_QUE_LEN_ARRAY] = { .type = NLA_UNSPEC },
        [NL80211_TESTMODE_STA_STATISTICS_ENQUEUE] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_STA_ENQUEUE] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_DEQUEUE] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_STA_DEQUEUE] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_RB_ARRAY] = { .type = NLA_UNSPEC },
        [NL80211_TESTMODE_STA_STATISTICS_NO_TC_ARRAY] = { .type = NLA_UNSPEC },
        [NL80211_TESTMODE_STA_STATISTICS_TC_USED_ARRAY] = { .type = NLA_UNSPEC },
        [NL80211_TESTMODE_STA_STATISTICS_TC_WANTED_ARRAY] = { .type = NLA_UNSPEC },
        [NL80211_TESTMODE_STA_STATISTICS_IRQ_ISR_CNT] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_IRQ_ISR_PASS_CNT] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_IRQ_TASK_CNT] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_IRQ_AB_CNT] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_IRQ_SW_CNT] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_IRQ_TX_CNT] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_IRQ_RX_CNT] = { .type = NLA_U32 },
        [NL80211_TESTMODE_STA_STATISTICS_RESERVED_ARRAY] = { .type = NLA_UNSPEC }
    };

    /* Parse the message */
    nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0),
              genlmsg_attrlen(gnlh, 0), NULL);

    if (!tb[NL80211_ATTR_TESTDATA] ||
        nla_parse_nested(sinfo, NL80211_TESTMODE_STA_STATISTICS_MAX,
                        tb[NL80211_ATTR_TESTDATA], stats_policy))
        return NL_SKIP;

    /* Process all statistics fields from the response */
    for (int i = 1; i < NL80211_TESTMODE_STA_STATISTICS_NUM; i++) {
        if (!sinfo[i])
            continue;

        switch (i) {
        case NL80211_TESTMODE_STA_STATISTICS_VERSION:
            sta_stats->version = nla_get_u8(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_MAC:
            nla_memcpy(sta_stats->addr, sinfo[i], ETH_ALEN);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_LINK_SCORE:
            sta_stats->link_score = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_FLAG:
            sta_stats->flag = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_PER:
            sta_stats->per = nla_get_u8(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_RSSI:
            sta_stats->rssi = (((int)nla_get_u8(sinfo[i]) - 220) / 2);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_PHY_MODE:
            sta_stats->phy_mode = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_TX_RATE:
            sta_stats->tx_rate = (((double)nla_get_u16(sinfo[i])) / 2);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_FAIL_CNT:
            sta_stats->tx_fail_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_TIMEOUT_CNT:
            sta_stats->tx_timeout_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_AVG_AIR_TIME:
            sta_stats->tx_avg_air_time = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_TOTAL_CNT:
            sta_stats->tx_total_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_THRESHOLD_CNT:
            sta_stats->tx_exc_threshold_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_AVG_PROCESS_TIME:
            sta_stats->tx_avg_process_time = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_MAX_PROCESS_TIME:
            sta_stats->tx_max_process_time = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_AVG_HIF_PROCESS_TIME:
            sta_stats->tx_avg_hif_process_time = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_MAX_HIF_PROCESS_TIME:
            sta_stats->tx_max_hif_process_time = nla_get_u32(sinfo[i]);
            break;

        /* Parse array data */
        case NL80211_TESTMODE_STA_STATISTICS_TC_EMPTY_CNT_ARRAY:
            nla_memcpy(sta_stats->tc_buf_full_cnt, sinfo[i],
                       sizeof(sta_stats->tc_buf_full_cnt));
            break;
        case NL80211_TESTMODE_STA_STATISTICS_TC_QUE_LEN_ARRAY:
            nla_memcpy(sta_stats->tc_que_len, sinfo[i],
                       sizeof(sta_stats->tc_que_len));
            break;
        case NL80211_TESTMODE_STA_STATISTICS_TC_AVG_QUE_LEN_ARRAY:
            nla_memcpy(sta_stats->tc_avg_que_len, sinfo[i],
                       sizeof(sta_stats->tc_avg_que_len));
            break;
        case NL80211_TESTMODE_STA_STATISTICS_TC_CUR_QUE_LEN_ARRAY:
            nla_memcpy(sta_stats->tc_cur_que_len, sinfo[i],
                       sizeof(sta_stats->tc_cur_que_len));
            break;

        /* Queue management counters */
        case NL80211_TESTMODE_STA_STATISTICS_ENQUEUE:
            sta_stats->enqueue_total_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_DEQUEUE:
            sta_stats->dequeue_total_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_STA_ENQUEUE:
            sta_stats->enqueue_sta_total_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_STA_DEQUEUE:
            sta_stats->dequeue_sta_total_cnt = nla_get_u32(sinfo[i]);
            break;

        /* Traffic class resource arrays */
        case NL80211_TESTMODE_STA_STATISTICS_RB_ARRAY:
            nla_memcpy(sta_stats->tc_back_count, sinfo[i],
                       sizeof(sta_stats->tc_back_count));
            break;
        case NL80211_TESTMODE_STA_STATISTICS_NO_TC_ARRAY:
            nla_memcpy(sta_stats->dequeue_no_tc_res, sinfo[i],
                       sizeof(sta_stats->dequeue_no_tc_res));
            break;
        case NL80211_TESTMODE_STA_STATISTICS_TC_USED_ARRAY:
            nla_memcpy(sta_stats->tc_used_res, sinfo[i],
                       sizeof(sta_stats->tc_used_res));
            break;
        case NL80211_TESTMODE_STA_STATISTICS_TC_WANTED_ARRAY:
            nla_memcpy(sta_stats->tc_wanted_res, sinfo[i],
                       sizeof(sta_stats->tc_wanted_res));
            break;

        /* IRQ counters */
        case NL80211_TESTMODE_STA_STATISTICS_IRQ_ISR_CNT:
            sta_stats->isr_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_IRQ_ISR_PASS_CNT:
            sta_stats->isr_pass_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_IRQ_TASK_CNT:
            sta_stats->isr_task_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_IRQ_AB_CNT:
            sta_stats->isr_ab_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_IRQ_SW_CNT:
            sta_stats->isr_sw_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_IRQ_TX_CNT:
            sta_stats->isr_tx_cnt = nla_get_u32(sinfo[i]);
            break;
        case NL80211_TESTMODE_STA_STATISTICS_IRQ_RX_CNT:
            sta_stats->isr_rx_cnt = nla_get_u32(sinfo[i]);
            break;

        /* Reserved data */
        case NL80211_TESTMODE_STA_STATISTICS_RESERVED_ARRAY:
            nla_memcpy(sta_stats->reserved, sinfo[i],
                      sizeof(sta_stats->reserved));
            break;
        }
    }

    return NL_SKIP;
}

/**
 * mediatek_testmode_cmd_handler - Reply handler for testmode string commands
 */
static int mediatek_testmode_cmd_handler(struct nl_msg *msg, void *arg)
{
    struct nlattr *tb[NL80211_ATTR_MAX + 1] = {0};
    struct genlmsghdr *gnlh = nlmsg_data(nlmsg_hdr(msg));
    struct wpa_driver_cmd_reply_s *reply = arg;

    nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0),
              genlmsg_attrlen(gnlh, 0), NULL);

    if (!tb[NL80211_ATTR_TESTDATA])
        return NL_SKIP;

    /* Copy response data to reply buffer */
    os_strlcpy(reply->reply_buf.ptr, nla_data(tb[NL80211_ATTR_TESTDATA]),
              reply->reply_buf_size - 1);
    reply->reply_buf.ptr[reply->reply_buf_size - 1] = '\0';
    reply->reply_len = os_strlen(reply->reply_buf.ptr);

    return NL_SKIP;
}

/**
 * mediatek_testmode_send - Send a testmode command to the driver
 * @priv: Private driver data
 * @data: Testmode command data
 * @data_len: Length of command data
 *
 * Returns: 0 on success, negative on failure
 */
static int mediatek_testmode_send(void *priv, const u8 *data, size_t data_len)
{
    struct i802_bss *bss = priv;
    struct wpa_driver_nl80211_data *drv = bss->drv;
    struct nl_msg *msg;
    struct wpa_driver_testmode_params *params;
    int index, ret = -1;

    msg = nlmsg_alloc();
    if (!msg)
        return -ENOMEM;

    wpa_printf(MSG_DEBUG, "MediaTek: Testmode command for ifindex=%d", drv->ifindex);

    nl80211_cmd(drv, msg, 0, NL80211_CMD_TESTMODE);

    NLA_PUT_U32(msg, NL80211_ATTR_IFINDEX, drv->ifindex);
    NLA_PUT(msg, NL80211_ATTR_TESTDATA, data_len, data);

    params = (struct wpa_driver_testmode_params *)data;
    index = params->hdr.index & BITS(0, 23);

    switch (index) {
        case NL80211_TESTMODE_STATISTICS: {
            struct wpa_driver_get_sta_statistics_params *sta_params =
                (struct wpa_driver_get_sta_statistics_params *)data;
            return send_and_recv_msgs(drv, msg, station_stats_handler,
                                     sta_params->buf, NULL, NULL);
        }
        case NL80211_TESTMODE_STR_CMD: {
            struct wpa_driver_cmd_reply_params *reply_params =
                (struct wpa_driver_cmd_reply_params *)data;
            return send_and_recv_msgs(drv, msg, mediatek_testmode_cmd_handler,
                                     &(reply_params->reply_info), NULL, NULL);
        }
        default: {
            ret = send_and_recv_msgs(drv, msg, NULL, NULL, NULL, NULL);
            wpa_printf(MSG_DEBUG, "MediaTek: Testmode command result: %d", ret);
            return ret;
        }
    }

nla_put_failure:
    nlmsg_free(msg);
    return -ENOBUFS;
}

/**
 * mediatek_send_sw_cmd - Send a software command to the driver
 * @priv: Private driver data
 * @set: Set (1) or get (0) operation
 * @adr: Command address
 * @dat: Command data
 *
 * Returns: 0 on success, negative on failure
 */
static int mediatek_send_sw_cmd(void *priv, int set, u32 *adr, u32 *dat)
{
    struct wpa_driver_sw_cmd_params params;

    os_memset(&params, 0, sizeof(params));

    params.hdr.index = NL80211_TESTMODE_SW_CMD;
    params.hdr.index = params.hdr.index | (0x01 << 24);
    params.hdr.buflen = sizeof(struct wpa_driver_sw_cmd_params);

    params.adr = *adr;
    params.data = *dat;
    params.set = set ? 1 : 0;

    return mediatek_testmode_send(priv, (u8 *)&params, sizeof(params));
}

/**
 * mediatek_set_country_code - Set country code via private ioctl
 * @priv: Private driver data
 * @alpha2_arg: Two letter country code
 *
 * Returns: 0 on success, negative on failure
 */
static int mediatek_set_country_code(void *priv, const char *alpha2_arg)
{
    struct i802_bss *bss = priv;
    struct wpa_driver_nl80211_data *drv = bss->drv;
    int ioctl_sock = -1;
    struct iwreq iwr;
    int ret = -1;
    char buf[11];

    ioctl_sock = socket(PF_INET, SOCK_DGRAM, 0);
    if (ioctl_sock < 0) {
        wpa_printf(MSG_ERROR, "MediaTek: Failed to create socket for country code setting");
        return -1;
    }

    os_memset(&iwr, 0, sizeof(iwr));
    os_strlcpy(iwr.ifr_name, drv->first_bss->ifname, IFNAMSIZ);
    iwr.ifr_name[IFNAMSIZ - 1] = '\0';

    ret = os_snprintf(buf, sizeof(buf), "COUNTRY %s", alpha2_arg);
    if (ret < 0 || ret >= sizeof(buf)) {
        wpa_printf(MSG_ERROR, "MediaTek: Country code buffer overflow");
        close(ioctl_sock);
        return -1;
    }

    iwr.u.data.pointer = buf;
    iwr.u.data.length = os_strlen(buf);

    if ((ret = ioctl(ioctl_sock, 0x8B0C, &iwr)) < 0) {  // SIOCSIWPRIV
        wpa_printf(MSG_ERROR, "MediaTek: Country code ioctl failed: %s", buf);
    }

    close(ioctl_sock);
    return ret;
}

/**
 * mediatek_notify_country_change - Notify interfaces about country code change
 * @global: Global wpa_supplicant data
 * @cmd: Command string containing country information
 */
static void mediatek_notify_country_change(struct wpa_global *global, char *cmd)
{
    if (os_strncasecmp(cmd, "COUNTRY", 7) != 0)
        return;

    struct wpa_supplicant *wpa_s;
    union wpa_event_data event;

    os_memset(&event, 0, sizeof(event));
    event.channel_list_changed.initiator = REGDOM_SET_BY_USER;

    if (os_strlen(cmd) > 9) {
        event.channel_list_changed.type = REGDOM_TYPE_COUNTRY;
        event.channel_list_changed.alpha2[0] = cmd[8];
        event.channel_list_changed.alpha2[1] = cmd[9];
    } else {
        event.channel_list_changed.type = REGDOM_TYPE_UNKNOWN;
    }

    /* Notify all interfaces */
    for (wpa_s = global->ifaces; wpa_s; wpa_s = wpa_s->next) {
        wpa_supplicant_event(wpa_s, EVENT_CHANNEL_LIST_CHANGED, &event);
    }
}

/**
 * mediatek_get_p2p_device - Find a P2P device by address
 */
static struct p2p_device *mediatek_get_p2p_device(struct p2p_data *p2p, const u8 *addr)
{
    struct p2p_device *dev;

    if (!p2p)
        return NULL;

    dl_list_for_each(dev, &p2p->devices, struct p2p_device, list) {
        if (os_memcmp(dev->info.p2p_device_addr, addr, ETH_ALEN) == 0)
            return dev;
    }

    return NULL;
}

/**
 * mediatek_get_p2p_addr - Get correct MAC address for P2P stations
 * @wpa_s: wpa_supplicant data
 * @org_addr: Original MAC address
 *
 * Returns: Pointer to appropriate MAC address or NULL if not found
 */
static u8 *mediatek_get_p2p_addr(struct wpa_supplicant *wpa_s, u8 *org_addr)
{
    struct p2p_data *p2p = wpa_s->global->p2p;
    struct wpa_ssid *ssid = wpa_s->current_ssid;
    struct p2p_device *dev;

    if (!p2p || !ssid) {
        wpa_printf(MSG_DEBUG, "MediaTek: P2P not available or SSID not connected");
        return NULL;
    }

    dev = mediatek_get_p2p_device(p2p, org_addr);
    if (!dev) {
        wpa_printf(MSG_DEBUG, "MediaTek: P2P device not found: " MACSTR, MAC2STR(org_addr));
        return NULL;
    }

    /* If we're a P2P client */
    if (ssid->mode == WPAS_MODE_INFRA) {
        if (os_memcmp(dev->info.p2p_device_addr, wpa_s->bssid, ETH_ALEN) &&
            !is_zero_ether_addr(wpa_s->bssid)) {
            wpa_printf(MSG_DEBUG, "MediaTek: Using GC interface address " MACSTR
                      " instead of " MACSTR, MAC2STR(wpa_s->bssid),
                      MAC2STR(org_addr));
            return wpa_s->bssid;
        }
    }

    /* If we're a P2P GO */
    if (os_memcmp(dev->info.p2p_device_addr, dev->interface_addr, ETH_ALEN) &&
        !is_zero_ether_addr(dev->interface_addr)) {
        wpa_printf(MSG_DEBUG, "MediaTek: Using GO interface address " MACSTR
                 " instead of " MACSTR, MAC2STR(dev->interface_addr),
                 MAC2STR(org_addr));
        return dev->interface_addr;
    }

    return NULL;
}

/**
 * mediatek_get_station_stats - Get station statistics from driver
 * @wpa_s: wpa_supplicant data
 * @sta_addr: Station MAC address
 * @buf: Buffer to store statistics data
 *
 * Returns: 0 on success, negative on failure
 */
static int mediatek_get_station_stats(struct wpa_supplicant *wpa_s, u8 *sta_addr, u8 *buf)
{
    struct wpa_driver_get_sta_statistics_params params;

    os_memset(&params, 0, sizeof(params));

    if (sta_addr)
        os_memcpy(params.addr, sta_addr, ETH_ALEN);

    wpa_printf(MSG_DEBUG, "MediaTek: Requesting statistics for STA " MACSTR,
              MAC2STR(params.addr));

    params.hdr.index = NL80211_TESTMODE_STATISTICS;
    params.hdr.index = params.hdr.index | (0x01 << 24);
    params.hdr.buflen = sizeof(struct wpa_driver_get_sta_statistics_params);
    params.buf = buf;

    return mediatek_testmode_send(wpa_s->drv_priv, (u8 *)&params, sizeof(params));
}

/**
 * mediatek_format_station_stats - Format station statistics as text
 * @wpa_s: wpa_supplicant data
 * @stats: Station statistics data
 * @buf: Buffer to store formatted output
 * @buflen: Length of buffer
 *
 * Returns: Length of formatted data or 0 on error
 */
static int mediatek_format_station_stats(struct wpa_supplicant *wpa_s,
                                     struct wpa_driver_sta_statistics_s *stats,
                                     char *buf, size_t buflen)
{
    int ret;
    size_t i;
    char *pos = buf;
    char *end = buf + buflen;

    /* Format basic station information */
    ret = os_snprintf(pos, end - pos, "sta_addr=" MACSTR "\n", MAC2STR(stats->addr));
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "link_score=%d\n", stats->link_score);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "per=%d\n", stats->per);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "rssi=%d\n", stats->rssi);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "phy=0x%08X\n", stats->phy_mode);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "rate=%.1f\n", stats->tx_rate);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    /* Packet statistics */
    ret = os_snprintf(pos, end - pos, "total_cnt=%d\n", stats->tx_total_cnt);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "threshold_cnt=%d\n", stats->tx_exc_threshold_cnt);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "fail_cnt=%d\n", stats->tx_fail_cnt);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "timeout_cnt=%d\n", stats->tx_timeout_cnt);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "apt=%d\n", stats->tx_avg_process_time);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "aat=%d\n", stats->tx_avg_air_time);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    /* Traffic class information */
    ret = os_snprintf(pos, end - pos, "TC_buf_full_cnt=%d:%d:%d:%d\n",
                     stats->tc_buf_full_cnt[TC0_INDEX],
                     stats->tc_buf_full_cnt[TC1_INDEX],
                     stats->tc_buf_full_cnt[TC2_INDEX],
                     stats->tc_buf_full_cnt[TC3_INDEX]);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "TC_sta_que_len=%d:%d:%d:%d\n",
                     stats->tc_que_len[TC0_INDEX],
                     stats->tc_que_len[TC1_INDEX],
                     stats->tc_que_len[TC2_INDEX],
                     stats->tc_que_len[TC3_INDEX]);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "TC_avg_que_len=%d:%d:%d:%d\n",
                     stats->tc_avg_que_len[TC0_INDEX],
                     stats->tc_avg_que_len[TC1_INDEX],
                     stats->tc_avg_que_len[TC2_INDEX],
                     stats->tc_avg_que_len[TC3_INDEX]);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "TC_cur_que_len=%d:%d:%d:%d\n",
                     stats->tc_cur_que_len[TC0_INDEX],
                     stats->tc_cur_que_len[TC1_INDEX],
                     stats->tc_cur_que_len[TC2_INDEX],
                     stats->tc_cur_que_len[TC3_INDEX]);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    /* Firmware flags and reserved data */
    ret = os_snprintf(pos, end - pos, "flag=0x%08X\n", stats->flag);
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    /* Format reserved data as hex */
    ret = os_snprintf(pos, end - pos, "reserved0=");
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    for (i = 0; i < 16; i++) {
        ret = os_snprintf(pos, end - pos, "%02X", stats->reserved[i]);
        if (ret < 0 || ret >= end - pos) return 0;
        pos += ret;

        if (((i + 1) % 4) == 0) {
            ret = os_snprintf(pos, end - pos, " ");
            if (ret < 0 || ret >= end - pos) return 0;
            pos += ret;
        }
    }

    ret = os_snprintf(pos, end - pos, "\n");
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "reserved1=");
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    for (i = 16; i < 32; i++) {
        ret = os_snprintf(pos, end - pos, "%02X", stats->reserved[i]);
        if (ret < 0 || ret >= end - pos) return 0;
        pos += ret;

        if (((i + 1) % 4) == 0) {
            ret = os_snprintf(pos, end - pos, " ");
            if (ret < 0 || ret >= end - pos) return 0;
            pos += ret;
        }
    }

    ret = os_snprintf(pos, end - pos, "\n");
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    ret = os_snprintf(pos, end - pos, "====\n");
    if (ret < 0 || ret >= end - pos) return 0;
    pos += ret;

    return pos - buf;
}

/**
 * mediatek_log_station_stats - Log detailed station statistics
 * @s: Station statistics data
 */
static void mediatek_log_station_stats(struct wpa_driver_sta_statistics_s *s)
{
    wpa_printf(MSG_DEBUG, "MediaTek: STA Stats: Basic info* AVG:%4d:EN:%4d:DE:%4d:SEN:%4d:SDE:%4d:HIF:%4d",
              s->tx_avg_process_time,
              s->enqueue_total_cnt,
              s->dequeue_total_cnt,
              s->enqueue_sta_total_cnt,
              s->dequeue_sta_total_cnt,
              s->tx_total_cnt);

    wpa_printf(MSG_DEBUG, "MediaTek: STA Stats: Time info* TTL:%4d:AVG:%4d:MAX:%4d:HIFAVG:%4d:HIFMAX:%4d",
              s->tx_total_cnt,
              s->tx_avg_process_time,
              s->tx_max_process_time,
              s->tx_avg_hif_process_time,
              s->tx_max_hif_process_time);

    wpa_printf(MSG_DEBUG, "MediaTek: STA Stats: TC Resource* Score:%4d:EN:%4d#%4d#%4d#%4d:DE:%4d#%4d#%4d#%4d",
              s->link_score,
              s->tc_buf_full_cnt[TC0_INDEX],
              s->tc_buf_full_cnt[TC1_INDEX],
              s->tc_buf_full_cnt[TC2_INDEX],
              s->tc_buf_full_cnt[TC3_INDEX],
              s->dequeue_no_tc_res[TC0_INDEX],
              s->dequeue_no_tc_res[TC1_INDEX],
              s->dequeue_no_tc_res[TC2_INDEX],
              s->dequeue_no_tc_res[TC3_INDEX]);

    wpa_printf(MSG_DEBUG, "MediaTek: STA Stats: IRQ info* T:%4d:P:%4d:TT:%4d:A:%4d:S:%4d:R:%4d:T:%4d",
              s->isr_cnt,
              s->isr_pass_cnt,
              s->isr_task_cnt,
              s->isr_ab_cnt,
              s->isr_sw_cnt,
              s->isr_rx_cnt,
              s->isr_tx_cnt);

    wpa_printf(MSG_DEBUG, "MediaTek: STA Stats: TC Res details [W:U:B]* Score:%4d:"
              "#%5d:%5d:%5d#"
              "#%5d:%5d:%5d#"
              "#%5d:%5d:%5d#"
              "#%5d:%5d:%5d#",
              s->link_score,
              s->tc_wanted_res[TC0_INDEX],
              s->tc_used_res[TC0_INDEX],
              s->tc_back_count[TC0_INDEX],
              s->tc_wanted_res[TC1_INDEX],
              s->tc_used_res[TC1_INDEX],
              s->tc_back_count[TC1_INDEX],
              s->tc_wanted_res[TC2_INDEX],
              s->tc_used_res[TC2_INDEX],
              s->tc_back_count[TC2_INDEX],
              s->tc_wanted_res[TC3_INDEX],
              s->tc_used_res[TC3_INDEX],
              s->tc_back_count[TC3_INDEX]);
}

/**
 * mediatek_handle_p2p_noa_cmd - Handle P2P NoA command
 * @wpa_s: wpa_supplicant data
 * @cmd: Command string
 * @buf: Buffer for command response
 * @buflen: Length of buffer
 *
 * Returns: 0 on success, negative on failure
 */
static int mediatek_handle_p2p_noa_cmd(struct wpa_supplicant *wpa_s, char *cmd,
                                      char *buf, size_t buflen)
{
    char *argv[MAX_ARGS] = { 0 };
    int argc;
    struct wpa_driver_p2p_noa_params {
        struct wpa_driver_test_mode_info hdr;
        u32 idx;
        u32 value;
        u32 count;
        u32 interval;
        u32 duration;
    } noa_param;

    os_memset(&noa_param, 0, sizeof(noa_param));

    argc = tokenize_cmd(cmd, argv, MAX_ARGS);
    if (argc < 4) {
        wpa_printf(MSG_ERROR, "MediaTek: Invalid P2P NoA command format");
        return -1;
    }

    noa_param.hdr.index = 1;
    noa_param.hdr.index = noa_param.hdr.index | (0x01 << 24);
    noa_param.hdr.buflen = sizeof(noa_param);

    noa_param.idx = 4;
    noa_param.count = (u32)atoi(argv[1]);
    noa_param.interval = (u32)atoi(argv[2]);
    noa_param.duration = (u32)atoi(argv[3]);

    wpa_printf(MSG_DEBUG, "MediaTek: Setting NoA parameters: count=%d interval=%d duration=%d",
             noa_param.count, noa_param.interval, noa_param.duration);

    return mediatek_testmode_send(wpa_s->drv_priv, (u8 *)&noa_param, sizeof(noa_param));
}

/**
 * mediatek_handle_p2p_ps_cmd - Handle P2P power save command
 * @wpa_s: wpa_supplicant data
 * @cmd: Command string
 * @buf: Buffer for command response
 * @buflen: Length of buffer
 *
 * Returns: 0 on success, negative on failure
 */
static int mediatek_handle_p2p_ps_cmd(struct wpa_supplicant *wpa_s, char *cmd,
                                     char *buf, size_t buflen)
{
    char *argv[MAX_ARGS] = { 0 };
    int argc;
    int enable;
    s32 ctw;
    struct wpa_driver_p2p_sigma_params opps_param;

    os_memset(&opps_param, 0, sizeof(opps_param));

    argc = tokenize_cmd(cmd, argv, MAX_ARGS);
    if (argc < 4) {
        wpa_printf(MSG_ERROR, "MediaTek: Invalid P2P power save command format");
        return -1;
    }

    opps_param.hdr.index = 1;
    opps_param.hdr.index = opps_param.hdr.index | (0x01 << 24);
    opps_param.hdr.buflen = sizeof(opps_param);

    opps_param.idx = 107;

    enable = atoi(argv[2]);
    ctw = atoi(argv[3]);

    /* BIT 7 controls opportunistic power save on/off */
    if (enable)
        ctw |= BIT(7);

    opps_param.value = ctw;

    wpa_printf(MSG_DEBUG, "MediaTek: Setting power save: 0x%x", opps_param.value);

    return mediatek_testmode_send(wpa_s->drv_priv, (u8 *)&opps_param, sizeof(opps_param));
}

/**
 * mediatek_handle_suspendmode_cmd - Handle suspend mode command
 * @priv: Private driver data
 * @suspend: Suspend (1) or resume (0)
 *
 * Returns: 0 on success, negative on failure
 */
static int mediatek_handle_suspendmode_cmd(void *priv, int suspend)
{
    struct wpa_driver_suspendmode_params params;

    os_memset(&params, 0, sizeof(params));
    params.hdr.index = NL80211_TESTMODE_SUSPEND;
    params.hdr.index = params.hdr.index | (0x01 << 24);
    params.hdr.buflen = sizeof(params);
    params.suspend = suspend;

    return mediatek_testmode_send(priv, (u8*)&params, sizeof(params));
}

/**
 * mediatek_handle_generic_command - Send a generic command through testmode
 * @ctx: Driver command context
 *
 * Returns: Length of response or negative on error
 */
static int mediatek_handle_generic_command(struct driver_cmd_context *ctx)
{
    u8 buffer[sizeof(struct wpa_driver_cmd_reply_params) + 4096];
    struct wpa_driver_cmd_reply_params *params =
        (struct wpa_driver_cmd_reply_params *)buffer;
    int result;

    params->hdr.index = NL80211_TESTMODE_STR_CMD | (0x01 << 24);
    params->hdr.buflen = sizeof(*params) + os_strlen(ctx->cmd) + 1;
    params->reply_info.reply_buf_size = ctx->buf_len;
    params->reply_info.reply_len = 0;
    params->reply_info.reply_buf.ptr = ctx->buf;
    os_memcpy((char*)(params + 1), ctx->cmd, os_strlen(ctx->cmd) + 1);

    result = mediatek_testmode_send(ctx->bss, buffer, params->hdr.buflen);

    if (result != 0 && params->reply_info.reply_len == 0) {
        os_strlcpy(ctx->buf, "FAILED", ctx->buf_len);
        return os_strlen("FAILED");
    }

    return params->reply_info.reply_len;
}

/**
 * mediatek_process_driver_cmd - Process various driver commands
 * @ctx: Driver command context
 * @handled: Set to 1 if command was handled
 *
 * Returns: Length of response or negative on error
 */
static int mediatek_process_driver_cmd(struct driver_cmd_context *ctx, int *handled)
{
    int ret = 0;

    *handled = 1;

    /* Handle different command types */
    if (os_strncasecmp(ctx->cmd, "POWERMODE ", 10) == 0) {
        int state = atoi(ctx->cmd + 10);
        wpa_printf(MSG_DEBUG, "MediaTek: Setting power mode to %d", state);
        ret = 0; /* Power mode command just returns success */
    }
    else if (os_strncasecmp(ctx->cmd, "MACADDR", os_strlen("MACADDR")) == 0) {
        u8 macaddr[ETH_ALEN] = {0};
        if (ctx->wpa_s)
            os_memcpy(macaddr, ctx->wpa_s->own_addr, ETH_ALEN);

        ret = os_snprintf(ctx->buf, ctx->buf_len,
                       "Macaddr = %02x:%02x:%02x:%02x:%02x:%02x\n",
                       macaddr[0], macaddr[1], macaddr[2],
                       macaddr[3], macaddr[4], macaddr[5]);

        if (ret < 0 || (size_t) ret >= ctx->buf_len) {
            wpa_printf(MSG_ERROR, "MediaTek: MAC address buffer formatting error");
            ret = -1;
        }
    }
    else if (os_strncasecmp(ctx->cmd, "COUNTRY", os_strlen("COUNTRY")) == 0) {
        if (os_strlen(ctx->cmd) != os_strlen("COUNTRY") + 3) {
            wpa_printf(MSG_DEBUG, "MediaTek: Ignoring invalid country code command: %s", ctx->cmd);
            ret = 0;
        } else {
            wpa_printf(MSG_INFO, "MediaTek: Setting country code: %s", ctx->cmd + 8);
            ret = mediatek_set_country_code(ctx->bss, ctx->cmd + 8);

            if (ret == 0 && ctx->wpa_s) {
                wpa_printf(MSG_DEBUG, "MediaTek: Updating channel list for new country code");
                mediatek_notify_country_change(ctx->wpa_s->global, ctx->cmd);
            }
        }
    }
    else if (os_strcasecmp(ctx->cmd, "start") == 0) {
        ret = linux_set_iface_flags(ctx->drv->global->ioctl_sock,
                                  ctx->drv->first_bss->ifname, 1);
        if (ret != 0) {
            wpa_printf(MSG_ERROR, "MediaTek: Failed to set interface UP, ret=%d", ret);
        } else {
            wpa_msg(ctx->drv->ctx, MSG_INFO, "CTRL-EVENT-DRIVER-STATE STARTED");
        }
    }
    else if (os_strcasecmp(ctx->cmd, "stop") == 0) {
        if (ctx->drv->associated && ctx->wpa_s) {
            ret = wpa_drv_deauthenticate(ctx->wpa_s, ctx->drv->bssid,
                                       WLAN_REASON_DEAUTH_LEAVING);
            if (ret != 0)
                wpa_printf(MSG_ERROR, "MediaTek: Deauthentication failed, ret=%d", ret);
        }

        ret = linux_set_iface_flags(ctx->drv->global->ioctl_sock,
                                  ctx->drv->first_bss->ifname, 0);
        if (ret != 0) {
            wpa_printf(MSG_ERROR, "MediaTek: Failed to set interface DOWN, ret=%d", ret);
        } else {
            wpa_msg(ctx->drv->ctx, MSG_INFO, "CTRL-EVENT-DRIVER-STATE STOPPED");
        }
    }
    else if (os_strncasecmp(ctx->cmd, "getpower", 8) == 0) {
        u32 mode = 0;
        ret = os_snprintf(ctx->buf, ctx->buf_len, "powermode = %u\n", mode);
        if (ret < 0 || (size_t) ret >= ctx->buf_len) {
            wpa_printf(MSG_ERROR, "MediaTek: Power mode buffer formatting error");
            ret = -1;
        }
    }
    else if (os_strncasecmp(ctx->cmd, "rxfilter-add", 12) == 0) {
        u32 sw_cmd = 0x9F000000;
        u32 idx = 0;
        char *cp = ctx->cmd + 12;
        char *endp;

        if (*cp != '\0') {
            idx = (u32)strtol(cp, &endp, 0);
            if (endp != cp) {
                idx += 0x00900200;
                mediatek_send_sw_cmd(ctx->bss, 1, &sw_cmd, &idx);
                ret = 0;
            }
        }
    }
    else if (os_strncasecmp(ctx->cmd, "rxfilter-remove", 15) == 0) {
        u32 sw_cmd = 0x9F000000;
        u32 idx = 0;
        char *cp = ctx->cmd + 15;
        char *endp;

        if (*cp != '\0') {
            idx = (u32)strtol(cp, &endp, 0);
            if (endp != cp) {
                idx += 0x00900300;
                mediatek_send_sw_cmd(ctx->bss, 1, &sw_cmd, &idx);
                ret = 0;
            }
        }
    }
    else if (os_strncasecmp(ctx->cmd, "rxfilter-stop", 13) == 0) {
        u32 sw_cmd = 0x9F000000;
        u32 idx = 0x00900000;
        mediatek_send_sw_cmd(ctx->bss, 1, &sw_cmd, &idx);
        ret = 0;
    }
    else if (os_strncasecmp(ctx->cmd, "rxfilter-start", 14) == 0) {
        u32 sw_cmd = 0x9F000000;
        u32 idx = 0x00900100;
        mediatek_send_sw_cmd(ctx->bss, 1, &sw_cmd, &idx);
        ret = 0;
    }
    else if (os_strcasecmp(ctx->cmd, "btcoexscan-start") == 0 ||
             os_strcasecmp(ctx->cmd, "btcoexscan-stop") == 0 ||
             os_strncasecmp(ctx->cmd, "btcoexmode", 10) == 0) {
        /* Just return success as these features aren't implemented */
        ret = 0;
    }
    else if (os_strncmp(ctx->cmd, "MIRACAST ", os_strlen("MIRACAST ")) == 0) {
        /* Pass this to driver directly */
        *handled = 0;
    }
    else if (os_strncmp(ctx->cmd, "HAPD_GET_CHANNEL ", os_strlen("HAPD_GET_CHANNEL ")) == 0) {
        /* Pass this to driver directly via ioctl */
        ret = mediatek_send_command(ctx, ctx->cmd);
    }
    else if (os_strncmp(ctx->cmd, "P2P_SET_NOA", os_strlen("P2P_SET_NOA")) == 0) {
        if (ctx->wpa_s)
            ret = mediatek_handle_p2p_noa_cmd(ctx->wpa_s, ctx->cmd, ctx->buf, ctx->buf_len);
    }
    else if (os_strncmp(ctx->cmd, "P2P_SET_PS", os_strlen("P2P_SET_PS")) == 0) {
        if (ctx->wpa_s)
            ret = mediatek_handle_p2p_ps_cmd(ctx->wpa_s, ctx->cmd, ctx->buf, ctx->buf_len);
    }
    else if (os_strncasecmp(ctx->cmd, "SETSUSPENDMODE ", 15) == 0) {
        int suspend = *(ctx->cmd+15)-'0';
        ret = mediatek_handle_suspendmode_cmd(ctx->bss, suspend);
        *handled = 0; /* Allow other drivers to handle this command too */
    }
    else {
        /* Pass other commands via testmode command interface */
        ret = mediatek_handle_generic_command(ctx);
        *handled = (ret >= 0);
    }

    return ret;
}

/**
 * wpa_driver_nl80211_driver_cmd - Execute driver-specific commands
 * @priv: Private driver interface data
 * @cmd: Command to be executed
 * @buf: Return buffer
 * @buf_len: Buffer length
 *
 * Returns: 0 for success, >0 for length of returned data, negative on error
 */
int wpa_driver_nl80211_driver_cmd(void *priv, char *cmd, char *buf, size_t buf_len)
{
    struct driver_cmd_context ctx;
    int handled = 0;
    int ret;

    /* Initialize command context */
    if (init_driver_cmd_context(&ctx, priv, cmd, buf, buf_len) < 0) {
        wpa_printf(MSG_ERROR, "MediaTek: Failed to initialize driver command context");
        return -1;
    }

    wpa_printf(MSG_INFO, "MediaTek: Interface %s received command: %s",
              ctx.bss->ifname, cmd);

    /* Process driver-specific commands */
    ret = mediatek_process_driver_cmd(&ctx, &handled);

    /* If not handled by our custom handlers, pass to driver via ioctl */
    if (!handled) {
        if (ctx.cmd_len >= PRIV_CMD_SIZE) {
            wpa_printf(MSG_ERROR, "MediaTek: Command too long: %s", cmd);
            return -1;
        }

        os_strlcpy(ctx.ifr.ifr_name, ctx.bss->ifname, IFNAMSIZ);
        os_memcpy(ctx.priv_cmd.buf, cmd, ctx.cmd_len + 1);
        ctx.priv_cmd.used_len = ctx.cmd_len + 1;
        ctx.priv_cmd.total_len = PRIV_CMD_SIZE;
        ctx.ifr.ifr_data = &ctx.priv_cmd;

        ret = ioctl(ctx.drv->global->ioctl_sock, SIOCDEVPRIVATE + 1, &ctx.ifr);
        if (ret < 0) {
            wpa_printf(MSG_ERROR, "MediaTek: Private command failed: %s, error: %s",
                     cmd, strerror(errno));
            track_driver_error();
            return -1;
        } else {
            reset_error_counter();

            if ((os_strncasecmp(cmd, "WLS_BATCHING", 12) == 0)) {
                /* Ensure null termination for batching command */
                buf[buf_len - 1] = '\0';
                ret = os_strlen(buf);
            } else {
                /* Copy response data from driver to our buffer */
                os_strlcpy(buf, ctx.priv_cmd.buf, buf_len);
                ret = os_strlen(buf);
            }
        }
    }

    return ret;
}

/**
 * wpa_driver_set_p2p_noa - Set P2P NoA parameters
 * @priv: Private driver interface data
 * @count: NoA count
 * @start: NoA start time
 * @duration: NoA duration
 *
 * Returns: 0 on success, -1 on failure
 */
int wpa_driver_set_p2p_noa(void *priv, u8 count, int start, int duration)
{
    char cmd[64];
    char buf[64];
    struct i802_bss *bss = priv;

    wpa_printf(MSG_DEBUG, "MediaTek: Interface %s: P2P_SET_NOA count=%d start=%d duration=%d",
              bss->ifname, count, start, duration);

    os_snprintf(cmd, sizeof(cmd), "P2P_SET_NOA %d %d %d", count, start, duration);
    return wpa_driver_nl80211_driver_cmd(priv, cmd, buf, sizeof(buf));
}

/**
 * wpa_driver_get_p2p_noa - Get P2P NoA parameters
 * @priv: Private driver interface data
 * @buf: Buffer to receive NoA parameters
 * @len: Buffer length
 *
 * Returns: 0 on success, -1 on failure
 */
int wpa_driver_get_p2p_noa(void *priv, u8 *buf, size_t len)
{
    struct i802_bss *bss = priv;

    wpa_printf(MSG_DEBUG, "MediaTek: Interface %s: P2P_GET_NOA (not implemented)", bss->ifname);
    return -1; /* Not implemented */
}

/**
 * wpa_driver_set_p2p_ps - Set P2P power save parameters
 * @priv: Private driver interface data
 * @legacy_ps: Legacy power save mode
 * @opp_ps: Opportunistic power save enabled
 * @ctwindow: CT window
 *
 * Returns: 0 on success, -1 on failure
 */
int wpa_driver_set_p2p_ps(void *priv, int legacy_ps, int opp_ps, int ctwindow)
{
    char cmd[64];
    char buf[64];
    struct i802_bss *bss = priv;

    wpa_printf(MSG_DEBUG, "MediaTek: Interface %s: P2P_SET_PS legacy=%d opp=%d ctwindow=%d",
              bss->ifname, legacy_ps, opp_ps, ctwindow);

    os_snprintf(cmd, sizeof(cmd), "P2P_SET_PS %d %d %d", legacy_ps, opp_ps, ctwindow);
    return wpa_driver_nl80211_driver_cmd(priv, cmd, buf, sizeof(buf));
}

/**
 * wpa_driver_set_ap_wps_p2p_ie - Set WPS/P2P IEs for AP mode
 * @priv: Private driver interface data
 * @beacon: Beacon IE
 * @proberesp: Probe Response IE
 * @assocresp: Association Response IE
 *
 * Returns: 0 on success, -1 on failure
 */
int wpa_driver_set_ap_wps_p2p_ie(void *priv, const struct wpabuf *beacon,
                                const struct wpabuf *proberesp,
                                const struct wpabuf *assocresp)
{
    struct i802_bss *bss = priv;

    wpa_printf(MSG_DEBUG, "MediaTek: Interface %s: set_ap_wps_p2p_ie", bss->ifname);
    return 0; /* Function stubbed out, actual functionality not needed */
}

/**
 * nl80211_vendor_event_mtk - Process MediaTek vendor events
 * @drv: Driver private data
 * @subcmd: Vendor subcmd
 * @data: Event data
 * @len: Data length
 */
void nl80211_vendor_event_mtk(struct wpa_driver_nl80211_data *drv,
                             u32 subcmd, u8 *data, size_t len)
{
    switch (subcmd) {
    case WIFI_EVENT_DRIVER_ERROR:
        wpa_printf(MSG_INFO, "MediaTek: Driver error notification received");
        break;
    case WIFI_EVENT_GENERIC_RESPONSE:
        wpa_printf(MSG_DEBUG, "MediaTek: Generic driver response received");
        break;
    default:
        wpa_printf(MSG_DEBUG, "MediaTek: Ignoring unsupported vendor event %u", subcmd);
        break;
    }
}
