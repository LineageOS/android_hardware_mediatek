/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include "includes.h"
#include "netlink/genl/genl.h"

#include "common.h"
#include "config.h"
#include "driver_nl80211.h"
#include "wpa_supplicant_i.h"
#ifdef ANDROID
#include "android_drv.h"
#endif

#include "driver_i.h"
#include "mediatek_driver_nl80211.h"

#include "eloop.h"
#define PRIV_CMD_SIZE 512
#define CMD_SIZE 4098

#ifdef MAC2STR
#undef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[5]
#endif

#ifdef MACSTR
#undef MACSTR
#define MACSTR "%02x:%02x:*:*:*:%02x"
#endif

typedef struct android_wifi_priv_cmd {
    char buf[PRIV_CMD_SIZE];
    int used_len;
    int total_len;
} android_wifi_priv_cmd;

static int vendor_cmd_reply_info_handler(struct nl_msg* msg, void* arg) {
    struct nlattr* tb[NL80211_ATTR_MAX + 1] = {NULL};
    struct genlmsghdr* gnlh = nlmsg_data(nlmsg_hdr(msg));
    struct wpa_driver_cmd_reply_s* reply_info = (struct wpa_driver_cmd_reply_s*)arg;
    int reply_len = 0;

    nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0), genlmsg_attrlen(gnlh, 0), NULL);

    if (!tb[NL80211_ATTR_VENDOR_DATA]) return NL_SKIP;

    strncpy(reply_info->reply_buf.ptr, nla_data(tb[NL80211_ATTR_VENDOR_DATA]),
            reply_info->reply_buf_size - 1);
    (reply_info->reply_buf.ptr)[reply_info->reply_buf_size - 1] = '\0';
    reply_info->reply_len = strlen(reply_info->reply_buf.ptr);

    return NL_SKIP;
}

static int mtk_string_vendor_cmd(void* priv, const char* cmd,
                                 struct wpa_driver_cmd_reply_s* reply_info) {
    struct i802_bss* bss = priv;
    struct wpa_driver_nl80211_data* drv = bss->drv;
    struct nl_msg* msg;
    struct nlattr* params;
    int ret;

    wpa_printf(MSG_DEBUG, "mtk string command: %s", cmd);

    if (!(msg = nl80211_drv_msg(drv, 0, NL80211_CMD_VENDOR)) ||
        nla_put_u32(msg, NL80211_ATTR_VENDOR_ID, OUI_MTK) ||
        nla_put_u32(msg, NL80211_ATTR_VENDOR_SUBCMD, MTK_SUBCMD_STRING_CMD)) {
        wpa_printf(MSG_DEBUG, "nl operation error");
        goto fail;
    }

    params = nla_nest_start(msg, NL80211_ATTR_VENDOR_DATA);
    if (!params) {
        wpa_printf(MSG_DEBUG, "nl start failed");
        goto fail;
    }

    if (nla_put_string(msg, STRING_ATTRIBUTE_DATA, cmd)) {
        wpa_printf(MSG_DEBUG, "nl put attribute failed");
        goto fail;
    }
    nla_nest_end(msg, params);

    ret = send_and_recv_msgs(drv, msg, vendor_cmd_reply_info_handler, reply_info, NULL, NULL);
    msg = NULL;
    if (ret) {
        wpa_printf(MSG_DEBUG, "str_vendor_cmd ret=%d", ret);
        return ret;
    }
    return 0;
fail:
    nlmsg_free(msg);
    return -1;
}

static int wpa_driver_nl80211_testmode(void* priv, const u8* data, size_t data_len) {
    struct i802_bss* bss = priv;
    struct wpa_driver_nl80211_data* drv = bss->drv;
    struct nl_msg* msg;
    int ret;

    msg = nlmsg_alloc();
    if (!msg) return -1;

    wpa_printf(MSG_DEBUG, "nl80211 test mode: ifindex=%d", drv->ifindex);

    nl80211_cmd(drv, msg, 0, NL80211_CMD_TESTMODE);

    NLA_PUT_U32(msg, NL80211_ATTR_IFINDEX, drv->ifindex);
    NLA_PUT(msg, NL80211_ATTR_TESTDATA, data_len, data);

    ret = send_and_recv_msgs(drv, msg, NULL, NULL, NULL, NULL);
    wpa_printf(MSG_EXCESSIVE, "ret=%d, nl=%p", ret, drv->global->nl);
    return ret;

nla_put_failure:
    nlmsg_free(msg);
    return -ENOBUFS;
}

static int wpa_driver_nl80211_driver_sw_cmd(void* priv, int set, u32* adr, u32* dat) {
    struct i802_bss* bss = priv;
    struct wpa_driver_nl80211_data* drv = bss->drv;
    struct wpa_driver_sw_cmd_params params;
    struct nl_msg *msg, *cqm = NULL;
    int ret = 0;

    os_memset(&params, 0, sizeof(params));

    params.hdr.index = NL80211_TESTMODE_SW_CMD;
    params.hdr.index = params.hdr.index | (0x01 << 24);
    params.hdr.buflen = sizeof(struct wpa_driver_sw_cmd_params);

    params.adr = *adr;
    params.data = *dat;

    if (set)
        params.set = 1;
    else
        params.set = 0;

    wpa_driver_nl80211_testmode(priv, (u8*)&params, sizeof(struct wpa_driver_sw_cmd_params));
    return 0;
}

/**********************************************************************
 * OVERLAPPED functins, previous defination is in driver_nl80211.c,
 * it will be modified
 ***********************************************************************/

/**********************************************************************/
static int wpa_driver_mediatek_set_country(void* priv, const char* alpha2_arg) {
    struct i802_bss* bss = priv;
    struct wpa_driver_nl80211_data* drv = bss->drv;
    int ioctl_sock = -1;
    struct iwreq iwr;
    int ret = -1;
    char buf[11];

    ioctl_sock = socket(PF_INET, SOCK_DGRAM, 0);
    if (ioctl_sock < 0) {
        wpa_printf(MSG_ERROR, "%s: socket(PF_INET,SOCK_DGRAM)", __func__);
        return -1;
    }
    os_memset(&iwr, 0, sizeof(iwr));
    os_strlcpy(iwr.ifr_name, drv->first_bss->ifname, IFNAMSIZ);
    ret = snprintf(buf, sizeof(buf), "COUNTRY %s", alpha2_arg);
    if (ret < 0 || ret > sizeof(buf)) {
        wpa_printf(MSG_DEBUG, "%s: snprintf failed:%d", __func__, ret);
    }
    iwr.u.data.pointer = buf;
    iwr.u.data.length = strlen(buf);
    if ((ret = ioctl(ioctl_sock, 0x8B0C, &iwr)) < 0) {  // SIOCSIWPRIV
        wpa_printf(MSG_DEBUG, "ioctl[SIOCSIWPRIV]: %s", buf);
        close(ioctl_sock);
        return ret;
    } else {
        close(ioctl_sock);
        return 0;
    }
}

/*
 * update channel list in wpa_supplicant
 * if coutry code chanaged
 */
static void wpa_driver_notify_country_change(struct wpa_global* global, char* cmd) {
    struct wpa_supplicant* wpa_s;

    if (os_strncasecmp(cmd, "COUNTRY", 7) == 0) {
        union wpa_event_data event;

        os_memset(&event, 0, sizeof(event));
        event.channel_list_changed.initiator = REGDOM_SET_BY_USER;
        if (os_strncasecmp(cmd, "COUNTRY", 7) == 0) {
            event.channel_list_changed.type = REGDOM_TYPE_COUNTRY;
            if (os_strlen(cmd) > 9) {
                event.channel_list_changed.alpha2[0] = cmd[8];
                event.channel_list_changed.alpha2[1] = cmd[9];
            }
        } else {
            event.channel_list_changed.type = REGDOM_TYPE_UNKNOWN;
        }
        // Notify all interfaces
        for (wpa_s = global->ifaces; wpa_s; wpa_s = wpa_s->next) {
            wpa_supplicant_event(wpa_s, EVENT_CHANNEL_LIST_CHANGED, &event);
        }
    }
}

/* utils for parse cmdline:
 * cmd: paramters in cmd line
 * argv: paramter vector
 * len: cmd lenght
 * example:
 * cmd = "driver P2P_SET_NOA 1 2 3"
 * argv[0] = "driver"
 * argv[1] = "P2P_SET_NOA"
 * argv[2] = "1"
 * argv[3] = "2"
 * argv[4] = "3"
 */
int tokenize_space(char* cmd, char* argv[], int len) {
    char* pos;
    char* start;
    int argc = 0;

    start = pos = cmd;
    for (;;) {
        argv[argc] = pos;
        argc++;
        while (*pos != '\n' && *pos != ' ' && *pos != '\0') {
            pos++;
            if (pos - start >= len) break;
        }

        if (*pos == '\0') break;

        if (*pos == '\n' || *pos == ' ') {
            *pos++ = '\0';
            if (pos - start >= len) break;
        }
    }

    return argc;
}

static int p2p_ctrl_iface_set_noa(struct wpa_supplicant* wpa_s, char* cmd, char* buf,
                                  size_t buflen) {
    struct wpa_driver_p2p_noa_params {
        struct wpa_driver_test_mode_info hdr;
        u32 idx;
        u32 value; /* should not be used in this case */
        u32 count;
        u32 interval;
        u32 duration;
    };
    char* argv[64] = {0};
    int argc;
    struct wpa_driver_p2p_noa_params noa_param;

    os_memset(&noa_param, 0, sizeof(noa_param));

    /* P2P_SET_NOA 255 100 3 */
    /*
     * argv format:
     * argv[0] = "P2P_SET_NOA"
     * argv[1] = "255"
     * argv[2] = "100"
     * argv[3] = "3"
     */
    argc = tokenize_space(cmd, argv, os_strlen(cmd));

    if (argc != 4) {
        wpa_printf(MSG_DEBUG, "P2P: NOA: invalid cmd format");
        return -1;
    }

    /* fill in the params structure */
    noa_param.hdr.index = 1;
    noa_param.hdr.index = noa_param.hdr.index | (0x01 << 24);
    noa_param.hdr.buflen = sizeof(struct wpa_driver_p2p_noa_params);

    noa_param.idx = 4;
    noa_param.count = (u32)atoi(argv[1]);
    noa_param.interval = (u32)atoi(argv[2]);
    noa_param.duration = (u32)atoi(argv[3]);

    wpa_printf(MSG_DEBUG, "P2P: set noa: %d %d %d", noa_param.count, noa_param.interval,
               noa_param.duration);

    return wpa_driver_nl80211_testmode(wpa_s->drv_priv, (u8*)&noa_param,
                                       sizeof(struct wpa_driver_p2p_noa_params));
}

static int p2p_ctrl_iface_set_ps(struct wpa_supplicant* wpa_s, char* cmd, char* buf,
                                 size_t buflen) {
    char* argv[64] = {0};
    int argc;
    int enable;
    s32 ctw;
    struct wpa_driver_p2p_sigma_params opps_param;

    os_memset(&opps_param, 0, sizeof(opps_param));

    /* P2P_SET_PS 2 1 3
     * argv format:
     * argv[0] = "P2P_SET_PS"
     * argv[1] = "2"
     * argv[2] = "1"
     * argv[3] = "3"
     */
    argc = tokenize_space(cmd, argv, os_strlen(cmd));

    if (argc != 4) {
        wpa_printf(MSG_DEBUG, "P2P: Opps: invalid cmd format");
        return -1;
    }

    /* fill in the params structure */
    opps_param.hdr.index = 1;
    opps_param.hdr.index = opps_param.hdr.index | (0x01 << 24);
    opps_param.hdr.buflen = sizeof(struct wpa_driver_p2p_sigma_params);

    opps_param.idx = 107;

    enable = atoi(argv[2]);
    ctw = atoi(argv[3]);

    /* BIT 7 control OPPS on / off */
    if (enable) ctw |= BIT(7);

    opps_param.value = ctw;

    wpa_printf(MSG_DEBUG, "P2P: set opps: 0x%x", opps_param.value);

    return wpa_driver_nl80211_testmode(wpa_s->drv_priv, (u8*)&opps_param,
                                       sizeof(struct wpa_driver_p2p_sigma_params));
}

int wpa_driver_nl80211_driver_cmd(void* priv, char* cmd_src, char* buf, size_t buf_len) {
    struct i802_bss* bss = priv;
    struct wpa_driver_nl80211_data* drv = bss->drv;
    struct ifreq ifr;
    android_wifi_priv_cmd priv_cmd;
    struct wpa_supplicant* wpa_s = NULL;
    int handled = 0;
    int cmd_len = 0;
    int ret = -1;
    char cmd[CMD_SIZE] = {0};

    if (drv == NULL) {
        wpa_printf(MSG_ERROR, "%s: drv is NULL, exit", __func__);
        return -1;
    }
    if (drv->ctx == NULL) {
        wpa_printf(MSG_ERROR, "%s: drv->ctx is NULL, exit", __func__);
        return -1;
    }

    if (bss->drv->nlmode != NL80211_IFTYPE_AP) {
        wpa_s = (struct wpa_supplicant*)(drv->ctx);
        if (wpa_s->conf == NULL) {
            wpa_printf(MSG_ERROR, "%s: wpa_s->conf is NULL, exit", __func__);
            return -1;
        }
    }

    os_memcpy(cmd, cmd_src, strlen(cmd_src) > CMD_SIZE ? CMD_SIZE : strlen(cmd_src));
    cmd[CMD_SIZE - 1] = '\0';

    wpa_printf(MSG_INFO, "%s: %s recv cmd %s", __func__, bss->ifname, cmd);
    handled = 1;

    if (os_strncmp(cmd, "MACADDR", os_strlen("MACADDR")) == 0) {
        u8 macaddr[ETH_ALEN] = {};
        if (wpa_s) os_memcpy(&macaddr, wpa_s->own_addr, ETH_ALEN);
        ret = snprintf(buf, buf_len, "Macaddr = %02x:%02x:%02x:%02x:%02x:%02x\n", macaddr[0],
                       macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
        if (ret < 0 || ret > buf_len) {
            wpa_printf(MSG_DEBUG, "snprintf failed");
        } else {
            wpa_printf(MSG_DEBUG, "Macaddr = " MACSTR, MAC2STR(macaddr));
        }
    } else if (os_strncasecmp(cmd, "COUNTRY", os_strlen("COUNTRY")) == 0) {
        if (os_strlen(cmd) != os_strlen("COUNTRY") + 3) {
            wpa_printf(MSG_DEBUG, "Ignore COUNTRY cmd %s", cmd);
            ret = 0;
        } else {
            wpa_printf(MSG_INFO, "Set country: %s", cmd + 8);
            ret = wpa_driver_mediatek_set_country(priv, cmd + 8);
            if (ret == 0 && wpa_s) {
                wpa_printf(MSG_DEBUG, "Update channel list after country code changed");
                wpa_driver_notify_country_change(wpa_s->global, cmd);
            }
        }
    } else if (os_strncasecmp(cmd, "rxfilter-add", 12) == 0) {
        u32 sw_cmd = 0x9F000000;
        u32 idx = 0;
        char* cp = cmd + 12;
        char* endp;

        if (*cp != '\0') {
            idx = (u32)strtol(cp, &endp, 0);
            if (endp != cp) {
                idx += 0x00900200;
                wpa_driver_nl80211_driver_sw_cmd(priv, 1, &sw_cmd, &idx);
                ret = 0;
            }
        }
    } else if (os_strncasecmp(cmd, "rxfilter-remove", 15) == 0) {
        u32 sw_cmd = 0x9F000000;
        u32 idx = 0;
        char* cp = cmd + 15;
        char* endp;

        if (*cp != '\0') {
            idx = (u32)strtol(cp, &endp, 0);
            if (endp != cp) {
                idx += 0x00900300;
                wpa_driver_nl80211_driver_sw_cmd(priv, 1, &sw_cmd, &idx);
                ret = 0;
            }
        }
    } else if (os_strncasecmp(cmd, "rxfilter-stop", 13) == 0) {
        u32 sw_cmd = 0x9F000000;
        u32 idx = 0x00900000;
        wpa_driver_nl80211_driver_sw_cmd(priv, 1, &sw_cmd, &idx);
        ret = 0;
    } else if (os_strncasecmp(cmd, "rxfilter-start", 14) == 0) {
        u32 sw_cmd = 0x9F000000;
        u32 idx = 0x00900100;
        wpa_driver_nl80211_driver_sw_cmd(priv, 1, &sw_cmd, &idx);
        ret = 0;
    } else if (os_strcasecmp(cmd, "btcoexscan-start") == 0) {
        ret = 0; /* mt5921 linux driver not implement yet */
    } else if (os_strcasecmp(cmd, "btcoexscan-stop") == 0) {
        ret = 0; /* mt5921 linux driver not implement yet */
    } else if (os_strncasecmp(cmd, "btcoexmode", 10) == 0) {
        ret = 0; /* mt5921 linux driver not implement yet */
    } else if (os_strncmp(cmd, "P2P_SET_NOA", os_strlen("P2P_SET_NOA")) == 0) {
        if (wpa_s) ret = p2p_ctrl_iface_set_noa(wpa_s, cmd, buf, buf_len);
    } else if (os_strncmp(cmd, "P2P_SET_PS", os_strlen("P2P_SET_PS")) == 0) {
        if (wpa_s) ret = p2p_ctrl_iface_set_ps(wpa_s, cmd, buf, buf_len);
    } else if (os_strncasecmp(cmd, "SETSUSPENDMODE ", 15) == 0) {
        struct wpa_driver_suspendmode_params params;
        params.hdr.index = NL80211_TESTMODE_SUSPEND;
        params.hdr.index = params.hdr.index | (0x01 << 24);
        params.hdr.buflen = sizeof(params);
        params.suspend = *(cmd + 15) - '0';
        wpa_driver_nl80211_testmode(priv, (u8*)&params, sizeof(params));
        handled =
                0; /* 6630 driver handled this command in driver, so give a chance to 6630 driver */
    } else {
        u8 data[4096];
        int result;
        struct wpa_driver_cmd_reply_s* reply_info = (struct wpa_driver_cmd_reply_s*)data;

        memset(reply_info, 0, sizeof(data));
        reply_info->reply_buf_size = buf_len;
        reply_info->reply_buf.ptr = buf;
        result = mtk_string_vendor_cmd(priv, cmd, reply_info);
        ret = reply_info->reply_len;
        handled = result != -EOPNOTSUPP;

        if (handled && result != 0 && ret == 0) {
            strncpy(buf, "FAILED", buf_len - 1);
            ret = strlen("FAILED");
        }

        wpa_printf(MSG_INFO,
                   "Transparent command for driver nl80211, result %d, reply_len=%d, reply=%s "
                   "handled=%d",
                   result, ret, buf, handled);
    }

    if (handled == 0) {
        int str_len = 0;
        cmd_len = strlen(cmd);

        memset(&ifr, 0, sizeof(ifr));
        memset(&priv_cmd, 0, sizeof(priv_cmd));
        memset(buf, 0, buf_len);
        strncpy(ifr.ifr_name, bss->ifname, IFNAMSIZ);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (cmd_len >= PRIV_CMD_SIZE) {
            wpa_printf(MSG_INFO, "%s: cmd: %s overflow", __func__, cmd);
            cmd_len = PRIV_CMD_SIZE - 1;
        }

        memcpy(priv_cmd.buf, cmd, cmd_len);
        priv_cmd.used_len = cmd_len + 1;
        priv_cmd.total_len = PRIV_CMD_SIZE;
        ifr.ifr_data = &priv_cmd;

        ret = ioctl(drv->global->ioctl_sock, SIOCDEVPRIVATE + 1, &ifr);
        if (ret < 0) {
            wpa_printf(MSG_ERROR,
                       "%s: failed to issue private commands,"
                       " error msg: %s\n",
                       __func__, strerror(errno));
            ret = snprintf(buf, buf_len, "%s\n", "FAIL");
        } else {
            wpa_printf(MSG_INFO, "%s: ret = %d used = %u total = %u buf = %s", __func__, ret,
                       priv_cmd.used_len, priv_cmd.total_len, priv_cmd.buf);

            str_len = strlen(priv_cmd.buf);
            memcpy(buf, priv_cmd.buf, str_len + 1);
            ret = strlen(buf);
        }
    } /* handled == 0 */

    return ret;
}

int wpa_driver_set_p2p_noa(void* priv, u8 count, int start, int duration) {
    int ret;
    char buf[MAX_DRV_CMD_SIZE];
    struct i802_bss* bss = priv;
    struct wpa_driver_nl80211_data* drv = bss->drv;

    wpa_printf(MSG_DEBUG, "iface %s P2P_SET_NOA %d %d %d", bss->ifname, count, start, duration);
    ret = snprintf(buf, sizeof(buf), "P2P_SET_NOA %d %d %d", count, start, duration);
    if (ret < 0 || ret >= MAX_DRV_CMD_SIZE)
        return -1;
    else
        return wpa_driver_nl80211_driver_cmd(priv, buf, buf, strlen(buf) + 1);
}

int wpa_driver_get_p2p_noa(void* priv, u8* buf, size_t len) {
    struct i802_bss* bss = priv;
    struct wpa_driver_nl80211_data* drv = bss->drv;

    wpa_printf(MSG_DEBUG, "iface %s P2P_GET_NOA, ignored", bss->ifname);
    return -1;
}

int wpa_driver_set_p2p_ps(void* priv, int legacy_ps, int opp_ps, int ctwindow) {
    int ret;
    char buf[MAX_DRV_CMD_SIZE];
    struct i802_bss* bss = priv;
    struct wpa_driver_nl80211_data* drv = bss->drv;

    wpa_printf(MSG_DEBUG, "iface %s P2P_SET_PS %d %d %d", bss->ifname, legacy_ps, opp_ps, ctwindow);
    ret = snprintf(buf, sizeof(buf), "P2P_SET_PS %d %d %d", legacy_ps, opp_ps, ctwindow);
    if (ret < 0 || ret >= MAX_DRV_CMD_SIZE)
        return -1;
    else
        return wpa_driver_nl80211_driver_cmd(priv, buf, buf, strlen(buf) + 1);
}

int wpa_driver_set_ap_wps_p2p_ie(void* priv, const struct wpabuf* beacon,
                                 const struct wpabuf* proberesp, const struct wpabuf* assocresp) {
    struct i802_bss* bss = priv;
    struct wpa_driver_nl80211_data* drv = bss->drv;

    wpa_printf(MSG_DEBUG, "iface %s set_ap_wps_p2p_ie, ignored", bss->ifname);
    return 0;
}
