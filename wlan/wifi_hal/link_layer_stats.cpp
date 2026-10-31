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
#include <netlink/handlers.h>

#include "sync.h"

#define LOG_TAG  "WifiHAL"

#include <utils/Log.h>

#include <hardware_legacy/wifi_hal.h>
#include "common.h"
#include "cpp_bindings.h"

typedef enum {
    ANDR_WIFI_ATTRIBUTE_INVALID        = 0,
    ANDR_WIFI_ATTRIBUTE_NUM_RADIO      = 1,
    ANDR_WIFI_ATTRIBUTE_STATS_INFO     = 2,
    ANDR_WIFI_ATTRIBUTE_ML_STATS_INFO  = 3,
    ANDR_WIFI_ATTRIBUTE_STATS_MAX      = 4
} LINK_STAT_ATTRIBUTE;

/* Internal radio statistics structure in the driver */
typedef struct {
	wifi_radio radio;
	uint32_t on_time;
	uint32_t tx_time;
	uint32_t rx_time;
	uint32_t on_time_scan;
	uint32_t on_time_nbd;
	uint32_t on_time_gscan;
	uint32_t on_time_roam_scan;
	uint32_t on_time_pno_scan;
	uint32_t on_time_hs20;
	uint32_t num_channels;
	wifi_channel_stat channels[];
} wifi_radio_stat_internal;

enum {
    LSTATS_SUBCMD_GET_INFO = ANDROID_NL80211_SUBCMD_LSTATS_RANGE_START,
};

class GetLinkStatsCommand : public WifiCommand
{
    wifi_stats_result_handler mHandler;
public:
    GetLinkStatsCommand(wifi_interface_handle iface, wifi_stats_result_handler handler)
        : WifiCommand("GetLinkStatsCommand", iface, 0), mHandler(handler)
    { }

    virtual int create() {
        ALOGI("Creating message to get link statistics; iface = %d", mIfaceInfo->id);

        int ret = mMsg.create(GOOGLE_OUI, LSTATS_SUBCMD_GET_INFO);
        if (ret < 0) {
            ALOGE("Failed to create %x - %d", LSTATS_SUBCMD_GET_INFO, ret);
            return ret;
        }

        return ret;
    }

protected:
    virtual int handleResponse(WifiEvent& reply) {
        bool ml_data = false;
        wifi_iface_ml_stat *iface_ml_stat_ptr = NULL;
        u8 *radioStatsBuf = NULL, *ifaceMlStatsBuf = NULL, *outbuf = NULL, *data_ptr = NULL;
        u8 *data = NULL, *iface_stat = NULL;
        uint32_t offset = 0, num_radios = 0, per_radio_size = 0, data_len = 0;
        uint32_t outbuf_rem_len = 0, data_rem_len = 0;
        int ret = 0, id = 0, subcmd = 0, len = 0;
        u32 fixed_iface_ml_stat_size = 0, all_links_stat_size = 0;
        u8 num_links = 0;

        // ALOGI("In GetLinkStatsCommand::handleResponse");

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        id = reply.get_vendor_id();
        subcmd = reply.get_vendor_subcmd();
        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        len = reply.get_vendor_data_len();

        ALOGV("Id = %0x, subcmd = %d, len = %d\n", id, subcmd, len);
        if (vendor_data == NULL || len == 0) {
            ALOGE("no vendor data in GetLinkStatCommand response; ignoring it");
            return NL_SKIP;
        }

        for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
            if (it.get_type() == ANDR_WIFI_ATTRIBUTE_NUM_RADIO) {
                num_radios = it.get_u32();
            } else if (it.get_type() == ANDR_WIFI_ATTRIBUTE_STATS_INFO) {
                data = (u8 *)it.get_data();
                data_len = it.get_len();
            } else if (it.get_type() == ANDR_WIFI_ATTRIBUTE_ML_STATS_INFO) {
                data = (u8 *)it.get_data();
                data_len = it.get_len();
                ml_data = true;
            } else {
                ALOGW("Ignoring invalid attribute type = %d, size = %d",
                it.get_type(), it.get_len());
                return NL_SKIP;
            }
        }

        if (num_radios > MAX_NUM_RADIOS) {
            ALOGE("Invalid num radios :%d\n", num_radios);
            ret = WIFI_ERROR_INVALID_ARGS;
            goto exit;
        }

        outbuf_rem_len = MAX_CMD_RESP_BUF_LEN;
        radioStatsBuf = (u8 *)malloc(MAX_CMD_RESP_BUF_LEN);
        if (!radioStatsBuf) {
            ALOGE("No memory\n");
            return NL_SKIP;
        }
        memset(radioStatsBuf, 0, MAX_CMD_RESP_BUF_LEN);
        outbuf = radioStatsBuf;

        if (!data || !data_len) {
            ALOGE("%s: null data\n", __func__);
            ret = WIFI_ERROR_INVALID_ARGS;
            goto exit;
        }

        data_ptr = data;
        for (int i = 0; i < num_radios; i++) {
            outbuf_rem_len -= per_radio_size;
            if (outbuf_rem_len < per_radio_size) {
                ALOGE("No data left for radio %d\n", i);
                ret = WIFI_ERROR_INVALID_ARGS;
                goto exit;
            }

            data_ptr = data + offset;
            if (!data_ptr) {
                ALOGE("Invalid data for radio index = %d\n", i);
                ret = WIFI_ERROR_INVALID_ARGS;
                goto exit;
            }
            ret = convertToExternalRadioStatStructure((wifi_radio_stat*)data_ptr,
                &per_radio_size, &outbuf, &outbuf_rem_len);
            if (ret < 0) {
                ALOGE(("Failed to map data radioStat struct\n"));
                goto exit;
            }
            if (!per_radio_size) {
                ALOGE("No data for radio %d\n", i);
                continue;
            }
            outbuf += per_radio_size;
            /* Accounted size of all the radio data len */
            offset += per_radio_size;
        }

        if (ml_data) {
            outbuf = NULL;
            data_rem_len = data_len - offset;
            fixed_iface_ml_stat_size = offsetof(wifi_iface_ml_stat, links);

            /* Allocate vendor hal buffer, instead of using the nlmsg allocated buffer */
            ifaceMlStatsBuf = (u8 *)malloc(MAX_CMD_RESP_BUF_LEN);
            if (!ifaceMlStatsBuf) {
                ALOGE("No memory\n");
                ret = WIFI_ERROR_OUT_OF_MEMORY;
                goto exit;
            }
            outbuf_rem_len = MAX_CMD_RESP_BUF_LEN;
            memset(ifaceMlStatsBuf, 0, MAX_CMD_RESP_BUF_LEN);
            outbuf = ifaceMlStatsBuf;

            if (data_rem_len >= fixed_iface_ml_stat_size) {
                data_ptr = (data + offset);
                if (!data_ptr) {
                    ALOGE("No iface ml stats fixed data!!, data_len = %d, offset = %d\n",
                        data_len, offset);
                    ret = WIFI_ERROR_INVALID_ARGS;
                    goto exit;
                }

                if (outbuf_rem_len < fixed_iface_ml_stat_size) {
                    ALOGE("No space to copy fixed iface ml stats!, rem_len %d, req_len %d\n",
                        outbuf_rem_len, fixed_iface_ml_stat_size);
                    ret = WIFI_ERROR_OUT_OF_MEMORY;
                    goto exit;
                }

                memcpy(outbuf, data_ptr, fixed_iface_ml_stat_size);
                data_rem_len -= fixed_iface_ml_stat_size;
                outbuf_rem_len -= fixed_iface_ml_stat_size;
                outbuf += fixed_iface_ml_stat_size;
                offset += fixed_iface_ml_stat_size;

                iface_ml_stat_ptr = (wifi_iface_ml_stat *)ifaceMlStatsBuf;
                if (!iface_ml_stat_ptr) {
                    ALOGE("No iface ml stats data!!");
                    ret = WIFI_ERROR_INVALID_ARGS;
                    goto exit;
                }

                num_links = iface_ml_stat_ptr->num_links;
                all_links_stat_size = (num_links * offsetof(wifi_link_stat, peer_info));
                if (num_links > MAX_MLO_LINK) {
                    ALOGE("Invalid num links :%d\n", num_links);
                    ret = WIFI_ERROR_INVALID_ARGS;
                    goto exit;
                }

                if (num_links && (data_rem_len >= all_links_stat_size)) {
                    ret = convertToExternalIfaceMlstatStructure(&data, &offset, &outbuf,
                            &data_rem_len, num_links, &outbuf_rem_len);
                    if (ret < 0) {
                        ALOGE(("Failed to map data to iface ml struct\n"));
                        goto exit;
                    }
                } else {
                    ALOGE("num_links %d, Required data not found: expected len %d,"
                            " data_rem_len %d\n", num_links, all_links_stat_size, data_rem_len);
                    ret = WIFI_ERROR_INVALID_ARGS;
                    goto exit;
                }
                (*mHandler.on_multi_link_stats_results)(id,
                    (wifi_iface_ml_stat *)ifaceMlStatsBuf, num_radios,
                    (wifi_radio_stat *)radioStatsBuf);
            }
        } else if ((data_len >= (offset + sizeof(wifi_iface_stat)))) {
            iface_stat = (data + offset);
            if (!iface_stat) {
                ALOGE("No data for legacy iface stats!!, data_len = %d, offset = %d\n",
                        data_len, offset);
                ret = WIFI_ERROR_INVALID_ARGS;
                goto exit;
            }
            (*mHandler.on_link_stats_results)(id, (wifi_iface_stat *)iface_stat,
                num_radios, (wifi_radio_stat *)radioStatsBuf);
        } else {
            ALOGE("No data for iface stats!!, data_len = %d, offset = %d\n",
                    data_len, offset);
            ret = WIFI_ERROR_INVALID_ARGS;
            goto exit;
        }

exit:
        /* report valid radiostat eventhough there is no linkstat info
         * (non assoc/error case)
         */
        if ((ret != WIFI_SUCCESS) && num_radios && per_radio_size && ml_data) {
            (*mHandler.on_multi_link_stats_results)(id, NULL, num_radios,
                    (wifi_radio_stat *)radioStatsBuf);
        } else if ((ret != WIFI_SUCCESS) && num_radios && per_radio_size) {
            (*mHandler.on_link_stats_results)(id, NULL,
                    num_radios, (wifi_radio_stat *)radioStatsBuf);
        }

        if (radioStatsBuf) {
            free(radioStatsBuf);
            radioStatsBuf = NULL;
        }
        if (ifaceMlStatsBuf) {
            free(ifaceMlStatsBuf);
            ifaceMlStatsBuf = NULL;
        }
        return NL_OK;
    }

private:
    int convertToExternalRadioStatStructure(wifi_radio_stat *internal_stat_ptr,
        uint32_t *per_radio_size, u8 **outbuf, uint32_t *outbuf_rem_len)
    {
        int ret = 0;
        if (!internal_stat_ptr) {
            ALOGE("Incoming data is null\n");
            ret = WIFI_ERROR_INVALID_ARGS;
        } else {
            uint32_t channel_size = internal_stat_ptr->num_channels * sizeof(wifi_channel_stat);
            *per_radio_size = offsetof(wifi_radio_stat, channels) + channel_size;
            if (*outbuf_rem_len >= *per_radio_size) {
                memcpy(*outbuf, internal_stat_ptr, *per_radio_size);
            } else {
                ALOGE("Insufficient buf size to copy. rem len %d, req size %d\n",
                    *outbuf_rem_len, *per_radio_size);
                ret = WIFI_ERROR_OUT_OF_MEMORY;
            }
        }
        return ret;
    }

    int convertToExternalRatestatsStructure(u8 **data, u32 *offset, u8 **outbuf,
        u32 *data_rem_len, wifi_peer_info *peer_info, u8 num_rate, u32 *outbuf_rem_len)
    {
        u8 k = 0, num_peers = 0;
        int ret = 0;
        u8 *data_ptr = NULL;
        u32 all_rates_size = 0, per_rate_size = 0;

        per_rate_size = sizeof(wifi_rate_stat);
        all_rates_size = num_rate * per_rate_size;
        if (!peer_info && (*data_rem_len != all_rates_size)) {
            ALOGE("Insufficient data for rate_stats, data_rem_len %d,"
                "required data size %d \n", *data_rem_len, all_rates_size);
            ret = WIFI_ERROR_INVALID_ARGS;
            goto exit;
        }

        ALOGV("Available data_rem_len %d, expected size %d for rate_stat\n",
                *data_rem_len, all_rates_size);
        for (k = 0; k < num_rate; k++) {
            data_ptr = ((*data) + (*offset));
            if (!data_ptr) {
                ALOGE("rate_stats not found!! num_rate %d, *data_rem_len = %d, *offset = %d\n",
                    num_rate, *data_rem_len, *offset);
                ret = WIFI_ERROR_OUT_OF_MEMORY;
                goto exit;
            }

            if (*data_rem_len < per_rate_size) {
                ALOGE("no rate_stats!!, data_rem_len %d, rate_stat size %d\n",
                    *data_rem_len, per_rate_size);
                ret = WIFI_ERROR_INVALID_ARGS;
                goto exit;
            }

            if (*outbuf_rem_len < per_rate_size) {
                ALOGE("No space to copy rate_stats of index [%d]!, rem_len %d, req_len %d\n",
                    k, *outbuf_rem_len, per_rate_size);
                ret = WIFI_ERROR_OUT_OF_MEMORY;
                goto exit;
            }

            ALOGV("index: %d per_rate_size %d, data_rem_len %d\n",
                   k, per_rate_size, *data_rem_len);
            memcpy(*outbuf, data_ptr, per_rate_size);
            *data_rem_len -= per_rate_size;
            *outbuf_rem_len -= per_rate_size;
            *outbuf += per_rate_size;
            *offset += per_rate_size;
        }
    exit:
        return ret;
    }

    int convertToExternalIfaceLinkstatStructure(u8 **data, uint32_t *offset, u8 **outbuf,
            u32 *data_rem_len, u8 num_peers, wifi_link_stat *links, u32 *outbuf_rem_len)
    {
        int ret = 0, j = 0, num_rate = 0;
        u32 all_rate_stats_per_peer_per_link_size = 0, fixed_peer_info_size = 0;
        u8 *data_ptr = NULL;
        wifi_peer_info *peer_info_ptr = NULL;

        for (j = 0; j < num_peers; j++) {
            data_ptr = ((*data) + (*offset));
            if (!data_ptr || (!*data_rem_len)) {
                ALOGE("no peer_info data!! num_peers %d, data_rem_len = %d, *offset = %d\n",
                    num_peers, *data_rem_len, *offset);
                ret = WIFI_ERROR_OUT_OF_MEMORY;
                goto exit;
            }

            fixed_peer_info_size = offsetof(wifi_peer_info, rate_stats);
            if (*data_rem_len < fixed_peer_info_size) {
                ALOGE("no fixed peer_info data!!, data_rem_len %d, fixed peer info %d\n",
                    *data_rem_len, fixed_peer_info_size);
                ret = WIFI_ERROR_INVALID_ARGS;
                goto exit;
            }

            if (*outbuf_rem_len < fixed_peer_info_size) {
                ALOGE("No space to copy fixed peer_info of index[%d]!, rem_len %d, req_len %d\n",
                    j, *outbuf_rem_len, fixed_peer_info_size);
                ret = WIFI_ERROR_OUT_OF_MEMORY;
                goto exit;
            }

            memcpy(*outbuf, data_ptr, fixed_peer_info_size);
            *data_rem_len -= fixed_peer_info_size;
            *outbuf_rem_len -= fixed_peer_info_size;
            *outbuf += fixed_peer_info_size;
            *offset += fixed_peer_info_size;

            peer_info_ptr = (wifi_peer_info *)data_ptr;
            if (!peer_info_ptr) {
                ALOGE("no peer_info data!!");
                ret = WIFI_ERROR_INVALID_ARGS;
                goto exit;
            }

            num_rate = peer_info_ptr->num_rate;
            /* boundary check as per max supported num rate */
            if (num_rate <= MAX_NUM_RATE) {
                all_rate_stats_per_peer_per_link_size = num_rate * sizeof(wifi_rate_stat);
                if (num_rate && (*data_rem_len >= all_rate_stats_per_peer_per_link_size)) {
                    ret = convertToExternalRatestatsStructure(data, offset, outbuf, data_rem_len,
                        peer_info_ptr, num_rate, outbuf_rem_len);
                    if (ret != WIFI_SUCCESS) {
                        ALOGE(("Failed to convert it to rate stats\n"));
                        goto exit;
                    }
                } else {
                    ALOGI("num_rate %d, Required rate_stats not found: expected len %d,"
                        " data_rem_len %d\n", num_rate, all_rate_stats_per_peer_per_link_size,
                        *data_rem_len);
                    continue;
                }
            } else if (num_rate == 0) {
                ALOGI("Sta is not associated, num rate :%d\n", num_rate);
            } else {
                ALOGE("Invalid num rate :%d\n", num_rate);
                ret = WIFI_ERROR_INVALID_ARGS;
                goto exit;
            }
        }

    exit:
        return ret;
    }

    int convertToExternalIfaceMlstatStructure(u8 **data, u32 *offset, u8 **outbuf,
        u32 *data_rem_len, u8 num_links, u32 *outbuf_rem_len)
    {
        int ret = 0, i = 0;
        u32 all_peers_per_link_size = 0, fixed_link_stat_size = 0;
        u8 *data_ptr = NULL;
        u8 num_peers = 0;
        wifi_link_stat *links_ptr = NULL;

        for (i = 0; i < num_links; i++) {
            data_ptr = ((*data) + (*offset));
            if (!data_ptr || !(*data_rem_len)) {
                ALOGE("no variable links data!! num_links %d, data_rem_len = %d, offset = %d\n",
                    num_links, *data_rem_len, *offset);
                ret = WIFI_ERROR_OUT_OF_MEMORY;
                goto exit;
            }

            fixed_link_stat_size = offsetof(wifi_link_stat, peer_info);
            if (*data_rem_len < fixed_link_stat_size) {
                ALOGE("no fixed wifi_link_stat data!!, data_rem_len %d, fixed link stat data %d\n",
                    *data_rem_len, fixed_link_stat_size);
                ret = WIFI_ERROR_INVALID_ARGS;
                goto exit;
            }

            if (*outbuf_rem_len < fixed_link_stat_size) {
                ALOGE("No space to copy fixed link stats of index[%d]!, rem_len %d, req_len %d\n",
                    i, *outbuf_rem_len, fixed_link_stat_size);
                ret = WIFI_ERROR_OUT_OF_MEMORY;
                goto exit;
            }

            memcpy(*outbuf, data_ptr, fixed_link_stat_size);
            *outbuf_rem_len -= fixed_link_stat_size;
            *data_rem_len -= fixed_link_stat_size;
            *outbuf += fixed_link_stat_size;
            *offset += fixed_link_stat_size;

            links_ptr = (wifi_link_stat *)data_ptr;
            if (!links_ptr) {
                ALOGE("no link_stat data!!");
                ret = WIFI_ERROR_INVALID_ARGS;
                goto exit;
            }

            if (!links_ptr->num_peers) {
                ALOGI("no peers in unassoc case, skip processing peer stats\n");
                continue;
            }

            if (links_ptr->num_peers != NUM_PEER_AP) {
                ALOGE("Invalid num peer :%d\n", links_ptr->num_peers);
                ret = WIFI_ERROR_INVALID_ARGS;
                goto exit;
            }

            num_peers = links_ptr->num_peers;
            all_peers_per_link_size = num_peers * offsetof(wifi_peer_info, rate_stats);
            if ((num_peers == NUM_PEER_AP) && (*data_rem_len >= all_peers_per_link_size)) {
                ret = convertToExternalIfaceLinkstatStructure(data, offset, outbuf,
                    data_rem_len, num_peers, links_ptr, outbuf_rem_len);
                if (ret != WIFI_SUCCESS) {
                    ALOGE(("Failed to convert it to iface link stats\n"));
                    goto exit;
                }
            } else {
                ALOGI("num_peers %d, Required data not found: expected len %d, data_rem_len %d\n",
                    num_peers, all_peers_per_link_size, *data_rem_len);
                continue;
            }
        }
    exit:
        return ret;
    }
};

wifi_error wifi_get_link_stats(wifi_request_id id,
        wifi_interface_handle iface, wifi_stats_result_handler handler)
{
    GetLinkStatsCommand command(iface, handler);
    return (wifi_error) command.requestResponse();
}

wifi_error wifi_set_link_stats(
        wifi_interface_handle /* iface */, wifi_link_layer_params /* params */)
{
    /* Return success here since bcom HAL does not need set link stats. */
    return WIFI_SUCCESS;
}

wifi_error wifi_clear_link_stats(
        wifi_interface_handle /* iface */, u32 /* stats_clear_req_mask */,
        u32 * /* stats_clear_rsp_mask */, u8 /* stop_req */, u8 * /* stop_rsp */)
{
    /* Return success here since bcom HAL does not support clear link stats. */
    return WIFI_SUCCESS;
}
