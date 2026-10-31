/*
 * Copyright (C) 2022 The Android Open Source Project
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

#include <inttypes.h>
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

#define CACHE_SCAN_RESULT_SIZE sizeof(wifi_cached_scan_result)
#define MAX_CACHE_SCAN_RESULT_SIZE (MAX_CACHED_SCAN_RESULT*CACHE_SCAN_RESULT_SIZE)
typedef enum {
    WIFI_ATTRIBUTE_CACHED_SCAN_INVALID        = 0,
    WIFI_ATTRIBUTE_CACHED_SCAN_BOOT_TIMESTAMP = 1,
    WIFI_ATTRIBUTE_CACHED_SCANNED_FREQ_NUM    = 2,
    WIFI_ATTRIBUTE_CACHED_SCANNED_FREQ_LIST   = 3,
    WIFI_ATTRIBUTE_CACHED_SCAN_RESULT_CNT     = 4,
    WIFI_ATTRIBUTE_CACHED_SCAN_RESULTS        = 5,
    WIFI_ATTRIBUTE_CACHED_SCAN_MAX
} SCAN_ATTRIBUTE;
/////////////////////////////////////////////////////////////////////////////

class GetCachedScanResultsCommand : public WifiCommand {
    wifi_cached_scan_result_handler mHandler;
    wifi_cached_scan_report *mReport;

public:
    GetCachedScanResultsCommand(wifi_interface_handle iface, int id,
            wifi_cached_scan_result_handler handler)
        : WifiCommand("GetCachedScanResultsCommand", iface, id), mHandler(handler)
    {
        mReport = NULL;
    }

    virtual int create() {
        ALOGE("Creating message to get cached scan results");

        int ret = mMsg.create(GOOGLE_OUI, WIFI_SUBCMD_GET_CACHED_SCAN_RESULTS);
        if (ret < 0) {
            return ret;
        }

        return ret;
    }

    protected:
    virtual int handleResponse(WifiEvent& reply) {
        ALOGI("In GetCachedScanResultsCommand::handleResponse");

        if (reply.get_cmd() != NL80211_CMD_VENDOR) {
            ALOGD("Ignoring reply with cmd = %d", reply.get_cmd());
            return NL_SKIP;
        }

        int id = reply.get_vendor_id();
        int subcmd = reply.get_vendor_subcmd();
        int valid_entries = 0;
        int result_size = 0;
        wifi_timestamp timestamp = 0;
        int size = 0;

        ALOGV("Id = %0x, subcmd = %d", id, subcmd);

        nlattr *vendor_data = reply.get_attribute(NL80211_ATTR_VENDOR_DATA);
        int len = reply.get_vendor_data_len();

        if (vendor_data == NULL || len == 0) {
            ALOGE("no vendor data in GetCachedScanResults response; ignoring it");
            return NL_SKIP;
        }

        mReport = (wifi_cached_scan_report *)malloc(sizeof(wifi_cached_scan_report));
        if (mReport == NULL) {
            ALOGE("Failed to allocate!!\n");
            return NL_SKIP;
        }
        memset(mReport, 0, sizeof(wifi_cached_scan_report));

        ALOGV("Id = %0x, subcmd = %d, len = %d", id, subcmd, len);
        for (nl_iterator it(vendor_data); it.has_next(); it.next()) {
            if (it.get_type() == WIFI_ATTRIBUTE_CACHED_SCAN_BOOT_TIMESTAMP) {
                timestamp = it.get_u32();
                ALOGV("Time since boot_ms: %" PRId64 "\n", timestamp);
            } else if (it.get_type() == WIFI_ATTRIBUTE_CACHED_SCAN_RESULT_CNT) {
                valid_entries = min(it.get_u16(), MAX_CACHED_SCAN_RESULT);
                ALOGV("cached scan result cnt: %d", valid_entries);
            } else if ((it.get_type() == WIFI_ATTRIBUTE_CACHED_SCAN_RESULTS) &&
                valid_entries && timestamp) {
                size = min(it.get_len(), MAX_CACHE_SCAN_RESULT_SIZE);
                if (size > (valid_entries*CACHE_SCAN_RESULT_SIZE)) {
                    ALOGE("Not enough space to copy!!\n");
                    return NL_SKIP;
                }
                result_size = valid_entries * CACHE_SCAN_RESULT_SIZE;
                mReport->results = (wifi_cached_scan_result *)malloc(result_size);
                if (mReport->results == NULL) {
                    ALOGE("Failed to allocate!!\n");
                    return NL_SKIP;
                }
                memcpy((void *)mReport->results, (void *)it.get_data(), size);
            } else {
                ALOGW("Ignoring invalid attribute type = %d, size = %d",
                        it.get_type(), it.get_len());
            }
        }

        mReport->ts = timestamp;
        mReport->result_cnt = valid_entries;

        if (*mHandler.on_cached_scan_results && mReport) {
            (*mHandler.on_cached_scan_results)(mReport);
            ALOGV("Notified cache scan report!!");
        }

        if (mReport) {
            if (mReport->results) {
                free((void *)mReport->results);
            }
            free(mReport);
        }
        return NL_OK;
    }
};

wifi_error wifi_get_cached_scan_results(wifi_interface_handle iface,
    wifi_cached_scan_result_handler handler)
{
    ALOGI("Getting cached scan results, iface handle = %p", iface);
    GetCachedScanResultsCommand command(iface, 0, handler);
    return (wifi_error) command.requestResponse();
}
