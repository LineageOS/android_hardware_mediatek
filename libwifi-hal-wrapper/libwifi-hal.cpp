/*
 * SPDX-FileCopyrightText: 2024 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "libwifi-hal-wrapper"

#include <dlfcn.h>
#include <stdlib.h>
#include <unistd.h>

#include <log/log.h>

#include "wifi_hal_legacy.h"

#define WIFI_HAL_LIB_NAME "libwifi-hal-mtk.so"
#define WIFI_HAL_INIT_FUNC_NAME "init_wifi_vendor_hal_func_table"

#define SET_IF_NOT_NULL(fn_ptr, fln_ptr, member)  \
    do {                                          \
        if ((fln_ptr) && (fln_ptr)->member) {     \
            (fn_ptr)->member = (fln_ptr)->member; \
        }                                         \
    } while (0)

static wifi_hal_legacy_fn* fln = nullptr;
static void* lib_handle = nullptr;

wifi_error init_wifi_vendor_hal_func_table(wifi_hal_fn* fn) {
    init_wifi_vendor_hal_func_table_legacy_t init_fn;
    wifi_error ret = WIFI_ERROR_NONE;

    fln = (wifi_hal_legacy_fn*)calloc(1, sizeof(wifi_hal_legacy_fn));
    if (!fln) {
        ret = WIFI_ERROR_OUT_OF_MEMORY;
        goto out;
    }

    lib_handle = dlopen(WIFI_HAL_LIB_NAME, RTLD_LAZY);
    if (!lib_handle) {
        ALOGE("Failed to open vendor library %s.", WIFI_HAL_LIB_NAME);
        ret = WIFI_ERROR_UNKNOWN;
        goto error;
    }

    init_fn = (init_wifi_vendor_hal_func_table_legacy_t)dlsym(lib_handle, WIFI_HAL_INIT_FUNC_NAME);
    if (!init_fn) {
        ALOGE("Failed to locate vendor HAL init function.");
        ret = WIFI_ERROR_UNKNOWN;
        goto error;
    }

    ret = init_fn(fln);
    if (ret != WIFI_ERROR_NONE) {
        ALOGE("Vendor HAL init function returns %d.", ret);
        goto error;
    }

    SET_IF_NOT_NULL(fn, fln, wifi_initialize);
    SET_IF_NOT_NULL(fn, fln, wifi_wait_for_driver_ready);
    SET_IF_NOT_NULL(fn, fln, wifi_cleanup);
    SET_IF_NOT_NULL(fn, fln, wifi_event_loop);
    SET_IF_NOT_NULL(fn, fln, wifi_get_error_info);
    SET_IF_NOT_NULL(fn, fln, wifi_get_supported_feature_set);
    SET_IF_NOT_NULL(fn, fln, wifi_get_concurrency_matrix);
    SET_IF_NOT_NULL(fn, fln, wifi_set_scanning_mac_oui);
    SET_IF_NOT_NULL(fn, fln, wifi_get_supported_channels);
    SET_IF_NOT_NULL(fn, fln, wifi_is_epr_supported);
    SET_IF_NOT_NULL(fn, fln, wifi_get_ifaces);
    SET_IF_NOT_NULL(fn, fln, wifi_get_iface_name);
    SET_IF_NOT_NULL(fn, fln, wifi_set_iface_event_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_reset_iface_event_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_start_gscan);
    SET_IF_NOT_NULL(fn, fln, wifi_stop_gscan);
    SET_IF_NOT_NULL(fn, fln, wifi_get_cached_gscan_results);
    SET_IF_NOT_NULL(fn, fln, wifi_set_bssid_hotlist);
    SET_IF_NOT_NULL(fn, fln, wifi_reset_bssid_hotlist);
    SET_IF_NOT_NULL(fn, fln, wifi_set_significant_change_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_reset_significant_change_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_get_gscan_capabilities);
    SET_IF_NOT_NULL(fn, fln, wifi_set_link_stats);
    SET_IF_NOT_NULL(fn, fln, wifi_get_link_stats);
    SET_IF_NOT_NULL(fn, fln, wifi_clear_link_stats);
    SET_IF_NOT_NULL(fn, fln, wifi_get_valid_channels);
    SET_IF_NOT_NULL(fn, fln, wifi_rtt_range_request);
    SET_IF_NOT_NULL(fn, fln, wifi_rtt_range_cancel);
    SET_IF_NOT_NULL(fn, fln, wifi_get_rtt_capabilities);
    SET_IF_NOT_NULL(fn, fln, wifi_rtt_get_responder_info);
    SET_IF_NOT_NULL(fn, fln, wifi_enable_responder);
    SET_IF_NOT_NULL(fn, fln, wifi_disable_responder);
    SET_IF_NOT_NULL(fn, fln, wifi_set_nodfs_flag);
    SET_IF_NOT_NULL(fn, fln, wifi_start_logging);
    SET_IF_NOT_NULL(fn, fln, wifi_set_epno_list);
    SET_IF_NOT_NULL(fn, fln, wifi_reset_epno_list);
    SET_IF_NOT_NULL(fn, fln, wifi_set_country_code);
    SET_IF_NOT_NULL(fn, fln, wifi_get_firmware_memory_dump);
    SET_IF_NOT_NULL(fn, fln, wifi_set_log_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_reset_log_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_set_alert_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_reset_alert_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_get_firmware_version);
    SET_IF_NOT_NULL(fn, fln, wifi_get_ring_buffers_status);
    SET_IF_NOT_NULL(fn, fln, wifi_get_logger_supported_feature_set);
    SET_IF_NOT_NULL(fn, fln, wifi_get_ring_data);
    SET_IF_NOT_NULL(fn, fln, wifi_enable_tdls);
    SET_IF_NOT_NULL(fn, fln, wifi_disable_tdls);
    SET_IF_NOT_NULL(fn, fln, wifi_get_tdls_status);
    SET_IF_NOT_NULL(fn, fln, wifi_get_tdls_capabilities);
    SET_IF_NOT_NULL(fn, fln, wifi_get_driver_version);
    SET_IF_NOT_NULL(fn, fln, wifi_set_passpoint_list);
    SET_IF_NOT_NULL(fn, fln, wifi_reset_passpoint_list);
    SET_IF_NOT_NULL(fn, fln, wifi_set_lci);
    SET_IF_NOT_NULL(fn, fln, wifi_set_lcr);
    SET_IF_NOT_NULL(fn, fln, wifi_start_sending_offloaded_packet);
    SET_IF_NOT_NULL(fn, fln, wifi_stop_sending_offloaded_packet);
    SET_IF_NOT_NULL(fn, fln, wifi_start_rssi_monitoring);
    SET_IF_NOT_NULL(fn, fln, wifi_stop_rssi_monitoring);
    SET_IF_NOT_NULL(fn, fln, wifi_get_wake_reason_stats);
    SET_IF_NOT_NULL(fn, fln, wifi_configure_nd_offload);
    SET_IF_NOT_NULL(fn, fln, wifi_get_driver_memory_dump);
    SET_IF_NOT_NULL(fn, fln, wifi_start_pkt_fate_monitoring);
    SET_IF_NOT_NULL(fn, fln, wifi_get_tx_pkt_fates);
    SET_IF_NOT_NULL(fn, fln, wifi_get_rx_pkt_fates);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_enable_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_disable_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_publish_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_publish_cancel_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_subscribe_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_subscribe_cancel_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_transmit_followup_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_stats_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_config_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_tca_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_beacon_sdf_payload_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_register_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_get_version);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_get_capabilities);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_data_interface_create);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_data_interface_delete);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_data_request_initiator);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_data_indication_response);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_pairing_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_pairing_indication_response);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_bootstrapping_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_bootstrapping_indication_response);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_data_end);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_pairing_end);
    SET_IF_NOT_NULL(fn, fln, wifi_get_packet_filter_capabilities);
    SET_IF_NOT_NULL(fn, fln, wifi_set_packet_filter);
    SET_IF_NOT_NULL(fn, fln, wifi_read_packet_filter);
    SET_IF_NOT_NULL(fn, fln, wifi_get_roaming_capabilities);
    SET_IF_NOT_NULL(fn, fln, wifi_enable_firmware_roaming);
    SET_IF_NOT_NULL(fn, fln, wifi_configure_roaming);
    SET_IF_NOT_NULL(fn, fln, wifi_select_tx_power_scenario);
    SET_IF_NOT_NULL(fn, fln, wifi_reset_tx_power_scenario);
    SET_IF_NOT_NULL(fn, fln, wifi_set_radio_mode_change_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_set_latency_mode);
    SET_IF_NOT_NULL(fn, fln, wifi_set_thermal_mitigation_mode);
    SET_IF_NOT_NULL(fn, fln, wifi_virtual_interface_create);
    SET_IF_NOT_NULL(fn, fln, wifi_virtual_interface_delete);
    SET_IF_NOT_NULL(fn, fln, wifi_map_dscp_access_category);
    SET_IF_NOT_NULL(fn, fln, wifi_reset_dscp_mapping);
    SET_IF_NOT_NULL(fn, fln, wifi_set_subsystem_restart_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_get_supported_iface_name);
    SET_IF_NOT_NULL(fn, fln, wifi_early_initialize);
    SET_IF_NOT_NULL(fn, fln, wifi_get_chip_feature_set);
    SET_IF_NOT_NULL(fn, fln, wifi_multi_sta_set_primary_connection);
    SET_IF_NOT_NULL(fn, fln, wifi_multi_sta_set_use_case);
    SET_IF_NOT_NULL(fn, fln, wifi_set_coex_unsafe_channels);
    SET_IF_NOT_NULL(fn, fln, wifi_set_voip_mode);
    SET_IF_NOT_NULL(fn, fln, wifi_twt_register_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_twt_get_capability);
    SET_IF_NOT_NULL(fn, fln, wifi_twt_setup_request);
    SET_IF_NOT_NULL(fn, fln, wifi_twt_teardown_request);
    SET_IF_NOT_NULL(fn, fln, wifi_twt_info_frame_request);
    SET_IF_NOT_NULL(fn, fln, wifi_twt_get_stats);
    SET_IF_NOT_NULL(fn, fln, wifi_twt_clear_stats);
    SET_IF_NOT_NULL(fn, fln, wifi_set_dtim_config);
    SET_IF_NOT_NULL(fn, fln, wifi_get_usable_channels);
    SET_IF_NOT_NULL(fn, fln, wifi_trigger_subsystem_restart);
    SET_IF_NOT_NULL(fn, fln, wifi_set_indoor_state);
    SET_IF_NOT_NULL(fn, fln, wifi_get_supported_radio_combinations_matrix);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_rtt_chre_enable_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_rtt_chre_disable_request);
    SET_IF_NOT_NULL(fn, fln, wifi_chre_register_handler);
    SET_IF_NOT_NULL(fn, fln, wifi_enable_tx_power_limits);
    SET_IF_NOT_NULL(fn, fln, wifi_get_cached_scan_results);
    SET_IF_NOT_NULL(fn, fln, wifi_get_chip_capabilities);
    SET_IF_NOT_NULL(fn, fln, wifi_enable_sta_channel_for_peer_network);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_suspend_request);
    SET_IF_NOT_NULL(fn, fln, wifi_nan_resume_request);
    SET_IF_NOT_NULL(fn, fln, wifi_set_scan_mode);
    SET_IF_NOT_NULL(fn, fln, wifi_set_mlo_mode);
    SET_IF_NOT_NULL(fn, fln, wifi_get_supported_iface_concurrency_matrix);

out:
    return ret;
error:
    free(fln);
    if (lib_handle) dlclose(lib_handle);
    return ret;
}
