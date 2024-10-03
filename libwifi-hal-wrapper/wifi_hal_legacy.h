/*
 * SPDX-FileCopyrightText: 2016 The Android Open Source Project
 * SPDX-FileCopyrightText: 2024 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <hardware_legacy/wifi_hal.h>

// wifi HAL function pointer table
typedef struct {
    wifi_error (*wifi_initialize)(wifi_handle*);
    wifi_error (*wifi_wait_for_driver_ready)(void);
    void (*wifi_cleanup)(wifi_handle, wifi_cleaned_up_handler);
    void (*wifi_event_loop)(wifi_handle);
    void (*wifi_get_error_info)(wifi_error, const char**);
    wifi_error (*wifi_get_supported_feature_set)(wifi_interface_handle, feature_set*);
    wifi_error (*wifi_get_concurrency_matrix)(wifi_interface_handle, int, feature_set*, int*);
    wifi_error (*wifi_set_scanning_mac_oui)(wifi_interface_handle, unsigned char*);
    wifi_error (*wifi_get_supported_channels)(wifi_handle, int*, wifi_channel*);
    wifi_error (*wifi_is_epr_supported)(wifi_handle);
    wifi_error (*wifi_get_ifaces)(wifi_handle, int*, wifi_interface_handle**);
    wifi_error (*wifi_get_iface_name)(wifi_interface_handle, char* name, size_t);
    wifi_error (*wifi_set_iface_event_handler)(wifi_request_id, wifi_interface_handle,
                                               wifi_event_handler);
    wifi_error (*wifi_reset_iface_event_handler)(wifi_request_id, wifi_interface_handle);
    wifi_error (*wifi_start_gscan)(wifi_request_id, wifi_interface_handle, wifi_scan_cmd_params,
                                   wifi_scan_result_handler);
    wifi_error (*wifi_stop_gscan)(wifi_request_id, wifi_interface_handle);
    wifi_error (*wifi_get_cached_gscan_results)(wifi_interface_handle, byte, int,
                                                wifi_cached_scan_results*, int*);
    wifi_error (*wifi_set_bssid_hotlist)(wifi_request_id, wifi_interface_handle,
                                         wifi_bssid_hotlist_params, wifi_hotlist_ap_found_handler);
    wifi_error (*wifi_reset_bssid_hotlist)(wifi_request_id, wifi_interface_handle);
    wifi_error (*wifi_set_significant_change_handler)(wifi_request_id, wifi_interface_handle,
                                                      wifi_significant_change_params,
                                                      wifi_significant_change_handler);
    wifi_error (*wifi_reset_significant_change_handler)(wifi_request_id, wifi_interface_handle);
    wifi_error (*wifi_get_gscan_capabilities)(wifi_interface_handle, wifi_gscan_capabilities*);
    wifi_error (*wifi_set_link_stats)(wifi_interface_handle, wifi_link_layer_params);
    wifi_error (*wifi_get_link_stats)(wifi_request_id, wifi_interface_handle,
                                      wifi_stats_result_handler);
    wifi_error (*wifi_clear_link_stats)(wifi_interface_handle, u32, u32*, u8, u8*);
    wifi_error (*wifi_get_valid_channels)(wifi_interface_handle, int, int, wifi_channel*, int*);
    wifi_error (*wifi_rtt_range_request)(wifi_request_id, wifi_interface_handle, unsigned,
                                         wifi_rtt_config[], wifi_rtt_event_handler);
    wifi_error (*wifi_rtt_range_cancel)(wifi_request_id, wifi_interface_handle, unsigned,
                                        mac_addr[]);
    wifi_error (*wifi_get_rtt_capabilities)(wifi_interface_handle, wifi_rtt_capabilities*);
    wifi_error (*wifi_rtt_get_responder_info)(wifi_interface_handle iface,
                                              wifi_rtt_responder* responder_info);
    wifi_error (*wifi_enable_responder)(wifi_request_id id, wifi_interface_handle iface,
                                        wifi_channel_info channel_hint,
                                        unsigned max_duration_seconds,
                                        wifi_rtt_responder* responder_info);
    wifi_error (*wifi_disable_responder)(wifi_request_id id, wifi_interface_handle iface);
    wifi_error (*wifi_set_nodfs_flag)(wifi_interface_handle, u32);
    wifi_error (*wifi_start_logging)(wifi_interface_handle, u32, u32, u32, u32, char*);
    wifi_error (*wifi_set_epno_list)(wifi_request_id, wifi_interface_handle,
                                     const wifi_epno_params*, wifi_epno_handler);
    wifi_error (*wifi_reset_epno_list)(wifi_request_id, wifi_interface_handle);
    wifi_error (*wifi_set_country_code)(wifi_interface_handle, const char*);
    wifi_error (*wifi_get_firmware_memory_dump)(wifi_interface_handle iface,
                                                wifi_firmware_memory_dump_handler handler);
    wifi_error (*wifi_set_log_handler)(wifi_request_id id, wifi_interface_handle iface,
                                       wifi_ring_buffer_data_handler handler);
    wifi_error (*wifi_reset_log_handler)(wifi_request_id id, wifi_interface_handle iface);
    wifi_error (*wifi_set_alert_handler)(wifi_request_id id, wifi_interface_handle iface,
                                         wifi_alert_handler handler);
    wifi_error (*wifi_reset_alert_handler)(wifi_request_id id, wifi_interface_handle iface);
    wifi_error (*wifi_get_firmware_version)(wifi_interface_handle iface, char* buffer,
                                            int buffer_size);
    wifi_error (*wifi_get_ring_buffers_status)(wifi_interface_handle iface, u32* num_rings,
                                               wifi_ring_buffer_status* status);
    wifi_error (*wifi_get_logger_supported_feature_set)(wifi_interface_handle iface,
                                                        unsigned int* support);
    wifi_error (*wifi_get_ring_data)(wifi_interface_handle iface, char* ring_name);
    wifi_error (*wifi_enable_tdls)(wifi_interface_handle, mac_addr, wifi_tdls_params*,
                                   wifi_tdls_handler);
    wifi_error (*wifi_disable_tdls)(wifi_interface_handle, mac_addr);
    wifi_error (*wifi_get_tdls_status)(wifi_interface_handle, mac_addr, wifi_tdls_status*);
    wifi_error (*wifi_get_tdls_capabilities)(wifi_interface_handle iface,
                                             wifi_tdls_capabilities* capabilities);
    wifi_error (*wifi_get_driver_version)(wifi_interface_handle iface, char* buffer,
                                          int buffer_size);
    wifi_error (*wifi_set_passpoint_list)(wifi_request_id id, wifi_interface_handle iface, int num,
                                          wifi_passpoint_network* networks,
                                          wifi_passpoint_event_handler handler);
    wifi_error (*wifi_reset_passpoint_list)(wifi_request_id id, wifi_interface_handle iface);
    wifi_error (*wifi_set_lci)(wifi_request_id id, wifi_interface_handle iface,
                               wifi_lci_information* lci);
    wifi_error (*wifi_set_lcr)(wifi_request_id id, wifi_interface_handle iface,
                               wifi_lcr_information* lcr);
    wifi_error (*wifi_start_sending_offloaded_packet)(wifi_request_id id,
                                                      wifi_interface_handle iface, u16 ether_type,
                                                      u8* ip_packet, u16 ip_packet_len,
                                                      u8* src_mac_addr, u8* dst_mac_addr,
                                                      u32 period_msec);
    wifi_error (*wifi_stop_sending_offloaded_packet)(wifi_request_id id,
                                                     wifi_interface_handle iface);
    wifi_error (*wifi_start_rssi_monitoring)(wifi_request_id id, wifi_interface_handle iface,
                                             s8 max_rssi, s8 min_rssi, wifi_rssi_event_handler eh);
    wifi_error (*wifi_stop_rssi_monitoring)(wifi_request_id id, wifi_interface_handle iface);
    wifi_error (*wifi_get_wake_reason_stats)(wifi_interface_handle iface,
                                             WLAN_DRIVER_WAKE_REASON_CNT* wifi_wake_reason_cnt);
    wifi_error (*wifi_configure_nd_offload)(wifi_interface_handle iface, u8 enable);
    wifi_error (*wifi_get_driver_memory_dump)(wifi_interface_handle iface,
                                              wifi_driver_memory_dump_callbacks callbacks);
    wifi_error (*wifi_start_pkt_fate_monitoring)(wifi_interface_handle iface);
    wifi_error (*wifi_get_tx_pkt_fates)(wifi_interface_handle handle,
                                        wifi_tx_report* tx_report_bufs, size_t n_requested_fates,
                                        size_t* n_provided_fates);
    wifi_error (*wifi_get_rx_pkt_fates)(wifi_interface_handle handle,
                                        wifi_rx_report* rx_report_bufs, size_t n_requested_fates,
                                        size_t* n_provided_fates);
    /* NAN functions */
    wifi_error (*wifi_nan_enable_request)(transaction_id id, wifi_interface_handle iface,
                                          NanEnableRequest* msg);
    wifi_error (*wifi_nan_disable_request)(transaction_id id, wifi_interface_handle iface);
    wifi_error (*wifi_nan_publish_request)(transaction_id id, wifi_interface_handle iface,
                                           NanPublishRequest* msg);
    wifi_error (*wifi_nan_publish_cancel_request)(transaction_id id, wifi_interface_handle iface,
                                                  NanPublishCancelRequest* msg);
    wifi_error (*wifi_nan_subscribe_request)(transaction_id id, wifi_interface_handle iface,
                                             NanSubscribeRequest* msg);
    wifi_error (*wifi_nan_subscribe_cancel_request)(transaction_id id, wifi_interface_handle iface,
                                                    NanSubscribeCancelRequest* msg);
    wifi_error (*wifi_nan_transmit_followup_request)(transaction_id id, wifi_interface_handle iface,
                                                     NanTransmitFollowupRequest* msg);
    wifi_error (*wifi_nan_stats_request)(transaction_id id, wifi_interface_handle iface,
                                         NanStatsRequest* msg);
    wifi_error (*wifi_nan_config_request)(transaction_id id, wifi_interface_handle iface,
                                          NanConfigRequest* msg);
    wifi_error (*wifi_nan_tca_request)(transaction_id id, wifi_interface_handle iface,
                                       NanTCARequest* msg);
    wifi_error (*wifi_nan_beacon_sdf_payload_request)(transaction_id id,
                                                      wifi_interface_handle iface,
                                                      NanBeaconSdfPayloadRequest* msg);
    wifi_error (*wifi_nan_register_handler)(wifi_interface_handle iface,
                                            NanCallbackHandler handlers);
    wifi_error (*wifi_nan_get_version)(wifi_handle handle, NanVersion* version);
    wifi_error (*wifi_nan_get_capabilities)(transaction_id id, wifi_interface_handle iface);
    wifi_error (*wifi_nan_data_interface_create)(transaction_id id, wifi_interface_handle iface,
                                                 char* iface_name);
    wifi_error (*wifi_nan_data_interface_delete)(transaction_id id, wifi_interface_handle iface,
                                                 char* iface_name);
    wifi_error (*wifi_nan_data_request_initiator)(transaction_id id, wifi_interface_handle iface,
                                                  NanDataPathInitiatorRequest* msg);
    wifi_error (*wifi_nan_data_indication_response)(transaction_id id, wifi_interface_handle iface,
                                                    NanDataPathIndicationResponse* msg);
    wifi_error (*wifi_nan_data_end)(transaction_id id, wifi_interface_handle iface,
                                    NanDataPathEndRequest* msg);
    wifi_error (*wifi_select_tx_power_scenario)(wifi_interface_handle iface,
                                                wifi_power_scenario scenario);
    wifi_error (*wifi_reset_tx_power_scenario)(wifi_interface_handle iface);
    /**
     * Returns the chipset's hardware filtering capabilities:
     * @param version pointer to version of the packet filter interpreter
     *                supported, filled in upon return. 0 indicates no support.
     * @param max_len pointer to maximum size of the filter bytecode, filled in
     *                upon return.
     */
    wifi_error (*wifi_get_packet_filter_capabilities)(wifi_interface_handle handle, u32* version,
                                                      u32* max_len);
    /**
     * Programs the packet filter.
     * @param program pointer to the program byte-code.
     * @param len length of the program byte-code.
     */
    wifi_error (*wifi_set_packet_filter)(wifi_interface_handle handle, const u8* program, u32 len);
    wifi_error (*wifi_read_packet_filter)(wifi_interface_handle handle, u32 src_offset,
                                          u8* host_dst, u32 length);
    wifi_error (*wifi_get_roaming_capabilities)(wifi_interface_handle handle,
                                                wifi_roaming_capabilities* caps);
    wifi_error (*wifi_enable_firmware_roaming)(wifi_interface_handle handle,
                                               fw_roaming_state_t state);
    wifi_error (*wifi_configure_roaming)(wifi_interface_handle handle,
                                         wifi_roaming_config* roaming_config);
    wifi_error (*wifi_set_radio_mode_change_handler)(wifi_request_id id,
                                                     wifi_interface_handle iface,
                                                     wifi_radio_mode_change_handler eh);
    wifi_error (*wifi_set_latency_mode)(wifi_interface_handle iface, wifi_latency_mode mode);
    wifi_error (*wifi_set_thermal_mitigation_mode)(wifi_handle handle, wifi_thermal_mode mode,
                                                   u32 completion_window);
    wifi_error (*wifi_map_dscp_access_category)(wifi_handle handle, u32 start, u32 end,
                                                u32 access_category);
    wifi_error (*wifi_reset_dscp_mapping)(wifi_handle handle);
    wifi_error (*wifi_virtual_interface_create)(wifi_handle handle, const char* ifname,
                                                wifi_interface_type iface_type);
    wifi_error (*wifi_virtual_interface_delete)(wifi_handle handle, const char* ifname);
    wifi_error (*wifi_set_subsystem_restart_handler)(wifi_handle handle,
                                                     wifi_subsystem_restart_handler handler);
    /**
     * Allow vendor HAL to choose interface name when creating
     * an interface. This can be implemented by chips with their
     * own interface naming policy.
     * If not implemented, the default naming will be used.
     */
    wifi_error (*wifi_get_supported_iface_name)(wifi_handle handle, u32 iface_type, char* name,
                                                size_t len);
    /**
     * Perform early initialization steps that are needed when WIFI
     * is disabled.
     * If the function returns failure, it means the vendor HAL is unusable
     * (for example, if chip hardware is not installed) and no further
     * functions should be called.
     */
    wifi_error (*wifi_early_initialize)(void);
    /**
     * Get supported feature set which are chip-global, that is
     * not dependent on any created interface.
     */
    wifi_error (*wifi_get_chip_feature_set)(wifi_handle handle, feature_set* set);
    /**
     * Invoked to indicate that the provided iface is the primary STA iface when there are more
     * than 1 STA iface concurrently active.
     */
    wifi_error (*wifi_multi_sta_set_primary_connection)(wifi_handle handle,
                                                        wifi_interface_handle iface);
    /**
     * When there are 2 simultaneous STA connections, this use case hint
     * indicates what STA + STA use-case is being enabled by the framework.
     */
    wifi_error (*wifi_multi_sta_set_use_case)(wifi_handle handle, wifi_multi_sta_use_case use_case);
    /**
     * Invoked to indicate that the following list of wifi_coex_unsafe_channel should be avoided
     * with the specified restrictions.
     * @param unsafeChannels list of current |wifi_coex_unsafe_channel| to avoid.
     * @param restrictions bitmask of |wifi_coex_restriction| indicating wifi interfaces to
     *         restrict from the current unsafe channels.
     */
    wifi_error (*wifi_set_coex_unsafe_channels)(wifi_handle handle, u32 num_channels,
                                                wifi_coex_unsafe_channel* unsafeChannels,
                                                u32 restrictions);
    /**
     * Invoked to set voip optimization mode for the provided STA iface
     */
    wifi_error (*wifi_set_voip_mode)(wifi_interface_handle iface, wifi_voip_mode mode);
    /**@brief twt_register_handler
     *        Request to register TWT callback before sending any TWT request
     * @param wifi_interface_handle:
     * @param TwtCallbackHandler: callback function pointers
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_twt_register_handler)(wifi_interface_handle iface,
                                            TwtCallbackHandler handler);
    /**@brief twt_get_capability
     *        Request TWT capability
     * @param wifi_interface_handle:
     * @return Synchronous wifi_error and TwtCapabilitySet
     */
    wifi_error (*wifi_twt_get_capability)(wifi_interface_handle iface,
                                          TwtCapabilitySet* twt_cap_set);
    /**@brief twt_setup_request
     *        Request to send TWT setup frame
     * @param wifi_interface_handle:
     * @param TwtSetupRequest: detailed parameters of setup request
     * @return Synchronous wifi_error
     * @return Asynchronous EventTwtSetupResponse CB return TwtSetupResponse
     */
    wifi_error (*wifi_twt_setup_request)(wifi_interface_handle iface, TwtSetupRequest* msg);
    /**@brief twt_teardown_request
     *        Request to send TWT teardown frame
     * @param wifi_interface_handle:
     * @param TwtTeardownRequest: detailed parameters of teardown request
     * @return Synchronous wifi_error
     * @return Asynchronous EventTwtTeardownCompletion CB return TwtTeardownCompletion
     * TwtTeardownCompletion may also be received due to other events
     * like CSA, BTCX, TWT scheduler, MultiConnection, peer-initiated teardown, etc.
     */
    wifi_error (*wifi_twt_teardown_request)(wifi_interface_handle iface, TwtTeardownRequest* msg);
    /**@brief twt_info_frame_request
     *        Request to send TWT info frame
     * @param wifi_interface_handle:
     * @param TwtInfoFrameRequest: detailed parameters in info frame
     * @return Synchronous wifi_error
     * @return Asynchronous EventTwtInfoFrameReceived CB return TwtInfoFrameReceived
     * Driver may also receive Peer-initiated TwtInfoFrame
     */
    wifi_error (*wifi_twt_info_frame_request)(wifi_interface_handle iface,
                                              TwtInfoFrameRequest* msg);
    /**@brief twt_get_stats
     *        Request to get TWT stats
     * @param wifi_interface_handle:
     * @param config_id: configuration ID of TWT request
     * @return Synchronous wifi_error and TwtStats
     */
    wifi_error (*wifi_twt_get_stats)(wifi_interface_handle iface, u8 config_id, TwtStats* stats);
    /**@brief twt_clear_stats
     *        Request to clear TWT stats
     * @param wifi_interface_handle:
     * @param config_id: configuration ID of TWT request
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_twt_clear_stats)(wifi_interface_handle iface, u8 config_id);
    /**
     * Invoked to set DTIM configuration when the host is in the suspend mode
     * @param wifi_interface_handle:
     * @param multiplier: when STA in the power saving mode, the wake up interval will be set to
     *              1) multiplier * DTIM period if multiplier > 0.
     *              2) the device default value if multiplier <=0
     * Some implementations may apply an additional cap to wake up interval in the case of 1).
     */
    wifi_error (*wifi_set_dtim_config)(wifi_interface_handle handle, u32 multiplier);
    /**@brief wifi_get_usable_channels
     *        Request list of usable channels for the requested bands and modes. Usable
     *        implies channel is allowed as per regulatory for the current country code
     *        and not restricted due to other hard limitations (e.g. DFS, Coex) In
     *        certain modes (e.g. STA+SAP) there could be other hard restrictions
     *        since MCC operation many not be supported by SAP. This API also allows
     *        driver to return list of usable channels for each mode uniquely to
     *        distinguish cases where only a limited set of modes are allowed on
     *        a given channel e.g. srd channels may be supported for P2P but not
     *        for SAP or P2P-Client may be allowed on an indoor channel but P2P-GO
     *        may not be allowed. This API is not interface specific and will be
     *        used to query capabilities of driver in terms of what modes (STA, SAP,
     *        P2P_CLI, P2P_GO, NAN, TDLS) can be supported on each of the channels.
     * @param handle global wifi_handle
     * @param band_mask BIT MASK of WLAN_MAC* as represented by |wlan_mac_band|
     * @param iface_mode_mask BIT MASK of BIT(WIFI_INTERFACE_*) represented by
     *        |wifi_interface_mode|. Bitmask respresents all the modes that the
     *        caller is interested in (e.g. STA, SAP, WFD-CLI, WFD-GO, TDLS, NAN).
     *        Note: Bitmask does not represent concurrency matrix. If the caller
     *        is interested in CLI, GO modes, the iface_mode_mask would be set
     *        to WIFI_INTERFACE_P2P_CLIENT|WIFI_INTERFACE_P2P_GO.
     * @param filter_mask BIT MASK of WIFI_USABLE_CHANNEL_FILTER_* represented by
     *        |wifi_usable_channel_filter|. Indicates if the channel list should
     *        be filtered based on additional criteria. If filter_mask is not
     *        specified, driver should return list of usable channels purely
     *        based on regulatory constraints.
     * @param max_size maximum number of |wifi_usable_channel|
     * @param size actual number of |wifi_usable_channel| entries returned by driver
     * @param channels list of usable channels represented by |wifi_usable_channel|
     */
    wifi_error (*wifi_get_usable_channels)(wifi_handle handle, u32 band_mask, u32 iface_mode_mask,
                                           u32 filter_mask, u32 max_size, u32* size,
                                           wifi_usable_channel* channels);
    /**
     * Trigger wifi subsystem restart to reload firmware
     */
    wifi_error (*wifi_trigger_subsystem_restart)(wifi_handle handle);
    /**
     * Invoked to set that the device is operating in an indoor environment.
     * @param handle global wifi_handle
     * @param isIndoor: true if the device is operating in an indoor
     *        environment, false otherwise.
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_set_indoor_state)(wifi_handle handle, bool isIndoor);
    /**@brief wifi_get_supported_radio_combinations_matrix
     *        Request all the possible radio combinations this device can offer.
     * @param handle global wifi_handle
     * @param max_size maximum size allocated for filling the wifi_radio_combination_matrix
     * @param wifi_radio_combination_matrix to return all the possible radio
     *        combinations.
     * @param size actual size of wifi_radio_combination_matrix returned from
     *        lower layer
     *
     */
    wifi_error (*wifi_get_supported_radio_combinations_matrix)(
            wifi_handle handle, u32 max_size, u32* size,
            wifi_radio_combination_matrix* radio_combination_matrix);
    /**@brief wifi_nan_rtt_chre_enable_request
     *        Request to enable CHRE NAN RTT
     * @param transaction_id: NAN transaction id
     * @param wifi_interface_handle
     * @param NanEnableRequest request message
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_nan_rtt_chre_enable_request)(transaction_id id, wifi_interface_handle iface,
                                                   NanEnableRequest* msg);
    /**@brief wifi_nan_rtt_chre_disable_request
     *        Request to disable CHRE NAN RTT
     * @param transaction_id: NAN transaction id
     * @param wifi_interface_handle
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_nan_rtt_chre_disable_request)(transaction_id id, wifi_interface_handle iface);
    /**@brief wifi_chre_register_handler
     *        register a handler to get the state of CHR
     * @param wifi_interface_handle
     * @param wifi_chre_handler: callback function pointer
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_chre_register_handler)(wifi_interface_handle iface,
                                             wifi_chre_handler handler);
    /**@brief wifi_enable_tx_power_limits
     *        Enable WiFi Tx power limis
     * @param wifi_interface_handle
     * @param isEnable : If enable TX limit or not
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_enable_tx_power_limits)(wifi_interface_handle iface, bool isEnable);
    /**@brief wifi_get_cached_scan_results
     *        Retrieve scan results cached in wifi firmware
     * @param wifi_interface_handle
     * @param wifi_cached_scan_result_handler : callback function pointer
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_get_cached_scan_results)(wifi_interface_handle iface,
                                               wifi_cached_scan_result_handler handler);
    /**@brief wifi_get_chip_capabilities
     *        Retrieve capabilities supported by this chip
     * @param wifi_handle
     * @return Synchronous wifi_error and chip capabilites
     */
    wifi_error (*wifi_get_chip_capabilities)(wifi_handle handle,
                                             wifi_chip_capabilities* chip_capabilities);
    /**@brief wifi_get_supported_iface_concurrency_matrix
     *        Request all the possible interface concurrency combinations this
     *        Wifi Chip can offer.
     * @param handle global wifi_handle
     * @param wifi_iface_concurrency_matrix to return all the possible
     *        interface concurrency combinations.
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_get_supported_iface_concurrency_matrix)(
            wifi_handle handle, wifi_iface_concurrency_matrix* matrix);
    /**@brief wifi_enable_sta_channel_for_peer_network
     *        enable or disable the feature of allowing current STA-connected
     *        channel for WFA GO, SAP and Wi-Fi Aware when the regulatory allows.
     * @param handle global wifi_handle
     * @param channelCategoryEnableFlag bitmask of |wifi_channel_category|.
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_enable_sta_channel_for_peer_network)(wifi_handle handle,
                                                           u32 channelCategoryEnableFlag);
    /**@brief wifi_nan_suspend_request
     * Request that the specified NAN session be suspended.
     * @param transaction_id: NAN transaction id
     * @param wifi_interface_handle
     * @param NanSuspendRequest request message
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_nan_suspend_request)(transaction_id id, wifi_interface_handle iface,
                                           NanSuspendRequest* msg);
    /**@brief wifi_nan_resume_request
     * Request that the specified NAN session be resumed.
     * @param transaction_id: NAN transaction id
     * @param wifi_interface_handle
     * @param NanResumeRequest request message
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_nan_resume_request)(transaction_id id, wifi_interface_handle iface,
                                          NanResumeRequest* msg);
    wifi_error (*wifi_nan_pairing_request)(transaction_id id, wifi_interface_handle iface,
                                           NanPairingRequest* msg);
    wifi_error (*wifi_nan_pairing_indication_response)(transaction_id id,
                                                       wifi_interface_handle iface,
                                                       NanPairingIndicationResponse* msg);
    wifi_error (*wifi_nan_bootstrapping_request)(transaction_id id, wifi_interface_handle iface,
                                                 NanBootstrappingRequest* msg);
    wifi_error (*wifi_nan_bootstrapping_indication_response)(
            transaction_id id, wifi_interface_handle iface,
            NanBootstrappingIndicationResponse* msg);
    /**@brief wifi_set_scan_mode
     *        Notify driver/firmware current is scan only mode to allow lower
     *        level to optimize power consumption.
     * @param enable true if current is scan only mode
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_set_scan_mode)(wifi_interface_handle iface, bool enable);
    wifi_error (*wifi_nan_pairing_end)(transaction_id id, wifi_interface_handle iface,
                                       NanPairingEndRequest* msg);
    /**@brief wifi_set_mlo_mode
     * Set Multi-Link Operation mode.
     * @param handle global wifi_handle
     * @param mode: MLO mode
     * @return Synchronous wifi_error
     */
    wifi_error (*wifi_set_mlo_mode)(wifi_handle handle, wifi_mlo_mode mode);
    /*
     * when adding new functions make sure to add stubs in
     * wifi_legacy_hal_stubs.cpp::initHalFuncTableWithStubs
     */
} wifi_hal_legacy_fn;

typedef wifi_error (*init_wifi_vendor_hal_func_table_legacy_t)(wifi_hal_legacy_fn*);
