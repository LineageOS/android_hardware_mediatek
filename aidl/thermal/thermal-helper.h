/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/android/hardware/thermal/IThermal.h>

#include <array>
#include <chrono>
#include <map>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "utils/power_files.h"
#include "utils/powerhal_helper.h"
#include "utils/thermal_files.h"
#include "utils/thermal_info.h"
#include "utils/thermal_stats_helper.h"
#include "utils/thermal_throttling.h"
#include "utils/thermal_watcher.h"

namespace aidl {
namespace android {
namespace hardware {
namespace thermal {
namespace implementation {

using ::android::sp;

using NotificationCallback = std::function<void(const Temperature &t)>;

// Get thermal_zone type
bool getThermalZoneTypeById(int tz_id, std::string *);

struct ThermalSample {
    float temp;
    boot_clock::time_point timestamp;
};

struct EmulTemp {
    float temp;
    int severity;
};

struct OverrideStatus {
    std::unique_ptr<EmulTemp> emul_temp;
    bool max_throttling;
    bool pending_update;
};

struct SensorStatus {
    ThrottlingSeverity severity;
    ThrottlingSeverity prev_hot_severity;
    ThrottlingSeverity prev_cold_severity;
    boot_clock::time_point last_update_time;
    ThermalSample thermal_cached;
    bool pending_notification;
    OverrideStatus override_status;
};

class ThermalHelper {
  public:
    virtual ~ThermalHelper() = default;
    virtual bool fillCurrentTemperatures(bool filterType, bool filterCallback, TemperatureType type,
                                         std::vector<Temperature> *temperatures) = 0;
    virtual bool fillTemperatureThresholds(bool filterType, TemperatureType type,
                                           std::vector<TemperatureThreshold> *thresholds) const = 0;
    virtual bool fillCurrentCoolingDevices(bool filterType, CoolingType type,
                                           std::vector<CoolingDevice> *coolingdevices) const = 0;
    virtual bool emulTemp(std::string_view target_sensor, const float temp,
                          const bool max_throttling) = 0;
    virtual bool emulSeverity(std::string_view target_sensor, const int severity,
                              const bool max_throttling) = 0;
    virtual bool emulClear(std::string_view target_sensor) = 0;
    virtual bool isInitializedOk() const = 0;
    virtual bool readTemperature(std::string_view sensor_name, Temperature *out,
                                 const bool force_sysfs = false) = 0;
    virtual bool readTemperatureThreshold(std::string_view sensor_name,
                                          TemperatureThreshold *out) const = 0;
    virtual bool readCoolingDevice(std::string_view cooling_device, CoolingDevice *out) const = 0;
    virtual void dumpVtEstimatorStatus(std::string_view senor_name,
                                       std::ostringstream *dump_buf) const = 0;
    virtual const std::unordered_map<std::string, SensorInfo> &GetSensorInfoMap() const = 0;
    virtual const std::unordered_map<std::string, CdevInfo> &GetCdevInfoMap() const = 0;
    virtual const std::unordered_map<std::string, SensorStatus> &GetSensorStatusMap() const = 0;
    virtual const std::unordered_map<std::string, ThermalThrottlingStatus> &
    GetThermalThrottlingStatusMap() const = 0;
    virtual const std::unordered_map<std::string, PowerRailInfo> &GetPowerRailInfoMap() const = 0;
    virtual const std::unordered_map<std::string, PowerStatus> &GetPowerStatusMap() const = 0;
    virtual const std::unordered_map<std::string, SensorTempStats> GetSensorTempStatsSnapshot() = 0;
    virtual const std::unordered_map<std::string,
                                     std::unordered_map<std::string, ThermalStats<int>>>
    GetSensorCoolingDeviceRequestStatsSnapshot() = 0;
    virtual bool isAidlPowerHalExist() = 0;
    virtual bool isPowerHalConnected() = 0;
    virtual bool isPowerHalExtConnected() = 0;
    virtual void dumpTraces(std::string_view target_sensor) = 0;
};

class ThermalHelperImpl : public ThermalHelper {
  public:
    explicit ThermalHelperImpl(const NotificationCallback &cb);
    ~ThermalHelperImpl() override = default;

    bool fillCurrentTemperatures(bool filterType, bool filterCallback, TemperatureType type,
                                 std::vector<Temperature> *temperatures) override;
    bool fillTemperatureThresholds(bool filterType, TemperatureType type,
                                   std::vector<TemperatureThreshold> *thresholds) const override;
    bool fillCurrentCoolingDevices(bool filterType, CoolingType type,
                                   std::vector<CoolingDevice> *coolingdevices) const override;
    bool emulTemp(std::string_view target_sensor, const float temp,
                  const bool max_throttling) override;
    bool emulSeverity(std::string_view target_sensor, const int severity,
                      const bool max_throttling) override;
    bool emulClear(std::string_view target_sensor) override;
    void dumpTraces(std::string_view target_sensor) override;

    // Disallow copy and assign.
    ThermalHelperImpl(const ThermalHelperImpl &) = delete;
    void operator=(const ThermalHelperImpl &) = delete;

    bool isInitializedOk() const override { return is_initialized_; }

    // Read the temperature of a single sensor.
    bool readTemperature(std::string_view sensor_name, Temperature *out,
                         const bool force_sysfs = false) override;

    bool readTemperatureThreshold(std::string_view sensor_name,
                                  TemperatureThreshold *out) const override;
    // Read the value of a single cooling device.
    bool readCoolingDevice(std::string_view cooling_device, CoolingDevice *out) const override;
    // Dump VtEstimator status
    void dumpVtEstimatorStatus(std::string_view sensor_name,
                               std::ostringstream *dump_buf) const override;
    // Get SensorInfo Map
    const std::unordered_map<std::string, SensorInfo> &GetSensorInfoMap() const override {
        return sensor_info_map_;
    }
    // Get CdevInfo Map
    const std::unordered_map<std::string, CdevInfo> &GetCdevInfoMap() const override {
        return cooling_device_info_map_;
    }
    // Get SensorStatus Map
    const std::unordered_map<std::string, SensorStatus> &GetSensorStatusMap() const override {
        std::shared_lock<std::shared_mutex> _lock(sensor_status_map_mutex_);
        return sensor_status_map_;
    }
    // Get ThermalThrottling Map
    const std::unordered_map<std::string, ThermalThrottlingStatus> &GetThermalThrottlingStatusMap()
            const override {
        return thermal_throttling_.GetThermalThrottlingStatusMap();
    }
    // Get PowerRailInfo Map
    const std::unordered_map<std::string, PowerRailInfo> &GetPowerRailInfoMap() const override {
        return power_files_.GetPowerRailInfoMap();
    }

    // Get PowerStatus Map
    const std::unordered_map<std::string, PowerStatus> &GetPowerStatusMap() const override {
        return power_files_.GetPowerStatusMap();
    }

    // Get Thermal Stats Sensor Map
    const std::unordered_map<std::string, SensorTempStats> GetSensorTempStatsSnapshot() override {
        return thermal_stats_helper_.GetSensorTempStatsSnapshot();
    }
    // Get Thermal Stats Sensor, Binded Cdev State Request Map
    const std::unordered_map<std::string, std::unordered_map<std::string, ThermalStats<int>>>
    GetSensorCoolingDeviceRequestStatsSnapshot() override {
        return thermal_stats_helper_.GetSensorCoolingDeviceRequestStatsSnapshot();
    }

    bool isAidlPowerHalExist() override { return power_hal_service_.isAidlPowerHalExist(); }
    bool isPowerHalConnected() override { return power_hal_service_.isPowerHalConnected(); }
    bool isPowerHalExtConnected() override { return power_hal_service_.isPowerHalExtConnected(); }

  private:
    bool initializeSensorMap(const std::unordered_map<std::string, std::string> &path_map);
    bool initializeCoolingDevices(const std::unordered_map<std::string, std::string> &path_map);
    bool isSubSensorValid(std::string_view sensor_data, const SensorFusionType sensor_fusion_type);
    void setMinTimeout(SensorInfo *sensor_info);
    void initializeTrip(const std::unordered_map<std::string, std::string> &path_map,
                        std::set<std::string> *monitored_sensors, bool thermal_genl_enabled);
    void clearAllThrottling();
    // For thermal_watcher_'s polling thread, return the sleep interval
    std::chrono::milliseconds thermalWatcherCallbackFunc(
            const std::unordered_map<std::string, float> &uevent_sensor_map);
    // Return hot and cold severity status as std::pair
    std::pair<ThrottlingSeverity, ThrottlingSeverity> getSeverityFromThresholds(
            const ThrottlingArray &hot_thresholds, const ThrottlingArray &cold_thresholds,
            const ThrottlingArray &hot_hysteresis, const ThrottlingArray &cold_hysteresis,
            ThrottlingSeverity prev_hot_severity, ThrottlingSeverity prev_cold_severity,
            float value) const;
    // Read sensor data according to the type
    bool readDataByType(std::string_view sensor_data, float *reading_value,
                        const SensorFusionType type, const bool force_no_cache,
                        std::map<std::string, float> *sensor_log_map);
    bool readThermalSensor(std::string_view sensor_name, float *temp, const bool force_sysfs,
                           std::map<std::string, float> *sensor_log_map);
    bool runVirtualTempEstimator(std::string_view sensor_name,
                                 std::map<std::string, float> *sensor_log_map,
                                 const bool force_no_cache, std::vector<float> *outputs);
    size_t getPredictionMaxWindowMs(std::string_view sensor_name);
    float readPredictionAfterTimeMs(std::string_view sensor_name, const size_t time_ms);
    bool readTemperaturePredictions(std::string_view sensor_name, std::vector<float> *predictions);
    void updateCoolingDevices(const std::vector<std::string> &cooling_devices_to_update);
    // Check the max CDEV state for cdev_ceiling
    void maxCoolingRequestCheck(
            std::unordered_map<std::string, BindedCdevInfo> *binded_cdev_info_map);
    void checkUpdateSensorForEmul(std::string_view target_sensor, const bool max_throttling);
    ThrottlingSeverity getSeverityReference(std::string_view sensor_name);

    sp<ThermalWatcher> thermal_watcher_;
    PowerFiles power_files_;
    ThermalFiles thermal_sensors_;
    ThermalFiles cooling_devices_;
    ThermalThrottling thermal_throttling_;
    bool is_initialized_;
    const NotificationCallback cb_;
    std::unordered_map<std::string, CdevInfo> cooling_device_info_map_;
    std::unordered_map<std::string, SensorInfo> sensor_info_map_;
    std::unordered_map<std::string, std::unordered_map<ThrottlingSeverity, ThrottlingSeverity>>
            supported_powerhint_map_;
    PowerHalService power_hal_service_;
    ThermalStatsHelper thermal_stats_helper_;
    mutable std::shared_mutex sensor_status_map_mutex_;
    std::unordered_map<std::string, SensorStatus> sensor_status_map_;
};

}  // namespace implementation
}  // namespace thermal
}  // namespace hardware
}  // namespace android
}  // namespace aidl
