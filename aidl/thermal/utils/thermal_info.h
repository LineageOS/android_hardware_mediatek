/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/android/hardware/thermal/CoolingType.h>
#include <aidl/android/hardware/thermal/TemperatureType.h>
#include <aidl/android/hardware/thermal/ThrottlingSeverity.h>
#include <json/value.h>

#include <chrono>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "virtualtemp_estimator/virtualtemp_estimator.h"

namespace aidl {
namespace android {
namespace hardware {
namespace thermal {
namespace implementation {

constexpr size_t kThrottlingSeverityCount =
        std::distance(::ndk::enum_range<ThrottlingSeverity>().begin(),
                      ::ndk::enum_range<ThrottlingSeverity>().end());
using ThrottlingArray = std::array<float, static_cast<size_t>(kThrottlingSeverityCount)>;
using CdevArray = std::array<int, static_cast<size_t>(kThrottlingSeverityCount)>;
constexpr std::chrono::milliseconds kMinPollIntervalMs = std::chrono::milliseconds(2000);
constexpr std::chrono::milliseconds kUeventPollTimeoutMs = std::chrono::milliseconds(300000);
// TODO(b/292044404): Add debug config to make them easily configurable
constexpr std::chrono::milliseconds kPowerLogIntervalMs = std::chrono::milliseconds(60000);
constexpr int kMaxPowerLogPerLine = 6;
// Max number of time_in_state buckets is 20 in atoms
// VendorSensorCoolingDeviceStats, VendorTempResidencyStats
constexpr int kMaxStatsResidencyCount = 20;
constexpr int kMaxStatsThresholdCount = kMaxStatsResidencyCount - 1;

enum class FormulaOption : uint32_t {
    COUNT_THRESHOLD = 0,
    WEIGHTED_AVG,
    MAXIMUM,
    MINIMUM,
    USE_ML_MODEL,
    USE_LINEAR_MODEL
};

template <typename T>
struct ThresholdList {
    std::optional<std::string> logging_name;
    std::vector<T> thresholds;
    explicit ThresholdList(std::optional<std::string> logging_name, std::vector<T> thresholds)
        : logging_name(logging_name), thresholds(thresholds) {}

    ThresholdList() = default;
    ThresholdList(const ThresholdList &) = default;
    ThresholdList &operator=(const ThresholdList &) = default;
    ThresholdList(ThresholdList &&) = default;
    ThresholdList &operator=(ThresholdList &&) = default;
    ~ThresholdList() = default;
};

template <typename T>
struct StatsInfo {
    // if bool, record all or none depending on flag
    // if set, check name present in set
    std::variant<bool, std::unordered_set<std::string> >
            record_by_default_threshold_all_or_name_set_;
    // map name to list of thresholds
    std::unordered_map<std::string, std::vector<ThresholdList<T> > > record_by_threshold;
    void clear() {
        record_by_default_threshold_all_or_name_set_ = false;
        record_by_threshold.clear();
    }
};

struct StatsConfig {
    StatsInfo<float> sensor_stats_info;
    StatsInfo<int> cooling_device_request_info;
    void clear() {
        sensor_stats_info.clear();
        cooling_device_request_info.clear();
    }
};

struct TempRangeInfo {
    int max_temp_threshold;
    int min_temp_threshold;
};

struct TempStuckInfo {
    int min_polling_count;
    std::chrono::milliseconds min_stuck_duration;
};

struct AbnormalStatsInfo {
    struct SensorsTempRangeInfo {
        std::vector<std::string> sensors;
        TempRangeInfo temp_range_info;
    };
    struct SensorsTempStuckInfo {
        std::vector<std::string> sensors;
        TempStuckInfo temp_stuck_info;
    };

    std::optional<TempRangeInfo> default_temp_range_info;
    std::vector<SensorsTempRangeInfo> sensors_temp_range_infos;
    std::optional<TempStuckInfo> default_temp_stuck_info;
    std::vector<SensorsTempStuckInfo> sensors_temp_stuck_infos;
};

enum class SensorFusionType : uint32_t {
    SENSOR = 0,
    ODPM,
    CONSTANT,
};

std::ostream &operator<<(std::ostream &os, const SensorFusionType &sensor_fusion_type);

struct VirtualSensorInfo {
    std::vector<std::string> linked_sensors;
    std::vector<SensorFusionType> linked_sensors_type;
    std::vector<std::string> coefficients;
    std::vector<SensorFusionType> coefficients_type;

    float offset;
    std::vector<std::string> trigger_sensors;
    FormulaOption formula;
    std::string vt_estimator_model_file;
    std::unique_ptr<::thermal::vtestimator::VirtualTempEstimator> vt_estimator;
};

struct VirtualPowerRailInfo {
    std::vector<std::string> linked_power_rails;
    std::vector<float> coefficients;
    float offset;
    FormulaOption formula;
};

// The method when the ODPM power is lower than threshold
enum class ReleaseLogic : uint32_t {
    INCREASE = 0,      // Increase throttling by step
    DECREASE,          // Decrease throttling by step
    STEPWISE,          // Support both increase and decrease logix
    RELEASE_TO_FLOOR,  // Release throttling to floor directly
    NONE,
};

struct BindedCdevInfo {
    CdevArray limit_info;
    ThrottlingArray power_thresholds;
    ReleaseLogic release_logic;
    ThrottlingArray cdev_weight_for_pid;
    CdevArray cdev_ceiling;
    int max_release_step;
    int max_throttle_step;
    CdevArray cdev_floor_with_power_link;
    std::string power_rail;
    // The flag for activate release logic when power is higher than power threshold
    bool high_power_check;
    // The flag for only triggering throttling until all power samples are collected
    bool throttling_with_power_link;
    bool enabled;
};

// The map to store the CDEV throttling info for each profile
using ProfileMap = std::unordered_map<std::string, std::unordered_map<std::string, BindedCdevInfo>>;

struct ThrottlingInfo {
    ThrottlingArray k_po;
    ThrottlingArray k_pu;
    ThrottlingArray k_i;
    ThrottlingArray k_d;
    ThrottlingArray i_max;
    ThrottlingArray max_alloc_power;
    ThrottlingArray min_alloc_power;
    ThrottlingArray s_power;
    ThrottlingArray i_cutoff;
    float i_default;
    int tran_cycle;
    std::unordered_map<std::string, ThrottlingArray> excluded_power_info_map;
    std::unordered_map<std::string, BindedCdevInfo> binded_cdev_info_map;
    ProfileMap profile_map;
};

struct SensorInfo {
    TemperatureType type;
    ThrottlingArray hot_thresholds;
    ThrottlingArray cold_thresholds;
    ThrottlingArray hot_hysteresis;
    ThrottlingArray cold_hysteresis;
    std::string temp_path;
    std::string zone_name;
    float vr_threshold;
    float multiplier;
    std::chrono::milliseconds polling_delay;
    std::chrono::milliseconds passive_delay;
    std::chrono::milliseconds time_resolution;
    // The StepRatio value which is used for smoothing transient w/ the equation:
    // Temp = CurrentTemp * StepRatio + LastTemp * (1 - StepRatio)
    float step_ratio;
    bool send_cb;
    bool send_powerhint;
    bool is_watch;
    bool is_hidden;
    std::unique_ptr<VirtualSensorInfo> virtual_sensor_info;
    std::shared_ptr<ThrottlingInfo> throttling_info;
};

struct CdevInfo {
    CoolingType type;
    std::string read_path;
    std::string write_path;
    std::vector<float> state2power;
    int max_state;
};

struct PowerRailInfo {
    int power_sample_count;
    std::chrono::milliseconds power_sample_delay;
    std::unique_ptr<VirtualPowerRailInfo> virtual_power_rail_info;
};

bool ParseThermalConfig(std::string_view config_path, Json::Value *config);
bool ParseSensorInfo(const Json::Value &config,
                     std::unordered_map<std::string, SensorInfo> *sensors_parsed);
bool ParseCoolingDevice(const Json::Value &config,
                        std::unordered_map<std::string, CdevInfo> *cooling_device_parsed);
bool ParsePowerRailInfo(const Json::Value &config,
                        std::unordered_map<std::string, PowerRailInfo> *power_rail_parsed);
bool ParseSensorStatsConfig(const Json::Value &config,
                            const std::unordered_map<std::string, SensorInfo> &sensor_info_map_,
                            StatsInfo<float> *sensor_stats_info_parsed,
                            AbnormalStatsInfo *abnormal_stats_info_parsed);
bool ParseCoolingDeviceStatsConfig(
        const Json::Value &config,
        const std::unordered_map<std::string, CdevInfo> &cooling_device_info_map_,
        StatsInfo<int> *cooling_device_request_info_parsed);
}  // namespace implementation
}  // namespace thermal
}  // namespace hardware
}  // namespace android
}  // namespace aidl
