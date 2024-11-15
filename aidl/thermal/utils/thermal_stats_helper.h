/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/android/frameworks/stats/IStats.h>
#include <aidl/android/hardware/thermal/Temperature.h>
#include <android-base/chrono_utils.h>
#include <hardware/google/pixel/pixelstats/pixelatoms.pb.h>

#include <chrono>
#include <shared_mutex>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "thermal_info.h"

namespace aidl {
namespace android {
namespace hardware {
namespace thermal {
namespace implementation {

class ThermalHelper;

using aidl::android::frameworks::stats::IStats;
using aidl::android::frameworks::stats::VendorAtomValue;
using ::android::base::boot_clock;
using ::android::hardware::google::pixel::PixelAtoms::ThermalSensorAbnormalityDetected;
using std::chrono::system_clock;
using SystemTimePoint = std::chrono::time_point<std::chrono::system_clock>;

// Number of abnormal atoms to be logged per kUpdateIntervalMs
constexpr int kMaxAbnormalLoggingPerUpdateInterval = 20;
constexpr int kMaxStatsReportingFailCount = 3;
// Proto messages are 1-indexed and VendorAtom field numbers start at 2, so
// store everything in the values array at the index of the field number
// -2.
constexpr int kVendorAtomOffset = 2;
constexpr float kPrecisionThreshold = 1e-4;

struct StatsRecord {
    int cur_state; /* temperature / cdev state at current time */
    boot_clock::time_point cur_state_start_time;
    boot_clock::time_point last_stats_report_time = boot_clock::time_point::min();
    std::vector<std::chrono::milliseconds> time_in_state_ms; /* stats array */
    int report_fail_count = 0; /* Number of times failed to report stats */
    explicit StatsRecord(const size_t &time_in_state_size, int state = 0)
        : cur_state(state),
          cur_state_start_time(boot_clock::now()),
          last_stats_report_time(boot_clock::now()),
          report_fail_count(0) {
        time_in_state_ms = std::vector<std::chrono::milliseconds>(
                time_in_state_size, std::chrono::milliseconds::zero());
    }
    StatsRecord() = default;
    StatsRecord(const StatsRecord &) = default;
    StatsRecord &operator=(const StatsRecord &) = default;
    StatsRecord(StatsRecord &&) = default;
    StatsRecord &operator=(StatsRecord &&) = default;
    ~StatsRecord() = default;
};

template <typename ValueType>
struct StatsByThreshold {
    std::vector<ValueType> thresholds;
    std::optional<std::string> logging_name;
    StatsRecord stats_record;
    explicit StatsByThreshold(ThresholdList<ValueType> threshold_list)
        : thresholds(threshold_list.thresholds), logging_name(threshold_list.logging_name) {
        // number of states = number of thresholds + 1
        // e.g. threshold: [30, 50, 60]
        //      buckets: [MIN - 30, 30 - 50, 50-60, 60-MAX]
        int time_in_state_size = threshold_list.thresholds.size() + 1;
        stats_record = StatsRecord(time_in_state_size);
    }
    StatsByThreshold() = default;
    StatsByThreshold(const StatsByThreshold &) = default;
    StatsByThreshold &operator=(const StatsByThreshold &) = default;
    StatsByThreshold(StatsByThreshold &&) = default;
    StatsByThreshold &operator=(StatsByThreshold &&) = default;
    ~StatsByThreshold() = default;
};

template <typename ValueType>
struct ThermalStats {
    std::vector<StatsByThreshold<ValueType>> stats_by_custom_threshold;
    std::optional<StatsRecord> stats_by_default_threshold;
};

struct SensorTempStats : ThermalStats<float> {
    float max_temp = std::numeric_limits<float>::min();
    SystemTimePoint max_temp_timestamp = SystemTimePoint::min();
    float min_temp = std::numeric_limits<float>::max();
    SystemTimePoint min_temp_timestamp = SystemTimePoint::min();
};

struct CurrTempStatus {
    float temp;
    boot_clock::time_point start_time;
    int repeat_count;
};

struct SensorStats {
    // Temperature residency stats for each sensor being watched
    std::unordered_map<std::string, SensorTempStats> temp_stats_map_;
    // Min, Max Temp threshold info for each sensor being monitored
    std::unordered_map<std::string, std::shared_ptr<TempRangeInfo>> temp_range_info_map_;
    // Temperature Stuck info for each sensor being monitored
    std::unordered_map<std::string, std::shared_ptr<TempStuckInfo>> temp_stuck_info_map_;
    // Current temperature status for each sensor being monitored for stuck
    std::unordered_map<std::string, CurrTempStatus> curr_temp_status_map_;
};

class ThermalStatsHelper {
  public:
    ThermalStatsHelper() = default;
    ~ThermalStatsHelper() = default;
    // Disallow copy and assign
    ThermalStatsHelper(const ThermalStatsHelper &) = delete;
    void operator=(const ThermalStatsHelper &) = delete;

    bool initializeStats(const Json::Value &config,
                         const std::unordered_map<std::string, SensorInfo> &sensor_info_map_,
                         const std::unordered_map<std::string, CdevInfo> &cooling_device_info_map_,
                         ThermalHelper *const thermal_helper_handle);
    void updateSensorCdevRequestStats(std::string_view trigger_sensor, std::string_view cdev,
                                      int new_state);
    void updateSensorTempStatsBySeverity(std::string_view sensor,
                                         const ThrottlingSeverity &severity);
    void updateSensorTempStatsByThreshold(std::string_view sensor, float temperature);
    /*
     * Function to report all the stats by calling all specific stats reporting function.
     * Returns:
     *   0, if time_elapsed < kUpdateIntervalMs or if no failure in reporting
     *  -1, if failed to get AIDL stats services
     *  >0, count represents the number of stats failed to report.
     */
    int reportStats();
    bool reportThermalAbnormality(const ThermalSensorAbnormalityDetected::AbnormalityType &type,
                                  std::string_view name, std::optional<int> reading);
    // Get a snapshot of Thermal Stats Sensor Map till that point in time
    std::unordered_map<std::string, SensorTempStats> GetSensorTempStatsSnapshot();
    // Get a snapshot of Thermal Stats Sensor Map till that point in time
    std::unordered_map<std::string, std::unordered_map<std::string, ThermalStats<int>>>
    GetSensorCoolingDeviceRequestStatsSnapshot();

  private:
    static constexpr std::chrono::milliseconds kUpdateIntervalMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(24h);
    boot_clock::time_point last_total_stats_report_time = boot_clock::time_point::min();
    int abnormal_stats_reported_per_update_interval = 0;
    mutable std::shared_mutex sensor_stats_mutex_;
    SensorStats sensor_stats;
    mutable std::shared_mutex sensor_cdev_request_stats_map_mutex_;
    ThermalHelper *thermal_helper_handle_;
    // userVote request stat for the sensor to the corresponding cdev (sensor -> cdev ->
    // StatsRecord)
    std::unordered_map<std::string, std::unordered_map<std::string, ThermalStats<int>>>
            sensor_cdev_request_stats_map_;

    bool initializeSensorTempStats(
            const StatsInfo<float> &sensor_stats_info,
            const std::unordered_map<std::string, SensorInfo> &sensor_info_map_);
    bool initializeSensorCdevRequestStats(
            const StatsInfo<int> &request_stats_info,
            const std::unordered_map<std::string, SensorInfo> &sensor_info_map_,
            const std::unordered_map<std::string, CdevInfo> &cooling_device_info_map_);
    bool initializeSensorAbnormalityStats(
            const AbnormalStatsInfo &abnormal_stats_info,
            const std::unordered_map<std::string, SensorInfo> &sensor_info_map_);
    void updateStatsRecord(StatsRecord *stats_record, int new_state);
    void verifySensorAbnormality(std::string_view sensor, float temperature);
    int reportAllSensorTempStats(const std::shared_ptr<IStats> &stats_client);
    bool reportSensorTempStats(const std::shared_ptr<IStats> &stats_client, std::string_view sensor,
                               const SensorTempStats &sensor_temp_stats, StatsRecord *stats_record);
    int reportAllSensorCdevRequestStats(const std::shared_ptr<IStats> &stats_client);
    bool reportSensorCdevRequestStats(const std::shared_ptr<IStats> &stats_client,
                                      std::string_view sensor, std::string_view cdev,
                                      StatsRecord *stats_record);
    bool reportAtom(const std::shared_ptr<IStats> &stats_client, const int32_t &atom_id,
                    std::vector<VendorAtomValue> &&values);
    std::vector<int64_t> processStatsRecordForReporting(StatsRecord *stats_record);
    StatsRecord restoreStatsRecordOnFailure(StatsRecord &&stats_record_before_failure);
};

}  // namespace implementation
}  // namespace thermal
}  // namespace hardware
}  // namespace android
}  // namespace aidl
