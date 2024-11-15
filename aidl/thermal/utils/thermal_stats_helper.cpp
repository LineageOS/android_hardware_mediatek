/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "thermal_stats_helper.h"

#include <android-base/logging.h>
#include <android/binder_manager.h>

#include <algorithm>
#include <numeric>
#include <string_view>

#include "../thermal-helper.h"

namespace aidl {
namespace android {
namespace hardware {
namespace thermal {
namespace implementation {

constexpr std::string_view kCustomThresholdSetSuffix("-TH-");
constexpr std::string_view kCompressedThresholdSuffix("-CMBN-TH");

using aidl::android::frameworks::stats::VendorAtom;
namespace PixelAtoms = ::android::hardware::google::pixel::PixelAtoms;

namespace {
static std::shared_ptr<IStats> stats_client = nullptr;
std::shared_ptr<IStats> getStatsService() {
    static std::once_flag statsServiceFlag;
    std::call_once(statsServiceFlag, []() {
        const std::string instance = std::string() + IStats::descriptor + "/default";
        bool isStatsDeclared = AServiceManager_isDeclared(instance.c_str());
        if (!isStatsDeclared) {
            LOG(ERROR) << "Stats service is not registered.";
            return;
        }
        stats_client = IStats::fromBinder(
                ndk::SpAIBinder(AServiceManager_waitForService(instance.c_str())));
    });
    return stats_client;
}

bool isRecordByDefaultThreshold(const std::variant<bool, std::unordered_set<std::string>>
                                        &record_by_default_threshold_all_or_name_set_,
                                std::string_view name) {
    if (std::holds_alternative<bool>(record_by_default_threshold_all_or_name_set_)) {
        return std::get<bool>(record_by_default_threshold_all_or_name_set_);
    }
    return std::get<std::unordered_set<std::string>>(record_by_default_threshold_all_or_name_set_)
            .count(name.data());
}

template <typename T>
int calculateThresholdBucket(const std::vector<T> &thresholds, T value) {
    if (thresholds.empty()) {
        LOG(VERBOSE) << "No threshold present, so bucket is " << value << " as int.";
        return static_cast<int>(value);
    }
    auto threshold_idx = std::upper_bound(thresholds.begin(), thresholds.end(), value);
    int bucket = (threshold_idx - thresholds.begin());
    LOG(VERBOSE) << "For value: " << value << " bucket is: " << bucket;
    return bucket;
}

void resetCurrentTempStatus(CurrTempStatus *curr_temp_status, float new_temp) {
    curr_temp_status->temp = new_temp;
    curr_temp_status->start_time = boot_clock::now();
    curr_temp_status->repeat_count = 1;
}

}  // namespace

bool ThermalStatsHelper::initializeStats(
        const Json::Value &config,
        const std::unordered_map<std::string, SensorInfo> &sensor_info_map_,
        const std::unordered_map<std::string, CdevInfo> &cooling_device_info_map_,
        ThermalHelper *const thermal_helper_handle) {
    StatsInfo<float> sensor_stats_info;
    AbnormalStatsInfo abnormal_stats_info;
    if (!ParseSensorStatsConfig(config, sensor_info_map_, &sensor_stats_info,
                                &abnormal_stats_info)) {
        LOG(ERROR) << "Failed to parse sensor stats config";
        return false;
    }
    StatsInfo<int> cooling_device_request_info;
    if (!ParseCoolingDeviceStatsConfig(config, cooling_device_info_map_,
                                       &cooling_device_request_info)) {
        LOG(ERROR) << "Failed to parse cooling device stats config";
        return false;
    }
    if (!initializeSensorTempStats(sensor_stats_info, sensor_info_map_)) {
        LOG(ERROR) << "Failed to initialize sensor temp stats";
        return false;
    }
    if (!initializeSensorCdevRequestStats(cooling_device_request_info, sensor_info_map_,
                                          cooling_device_info_map_)) {
        LOG(ERROR) << "Failed to initialize sensor cooling device request stats";
        return false;
    }
    if (!initializeSensorAbnormalityStats(abnormal_stats_info, sensor_info_map_)) {
        LOG(ERROR) << "Failed to initialize sensor abnormal stats";
        return false;
    }

    thermal_helper_handle_ = thermal_helper_handle;
    last_total_stats_report_time = boot_clock::now();
    abnormal_stats_reported_per_update_interval = 0;
    LOG(INFO) << "Thermal Stats Initialized Successfully";
    return true;
}

bool ThermalStatsHelper::initializeSensorCdevRequestStats(
        const StatsInfo<int> &request_stats_info,
        const std::unordered_map<std::string, SensorInfo> &sensor_info_map_,
        const std::unordered_map<std::string, CdevInfo> &cooling_device_info_map_) {
    std::unique_lock<std::shared_mutex> _lock(sensor_cdev_request_stats_map_mutex_);
    for (const auto &[sensor, sensor_info] : sensor_info_map_) {
        for (const auto &binded_cdev_info_pair :
             sensor_info.throttling_info->binded_cdev_info_map) {
            const auto &cdev = binded_cdev_info_pair.first;
            const auto &max_state =
                    cooling_device_info_map_.at(binded_cdev_info_pair.first).max_state;
            // Record by all state
            if (isRecordByDefaultThreshold(
                        request_stats_info.record_by_default_threshold_all_or_name_set_, cdev)) {
                // if the number of states is greater / equal(as state starts from 0) than
                // residency_buckets in atom combine the initial states
                if (max_state >= kMaxStatsResidencyCount) {
                    // buckets = [max_state -kMaxStatsResidencyCount + 1, ...max_state]
                    //     idx = [1, .. max_state - (max_state - kMaxStatsResidencyCount + 1) + 1]
                    //     idx = [1, .. kMaxStatsResidencyCount]
                    const auto starting_state = max_state - kMaxStatsResidencyCount + 1;
                    std::vector<int> thresholds(kMaxStatsResidencyCount);
                    std::iota(thresholds.begin(), thresholds.end(), starting_state);
                    const auto logging_name = cdev + kCompressedThresholdSuffix.data();
                    ThresholdList<int> threshold_list(logging_name, thresholds);
                    sensor_cdev_request_stats_map_[sensor][cdev]
                            .stats_by_custom_threshold.emplace_back(threshold_list);
                } else {
                    // buckets = [0, 1, 2, 3, ...max_state]
                    const auto default_threshold_time_in_state_size = max_state + 1;
                    sensor_cdev_request_stats_map_[sensor][cdev].stats_by_default_threshold =
                            StatsRecord(default_threshold_time_in_state_size);
                }
                LOG(INFO) << "Sensor Cdev user vote stats on basis of all state initialized for ["
                          << sensor << "-" << cdev << "]";
            }

            // Record by custom threshold
            if (request_stats_info.record_by_threshold.count(cdev)) {
                for (const auto &threshold_list : request_stats_info.record_by_threshold.at(cdev)) {
                    // check last threshold value(which is >= number of buckets as numbers in
                    // threshold are strictly increasing from 0) is less than max_state
                    if (threshold_list.thresholds.back() >= max_state) {
                        LOG(ERROR) << "For sensor " << sensor << " bindedCdev: " << cdev
                                   << "Invalid bindedCdev stats threshold: "
                                   << threshold_list.thresholds.back() << " >= " << max_state;
                        sensor_cdev_request_stats_map_.clear();
                        return false;
                    }
                    sensor_cdev_request_stats_map_[sensor][cdev]
                            .stats_by_custom_threshold.emplace_back(threshold_list);
                    LOG(INFO)
                            << "Sensor Cdev user vote stats on basis of threshold initialized for ["
                            << sensor << "-" << cdev << "]";
                }
            }
        }
    }
    return true;
}

bool ThermalStatsHelper::initializeSensorTempStats(
        const StatsInfo<float> &sensor_stats_info,
        const std::unordered_map<std::string, SensorInfo> &sensor_info_map_) {
    std::unique_lock<std::shared_mutex> _lock(sensor_stats_mutex_);
    auto &temp_stats_map_ = sensor_stats.temp_stats_map_;
    const int severity_time_in_state_size = kThrottlingSeverityCount;
    for (const auto &[sensor, sensor_info] : sensor_info_map_) {
        // Record by severity
        if (sensor_info.is_watch &&
            isRecordByDefaultThreshold(
                    sensor_stats_info.record_by_default_threshold_all_or_name_set_, sensor)) {
            // number of buckets = number of severity
            temp_stats_map_[sensor].stats_by_default_threshold =
                    StatsRecord(severity_time_in_state_size);
            LOG(INFO) << "Sensor temp stats on basis of severity initialized for [" << sensor
                      << "]";
        }

        // Record by custom threshold
        if (sensor_stats_info.record_by_threshold.count(sensor)) {
            for (const auto &threshold_list : sensor_stats_info.record_by_threshold.at(sensor)) {
                temp_stats_map_[sensor].stats_by_custom_threshold.emplace_back(threshold_list);
                LOG(INFO) << "Sensor temp stats on basis of threshold initialized for [" << sensor
                          << "]";
            }
        }
    }
    return true;
}

bool ThermalStatsHelper::initializeSensorAbnormalityStats(
        const AbnormalStatsInfo &abnormal_stats_info,
        const std::unordered_map<std::string, SensorInfo> &sensor_info_map_) {
    std::unique_lock<std::shared_mutex> _lock(sensor_stats_mutex_);
    auto &temp_range_info_map_ = sensor_stats.temp_range_info_map_;
    for (const auto &sensors_temp_range_info : abnormal_stats_info.sensors_temp_range_infos) {
        const auto &temp_range_info_ptr =
                std::make_shared<TempRangeInfo>(sensors_temp_range_info.temp_range_info);
        for (const auto &sensor : sensors_temp_range_info.sensors) {
            temp_range_info_map_[sensor] = temp_range_info_ptr;
        }
    }
    auto &temp_stuck_info_map_ = sensor_stats.temp_stuck_info_map_;
    for (const auto &sensors_temp_stuck_info : abnormal_stats_info.sensors_temp_stuck_infos) {
        const auto &temp_stuck_info_ptr =
                std::make_shared<TempStuckInfo>(sensors_temp_stuck_info.temp_stuck_info);
        for (const auto &sensor : sensors_temp_stuck_info.sensors) {
            temp_stuck_info_map_[sensor] = temp_stuck_info_ptr;
        }
    }
    const auto &default_temp_range_info_ptr =
            abnormal_stats_info.default_temp_range_info
                    ? std::make_shared<TempRangeInfo>(
                              abnormal_stats_info.default_temp_range_info.value())
                    : nullptr;
    const auto &default_temp_stuck_info_ptr =
            abnormal_stats_info.default_temp_stuck_info
                    ? std::make_shared<TempStuckInfo>(
                              abnormal_stats_info.default_temp_stuck_info.value())
                    : nullptr;
    for (const auto &sensor_info : sensor_info_map_) {
        const auto &sensor = sensor_info.first;
        if (default_temp_range_info_ptr && !temp_range_info_map_.count(sensor))
            temp_range_info_map_[sensor] = default_temp_range_info_ptr;
        if (default_temp_stuck_info_ptr && !temp_stuck_info_map_.count(sensor))
            temp_stuck_info_map_[sensor] = default_temp_stuck_info_ptr;
    }

    for (const auto &sensor_temp_stuck_info : temp_stuck_info_map_) {
        sensor_stats.curr_temp_status_map_[sensor_temp_stuck_info.first] = {
                .temp = std::numeric_limits<float>::min(),
                .start_time = boot_clock::time_point::min(),
                .repeat_count = 0,
        };
    }
    return true;
}

void ThermalStatsHelper::updateStatsRecord(StatsRecord *stats_record, int new_state) {
    const auto now = boot_clock::now();
    const auto cur_state_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - stats_record->cur_state_start_time);
    LOG(VERBOSE) << "Adding duration " << cur_state_duration.count()
                 << " for cur_state: " << stats_record->cur_state << " with value: "
                 << stats_record->time_in_state_ms[stats_record->cur_state].count();
    // Update last record end time
    stats_record->time_in_state_ms[stats_record->cur_state] += cur_state_duration;
    stats_record->cur_state_start_time = now;
    stats_record->cur_state = new_state;
}

void ThermalStatsHelper::updateSensorCdevRequestStats(std::string_view sensor,
                                                      std::string_view cdev, int new_value) {
    std::unique_lock<std::shared_mutex> _lock(sensor_cdev_request_stats_map_mutex_);
    if (!sensor_cdev_request_stats_map_.count(sensor.data()) ||
        !sensor_cdev_request_stats_map_[sensor.data()].count(cdev.data())) {
        return;
    }
    auto &request_stats = sensor_cdev_request_stats_map_[sensor.data()][cdev.data()];
    for (auto &stats_by_threshold : request_stats.stats_by_custom_threshold) {
        int value = calculateThresholdBucket(stats_by_threshold.thresholds, new_value);
        if (value != stats_by_threshold.stats_record.cur_state) {
            LOG(VERBOSE) << "Updating bindedCdev stats for sensor: " << sensor.data()
                         << " , cooling_device: " << cdev.data() << " with new value: " << value;
            updateStatsRecord(&stats_by_threshold.stats_record, value);
        }
    }

    if (request_stats.stats_by_default_threshold.has_value()) {
        auto &stats_record = request_stats.stats_by_default_threshold.value();
        if (new_value != stats_record.cur_state) {
            LOG(VERBOSE) << "Updating bindedCdev stats for sensor: " << sensor.data()
                         << " , cooling_device: " << cdev.data()
                         << " with new value: " << new_value;
            updateStatsRecord(&stats_record, new_value);
        }
    }
}

void ThermalStatsHelper::updateSensorTempStatsByThreshold(std::string_view sensor,
                                                          float temperature) {
    std::unique_lock<std::shared_mutex> _lock(sensor_stats_mutex_);
    verifySensorAbnormality(sensor, temperature);
    auto &temp_stats_map_ = sensor_stats.temp_stats_map_;
    if (!temp_stats_map_.count(sensor.data())) {
        return;
    }
    auto &sensor_temp_stats = temp_stats_map_[sensor.data()];
    for (auto &stats_by_threshold : sensor_temp_stats.stats_by_custom_threshold) {
        int value = calculateThresholdBucket(stats_by_threshold.thresholds, temperature);
        if (value != stats_by_threshold.stats_record.cur_state) {
            LOG(VERBOSE) << "Updating sensor stats for sensor: " << sensor.data()
                         << " with value: " << value;
            updateStatsRecord(&stats_by_threshold.stats_record, value);
        }
    }
    if (temperature > sensor_temp_stats.max_temp) {
        sensor_temp_stats.max_temp = temperature;
        sensor_temp_stats.max_temp_timestamp = system_clock::now();
    }
    if (temperature < sensor_temp_stats.min_temp) {
        sensor_temp_stats.min_temp = temperature;
        sensor_temp_stats.min_temp_timestamp = system_clock::now();
    }
}

void ThermalStatsHelper::updateSensorTempStatsBySeverity(std::string_view sensor,
                                                         const ThrottlingSeverity &severity) {
    std::unique_lock<std::shared_mutex> _lock(sensor_stats_mutex_);
    auto &temp_stats_map_ = sensor_stats.temp_stats_map_;
    if (temp_stats_map_.count(sensor.data()) &&
        temp_stats_map_[sensor.data()].stats_by_default_threshold.has_value()) {
        auto &stats_record = temp_stats_map_[sensor.data()].stats_by_default_threshold.value();
        int value = static_cast<int>(severity);
        if (value != stats_record.cur_state) {
            LOG(VERBOSE) << "Updating sensor stats for sensor: " << sensor.data()
                         << " with value: " << value;
            updateStatsRecord(&stats_record, value);
        }
    }
}

void ThermalStatsHelper::verifySensorAbnormality(std::string_view sensor, float temp) {
    LOG(VERBOSE) << "Verify sensor abnormality for " << sensor << " with temp " << temp;
    if (sensor_stats.temp_range_info_map_.count(sensor.data())) {
        const auto &temp_range_info = sensor_stats.temp_range_info_map_[sensor.data()];
        if (temp < temp_range_info->min_temp_threshold) {
            LOG(ERROR) << "Outlier Temperature Detected, sensor: " << sensor.data()
                       << " temp: " << temp << " < " << temp_range_info->min_temp_threshold;
            reportThermalAbnormality(ThermalSensorAbnormalityDetected::EXTREME_LOW_TEMP, sensor,
                                     std::round(temp));
        } else if (temp > temp_range_info->max_temp_threshold) {
            LOG(ERROR) << "Outlier Temperature Detected, sensor: " << sensor.data()
                       << " temp: " << temp << " > " << temp_range_info->max_temp_threshold;
            reportThermalAbnormality(ThermalSensorAbnormalityDetected::EXTREME_HIGH_TEMP, sensor,
                                     std::round(temp));
        }
    }
    if (sensor_stats.temp_stuck_info_map_.count(sensor.data())) {
        const auto &temp_stuck_info = sensor_stats.temp_stuck_info_map_[sensor.data()];
        auto &curr_temp_status = sensor_stats.curr_temp_status_map_[sensor.data()];
        LOG(VERBOSE) << "Current Temp Status: temp=" << curr_temp_status.temp
                     << " repeat_count=" << curr_temp_status.repeat_count
                     << " start_time=" << curr_temp_status.start_time.time_since_epoch().count();
        if (std::fabs(curr_temp_status.temp - temp) <= kPrecisionThreshold) {
            curr_temp_status.repeat_count++;
            if (temp_stuck_info->min_polling_count <= curr_temp_status.repeat_count) {
                auto time_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        boot_clock::now() - curr_temp_status.start_time);
                if (temp_stuck_info->min_stuck_duration <= time_elapsed_ms) {
                    LOG(ERROR) << "Stuck Temperature Detected, sensor: " << sensor.data()
                               << " temp: " << temp << " repeated "
                               << temp_stuck_info->min_polling_count << " times for "
                               << time_elapsed_ms.count() << "ms";
                    if (reportThermalAbnormality(ThermalSensorAbnormalityDetected::SENSOR_STUCK,
                                                 sensor, std::round(temp))) {
                        // reset current status to verify for sensor stuck with start time as
                        // current polling
                        resetCurrentTempStatus(&curr_temp_status, temp);
                    }
                }
            }
        } else {
            resetCurrentTempStatus(&curr_temp_status, temp);
        }
    }
}

int ThermalStatsHelper::reportStats() {
    const auto curTime = boot_clock::now();
    const auto since_last_total_stats_update_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(curTime -
                                                                  last_total_stats_report_time);
    LOG(VERBOSE) << "Duration from last total stats update is: "
                 << since_last_total_stats_update_ms.count();
    if (since_last_total_stats_update_ms < kUpdateIntervalMs) {
        LOG(VERBOSE) << "Time elapsed since last update less than " << kUpdateIntervalMs.count();
        return 0;
    }

    const std::shared_ptr<IStats> stats_client = getStatsService();
    if (!stats_client) {
        LOG(ERROR) << "Unable to get AIDL Stats service";
        return -1;
    }
    int count_failed_reporting =
            reportAllSensorTempStats(stats_client) + reportAllSensorCdevRequestStats(stats_client);
    last_total_stats_report_time = curTime;
    abnormal_stats_reported_per_update_interval = 0;
    return count_failed_reporting;
}

int ThermalStatsHelper::reportAllSensorTempStats(const std::shared_ptr<IStats> &stats_client) {
    int count_failed_reporting = 0;
    std::unique_lock<std::shared_mutex> _lock(sensor_stats_mutex_);
    for (auto &[sensor, temp_stats] : sensor_stats.temp_stats_map_) {
        for (size_t threshold_set_idx = 0;
             threshold_set_idx < temp_stats.stats_by_custom_threshold.size(); threshold_set_idx++) {
            auto &stats_by_threshold = temp_stats.stats_by_custom_threshold[threshold_set_idx];
            std::string sensor_name = stats_by_threshold.logging_name.value_or(
                    sensor + kCustomThresholdSetSuffix.data() + std::to_string(threshold_set_idx));
            if (!reportSensorTempStats(stats_client, sensor_name, temp_stats,
                                       &stats_by_threshold.stats_record)) {
                count_failed_reporting++;
            }
        }
        if (temp_stats.stats_by_default_threshold.has_value()) {
            if (!reportSensorTempStats(stats_client, sensor, temp_stats,
                                       &temp_stats.stats_by_default_threshold.value())) {
                count_failed_reporting++;
            }
        }
        // Reset temp stats after reporting
        temp_stats.max_temp = std::numeric_limits<float>::min();
        temp_stats.min_temp = std::numeric_limits<float>::max();
    }
    return count_failed_reporting;
}

bool ThermalStatsHelper::reportSensorTempStats(const std::shared_ptr<IStats> &stats_client,
                                               std::string_view sensor,
                                               const SensorTempStats &sensor_temp_stats,
                                               StatsRecord *stats_record) {
    LOG(VERBOSE) << "Reporting sensor stats for " << sensor;
    // maintain a copy in case reporting fails
    StatsRecord thermal_stats_before_reporting = *stats_record;
    std::vector<VendorAtomValue> values(2);
    values[0].set<VendorAtomValue::stringValue>(sensor);
    std::vector<int64_t> time_in_state_ms = processStatsRecordForReporting(stats_record);
    const auto since_last_update_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            stats_record->cur_state_start_time - stats_record->last_stats_report_time);
    values[1].set<VendorAtomValue::longValue>(since_last_update_ms.count());
    VendorAtomValue tmp;
    for (auto &time_in_state : time_in_state_ms) {
        tmp.set<VendorAtomValue::longValue>(time_in_state);
        values.push_back(tmp);
    }
    auto remaining_residency_buckets_count = kMaxStatsResidencyCount - time_in_state_ms.size();
    if (remaining_residency_buckets_count > 0) {
        tmp.set<VendorAtomValue::longValue>(0);
        values.insert(values.end(), remaining_residency_buckets_count, tmp);
    }
    tmp.set<VendorAtomValue::floatValue>(sensor_temp_stats.max_temp);
    values.push_back(tmp);
    tmp.set<VendorAtomValue::longValue>(
            system_clock::to_time_t(sensor_temp_stats.max_temp_timestamp));
    values.push_back(tmp);
    tmp.set<VendorAtomValue::floatValue>(sensor_temp_stats.min_temp);
    values.push_back(tmp);
    tmp.set<VendorAtomValue::longValue>(
            system_clock::to_time_t(sensor_temp_stats.min_temp_timestamp));
    values.push_back(tmp);

    if (!reportAtom(stats_client, PixelAtoms::Atom::kVendorTempResidencyStats, std::move(values))) {
        LOG(ERROR) << "Unable to report VendorTempResidencyStats to Stats service for "
                      "sensor: "
                   << sensor;
        *stats_record = restoreStatsRecordOnFailure(std::move(thermal_stats_before_reporting));
        return false;
    }
    // Update last time of stats reporting
    stats_record->last_stats_report_time = boot_clock::now();
    return true;
}

int ThermalStatsHelper::reportAllSensorCdevRequestStats(
        const std::shared_ptr<IStats> &stats_client) {
    int count_failed_reporting = 0;
    std::unique_lock<std::shared_mutex> _lock(sensor_cdev_request_stats_map_mutex_);
    for (auto &[sensor, cdev_request_stats_map] : sensor_cdev_request_stats_map_) {
        for (auto &[cdev, request_stats] : cdev_request_stats_map) {
            for (size_t threshold_set_idx = 0;
                 threshold_set_idx < request_stats.stats_by_custom_threshold.size();
                 threshold_set_idx++) {
                auto &stats_by_threshold =
                        request_stats.stats_by_custom_threshold[threshold_set_idx];
                std::string cdev_name = stats_by_threshold.logging_name.value_or(
                        cdev + kCustomThresholdSetSuffix.data() +
                        std::to_string(threshold_set_idx));
                if (!reportSensorCdevRequestStats(stats_client, sensor, cdev_name,
                                                  &stats_by_threshold.stats_record)) {
                    count_failed_reporting++;
                }
            }

            if (request_stats.stats_by_default_threshold.has_value()) {
                if (!reportSensorCdevRequestStats(
                            stats_client, sensor, cdev,
                            &request_stats.stats_by_default_threshold.value())) {
                    count_failed_reporting++;
                }
            }
        }
    }
    return count_failed_reporting;
}

bool ThermalStatsHelper::reportSensorCdevRequestStats(const std::shared_ptr<IStats> &stats_client,
                                                      std::string_view sensor,
                                                      std::string_view cdev,
                                                      StatsRecord *stats_record) {
    LOG(VERBOSE) << "Reporting bindedCdev stats for sensor: " << sensor
                 << " cooling_device: " << cdev;
    // maintain a copy in case reporting fails
    StatsRecord thermal_stats_before_reporting = *stats_record;
    std::vector<VendorAtomValue> values(3);
    values[0].set<VendorAtomValue::stringValue>(sensor);
    values[1].set<VendorAtomValue::stringValue>(cdev);
    std::vector<int64_t> time_in_state_ms = processStatsRecordForReporting(stats_record);
    const auto since_last_update_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            stats_record->cur_state_start_time - stats_record->last_stats_report_time);
    values[2].set<VendorAtomValue::longValue>(since_last_update_ms.count());
    VendorAtomValue tmp;
    for (auto &time_in_state : time_in_state_ms) {
        tmp.set<VendorAtomValue::longValue>(time_in_state);
        values.push_back(tmp);
    }

    if (!reportAtom(stats_client, PixelAtoms::Atom::kVendorSensorCoolingDeviceStats,
                    std::move(values))) {
        LOG(ERROR) << "Unable to report VendorSensorCoolingDeviceStats to Stats "
                      "service for sensor: "
                   << sensor << " cooling_device: " << cdev;
        *stats_record = restoreStatsRecordOnFailure(std::move(thermal_stats_before_reporting));
        return false;
    }
    // Update last time of stats reporting
    stats_record->last_stats_report_time = boot_clock::now();
    return true;
}

std::vector<int64_t> ThermalStatsHelper::processStatsRecordForReporting(StatsRecord *stats_record) {
    // update the last unclosed entry and start new record with same state
    updateStatsRecord(stats_record, stats_record->cur_state);
    std::vector<std::chrono::milliseconds> &time_in_state_ms = stats_record->time_in_state_ms;
    // convert std::chrono::milliseconds time_in_state to int64_t vector for reporting
    std::vector<int64_t> stats_residency(time_in_state_ms.size());
    std::transform(time_in_state_ms.begin(), time_in_state_ms.end(), stats_residency.begin(),
                   [](std::chrono::milliseconds time_ms) { return time_ms.count(); });
    // clear previous stats
    std::fill(time_in_state_ms.begin(), time_in_state_ms.end(), std::chrono::milliseconds::zero());
    return stats_residency;
}

bool ThermalStatsHelper::reportThermalAbnormality(
        const ThermalSensorAbnormalityDetected::AbnormalityType &type, std::string_view name,
        std::optional<int> reading) {
    const auto value_str = reading.has_value() ? std::to_string(reading.value()) : "undefined";
    if (abnormal_stats_reported_per_update_interval >= kMaxAbnormalLoggingPerUpdateInterval) {
        LOG(ERROR) << "Thermal abnormal atom logging rate limited for " << name.data()
                   << " with value " << value_str;
        return true;
    }
    const std::shared_ptr<IStats> stats_client = getStatsService();
    if (!stats_client) {
        LOG(ERROR) << "Unable to get AIDL Stats service";
        return false;
    }
    std::vector<VendorAtomValue> values(3);
    values[ThermalSensorAbnormalityDetected::kTypeFieldNumber - kVendorAtomOffset] =
            VendorAtomValue::make<VendorAtomValue::intValue>(type);
    values[ThermalSensorAbnormalityDetected::kSensorFieldNumber - kVendorAtomOffset] =
            VendorAtomValue::make<VendorAtomValue::stringValue>(name);
    if (reading.has_value()) {
        values[ThermalSensorAbnormalityDetected::kTempFieldNumber - kVendorAtomOffset] =
                VendorAtomValue::make<VendorAtomValue::intValue>(reading.value());
    }

    // Dump additional traces before reporting abnormal event
    if (thermal_helper_handle_) {
        thermal_helper_handle_->dumpTraces(name);
    }

    if (!reportAtom(stats_client, PixelAtoms::Atom::kThermalSensorAbnormalityDetected,
                    std::move(values))) {
        LOG(ERROR) << "Failed to log thermal abnormal atom for " << name.data() << " with value "
                   << value_str;
        return false;
    }
    LOG(INFO) << "Thermal abnormality reported for " << name.data() << " with value " << value_str;
    abnormal_stats_reported_per_update_interval++;
    return true;
}

bool ThermalStatsHelper::reportAtom(const std::shared_ptr<IStats> &stats_client,
                                    const int32_t &atom_id, std::vector<VendorAtomValue> &&values) {
    LOG(VERBOSE) << "Reporting thermal stats for atom_id " << atom_id;
    // Send vendor atom to IStats HAL
    VendorAtom event = {.reverseDomainName = "", .atomId = atom_id, .values = std::move(values)};
    const ndk::ScopedAStatus ret = stats_client->reportVendorAtom(event);
    return ret.isOk();
}

StatsRecord ThermalStatsHelper::restoreStatsRecordOnFailure(
        StatsRecord &&stats_record_before_failure) {
    stats_record_before_failure.report_fail_count += 1;
    // If consecutive count of failure is high, reset stat to avoid overflow
    if (stats_record_before_failure.report_fail_count >= kMaxStatsReportingFailCount) {
        return StatsRecord(stats_record_before_failure.time_in_state_ms.size(),
                           stats_record_before_failure.cur_state);
    } else {
        return stats_record_before_failure;
    }
}

std::unordered_map<std::string, SensorTempStats> ThermalStatsHelper::GetSensorTempStatsSnapshot() {
    auto sensor_temp_stats_snapshot = sensor_stats.temp_stats_map_;
    for (auto &sensor_temp_stats_pair : sensor_temp_stats_snapshot) {
        for (auto &temp_stats : sensor_temp_stats_pair.second.stats_by_custom_threshold) {
            // update the last unclosed entry and start new record with same state
            updateStatsRecord(&temp_stats.stats_record, temp_stats.stats_record.cur_state);
        }
        if (sensor_temp_stats_pair.second.stats_by_default_threshold.has_value()) {
            auto &stats_by_default_threshold =
                    sensor_temp_stats_pair.second.stats_by_default_threshold.value();
            // update the last unclosed entry and start new record with same state
            updateStatsRecord(&stats_by_default_threshold, stats_by_default_threshold.cur_state);
        }
    }
    return sensor_temp_stats_snapshot;
}

std::unordered_map<std::string, std::unordered_map<std::string, ThermalStats<int>>>
ThermalStatsHelper::GetSensorCoolingDeviceRequestStatsSnapshot() {
    auto sensor_cdev_request_stats_snapshot = sensor_cdev_request_stats_map_;
    for (auto &sensor_cdev_request_stats_pair : sensor_cdev_request_stats_snapshot) {
        for (auto &cdev_request_stats_pair : sensor_cdev_request_stats_pair.second) {
            for (auto &request_stats : cdev_request_stats_pair.second.stats_by_custom_threshold) {
                // update the last unclosed entry and start new record with same state
                updateStatsRecord(&request_stats.stats_record,
                                  request_stats.stats_record.cur_state);
            }
            if (cdev_request_stats_pair.second.stats_by_default_threshold.has_value()) {
                auto &stats_by_default_threshold =
                        cdev_request_stats_pair.second.stats_by_default_threshold.value();
                // update the last unclosed entry and start new record with same state
                updateStatsRecord(&stats_by_default_threshold,
                                  stats_by_default_threshold.cur_state);
            }
        }
    }
    return sensor_cdev_request_stats_snapshot;
}

}  // namespace implementation
}  // namespace thermal
}  // namespace hardware
}  // namespace android
}  // namespace aidl
