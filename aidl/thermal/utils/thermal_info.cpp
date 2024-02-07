/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "thermal_info.h"

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/strings.h>
#include <json/reader.h>

#include <cmath>
#include <unordered_set>

namespace aidl {
namespace android {
namespace hardware {
namespace thermal {
namespace implementation {

constexpr std::string_view kPowerLinkDisabledProperty("vendor.disable.thermal.powerlink");

namespace {

template <typename T>
// Return false when failed parsing
bool getTypeFromString(std::string_view str, T *out) {
    auto types = ::ndk::enum_range<T>();
    for (const auto &type : types) {
        if (::aidl::android::hardware::thermal::toString(type) == str) {
            *out = type;
            return true;
        }
    }
    return false;
}

float getFloatFromValue(const Json::Value &value) {
    if (value.isString()) {
        return std::stof(value.asString());
    } else {
        return value.asFloat();
    }
}

int getIntFromValue(const Json::Value &value) {
    if (value.isString()) {
        return (value.asString() == "max") ? std::numeric_limits<int>::max()
                                           : std::stoul(value.asString());
    } else {
        return value.asInt();
    }
}

bool getIntFromJsonValues(const Json::Value &values, CdevArray *out, bool inc_check,
                          bool dec_check) {
    CdevArray ret;

    if (inc_check && dec_check) {
        LOG(ERROR) << "Cannot enable inc_check and dec_check at the same time";
        return false;
    }

    if (values.size() != kThrottlingSeverityCount) {
        LOG(ERROR) << "Values size is invalid";
        return false;
    } else {
        int last;
        for (Json::Value::ArrayIndex i = 0; i < kThrottlingSeverityCount; ++i) {
            ret[i] = getIntFromValue(values[i]);
            if (inc_check && ret[i] < last) {
                LOG(FATAL) << "Invalid array[" << i << "]" << ret[i] << " min=" << last;
                return false;
            }
            if (dec_check && ret[i] > last) {
                LOG(FATAL) << "Invalid array[" << i << "]" << ret[i] << " max=" << last;
                return false;
            }
            last = ret[i];
            LOG(INFO) << "[" << i << "]: " << ret[i];
        }
    }

    *out = ret;
    return true;
}

bool getFloatFromJsonValues(const Json::Value &values, ThrottlingArray *out, bool inc_check,
                            bool dec_check) {
    ThrottlingArray ret;

    if (inc_check && dec_check) {
        LOG(ERROR) << "Cannot enable inc_check and dec_check at the same time";
        return false;
    }

    if (values.size() != kThrottlingSeverityCount) {
        LOG(ERROR) << "Values size is invalid";
        return false;
    } else {
        float last = std::nanf("");
        for (Json::Value::ArrayIndex i = 0; i < kThrottlingSeverityCount; ++i) {
            ret[i] = getFloatFromValue(values[i]);
            if (inc_check && !std::isnan(last) && !std::isnan(ret[i]) && ret[i] < last) {
                LOG(FATAL) << "Invalid array[" << i << "]" << ret[i] << " min=" << last;
                return false;
            }
            if (dec_check && !std::isnan(last) && !std::isnan(ret[i]) && ret[i] > last) {
                LOG(FATAL) << "Invalid array[" << i << "]" << ret[i] << " max=" << last;
                return false;
            }
            last = std::isnan(ret[i]) ? last : ret[i];
            LOG(INFO) << "[" << i << "]: " << ret[i];
        }
    }

    *out = ret;
    return true;
}

bool getTempRangeInfoFromJsonValues(const Json::Value &values, TempRangeInfo *temp_range_info) {
    if (values.size() != 2) {
        LOG(ERROR) << "Temp Range Values size: " << values.size() << "is invalid.";
        return false;
    }

    float min_temp = getFloatFromValue(values[0]);
    float max_temp = getFloatFromValue(values[1]);

    if (std::isnan(min_temp) || std::isnan(max_temp)) {
        LOG(ERROR) << "Illegal temp range: thresholds not defined properly " << min_temp << " : "
                   << max_temp;
        return false;
    }

    if (min_temp > max_temp) {
        LOG(ERROR) << "Illegal temp range: temp_min_threshold(" << min_temp
                   << ") > temp_max_threshold(" << max_temp << ")";
        return false;
    }
    temp_range_info->min_temp_threshold = min_temp;
    temp_range_info->max_temp_threshold = max_temp;
    LOG(INFO) << "Temp Range Info: " << temp_range_info->min_temp_threshold
              << " <= t <= " << temp_range_info->max_temp_threshold;
    return true;
}

bool getTempStuckInfoFromJsonValue(const Json::Value &values, TempStuckInfo *temp_stuck_info) {
    if (values["MinStuckDuration"].empty()) {
        LOG(ERROR) << "Minimum stuck duration not present.";
        return false;
    }
    int min_stuck_duration_int = getIntFromValue(values["MinStuckDuration"]);
    if (min_stuck_duration_int <= 0) {
        LOG(ERROR) << "Invalid Minimum stuck duration " << min_stuck_duration_int;
        return false;
    }

    if (values["MinPollingCount"].empty()) {
        LOG(ERROR) << "Minimum polling count not present.";
        return false;
    }
    int min_polling_count = getIntFromValue(values["MinPollingCount"]);
    if (min_polling_count <= 0) {
        LOG(ERROR) << "Invalid Minimum stuck duration " << min_polling_count;
        return false;
    }
    temp_stuck_info->min_stuck_duration = std::chrono::milliseconds(min_stuck_duration_int);
    temp_stuck_info->min_polling_count = min_polling_count;
    LOG(INFO) << "Temp Stuck Info: polling_count=" << temp_stuck_info->min_polling_count
              << " stuck_duration=" << temp_stuck_info->min_stuck_duration.count();
    return true;
}
}  // namespace

std::ostream &operator<<(std::ostream &stream, const SensorFusionType &sensor_fusion_type) {
    switch (sensor_fusion_type) {
        case SensorFusionType::SENSOR:
            return stream << "SENSOR";
        case SensorFusionType::ODPM:
            return stream << "ODPM";
        case SensorFusionType::CONSTANT:
            return stream << "CONSTANT";
        default:
            return stream << "UNDEFINED";
    }
}

bool ParseThermalConfig(std::string_view config_path, Json::Value *config) {
    std::string json_doc;
    if (!::android::base::ReadFileToString(config_path.data(), &json_doc)) {
        LOG(ERROR) << "Failed to read JSON config from " << config_path;
        return false;
    }
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    std::string errorMessage;
    if (!reader->parse(&*json_doc.begin(), &*json_doc.end(), config, &errorMessage)) {
        LOG(ERROR) << "Failed to parse JSON config: " << errorMessage;
        return false;
    }
    return true;
}

bool ParseVirtualSensorInfo(const std::string_view name, const Json::Value &sensor,
                            std::unique_ptr<VirtualSensorInfo> *virtual_sensor_info) {
    if (sensor["VirtualSensor"].empty() || !sensor["VirtualSensor"].isBool()) {
        LOG(INFO) << "Failed to read Sensor[" << name << "]'s VirtualSensor";
        return true;
    }
    bool is_virtual_sensor = sensor["VirtualSensor"].asBool();
    LOG(INFO) << "Sensor[" << name << "]'s' VirtualSensor: " << is_virtual_sensor;
    if (!is_virtual_sensor) {
        return true;
    }
    float offset = 0;
    std::vector<std::string> linked_sensors;
    std::vector<SensorFusionType> linked_sensors_type;
    std::vector<std::string> trigger_sensors;
    std::vector<std::string> coefficients;
    std::vector<SensorFusionType> coefficients_type;
    FormulaOption formula = FormulaOption::COUNT_THRESHOLD;
    std::string vt_estimator_model_file;
    std::unique_ptr<::thermal::vtestimator::VirtualTempEstimator> vt_estimator;

    Json::Value values = sensor["Combination"];
    if (values.size()) {
        linked_sensors.reserve(values.size());
        for (Json::Value::ArrayIndex j = 0; j < values.size(); ++j) {
            linked_sensors.emplace_back(values[j].asString());
            LOG(INFO) << "Sensor[" << name << "]'s Combination[" << j << "]: " << linked_sensors[j];
        }
    } else {
        LOG(ERROR) << "Sensor[" << name << "] has no Combination setting";
        return false;
    }

    if (sensor["Formula"].asString().compare("COUNT_THRESHOLD") == 0) {
        formula = FormulaOption::COUNT_THRESHOLD;
    } else if (sensor["Formula"].asString().compare("WEIGHTED_AVG") == 0) {
        formula = FormulaOption::WEIGHTED_AVG;
    } else if (sensor["Formula"].asString().compare("MAXIMUM") == 0) {
        formula = FormulaOption::MAXIMUM;
    } else if (sensor["Formula"].asString().compare("MINIMUM") == 0) {
        formula = FormulaOption::MINIMUM;
    } else if (sensor["Formula"].asString().compare("USE_ML_MODEL") == 0) {
        formula = FormulaOption::USE_ML_MODEL;
    } else if (sensor["Formula"].asString().compare("USE_LINEAR_MODEL") == 0) {
        formula = FormulaOption::USE_LINEAR_MODEL;
    } else {
        LOG(ERROR) << "Sensor[" << name << "]'s Formula is invalid";
        return false;
    }

    values = sensor["CombinationType"];
    if (!values.size()) {
        linked_sensors_type.reserve(linked_sensors.size());
        for (size_t j = 0; j < linked_sensors.size(); ++j) {
            linked_sensors_type.emplace_back(SensorFusionType::SENSOR);
        }
    } else if (values.size() != linked_sensors.size()) {
        LOG(ERROR) << "Sensor[" << name << "] has invalid CombinationType size";
        return false;
    } else {
        for (Json::Value::ArrayIndex j = 0; j < values.size(); ++j) {
            if (values[j].asString().compare("SENSOR") == 0) {
                linked_sensors_type.emplace_back(SensorFusionType::SENSOR);
            } else if (values[j].asString().compare("ODPM") == 0) {
                linked_sensors_type.emplace_back(SensorFusionType::ODPM);
            } else if (values[j].asString().compare("CONSTANT") == 0) {
                linked_sensors_type.emplace_back(SensorFusionType::CONSTANT);
            } else {
                LOG(ERROR) << "Sensor[" << name << "] has invalid CombinationType settings "
                           << values[j].asString();
                return false;
            }
            LOG(INFO) << "Sensor[" << name << "]'s CombinationType[" << j
                      << "]: " << linked_sensors_type[j];
        }
    }

    values = sensor["Coefficient"];
    if (values.size()) {
        coefficients.reserve(values.size());
        for (Json::Value::ArrayIndex j = 0; j < values.size(); ++j) {
            coefficients.emplace_back(values[j].asString());
            LOG(INFO) << "Sensor[" << name << "]'s coefficient[" << j << "]: " << coefficients[j];
        }
    } else if ((formula != FormulaOption::USE_ML_MODEL)) {
        LOG(ERROR) << "Sensor[" << name << "] has no Coefficient setting";
        return false;
    }
    if ((linked_sensors.size() != coefficients.size()) &&
        (formula != FormulaOption::USE_ML_MODEL) && (formula != FormulaOption::USE_LINEAR_MODEL)) {
        LOG(ERROR) << "Sensor[" << name << "] has invalid Coefficient size";
        return false;
    }

    values = sensor["CoefficientType"];
    if (!values.size()) {
        coefficients_type.reserve(linked_sensors.size());
        for (size_t j = 0; j < linked_sensors.size(); ++j) {
            coefficients_type.emplace_back(SensorFusionType::CONSTANT);
        }
    } else if (values.size() != coefficients.size()) {
        LOG(ERROR) << "Sensor[" << name << "] has invalid coefficient type size";
        return false;
    } else {
        for (Json::Value::ArrayIndex j = 0; j < values.size(); ++j) {
            if (values[j].asString().compare("CONSTANT") == 0) {
                coefficients_type.emplace_back(SensorFusionType::CONSTANT);
            } else if (values[j].asString().compare("SENSOR") == 0) {
                coefficients_type.emplace_back(SensorFusionType::SENSOR);
            } else if (values[j].asString().compare("ODPM") == 0) {
                coefficients_type.emplace_back(SensorFusionType::ODPM);
            } else {
                LOG(ERROR) << "Sensor[" << name << "] has invalid coefficient options "
                           << values[j].asString();
                return false;
            }
            LOG(INFO) << "Sensor[" << name << "]'s coefficient type[" << j
                      << "]: " << coefficients_type[j];
        }
    }

    if (linked_sensors.size() != coefficients_type.size()) {
        LOG(ERROR) << "Sensor[" << name
                   << "]'s combination size is not matched with coefficient type size";
        return false;
    }

    if (!sensor["Offset"].empty()) {
        offset = sensor["Offset"].asFloat();
    }

    values = sensor["TriggerSensor"];
    if (!values.empty()) {
        if (values.isString()) {
            trigger_sensors.emplace_back(values.asString());
            LOG(INFO) << "Sensor[" << name << "]'s TriggerSensor: " << values.asString();
        } else if (values.size()) {
            trigger_sensors.reserve(values.size());
            for (Json::Value::ArrayIndex j = 0; j < values.size(); ++j) {
                if (!values[j].isString()) {
                    LOG(ERROR) << name << " TriggerSensor should be an array of string";
                    return false;
                }
                trigger_sensors.emplace_back(values[j].asString());
                LOG(INFO) << "Sensor[" << name << "]'s TriggerSensor[" << j
                          << "]: " << trigger_sensors[j];
            }
        } else {
            LOG(ERROR) << "Sensor[" << name << "]'s TriggerSensor should be a string";
            return false;
        }
    }

    if (formula == FormulaOption::USE_ML_MODEL) {
        ::thermal::vtestimator::VtEstimationInitData init_data;
        if (sensor["ModelPath"].empty()) {
            LOG(ERROR) << "Sensor[" << name << "] has no ModelPath";
            return false;
        }

        if (!linked_sensors.size()) {
            LOG(ERROR) << "Sensor[" << name << "] uses USE_ML_MODEL and has zero linked_sensors";
            return false;
        }

        vt_estimator = std::make_unique<::thermal::vtestimator::VirtualTempEstimator>(
                ::thermal::vtestimator::kUseMLModel, linked_sensors.size());
        if (!vt_estimator) {
            LOG(ERROR) << "Failed to create vt estimator for Sensor[" << name
                       << "] with linked sensor size : " << linked_sensors.size();
            return false;
        }

        vt_estimator_model_file = "vendor/etc/" + sensor["ModelPath"].asString();
        init_data.ml_model_init_data.model_path = vt_estimator_model_file;
        init_data.ml_model_init_data.offset = offset;

        if (!sensor["PreviousSampleCount"].empty()) {
            init_data.ml_model_init_data.use_prev_samples = true;
            init_data.ml_model_init_data.prev_samples_order = sensor["PreviousSampleCount"].asInt();
            LOG(INFO) << "Sensor[" << name << "] takes "
                      << init_data.ml_model_init_data.prev_samples_order << " historic samples";
        } else {
            init_data.ml_model_init_data.use_prev_samples = false;
            init_data.ml_model_init_data.prev_samples_order = 1;
        }

        if (!sensor["OutputLabelCount"].empty()) {
            init_data.ml_model_init_data.output_label_count = sensor["OutputLabelCount"].asInt();
            LOG(INFO) << "Sensor[" << name << "] outputs "
                      << init_data.ml_model_init_data.output_label_count << " labels";
        } else {
            init_data.ml_model_init_data.output_label_count = 1;
        }

        if (!sensor["PredictHotSpotCount"].empty()) {
            init_data.ml_model_init_data.num_hot_spots = sensor["PredictHotSpotCount"].asInt();
            LOG(INFO) << "Sensor[" << name << "] predicts temperature at "
                      << init_data.ml_model_init_data.num_hot_spots << " hot spots";
        } else {
            init_data.ml_model_init_data.num_hot_spots = 1;
        }

        ::thermal::vtestimator::VtEstimatorStatus ret = vt_estimator->Initialize(init_data);
        if (ret != ::thermal::vtestimator::kVtEstimatorOk) {
            LOG(ERROR) << "Failed to initialize vt estimator for Sensor[" << name
                       << "] with ModelPath: " << vt_estimator_model_file
                       << " with ret code : " << ret;
            return false;
        }

        LOG(INFO) << "Successfully created vt_estimator for Sensor[" << name
                  << "] with input samples: " << linked_sensors.size();

    } else if (formula == FormulaOption::USE_LINEAR_MODEL) {
        ::thermal::vtestimator::VtEstimationInitData init_data;

        if ((!linked_sensors.size()) || (linked_sensors.size() > coefficients.size())) {
            LOG(ERROR) << "Sensor[" << name
                       << "] uses USE_LINEAR_MODEL and has invalid linked_sensors size["
                       << linked_sensors.size() << "] or coefficients size[" << coefficients.size()
                       << "]";
            return false;
        }

        vt_estimator = std::make_unique<::thermal::vtestimator::VirtualTempEstimator>(
                ::thermal::vtestimator::kUseLinearModel, linked_sensors.size());
        if (!vt_estimator) {
            LOG(ERROR) << "Failed to create vt estimator for Sensor[" << name
                       << "] with linked sensor size : " << linked_sensors.size();
            return false;
        }

        init_data.linear_model_init_data.prev_samples_order =
                coefficients.size() / linked_sensors.size();
        init_data.linear_model_init_data.offset = offset;

        for (size_t i = 0; i < coefficients.size(); ++i) {
            float coefficient = getFloatFromValue(coefficients[i]);
            if (std::isnan(coefficient)) {
                LOG(ERROR) << "Nan coefficient unexpected for sensor " << name;
                return false;
            }
            init_data.linear_model_init_data.coefficients.emplace_back(coefficient);
        }
        if (coefficients.size() > linked_sensors.size()) {
            init_data.linear_model_init_data.use_prev_samples = true;
        }

        ::thermal::vtestimator::VtEstimatorStatus ret = vt_estimator->Initialize(init_data);
        if (ret != ::thermal::vtestimator::kVtEstimatorOk) {
            LOG(ERROR) << "Failed to initialize vt estimator for Sensor[" << name
                       << "] with ret code : " << ret;
            return false;
        }

        LOG(INFO) << "Successfully created vt_estimator for Sensor[" << name
                  << "] with input samples: " << linked_sensors.size();
    }

    virtual_sensor_info->reset(new VirtualSensorInfo{
            linked_sensors, linked_sensors_type, coefficients, coefficients_type, offset,
            trigger_sensors, formula, vt_estimator_model_file, std::move(vt_estimator)});
    return true;
}

bool ParseBindedCdevInfo(const Json::Value &values,
                         std::unordered_map<std::string, BindedCdevInfo> *binded_cdev_info_map,
                         const bool support_pid, bool *support_hard_limit) {
    for (Json::Value::ArrayIndex j = 0; j < values.size(); ++j) {
        Json::Value sub_values;
        const std::string &cdev_name = values[j]["CdevRequest"].asString();
        ThrottlingArray cdev_weight_for_pid;
        cdev_weight_for_pid.fill(NAN);
        CdevArray cdev_ceiling;
        cdev_ceiling.fill(std::numeric_limits<int>::max());
        int max_release_step = std::numeric_limits<int>::max();
        int max_throttle_step = std::numeric_limits<int>::max();
        if (support_pid) {
            if (!values[j]["CdevWeightForPID"].empty()) {
                LOG(INFO) << "Star to parse " << cdev_name << "'s CdevWeightForPID";
                if (!getFloatFromJsonValues(values[j]["CdevWeightForPID"], &cdev_weight_for_pid,
                                            false, false)) {
                    LOG(ERROR) << "Failed to parse CdevWeightForPID";
                    binded_cdev_info_map->clear();
                    return false;
                }
            }
            if (!values[j]["CdevCeiling"].empty()) {
                LOG(INFO) << "Start to parse CdevCeiling: " << cdev_name;
                if (!getIntFromJsonValues(values[j]["CdevCeiling"], &cdev_ceiling, false, false)) {
                    LOG(ERROR) << "Failed to parse CdevCeiling";
                    binded_cdev_info_map->clear();
                    return false;
                }
            }

            if (!values[j]["MaxReleaseStep"].empty()) {
                max_release_step = getIntFromValue(values[j]["MaxReleaseStep"]);
                if (max_release_step < 0) {
                    LOG(ERROR) << cdev_name << " MaxReleaseStep: " << max_release_step;
                    binded_cdev_info_map->clear();
                    return false;
                } else {
                    LOG(INFO) << cdev_name << " MaxReleaseStep: " << max_release_step;
                }
            }
            if (!values[j]["MaxThrottleStep"].empty()) {
                max_throttle_step = getIntFromValue(values[j]["MaxThrottleStep"]);
                if (max_throttle_step < 0) {
                    LOG(ERROR) << cdev_name << " MaxThrottleStep: " << max_throttle_step;
                    binded_cdev_info_map->clear();
                    return false;
                } else {
                    LOG(INFO) << cdev_name << " MaxThrottleStep: " << max_throttle_step;
                }
            }
        }
        CdevArray limit_info;
        limit_info.fill(0);
        ThrottlingArray power_thresholds;
        power_thresholds.fill(NAN);
        ReleaseLogic release_logic = ReleaseLogic::NONE;

        sub_values = values[j]["LimitInfo"];
        if (sub_values.size()) {
            LOG(INFO) << "Start to parse LimitInfo: " << cdev_name;
            if (!getIntFromJsonValues(sub_values, &limit_info, false, false)) {
                LOG(ERROR) << "Failed to parse LimitInfo";
                binded_cdev_info_map->clear();
                return false;
            }
            *support_hard_limit = true;
        }
        // Parse linked power info
        std::string power_rail;
        bool high_power_check = false;
        bool throttling_with_power_link = false;
        bool enabled = true;
        CdevArray cdev_floor_with_power_link;
        cdev_floor_with_power_link.fill(0);

        const bool power_link_disabled =
                ::android::base::GetBoolProperty(kPowerLinkDisabledProperty.data(), false);
        if (!power_link_disabled) {
            power_rail = values[j]["BindedPowerRail"].asString();

            if (values[j]["HighPowerCheck"].asBool()) {
                high_power_check = true;
            }
            LOG(INFO) << "Highpowercheck: " << std::boolalpha << high_power_check;

            if (values[j]["ThrottlingWithPowerLink"].asBool()) {
                throttling_with_power_link = true;
            }
            LOG(INFO) << "ThrottlingwithPowerLink: " << std::boolalpha
                      << throttling_with_power_link;

            sub_values = values[j]["CdevFloorWithPowerLink"];
            if (sub_values.size()) {
                LOG(INFO) << "Start to parse " << cdev_name << "'s CdevFloorWithPowerLink";
                if (!getIntFromJsonValues(sub_values, &cdev_floor_with_power_link, false, false)) {
                    LOG(ERROR) << "Failed to parse CdevFloor";
                    binded_cdev_info_map->clear();
                    return false;
                }
            }
            sub_values = values[j]["PowerThreshold"];
            if (sub_values.size()) {
                LOG(INFO) << "Start to parse " << cdev_name << "'s PowerThreshold";
                if (!getFloatFromJsonValues(sub_values, &power_thresholds, false, false)) {
                    LOG(ERROR) << "Failed to parse power thresholds";
                    binded_cdev_info_map->clear();
                    return false;
                }
                if (values[j]["ReleaseLogic"].asString() == "INCREASE") {
                    release_logic = ReleaseLogic::INCREASE;
                    LOG(INFO) << "Release logic: INCREASE";
                } else if (values[j]["ReleaseLogic"].asString() == "DECREASE") {
                    release_logic = ReleaseLogic::DECREASE;
                    LOG(INFO) << "Release logic: DECREASE";
                } else if (values[j]["ReleaseLogic"].asString() == "STEPWISE") {
                    release_logic = ReleaseLogic::STEPWISE;
                    LOG(INFO) << "Release logic: STEPWISE";
                } else if (values[j]["ReleaseLogic"].asString() == "RELEASE_TO_FLOOR") {
                    release_logic = ReleaseLogic::RELEASE_TO_FLOOR;
                    LOG(INFO) << "Release logic: RELEASE_TO_FLOOR";
                } else {
                    LOG(ERROR) << "Release logic is invalid";
                    binded_cdev_info_map->clear();
                    return false;
                }
            }
        }
        if (values[j]["Disabled"].asBool()) {
            enabled = false;
        }

        (*binded_cdev_info_map)[cdev_name] = {
                .limit_info = limit_info,
                .power_thresholds = power_thresholds,
                .release_logic = release_logic,
                .cdev_weight_for_pid = cdev_weight_for_pid,
                .cdev_ceiling = cdev_ceiling,
                .max_release_step = max_release_step,
                .max_throttle_step = max_throttle_step,
                .cdev_floor_with_power_link = cdev_floor_with_power_link,
                .power_rail = power_rail,
                .high_power_check = high_power_check,
                .throttling_with_power_link = throttling_with_power_link,
                .enabled = enabled,
        };
    }
    return true;
}

bool ParseSensorThrottlingInfo(const std::string_view name, const Json::Value &sensor,
                               bool *support_throttling,
                               std::shared_ptr<ThrottlingInfo> *throttling_info) {
    std::array<float, kThrottlingSeverityCount> k_po;
    k_po.fill(0.0);
    std::array<float, kThrottlingSeverityCount> k_pu;
    k_pu.fill(0.0);
    std::array<float, kThrottlingSeverityCount> k_i;
    k_i.fill(0.0);
    std::array<float, kThrottlingSeverityCount> k_d;
    k_d.fill(0.0);
    std::array<float, kThrottlingSeverityCount> i_max;
    i_max.fill(NAN);
    std::array<float, kThrottlingSeverityCount> max_alloc_power;
    max_alloc_power.fill(NAN);
    std::array<float, kThrottlingSeverityCount> min_alloc_power;
    min_alloc_power.fill(NAN);
    std::array<float, kThrottlingSeverityCount> s_power;
    s_power.fill(NAN);
    std::array<float, kThrottlingSeverityCount> i_cutoff;
    i_cutoff.fill(NAN);
    float i_default = 0.0;
    int tran_cycle = 0;
    bool support_pid = false;
    bool support_hard_limit = false;

    // Parse PID parameters
    if (!sensor["PIDInfo"].empty()) {
        LOG(INFO) << "Start to parse"
                  << " Sensor[" << name << "]'s K_Po";
        if (sensor["PIDInfo"]["K_Po"].empty() ||
            !getFloatFromJsonValues(sensor["PIDInfo"]["K_Po"], &k_po, false, false)) {
            LOG(ERROR) << "Sensor[" << name << "]: Failed to parse K_Po";
            return false;
        }
        LOG(INFO) << "Start to parse"
                  << " Sensor[" << name << "]'s  K_Pu";
        if (sensor["PIDInfo"]["K_Pu"].empty() ||
            !getFloatFromJsonValues(sensor["PIDInfo"]["K_Pu"], &k_pu, false, false)) {
            LOG(ERROR) << "Sensor[" << name << "]: Failed to parse K_Pu";
            return false;
        }
        LOG(INFO) << "Start to parse"
                  << " Sensor[" << name << "]'s K_I";
        if (sensor["PIDInfo"]["K_I"].empty() ||
            !getFloatFromJsonValues(sensor["PIDInfo"]["K_I"], &k_i, false, false)) {
            LOG(ERROR) << "Sensor[" << name << "]: Failed to parse K_I";
            return false;
        }
        LOG(INFO) << "Start to parse"
                  << " Sensor[" << name << "]'s K_D";
        if (sensor["PIDInfo"]["K_D"].empty() ||
            !getFloatFromJsonValues(sensor["PIDInfo"]["K_D"], &k_d, false, false)) {
            LOG(ERROR) << "Sensor[" << name << "]: Failed to parse K_D";
            return false;
        }
        LOG(INFO) << "Start to parse"
                  << " Sensor[" << name << "]'s I_Max";
        if (sensor["PIDInfo"]["I_Max"].empty() ||
            !getFloatFromJsonValues(sensor["PIDInfo"]["I_Max"], &i_max, false, false)) {
            LOG(ERROR) << "Sensor[" << name << "]: Failed to parse I_Max";
            return false;
        }
        LOG(INFO) << "Start to parse"
                  << " Sensor[" << name << "]'s MaxAllocPower";
        if (sensor["PIDInfo"]["MaxAllocPower"].empty() ||
            !getFloatFromJsonValues(sensor["PIDInfo"]["MaxAllocPower"], &max_alloc_power, false,
                                    true)) {
            LOG(ERROR) << "Sensor[" << name << "]: Failed to parse MaxAllocPower";
            return false;
        }
        LOG(INFO) << "Start to parse"
                  << " Sensor[" << name << "]'s MinAllocPower";
        if (sensor["PIDInfo"]["MinAllocPower"].empty() ||
            !getFloatFromJsonValues(sensor["PIDInfo"]["MinAllocPower"], &min_alloc_power, false,
                                    true)) {
            LOG(ERROR) << "Sensor[" << name << "]: Failed to parse MinAllocPower";
            return false;
        }
        LOG(INFO) << "Start to parse Sensor[" << name << "]'s S_Power";
        if (sensor["PIDInfo"]["S_Power"].empty() ||
            !getFloatFromJsonValues(sensor["PIDInfo"]["S_Power"], &s_power, false, true)) {
            LOG(ERROR) << "Sensor[" << name << "]: Failed to parse S_Power";
            return false;
        }
        LOG(INFO) << "Start to parse Sensor[" << name << "]'s I_Cutoff";
        if (sensor["PIDInfo"]["I_Cutoff"].empty() ||
            !getFloatFromJsonValues(sensor["PIDInfo"]["I_Cutoff"], &i_cutoff, false, false)) {
            LOG(ERROR) << "Sensor[" << name << "]: Failed to parse I_Cutoff";
            return false;
        }
        i_default = getFloatFromValue(sensor["PIDInfo"]["I_Default"]);
        LOG(INFO) << "Sensor[" << name << "]'s I_Default: " << i_default;

        tran_cycle = getFloatFromValue(sensor["PIDInfo"]["TranCycle"]);
        LOG(INFO) << "Sensor[" << name << "]'s TranCycle: " << tran_cycle;

        // Confirm we have at least one valid PID combination
        bool valid_pid_combination = false;
        for (Json::Value::ArrayIndex j = 0; j < kThrottlingSeverityCount; ++j) {
            if (!std::isnan(s_power[j])) {
                if (std::isnan(k_po[j]) || std::isnan(k_pu[j]) || std::isnan(k_i[j]) ||
                    std::isnan(k_d[j]) || std::isnan(i_max[j]) || std::isnan(max_alloc_power[j]) ||
                    std::isnan(min_alloc_power[j]) || std::isnan(i_cutoff[j])) {
                    valid_pid_combination = false;
                    break;
                } else {
                    valid_pid_combination = true;
                }
            }
        }
        if (!valid_pid_combination) {
            LOG(ERROR) << "Sensor[" << name << "]: Invalid PID parameters combinations";
            return false;
        } else {
            support_pid = true;
        }
    }

    // Parse binded cooling device
    std::unordered_map<std::string, BindedCdevInfo> binded_cdev_info_map;
    if (!ParseBindedCdevInfo(sensor["BindedCdevInfo"], &binded_cdev_info_map, support_pid,
                             &support_hard_limit)) {
        LOG(ERROR) << "Sensor[" << name << "]: failed to parse BindedCdevInfo";
        return false;
    }
    Json::Value values;
    ProfileMap profile_map;

    values = sensor["Profile"];
    for (Json::Value::ArrayIndex j = 0; j < values.size(); ++j) {
        Json::Value sub_values;
        const std::string &mode = values[j]["Mode"].asString();
        std::unordered_map<std::string, BindedCdevInfo> binded_cdev_info_map_profile;
        if (!ParseBindedCdevInfo(values[j]["BindedCdevInfo"], &binded_cdev_info_map_profile,
                                 support_pid, &support_hard_limit)) {
            LOG(ERROR) << "Sensor[" << name << " failed to parse BindedCdevInfo profile";
        }
        // Check if the binded_cdev_info_map_profile is valid
        if (binded_cdev_info_map.size() != binded_cdev_info_map_profile.size()) {
            LOG(ERROR) << "Sensor[" << name << "]:'s profile map size should not be changed";
            return false;
        } else {
            for (const auto &binded_cdev_info_pair : binded_cdev_info_map_profile) {
                if (binded_cdev_info_map.count(binded_cdev_info_pair.first)) {
                    if (binded_cdev_info_pair.second.power_rail !=
                        binded_cdev_info_map.at(binded_cdev_info_pair.first).power_rail) {
                        LOG(ERROR) << "Sensor[" << name << "]:'s profile " << mode << " binded "
                                   << binded_cdev_info_pair.first
                                   << "'s power rail is not included in default rules";
                        return false;
                    } else {
                        LOG(INFO) << "Sensor[" << name << "]:'s profile " << mode
                                  << " is parsed successfully";
                    }
                } else {
                    LOG(ERROR) << "Sensor[" << name << "]'s profile " << mode << " binded "
                               << binded_cdev_info_pair.first
                               << " is not included in default rules";
                    return false;
                }
            }
        }
        profile_map[mode] = binded_cdev_info_map_profile;
    }

    std::unordered_map<std::string, ThrottlingArray> excluded_power_info_map;
    values = sensor["ExcludedPowerInfo"];
    for (Json::Value::ArrayIndex j = 0; j < values.size(); ++j) {
        Json::Value sub_values;
        const std::string &power_rail = values[j]["PowerRail"].asString();
        if (power_rail.empty()) {
            LOG(ERROR) << "Sensor[" << name << "] failed to parse excluded PowerRail";
            return false;
        }
        ThrottlingArray power_weight;
        power_weight.fill(1);
        if (!values[j]["PowerWeight"].empty()) {
            LOG(INFO) << "Sensor[" << name << "]: Start to parse " << power_rail
                      << "'s PowerWeight";
            if (!getFloatFromJsonValues(values[j]["PowerWeight"], &power_weight, false, false)) {
                LOG(ERROR) << "Failed to parse PowerWeight";
                return false;
            }
        }
        excluded_power_info_map[power_rail] = power_weight;
    }
    throttling_info->reset(new ThrottlingInfo{
            k_po, k_pu, k_i, k_d, i_max, max_alloc_power, min_alloc_power, s_power, i_cutoff,
            i_default, tran_cycle, excluded_power_info_map, binded_cdev_info_map, profile_map});
    *support_throttling = support_pid | support_hard_limit;
    return true;
}

bool ParseSensorInfo(const Json::Value &config,
                     std::unordered_map<std::string, SensorInfo> *sensors_parsed) {
    Json::Value sensors = config["Sensors"];
    std::size_t total_parsed = 0;
    std::unordered_set<std::string> sensors_name_parsed;

    for (Json::Value::ArrayIndex i = 0; i < sensors.size(); ++i) {
        const std::string &name = sensors[i]["Name"].asString();
        LOG(INFO) << "Sensor[" << i << "]'s Name: " << name;
        if (name.empty()) {
            LOG(ERROR) << "Failed to read Sensor[" << i << "]'s Name";
            sensors_parsed->clear();
            return false;
        }

        auto result = sensors_name_parsed.insert(name);
        if (!result.second) {
            LOG(ERROR) << "Duplicate Sensor[" << i << "]'s Name";
            sensors_parsed->clear();
            return false;
        }

        std::string sensor_type_str = sensors[i]["Type"].asString();
        LOG(INFO) << "Sensor[" << name << "]'s Type: " << sensor_type_str;
        TemperatureType sensor_type;

        if (!getTypeFromString(sensor_type_str, &sensor_type)) {
            LOG(ERROR) << "Invalid Sensor[" << name << "]'s Type: " << sensor_type_str;
            sensors_parsed->clear();
            return false;
        }

        bool send_cb = false;
        if (!sensors[i]["Monitor"].empty() && sensors[i]["Monitor"].isBool()) {
            send_cb = sensors[i]["Monitor"].asBool();
        } else if (!sensors[i]["SendCallback"].empty() && sensors[i]["SendCallback"].isBool()) {
            send_cb = sensors[i]["SendCallback"].asBool();
        }
        LOG(INFO) << "Sensor[" << name << "]'s SendCallback: " << std::boolalpha << send_cb
                  << std::noboolalpha;

        bool send_powerhint = false;
        if (sensors[i]["SendPowerHint"].empty() || !sensors[i]["SendPowerHint"].isBool()) {
            LOG(INFO) << "Failed to read Sensor[" << name << "]'s SendPowerHint, set to 'false'";
        } else if (sensors[i]["SendPowerHint"].asBool()) {
            send_powerhint = true;
        }
        LOG(INFO) << "Sensor[" << name << "]'s SendPowerHint: " << std::boolalpha << send_powerhint
                  << std::noboolalpha;

        bool is_hidden = false;
        if (sensors[i]["Hidden"].empty() || !sensors[i]["Hidden"].isBool()) {
            LOG(INFO) << "Failed to read Sensor[" << name << "]'s Hidden, set to 'false'";
        } else if (sensors[i]["Hidden"].asBool()) {
            is_hidden = true;
        }
        LOG(INFO) << "Sensor[" << name << "]'s Hidden: " << std::boolalpha << is_hidden
                  << std::noboolalpha;

        std::array<float, kThrottlingSeverityCount> hot_thresholds;
        hot_thresholds.fill(NAN);
        std::array<float, kThrottlingSeverityCount> cold_thresholds;
        cold_thresholds.fill(NAN);
        std::array<float, kThrottlingSeverityCount> hot_hysteresis;
        hot_hysteresis.fill(0.0);
        std::array<float, kThrottlingSeverityCount> cold_hysteresis;
        cold_hysteresis.fill(0.0);

        Json::Value values = sensors[i]["HotThreshold"];
        if (!values.size()) {
            LOG(INFO) << "Sensor[" << name << "]'s HotThreshold, default all to NAN";
        } else if (values.size() != kThrottlingSeverityCount) {
            LOG(ERROR) << "Invalid Sensor[" << name << "]'s HotThreshold count:" << values.size();
            sensors_parsed->clear();
            return false;
        } else {
            float min = std::numeric_limits<float>::min();
            for (Json::Value::ArrayIndex j = 0; j < kThrottlingSeverityCount; ++j) {
                hot_thresholds[j] = getFloatFromValue(values[j]);
                if (!std::isnan(hot_thresholds[j])) {
                    if (hot_thresholds[j] < min) {
                        LOG(ERROR) << "Invalid "
                                   << "Sensor[" << name << "]'s HotThreshold[j" << j
                                   << "]: " << hot_thresholds[j] << " < " << min;
                        sensors_parsed->clear();
                        return false;
                    }
                    min = hot_thresholds[j];
                }
                LOG(INFO) << "Sensor[" << name << "]'s HotThreshold[" << j
                          << "]: " << hot_thresholds[j];
            }
        }

        values = sensors[i]["HotHysteresis"];
        if (!values.size()) {
            LOG(INFO) << "Sensor[" << name << "]'s HotHysteresis, default all to 0.0";
        } else if (values.size() != kThrottlingSeverityCount) {
            LOG(ERROR) << "Invalid Sensor[" << name << "]'s HotHysteresis, count:" << values.size();
            sensors_parsed->clear();
            return false;
        } else {
            for (Json::Value::ArrayIndex j = 0; j < kThrottlingSeverityCount; ++j) {
                hot_hysteresis[j] = getFloatFromValue(values[j]);
                if (std::isnan(hot_hysteresis[j])) {
                    LOG(ERROR) << "Invalid Sensor[" << name
                               << "]'s HotHysteresis: " << hot_hysteresis[j];
                    sensors_parsed->clear();
                    return false;
                }
                LOG(INFO) << "Sensor[" << name << "]'s HotHysteresis[" << j
                          << "]: " << hot_hysteresis[j];
            }
        }

        for (Json::Value::ArrayIndex j = 0; j < (kThrottlingSeverityCount - 1); ++j) {
            if (std::isnan(hot_thresholds[j])) {
                continue;
            }
            for (auto k = j + 1; k < kThrottlingSeverityCount; ++k) {
                if (std::isnan(hot_thresholds[k])) {
                    continue;
                } else if (hot_thresholds[j] > (hot_thresholds[k] - hot_hysteresis[k])) {
                    LOG(ERROR) << "Sensor[" << name << "]'s hot threshold " << j
                               << " is overlapped";
                    sensors_parsed->clear();
                    return false;
                } else {
                    break;
                }
            }
        }

        values = sensors[i]["ColdThreshold"];
        if (!values.size()) {
            LOG(INFO) << "Sensor[" << name << "]'s ColdThreshold, default all to NAN";
        } else if (values.size() != kThrottlingSeverityCount) {
            LOG(ERROR) << "Invalid Sensor[" << name << "]'s ColdThreshold count:" << values.size();
            sensors_parsed->clear();
            return false;
        } else {
            float max = std::numeric_limits<float>::max();
            for (Json::Value::ArrayIndex j = 0; j < kThrottlingSeverityCount; ++j) {
                cold_thresholds[j] = getFloatFromValue(values[j]);
                if (!std::isnan(cold_thresholds[j])) {
                    if (cold_thresholds[j] > max) {
                        LOG(ERROR) << "Invalid "
                                   << "Sensor[" << name << "]'s ColdThreshold[j" << j
                                   << "]: " << cold_thresholds[j] << " > " << max;
                        sensors_parsed->clear();
                        return false;
                    }
                    max = cold_thresholds[j];
                }
                LOG(INFO) << "Sensor[" << name << "]'s ColdThreshold[" << j
                          << "]: " << cold_thresholds[j];
            }
        }

        values = sensors[i]["ColdHysteresis"];
        if (!values.size()) {
            LOG(INFO) << "Sensor[" << name << "]'s ColdHysteresis, default all to 0.0";
        } else if (values.size() != kThrottlingSeverityCount) {
            LOG(ERROR) << "Invalid Sensor[" << name << "]'s ColdHysteresis count:" << values.size();
            sensors_parsed->clear();
            return false;
        } else {
            for (Json::Value::ArrayIndex j = 0; j < kThrottlingSeverityCount; ++j) {
                cold_hysteresis[j] = getFloatFromValue(values[j]);
                if (std::isnan(cold_hysteresis[j])) {
                    LOG(ERROR) << "Invalid Sensor[" << name
                               << "]'s ColdHysteresis: " << cold_hysteresis[j];
                    sensors_parsed->clear();
                    return false;
                }
                LOG(INFO) << "Sensor[" << name << "]'s ColdHysteresis[" << j
                          << "]: " << cold_hysteresis[j];
            }
        }

        for (Json::Value::ArrayIndex j = 0; j < (kThrottlingSeverityCount - 1); ++j) {
            if (std::isnan(cold_thresholds[j])) {
                continue;
            }
            for (auto k = j + 1; k < kThrottlingSeverityCount; ++k) {
                if (std::isnan(cold_thresholds[k])) {
                    continue;
                } else if (cold_thresholds[j] < (cold_thresholds[k] + cold_hysteresis[k])) {
                    LOG(ERROR) << "Sensor[" << name << "]'s cold threshold " << j
                               << " is overlapped";
                    sensors_parsed->clear();
                    return false;
                } else {
                    break;
                }
            }
        }

        std::string zone_name;
        if (!sensors[i]["ZoneName"].empty()) {
            zone_name = sensors[i]["ZoneName"].asString();
        } else {
            zone_name = name;
        }

        LOG(INFO) << "Sensor[" << name << "]'s ZoneName: " << zone_name;

        std::string temp_path;
        if (!sensors[i]["TempPath"].empty()) {
            temp_path = sensors[i]["TempPath"].asString();
            LOG(INFO) << "Sensor[" << name << "]'s TempPath: " << temp_path;
        }

        float vr_threshold = NAN;
        if (!sensors[i]["VrThreshold"].empty()) {
            vr_threshold = getFloatFromValue(sensors[i]["VrThreshold"]);
            LOG(INFO) << "Sensor[" << name << "]'s VrThreshold: " << vr_threshold;
        }

        float multiplier = 1.0;
        if (!sensors[i]["Multiplier"].empty()) {
            multiplier = sensors[i]["Multiplier"].asFloat();
        }
        LOG(INFO) << "Sensor[" << name << "]'s Multiplier: " << multiplier;

        std::chrono::milliseconds polling_delay = kUeventPollTimeoutMs;
        if (!sensors[i]["PollingDelay"].empty()) {
            const auto value = getIntFromValue(sensors[i]["PollingDelay"]);
            polling_delay = (value > 0) ? std::chrono::milliseconds(value)
                                        : std::chrono::milliseconds::max();
        }
        LOG(INFO) << "Sensor[" << name << "]'s Polling delay: " << polling_delay.count();

        std::chrono::milliseconds passive_delay = kMinPollIntervalMs;
        if (!sensors[i]["PassiveDelay"].empty()) {
            const auto value = getIntFromValue(sensors[i]["PassiveDelay"]);
            passive_delay = (value > 0) ? std::chrono::milliseconds(value)
                                        : std::chrono::milliseconds::max();
        }
        LOG(INFO) << "Sensor[" << name << "]'s Passive delay: " << passive_delay.count();

        std::chrono::milliseconds time_resolution;
        if (sensors[i]["TimeResolution"].empty()) {
            time_resolution = kMinPollIntervalMs;
        } else {
            time_resolution =
                    std::chrono::milliseconds(getIntFromValue(sensors[i]["TimeResolution"]));
        }
        LOG(INFO) << "Sensor[" << name << "]'s Time resolution: " << time_resolution.count();

        float step_ratio = NAN;
        if (!sensors[i]["StepRatio"].empty()) {
            step_ratio = sensors[i]["StepRatio"].asFloat();
            if (step_ratio < 0 || step_ratio > 1) {
                LOG(ERROR) << "Sensor[" << name << "]'s StepRatio should be set 0 ~ 1";
                sensors_parsed->clear();
                return false;
            }
        }

        if (is_hidden && send_cb) {
            LOG(ERROR) << "is_hidden and send_cb cannot be enabled together";
            sensors_parsed->clear();
            return false;
        }

        std::unique_ptr<VirtualSensorInfo> virtual_sensor_info;
        if (!ParseVirtualSensorInfo(name, sensors[i], &virtual_sensor_info)) {
            LOG(ERROR) << "Sensor[" << name << "]: failed to parse virtual sensor info";
            sensors_parsed->clear();
            return false;
        }

        bool support_throttling = false;  // support pid or hard limit
        std::shared_ptr<ThrottlingInfo> throttling_info;
        if (!ParseSensorThrottlingInfo(name, sensors[i], &support_throttling, &throttling_info)) {
            LOG(ERROR) << "Sensor[" << name << "]: failed to parse throttling info";
            sensors_parsed->clear();
            return false;
        }

        bool is_watch = (send_cb | send_powerhint | support_throttling);
        LOG(INFO) << "Sensor[" << name << "]'s is_watch: " << std::boolalpha << is_watch;

        (*sensors_parsed)[name] = {
                .type = sensor_type,
                .hot_thresholds = hot_thresholds,
                .cold_thresholds = cold_thresholds,
                .hot_hysteresis = hot_hysteresis,
                .cold_hysteresis = cold_hysteresis,
                .temp_path = temp_path,
                .zone_name = zone_name,
                .vr_threshold = vr_threshold,
                .multiplier = multiplier,
                .polling_delay = polling_delay,
                .passive_delay = passive_delay,
                .time_resolution = time_resolution,
                .step_ratio = step_ratio,
                .send_cb = send_cb,
                .send_powerhint = send_powerhint,
                .is_watch = is_watch,
                .is_hidden = is_hidden,
                .virtual_sensor_info = std::move(virtual_sensor_info),
                .throttling_info = std::move(throttling_info),
        };

        ++total_parsed;
    }
    LOG(INFO) << total_parsed << " Sensors parsed successfully";
    return true;
}

bool ParseCoolingDevice(const Json::Value &config,
                        std::unordered_map<std::string, CdevInfo> *cooling_devices_parsed) {
    Json::Value cooling_devices = config["CoolingDevices"];
    std::size_t total_parsed = 0;
    std::unordered_set<std::string> cooling_devices_name_parsed;

    for (Json::Value::ArrayIndex i = 0; i < cooling_devices.size(); ++i) {
        const std::string &name = cooling_devices[i]["Name"].asString();
        LOG(INFO) << "CoolingDevice[" << i << "]'s Name: " << name;
        if (name.empty()) {
            LOG(ERROR) << "Failed to read CoolingDevice[" << i << "]'s Name";
            cooling_devices_parsed->clear();
            return false;
        }

        auto result = cooling_devices_name_parsed.insert(name.data());
        if (!result.second) {
            LOG(ERROR) << "Duplicate CoolingDevice[" << i << "]'s Name";
            cooling_devices_parsed->clear();
            return false;
        }

        std::string cooling_device_type_str = cooling_devices[i]["Type"].asString();
        LOG(INFO) << "CoolingDevice[" << name << "]'s Type: " << cooling_device_type_str;
        CoolingType cooling_device_type;

        if (!getTypeFromString(cooling_device_type_str, &cooling_device_type)) {
            LOG(ERROR) << "Invalid CoolingDevice[" << name
                       << "]'s Type: " << cooling_device_type_str;
            cooling_devices_parsed->clear();
            return false;
        }

        const std::string &read_path = cooling_devices[i]["ReadPath"].asString();
        LOG(INFO) << "Cdev Read Path: " << (read_path.empty() ? "default" : read_path);

        const std::string &write_path = cooling_devices[i]["WritePath"].asString();
        LOG(INFO) << "Cdev Write Path: " << (write_path.empty() ? "default" : write_path);

        std::vector<float> state2power;
        Json::Value values = cooling_devices[i]["State2Power"];
        if (values.size()) {
            state2power.reserve(values.size());
            for (Json::Value::ArrayIndex j = 0; j < values.size(); ++j) {
                state2power.emplace_back(getFloatFromValue(values[j]));
                LOG(INFO) << "Cooling device[" << name << "]'s Power2State[" << j
                          << "]: " << state2power[j];
            }
        } else {
            LOG(INFO) << "CoolingDevice[" << i << "]'s Name: " << name
                      << " does not support State2Power";
        }

        const std::string &power_rail = cooling_devices[i]["PowerRail"].asString();
        LOG(INFO) << "Cooling device power rail : " << power_rail;

        (*cooling_devices_parsed)[name] = {
                .type = cooling_device_type,
                .read_path = read_path,
                .write_path = write_path,
                .state2power = state2power,
        };
        ++total_parsed;
    }
    LOG(INFO) << total_parsed << " CoolingDevices parsed successfully";
    return true;
}

bool ParsePowerRailInfo(const Json::Value &config,
                        std::unordered_map<std::string, PowerRailInfo> *power_rails_parsed) {
    Json::Value power_rails = config["PowerRails"];
    std::size_t total_parsed = 0;
    std::unordered_set<std::string> power_rails_name_parsed;

    for (Json::Value::ArrayIndex i = 0; i < power_rails.size(); ++i) {
        const std::string &name = power_rails[i]["Name"].asString();
        LOG(INFO) << "PowerRail[" << i << "]'s Name: " << name;
        if (name.empty()) {
            LOG(ERROR) << "Failed to read PowerRail[" << i << "]'s Name";
            power_rails_parsed->clear();
            return false;
        }

        std::vector<std::string> linked_power_rails;
        std::vector<float> coefficient;
        float offset = 0;
        FormulaOption formula = FormulaOption::COUNT_THRESHOLD;
        bool is_virtual_power_rail = false;
        Json::Value values;
        int power_sample_count = 0;
        std::chrono::milliseconds power_sample_delay;

        if (!power_rails[i]["VirtualRails"].empty() && power_rails[i]["VirtualRails"].isBool()) {
            is_virtual_power_rail = power_rails[i]["VirtualRails"].asBool();
            LOG(INFO) << "PowerRails[" << name << "]'s VirtualRail, set to 'true'";
        }

        if (is_virtual_power_rail) {
            values = power_rails[i]["Combination"];
            if (values.size()) {
                linked_power_rails.reserve(values.size());
                for (Json::Value::ArrayIndex j = 0; j < values.size(); ++j) {
                    linked_power_rails.emplace_back(values[j].asString());
                    LOG(INFO) << "PowerRail[" << name << "]'s combination[" << j
                              << "]: " << linked_power_rails[j];
                }
            } else {
                LOG(ERROR) << "PowerRails[" << name << "] has no combination for VirtualRail";
                power_rails_parsed->clear();
                return false;
            }

            values = power_rails[i]["Coefficient"];
            if (values.size()) {
                coefficient.reserve(values.size());
                for (Json::Value::ArrayIndex j = 0; j < values.size(); ++j) {
                    coefficient.emplace_back(getFloatFromValue(values[j]));
                    LOG(INFO) << "PowerRail[" << name << "]'s coefficient[" << j
                              << "]: " << coefficient[j];
                }
            } else {
                LOG(ERROR) << "PowerRails[" << name << "] has no coefficient for VirtualRail";
                power_rails_parsed->clear();
                return false;
            }

            if (linked_power_rails.size() != coefficient.size()) {
                LOG(ERROR) << "PowerRails[" << name
                           << "]'s combination size is not matched with coefficient size";
                power_rails_parsed->clear();
                return false;
            }

            if (!power_rails[i]["Offset"].empty()) {
                offset = power_rails[i]["Offset"].asFloat();
            }

            if (linked_power_rails.size() != coefficient.size()) {
                LOG(ERROR) << "PowerRails[" << name
                           << "]'s combination size is not matched with coefficient size";
                power_rails_parsed->clear();
                return false;
            }

            if (power_rails[i]["Formula"].asString().compare("COUNT_THRESHOLD") == 0) {
                formula = FormulaOption::COUNT_THRESHOLD;
            } else if (power_rails[i]["Formula"].asString().compare("WEIGHTED_AVG") == 0) {
                formula = FormulaOption::WEIGHTED_AVG;
            } else if (power_rails[i]["Formula"].asString().compare("MAXIMUM") == 0) {
                formula = FormulaOption::MAXIMUM;
            } else if (power_rails[i]["Formula"].asString().compare("MINIMUM") == 0) {
                formula = FormulaOption::MINIMUM;
            } else {
                LOG(ERROR) << "PowerRails[" << name << "]'s Formula is invalid";
                power_rails_parsed->clear();
                return false;
            }
        }

        std::unique_ptr<VirtualPowerRailInfo> virtual_power_rail_info;
        if (is_virtual_power_rail) {
            virtual_power_rail_info.reset(
                    new VirtualPowerRailInfo{linked_power_rails, coefficient, offset, formula});
        }

        power_sample_count = power_rails[i]["PowerSampleCount"].asInt();
        LOG(INFO) << "Power sample Count: " << power_sample_count;

        if (!power_rails[i]["PowerSampleDelay"]) {
            power_sample_delay = std::chrono::milliseconds::max();
        } else {
            power_sample_delay =
                    std::chrono::milliseconds(getIntFromValue(power_rails[i]["PowerSampleDelay"]));
        }

        (*power_rails_parsed)[name] = {
                .power_sample_count = power_sample_count,
                .power_sample_delay = power_sample_delay,
                .virtual_power_rail_info = std::move(virtual_power_rail_info),
        };
        ++total_parsed;
    }
    LOG(INFO) << total_parsed << " PowerRails parsed successfully";
    return true;
}

template <typename T, typename U>
bool ParseStatsInfo(const Json::Value &stats_config,
                    const std::unordered_map<std::string, U> &entity_info, StatsInfo<T> *stats_info,
                    T min_value) {
    if (stats_config.empty()) {
        LOG(INFO) << "No stats config";
        return true;
    }
    std::variant<bool, std::unordered_set<std::string>>
            record_by_default_threshold_all_or_name_set_ = false;
    if (stats_config["DefaultThresholdEnableAll"].empty() ||
        !stats_config["DefaultThresholdEnableAll"].isBool()) {
        LOG(INFO) << "Failed to read stats DefaultThresholdEnableAll, set to 'false'";
    } else if (stats_config["DefaultThresholdEnableAll"].asBool()) {
        record_by_default_threshold_all_or_name_set_ = true;
    }
    LOG(INFO) << "DefaultThresholdEnableAll " << std::boolalpha
              << std::get<bool>(record_by_default_threshold_all_or_name_set_) << std::noboolalpha;

    Json::Value values = stats_config["RecordWithDefaultThreshold"];
    if (values.size()) {
        if (std::get<bool>(record_by_default_threshold_all_or_name_set_)) {
            LOG(ERROR) << "Cannot enable record with default threshold when "
                          "DefaultThresholdEnableAll true.";
            return false;
        }
        record_by_default_threshold_all_or_name_set_ = std::unordered_set<std::string>();
        for (Json::Value::ArrayIndex i = 0; i < values.size(); ++i) {
            std::string name = values[i].asString();
            if (!entity_info.count(name)) {
                LOG(ERROR) << "Unknown name [" << name << "] not present in entity_info.";
                return false;
            }
            std::get<std::unordered_set<std::string>>(record_by_default_threshold_all_or_name_set_)
                    .insert(name);
        }
    } else {
        LOG(INFO) << "No stat by default threshold enabled.";
    }

    std::unordered_map<std::string, std::vector<ThresholdList<T>>> record_by_threshold;
    values = stats_config["RecordWithThreshold"];
    if (values.size()) {
        Json::Value threshold_values;
        for (Json::Value::ArrayIndex i = 0; i < values.size(); i++) {
            const std::string &name = values[i]["Name"].asString();
            if (!entity_info.count(name)) {
                LOG(ERROR) << "Unknown name [" << name << "] not present in entity_info.";
                return false;
            }

            std::optional<std::string> logging_name;
            if (!values[i]["LoggingName"].empty()) {
                logging_name = values[i]["LoggingName"].asString();
                LOG(INFO) << "For [" << name << "]"
                          << ", stats logging name is [" << logging_name.value() << "]";
            }

            LOG(INFO) << "Start to parse stats threshold for [" << name << "]";
            threshold_values = values[i]["Thresholds"];
            if (threshold_values.empty()) {
                LOG(ERROR) << "Empty stats threshold not valid.";
                return false;
            }
            const auto &threshold_values_count = threshold_values.size();
            if (threshold_values_count > kMaxStatsThresholdCount) {
                LOG(ERROR) << "Number of stats threshold " << threshold_values_count
                           << " greater than max " << kMaxStatsThresholdCount;
                return false;
            }
            std::vector<T> stats_threshold(threshold_values_count);
            T prev_value = min_value;
            LOG(INFO) << "Thresholds:";
            for (Json::Value::ArrayIndex i = 0; i < threshold_values_count; ++i) {
                stats_threshold[i] = std::is_floating_point_v<T>
                                             ? getFloatFromValue(threshold_values[i])
                                             : getIntFromValue(threshold_values[i]);
                if (stats_threshold[i] <= prev_value) {
                    LOG(ERROR) << "Invalid array[" << i << "]" << stats_threshold[i]
                               << " is <=" << prev_value;
                    return false;
                }
                prev_value = stats_threshold[i];
                LOG(INFO) << "[" << i << "]: " << stats_threshold[i];
            }
            record_by_threshold[name].emplace_back(logging_name, stats_threshold);
        }
    } else {
        LOG(INFO) << "No stat by threshold enabled.";
    }

    (*stats_info) = {.record_by_default_threshold_all_or_name_set_ =
                             record_by_default_threshold_all_or_name_set_,
                     .record_by_threshold = record_by_threshold};
    return true;
}

bool ParseSensorAbnormalStatsConfig(
        const Json::Value &abnormal_stats_config,
        const std::unordered_map<std::string, SensorInfo> &sensor_info_map_,
        AbnormalStatsInfo *abnormal_stats_info_parsed) {
    if (abnormal_stats_config.empty()) {
        LOG(INFO) << "No sensors abnormality monitoring info present.";
        return true;
    }

    Json::Value values;

    std::optional<TempRangeInfo> default_temp_range_info;
    std::vector<AbnormalStatsInfo::SensorsTempRangeInfo> sensors_temp_range_infos;
    Json::Value outlier_temp_config = abnormal_stats_config["Outlier"];
    if (outlier_temp_config) {
        LOG(INFO) << "Start to parse outlier temp config.";

        if (outlier_temp_config["Default"]) {
            LOG(INFO) << "Start to parse defaultTempRange.";
            if (!getTempRangeInfoFromJsonValues(outlier_temp_config["Default"],
                                                &default_temp_range_info.value())) {
                LOG(ERROR) << "Failed to parse default temp range config.";
                return false;
            }
        }

        Json::Value configs = outlier_temp_config["Configs"];
        if (configs) {
            std::unordered_set<std::string> sensors_parsed;
            for (Json::Value::ArrayIndex i = 0; i < configs.size(); i++) {
                LOG(INFO) << "Start to parse temp range config[" << i << "]";
                AbnormalStatsInfo::SensorsTempRangeInfo sensors_temp_range_info;
                values = configs[i]["Monitor"];
                if (!values.size()) {
                    LOG(ERROR) << "Invalid config no sensor list present for outlier temp "
                                  "config.";
                    return false;
                }
                for (Json::Value::ArrayIndex j = 0; j < values.size(); j++) {
                    const std::string &sensor = values[j].asString();
                    if (!sensor_info_map_.count(sensor)) {
                        LOG(ERROR) << "Unknown sensor " << sensor;
                        return false;
                    }
                    auto result = sensors_parsed.insert(sensor);
                    if (!result.second) {
                        LOG(ERROR) << "Duplicate Sensor Temp Range Config: " << sensor;
                        return false;
                    }
                    LOG(INFO) << "Monitored sensor [" << j << "]: " << sensor;
                    sensors_temp_range_info.sensors.push_back(sensor);
                }
                if (!getTempRangeInfoFromJsonValues(configs[i]["TempRange"],
                                                    &sensors_temp_range_info.temp_range_info)) {
                    LOG(ERROR) << "Failed to parse temp range config.";
                    return false;
                }
                sensors_temp_range_infos.push_back(sensors_temp_range_info);
            }
        }
    }
    std::optional<TempStuckInfo> default_temp_stuck_info;
    std::vector<AbnormalStatsInfo::SensorsTempStuckInfo> sensors_temp_stuck_infos;
    Json::Value stuck_temp_config = abnormal_stats_config["Stuck"];
    if (stuck_temp_config) {
        LOG(INFO) << "Start to parse stuck temp config.";

        if (stuck_temp_config["Default"]) {
            LOG(INFO) << "Start to parse defaultTempStuck.";
            if (!getTempStuckInfoFromJsonValue(stuck_temp_config["Default"],
                                               &default_temp_stuck_info.value())) {
                LOG(ERROR) << "Failed to parse default temp stuck config.";
                return false;
            }
        }

        Json::Value configs = stuck_temp_config["Configs"];
        if (configs) {
            std::unordered_set<std::string> sensors_parsed;
            for (Json::Value::ArrayIndex i = 0; i < configs.size(); i++) {
                LOG(INFO) << "Start to parse temp stuck config[" << i << "]";
                AbnormalStatsInfo::SensorsTempStuckInfo sensor_temp_stuck_info;
                values = configs[i]["Monitor"];
                if (!values.size()) {
                    LOG(ERROR) << "Invalid config no sensor list present for stuck temp "
                                  "config.";
                    return false;
                }
                for (Json::Value::ArrayIndex j = 0; j < values.size(); j++) {
                    const std::string &sensor = values[j].asString();
                    if (!sensor_info_map_.count(sensor)) {
                        LOG(ERROR) << "Unknown sensor " << sensor;
                        return false;
                    }
                    auto result = sensors_parsed.insert(sensor);
                    if (!result.second) {
                        LOG(ERROR) << "Duplicate Sensor Temp Stuck Config: " << sensor;
                        return false;
                    }
                    LOG(INFO) << "Monitored sensor [" << j << "]: " << sensor;
                    sensor_temp_stuck_info.sensors.push_back(sensor);
                }
                if (!getTempStuckInfoFromJsonValue(configs[i]["TempStuck"],
                                                   &sensor_temp_stuck_info.temp_stuck_info)) {
                    LOG(ERROR) << "Failed to parse temp stuck config.";
                    return false;
                }
                sensors_temp_stuck_infos.push_back(sensor_temp_stuck_info);
            }
        }
    }
    *abnormal_stats_info_parsed = {
            .default_temp_range_info = default_temp_range_info,
            .sensors_temp_range_infos = sensors_temp_range_infos,
            .default_temp_stuck_info = default_temp_stuck_info,
            .sensors_temp_stuck_infos = sensors_temp_stuck_infos,
    };
    return true;
}

bool ParseSensorStatsConfig(const Json::Value &config,
                            const std::unordered_map<std::string, SensorInfo> &sensor_info_map_,
                            StatsInfo<float> *sensor_stats_info_parsed,
                            AbnormalStatsInfo *abnormal_stats_info_parsed) {
    Json::Value stats_config = config["Stats"];
    if (stats_config.empty()) {
        LOG(INFO) << "No Stats Config present.";
        return true;
    }
    // Parse cooling device user vote
    Json::Value sensor_config = stats_config["Sensors"];
    if (sensor_config.empty()) {
        LOG(INFO) << "No Sensor Stats Config present.";
        return true;
    }
    LOG(INFO) << "Parse Stats Config for Sensor Temp.";
    // Parse sensor stats config
    if (!ParseStatsInfo(stats_config["Sensors"], sensor_info_map_, sensor_stats_info_parsed,
                        std::numeric_limits<float>::lowest())) {
        LOG(ERROR) << "Failed to parse sensor temp stats info.";
        sensor_stats_info_parsed->clear();
        return false;
    }
    if (!ParseSensorAbnormalStatsConfig(sensor_config["Abnormality"], sensor_info_map_,
                                        abnormal_stats_info_parsed)) {
        LOG(ERROR) << "Failed to parse sensor abnormal stats config.";
        return false;
    }
    return true;
}

bool ParseCoolingDeviceStatsConfig(
        const Json::Value &config,
        const std::unordered_map<std::string, CdevInfo> &cooling_device_info_map_,
        StatsInfo<int> *cooling_device_request_info_parsed) {
    Json::Value stats_config = config["Stats"];
    if (stats_config.empty()) {
        LOG(INFO) << "No Stats Config present.";
        return true;
    }
    // Parse cooling device user vote
    if (stats_config["CoolingDevices"].empty()) {
        LOG(INFO) << "No cooling device stats present.";
        return true;
    }
    LOG(INFO) << "Parse Stats Config for Sensor CDev Request.";
    if (!ParseStatsInfo(stats_config["CoolingDevices"]["RecordVotePerSensor"],
                        cooling_device_info_map_, cooling_device_request_info_parsed, -1)) {
        LOG(ERROR) << "Failed to parse cooling device user vote stats info.";
        cooling_device_request_info_parsed->clear();
        return false;
    }
    return true;
}
}  // namespace implementation
}  // namespace thermal
}  // namespace hardware
}  // namespace android
}  // namespace aidl
