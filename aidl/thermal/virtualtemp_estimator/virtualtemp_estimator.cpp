/*
 * Copyright (C) 2023 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define ATRACE_TAG (ATRACE_TAG_THERMAL | ATRACE_TAG_HAL)

#include "virtualtemp_estimator.h"

#include <android-base/logging.h>
#include <android-base/stringprintf.h>
#include <dlfcn.h>
#include <json/reader.h>
#include <utils/Trace.h>

#include <cmath>
#include <sstream>
#include <vector>

namespace thermal {
namespace vtestimator {
namespace {
float getFloatFromValue(const Json::Value &value) {
    if (value.isString()) {
        return std::atof(value.asString().c_str());
    } else {
        return value.asFloat();
    }
}

bool getInputRangeInfoFromJsonValues(const Json::Value &values, InputRangeInfo *input_range_info) {
    if (values.size() != 2) {
        LOG(ERROR) << "Data Range Values size: " << values.size() << "is invalid.";
        return false;
    }

    float min_val = getFloatFromValue(values[0]);
    float max_val = getFloatFromValue(values[1]);

    if (std::isnan(min_val) || std::isnan(max_val)) {
        LOG(ERROR) << "Illegal data range: thresholds not defined properly " << min_val << " : "
                   << max_val;
        return false;
    }

    if (min_val > max_val) {
        LOG(ERROR) << "Illegal data range: data_min_threshold(" << min_val
                   << ") > data_max_threshold(" << max_val << ")";
        return false;
    }
    input_range_info->min_threshold = min_val;
    input_range_info->max_threshold = max_val;
    LOG(INFO) << "Data Range Info: " << input_range_info->min_threshold
              << " <= val <= " << input_range_info->max_threshold;
    return true;
}

float CalculateOffset(const std::vector<float> &offset_thresholds,
                      const std::vector<float> &offset_values, const float value) {
    for (int i = offset_thresholds.size(); i > 0; --i) {
        if (offset_thresholds[i - 1] < value) {
            return offset_values[i - 1];
        }
    }

    return 0;
}
}  // namespace

VtEstimatorStatus VirtualTempEstimator::DumpTraces() {
    if (type != kUseMLModel) {
        return kVtEstimatorUnSupported;
    }

    if (tflite_instance_ == nullptr || common_instance_ == nullptr) {
        LOG(ERROR) << "tflite_instance_ or common_instance_ is nullptr during DumpTraces\n";
        return kVtEstimatorInitFailed;
    }

    std::unique_lock<std::mutex> lock(tflite_instance_->tflite_methods.mutex);

    if (!common_instance_->is_initialized) {
        LOG(ERROR) << "tflite_instance_ not initialized for " << common_instance_->sensor_name;
        return kVtEstimatorInitFailed;
    }

    // get model input/output buffers
    float *model_input = tflite_instance_->input_buffer;
    float *model_output = tflite_instance_->output_buffer;
    auto input_buffer_size = tflite_instance_->input_buffer_size;
    auto output_buffer_size = tflite_instance_->output_buffer_size;

    // In Case of use_prev_samples, inputs are available in order in scratch buffer
    if (common_instance_->use_prev_samples) {
        model_input = tflite_instance_->scratch_buffer;
    }

    // Add traces for model input/output buffers
    std::string sensor_name = common_instance_->sensor_name;
    for (size_t i = 0; i < input_buffer_size; ++i) {
        ATRACE_INT((sensor_name + "_input_" + std::to_string(i)).c_str(),
                   static_cast<int>(model_input[i]));
    }

    for (size_t i = 0; i < output_buffer_size; ++i) {
        ATRACE_INT((sensor_name + "_output_" + std::to_string(i)).c_str(),
                   static_cast<int>(model_output[i]));
    }

    // log input data and output data buffers
    std::string input_data_str = "model_input_buffer: [";
    for (size_t i = 0; i < input_buffer_size; ++i) {
        input_data_str += ::android::base::StringPrintf("%0.2f ", model_input[i]);
    }
    input_data_str += "]";
    LOG(INFO) << input_data_str;

    std::string output_data_str = "model_output_buffer: [";
    for (size_t i = 0; i < output_buffer_size; ++i) {
        output_data_str += ::android::base::StringPrintf("%0.2f ", model_output[i]);
    }
    output_data_str += "]";
    LOG(INFO) << output_data_str;

    return kVtEstimatorOk;
}

void VirtualTempEstimator::LoadTFLiteWrapper() {
    if (!tflite_instance_) {
        LOG(ERROR) << "tflite_instance_ is nullptr during LoadTFLiteWrapper";
        return;
    }

    std::unique_lock<std::mutex> lock(tflite_instance_->tflite_methods.mutex);

    void *mLibHandle = dlopen("/vendor/lib64/libthermal_tflite_wrapper.so", 0);
    if (mLibHandle == nullptr) {
        LOG(ERROR) << "Could not load libthermal_tflite_wrapper library with error: " << dlerror();
        return;
    }

    tflite_instance_->tflite_methods.create =
            reinterpret_cast<tflitewrapper_create>(dlsym(mLibHandle, "ThermalTfliteCreate"));
    if (!tflite_instance_->tflite_methods.create) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_create with error: " << dlerror();
    }

    tflite_instance_->tflite_methods.init =
            reinterpret_cast<tflitewrapper_init>(dlsym(mLibHandle, "ThermalTfliteInit"));
    if (!tflite_instance_->tflite_methods.init) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_init with error: " << dlerror();
    }

    tflite_instance_->tflite_methods.invoke =
            reinterpret_cast<tflitewrapper_invoke>(dlsym(mLibHandle, "ThermalTfliteInvoke"));
    if (!tflite_instance_->tflite_methods.invoke) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_invoke with error: " << dlerror();
    }

    tflite_instance_->tflite_methods.destroy =
            reinterpret_cast<tflitewrapper_destroy>(dlsym(mLibHandle, "ThermalTfliteDestroy"));
    if (!tflite_instance_->tflite_methods.destroy) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_destroy with error: " << dlerror();
    }

    tflite_instance_->tflite_methods.get_input_config_size =
            reinterpret_cast<tflitewrapper_get_input_config_size>(
                    dlsym(mLibHandle, "ThermalTfliteGetInputConfigSize"));
    if (!tflite_instance_->tflite_methods.get_input_config_size) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_get_input_config_size with error: "
                   << dlerror();
    }

    tflite_instance_->tflite_methods.get_input_config =
            reinterpret_cast<tflitewrapper_get_input_config>(
                    dlsym(mLibHandle, "ThermalTfliteGetInputConfig"));
    if (!tflite_instance_->tflite_methods.get_input_config) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_get_input_config with error: "
                   << dlerror();
    }
}

VirtualTempEstimator::VirtualTempEstimator(std::string_view sensor_name,
                                           VtEstimationType estimationType,
                                           size_t num_linked_sensors) {
    type = estimationType;

    common_instance_ = std::make_unique<VtEstimatorCommonData>(sensor_name, num_linked_sensors);
    if (estimationType == kUseMLModel) {
        tflite_instance_ = std::make_unique<VtEstimatorTFLiteData>();
        LoadTFLiteWrapper();
    } else if (estimationType == kUseLinearModel) {
        linear_model_instance_ = std::make_unique<VtEstimatorLinearModelData>();
    } else {
        LOG(ERROR) << "Unsupported estimationType [" << estimationType << "]";
    }
}

VirtualTempEstimator::~VirtualTempEstimator() {
    LOG(INFO) << "VirtualTempEstimator destructor";
}

VtEstimatorStatus VirtualTempEstimator::LinearModelInitialize(LinearModelInitData data) {
    if (linear_model_instance_ == nullptr || common_instance_ == nullptr) {
        LOG(ERROR) << "linear_model_instance_ or common_instance_ is nullptr during Initialize";
        return kVtEstimatorInitFailed;
    }

    size_t num_linked_sensors = common_instance_->num_linked_sensors;
    std::unique_lock<std::mutex> lock(linear_model_instance_->mutex);

    if ((num_linked_sensors == 0) || (data.coefficients.size() == 0) ||
        (data.prev_samples_order == 0)) {
        LOG(ERROR) << "Invalid num_linked_sensors [" << num_linked_sensors
                   << "] or coefficients.size() [" << data.coefficients.size()
                   << "] or prev_samples_order [" << data.prev_samples_order << "]";
        return kVtEstimatorInitFailed;
    }

    if (data.coefficients.size() != (num_linked_sensors * data.prev_samples_order)) {
        LOG(ERROR) << "In valid args coefficients.size()[" << data.coefficients.size()
                   << "] num_linked_sensors [" << num_linked_sensors << "] prev_samples_order["
                   << data.prev_samples_order << "]";
        return kVtEstimatorInvalidArgs;
    }

    common_instance_->use_prev_samples = data.use_prev_samples;
    common_instance_->prev_samples_order = data.prev_samples_order;

    linear_model_instance_->input_samples.reserve(common_instance_->prev_samples_order);
    linear_model_instance_->coefficients.reserve(common_instance_->prev_samples_order);

    // Store coefficients
    for (size_t i = 0; i < data.prev_samples_order; ++i) {
        std::vector<float> single_order_coefficients;
        for (size_t j = 0; j < num_linked_sensors; ++j) {
            single_order_coefficients.emplace_back(data.coefficients[i * num_linked_sensors + j]);
        }
        linear_model_instance_->coefficients.emplace_back(single_order_coefficients);
    }

    common_instance_->offset_thresholds = data.offset_thresholds;
    common_instance_->offset_values = data.offset_values;
    common_instance_->is_initialized = true;

    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::TFliteInitialize(MLModelInitData data) {
    if (!tflite_instance_ || !common_instance_) {
        LOG(ERROR) << "tflite_instance_ or common_instance_ is nullptr during Initialize\n";
        return kVtEstimatorInitFailed;
    }

    std::string_view sensor_name = common_instance_->sensor_name;
    size_t num_linked_sensors = common_instance_->num_linked_sensors;
    bool use_prev_samples = data.use_prev_samples;
    size_t prev_samples_order = data.prev_samples_order;
    size_t num_hot_spots = data.num_hot_spots;
    size_t output_label_count = data.output_label_count;

    std::unique_lock<std::mutex> lock(tflite_instance_->tflite_methods.mutex);

    if (data.model_path.empty()) {
        LOG(ERROR) << "Invalid model_path:" << data.model_path << " for " << sensor_name;
        return kVtEstimatorInvalidArgs;
    }

    if (num_linked_sensors == 0 || prev_samples_order < 1 ||
        (!use_prev_samples && prev_samples_order > 1)) {
        LOG(ERROR) << "Invalid tflite_instance_ config: " << "number of linked sensor: "
                   << num_linked_sensors << " use previous: " << use_prev_samples
                   << " previous sample order: " << prev_samples_order << " for " << sensor_name;
        return kVtEstimatorInitFailed;
    }

    common_instance_->use_prev_samples = data.use_prev_samples;
    common_instance_->prev_samples_order = prev_samples_order;
    tflite_instance_->support_under_sampling = data.support_under_sampling;
    tflite_instance_->enable_input_validation = data.enable_input_validation;
    tflite_instance_->input_buffer_size = num_linked_sensors * prev_samples_order;
    tflite_instance_->input_buffer = new float[tflite_instance_->input_buffer_size];
    if (common_instance_->use_prev_samples) {
        tflite_instance_->scratch_buffer = new float[tflite_instance_->input_buffer_size];
    }

    if (output_label_count < 1 || num_hot_spots < 1) {
        LOG(ERROR) << "Invalid tflite_instance_ config:" << "number of hot spots: " << num_hot_spots
                   << " predicted sample order: " << output_label_count << " for " << sensor_name;
        return kVtEstimatorInitFailed;
    }

    tflite_instance_->output_label_count = output_label_count;
    tflite_instance_->num_hot_spots = num_hot_spots;
    tflite_instance_->output_buffer_size = output_label_count * num_hot_spots;
    tflite_instance_->output_buffer = new float[tflite_instance_->output_buffer_size];

    if (!tflite_instance_->tflite_methods.create || !tflite_instance_->tflite_methods.init ||
        !tflite_instance_->tflite_methods.invoke || !tflite_instance_->tflite_methods.destroy ||
        !tflite_instance_->tflite_methods.get_input_config_size ||
        !tflite_instance_->tflite_methods.get_input_config) {
        LOG(ERROR) << "Invalid tflite methods for " << sensor_name;
        return kVtEstimatorInitFailed;
    }

    tflite_instance_->tflite_wrapper =
            tflite_instance_->tflite_methods.create(kNumInputTensors, kNumOutputTensors);
    if (!tflite_instance_->tflite_wrapper) {
        LOG(ERROR) << "Failed to create tflite wrapper for " << sensor_name;
        return kVtEstimatorInitFailed;
    }

    int ret = tflite_instance_->tflite_methods.init(tflite_instance_->tflite_wrapper,
                                                    data.model_path.c_str());
    if (ret) {
        LOG(ERROR) << "Failed to Init tflite_wrapper for " << sensor_name << " (ret: " << ret
                   << ")";
        return kVtEstimatorInitFailed;
    }

    Json::Value input_config;
    if (!GetInputConfig(&input_config)) {
        LOG(ERROR) << "Get Input Config failed for " << sensor_name;
        return kVtEstimatorInitFailed;
    }

    if (!ParseInputConfig(input_config)) {
        LOG(ERROR) << "Parse Input Config failed for " << sensor_name;
        return kVtEstimatorInitFailed;
    }

    if (tflite_instance_->enable_input_validation && !tflite_instance_->input_range.size()) {
        LOG(ERROR) << "Input ranges missing when input data validation is enabled for "
                   << sensor_name;
        return kVtEstimatorInitFailed;
    }

    common_instance_->offset_thresholds = data.offset_thresholds;
    common_instance_->offset_values = data.offset_values;
    tflite_instance_->model_path = data.model_path;

    common_instance_->is_initialized = true;
    LOG(INFO) << "Successfully initialized VirtualTempEstimator for " << sensor_name;
    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::LinearModelEstimate(const std::vector<float> &thermistors,
                                                            std::vector<float> *output) {
    if (linear_model_instance_ == nullptr || common_instance_ == nullptr) {
        LOG(ERROR) << "linear_model_instance_ or common_instance_ is nullptr during Initialize";
        return kVtEstimatorInitFailed;
    }

    std::string_view sensor_name = common_instance_->sensor_name;
    size_t prev_samples_order = common_instance_->prev_samples_order;
    size_t num_linked_sensors = common_instance_->num_linked_sensors;

    std::unique_lock<std::mutex> lock(linear_model_instance_->mutex);

    if ((thermistors.size() != num_linked_sensors) || (output == nullptr)) {
        LOG(ERROR) << "Invalid args Thermistors size[" << thermistors.size()
                   << "] num_linked_sensors[" << num_linked_sensors << "] output[" << output << "]"
                   << " for " << sensor_name;
        return kVtEstimatorInvalidArgs;
    }

    if (common_instance_->is_initialized == false) {
        LOG(ERROR) << "tflite_instance_ not initialized for " << sensor_name;
        return kVtEstimatorInitFailed;
    }

    // For the first iteration copy current inputs to all previous inputs
    // This would allow the estimator to have previous samples from the first iteration itself
    // and provide a valid predicted value
    if (common_instance_->cur_sample_count == 0) {
        for (size_t i = 0; i < prev_samples_order; ++i) {
            linear_model_instance_->input_samples[i] = thermistors;
        }
    }

    size_t cur_sample_index = common_instance_->cur_sample_count % prev_samples_order;
    linear_model_instance_->input_samples[cur_sample_index] = thermistors;

    // Calculate Weighted Average Value
    int input_level = cur_sample_index;
    float estimated_value = 0;
    for (size_t i = 0; i < prev_samples_order; ++i) {
        for (size_t j = 0; j < num_linked_sensors; ++j) {
            estimated_value += linear_model_instance_->coefficients[i][j] *
                               linear_model_instance_->input_samples[input_level][j];
        }
        input_level--;  // go to previous samples
        input_level = (input_level >= 0) ? input_level : (prev_samples_order - 1);
    }

    // Update sample count
    common_instance_->cur_sample_count++;

    // add offset to estimated value if applicable
    estimated_value += CalculateOffset(common_instance_->offset_thresholds,
                                       common_instance_->offset_values, estimated_value);

    std::vector<float> data = {estimated_value};
    *output = data;
    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::TFliteEstimate(const std::vector<float> &thermistors,
                                                       std::vector<float> *output) {
    if (tflite_instance_ == nullptr || common_instance_ == nullptr) {
        LOG(ERROR) << "tflite_instance_ or common_instance_ is nullptr during Estimate\n";
        return kVtEstimatorInitFailed;
    }

    std::unique_lock<std::mutex> lock(tflite_instance_->tflite_methods.mutex);

    if (!common_instance_->is_initialized) {
        LOG(ERROR) << "tflite_instance_ not initialized for " << tflite_instance_->model_path;
        return kVtEstimatorInitFailed;
    }

    std::string_view sensor_name = common_instance_->sensor_name;
    size_t num_linked_sensors = common_instance_->num_linked_sensors;
    if ((thermistors.size() != num_linked_sensors) || (output == nullptr)) {
        LOG(ERROR) << "Invalid args for " << sensor_name
                   << " thermistors.size(): " << thermistors.size()
                   << " num_linked_sensors: " << num_linked_sensors << " output: " << output;
        return kVtEstimatorInvalidArgs;
    }

    // log input data
    std::string input_data_str = "model_input: [";
    for (size_t i = 0; i < num_linked_sensors; ++i) {
        input_data_str += ::android::base::StringPrintf("%0.2f ", thermistors[i]);
    }
    input_data_str += "]";
    LOG(INFO) << sensor_name << ": " << input_data_str;

    // check time gap between samples and ignore stale previous samples
    if (std::chrono::duration_cast<std::chrono::milliseconds>(boot_clock::now() -
                                                              tflite_instance_->prev_sample_time) >=
        tflite_instance_->max_sample_interval) {
        LOG(INFO) << "Ignoring stale previous samples for " << sensor_name;
        common_instance_->cur_sample_count = 0;
    }

    // copy input data into input tensors
    size_t prev_samples_order = common_instance_->prev_samples_order;
    size_t cur_sample_index = common_instance_->cur_sample_count % prev_samples_order;
    size_t sample_start_index = cur_sample_index * num_linked_sensors;
    for (size_t i = 0; i < num_linked_sensors; ++i) {
        if (tflite_instance_->enable_input_validation) {
            if (thermistors[i] < tflite_instance_->input_range[i].min_threshold ||
                thermistors[i] > tflite_instance_->input_range[i].max_threshold) {
                LOG(INFO) << "thermistors[" << i << "] value: " << thermistors[i]
                          << " not in range: " << tflite_instance_->input_range[i].min_threshold
                          << " <= val <= " << tflite_instance_->input_range[i].max_threshold
                          << " for " << sensor_name;
                common_instance_->cur_sample_count = 0;
                return kVtEstimatorLowConfidence;
            }
        }
        tflite_instance_->input_buffer[sample_start_index + i] = thermistors[i];
        if (cur_sample_index == 0 && tflite_instance_->support_under_sampling) {
            // fill previous samples if support under sampling
            for (size_t j = 1; j < prev_samples_order; ++j) {
                size_t copy_start_index = j * num_linked_sensors;
                tflite_instance_->input_buffer[copy_start_index + i] = thermistors[i];
            }
        }
    }

    // Update sample count
    common_instance_->cur_sample_count++;
    tflite_instance_->prev_sample_time = boot_clock::now();
    if ((common_instance_->cur_sample_count < prev_samples_order) &&
        !(tflite_instance_->support_under_sampling)) {
        return kVtEstimatorUnderSampling;
    }

    // prepare model input
    float *model_input;
    size_t input_buffer_size = tflite_instance_->input_buffer_size;
    size_t output_buffer_size = tflite_instance_->output_buffer_size;
    if (!common_instance_->use_prev_samples) {
        model_input = tflite_instance_->input_buffer;
    } else {
        sample_start_index = ((cur_sample_index + 1) * num_linked_sensors) % input_buffer_size;
        for (size_t i = 0; i < input_buffer_size; ++i) {
            size_t input_index = (sample_start_index + i) % input_buffer_size;
            tflite_instance_->scratch_buffer[i] = tflite_instance_->input_buffer[input_index];
        }
        model_input = tflite_instance_->scratch_buffer;
    }

    int ret = tflite_instance_->tflite_methods.invoke(
            tflite_instance_->tflite_wrapper, model_input, input_buffer_size,
            tflite_instance_->output_buffer, output_buffer_size);
    if (ret) {
        LOG(ERROR) << "Failed to Invoke for " << sensor_name << " (ret: " << ret << ")";
        return kVtEstimatorInvokeFailed;
    }
    tflite_instance_->last_update_time = boot_clock::now();

    // prepare output
    std::vector<float> data;
    std::ostringstream model_out_log, predict_log;
    data.reserve(output_buffer_size);
    for (size_t i = 0; i < output_buffer_size; ++i) {
        // add offset to predicted value
        float predicted_value = tflite_instance_->output_buffer[i];
        model_out_log << predicted_value << " ";
        predicted_value += CalculateOffset(common_instance_->offset_thresholds,
                                           common_instance_->offset_values, predicted_value);
        predict_log << predicted_value << " ";
        data.emplace_back(predicted_value);
    }
    LOG(INFO) << sensor_name << ": model_output: [" << model_out_log.str() << "]";
    LOG(INFO) << sensor_name << ": predicted_value: [" << predict_log.str() << "]";
    *output = data;

    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::Estimate(const std::vector<float> &thermistors,
                                                 std::vector<float> *output) {
    if (type == kUseMLModel) {
        return TFliteEstimate(thermistors, output);
    } else if (type == kUseLinearModel) {
        return LinearModelEstimate(thermistors, output);
    }

    LOG(ERROR) << "Unsupported estimationType [" << type << "]";
    return kVtEstimatorUnSupported;
}

VtEstimatorStatus VirtualTempEstimator::TFliteGetMaxPredictWindowMs(size_t *predict_window_ms) {
    if (tflite_instance_ == nullptr || common_instance_ == nullptr) {
        LOG(ERROR) << "tflite_instance_ or common_instance_ is nullptr for predict window\n";
        return kVtEstimatorInitFailed;
    }

    if (!common_instance_->is_initialized) {
        LOG(ERROR) << "tflite_instance_ not initialized for " << common_instance_->sensor_name;
        return kVtEstimatorInitFailed;
    }

    size_t window = tflite_instance_->predict_window_ms;
    if (window == 0) {
        return kVtEstimatorUnSupported;
    }
    *predict_window_ms = window;
    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::GetMaxPredictWindowMs(size_t *predict_window_ms) {
    if (type == kUseMLModel) {
        return TFliteGetMaxPredictWindowMs(predict_window_ms);
    }

    LOG(ERROR) << "Unsupported estimationType [" << type << "]";
    return kVtEstimatorUnSupported;
}

VtEstimatorStatus VirtualTempEstimator::TFlitePredictAfterTimeMs(const size_t time_ms,
                                                                 float *output) {
    if (tflite_instance_ == nullptr || common_instance_ == nullptr) {
        LOG(ERROR) << "tflite_instance_ or common_instance_ is nullptr for predict window\n";
        return kVtEstimatorInitFailed;
    }

    if (!common_instance_->is_initialized) {
        LOG(ERROR) << "tflite_instance_ not initialized for " << common_instance_->sensor_name;
        return kVtEstimatorInitFailed;
    }

    std::unique_lock<std::mutex> lock(tflite_instance_->tflite_methods.mutex);

    size_t window = tflite_instance_->predict_window_ms;
    auto sample_interval = tflite_instance_->sample_interval;
    auto last_update_time = tflite_instance_->last_update_time;
    auto request_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(boot_clock::now() -
                                                                                 last_update_time);
    // check for under sampling
    if ((common_instance_->cur_sample_count < common_instance_->prev_samples_order) &&
        !(tflite_instance_->support_under_sampling)) {
        LOG(INFO) << tflite_instance_->model_path
                  << " cannot provide prediction while under sampling";
        return kVtEstimatorUnderSampling;
    }

    // calculate requested time since last update
    request_time_ms = request_time_ms + std::chrono::milliseconds{time_ms};
    if (sample_interval.count() == 0 || window == 0 ||
        window < static_cast<size_t>(request_time_ms.count())) {
        LOG(INFO) << tflite_instance_->model_path << " cannot predict temperature after ("
                  << time_ms << " + " << request_time_ms.count() - time_ms
                  << ") ms since last update with sample interval [" << sample_interval.count()
                  << "] ms and predict window [" << window << "] ms";
        return kVtEstimatorUnSupported;
    }

    size_t request_step = request_time_ms / sample_interval;
    size_t output_label_count = tflite_instance_->output_label_count;
    float *output_buffer = tflite_instance_->output_buffer;
    float prediction;
    if (request_step == output_label_count - 1) {
        // request prediction is on the right boundary of the window
        prediction = output_buffer[output_label_count - 1];
    } else {
        float left = output_buffer[request_step], right = output_buffer[request_step + 1];
        prediction = left;
        if (left != right) {
            prediction += (request_time_ms - sample_interval * request_step) * (right - left) /
                          sample_interval;
        }
    }

    *output = prediction;

    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::PredictAfterTimeMs(const size_t time_ms, float *output) {
    if (type == kUseMLModel) {
        return TFlitePredictAfterTimeMs(time_ms, output);
    }

    LOG(ERROR) << "PredictAfterTimeMs not supported for type [" << type << "]";
    return kVtEstimatorUnSupported;
}

VtEstimatorStatus VirtualTempEstimator::TFliteGetAllPredictions(std::vector<float> *output) {
    if (tflite_instance_ == nullptr || common_instance_ == nullptr) {
        LOG(ERROR) << "tflite_instance_ or common_instance_ is nullptr for predict window\n";
        return kVtEstimatorInitFailed;
    }

    std::unique_lock<std::mutex> lock(tflite_instance_->tflite_methods.mutex);

    if (!common_instance_->is_initialized) {
        LOG(ERROR) << "tflite_instance_ not initialized for " << tflite_instance_->model_path;
        return kVtEstimatorInitFailed;
    }

    if (output == nullptr) {
        LOG(ERROR) << "output is nullptr";
        return kVtEstimatorInvalidArgs;
    }

    std::vector<float> tflite_output;
    size_t output_buffer_size = tflite_instance_->output_buffer_size;
    tflite_output.reserve(output_buffer_size);
    for (size_t i = 0; i < output_buffer_size; ++i) {
        tflite_output.emplace_back(tflite_instance_->output_buffer[i]);
    }
    *output = tflite_output;

    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::GetAllPredictions(std::vector<float> *output) {
    if (type == kUseMLModel) {
        return TFliteGetAllPredictions(output);
    }

    LOG(INFO) << "GetAllPredicts not supported by estimationType [" << type << "]";
    return kVtEstimatorUnSupported;
}

VtEstimatorStatus VirtualTempEstimator::TFLiteDumpStatus(std::string_view sensor_name,
                                                         std::ostringstream *dump_buf) {
    if (dump_buf == nullptr) {
        LOG(ERROR) << "dump_buf is nullptr for " << sensor_name;
        return kVtEstimatorInvalidArgs;
    }

    if (!common_instance_->is_initialized) {
        LOG(ERROR) << "tflite_instance_ not initialized for " << tflite_instance_->model_path;
        return kVtEstimatorInitFailed;
    }

    std::unique_lock<std::mutex> lock(tflite_instance_->tflite_methods.mutex);

    *dump_buf << " Sensor Name: " << sensor_name << std::endl;
    *dump_buf << "  Current Values: ";
    size_t output_buffer_size = tflite_instance_->output_buffer_size;
    for (size_t i = 0; i < output_buffer_size; ++i) {
        // add offset to predicted value
        float predicted_value = tflite_instance_->output_buffer[i];
        predicted_value += CalculateOffset(common_instance_->offset_thresholds,
                                           common_instance_->offset_values, predicted_value);
        *dump_buf << predicted_value << ", ";
    }
    *dump_buf << std::endl;

    *dump_buf << "  Model Path: \"" << tflite_instance_->model_path << "\"" << std::endl;

    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::DumpStatus(std::string_view sensor_name,
                                                   std::ostringstream *dump_buff) {
    if (type == kUseMLModel) {
        return TFLiteDumpStatus(sensor_name, dump_buff);
    }

    LOG(INFO) << "DumpStatus not supported by estimationType [" << type << "]";
    return kVtEstimatorUnSupported;
}

VtEstimatorStatus VirtualTempEstimator::Initialize(const VtEstimationInitData &data) {
    LOG(INFO) << "Initialize VirtualTempEstimator for " << type;

    if (type == kUseMLModel) {
        return TFliteInitialize(data.ml_model_init_data);
    } else if (type == kUseLinearModel) {
        return LinearModelInitialize(data.linear_model_init_data);
    }

    LOG(ERROR) << "Unsupported estimationType [" << type << "]";
    return kVtEstimatorUnSupported;
}

bool VirtualTempEstimator::ParseInputConfig(const Json::Value &input_config) {
    if (!input_config["ModelConfig"].empty()) {
        if (!input_config["ModelConfig"]["sample_interval_ms"].empty()) {
            // read input sample interval
            int sample_interval_ms = input_config["ModelConfig"]["sample_interval_ms"].asInt();
            if (sample_interval_ms <= 0) {
                LOG(ERROR) << "Invalid sample_interval_ms: " << sample_interval_ms;
                return false;
            }

            tflite_instance_->sample_interval = std::chrono::milliseconds{sample_interval_ms};
            LOG(INFO) << "Parsed tflite model input sample_interval: " << sample_interval_ms
                      << " for " << common_instance_->sensor_name;

            // determine predict window
            tflite_instance_->predict_window_ms =
                    sample_interval_ms * (tflite_instance_->output_label_count - 1);
            LOG(INFO) << "Max prediction window size: " << tflite_instance_->predict_window_ms
                      << " ms for " << common_instance_->sensor_name;
        }

        if (!input_config["ModelConfig"]["max_sample_interval_ms"].empty()) {
            // read input max sample interval
            int max_sample_interval_ms =
                    input_config["ModelConfig"]["max_sample_interval_ms"].asInt();
            if (max_sample_interval_ms <= 0) {
                LOG(ERROR) << "Invalid max_sample_interval_ms " << max_sample_interval_ms;
                return false;
            }

            tflite_instance_->max_sample_interval =
                    std::chrono::milliseconds{max_sample_interval_ms};
            LOG(INFO) << "Parsed tflite model max_sample_interval: " << max_sample_interval_ms
                      << " for " << common_instance_->sensor_name;
        }
    }

    if (!input_config["InputData"].empty()) {
        Json::Value input_data = input_config["InputData"];
        if (input_data.size() != common_instance_->num_linked_sensors) {
            LOG(ERROR) << "Input ranges size: " << input_data.size()
                       << " does not match num_linked_sensors: "
                       << common_instance_->num_linked_sensors;
            return false;
        }

        LOG(INFO) << "Start to parse tflite model input config for "
                  << common_instance_->num_linked_sensors;
        tflite_instance_->input_range.assign(input_data.size(), InputRangeInfo());
        for (Json::Value::ArrayIndex i = 0; i < input_data.size(); ++i) {
            const std::string &name = input_data[i]["Name"].asString();
            LOG(INFO) << "Sensor[" << i << "] Name: " << name;
            if (!getInputRangeInfoFromJsonValues(input_data[i]["Range"],
                                                 &tflite_instance_->input_range[i])) {
                LOG(ERROR) << "Failed to parse tflite model temp range for sensor: [" << name
                           << "]";
                return false;
            }
        }
    }

    return true;
}

bool VirtualTempEstimator::GetInputConfig(Json::Value *config) {
    int config_size = 0;
    int ret = tflite_instance_->tflite_methods.get_input_config_size(
            tflite_instance_->tflite_wrapper, &config_size);
    if (ret || config_size <= 0) {
        LOG(ERROR) << "Failed to get tflite input config size (ret: " << ret
                   << ") with size: " << config_size;
        return false;
    }

    LOG(INFO) << "Model input config_size: " << config_size << " for "
              << common_instance_->sensor_name;

    char *config_str = new char[config_size];
    ret = tflite_instance_->tflite_methods.get_input_config(tflite_instance_->tflite_wrapper,
                                                            config_str, config_size);
    if (ret) {
        LOG(ERROR) << "Failed to get tflite input config (ret: " << ret << ")";
        delete[] config_str;
        return false;
    }

    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    std::string errorMessage;

    bool success = true;
    if (!reader->parse(config_str, config_str + config_size, config, &errorMessage)) {
        LOG(ERROR) << "Failed to parse tflite JSON input config: " << errorMessage;
        success = false;
    }
    delete[] config_str;
    return success;
}

}  // namespace vtestimator
}  // namespace thermal
