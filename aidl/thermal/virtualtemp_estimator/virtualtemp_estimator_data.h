/*
 * Copyright (C) 2023 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <android-base/chrono_utils.h>

#include <cstddef>
#include <mutex>
#include <string>

#pragma once

namespace thermal {
namespace vtestimator {

using android::base::boot_clock;

// Current version only supports single input/output tensors
constexpr int kNumInputTensors = 1;
constexpr int kNumOutputTensors = 1;

typedef void *(*tflitewrapper_create)(int num_input_tensors, int num_output_tensors);
typedef bool (*tflitewrapper_init)(void *handle, const char *model_path);
typedef bool (*tflitewrapper_invoke)(void *handle, float *input_samples, int num_input_samples,
                                     float *output_samples, int num_output_samples);
typedef void (*tflitewrapper_destroy)(void *handle);
typedef bool (*tflitewrapper_get_input_config_size)(void *handle, int *config_size);
typedef bool (*tflitewrapper_get_input_config)(void *handle, char *config_buffer,
                                               int config_buffer_size);

struct TFLiteWrapperMethods {
    tflitewrapper_create create;
    tflitewrapper_init init;
    tflitewrapper_invoke invoke;
    tflitewrapper_destroy destroy;
    tflitewrapper_get_input_config_size get_input_config_size;
    tflitewrapper_get_input_config get_input_config;
    mutable std::mutex mutex;
};

struct InputRangeInfo {
    float max_threshold = std::numeric_limits<float>::max();
    float min_threshold = std::numeric_limits<float>::min();
};

struct VtEstimatorCommonData {
    VtEstimatorCommonData(std::string_view name, size_t num_input_sensors) {
        sensor_name = name;
        num_linked_sensors = num_input_sensors;
        prev_samples_order = 1;
        is_initialized = false;
        use_prev_samples = false;
        cur_sample_count = 0;
    }
    std::string sensor_name;

    std::vector<float> offset_thresholds;
    std::vector<float> offset_values;

    size_t num_linked_sensors;
    size_t prev_samples_order;
    size_t cur_sample_count;
    bool use_prev_samples;
    bool is_initialized;
};

struct VtEstimatorTFLiteData {
    VtEstimatorTFLiteData() {
        scratch_buffer = nullptr;
        input_buffer = nullptr;
        input_buffer_size = 0;
        output_label_count = 1;
        num_hot_spots = 1;
        output_buffer = nullptr;
        output_buffer_size = 1;
        support_under_sampling = false;
        sample_interval = std::chrono::milliseconds{0};
        max_sample_interval = std::chrono::milliseconds{std::numeric_limits<int>::max()};
        predict_window_ms = 0;
        last_update_time = boot_clock::time_point::min();
        prev_sample_time = boot_clock::time_point::min();
        enable_input_validation = false;

        tflite_wrapper = nullptr;
        tflite_methods.create = nullptr;
        tflite_methods.init = nullptr;
        tflite_methods.get_input_config_size = nullptr;
        tflite_methods.get_input_config = nullptr;
        tflite_methods.invoke = nullptr;
        tflite_methods.destroy = nullptr;
    }

    void *tflite_wrapper;
    float *scratch_buffer;
    float *input_buffer;
    size_t input_buffer_size;
    size_t num_hot_spots;
    size_t output_label_count;
    float *output_buffer;
    size_t output_buffer_size;
    std::string model_path;
    TFLiteWrapperMethods tflite_methods;
    std::vector<InputRangeInfo> input_range;
    bool support_under_sampling;
    std::chrono::milliseconds sample_interval{};
    std::chrono::milliseconds max_sample_interval{};
    size_t predict_window_ms;
    boot_clock::time_point last_update_time;
    boot_clock::time_point prev_sample_time;
    bool enable_input_validation;

    ~VtEstimatorTFLiteData() {
        if (tflite_wrapper && tflite_methods.destroy) {
            tflite_methods.destroy(tflite_wrapper);
        }

        if (scratch_buffer) {
            delete scratch_buffer;
        }

        if (input_buffer) {
            delete input_buffer;
        }

        if (output_buffer) {
            delete output_buffer;
        }
    }
};

struct VtEstimatorLinearModelData {
    VtEstimatorLinearModelData() {}

    ~VtEstimatorLinearModelData() {}

    std::vector<std::vector<float>> input_samples;
    std::vector<std::vector<float>> coefficients;
    mutable std::mutex mutex;
};

}  // namespace vtestimator
}  // namespace thermal
