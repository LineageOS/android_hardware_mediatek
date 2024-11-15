/*
 * Copyright (C) 2023 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <json/value.h>

#include <sstream>
#include <vector>

#include "virtualtemp_estimator_data.h"

namespace thermal {
namespace vtestimator {

enum VtEstimatorStatus {
    kVtEstimatorOk = 0,
    kVtEstimatorInvalidArgs = 1,
    kVtEstimatorInitFailed = 2,
    kVtEstimatorInvokeFailed = 3,
    kVtEstimatorUnSupported = 4,
    kVtEstimatorLowConfidence = 5,
    kVtEstimatorUnderSampling = 6,
};

enum VtEstimationType { kUseMLModel = 0, kUseLinearModel = 1, kInvalidEstimationType = 2 };

struct MLModelInitData {
    std::string model_path;
    bool use_prev_samples;
    size_t prev_samples_order;
    size_t output_label_count;
    size_t num_hot_spots;
    bool enable_input_validation;
    std::vector<float> offset_thresholds;
    std::vector<float> offset_values;
    bool support_under_sampling;
};

struct LinearModelInitData {
    bool use_prev_samples;
    size_t prev_samples_order;
    std::vector<float> coefficients;
    std::vector<float> offset_thresholds;
    std::vector<float> offset_values;
};

union VtEstimationInitData {
    VtEstimationInitData(VtEstimationType type) {
        if (type == kUseMLModel) {
            ml_model_init_data.model_path = "";
            ml_model_init_data.use_prev_samples = false;
            ml_model_init_data.prev_samples_order = 1;
            ml_model_init_data.output_label_count = 1;
            ml_model_init_data.num_hot_spots = 1;
            ml_model_init_data.enable_input_validation = false;
            ml_model_init_data.support_under_sampling = false;
        } else if (type == kUseLinearModel) {
            linear_model_init_data.use_prev_samples = false;
            linear_model_init_data.prev_samples_order = 1;
        }
    }
    ~VtEstimationInitData() {}

    MLModelInitData ml_model_init_data;
    LinearModelInitData linear_model_init_data;
};

// Class to estimate virtual temperature
class VirtualTempEstimator {
  public:
    // Implicit copy-move headers.
    VirtualTempEstimator(const VirtualTempEstimator &) = delete;
    VirtualTempEstimator(VirtualTempEstimator &&) = default;
    VirtualTempEstimator &operator=(const VirtualTempEstimator &) = delete;
    VirtualTempEstimator &operator=(VirtualTempEstimator &&) = default;

    VirtualTempEstimator(std::string_view sensor_name, VtEstimationType type,
                         size_t num_linked_sensors);
    ~VirtualTempEstimator();

    // Initializes the estimator based on init_data
    VtEstimatorStatus Initialize(const VtEstimationInitData &init_data);

    // Performs the prediction and returns estimated value in output
    VtEstimatorStatus Estimate(const std::vector<float> &thermistors, std::vector<float> *output);

    // Dump estimator status
    VtEstimatorStatus DumpStatus(std::string_view sensor_name, std::ostringstream *dump_buf);
    // Get predict window width in milliseconds
    VtEstimatorStatus GetMaxPredictWindowMs(size_t *predict_window_ms);
    // Predict temperature after desired milliseconds
    VtEstimatorStatus PredictAfterTimeMs(const size_t time_ms, float *output);
    // Get entire output buffer of the estimator
    VtEstimatorStatus GetAllPredictions(std::vector<float> *output);

    // Adds traces to help debug
    VtEstimatorStatus DumpTraces();

  private:
    void LoadTFLiteWrapper();
    VtEstimationType type;
    std::unique_ptr<VtEstimatorCommonData> common_instance_;
    std::unique_ptr<VtEstimatorTFLiteData> tflite_instance_;
    std::unique_ptr<VtEstimatorLinearModelData> linear_model_instance_;

    VtEstimatorStatus LinearModelInitialize(LinearModelInitData data);
    VtEstimatorStatus TFliteInitialize(MLModelInitData data);

    VtEstimatorStatus LinearModelEstimate(const std::vector<float> &thermistors,
                                          std::vector<float> *output);
    VtEstimatorStatus TFliteEstimate(const std::vector<float> &thermistors,
                                     std::vector<float> *output);
    VtEstimatorStatus TFliteGetMaxPredictWindowMs(size_t *predict_window_ms);
    VtEstimatorStatus TFlitePredictAfterTimeMs(const size_t time_ms, float *output);
    VtEstimatorStatus TFliteGetAllPredictions(std::vector<float> *output);

    VtEstimatorStatus TFLiteDumpStatus(std::string_view sensor_name, std::ostringstream *dump_buf);
    bool GetInputConfig(Json::Value *config);
    bool ParseInputConfig(const Json::Value &config);
};

}  // namespace vtestimator
}  // namespace thermal
