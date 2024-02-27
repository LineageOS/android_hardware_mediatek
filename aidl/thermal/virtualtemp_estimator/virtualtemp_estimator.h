/*
 * Copyright (C) 2023 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

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
};

enum VtEstimationType { kUseMLModel = 0, kUseLinearModel = 1, kInvalidEstimationType = 2 };

struct MLModelInitData {
    std::string model_path;
    bool use_prev_samples;
    size_t prev_samples_order;
    float offset;
    size_t output_label_count;
    size_t num_hot_spots;
};

struct LinearModelInitData {
    bool use_prev_samples;
    size_t prev_samples_order;
    std::vector<float> coefficients;
    float offset;
};

union VtEstimationInitData {
    VtEstimationInitData(VtEstimationType type) {
        if (type == kUseMLModel) {
            ml_model_init_data.model_path = "";
            ml_model_init_data.use_prev_samples = false;
            ml_model_init_data.prev_samples_order = 1;
            ml_model_init_data.offset = 0;
            ml_model_init_data.output_label_count = 1;
            ml_model_init_data.num_hot_spots = 1;
        } else if (type == kUseLinearModel) {
            linear_model_init_data.use_prev_samples = false;
            linear_model_init_data.prev_samples_order = 1;
            linear_model_init_data.offset = 0;
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

    VirtualTempEstimator(VtEstimationType type, size_t num_linked_sensors);
    ~VirtualTempEstimator();

    // Initializes the estimator based on init_data
    VtEstimatorStatus Initialize(const VtEstimationInitData &init_data);

    // Performs the prediction and returns estimated value in output
    VtEstimatorStatus Estimate(const std::vector<float> &thermistors, float *output);

  private:
    void LoadTFLiteWrapper();
    VtEstimationType type;
    std::unique_ptr<VtEstimatorCommonData> common_instance_;
    std::unique_ptr<VtEstimatorTFLiteData> tflite_instance_;
    std::unique_ptr<VtEstimatorLinearModelData> linear_model_instance_;

    VtEstimatorStatus LinearModelInitialize(LinearModelInitData data);
    VtEstimatorStatus TFliteInitialize(MLModelInitData data);

    VtEstimatorStatus LinearModelEstimate(const std::vector<float> &thermistors, float *output);
    VtEstimatorStatus TFliteEstimate(const std::vector<float> &thermistors, float *output);
};

}  // namespace vtestimator
}  // namespace thermal
