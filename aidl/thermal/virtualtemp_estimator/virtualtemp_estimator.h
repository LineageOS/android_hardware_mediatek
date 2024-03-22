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

// Class to estimate virtual temperature based on a model
class VirtualTempEstimator {
  public:
    // Implicit copy-move headers.
    VirtualTempEstimator(const VirtualTempEstimator &) = delete;
    VirtualTempEstimator(VirtualTempEstimator &&) = default;
    VirtualTempEstimator &operator=(const VirtualTempEstimator &) = delete;
    VirtualTempEstimator &operator=(VirtualTempEstimator &&) = default;

    VirtualTempEstimator(size_t num_input_samples);
    ~VirtualTempEstimator();

    // Initializes the model provided by model_path.
    VtEstimatorStatus Initialize(const char *model_path);

    // Performs the inference on the loaded VT model.
    // Output of the inference is returned in output argument
    VtEstimatorStatus Estimate(const std::vector<float> &thermistors, float *output);

  private:
    void LoadTFLiteWrapper();
    std::unique_ptr<VirtualTempEstimatorTFLiteData> data_;
};

}  // namespace vtestimator
}  // namespace thermal
