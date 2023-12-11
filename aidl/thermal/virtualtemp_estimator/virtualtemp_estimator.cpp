/*
 * Copyright (C) 2023 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "virtualtemp_estimator.h"

#include <android-base/logging.h>
#include <dlfcn.h>

#include <vector>

namespace thermal {
namespace vtestimator {

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
            reinterpret_cast<tflitewrapper_create>(dlsym(mLibHandle, "Create"));
    if (!tflite_instance_->tflite_methods.create) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_create with error: " << dlerror();
    }

    tflite_instance_->tflite_methods.init =
            reinterpret_cast<tflitewrapper_init>(dlsym(mLibHandle, "Init"));
    if (!tflite_instance_->tflite_methods.init) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_init with error: " << dlerror();
    }

    tflite_instance_->tflite_methods.invoke =
            reinterpret_cast<tflitewrapper_invoke>(dlsym(mLibHandle, "Invoke"));
    if (!tflite_instance_->tflite_methods.invoke) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_invoke with error: " << dlerror();
    }

    tflite_instance_->tflite_methods.destroy =
            reinterpret_cast<tflitewrapper_destroy>(dlsym(mLibHandle, "Destroy"));
    if (!tflite_instance_->tflite_methods.destroy) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_destroy with error: " << dlerror();
    }
}

VirtualTempEstimator::VirtualTempEstimator(VtEstimationType estimationType,
                                           size_t num_linked_sensors) {
    type = estimationType;

    if (estimationType == kUseMLModel) {
        tflite_instance_ = std::make_unique<VtEstimatorTFLiteData>(num_linked_sensors);
        LoadTFLiteWrapper();
    } else if (estimationType == kUseLinearModel) {
        linear_model_instance_ = std::make_unique<VtEstimatorLinearModelData>(num_linked_sensors);
    } else {
        LOG(ERROR) << "Unsupported estimationType [" << estimationType << "]";
    }
}

VirtualTempEstimator::~VirtualTempEstimator() {
    LOG(INFO) << "VirtualTempEstimator destructor";
}

VtEstimatorStatus VirtualTempEstimator::LinearModelInitialize(LinearModelInitData data) {
    if (linear_model_instance_ == nullptr) {
        LOG(ERROR) << "linear_model_instance_ is nullptr during Initialize";
        return kVtEstimatorInitFailed;
    }

    size_t num_linked_sensors = linear_model_instance_->num_linked_sensors;
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

    linear_model_instance_->use_prev_samples = data.use_prev_samples;
    linear_model_instance_->prev_samples_order = data.prev_samples_order;

    linear_model_instance_->input_samples.reserve(linear_model_instance_->prev_samples_order);
    linear_model_instance_->coefficients.reserve(linear_model_instance_->prev_samples_order);

    // Store coefficients
    for (size_t i = 0; i < data.prev_samples_order; ++i) {
        std::vector<float> single_order_coefficients;
        for (size_t j = 0; j < num_linked_sensors; ++j) {
            single_order_coefficients.emplace_back(data.coefficients[i * num_linked_sensors + j]);
        }
        linear_model_instance_->coefficients.emplace_back(single_order_coefficients);
    }

    linear_model_instance_->cur_sample_index = 0;
    linear_model_instance_->offset = data.offset;
    linear_model_instance_->is_initialized = true;

    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::TFliteInitialize(MLModelInitData data) {
    std::string model_path = data.model_path;

    if (!tflite_instance_) {
        LOG(ERROR) << "tflite_instance_ is nullptr during Initialize\n";
        return kVtEstimatorInitFailed;
    }

    std::unique_lock<std::mutex> lock(tflite_instance_->tflite_methods.mutex);

    if (model_path.empty()) {
        LOG(ERROR) << "Invalid model_path:" << model_path;
        return kVtEstimatorInvalidArgs;
    }

    if (!tflite_instance_->input_buffer || !tflite_instance_->input_buffer_size) {
        LOG(ERROR) << "Invalid tflite_instance_ members " << model_path
                   << " input_buffer: " << tflite_instance_->input_buffer
                   << " input_buffer_size: " << tflite_instance_->input_buffer_size;
        return kVtEstimatorInitFailed;
    }

    if (!tflite_instance_->tflite_methods.create || !tflite_instance_->tflite_methods.init ||
        !tflite_instance_->tflite_methods.invoke || !tflite_instance_->tflite_methods.destroy) {
        LOG(ERROR) << "Invalid tflite methods";
        return kVtEstimatorInitFailed;
    }

    tflite_instance_->tflite_wrapper =
            tflite_instance_->tflite_methods.create(kNumInputTensors, kNumOutputTensors);
    if (!tflite_instance_->tflite_wrapper) {
        LOG(ERROR) << "Failed to create tflite wrapper";
        return kVtEstimatorInitFailed;
    }

    int ret = tflite_instance_->tflite_methods.init(tflite_instance_->tflite_wrapper,
                                                    model_path.c_str());
    if (ret) {
        LOG(ERROR) << "Failed to Init tflite_wrapper for " << model_path << " (ret: )" << ret
                   << ")";
        return kVtEstimatorInitFailed;
    }

    tflite_instance_->is_initialized = true;
    tflite_instance_->model_path = model_path;

    LOG(INFO) << "Successfully initialized VirtualTempEstimator for " << model_path;
    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::LinearModelEstimate(const std::vector<float> &thermistors,
                                                            float *output) {
    if (linear_model_instance_ == nullptr) {
        LOG(ERROR) << "linear_model_instance_ is nullptr during Initialize";
        return kVtEstimatorInitFailed;
    }

    size_t prev_samples_order = linear_model_instance_->prev_samples_order;
    size_t num_linked_sensors = linear_model_instance_->num_linked_sensors;

    std::unique_lock<std::mutex> lock(linear_model_instance_->mutex);

    if ((thermistors.size() != num_linked_sensors) || (output == nullptr)) {
        LOG(ERROR) << "Invalid args Thermistors size[" << thermistors.size()
                   << "] num_linked_sensors[" << num_linked_sensors << "] output[" << output << "]";
        return kVtEstimatorInvalidArgs;
    }

    if (linear_model_instance_->is_initialized == false) {
        LOG(ERROR) << "VirtualTempEstimator not initialized to estimate";
        return kVtEstimatorInitFailed;
    }

    // For the first iteration copy current inputs to all previous inputs
    // This would allow the estimator to have previous samples from the first iteration itself
    // and provide a valid predicted value
    if (linear_model_instance_->first_iteration) {
        for (size_t i = 0; i < prev_samples_order; ++i) {
            linear_model_instance_->input_samples[i] = thermistors;
        }
        linear_model_instance_->first_iteration = false;
    }

    size_t cur_sample_index = linear_model_instance_->cur_sample_index;
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

    estimated_value += linear_model_instance_->offset;

    // Update sample index
    cur_sample_index++;
    cur_sample_index = (cur_sample_index % prev_samples_order);
    linear_model_instance_->cur_sample_index = cur_sample_index;

    *output = estimated_value;
    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::TFliteEstimate(const std::vector<float> &thermistors,
                                                       float *output) {
    if (tflite_instance_ == nullptr) {
        LOG(ERROR) << "tflite_instance_ is nullptr during Estimate\n";
        return kVtEstimatorInitFailed;
    }

    std::unique_lock<std::mutex> lock(tflite_instance_->tflite_methods.mutex);

    if (!tflite_instance_->is_initialized) {
        LOG(ERROR) << "tflite_instance_ not initialized for " << tflite_instance_->model_path;
        return kVtEstimatorInitFailed;
    }

    if ((thermistors.size() != tflite_instance_->input_buffer_size) || (!output)) {
        LOG(ERROR) << "Invalid args for " << tflite_instance_->model_path
                   << " thermistors.size(): " << thermistors.size()
                   << " input_buffer_size: " << tflite_instance_->input_buffer_size
                   << " output: " << output;
        return kVtEstimatorInvalidArgs;
    }

    // copy input data into input tensors
    for (size_t i = 0; i < tflite_instance_->input_buffer_size; ++i) {
        tflite_instance_->input_buffer[i] = thermistors[i];
    }

    int ret = tflite_instance_->tflite_methods.invoke(
            tflite_instance_->tflite_wrapper, tflite_instance_->input_buffer,
            tflite_instance_->input_buffer_size, output, 1);
    if (ret) {
        LOG(ERROR) << "Failed to Invoke for " << tflite_instance_->model_path << " (ret: " << ret
                   << ")";
        return kVtEstimatorInvokeFailed;
    }

    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::Estimate(const std::vector<float> &thermistors,
                                                 float *output) {
    if (type == kUseMLModel) {
        return TFliteEstimate(thermistors, output);
    } else if (type == kUseLinearModel) {
        return LinearModelEstimate(thermistors, output);
    }

    LOG(ERROR) << "Unsupported estimationType [" << type << "]";
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

}  // namespace vtestimator
}  // namespace thermal
