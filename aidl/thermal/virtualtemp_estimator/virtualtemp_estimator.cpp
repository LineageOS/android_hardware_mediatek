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
    if (!data_) {
        LOG(ERROR) << "data_ is nullptr during LoadTFLiteWrapper";
        return;
    }

    std::unique_lock<std::mutex> lock(data_->tflite_methods.mutex_);

    void *mLibHandle = dlopen("/vendor/lib64/libthermal_tflite_wrapper.so", 0);
    if (mLibHandle == nullptr) {
        LOG(ERROR) << "Could not load libthermal_tflite_wrapper library with error: " << dlerror();
        return;
    }

    data_->tflite_methods.create =
            reinterpret_cast<tflitewrapper_create>(dlsym(mLibHandle, "Create"));
    if (!data_->tflite_methods.create) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_create with error: " << dlerror();
    }

    data_->tflite_methods.init = reinterpret_cast<tflitewrapper_init>(dlsym(mLibHandle, "Init"));
    if (!data_->tflite_methods.init) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_init with error: " << dlerror();
    }

    data_->tflite_methods.invoke =
            reinterpret_cast<tflitewrapper_invoke>(dlsym(mLibHandle, "Invoke"));
    if (!data_->tflite_methods.invoke) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_invoke with error: " << dlerror();
    }

    data_->tflite_methods.destroy =
            reinterpret_cast<tflitewrapper_destroy>(dlsym(mLibHandle, "Destroy"));
    if (!data_->tflite_methods.destroy) {
        LOG(ERROR) << "Could not link and cast tflitewrapper_destroy with error: " << dlerror();
    }
}

VirtualTempEstimator::VirtualTempEstimator(size_t num_input_samples) {
    data_ = std::make_unique<VirtualTempEstimatorTFLiteData>(num_input_samples);
    LoadTFLiteWrapper();
}

VirtualTempEstimator::~VirtualTempEstimator() {
    LOG(INFO) << "VirtualTempEstimator destructor";
}

VtEstimatorStatus VirtualTempEstimator::Initialize(const char *model_path) {
    LOG(INFO) << "Initialize VirtualTempEstimator\n";

    if (!data_) {
        LOG(ERROR) << "data_ is nullptr during Initialize\n";
        return kVtEstimatorInitFailed;
    }

    std::unique_lock<std::mutex> lock(data_->tflite_methods.mutex_);

    if (!model_path) {
        LOG(ERROR) << "Invalid model_path:" << model_path;
        return kVtEstimatorInvalidArgs;
    }

    if (!data_->input_buffer || !data_->input_buffer_size) {
        LOG(ERROR) << "Invalid data_ members " << model_path
                   << " input_buffer: " << data_->input_buffer
                   << " input_buffer_size: " << data_->input_buffer_size;
        return kVtEstimatorInitFailed;
    }

    if (!data_->tflite_methods.create || !data_->tflite_methods.init ||
        !data_->tflite_methods.invoke || !data_->tflite_methods.destroy) {
        LOG(ERROR) << "Invalid tflite methods";
        return kVtEstimatorInitFailed;
    }

    data_->tflite_wrapper = data_->tflite_methods.create(kNumInputTensors, kNumOutputTensors);
    if (!data_->tflite_wrapper) {
        LOG(ERROR) << "Failed to create tflite wrapper";
        return kVtEstimatorInitFailed;
    }

    int ret = data_->tflite_methods.init(data_->tflite_wrapper, model_path);
    if (ret) {
        LOG(ERROR) << "Failed to Init tflite_wrapper for " << model_path << " (ret: )" << ret
                   << ")";
        return kVtEstimatorInitFailed;
    }

    data_->is_initialized = true;
    data_->model_path = model_path;

    LOG(INFO) << "Successfully initialized VirtualTempEstimator for " << model_path;
    return kVtEstimatorOk;
}

VtEstimatorStatus VirtualTempEstimator::Estimate(const std::vector<float> &thermistors,
                                                 float *output) {
    if (!data_) {
        LOG(ERROR) << "data_ is nullptr during Estimate\n";
        return kVtEstimatorInitFailed;
    }

    std::unique_lock<std::mutex> lock(data_->tflite_methods.mutex_);

    if (!data_->is_initialized) {
        LOG(ERROR) << "data_ not initialized for " << data_->model_path;
        return kVtEstimatorInitFailed;
    }

    if ((thermistors.size() != data_->input_buffer_size) || (!output)) {
        LOG(ERROR) << "Invalid args for " << data_->model_path
                   << " thermistors.size(): " << thermistors.size()
                   << " input_buffer_size: " << data_->input_buffer_size << " output: " << output;
        return kVtEstimatorInvalidArgs;
    }

    // copy input data into input tensors
    for (size_t i = 0; i < data_->input_buffer_size; ++i) {
        data_->input_buffer[i] = thermistors[i];
    }

    int ret = data_->tflite_methods.invoke(data_->tflite_wrapper, data_->input_buffer,
                                           data_->input_buffer_size, output, 1);
    if (ret) {
        LOG(ERROR) << "Failed to Invoke for " << data_->model_path << " (ret: " << ret << ")";
        return kVtEstimatorInvokeFailed;
    }

    return kVtEstimatorOk;
}

}  // namespace vtestimator
}  // namespace thermal
