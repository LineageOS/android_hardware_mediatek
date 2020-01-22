/*
 * Copyright (C) 2019 The LineageOS Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifdef LIVES_IN_SYSTEM
#define LOG_TAG "lineage.livedisplay@2.0-service-mediatek"
#else
#define LOG_TAG "vendor.lineage.livedisplay@2.0-service-mediatek"
#endif

#include <android-base/logging.h>
#include <binder/ProcessState.h>
#include <hidl/HidlTransportSupport.h>

#include "DisplayColorCalibration.h"

using android::OK;
using android::sp;
using android::status_t;
using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;

using ::vendor::lineage::livedisplay::V2_0::IDisplayColorCalibration;
using ::vendor::lineage::livedisplay::V2_0::mediatek::DisplayColorCalibration;

int main() {
    // HALs
    sp<DisplayColorCalibration> dcc;

    status_t status = OK;

    LOG(INFO) << "LiveDisplay HAL service is starting.";

    dcc = new DisplayColorCalibration();
    if (dcc == nullptr) {
        LOG(ERROR) << "Can not create an instance of LiveDisplay HAL DisplayColorCalibration Iface,"
                   << " exiting.";
        goto shutdown;
    }

    configureRpcThreadpool(1, true /*callerWillJoin*/);

    if (dcc->isSupported()) {
        status = dcc->registerAsService();
        if (status != OK) {
            LOG(ERROR) << "Could not register service for LiveDisplay HAL DisplayColorCalibration"
                       << " Iface (" << status << ")";
            goto shutdown;
        }
    }

    LOG(INFO) << "LiveDisplay HAL service is ready.";
    joinRpcThreadpool();
    // Should not pass this line

shutdown:
    // In normal operation, we don't expect the thread pool to shutdown
    LOG(ERROR) << "LiveDisplay HAL service is shutting down.";
    return 1;
}
