/*
 * Copyright (C) 2022 The LineageOS Project
 *
 * Licensed under the Apache License, Version 2.1 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.1
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "vendor.mediatek.hardware.mtkpower@1.2-service.stub"

#include <android-base/logging.h>
#include <hidl/HidlTransportSupport.h>

#include "MtkPower.h"
#include "MtkPerf.h"

using android::OK;
using android::sp;
using android::status_t;
using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;

using vendor::mediatek::hardware::mtkpower::V1_2::IMtkPower;
using vendor::mediatek::hardware::mtkpower::V1_2::IMtkPerf;

using vendor::mediatek::hardware::mtkpower::implementation::MtkPower;
using vendor::mediatek::hardware::mtkpower::implementation::MtkPerf;

int main() {
    configureRpcThreadpool(1, true /*callerWillJoin*/);

    sp<IMtkPower> mtkPower = new MtkPower();
    sp<IMtkPerf> mtkPerf = new MtkPerf();

    if (mtkPower->registerAsService() != android::OK) {
        LOG(ERROR) << "Can't register MtkPower Stub HAL service";
        return 1;
    }

    if (mtkPerf->registerAsService() != android::OK) {
        LOG(ERROR) << "Can't register MtkPerf Stub HAL service";
        return 1;
    }

    joinRpcThreadpool();

    return 0;  // should never get here
}
