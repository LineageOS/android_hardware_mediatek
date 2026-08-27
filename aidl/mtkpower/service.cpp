/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vendor.mediatek.hardware.mtkpower-service.stub"

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>

#include "MtkPowerService.h"

using ::aidl::vendor::mediatek::hardware::mtkpower::IMtkPowerService;

int main() {
    ABinderProcess_setThreadPoolMaxThreadCount(0);
    std::shared_ptr<IMtkPowerService> mtkPowerService =
            ndk::SharedRefBase::make<IMtkPowerService>();

    const std::string instance = std::string() + IMtkPowerService::descriptor + "/default";
    binder_status_t status =
            AServiceManager_addService(mtkPowerService->asBinder().get(), instance.c_str());
    CHECK_EQ(status, STATUS_OK);

    ABinderProcess_joinThreadPool();
    return EXIT_FAILURE;  // // should not reach
}
