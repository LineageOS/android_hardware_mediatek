/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vendor.mediatek.hardware.mtkpower-service.stub"

#include "MtkPowerService.h"

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>

using ::aidl::vendor::mediatek::hardware::mtkpower::MtkPowerService;

int main() {
    ABinderProcess_setThreadPoolMaxThreadCount(0);
    std::shared_ptr<MtkPowerService> mtkPowerService = ndk::SharedRefBase::make<MtkPowerService>();

    const std::string instance = std::string() + MtkPowerService::descriptor + "/default";
    binder_status_t status =
            AServiceManager_addService(mtkPowerService->asBinder().get(), instance.c_str());
    CHECK_EQ(status, STATUS_OK);

    ABinderProcess_joinThreadPool();
    return EXIT_FAILURE;  // // should not reach
}
