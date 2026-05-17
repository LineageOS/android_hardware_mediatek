//
// SPDX-FileCopyrightText: 2022 The Android Open Source Project
// SPDX-FileCopyrightText: The LineageOS Project
// SPDX-License-Identifier: Apache-2.0
//

#include "BootControl.h"

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>

using aidl::android::hardware::boot::BootControl;
using aidl::android::hardware::boot::IBootControl;

int main(int, char* argv[]) {
    android::base::InitLogging(argv, android::base::KernelLogger);
    ABinderProcess_setThreadPoolMaxThreadCount(0);
    std::shared_ptr<IBootControl> service = ndk::SharedRefBase::make<BootControl>();

    const std::string instance = std::string(BootControl::descriptor) + "/default";
    auto status = AServiceManager_addService(service->asBinder().get(), instance.c_str());
    CHECK_EQ(status, STATUS_OK) << "Failed to add service " << instance << " " << status;
    LOG(INFO) << "IBootControl AIDL service running...";

    ABinderProcess_joinThreadPool();
    return EXIT_FAILURE;  // should not reach
}
