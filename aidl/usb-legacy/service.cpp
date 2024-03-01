/*
 * Copyright (C) 2021 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>

#include "Usb.h"

using ::aidl::android::hardware::usb::Usb;

int main() {
    ABinderProcess_setThreadPoolMaxThreadCount(0);
    std::shared_ptr<Usb> usb = ndk::SharedRefBase::make<Usb>();

    const std::string instance = std::string() + Usb::descriptor + "/default";
    binder_status_t status = AServiceManager_addService(usb->asBinder().get(), instance.c_str());
    CHECK(status == STATUS_OK);

    ABinderProcess_joinThreadPool();
    return -1;  // Should never be reached
}
