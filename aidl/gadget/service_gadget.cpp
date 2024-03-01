/*
 * SPDX-FileCopyrightText: 2021 The Android Open Source Project
 * SPDX-FileCopyrightText: 2024 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "android.hardware.usb.gadget-service.mediatek"

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <utils/Log.h>
#include "UsbGadget.h"

using android::OK;
using android::sp;
using android::status_t;

using aidl::android::hardware::usb::gadget::UsbGadget;

int main() {
    ABinderProcess_setThreadPoolMaxThreadCount(0);
    std::shared_ptr<UsbGadget> usbgadget = ndk::SharedRefBase::make<UsbGadget>();

    const std::string instance = std::string() + UsbGadget::descriptor + "/default";
    binder_status_t status =
            AServiceManager_addService(usbgadget->asBinder().get(), instance.c_str());
    CHECK(status == STATUS_OK);

    ALOGV("AIDL USB Gadget HAL about to start");
    ABinderProcess_joinThreadPool();
    return -1;  // Should never be reached
}
