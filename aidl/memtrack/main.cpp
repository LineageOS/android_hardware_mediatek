/*
 * SPDX-FileCopyrightText: 2020 The Android Open Source Project
 * SPDX-FileCopyrightText: 2025-2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Memtrack.h"

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>

using aidl::android::hardware::memtrack::Memtrack;

int main() {
    ABinderProcess_setThreadPoolMaxThreadCount(0);
    std::shared_ptr<Memtrack> memtrack = ndk::SharedRefBase::make<Memtrack>();

    const std::string instance = std::string() + Memtrack::descriptor + "/default";
    binder_status_t status =
            AServiceManager_addService(memtrack->asBinder().get(), instance.c_str());
    CHECK(status == STATUS_OK);

    ABinderProcess_joinThreadPool();
    return EXIT_FAILURE;  // Unreachable
}
