/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include <aidl/vendor/mediatek/hardware/mtkpower/IMtkPowerService.h>
#include <android/binder_manager.h>

#include <mutex>

using aidl::vendor::mediatek::hardware::mtkpower::IMtkPowerService;

static const std::string kInstance = std::string(IMtkPowerService::descriptor) + "/default";

static std::mutex gLock;
static std::shared_ptr<IMtkPowerService> gService;

static std::shared_ptr<IMtkPowerService> getService() {
    static const bool declared = AServiceManager_isDeclared(kInstance.c_str());
    if (!declared) {
        return nullptr;
    }

    std::lock_guard<std::mutex> lock(gLock);
    if (!gService) {
        gService = IMtkPowerService::fromBinder(
                ndk::SpAIBinder(AServiceManager_checkService(kInstance.c_str())));
    }
    return gService;
}

static void checkStatus(const ndk::ScopedAStatus& status) {
    if (status.getStatus() == STATUS_DEAD_OBJECT) {
        std::lock_guard<std::mutex> lock(gLock);
        gService = nullptr;
    }
}

extern "C" int PowerHal_Wrap_mtkPowerHint(int hint, int data) {
    if (auto service = getService()) {
        checkStatus(service->mtkPowerHint(hint, data));
    }
    return 0;
}

extern "C" int PowerHal_Wrap_mtkCusPowerHint(int hint, int data) {
    if (auto service = getService()) {
        checkStatus(service->mtkCusPowerHint(hint, data));
    }
    return 0;
}
