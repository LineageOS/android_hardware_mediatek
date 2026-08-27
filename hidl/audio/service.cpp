/*
 * SPDX-FileCopyrightText: The Android Open Source Project
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "mtkaudiohalservice"

#include <signal.h>
#include <string>
#include <vector>

#include <android-base/logging.h>
#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <binder/ProcessState.h>
#include <cutils/properties.h>
#include <dlfcn.h>
#include <hidl/HidlTransportSupport.h>
#include <hidl/LegacySupport.h>
#include <hwbinder/ProcessState.h>

#include <aidl/android/hardware/soundtrigger3/BnSoundTriggerHw.h>
#include <aidl/vendor/mediatek/hardware/audio/BnMtkAudio.h>

using namespace android::hardware;
using android::OK;

using InterfacesList = std::vector<std::string>;
using InterfaceFn = void (*)(void*);

using ::aidl::android::hardware::soundtrigger3::BnSoundTriggerHw;
using ::aidl::android::hardware::soundtrigger3::ISoundTriggerHw;
using ::aidl::vendor::mediatek::hardware::audio::BnMtkAudio;
using ::aidl::vendor::mediatek::hardware::audio::IMtkAudio;

/** Try to register the provided factories in the provided order.
 *  If any registers successfully, do not register any other and return true.
 *  If all fail, return false.
 */
template <class Iter>
static bool registerPassthroughServiceImplementations(Iter first, Iter last) {
    for (; first != last; ++first) {
        if (registerPassthroughServiceImplementation(*first) == OK) {
            return true;
        }
    }
    return false;
}

static bool registerExternalServiceImplementation(const std::string& libName,
                                                  const std::string& funcName) {
    constexpr int dlMode = RTLD_LAZY;
    void* handle = nullptr;
    dlerror();  // clear
    auto libPath = libName + ".so";
    handle = dlopen(libPath.c_str(), dlMode);
    if (handle == nullptr) {
        const char* error = dlerror();
        ALOGE("Failed to dlopen %s: %s", libPath.c_str(),
              error != nullptr ? error : "unknown error");
        return false;
    }
    binder_status_t (*factoryFunction)();
    *(void**)(&factoryFunction) = dlsym(handle, funcName.c_str());
    if (!factoryFunction) {
        const char* error = dlerror();
        ALOGE("Factory function %s not found in libName %s: %s", funcName.c_str(), libPath.c_str(),
              error != nullptr ? error : "unknown error");
        dlclose(handle);
        return false;
    }
    return ((*factoryFunction)() == STATUS_OK);
}

template <typename T>
static std::shared_ptr<T> loadVendorAidlImpl(const char* libPath, const char* ctorSymb,
                                             const char* dtorSymb) {
    void* handle = dlopen(libPath, RTLD_NOW | RTLD_GLOBAL);
    if (!handle) {
        LOG(ERROR) << "Failed to dlopen " << libPath;
        return nullptr;
    }

    auto ctor = reinterpret_cast<InterfaceFn>(dlsym(handle, ctorSymb));
    if (!ctor) {
        LOG(ERROR) << "Failed to find ctor symbol in " << libPath;
        dlclose(handle);
        return nullptr;
    }

    auto dtor = reinterpret_cast<InterfaceFn>(dlsym(handle, dtorSymb));
    if (!dtor) {
        LOG(ERROR) << "Failed to find dtor symbol in " << libPath;
        dlclose(handle);
        return nullptr;
    }

    void* mem = aligned_alloc(alignof(std::max_align_t), 4096);
    ctor(mem);

    return std::shared_ptr<T>(reinterpret_cast<T*>(mem), [handle, dtor](T* p) {
        dtor(p);
        free(p);
        dlclose(handle);
    });
}

int main(int /* argc */, char* /* argv */[]) {
    signal(SIGPIPE, SIG_IGN);

    if (::android::ProcessState::isVndservicemanagerEnabled()) {
        ::android::ProcessState::initWithDriver("/dev/vndbinder");
        ::android::ProcessState::self()->startThreadPool();
    }

    ABinderProcess_setThreadPoolMaxThreadCount(1);
    ABinderProcess_startThreadPool();

    const int32_t defaultValue = -1;
    int32_t value =
            property_get_int32("persist.vendor.audio.service.hwbinder.size_kbyte", defaultValue);
    if (value != defaultValue) {
        ALOGD("Configuring hwbinder with mmap size %d KBytes", value);
        ProcessState::initWithMmapSize(static_cast<size_t>(value) * 1024);
    }
    configureRpcThreadpool(16, true /*callerWillJoin*/);

    // Automatic formatting tries to compact the lines, making them less readable
    // clang-format off
    const std::vector<InterfacesList> mandatoryInterfaces = {
        {
            "Audio Core API",
            "android.hardware.audio@7.1::IDevicesFactory",
        },
        {
            "Audio Effect API",
            "android.hardware.audio.effect@7.0::IEffectsFactory",
        }
    };

    const std::vector<std::pair<std::string,std::string>> optionalInterfaceSharedLibs = {
        {
            "android.hardware.bluetooth.audio-impl-mediatek",
            "createIBluetoothAudioProviderFactory",
        },
    };
    // clang-format on

    for (const auto& listIter : mandatoryInterfaces) {
        auto iter = listIter.begin();
        const std::string& interfaceFamilyName = *iter++;
        LOG_ALWAYS_FATAL_IF(!registerPassthroughServiceImplementations(iter, listIter.end()),
                            "Could not register %s", interfaceFamilyName.c_str());
    }

    for (const auto& interfacePair : optionalInterfaceSharedLibs) {
        const std::string& libraryName = interfacePair.first;
        const std::string& interfaceLoaderFuncName = interfacePair.second;
        if (registerExternalServiceImplementation(libraryName, interfaceLoaderFuncName)) {
            ALOGI("%s() from %s success", interfaceLoaderFuncName.c_str(), libraryName.c_str());
        } else {
            ALOGW("%s() from %s failed", interfaceLoaderFuncName.c_str(), libraryName.c_str());
        }
    }

    auto mtkSoundTriggerHw = loadVendorAidlImpl<BnSoundTriggerHw>(
            "/vendor/lib64/hw/android.hardware.soundtrigger3-impl.so",
            "_ZN4aidl7android8hardware13soundtrigger314SoundTriggerHwC1Ev",
            "_ZN4aidl7android8hardware13soundtrigger314SoundTriggerHwD1Ev");
    const std::string soundTriggerHw_instance =
            std::string() + ISoundTriggerHw::descriptor + "/default";
    binder_status_t soundTriggerHw_status = AServiceManager_addService(
            mtkSoundTriggerHw->asBinder().get(), soundTriggerHw_instance.c_str());
    CHECK_EQ(soundTriggerHw_status, STATUS_OK);

    auto mtkAudio = loadVendorAidlImpl<BnMtkAudio>(
            "/vendor/lib64/hw/vendor.mediatek.hardware.audio-impl.so",
            "_ZN4aidl6vendor8mediatek8hardware5audio8MtkAudioC1Ev",
            "_ZN4aidl6vendor8mediatek8hardware5audio8MtkAudioD1Ev");
    const std::string instance = std::string() + IMtkAudio::descriptor + "/default";
    binder_status_t mtkAudio_status =
            AServiceManager_addService(mtkAudio->asBinder().get(), instance.c_str());
    CHECK_EQ(mtkAudio_status, STATUS_OK);

    joinRpcThreadpool();
}
