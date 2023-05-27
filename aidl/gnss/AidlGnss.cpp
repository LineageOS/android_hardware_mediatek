/*
 * Copyright (C) 2020 The Android Open Source Project
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

#define LOG_TAG "AidlGnss"

#include "AidlGnss.h"
#include <log/log.h>
#include "AidlGnssConfiguration.h"
#include "AidlGnssMeasurement.h"
#include "AidlGnssPowerIndication.h"
#include "AidlGnssPsds.h"

namespace aidl::android::hardware::gnss {

// 12.00  Init version for Android S GNSS AIDL impl library
// 12.01  Fix GnssMeasurement report and close caused semaphore lock issue
#define AIDL_SW_VERSION  "12.01"

std::shared_ptr<IGnssCallback> AidlGnss::sGnssCallback = nullptr;
sem_t AidlGnss::sSem;
const char* AidlGnss::sVersion = AIDL_SW_VERSION;

AidlGnssCallbacks_ext AidlGnss::sAidlGnssCb = {
    .size = sizeof(AidlGnssCallbacks_ext),
    .set_capabilities_cb = gnssCapabilitiesCb,
};

AidlGnss::AidlGnss() {
    ALOGE("[%s] %s: AidlGnss constructor", AIDL_SW_VERSION, __func__);

    hw_module_t* module;
    IGnss* iface = nullptr;
    gps_device_t_ext* gnssDevice = nullptr;
    int err = hw_get_module(GPS_HARDWARE_MODULE_ID, (hw_module_t const**)&module);
    mGnssHalIface = nullptr;
    mAidlGnssHalIface = nullptr;

    if (err == 0) {
        hw_device_t* device;
        err = module->methods->open(module, GPS_HARDWARE_MODULE_ID, &device);
        if (err == 0) {
            gps_device_t_ext* gnssDevice = reinterpret_cast<gps_device_t_ext*>(device);
            mGnssHalIface = gnssDevice->get_gps_interface(gnssDevice);
            ALOGE("[%s] %s: Successfully get GPS HAL iface", AIDL_SW_VERSION, __func__);

            mAidlGnssHalIface = static_cast<const AidlGnssInterface*>(
                    mGnssHalIface->get_extension(AIDL_GNSS_INTERFACE));

            if (mAidlGnssHalIface == nullptr) {
                ALOGE("[%s] %s: Aidl Gnss interface is unavailable from GPS HAL",
                        AIDL_SW_VERSION, __func__);
            } else {
                ALOGE("[%s] %s: Successfully get Aidl Gnss interface from GPS HAL",
                        AIDL_SW_VERSION, __func__);
            }
        } else {
            ALOGE("[%s] gnssDevice open %s failed: %d",
                    AIDL_SW_VERSION, GPS_HARDWARE_MODULE_ID, err);
        }
    } else {
        ALOGE("gnss hw_get_module %s failed: %d", GPS_HARDWARE_MODULE_ID, err);
    }

    sem_init(&sSem, 0, 1);
}

AidlGnss::~AidlGnss() {
    sem_destroy(&sSem);
}


ndk::ScopedAStatus AidlGnss::setCallback(const std::shared_ptr<IGnssCallback>& callback) {
    bool ret = false;

    ALOGD("[%s] AidlGnss::setCallback", AIDL_SW_VERSION);
    if (mAidlGnssHalIface == nullptr) {
        ALOGE("[%s] %s: HAL Aidl Gnss interface is unavailable", AIDL_SW_VERSION, __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    if (callback == nullptr) {
        ALOGE("[%s] %s: Null callback ignored", AIDL_SW_VERSION, __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    sem_wait(&sSem);
    sGnssCallback = callback;
    sem_post(&sSem);
    mAidlGnssHalIface->setCallback(&sAidlGnssCb);

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnss::close() {
    ALOGD("[%s] AidlGnss::close", AIDL_SW_VERSION);
    sem_wait(&sSem);
    sGnssCallback = nullptr;
    sem_post(&sSem);
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnss::getExtensionPsds(std::shared_ptr<IGnssPsds>* iGnssPsds) {
    ALOGD("[%s] AidlGnss::getExtensionPsds", AIDL_SW_VERSION);

    if (mGnssHalIface == nullptr) {
        ALOGE("[%s] %s: Gnss Hal interface is unavailable", AIDL_SW_VERSION, __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }
    const GnssPsdsRequestInterface* halPsdsRequestIface =
            static_cast<const GnssPsdsRequestInterface*>(
            mGnssHalIface->get_extension(AIDL_GNSS_PSDS_INTERFACE));

    if (halPsdsRequestIface == nullptr) {
        ALOGE("[%s] %s: Gnss PSDS interface is not implemented by HAL", AIDL_SW_VERSION, __func__);
    } else {
        *iGnssPsds = SharedRefBase::make<AidlGnssPsds>(halPsdsRequestIface);
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnss::getExtensionGnssConfiguration(
        std::shared_ptr<IGnssConfiguration>* iGnssConfiguration) {
    ALOGD("[%s] AidlGnss::getExtensionGnssConfiguration", AIDL_SW_VERSION);

    if (mGnssHalIface == nullptr) {
        ALOGE("[%s] %s: Gnss Hal interface is unavailable", AIDL_SW_VERSION, __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }
    const GnssConfigurationInterface_ext* halConfigureationIface =
            static_cast<const GnssConfigurationInterface_ext*>(
            mGnssHalIface->get_extension(GNSS_CONFIGURATION_INTERFACE));

    if (halConfigureationIface == nullptr) {
        ALOGE("[%s] %s: Gnss Configration interface is not implemented by HAL",
            AIDL_SW_VERSION, __func__);
    } else {
        *iGnssConfiguration = SharedRefBase::make<AidlGnssConfiguration>(halConfigureationIface);
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnss::getExtensionGnssPowerIndication(
        std::shared_ptr<IGnssPowerIndication>* iGnssPowerIndication) {
    ALOGD("[%s] AidlGnss::getExtensionGnssPowerIndication", AIDL_SW_VERSION);

    if (mGnssHalIface == nullptr) {
        ALOGE("[%s] %s: Gnss Hal interface is unavailable", AIDL_SW_VERSION, __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    const GnssPowerIndicationInterface* halPowerIface =
            static_cast<const GnssPowerIndicationInterface*>(
            mGnssHalIface->get_extension(AIDL_POWER_INDICATION_INTERFACE));

    if (halPowerIface == nullptr) {
        ALOGE("[%s] %s: aidl PowerIndication interface not implemented by HAL",
                AIDL_SW_VERSION, __func__);
    } else {
        *iGnssPowerIndication = SharedRefBase::make<AidlGnssPowerIndication>(halPowerIface);
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnss::getExtensionGnssMeasurement(
        std::shared_ptr<IGnssMeasurementInterface>* iGnssMeasurement) {
    ALOGD("[%s] AidlGnss::getExtensionGnssMeasurement", AIDL_SW_VERSION);

    if (mGnssHalIface == nullptr) {
        ALOGE("[%s] %s: Gnss interface is unavailable", AIDL_SW_VERSION, __func__);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    const GpsMeasurementInterface_ext* halMeasurementIface =
            static_cast<const GpsMeasurementInterface_ext*>(
            mGnssHalIface->get_extension(GPS_MEASUREMENT_INTERFACE));

    if (halMeasurementIface == nullptr) {
        ALOGE("[%s] %s: GnssMeasurement interface not implemented by HAL",
                AIDL_SW_VERSION, __func__);
    } else {
        *iGnssMeasurement = SharedRefBase::make<AidlGnssMeasurement>(halMeasurementIface);
    }

    return ndk::ScopedAStatus::ok();
}

void AidlGnss::gnssCapabilitiesCb(uint32_t capabilities) {
    sem_wait(&sSem);
    if (sGnssCallback == nullptr) {
        ALOGE("[%s] %s: GNSS Callback Interface configured incorrectly", AIDL_SW_VERSION, __func__);
        sem_post(&sSem);
        return;
    }

    auto ret = sGnssCallback->gnssSetCapabilitiesCb(capabilities);
    if (!ret.isOk()) {
        ALOGE("[%s] %s: Unable to invoke callback", AIDL_SW_VERSION, __func__);
    }
    sem_post(&sSem);
}

}  // namespace aidl::android::hardware::gnss
