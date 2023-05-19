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

#define LOG_TAG "GnssPowerIndicationAidl"

#include "AidlGnssPowerIndication.h"
#include <aidl/android/hardware/gnss/BnGnss.h>
#include <log/log.h>

namespace aidl::android::hardware::gnss {
using aidl::android::hardware::gnss::GnssPowerStats;

std::shared_ptr<IGnssPowerIndicationCallback> AidlGnssPowerIndication::sGnssPowerCbIface = nullptr;
GnssPowerIndicationCallbacks_ext AidlGnssPowerIndication::sAidlGnssPowerIndicationCbs = {
    .size = sizeof(GnssPowerIndicationCallbacks_ext),
    .gnss_capabilities_cb = powerCapabilitiesCallback,
    .gnss_power_stats_cb = powerStatsCallback
};
sem_t AidlGnssPowerIndication::sSem;

AidlGnssPowerIndication:: AidlGnssPowerIndication(
        const GnssPowerIndicationInterface* halPowerIface) :
        mGnssHalPowerIface(halPowerIface) {
    sem_init(&sSem, 0, 1);
}

AidlGnssPowerIndication::~AidlGnssPowerIndication() {
    sem_destroy(&sSem);
}

ndk::ScopedAStatus AidlGnssPowerIndication::setCallback(
        const std::shared_ptr<IGnssPowerIndicationCallback>& callback) {
    ALOGD("AidlGnssPowerIndication setCallback");

    sem_wait(&sSem);
    if (mGnssHalPowerIface == nullptr) {
        ALOGE("%s: Aidl Gnsss Hal Power interface is unavailable", __func__);
        sem_post(&sSem);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    sGnssPowerCbIface = callback;
    sem_post(&sSem);
    mGnssHalPowerIface->setCallback(&sAidlGnssPowerIndicationCbs);

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnssPowerIndication::requestGnssPowerStats() {
    ALOGD("requestGnssPowerStats");

    sem_wait(&sSem);
    if (mGnssHalPowerIface == nullptr) {
        ALOGE("%s: Aidl Gnsss Hal Power interface is unavailable", __func__);
        sem_post(&sSem);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    sem_post(&sSem);
    mGnssHalPowerIface->requestGnssPowerStats();
    return ndk::ScopedAStatus::ok();
}

void AidlGnssPowerIndication::powerCapabilitiesCallback(uint32_t capabilities) {
    ALOGD("powerCapabilitiesCallback cap = 0x%x", (int)capabilities);
    sem_wait(&sSem);
    if (sGnssPowerCbIface == nullptr) {
        ALOGE("%s: GnssPowerCbIface Callback Interface is null", __func__);
        sem_post(&sSem);
        return;
    }

    sGnssPowerCbIface->setCapabilitiesCb(capabilities);
    sem_post(&sSem);
}

void AidlGnssPowerIndication::powerStatsCallback(GnssPowerStats_ext* powerStatsData) {
    ALOGD("powerStatsCallback");
    sem_wait(&sSem);
    if (sGnssPowerCbIface == nullptr) {
        ALOGE("%s: GnssPowerCbIface Callback Interface is null", __func__);
        sem_post(&sSem);
        return;
    }

    if (powerStatsData == nullptr) {
        ALOGE("%s: Invalid gnssPowerStats from GNSS HAL", __func__);
        sem_post(&sSem);
        return;
    }

    GnssPowerStats powerStat;

    ALOGI("power elapsedRealtime.flags: 0x%x", powerStatsData->elapsedRealtime.flags);

    ALOGI("power elapsedRealtime: %ld, totalEnergyMilliJoule: %f",
          (long)powerStatsData->elapsedRealtime.timestampNs, powerStatsData->totalEnergyMilliJoule);
    ALOGI("power singlebandTrackingModeEnergyMilliJoule: %f, multibandTrackingModeEnergyMilliJoule: %f",
          powerStatsData->singlebandTrackingModeEnergyMilliJoule,
          powerStatsData->multibandTrackingModeEnergyMilliJoule);
    ALOGI("power singlebandAcquisitionModeEnergyMilliJoule: %f, "
          "power multibandAcquisitionModeEnergyMilliJoule: %f",
          powerStatsData->singlebandAcquisitionModeEnergyMilliJoule,
          powerStatsData->multibandAcquisitionModeEnergyMilliJoule);

    powerStat = (GnssPowerStats) {
        .elapsedRealtime = {
            .flags = powerStatsData->elapsedRealtime.flags,
            .timestampNs = (int64_t) powerStatsData->elapsedRealtime.timestampNs,
            .timeUncertaintyNs = (double) powerStatsData->elapsedRealtime.timeUncertaintyNs},
            .totalEnergyMilliJoule = powerStatsData->totalEnergyMilliJoule,
            .singlebandTrackingModeEnergyMilliJoule = powerStatsData->singlebandTrackingModeEnergyMilliJoule,
            .multibandTrackingModeEnergyMilliJoule = powerStatsData->multibandTrackingModeEnergyMilliJoule,
            .singlebandAcquisitionModeEnergyMilliJoule = powerStatsData->singlebandAcquisitionModeEnergyMilliJoule,
            .multibandAcquisitionModeEnergyMilliJoule = powerStatsData->multibandAcquisitionModeEnergyMilliJoule,
            .otherModesEnergyMilliJoule = {}};

    powerStat.otherModesEnergyMilliJoule.resize(
            powerStatsData->otherModesEnergyMilliJouleSize);
    for (size_t i = 0; i < powerStatsData->otherModesEnergyMilliJouleSize; i++) {
        powerStat.otherModesEnergyMilliJoule[i] = powerStatsData->otherModesEnergyMilliJoule[i];
        ALOGI("otherModeEnergyMilliJoule[%d]: %f", (int)i, powerStat.otherModesEnergyMilliJoule[i]);
    }

    sGnssPowerCbIface->gnssPowerStatsCb(powerStat);
    sem_post(&sSem);
}

}  // namespace aidl::android::hardware::gnss
