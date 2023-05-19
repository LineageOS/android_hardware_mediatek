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

#define LOG_TAG "AidlGnssPsds"

#include "AidlGnssPsds.h"
#include <aidl/android/hardware/gnss/BnGnss.h>
#include <log/log.h>

namespace aidl::android::hardware::gnss {

std::shared_ptr<IGnssPsdsCallback> AidlGnssPsds::sPsdsCbIface = nullptr;
GnssPsdsCallbacks_ext AidlGnssPsds::sAidlGnssPsdsCbs = {
    .size = sizeof(GnssPsdsCallbacks_ext),
    .psds_request_cb = psdsRequestCb,
};
sem_t AidlGnssPsds::sSem;

AidlGnssPsds::AidlGnssPsds(const GnssPsdsRequestInterface* halPsdsRequestIface) :
        mGnssHalPsdsIface(halPsdsRequestIface) {
    sem_init(&sSem, 0, 1);
}

AidlGnssPsds::~AidlGnssPsds() {
    sem_destroy(&sSem);
}



ndk::ScopedAStatus AidlGnssPsds::setCallback(const std::shared_ptr<IGnssPsdsCallback>& callback) {
    ALOGD("AidlGnssPsds setCallback");

    sem_wait(&sSem);
    if (mGnssHalPsdsIface == nullptr) {
        ALOGE("%s: Gnss Hal Psds interface is unavailable", __func__);
        sem_post(&sSem);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    sPsdsCbIface = callback;
    sem_post(&sSem);
    mGnssHalPsdsIface->setCallback(&sAidlGnssPsdsCbs);

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus AidlGnssPsds::injectPsdsData(PsdsType psdsType,
        const std::vector<uint8_t>& psdsData) {
    ALOGD("injectPsdsData. psdsType: %d, psdsData: %d bytes", static_cast<int>(psdsType),
          static_cast<int>(psdsData.size()));

    sem_wait(&sSem);
    if (mGnssHalPsdsIface == nullptr) {
        ALOGE("%s: Gnss Hal Psds interface is unavailable", __func__);
        sem_post(&sSem);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    if (psdsData.size() == 0) {
        ALOGE("%s: Framework injected psds data size is 0", __func__);
        sem_post(&sSem);
        return ndk::ScopedAStatus::fromServiceSpecificError(IGnss::ERROR_INVALID_ARGUMENT);
    }

    mGnssHalPsdsIface->injectPsdsData((Psds_type)psdsType, (const char*) &psdsData[0], psdsData.size());

    sem_post(&sSem);

    return ndk::ScopedAStatus::ok();
}

void AidlGnssPsds::psdsRequestCb(Psds_type psdsType) {
    ALOGE("%s", __func__);
    sem_wait(&sSem);
    if (sPsdsCbIface == nullptr) {
        ALOGE("%s: PsdsRequest Callback Interface is null", __func__);
        sem_post(&sSem);
        return;
    }

    sPsdsCbIface->downloadRequestCb((PsdsType)psdsType);

    sem_post(&sSem);
}

}  // namespace aidl::android::hardware::gnss
