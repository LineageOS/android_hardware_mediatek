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

#pragma once

#include <aidl/android/hardware/gnss/BnGnss.h>
#include <aidl/android/hardware/gnss/BnGnssConfiguration.h>
#include <aidl/android/hardware/gnss/BnGnssMeasurementInterface.h>
#include <aidl/android/hardware/gnss/BnGnssPowerIndication.h>
#include <aidl/android/hardware/gnss/BnGnssPsds.h>
#include <hardware/gps.h>
#include <mediatek/gps_mtk.h>
#include <semaphore.h>

namespace aidl::android::hardware::gnss {

class AidlGnss : public BnGnss {
  public:
    AidlGnss();
    ~AidlGnss();
    ndk::ScopedAStatus setCallback(const std::shared_ptr<IGnssCallback>& callback) override;
    ndk::ScopedAStatus close() override;
    ndk::ScopedAStatus getExtensionPsds(std::shared_ptr<IGnssPsds>* iGnssPsds) override;
    ndk::ScopedAStatus getExtensionGnssConfiguration(
            std::shared_ptr<IGnssConfiguration>* iGnssConfiguration) override;
    ndk::ScopedAStatus getExtensionGnssPowerIndication(
            std::shared_ptr<IGnssPowerIndication>* iGnssPowerIndication) override;
    ndk::ScopedAStatus getExtensionGnssMeasurement(
            std::shared_ptr<IGnssMeasurementInterface>* iGnssMeasurement) override;

    static void gnssCapabilitiesCb(uint32_t capabilities);

  private:

    const GpsInterface_ext* mGnssHalIface;    // gnss hal interface
    const AidlGnssInterface* mAidlGnssHalIface;  // aidl gnss hal interface
    static sem_t sSem;
    static const char* sVersion;
    static AidlGnssCallbacks_ext sAidlGnssCb; // local static callback for gps hal
    static std::shared_ptr<IGnssCallback> sGnssCallback; // Aidl client callback from fwr
};

}  // namespace aidl::android::hardware::gnss
