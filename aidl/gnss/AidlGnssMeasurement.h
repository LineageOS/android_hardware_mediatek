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

#include <aidl/android/hardware/gnss/BnGnssMeasurementCallback.h>
#include <aidl/android/hardware/gnss/BnGnssMeasurementInterface.h>
#include <hardware/gps.h>
#include "gps_mtk.h"
#include <semaphore.h>

namespace aidl::android::hardware::gnss {

struct AidlGnssMeasurement : public BnGnssMeasurementInterface {
  public:
    AidlGnssMeasurement(const GpsMeasurementInterface_ext* gpsMeasurementIface);
    ~AidlGnssMeasurement();
    ndk::ScopedAStatus setCallback(const std::shared_ptr<IGnssMeasurementCallback>& callback,
                                   const bool enableFullTracking,
                                   const bool enableCorrVecOutputs) override;
    ndk::ScopedAStatus close() override;

  private:

    /*
     * Callback methods to be passed into the conventional GNSS HAL by the default
     * implementation. These methods are not part of the IGnssMeasurement base class.
     */
    static void gnssMeasurementCb(GnssData_ext* data);

    // callback from fwr
    static std::shared_ptr<IGnssMeasurementCallback> sGnssMeasureCbIface;

    // hal implemented Gnss Measurement interface
    const GpsMeasurementInterface_ext* mGnssHalMeasureIface;

    // local callback structure for gnss hal
    static GpsMeasurementCallbacks_ext sGnssMeasurementCbs;

    static sem_t sSem;
};

}  // namespace aidl::android::hardware::gnss
