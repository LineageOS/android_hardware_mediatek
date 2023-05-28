/*
 * Copyright (C) 2019 The Android Open Source Project
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

#define LOG_TAG "GnssMeasurementCorrections"

#include "GnssMeasurementCorrections.h"
#include <log/log.h>

namespace android {
namespace hardware {
namespace gnss {
namespace measurement_corrections {
namespace V1_1 {
namespace implementation {

sp<V1_0::IMeasurementCorrectionsCallback> GnssMeasurementCorrections::sMeasureCallbackCbIface = nullptr;

MeasurementCorrectionCallbacks_ext GnssMeasurementCorrections::sMeasurementCorrectionCbs = {
    .size = sizeof(MeasurementCorrectionCallbacks_ext),
    .set_capabilities_cb = setCapabilitiesCb
};

GnssMeasurementCorrections::GnssMeasurementCorrections(
        const MeasurementCorrectionInterface* mesCorrectionIface)
        : mMeasurementCorrectionInterface(mesCorrectionIface){}


// Methods from V1_0::IMeasurementCorrections follow.
Return<bool> GnssMeasurementCorrections::setCorrections(const V1_0::MeasurementCorrections& corrections) {
    ALOGD("unused setCorrections, size: %d ", static_cast<int>(corrections.satCorrections.size()));
    return true;
}

/// v1.1
Return<bool> GnssMeasurementCorrections::setCorrections_1_1(const V1_1::MeasurementCorrections& corrections) {
    if (mMeasurementCorrectionInterface == nullptr) {
        ALOGE("%s: GnssMeasurementCorrections interface is unavailable", __func__);
        return false;
    }
    ::MeasurementCorrections correctData;
    int max_sat_correction = sizeof(correctData.satCorrections) / sizeof(correctData.satCorrections[0]);

    memset(&correctData, 0x00, sizeof(::MeasurementCorrections));
    correctData.latitudeDegrees = corrections.v1_0.latitudeDegrees;
    correctData.latitudeDegrees = corrections.v1_0.latitudeDegrees;
    correctData.longitudeDegrees = corrections.v1_0.longitudeDegrees;
    correctData.altitudeMeters = corrections.v1_0.altitudeMeters;
    correctData.horizontalPositionUncertaintyMeters = corrections.v1_0.horizontalPositionUncertaintyMeters;
    correctData.verticalPositionUncertaintyMeters = corrections.v1_0.verticalPositionUncertaintyMeters;
    correctData.toaGpsNanosecondsOfWeek =
            static_cast<unsigned long long>(corrections.v1_0.toaGpsNanosecondsOfWeek);

    correctData.num_satCorrection = corrections.satCorrections.size();
    /// v1.1
    correctData.hasEnvironmentBearing = corrections.hasEnvironmentBearing;
    correctData.environmentBearingDegrees = corrections.environmentBearingDegrees;
    correctData.environmentBearingUncertaintyDegrees = corrections.environmentBearingUncertaintyDegrees;

    ALOGD("setCorrections size: %d max size: %d", static_cast<int>(corrections.satCorrections.size()),
            max_sat_correction);
/*    ALOGD("corrections = lat: %f, lng: %f, alt: %f, hUnc: %f, vUnc: %f, toa: %llu, "
          "satCorrections.size: %d",
          corrections.latitudeDegrees, corrections.longitudeDegrees, corrections.altitudeMeters,
          corrections.horizontalPositionUncertaintyMeters,
          corrections.verticalPositionUncertaintyMeters,
          static_cast<unsigned long long>(corrections.toaGpsNanosecondsOfWeek),
          static_cast<int>(corrections.satCorrections.size()));*/
    int i = 0;
    for (auto ssc : corrections.satCorrections) {
        ::SingleSatCorrection* satCorrection = &correctData.satCorrections[i];
        satCorrection->singleSatCorrectionFlags = ssc.v1_0.singleSatCorrectionFlags;
        satCorrection->constellation = static_cast<::GnssConstellationType>(ssc.constellation);
        satCorrection->svid = ssc.v1_0.svid;
        satCorrection->carrierFrequencyHz = ssc.v1_0.carrierFrequencyHz;
        satCorrection->probSatIsLos = ssc.v1_0.probSatIsLos;
        satCorrection->excessPathLengthMeters = ssc.v1_0.excessPathLengthMeters;
        satCorrection->excessPathLengthUncertaintyMeters = ssc.v1_0.excessPathLengthUncertaintyMeters;
        satCorrection->reflectingPlane.latitudeDegrees = ssc.v1_0.reflectingPlane.latitudeDegrees;
        satCorrection->reflectingPlane.longitudeDegrees = ssc.v1_0.reflectingPlane.longitudeDegrees;
        satCorrection->reflectingPlane.altitudeMeters = ssc.v1_0.reflectingPlane.altitudeMeters;
        satCorrection->reflectingPlane.azimuthDegrees = ssc.v1_0.reflectingPlane.azimuthDegrees;
        /*ALOGD("singleSatCorrection = flags: %d, constellation: %d, svid: %d, cfHz: %f, probLos: %f,"
              " epl: %f, eplUnc: %f",
              static_cast<int>(ssc.singleSatCorrectionFlags),
              static_cast<int>(ssc.constellation),
              static_cast<int>(ssc.svid), ssc.carrierFrequencyHz,
              ssc.probSatIsLos, ssc.excessPathLengthMeters,
              ssc.excessPathLengthUncertaintyMeters);
        ALOGD("reflecting plane = lat: %f, lng: %f, alt: %f, azm: %f",
              ssc.reflectingPlane.latitudeDegrees,
              ssc.reflectingPlane.longitudeDegrees,
              ssc.reflectingPlane.altitudeMeters,
              ssc.reflectingPlane.azimuthDegrees);*/

        if (++i >= max_sat_correction) {
            correctData.num_satCorrection = max_sat_correction;
            break;
        }
    }

    return mMeasurementCorrectionInterface->meac_set_corrections(&correctData);
}

Return<bool> GnssMeasurementCorrections::setCallback(
        const sp<V1_0::IMeasurementCorrectionsCallback>& callback) {
    if (mMeasurementCorrectionInterface == nullptr) {
        ALOGE("%s: GnssMeasurementCorrections interface is unavailable", __func__);
        return false;
    }

    sMeasureCallbackCbIface = callback;

    return mMeasurementCorrectionInterface->meac_set_callback(&sMeasurementCorrectionCbs);
}

void GnssMeasurementCorrections::setCapabilitiesCb(uint32_t capabilities) {
    if (sMeasureCallbackCbIface == nullptr) {
        ALOGE("%s: MeasurementCorrection Callback Interface configured incorrectly", __func__);
        return;
    }

    auto ret = sMeasureCallbackCbIface->setCapabilitiesCb(capabilities);
    if (!ret.isOk()) {
        ALOGE("%s: Unable to invoke callback", __func__);
    }
}


}  // namespace implementation
}  // namespace V1_1
}  // namespace measurement_corrections
}  // namespace gnss
}  // namespace hardware
}  // namespace android
