/*
 * Copyright (C) 2018 The Android Open Source Project
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
#define LOG_TAG "GnssMeasurement"

#include "GnssMeasurement.h"

#include <log/log.h>
#include <utils/SystemClock.h>

namespace android {
namespace hardware {
namespace gnss {
namespace V2_1 {
namespace implementation {

sp<V2_1::IGnssMeasurementCallback> GnssMeasurement::sGnssMeasureCbIface = nullptr;
sem_t GnssMeasurement::sSem;

GpsMeasurementCallbacks_ext GnssMeasurement::sGnssMeasurementCbs = {
    .size = sizeof(GpsMeasurementCallbacks_ext),
    .measurement_callback = gpsMeasurementCb,
    .gnss_measurement_callback = gnssMeasurementCb
};

GnssMeasurement::GnssMeasurement(const GpsMeasurementInterface_ext* gpsMeasurementIface)
        : mGnssMeasureIface(gpsMeasurementIface) {
    sem_init(&sSem, 0, 1);
}

GnssMeasurement::~GnssMeasurement() {
    sem_destroy(&sSem);
}

void GnssMeasurement::gnssMeasurementCb(GnssData_ext* halGnssData) {
    sem_wait(&sSem);
    if (sGnssMeasureCbIface == nullptr) {
        ALOGE("%s: GNSSMeasurement Callback Interface is null", __func__);
        sem_post(&sSem);
        return;
    }

    if (halGnssData == nullptr) {
        ALOGE("%s: Invalid GnssData from GNSS HAL", __func__);
        sem_post(&sSem);
        return;
    }

    V2_1::IGnssMeasurementCallback::GnssData gnssData;
    size_t measurementCount = halGnssData->measurement_count;
    gnssData.measurements.resize(measurementCount);

    for (size_t i = 0; i < measurementCount; i++) {
        auto entry = halGnssData->measurements[i];
        auto state = static_cast<GnssMeasurementState>(entry.legacyMeasurement.state);
        if (state & IGnssMeasurementCallback::GnssMeasurementState::STATE_TOW_DECODED) {
          state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_TOW_KNOWN;
        }
        if (state & IGnssMeasurementCallback::GnssMeasurementState::STATE_GLO_TOD_DECODED) {
          state |= IGnssMeasurementCallback::GnssMeasurementState::STATE_GLO_TOD_KNOWN;
        }

        gnssData.measurements[i].v2_0.v1_1.v1_0 = (V1_0::IGnssMeasurementCallback::GnssMeasurement){
            .flags = entry.legacyMeasurement.flags,
            .svid = entry.legacyMeasurement.svid,
            .constellation = V1_0::GnssConstellationType::UNKNOWN,
            .timeOffsetNs = entry.legacyMeasurement.time_offset_ns,
            .state = state,
            .receivedSvTimeInNs = entry.legacyMeasurement.received_sv_time_in_ns,
            .receivedSvTimeUncertaintyInNs =
                    entry.legacyMeasurement.received_sv_time_uncertainty_in_ns,
            .cN0DbHz = entry.legacyMeasurement.c_n0_dbhz,
            .pseudorangeRateMps = entry.legacyMeasurement.pseudorange_rate_mps,
            .pseudorangeRateUncertaintyMps =
                    entry.legacyMeasurement.pseudorange_rate_uncertainty_mps,
            .accumulatedDeltaRangeState = entry.legacyMeasurement.accumulated_delta_range_state,
            .accumulatedDeltaRangeM = entry.legacyMeasurement.accumulated_delta_range_m,
            .accumulatedDeltaRangeUncertaintyM =
                    entry.legacyMeasurement.accumulated_delta_range_uncertainty_m,
            .carrierFrequencyHz = entry.legacyMeasurement.carrier_frequency_hz,
            .carrierCycles = entry.legacyMeasurement.carrier_cycles,
            .carrierPhase = entry.legacyMeasurement.carrier_phase,
            .carrierPhaseUncertainty = entry.legacyMeasurement.carrier_phase_uncertainty,
            .multipathIndicator = static_cast<IGnssMeasurementCallback::GnssMultipathIndicator>(
                    entry.legacyMeasurement.multipath_indicator),
            .snrDb = entry.legacyMeasurement.snr_db,
            .agcLevelDb = entry.agc_level_db
        };
        /// v1.1
        gnssData.measurements[i].v2_0.v1_1.accumulatedDeltaRangeState =
                    entry.legacyMeasurement.accumulated_delta_range_state;
        /// v2.0
        entry.codeType[7] = '\0';  // one bye word
        gnssData.measurements[i].v2_0.codeType.setToExternal(entry.codeType, strlen(entry.codeType));

        gnssData.measurements[i].v2_0.state = state;
        gnssData.measurements[i].v2_0.constellation = static_cast<V2_0::GnssConstellationType>(
                    entry.legacyMeasurement.constellation);
        /// v2.1
        gnssData.measurements[i].flags = entry.legacyMeasurement.flags;
        gnssData.measurements[i].fullInterSignalBiasNs = entry.fullInterSignalBiasNs;
        gnssData.measurements[i].fullInterSignalBiasUncertaintyNs = entry.fullInterSignalBiasUncertaintyNs;
        gnssData.measurements[i].satelliteInterSignalBiasNs = entry.satelliteInterSignalBiasNs;
        gnssData.measurements[i].satelliteInterSignalBiasUncertaintyNs = entry.satelliteInterSignalBiasUncertaintyNs;
        gnssData.measurements[i].basebandCN0DbHz = entry.basebandCN0DbHz;
    }

    auto clockVal = halGnssData->clock;
    gnssData.clock.v1_0 = {
        .gnssClockFlags = clockVal.legacyClock.flags,
        .leapSecond = clockVal.legacyClock.leap_second,
        .timeNs = clockVal.legacyClock.time_ns,
        .timeUncertaintyNs = clockVal.legacyClock.time_uncertainty_ns,
        .fullBiasNs = clockVal.legacyClock.full_bias_ns,
        .biasNs = clockVal.legacyClock.bias_ns,
        .biasUncertaintyNs = clockVal.legacyClock.bias_uncertainty_ns,
        .driftNsps = clockVal.legacyClock.drift_nsps,
        .driftUncertaintyNsps = clockVal.legacyClock.drift_uncertainty_nsps,
        .hwClockDiscontinuityCount = clockVal.legacyClock.hw_clock_discontinuity_count
    };
    /// v2.1
    gnssData.clock.referenceSignalTypeForIsb = {
        .constellation = static_cast<V2_0::GnssConstellationType>(
                clockVal.referenceSignalTypeForIsb.constellation),
        .carrierFrequencyHz = clockVal.referenceSignalTypeForIsb.carrierFrequencyHz,
    };
    clockVal.referenceSignalTypeForIsb.codeType[7] = '\0';  // one bye word
    gnssData.clock.referenceSignalTypeForIsb.codeType.setToExternal(
            clockVal.referenceSignalTypeForIsb.codeType,
            strlen(clockVal.referenceSignalTypeForIsb.codeType));

    /// v2.0
    ElapsedRealtime timestamp = {
            .flags = halGnssData->elapsedRealtime.flags,
            .timestampNs = halGnssData->elapsedRealtime.timestampNs,
            .timeUncertaintyNs = halGnssData->elapsedRealtime.timeUncertaintyNs
    };

    gnssData.elapsedRealtime = timestamp;

    auto ret = sGnssMeasureCbIface->gnssMeasurementCb_2_1(gnssData);
    if (!ret.isOk()) {
        ALOGE("%s: Unable to invoke callback", __func__);
    }
    sem_post(&sSem);
}

/*
 * The code in the following method has been moved here from GnssLocationProvider.
 * It converts GpsData to GnssData. This code is no longer required in
 * GnssLocationProvider since GpsData is deprecated and no longer part of the
 * GNSS interface.
 */
void GnssMeasurement::gpsMeasurementCb(GpsData*) {
}

// Methods from ::android::hardware::gnss::V1_0::IGnssMeasurement follow.
Return<V1_0::IGnssMeasurement::GnssMeasurementStatus>
GnssMeasurement::setCallback(const sp<V1_0::IGnssMeasurementCallback>&) {
    return V1_0::IGnssMeasurement::GnssMeasurementStatus::ERROR_GENERIC;
}


// Methods from ::android::hardware::gnss::V1_1::IGnssMeasurement follow.
Return<V1_0::IGnssMeasurement::GnssMeasurementStatus>
GnssMeasurement::setCallback_1_1(const sp<V1_1::IGnssMeasurementCallback>&, bool) {
    return V1_1::IGnssMeasurement::GnssMeasurementStatus::ERROR_GENERIC;
}

// Methods from ::android::hardware::gnss::V1_1::IGnssMeasurement follow.
Return<V1_0::IGnssMeasurement::GnssMeasurementStatus>
GnssMeasurement::setCallback_2_0(const sp<V2_0::IGnssMeasurementCallback>&, bool) {
    return V2_0::IGnssMeasurement::GnssMeasurementStatus::ERROR_GENERIC;
}

Return<V1_0::IGnssMeasurement::GnssMeasurementStatus> GnssMeasurement::setCallback_2_1(
    const sp<V2_1::IGnssMeasurementCallback>& callback, bool enableFullTracking) {

    sem_wait(&sSem);
    if (mGnssMeasureIface == nullptr) {
        ALOGE("%s: GnssMeasure interface is unavailable", __func__);
        sem_post(&sSem);
        return GnssMeasurementStatus::ERROR_GENERIC;
    }
    sGnssMeasureCbIface = callback;

    int ret = mGnssMeasureIface->init(&sGnssMeasurementCbs, enableFullTracking, false);
    sem_post(&sSem);
    return static_cast<GnssMeasurement::GnssMeasurementStatus>(ret);
}

Return<void> GnssMeasurement::close()  {
    sem_wait(&sSem);
    if (mGnssMeasureIface == nullptr) {
        ALOGE("%s: GnssMeasure interface is unavailable", __func__);
    } else {
        mGnssMeasureIface->close();
        sGnssMeasureCbIface = nullptr;
    }
    sem_post(&sSem);
    return Void();
}

}  // namespace implementation
}  // namespace V2_1
}  // namespace gnss
}  // namespace hardware
}  // namespace android
