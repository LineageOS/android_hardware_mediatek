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

#define LOG_TAG "GnssMeasIfaceAidl"

#include "AidlGnssMeasurement.h"
#include <aidl/android/hardware/gnss/BnGnss.h>
#include <log/log.h>

namespace aidl::android::hardware::gnss {

using aidl::android::hardware::gnss::CorrelationVector;
using aidl::android::hardware::gnss::ElapsedRealtime;
using aidl::android::hardware::gnss::GnssClock;
using aidl::android::hardware::gnss::GnssData;
using aidl::android::hardware::gnss::GnssMeasurement;
using aidl::android::hardware::gnss::GnssMultipathIndicator;
using aidl::android::hardware::gnss::IGnss;
using aidl::android::hardware::gnss::IGnssMeasurementCallback;



//using Utils = ::android::hardware::gnss::common::Utils;

std::shared_ptr<IGnssMeasurementCallback> AidlGnssMeasurement::sGnssMeasureCbIface = nullptr;
GpsMeasurementCallbacks_ext AidlGnssMeasurement::sGnssMeasurementCbs = {
    .size = sizeof(GpsMeasurementCallbacks_ext),
    .measurement_callback = nullptr,
    .gnss_measurement_callback = gnssMeasurementCb
};
sem_t AidlGnssMeasurement::sSem;

AidlGnssMeasurement::AidlGnssMeasurement(
        const GpsMeasurementInterface_ext* gpsMeasurementIface) :
        mGnssHalMeasureIface(gpsMeasurementIface) {
    sem_init(&sSem, 0, 1);
}

AidlGnssMeasurement::~AidlGnssMeasurement() {
    sem_destroy(&sSem);
}

ndk::ScopedAStatus AidlGnssMeasurement::setCallback(
        const std::shared_ptr<IGnssMeasurementCallback>& callback, const bool enableFullTracking,
        const bool enableCorrVecOutputs) {
    ALOGD("AidlGnssMeasurement setCallback: enableFullTracking: %d enableCorrVecOutputs: %d",
            (int)enableFullTracking, (int)enableCorrVecOutputs);

    sem_wait(&sSem);
    if (mGnssHalMeasureIface == nullptr) {
        ALOGE("%s: Aidl Gnss Hal Measure interface is unavailable", __func__);
        sem_post(&sSem);
        return ndk::ScopedAStatus::fromExceptionCode(STATUS_INVALID_OPERATION);
    }

    sGnssMeasureCbIface = callback;
    mGnssHalMeasureIface->init(&sGnssMeasurementCbs, enableFullTracking, enableCorrVecOutputs);
    sem_post(&sSem);

    return ndk::ScopedAStatus::ok();
}


void AidlGnssMeasurement::gnssMeasurementCb(GnssData_ext* halGnssData) {
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

    GnssData gnssData;
    size_t measurementCount = halGnssData->measurement_count;
    gnssData.measurements.resize(measurementCount);
    ALOGD("AidlGnssMeasurement measurementCount: %d", (int) measurementCount);
    int maxFlags = (GnssMeasurement::HAS_SNR | GnssMeasurement::HAS_CARRIER_FREQUENCY |
        GnssMeasurement::HAS_CARRIER_CYCLES | GnssMeasurement::HAS_CARRIER_PHASE |
        GnssMeasurement::HAS_CARRIER_PHASE_UNCERTAINTY |
        GnssMeasurement::HAS_AUTOMATIC_GAIN_CONTROL |
        GnssMeasurement::HAS_FULL_ISB |
        GnssMeasurement::HAS_FULL_ISB_UNCERTAINTY |
        GnssMeasurement::HAS_SATELLITE_ISB |
        GnssMeasurement::HAS_SATELLITE_ISB_UNCERTAINTY |
        GnssMeasurement::HAS_SATELLITE_PVT |
        GnssMeasurement::HAS_CORRELATION_VECTOR);

    for (size_t i = 0; i < measurementCount; i++) {
        auto entry = halGnssData->measurements[i];

        gnssData.measurements[i] = (GnssMeasurement) {
            .signalType = {
                .constellation = (GnssConstellationType) entry.legacyMeasurement.constellation,
                .carrierFrequencyHz = entry.legacyMeasurement.carrier_frequency_hz},
            .flags = (int) (entry.legacyMeasurement.flags & maxFlags),
            .svid = entry.legacyMeasurement.svid,
            .timeOffsetNs = entry.legacyMeasurement.time_offset_ns,
            .state = (int) entry.legacyMeasurement.state,
            .receivedSvTimeInNs = entry.legacyMeasurement.received_sv_time_in_ns,
            .receivedSvTimeUncertaintyInNs =
                    entry.legacyMeasurement.received_sv_time_uncertainty_in_ns,
            .antennaCN0DbHz = entry.legacyMeasurement.c_n0_dbhz,
            .basebandCN0DbHz = entry.basebandCN0DbHz,
            .agcLevelDb = entry.agc_level_db,

            .pseudorangeRateMps = entry.legacyMeasurement.pseudorange_rate_mps,
            .pseudorangeRateUncertaintyMps =
                    entry.legacyMeasurement.pseudorange_rate_uncertainty_mps,
            .accumulatedDeltaRangeState = entry.legacyMeasurement.accumulated_delta_range_state,
            .accumulatedDeltaRangeM = entry.legacyMeasurement.accumulated_delta_range_m,
            .accumulatedDeltaRangeUncertaintyM =
                    entry.legacyMeasurement.accumulated_delta_range_uncertainty_m,
            .carrierCycles = entry.legacyMeasurement.carrier_cycles,
            .carrierPhase = entry.legacyMeasurement.carrier_phase,
            .carrierPhaseUncertainty = entry.legacyMeasurement.carrier_phase_uncertainty,
            .multipathIndicator = static_cast<GnssMultipathIndicator>(
                    entry.legacyMeasurement.multipath_indicator),
            .snrDb = entry.legacyMeasurement.snr_db,

            .fullInterSignalBiasNs = entry.fullInterSignalBiasNs,
            .fullInterSignalBiasUncertaintyNs = entry.fullInterSignalBiasUncertaintyNs,
            .satelliteInterSignalBiasNs = entry.satelliteInterSignalBiasNs,
            .satelliteInterSignalBiasUncertaintyNs = entry.satelliteInterSignalBiasUncertaintyNs,
            .satellitePvt = {
                .flags = entry.satellitePvt.flags,
                .satPosEcef = {
                    .posXMeters = entry.satellitePvt.satPosEcef.posXMeters,
                    .posYMeters = entry.satellitePvt.satPosEcef.posYMeters,
                    .posZMeters = entry.satellitePvt.satPosEcef.posZMeters,
                    .ureMeters  = entry.satellitePvt.satPosEcef.ureMeters},
                .satVelEcef = {
                    .velXMps = entry.satellitePvt.satVelEcef.velXMps,
                    .velYMps = entry.satellitePvt.satVelEcef.velYMps,
                    .velZMps = entry.satellitePvt.satVelEcef.velZMps,
                    .ureRateMps = entry.satellitePvt.satVelEcef.ureRateMps},
                .satClockInfo = {
                    .satHardwareCodeBiasMeters = entry.satellitePvt.satClockInfo.satHardwareCodeBiasMeters,
                    .satTimeCorrectionMeters = entry.satellitePvt.satClockInfo.satTimeCorrectionMeters,
                    .satClkDriftMps = entry.satellitePvt.satClockInfo.satClkDriftMps},
                .ionoDelayMeters = entry.satellitePvt.ionoDelayMeters,
                .tropoDelayMeters = entry.satellitePvt.tropoDelayMeters},
            .correlationVectors = {} };  // end of single measurement


        entry.codeType[7] = '\0';  // one bye word
        gnssData.measurements[i].signalType.codeType = entry.codeType;

        gnssData.measurements[i].correlationVectors.resize(entry.correlationVectorsSize);
        for (size_t j = 0; j < entry.correlationVectorsSize; j++) {
            auto entryCv = entry.correlationVectors[j];

            gnssData.measurements[i].correlationVectors[j] = (CorrelationVector) {
                .frequencyOffsetMps = entryCv.frequencyOffsetMps,
                .samplingWidthM = entryCv.samplingWidthM,
                .samplingStartM = entryCv.samplingStartM,
                .magnitude = {} }; // end of single correlationVector

            gnssData.measurements[i].correlationVectors[j].magnitude.resize(entryCv.magnitudeSize);
            for (size_t k = 0; k < entryCv.magnitudeSize; k++) {
                gnssData.measurements[i].correlationVectors[j].magnitude[k] = (int) entryCv.magnitude[k];
            }
        }
    } // end of meansurements for loop

    auto clockVal = halGnssData->clock;
    gnssData.clock = (GnssClock) {
        .gnssClockFlags = clockVal.legacyClock.flags,
        .leapSecond = clockVal.legacyClock.leap_second,
        .timeNs = clockVal.legacyClock.time_ns,
        .timeUncertaintyNs = clockVal.legacyClock.time_uncertainty_ns,
        .fullBiasNs = clockVal.legacyClock.full_bias_ns,
        .biasNs = clockVal.legacyClock.bias_ns,
        .biasUncertaintyNs = clockVal.legacyClock.bias_uncertainty_ns,
        .driftNsps = clockVal.legacyClock.drift_nsps,
        .driftUncertaintyNsps = clockVal.legacyClock.drift_uncertainty_nsps,
        .hwClockDiscontinuityCount = (int) clockVal.legacyClock.hw_clock_discontinuity_count,
        .referenceSignalTypeForIsb = {
                .constellation = (GnssConstellationType) clockVal.referenceSignalTypeForIsb.constellation,
                .carrierFrequencyHz = clockVal.referenceSignalTypeForIsb.carrierFrequencyHz
            }
    };
    clockVal.referenceSignalTypeForIsb.codeType[7] = '\0';  // one bye word
    gnssData.clock.referenceSignalTypeForIsb.codeType =
            clockVal.referenceSignalTypeForIsb.codeType;

    gnssData.elapsedRealtime = (ElapsedRealtime) {
            .flags = halGnssData->elapsedRealtime.flags,
            .timestampNs = (int64_t) halGnssData->elapsedRealtime.timestampNs,
            .timeUncertaintyNs = (double) halGnssData->elapsedRealtime.timeUncertaintyNs
    };
    sem_post(&sSem);

    auto ret = sGnssMeasureCbIface->gnssMeasurementCb(gnssData);
    if (!ret.isOk()) {
        ALOGE("%s: Unable to invoke callback", __func__);
    }
}


ndk::ScopedAStatus AidlGnssMeasurement::close() {
    ALOGD("%s", __func__);
    sem_wait(&sSem);
    if (mGnssHalMeasureIface == nullptr) {
        ALOGE("%s: GnssMeasure interface is unavailable", __func__);
    } else {
        mGnssHalMeasureIface->close();
        sGnssMeasureCbIface = nullptr;
    }
    sem_post(&sSem);
    return ndk::ScopedAStatus::ok();
}

}  // namespace aidl::android::hardware::gnss
