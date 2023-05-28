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

#define LOG_TAG "GnssAntennaInfo"

#include "GnssAntennaInfo.h"
#include <log/log.h>

namespace android {
namespace hardware {
namespace gnss {
namespace V2_1 {
namespace implementation {

sp<IGnssAntennaInfoCallback> GnssAntennaInfo::sGnssAntennaInfoCbIface = nullptr;
sem_t GnssAntennaInfo::sSem;
bool GnssAntennaInfo::sReported = false;

GnssAntennaInfoCallbacks GnssAntennaInfo::sGnssAntennaInfoCbs = {
    .size = sizeof(GnssAntennaInfoCallbacks),
    .antenna_info_callback = gnssAntennaInfoCb
};


GnssAntennaInfo::GnssAntennaInfo(const GnssAntennaInfoInterface* gnssAntennaInfoIface)
        : mGnssAntennaInfoIface(gnssAntennaInfoIface) {
    sem_init(&sSem, 0, 1);
    mIsActive = false;
    ALOGD("construct mIsActive: %d", mIsActive);
}

GnssAntennaInfo::~GnssAntennaInfo() {
    close();
    sem_destroy(&sSem);
}

// Methods from ::android::hardware::gnss::V2_1::IGnssAntennaInfo follow.
Return<GnssAntennaInfo::GnssAntennaInfoStatus> GnssAntennaInfo::setCallback(
        const sp<IGnssAntennaInfoCallback>& callback) {
    ALOGD("setCallback");
    sem_wait(&sSem);
    sGnssAntennaInfoCbIface = callback;

    if (mIsActive) {
        sem_post(&sSem);
        ALOGD("return already init");
        return GnssAntennaInfoStatus::ERROR_ALREADY_INIT;
    }
    mIsActive = true;
    sReported = false;

    int ret = mGnssAntennaInfoIface->setCallback(&sGnssAntennaInfoCbs);
    sem_post(&sSem);
    /// testing
    //sendMockAntennaInfos();
    ALOGD("hal return init ret: %d", ret);
    return (ret==0) ? GnssAntennaInfoStatus::SUCCESS : GnssAntennaInfoStatus::ERROR_GENERIC;
}

Return<void> GnssAntennaInfo::close() {
    ALOGD("close");
    sem_wait(&sSem);

    mIsActive = false;
    sGnssAntennaInfoCbIface = nullptr;
    sem_post(&sSem);
    return Void();
}

void GnssAntennaInfo::gnssAntennaInfoCb(GnssAntennaInfos_ext* halGnssAntennaInfos) {
    ALOGD("gnssAntennaInfoCb");
    sem_wait(&sSem);
    sReported = true;

    /// parsing hal reported data.
    hidl_vec<IGnssAntennaInfoCallback::GnssAntennaInfo> antennaInfo;
    int antennaInfoSize = sizeof(halGnssAntennaInfos->antennaInfos) / sizeof(halGnssAntennaInfos->antennaInfos[0]);
    antennaInfo.resize(antennaInfoSize);

    for (int i=0; i < antennaInfoSize; i++) {
        auto entry = halGnssAntennaInfos->antennaInfos[i];
        antennaInfo[i].carrierFrequencyMHz =  entry.carrierFrequencyMHz;
        antennaInfo[i].phaseCenterOffsetCoordinateMillimeters = {
            .x = entry.phaseCenterOffsetCoordinateMillimeters.x,
            .xUncertainty = entry.phaseCenterOffsetCoordinateMillimeters.xUncertainty,
            .y = entry.phaseCenterOffsetCoordinateMillimeters.y,
            .yUncertainty = entry.phaseCenterOffsetCoordinateMillimeters.yUncertainty,
            .z = entry.phaseCenterOffsetCoordinateMillimeters.z,
            .zUncertainty = entry.phaseCenterOffsetCoordinateMillimeters.zUncertainty
        };

        int pcvcmSize = sizeof(entry.phaseCenterVariationCorrectionMillimeters) / sizeof(entry.phaseCenterVariationCorrectionMillimeters[0]);
        antennaInfo[i].phaseCenterVariationCorrectionMillimeters.resize(pcvcmSize);
        ALOGD("pcvcmSize: %d", pcvcmSize);
        for (int j=0; j < pcvcmSize; j++) {
            auto entry2 = entry.phaseCenterVariationCorrectionMillimeters[j];
            int rowSize = sizeof(entry2.row) / sizeof(entry2.row[0]);
            antennaInfo[i].phaseCenterVariationCorrectionMillimeters[j].row.resize(rowSize);
            for (int k=0; k < rowSize; k++) {
                antennaInfo[i].phaseCenterVariationCorrectionMillimeters[j].row[k] = entry2.row[k];
            }
        }

        int pcvcumSize = sizeof(entry.phaseCenterVariationCorrectionUncertaintyMillimeters) / sizeof(entry.phaseCenterVariationCorrectionUncertaintyMillimeters[0]);
        antennaInfo[i].phaseCenterVariationCorrectionUncertaintyMillimeters.resize(pcvcumSize);
        ALOGD("pcvcumSize: %d", pcvcumSize);
        for (int j=0; j < pcvcumSize; j++) {
            auto entry2 = entry.phaseCenterVariationCorrectionUncertaintyMillimeters[j];
            int rowSize = sizeof(entry2.row) / sizeof(entry2.row[0]);
            antennaInfo[i].phaseCenterVariationCorrectionUncertaintyMillimeters[j].row.resize(rowSize);
            for (int k=0; k < rowSize; k++) {
                antennaInfo[i].phaseCenterVariationCorrectionUncertaintyMillimeters[j].row[k] = entry2.row[k];
            }
        }

        int sgcdSize = sizeof(entry.signalGainCorrectionDbi) / sizeof(entry.signalGainCorrectionDbi[0]);
        antennaInfo[i].signalGainCorrectionDbi.resize(sgcdSize);
        ALOGD("sgcdSize: %d", sgcdSize);
        for (int j=0; j < sgcdSize; j++) {
            auto entry2 = entry.signalGainCorrectionDbi[j];
            int rowSize = sizeof(entry2.row) / sizeof(entry2.row[0]);
            antennaInfo[i].signalGainCorrectionDbi[j].row.resize(rowSize);
            for (int k=0; k < rowSize; k++) {
                antennaInfo[i].signalGainCorrectionDbi[j].row[k] = entry2.row[k];
            }
        }

        int sgcudSize = sizeof(entry.signalGainCorrectionUncertaintyDbi) / sizeof(entry.signalGainCorrectionUncertaintyDbi[0]);
        antennaInfo[i].signalGainCorrectionUncertaintyDbi.resize(sgcudSize);
        ALOGD("sgcudSize: %d", sgcudSize);
        for (int j=0; j < sgcudSize; j++) {
            auto entry2 = entry.signalGainCorrectionUncertaintyDbi[j];
            int rowSize = sizeof(entry2.row) / sizeof(entry2.row[0]);
            antennaInfo[i].signalGainCorrectionUncertaintyDbi[j].row.resize(rowSize);
            for (int k=0; k < rowSize; k++) {
                antennaInfo[i].signalGainCorrectionUncertaintyDbi[j].row[k] = entry2.row[k];
            }
        }
    }

    /// callback antenna information
    if (sGnssAntennaInfoCbIface == nullptr) {
        ALOGE("%s: No non-null callback", __func__);
        sem_post(&sSem);
        return;
    }

    auto ret = sGnssAntennaInfoCbIface->gnssAntennaInfoCb(antennaInfo);
    if (!ret.isOk()) {
        ALOGE("%s: Unable to invoke callback", __func__);
    }
    sem_post(&sSem);
}

void GnssAntennaInfo::sendMockAntennaInfos() {
    ALOGD("send mock antenna info");
    GnssAntennaInfo_ext mockAntennaInfo_1 = {
            .carrierFrequencyMHz = 123412.12,
            .phaseCenterOffsetCoordinateMillimeters = Coord{.x = 1,
                                                            .xUncertainty = 0.1,
                                                            .y = 2,
                                                            .yUncertainty = 0.1,
                                                            .z = 3,
                                                            .zUncertainty = 0.1},
            .phaseCenterVariationCorrectionMillimeters =
                    {
                            Row{1, -1, 5, -2, 3, -1},
                            Row{-2, 3, 2, 0, 1, 2},
                            Row{1, 3, 2, -1, -3, 5},
                    },
            .phaseCenterVariationCorrectionUncertaintyMillimeters =
                    {
                            Row{0.1, 0.2, 0.4, 0.1, 0.2, 0.3},
                            Row{0.3, 0.2, 0.3, 0.6, 0.1, 0.1},
                            Row{0.1, 0.1, 0.4, 0.2, 0.5, 0.3},
                    },
            .signalGainCorrectionDbi =
                    {
                            Row{2, -3, 1, -3, 0, -4},
                            Row{1, 0, -4, 1, 3, -2},
                            Row{3, -2, 0, -2, 3, 0},
                    },
            .signalGainCorrectionUncertaintyDbi =
                    {
                            Row{0.3, 0.1, 0.2, 0.6, 0.1, 0.3},
                            Row{0.1, 0.1, 0.5, 0.2, 0.3, 0.1},
                            Row{0.2, 0.4, 0.2, 0.1, 0.1, 0.2},
                    },
    };

    GnssAntennaInfo_ext mockAntennaInfo_2 = {
            .carrierFrequencyMHz = 532324.23,
            .phaseCenterOffsetCoordinateMillimeters = Coord{.x = 5,
                                                            .xUncertainty = 0.1,
                                                            .y = 6,
                                                            .yUncertainty = 0.1,
                                                            .z = 7,
                                                            .zUncertainty = 0.1},
    };

    GnssAntennaInfos_ext mockAntennaInfos = {
            mockAntennaInfo_1,
            mockAntennaInfo_2,
    };

    GnssAntennaInfo::gnssAntennaInfoCb(&mockAntennaInfos);
}

}  // namespace implementation
}  // namespace V2_1
}  // namespace gnss
}  // namespace hardware
}  // namespace android
