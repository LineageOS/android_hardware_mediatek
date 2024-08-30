/*
 * SPDX-FileCopyrightText: 2019 The Android Open Source Project
 * SPDX-FileCopyrightText: 2024 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <hardware/sensors.h>
#include <sensors/convert.h>
#include <thread>
#include <vector>

#include "V2_0/SubHal.h"

namespace android {
namespace hardware {
namespace sensors {
namespace V2_0 {
namespace subhal {
namespace implementation {

using ::android::hardware::sensors::V1_0::Event;
using ::android::hardware::sensors::V1_0::OperationMode;
using ::android::hardware::sensors::V1_0::RateLevel;
using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V1_0::SensorInfo;
using ::android::hardware::sensors::V1_0::SharedMemInfo;
using ::android::hardware::sensors::V2_0::implementation::IHalProxyCallback;
using ::android::hardware::sensors::V2_0::implementation::ISensorsSubHal;

class SensorsSubHal : public ISensorsSubHal {
  public:
    SensorsSubHal();

    Return<void> getSensorsList(ISensors::getSensorsList_cb _hidl_cb);
    Return<Result> injectSensorData(const Event& event);
    Return<Result> initialize(const sp<IHalProxyCallback>& halProxyCallback);

    virtual Return<Result> setOperationMode(OperationMode mode);

    OperationMode getOperationMode() const { return mCurrentOperationMode; }

    Return<Result> activate(int32_t sensorHandle, bool enabled);

    Return<Result> batch(int32_t sensorHandle, int64_t samplingPeriodNs,
                         int64_t maxReportLatencyNs);

    Return<Result> flush(int32_t sensorHandle);

    Return<void> registerDirectChannel(const SharedMemInfo& mem,
                                       ISensors::registerDirectChannel_cb _hidl_cb);

    Return<Result> unregisterDirectChannel(int32_t channelHandle);

    Return<void> configDirectReport(int32_t sensorHandle, int32_t channelHandle, RateLevel rate,
                                    ISensors::configDirectReport_cb _hidl_cb);

    Return<void> debug(const hidl_handle& fd, const hidl_vec<hidl_string>& args);

    const std::string getName() { return "Sensors1SubHal"; }

  protected:
    std::map<int32_t, SensorInfo> mSensors;

    sp<IHalProxyCallback> mCallback;

  private:
    OperationMode mCurrentOperationMode = OperationMode::NORMAL;

    static constexpr int32_t kPollMaxBufferSize = 128;
    std::thread mPollThread;
    sensors_poll_device_1_t* mSensorDevice;
    struct sensors_module_t* mSensorModule;

    bool isWakeUpSensor(int32_t handle);
    int getHalDeviceVersion() const;
    void convertFromSensorEvents(size_t count, const sensors_event_t* srcArray,
                                 std::vector<Event>* dstVec, bool* wakeup);
    void enumerateSensors();
    void pollForEvents();
    void postEvents(const std::vector<Event>& events, bool wakeup);
};

}  // namespace implementation
}  // namespace subhal
}  // namespace V2_0
}  // namespace sensors
}  // namespace hardware
}  // namespace android
