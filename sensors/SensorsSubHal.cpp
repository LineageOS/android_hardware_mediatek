/*
 * SPDX-FileCopyrightText: 2019 The Android Open Source Project
 * SPDX-FileCopyrightText: 2024 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include "SensorsSubHal.h"

#include <android-base/logging.h>
#include <android/hardware/sensors/2.0/types.h>

using ::android::hardware::sensors::V1_0::SensorFlagBits;
using ::android::hardware::sensors::V1_0::implementation::convertFromRateLevel;
using ::android::hardware::sensors::V1_0::implementation::convertFromSensor;
using ::android::hardware::sensors::V1_0::implementation::convertFromSensorEvent;
using ::android::hardware::sensors::V1_0::implementation::convertFromSharedMemInfo;
using ::android::hardware::sensors::V1_0::implementation::convertToSensorEvent;
using ::android::hardware::sensors::V2_0::implementation::ISensorsSubHal;
using ::android::hardware::sensors::V2_0::subhal::implementation::SensorsSubHal;

namespace android {
namespace hardware {
namespace sensors {
namespace V2_0 {
namespace subhal {
namespace implementation {

using ::android::hardware::Void;
using ::android::hardware::sensors::V2_0::implementation::ScopedWakelock;

Result ResultFromStatus(status_t err) {
    switch (err) {
        case OK:
            return Result::OK;
        case PERMISSION_DENIED:
            return Result::PERMISSION_DENIED;
        case NO_MEMORY:
            return Result::NO_MEMORY;
        case BAD_VALUE:
            return Result::BAD_VALUE;
        default:
            return Result::INVALID_OPERATION;
    }
}

SensorsSubHal::SensorsSubHal() : mCallback(nullptr) {
    status_t err = OK;

    err = hw_get_module(SENSORS_HARDWARE_MODULE_ID, (const hw_module_t**)&mSensorModule);
    if (mSensorModule == nullptr) {
        err = UNKNOWN_ERROR;
    }

    if (err != OK) {
        LOG(ERROR) << "Couldn't load " << SENSORS_HARDWARE_MODULE_ID << " module ("
                   << strerror(-err) << ")";
        return;
    }

    err = sensors_open_1(&mSensorModule->common, &mSensorDevice);
    if (err != OK) {
        LOG(ERROR) << "Couldn't open device for module " << SENSORS_HARDWARE_MODULE_ID << " ("
                   << strerror(-err) << ")";
        return;
    }

    // Require all the old HAL APIs to be present except for injection, which
    // is considered optional.
    CHECK_GE(getHalDeviceVersion(), SENSORS_DEVICE_API_VERSION_1_3);

    if (getHalDeviceVersion() == SENSORS_DEVICE_API_VERSION_1_4) {
        if (mSensorDevice->inject_sensor_data == nullptr) {
            LOG(ERROR) << "HAL specifies version 1.4, but does not implement inject_sensor_data()";
        }
        if (mSensorModule->set_operation_mode == nullptr) {
            LOG(ERROR) << "HAL specifies version 1.4, but does not implement set_operation_mode()";
        }
    }

    enumerateSensors();
    mPollThread = std::thread(&SensorsSubHal::pollForEvents, this);
}

int SensorsSubHal::getHalDeviceVersion() const {
    if (!mSensorDevice) {
        return -1;
    }

    return mSensorDevice->common.version;
}

bool SensorsSubHal::isWakeUpSensor(int32_t handle) {
    auto sensor = mSensors.find(handle);
    return sensor != mSensors.end() &&
           (sensor->second.flags & static_cast<uint32_t>(SensorFlagBits::WAKE_UP));
}

void SensorsSubHal::convertFromSensorEvents(size_t count, const sensors_event_t* srcArray,
                                            std::vector<Event>* dstVec, bool* wakeup) {
    for (size_t i = 0; i < count; ++i) {
        const sensors_event_t& src = srcArray[i];
        Event* dst = &(*dstVec)[i];

        convertFromSensorEvent(src, dst);
        *wakeup |= isWakeUpSensor(dst->sensorHandle);
    }
}

void SensorsSubHal::enumerateSensors() {
    sensor_t const* list;
    size_t count = mSensorModule->get_sensors_list(mSensorModule, &list);

    std::vector<SensorInfo> out;
    out.resize(count);

    for (size_t i = 0; i < count; ++i) {
        const sensor_t* src = &list[i];
        SensorInfo* dst = &out[i];

        convertFromSensor(*src, dst);
        mSensors.insert(std::make_pair(dst->sensorHandle, *dst));
    }
}

// Sensors HAL V2.0 doesn't do any form of polling.
// Instead, it's up to the Sensor HALs themselves to
// send events when they happen.
void SensorsSubHal::pollForEvents() {
    std::vector<Event> out;
    std::unique_ptr<sensors_event_t[]> data;
    int err = android::NO_ERROR;
    bool wakeup = false;

    data.reset(new sensors_event_t[kPollMaxBufferSize]);
    while (true) {
        memset(data.get(), 0, sizeof(sensors_event_t) * kPollMaxBufferSize);
        wakeup = false;

        err = mSensorDevice->poll(reinterpret_cast<sensors_poll_device_t*>(mSensorDevice),
                                  data.get(), kPollMaxBufferSize);
        if (err < 0) {
            // poll shouldn't return an error.
            // if it does, just poll again.
            continue;
        }

        const size_t count = (size_t)err;
        out.resize(count);
        convertFromSensorEvents(err, data.get(), &out, &wakeup);
        postEvents(out, wakeup);
    }
}

Return<void> SensorsSubHal::getSensorsList(ISensors::getSensorsList_cb _hidl_cb) {
    std::vector<SensorInfo> sensors;
    for (const auto& sensor : mSensors) {
        sensors.push_back(sensor.second);
    }
    _hidl_cb(sensors);
    return Void();
}

Return<Result> SensorsSubHal::setOperationMode(OperationMode mode) {
    status_t err = OK;

    if (getHalDeviceVersion() < SENSORS_DEVICE_API_VERSION_1_4 ||
        mSensorModule->set_operation_mode == nullptr) {
        return Result::INVALID_OPERATION;
    }

    err = mSensorModule->set_operation_mode((uint32_t)mode);
    if (err == OK) {
        mCurrentOperationMode = mode;
    }
    return ResultFromStatus(err);
}

Return<Result> SensorsSubHal::activate(int32_t sensorHandle, bool enabled) {
    return ResultFromStatus(mSensorDevice->activate(
            reinterpret_cast<sensors_poll_device_t*>(mSensorDevice), sensorHandle, enabled));
}

Return<Result> SensorsSubHal::batch(int32_t sensorHandle, int64_t samplingPeriodNs,
                                    int64_t maxReportLatencyNs) {
    return ResultFromStatus(mSensorDevice->batch(mSensorDevice, sensorHandle, 0, /*flags*/
                                                 samplingPeriodNs, maxReportLatencyNs));
}

Return<Result> SensorsSubHal::flush(int32_t sensorHandle) {
    return ResultFromStatus(mSensorDevice->flush(mSensorDevice, sensorHandle));
}

Return<Result> SensorsSubHal::injectSensorData(const Event& event) {
    if (getHalDeviceVersion() < SENSORS_DEVICE_API_VERSION_1_4 ||
        mSensorDevice->inject_sensor_data == nullptr) {
        return Result::INVALID_OPERATION;
    }

    sensors_event_t out;
    convertToSensorEvent(event, &out);

    return ResultFromStatus(mSensorDevice->inject_sensor_data(mSensorDevice, &out));
}

Return<void> SensorsSubHal::registerDirectChannel(const SharedMemInfo& mem,
                                                  ISensors::registerDirectChannel_cb _hidl_cb) {
    if (mSensorDevice->register_direct_channel == nullptr ||
        mSensorDevice->config_direct_report == nullptr) {
        // HAL does not support
        _hidl_cb(Result::INVALID_OPERATION, -1);
        return Void();
    }

    sensors_direct_mem_t m;
    if (!convertFromSharedMemInfo(mem, &m)) {
        _hidl_cb(Result::BAD_VALUE, -1);
        return Void();
    }

    int err = mSensorDevice->register_direct_channel(mSensorDevice, &m, -1);

    if (err < 0) {
        _hidl_cb(ResultFromStatus(err), -1);
    } else {
        int32_t channelHandle = static_cast<int32_t>(err);
        _hidl_cb(Result::OK, channelHandle);
    }
    return Void();
}

Return<Result> SensorsSubHal::unregisterDirectChannel(int32_t channelHandle) {
    if (mSensorDevice->register_direct_channel == nullptr ||
        mSensorDevice->config_direct_report == nullptr) {
        // HAL does not support
        return Result::INVALID_OPERATION;
    }

    mSensorDevice->register_direct_channel(mSensorDevice, nullptr, channelHandle);

    return Result::OK;
}

Return<void> SensorsSubHal::configDirectReport(int32_t sensorHandle, int32_t channelHandle,
                                               RateLevel rate,
                                               ISensors::configDirectReport_cb _hidl_cb) {
    if (mSensorDevice->register_direct_channel == nullptr ||
        mSensorDevice->config_direct_report == nullptr) {
        // HAL does not support
        _hidl_cb(Result::INVALID_OPERATION, -1);
        return Void();
    }

    sensors_direct_cfg_t cfg = {.rate_level = convertFromRateLevel(rate)};
    if (cfg.rate_level < 0) {
        _hidl_cb(Result::BAD_VALUE, -1);
        return Void();
    }

    int err = mSensorDevice->config_direct_report(mSensorDevice, sensorHandle, channelHandle, &cfg);

    if (rate == RateLevel::STOP) {
        _hidl_cb(ResultFromStatus(err), -1);
    } else {
        _hidl_cb(err > 0 ? Result::OK : ResultFromStatus(err), err);
    }
    return Void();
}

Return<void> SensorsSubHal::debug(const hidl_handle& fd, const hidl_vec<hidl_string>& args) {
    if (fd.getNativeHandle() == nullptr || fd->numFds < 1) {
        LOG(ERROR) << "missing fd for writing";
        return Void();
    }

    FILE* out = fdopen(dup(fd->data[0]), "w");

    if (args.size() != 0) {
        fprintf(out,
                "Note: sub-HAL %s currently does not support args. Input arguments are "
                "ignored.\n",
                getName().c_str());
    }

    std::ostringstream stream;
    stream << "Available sensors:" << std::endl;
    for (auto sensor : mSensors) {
        SensorInfo info = sensor.second;
        stream << "Name: " << info.name << std::endl;
        stream << "Min delay: " << info.minDelay << std::endl;
        stream << "Flags: " << info.flags << std::endl;
    }
    stream << std::endl;

    fprintf(out, "%s", stream.str().c_str());

    fclose(out);
    return Return<void>();
}

Return<Result> SensorsSubHal::initialize(const sp<IHalProxyCallback>& halProxyCallback) {
    mCallback = halProxyCallback;
    setOperationMode(OperationMode::NORMAL);
    return Result::OK;
}

void SensorsSubHal::postEvents(const std::vector<Event>& events, bool wakeup) {
    ScopedWakelock wakelock = mCallback->createScopedWakelock(wakeup);
    mCallback->postEvents(events, std::move(wakelock));
}

}  // namespace implementation
}  // namespace subhal
}  // namespace V2_0
}  // namespace sensors
}  // namespace hardware
}  // namespace android

ISensorsSubHal* sensorsHalGetSubHal(uint32_t* version) {
    static SensorsSubHal subHal;
    *version = SUB_HAL_2_0_VERSION;
    return &subHal;
}
