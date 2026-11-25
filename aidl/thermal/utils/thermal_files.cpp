/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define ATRACE_TAG (ATRACE_TAG_THERMAL | ATRACE_TAG_HAL)

#include "thermal_files.h"

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/stringprintf.h>
#include <android-base/strings.h>
#include <utils/Trace.h>

#include <cerrno>
#include <string_view>

namespace aidl {
namespace android {
namespace hardware {
namespace thermal {
namespace implementation {

using ::android::base::StringPrintf;

constexpr std::string_view kDefaultFileValue("0");

PathInfo ThermalFiles::getThermalFilePath(std::string_view thermal_name) const {
    auto sensor_itr = thermal_name_to_path_map_.find(thermal_name.data());
    if (sensor_itr == thermal_name_to_path_map_.end()) {
        return PathInfo();
    }
    return sensor_itr->second;
}

bool ThermalFiles::addThermalFile(std::string_view thermal_name, std::string_view path,
                                  TempPathType temp_path_type) {
    return thermal_name_to_path_map_
            .emplace(thermal_name,
                     PathInfo{
                             .path = std::string(path),
                             .temp_path_type = temp_path_type,
                     })
            .second;
}

bool ThermalFiles::readThermalFile(std::string_view thermal_name, std::string *data) const {
    std::string sensor_reading;
    const auto path_info = getThermalFilePath(thermal_name);
    *data = "";

    ATRACE_NAME(StringPrintf("ThermalFiles::readThermalFile - %s", thermal_name.data()).c_str());
    if (path_info.path.empty()) {
        PLOG(WARNING) << "Failed to find " << thermal_name << "'s path";
        return false;
    }

    if (path_info.temp_path_type == TempPathType::SYSFS) {
        if (!::android::base::ReadFileToString(path_info.path, &sensor_reading)) {
            PLOG(WARNING) << "Failed to read sensor: " << thermal_name;
            return false;
        }

        if (sensor_reading.size() <= 1) {
            LOG(ERROR) << thermal_name << "'s return size:" << sensor_reading.size()
                       << " is invalid";
            return false;
        }
    } else if (path_info.temp_path_type == TempPathType::DEVICE_PROPERTY) {
        sensor_reading = ::android::base::GetProperty(path_info.path, kDefaultFileValue.data());
    } else {
        LOG(ERROR) << "Unsupported temp path type: "
                   << static_cast<std::underlying_type<TempPathType>::type>(
                              path_info.temp_path_type);
        return false;
    }

    // Strip the newline.
    *data = ::android::base::Trim(sensor_reading);
    return true;
}

bool ThermalFiles::writeCdevFile(std::string_view cdev_name, std::string_view data) {
    return writeCdevFile(cdev_name, data, nullptr);
}

bool ThermalFiles::writeCdevFile(std::string_view cdev_name, std::string_view data, int *error) {
    const auto path_info =
            getThermalFilePath(::android::base::StringPrintf("%s_%s", cdev_name.data(), "w"));

    ATRACE_NAME(StringPrintf("ThermalFiles::writeCdevFile - %s", cdev_name.data()).c_str());
    if (error) {
        *error = 0;
    }
    errno = 0;
    if (!::android::base::WriteStringToFile(data.data(), path_info.path)) {
        const int write_errno = errno;
        if (error) {
            *error = write_errno;
        }
        PLOG(WARNING) << "Failed to write cdev: " << cdev_name << " to " << data.data();
        return false;
    }

    return true;
}

}  // namespace implementation
}  // namespace thermal
}  // namespace hardware
}  // namespace android
}  // namespace aidl
