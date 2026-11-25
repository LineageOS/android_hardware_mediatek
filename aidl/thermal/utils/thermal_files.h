/*
 * Copyright (C) 2022 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <string>
#include <unordered_map>

#include "thermal_info.h"

namespace aidl {
namespace android {
namespace hardware {
namespace thermal {
namespace implementation {

struct PathInfo {
    std::string path = "";
    TempPathType temp_path_type = TempPathType::SYSFS;
};

class ThermalFiles {
  public:
    ThermalFiles() = default;
    ~ThermalFiles() = default;
    ThermalFiles(const ThermalFiles &) = delete;
    void operator=(const ThermalFiles &) = delete;

    PathInfo getThermalFilePath(std::string_view thermal_name) const;
    // Returns true if add was successful, false otherwise.
    bool addThermalFile(std::string_view thermal_name, std::string_view path,
                        TempPathType temp_path_type = TempPathType::SYSFS);
    // If thermal_name is not found in the thermal names to path map, this will set
    // data to empty and return false. If the thermal_name is found and its content
    // is read, this function will fill in data accordingly then return true.
    bool readThermalFile(std::string_view thermal_name, std::string *data) const;
    bool writeCdevFile(std::string_view thermal_name, std::string_view data);
    bool writeCdevFile(std::string_view thermal_name, std::string_view data, int *error);
    size_t getNumThermalFiles() const { return thermal_name_to_path_map_.size(); }

  private:
    std::unordered_map<std::string, PathInfo> thermal_name_to_path_map_;
};

}  // namespace implementation
}  // namespace thermal
}  // namespace hardware
}  // namespace android
}  // namespace aidl
