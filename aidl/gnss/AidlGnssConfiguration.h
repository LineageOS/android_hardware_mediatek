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

#include <aidl/android/hardware/gnss/BnGnssConfiguration.h>
#include <vector>
#include <mediatek/gps_mtk.h>

namespace aidl::android::hardware::gnss {
using std::vector;

struct AidlGnssConfiguration : public BnGnssConfiguration {
  public:
    AidlGnssConfiguration(const GnssConfigurationInterface_ext* halConfigurationIface);
    ~AidlGnssConfiguration();

    ndk::ScopedAStatus setSuplVersion(int) override;

    ndk::ScopedAStatus setSuplMode(int) override;

    ndk::ScopedAStatus setLppProfile(int) override;

    ndk::ScopedAStatus setGlonassPositioningProtocol(int) override;

    ndk::ScopedAStatus setEmergencySuplPdn(bool) override;

    ndk::ScopedAStatus setEsExtensionSec(int) override;

    ndk::ScopedAStatus setBlocklist(const vector<BlocklistedSource>& blocklist) override;

  private:
    const GnssConfigurationInterface_ext* mGnssHalConfigIface = nullptr;

};

}  // namespace aidl::android::hardware::gnss
