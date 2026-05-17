//
// SPDX-FileCopyrightText: 2022 The Android Open Source Project
// SPDX-FileCopyrightText: The LineageOS Project
// SPDX-License-Identifier: Apache-2.0
//

#include <android-base/logging.h>

#include "MtkBootRegionControl.h"
#include "emmc-bootpart.h"
#include "ufs-bsg.h"

namespace aidl::android::hardware::boot {

MtkBootRegionControl::MtkBootRegionControl() {
    if (access(UFS_BSG0_DEV, F_OK) == 0)
        mSetBootPartCb = ufs_set_boot_part;
    else if (access(MMCBLK0_DEV, F_OK) == 0)
        mSetBootPartCb = emmc_set_boot_part;
    else {
        LOG(FATAL) << "Did not find UFS or eMMC device";
        abort();
    }
}

bool MtkBootRegionControl::SetActiveBootSlot(int32_t in_slot) {
    return mSetBootPartCb(in_slot) == 0;
}

}  // namespace aidl::android::hardware::boot
