/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include <android-base/properties.h>
#include <android-base/stringprintf.h>
#include <cutils/android_filesystem_config.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <iterator>
#include <string>
#include <thread>

#include "wmt.h"

using android::base::GetProperty;
using android::base::SetProperty;
using android::base::StringPrintf;
using android::base::unique_fd;
using std::chrono::milliseconds;

namespace {

constexpr char kDevice[] = "/dev/wmtdetect";
constexpr const char* kProcPaths[] = {"/proc/driver/wmt_dbg", "/proc/driver/wmt_aee"};
constexpr const char* kAdiePerRadioProps[] = {
        "vendor.connsys.wifi.adie.chipid",
        "vendor.connsys.bt.adie.chipid",
        "vendor.connsys.gps.adie.chipid",
        "vendor.connsys.fm.adie.chipid",
};

constexpr unsigned kIocMagic = 'w';
constexpr unsigned long kIocGetChipId = _IOR(kIocMagic, 0, int);
constexpr unsigned long kIocSetChipId = _IOW(kIocMagic, 1, int);
constexpr unsigned long kIocExtChipDetect = _IOR(kIocMagic, 2, int);
constexpr unsigned long kIocGetSocChipId = _IOR(kIocMagic, 3, int);
constexpr unsigned long kIocDoModuleInit = _IOR(kIocMagic, 4, int);
constexpr unsigned long kIocModuleCleanup = _IOR(kIocMagic, 5, int);
constexpr unsigned long kIocExtChipPwrOn = _IOR(kIocMagic, 6, int);
constexpr unsigned long kIocExtChipPwrOff = _IOR(kIocMagic, 7, int);
constexpr unsigned long kIocDoSdioAutok = _IOR(kIocMagic, 8, int);
constexpr unsigned long kIocGetAdieChipId = _IOR(kIocMagic, 9, int);
constexpr unsigned long kIocConnsysSocHwInit = _IOR(kIocMagic, 10, int);

constexpr int kKnownChipIds[] = {
        0x6620, 0x6628, 0x6630, 0x6632, 0x6572, 0x6582, 0x6592, 0x8127, 0x6571, 0x6752, 0x6735,
        0x0321, 0x0335, 0x0337, 0x8163, 0x6580, 0x6755, 0x0326, 0x6797, 0x0279, 0x6757, 0x0551,
        0x8167, 0x6759, 0x0507, 0x6763, 0x0690, 0x6570, 0x0713, 0x6775, 0x0788, 0x6771, 0x6765,
        0x3967, 0x6761, 0x6779, 0x6768, 0x6785, 0x6873, 0x8168, 0x6853, 0x6833, 0x6781, 0x6855,
};

constexpr int kAutokRetries = 10;
constexpr int kDetectRetries = 20;
constexpr milliseconds kRetryDelay{500};

bool IsKnownChipId(int id) {
    return std::find(std::begin(kKnownChipIds), std::end(kKnownChipIds), id) !=
           std::end(kKnownChipIds);
}

bool SetHexProperty(const char* prop, int id) {
    const std::string value = StringPrintf("0x%04x", id);
    if (!SetProperty(prop, value)) {
        LOG(ERROR) << "Failed to set " << prop << " to " << value;
        return false;
    }
    LOG(INFO) << "Set " << prop << " to " << value;
    return true;
}

void RunSdioAutok(int fd, int chip_id) {
    if (Ioctl(fd, kIocDoSdioAutok, chip_id) != 0) {
        LOG(ERROR) << "SDIO3.0 autok failed";
    } else {
        LOG(INFO) << "SDIO3.0 autok succeeded";
    }
}

void PowerOffExtChip(int fd) {
    if (Ioctl(fd, kIocExtChipPwrOff) != 0) {
        LOG(INFO) << "External combo chip power off failed";
    } else {
        LOG(INFO) << "External combo chip power off succeeded";
    }
}

// Known combo chip (6630/6632): re-run autok in case the last one was interrupted
// by an abnormal power off or a battery drop.
void RunAutokForKnownChip(int fd, int chip_id) {
    LOG(INFO) << StringPrintf("Chip id 0x%x, triggering autok", chip_id);

    for (int retries_left = kAutokRetries; retries_left > 0;) {
        bool ext_chip = false;
        if (Ioctl(fd, kIocExtChipPwrOn) != 0) {
            LOG(ERROR) << "External combo chip power on failed";
        } else {
            ext_chip = Ioctl(fd, kIocExtChipDetect) == 0;
        }

        const int detected_id = Ioctl(fd, kIocGetChipId);
        LOG(INFO) << StringPrintf("chipid (0x%x) detected, ext chip: %d", detected_id, ext_chip);

        if (ext_chip) {
            RunSdioAutok(fd, detected_id);
        }
        PowerOffExtChip(fd);

        if (!(ext_chip && detected_id == -1)) {
            break;
        }
        // External chip present but no valid chip id: retry.
        --retries_left;
        LOG(ERROR) << "chipId detect failed, retries left: " << retries_left;
        std::this_thread::sleep_for(kRetryDelay);
    }
}

// First boot / unknown chip: detect it and persist the ID. Returns -1 on failure.
int DetectChipId(int fd) {
    int chip_id = -1;

    for (int retries_left = kDetectRetries; retries_left > 0;) {
        bool ext_chip = false;

        const int ret = Ioctl(fd, kIocExtChipPwrOn);
        if (ret == -1) {
            LOG(INFO) << "SoC chip, no combo chip power on needed";
            chip_id = Ioctl(fd, kIocGetSocChipId);
        } else if (ret == 0) {
            ext_chip = true;
            if (const int detect = Ioctl(fd, kIocExtChipDetect); detect != 0) {
                LOG(ERROR) << "External combo chip detect failed (" << detect << ")";
            } else {
                chip_id = Ioctl(fd, kIocGetChipId);
            }
        } else {
            LOG(ERROR) << "External combo chip power on failed (" << ret << ")";
        }

        LOG(INFO) << StringPrintf("chipid (0x%x) detected", chip_id);
        if (chip_id != -1) {
            SetHexProperty(kChipIdProp, chip_id);
        }

        if (ext_chip) {
            RunSdioAutok(fd, chip_id);
            PowerOffExtChip(fd);
        }

        if (!(ext_chip && chip_id == -1)) {
            break;
        }
        // External chip present but no valid chip id: retry.
        if (--retries_left > 0) {
            LOG(ERROR) << "chipId detect failed, retries left: " << retries_left;
            std::this_thread::sleep_for(kRetryDelay);
        }
    }
    return chip_id;
}

bool DoKernelModuleInit(int fd, int chip_id) {
    if (int ret = Ioctl(fd, kIocModuleCleanup, chip_id); ret != 0) {
        LOG(ERROR) << "WMT-DETECT module cleanup failed: " << ret;
        return false;
    }
    if (int ret = Ioctl(fd, kIocDoModuleInit, chip_id); ret != 0) {
        LOG(ERROR) << "Kernel module init failed: " << ret;
        return false;
    }
    LOG(INFO) << "Kernel module init succeeded";
    return true;
}

// The driver returns 0 if dynamic a-die detection is unsupported, -1 if no a-die is present.
bool SetAdieChipIds(int fd) {
    const int id = Ioctl(fd, kIocGetAdieChipId);
    LOG(INFO) << StringPrintf("a-die chip id = 0x%x", id);
    if (id == -1) {
        return false;
    }

    if (!SetHexProperty(kAdieChipIdProp, id)) {
        return false;
    }
    for (const char* prop : kAdiePerRadioProps) {
        SetHexProperty(prop, id);  // logged on failure, not fatal
    }
    return true;
}

bool SetDriverReady() {
    if (!SetProperty(kDriverReadyProp, "yes")) {
        LOG(ERROR) << "Failed to set " << kDriverReadyProp;
        return false;
    }
    LOG(INFO) << "Set " << kDriverReadyProp << " to yes";
    return true;
}

}  // namespace

bool RunLoader() {
    unique_fd fd = OpenDeviceBlocking(kDevice);

    if (int ret = Ioctl(fd.get(), kIocConnsysSocHwInit); ret != 0) {
        LOG(ERROR) << StringPrintf("CONNSYS_SOC_HW_INIT failed (0x%x)", ret);
        return false;
    }

    int chip_id = -1;
    const std::string stored = GetProperty(kChipIdProp, "");
    const int stored_id = static_cast<int>(strtoul(stored.c_str(), nullptr, 16));
    LOG(INFO) << "Chip id from property: " << stored << " => " << stored_id;

    if (!stored.empty() && IsKnownChipId(stored_id)) {
        chip_id = stored_id;
        if (chip_id == 0x6630 || chip_id == 0x6632) {
            RunAutokForKnownChip(fd.get(), chip_id);
        }
    } else {
        chip_id = DetectChipId(fd.get());
    }

    if (chip_id == -1) {
        LOG(ERROR) << "Chip id error, aborting";
        return false;
    }

    if (Ioctl(fd.get(), kIocSetChipId, chip_id) != 0) {
        LOG(ERROR) << "Failed to set chip id in kernel";
    }

    // The kernel keeps the raw ID above; the module init wants the platform ID.
    chip_id = NormalizeChipId(chip_id);

    DoKernelModuleInit(fd.get(), chip_id);

    if (!SetAdieChipIds(fd.get())) {
        LOG(ERROR) << "A-die chip id error";
        return false;
    }
    fd.reset();

    for (const char* path : kProcPaths) {
        if (chown(path, AID_SHELL, AID_SYSTEM) == -1) {
            PLOG(ERROR) << "chown " << path << " failed";
        }
    }

    return SetDriverReady();
}
