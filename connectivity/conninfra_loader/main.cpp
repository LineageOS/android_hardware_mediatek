/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "conninfra_loader"

#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/stringprintf.h>
#include <android-base/unique_fd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

using android::base::GetProperty;
using android::base::SetProperty;
using android::base::StringPrintf;
using android::base::unique_fd;

namespace {

constexpr char kConninfraDev[] = "/dev/conninfra_dev";

constexpr char kChipIdProp[] = "persist.vendor.connsys.chipid";
constexpr char kDriverReadyProp[] = "vendor.connsys.driver.ready";

constexpr unsigned kIocMagic = 0xc2;
#define CONNINFRA_IOCTL_GET_CHIP_ID _IOR(kIocMagic, 0, int)
#define CONNINFRA_IOCTL_DO_MODULE_INIT _IOR(kIocMagic, 2, int)
#define CONNINFRA_IOCTL_GET_ADIE_CHIP_ID _IOR(kIocMagic, 3, int)

struct Radio {
    const char* name;
    const char* prop;
};

// Index == conn_adaptor_drv_type used by the driver.
constexpr std::array<Radio, 4> kRadios{{
        {"WIFI", "vendor.connsys.wifi.adie.chipid"},
        {"BT", "vendor.connsys.bt.adie.chipid"},
        {"GPS", "vendor.connsys.gps.adie.chipid"},
        {"FM", "vendor.connsys.fm.adie.chipid"},
}};
constexpr size_t kGpsIndex = 2;

// The driver returns the ID as the ioctl return value; <= 0 means failure.
int QueryId(int fd, unsigned long request, unsigned long arg = 0) {
    return ioctl(fd, request, arg);
}

void SetHexProperty(const char* prop, int id) {
    const std::string value = StringPrintf("0x%04x", id);
    if (SetProperty(prop, value)) {
        LOG(INFO) << "Set " << prop << " to " << value;
    } else {
        LOG(ERROR) << "Failed to set " << prop << " to " << value;
    }
}

unique_fd OpenConninfra() {
    for (int attempt = 1;; ++attempt) {
        unique_fd fd(open(kConninfraDev, O_RDWR | O_NOCTTY | O_CLOEXEC));
        if (fd.get() >= 0) {
            return fd;
        }
        if (attempt == 1 || attempt % 20 == 0) {  // ~6 s between logs
            LOG(INFO) << "Can't open " << kConninfraDev << " (attempt " << attempt
                      << "): " << strerror(errno);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}

void DoModuleInit(int fd) {
    if (int ret = ioctl(fd, CONNINFRA_IOCTL_DO_MODULE_INIT, 0); ret != 0) {
        PLOG(ERROR) << "Kernel module init failed, ret=" << ret;
    }
}

void SetChipId(int fd) {
    const int id = QueryId(fd, CONNINFRA_IOCTL_GET_CHIP_ID);
    if (id <= 0) {
        PLOG(ERROR) << "Get chip id failed, ret=" << id;
        return;
    }
    LOG(INFO) << StringPrintf("Chip id: 0x%x", id);
    SetHexProperty(kChipIdProp, id);
}

void SetAdieChipIds(int fd) {
    for (size_t i = 0; i < kRadios.size(); ++i) {
        const int id = QueryId(fd, CONNINFRA_IOCTL_GET_ADIE_CHIP_ID, i);
        if (id <= 0) {
            PLOG(ERROR) << "Get " << kRadios[i].name << " a-die chip id failed, ret=" << id;
            continue;
        }
        SetHexProperty(kRadios[i].prop, id);
    }
}

void SetDriverReady() {
    if (GetProperty(kDriverReadyProp, "") == "yes") {
        LOG(INFO) << kDriverReadyProp << " was already set";
    }
    if (!SetProperty(kDriverReadyProp, "yes")) {
        LOG(ERROR) << "Failed to set " << kDriverReadyProp;
    }
}

}  // namespace

int main() {
    unique_fd fd = OpenConninfra();

    DoModuleInit(fd.get());
    SetChipId(fd.get());
    SetAdieChipIds(fd.get());
    SetDriverReady();

    return EXIT_SUCCESS;
}
