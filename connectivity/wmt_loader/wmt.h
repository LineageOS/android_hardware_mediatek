/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <android-base/logging.h>
#include <android-base/unique_fd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <chrono>
#include <thread>

// Properties used by both phases.
inline constexpr char kChipIdProp[] = "persist.vendor.connsys.chipid";
inline constexpr char kAdieChipIdProp[] = "vendor.connsys.adie.chipid";
inline constexpr char kDriverReadyProp[] = "vendor.connsys.driver.ready";

struct LauncherOptions {
    int default_on = 0;  // -o: > 0 powers connsys on in mode 2, otherwise mode 1
};

// Loader phase: detect the chip, init the kernel modules, publish the IDs, set driver.ready.
bool RunLoader();

// Launcher phase: configure /dev/stpwmt and serve the kernel's patch requests until killed.
int RunLauncher(const LauncherOptions& options);

inline int Ioctl(int fd, unsigned long request, int arg = 0) {
    return ioctl(fd, request, static_cast<unsigned long>(arg));
}

template <typename T>
inline int IoctlPtr(int fd, unsigned long request, T* arg) {
    return ioctl(fd, request, arg);
}

// Some chips report a legacy ID that maps to the platform ID used by the kernel modules
// and the firmware patch names.
inline int NormalizeChipId(int id) {
    struct Alias {
        int from;
        int to;
    };
    static constexpr Alias kAliases[] = {
            {0x0321, 0x6735}, {0x0335, 0x6735}, {0x0337, 0x6735}, {0x0326, 0x6755},
            {0x0279, 0x6797}, {0x0551, 0x6757}, {0x0633, 0x6570}, {0x0690, 0x6763},
            {0x0507, 0x6759}, {0x0699, 0x6739}, {0x0713, 0x6775}, {0x0788, 0x6771},
    };
    for (const Alias& alias : kAliases) {
        if (alias.from == id) {
            return alias.to;
        }
    }
    return id;
}

// Waits for a device node to show up, then opens it.
inline android::base::unique_fd OpenDeviceBlocking(const char* path) {
    for (int attempt = 1;; ++attempt) {
        android::base::unique_fd fd(open(path, O_RDWR | O_NOCTTY | O_CLOEXEC));
        if (fd.get() >= 0) {
            return fd;
        }
        if (attempt == 1 || attempt % 20 == 0) {  // ~6 s between logs
            PLOG(INFO) << "Can't open " << path << " (attempt " << attempt << ")";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}
