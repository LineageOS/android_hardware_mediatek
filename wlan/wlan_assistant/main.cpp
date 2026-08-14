//
// SPDX-FileCopyrightText: The LineageOS Project
// SPDX-License-Identifier: Apache-2.0
//

#define LOG_TAG "WLAN-ASSISTANT"

#include <linux/fs.h>
#include <linux/inotify.h>
#include <sys/inotify.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <string.h>
#include <unistd.h>

#include <string>
#include <vector>

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/strings.h>

#define MAX_WAIT_SECOND LONG_MAX
#define MAX_RETRY_COUNT 5
#define NVRAM_MAC_ADDRESS_OFFSET 4
#define WIFI_LOADER_DEV "/dev/wmtWifi"
#define WIFI_NVRAM_PATH "/mnt/vendor/nvdata/APCFG/APRDEB"
#define WIFI_NVRAM_INI_FILE "/data/vendor/nvramwifi"
#define WIFI_MACADDR_FILE "/data/vendor/macwifi"

#define BOOT_META_STR "meta"

#define FILE_REMOVE_MASK (IN_DELETE_SELF | IN_MOVE_SELF)
#define FILE_MODIFY_MASK IN_MODIFY
#define WATCH_FILE_MASK (FILE_REMOVE_MASK | FILE_MODIFY_MASK)
#define WATCH_PATH_MASK (IN_MOVED_TO | IN_CREATE)

#define NVRAM_CHANGED 4

#define INOT_BUF_SIZE (4 * (sizeof(struct inotify_event) + NAME_MAX + 1))

using android::base::GetProperty;
using android::base::ReadFileToString;
using android::base::SetProperty;
using android::base::Trim;
using android::base::WriteStringToFile;

bool write_data_to_driver(const std::string& data) {
    if (data.empty()) return false;

    if (!WriteStringToFile(data, WIFI_LOADER_DEV)) {
        LOG(DEBUG) << "write data to driver failed reason is " << strerror(errno);
        return false;
    }

    return true;
}

bool get_custom_mac_address(std::vector<char>& mac) {
    std::string mac_str;
    if (!ReadFileToString(WIFI_MACADDR_FILE, &mac_str)) {
        LOG(DEBUG) << "Unable to access mac file";
        return false;
    }

    mac_str = Trim(mac_str);
    if (mac_str.length() >= 17) {
        LOG(DEBUG) << "MAC ADDR = " << mac_str;
        mac.resize(6);
        for (int i = 0; i < 6; i++) {
            mac[i] = static_cast<char>(std::strtol(mac_str.c_str() + i * 3, nullptr, 16));
            LOG(DEBUG) << "mac[" << i << "] = " << std::hex << (int)mac[i];
        }
        return true;
    }

    return false;
}

bool write_nvram(const std::string& filename) {
    struct stat stat_nvram;
    int nvram_size = 0;

    /* sleep 1 more second in case that daemon is still writing */
    for (int i = 0; i < MAX_RETRY_COUNT; i++) {
        if (stat(filename.c_str(), &stat_nvram) == -1) {
            LOG(DEBUG) << "stat " << filename << " error";
            sleep(1);
            continue;
        }
        nvram_size = stat_nvram.st_size - 2;

        LOG(DEBUG) << "nvram size = " << nvram_size;
        if (nvram_size > 0 && (nvram_size & 0x0ff) == 0) break;
        sleep(1);
    }

    if (nvram_size <= 0 || (nvram_size & 0x0ff) != 0) {
        LOG(DEBUG) << "invalid nvram size";
        return false;
    }

    std::string nvram_data;
    if (!ReadFileToString(filename, &nvram_data)) {
        LOG(DEBUG) << "Error opening nvram file";
        return false;
    }

    if (nvram_data.length() < static_cast<size_t>(nvram_size)) {
        LOG(DEBUG) << "short read on nvram file (" << nvram_data.length() << " of " << nvram_size
                   << ")";
        return false;
    }

    std::string acnvram = "WR-BUF:NVRAM";
    acnvram += nvram_data.substr(0, nvram_size);

    std::vector<char> mac;
    if (get_custom_mac_address(mac)) {
        for (size_t i = 0; i < mac.size(); i++) {
            acnvram[12 + NVRAM_MAC_ADDRESS_OFFSET + i] = mac[i];
        }
    }

    if (!write_data_to_driver(acnvram)) {
        LOG(DEBUG) << "write nvram to driver error";
        return false;
    } else {
        SetProperty("vendor.mtk.nvram.ready", "1");
    }

    return true;
}

int file_event_hander(int inot_fd, int wd, struct inotify_event* event, const std::string& path,
                      const std::string& file_name) {
    std::string file_path = path + "/" + file_name;

    if ((event->mask & WATCH_PATH_MASK) && event->name == file_name) {
        inotify_rm_watch(inot_fd, wd);
        wd = inotify_add_watch(inot_fd, file_path.c_str(), WATCH_FILE_MASK);
    } else if (event->mask & FILE_REMOVE_MASK) {
        inotify_rm_watch(inot_fd, wd);
        if (access(file_path.c_str(), R_OK) >= 0)
            wd = inotify_add_watch(inot_fd, file_path.c_str(), WATCH_FILE_MASK);
        else
            wd = inotify_add_watch(inot_fd, path.c_str(), WATCH_PATH_MASK);
    }
    return wd;
}

std::string get_custom_nvram_file_name() {
    std::string filename = std::string(WIFI_NVRAM_PATH) + "/";
    std::string buf;

    if (ReadFileToString(WIFI_NVRAM_INI_FILE, &buf) && !buf.empty()) {
        filename += Trim(buf);
    } else {
        filename += "WIFI";
    }

    LOG(DEBUG) << "custom nvram filename = " << filename;
    return filename;
}

void* wlan_files_monitor(void* /* pdata */) {
    LOG(DEBUG) << "Running wlan_files_monitor";

    LOG(DEBUG) << "Start to wait wmtWifi ready";
    while (access(WIFI_LOADER_DEV, R_OK | W_OK) < 0) usleep(100000);

    struct stat stat_buf;
    if (stat(WIFI_LOADER_DEV, &stat_buf) == -1) {
        LOG(DEBUG) << "stat WIFI_LOADER_DEV fail";
        return nullptr;
    }
    if (!S_ISCHR(stat_buf.st_mode)) {
        LOG(DEBUG) << WIFI_LOADER_DEV << " is not a char device";
        return nullptr;
    }

    int inot_fd = inotify_init();
    if (inot_fd < 0) {
        LOG(DEBUG) << "inotify_init error";
        return nullptr;
    }
    int dev_wd = inotify_add_watch(inot_fd, WIFI_LOADER_DEV, FILE_REMOVE_MASK);

    std::string nvram_filename = get_custom_nvram_file_name();
    std::string just_filename = nvram_filename.substr(strlen(WIFI_NVRAM_PATH) + 1);

    LOG(DEBUG) << "nvram_path_file = " << nvram_filename << ", nvrma_file = " << just_filename;

    LOG(DEBUG) << "Start to check if NVRAM existed";
    while (access(nvram_filename.c_str(), R_OK) < 0) sleep(1);

    int nvram_wd = inotify_add_watch(inot_fd, nvram_filename.c_str(), WATCH_FILE_MASK);
    write_nvram(nvram_filename);

    std::string boot_mode = GetProperty("ro.bootmode", "-1");
    LOG(DEBUG) << "read boot mode:" << boot_mode;

    bool running = true;
    if (boot_mode.find(BOOT_META_STR) == 0) {
        running = false;
    }

    // Force memory alignment for inotify_event struct casting
    char buf[INOT_BUF_SIZE] __attribute__((aligned(__alignof__(struct inotify_event))));
    long timeout_sec = MAX_WAIT_SECOND;
    int changed = 0;

    while (running) {
        struct timeval timeout;
        timeout.tv_sec = timeout_sec;
        timeout.tv_usec = 0;

        fd_set inot_fd_set;
        FD_ZERO(&inot_fd_set);
        FD_SET(inot_fd, &inot_fd_set);

        int readlen = select(inot_fd + 1, &inot_fd_set, nullptr, nullptr, &timeout);
        if (readlen < 0 && errno == EINTR) continue;

        if (!readlen) {
            LOG(DEBUG) << "will sync to driver, changed " << std::hex << changed;
            if (changed & NVRAM_CHANGED) write_nvram(nvram_filename);
            changed = 0;
            timeout_sec = MAX_WAIT_SECOND;
            continue;
        }

        ssize_t readbytes = read(inot_fd, buf, sizeof(buf));
        ssize_t offset = 0;

        while (readbytes > offset) {
            struct inotify_event* event = reinterpret_cast<struct inotify_event*>(&buf[offset]);
            offset += sizeof(struct inotify_event) + event->len;

            if (event->mask & IN_IGNORED) continue;

            if (event->wd == dev_wd && (event->mask & FILE_REMOVE_MASK)) {
                LOG(DEBUG) << "/dev/wmtWifi was removed, need to exit";
                running = false;
                break;
            }

            if (event->wd == nvram_wd) {
                nvram_wd =
                        file_event_hander(inot_fd, nvram_wd, event, WIFI_NVRAM_PATH, just_filename);
                changed |= NVRAM_CHANGED;
            }
        }
        if (changed > 0) timeout_sec = 1;
    }

    if (dev_wd > 0) inotify_rm_watch(inot_fd, dev_wd);
    if (nvram_wd > 0) inotify_rm_watch(inot_fd, nvram_wd);
    close(inot_fd);

    return nullptr;
}

int main(int /* argc */, char** /* argv */) {
    wlan_files_monitor(nullptr);
    return 0;
}
