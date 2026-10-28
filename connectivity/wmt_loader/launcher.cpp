/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include <android-base/file.h>
#include <android-base/properties.h>
#include <android-base/stringprintf.h>
#include <android-base/strings.h>
#include <android-base/unique_fd.h>
#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <strings.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

#include "wmt.h"
#include "wmt_ioctl.h"

using android::base::GetProperty;
using android::base::ReadFileToString;
using android::base::StringPrintf;
using android::base::Trim;
using android::base::unique_fd;
using std::chrono::milliseconds;

namespace {

constexpr char kWmtDev[] = "/dev/stpwmt";
constexpr char kFirmwareDir[] = "/vendor/firmware";
constexpr char kWmtCfgFile[] = "/vendor/firmware/WMT.cfg";

constexpr char kFwLogProp[] = "persist.vendor.connsys.fwlog.status";
constexpr char kDynamicDumpProp[] = "persist.vendor.connsys.dynamic.dump";
constexpr char kPatchVersionProp[] = "persist.vendor.connsys.patch.version";
constexpr char kBtVersionProp[] = "vendor.connsys.bt_fw_ver";
constexpr char kWifiVersionProp[] = "persist.vendor.connsys.wifi_fw_ver";

constexpr int kStpBtifFull = 3;
constexpr int kFmMode = 2;
constexpr int kComboBaudRate = 4000000;

constexpr size_t kPropValueMax = 92;  // PROPERTY_VALUE_MAX
constexpr size_t kVersionLen = 16;
constexpr size_t kMaxBtPatchSize = 1024 * 1024;
constexpr int kFwDebugRetryLimit = 5;
constexpr int kPowerOnRetries = 20;

struct StpPatchInfo {
    int download_seq;
    char address[4];
    char patch_name[NAME_MAX + 1];
};

struct StpRomPatchInfo {
    int type;
    char address[4];
    char patch_name[NAME_MAX + 1];
};

struct PrefixEntry {
    int chip_id;
    const char* prefix;
};

constexpr PrefixEntry kRomPatchPrefixes[] = {
        {0x6572, "ROMv1"},    {0x6582, "ROMv1"},    {0x6592, "ROMv1"},    {0x8127, "ROMv2"},
        {0x6571, "ROMv2"},    {0x6755, "ROMv2_lm"}, {0x6752, "ROMv2_lm"}, {0x8163, "ROMv2_lm"},
        {0x7623, "ROMv2_lm"}, {0x6735, "ROMv2_lm"}, {0x6570, "ROMv2_lm"}, {0x6580, "ROMv2_lm"},
        {0x6757, "ROMv2_lm"}, {0x6763, "ROMv2_lm"}, {0x6739, "ROMv2_lm"}, {0x8167, "ROMv2_lm"},
        {0x6797, "ROMv3"},    {0x6759, "ROMv4"},    {0x6775, "ROMv4_be"}, {0x6771, "ROMv4_be"},
        {0x6765, "soc1_0"},   {0x3967, "soc1_0"},   {0x6761, "soc1_0"},   {0x6768, "soc1_0"},
        {0x6785, "soc1_0"},   {0x8168, "soc1_0"},   {0x6779, "soc2_0"},   {0x6873, "soc2_0"},
        {0x6853, "soc2_0"},
};

// Anything not listed uses "soc1_0".
constexpr PrefixEntry kRamPatchPrefixes[] = {
        {0x6779, "soc2_0"}, {0x6873, "soc2_0"}, {0x6853, "soc2_0"}, {0x6781, "soc2_2"},
        {0x6833, "soc2_2"}, {0x6835, "soc2_2"}, {0x6855, "soc2_2"},
};

// Chips paired with an a-die pick the patch files by a-die: {chip, mt6635 pattern, mt6631 pattern}.
struct AdiePattern {
    int chip_id;
    const char* mt6635;
    const char* mt6631;
};

constexpr AdiePattern kAdiePatterns[] = {
        {0x6779, "3_1", "3a_1"},
        {0x6853, "3d_1", "3c_1"},
        {0x6855, "1c_1", "1b_1"},
        {0x6833, "1a_1", "1_1"},
        {0x6835, "mt6835_mt6635_1", "mt6835_mt6631_1"},
        {0x6781, "2a_1", "2_1"},
};

struct ComboChip {
    int chip_id;
    const char* cfg_key;
    const char* default_cfg;
};

constexpr ComboChip kComboChips[] = {
        {0x6630, "mt6630.defAnt", "mt6630_ant_m1.cfg"},
        {0x6632, "mt6632.defAnt", "mt6632_ant_m1.cfg"},
};

volatile sig_atomic_t g_canceled = 0;
int g_wmt_fd = -1;

void SetPropertyAsync(std::string key, std::string value) {
    std::thread([key = std::move(key), value = std::move(value)] {
        if (android::base::SetProperty(key, value)) {
            LOG(INFO) << "Set " << key << " to " << value;
        } else {
            LOG(ERROR) << "Failed to set " << key << " to " << value;
        }
    }).detach();
}

wmt_chip_type GetChipType(int chip_id) {
    if (chip_id <= 0) {
        return WMT_CHIP_TYPE_INVALID;
    }
    if (chip_id == 0x6620 || chip_id == 0x6628 || chip_id == 0x6630 || chip_id == 0x6632) {
        return WMT_CHIP_TYPE_COMBO;
    }
    return WMT_CHIP_TYPE_SOC;
}

template <size_t N>
const char* FindPrefix(const PrefixEntry (&table)[N], int chip_id) {
    for (const PrefixEntry& entry : table) {
        if (entry.chip_id == chip_id) {
            return entry.prefix;
        }
    }
    return nullptr;
}

std::string RomPatchPrefix(int chip_id) {
    chip_id = NormalizeChipId(chip_id);
    const char* prefix = FindPrefix(kRomPatchPrefixes, chip_id);
    return (prefix ? std::string(prefix) : StringPrintf("mt%04x", chip_id)) + "_patch";
}

std::string RamPatchPrefix(int chip_id) {
    const char* prefix = FindPrefix(kRamPatchPrefixes, chip_id);
    return std::string(prefix ? prefix : "soc1_0") + "_ram";
}

// Substring that selects the patch files of this chip. Chips with an a-die need it to be known.
std::optional<std::string> PatchPattern(int chip_id) {
    for (const AdiePattern& entry : kAdiePatterns) {
        if (entry.chip_id != chip_id) {
            continue;
        }
        const int adie =
                static_cast<int>(strtoul(GetProperty(kAdieChipIdProp, "").c_str(), nullptr, 16));
        if (adie == 0x6635) {
            return entry.mt6635;
        }
        if (adie == 0x6631) {
            return entry.mt6631;
        }
        LOG(ERROR) << StringPrintf("Unknown a-die chip id 0x%x for chip 0x%x", adie, chip_id);
        return std::nullopt;
    }
    return ".bin";
}

bool ReadAt(int fd, off_t offset, void* buf, size_t len) {
    return pread(fd, buf, len, offset) == static_cast<ssize_t>(len);
}

// Patch files carry a big-endian 16-bit hardware version at offset 22.
std::optional<unsigned> ReadPatchVersion(int fd) {
    uint8_t raw[2];
    if (!ReadAt(fd, 22, raw, sizeof(raw))) {
        return std::nullopt;
    }
    return (static_cast<unsigned>(raw[0]) << 8) | raw[1];
}

bool MatchesFwVersion(unsigned patch_version, int fw_version) {
    return ((patch_version ^ static_cast<unsigned>(fw_version)) & 0xff) == 0;
}

// The first bytes of a patch file hold a build string, cut at the first NUL.
void PublishVersionString(int fd, const char* prop) {
    char buf[kVersionLen] = {};
    if (pread(fd, buf, sizeof(buf), 0) < 0) {
        PLOG(ERROR) << "read patch build version failed";
        return;
    }
    SetPropertyAsync(prop, std::string(buf, strnlen(buf, kVersionLen - 1)));
}

void PublishVersionFromPath(const std::string& path, const char* prop) {
    unique_fd fd(open(path.c_str(), O_RDONLY | O_CLOEXEC));
    if (fd.get() < 0) {
        return;
    }
    PublishVersionString(fd.get(), prop);
}

// The BT patch embeds its version between two fingerprints.
void PublishBtVersion(const std::string& path) {
    std::string data;
    if (!ReadFileToString(path, &data)) {
        PLOG(ERROR) << "read " << path << " failed";
        return;
    }
    if (data.empty() || data.size() > kMaxBtPatchSize) {
        LOG(ERROR) << "Unexpected BT patch size " << data.size();
        return;
    }

    const size_t start = data.find("BABEFACEBABEFACE");
    const size_t end =
            start == std::string::npos ? start : data.find("DEADBEEFDEADBEEF", start + 1);
    if (start == std::string::npos || end == std::string::npos) {
        LOG(ERROR) << "BT patch fingerprints not found";
        return;
    }

    std::string version;
    if (const size_t tag = data.find("t-neptune", start); tag != std::string::npos) {
        if (tag < end) {
            version = data.substr(tag, end - tag);
        }
    } else if (data.find("= debug", start) != std::string::npos) {
        version = data.substr(0, 14);  // debug builds: same as the original, first 14 bytes
    } else {
        LOG(WARNING) << "No version tag found in BT patch";
        return;
    }

    version.resize(std::min(version.size(), kPropValueMax - 1));
    if (const size_t cut = version.find_first_of("\n\0", 0, 2); cut != std::string::npos) {
        version.resize(cut);
    }
    if (version.empty()) {
        return;
    }
    LOG(INFO) << "BT patch version: " << version;
    if (GetProperty(kBtVersionProp, "") != version) {
        SetPropertyAsync(kBtVersionProp, version);
    }
}

void PublishRamPatchVersions(const std::string& name, const std::string& path) {
    if (name.find("ram_bt") != std::string::npos) {
        PublishBtVersion(path);
    }
    if (name.find("ram_wifi") != std::string::npos) {
        PublishVersionFromPath(path, kWifiVersionProp);
    }
    if (name.find("ram_mcu") != std::string::npos) {
        PublishVersionFromPath(path, kPatchVersionProp);
    }
}

struct DirCloser {
    void operator()(DIR* dir) const { closedir(dir); }
};

// Calls fn(name, path) for every firmware file starting with `prefix` and containing `pattern`.
template <typename Fn>
bool ForEachPatchFile(const std::string& prefix, const std::string& pattern, Fn&& fn) {
    std::unique_ptr<DIR, DirCloser> dir(opendir(kFirmwareDir));
    if (!dir) {
        PLOG(ERROR) << "Cannot open " << kFirmwareDir;
        return false;
    }
    while (const dirent* entry = readdir(dir.get())) {
        const std::string name = entry->d_name;
        if (name.compare(0, prefix.size(), prefix) == 0 &&
            name.find(pattern) != std::string::npos) {
            fn(name, std::string(kFirmwareDir) + "/" + name);
        }
    }
    return true;
}

template <typename Info>
void FillPatchInfo(Info& info, const uint8_t raw[4], const std::string& name) {
    memcpy(info.address, raw, sizeof(info.address));
    info.address[0] = 0;
    snprintf(info.patch_name, sizeof(info.patch_name), "%s", name.c_str());
}

// srh_patch: ROM patches, downloaded in sequence (the first header gives the patch count).
int HandleSrhPatch(int fd) {
    const int chip_id = Ioctl(fd, WMT_IOCTL_GET_CHIP_INFO, 0);
    const int fw_version = Ioctl(fd, WMT_IOCTL_GET_CHIP_INFO, 2);
    LOG(INFO) << StringPrintf("chip 0x%04x, fw version 0x%04x", chip_id, fw_version);

    const auto pattern = PatchPattern(chip_id);
    if (!pattern) {
        return -10;
    }

    int ret = 0;
    bool first = true;
    const bool listed = ForEachPatchFile(
            RomPatchPrefix(chip_id), *pattern,
            [&](const std::string& name, const std::string& path) {
                unique_fd patch_fd(open(path.c_str(), O_RDONLY | O_CLOEXEC));
                if (patch_fd.get() < 0) {
                    PLOG(ERROR) << "open " << path << " failed";
                    return;
                }
                PublishVersionString(patch_fd.get(), kPatchVersionProp);

                const auto version = ReadPatchVersion(patch_fd.get());
                if (!version) {
                    LOG(ERROR) << "read patch version failed: " << name;
                    ret = -4;
                    return;
                }
                LOG(INFO) << StringPrintf("%s: fw version in patch 0x%04x", name.c_str(), *version);
                if (!MatchesFwVersion(*version, fw_version)) {
                    return;
                }

                uint8_t info[4];
                if (!ReadAt(patch_fd.get(), 24, info, sizeof(info))) {
                    LOG(ERROR) << "read patch info failed: " << name;
                    ret = -5;
                    return;
                }

                if (first) {
                    const int patch_num = info[0] >> 4;
                    LOG(INFO) << "patch num = " << patch_num;
                    Ioctl(fd, WMT_IOCTL_SET_PATCH_NUM, patch_num);
                }
                StpPatchInfo patch{};
                patch.download_seq = info[0] & 0x0f;
                FillPatchInfo(patch, info, name);
                LOG(INFO) << "patch seq = " << patch.download_seq;
                IoctlPtr(fd, WMT_IOCTL_SET_PATCH_INFO, &patch);
                first = false;
            });
    return listed ? ret : -3;
}

// srh_rom_patch: RAM patches for the newer SoCs, one info block per patch type.
int HandleSrhRomPatch(int fd) {
    const int chip_id = Ioctl(fd, WMT_IOCTL_GET_CHIP_INFO, 0);
    const int fw_version = Ioctl(fd, WMT_IOCTL_GET_CHIP_INFO, 2);
    LOG(INFO) << StringPrintf("chip 0x%04x, fw version 0x%04x", chip_id, fw_version);

    const auto pattern = PatchPattern(chip_id);
    if (!pattern) {
        return -10;
    }

    int ret = 0;
    const bool listed = ForEachPatchFile(
            RamPatchPrefix(chip_id), *pattern,
            [&](const std::string& name, const std::string& path) {
                PublishRamPatchVersions(name, path);

                unique_fd patch_fd(open(path.c_str(), O_RDONLY | O_CLOEXEC));
                if (patch_fd.get() < 0) {
                    PLOG(ERROR) << "open " << path << " failed";
                    return;
                }
                const auto version = ReadPatchVersion(patch_fd.get());
                if (!version) {
                    LOG(ERROR) << "read patch version failed: " << name;
                    ret = -4;
                    return;
                }
                LOG(INFO) << StringPrintf("%s: fw version in patch 0x%04x", name.c_str(), *version);
                if (!MatchesFwVersion(*version, fw_version)) {
                    return;
                }

                uint8_t info[8];
                if (!ReadAt(patch_fd.get(), 24, info, sizeof(info))) {
                    LOG(ERROR) << "read patch info failed: " << name;
                    ret = -5;
                    return;
                }
                if ((info[3] & 0xf0) == 0 || info[7] > 5) {
                    return;
                }

                StpRomPatchInfo patch{};
                patch.type = info[7];
                FillPatchInfo(patch, info, name);
                LOG(INFO) << "rom patch type = " << patch.type;
                if (IoctlPtr(fd, WMT_IOCTL_SET_ROM_PATCH_INFO, &patch) != 0) {
                    LOG(ERROR) << "WMT_IOCTL_SET_ROM_PATCH_INFO failed for " << name;
                    ret = -6;
                }
            });
    return listed ? ret : -3;
}

using CommandHandler = int (*)(int fd);

struct Command {
    std::string_view name;
    CommandHandler handler;
};

constexpr Command kCommands[] = {
        {"srh_patch", HandleSrhPatch},
        {"srh_rom_patch", HandleSrhRomPatch},
};

// 0: success, 1: unknown command, negative: handler error.
int Dispatch(int fd, std::string_view cmd) {
    for (const Command& command : kCommands) {
        if (cmd.substr(0, command.name.size()) == command.name) {
            return command.handler(fd);
        }
    }
    return 1;
}

// The chip ID comes from the property the loader phase published, or from the kernel.
int QueryChipId(int fd) {
    for (;;) {
        const std::string stored = GetProperty(kChipIdProp, "");
        if (!stored.empty()) {
            return static_cast<int>(strtoul(stored.c_str(), nullptr, 16));
        }
        LOG(ERROR) << "Property " << kChipIdProp << " is not set";

        const int chip_id = Ioctl(fd, WMT_IOCTL_WMT_QUERY_CHIPID);
        LOG(INFO) << StringPrintf("chip id from kernel: 0x%04x", chip_id);
        if (chip_id != -1) {
            return chip_id;
        }
        std::this_thread::sleep_for(milliseconds(300));
    }
}

// Combo chips get an antenna config name from WMT.cfg (e.g. "mt6630.defAnt=mt6630_ant_m1.cfg").
void SendWmtCfgName(int fd, int chip_id) {
    std::string content;
    if (!ReadFileToString(kWmtCfgFile, &content)) {
        PLOG(ERROR) << "Cannot read " << kWmtCfgFile;
        return;
    }

    std::string cfg;  // empty for combo chips without a table entry, as before
    for (const ComboChip& chip : kComboChips) {
        if (chip.chip_id != chip_id) {
            continue;
        }
        cfg = chip.default_cfg;
        for (const std::string& line : android::base::Split(content, "\n")) {
            const size_t eq = line.find('=');
            if (eq == std::string::npos ||
                strcasecmp(Trim(line.substr(0, eq)).c_str(), chip.cfg_key) != 0) {
                continue;
            }
            const std::string value = Trim(line.substr(eq + 1));
            cfg = value.substr(0, value.find_first_of(" \t\r"));
            LOG(INFO) << "WMT.cfg: " << chip.cfg_key << " = " << cfg;
        }
    }

    char name[NAME_MAX + 1] = {};
    snprintf(name, sizeof(name), "%s", cfg.c_str());
    IoctlPtr(fd, WMT_IOCTL_WMT_CFG_NAME, name);
}

void ConfigureDriver(int fd, int chip_id, wmt_chip_type chip_type, const LauncherOptions& options) {
    const bool combo = chip_type == WMT_CHIP_TYPE_COMBO;

    if (combo) {
        Ioctl(fd, WMT_IOCTL_WMT_TELL_CHIPID, chip_id);
        LOG(INFO) << StringPrintf("set chip_id(0x%x) to driver", chip_id);
        SendWmtCfgName(fd, chip_id);
    }
    Ioctl(fd, WMT_IOCTL_SET_STP_MODE,
          ((kComboBaudRate & 0xFFFFFF) << 8) | ((kFmMode & 0xF) << 4) | (kStpBtifFull & 0xF));
    Ioctl(fd, WMT_IOCTL_SET_LAUNCHER_KILL, 0);
}

// Powering on makes the kernel ask for patches, so it must not block the request loop.
void PowerOnChip(int fd, int default_on) {
    std::thread([fd, default_on] {
        pthread_setname_np(pthread_self(), "pwr_on_conn");
        const int mode = default_on > 0 ? 2 : 1;
        LOG(INFO) << "Powering on connsys";
        for (int retry = 1; retry <= kPowerOnRetries; ++retry) {
            if (Ioctl(fd, WMT_IOCTL_LPBK_POWER_CTRL, mode) == 0) {
                return;
            }
            Ioctl(fd, WMT_IOCTL_LPBK_POWER_CTRL, 0);
            LOG(INFO) << "Power on failed, retry " << retry << "/" << kPowerOnRetries;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }).detach();
}

void PublishFwVersion(int fd) {
    char full[128] = {};
    const int ret = IoctlPtr(fd, WMT_IOCTL_GET_FW_VERSION, full);
    const size_t len = strnlen(full, sizeof(full));
    if (ret < 0 || len < 14) {
        LOG(ERROR) << "No valid fw version, ret=" << ret << " version=" << full;
        return;
    }
    SetPropertyAsync(kBtVersionProp, full);
    SetPropertyAsync(kPatchVersionProp, full + len - 14);
    SetPropertyAsync(kWifiVersionProp, full + len - 14);
}

// SoC chips: apply the persistent FW log / dynamic dump settings. They may only become
// readable a little after startup, so the properties are polled for a few request-loop turns.
class FwDebugKnobs {
  public:
    explicit FwDebugKnobs(int fd)
        : fd_(fd),
          log_{kFwLogProp, GetProperty(kFwLogProp, ""), {}},
          dump_{kDynamicDumpProp, GetProperty(kDynamicDumpProp, ""), {}} {}

    void Poll() {
        PollKnob(log_, /*apply_if_unchanged=*/true,
                 [this](const std::string& value) { ApplyFwLog(value == "yes"); });
        PollKnob(dump_, /*apply_if_unchanged=*/false,
                 [this](const std::string& value) { ApplyDynamicDump(value); });
    }

  private:
    struct Knob {
        const char* prop;
        std::string original;
        std::string latest;
        int reads = 0;
        bool changed = false;
        bool done = false;
    };

    template <typename Apply>
    void PollKnob(Knob& knob, bool apply_if_unchanged, Apply apply) {
        if (knob.done) {
            return;
        }
        if (knob.changed || knob.reads > kFwDebugRetryLimit) {
            if (knob.changed || apply_if_unchanged) {
                apply(knob.latest);
            }
            knob.done = true;
            return;
        }
        ++knob.reads;
        knob.latest = GetProperty(knob.prop, "");
        if (!knob.latest.empty()) {
            knob.changed = knob.latest != knob.original;
        }
    }

    void ApplyFwLog(bool enable) {
        if (!enable) {
            Ioctl(fd_, WMT_IOCTL_FW_DBGLOG_CTRL, 0);
            return;
        }
        std::thread([fd = fd_] {
            pthread_setname_np(pthread_self(), "dump_fwemi_log");
            LOG(INFO) << "Dumping firmware debug log from the EMI buffer";
            if (Ioctl(fd, WMT_IOCTL_FW_DBGLOG_CTRL, 1) < 0) {
                PLOG(ERROR) << "WMT_IOCTL_FW_DBGLOG_CTRL failed";
            }
        }).detach();
    }

    void ApplyDynamicDump(const std::string& value) {
        char buf[kPropValueMax] = {};
        snprintf(buf, sizeof(buf), "%s", value.c_str());
        if (IoctlPtr(fd_, WMT_IOCTL_DYNAMIC_DUMP_CTRL, buf) < 0) {
            PLOG(ERROR) << "WMT_IOCTL_DYNAMIC_DUMP_CTRL failed";
        }
    }

    int fd_;
    Knob log_;
    Knob dump_;
};

void OnTerminate(int) {
    g_canceled = 1;
    if (g_wmt_fd >= 0) {
        ioctl(g_wmt_fd, WMT_IOCTL_SET_LAUNCHER_KILL, 1UL);  // async-signal-safe
    }
}

void InstallSignalHandlers() {
    struct sigaction sa = {};
    sa.sa_handler = SIG_IGN;
    sigaction(SIGCHLD, &sa, nullptr);
    sigaction(SIGPIPE, &sa, nullptr);
    sigaction(SIGHUP, &sa, nullptr);

    sa.sa_handler = OnTerminate;  // no SA_RESTART: poll() must return EINTR
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGINT, &sa, nullptr);
}

void ServeRequests(int fd, FwDebugKnobs* knobs) {
    pollfd pfd = {fd, POLLIN | POLLRDNORM, 0};
    char cmd[NAME_MAX + 1];

    while (!g_canceled) {
        if (knobs != nullptr) {
            knobs->Poll();
        }

        pfd.revents = 0;
        const int ready = poll(&pfd, 1, -1);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            PLOG(ERROR) << "poll failed";
            break;
        }
        if (ready == 0) {
            continue;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            LOG(ERROR) << "poll error on " << kWmtDev << ", revents=0x" << std::hex << pfd.revents;
            break;
        }
        if (!(pfd.revents & POLLIN)) {
            continue;
        }

        const ssize_t len = read(fd, cmd, sizeof(cmd) - 1);
        if (len <= 0) {
            LOG(ERROR) << "POLLIN but read failed: " << len;
            continue;
        }
        cmd[len] = '\0';

        const int err = Dispatch(fd, std::string_view(cmd, len));
        const std::string response = err == 0   ? "ok"
                                     : err == 1 ? "cmd not found"
                                                : StringPrintf("resp_%d", err);
        LOG(INFO) << "cmd(" << cmd << ") resp(" << response << ")";
        if (write(fd, response.data(), response.size()) != static_cast<ssize_t>(response.size())) {
            PLOG(ERROR) << "write response failed";
        }
    }
}

}  // namespace

int RunLauncher(const LauncherOptions& options) {
    unique_fd fd = OpenDeviceBlocking(kWmtDev);
    g_wmt_fd = fd.get();

    const int chip_id = QueryChipId(fd.get());
    LOG(INFO) << StringPrintf("chip id 0x%04x", chip_id);
    wmt_chip_type chip_type = GetChipType(chip_id);
    if (chip_type == WMT_CHIP_TYPE_INVALID) {
        LOG(ERROR) << "Unknown chip type, assuming SoC";
    }

    ConfigureDriver(fd.get(), chip_id,
                    chip_type == WMT_CHIP_TYPE_INVALID ? WMT_CHIP_TYPE_SOC : chip_type, options);
    InstallSignalHandlers();

    std::optional<FwDebugKnobs> knobs;
    if (chip_type == WMT_CHIP_TYPE_SOC) {
        knobs.emplace(fd.get());
    }

    PowerOnChip(fd.get(), options.default_on);
    PublishFwVersion(fd.get());

    ServeRequests(fd.get(), knobs ? &*knobs : nullptr);

    g_wmt_fd = -1;
    return EXIT_SUCCESS;
}
