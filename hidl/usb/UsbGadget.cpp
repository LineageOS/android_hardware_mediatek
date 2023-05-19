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

#define LOG_TAG "android.hardware.usb@1.3-service-mediatekv2"

#include "UsbGadget.h"
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/inotify.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace android {
namespace hardware {
namespace usb {
namespace gadget {
namespace V1_1 {
namespace implementation {

bool kByPassUsbd;
bool kExitCheckThread;
bool kUsbPullUp;
std::condition_variable kCv;

constexpr char kUsbController[] = "sys.usb.controller";

#define UDC_PATH "/sys/class/udc/"
#define SOFT_CONN "soft_connect"

bool readFile(const std::string &filename, std::string *contents) {
  FILE *fp;
  ssize_t read = 0;
  char *line = NULL;
  size_t len = 0;

  fp = fopen(filename.c_str(), "r");
  if (fp != NULL) {
    if ((read = getline(&line, &len, fp)) != -1) {
      char *pos;
      if ((pos = strchr(line, '\n')) != NULL) *pos = '\0';
      *contents = line;
    }
    free(line);
    fclose(fp);
  } else {
    ALOGE("fopen failed");
    return false;
  }

  return true;
}

void waitForUsbPullDown() {
    string udc = "udc";

    while (!kExitCheckThread) {
        if (readFile(PULLUP_PATH, &udc) && (udc == "" || udc == "none")) {
            kUsbPullUp = false;
            break;
        }
        usleep(100);
    }

    kCv.notify_all();
}

void waitForUsbPullUp() {
    string udc = "none";

    while (!kExitCheckThread) {
        if (readFile(PULLUP_PATH, &udc) && udc != "" && udc != "none") {
            kUsbPullUp = true;
            break;
        }
        usleep(100);
    }

    kCv.notify_all();
}

static string getUdcPath(const char *subPath) {
    string gadgetName = GetProperty(kUsbController, "");
    string ret = "";

    if (!subPath) {
        ALOGI("subPath is null");
        return ret;
    }

    if (gadgetName.empty()) {
        ALOGI("can't get controller name");
        return ret;
    }

    ret = UDC_PATH + gadgetName + "/" + subPath;

    return ret;
}

UsbGadget::UsbGadget() {
    if (access(OS_DESC_PATH, R_OK) != 0) {
        ALOGE("configfs setup not done yet");
        abort();
    }

    ALOGI("UsbGadget");
    kByPassUsbd = true;
}

void currentFunctionsAppliedCallback(bool functionsApplied, void* payload) {
    UsbGadget* gadget = (UsbGadget*)payload;
    gadget->mCurrentUsbFunctionsApplied = functionsApplied;
}

Return<void> UsbGadget::getCurrentUsbFunctions(const sp<V1_0::IUsbGadgetCallback>& callback) {
    Return<void> ret = callback->getCurrentUsbFunctionsCb(
            mCurrentUsbFunctions, mCurrentUsbFunctionsApplied ? Status::FUNCTIONS_APPLIED
                                                              : Status::FUNCTIONS_NOT_APPLIED);
    if (!ret.isOk()) ALOGE("Call to getCurrentUsbFunctionsCb failed %s", ret.description().c_str());

    return Void();
}

V1_0::Status UsbGadget::tearDownGadget() {
    std::unique_lock<std::mutex> lk(mLockSetCurrentFunction);
    V1_0::Status ret = V1_0::Status::SUCCESS;
    int timeout = 500; //500 ms
    string soft_connect = getUdcPath(SOFT_CONN);

    ALOGI("tearDownGadget");

    if (soft_connect != "") {
        if (!WriteStringToFile("disconnect", soft_connect)) {
            ALOGI("Gadget cannot be disconnected");
        } else {
           // wait userspace process close
	   std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
    }

    if (!SetProperty("vendor.usb.config", "none"))
        return V1_0::Status::ERROR;

    kExitCheckThread = false;
    std::thread checkThread(waitForUsbPullDown);

    // Monitors the paths to pull down the gadget
    if (kCv.wait_for(lk, timeout * 1ms, [] { return !kUsbPullUp; })) {
    } else {
        ALOGE("waitForUsbPullDown timeout");
        ret = V1_0::Status::ERROR;
    }

    kExitCheckThread = true;
    checkThread.join();
    return ret;
}

Return<Status> UsbGadget::reset() {
    if (!WriteStringToFile("none", PULLUP_PATH)) {
        ALOGI("Gadget cannot be pulled down");
        return Status::ERROR;
    }

    usleep(kDisconnectWaitUs);

    string gadgetName = GetProperty(kUsbController, "");

    if (gadgetName.empty()) {
        ALOGI("can't get controller name");
        return Status::ERROR;
    }

    ALOGI("pulled up %s", gadgetName.c_str());

    if (!WriteStringToFile(gadgetName, PULLUP_PATH)) {
        ALOGI("Gadget cannot be pulled up");
        return Status::ERROR;
    }

    return Status::SUCCESS;
}

static V1_0::Status switchUsbFunctions(uint64_t functions) {
    V1_0::Status ret = V1_0::Status::SUCCESS;
    string f_str = "";

    switch (functions) {
        case static_cast<uint64_t>(V1_0::GadgetFunction::MTP):
            f_str = "mtp";
            break;
        case V1_0::GadgetFunction::ADB | V1_0::GadgetFunction::MTP:
            f_str = "mtp,adb";
            break;
        case static_cast<uint64_t>(V1_0::GadgetFunction::RNDIS):
            f_str = "rndis";
            break;
        case V1_0::GadgetFunction::ADB | V1_0::GadgetFunction::RNDIS:
            f_str = "rndis,adb";
            break;
        case static_cast<uint64_t>(V1_0::GadgetFunction::PTP):
            f_str = "ptp";
            break;
        case V1_0::GadgetFunction::ADB | V1_0::GadgetFunction::PTP:
            f_str = "ptp,adb";
            break;
        case static_cast<uint64_t>(V1_0::GadgetFunction::ADB):
            f_str = "adb";
            break;
        case static_cast<uint64_t>(V1_0::GadgetFunction::MIDI):
            f_str = "midi";
            break;
        case V1_0::GadgetFunction::ADB | V1_0::GadgetFunction::MIDI:
            f_str = "midi,adb";
            break;
        case static_cast<uint64_t>(V1_0::GadgetFunction::ACCESSORY):
            f_str = "accessory";
            break;
        case V1_0::GadgetFunction::ADB | V1_0::GadgetFunction::ACCESSORY:
            f_str = "accessory,adb";
            break;
        case static_cast<uint64_t>(V1_0::GadgetFunction::AUDIO_SOURCE):
            f_str = "audio_source";
            break;
        case V1_0::GadgetFunction::ADB | V1_0::GadgetFunction::AUDIO_SOURCE:
            f_str = "audio_source,adb";
            break;
        case V1_0::GadgetFunction::ACCESSORY | V1_0::GadgetFunction::AUDIO_SOURCE:
            f_str = "accessory,audio_source";
            break;
        case V1_0::GadgetFunction::ADB | V1_0::GadgetFunction::ACCESSORY |
                V1_0::GadgetFunction::AUDIO_SOURCE:
            f_str = "accessory,audio_source,adb";
            break;
        default:
            ALOGE("Combination not supported");
            return V1_0::Status::CONFIGURATION_NOT_SUPPORTED;
    }

    ALOGI("switchUsbFunctions %s", f_str.c_str());

    if (!SetProperty("vendor.usb.config", f_str))
        ret = V1_0::Status::ERROR;

    return ret;
}

bool isMTPReady(uint64_t functions) {
    bool mtpReady = true;

    if (((functions & GadgetFunction::MTP) != 0)) {
        if (access("/dev/usb-ffs/mtp/ep1", R_OK)) {
            ALOGI("%s absent", "/dev/usb-ffs/mtp/ep1");
            mtpReady = false;
        }

        if (access("/dev/usb-ffs/mtp/ep2", R_OK)) {
            ALOGI("%s absent", "/dev/usb-ffs/mtp/ep2");
            mtpReady = false;
        }

        if (access("/dev/usb-ffs/mtp/ep3", R_OK)) {
            ALOGI("%s absent", "/dev/usb-ffs/mtp/ep3");
            mtpReady = false;
        }
    } else if (((functions & GadgetFunction::PTP) != 0)) {
        if (access("/dev/usb-ffs/ptp/ep1", R_OK)) {
            ALOGI("%s absent", "/dev/usb-ffs/ptp/ep1");
            mtpReady = false;
        }

        if (access("/dev/usb-ffs/ptp/ep2", R_OK)) {
            ALOGI("%s absent", "/dev/usb-ffs/ptp/ep2");
            mtpReady = false;
        }

        if (access("/dev/usb-ffs/ptp/ep3", R_OK)) {
            ALOGI("%s absent", "/dev/usb-ffs/ptp/ep3");
            mtpReady = false;
        }
    }
    return mtpReady;
}

void checkMTP(uint64_t functions) {
    bool mtpReady = true;

    mtpReady = isMTPReady(functions);
    ALOGI("checkMTP(%d)", mtpReady);

    // wait and retry
    if (mtpReady == false) {
        for (int i = 0; i < 20; i++) {
            mtpReady = isMTPReady(functions);
            ALOGI("checkMTP(%d, %d)", i, mtpReady);

            if (mtpReady == true)
                break;

            usleep(50*1000); // 50 ms
        }
    }

    if (((functions & GadgetFunction::MTP) != 0)) {
        if (mtpReady == true)
            SetProperty("vendor.usb.ffs.mtp.ready", "1");
        else
            SetProperty("vendor.usb.ffs.mtp.ready", "0");
    } else if (((functions & GadgetFunction::PTP) != 0)) {
        if (mtpReady == true)
            SetProperty("vendor.usb.ffs.ptp.ready", "1");
        else
            SetProperty("vendor.usb.ffs.ptp.ready", "0");
    }
}

V1_0::Status UsbGadget::setupFunctions(uint64_t functions,
                                       const sp<V1_0::IUsbGadgetCallback>& callback,
                                       uint64_t timeout) {
    std::unique_lock<std::mutex> lk(mLockSetCurrentFunction);
    V1_0::Status ret = V1_0::Status::SUCCESS;

    checkMTP(functions);

    ret = switchUsbFunctions(functions);
    if (ret != V1_0::Status::SUCCESS)
        return ret;

    kExitCheckThread = false;
    std::thread checkThread(waitForUsbPullUp);

    // Monitors the paths to pull up the gadget
    if (kCv.wait_for(lk, timeout * 1ms, [] { return kUsbPullUp; })) {
        mCurrentUsbFunctionsApplied = true;
        if (callback) callback->setCurrentUsbFunctionsCb(functions, V1_0::Status::SUCCESS);
    } else {
        ALOGE("waitForUsbPullUp timeout");
        ret = V1_0::Status::ERROR;
    }

    kExitCheckThread = true;
    checkThread.join();
    return ret;
}

Return<void> UsbGadget::setCurrentUsbFunctions(uint64_t functions,
                                               const sp<V1_0::IUsbGadgetCallback>& callback,
                                               uint64_t timeout) {
    V1_0::Status status = V1_0::Status::ERROR;

    mCurrentUsbFunctions = functions;
    mCurrentUsbFunctionsApplied = false;
    kUsbPullUp = true;

    if (kByPassUsbd) {
        ALOGI("setCurrentUsbFunctions: skip first time for usbd\n");
        kByPassUsbd = false;
        mCurrentUsbFunctionsApplied = true;
        goto error;
    }

    // Unlink the gadget and stop the monitor if running.
    status = tearDownGadget();
    if (status != V1_0::Status::SUCCESS) {
        goto error;
    }

    ALOGI("Returned from tearDown gadget");

    // Leave the gadget pulled down to give time for the host to sense disconnect.
    usleep(kDisconnectWaitUs);

    if (functions == static_cast<uint64_t>(V1_0::GadgetFunction::NONE)) {
        if (callback == NULL) return Void();
        Return<void> ret = callback->setCurrentUsbFunctionsCb(functions, V1_0::Status::SUCCESS);
        if (!ret.isOk())
            ALOGE("Error while calling setCurrentUsbFunctionsCb %s", ret.description().c_str());
        return Void();
    }

    status = setupFunctions(functions, callback, timeout);
    if (status != V1_0::Status::SUCCESS) {
        goto error;
    }

    ALOGI("Usb Gadget setcurrent functions called successfully");
    return Void();

error:
    ALOGI("Usb Gadget setcurrent functions failed");
    if (callback == NULL) return Void();
    Return<void> ret = callback->setCurrentUsbFunctionsCb(functions, status);
    if (!ret.isOk())
        ALOGE("Error while calling setCurrentUsbFunctionsCb %s", ret.description().c_str());
    return Void();
}
}  // namespace implementation
}  // namespace V1_1
}  // namespace gadget
}  // namespace usb
}  // namespace hardware
}  // namespace android
