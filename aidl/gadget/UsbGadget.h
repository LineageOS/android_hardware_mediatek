/*
 * SPDX-FileCopyrightText: 2020 The Android Open Source Project
 * SPDX-FileCopyrightText: 2024 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/android/hardware/usb/gadget/BnUsbGadget.h>
#include <aidl/android/hardware/usb/gadget/BnUsbGadgetCallback.h>
#include <aidl/android/hardware/usb/gadget/GadgetFunction.h>
#include <aidl/android/hardware/usb/gadget/IUsbGadget.h>
#include <aidl/android/hardware/usb/gadget/IUsbGadgetCallback.h>
#include <android-base/file.h>
#include <android-base/properties.h>
#include <android-base/strings.h>
#include <android-base/unique_fd.h>
#include <pixelusb/UsbGadgetAidlCommon.h>
#include <sched.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <utils/Log.h>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace aidl {
namespace android {
namespace hardware {
namespace usb {
namespace gadget {

using ::aidl::android::hardware::usb::gadget::GadgetFunction;
using ::aidl::android::hardware::usb::gadget::IUsbGadget;
using ::aidl::android::hardware::usb::gadget::IUsbGadgetCallback;
using ::aidl::android::hardware::usb::gadget::Status;
using ::aidl::android::hardware::usb::gadget::UsbSpeed;
using ::android::base::GetProperty;
using ::android::base::ReadFileToString;
using ::android::base::SetProperty;
using ::android::base::Trim;
using ::android::base::unique_fd;
using ::android::base::WriteStringToFile;
using ::android::hardware::google::pixel::usb::addAdb;
using ::android::hardware::google::pixel::usb::addEpollFd;
using ::android::hardware::google::pixel::usb::kDebug;
using ::android::hardware::google::pixel::usb::kDisconnectWaitUs;
using ::android::hardware::google::pixel::usb::linkFunction;
using ::android::hardware::google::pixel::usb::MonitorFfs;
using ::android::hardware::google::pixel::usb::resetGadget;
using ::android::hardware::google::pixel::usb::setVidPid;
using ::android::hardware::google::pixel::usb::unlinkFunctions;
using ::ndk::ScopedAStatus;
using ::std::shared_ptr;
using ::std::string;

const std::string kGadgetName = GetProperty("sys.usb.controller", "");

#define UDC_PATH "/sys/class/udc/"
#define DEVICE "device/"
#define SPEED_PATH "current_speed"
#define SAVING_PATH DEVICE "saving"

struct UsbGadget : public BnUsbGadget {
    UsbGadget();

    // Makes sure that only one request is processed at a time.
    std::mutex mLockSetCurrentFunction;
    long mCurrentUsbFunctions;
    bool mCurrentUsbFunctionsApplied;
    UsbSpeed mUsbSpeed;
    MonitorFfs* mMonitorFfs;

    ScopedAStatus setCurrentUsbFunctions(long functions,
                                         const shared_ptr<IUsbGadgetCallback>& callback,
                                         int64_t timeout, int64_t in_transactionId) override;

    ScopedAStatus getCurrentUsbFunctions(const shared_ptr<IUsbGadgetCallback>& callback,
                                         int64_t in_transactionId) override;

    ScopedAStatus reset(const shared_ptr<IUsbGadgetCallback>& callback,
                        int64_t in_transactionId) override;

    ScopedAStatus getUsbSpeed(const shared_ptr<IUsbGadgetCallback>& callback,
                              int64_t in_transactionId) override;

    ScopedAStatus setVidPid(const char* vid, const char* pid);

  private:
    Status tearDownGadget();
    Status setupFunctions(long functions, const shared_ptr<IUsbGadgetCallback>& callback,
                          uint64_t timeout, int64_t in_transactionId);
};

}  // namespace gadget
}  // namespace usb
}  // namespace hardware
}  // namespace android
}  // namespace aidl
