/*
 * Copyright 2022 The Android Open Source Project
 * Copyright 2024-2025 NXP
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

#define LOG_TAG "android.hardware.bluetooth.service.default"

#include "BluetoothHci.h"

#include <cutils/properties.h>
#include <fcntl.h>
#include <hidl/HidlSupport.h>
#include <hidl/HidlTransportSupport.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <string.h>
#include <sys/uio.h>
#include <termios.h>

#include <iostream>

#include "log/log.h"
#include "vendor_interface.h"

namespace {
int SetTerminalRaw(int fd) {
  termios terminal_settings;
  int rval = tcgetattr(fd, &terminal_settings);
  if (rval < 0) {
    return rval;
  }
  cfmakeraw(&terminal_settings);
  rval = tcsetattr(fd, TCSANOW, &terminal_settings);
  return rval;
}
}  // namespace

using namespace ::android::hardware::bluetooth::hci;
using namespace ::android::hardware::bluetooth::async;
using aidl::android::hardware::bluetooth::Status;

namespace aidl::android::hardware::bluetooth::impl {

void OnDeath(void* cookie);
std::optional<std::string> GetSystemProperty(const std::string& property) {
  std::array<char, PROPERTY_VALUE_MAX> value_array{0};
  auto value_len = property_get(property.c_str(), value_array.data(), nullptr);
  if (value_len <= 0) {
    return std::nullopt;
  }
  return std::string(value_array.data(), value_len);
}
bool starts_with(const std::string& str, const std::string& prefix) {
  return str.compare(0, prefix.length(), prefix) == 0;
}

class BluetoothDeathRecipient {
 public:
  BluetoothDeathRecipient(BluetoothHci* hci) : mHci(hci) {}

  void LinkToDeath(const std::shared_ptr<IBluetoothHciCallbacks>& cb) {
    mCb = cb;
    clientDeathRecipient_ = AIBinder_DeathRecipient_new(OnDeath);
    auto linkToDeathReturnStatus = AIBinder_linkToDeath(
        mCb->asBinder().get(), clientDeathRecipient_, this /* cookie */);
    LOG_ALWAYS_FATAL_IF(linkToDeathReturnStatus != STATUS_OK,
                        "Unable to link to death recipient");
  }

  void UnlinkToDeath(const std::shared_ptr<IBluetoothHciCallbacks>& cb) {
    LOG_ALWAYS_FATAL_IF(cb != mCb, "Unable to unlink mismatched pointers");
  }

  void serviceDied() {
    if (mCb != nullptr && !AIBinder_isAlive(mCb->asBinder().get())) {
      ALOGE("Bluetooth remote service has died");
    } else {
      ALOGE("BluetoothDeathRecipient::serviceDied called but service not dead");
      return;
    }
    {
      std::lock_guard<std::mutex> guard(mHasDiedMutex);
      has_died_ = true;
    }
    mHci->close();
  }
  BluetoothHci* mHci;
  std::shared_ptr<IBluetoothHciCallbacks> mCb;
  AIBinder_DeathRecipient* clientDeathRecipient_;
  bool getHasDied() {
    std::lock_guard<std::mutex> guard(mHasDiedMutex);
    return has_died_;
  }

 private:
  std::mutex mHasDiedMutex;
  bool has_died_{false};
};

void OnDeath(void* cookie) {
  auto* death_recipient = static_cast<BluetoothDeathRecipient*>(cookie);
  death_recipient->serviceDied();
}

BluetoothHci::BluetoothHci(const std::string& dev_path) {
  char property_bytes[PROPERTY_VALUE_MAX];
  property_get("vendor.ser.bt-uart", property_bytes, dev_path.c_str());
  mDevPath = std::string(property_bytes);
  mDeathRecipient = std::make_shared<BluetoothDeathRecipient>(this);
}

int BluetoothHci::getFdFromDevPath() {
  int fd = open(mDevPath.c_str(), O_RDWR);
  if (fd < 0) {
    ALOGE("Could not connect to bt: %s (%s)", mDevPath.c_str(),
          strerror(errno));
    return fd;
  }
  if (int ret = SetTerminalRaw(fd) < 0) {
    ALOGI("Could not make %s a raw terminal %d(%s)", mDevPath.c_str(), ret,
          strerror(errno));
  }
  return fd;
}

ndk::ScopedAStatus BluetoothHci::initialize(
    const std::shared_ptr<IBluetoothHciCallbacks>& cb) {
  ALOGI("Initializing Bluetooth HCI via AIDL");

  if (cb == nullptr) {
    ALOGE("cb == nullptr! -> Unable to call initializationComplete(ERR)");
    return ndk::ScopedAStatus::fromServiceSpecificError(STATUS_BAD_VALUE);
  }

  HalState old_state = HalState::READY;
  {
    std::lock_guard<std::mutex> guard(mStateMutex);
    if (mState != HalState::READY) {
      old_state = mState;
    } else {
      mState = HalState::INITIALIZING;
    }
  }

  if (old_state != HalState::READY) {
    ALOGE("initialize: Unexpected State %d", static_cast<int>(old_state));
    close();
    cb->initializationComplete(Status::ALREADY_INITIALIZED);
    return ndk::ScopedAStatus::ok();
  }

  bool rc = VendorInterface::Initialize(
      [cb](bool status) {
        cb->initializationComplete(
            status ? Status::SUCCESS : Status::HARDWARE_INITIALIZATION_ERROR);
      },
      [](const std::vector<uint8_t>&) {
        LOG_ALWAYS_FATAL("Unexpected command!");
      },
      [cb](const std::vector<uint8_t>& raw_acl) {
        cb->aclDataReceived(raw_acl);
      },
      [cb](const std::vector<uint8_t>& raw_sco) {
        cb->scoDataReceived(raw_sco);
      },
      [cb](const std::vector<uint8_t>& raw_event) {
        cb->hciEventReceived(raw_event);
      },
      [cb](const std::vector<uint8_t>& raw_iso) {
        cb->isoDataReceived(raw_iso);
      },
      [this]() {
        ALOGI("HCI socket device disconnected");
      });
  if (!rc) {
    ALOGE("VendorInterface::Initialize failed");
    VendorInterface::Shutdown();
    {
      std::lock_guard<std::mutex> guard(mStateMutex);
      mState = HalState::READY;
    }
    return ndk::ScopedAStatus::fromServiceSpecificError(STATUS_BAD_VALUE);
  }

  mCb = cb;
  {
    std::lock_guard<std::mutex> guard(mStateMutex);
    mState = HalState::ONE_CLIENT;
  }

  ALOGI("%s:Bluetooth HCI initialized successfully, state = %d", __func__,
        static_cast<int>(mState));
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus BluetoothHci::close() {
  ALOGI("%s:Bluetooth HCI close sequence initiated via AIDL", __func__);
  {
    std::lock_guard<std::mutex> guard(mStateMutex);
    if (mState != HalState::ONE_CLIENT) {
      ALOGI("Already closed");
      return ndk::ScopedAStatus::ok();
    }
    mState = HalState::CLOSING;
  }
  ALOGI("%s: HalState set moving to CLOSING", __func__);
  VendorInterface::Shutdown();
  {
    std::lock_guard<std::mutex> guard(mStateMutex);
    mState = HalState::READY;
  }
  ALOGI("%s: Shutdown complete, HalState moving to READY", __func__);
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus BluetoothHci::sendHciCommand(
    const std::vector<uint8_t>& packet) {
  return send(PacketType::COMMAND, packet);
}

ndk::ScopedAStatus BluetoothHci::sendAclData(
    const std::vector<uint8_t>& packet) {
  return send(PacketType::ACL_DATA, packet);
}

ndk::ScopedAStatus BluetoothHci::sendScoData(
    const std::vector<uint8_t>& packet) {
  return send(PacketType::SCO_DATA, packet);
}

ndk::ScopedAStatus BluetoothHci::sendIsoData(
    const std::vector<uint8_t>& packet) {
  return send(PacketType::ISO_DATA, packet);
}

ndk::ScopedAStatus BluetoothHci::send(PacketType type,
                                      const std::vector<uint8_t>& data) {
  VendorInterface::get()->Send(type, data.data(), data.size());
  return ndk::ScopedAStatus::ok();
}

}  // namespace aidl::android::hardware::bluetooth::impl
