//
// Copyright 2016 The Android Open Source Project
// Copyright 2024-2025 NXP
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#pragma once

#include <hidl/HidlSupport.h>

#include "BluetoothHci.h"
#include "async_fd_watcher.h"
#include "bluetooth_address.h"
#include "bt_vendor_lib.h"
#include "hci_internals.h"
#include "hci_packetizer.h"

namespace aidl::android::hardware::bluetooth::impl {

using InitializeCompleteCallback = std::function<void(bool success)>;
using PacketReadCallback = std::function<void(const std::vector<uint8_t>&)>;
using namespace ::android::hardware::bluetooth::async;
using namespace ::android::hardware::bluetooth::hci;

class FirmwareStartupTimer;

class VendorInterface {
 public:
  static bool Initialize(InitializeCompleteCallback initialize_complete_cb,
                         PacketReadCallback cmd_cb, PacketReadCallback acl_cb,
                         PacketReadCallback sco_cb, PacketReadCallback event_cb,
                         PacketReadCallback iso_cb,
                         DisconnectCallback disconnect_cb);
  static void Shutdown();
  static VendorInterface* get();

  size_t Send(PacketType type, const uint8_t* data, size_t length);

  void OnFirmwareConfigured(uint8_t result);

 private:
  virtual ~VendorInterface() = default;

  bool Open(InitializeCompleteCallback initialize_complete_cb,
            PacketReadCallback cmd_cb, PacketReadCallback acl_cb,
            PacketReadCallback sco_cb, PacketReadCallback event_cb,
            PacketReadCallback iso_cb, DisconnectCallback disconnect_cb);
  void Close();

  void OnTimeout();

  void HandleIncomingEvent(const std::vector<uint8_t>& hci_packet);

  void LocalconfigHciandFw(H4Protocol* h4_hci);

  void* lib_handle_ = nullptr;
  bt_vendor_interface_t* lib_interface_ = nullptr;
  AsyncFdWatcher fd_watcher_;
  InitializeCompleteCallback initialize_complete_cb_;
  H4Protocol* hci_ = nullptr;

  PacketReadCallback event_cb_;

  FirmwareStartupTimer* firmware_startup_timer_ = nullptr;
};

}  // namespace aidl::android::hardware::bluetooth::impl
