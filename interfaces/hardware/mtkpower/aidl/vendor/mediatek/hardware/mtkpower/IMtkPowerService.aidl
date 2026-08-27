/*
 * SPDX-FileCopyrightText: The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */
///////////////////////////////////////////////////////////////////////////////
// THIS FILE IS IMMUTABLE. DO NOT EDIT IN ANY CASE.                          //
///////////////////////////////////////////////////////////////////////////////

// This file is a snapshot of an AIDL file. Do not edit it manually. There are
// two cases:
// 1). this is a frozen version file - do not edit this in any case.
// 2). this is a 'current' file. If you make a backwards compatible change to
//     the interface (from the latest frozen version), the build system will
//     prompt you to update this file with `m <name>-update-api`.
//
// You must not make a backward incompatible change to any AIDL file built
// with the aidl_interface module type with versions property set. The module
// type is used to build AIDL files in a way that they can be used across
// independently updatable components of the system. If a device is shipped
// with such a backward incompatible change, it has a high risk of breaking
// later when a module using the interface is updated, e.g., Mainline modules.

package vendor.mediatek.hardware.mtkpower;
import vendor.mediatek.hardware.mtkpower.IMtkPowerCallback;
import vendor.mediatek.hardware.mtkpower.IMtkPowerCallback;
@VintfStability
interface IMtkPowerService {
  int perfCusLockHint(in int hint, in int duration, in int pid);
  int perfLockAcquire(in int hdl, in int duration, in int[] boostList, in int pid, in int reserved);
  oneway void perfLockRelease(in int hdl, in int reserved);
  int perfLockReleaseSync(in int hdl, in int reserved);
  oneway void mtkCusPowerHint(in int hint, in int data);
  oneway void mtkPowerHint(in int hint, in int data);
  int querySysInfo(in int cmd, in int param);
  int setMtkPowerCallback(in IMtkPowerCallback callback);
  int setMtkScnUpdateCallback(in int scn, in IMtkPowerCallback callback);
  int setSysInfo(in int cmd, in String data);
  oneway void setSysInfoAsync(in int cmd, in String data);
}
