/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

package org.lineageos.mediatek.telephonyexservice;

interface IMtkTelephonyEx {
    boolean supportSimTypeSwitch();
    int getCurrentSimType(int slotId);
    int switchSimType(int slotId, int simType);
}
