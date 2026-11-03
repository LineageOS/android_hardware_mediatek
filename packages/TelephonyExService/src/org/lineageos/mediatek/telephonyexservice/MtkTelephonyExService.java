/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

package org.lineageos.mediatek.telephonyexservice;

import org.lineageos.mediatek.telephonyexservice.MtkPhoneInterfaceManagerEx;

import android.app.Application;
import android.util.Log;

public class MtkTelephonyExService extends Application {
    private static final String TAG = "MtkTelephonyExService";
    private MtkPhoneInterfaceManagerEx mPhoneExService;

    @Override
    public void onCreate() {
        super.onCreate();
        Log.i(TAG, "Initializing");

        try {
            mPhoneExService = new MtkPhoneInterfaceManagerEx(this);
            Log.i(TAG, "phoneEx service initialization complete");
        } catch (Exception e) {
            Log.e(TAG, "Failed to initialize phoneEx service", e);
        }
    }
}
