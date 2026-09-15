/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

package com.mediatek.internal.telephony;

import android.os.SystemProperties;

public class RadioCapabilitySwitchUtil {
    private static final String PROPERTY_CAPABILITY_SWITCH = "persist.vendor.radio.simswitch";

    /**
     * Get main capability phone ID.
     *
     * @return Phone ID with main capability
     */
    public static int getMainCapabilityPhoneId() {
        int phoneId = 0;
        phoneId = SystemProperties.getInt(PROPERTY_CAPABILITY_SWITCH, 1) - 1;
        return phoneId;
    }
}
