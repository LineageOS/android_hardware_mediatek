/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

package com.mediatek.internal.telephony.selfactivation;

public interface ISelfActivation {
    /**
     * Actions for selfActivationAction API
     */
    public int ACTION_MO_CALL = 1;

    /**
     * Definition for call types
     */
    public int CALL_TYPE_NORMAL = 0;
    public int CALL_TYPE_EMERGENCY = 1;

    /**
     * Definition for activation states
     */
    public int STATE_NOT_ACTIVATED = 2;

    /**
     * Extra key for call type
     */
    public String EXTRA_KEY_MO_CALL_TYPE = "key_mo_call_type";
}
