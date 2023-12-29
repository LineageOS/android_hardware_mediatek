/*
 * Copyright (c) 2023 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

package org.lineageos.settings.device;

import android.content.SharedPreferences;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.media.AudioManager;
import android.util.Log;

import org.lineageos.settings.besloudness.R;

public class BesLoudnessManager {
    public static final String TAG = "BesLoudness";
    public static final String SHARED_PREFS_NAME = "besloudness";
    public static final String KEY_BESLOUDNESS = "besloudness";

    public static void set(final Context context, Boolean value) {
        final boolean defaultValue = context.getResources().getBoolean(
            R.bool.besloudness_default_value);
        final SharedPreferences prefs = context.getSharedPreferences(SHARED_PREFS_NAME, 0);
        if (value != null) {
            prefs.edit().putBoolean(KEY_BESLOUDNESS, value).apply();
        }
        boolean newValue = prefs.getBoolean(KEY_BESLOUDNESS, defaultValue);
        final AudioManager amgr = (AudioManager) context.getSystemService("audio");
        amgr.setParameters("SetBesLoudnessStatus=" + (newValue ? "1" : "0"));
    }

    public static boolean get(final Context context) {
        final boolean defaultValue = context.getResources().getBoolean(
            R.bool.besloudness_default_value);
        final SharedPreferences prefs = context.getSharedPreferences(SHARED_PREFS_NAME, 0);
        boolean expectedValue = prefs.getBoolean(KEY_BESLOUDNESS, defaultValue);
        final AudioManager amgr = (AudioManager) context.getSystemService("audio");
        boolean actualValue = !("GetBesLoudnessStatus=0".equals(
                amgr.getParameters("GetBesLoudnessStatus")));
        if (actualValue != expectedValue) {
            Log.e(TAG, "value mismatch, expected " + expectedValue + ", got " + actualValue);
        }
        return actualValue;
    }
}
