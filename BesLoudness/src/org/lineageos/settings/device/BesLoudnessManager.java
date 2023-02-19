/*
 * Copyright (c) 2023 The LineageOS Project
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
