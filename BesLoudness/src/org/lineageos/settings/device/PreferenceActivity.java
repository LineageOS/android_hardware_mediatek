/*
 * Copyright (c) 2023 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

package org.lineageos.settings.device;

import android.os.Bundle;
import android.widget.CompoundButton;
import android.widget.CompoundButton.OnCheckedChangeListener;

import androidx.preference.PreferenceFragment;

import com.android.settingslib.collapsingtoolbar.CollapsingToolbarBaseActivity;
import com.android.settingslib.widget.MainSwitchPreference;

public class PreferenceActivity extends CollapsingToolbarBaseActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getFragmentManager()
                .beginTransaction()
                .replace(com.android.settingslib.collapsingtoolbar.R.id.content_frame,
                        new SoundPreferenceFragment())
                .commit();
    }

    public static class SoundPreferenceFragment extends PreferenceFragment
        implements OnCheckedChangeListener {

        @Override
        public void onCreatePreferences(Bundle savedInstanceState, String rootKey) {
            addPreferencesFromResource(org.lineageos.settings.besloudness.R.xml.besloudness_panel);
            MainSwitchPreference toggle = (MainSwitchPreference)
                    findPreference(BesLoudnessManager.KEY_BESLOUDNESS);
            assert toggle != null;
            toggle.updateStatus(BesLoudnessManager.get(getContext()));
            toggle.addOnSwitchChangeListener(this);
        }

        @Override
        public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
            BesLoudnessManager.set(getContext(), isChecked);
        }
    }
}
