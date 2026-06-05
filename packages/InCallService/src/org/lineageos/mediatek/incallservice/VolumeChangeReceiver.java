/*
 * Copyright (C) 2023 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

package org.lineageos.mediatek.incallservice;

import android.content.Intent;
import android.content.Context;
import android.content.BroadcastReceiver;

import android.media.AudioManager;
import android.media.AudioSystem;
import android.media.AudioDeviceInfo;

import android.util.Log;

public class VolumeChangeReceiver extends BroadcastReceiver {
    public static final String LOG_TAG = "MediatekInCallService";

    private AudioManager mAudioManager;

    public VolumeChangeReceiver(AudioManager audioManager) {
        mAudioManager = audioManager;
    }

    private void handleVolumeStateChange(Intent intent) {
        int streamType = intent.getIntExtra(AudioManager.EXTRA_VOLUME_STREAM_TYPE, -1);

        if (streamType == AudioManager.STREAM_VOICE_CALL) {
            AudioDeviceInfo callDevice = mAudioManager.getCommunicationDevice();

            int volumeIndex = intent.getIntExtra(AudioManager.EXTRA_VOLUME_STREAM_VALUE, -1);
            if (volumeIndex == -1)
                volumeIndex = mAudioManager.getStreamVolume(AudioManager.STREAM_VOICE_CALL);

            GainUtils.setGainLevel(callDevice.getPort().type(), volumeIndex, AudioSystem.STREAM_VOICE_CALL);
        } else if (GainUtils.getMusicGainEnabled() && streamType == AudioManager.STREAM_MUSIC) {
            int volumeIndex = intent.getIntExtra(AudioManager.EXTRA_VOLUME_STREAM_VALUE, -1);
            if (volumeIndex == -1)
                volumeIndex = mAudioManager.getStreamVolume(AudioManager.STREAM_MUSIC);
            GainUtils.setMusicGainLevel(volumeIndex);
        }
    }

    @Override
    public void onReceive(Context context, Intent intent) {
            handleVolumeStateChange(intent);
    }
}
