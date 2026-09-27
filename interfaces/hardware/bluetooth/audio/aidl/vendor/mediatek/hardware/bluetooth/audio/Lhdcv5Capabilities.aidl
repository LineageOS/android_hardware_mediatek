/*
 * Copyright 2021 The Android Open Source Project
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

package vendor.mediatek.hardware.bluetooth.audio;

import vendor.mediatek.hardware.bluetooth.audio.ChannelMode;
import vendor.mediatek.hardware.bluetooth.audio.Lhdcv5DataInterval;
import vendor.mediatek.hardware.bluetooth.audio.Lhdcv5FrameDuration;
import vendor.mediatek.hardware.bluetooth.audio.Lhdcv5QualityIndex;
import vendor.mediatek.hardware.bluetooth.audio.Lhdcv5Specific;
import vendor.mediatek.hardware.bluetooth.audio.Lhdcv5Version;

@VintfStability
parcelable Lhdcv5Capabilities {
    int[] sampleRateHz;
    ChannelMode[] channelMode;
    byte[] bitsPerSample;
    Lhdcv5Version[] codecVersion;
    Lhdcv5QualityIndex[] qualityIndex;
    Lhdcv5QualityIndex[] maxQualityIndex;
    Lhdcv5QualityIndex[] minQualityIndex;
    Lhdcv5FrameDuration[] frameDuration;
    Lhdcv5DataInterval[] dataInterval;
    Lhdcv5Specific[] codecSpecific_1;
    Lhdcv5Specific[] codecSpecific_2;
    byte[] metaData;
}
