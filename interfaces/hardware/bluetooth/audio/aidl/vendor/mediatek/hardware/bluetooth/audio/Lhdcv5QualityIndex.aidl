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

@VintfStability
@Backing(type="int")
enum Lhdcv5QualityIndex {
    UNKNOWN = 0,
    QUALITY_LOW0 = 1, // 64
    QUALITY_LOW1 = 2, // 128
    QUALITY_LOW2 = 3, // 192
    QUALITY_LOW3 = 4, // 256
    QUALITY_LOW4 = 5, // 320
    QUALITY_LOW = 6, // 400
    QUALITY_MID = 7, // 500
    QUALITY_HIGH = 8, // 900
    QUALITY_HIGH1 = 9, // 1000 (supported in LHDCV5+)
    QUALITY_ABR = 10, // ABR mode
}
