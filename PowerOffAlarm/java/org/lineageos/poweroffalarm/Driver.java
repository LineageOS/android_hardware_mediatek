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

package org.lineageos.poweroffalarm;

/**
 * Driver interface
 * guranteed to be called on single thread
 */
public interface Driver {

    /**
     * Set auto boot to RTC time
     * @param time the unix time (as per RTC) when to wake up
     */
    public void set(long time);

    /**
     * cancel RTC auto boot
     */
    public void cancel();

    /**
     * get current unix time as per RTC if supported
     * used to account for differences between RTC and AP time
     * @return value if supported, or -1L
     */
    public long getRtcTimeOptional();

    /**
     * indicate if this driver backend is supported for RTC wakeup
     * if this method returns false, it is guranteed that no other methods wil be called
     * @return true if this driver is supported
     */
    public boolean isSupported();

}
