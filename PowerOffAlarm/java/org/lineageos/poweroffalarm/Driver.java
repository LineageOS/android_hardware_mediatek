/*
 * SPDX-FileCopyrightText: 2023 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
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
     * if this method returns false, it is guranteed that no other methods will be called
     * @return true if this driver is supported
     */
    public boolean isSupported();

}
