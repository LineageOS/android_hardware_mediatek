/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

package mediatek.telecom;

public class MtkVideoProfile {
    private static final int MTK_STATE_BASE = 0x00010000;
    public static final int STATE_CANCEL_UPGRADE = MTK_STATE_BASE << 0;
    public static final int STATE_CANCEL_UPGRADE_FOR_TIMEOUT = MTK_STATE_BASE << 1;
}
