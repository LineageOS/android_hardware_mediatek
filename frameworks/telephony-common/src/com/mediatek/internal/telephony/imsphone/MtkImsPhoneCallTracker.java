/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

package com.mediatek.internal.telephony.imsphone;

import com.android.internal.telephony.flags.FeatureFlags;
import com.android.internal.telephony.imsphone.ImsPhone;
import com.android.internal.telephony.imsphone.ImsPhoneCallTracker;
import com.android.internal.telephony.imsphone.ImsPullCall;

public class MtkImsPhoneCallTracker extends ImsPhoneCallTracker implements ImsPullCall {
    public static final int IMS_SESSION_MODIFY_OPERATION_FLAG = 0x8000;

    public MtkImsPhoneCallTracker(ImsPhone phone, ConnectorFactory connectorFactory,
            FeatureFlags featureFlags) {
        super(phone, connectorFactory, featureFlags);
    }
}
