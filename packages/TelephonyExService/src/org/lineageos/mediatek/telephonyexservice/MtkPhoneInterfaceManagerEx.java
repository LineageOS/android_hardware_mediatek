/*
 * SPDX-FileCopyrightText: 2026 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

package org.lineageos.mediatek.telephonyexservice;

import android.content.Context;
import android.content.Intent;
import android.os.AsyncResult;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.os.ServiceManager;
import android.os.SystemProperties;
import android.preference.PreferenceManager;
import android.telephony.TelephonyManager;
import android.telephony.UiccSlotMapping;
import android.util.Log;

import com.android.internal.telephony.CommandsInterface;
import com.android.internal.telephony.PhoneFactory;
import com.android.internal.telephony.RadioConfig;
import org.lineageos.mediatek.telephonyexservice.IMtkTelephonyEx;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MtkPhoneInterfaceManagerEx extends IMtkTelephonyEx.Stub {
    private static final String TAG = "MtkPhoneIntfMgrEx";

    // Command codes
    private static final int CMD_SWITCH_SIM_TYPE = 116;
    private static final int EVENT_SWITCH_SIM_TYPE_DONE = 117;
    private static final int EVENT_SIM_TYPE_CHANGED_IND = 118;
    private static final int EVENT_GET_SIM_TYPE_DONE = 119;
    private static final int EVENT_INTERNAL_SLOT_MAPPING_DONE = 2002;

    private static final String SLOT_MAPPING_STATUS = "slot_mapping_status";

    private final Context mContext;
    private final MainThreadHandler mMainThreadHandler;

    private CommandsInterface[] mCis;
    private int[] mPhoneIdToSlotId;
    private int[] mCanChangeType;
    private int[] mSimType;

    public MtkPhoneInterfaceManagerEx(Context context) {
        mContext = context;
        mMainThreadHandler = new MainThreadHandler();

        ServiceManager.addService("phoneEx", this);
        Log.i(TAG, "phoneEx service registered");

        mMainThreadHandler.post(this::initWhenPhoneFactoryReady);
    }

    private void initWhenPhoneFactoryReady() {
        mCis = PhoneFactory.getCommandsInterfaces();

        if (mCis == null || mCis.length == 0) {
            Log.w(TAG, "PhoneFactory not ready yet, retrying in 2 seconds...");
            mMainThreadHandler.postDelayed(this::initWhenPhoneFactoryReady, 2000);
            return;
        }

        int activeModems = mCis.length;

        mPhoneIdToSlotId = new int[activeModems];
        mCanChangeType = new int[activeModems];
        mSimType = new int[activeModems];

        Arrays.fill(mCanChangeType, -1);
        Arrays.fill(mSimType, -1);

        for (int i = 0; i < activeModems; i++) {
            mPhoneIdToSlotId[i] = i;
            invokeRegisterForSimTypeChanged(mCis[i], mMainThreadHandler, EVENT_SIM_TYPE_CHANGED_IND, Integer.valueOf(i));

            // Query the initial eSIM/pSIM state from the modem immediately on boot
            invokeGetSimType(mCis[i], mMainThreadHandler.obtainMessage(EVENT_GET_SIM_TYPE_DONE, Integer.valueOf(i)));
        }

        Log.i(TAG, "RIL connections established successfully for phoneEx");
    }

    private void invokeGetSimType(CommandsInterface ci, Message response) {
        try {
            Method m = ci.getClass().getMethod("getSimType", Message.class);
            m.invoke(ci, response);
        } catch (Exception e) {
            Log.w(TAG, "getSimType not found on runtime RIL.");
        }
    }

    private void invokeRegisterForSimTypeChanged(CommandsInterface ci, Handler h, int what, Object obj) {
        try {
            Method m = ci.getClass().getMethod("registerForSimTypeChanged", Handler.class, int.class, Object.class);
            m.invoke(ci, h, what, obj);
        } catch (Exception e) {
            Log.w(TAG, "registerForSimTypeChanged not found on runtime RIL.");
        }
    }

    private void invokeSwitchSimType(CommandsInterface ci, int mode, Message response) {
        try {
            Method m = ci.getClass().getMethod("switchSimType", int.class, Message.class);
            m.invoke(ci, mode, response);
        } catch (Exception e) {
            Log.e(TAG, "switchSimType not found on runtime RIL", e);
            if (response != null) {
                AsyncResult.forMessage(response, null, new RuntimeException("switchSimType not supported"));
                response.sendToTarget();
            }
        }
    }

    private static final class MainThreadRequest {
        public Object argument;
        public Integer subId;
        public Object result;

        public MainThreadRequest(Object argument, Integer subId) {
            this.argument = argument;
            this.subId = subId;
        }
    }

    private Object sendRequest(int command, Object argument, Integer subId) {
        if (Looper.myLooper() == mMainThreadHandler.getLooper()) {
            throw new RuntimeException("Deadlock: called from main thread.");
        }
        MainThreadRequest request = new MainThreadRequest(argument, subId);
        mMainThreadHandler.obtainMessage(command, request).sendToTarget();

        synchronized (request) {
            while (request.result == null) {
                try { request.wait(); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
            }
        }
        return request.result;
    }

    private class MainThreadHandler extends Handler {
        @Override
        public void handleMessage(Message message) {
            switch (message.what) {
                case CMD_SWITCH_SIM_TYPE:
                    MainThreadRequest request = (MainThreadRequest) message.obj;
                    int simType = ((Integer) request.argument).intValue();
                    int phoneId = request.subId.intValue();

                    int srcSlotId = mPhoneIdToSlotId[phoneId];
                    int targetSlotId = resetCorrectSlotId(srcSlotId, simType);
                    if (targetSlotId != srcSlotId) {
                        phoneId = findPhoneIdBySlotId(targetSlotId);
                    }

                    new SwitchSimTypeWorker(simType, phoneId, obtainMessage(EVENT_SWITCH_SIM_TYPE_DONE, request)).execute();
                    break;

                case EVENT_SWITCH_SIM_TYPE_DONE:
                    AsyncResult arDone = (AsyncResult) message.obj;
                    MainThreadRequest reqDone = (MainThreadRequest) arDone.userObj;

                    if (arDone.exception == null) {
                        reqDone.result = 0; // Success
                    } else {
                        reqDone.result = 10; // Error
                    }
                    synchronized (reqDone) { reqDone.notifyAll(); }
                    break;

                case EVENT_GET_SIM_TYPE_DONE:
                case EVENT_SIM_TYPE_CHANGED_IND:
                    AsyncResult arInd = (AsyncResult) message.obj;
                    Integer index = (Integer) arInd.userObj;

                    if (arInd.exception == null && arInd.result != null && index != null) {
                        int pId = index.intValue();
                        if (supportEsimSwitch(mPhoneIdToSlotId[pId])) {
                            String[] responseArray = (String[]) arInd.result;
                            if (responseArray.length >= 2) {
                                mCanChangeType[pId] = responseArray[0] != null ? Integer.valueOf(responseArray[0]).intValue() : 0;
                                mSimType[pId] = responseArray[1] != null ? Integer.valueOf(responseArray[1]).intValue() : 0;

                                Intent simTypeIntent = new Intent("com.mediatek.intent.action.ACTION_SIM_TYPE_CHANGED");
                                simTypeIntent.putExtra("SIM_TYPE_RESULT", mCanChangeType[pId] == 1);
                                simTypeIntent.putExtra("SIM_TYPE", mSimType[pId]);
                                mContext.sendBroadcast(simTypeIntent);
                            }
                        }
                    }
                    break;
            }
        }
    }

    private class SwitchSimTypeWorker extends Handler {
        private final int mode;
        private final int phoneId;
        private final Message callbackMessage;
        private final List<UiccSlotMapping> slotMappingList;

        public SwitchSimTypeWorker(int mode, int phoneId, Message callbackMessage) {
            this.mode = mode;
            this.phoneId = phoneId;
            this.callbackMessage = callbackMessage;
            this.slotMappingList = getSlotMappingList(mode);
        }

        public void execute() {
            if (this.mode == 1 || this.slotMappingList == null || this.slotMappingList.isEmpty()) {
                invokeSwitchSimType(mCis[this.phoneId], this.mode, this.callbackMessage);
            } else {
                RadioConfig.getInstance().setSimSlotsMapping(this.slotMappingList, obtainMessage(EVENT_INTERNAL_SLOT_MAPPING_DONE));
            }
        }

        @Override
        public void handleMessage(Message msg) {
            if (msg.what == EVENT_INTERNAL_SLOT_MAPPING_DONE) {
                if (this.mode == 0) {
                    invokeSwitchSimType(mCis[this.phoneId], this.mode, this.callbackMessage);
                }
            }
        }

        private List<UiccSlotMapping> getSlotMappingList(int simType) {
            int mappingValue = PreferenceManager.getDefaultSharedPreferences(mContext).getInt(SLOT_MAPPING_STATUS, 0);
            if (mappingValue == 0) {
                if ("tsts".equals(SystemProperties.get("persist.radio.multisim.config"))) {
                    List<UiccSlotMapping> mappings = new ArrayList<>();
                    mappings.add(new UiccSlotMapping(0, 0, 0));
                    mappings.add(new UiccSlotMapping(simType == 0 ? 0 : 1, 1, 1));
                    return mappings;
                }
                return null;
            }
            List<UiccSlotMapping> mappings = new ArrayList<>();
            mappings.add(new UiccSlotMapping(0, 0, 0));
            mappings.add(new UiccSlotMapping(0, 1, 1));
            return mappings;
        }
    }

    private boolean supportEsimSwitch(int slot_id) {
        return mContext != null && slot_id == 1;
    }

    private int findPhoneIdBySlotId(int slotId) {
        for (int i = 0; i < mPhoneIdToSlotId.length; i++) {
            if (mPhoneIdToSlotId[i] == slotId) return i;
        }
        return 0;
    }

    private int resetCorrectSlotId(int slotId, int mode) {
        if (mode == 0) {
            for (int i = 0; i < mSimType.length; i++) {
                if (mSimType[i] == 1) return i;
            }
        }
        return slotId;
    }

    @Override
    public boolean supportSimTypeSwitch() {
        if (mCis == null || TelephonyManager.getDefault().getActiveModemCount() <= 1) return false;

        if (mCanChangeType != null) {
            for (int i = 0; i < mCanChangeType.length; i++) {
                if (mCanChangeType[i] == 1) {
                    return true;
                }
            }
        }
        return false;
    }

    @Override
    public int getCurrentSimType(int slotId) {
        if (mPhoneIdToSlotId == null || mSimType == null) return -1;

        int phoneId = -1;
        for (int i = 0; i < mPhoneIdToSlotId.length; i++) {
            if (mPhoneIdToSlotId[i] == slotId) {
                phoneId = i;
                break;
            }
        }
        if (phoneId < 0 || phoneId >= mSimType.length) return -1;
        return mSimType[phoneId];
    }

    @Override
    public int switchSimType(int slotId, int simType) {
        if (mCis == null || !supportSimTypeSwitch() || TelephonyManager.getDefault().getActiveModemCount() <= 1) {
            return -1;
        }
        Log.i(TAG, "switchSimType slotId: " + slotId + " simType: " + simType);

        int phoneId = -1;
        for (int i = 0; i < mPhoneIdToSlotId.length; i++) {
            if (mPhoneIdToSlotId[i] == slotId) {
                phoneId = i;
                break;
            }
        }
        if (phoneId < 0) return -1;

        return ((Integer) sendRequest(CMD_SWITCH_SIM_TYPE, Integer.valueOf(simType), Integer.valueOf(phoneId))).intValue();
    }
}
