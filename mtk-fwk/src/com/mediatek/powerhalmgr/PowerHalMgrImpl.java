package com.mediatek.powerhalmgr;

import android.content.Context;
import android.os.IRemoteCallback;

public class PowerHalMgrImpl extends PowerHalMgr {
    private static PowerHalMgrImpl sInstance = null;
    private static final Object lock = new Object();

    public static native int nativeGetPid();

    public static native int nativeGetTid();

    public static PowerHalMgrImpl getInstance() {
        synchronized (lock) {
            if (sInstance == null) {
                sInstance = new PowerHalMgrImpl();
            }
            return sInstance;
        }
    }

    public int scnReg() {
        return -1;
    }

    public void scnConfig(int handle, int cmd, int param_1, int param_2, int param_3, int param_4) {}

    public void scnUnreg(int handle) {}

    public void scnEnable(int handle, int timeout) {}

    public void scnDisable(int handle) {}

    public void scnUltraCfg(int handle, int ultracmd, int param_1, int param_2, int param_3, int param_4) {}

    public void mtkCusPowerHint(int hint, int data) {}

    public void getCpuCap() {}

    public void getGpuCap() {}

    public void getGpuRTInfo() {}

    public void getCpuRTInfo() {}

    public void UpdateManagementPkt(int type, String packet) {}

    public void setForegroundSports() {}

    public void setSysInfo(int type, String data) {}

    public boolean startDuplicatePacketPrediction() {
        return false;
    }

    public boolean stopDuplicatePacketPrediction() {
        return false;
    }

    public boolean isDupPacketPredictionStarted() {
        return false;
    }

    public boolean registerDuplicatePacketPredictionEvent(IRemoteCallback listener) {
        return false;
    }

    public boolean unregisterDuplicatePacketPredictionEvent(IRemoteCallback listener) {
        return false;
    }

    public boolean updateMultiDuplicatePacketLink(DupLinkInfo[] linkList) {
        return false;
    }

    public boolean setPriorityByUid(int action, int uid) {
        return false;
    }

    public boolean setPriorityByLinkinfo(int action, DupLinkInfo linkinfo) {
        return false;
    }

    public boolean flushPriorityRules(int type) {
        return false;
    }

    public boolean configBoosterInfo(BoosterInfo info) {
        return false;
    }

    public void setPredictInfo(String pack_name, int uid) {}

    public int perfLockAcquire(int handle, int duration, int[] list) {
        return handle;
    }

    public void perfLockRelease(int handle) {}

    public int perfCusLockHint(int hint, int duration) {
        return -1;
    }

    public int querySysInfo(int cmd, int param) {
        return -1;
    }

    public void mtkPowerHint(int hint, int data) {}

    public int setSysInfoSync(int type, String data) {
        return -1;
    }
}
