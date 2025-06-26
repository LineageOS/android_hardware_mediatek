package com.mediatek.powerhalmgr;

import android.os.Parcel;
import android.os.Parcelable;
import java.util.Arrays;

public class BoosterInfo implements Parcelable {
    public static final int BOOSTER_ACTION_ADD_BY_LINKINFO = 4;
    public static final int BOOSTER_ACTION_ADD_BY_UID = 1;
    public static final int BOOSTER_ACTION_BASE = 0;
    public static final int BOOSTER_ACTION_DEL_ALL = 7;
    public static final int BOOSTER_ACTION_DEL_BY_LINKINFO = 5;
    public static final int BOOSTER_ACTION_DEL_BY_LINKINFO_ALL = 6;
    public static final int BOOSTER_ACTION_DEL_BY_UID = 2;
    public static final int BOOSTER_ACTION_DEL_BY_UID_ALL = 3;

    public static int BOOSTER_GROUP_BASE = 0;
    public static final int BOOSTER_GROUP_A = BOOSTER_GROUP_BASE + 1;
    public static final int BOOSTER_GROUP_B = BOOSTER_GROUP_BASE + 2;
    public static final int BOOSTER_GROUP_C = BOOSTER_GROUP_BASE + 3;
    public static final int BOOSTER_GROUP_D = BOOSTER_GROUP_BASE + 4;
    public static final int BOOSTER_GROUP_MAX = BOOSTER_GROUP_D;

    private int mGroup;
    private int mAction;
    private int mUid;
    private String mSrcIp;
    private String mDstIp;
    private int mSrcPort;
    private int mDstPort;
    private int mProto;
    private String[] mMoreInfo;
    private int[] mMoreValue;

    public BoosterInfo(int group, int action, int uid, String srcIp, String dstIp, int srcPort, int dstPort, int proto, String[] moreInfo, int[] moreValue) {
        this.mGroup = group;
        this.mAction = action;
        this.mUid = uid;
        this.mSrcIp = srcIp;
        this.mDstIp = dstIp;
        this.mSrcPort = srcPort;
        this.mDstPort = dstPort;
        this.mProto = proto;
        this.mMoreInfo = moreInfo;
        this.mMoreValue = moreValue;
    }

    protected BoosterInfo(Parcel in) {
        mGroup = in.readInt();
        mAction = in.readInt();
        mUid = in.readInt();
        mSrcIp = in.readString();
        mDstIp = in.readString();
        mSrcPort = in.readInt();
        mDstPort = in.readInt();
        mProto = in.readInt();
        mMoreInfo = in.createStringArray();
        mMoreValue = in.createIntArray();
    }

    public static final Creator<BoosterInfo> CREATOR = new Creator<BoosterInfo>() {
        @Override
        public BoosterInfo createFromParcel(Parcel in) {
            return new BoosterInfo(in);
        }

        @Override
        public BoosterInfo[] newArray(int size) {
            return new BoosterInfo[size];
        }
    };

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel out, int flags) {
        out.writeInt(mGroup);
        out.writeInt(mAction);
        out.writeInt(mUid);
        out.writeString(mSrcIp);
        out.writeString(mDstIp);
        out.writeInt(mSrcPort);
        out.writeInt(mDstPort);
        out.writeInt(mProto);
        out.writeStringArray(mMoreInfo);
        out.writeIntArray(mMoreValue);
    }

    public int getGroup() {
        return mGroup;
    }

    public int getAction() {
        return mAction;
    }

    public int getUid() {
        return mUid;
    }

    public String getSrcIp() {
        return mSrcIp;
    }

    public String getDstIp() {
        return mDstIp;
    }

    public int getSrcPort() {
        return mSrcPort;
    }

    public int getDstPort() {
        return mDstPort;
    }

    public int getProto() {
        return mProto;
    }

    public String[] getMoreInfo() {
        return mMoreInfo;
    }

    public int[] getMoreValue() {
        return mMoreValue;
    }

    @Override
    public String toString() {
        return "BoosterInfo(" + mGroup + "," + mAction + "," + mUid + "," + mSrcIp + "," + mDstIp + "," + mSrcPort + "," + mDstPort + "," + mProto + "," + Arrays.toString(mMoreInfo) + "," + Arrays.toString(mMoreValue) + ")";
    }
}
