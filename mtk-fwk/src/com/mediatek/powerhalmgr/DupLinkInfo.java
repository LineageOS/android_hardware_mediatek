package com.mediatek.powerhalmgr;

import android.os.Parcel;
import android.os.Parcelable;

public class DupLinkInfo implements Parcelable {
    private String mSrcIp;
    private String mDstIp;
    private int mSrcPort;
    private int mDstPort;
    private int mProto;

    public DupLinkInfo(String src_ip, String dst_ip, int src_port, int dst_port, int proto) {
        mSrcIp = src_ip;
        mDstIp = dst_ip;
        mSrcPort = src_port;
        mDstPort = dst_port;
        mProto = proto;
    }

    protected DupLinkInfo(Parcel in) {
        mSrcIp = in.readString();
        mDstIp = in.readString();
        mSrcPort = in.readInt();
        mDstPort = in.readInt();
        mProto = in.readInt();
    }

    public static final Creator<DupLinkInfo> CREATOR = new Creator<DupLinkInfo>() {
        @Override
        public DupLinkInfo createFromParcel(Parcel in) {
            return new DupLinkInfo(in);
        }

        @Override
        public DupLinkInfo[] newArray(int size) {
            return new DupLinkInfo[size];
        }
    };

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel parcel, int flags) {
        parcel.writeString(mSrcIp);
        parcel.writeString(mDstIp);
        parcel.writeInt(mSrcPort);
        parcel.writeInt(mDstPort);
        parcel.writeInt(mProto);
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

    @Override
    public String toString() {
        return "DupLinkInfo(" + mSrcIp + "," + mDstIp + "," + mSrcPort + "," + mDstPort + "," + mProto + ")";
    }
}
