#include <android-base/unique_fd.h>
#include <log/log.h>
#include <jni.h>

#include <time.h>
#include <string>
#include <errno.h>
#include <unistd.h>
#include <linux/ioctl.h>
#include <linux/rtc.h>

#define RTC_POFF_ALM_SET _IOW('p', 0x15, struct rtc_time)

static std::string rtc_dev = "/dev/rtc0";

extern "C" jboolean Java_org_lineageos_poweroffalarm_RtcDriver_isSupported(
            JNIEnv*, jobject) {
    return access(rtc_dev.c_str(), F_OK) == 0;
}

extern "C" void Java_org_lineageos_poweroffalarm_RtcDriver_set(
            JNIEnv*, jobject, jlong time) {
    android::base::unique_fd fd{open(rtc_dev.c_str(), O_RDWR)};
    if (!fd.ok()) {
        ALOGE("Unable to open %s: %s", rtc_dev.c_str(), strerror(errno));
        return;
    }

    struct tm tm = {};
    long ctime = time / 1000;
    if (!gmtime_r(&ctime, &tm)) {
        ALOGV("gmtime_r() failed: %s", strerror(errno));
        return;
    }

    struct rtc_time rtc = {};
    rtc.tm_sec = tm.tm_sec;
    rtc.tm_min = tm.tm_min;
    rtc.tm_hour = tm.tm_hour;
    rtc.tm_mday = tm.tm_mday;
    rtc.tm_mon = tm.tm_mon;
    rtc.tm_year = tm.tm_year;
    rtc.tm_wday = tm.tm_wday;
    rtc.tm_yday = tm.tm_yday;
    rtc.tm_isdst = tm.tm_isdst;
    if (ioctl(fd, RTC_POFF_ALM_SET, &rtc) == -1) {
        ALOGV("RTC_POFF_ALM_SET ioctl failed: %s", strerror(errno));
        return;
    }
}

extern "C" void Java_org_lineageos_poweroffalarm_RtcDriver_cancel(
            JNIEnv* env, jobject thiz) {
    Java_org_lineageos_poweroffalarm_RtcDriver_set(env, thiz, 0);
}

extern "C" jlong Java_org_lineageos_poweroffalarm_RtcDriver_getRtcTimeOptional(
            JNIEnv*, jobject) {
    android::base::unique_fd fd{open(rtc_dev.c_str(), O_RDWR)};
    if (!fd.ok()) {
        ALOGE("Unable to open %s: %s", rtc_dev.c_str(), strerror(errno));
        return -1;
    }

    struct rtc_time rtc = {};
    if (ioctl(fd, RTC_RD_TIME, &rtc) == -1) {
        ALOGV("RTC_RD_TIME ioctl failed: %s", strerror(errno));
        return -1;
    }

    struct tm tm = {};
    tm.tm_sec = rtc.tm_sec;
    tm.tm_min = rtc.tm_min;
    tm.tm_hour = rtc.tm_hour;
    tm.tm_mday = rtc.tm_mday;
    tm.tm_mon = rtc.tm_mon;
    tm.tm_year = rtc.tm_year;
    tm.tm_wday = rtc.tm_wday;
    tm.tm_yday = rtc.tm_yday;
    tm.tm_isdst = rtc.tm_isdst;
    long time = timegm(&tm);
    if (time < 0) {
        ALOGV("timegm() failed: %s", strerror(errno));
        return -1;
    }

    return time * 1000;
}
