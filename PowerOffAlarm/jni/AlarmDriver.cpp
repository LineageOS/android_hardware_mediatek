#include <android-base/unique_fd.h>
#include <log/log.h>
#include <jni.h>

#include <time.h>
#include <string>
#include <errno.h>
#include <unistd.h>
#include <linux/ioctl.h>

#define ALARM_IOW(c, type, size) _IOW('a', (c) | ((type) << 4), size)
#define ANDROID_ALARM_SET(type) ALARM_IOW(2,type,struct timespec)
#define ANDROID_ALARM_POWER_ON_LOGO 7

static std::string alarm_dev = "/dev/alarm";

extern "C" jboolean Java_org_lineageos_poweroffalarm_AlarmDriver_isSupported(
            JNIEnv*, jobject) {
    return access(alarm_dev.c_str(), F_OK) == 0;
}

extern "C" void Java_org_lineageos_poweroffalarm_AlarmDriver_set(
            JNIEnv*, jobject, jlong time) {
    android::base::unique_fd fd{open(alarm_dev.c_str(), O_RDWR)};
    if (!fd.ok()) {
        ALOGE("Unable to open %s: %s", alarm_dev.c_str(), strerror(errno));
        return;
    }

    struct timespec ts;
    ts.tv_sec = time / 1000;
    ts.tv_nsec = (time % 1000) * 1000000;

    if (ioctl(fd, ANDROID_ALARM_SET(ANDROID_ALARM_POWER_ON_LOGO), &ts) == -1) {
        ALOGV("ANDROID_ALARM_SET ioctl failed: %s", strerror(errno));
        return;
    }
}

extern "C" void Java_org_lineageos_poweroffalarm_AlarmDriver_cancel(
            JNIEnv* env, jobject thiz) {
    Java_org_lineageos_poweroffalarm_AlarmDriver_set(env, thiz, 0);
}

extern "C" jlong Java_org_lineageos_poweroffalarm_AlarmDriver_getRtcTimeOptional(
            JNIEnv*, jobject) {
    return -1L;
}
