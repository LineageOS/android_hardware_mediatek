#include <android-base/unique_fd.h>
#include <log/log.h>
#include <jni.h>

#include <time.h>
#include <string>
#include <errno.h>
#include <unistd.h>
#include <linux/ioctl.h>
#include <sys/timerfd.h>

extern "C" jboolean Java_org_lineageos_poweroffalarm_TimerFdDriver_isSupported(
            JNIEnv*, jobject) {
#ifdef CLOCK_POWER_OFF_ALARM
    return true;
#else
    return false;
#endif
}

extern "C" void Java_org_lineageos_poweroffalarm_TimerFdDriver_set(
            JNIEnv*, jobject,
#ifdef CLOCK_POWER_OFF_ALARM
            jlong time) {
    android::base::unique_fd fd{timerfd_create(CLOCK_POWER_OFF_ALARM, TFD_NONBLOCK)};
    if (!fd.ok()) {
        ALOGE("Unable to open CLOCK_POWER_OFF_ALARM: %s", strerror(errno));
        return;
    }

    struct timespec ts;
    ts.tv_sec = time / 1000;
    ts.tv_nsec = (time % 1000) * 1000000;

    struct itimerspec spec;
    memset(&spec, 0, sizeof(spec));
    memcpy(&spec.it_value, &ts, sizeof(spec.it_value));

    timerfd_settime(fd, TFD_TIMER_ABSTIME, &spec, NULL);
#else
        jlong) {
#endif
}

extern "C" void Java_org_lineageos_poweroffalarm_TimerFdDriver_cancel(
            JNIEnv*, jobject) {
#ifdef CLOCK_POWER_OFF_ALARM
    android::base::unique_fd fd{timerfd_create(CLOCK_POWER_OFF_ALARM, TFD_NONBLOCK)};
    if (!fd.ok()) {
        ALOGE("Unable to open CLOCK_POWER_OFF_ALARM: %s", strerror(errno));
        return;
    }

    struct itimerspec spec = {};
    timerfd_settime(fd, TFD_TIMER_ABSTIME, &spec, NULL);
#endif
}

extern "C" jlong Java_org_lineageos_poweroffalarm_TimerFdDriver_getRtcTimeOptional(
            JNIEnv*, jobject) {
    return -1L;
}
