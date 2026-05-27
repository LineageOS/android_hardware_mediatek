/*
 * Copyright (C) 2025 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "libperfctl_vendor"

#include <log/log.h>

uint64_t earaNotifyCVJobBegin(uint64_t jobId, uint64_t jobPriority, uint64_t* result) {
    return 0;
}

uint64_t earaNotifyCVJobEnd(uint64_t jobId, uint64_t timestamp, uint32_t* status) {
    return 0;
}

void earaGetUsage(uint32_t param1, uint32_t* usage, uint32_t* status) {}

void earaNotifyJobBegin(uint32_t jobId, uint64_t jobData, int64_t* param3, int64_t* param4) {}

uint64_t earaNotifyJobEnd(uint32_t jobId, uint64_t timestamp, uint32_t* status, uint32_t flags,
                          int64_t* inputData, int64_t* outputData, uint64_t context,
                          int64_t* result) {
    return 0;
}

void fbcNotifyTouch(int touchEvent) {}

void xgfGetCamApkPid(uint32_t pid, uint32_t apkId, uint32_t flag) {}

void xgfGetCamServerPid(void) {}

void xgfGetFPS(int32_t* currentFPS, int32_t* averageFPS) {}

void xgfGetFstbActive(uint64_t status) {}

void xgfWaitFstbActive(void) {}

uint32_t xgfGetCmd(uint64_t* param) {
    return 0;
}
