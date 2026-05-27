/*
 * Copyright (C) 2023 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "libpowerhalwrap_vendor"

#include <log/log.h>

int PowerHal_Wrap_mtkPowerHint(int hint, int data) {
    return 0;
}

int PowerHal_Wrap_mtkCusPowerHint(int hint, int data) {
    return 0;
}

int PowerHal_Wrap_querySysInfo(unsigned int param, unsigned int data) {
    return 0;
}

int64_t PowerHal_Wrap_notifyAppState(const char* pname, const char* aname, unsigned int pid,
                                     int status, unsigned int uid) {
    return 0;
}

int PowerHal_Wrap_scnReg() {
    return 0;
}

int PowerHal_Wrap_scnConfig() {
    return 0;
}

int PowerHal_Wrap_scnUnreg() {
    return 0;
}

int PowerHal_Wrap_scnEnable() {
    return 0;
}

int PowerHal_Wrap_scnDisable() {
    return 0;
}

int PowerHal_Wrap_scnUltraCfg() {
    return 0;
}

int PowerHal_TouchBoost(int duration) {
    return 0;
}

int PowerHal_Wrap_setSysInfo(int type, const char* data) {
    return 0;
}

int PowerHal_Wrap_setSysInfoAsync(int type, const char* data) {
    return 0;
}

int PowerHal_Wrap_EnableMultiDisplayMode(int enable, int fps) {
    return 0;
}
