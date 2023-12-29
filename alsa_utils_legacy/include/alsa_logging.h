/*
 * Copyright (C) 2014 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ANDROID_SYSTEM_MEDIA_ALSA_UTILS_ALSA_LOGGING_H
#define ANDROID_SYSTEM_MEDIA_ALSA_UTILS_ALSA_LOGGING_H

#include <tinyalsa/asoundlib.h>

void log_pcm_mask(const char* mask_name, struct pcm_mask* mask);
void log_pcm_params(struct pcm_params * alsa_hw_params);
void log_pcm_config(struct pcm_config * config, const char* label);

#endif /* ANDROID_SYSTEM_MEDIA_ALSA_UTILS_ALSA_LOGGING_H */
