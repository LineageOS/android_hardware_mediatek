/*
 * Copyright (C) 2014 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ANDROID_SYSTEM_MEDIA_ALSA_UTILS_ALSA_FORMAT_H
#define ANDROID_SYSTEM_MEDIA_ALSA_UTILS_ALSA_FORMAT_H

#include <system/audio.h>

#include <tinyalsa/asoundlib.h>

enum pcm_format get_pcm_format_for_mask(struct pcm_mask* mask);

#endif /* ANDROID_SYSTEM_MEDIA_ALSA_UTILS_ALSA_FORMAT_H */
