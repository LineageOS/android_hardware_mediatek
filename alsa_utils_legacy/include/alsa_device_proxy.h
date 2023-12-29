/*
 * Copyright (C) 2014 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ANDROID_SYSTEM_MEDIA_ALSA_UTILS_ALSA_DEVICE_PROXY_H
#define ANDROID_SYSTEM_MEDIA_ALSA_UTILS_ALSA_DEVICE_PROXY_H

#include <tinyalsa/asoundlib.h>

#include "alsa_device_profile.h"

typedef struct {
    const alsa_device_profile* profile;

    struct pcm_config alsa_config;

    struct pcm * pcm;

    size_t frame_size;    /* valid after proxy_prepare(), the frame size in bytes */
    uint64_t transferred; /* the total frames transferred, not cleared on standby */
} alsa_device_proxy;


/* State */
int proxy_prepare(alsa_device_proxy * proxy, const alsa_device_profile * profile,
                   struct pcm_config * config);
int proxy_open(alsa_device_proxy * proxy);
void proxy_close(alsa_device_proxy * proxy);
int proxy_get_presentation_position(const alsa_device_proxy * proxy,
        uint64_t *frames, struct timespec *timestamp);
int proxy_get_capture_position(const alsa_device_proxy * proxy,
        int64_t *frames, int64_t *time);

/* Attributes */
unsigned proxy_get_sample_rate(const alsa_device_proxy * proxy);
enum pcm_format proxy_get_format(const alsa_device_proxy * proxy);
unsigned proxy_get_channel_count(const alsa_device_proxy * proxy);
unsigned int proxy_get_period_size(const alsa_device_proxy * proxy);
unsigned proxy_get_latency(const alsa_device_proxy * proxy);

/*
 * Scans the provided list of sample rates and finds the first one that works.
 *
 * returns the index of the first rate for which the ALSA device can be opened.
 * return negative value if none work or an error occurs.
 */
int proxy_scan_rates(alsa_device_proxy * proxy, const unsigned sample_rates[]);

/* I/O */
int proxy_write(alsa_device_proxy * proxy, const void *data, unsigned int count);
int proxy_read(alsa_device_proxy * proxy, void *data, unsigned int count);

/* Debugging */
void proxy_dump(const alsa_device_proxy * proxy, int fd);

#endif /* ANDROID_SYSTEM_MEDIA_ALSA_UTILS_ALSA_DEVICE_PROXY_H */
