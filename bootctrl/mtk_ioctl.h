/*
 * Copyright (c) 2020, Mediatek Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MTK_IOCTL_H_
#define MTK_IOCTL_H_


int ioctrl_w_attr(const char path[], uint8_t idn, uint8_t index, uint8_t selector, uint32_t value);
// int ioctrl_r_attr(int fd, uint8_t idn, uint8_t index, uint8_t selector, uint32_t *value);


#endif
