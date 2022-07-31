/*
 * Copyright (c) 2020, Mediatek Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef UFS_FFU_H_
#define UFS_FFU_H_


enum ffu_type {
	UFS_FFU = 0,
	UFS_CHECK_FFU_STATUS,
	UFS_FFU_MAX
};

void ffu_help(char *tool_name);
int do_ffu(struct tool_options *opt);
#endif /*UFS_FFU_H_*/
