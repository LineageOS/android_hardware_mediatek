/*
 * Copyright (c) 2020, Mediatek Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Wrapping the ioctrl of linux kernel*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdint.h>
#include <endian.h>
#include <fcntl.h>
#include <unistd.h>

#include "ufs_cmds.h"
#include "options.h"
#include "ufs.h"



static void initialized_options(struct tool_options *options)
{
	memset(options, INVALID, sizeof(*options));
	options->path[0] = '\0';
	options->keypath[0] = '\0';
	options->data = NULL;
	options->sg_type = SG4_TYPE;
}


/**
 * @brief write attribute to bsg device
 * 
 * @param path device path e.g /dev/block/sdc
 * @param idn idn of attribute 
 * @param index index of attribute 
 * @param selector selector of attribute 
 * @param value the value to write to attribute. If the descriptor accepts only size less than DWORD, 
 * other bytes are ignored.
 * @return int return OK on success write.
 */
int ioctrl_w_attr(const char path[], uint8_t idn, uint8_t index, uint8_t selector, uint32_t value){
    
    struct tool_options options;
 	initialized_options(&options);
	
    options.config_type_inx = ATTR_TYPE;
	options.opr = WRITE; // write

	// TODO: check reseult
	// alloc buffer
	options.data = (uint32_t *)calloc(1, sizeof(uint32_t));

	// verify_write(&options);
	
	options.idn = idn;
	options.index = index;
	options.selector = selector;
	strcpy(options.path, path);// TODO: check copy error
	*((uint32_t *)options.data) = value;
		
	verify_arg_and_set_default(&options);
	
	if (do_attributes(&options)){
		print_error("attribute write failed");
		return ERROR;
	}

	return OK;
}


