/*
 * Part of Intel(R) Data Center Host Interface (DCMI-HI) Linux driver
 *
 * Copyright (c) 2010 Intel Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    substantially similar to the "NO WARRANTY" disclaimer below
 *    ("Disclaimer") and any redistribution must be conditioned upon
 *    including a substantially similar Disclaimer requirement for further
 *    binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES.
 *
 */

#ifndef _DCMI_H
#define _DCMI_H

#include "dcmi_data_structures.h"

/* DCMI over HECI1, HECI1 cyclic buffer is 512 */
#define DCMI_MAX_MSG_LEN	128
#define DCMI_MIN_MSG_LEN	6

typedef enum _DCMI_STATUS
{
	TRANS_NOT_FOUND=0,
	TRANS_INTERFACE_NOT_FOUND,
	TRANS_DRIVER_FAIL,
	TRANS_NOT_OPEN,
	TRANS_BUS_ERROR,
	TRANS_TIMEOUT,
	TRANS_WRITE_ERROR,
	TRANS_READ_ERROR,
	TRANS_INSUFFICIENT_RESOURCE,
	TRANS_RECONNECT_NEEDED,
	DCMI_INTERNAL_ERROR,		//10
	DCMI_CONNECT_FAIL,
	DCMI_WRITE_FAIL,
	DCMI_READ_FAIL,
	DCMI_NOT_CONNECTED,
	DCMI_INVALID_MSG_LEN,
	DCMI_MSG_NOT_AVAILABLE,
	DCMI_OK,					//17
	TRANS_OK,
} DCMI_STATUS;

/**
 * debug kernel print macro define
 */
extern int dcmi_debug;

#define DBG(format, arg...) \
do { \
	if (dcmi_debug) \
		printk(KERN_ERR "%s(%d): " format, __func__ , __LINE__, ## arg); \
} while (0)

/**
 * dcmi init function prototypes
 */
struct dcmi_device *init_dcmi_device(struct dcmi_device *device);

struct dcmi_file_private *dcmi_alloc_file_private(struct file *file);

#endif /* _DCMI_H_ */
