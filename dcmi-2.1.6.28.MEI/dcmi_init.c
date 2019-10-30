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

#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/moduleparam.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include "kcompat.h"

#include "dcmi_data_structures.h"
#include "dcmi.h"

/**
 * dcmi_init_file_private - initializes private file structure.
 *
 * @priv: private file structure to be initialized
 * @file: the file structure
 *
 */
static void dcmi_init_file_private(struct dcmi_file_private *priv,
				   struct file *file)
{
//	INIT_LIST_HEAD(&priv->link);
	memset(priv, 0, sizeof(struct dcmi_file_private));
//	priv->state = DCMI_FILE_INITIALIZING;
	spin_lock_init(&priv->file_lock);
}

/**
 * dcmi_alloc_file_private - allocates a private file structure and set it up.
 * @file: the file structure
 *
 * @return  The allocated file or NULL on failure
 */
struct dcmi_file_private *dcmi_alloc_file_private(struct file *file)
{
	static struct dcmi_file_private *priv;

	priv = kmalloc(sizeof(struct dcmi_file_private), GFP_KERNEL);
	if (!priv)
		return NULL;

	dcmi_init_file_private(priv, file);

	return priv;
}

/**
 * init_dcmi_device - allocates and initializes the dcmi device structure
 *
 * @param device - pointing our dcmi device structure
 *
 * @return The dcmi_device_device pointer on success, NULL on failure.
 */
struct dcmi_device *init_dcmi_device(struct dcmi_device *device)
{


	INIT_LIST_HEAD(&device->file_list);
	spin_lock_init(&device->device_lock);
	spin_lock_init(&device->trans_lock);

	atomic_set(&device->dcmi_state, DCMI_INITIALIZING);

	device->open_handle_count = 0;

	/*
	 * Transport cache filled with messages by transport thread
	 * started
	 */
	spin_lock_init(&device->transMsgListLock);
	INIT_LIST_HEAD(&device->transMsgList);
	atomic_set(&device->isTransOpen, FALSE);
	atomic_set(&device->isTransConnected,FALSE);
	atomic_set(&device->MsgCount, 0);
	//wait queue initialization for signalling of message availability
	init_waitqueue_head(&device->trans_msg_wait);

	return device;
}
