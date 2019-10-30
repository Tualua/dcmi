/*
 * Part of Intel(R) Data Center Host Interface (DCMI-HI) Linux Driver
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

#ifndef _DCMI_DATA_STRUCTURES_H_
#define _DCMI_DATA_STRUCTURES_H_

#include <linux/module.h>
//#include <linux/spinlock.h>
//#include <linux/list.h>
struct inode;				//suppressing warning: ‘struct inode’ declared inside parameter list in cdev.h
#include <linux/cdev.h>


#define DCMI_MINORS_BASE	1
#define DCMI_MINORS_COUNT	1

#define  DCMI_MINOR_NUMBER	1
#define  DCMI_MAX_OPEN_HANDLE_COUNT	1	//maximum number of opened connections on /dev/dcmi

typedef enum _BOOLEAN {
	FALSE=0,
	TRUE,
} BOOLEAN;

struct guid {
	__u32 data1;
	__u16 data2;
	__u16 data3;
	__u8 data4[8];
} __attribute__((packed));


/* File state */
enum file_state {
        DCMI_FILE_INITIALIZING = 0,
        DCMI_FILE_CONNECTING,
        DCMI_FILE_CONNECTED,
        DCMI_FILE_DISCONNECTING,
        DCMI_FILE_DISCONNECTED
};

/* DCMI device states */
enum dcmi_states {
        DCMI_INITIALIZING = 0,
        DCMI_ENABLED,
        DCMI_RESETING,
        DCMI_DISABLED,
        DCMI_RECOVERING_FROM_RESET,
        DCMI_POWER_DOWN,
        DCMI_POWER_UP
};

enum dcmi_file_transaction_states {
        DCMI_IDLE,
        DCMI_WRITING,
        DCMI_WRITE_COMPLETE,
        DCMI_FLOW_CONTROL,
        DCMI_READING,
        DCMI_READ_COMPLETE
};

#if 0
/* DCMI user data struct used by ioctl interface */
struct dcmi_message_data {
	u32 size;
	char *data;
} __attribute__((packed));
#endif

/* DCMI device states */
typedef enum _DCMI_TRANSPORT_STATES {
        DCMI_NO_TRANSPORT = 0,
        DCMI_TRANS_ALLOCATED,
        DCMI_TRANS_NOT_OPENED,
        DCMI_TRANS_OPENED,
        DCMI_TRANS_OPENED_NO_READ,
        DCMI_TRANS_OPENED_NO_WRITE,
        DCMI_TRANS_NOT_CONNECTED,
        DCMI_TRANS_CONNECTED,
        DCMI_TRANS_ACTIVE,
} DCMI_TRANSPORT_STATES;

typedef struct _DCMI_TRANSPORT {

	atomic_t				state;	//DCMI_TRANSPORT_STATES

	char 					*buffer;
	int				 		transportBufferLength;
	char					*device_file_name;

	/* transport device file */
	struct file 			*filp;
	atomic_t 				resetRequested;
} DCMI_TRANSPORT;


/* Private file struct */
struct dcmi_file_private {
	//enum file_state state;
	spinlock_t file_lock; /* file lock */
};

struct task_struct;
/* private device struct */
struct dcmi_device {

	/* dcmi char device for registration */
	struct cdev cdev;

	/*
	 * list of files
	 */
	struct list_head file_list;

	/*
	 * lock for the device
	 */
	spinlock_t device_lock;

	/*
	 * lock for the transport
	 */
	spinlock_t trans_lock;

	/*
	 * dcmi device  states
	 */
	//enum dcmi_states
	atomic_t dcmi_state;

	long open_handle_count;

	/*
	 * Transport layer buffer list
	 */
	DCMI_TRANSPORT	 trans;
	struct list_head transMsgList;		//TRANS_MSG_ENTRY list, see dcmi_transport.h
	spinlock_t 		 transMsgListLock;	//protect operations on TRANS_MSG_ENTRY list
	atomic_t		 isTransOpen;		//BOOLEAN is open
	atomic_t		 isTransConnected;	//BOOLEAN and opened ready for communication

	struct task_struct *transReadThreadTask;
	atomic_t		 MsgCount;
	wait_queue_head_t trans_msg_wait;
};

#endif /* _DCMI_DATA_STRUCTURES_H_ */
