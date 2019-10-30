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

#ifndef _DCMI_TRANSPORT_H
#define _DCMI_TRANSPORT_H

#include "dcmi.h"

#define TIMEOUT						10000	/* mili seconds*/
#define TRANS_SEQUENCE_START 	 	0
#define TRANS_SEQUENCE_NO_OFFSET 	2

/* user data struct used by ioctl interface to the transport driver*/
struct heci_message_data {
	u32 size;
	char *data;
} __attribute__((packed));


/* IOCTL commands */
/*
 * Intel MEI client information struct
 */
struct mei_client {
	u32 max_msg_length;
	u8 protocol_version;
	u8 reserved[3];
} __packed;

/*
 * IOCTL Connect Client Data structure
 */
struct mei_connect_client_data {
	union {
		struct guid in_client_uuid;
		struct mei_client out_client_properties;
	} d;
} __packed;

#define IOCTL_HECI_CONNECT_CLIENT \
		_IOWR('H' , 0x01, struct mei_connect_client_data)

#if 0
#define IOCTL_HECI_GET_VERSION \
    _IOWR('H' , 0x0, struct heci_message_data)
#define IOCTL_HECI_CONNECT_CLIENT \
    _IOWR('H' , 0x01, struct heci_message_data)
#endif

#define MAX_MSG_COUNT		127		//queue depth
typedef struct _TRANS_MSG_ENTRY
{
	//Raw message data returned from transport bus driver
	u8		Data[DCMI_MAX_MSG_LEN];
	u32		DataLen;

	// message chain - double linked list
	struct list_head listItem;
} TRANS_MSG_ENTRY;

#if 0
/**
 * Requests transport layer kernel module and locks its usage.
 */
int  dcmi_transport_load(void);
#endif

/**
 * Just locks transport layer kernel module and usage.
 */
int  dcmi_transport_lock(void);

/**
 * Unlocks transport layer kernel module.
 */
void  dcmi_transport_unlock(void);


/**
 * Initialize transport structure (allocate buffers)
 */
int dcmi_transport_init(DCMI_TRANSPORT *trans);

/**
 * Free transport structure (deallocate buffers)
 */
void dcmi_transport_free(DCMI_TRANSPORT *trans);

DCMI_STATUS dcmi_transport_connect_client(const struct guid clientGuid);
DCMI_STATUS dcmi_transport_open(void);
DCMI_STATUS dcmi_transport_read(u8 *buffer, u32 *len, u8 *seqNo);
DCMI_STATUS dcmi_transport_write(u8 *buffer, u32 len);
DCMI_STATUS dcmi_transport_close(void);

void trans_flush_queue(void);

#endif
