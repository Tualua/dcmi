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
#ifndef _IO_DCMI_H_
#define _IO_DCMI_H_

#define INVALID_INPUT_BUFFER             (0xE0070002L)
#define INVALID_OUTPUT_BUFFER            (0xE0070003L)
#define HARDWARE_FAILURE                 (0xE0040008L)
#define DRIVER_FAILURE                   (0xE0040009L)
#define INPUT_BUFFER_TOO_SMALL           (0xE0070011L)
#define INVALID_DRIVER_IOCTL             (0xE007000EL)
#define INVALID_DRIVER_REQUEST           (0xE007000FL)


#define IMB_SUCCESS						 0L

#define IMB_SEND_TIMEOUT                 0xE0070004L
#define IMB_RECEIVE_TIMEOUT              0xE0070005L
#define IMB_IF_SEND_TIMEOUT              0xE0070006L
#define IMB_IF_RECEIVE_TIMEOUT           0xE0040007L
#define IMB_INVALID_IF_RESPONSE          0xE004000AL
#define IMB_INVALID_PACKET               0xE004000BL
#define IMB_RESPONSE_DATA_OVERFLOW       0xE004000CL
#define IMB_INVALID_REQUEST              0xE007000DL
#define IMB_CANT_GET_SMS_BUFFER          0xE0070010L
#define IMB_MSG_NOT_AVAILABLE            0xE0070012L
#define IMB_SEND_ERROR                   0xE0070013L

#ifndef __KERNEL__
typedef signed char s8;
typedef unsigned char u8;
typedef signed short s16;
typedef unsigned short u16;
typedef signed int s32;
typedef unsigned int u32;
typedef signed long long s64;
typedef unsigned long long u64;
#endif

typedef u32    			DWORD;
typedef unsigned short  WORD;
typedef unsigned char   BYTE;

typedef struct {
	BYTE rsSa;          // IMB address of responder
	BYTE cmd;           // IMB command
	BYTE netFn;		   	// netFn for command
	BYTE rsLun;		   	// logical unit on responder
	BYTE dataLength;	// length of following data
	BYTE data[1];		// the first byte of requested data
} __attribute__((packed)) ImbRequest;


typedef struct {

	#define	NO_RESPONSE_EXPECTED	0x01	// don't wait around for an IMB response SEnd
	DWORD	flags; 			// request flags
	DWORD	timeOut;	 	// in uSec units
   ImbRequest	req;	// message buffer
} __attribute__((packed)) ImbRequestBuffer;

#define MIN_IMB_REQ_BUF_SIZE	13	// a buffer without any request data


typedef struct {
    BYTE       cCode;		// completion code
    BYTE       data[1];	// response data buffer
} __attribute__((packed)) ImbResponseBuffer;

#define far
typedef void  far *		LPVOID;
typedef DWORD far *		LPDWORD;

typedef struct _OVERLAPPED {
    DWORD   Internal;
    DWORD   InternalHigh;
    DWORD   Offset;
    DWORD   OffsetHigh;
/*    HANDLE  hEvent; */
} OVERLAPPED, *LPOVERLAPPED;

/*
 * DCMI data struct used by ioctl interface.
 * Pointer to struct smi is the only defined parameter of IOCTL_IMB_SEND_MESSAGE command.
 */
struct smi {
    DWORD smi_VersionNo;		/* not used by Linux DoH driver */
    DWORD smi_Reserved1;		/* not used by Linux DoH driver */
    DWORD smi_Reserved2;		/* not used by Linux DoH driver */
    LPVOID ntstatus;			/* address of NT status block, not used in Linux DoH driver */
    LPVOID  lpvInBuffer;        /* address of buffer for input data, see ImbRequestBuffer*/
    DWORD  cbInBuffer;  		/* size of input buffer*/
    LPVOID  lpvOutBuffer;       /* address of output buffer, see ImbResponseBuffer*/
    DWORD  cbOutBuffer; 		/* size of output buffer*/
    LPDWORD  lpcbBytesReturned; /* address of actual bytes of output*/
    LPOVERLAPPED  lpoOverlapped;/* address of overlapped structure, not used in Linux DoH driver */
};

/* IOCTL commands codes like in IPMI linux driver for compatibility */
#define FILE_DEVICE_IMB		0x00008010
#define IOCTL_IMB_BASE		0x00000880
#define CTL_CODE(DeviceType, Function, Method, Access) \
		_IO((DeviceType) & 0x00FF, (Function) & 0x00FF)

/*
 * IOCTL_IMB_SEND_MESSAGE
 * Usage:	ioctl(fd, IOCTL_IMB_SEND_MESSAGE, &smi)
 * where 	smi is struct smi *
 * Return: 	0 (IMB_SUCCESS) on success,
 * 			<0 on ioctl failure, see errno
 * 			>0 on recoverable error, possible values:
 *				IMB_SEND_ERROR
 *				IMB_INVALID_IF_RESPONSE
 *				IMB_MSG_NOT_AVAILABLE
 *				INVALID_INPUT_BUFFER,	when header or data too short
 *				INVALID_OUTPUT_BUFFER,	when received data lenght >= prepared buffer length
 */
#define IOCTL_IMB_SEND_MESSAGE \
	CTL_CODE(FILE_DEVICE_IMB, IOCTL_IMB_BASE + 2, METHOD_BUFFERED, FILE_ANY_ACCESS)


#ifdef __KERNEL__
/*
 *  IOCTLs functions prototypes
 */

int dcmi_ioctl_imb_send_message(struct dcmi_device *dev, int if_num,
			   struct smi *u_msg,
			   struct smi k_msg,
			   struct dcmi_file_private *file_ext);
#endif

#endif /* _IO_DCMI_H_ */
