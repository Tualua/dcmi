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

#include "kcompat.h"

#include "dcmi_data_structures.h"
#include "io_dcmi.h"
#include "dcmi_protocol.h"

/**
 * dcmi_ioctl_send_get_message - send message and get response
 *
 * @dev: Device object for our driver
 * @if_num:  minor number
 * @*u_msg: pointer to user data struct in user space
 * @k_msg: data in kernel on the stack
 * @file_ext: private data of the file object
 *
 * @return 0 (IMB_SUCCESS) on success, <0 on failure, >0 on recoverable error
 */
int dcmi_ioctl_imb_send_message(struct dcmi_device *dev, int if_num,
			   struct smi *u_msg,
			   struct smi k_msg,
			   struct dcmi_file_private *file_ext)
{

	int rets = 0;

	u8 	rqSA, netFn, LUN, cmd, cCode, readSeq, writeSeq;
	u8 	k_buffer[DCMI_MAX_MSG_LEN];		/* allocated on stack kernel buffer for request */
	u8 	rsData[DCMI_MAX_MSG_LEN];		/* and for the response */

	u32	inputBufferLength, outputBufferLength, length;

	ImbRequestBuffer  *u_req, *req;

	ImbResponseBuffer *res;
	u32	dataLen;						/* received response data length */

	int status;
	u8	retry = 0;
	u8	isRightMsg = 0;
	u8 	i;

	DBG("requested dcmi_ioctl_send_get_message\n");

	if ((if_num != DCMI_MINOR_NUMBER) || (!dev) || (!file_ext))
		return -ENODEV;

    inputBufferLength  = k_msg.cbInBuffer;
    outputBufferLength = k_msg.cbOutBuffer;

    DBG("imb_ioctl: inputBufferLength = %d outputBufferLength = %d\n",
			inputBufferLength, outputBufferLength);

    /*
	 * Check buffers minima
	 */
	if ( inputBufferLength < sizeof(ImbRequestBuffer) )
	{
		DBG("dcmi_ioctl: IOCTL_IMB_SEND_MESSAGE - request header too short\n");
		return INVALID_INPUT_BUFFER;	//header too short
	}

	/*
	 * Copy whole IMB request to kernel space
	 */
	u_req = (ImbRequestBuffer __user *)  k_msg.lpvInBuffer;		//user space data pointer
	if (copy_from_user(k_buffer,
					u_req,
			(inputBufferLength<DCMI_MAX_MSG_LEN ? inputBufferLength : DCMI_MAX_MSG_LEN)))
	{
		printk(KERN_ERR "dcmi_ioctl: IOCTL_IMB_SEND_MESSAGE - copy_from_user ImbRequestBuffer failed\n");
		return -EFAULT;
	}
	req = (ImbRequestBuffer *) k_buffer;	//our request located on stack

	// and check for subsequent data length
	if ( inputBufferLength < sizeof(ImbRequestBuffer) - sizeof(req->req.data[0] + req->req.dataLength))
	{
		DBG("dcmi_ioctl: IOCTL_IMB_SEND_MESSAGE - request data too short\n");
		return INVALID_INPUT_BUFFER;	//data too short
	}
	DBG("dcmi_ioctl: IOCTL_IMB_SEND_MESSAGE - request validated\n");

	/*
	 * Send the request down to the DCMI layer
  	 */
	retry = 0;
	do
	{
		if (dcmi_debug) {
			DBG("\nSENDING: netFn: 0x%X LUN: 0x%X Cmd: 0x%X\n", req->req.netFn, req->req.rsLun, req->req.cmd);
			printk("Data:");
			for(i=0; i < req->req.dataLength; i++)
			{
				printk(" 0x%02X",req->req.data[i]);
			}
			printk("\n");
		}
		status = dcmi_send_message( req->req.netFn,
							  req->req.rsLun,
							  req->req.cmd,
							  req->req.data,
							  (u32)req->req.dataLength,
							  &writeSeq);
		DBG("\nStatus = %d\n", status);
	} while ( (status!=DCMI_OK) && (++retry<3) );

	if (status!=DCMI_OK)
	{
		DBG("dcmi_ioctl: IOCTL_IMB_SEND_MESSAGE - DCMISendMsg failed with status = %d\n", status);
		return IMB_SEND_ERROR;
	}

	if (req->flags == NO_RESPONSE_EXPECTED)
	{
		return IMB_SUCCESS;
	}

	/*
	 * Now get the response.
	 * Find the message with matched sequence number.
	 */
	isRightMsg = 0;
	do
	{
				//DBG("Reading Msg.....\n");
		readSeq = writeSeq;
		status = dcmi_get_message(&rqSA,&netFn,&LUN,&cmd,&cCode,rsData,&dataLen,&readSeq);
				//DBG("dcmi_get_message status = 0x%x\n",status);

		//Check to make sure it's the right response
		isRightMsg = ((netFn&0xFE)==(req->req.netFn&0xFE)) && (cmd==req->req.cmd);

		if((status != DCMI_OK) && (status != DCMI_MSG_NOT_AVAILABLE))
		{
			DBG("dcmi_ioctl: IOCTL_IMB_SEND_MESSAGE - Bad response getting DCMI message. Status=%d\n", status);
			return IMB_INVALID_IF_RESPONSE;
		}

		//isRightMsg = 1; //Just for testing
	} while ( (status != DCMI_MSG_NOT_AVAILABLE) && (!isRightMsg) );

	DBG("dcmi_ioctl: IOCTL_IMB_SEND_MESSAGE - Received right msg with GetMsg Status = %d\n", status);

	if(status==DCMI_MSG_NOT_AVAILABLE)
	{
		DBG("dcmi_ioctl: IOCTL_IMB_SEND_MESSAGE - No Msg available. Status = %d\n", status);
		return IMB_MSG_NOT_AVAILABLE;
	}

	/*
	 * Sets the results
	 */
	res = (ImbResponseBuffer *) k_buffer;
	res->cCode = cCode;
	memcpy(res->data, rsData, dataLen);
	length = dataLen + sizeof(res->cCode);

	rets = IMB_SUCCESS;
	//XXX uncomment for debug
	//rets = (status == DCMI_OK ? IMB_SUCCESS : status);

	/*
	 * Copy the results to user space
	 */
	if (length > outputBufferLength)
	{
		return INVALID_OUTPUT_BUFFER;
	}

	if (copy_to_user( 	(void *)k_msg.lpvOutBuffer,
						(void *)res,
						length ))
	{
		printk(KERN_ERR
			"dcmi_ioctl: IOCTL_IMB_SEND_MESSAGE - copy_to_user lpvOutBuffer failed\n");
		rets = -EFAULT;
		goto end;
	}

	if ( copy_to_user(	(void __user *)k_msg.lpcbBytesReturned,
						(void *)&length,
						sizeof(*k_msg.lpcbBytesReturned)
					  ) == -1 )
	{
		printk(KERN_ERR
			"dcmi_ioctl: IOCTL_IMB_SEND_MESSAGE - copy_to_user lpcbBytesReturned failed\n");
		rets = -EFAULT;
		goto end;
	}

	DBG("dcmi_ioctl: IOCTL_IMB_SEND_MESSAGE - success\n");
end:
	return rets;
}
