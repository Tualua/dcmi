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
#include "dcmi_protocol.h"

// IDC-HI 7519b383-48fc-43e5-a5eb-5959cb581000
const struct guid IDC_HI_GUID = {
	0x7519b383, 0x48fc, 0x43e5,
	{0xa5, 0xeb, 0x59, 0x59, 0xcb, 0x58, 0x10, 0x00}
};

u8		g_Sequence  = 0;
BOOLEAN	g_Connected = FALSE;

/**
 * Connect to the IDC-HI F/W Client.
 * @return DCMI_OK	Successfully connect to the IDC-HI F/W client
 * @return TRANS_DRIVER_FAIL Unable to locate HECI bus driver
 * @return DCMI_CONNECT_FAIL Fail to connect to the IDC-HI F/W client.
 */
DCMI_STATUS dcmi_connect()
{
	// Already connected
	if (g_Connected)
		goto connected;

	// First needs to open the HECI Bus Layer
	if (dcmi_transport_open()!=TRANS_OK)
		return TRANS_DRIVER_FAIL;

	// Now try to connect to the IDC-HI F/W Client
	if (dcmi_transport_connect_client(IDC_HI_GUID) != TRANS_OK)
	{
		return DCMI_CONNECT_FAIL;
	}

	g_Connected = TRUE;

connected:
	g_Sequence = 0;

	DBG("dcmi: dcmi connected.\n");
	return DCMI_OK;
}

/**
 * Disconnect to the IDC-HI F/W Client.
 * @return DCMI_OK	Disconnect successfully
 */
DCMI_STATUS dcmi_disconnect()
{
	if (g_Connected)
		dcmi_transport_close();

	g_Connected = FALSE;
	return DCMI_OK;
}

/**
 * Reset IDC-HI connection.
 */
DCMI_STATUS dcmi_reset_connection()
{
	// Close the current connection to flush all queue
	// and reconnect.
	dcmi_disconnect();
	return dcmi_connect();
}

/**
 * Send IDC-HI message.
 * @param	netFn	  net function
 * @param	LUN		  Logical Unit Number
 * @param	cmd		  Command
 * @param	*data	  pointer to request data buffer
 * @param	dataLen	  length of the request data
 * @param	*sequence pointer to this message sequence number
 * @return	DCMI_OK   if send successfuly
 * @return  DCMI_NOT_CONNECTED if no IDC-HI connection
 * @return  DCMI_WRITE_FAIL if error
 */
DCMI_STATUS dcmi_send_message(
				IN	u8	netFn,
				IN	u8	LUN,
				IN	u8	cmd,
				IN	u8	*data,
				IN	u32	dataLen,
				OUT	u8	*sequence)
{
	u8 DCMIMsg[DCMI_MAX_MSG_LEN];
	u32 i;
	DCMI_STATUS status;

	/*
	 * Make sure we're connected.
	 */
	if (!g_Connected)
		return DCMI_NOT_CONNECTED;

	/*
	 * Build the DCMI Message
	 */
	DCMIMsg[0] = 0x00; 				//rsSA can be anything
	DCMIMsg[1] = (netFn<<2) | LUN;
	DCMIMsg[2] = g_Sequence;
	DCMIMsg[3] = cmd;
	for (i=0; i<dataLen; i++)
		DCMIMsg[i+4] = data[i];

	/*
	 * Set the Commit byte to accept
	 */
	DCMIMsg[dataLen+4] = 0x1;

	/*
	 * Write it
	 */
	status = dcmi_transport_write(DCMIMsg, dataLen+5);
	if (status != DCMI_OK)
		return status;

	*sequence = g_Sequence++;
	return DCMI_OK;
}

/**
 * Get IDC-HI message.
 * @param	netFn	  net function
 * @param	LUN		  Logical Unit Number
 * @param	cmd		  Command
 * @param 	cCode	  completion code
 * @param	*data	  pointer to request data buffer
 * @param	dataLen	  length of the request data
 * @param	*sequence sequence number: IN of the msg to get, OUT of the next message in queue
 * @return	DCMI_OK   if received successfuly
 * @return  DCMI_NOT_CONNECTED 		if no IDC-HI connection
 * @return  TRANS_TIMEOUT timeout 	waiting HECI bus driver to read data
 * @return  TRANS_READ_ERROR HECI 	bus driver fails to read data
 * @return  DCMI_MSG_NOT_AVAILABLE 	no message available
 * @return  DCMI_INVALID_MSG_LEN	invalid message was returned
 * @return 	unlikely(DCMI_INTERNAL_ERROR) if system mechanism of waiting for message do not work
 */
DCMI_STATUS dcmi_get_message(
				OUT	u8	*rqSA,
				OUT	u8	*netFn,
				OUT	u8	*LUN,
				OUT	u8	*cmd,
				OUT u8	*cCode,
				OUT	u8	*data,
				OUT	u32	*dataLen,
				IN OUT u8	*sequence)
{
	u8  DCMIMsg[DCMI_MAX_MSG_LEN];
	u32 msgLen = DCMI_MAX_MSG_LEN;
	u32 i;
	DCMI_STATUS status;


	/*
	 * Make sure we're connected.
	 */
	if (!g_Connected)
		return DCMI_NOT_CONNECTED;
	/*
	 * Read data from HECI
	 */
	status = dcmi_transport_read(DCMIMsg, &msgLen, sequence);
	if (status == TRANS_TIMEOUT)
	{
		//DCMIResetConnection(); //The HECI Bus driver closes the connection when canceling the IRP
		return DCMI_MSG_NOT_AVAILABLE;
	}

	/*
	 * Other Status
	 */
	if (status != TRANS_OK)
		return status;

	/*
	 * Make sure the MsgLen is valid
	 */
	if ( (msgLen<DCMI_MIN_MSG_LEN) || (msgLen>DCMI_MAX_MSG_LEN) )
		return DCMI_INVALID_MSG_LEN;

	/*
	 * Decode returned data
	 */
	*rqSA	= DCMIMsg[0];
	*netFn	= ((DCMIMsg[1])>>2);
	*LUN	= (DCMIMsg[1]&0x03);
	*sequence = DCMIMsg[2];
	*cmd	= DCMIMsg[3];
	*cCode	= DCMIMsg[4];

	/*
	 * Copy data - drop the commit byte
	 */
	*dataLen = msgLen - 6;
	for (i=0; i<*dataLen; i++)
		data[i]	= DCMIMsg[i+5];

	return DCMI_OK;
}
