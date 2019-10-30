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
 * DCMIProtocol is DCMI non-OS Specific Layer - core module
 */
#ifndef _DCMI_PROTOCOL_H
#define _DCMI_PROTOCOL_H

#include "dcmi_data_structures.h"
#include "dcmi.h"
#include "dcmi_transport.h"

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif


DCMI_STATUS dcmi_connect(void);
DCMI_STATUS dcmi_reset_connection(void);

DCMI_STATUS dcmi_send_message(
						IN	u8	netFn,
	                    IN	u8	LUN,
	                    IN	u8	cmd,
	                    IN	u8	*data,
	                    IN	u32	dataLen,
	                    OUT	u8	*sequence);

DCMI_STATUS dcmi_get_message(
						OUT u8    *rqSA,
                        OUT u8	*netFn,
	                    OUT	u8	*LUN,
	                    OUT	u8	*cmd,
	                    OUT u8	*cCode,
	                    OUT	u8	*data,
	                    OUT	u32	*dataLen,
	                    OUT	u8	*sequence);

DCMI_STATUS dcmi_disconnect(void);


#endif
