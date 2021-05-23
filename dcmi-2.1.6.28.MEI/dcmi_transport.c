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

#include <linux/fs.h>
#include <linux/delay.h>	//mdelay
#include <linux/kthread.h>
#include <linux/slab.h>
#include "kcompat.h"
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/sched/signal.h>
#endif
#include "dcmi_transport.h"

/* dcmi device structure defined in dcmi_main.c */
extern struct dcmi_device dcmi_device;

#define DEVICE_NAME "mei"

static char DCMI_TRANSPORT_DEVICE[] = "/dev/" DEVICE_NAME;

/**
 * Holds the pointer to transport layer module structure for the module usage management.
 */
static struct module *gDcmiTransportModuleStruct;

static int trans_read_thread(void *unused);

#if 0
/**
 * Check if the transport device exists by trying to open it.
 * @param	trans		transport device data
 * @return 0 success, -1 error opening the device read/write
 */
static int dcmi_transport_devicefile_exists(DCMI_TRANSPORT *trans)
{
	int ret = 0;
	mm_segment_t fs;

	if (trans == NULL) return 1;

	printk(KERN_INFO "dcmi: trying %s...\n", trans->device_file_name);
	fs = get_fs();
	set_fs(KERNEL_DS);
	trans->filp = filp_open(trans->device_file_name, O_RDWR , 0);
	if (trans->filp == NULL || IS_ERR(trans->filp))
	{
		if (IS_ERR(trans->filp))
		{
			printk(KERN_ERR "dcmi: %s is not available for dcmi driver!!!, error %ld\n",
					trans->device_file_name, (unsigned long)trans->filp);
		}
		else
		{
			printk(KERN_ERR "dcmi: %s is not available for dcmi driver!!!\n", trans->device_file_name);
		}
		ret = -1;
	}
	else
	{
		printk(KERN_INFO "dcmi: %s availability confirmed\n", trans->device_file_name);

		/* close the file */
		filp_close(trans->filp, NULL);
	}
	set_fs(fs);
	trans->filp = NULL;

	return ret;
}
#endif


#if 0
/**
 * Finds module struct pointer based on module name.
 * @param	modname name of the module to look for
 * @returns NULL if modname not found on modules list, otherwise the pointer to modname struct
 */
static struct module *find_module(char *modname)
{
	struct module *res = NULL;
	struct module *mod;

	list_for_each_entry(mod, &THIS_MODULE->list, list)
	{
		if (strcmp(mod->name, modname) == 0)
		{
			//printk(KERN_INFO "dcmi: heci module located\n");
			DBG("dcmi: heci module located\n");
			res = mod;
			break;
		}
	}

	return res;
}
#endif

#if 0
/**
 * Requests and bounds to the transport layer kernel module
 * @return	0 succes, -ENODEV if transport cannot be loaded
 */
int dcmi_transport_load(void)
{
	request_module(DEVICE_NAME); /* and try to load it */
	mdelay(1000);

	return dcmi_transport_lock();
}
#endif


/**
 * Just bounds to the transport layer kernel module
 * @return	0 succes, -ENODEV if transport cannot be loaded
 */
int dcmi_transport_lock(void)
{
	int ret = 0;

	gDcmiTransportModuleStruct = find_module(DEVICE_NAME);
	if (!try_module_get(gDcmiTransportModuleStruct))
	{
		ret = -ENODEV;
		//printk(KERN_ERR "dcmi: heci module not bound\n");
		DBG("dcmi: heci module not bound\n");
	}
	else
	{
		//printk(KERN_INFO "dcmi: heci module locked as beeing used\n");
		DBG("dcmi: heci module locked as beeing used\n");
	}

	return ret;
}

/**
 * Releases and unbounds from the transport layer kernel module
 */
void dcmi_transport_unlock(void)
{
	if (gDcmiTransportModuleStruct != NULL) {
		module_put(gDcmiTransportModuleStruct);
		//printk(KERN_INFO "dcmi: heci module unlocked\n");
		DBG("dcmi: heci module unlocked\n");
	}
}


/**
 * Layer initialization and allocation
 * @param	trans 	structure used by transport layer
 * @return	0 - OK, <>0 errno failure, where <0 is errno
 * @note	dcmi_transport_free() for transport layer module deallocation
 */
int dcmi_transport_init(DCMI_TRANSPORT *trans)
{
	int ret = 0;

	if (trans == NULL) return 1;

	atomic_set(&trans->state, DCMI_NO_TRANSPORT);
	trans->buffer = kmalloc(DCMI_MAX_MSG_LEN, GFP_KERNEL /* | __GFP_REPEAT */);
	if (NULL == trans->buffer)
	{
		ret = -ENOMEM;
		goto end;
	}
	else
		atomic_set(&trans->state, DCMI_TRANS_ALLOCATED);

	trans->transportBufferLength = 0;
	atomic_set(&trans->resetRequested, 0);
	trans->device_file_name = DCMI_TRANSPORT_DEVICE;
	trans->filp = NULL;

#if 0
	if (dcmi_transport_devicefile_exists(trans) != 0)
	{
		ret = -ENODEV;
		atomic_set(&trans->state, DCMI_NO_TRANSPORT);
	}
#endif

end:
	return ret;

}

/**
 * Layer deinitialization and deallocation
 * @param	trans 	structure used by transport layer
 * @return	0 - OK, <>0 errno failure, where <0 is errno
 * @note	dcmi_transport_init() for transport layer module initialization
 */
void dcmi_transport_free(DCMI_TRANSPORT *trans)
{
	if (trans != NULL)
	{
		atomic_set(&trans->state, DCMI_NO_TRANSPORT);
		kfree(trans->buffer); /* buffer can be null for kfree */
		trans->buffer = NULL;
		trans->device_file_name = "";
		trans->transportBufferLength = 0;
	}
}

/**
 * Low level reading from transport.
 * assuming a non blocking read operation
 * @param  trans structure holding transport device context
 * @return -1 error, >=0 number of bytes read
 */
static int read_from_heci_file(DCMI_TRANSPORT *trans)
{
	int 			bytesRead = 0;
	mm_segment_t 	old_fs;
	int 			startPos = 0;
//	int 			retries = 0;

	//struct dcmi_device *dev = &dcmi_device;

	if (trans == NULL) return -1;

	if (atomic_read(&trans->resetRequested) == 1)
		return 0;

	if (atomic_read(&trans->state) !=  DCMI_TRANS_CONNECTED)
	{
		//printk(KERN_DEBUG "dcmi: read_from_heci_file() %s not CONNECTED. TransState: %X\n", trans->device_file_name, atomic_read(&trans->state));
		return -1;
	}
	else
	{
		if ( (trans->filp) && (trans->filp->f_op) && (trans->filp->f_op->read == NULL) )
		{
			printk(KERN_ERR "dcmi: no reads possible from %s\n", trans->device_file_name);
			atomic_set(&trans->state, DCMI_TRANS_OPENED_NO_READ); /* no reads allowed */
			return -1;
		}

		/* read file from startPos */
		trans->filp->f_pos = startPos;

		//atomic_set(&trans->state, DCMI_TRANS_ACTIVE);

		old_fs = get_fs();
		set_fs(KERNEL_DS);
//read_again:
		bytesRead = trans->filp->f_op->read(
					trans->filp,
					trans->buffer,
					DCMI_MAX_MSG_LEN,
					&trans->filp->f_pos);

		//DBG("Heci read returned: %d\n", bytesRead);
		if ( bytesRead < 0 && bytesRead != -EAGAIN )
		{
			DBG(KERN_ERR "dcmi: heci read status:%d, errno=%d\n", bytesRead, -bytesRead);
			if (bytesRead == -EINTR) // interrupted by signal before any data was read
			{
				//schedule();
				DBG(KERN_ERR "dcmi thread: read interrupted!\n");
//				if (retries++ < 3)
//					goto read_again;
//				else
					bytesRead = -1;
			}
		}
		else
		{
			if (bytesRead == -EAGAIN)	//no data immediatelly available for reading
			{
				//DBG("dcmi: EAGAIN no transport data immediatelly available\n");
				bytesRead = 0;
			}
		}

		trans->transportBufferLength = (bytesRead < 0 ? 0 :
											(bytesRead<DCMI_MAX_MSG_LEN ?
													bytesRead : DCMI_MAX_MSG_LEN));
		//atomic_set(&trans->state, DCMI_TRANS_CONNECTED);
		set_fs(old_fs);
	}
	return bytesRead;
}

/* Low level writing to transport.
*  assuming a non blocking write operation
*  @param  trans structure holding transport device context
*  @return -1 error, -EAGAIN for repeat, >=0 number of bytes written
*/
static int write_to_heci_file(DCMI_TRANSPORT *trans)
{
	int 			bytesWritten = 0;
	mm_segment_t 	old_fs;
	int 			startPos = 0;
	int 			bytesToWrite = 0;
//	int 			intrRetries = 0;
//	int 			againRetries = 0;

	//struct dcmi_device *dev = &dcmi_device;

	if (trans == NULL) return -1;

	if (atomic_read(&trans->state) !=  DCMI_TRANS_CONNECTED)
	{
		printk(KERN_ERR "dcmi: write_to_heci_file() %s not CONNECTED. TransState: %X\n", trans->device_file_name, atomic_read(&trans->state));
		return -1;
	}
	else {
		if ( (trans->filp) && (trans->filp->f_op) && (trans->filp->f_op->write == NULL) )
		{
			printk(KERN_ERR "dcmi: no writes possible to %s\n", trans->device_file_name);
			atomic_set(&trans->state, DCMI_TRANS_OPENED_NO_WRITE); /* no writes allowed */
			return -1;
		}

		/* write file from startPos */
		trans->filp->f_pos = startPos;


		bytesToWrite = (trans->transportBufferLength <= DCMI_MAX_MSG_LEN
						? ( trans->transportBufferLength < 0 ? 0 : trans->transportBufferLength )
						: DCMI_MAX_MSG_LEN);
		if (bytesToWrite > 0)
		{
			//atomic_set(&trans->state, DCMI_TRANS_ACTIVE);
			old_fs = get_fs();
			set_fs(KERNEL_DS);
//write_again:
			bytesWritten = trans->filp->f_op->write(
								trans->filp,
								trans->buffer,
								bytesToWrite,
								&trans->filp->f_pos);

			if ( bytesWritten < 0 && bytesWritten != -EAGAIN )
			{
				printk(KERN_ERR "dcmi: heci write errno=%d\n", -bytesWritten);
				if (bytesWritten == -EINTR) // interrupted by signal before any data was written
				{
					//schedule();

//					if (intrRetries++ < 3) {
//						goto write_again;
//					} else {
						bytesWritten =  -1;
//					}
				}
			}
			else
			{
				if (bytesWritten == -EAGAIN)
				{
					DBG("dcmi: write_to_heci_file would block, will be repeated\n");
					//schedule();

//					if (againRetries++ < 3) {
//						goto write_again;
//					} else {
						bytesWritten = -1;
//					}
				}
			}
			set_fs(old_fs);
			//atomic_set(&trans->state, DCMI_TRANS_CONNECTED);
		}

		if (bytesWritten > -1 && bytesWritten != bytesToWrite)
		{
			printk(KERN_ERR "dcmi: %d bytes out of %d bytes written!\n", bytesWritten, bytesToWrite);
			return -1;
		}
	}
	return bytesWritten;
}

/**
 * Connecting to the HECI transport with the client GUID
 * @param guid client identifier to connect
 * @return	DCMI_STATUS enum, see dcmi.h
 */
DCMI_STATUS dcmi_transport_connect_client(const struct guid clientGuid)
{
	int error = -ENOTTY;
	mm_segment_t 	old_fs;
	struct dcmi_device *dev = &dcmi_device;

	DBG("dcmi: transport connecting client %s...\n", dcmi_device.trans.device_file_name);
	//printk(KERN_ERR "dcmi: transport connecting client %s...\n", dcmi_device.trans.device_file_name);

	spin_lock(&dev->trans_lock);

	// Make sure the transport layer device is open
	if (atomic_read(&dev->isTransOpen) == FALSE)
	{
		spin_unlock(&dev->trans_lock);
		return TRANS_NOT_OPEN;
	}

	//only one transReadThread should be started
	if (atomic_read(&dev->isTransConnected) == TRUE)	//if already started
		goto end_ok;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	{
		if (dev->trans.filp->f_op)
		{
			DBG("dcmi: connect client cmd: %lu to transport: %s\n",IOCTL_HECI_CONNECT_CLIENT, dcmi_device.trans.device_file_name);

#if HAVE_UNLOCKED_IOCTL
			{
			struct mei_connect_client_data connect_client_data;
			connect_client_data.d.in_client_uuid = clientGuid;

			error = dev->trans.filp->f_op->unlocked_ioctl(
							dev->trans.filp,
							IOCTL_HECI_CONNECT_CLIENT,
							(unsigned long) &connect_client_data
							);
			}
#else
			{
			struct heci_message_data heci_message_data;
			heci_message_data.size = sizeof(struct guid);
			heci_message_data.data = (char *) &clientGuid;

			error = dev->trans.filp->f_op->ioctl(
							dev->trans.filp->f_dentry->d_inode,
							dev->trans.filp,
							IOCTL_HECI_CONNECT_CLIENT,
							(unsigned long) &heci_message_data);
			}
#endif
			if (error) {
				set_fs(old_fs);
				goto not_connected;
			}
			else
			{
				atomic_set(&dev->trans.state, DCMI_TRANS_CONNECTED);
				printk(KERN_INFO "dcmi: transport %s connected successfully.\n",dev->trans.device_file_name);
			}
		}
		else
		{
			set_fs(old_fs);
			goto not_connected;
		}
	}
	set_fs(old_fs);

	//
	// Let's install a message handler thread
	//
	spin_lock_init(&dev->transMsgListLock);
	INIT_LIST_HEAD(&dev->transMsgList);

	//wait queue initialization for signalling of message availability
	init_waitqueue_head(&dev->trans_msg_wait);

	//
	// Start the transport reading thread
	//
	dev->transReadThreadTask = kthread_create(trans_read_thread, NULL, "%s", "dcmi");
	if (!IS_ERR(dev->transReadThreadTask))
	{
		wake_up_process(dev->transReadThreadTask);
		atomic_set(&dev->isTransConnected, TRUE);
	}
	else
	{
		printk(KERN_ERR "dcmi: Failed to create transport read thread. error=%ld", PTR_ERR(dev->transReadThreadTask));
		spin_unlock(&dev->trans_lock);
		return TRANS_INSUFFICIENT_RESOURCE;
	}
end_ok:
	spin_unlock(&dev->trans_lock);
	return TRANS_OK;

not_connected:
	atomic_set(&dev->trans.state, DCMI_TRANS_NOT_CONNECTED);
	spin_unlock(&dev->trans_lock);
	printk(KERN_ERR "dcmi: connecting %s failed. Error: %d (0x%X)\n", dev->trans.device_file_name, error, error);
	return DCMI_CONNECT_FAIL;
}


DCMI_STATUS dcmi_transport_open()
{
	mm_segment_t fs;
	DCMI_STATUS status = TRANS_OK;
	struct dcmi_device *dev = &dcmi_device;

	DBG("dcmi: transport opening %s...\n", dcmi_device.trans.device_file_name);

	spin_lock(&dev->trans_lock);
	if (atomic_read(&dev->isTransOpen) != TRUE)
	{
		fs = get_fs();
		set_fs(KERNEL_DS);
		dev->trans.filp = filp_open(dev->trans.device_file_name, O_RDWR|O_NONBLOCK , 0);
		set_fs(fs);
		if (/*dev->trans.filp == NULL ||*/ IS_ERR_OR_NULL(dev->trans.filp)) //not possible to have null here but..
		{
			DBG("dcmi: %s transport opening error: %ld!\n", dev->trans.device_file_name, (long) dev->trans.filp);
			atomic_set(&dev->trans.state, DCMI_TRANS_NOT_OPENED);
			status = TRANS_NOT_OPEN;
		}
		else
		{
			atomic_set(&dev->isTransOpen, TRUE);
			atomic_set(&dev->trans.state, DCMI_TRANS_OPENED);
			DBG("dcmi: %s transport opening OK.\n", dev->trans.device_file_name);
		}
	}
	spin_unlock(&dev->trans_lock);

	return status;
}

DCMI_STATUS dcmi_transport_close()
{
	mm_segment_t fs;
	struct dcmi_device *dev = &dcmi_device;

	DBG("dcmi: transport closing %s...\n", dcmi_device.trans.device_file_name);

	spin_lock(&dev->trans_lock);
	if (atomic_read(&dev->isTransOpen) == TRUE)
	{
		/* wait for the read thread to cleanup - sleep on readThreadExitEvent */
		if (!IS_ERR(dev->transReadThreadTask)) {
			kthread_stop(dev->transReadThreadTask);
			dev->transReadThreadTask  = (void *) -1; //make sure assert IS_ERR(-1) is TRUE
		}

		/* close the file */
		fs = get_fs();
		set_fs(KERNEL_DS);
		filp_close(dev->trans.filp, NULL);
		set_fs(fs);

		dev->trans.filp = NULL;
		atomic_set(&dev->trans.state, DCMI_TRANS_ALLOCATED);
		atomic_set(&dev->isTransOpen, FALSE);
		atomic_set(&dev->isTransConnected, FALSE);
	}
	spin_unlock(&dev->trans_lock);

	DBG("dcmi: transport closing OK\n");
	return TRANS_OK;
}

DCMI_STATUS dcmi_transport_write(u8 *buffer, u32 len)
{
	int i, bytesWritten;
	struct dcmi_device *dev = &dcmi_device;
	DCMI_TRANSPORT *trans = &dev->trans;

	DBG("dcmi_transport_write starting\n");
	/*
	 * Make sure we are connected
	 */
	if (atomic_read(&dev->isTransConnected) != TRUE)
	{
		DBG("DCMI transport device not ready.\n");
		return DCMI_NOT_CONNECTED;
	}

	//debug showing what we are about send
	if (dcmi_debug)
	{
		printk("dcmi_transport_write called for  %d bytes:\n", len);
		for (i=0; i<len; i++)
			printk(" 0x%X", buffer[i]);
		printk("\n");
	}

	trans->transportBufferLength = len;
	memcpy( trans->buffer, buffer, len);

	bytesWritten = write_to_heci_file(trans);
	if ( bytesWritten <= 0 )
	{
		int status = atomic_read(&trans->state);
		//printk(KERN_ERR "dcmi_transport_write() error. Transport in state=%d\n", status);
		DBG("dcmi_transport_write() error. Transport in state=%d\n", status);
		return DCMI_WRITE_FAIL;
	}

	DBG("dcmi_transport_write leaving\n");
	return DCMI_OK;
}

DCMI_STATUS dcmi_transport_read(u8 *buffer, u32 *len, u8 *seqNo)
{
	struct dcmi_device *dev = &dcmi_device;
	TRANS_MSG_ENTRY *pMsgEntry, *pNextEntry;
	int res = 0;
	int i = 0;
	int intrRetries = 0;

	DBG("dcmi_transport_read Starting\n");

	/*
	 * Make sure we are connected
	 */
	if (atomic_read(&dev->isTransConnected) != TRUE)
	{
		DBG("DCMI transport device not ready\n");
		return TRANS_NOT_FOUND;
	}

	/*
	 * Try to read the message from queue waiting for a timeout if not available
	 */
sleep_again:
	res = wait_event_interruptible_timeout(dev->trans_msg_wait,
											!list_empty(&dev->transMsgList),
											TIMEOUT * HZ/1000);

	//printk(KERN_ERR "reader:waken up\n");
	if (unlikely(res<0))  //-ERESTARTSYS
	{
		//spin_unlock(&dev->transMsgListLock);
		DBG("reader: dcmi_transport read wait interrupted\n");

		if (intrRetries++ < 3) {
			goto sleep_again;
		}
		return DCMI_INTERNAL_ERROR;
	}
	else if (res == 0)
	{
		//spin_unlock(&dev->transMsgListLock);
		DBG("reader: Timeout while waiting for message\n");
		return TRANS_TIMEOUT;
	}

	/*
	 * Try to find the message with the same seqNo
	 */
	spin_lock(&dev->transMsgListLock);

	if ( list_empty(&dev->transMsgList) )
	{
		spin_unlock(&dev->transMsgListLock);
		DBG("dcmi_transport_read() waken up or interrupted before timeout or there is no messages\n");
		return DCMI_MSG_NOT_AVAILABLE;
	}

	//if requesting the first msg
	if (TRANS_SEQUENCE_START == *seqNo)
	{
		pMsgEntry = list_first_entry(&dev->transMsgList, TRANS_MSG_ENTRY, listItem);
	}
	else
	{
		int found = 0;
		list_for_each_entry(pMsgEntry, &dev->transMsgList, listItem)
		{
			// break if a matched sequence number was found
			if(pMsgEntry->Data[TRANS_SEQUENCE_NO_OFFSET] == *seqNo)
			{
				found = 1;
				break;
			}
		}

		/*
		 * Check if was found
		 */
		if (!found)
		{
			spin_unlock(&dev->transMsgListLock);
			DBG("No message matched\n");
			return DCMI_MSG_NOT_AVAILABLE;
		}
	}

	/*
	 * Put received data out
	 */
	*len = pMsgEntry->DataLen;
	memcpy(buffer, pMsgEntry->Data, pMsgEntry->DataLen);

	//debug showing what went out
	if ( dcmi_debug )
	{
		printk("dcmi_transport_read returned %d bytes:\n", *len);
		for (i=0; i<*len; i++)
			printk(" 0x%X", buffer[i]);
		printk("\n");
	}

	//if next list address is the address of the head of the list
	if(pMsgEntry->listItem.next == &dev->transMsgList)
	{
		*seqNo = TRANS_SEQUENCE_START;
	}
	else
	{
		//return next buffered frame sequence number
		pNextEntry = list_first_entry(&pMsgEntry->listItem, TRANS_MSG_ENTRY, listItem);
		*seqNo = pNextEntry->Data[TRANS_SEQUENCE_NO_OFFSET]; // next sequence number waiting in queue
	}

  	/*
	 *  Now remove the entry from the list
	 */
	list_del(&pMsgEntry->listItem);

	atomic_dec(&dev->MsgCount);

	kfree(pMsgEntry);

	spin_unlock(&dev->transMsgListLock);

	return TRANS_OK;
}

void trans_flush_queue(void)
{
	TRANS_MSG_ENTRY *pListEntry;
	struct list_head *pos, *n;

	struct dcmi_device *dev = &dcmi_device;

	spin_lock(&dev->transMsgListLock);
	list_for_each_safe(pos, n, &dev->transMsgList)
	{
		pListEntry = list_entry(pos, TRANS_MSG_ENTRY, listItem);
		list_del(pos);
		kfree(pListEntry);
	}
	INIT_LIST_HEAD(&dev->transMsgList);
	atomic_set(&dev->MsgCount, 0);
	spin_unlock(&dev->transMsgListLock);
	return;
}

static int trans_read_thread(void *unused)
{
	TRANS_MSG_ENTRY *pNewMsgEntry;
	int bytesRead;

	struct dcmi_device *dev = &dcmi_device;

	DBG("dcmi: transport read thread started\n");

	allow_signal(SIGKILL);

	while( !kthread_should_stop() )
	{
		if (signal_pending(current)) {
			/* after recover from hibernation file close() should not stop nonexisting task*/
			dev->transReadThreadTask  = (void *) -1; //make sure assert IS_ERR(-1) is TRUE, see dcmi_transport_close
			break;	//die on SIGKILL
		}

		//printk(KERN_ERR "dcmi: thread cycle started\n");

		/* Do the real work */
		bytesRead = read_from_heci_file(&dev->trans);
		if ( bytesRead > 0 )
		{
			spin_lock(&dev->transMsgListLock);

			//printk(KERN_ERR "dcmi: transport received new heci message still connected\n");

			if (atomic_read(&dev->MsgCount) >= MAX_MSG_COUNT)	//> just for safety
			{
				//remove head from the list taking its entry for assignment
				pNewMsgEntry = list_first_entry(&dev->transMsgList, TRANS_MSG_ENTRY, listItem);
				list_del(&pNewMsgEntry->listItem);
				atomic_dec(&dev->MsgCount);
			}
			else
			{
				pNewMsgEntry = kmalloc(sizeof(TRANS_MSG_ENTRY), GFP_KERNEL);
			}

			/* Update data */
			pNewMsgEntry->DataLen = (dev->trans.transportBufferLength < DCMI_MAX_MSG_LEN
					? dev->trans.transportBufferLength : DCMI_MAX_MSG_LEN);
			memcpy( pNewMsgEntry->Data,
					dev->trans.buffer,
					pNewMsgEntry->DataLen);

			if (dcmi_debug)
			{
				int i;
				//debug showing what we are about send
				printk("dcmi: transport thread received %d bytes:\n", pNewMsgEntry->DataLen);
				for (i=0; i < pNewMsgEntry->DataLen; i++)
					printk(" 0x%X", pNewMsgEntry->Data[i]);
				printk("\n");
			}

			/* Put it in the end of the list */
			list_add_tail(&pNewMsgEntry->listItem, &dev->transMsgList);
			atomic_inc(&dev->MsgCount);
			spin_unlock(&dev->transMsgListLock);

			DBG(KERN_INFO "dcmi: thread received new heci message no. %d\n", atomic_read(&dev->MsgCount));

			/*
			 * Signal the message is available
			 */
			wake_up_interruptible(&dev->trans_msg_wait);
		}
		else
		{
			if (bytesRead < 0) {
				DBG(KERN_ERR "read_from_heci_file error reported\n");
				atomic_set(&dev->trans.resetRequested, 1);
			}
		}
		/*
		 * Task cycle delay
		 */
		msleep_interruptible(1);	//1ms
	}

	DBG("dcmi: transport read thread exiting\n");

	/* clean and deallocate queued messages */
	trans_flush_queue();
	DBG("dcmi: transport queue freed\n");

	return 0;
}

