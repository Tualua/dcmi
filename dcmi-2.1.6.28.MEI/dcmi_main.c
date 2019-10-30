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
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/aio.h>
#include <linux/pci.h>
#include <linux/reboot.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/unistd.h>
#include <linux/kthread.h>
//#include <linux/reboot.h>
#include "kcompat.h"

#include "dcmi.h"
#include "dcmi_protocol.h"
#include "ver.h"
#include "io_dcmi.h"

#define DCMI_DRIVER_NAME   "dcmi"

/**
 *  dcmi driver strings
 */
char dcmi_driver_name[] = DCMI_DRIVER_NAME;
char dcmi_driver_string[] = "Intel(R) Data Center Host Interface (DCMI-HI)";
char dcmi_driver_version[] = DCMI_DRIVER_VERSION;
char dcmi_copyright[] = "Copyright (c) 2003 - 2011 Intel Corporation.";


#ifdef DCMI_DEBUG
int dcmi_debug = 1;
#else
int dcmi_debug = 0;
#endif
MODULE_PARM_DESC(dcmi_debug,  "Debug enabled or not");
module_param(dcmi_debug, int, 0644);


#define DCMI_DEV_NAME	"dcmi"

/* dcmi device structure */
struct dcmi_device dcmi_device;

/* major number for device */
static int dcmi_major;

/* sysfs */
struct class *dcmi_class;
struct device *dcmi_class_dev;

/**
 * Local Function Prototypes
 */
static int __init dcmi_init_module(void);
static void __exit dcmi_exit_module(void);

static int dcmi_open(struct inode *inode, struct file *file);
static int dcmi_release(struct inode *inode, struct file *file);
#if HAVE_UNLOCKED_IOCTL
static long u_dcmi_ioctl( struct file *file,
					  unsigned int cmd,
					  unsigned long data);
#endif
static int dcmi_ioctl(struct inode *inode,
					  struct file *file,
					  unsigned int cmd,
					  unsigned long data);

#if 0
static ssize_t dcmi_read(struct file *file,
						 char __user *ubuf,
						 size_t length,
						 loff_t *offset);
static ssize_t dcmi_write(struct file *file,
						  const char __user *ubuf,
						  size_t length,
						  loff_t *offset);
#endif

/**
 * file operations structure will be use dcmi char device.
 */
static struct file_operations dcmi_fops = {
	.owner = THIS_MODULE,
/*	.read = dcmi_read,*/
#if HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl = u_dcmi_ioctl,
#else
	.ioctl = dcmi_ioctl,
#endif
	.open = dcmi_open,
	.release = dcmi_release,
/*	.write = dcmi_write,*/
};

///**
// *	System Reboot Notifier
// */
//static int dcmi_reboot_handler(struct notifier_block *, unsigned long, void *);
//static struct notifier_block dcmi_nb = {
//		.notifier_call = dcmi_reboot_handler,
//		.next  = NULL,
//		.priority = INT_MAX, /* before real devices */
//};

///**
// * This function is called when the system is being rebooted.
// */
//static int dcmi_reboot_handler(struct notifier_block *nb, unsigned long event, void *unused)
//{
//	static int reboot_handled = 0;
//
//	if (reboot_handled)
//		return NOTIFY_DONE;
//
//	reboot_handled = 1;
//
//	switch(event) {
//	case SYS_RESTART:
//		printk(KERN_ERR "dcmi: system restart detected\n");
//		break;
//
//	case SYS_HALT:
//		printk(KERN_ERR "dcmi: system halting detected\n");
//		break;
//
//	case SYS_POWER_OFF:
//		printk(KERN_ERR "dcmi: system poweroff detected\n");
//		break;
//
//	default:
//		printk(KERN_ERR "dcmi: system reboot %0lx\n", event);
//	}
//
//	return NOTIFY_DONE;
//}

/**
 * Set up the cdev structure for dcmi device.
 *
 * @dev: char device struct
 * @hminor: minor number for registration char device
 * @fops: file operations structure
 *
 * @return 0 on success, <0 on failure.
 */
static int dcmi_registration_cdev(struct cdev *dev, int hminor,
				  struct file_operations *fops)
{
	int ret, devno = MKDEV(dcmi_major, hminor);

	cdev_init(dev, fops);
	dev->owner = THIS_MODULE;
	ret = cdev_add(dev, devno, 1);
	/* Fail gracefully if need be */
	if (ret) {
		printk(KERN_ERR "dcmi: Error %d registering dcmi device %d\n",
		       ret, hminor);
	}
	/* our device is "live" now */
	return ret;
}

/* Display the version of dcmi driver. */
static CLASS_ATTR_STRING(version, S_IRUGO,
			__stringify(dcmi_driver_string) " "
			__stringify(dcmi_driver_version) ".");

/**
 * dcmi_sysfs_create - creates a struct class to contain dcmi info
 *
 * @return 0 on success, <0 on failure.
 */
static int dcmi_sysfs_create(void)
{
	struct class *class;
	int err = 0;

	class = class_create(THIS_MODULE, DCMI_DRIVER_NAME);
	if (IS_ERR(class)) {
		err = PTR_ERR(class);
		printk(KERN_ERR "dcmi: Error creating dcmi class.\n");
		goto err_out;
	}

	err = class_create_file(class, &class_attr_version.attr);
	if (err) {
		class_destroy(class);
		printk(KERN_ERR "dcmi: Error creating dcmi class file.\n");
		goto err_out;
	}

	dcmi_class = class;
err_out:
	return err;
}

/**
 * dcmi_sysfs_destroy - destroys a struct class of dcmi info
 */
static void dcmi_sysfs_destroy(void)
{
	if ((dcmi_class == NULL) || (IS_ERR(dcmi_class)))
		return;

	class_remove_file(dcmi_class, &class_attr_version.attr);
	class_destroy(dcmi_class);
}

/**
 * dcmi_sysfs_device_create - adds device to sysfs for character devices
 *
 * @return 0 on success, <0 on failure.
 */
static int dcmi_sysfs_device_create(void)
{
	int err = 0;

	if ((dcmi_class == NULL) || (IS_ERR(dcmi_class))) {
		err = -EINVAL;
		goto err_out;
	}

	dcmi_class_dev = device_create(dcmi_class, NULL,
					     dcmi_device.cdev.dev,
					     NULL,
					     DCMI_DEV_NAME);
	if (IS_ERR(dcmi_class_dev))
		err = PTR_ERR(dcmi_class_dev);

err_out:
	return err;
}

/**
 * dcmi_sysfs_device_remove - unregisters the device entry on sysfs
 */
static void dcmi_sysfs_device_remove(void)
{
	if (dcmi_class_dev)
                device_destroy(dcmi_class, dcmi_device.cdev.dev);
}

/**
 * dcmi_init_module - Driver Registration Routine
 *
 * dcmi_init_module is the first routine called when the driver is
 * loaded.
 *
 * @return 0 on success, !=0 on failure.
 */
static int __init dcmi_init_module(void)
{
	int ret = 0;
	dev_t dev;
//	DCMI_STATUS dcmi_status;

	printk(KERN_INFO "dcmi: %s - version %s\n",
			dcmi_driver_string,
			dcmi_driver_version);
	printk(KERN_INFO "dcmi: %s\n", dcmi_copyright);

	/* registration of char devices */
	ret = alloc_chrdev_region(&dev,
							  DCMI_MINORS_BASE,
							  DCMI_MINORS_COUNT,
							  DCMI_DRIVER_NAME);
	if (ret) {
		printk(KERN_ERR "dcmi: Error allocating char device region.\n");
		goto end;
	}

	dcmi_major = MAJOR(dev);

	/* registration in sysfs interface */
	ret = dcmi_sysfs_create();
	if (ret)
		goto unregister;

	ret = dcmi_registration_cdev(&dcmi_device.cdev,
								 DCMI_MINOR_NUMBER,
								 &dcmi_fops);
	if (ret)
		goto destroy_sysfs;

	if (dcmi_sysfs_device_create()) {
		cdev_del(&dcmi_device.cdev);
		ret = -EAGAIN;
		goto destroy_sysfs;
	}

	/* allocates and initializes the dcmi device structure */
	if (init_dcmi_device(&dcmi_device) == NULL) {
		ret = -ENOMEM;
		goto dcmi_device_init_failure;
	}

#if 0
	if (dcmi_transport_load()) {
		//ret = -ENXIO;
		ret = -ENODEV;
		goto unlock_transport;
	}
#endif

	if ( (ret = dcmi_transport_init(&dcmi_device.trans)) ) {
		goto free_transport;
	}

//	ret = register_reboot_notifier(&dcmi_nb);
//	if ( ret != 0) {
//		printk(KERN_ERR "dcmi: Can't register reboot notifier\n");
//	}

//	/*
//	 * Connect Protocol Interface
//	 */
//	dcmi_status = dcmi_connect();
//	if (dcmi_status != DCMI_OK) {
//		printk(KERN_ERR "dcmi: dcmi_connect failed. Status=0x%X\n", dcmi_status);
//		ret = -ENODEV;
//		goto disconnect;
//	}
//	else
//	{
//		atomic_set(&dcmi_device.dcmi_state, DCMI_ENABLED);
//	}

	return ret;

//disconnect:
//	dcmi_disconnect();

free_transport:
	dcmi_transport_free(&dcmi_device.trans);

#if 0
unlock_transport:
	dcmi_transport_unlock();
#endif

dcmi_device_init_failure:
	;
destroy_sysfs:
	dcmi_sysfs_destroy();
unregister:
	unregister_chrdev_region(MKDEV(dcmi_major, DCMI_MINORS_BASE),
				 DCMI_MINORS_COUNT);
end:
	return ret;
}

module_init(dcmi_init_module);


/**
 * dcmi_exit_module - Driver Exit Cleanup Routine
 *
 * dcmi_exit_module is called just before the driver is removed
 * from memory.
 */
static void __exit dcmi_exit_module(void)
{
	//dcmi_disconnect();
//	unregister_reboot_notifier(&dcmi_nb);
	dcmi_transport_free(&dcmi_device.trans);

#if 0
	dcmi_transport_unlock();
#endif

	/* Now  unregister cdev. */
	cdev_del(&dcmi_device.cdev);
	dcmi_sysfs_device_remove();
	dcmi_sysfs_destroy();
	unregister_chrdev_region(MKDEV(dcmi_major, DCMI_MINORS_BASE),
							 DCMI_MINORS_COUNT);

	printk(KERN_INFO "dcmi: module unloaded.\n");
}

module_exit(dcmi_exit_module);

/**
 * dcmi_open - the open function
 */
static int dcmi_open(struct inode *inode, struct file *file)
{
	DCMI_STATUS dcmi_status;

	struct dcmi_file_private *file_ext;

	int if_num = iminor(inode);

	struct dcmi_device *dev = &dcmi_device;

	// For now we assume that only one minor exists for our driver
	// and we want to validate that assumption. In the future if
	// we support multiple devices, we would make use of this number
	// to associate each open with the proper device extension / file_private struct.
	if (if_num != DCMI_MINOR_NUMBER || (!dev)) {
		printk(KERN_ERR "dcmi: open failed due to invalid minor device ID\n");
		return -ENODEV;
	}

	if (dcmi_transport_lock()) {
		printk(KERN_ERR "dcmi: open failed - no heci driver module loaded?\n");
		return -ENODEV;
	}

	file_ext = dcmi_alloc_file_private(file);
	if (file_ext == NULL)
	{
		dcmi_transport_unlock();
		return -ENOMEM;
	}

	spin_lock_bh(&dev->device_lock);

	if (dev->open_handle_count >= DCMI_MAX_OPEN_HANDLE_COUNT) {
		spin_unlock_bh(&dev->device_lock);
		kfree(file_ext);
		dcmi_transport_unlock();
		return -ENFILE;
	}
	dev->open_handle_count++;
	//list_add_tail(&file_ext->link, &dev->file_list);
	spin_unlock_bh(&dev->device_lock);

	spin_lock(&file_ext->file_lock);
	file->private_data = file_ext;
	spin_unlock(&file_ext->file_lock);

	/*
	 * Connect Protocol Interface
	 */
	dcmi_status = dcmi_connect();
	if (dcmi_status != DCMI_OK) {
		printk(KERN_ERR "dcmi: dcmi_connect failure. Status=0x%X\n", dcmi_status);
		kfree(file_ext);
		spin_lock_bh(&dev->device_lock);
		dev->open_handle_count--;
		spin_unlock_bh(&dev->device_lock);
		dcmi_transport_unlock();
		return -ENODEV;
	}
	else
	{
		atomic_set(&dcmi_device.dcmi_state, DCMI_ENABLED);
	}

//used if heci connected while module init
//{
//	/*
//	 * ReConnect Protocol Interface
//	 * as maximum of one client can have opened dcmi file
//	 * purpose: reconnect after heci reset
//	 */
//	atomic_set(&dev->dcmi_state, DCMI_RESETING);
//
//	dcmi_status = dcmi_reset_connection();
//
//	if (dcmi_status != DCMI_OK) {
//		printk(KERN_ERR "dcmi: dcmi_reconnect failure. Status=0x%X\n", dcmi_status);
//		kfree(file_ext);
//		spin_lock_bh(&dev->device_lock);
//		dev->open_handle_count--;
//		spin_unlock_bh(&dev->device_lock);
//		return -ENODEV;
//	}
//	else
//	{
//		atomic_set(&dev->dcmi_state, DCMI_ENABLED);
//	}
//}
	/*
	 * Flush and reinitialize the transport buffer
	 */
	trans_flush_queue();

	try_module_get(THIS_MODULE);	//increase module usage counter
	return 0;
}

/**
 * dcmi_release - the release function
 */
static int dcmi_release(struct inode *inode, struct file *file)
{
	int rets = 0;
	int if_num = iminor(inode);
	struct dcmi_file_private *file_ext = file->private_data;
	struct dcmi_device *dev = &dcmi_device;

	if ((if_num != DCMI_MINOR_NUMBER) || (!file_ext))
		return -ENODEV;

	//printk(KERN_ERR "dcmi: about to call dcmi_disconnect on file closing\n");

    dcmi_disconnect();

	//spin_lock(&file_ext->file_lock);
	//spin_unlock(&file_ext->file_lock);
	kfree(file->private_data);
	file->private_data = NULL;

	spin_lock_bh(&dev->device_lock);
		if (dev->open_handle_count > 0)
			dev->open_handle_count--;
	spin_unlock_bh(&dev->device_lock);

	dcmi_transport_unlock();

	module_put(THIS_MODULE);		//decrease module usage counter
	return rets;
}

#if 0
/**
 * dcmi_read - the read client message function.
 */
static ssize_t dcmi_read(struct file *file, char __user *ubuf,
			 size_t length, loff_t *offset)
{
	int rets = 0;
	int if_num = iminor(file->f_path.dentry->d_inode);
	struct dcmi_file_private *file_ext = file->private_data;
	struct dcmi_device *dev = &dcmi_device;

	if ((if_num != DCMI_MINOR_NUMBER) || (!file_ext))
		return -ENODEV;

	if (atomic_read(&dcmi_device.trans.state) != DCMI_TRANS_CONNECTED) {
		return -ENODEV;
	}

	/* read */
	/* copy_to_user */
	//TODO implement in future

	DBG("end dcmi read rets= %d\n", rets);
	return rets;
}
#endif

#if 0
/**
 * dcmi_write - the write function.
 */
static ssize_t dcmi_write(struct file *file, const char __user *ubuf,
			  size_t length, loff_t *offset)
{
	int rets = 0;
	int if_num = iminor(file->f_path.dentry->d_inode);
	struct dcmi_file_private *file_ext = file->private_data;
	struct dcmi_device *dev = &dcmi_device;

	if ((if_num != DCMI_MINOR_NUMBER) || (!file_ext))
		return -ENODEV;

	if (atomic_read(&dcmi_device.trans.state) != DCMI_TRANS_CONNECTED) {
		return -ENODEV;
	}

	/* copy_from_user */
	/* write */
	/* copy_to_user result */
	//TODO implement in future
	return rets;
}
#endif

/**
 * dcmi_ioctl - the IOCTL function
 * Purpose:    user interface to the driver
 * Context:    called when user calls ioctl() on opened device node
 * Returns:    0 on success. errno on failure, error number > 0 on
 * 			   recoverable error
 * Parameters:
 * Notes:
 */
#if HAVE_UNLOCKED_IOCTL
static long u_dcmi_ioctl(struct file *file,
		      unsigned int cmd, unsigned long data)
{
    long ret_code = dcmi_ioctl(file->f_path.dentry->d_inode, file, cmd, data);
    return ret_code;
}
#endif

static int dcmi_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long data)
{
	int rets = 0;
	int if_num = iminor(inode);
	struct dcmi_device *dev = &dcmi_device;
	struct dcmi_file_private *file_ext = file->private_data;
	/* in user space */
	struct smi *u_msg = (struct smi *) data;
	struct smi k_msg;	/* all in kernel on the stack */

	DCMI_STATUS dcmi_status;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (if_num != DCMI_MINOR_NUMBER)
		return -ENODEV;

	if (!file_ext)
		return -ENXIO;

	if (u_msg == NULL)
		return -EINVAL;			//Invalid argument

	/*
	 * reconnect after heci reset
	 */
	if ( atomic_read(&dev->trans.resetRequested) )
	{

		atomic_set(&dev->dcmi_state, DCMI_RESETING);

		dcmi_status = dcmi_reset_connection(); //actual reset

		if (dcmi_status != DCMI_OK) {
			printk(KERN_ERR "dcmi: failure while recovering connection. Status=0x%X\n", dcmi_status);
			return -ENODEV;
		}
		else
		{
			atomic_set(&dev->dcmi_state, DCMI_ENABLED);
			atomic_set(&dev->trans.resetRequested, 0);
		}
	}

	if (atomic_read(&dev->trans.state) != DCMI_TRANS_CONNECTED) {
		DBG("dcmi: transport not connected. TransSate: %X\n", atomic_read(&dev->trans.state) );
		return -ENODEV;
	}

	/* first copy from user all data needed, shallow copy */
	if (copy_from_user(&k_msg, (void __user *) u_msg, sizeof(k_msg)) == -1) {
		DBG("dcmi_ioctl: copy_from_user failed.\n");
		return -EFAULT;
	}


	switch (cmd) {

	case IOCTL_IMB_SEND_MESSAGE:
		DBG(": IOCTL_IMB_SEND_MESSAGE\n");
		rets = dcmi_ioctl_imb_send_message(dev, if_num, u_msg, k_msg, file_ext);
		break;

	default:
		rets = -ENOTTY;		/* regarding POSIX standard ENOTTY should be returned, not EINVAL */
		break;
	}
	return rets;
}


MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel(R) Data Center Host Interface");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DCMI_DRIVER_VERSION);
