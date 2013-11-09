/*
 * linux/sound/soc/codecs/aic3xxx_tiload.c
 *
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 *
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * Rev 0.1 	 Tiload support    		Mistral         16-09-2010
 *
 *          The Tiload programming support is added to AIC3XXX.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/control.h>
#include <linux/slab.h>

#include <linux/mfd/tlv320aic3xxx-core.h>
#include "aic3xxx_tiload.h"

#undef DEBUG

/* enable debug prints in the driver */
#ifdef DEBUG
#define dprintk(x...) 	printk(x)
#else
#define dprintk(x...)
#endif

#define AIC3XXX_TiLoad

#ifdef AIC3XXX_TiLoad

static int cur_book = 0;
static int cur_page = 0;

static unsigned int aic3xxx_series_read(struct snd_soc_codec *codec,
		unsigned int reg, const char *pbuf, int count)
{
	struct aic3xxx *control;
	union aic3xxx_reg_union xreg;
	control = codec->control_data;

	xreg.aic3xxx_register.offset = reg;
	xreg.aic3xxx_register.book = cur_book;
	xreg.aic3xxx_register.page = cur_page;
	xreg.aic3xxx_register.reserved = 0;

	return aic3xxx_bulk_read(control, xreg.aic3xxx_register_int, count, pbuf);
}

static unsigned int aic3xxx_series_write(struct snd_soc_codec *codec,
		unsigned int reg, const char *pbuf, int count)
{
	struct aic3xxx *control;
	union aic3xxx_reg_union xreg;
	control = codec->control_data;

	xreg.aic3xxx_register.offset = reg;
	xreg.aic3xxx_register.book = cur_book;
	xreg.aic3xxx_register.page = cur_page;
	xreg.aic3xxx_register.reserved = 0;

	return aic3xxx_bulk_write(control, xreg.aic3xxx_register_int, count, pbuf);
}

int aic3xxx_driver_init(struct snd_soc_codec *codec);

static struct cdev *aic3xxx_cdev;
static int aic3xxx_major = 0;	/* Dynamic allocation of Mjr No. */
static int aic3xxx_opened = 0;	/* Dynamic allocation of Mjr No. */
static struct snd_soc_codec *aic3xxx_codec;
struct class *tiload_class;
static unsigned int magic_num = 0xE0;

/******************************** Debug section *****************************/

/*
 *----------------------------------------------------------------------------
 * Function : tiload_open
 *
 * Purpose  : open method for aic3xxx-tiload programming interface
 *----------------------------------------------------------------------------
 */
static int tiload_open(struct inode *in, struct file *filp)
{
	dprintk("TiLoad DRIVER : %s\n", __FUNCTION__);
	if (aic3xxx_opened) {
		printk("%s device is already opened\n", "aic3xxx");
		printk("%s: only one instance of driver is allowed\n",
		       "aic3xxx");
		return -1;
	}
	aic3xxx_opened++;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : tiload_release
 *
 * Purpose  : close method for aic3xxx_tilaod programming interface
 *----------------------------------------------------------------------------
 */
static int tiload_release(struct inode *in, struct file *filp)
{
	dprintk("TiLoad DRIVER : %s\n", __FUNCTION__);
	aic3xxx_opened--;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : tiload_read
 *
 * Purpose  : read method for mini dsp programming interface
 *----------------------------------------------------------------------------
 */
static ssize_t tiload_read(struct file *file, char __user * buf,
			   size_t count, loff_t * offset)
{
	static char rd_data[128];
	char reg_addr;
	size_t size;
#ifdef DEBUG
	int i;
#endif

	dprintk("TiLoad DRIVER : %s\n", __FUNCTION__);
	if (count > 128) {
		printk("Max 128 bytes can be read\n");
		count = 128;
	}

	/* copy register address from user space  */
	size = copy_from_user(&reg_addr, buf, 1);
	if (size != 0) {
		printk("read: copy_from_user failure\n");
		return -1;
	}
	/* Send the address to device thats is to be read */
	//mutex_lock(&aic3xxx_codec->mutex);
	aic3xxx_series_read(aic3xxx_codec, reg_addr, rd_data, count);
	//mutex_unlock(&aic3xxx_codec->mutex);
#ifdef DEBUG
	printk("[tiload]read reg_addr=%x, count=%x\n", reg_addr, count);
	for (i=0; i<count; i++)
	{
	    printk("[tiload]read rd_data[%d]=%x\n", reg_addr+i, rd_data[i]);
	}
#endif
	size = count;
	if (copy_to_user(buf, rd_data, size) != 0) {
		dprintk("copy_to_user failed\n");
		return -1;
	}

	return size;
}

/*
 *----------------------------------------------------------------------------
 * Function : tiload_write
 *
 * Purpose  : write method for aic3xxx_tiload programming interface
 *----------------------------------------------------------------------------
 */
static ssize_t tiload_write(struct file *file, const char __user * buf,
			    size_t count, loff_t * offset)
{
	static char wr_data[132];
#ifdef DEBUG
	int i;
#endif
	struct aic325x_priv *aic3xxx_private = snd_soc_codec_get_drvdata(aic3xxx_codec);
        int ret = 0;

	dprintk("TiLoad DRIVER : %s\n", __FUNCTION__);
	if (count > 132) {
		printk("Max 132 bytes can be write\n");
		count = 132;
	}
	/* copy buffer from user space  */
	if (copy_from_user(wr_data, buf, count)) {
		printk("copy_from_user failure\n");
		return -1;
	}
#ifdef DEBUG
	printk("[tiload] write size = %d\n", (int)count);
	for (i = 0; i < (int)count; i++) {
		printk("[tiload]write wr_data[%d]=%x\n", i, wr_data[i]);
	}
#endif
	if (wr_data[0] == 0) {
		//mutex_lock(&aic3xxx_codec->mutex);
		//aic3xxx_change_page(aic3xxx_codec, wr_data[1]);
		//mutex_unlock(&aic3xxx_codec->mutex);
		cur_page = wr_data[1];
		return count;
	}

	if ((wr_data[0] == 127) && (cur_page == 0)) {
		//mutex_lock(&aic3xxx_codec->mutex);
		//aic3xxx_change_book(aic3xxx_codec, wr_data[1]);
		//mutex_unlock(&aic3xxx_codec->mutex);
		cur_book = wr_data[1];
		return count;
	}

	//mutex_lock(&aic3xxx_codec->mutex);
	//if (0 == aic3xxx_codec->hw_write(aic3xxx_codec->control_data, wr_data, count))
	//{
	//	ret = count;
	//}
	//else
	//{
	//    ret = 0;
	//}
	//mutex_unlock(&aic3xxx_codec->mutex);

	ret = aic3xxx_series_write(aic3xxx_codec, wr_data[0], &wr_data[1], count-1);

	return ret;
}

static long tiload_ioctl( struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	int num = 0;
	u8 *buf;
	int i,j;
	void __user *argp = (void __user *)arg;
	if (_IOC_TYPE(cmd) != AIC3XXX_IOC_MAGIC)
		return -ENOTTY;

	dprintk("TiLoad DRIVER : %s cmd:0x%x\n", __FUNCTION__, cmd);
	switch (cmd) {
	case AIC3XXX_IOMAGICNUM_GET:
		num = copy_to_user(argp, &magic_num, sizeof(int));
		break;
	case AIC3XXX_IOMAGICNUM_SET:
		num = copy_from_user(&magic_num, argp, sizeof(int));
		break;
	}
	return num;
}

/*********** File operations structure for aic3xxx-tiload programming *************/
static struct file_operations aic3xxx_fops = {
	.owner = THIS_MODULE,
	.open = tiload_open,
	.release = tiload_release,
	.read = tiload_read,
	.write = tiload_write,
	.unlocked_ioctl = tiload_ioctl,
};

/*
 *----------------------------------------------------------------------------
 * Function : aic3xxx_driver_init
 *
 * Purpose  : Register a char driver for dynamic aic3xxx-tiload programming
 *----------------------------------------------------------------------------
 */
int aic3xxx_driver_init(struct snd_soc_codec *codec)
{
	int result;

	dev_t dev;
	aic3xxx_codec = codec;

	dprintk("TiLoad DRIVER : %s\n", __FUNCTION__);
	dprintk("allocating dynamic major number\n");

	result = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	if (result < 0) {
		dprintk("cannot allocate major number %d\n", aic3xxx_major);
		return result;
	}

	aic3xxx_cdev = cdev_alloc();
	cdev_init(aic3xxx_cdev, &aic3xxx_fops);
	aic3xxx_cdev->owner = THIS_MODULE;
	aic3xxx_cdev->ops = &aic3xxx_fops;

	if (cdev_add(aic3xxx_cdev, dev, 1) < 0) {
		dprintk("aic3xxx_driver: cdev_add failed \n");
		unregister_chrdev_region(dev, 1);
		aic3xxx_cdev = NULL;
		return 1;
	}

	tiload_class = class_create(THIS_MODULE, DEVICE_NAME);
	device_create(tiload_class, NULL, MKDEV(MAJOR(dev), 0), NULL, DEVICE_NAME);
	printk("Registered aic3xxx TiLoad driver, Major number: %d \n",
	       aic3xxx_major);
	return 0;
}

#endif
