#ifndef _HELLO_ANDROID_H_
#define  _HELLO_ANDROID_H_

#include <linux/cdev.h>
#include <linux/semaphore.h>

#define GPS_BCM4751_CTL_DEVICE_NODE_NAME "gps_bcm4751_ctl"
#define GPS_BCM4751_CTL_DEVICE_RESET_N "reset_n"
#define GPS_BCM4751_CTL_DEVICE_STANDBY "standby"
#define GPS_BCM4751_CTL_DEVICE_PROC_NAME "gps_bcm4751_ctl"
#define GPS_BCM4751_CTL_DEVICE_CLASS_NAME "gps_bcm4751_ctl"

struct gps_bcm4751_ctl_jelly_dev {
	int val;
	int reset;
	int enable;
	struct semaphore sem;
	struct cdev dev;
};
#endif