#ifndef _GPS_BCM4751_H_
#define  _GPS_BCM4751_H_

#include <linux/cdev.h>
#include <linux/semaphore.h>


#define GPS_BCM4751_RESET  RK30_PIN0_PD5
#define GPS_BCM4751_STANDBY  RK30_PIN0_PD4


struct gps_gpio_platform_data {
	int reset_state;
	int standby_state;
	void (*gps_standby_level)(int);
	void (*gps_reset_level)(int);
	void (*gps_power_init)(void);
};
#endif