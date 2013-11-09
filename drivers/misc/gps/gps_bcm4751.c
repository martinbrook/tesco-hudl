
#include <linux/sysdev.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/device.h>
#include <mach/gpio.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <linux/device.h>

#include "gps_bcm4751.h"
static ssize_t gps_standby_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        int len = 0;
        struct gps_gpio_platform_data *pdata = dev->platform_data;

        len += sprintf(buf + len, "%u\n", pdata->standby_state);
        printk("======== %s len = %d\n",__func__,len);
        return len;
}

static ssize_t gps_standby_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
        unsigned long state = simple_strtoul(buf, NULL, 10);
        struct gps_gpio_platform_data *pdata = dev->platform_data;

        pdata->standby_state = (int)state;
        printk("\n ****** standby_state = %d \n",pdata->standby_state);

        if(state)
                pdata->gps_standby_level(1); //standby on
        else
                pdata->gps_standby_level(0); //standby off

        return size;
}

static ssize_t gps_reset_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        int len = 0;
        struct gps_gpio_platform_data *pdata = dev->platform_data;


        len += sprintf(buf + len, "%u\n", pdata->reset_state);
        printk("======== %s len = %d\n",__func__,len);


        return len;
}

static ssize_t gps_reset_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
        unsigned long state = simple_strtoul(buf, NULL, 10);
        struct gps_gpio_platform_data *pdata = dev->platform_data;
        printk("\n ******%s  %s  line = %d \n",__func__,__FILE__,__LINE__);

        pdata->reset_state = (int)state;
        printk("\n ****** reset_state = %d \n",pdata->reset_state);

        if(state)
                pdata->gps_reset_level(1); //reset on
        else

                pdata->gps_reset_level(0); //reset off

        return size;
}

static DEVICE_ATTR(GPS_nRST, 0644, gps_reset_show, gps_reset_store);
static DEVICE_ATTR(GPS_PWR_EN, 0644, gps_standby_show, gps_standby_store);

static int gps_gpio_probe(struct platform_device *pdev)
{
    int ret;
    struct gps_gpio_platform_data *pdata = pdev->dev.platform_data;

    pdata->standby_state = 0;
    pdata->reset_state = 1;
    pdata->gps_power_init();
    ret = device_create_file(&pdev->dev, &dev_attr_GPS_nRST);
    printk("////// device_create_file dev_attr_GPS_nRST ret  = %d \n",ret);
    if(ret)
        return ret;
    else
        return device_create_file(&pdev->dev, &dev_attr_GPS_PWR_EN);

}

static int gps_gpio_remove(struct platform_device *pdev)
{
    struct gps_gpio_platform_data *pdata = pdev->dev.platform_data;
    pdata->gps_reset_level(0);
    pdata->gps_standby_level(0);

    return 0;
}

struct platform_driver gps_gpio_driver = {
    .probe = gps_gpio_probe,
    .remove = gps_gpio_remove,
    .driver = {
        .name   = "gps_bcm4751_gpio",
        .owner  = THIS_MODULE,
    },
};

static int __init gps_gpio_init(void)
{
    return platform_driver_register(&gps_gpio_driver);
}


static void __exit  gps_gpio_exit(void)
{
    platform_driver_unregister(&gps_gpio_driver);
}

late_initcall(gps_gpio_init);
module_exit(gps_gpio_exit);

MODULE_AUTHOR("Martin.Cai@KEENHI.COM");
MODULE_LICENSE("GPL v2");


