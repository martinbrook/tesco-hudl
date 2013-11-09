/* drivers/video/backlight/lp8556_backlight.c
 *
 * Copyright (C) 2009-2011 Rockchip Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/clk.h>

#include <linux/earlysuspend.h>
#include <asm/io.h>
#include <mach/board.h>
#include <plat/pwm.h>

#define PWM_DIV              PWM_DIV2
#define PWM_APB_PRE_DIV      1000
#define BL_STEP              (255)
#define MAX_BRIGHTNESS_CORRECT (50)
#define Line_BL_ratio             45


/*
 * Debug
 */
#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif

#define read_pwm_reg(addr)              __raw_readl(pwm_base + addr)

static struct backlight_device *lp8556_bl;

static int suspend_flag = 0;
static int lp8556_en_flag = 0;

static struct i2c_client *this_client;

static int i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;

	msg.flags=client->flags;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;
	msg.scl_rate = 200000;
	
	ret=i2c_transfer(client->adapter,&msg, 1);
	DBG("0x%x,0x%x,ret = %d\n",data[0],data[1],ret);
	return ret;
}
#if 0
static int i2c_read_bytes(struct i2c_client *client, u8 reg, int *val)
{
	struct i2c_msg msg;
	int ret=-1;
    u8 buf[2];

	msg.flags=0;
	msg.addr=client->addr;
	msg.len=1;
	msg.buf=buf;
	msg.scl_rate = 200000;
	buf[0]=reg;

    ret=i2c_transfer(client->adapter,&msg, 1);
    DBG("i2c_read_bytes write reg:0x%x,buf[0]:0x%x,buf[1]=0x%x\n",reg,buf[0],buf[1]);
    
    msg.flags = I2C_M_RD;
    
    ret=i2c_transfer(client->adapter,&msg, 1);
    printk("i2c_read_bytes read reg:0x%x,buf[0]:0x%x,buf[1]=0x%x\n",reg,buf[0],buf[1]);

	*val=buf[0];
	return ret;
}
#endif

int convertint(const char s[])  
{  
    int i;  
    int n = 0;  
    for (i = 0; s[i] >= '0' && s[i] <= '9'; ++i)  
    {  
        n = 10 * n + (s[i] - '0');  
    }  
    return n;  
} 

static ssize_t backlight_write(struct device *dev, 
		struct device_attribute *attr,const char *buf, size_t count)
{
   
	struct rk29_bl_info *lp8556_bl_info = bl_get_data(lp8556_bl);
	int number;

	number = convertint(buf);
	
	lp8556_bl_info->min_brightness=number;
	return 0;
}


static ssize_t backlight_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//DBG("lp8556_bl_info->min_brightness=%d\n",lp8556_bl_info->min_brightness);
	DBG("%s : %s\n", __FILE__, __FUNCTION__);
	return 0;
}
static DEVICE_ATTR(rk29backlight, 0660, backlight_read, backlight_write);

static DEFINE_MUTEX(backlight_mutex);

static inline void lp8556_bl_min_brightness_check(struct rk29_bl_info *lp8556_bl_info)
{

	if (lp8556_bl_info->min_brightness < 0 || lp8556_bl_info->min_brightness > BL_STEP)
		lp8556_bl_info->min_brightness = MAX_BRIGHTNESS_CORRECT;


}	
static inline void lp8556_bl_max_brightness_check(struct rk29_bl_info *lp8556_bl_info)
{
	if (lp8556_bl_info->max_brightness <= 0 || lp8556_bl_info->max_brightness > BL_STEP)
		lp8556_bl_info->max_brightness = BL_STEP;
}	

int lp8556_backlight_hw_init(struct i2c_client *client)
{
    uint8_t Write_data1[] ={0xa1, 0x76}; //hight bit(8~11)(0~0X66e set backlight)
    uint8_t Write_data2[] ={0xa0, 0x66};  //low bit(0~7)  20mA
    uint8_t Write_data3[] ={0x16, 0x0f};
    uint8_t Write_data4[] ={0xa9, 0x60};
    uint8_t Write_data5[] ={0x9e, 0x02};
    uint8_t Write_data6[] ={0xa2, 0x23}; //23
    uint8_t Write_data0[] ={0x01, 0x05}; //0x03 pwm+I2c set brightness,0x5 I2c set brightness

    i2c_write_bytes(client,Write_data0,2);
    i2c_write_bytes(client,Write_data1,2);
    i2c_write_bytes(client,Write_data2,2);
	i2c_write_bytes(client,Write_data3,2);
    i2c_write_bytes(client,Write_data4,2);
    i2c_write_bytes(client,Write_data5,2);
    i2c_write_bytes(client,Write_data6,2);
    return 0;
}

int lp8556_bl_val_scalor_line(struct rk29_bl_info *lp8556_bl_info,int brightness)
{
	//lp8556_bl_min_brightness_check(lp8556_bl_info);
	//lp8556_bl_max_brightness_check(lp8556_bl_info);
	if(lp8556_bl_info->max_brightness<lp8556_bl_info->min_brightness)
		lp8556_bl_info->max_brightness=lp8556_bl_info->min_brightness;

	if(brightness>lp8556_bl_info->max_brightness)
		brightness=lp8556_bl_info->max_brightness;
	else if(brightness<lp8556_bl_info->min_brightness)
		brightness=lp8556_bl_info->min_brightness;
	#if 0
		brightness = brightness*(lp8556_bl_info->max_brightness - lp8556_bl_info->min_brightness);
		brightness = (brightness/255) + lp8556_bl_info->min_brightness;
	#endif
	return brightness;
}
int lp8556_bl_val_scalor_conic(struct rk29_bl_info *lp8556_bl_info,int brightness)
{
	
	//lp8556_bl_min_brightness_check(lp8556_bl_info);
	//lp8556_bl_max_brightness_check(lp8556_bl_info);
	
	if(lp8556_bl_info->max_brightness<lp8556_bl_info->min_brightness)
		lp8556_bl_info->max_brightness=lp8556_bl_info->min_brightness;
	#if 0
	    	brightness = (brightness*brightness)*(lp8556_bl_info->max_brightness - lp8556_bl_info->min_brightness);
		brightness = (brightness/(BL_STEP*BL_STEP)) + lp8556_bl_info->min_brightness;
	#else
		if(brightness<lp8556_bl_info->min_brightness)
			brightness=lp8556_bl_info->min_brightness;
		brightness = (brightness-lp8556_bl_info->min_brightness)*(brightness-lp8556_bl_info->min_brightness);
		brightness = (brightness/(lp8556_bl_info->max_brightness - lp8556_bl_info->min_brightness)) + lp8556_bl_info->min_brightness;
	#endif
	
	if(brightness > lp8556_bl_info->max_brightness)
		brightness = lp8556_bl_info->max_brightness;
	if(brightness < lp8556_bl_info->min_brightness)	
		brightness = lp8556_bl_info->min_brightness;
	
	return brightness;
}

int lp8556_bl_val_scalor_Polyline(struct rk29_bl_info *lp8556_bl_info,int brightness)
{
	int conic_brightness;
	
	conic_brightness=lp8556_bl_val_scalor_conic(lp8556_bl_info,brightness);
	
	brightness=(brightness*Line_BL_ratio+conic_brightness*(100-Line_BL_ratio))/100;
	
	return brightness;
}


static int lp8556_bl_update_status(struct backlight_device *bl)
{
	struct rk29_bl_info *lp8556_bl_info = bl_get_data(bl);
	int brightness = 0;
	
	uint8_t Write_data[] ={0x00, 0x66};
	mutex_lock(&backlight_mutex);

	//BL_CORE_DRIVER2 is the flag if backlight is into early_suspend.
	if (suspend_flag && (bl->props.state & BL_CORE_DRIVER2))
	    goto out;

	brightness = bl->props.brightness;

	if(brightness)
	{
		if(lp8556_bl_info->brightness_mode==BRIGHTNESS_MODE_LINE)
			brightness=lp8556_bl_val_scalor_line(lp8556_bl_info,brightness);
		else if(lp8556_bl_info->brightness_mode==BRIGHTNESS_MODE_CONIC)
			brightness=lp8556_bl_val_scalor_conic(lp8556_bl_info,brightness);
		else if(lp8556_bl_info->brightness_mode==BRIGHTNESS_MODE_Polyline)
			brightness=lp8556_bl_val_scalor_Polyline(lp8556_bl_info,brightness);
	}

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;	

	if (bl->props.state & BL_CORE_DRIVER3)
		brightness = 0;	

    if ((bl->props.state & BL_CORE_DRIVER2) && !suspend_flag ){
		brightness = 0;
		suspend_flag = 1;
	}else if(!(bl->props.state & BL_CORE_DRIVER2) && suspend_flag ){
		suspend_flag = 0;
	}
    
    DBG("%s,req brightness=%d,real is=%d\n", __FUNCTION__,bl->props.brightness,brightness);

	DBG("%s,suspend_flag:%d,state:%x,power = %x,fb_blank=%x\n",__FUNCTION__,suspend_flag,bl->props.state,bl->props.power,bl->props.fb_blank);

	if((bl->props.state & BL_CORE_DRIVER1) && (brightness ==0)){  
		bl->props.state &= ~BL_CORE_DRIVER1;
		if (lp8556_bl_info->pwm_suspend){
			lp8556_bl_info->pwm_suspend();
            lp8556_en_flag = 0;
		}
	}else if(!(bl->props.state & BL_CORE_DRIVER1) && (brightness != 0)){
		bl->props.state |= BL_CORE_DRIVER1;
        if (lp8556_bl_info->pwm_resume){
			lp8556_bl_info->pwm_resume();
            lp8556_en_flag = 1;
        }
        lp8556_backlight_hw_init(this_client);
		Write_data[1]=brightness;	
		i2c_write_bytes(this_client,Write_data,2);
 	}else if(lp8556_en_flag){
 		Write_data[1]= brightness;
	    i2c_write_bytes(this_client,Write_data,2);
 	}

	DBG("%s:line=%d,brightness = %d, div_total = %d, divh = %d state=%x \n",__FUNCTION__,__LINE__,brightness, div_total, divh,bl->props.state);
out:
	mutex_unlock(&backlight_mutex);
	return 0;
}

static struct backlight_ops lp8556_bl_ops = {
	.update_status	= lp8556_bl_update_status,
};

static void lp8556_backlight_work_func(struct work_struct *work)
{
	lp8556_bl_update_status(lp8556_bl);
}
static DECLARE_DELAYED_WORK(lp8556_backlight_work, lp8556_backlight_work_func);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lp8556_bl_suspend(struct early_suspend *h)
{
	int brightness = lp8556_bl->props.brightness;
	DBG("%s\n",__func__);
	
	cancel_delayed_work_sync(&lp8556_backlight_work);

	lp8556_bl->props.state |= BL_CORE_DRIVER2;

	if (lp8556_bl->props.brightness) {
		lp8556_bl->props.brightness = 0;
		lp8556_bl_update_status(lp8556_bl);
		lp8556_bl->props.brightness = brightness;
	}
}

static void lp8556_bl_resume(struct early_suspend *h)
{
	struct rk29_bl_info *lp8556_bl_info = bl_get_data(lp8556_bl);
	DBG("%s\n",__func__);
	lp8556_bl->props.state &= ~BL_CORE_DRIVER2;
	
	schedule_delayed_work(&lp8556_backlight_work, msecs_to_jiffies(lp8556_bl_info->delay_ms));
}

static struct early_suspend lp8556_bl_early_suspend = {
	.suspend = lp8556_bl_suspend,
	.resume = lp8556_bl_resume,
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1,
};
#endif

void rk29_backlight_set(bool on)
{
	printk("%s: set %d\n", __func__, on);
	if(on){
		lp8556_bl->props.state &= ~BL_CORE_DRIVER3;
		lp8556_bl_update_status(lp8556_bl);
	}else{
		lp8556_bl->props.state |= BL_CORE_DRIVER3;
		lp8556_bl_update_status(lp8556_bl);
	}
	return;
}
EXPORT_SYMBOL(rk29_backlight_set);

static int lp8556_backlight_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

    uint8_t Write_data[] ={0x00, 0xff};  

  	struct rk29_bl_info *lp8556_bl_info = client->dev.platform_data;
  	struct backlight_properties props;
	printk("%s , line:%d .\n",__func__,__LINE__);
	
	this_client = client;
	
	if (lp8556_bl) {
		printk(KERN_CRIT "%s: backlight device register has existed \n",
				__func__);
        ret = -EEXIST;	
        goto INIT_OUT;
	}

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = BL_STEP;

    if(lp8556_bl_info->io_init)
    {
        lp8556_bl_info->io_init();
        lp8556_en_flag = 1;
    }
    
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk( "%s i2c check func err.\n",__func__);
		ret = -ENODEV;	
        goto INIT_OUT;
	}
    
	lp8556_bl = backlight_device_register("rk28_bl", &client->dev, lp8556_bl_info, &lp8556_bl_ops, &props);
	if (!lp8556_bl) {
		printk( "%s: backlight device register error\n",
				__func__);
		ret = -ENODEV;	
        goto INIT_OUT;	
	}

    lp8556_backlight_hw_init(client);

    lp8556_bl->props.brightness = BL_STEP/2;       // init brightness is 127
    Write_data[1] = lp8556_bl->props.brightness;
    i2c_write_bytes(client,Write_data,2);

	//schedule_delayed_work(&lp8556_backlight_work, msecs_to_jiffies(lp8556_bl_info->delay_ms));

	ret = device_create_file(&client->dev,&dev_attr_rk29backlight);
	if(ret)
	{
		dev_err(&client->dev, "failed to create sysfs file\n");
	}

	register_early_suspend(&lp8556_bl_early_suspend);

INIT_OUT:
    return ret;
}

static int lp8556_backlight_remove(struct i2c_client *client)
{
    //printk("%s , line:%d .\n",__func__,__LINE__);

	return 0;
}
static void lp8556_backlight_shutdown(struct i2c_client *client)
{
    struct rk29_bl_info *lp8556_bl_info = client->dev.platform_data;
    //printk("%s , line:%d .\n",__func__,__LINE__);
    if (lp8556_bl_info->pwm_suspend)
			lp8556_bl_info->pwm_suspend();
}

static const struct i2c_device_id lp8556_backlight_id[] = {
	{ "rk31_lcd_backlight", 0 },
	{ }
};

static struct i2c_driver lp8556_backlight_driver = {
	.probe		= lp8556_backlight_probe,
	.remove		= lp8556_backlight_remove,
	.id_table	= lp8556_backlight_id,
	.driver = {
		.name	= "struct",
		.owner = THIS_MODULE,
	},
	.shutdown	= lp8556_backlight_shutdown,
};

static int __init lp8556_backlight_init(void)
{
    i2c_add_driver(&lp8556_backlight_driver);
	return 0;
}
fs_initcall_sync(lp8556_backlight_init);
