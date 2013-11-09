#include <linux/init.h>   
#include <linux/module.h>   
#include <linux/types.h>   
#include <linux/fs.h>   
#include <linux/proc_fs.h>   
#include <linux/device.h>   
#include <asm/uaccess.h>   
#include <mach/gpio.h>


#include "gps_bcm4751_ctl.h"   

#define GPS_BCM4751_RESET  RK30_PIN0_PD5
#define GPS_BCM4751_EN  RK30_PIN0_PD4


/*主设备号和从设备号变量*/
static int gps_bcm4751_ctl_major = 0;
static int gps_bcm4751_ctl_minor = 0;

/*设备类别和设备变量*/
static struct class* gps_bcm4751_ctl_class = NULL;
static struct gps_bcm4751_ctl_jelly_dev* gps_bcm4751_ctl_dev = NULL;

/*传统的设备文件操作方法相关参数*/
static int gps_bcm4751_ctl_open(struct inode* node, struct file* filp);
static int gps_bcm4751_ctl_release(struct inode* node, struct file* filp);
static ssize_t gps_bcm4751_ctl_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos);
static ssize_t gps_bcm4751_ctl_write(struct file* filp, const char __user *buf, size_t count, loff_t* f_pos);

/*此设备文件的ops，操作方法表*/
static struct file_operations gps_bcm4751_ctl_fops = {
	.owner = THIS_MODULE,
	.open = gps_bcm4751_ctl_open,
	.release = gps_bcm4751_ctl_release,
	.read = gps_bcm4751_ctl_read,
	.write = gps_bcm4751_ctl_write,
}; /*此处不能缺少分号*/

/*访问设置属性方法*/
static ssize_t gps_bcm4751_ctl_reset_show 
(struct device *dev, struct device_attribute* attr, char* buf);
static ssize_t gps_bcm4751_ctl_reset_store
(struct device *dev, struct device_attribute* attr, const char* buf, size_t count);

static ssize_t gps_bcm4751_ctl_enable_show 
(struct device *dev, struct device_attribute* attr, char* buf);
static ssize_t gps_bcm4751_ctl_enable_store
(struct device *dev, struct device_attribute* attr, const char* buf, size_t count);

/*定义设备属性，val从哪来的呢？？？*/
static DEVICE_ATTR(reset, S_IRUGO|S_IWUSR, gps_bcm4751_ctl_reset_show, gps_bcm4751_ctl_reset_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, gps_bcm4751_ctl_enable_show, gps_bcm4751_ctl_enable_store);


/*定义传统的设备文件访问的四个方法*/
/*1. 打开设备文件的方法*/
static int gps_bcm4751_ctl_open(struct inode* node, struct file* filp){
	struct gps_bcm4751_ctl_jelly_dev* dev;
	/*将自定义的设备结构体保存在文件指针的私有数据中，以便访问设备时使用*/
	dev = container_of(node->i_cdev, struct gps_bcm4751_ctl_jelly_dev, dev);
	filp->private_data = dev;
	
	return 0;
}

/*2. 释放设备文件的方法, 空实现.*/
static int gps_bcm4751_ctl_release(struct inode* node, struct file* filp){
	return 0;
}

/*3. 读取设备的寄存器val的值*/
static ssize_t gps_bcm4751_ctl_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos){
	ssize_t err = 0;
	struct gps_bcm4751_ctl_jelly_dev* dev = filp->private_data;
	
	/*同步访问*/
	if(down_interruptible(&(dev->sem))){
		return -ERESTARTSYS;
	}
	/*检查是否提供足够的缓冲空间*/
	if(count < sizeof(dev->val)) {
		goto out;
	}
	
	/*将寄存器的值拷贝到用户提供的用户空间缓冲区*/
	if(copy_to_user(buf, &(dev->val), sizeof(dev->val))){
		err = -EFAULT;
		goto out;
	}
	
	err = sizeof(dev->val);
	
out:
	up(&(dev->sem));
	return err;
}

/*4. 写设备的寄存器val的值*/
static ssize_t gps_bcm4751_ctl_write(struct file* filp, const char __user *buf, size_t count, loff_t* f_pos){
	ssize_t err = 0;
	//long value = 0;
	struct gps_bcm4751_ctl_jelly_dev* dev = filp->private_data;
	
	
	/*同步访问*/
	if(down_interruptible(&(dev->sem))){
		return -ERESTARTSYS;
	}
	
	if(count != sizeof(dev->val)){
		goto out;
	}
	
	/*将用户提供的缓冲区的数据写入寄存器val中*/
	if(copy_from_user(&(dev->val), buf, count)) {
		err = -EFAULT;
		goto out;
	}
	
	err = sizeof(dev->val);

out:
	up(&(dev->sem));
	return err;
}
/*通过设备属性文件获取和设置信息*/
/*读取寄存器reset的值到缓冲buffer，内部使用！！*/
static ssize_t __gps_bcm4751_ctl_get_reset(struct gps_bcm4751_ctl_jelly_dev* dev, char* buf){
	int val = 0;
	/*同步访问*/
	if(down_interruptible(&(dev->sem))){
		return -ERESTARTSYS;
	}
	
	val = dev->reset;
	up(&(dev->sem));
	
	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

/*设置寄存器reset的值，从缓冲区buffer得到数据， 内部使用*/
static ssize_t __gps_bcm4751_ctl_set_reset(struct gps_bcm4751_ctl_jelly_dev* dev, const char* buf, size_t count){
	int val = 0;
	
	/*字符串转数字*/
	val = simple_strtol(buf,NULL,10);
	
	/*同步访问*/
	if(down_interruptible(&(dev->sem))){
		return -ERESTARTSYS;
	}
	gpio_set_value(GPS_BCM4751_RESET, val == 0? GPIO_LOW:GPIO_HIGH);
	dev->reset = val;
	printk(KERN_ALERT" __gps_bcm4751_ctl_set_reset %d\n", val);

	up(&(dev->sem));
	return count;
}
/*通过设备属性文件获取和设置信息*/
/*读取寄存器enable的值到缓冲buffer，内部使用！！*/
static ssize_t __gps_bcm4751_ctl_get_enable(struct gps_bcm4751_ctl_jelly_dev* dev, char* buf){
	int val = 0;
	/*同步访问*/
	if(down_interruptible(&(dev->sem))){
		return -ERESTARTSYS;
	}
	
	val = dev->enable;
	up(&(dev->sem));
	
	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

/*设置寄存器reset的值，从缓冲区buffer得到数据， 内部使用*/
static ssize_t __gps_bcm4751_ctl_set_enable(struct gps_bcm4751_ctl_jelly_dev* dev, const char* buf, size_t count){
	int val = 0;
	
	/*字符串转数字*/
	val = simple_strtol(buf,NULL,10);
	
	if(val !=1 && val != 0)
	{
		return -1;
	}
	/*同步访问*/
	if(down_interruptible(&(dev->sem))){
		return -ERESTARTSYS;
	}
	gpio_set_value(GPS_BCM4751_EN, val == 0? GPIO_LOW:GPIO_HIGH);
	dev->enable = val;
	printk(KERN_ALERT" __gps_bcm4751_ctl_set_enable %d\n", val);
	up(&(dev->sem));
	return count;
}

/*读取设备属性reset*/
static ssize_t gps_bcm4751_ctl_reset_show
(struct device* dev, struct device_attribute* attr, char* buf){
	struct gps_bcm4751_ctl_jelly_dev* hdev = (struct gps_bcm4751_ctl_jelly_dev*) dev_get_drvdata(dev);
	
	return __gps_bcm4751_ctl_get_reset(hdev, buf);
}

/*写设备属性reset*/   
static ssize_t gps_bcm4751_ctl_reset_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {    
    struct gps_bcm4751_ctl_jelly_dev* hdev = (struct gps_bcm4751_ctl_jelly_dev*)dev_get_drvdata(dev);     
       
    return __gps_bcm4751_ctl_set_reset(hdev, buf, count);   
}

/*读取设备属性enable*/
static ssize_t gps_bcm4751_ctl_enable_show
(struct device* dev, struct device_attribute* attr, char* buf){
	struct gps_bcm4751_ctl_jelly_dev* hdev = (struct gps_bcm4751_ctl_jelly_dev*) dev_get_drvdata(dev);
	
	return __gps_bcm4751_ctl_get_enable(hdev, buf);
}

/*写设备属性enable*/   
static ssize_t gps_bcm4751_ctl_enable_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {    
    struct gps_bcm4751_ctl_jelly_dev* hdev = (struct gps_bcm4751_ctl_jelly_dev*)dev_get_drvdata(dev);     
       
    return __gps_bcm4751_ctl_set_enable(hdev, buf, count);   
}

/*通过proc文件系统访问文件。*/
/*读取设备寄存器val的值，保存在page缓冲区中
static ssize_t gps_bcm4751_ctl_proc_read (char* page, char** start, off_t off, int count, int* eof, void* data){
	if(off>0){
		*eof = 1;
		return 0;
	}
	return __gps_bcm4751_ctl_get_val(gps_bcm4751_ctl_dev, page);
}
*/
/*把缓冲区的值buff保存到设备寄存器val中
static ssize_t gps_bcm4751_ctl_proc_write
(struct file* filp, const char __user *buff, unsigned long len, void* data){
	int err = 0;
	char* page = NULL;
	if(len > PAGE_SIZE){
		printk(KERN_ALERT"The buffer is too large: %lu.\n", len);
		return -EFAULT;
	}
	
	page = (char*)__get_free_page(GFP_KERNEL);
	if(!page){
		printk(KERN_ALERT"Failed to alloc page. \n");
		return -ENOMEM;
	}
	
	//先把用户提供的数据从用户区缓冲区拷贝到内核缓冲区去
	if(copy_from_user(page, buff, len)){
		printk(KERN_ALERT"Failed to copy buff from user.\n");
		err = -EFAULT;
		goto out;
	}
	
	err = __gps_bcm4751_ctl_set_val(gps_bcm4751_ctl_dev, page, len);
	
out:
	free_page((unsigned long)page);
	return 	err;
}
*/
/*创建/proc/hello文件
static void gps_bcm4751_ctl_create_proc(void){
	struct proc_dir_entry* entry;
	entry = create_proc_entry(GPS_BCM4751_CTL_DEVICE_PROC_NAME, 0, NULL);
	if(entry){
		//Linux don't have it anymore
		//entry->owner = THIS_MODULE;
		entry->read_proc = gps_bcm4751_ctl_proc_read;
		entry->write_proc = gps_bcm4751_ctl_proc_write;
	}
}
*/

/*删除/proc/hello文件
static void gps_bcm4751_ctl_remove_proc(){
	remove_proc_entry(GPS_BCM4751_CTL_DEVICE_PROC_NAME, NULL);
}
*/


/*加载和卸载模块的方法*/
/*初始化设备*/
static int __gps_bcm4751_ctl_setup_dev(struct gps_bcm4751_ctl_jelly_dev * dev) {
	int err;
	dev_t devno = MKDEV(gps_bcm4751_ctl_major, gps_bcm4751_ctl_minor);
	
	memset( dev, 0 , sizeof(struct gps_bcm4751_ctl_jelly_dev));
	/*初始化*/
	cdev_init(&(dev->dev), &gps_bcm4751_ctl_fops);
	dev->dev.owner = THIS_MODULE;
	dev->dev.ops = &gps_bcm4751_ctl_fops;
	
	/*注册*/
	err = cdev_add(&(dev->dev), devno, 1);
	if(err){
		return err;
	}
	
	/*初始化信号量和寄存器val的值*/
	//init_MUTEX(&(dev->sem));   
    sema_init(&(dev->sem),1);/*init_MUTEX已被启用，改成sema_init*/

	dev->val = 0;
	
	return 0;
}
/*模块加载*/
static int __init gps_bcm4751_ctl_init(void){
	int err = -1;
	dev_t dev = 0;
	struct device* reset_n = NULL;
	struct device* standby = NULL;
	
	printk(KERN_ALERT"Initializing hello device.\n");
	
	/*动态分配主设备号和从设备号*/
	err = alloc_chrdev_region(&dev, 0, 1, GPS_BCM4751_CTL_DEVICE_NODE_NAME);
	if(err < 0){
		printk(KERN_ALERT"Failed to alloc char dev region.\n");
		goto fail;
	}
	
	gps_bcm4751_ctl_major = MAJOR(dev);
	gps_bcm4751_ctl_minor = MINOR(dev);
	
	/*分配hello设备结构体变量*/
	gps_bcm4751_ctl_dev = kmalloc(sizeof(struct gps_bcm4751_ctl_jelly_dev), GFP_KERNEL);
	if(!gps_bcm4751_ctl_dev){
		err = -ENOMEM;
		printk(KERN_ALERT"Failed to alloc hello dev.\n");
		goto unregister;
	}
	
	/*初始化设备*/
	err = __gps_bcm4751_ctl_setup_dev(gps_bcm4751_ctl_dev);
	if(err < 0){
        printk(KERN_ALERT"Failed to setup dev: %d.\n", err);   
        goto cleanup;
	}
	
	/*创建设备类型，他会在/sys/class/下创建此目录*/
	gps_bcm4751_ctl_class = class_create(THIS_MODULE, GPS_BCM4751_CTL_DEVICE_CLASS_NAME);
	if(IS_ERR(gps_bcm4751_ctl_class)){
		err = PTR_ERR(gps_bcm4751_ctl_class);
		printk(KERN_ALERT"Failed to create hello class.\n");
		goto destroy_cdev;
	}
	/*创建以上类别所对应的设备文件*/
	reset_n = device_create(gps_bcm4751_ctl_class, NULL, dev, "%s", GPS_BCM4751_CTL_DEVICE_RESET_N);
	if(IS_ERR(reset_n)){
		err = PTR_ERR(reset_n);
		printk(KERN_ALERT"Failed to create hello device.\n");
		goto destroy_class;
	}

	/*在/sys/class/hello/hello目录下创建属性文件enable*/   
    err = device_create_file(reset_n, &dev_attr_reset);   
    if(err < 0) {   
        printk(KERN_ALERT"Failed to create attribute val.");                   
        goto destroy_device;   
    }   
   
    err = device_create_file(temp, &dev_attr_enable);   
    if(err < 0) {   
        printk(KERN_ALERT"Failed to create attribute val.");                   
        goto destroy_device;   
    }   
   
	
	err = gpio_request(GPS_BCM4751_RESET, "BCM4751 RESET PIN");
	if (err != 0) {
		gpio_free(GPS_BCM4751_RESET);
		printk("BCM4751 RESET PIN error\n");
		goto destroy_device;
	}
	gpio_direction_output(GPS_BCM4751_RESET, 1);
	gpio_set_value(GPS_BCM4751_RESET, GPIO_LOW);
	gps_bcm4751_ctl_dev->reset = 0;
	
	err = gpio_request(GPS_BCM4751_EN, "BCM4751 RESET PIN");
	if (err != 0) {
		gpio_free(GPS_BCM4751_EN);
		printk("BCM4751 RESET PIN error\n");
		goto destroy_device;
	}
	gpio_direction_output(GPS_BCM4751_EN, 1);
	gpio_set_value(GPS_BCM4751_EN, GPIO_LOW);
	gps_bcm4751_ctl_dev->enable = 0;
		
    dev_set_drvdata(reset_n, gps_bcm4751_ctl_dev); 
    dev_set_drvdata(enable, gps_bcm4751_ctl_dev); 
		
	/*创建/proc/hello文件*/
	//gps_bcm4751_ctl_create_proc();
	
    printk(KERN_ALERT"Succedded to initialize hello device.\n");   
    return 0;	
	
destroy_device:   
    device_destroy(gps_bcm4751_ctl_class, dev);   
   
destroy_class:   
    class_destroy(gps_bcm4751_ctl_class);   
   
destroy_cdev:   
    cdev_del(&(gps_bcm4751_ctl_dev->dev));   
   
cleanup:   
    kfree(gps_bcm4751_ctl_dev);   
   
unregister:   
    unregister_chrdev_region(MKDEV(gps_bcm4751_ctl_major, gps_bcm4751_ctl_minor), 1);   
   
fail:   
    return err;   

}

static 	void __exit gps_bcm4751_ctl_exit(void){
	dev_t devno = MKDEV(gps_bcm4751_ctl_major, gps_bcm4751_ctl_minor); 
	printk(KERN_ALERT"Destroy hello device.\n");

     
   
    /*销毁设备类别和设备*/   
    if(gps_bcm4751_ctl_class) {   
        device_destroy(gps_bcm4751_ctl_class, MKDEV(gps_bcm4751_ctl_major, gps_bcm4751_ctl_minor));   
        class_destroy(gps_bcm4751_ctl_class);   
    }   
	/*删除字符设备和释放字符设备内存*/
	if(gps_bcm4751_ctl_dev){
		cdev_del(&(gps_bcm4751_ctl_dev->dev));
		kfree(gps_bcm4751_ctl_dev);
	}
	
	/*释放设备号*/
	unregister_chrdev_region(devno, 1);	
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver to control bcm4751 reset and enable pin");
module_init(gps_bcm4751_ctl_init);   
module_exit(gps_bcm4751_ctl_exit); 