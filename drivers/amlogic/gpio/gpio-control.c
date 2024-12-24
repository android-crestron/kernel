#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/device.h>

#define MY_MAGIC 'G'
#define GET_POE_STATUS_IOCTL _IOR(MY_MAGIC, 0, int *)
#define SET_USB_POWER_IOCTL _IOW(MY_MAGIC, 1, int)


struct gpio_control
{	
	int config_major;
	char config_name[20];
	struct class *config_class;
	struct device *config_dev;
};

struct gpio_control gpio_dev;

#if 0
static char msg[200];
char buf[200];

static ssize_t device_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset)
{
	return simple_read_from_buffer(buffer, length, offset, msg, 200);
}

static ssize_t device_write(struct file *filp, const char __user *buff, size_t length, loff_t *offset)
{
	int err;
	
	if (len > 199)
		return -EINVAL;

	err = copy_from_user(msg, buff, length);
	msg[length] = '\0';
	
	return length;
}
#endif

static long device_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int poe_pin;
	int usb_pin;
	
	switch(cmd) {
	case GET_POE_STATUS_IOCTL:
		poe_pin = amlogic_gpio_name_map_num("GPIODV_16");
		amlogic_gpio_request(poe_pin, "GPIO_CONTROL");
		*(int *)arg = amlogic_get_value(poe_pin, "GPIO_CONTROL");
		printk("Get %d pin for POE status %d\n", poe_pin, *(int *)arg);
		break;
	
	case SET_USB_POWER_IOCTL:
		usb_pin = amlogic_gpio_name_map_num("GPIOX_18");
		printk("Set %d pin to %ld\n", usb_pin, arg);
		amlogic_gpio_request(usb_pin, "DWC_OTG");
		amlogic_set_value(usb_pin, (int)arg, "DWC_OTG");
		break;

	default:
		return -ENOTTY;
	}
	
	return 0;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
//	.read = device_read, 
//	.write = device_write,
	.unlocked_ioctl = device_ioctl,
};

static int __init cdevexample_module_init(void)
{
	int ret = 0;
	
	strcpy(gpio_dev.config_name, "gpio_control");
	
    gpio_dev.config_major = register_chrdev(0, gpio_dev.config_name, &fops);
    if(gpio_dev.config_major <= 0)
    {
        printk("register char device error\n");
        return -1 ;
    }

    printk("gpio control major:%d\n", gpio_dev.config_major);
    gpio_dev.config_class = class_create(THIS_MODULE, gpio_dev.config_name);
    gpio_dev.config_dev = device_create(gpio_dev.config_class, NULL, MKDEV(gpio_dev.config_major,0), NULL, gpio_dev.config_name);
    
    return ret;
}

static void __exit cdevexample_module_exit(void)
{
	unregister_chrdev(gpio_dev.config_major, gpio_dev.config_name);
	if(gpio_dev.config_class)
    {
        if(gpio_dev.config_dev)
			device_destroy(gpio_dev.config_class, MKDEV(gpio_dev.config_major,0));
        class_destroy(gpio_dev.config_class);
    }
}  

module_init(cdevexample_module_init);
module_exit(cdevexample_module_exit);
MODULE_LICENSE("GPL");
