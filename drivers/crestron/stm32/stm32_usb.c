/*
 * Driver for STM32 attached to Linux host device via USB
 * Copyright (C) 2016 Crestron
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/kref.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>

#include "stm32_ioctl.h"

static int stm32_probe(struct usb_interface *p_intfc, const struct usb_device_id *id);
static void stm32_disconnect(struct usb_interface *p_intfc);

#define DEFAULT_READ_TIMEOUT        1000
#define DEFAULT_WRITE_TIMEOUT       10000

static DEFINE_MUTEX(g_stm32_mutex);

struct Crestron_Data {
	struct usb_device *stm32_device;
	struct usb_class_driver class_drv;
	struct kref ref_cnt;

	char dev_id_str[64];
	char fw_str[64];

	// include timeouts on a per-instance basis
	int rx_timeout;
	int tx_timeout;

	u8 intfc_id; /* interface number */
	u8 ep_num; /* number of endpoints */
	u8 ep_in;
	u8 ep_out;
	bool btl_mode; /* bootloader mode */
	bool stm32_ready;
};

/* Table of devices that work with this driver */
static struct usb_device_id crestron_table[] = {
	{ USB_DEVICE(0x14BE, 0x09) }, /* Bootloader mode with just Cresnet endpoint */
	{ USB_DEVICE(0x14BE, 0x1A) }, /* Just Cresnet endpoint */
	{ USB_DEVICE(0x14BE, 0x1B) }, /* Cresnet (ep1) and console (ep2) endpoints */
	{} /* Terminating entry */
};

MODULE_DEVICE_TABLE (usb, crestron_table);

static struct usb_driver g_stm32_usb_driver = {
	.name = "stm32_usb_driver",
	.probe = stm32_probe,
	.disconnect = stm32_disconnect,
	.id_table = crestron_table,
};

static void release_stm32_data(struct kref *p_kref)
{
	//pr_info("%s() - Releasing stm device data\n", __FUNCTION__);
	struct Crestron_Data *p_data = container_of(p_kref, struct Crestron_Data, ref_cnt);
	kfree(p_data);
}

static struct Crestron_Data * get_stm32_data(struct device *p_dev)
{
	struct Crestron_Data *p_data;
	struct usb_interface *p_intfc = container_of(p_dev, struct usb_interface, dev);
	if (p_intfc)
	{
		mutex_lock(&g_stm32_mutex);

		p_data = usb_get_intfdata(p_intfc);
		if (p_data != NULL)
			kref_get(&p_data->ref_cnt);  // Increment the reference count since the object is going to be used

		mutex_unlock(&g_stm32_mutex);
		return p_data;
	}

	pr_err("%s() - ERROR: Could not get usb stm device data\n", __FUNCTION__);
	return NULL;
}

static void return_stm32_data(struct Crestron_Data *p_data)
{
	if (!p_data)
		return;

	// Decrease reference count so the object can be released if no longer needed
	mutex_lock(&g_stm32_mutex);
	kref_put(&p_data->ref_cnt, release_stm32_data);
	mutex_unlock(&g_stm32_mutex);
}

// sysfs interfaces to userspace

/**
 * \author		RJen
 * \brief		For retrieving information on the stm32 USB endpoint
 * \date		11/08/2016
 * \param       p_dev - device driver pointer
 * \param		dummy - not used
 * \param		p_buf - where to store the info, buf size is page size which should be 4096
 * \retval		> 0 - if there is info is saved correctly, <= 0 - error
 */
static ssize_t stm32_info_show(struct device *p_dev, struct device_attribute *dummy, char *p_buf)
{

	struct Crestron_Data *p_data = get_stm32_data(p_dev);
	char *p_start = p_buf;
	char *p_end = p_buf + PAGE_SIZE;
	int ret_code = 0;
	int print_ret = 0;

	if (!p_data)
		return -ENXIO;


	if ((print_ret = scnprintf(p_start, p_end - p_start, "interface_id:%d\nep_num:%d\n", p_data->intfc_id, p_data->ep_num)) <= 0)
		goto print_error;

	p_start += print_ret;
	if ((print_ret = scnprintf(p_start, p_end - p_start, "ep_in:%02X\nep_out:%02X\n", p_data->ep_in, p_data->ep_out)) <= 0)
		goto print_error;

	p_start += print_ret;
	if ((print_ret = scnprintf(p_start, p_end - p_start, "rx_timeout:%d\ntx_timeout:%d\n", p_data->rx_timeout, p_data->tx_timeout)) <= 0)
		goto print_error;

	p_start += print_ret;
	if (p_data->dev_id_str[0] != '\0')
	{
		if ((print_ret = scnprintf(p_start, p_end - p_start, "dev_id_str:%s\n", p_data->dev_id_str)) <= 0)
			goto print_error;

		p_start += print_ret;
	}

	if (p_data->fw_str[0] != '\0')
	{
		if ((print_ret = scnprintf(p_start, p_end - p_start, "fw_str:%s\n", p_data->fw_str)) <= 0)
			goto print_error;

		p_start += print_ret;
	}

	ret_code = p_start - p_buf;
	goto exit;

print_error:
	ret_code = -ENXIO;

exit:
	// Decrease reference count so the object can be released if no longer needed
	return_stm32_data(p_data);
	return ret_code;
}

/**
 * \author		RJen
 * \brief		For retrieving data from the stm32 USB console
 * \date		11/08/2016
 * \param       p_dev - device driver pointer
 * \param		dummy - not used
 * \param		p_buf - where to store the data, buf size is page size which should be 4096
 * \retval		> 0 - count of the data to send back, <= 0 - error
 */
static ssize_t stm32_console_show(struct device *p_dev, struct device_attribute *dummy, char *p_buf)
{
	char *p_start = p_buf;
	char *p_end = p_buf + PAGE_SIZE;
	char temp_buf[64] = {0};
	int read_cnt = 0;
	int retval = 0;
	struct Crestron_Data *p_data = get_stm32_data(p_dev);

	if (!p_buf)
	{
		pr_err("STM USB console %s() err: invalid arguments\n", __FUNCTION__);
		return -ENXIO;
	}

	if (!p_data)
		return -ENXIO;


	while ((retval = usb_bulk_msg(p_data->stm32_device, usb_rcvbulkpipe(p_data->stm32_device, p_data->ep_in),
	                              temp_buf, sizeof(temp_buf), &read_cnt, 1000)) >= 0)
	{
		if ((p_end - p_start) > read_cnt)
		{
			memcpy(p_start, temp_buf, read_cnt);
			p_start += read_cnt;
		}
		else
		{
			break;
		}
	}

	return_stm32_data(p_data); // Decrease reference count so the object can be released if no longer needed
	return p_start - p_buf;
}

/**
 * \author		RJen
 * \brief		For sending data to the stm32 USB console
 * \date		11/08/2016
 * \param       p_dev - device driver pointer
 * \param		dummy - not used
 * \param		p_buf - pointer to the data to send
 * \param		cnt - data count (should be in bytes)
 * \retval		> 0 - count of the data sent , <= 0 - error
 */
static ssize_t stm32_console_store(struct device *p_dev, struct device_attribute *dummy, const char *p_buf, size_t cnt)
{
	int retval;
	int write_cnt = 0;
	struct Crestron_Data *p_data = get_stm32_data(p_dev);
	if ((!p_buf) || (!cnt))
	{
		pr_err("STM USB console %s() err: invalid arguments\n", __FUNCTION__);
		return -ENXIO;
	}

	if (!p_data)
		return -ENXIO;

	retval = usb_bulk_msg(p_data->stm32_device, usb_sndbulkpipe(p_data->stm32_device, p_data->ep_out), (char*)p_buf, cnt, &write_cnt, 5000);
	return_stm32_data(p_data); // Decrease reference count so the object can be released if no longer needed

	if (retval < 0)
	{
		pr_err("STM USB console %s() err: %d\n", __FUNCTION__, -retval);
		return -ENXIO;
	}

	return write_cnt;
}

/**
 * \author		RJen
 * \brief		For retrieving data (in Cresnet packet format) from the stm32 IO endpoint
 * \date		11/08/2016
 * \param       p_dev - device driver pointer
 * \param		dummy - not used
 * \param		p_buf - where to store the data, buf size is page size which should be 4096
 * \retval		> 0 - count of the data to send back, <= 0 - error
 */
static ssize_t stm32_io_show(struct device *p_dev, struct device_attribute *dummy, char *p_buf)
{
	int read_cnt = 0;
	int retval;
	struct Crestron_Data *p_data = get_stm32_data(p_dev);
	if (!p_buf)
	{
		pr_err("STM USB IO %s() err: invalid arguments\n", __FUNCTION__);
		return -ENXIO;
	}

	if (!p_data)
		return -ENXIO;

	retval = usb_bulk_msg(p_data->stm32_device, usb_rcvbulkpipe(p_data->stm32_device, p_data->ep_in), p_buf, PAGE_SIZE, &read_cnt, 1000);
	return_stm32_data(p_data); // Decrease reference count so the object can be released if no longer needed
	//printk(KERN_INFO "STM32 USB read cnt %d %d \n", retval, read_cnt);

	if (retval < 0)
	{
		if (retval == -ETIMEDOUT)
			return -ETIMEDOUT;

		pr_err("STM USB IO %s() read err: %d\n", __FUNCTION__, -retval);
		return -ENXIO;
	}

	return read_cnt;
}

/**
 * \author		RJen
 * \brief		For sending data (in Cresnet packet format) to the stm32 IO endpoint
 * \date		11/08/2016
 * \param       p_dev - device driver pointer
 * \param		dummy - not used
 * \param		p_buf - pointer to the data to send
 * \param		cnt - data count (should be in bytes)
 * \retval		> 0 - count of the data sent , <= 0 - error
 */
static ssize_t stm32_io_store(struct device *p_dev, struct device_attribute *dummy, const char *p_buf, size_t cnt)
{
	int write_cnt = 0;
	int retval;
	struct Crestron_Data *p_data = get_stm32_data(p_dev);
	if ((!p_buf) || (!cnt) || (cnt > 257) || (cnt < 3))
	{
		pr_err("STM USB io %s() err: invalid arguments\n", __FUNCTION__);
		return -ENXIO;
	}

	if (p_buf[1] != cnt - 2)
	{
		pr_err("STM USB io %s() err: invalid packet %d %d\n", __FUNCTION__, p_buf[1], cnt);
		return -ENXIO;
	}

	if (!p_data)
		return -ENXIO;

	retval = usb_bulk_msg(p_data->stm32_device, usb_sndbulkpipe(p_data->stm32_device, p_data->ep_out), (char*)p_buf, cnt, &write_cnt, 5000);
	return_stm32_data(p_data); // Decrease reference count so the object can be released if no longer needed

	if (retval < 0)
	{
		pr_err("STM USB io %s() err: %d\n", __FUNCTION__, -retval);
		return -ENXIO;
	}

	return write_cnt;
}






static ssize_t stm32_ready_show(struct device *p_dev, struct device_attribute *dummy, char *p_buf)
{

	struct Crestron_Data *p_data = get_stm32_data(p_dev);
	int ret_code = 0;

	if (!p_data)
		return -ENXIO;

	ret_code = sprintf ( p_buf, "%u\n", p_data->stm32_ready );

	// Decrease reference count so the object can be released if no longer needed
	return_stm32_data(p_data);
	return ret_code;
}




static DEVICE_ATTR(stm32_info, S_IRUGO, stm32_info_show, NULL);
static DEVICE_ATTR(stm32_console, S_IRUGO | S_IWUSR, stm32_console_show, stm32_console_store);
static DEVICE_ATTR(stm32_io, S_IRUGO | S_IWUSR, stm32_io_show, stm32_io_store);
static DEVICE_ATTR(stm32_btl, S_IRUGO | S_IWUSR, stm32_io_show, stm32_io_store);
static DEVICE_ATTR(stm32_ready, S_IRUGO | S_IWUSR, stm32_ready_show, NULL);




static struct file_operations fops =
{
	.owner = THIS_MODULE,
};


/**
 * \author		RJen, AKN
 * \brief		USB interface probe function, sets up SYSFS interface based on USB interface ID
 * \date		08/29/2016
 * \param       p_intfc - pointer to USB interface data
 * \param		id - not used
 * \retval		< 0 - if initialization is not sucessful
 */
static int stm32_probe(struct usb_interface *p_intfc, const struct usb_device_id *id)
{
	/* Store our endpoints in an object */
	int retval;
	int err = 0;
	char sysfs_name[20] = {0};
	int driver_id = 0;
	struct usb_device *stm32_device;
	struct Crestron_Data *p_dev_data = kzalloc(sizeof(struct Crestron_Data), GFP_KERNEL);
	struct usb_host_interface *p_intfc_desc;

	if (p_dev_data == NULL)
	{
		printk(KERN_ERR "STM32 USB driver %s: not able to allocate memory for this device\n", __func__);
		return -ENOMEM;
	}

	memset(p_dev_data, 0, sizeof(struct Crestron_Data));
	kref_init(&p_dev_data->ref_cnt);

	// Save the interface info
	if (p_intfc && p_intfc->cur_altsetting)
	{
		p_intfc_desc = p_intfc->cur_altsetting;
		p_dev_data->intfc_id = p_intfc_desc->desc.bInterfaceNumber;
		p_dev_data->ep_num = p_intfc_desc->desc.bNumEndpoints;

		// Check the endpoint descriptor pointer first
		if (p_intfc_desc->endpoint)
		{
			u8 ep = 0;
			for (ep = 0; ep < p_dev_data->ep_num; ep++)
			{
				struct usb_endpoint_descriptor *p_ep_desc = &p_intfc_desc->endpoint[ep].desc;
				// usb_endpoint_is_bulk_in(p_ep_desc)
				if (p_ep_desc)
				{
					if (p_ep_desc->bEndpointAddress & USB_DIR_IN)
						p_dev_data->ep_in = p_ep_desc->bEndpointAddress;
					else
						p_dev_data->ep_out = p_ep_desc->bEndpointAddress;
				}
			}
		} // if (p_intfc_desc->endpoint)
	} // if (p_intfc && p_intfc->cur_altsetting)

	p_dev_data->rx_timeout = DEFAULT_READ_TIMEOUT;
	p_dev_data->tx_timeout = DEFAULT_WRITE_TIMEOUT;

	stm32_device = p_dev_data->stm32_device = interface_to_usbdev(p_intfc);

	printk("STM USB driver init for devnum %d devpath %s portnum %d\n", stm32_device->devnum, stm32_device->devpath, stm32_device->portnum);

	/* Get the string descriptors that contain the dev ID and firmware name string */
	if (usb_string(stm32_device, 3, p_dev_data->dev_id_str, sizeof(p_dev_data->dev_id_str)) <= 0)
		p_dev_data->dev_id_str[0] = '\0';  /* Null term if we can't get a string */
	else
		err = kstrtoint(p_dev_data->dev_id_str, 10, &driver_id);

	if (usb_string(stm32_device, 4, p_dev_data->fw_str, sizeof(p_dev_data->fw_str)) <= 0)
	{
		p_dev_data->fw_str[0] = '\0';  /* Null term if we can't get a string */
	}
	else
	{
		if (strstr(p_dev_data->fw_str, "BOOTLOADER") != NULL)
			p_dev_data->btl_mode = true;
	}

	p_dev_data->class_drv.devnode = NULL;
	p_dev_data->class_drv.fops = &fops;
	p_dev_data->class_drv.minor_base = 0;
	usb_set_intfdata(p_intfc, p_dev_data);

	p_dev_data->class_drv.name = sysfs_name;

	switch (p_dev_data->intfc_id)
	{
	case 0:
		if (!p_dev_data->btl_mode)
		{
			// STM32 USB console is enabled only for interface 0
			scnprintf(sysfs_name, sizeof(sysfs_name), "stm32_console_%d", driver_id);
			err = device_create_file(&p_intfc->dev, &dev_attr_stm32_console);
			if (err < 0)
				pr_err("STM USB driver %s() failed for console: err %d\n", __FUNCTION__, err);
		}
		else
		{
			// STM32 USB bootloader is enabled only for interface 0
			scnprintf(sysfs_name, sizeof(sysfs_name), "stm32_bootloader_%d", driver_id);
			err = device_create_file(&p_intfc->dev, &dev_attr_stm32_btl);
			if (err < 0)
				pr_err("STM USB driver %s() failed for bootloader: err %d\n", __FUNCTION__, err);
		}

		break;

	// STM32 USB io is enabled only for interface 1
	case 1:
		scnprintf(sysfs_name, sizeof(sysfs_name), "stm32_io_%d", driver_id);
		err = device_create_file(&p_intfc->dev, &dev_attr_stm32_io);
		if (err < 0)
			pr_err("STM USB driver %s() failed for io: err %d\n", __FUNCTION__, err);

		break;

	default:
		pr_info("Invalid USB interface ID found in %s()", __FUNCTION__);
		break;
	}

	p_dev_data->stm32_ready=1;

	if ((err = device_create_file(&p_intfc->dev, &dev_attr_stm32_info)) < 0)
		pr_err("STM USB driver %s() failed for dev info: err %d\n", __FUNCTION__, err);

	if ((err = device_create_file(&p_intfc->dev, &dev_attr_stm32_ready)) < 0)
		pr_err("STM USB driver %s() failed for dev ready: err %d\n", __FUNCTION__, err);


	retval = usb_register_dev(p_intfc, &p_dev_data->class_drv);
	if (retval < 0)
		pr_err("Not able to usb_register_dev for this device in %s(): err %d", __FUNCTION__, retval);
	else
		printk(KERN_INFO "STM32 USB Driver is bound to minor num: %d\n", p_intfc->minor);

	return retval;
}

/**
 * \author		RJen, AKN
 * \brief		Handles USB device disconnect, removes SYSFS interface based on USB interface ID
 * \date		08/29/2016
 * \param       p_intfc - pointer to USB interface data
 * \retval		none
 */
static void stm32_disconnect(struct usb_interface *p_intfc)
{
	struct Crestron_Data * p_data = usb_get_intfdata(p_intfc);

	if (!p_intfc)
		return;

	// kref_get() must be locked
	mutex_lock(&g_stm32_mutex);

	// Null out the pointer to the stm data
	usb_set_intfdata(p_intfc, NULL);
	mutex_unlock(&g_stm32_mutex);

	p_data->stm32_ready=0;
	// Remove sysfs interface and de register the device driver
	usb_deregister_dev(p_intfc, &p_data->class_drv);
	device_remove_file(&p_intfc->dev, &dev_attr_stm32_info);
	device_remove_file(&p_intfc->dev, &dev_attr_stm32_ready);

	switch (p_data->intfc_id)
	{
	case 0:
		if (!p_data->btl_mode)
			device_remove_file(&p_intfc->dev, &dev_attr_stm32_console);
		else
			device_remove_file(&p_intfc->dev, &dev_attr_stm32_btl);

		break;

	case 1:
		device_remove_file(&p_intfc->dev, &dev_attr_stm32_io);
		break;

	default:
		break;
	}

	// kref_put() must be locked
	mutex_lock(&g_stm32_mutex);
	kref_put(&p_data->ref_cnt, release_stm32_data);
	mutex_unlock(&g_stm32_mutex);

	printk("STM32 USB disconnected \n");
}

/**
 * \author		RJen, AKN
 * \brief		Kernel module init
 * \date		08/29/2016
 * \param       none
 * \retval		none
 */
static int __init stm32_init(void)
{
	int result;

	printk(KERN_DEBUG "STM32 USB driver is initializing\n");

	/* Register this driver with the USB subsystem */
	if ((result = usb_register(&g_stm32_usb_driver)))
		printk(KERN_ERR "usb_register failed. Error number %d", result);

	return result;
}

/**
 * \author		RJen, AKN
 * \brief		Kernel module exit
 * \date		08/29/2016
 * \param       none
 * \retval		none
 */
static void __exit stm32_exit(void)
{
	/* Deregister this driver with the USB subsystem */
	printk(KERN_DEBUG "STM32 USB driver is exiting\n");
	usb_deregister(&g_stm32_usb_driver);
}

module_init(stm32_init);
module_exit(stm32_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("R Jen");
MODULE_DESCRIPTION("Internal STM32 USB Driver");
