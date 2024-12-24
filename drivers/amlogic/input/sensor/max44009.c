/**
   Copyright (C) 2011 Maxim Integrated Products
 
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation.
  
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
  
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/sensor/max44009.h>
#include <linux/module.h>

static int max44009_probe(struct i2c_client *client,
                          const struct i2c_device_id *id );
//static int max44009_read_ALS(struct MAX44009Data *alsDevice,int *adjusted_lux);
static int max44009_remove(struct i2c_client *client);
static int max44009_initialize_device(struct MAX44009Data *device,bool use_defaults);
static irqreturn_t max44009_irq_handler(int irq, void *device);
static void max44009_work_queue(struct work_struct *work);
struct MAX44009Data *max_data;

#undef DEBUGD
/*#define DEBUGD*/
#ifdef DEBUGD
	#define dprintk(x...) printk(x)
#else
	#define dprintk(x...) 
#endif

/**
 *  Performs a write of one or more bytes to a target MAX44009 device. 
 *
 *  Will attempt to write to the device several times, pausing slightly 
 *  between each writing attempt. 
 *  Returns 0 upon success or a negative error code in the case of a error
 *  Please consult the MAX44009 datasheet for further information on 
 *  communicating with the MAX44009 over I2C.
 *
 *  Recommended usage:
 *    Put the desired register address in buffer[0], then the data in buffer[1]
 *
 *  Author's note:
 *    The MAX44009 does **not** auto-incremement the register pointer when 
 *    doing multiple reads or writes. There should not be an issue so long as 
 *    you write only one register at a time, using the frame formatting below.
 *  
 *    Frame format for single write:
 *    [S] [Slave Addr + 0] [ACK] [Register] [ACK] [Data] [ACK] [P]
 *                                  ^^buffer[0]     ^^buffer[1]
 *
 *	@param  *device 
 *	  a descriptor representing the MAX44009 as an object
 *  @param  *buffer 
 *	  points to data to transfer
 *  @param  length 
 *	  number of bytes to transfer (capped at ~64k)
 *	@return 
 *	  An error code (0 on success)
 */
int max44009_write_reg(struct MAX44009Data *device,u8 *buffer,int length) 
{
  int err;		// error code
  int tries = 0;	// retries


  dprintk(KERN_INFO "max44009_write_reg: addr=0x%02x val=0x%02x\n",
		buffer[0], buffer[1]);  

  do 
  {
	err = i2c_master_send(device->client, buffer, length);
	if (err != 2)
	  msleep_interruptible(MAX44009_RETRY_DELAY);
  } while ((err != 2) && (++tries < MAX44009_MAX_RETRIES));

  if (err != 2) 
  {
	printk("%s:write transfer error\n", __func__);
	err = -EIO;
  } 
  else 
	err = 0;
#ifdef ORIGINAL_WRITE_REG
  struct i2c_msg msgs[] = {
	{
	  .addr = device->client->addr,
	  .flags = device->client->flags,
	  .len = length + 1,
	  .buf = buffer,
	},
  };

  dprintk(KERN_INFO "max44009_write_reg: 0x%02x 0x%02x\n",
		buffer[0], buffer[1]);

  // transfer a single I2C message of length len
  do 
  {
	err = i2c_transfer(device->client->adapter, msgs, 1);
	if (err != 1)
	  msleep_interruptible(MAX44009_RETRY_DELAY);
  } while ((err != 1) && (++tries < MAX44009_MAX_RETRIES));

  if (err != 1) 
  {
	printk("%s:write transfer error\n", __func__);
	err = -EIO;
  } 
  else 
	err = 0;
#endif

  return err;
} // max44009_write_reg                              

/**
 *  Performs a read of one or more bytes to a target MAX44009 device. 
 *  Will attempt to read from the device several times, pausing slightly 
 *  between each attempt. 
 *  Returns 0 upon success or a negative error code in the case of a error
 *  Please consult the MAX44009 datasheet for further information on 
 *  communicating with the MAX44009 over I2C.
 *
 *  Recommended usage:
 *    buffer[0] contains the register addr to read when the function is called
 *    and is then overwritten with the data in that register. same goes for
 *    buffer[1]...buffer[n], so long as the frame format discussed below is used
 *    For example,
 *      Before:     buffer[] has {0x01,0x02}
 *      After:      buffer[] has {0x24,0x30}
 *
 *  Author's note:
 *    The MAX44009 does **not** auto-incremement the register pointer when 
 *    doing multiple reads or writes. To do consecutive reads/writes, you must
 *    follow the frame format described on pages 14-16 of the datasheet.
 *    To accomplish this, you should ensure that the implementation of the I2C
 *    protocol that your codebase uses supports this format.
 *  
 *    Frame format for single read:
 *    [S] [Slave Addr + 0] [ACK] [Register] [ACK] [Data] [ACK] [P]
 *  
 *    Frame format for two byte read:
 *    [S] [Slave Addr + 0] [ACK] [Register 1] [ACK] [Sr] [Slave Addr + 1]
 *    [ACK] [Data 1] [NACK] [Sr] [S] [Slave Addr + 0] [ACK] [Register 2] [ACK]
 *    [Sr] [Slave Addr + 1] [ACK] [Data 2] [NACK] [P]
 *
 *  @param  *device 
 *    a descriptor representing the MAX44009 as an object
 *  @param  *buffer 
 *    points to data to transfer
 *  @param  length 
 *    number of bytes to transfer (capped at ~64k)
 *  @return 
 *    An error code (0 on success)
 */
int max44009_read_reg(struct MAX44009Data *device,u8 *buffer,int length) 
{
  int err;	    // error code
  int tries = 0;    // retries
#ifdef DEBUGD
  u8 addr = *buffer;
#endif
  struct i2c_msg msgs[] = 
  {
	{
	  .addr = device->client->addr,
	  .flags = device->client->flags,
	  .len = 1,
	  .buf = buffer,
	}, // first message is the register address
	{
	  .addr = device->client->addr,
	  .flags = device->client->flags | I2C_M_RD,
	  .len = length,
	  .buf = buffer,
	}, // rest of the message
  };

  // perform the i2c transfer of two messages, one of length 1 and the 
  // other of length 'len' (i.e. first send the register to be read, then 
  // get all the data
  do 
  {
	err = i2c_transfer(device->client->adapter, msgs, 2);
	if (err != 2)
	  msleep_interruptible(MAX44009_RETRY_DELAY);
  } while ((err != 2) && (++tries < MAX44009_MAX_RETRIES));

  if (err != 2) 
  {
	printk("%s:read transfer error\n", __func__);
	err = -EIO;
  }  
  else {
	err = 0;
  }

  dprintk(KERN_INFO "max44009_read_reg addr=0x%02x val=0x%02x\n",
		addr,
		buffer[0]);

  // return error code (0 if no error)
  return err;

} // MAX44009ReadReg

/**

 *  Performs a read of one or more bytes to a target MAX44009 device. 
 *  Will attempt to read from the device several times, pausing slightly 
 *  between each attempt. 
 *  Returns 0 upon success or a negative error code in the case of a error
 *  Please consult the MAX44009 datasheet for further information on 
 *  communicating with the MAX44009 over I2C.
 *
 *  Recommended usage:
 *    buffer[0] contains the register addr to read when the function is called
 *    and is then overwritten with the data in that register. same goes for
 *    buffer[1]...buffer[n], so long as the frame format discussed below is used
 *    For example,
 *      Before:     buffer[] has {0x01,0x02}
 *      After:      buffer[] has {0x24,0x30}
 *
 *  Author's note:
 *    The MAX44009 does **not** auto-incremement the register pointer when 
 *    doing multiple reads or writes. To do consecutive reads/writes, you must
 *    follow the frame format described on pages 14-16 of the datasheet.
 *    To accomplish this, you should ensure that the implementation of the I2C
 *    protocol that your codebase uses supports this format.

 *  
 *    Frame format for single read:
 *    [S] [Slave Addr + 0] [ACK] [Register] [ACK] [Data] [ACK] [P]
 *  
 *    Frame format for two byte read:
 *    [S] [Slave Addr + 0] [ACK] [Register 1] [ACK] [Sr] [Slave Addr + 1]
 *    [ACK] [Data 1] [NACK] [Sr] [S] [Slave Addr + 0] [ACK] [Register 2] [ACK]
 *    [Sr] [Slave Addr + 1] [ACK] [Data 2] [NACK] [P]
 *
 *  @param *device 
 *    a descriptor representing the MAX44009 as an object
 *  @param *buffer 
 *    points to data to transfer
 *  @param length 
 *    number of bytes to transfer (capped at ~64k)
 *  @return 
 *    An error code (0 on success)
 */
int max44009_read_reg2(struct MAX44009Data *device,u8 *buffer,int length) 
{
  int err;	    // error code
  int tries = 0;    // retries
#ifdef DEBUGD
  u8 addr = *buffer;
#endif
  struct i2c_msg msgs[] = 
  {
	{
	  .addr = device->client->addr,
	  .flags = device->client->flags,
	  .len = 1,
	  .buf = &(buffer[0]),
	}, // first message is the lux high
	{
	  .addr = device->client->addr,
	  .flags = device->client->flags | I2C_M_RD,
	  .len = length,
	  .buf = &(buffer[0]),
	},
	{
	  .addr = device->client->addr,
	  .flags = device->client->flags,
	  .len = 1,
	  .buf = &(buffer[1]),
	}, // second message is the lux low
	{
	  .addr = device->client->addr,
	  .flags = device->client->flags | I2C_M_RD,
	  .len = length,
	  .buf = &(buffer[1]),
	}, // rest of the message
  };

  // perform the i2c transfer of two messages, one of length 1 and the 
  // other of length 'len' (i.e. first send the register to be read, then 
  // get all the data
  do 
  {
	err = i2c_transfer(device->client->adapter, msgs, 4);
	if (err != 4)
	  msleep_interruptible(MAX44009_RETRY_DELAY);
  } while ((err != 4) && (++tries < MAX44009_MAX_RETRIES));

  if (err != 4) 
  {
	printk("%s:read transfer error\n", __func__);
	err = -EIO;
  }  
  else {
	err = 0;
  }

  dprintk(KERN_INFO "max44009_read_reg2 addr=0x%02x buffer[0]=0x%02x "
		"buffer[1]=0x%02x\n",
		addr,
		buffer[0],
		buffer[1]);

  // return error code (0 if no error)
  return err;

} // MAX44009ReadReg2

/**
 *	Either sets or clears the state of the interrupt enable bit.
 *	
 *	@param *device 
 *	  Points to some data describing an instance of a MAX44009 device.
 *	  The state of the interrupt is updated in here
 *	@param enable
 *	  Desired state of the interrupt (true = enabled)
 *
 *	@return
 *	  0 on success, an error code otherwise
 */
int max44009_set_interrupt(struct MAX44009Data *device,bool enable)
{
  u8 configStatus[] = {MAX44009_INT_ENABLE_ADDR,0};
  bool currently_enabled;    // is the device already enabled?

  currently_enabled = (device->interrupt_en) ? true:false;

  dprintk(KERN_INFO "max44009_set_interrupt current=0x%02x new=0x%02x\n",
		currently_enabled,
		enable);
  
  // if the current status of the device and the desired status are the same,
  // do nothing
  if (currently_enabled != enable)
  {
	if(enable)
	  configStatus[1] |= 1;  // set lowest bit
	if(max44009_write_reg(device,configStatus,2))
	{
	  printk("%s: couldn't update interrupt status\n",__func__);
	  return -EINVAL;
	}
	// if write was successful, update the value in the data structure
	device->interrupt_en = enable ? 1:0;

	if (device->interrupt_en == 0)
	  schedule_delayed_work(&device->work_queue, 50);

	if (device->interrupt_en == 1)
	  cancel_delayed_work(&device->work_queue);
  }
  
  return 0;
}

/**
 *   Reads the interrupt status register to clear it and return the value of 
 *	the interrupt status bit
 *
 *	@param *device
 *	  Points to some data describing an instance of a MAX44009 device.
 *	  The state of the interrupt is updated in here
 *	@return
 *	  Interrupt status
 */
int max44009_read_int_status(struct MAX44009Data *device)
{
  u8 buf = MAX44009_INT_STATUS_ADDR;

  dprintk(KERN_INFO "max44009_read_int_status\n");
  
  if (max44009_read_reg(device,&buf,1) != 0) 
  {
	printk("%s: couldn't read interrupt status\n",__func__);
	return -EIO;
  }
  
  return ((buf & 1) == 0) ? 1:0;
} // max44009ReadIntStatus

/**
 *	Updates the integration time settings in register 0x02
 *
 *	@param struct MAX44009Data *device
 *	  Points to some data describing an instance of a MAX44009 device.
 *	  The state of the configuration register is updated in here
 *
 *	@param time
 *	  New integration time settings (see datasheet for details)
 *
 *	@return 
 *	  Error code, 0 on success
 */
int max44009_set_integration_time(struct MAX44009Data *device,u8 time)
{
	u8 config_reg[] = {MAX44009_CONFIG_ADDR,device->config};

	dprintk(KERN_INFO "max44009_set_integration_time\n");

	if (!(device->config&0x40))
	{
		printk("%s: not in manual mode\n",__func__);
		return 0;
	}
	
	config_reg[1]&=0xF8;		// clear lowest three bits
	config_reg[1]|=(time&0x7);	// set lowest three bits to contents of time

	if(max44009_write_reg(device,config_reg,2))
	{
		printk("%s: couldn't update interrupt status\n",__func__);
		return -EINVAL;
	}
	
	// update the stored settings on success and move along...
	device->config = config_reg[1];
	return 0;
}

/**
 *	Either enables or disables manual mode on the MAX44009
 *
 *	@param *device
 *	  Points to some data describing an instance of a MAX44009 device.
 *	  The state of the configuration register is updated in here
 *
 *	@param enable
 *	  whether to enable or disable manual mode
 *
 *	@return 
 *	  Error code, 0 on success
 */
int max44009_set_manual_mode(struct MAX44009Data *device,bool enable)
{
	u8 configStatus[] = {MAX44009_CONFIG_ADDR,0};
	bool currently_manual;    // is the device already in manual mode?

	dprintk(KERN_INFO "max44009_set_manual_mode\n");

	currently_manual = (device->config&0x40) ? true:false;
  
	// if the current status of the device and the desired status are the same
	// do nothing
	if (currently_manual != enable)
	{
		if (enable)
			configStatus[1] = (device->config)|0x40;
		else
			configStatus[1] = (device->config)&0xBF;
		if(max44009_write_reg(device,configStatus,2))
		{
			printk("%s: couldn't update auto/manual mode status\n",__func__);
			return -EINVAL;
		}
		// if write was successful, update the value in the data structure
		device->config = configStatus[1];
	}
  
	return 0;
}

/**
 *	Sets or clear the continuous mode bit
 *
 *	@param *device
 *	  Points to some data describing an instance of a MAX44009 device.
 *	  The state of the configuration register is updated in here
 *
 *	@param enable
 *	  whether to enable or disable the current divisor
 *
 *	@return 
 *	  Error code, 0 on success
 */
int max44009_set_continuous_mode(struct MAX44009Data *device,bool enable)
{
	u8 configStatus[] = {MAX44009_CONFIG_ADDR,0};
	bool currently_cont;    // is the device already in continuous mode?

	dprintk(KERN_INFO "max44009_set_continuous_mode\n");

	currently_cont = (device->config&0x80) ? true:false;
  
	// if the current status of the device and the desired status are the same
	// do nothing
	if (currently_cont != enable)
	{
		if (enable)
			configStatus[1] = (device->config)|0x80;
		else
			configStatus[1] = (device->config)&0x7F;
		if(max44009_write_reg(device,configStatus,2))
		{
			printk("%s: couldn't update continuous mode setting\n",__func__);
			return -EINVAL;
		}
		// if write was successful, update the value in the data structure
		device->config = configStatus[1];
	}
  
	return 0;
}

/**
 *	Sets or clears the current division ratio bit, if necessary
 *
 *	@param *device
 *	  Points to some data describing an instance of a MAX44009 device.
 *	  The state of the configuration register is updated in here
 *
 *	@param enable
 *	  whether to enable or disable the current divisor
 *
 *	@return 
 *	  Error code, 0 on success
 */
int max44009_set_current_div_ratio(struct MAX44009Data *device,bool enable)
{
	u8 configStatus[] = {MAX44009_CONFIG_ADDR,0};
	bool status;    // is the device already in CDR=1 mode?

	dprintk(KERN_INFO "max44009_set_current_div_ratio\n");

	// don't bother if not in manual mode
	if ( !(device->config&0x40) )
	{
        	printk("%s: not in manual mode\n",__func__);
        	return 0;
	}
	status = (device->config&0x08) ? true:false;
	
  
	// if the current status of the device and the desired status are the same
	// do nothing
	if (status != enable)
	{
		if (enable)
			configStatus[1] = (device->config)|0x08;
		else
			configStatus[1] = (device->config)&0xF7;
		if(max44009_write_reg(device,configStatus,2))
		{
			printk("%s: couldn't update current division ratio status\n",__func__);
			return -EINVAL;
		}
		// if write was successful, update the value in the data structure
		device->config = configStatus[1];
	}
  
	return 0;
}


/**
 *	Gets the lux reading from the light sensor
 *
 *  Does a consecutive two-byte read on the ADC register, then checks for
 *  the setting being in overflow. If it is, the counts are automatically set
 *  to their maximum possible value.
 *	Otherwise, the counts are converted to lux by the formula:
 *		Lux = 2^(exponent) * mantissa / 40	
 *
 *	** Author's note **
 *	  This function can easily be modified to return the counts instead of the lux
 *	  by removing the division by 40 
 *
 *  @param *device 
 *	  describes a particular ALS object.
 *  @param *adjusted_lux 
 *    points to where the result will be stored
 *
 *  @return
 *    0 on success
 *    -EIO if a communication error, -EINVAL if overflow
 */
static int max44009_read_ALS(struct MAX44009Data *device, int *adjusted_lux)
{
  u8 data_buffer[] = {MAX44009_LUX_HIGH_ADDR,MAX44009_LUX_LOW_ADDR};
  u8 exponent;
  u8 mantissa;

  dprintk(KERN_INFO "max44009_read_ALS\n");
  
  // do a consecutive read of the ADC register (0x03 and 0x04)
  // throw an error if needed
  if (max44009_read_reg2(device,data_buffer,1) != 0) 
  {
	printk("%s: couldn't read als data\n",__func__);
	return -EIO;
  } // if there was a read error
  
  /**
   *  Author's note:
   *    data_buffer should now contain the exponent and mantissa of the result
   *	per the datasheet's description of the lux readings
   */
  // update the most recent reading inside the object pointer
  device->lux_high = data_buffer[0];
  device->lux_low = data_buffer[1];
  
  // calculate the lux value
  exponent = (data_buffer[0]>>4)&0x0F;
  if (exponent==0x0F)
  {
	  printk("%s: overload on light sensor!\n",__func__);
	  *adjusted_lux = 188006;	// maximum reading (per datasheet)
	  return -EINVAL;
  }
  mantissa = (data_buffer[0]&0x0F)<<4 | (data_buffer[1]&0x0F);
  *adjusted_lux = ((int)(1 << exponent)) * mantissa * 45 / 1000; 
  dprintk(KERN_INFO "calculation_lux=%d\n", *adjusted_lux);
  
  // add any other adjustments you want to make here (e.g. correct for a lens,
  // glass, other filtering, etc)
  
  return 0;
} // read data from ALS

/**
 *  Updates threshold timer register inside physical device to a new value
 *  and also updates the value in the object descriptor passed to it if
 *  successful
 *
 *  @param *device 
 *    describes a particular ALS object
 *  @param new_tim_val
 *    desired setting of the integration time (must be in [0 3])
 *
 *  @return
 *    -EIO if there is a write error, 0 otherwise
 */
int max44009_set_thresh_tim(struct MAX44009Data *device,u8 new_tim_val)
{
  u8 buf[] = {MAX44009_THRESH_TIM_ADDR,new_tim_val};
  
  dprintk(KERN_INFO "max44009_set_thresh_tim\n");

  if(max44009_write_reg(device,buf,2)) 
	return -EIO;
 
  // update stored value
  device->thresh_tim = new_tim_val;  
  
  return 0;
} // max44009_set_thresh_tim

/**
 *  Gets the ambient light level from the MAX44009 device by calling 
 *  max44009_read_ALS(), and sets the reading as an input event on the 
 *  associated input device if the reading is valid (i.e. not an error)
 *  Optionally, this will also read the interrupt flag register to clear
 *  the interrupt flag.
 *
 *  @param *device 
 *	  describes a particular ALS object
 *  @param clear_int_flag 
 *	  should we clear the interrupt flag?
 *    set to true if using the interrupt functionality
 *
 *  Returns:
 *    0 upon success, otherwise an error flag
 */
int max44009_report_light_level(struct MAX44009Data *device,bool clear_int_flag)
{
  int result = 0;
  int err;
  int lux;		// calculated lux value

  dprintk(KERN_INFO "max44009_report_light_level\n");

  err = max44009_read_ALS(device, &lux);
  // lux goes into the input device as an input event, where the system will do
  // something with it
  if (err == 0) // no error occurred
  {
	// FIXME: Report event to upper layer
//	input_event(device->idev, EV_MSC, MSC_RAW, lux);
//	input_sync(device->idev);
	input_report_abs(device->idev, ABS_MISC, lux);
	input_sync(device->idev);
	dprintk(KERN_INFO "max44009_report_light_level to upper layer\n");
  }
  else 
  {
	result = -EIO;
	printk("%s: problem getting lux reading from MAX44009\n",__func__);
  }
  
  // clear the input status register, if desired
  // this is skipped if there was a problem communicating with the device
  if (clear_int_flag && result==0)
  {
	result = max44009_read_int_status(device);
	if (result!=0)
	{
	  printk("%s: couldn't read MAX44009 interrupt register, %d\n",
			  __func__,result);
	  return result;
	}
	max44009_enable_IRQ(device,true);
  } 
  
  dprintk("\n");
  
  return result;
} // max44009_report_light_level
/**
 *  Helps handle the work queue for Linux platform
 *
 *  @param *work 
 *    pointer to a working queue
 */
static void max44009_work_queue(struct work_struct *work)
{
  struct MAX44009Data *data = 
	container_of((struct delayed_work *)work, 
			  struct MAX44009Data, work_queue);

  dprintk(KERN_INFO "max44009_work_queue\n");

  if (data->interrupt_en == 0)
  {
    schedule_delayed_work(&data->work_queue, 50); // read lux value per 0.5 second
    max44009_report_light_level(data,false);
  }
  else
    max44009_report_light_level(data,true);

  
} // max44009_work_queue

/**
 *  Either enables or disables IRQ handling for the MAX44009 device passed to 
 *  it.
 *
 *  @param struct MAX44009Data *device 
 *	  describes a particular ALS object
 *  @param bool enable 
 *    true if we want to enable IRQ, false otherwise
 */
void max44009_enable_IRQ(struct MAX44009Data *device,bool enable)
{
  unsigned long flags;

  dprintk(KERN_INFO "max44009_enable_IRQ, enable=%d\n", enable);
  
  spin_lock_irqsave(&device->irqLock, flags);
  if (device->irqState != enable)
  {
	if (enable)
	  enable_irq(device->client->irq);
	else
	  disable_irq_nosync(device->client->irq);
  }
  spin_unlock_irqrestore(&device->irqLock, flags);
  
} //max44009_enable_IRQ

/**
 *  IRQ handler for this device
 *
 *  @param irq 
 *    irq number
 *  @param *device 
 *    points to the device for which the IRQ is handled 
 *
 *  @return
 *    A code indicating that the function handled the IRQ
 */
static irqreturn_t max44009_irq_handler(int irq, void *device)
{
  struct MAX44009Data *als_device = device;

  dprintk(KERN_INFO "max44009_irq_handler\n");

  // do something
  max44009_enable_IRQ(als_device, false);
  schedule_delayed_work(&als_device->work_queue, 0);

  return IRQ_HANDLED;
} // max44009IRQHandler

/**
	Goes through the retrim procedure for this part, whereby the following
	occurs:
	
	1. 	Values of registers 0x9 through 0xC are read and stored (note:
		these values must be complemented after reading for MAX44009)
	2.	The lowest bit of register 0xD is set
	3.	The original values of registers 0x9 and 0xA are written back (note:
		these values should never be changed from their original, as it will
		cause read errors)
	4. 	Multiply green gain by green_scale_val (see below) and write it to
		the green trim register
	5. 	Do the same for the IR gain with ir_scale_val (see below)
	
	@param *device
		device information, including some register values
	@param green_scale_val
		100 times the determined green scaling value for this glass application
		(original is sometihng like 1.23). the idea is one would pass 123 here
		and it would then be divided by 100 to get the proper scaling to 
		compensate for lack of floating point in the kernel
	@param ir_scale_val
		same idea as green_scale_val, but for the ir channel
	@return
		error if an I2C error occurs, else 0
 */
int max44009_scale_gains(struct MAX44009Data *device,u32 green_scale_val,
													 u32 ir_scale_val)
{
	u8 buf = MAX44009_INT_STATUS_ADDR;
	u8 write_buf[2];
	
	dprintk(KERN_INFO "max44009_scale_gains\n");
	
	buf = MAX44009_CLOCK_TRIM_COARSE;
	if (max44009_read_reg(device,&buf,1) != 0)
	{
		printk("%s: couldn't read coarse clock value\n",__func__);
		return -EIO;
	}
	else
	{
		device->clock_coarse = ~buf;
	}
	
	buf = MAX44009_CLOCK_TRIM_FINE;
	if (max44009_read_reg(device,&buf,1) != 0)
	{
		printk("%s: couldn't read fine clock value\n",__func__);
		return -EIO;
	}
	else
	{
		device->clock_fine = ~buf;
	}

	buf = MAX44009_GREEN_TRIM_ADDR;
	if (max44009_read_reg(device,&buf,1) != 0)
	{
		printk("%s: couldn't read green trim value\n",__func__);
		return -EIO;
	}
	else
	{
		device->green_trim = ~buf;
	}

	buf = MAX44009_IR_TRIM_ADDR;
	if (max44009_read_reg(device,&buf,1) != 0)
	{
		printk("%s: couldn't read ir trim value\n",__func__);
		return -EIO;
	}
	else
	{
		device->ir_trim = ~buf;
	}

	// now that we have all the values, set custom trim mode
	write_buf[0] = MAX44009_OTP_REG_ADDR;
	write_buf[1] = 0x01;
	if(max44009_write_reg(device,write_buf,2) != 0)	
		printk("%s: error enabling custom trims \n",__func__);
		
	// write the original clock trim values back
	write_buf[0] = MAX44009_CLOCK_TRIM_COARSE;
	write_buf[1] = device->clock_coarse;	
	if(max44009_write_reg(device,write_buf,2) != 0)	
		printk("%s: error resetting coarse clock trim \n",__func__);		
	write_buf[0] = MAX44009_CLOCK_TRIM_FINE;
	write_buf[1] = device->clock_fine;	
	if(max44009_write_reg(device,write_buf,2) != 0)	
		printk("%s: error resetting fine clock trim \n",__func__);		
		
	// scale the original trim values by their respective scaling constants
	write_buf[0] = MAX44009_GREEN_TRIM_ADDR;
	write_buf[1] = ( (device->green_trim * green_scale_val) / 100 );	
	if(max44009_write_reg(device,write_buf,2) != 0)	
		printk("%s: error scaling green channel trim \n",__func__);			
	write_buf[0] = MAX44009_IR_TRIM_ADDR;
	write_buf[1] = ( (device->ir_trim * ir_scale_val) / 100 );	
	if(max44009_write_reg(device,write_buf,2) != 0)	
		printk("%s: error scaling ir channel trim \n",__func__);
	
	// all done!
	return 0;
}
													 

/**
 *  Changes the threshold zone in the device by changing the values in 
 *  registers 0x5 and 0x6 (see datasheet for more info).
 *  It is recommended, but not mandatory, to disable the interrupt while 
 *  this is being done to avoid unexpected behavior.
 *
 *  @param  *device 
 *	  describes a particular ALS object
 *  @param  *new_zone 
 *    a new threshold zone to adopt
 *  @return
 *    Error code if something goes wrong, otherwise 0
 */
int max44009_set_threshold_zone(struct MAX44009Data *device,
								struct MAX44009ThreshZone *new_zone)
{
  u8 buf[2];

  dprintk(KERN_INFO "max44009_set_threshold_zone\n");
   
  buf[0] = MAX44009_THRESH_HIGH_ADDR;
  buf[1] = new_zone->upperThresh;
  if(max44009_write_reg(device,buf,2))
	goto thresholdSetError;

  buf[0] = MAX44009_THRESH_LOW_ADDR;
  buf[1] = new_zone->lowerThresh;
  if(max44009_write_reg(device,buf,2))
	goto thresholdSetError;
  
  // update the register data
  device->thresh_high = new_zone->upperThresh;
  device->thresh_low = new_zone->lowerThresh;
   
  return 0;
thresholdSetError:
  printk("%s: couldn't update thresholds\n",__func__);
  return -EINVAL; 
} // max44009_set_threshold_zone

/**
 *  Remove function for this I2C driver. 
 *
 *  ***Author's Note***
 *    Please adapt this function to your particular platform.
 *
 *  @param *client 
 *		the associated I2C client for the device
 *
 *	@return
 *    0
 */
static int max44009_remove(struct i2c_client *client)
{
  struct MAX44009Data *deviceData = i2c_get_clientdata(client);

  dprintk(KERN_INFO "max44009_remove\n");

  if (!IS_ERR_OR_NULL(deviceData->regulator))
	regulator_put(deviceData->regulator);
  free_irq(deviceData->client->irq, deviceData);
  input_unregister_device(deviceData->idev);
  input_free_device(deviceData->idev);
  kfree(deviceData);
  return 0;
} // max44009Remove

static int max44009_als_open(struct inode *inode, struct file *file)
{
    struct MAX44009Data *maxd = max_data;

    if (maxd->als_opened)
    {
        return -EBUSY;
    }
    maxd->als_opened = 1;

    return 0;
}

static int max44009_als_release(struct inode *inode, struct file *file)
{
    struct MAX44009Data *maxd = max_data;

    maxd->als_opened = 0;

    return 0;
}

static long max44009_als_ioctl(struct file *file, 
                               unsigned int cmd, 
                               unsigned long arg)
{
    int rc, val;
    struct MAX44009Data *maxd = max_data;

    dprintk(KERN_INFO "als ioctl cmd %d\n", _IOC_NR(cmd));
    //printk("max44009_als_ioctl cmd num =%d \n",_IOC_NR(cmd));
    switch(cmd)
    {
        case MAXIM_MAX44009_IOCTL_SET_INTE:
            if (get_user(val, (unsigned long __user *)arg)) {
                rc = -EFAULT;
                break;
            }
            dprintk(KERN_INFO "%s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
	    	__func__, val);
            rc = max44009_set_interrupt(maxd, val);
            break;
        case MAXIM_MAX44009_IOCTL_GET_INTE:
            val = maxd->interrupt_en;
            dprintk(KERN_INFO "%s MAXIM_MAX44009_IOCTL_GET_INTE, enabled %d\n",
	    	__func__, val);
            rc = put_user(val, (unsigned long __user *)arg);
            break;
        case MAXIM_MAX44009_IOCTL_GETDATA:
            max44009_read_ALS(maxd, &val);
            rc = put_user(val, (unsigned long __user *)arg);
            break;

        default:
            printk("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
            rc = -EINVAL;
    }

    return rc;
}

static struct file_operations max44009_als_fops =
{
    .owner = THIS_MODULE,
    .open = max44009_als_open,
    .release = max44009_als_release,
    .unlocked_ioctl = max44009_als_ioctl
};

static struct miscdevice max44009_als_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = MAX44009_NAME,
    .fops = &max44009_als_fops
};

/**
 *	Sets all the settable user registers to known values, either what's supplied in
 *	the device pointer or to the power-on defaults as specified in the datasheet
 *
 *	@param *device
 *	  points to a represenation of a MAX44009 device in memory
 *	@param use_defaults
 *	  set this to false if you want to set the registers to what's stored inside
 *    struct MAX44009Data *device
 *	@return
 *    0 if no error, error code otherwise
 *
 */
static int max44009_initialize_device(struct MAX44009Data *device,bool use_defaults)
{
  u8 buf[2];   // write buffer

  dprintk(KERN_INFO "max44009_initialize_device\n");

  // initialize receiver configuration register
  buf[0] = MAX44009_CONFIG_ADDR; /* 0x02 */
  /* 0xbx lets you set the gain to any value */
  /* else currently automated gain control */
  /* It is better to set this before enabling the interrupt */
  buf[1] = use_defaults ? 0x83:(device->config);
  if(max44009_write_reg(device,buf,2)) 
    goto max44009_failed_init;
  device->config = buf[1];
//  max44009_read_reg(device,buf,1);
  
  // initialize main configuration register
  buf[0] = MAX44009_INT_ENABLE_ADDR; /* 0x01 */
  buf[1] = use_defaults ? 0x00:(device->interrupt_en); /* Disable Int */
//  buf[1] = use_defaults ? 0x01:(device->interrupt_en); /* Enable Int */
  if(max44009_write_reg(device,buf,2)) 
    goto max44009_failed_init;
  device->interrupt_en = buf[1];
//  max44009_read_reg(device,buf,1);
  
  // initialize upper threshold
  buf[0] = MAX44009_THRESH_HIGH_ADDR; /* 0x05 */
//  buf[1] = use_defaults ? 0xFF:(device->thresh_high);
//  buf[1] = use_defaults ? 0xFD:(device->thresh_high); /* Flash/Sun Light */
  buf[1] = use_defaults ? 0x88:(device->thresh_high); /* Room Light */
  if(max44009_write_reg(device,buf,2))
    goto max44009_failed_init;
  device->thresh_high = buf[1];
//  max44009_read_reg(device,buf,1);

  // initialize lower threshold
  buf[0] = MAX44009_THRESH_LOW_ADDR; /* 0x06 */
//  buf[1] = use_defaults ? 0x00:(device->thresh_low);
//  buf[1] = use_defaults ? 0x58:(device->thresh_low); /* Shadow */
  buf[1] = use_defaults ? 0x16:(device->thresh_low); /* Cover */
  if(max44009_write_reg(device,buf,2))
    goto max44009_failed_init;  
  device->thresh_low = buf[1];
//  max44009_read_reg(device,buf,1);

  // initialize threshold timer
  buf[0] = MAX44009_THRESH_TIM_ADDR; /* 0x07 */
//  buf[1] = use_defaults ? 0xFF:(device->thresh_tim);
//  buf[1] = use_defaults ? 0x05:(device->thresh_tim); /* 0.5 sec */
//  buf[1] = use_defaults ? 0x01:(device->thresh_tim); /* 0.1 sec */
  buf[1] = use_defaults ? 0x00:(device->thresh_tim); /* 0.0 sec */
  if(max44009_write_reg(device,buf,2))
    goto max44009_failed_init;    
  device->thresh_tim = buf[1];
//  max44009_read_reg(device,buf,1);
  
  return 0;
max44009_failed_init:
  printk("%s: couldn't initialize register %x\n",__func__,buf[0]);
  return -EINVAL;
} // initialization

static ssize_t max44009_int_enable_show(struct device *dev, 
                                        struct device_attribute *attr, 
                                        char *buf)
{
	int ret = 0;
	struct MAX44009Data *maxd = max_data;

	ret = sprintf(buf, "INTE = %d\n", maxd->interrupt_en);

	return ret;
}

static ssize_t max44009_int_enable_store(struct device *dev, 
                                         struct device_attribute *attr, 
                                         const char *buf, 
                                         size_t count)
{
	int ret = 0;
	int val;
	struct MAX44009Data *maxd = max_data;

	val = -1;
	sscanf(buf, "%d", &val);

	if (val != 0 && val != 1 )
	{
		pr_err("[ERROR]%s: the value is not expectd, should be [0,1]\n", 
			__func__);
		return -EINVAL;
	}

	ret = max44009_set_interrupt(maxd, val);

	if (ret < 0)
		pr_err("[ERROR]%s: set interrupt enable fail\n", 
			__func__);

	return count;
}

static ssize_t max44009_cont_mode_show(struct device *dev, 
                                       struct device_attribute *attr, 
                                       char *buf)
{
	int ret = 0;
	struct MAX44009Data *maxd = max_data;

	ret = sprintf(buf, "CONT = %d\n", (maxd->config&0x80)?true:false);

	return ret;
}

static ssize_t max44009_cont_mode_store(struct device *dev, 
                                        struct device_attribute *attr, 
                                        const char *buf, 
                                        size_t count)
{
	int ret = 0;
	int val;
	struct MAX44009Data *maxd = max_data;

	val = -1;
	sscanf(buf, "%d", &val);

	if (val != 0 && val != 1 )
	{
		pr_err("[ERROR]%s: the value is not expectd, should be [0,1]\n", 
			__func__);
		return -EINVAL;
	}

	ret = max44009_set_continuous_mode(maxd, val);

	if (ret < 0)
		pr_err("[ERROR]%s: set continuous mode fail\n", 
			__func__);

	return count;
}

static ssize_t max44009_manual_config_show(struct device *dev, 
                                           struct device_attribute *attr, 
                                           char *buf)
{
	int ret = 0;
	struct MAX44009Data *maxd = max_data;

	ret = sprintf(buf, "MANUAL = %d\n", (maxd->config&0x40)?true:false);

	return ret;
}

static ssize_t max44009_manual_config_store(struct device *dev, 
                                            struct device_attribute *attr, 
                                            const char *buf, 
                                            size_t count)
{
	int ret = 0;
	int val;
	struct MAX44009Data *maxd = max_data;

	val = -1;
	sscanf(buf, "%d", &val);

	if (val != 0 && val != 1 )
	{
		pr_err("[ERROR]%s: the value is not expectd, should be [0,1]\n", 
			__func__);
		return -EINVAL;
	}

	ret = max44009_set_manual_mode(maxd, val);

	if (ret < 0)
		pr_err("[ERROR]%s: set manual configuration fail\n", 
			__func__);

	return count;
}

static ssize_t max44009_current_division_ratio_show(struct device *dev, 
                                                    struct device_attribute *attr, 
                                                    char *buf)
{
	int ret = 0;
	struct MAX44009Data *maxd = max_data;

	ret = sprintf(buf, "CDR = %d\n", (maxd->config&0x08)?true:false);

	return ret;
}

static ssize_t max44009_current_division_ratio_store(struct device *dev, 
                                                     struct device_attribute *attr, 
                                                     const char *buf, 
                                                     size_t count)
{
	int ret = 0;
	int val;
	struct MAX44009Data *maxd = max_data;

	val = -1;
	sscanf(buf, "%d", &val);

	if (val != 0 && val != 1 )
	{
		pr_err("[ERROR]%s: the value is not expectd, should be [0,1]\n", 
			__func__);
		return -EINVAL;
	}

	ret = max44009_set_current_div_ratio(maxd, val);

	if (ret < 0)
		pr_err("[ERROR]%s: set current division ratio fail\n", 
			__func__);

	return count;
}

static ssize_t max44009_integration_time_show(struct device *dev, 
                                              struct device_attribute *attr, 
                                              char *buf)
{
	int ret = 0;
	struct MAX44009Data *maxd = max_data;

	switch(maxd->config&0x07)
	{
		case 0:
			ret = sprintf(buf, "Intergration Time = 800ms(This is a preferred "
							"mode for boosting low-light sensitivity.)\n");
			break;
		case 1:
			ret = sprintf(buf, "Intergration Time = 400ms\n");
			break;
		case 2:
			ret = sprintf(buf, "Intergration Time = 200ms\n");
			break;
		case 3:
			ret = sprintf(buf, "Intergration Time = 100ms(This is a preferred "
							"mode for high-brightness applications.)\n");
			break;
		case 4:
			ret = sprintf(buf, "Intergration Time = 50ms(Manual mode only.)\n");
			break;
		case 5:
			ret = sprintf(buf, "Intergration Time = 25ms(Manual mode only.)\n");
			break;
		case 6:
			ret = sprintf(buf, "Intergration Time = 12.5ms(Manual mode only.)\n");
			break;
		case 7:
			ret = sprintf(buf, "Intergration Time = 6.25ms(Manual mode only.)\n");
			break;
	}

	return ret;
}

static ssize_t max44009_integration_time_store(struct device *dev, 
                                               struct device_attribute *attr, 
                                               const char *buf, 
                                               size_t count)
{
	int ret = 0;
	int val;
	struct MAX44009Data *maxd = max_data;

	val = -1;
	sscanf(buf, "%d", &val);

	if (val < 0 || val > 7 )
	{
		pr_err("[ERROR]%s: the value is not expectd, should be [0~7]\n", 
			__func__);
		return -EINVAL;
	}

	ret = max44009_set_integration_time(maxd, val);

	if (ret < 0)
		pr_err("[ERROR]%s: set integration time fail\n", 
			__func__);

	return count;
}

static ssize_t max44009_threshold_zone_show(struct device *dev, 
                                            struct device_attribute *attr, 
                                            char *buf)
{
	int ret = 0;
	struct MAX44009Data *maxd = max_data;

	ret = sprintf(buf, "Upper Threshold High-Byte = 0x%x\n"
						"Lower Threshold High-Byte = 0x%x\n", 
						maxd->thresh_high, 
						maxd->thresh_low);

	return ret;
}

static ssize_t max44009_threshold_zone_store(struct device *dev, 
                                             struct device_attribute *attr, 
                                             const char *buf, 
                                             size_t count)
{
	int ret = 0;
	struct MAX44009ThreshZone val;
	struct MAX44009Data *maxd = max_data;

	sscanf(buf, "0x%x 0x%x", &val.upperThresh, &val.lowerThresh);

	if (val.upperThresh < 0 || val.upperThresh > 0xef )
		return -EINVAL;
	if (val.lowerThresh < 0 || val.lowerThresh > 0xef )
		return -EINVAL;

	ret = max44009_set_threshold_zone(maxd, &val);

	if (ret < 0)
		pr_err("[ERROR]%s: set threshold zone fail\n", 
			__func__);

	return count;
}

static ssize_t max44009_threshold_timer_show(struct device *dev, 
                                             struct device_attribute *attr, 
                                             char *buf)
{
	int ret = 0;
	struct MAX44009Data *maxd = max_data;

	ret = sprintf(buf, "Threshold Timer = %d\n", maxd->thresh_tim);

	return ret;
}

static ssize_t max44009_threshold_timer_store(struct device *dev, 
                                              struct device_attribute *attr, 
                                              const char *buf, 
                                              size_t count)
{
	int ret = 0;
	int val;
	struct MAX44009Data *maxd = max_data;

	val = -1;
	sscanf(buf, "%d", &val);

	if (val < 0 || val > 255 )
	{
		pr_err("[ERROR]%s: the value is not expectd, should be [0~255]\n", 
			__func__);
		return -EINVAL;
	}

	ret = max44009_set_thresh_tim(maxd, val);

	if (ret < 0)
		pr_err("[ERROR]%s: set threshold timer fail\n", 
			__func__);

	return count;
}

static DEVICE_ATTR(max44009_int_enable, S_IRUGO|S_IWUSR|S_IWGRP, 
                   max44009_int_enable_show, 
                   max44009_int_enable_store);
static DEVICE_ATTR(max44009_cont_mode, S_IRUGO|S_IWUSR|S_IWGRP, 
                   max44009_cont_mode_show, 
                   max44009_cont_mode_store);
static DEVICE_ATTR(max44009_manual_config, S_IRUGO|S_IWUSR|S_IWGRP, 
                   max44009_manual_config_show, 
                   max44009_manual_config_store);
static DEVICE_ATTR(max44009_current_division_ratio, S_IRUGO|S_IWUSR|S_IWGRP, 
                   max44009_current_division_ratio_show, 
                   max44009_current_division_ratio_store);
static DEVICE_ATTR(max44009_integration_time, S_IRUGO|S_IWUSR|S_IWGRP, 
                   max44009_integration_time_show, 
                   max44009_integration_time_store);
static DEVICE_ATTR(max44009_threshold_zone, S_IRUGO|S_IWUSR|S_IWGRP, 
                   max44009_threshold_zone_show, 
                   max44009_threshold_zone_store);
static DEVICE_ATTR(max44009_threshold_timer, S_IRUGO|S_IWUSR|S_IWGRP, 
                   max44009_threshold_timer_show, 
                   max44009_threshold_timer_store);

static struct attribute *max44009_attributes[] = {
    &dev_attr_max44009_int_enable.attr,
    &dev_attr_max44009_cont_mode.attr,
    &dev_attr_max44009_manual_config.attr,
    &dev_attr_max44009_current_division_ratio.attr,
    &dev_attr_max44009_integration_time.attr,
    &dev_attr_max44009_threshold_zone.attr,
    &dev_attr_max44009_threshold_timer.attr,
    NULL
};

static struct attribute_group max44009_attribute_group = {
    .attrs = max44009_attributes,
};

/**
 * 	Probes the adapter for a valid MAX44009 device, then initializes it. 
 *
 *  ***Author's Note***
 *    You may need to modify this function to suit your system's specific needs
 *    I have a platform_data fetch in here, but you may not need it
 *
 *  @param *client 
 *	  points to an i2c client object
 *  @param *id 
 *    device id
 *  
 *  @return
 *    0 on success, otherwise an error code
 */
static int max44009_probe(struct i2c_client *client,
                          const struct i2c_device_id *id )
{
  /*struct MAX44009PlatformData *pdata = client->dev.platform_data;*/
  struct MAX44009Data *deviceData;
  int err = 0;
//  int i;
//  u8 buffer;

  printk(KERN_INFO "max44009_probe\n");
  
  // attempt to get platform data from the device
  /*if (pdata == NULL)
  {
    printk("%s: couldn't get platform data\n",__func__);
    return -ENODEV;
  }*/

  // check to make sure that the adapter supports I2C
  if(!i2c_check_functionality(client->adapter,I2C_FUNC_I2C)) 
  {
    printk("%s: I2C_FUNC_I2C not supported\n",__func__);
    return -ENODEV;
  }
  
  // allocate some memory for the device
  deviceData = kzalloc(sizeof(struct MAX44009Data),GFP_KERNEL);
  if (deviceData == NULL)
  {
    printk("%s: couldn't allocate memory\n",__func__);
    return -ENOMEM;
  }
  
  // start initializing...
  deviceData->client = client;
  /*deviceData->pdata = pdata;*/
  deviceData->idev = input_allocate_device();
  
  // couldn't allocate input device?
  if (!deviceData->idev)
  {
    printk("%s: couldn't allocate input device\n",__func__);
    kfree(deviceData);
    return -ENOMEM;
  }
  
  max_data = deviceData;
  
  // set up the input device
  deviceData->idev->name = MAX44009_NAME;
  /*input_set_capability(deviceData->idev, EV_MSC, MSC_RAW);*/
  input_set_capability(deviceData->idev, EV_ABS, ABS_MISC);
  
  atomic_set(&deviceData->enabled, 0);
  // initialize work queue
  INIT_DELAYED_WORK(&deviceData->work_queue, max44009_work_queue);
  
  // attempt to register the input device
  if ( (err = input_register_device(deviceData->idev)) ) 
  {
    printk("%s: input device register failed:%d\n", __func__,err);
    goto error_input_register_failed;
  }

  err = misc_register(&max44009_als_device);
  if (err < 0)
  {
    pr_err("%s: couldn't register ls misc device\n",__func__);
    goto errRegMiscFailed;
  }
  
  // attempt to initialize the device
  err = max44009_initialize_device(deviceData,true);
  if (err < 0) 
  {
    printk("%s: Register initialization failed: %d\n",__func__, err);
    err = -ENODEV;
    goto errRegInitFailed;
  }
  
  // set up an IRQ channel
  spin_lock_init(&deviceData->irqLock);
  deviceData->irqState = 1;

  err = request_irq(deviceData->client->irq+INT_GPIO_0, max44009_irq_handler,
	              IRQF_DISABLED, MAX44009_NAME, deviceData);
  if (err != 0) 
  {
    printk("%s: irq request failed: %d\n", __func__, err);
    err = -ENODEV;
    goto errReqIRQFailed;
  }

  dprintk(KERN_INFO "max44009_probe irq request succeeded.\n");
  
  i2c_set_clientdata(client, deviceData);

  /*deviceData->regulator = regulator_get(&client->dev, "vio");*/
  
  err = sysfs_create_group(&deviceData->idev->dev.kobj, &max44009_attribute_group);
  if (err !=0)
  {
      dev_err(&client->dev,"%s:create sysfs group error", __func__);
      goto errReqSysfsFailed;
  }

  dprintk(KERN_INFO "max44009_probe completed.\n");


  return 0;

// error handling
errReqSysfsFailed:
errReqIRQFailed:
errRegMiscFailed:
  misc_deregister(&max44009_als_device);
errRegInitFailed:
  input_unregister_device(deviceData->idev);
error_input_register_failed:
  input_free_device(deviceData->idev);
  kfree(deviceData);
  return err;
} // max44009Probe

// descriptor of the MAX44009 device ID
static const struct i2c_device_id max44009_id[] = {
  {MAX44009_NAME, 0},
  {}
};

// descriptor of the MAX44009 I2C driver
static struct i2c_driver max44009_i2c_driver = {
  .remove   = max44009_remove,
  .id_table = max44009_id,
  .probe = max44009_probe,
  .driver = {
	.name = MAX44009_NAME,
	.owner = THIS_MODULE,
  },
};

// initialization and exit functions
int __init max44009_init(void)
{
  dprintk(KERN_INFO "*************\n");
  dprintk(KERN_INFO "*************\n");
  dprintk(KERN_INFO "*************\n");
  dprintk(KERN_INFO "max44009_init\n");
  dprintk(KERN_INFO "*************\n");
  dprintk(KERN_INFO "*************\n");
  dprintk(KERN_INFO "*************\n");
  return i2c_add_driver(&max44009_i2c_driver);
}

void __exit max44009_exit(void)
{
  dprintk(KERN_INFO "max44009_exit\n");
  i2c_del_driver(&max44009_i2c_driver);
}

module_init(max44009_init);
module_exit(max44009_exit);

MODULE_DESCRIPTION("ALS driver for Maxim MAX44009");
MODULE_AUTHOR("Ilya Veygman <ilya.veygman@maxim-ic.com>");
MODULE_LICENSE("GPL");
