#ifndef __MAX44009_H_

#define MAX44009_NAME			"max44009"
#define MAX44009_I2C_ADDRESS0     	0x94
#define MAX44009_I2C_ADDRESS1     	0x96
#define MAX44009_INT_STATUS_ADDR  	0x00
#define MAX44009_INT_ENABLE_ADDR 	0x01
#define MAX44009_CONFIG_ADDR 		0x02
#define MAX44009_LUX_HIGH_ADDR		0x03
#define MAX44009_LUX_LOW_ADDR		0x04
#define MAX44009_THRESH_HIGH_ADDR	0x05
#define MAX44009_THRESH_LOW_ADDR	0x06
#define MAX44009_THRESH_TIM_ADDR	0x07
#define MAX44009_CLOCK_TRIM_COARSE	0x09
#define MAX44009_CLOCK_TRIM_FINE 	0x0A
#define MAX44009_GREEN_TRIM_ADDR	0x0B
#define MAX44009_IR_TRIM_ADDR		0x0C
#define MAX44009_OTP_REG_ADDR		0x0D
#define MAX44009_RETRY_DELAY      	10
#define MAX44009_MAX_RETRIES      	5	

#define MAXIM_IOCTL_MAGIC 'l'

#define MAXIM_MAX44009_IOCTL_GET_INTE _IOR(MAXIM_IOCTL_MAGIC, 1, int *)
#define MAXIM_MAX44009_IOCTL_SET_INTE _IOW(MAXIM_IOCTL_MAGIC, 2, int *)
#define MAXIM_MAX44009_IOCTL_GETDATA _IOR(MAXIM_IOCTL_MAGIC, 3, int *)

#include <linux/types.h>

struct MAX44009PlatformData 
{
	// put things here
	int placeHolder;
};
/** 
 *  A structure that defines an "operating zone" for the light sensor with upper
 *  and lower thresholds. The MAX44009 has two threshold registers that can be
 *  programmed to trigger an interrupt when the light level has exceeded the 
 *  values for a specified amount of time.
 *
*/
struct MAX44009ThreshZone {
  unsigned int upperThresh;
  unsigned int lowerThresh;
};

/**
 *  This structure has register data for the MAX44009, with each 8-bit register
 *  described for the sake of clarity. Please consult the datasheet for detailed
 *  information on each register
 *
 *  The settings here are meant to be a reflection of the internal register 
 *  settings of the MAX44009, as a hope that it will reduce the need for reading
 *  the device as much (to save power)
 */
struct MAX44009Data {
  struct i2c_client *client;          // represents the slave device
  struct input_dev *idev;             // an input device
  struct delayed_work work_queue;
  struct MAX44009ThreshZone threshZones;
  struct MAX44009PlatformData *pdata;
  atomic_t enabled;
  spinlock_t irqLock;
  int irqState;                       // device's IRQ state  
  struct regulator *regulator;
  
  int als_opened;
  
  // internal settings follow...
  u8 interrupt_status;	
  u8 interrupt_en;	// interrupt enable
  u8 config;		// configuration register
  u8 lux_high;		// lux high byte register
  u8 lux_low;		// lux low byte register
  u8 thresh_high;	// upper threshold: high byte
  u8 thresh_low;	// lower threshold: low byte
  u8 thresh_tim;	// threshold timer
  u8 clock_coarse;	// coarse clock value (DO NOT CHANGE)
  u8 clock_fine;	// fine clock value (DO NOT CHANGE)
  u8 green_trim;	// original green trim register
  u8 ir_trim;		// original ir trim register
  u8 otp_status;	// otp selection register
};

int max44009_read_reg(struct MAX44009Data *device,u8 *buffer,int length);
int max44009_write_reg(struct MAX44009Data *device,u8 *buffer,int length);
int max44009_set_interrupt(struct MAX44009Data *device,bool enable);
int max44009_read_int_status(struct MAX44009Data *device);
int max44009_set_integration_time(struct MAX44009Data *device,u8 time);
int max44009_set_manual_mode(struct MAX44009Data *device,bool enable);
int max44009_set_continuous_mode(struct MAX44009Data *device,bool enable);
int max44009_set_current_div_ratio(struct MAX44009Data *device,bool enable);
int max44009_set_thresh_tim(struct MAX44009Data *device,u8 new_tim_val);
int max44009_set_otp(struct MAX44009Data *device,bool enable);
int max44009_set_threshold_zone(struct MAX44009Data *device,
                                struct MAX44009ThreshZone *new_zone);
int max44009_report_light_level(struct MAX44009Data *alsDevice,
                                    bool clearInterruptFlag);
static int max44009_probe(struct i2c_client *client,
                          const struct i2c_device_id *id );
static int max44009_read_ALS(struct MAX44009Data *alsDevice,int *adjusted_lux);
static int max44009_remove(struct i2c_client *client);
static int max44009_initialize_device(struct MAX44009Data *device,bool use_defaults);
static irqreturn_t max44009_irq_handler(int irq, void *device);
static void max44009_work_queue(struct work_struct *work);
void max44009_enable_IRQ(struct MAX44009Data *device,bool enable);
int max44009_scale_gains(struct MAX44009Data *device,u32 green_scale_val,u32 ir_scale_val);

#define __MAX44009_H_
#endif
