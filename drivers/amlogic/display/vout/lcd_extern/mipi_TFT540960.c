/*
 * AMLOGIC lcd external driver.
 *
 * Communication protocol:
 * MIPI 
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/jiffies.h> 
#include <linux/i2c.h>
#include <linux/i2c-aml.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <asm/uaccess.h>
#include <mach/pinmux.h>
#include <mach/gpio.h>
#include <linux/amlogic/vout/aml_lcd_extern.h>

static struct lcd_extern_config_t *lcd_extern_config = NULL;
static struct i2c_client *aml_touch_i2c_client = NULL;
//#define LCD_EXT_DEBUG_INFO
#ifdef LCD_EXT_DEBUG_INFO
#define DBG_PRINT(...)		printk(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#endif

#define LCD_EXTERN_NAME			"lcd_mipi_TFT540960"

//******************** mipi command ********************//
//format:  data_type, num, data....
//special: data_type=0xff, num<0xff means delay ms, num=0xff means ending.
//******************************************************//
static unsigned char mipi_init_on_table[300] = {0}; // one element is 268, another is 288, so I use 300 for reserve mempoy space
static unsigned char mipi_init_on_edt_table[] = { //hx8389
    0x29, 4, 0xB9,0xFF,0x83,0x89,
    0x29, 21, 0xB1,0x7F,0x14,0x14,0x35,0x55,0x50,0xD0,0xED,0x51,0x80,
        0x20,0x20,0xF8,0xAA,0xAA,0xA3,0x00,0x80,0x30,0x00,
    0x29, 11, 0xB2, 0x80,0x50,0x0C,0x12,0x40,0x38,0x11,0x64,0x55,0x09,
    0x29, 12, 0xB4,0x10,0x6C,0x10,0x6C,0x00,0x00,0x08,0x78,0x08,0x78,0x88,
    0x29, 36, 0xD3, 0x00,0x00,0x00,0x00,0x00,0x20,0x00,0x32,0x10,0x07,
         0x00,0x07,0x03,0xCB,0x03,0xCB,0x00,0x00,0x00,0x00,
         0x37,0x04,0x09,0x09,0x37,0x00,0x00,0x00,0x0C,0x00,
         0x00,0x00,0x0A,0x00,0x01,
    0x29, 39, 0xD5, 0x18,0x20,0x18,0x22,0x21,0x18,0x23,0x18,0x18,0x02,
         0x18,0x06,0x03,0x18,0x07,0x18,0x18,0x04,0x18,0x00,
         0x05,0x18,0x01,0x18,0x27,0x18,0x25,0x18,0x18,0x26,
         0x18,0x24,0x18,0x00,0x18,0x18,0x18,0x18,
    0x29, 39, 0xD6, 0x18,0x27,0x18,0x25,0x26,0x18,0x24,0x18,0x18,0x01,
         0x18,0x05,0x00,0x18,0x04,0x18,0x18,0x07,0x18,0x03,
         0x06,0x18,0x02,0x18,0x20,0x18,0x22,0x18,0x18,0x21,
         0x18,0x23,0x18,0x18,0x18,0x18,0x18,0x18,

    0x29, 6, 0xD8,0xFF,0xFF,0xFF,0xFF,0XFC,
    0x29, 3, 0xB6,0x20,0x20,
    0x29, 43, 0xE0, 0x00,0x08,0x0E,0x19,0x16,0x3F,0x21,0x37,0x07,0x0C,0x0E,0x18,0x0F,0x12,0x14,0x13,0x14,0x0A,0x16,0x18,0x1A,
         0x00,0x08,0x0E,0x19,0x16,0x3F,0x21,0x38,0x08,0x0C,0x0D,0x18,0x0E,0x12,0x14,0x12,0x13,0x09,0x15,0x17,0x1A,
    0x23, 2,0xCC,0x02,
    0x23, 2,0xD2,0x55,
    0x29, 5,0xC0,0x30,0x15,0x00,0x03,
    0x29, 5,0xC7,0x00,0x80,0x00,0xC0,
    //////////////
    0x05, 1, 0x11,
    0xff, 150,   //delay 150ms
    0x05, 1, 0x29,
    0xff, 10,    //delay 10ms
    0xff, 0xff,  //ending flag
};

static unsigned char mipi_init_on_truly_table[] = { //hx8389
    0x29, 4, 0xB9,0xFF,0x83,0x89,
    0x29, 21, 0xB1,0x7F,0x10,0x10,0xf2,
        0x32,0x90,0x10,0xEC,0x52,
        0x80,0x20,0x20,0xF8,0xAA,
        0xAA,0xA1,0x00,0x80,0x30,0x00,

    0x29, 11, 0xB2,0x80,0x50,0x05,0x07,
        0x40,0x38,0x11,0x64,0x5D,0x09,
    0x29, 12, 0xB4,0x70,0x70,0x70,0x70,
        0x00,0x00,0x10,0x56,0x10,
        0x56,0xB0,
    0x29, 36, 0xD3,
        0x00,0x00,0x00,0x00,0x00,
        0x08,0x00,0x32,0x10,0x00,
        0x00,0x00,0x03,0xC6,0x03,
        0xC6,0x00,0x00,0x00,0x00,
        0x35,0x33,0x04,0x04,0x37,
        0x00,0x00,0x00,0x05,0x08,
        0x00,0x00,0x0A,0x00,0x01,
    0x29, 39, 0xD5,
        0x18,0x18,0x18,0x18,0x19,
        0x19,0x18,0x18,0x20,0x21,
        0x24,0x25,0x18,0x18,0x18,
        0x18,0x00,0x01,0x04,0x05,
        0x02,0x03,0x06,0x07,0x18,
        0x18,0x18,0x18,0x18,0x18,
        0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
    0x29, 39, 0xD6,
        0x18,0x18,0x18,0x18,0x18,
        0x18,0x19,0x19,0x25,0x24,
        0x21,0x20,0x18,0x18,0x18,
        0x18,0x07,0x06,0x03,0x02,
        0x05,0x04,0x01,0x00,0x18,
        0x18,0x18,0x18,0x18,0x18,
        0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
    0x29, 5, 0xBF,0xB6,0x43,0x50,0xE0,
    0x29, 5, 0xC9,0x1F,0x2E,0x00,0x1E,
    0x29, 43, 0xE0,
        0x00,0x05,0x07,0x33,0x3A,
        0x3F,0x1C,0x42,0x08,0x0B,
        0x0E,0x18,0x0F,0x12,0x15,
        0x13,0x14,0x06,0x11,0x13,
        0x18,0x00,0x05,0x07,0x33,
        0x3A,0x3F,0x1C,0x42,0x08,
        0x0B,0x0E,0x18,0x0E,0x12,
        0x14,0x12,0x13,0x06,0x11,0x13,0x18,
    0x23, 2, 0xD2,0x33,
    0x23, 2, 0xCC,0x02,
    0x29, 5, 0xC7,0x00,0x80,0x00,0xC0,
    0x29, 4, 0xB6,0x6A,0x6A,0x00,

    //***CABC***20120225///
    0x39, 2, 0x51,0xFF,//Write display brightness
    0x39, 2, 0x53,0x24,//Write CTRL display
    0x39, 2, 0x55,0x01,//Write content adaptive brightness control,01~UI mode, 02~ Still mode,03~Moving mode,00~Off
    0x39, 2, 0x5E,0x00,//Write CABC minimum brightness
    //////////////
    0x05, 1, 0x11,
    0xff, 150,   //delay 150ms
    0x05, 1, 0x29,
    0xff, 10,    //delay 10ms
    0xff, 0xff,  //ending flag
};

static unsigned char mipi_init_off_table[] = {
    0x05,1,0x28, //display off
    0xff,10,     //delay 10ms
    0x05,1,0x10, //sleep in
    0xff,10,     //delay 10ms
    0xff,0xff,   //ending flag
};

/*
static int aml_i2c_write(struct i2c_client *i2client,unsigned char *buff, unsigned len)
{
    int res = 0;
    struct i2c_msg msg[] = {
        {
        .addr = i2client->addr,
        .flags = 0,
        .len = len,
        .buf = buff,
        }
    };
    
    res = i2c_transfer(i2client->adapter, msg, 1);
    if (res < 0) {
        printk("%s: i2c transfer failed [addr 0x%02x]\n", __FUNCTION__, i2client->addr);
    }
    
    return res;
}
*/
static int aml_i2c_read(struct i2c_client *i2client,unsigned char *buff, unsigned len)
{
    int res = 0;
    struct i2c_msg msgs[] = {
        {
            .addr = i2client->addr,
            .flags = 0,
            .len = 1,
            .buf = buff,
        },
        {
            .addr = i2client->addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = buff,
        }
    };
    res = i2c_transfer(i2client->adapter, msgs, 2);
    if (res < 0) {
        printk("%s: i2c transfer failed [addr 0x%02x]\n", __FUNCTION__, i2client->addr);
    }

    return res;
}

static int get_lcd_extern_config(struct device_node* of_node, struct lcd_extern_config_t *lcd_ext_cfg)
{
    int ret = 0;
    struct aml_lcd_extern_driver_t* lcd_ext;

    ret = get_lcd_extern_dt_data(of_node, lcd_ext_cfg);
    if (ret) {
        printk("[error] %s: failed to get dt data\n", LCD_EXTERN_NAME);
        return ret;
    }

    //lcd extern driver update
    lcd_ext = aml_lcd_extern_get_driver();
    if (lcd_ext) {
        lcd_ext->type      = lcd_ext_cfg->type;
        lcd_ext->name      = lcd_ext_cfg->name;
        lcd_ext->init_on_cmd_8  = &mipi_init_on_table[0];
        lcd_ext->init_off_cmd_8 = &mipi_init_off_table[0];
    }
    else {
        printk("[error] %s get lcd_extern_driver failed\n", lcd_ext_cfg->name);
        ret = -1;
    }

    return ret;
}

static int aml_touch_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk("[error] %s: functionality check failed\n", __FUNCTION__);
        return -ENODEV;
    }
    else {
        aml_touch_i2c_client = client;
    }

    printk("%s OK\n", __FUNCTION__);
    return 0;
}

static int aml_touch_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id aml_touch_i2c_id[]= {
    {"touch_i2c", 0},
    { }
};

static struct i2c_driver aml_touch_i2c_driver= {
    .probe    = aml_touch_i2c_probe,
    .remove   = aml_touch_i2c_remove,
    .id_table = aml_touch_i2c_id,
    .driver = {
        .name = "touch_i2c",
        .owner =THIS_MODULE,
    },
};

static int aml_TFT540960_probe(struct platform_device *pdev)
{
	struct i2c_board_info i2c_info;
    struct i2c_adapter *adapter = NULL;
    struct i2c_client *i2c_client = NULL;
    unsigned char buff[2] = {0};
    int ret = 0;

    if (lcd_extern_driver_check()) {
        return -1;
    }
    if (lcd_extern_config == NULL)
        lcd_extern_config = kzalloc(sizeof(*lcd_extern_config), GFP_KERNEL);
    if (lcd_extern_config == NULL) {
        printk("[error] %s probe: failed to alloc data\n", LCD_EXTERN_NAME);
        return -1;
    }

    pdev->dev.platform_data = lcd_extern_config;

    ret = get_lcd_extern_config(pdev->dev.of_node, lcd_extern_config);
    if (ret) {
        goto lcd_extern_probe_failed;
    }
    
    memset(&i2c_info, 0, sizeof(i2c_info));
    
    adapter = i2c_get_adapter(2); // Touch IC use the I2C_B
    if (!adapter) {
        printk("[error] %s failed to get i2c adapter\n", LCD_EXTERN_NAME);
        goto lcd_extern_probe_failed;
    }

    strncpy(i2c_info.type, "touch_i2c", I2C_NAME_SIZE);
    i2c_info.addr = 0x38;//0x70 >> 1;
    i2c_info.flags = 0;
    if (i2c_info.addr > 0x7F)
        i2c_info.flags = 0x10;
    i2c_client = i2c_new_device(adapter, &i2c_info);
    if (!i2c_client) {
        printk("[error] %x :failed to new i2c device\n", i2c_info.addr);
        goto lcd_extern_probe_failed;
    }
    else{
        DBG_PRINT("[error] %x: new i2c device succeed\n",i2c_info.addr);
    }

    if (!aml_touch_i2c_client) {
        ret = i2c_add_driver(&aml_touch_i2c_driver);
        if (ret) {
            printk("[error] lcd_extern probe: add i2c_driver failed\n");
            goto lcd_extern_probe_failed;
        }
    }
    
    buff[0] = 0xA8;
    aml_i2c_read(aml_touch_i2c_client, buff, 1);
    printk("[error] touch id: 0x%X\n", buff[0]);
    if (((buff[0] & 0xC0) == 0x40) || ((buff[0] & 0xC0) == 0x80))
    {
        // EDT
        memcpy( mipi_init_on_table, mipi_init_on_edt_table, sizeof(mipi_init_on_edt_table) );
    }
    else
    {
        // Truly
        memcpy( mipi_init_on_table, mipi_init_on_truly_table, sizeof(mipi_init_on_truly_table) );
    }
    i2c_unregister_device(i2c_client);
    i2c_del_driver(&aml_touch_i2c_driver);

    printk("%s probe ok\n", LCD_EXTERN_NAME);
    return ret;

lcd_extern_probe_failed:
    if (lcd_extern_config) {
        kfree(lcd_extern_config);
        lcd_extern_config = NULL;
    }
    return -1;
}

static int aml_TFT540960_remove(struct platform_device *pdev)
{
    if (pdev->dev.platform_data)
        kfree (pdev->dev.platform_data);
    return 0;
}

#ifdef CONFIG_USE_OF
static const struct of_device_id aml_TFT540960_dt_match[]={
    {
        .compatible = "amlogic,lcd_mipi_TFT540960",
    },
    {},
};
#else
#define aml_TFT540960_dt_match NULL
#endif

static struct platform_driver aml_TFT540960_driver = {
    .probe  = aml_TFT540960_probe,
    .remove = aml_TFT540960_remove,
    .driver = {
        .name  = LCD_EXTERN_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_USE_OF
        .of_match_table = aml_TFT540960_dt_match,
#endif
    },
};

static int __init aml_TFT540960_init(void)
{
    int ret;
    DBG_PRINT("%s\n", __FUNCTION__);

    ret = platform_driver_register(&aml_TFT540960_driver);
    if (ret) {
        printk("[error] %s failed to register lcd extern driver module\n", __FUNCTION__);
        return -ENODEV;
    }

    return ret;
}

static void __exit aml_TFT540960_exit(void)
{
    platform_driver_unregister(&aml_TFT540960_driver);
}

//late_initcall(aml_TFT540960_init);
module_init(aml_TFT540960_init);
module_exit(aml_TFT540960_exit);

MODULE_AUTHOR("AMLOGIC");
MODULE_DESCRIPTION("LCD Extern driver for TFT540960");
MODULE_LICENSE("GPL");
