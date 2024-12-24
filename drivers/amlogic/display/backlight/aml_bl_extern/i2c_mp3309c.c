/*
 * AMLOGIC backlight external driver.
 *
 * Communication protocol:
 * I2C 
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
#include <linux/amlogic/aml_bl_extern.h>

static struct bl_extern_config_t *bl_ext_config = NULL;

static struct i2c_client *aml_mp3309c_i2c_client;
static struct i2c_client *aml_touch_i2c_client = NULL;

//#define BL_EXT_DEBUG_INFO
#ifdef BL_EXT_DEBUG_INFO
#define DBG_PRINT(...)		printk(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#endif

#define BL_EXTERN_NAME			"bl_i2c_mp3309c"
static unsigned int bl_status = 1;
static unsigned int bl_level = 0;

static unsigned char i2c_init_table[][2] = {
    {0x00, 0xfc}, //bit(7):EN; bit(6~2):D4~D0 (0~0x1F set backlight); bit(1~0):don't care
    {0x01, 0x28}, //bit(7):don't care; bit(6):DIMS; bit(5):SYNC; bit(4~3):OVP1~OVP0; bit(2):VOS; bit(1):LEDO; bit(0):OTP;
    {0xff, 0xff}, //ending flag
};

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
    
    /* i2c_transfer-> Returns negative errno, else the number of messages executed. */
    res = i2c_transfer(i2client->adapter, msg, 1);
    if (res < 0) {
        printk("%s: i2c transfer failed [addr 0x%02x]\n", __FUNCTION__, i2client->addr);
    }
    else if (res == 1)
        res = 0;
    
    return res;
}
#if 1
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
    /* i2c_transfer-> Returns negative errno, else the number of messages executed. */
    res = i2c_transfer(i2client->adapter, msgs, 2);
    if (res < 0) {
        printk("%s: i2c transfer failed [addr 0x%02x]\n", __FUNCTION__, i2client->addr);
    }
    else if (res == 2)
        res = 0;

    return res;
}
#endif
static int bl_extern_set_level(unsigned int level)
{
    unsigned char tData[3];
    int ret = 0;

    bl_level = level;

    if (bl_ext_config == NULL) {
        printk("no %s driver\n", BL_EXTERN_NAME);
        return -1;
    }
    get_bl_ext_level(bl_ext_config);
    DBG_PRINT("level=%d, dim_min=%d, dim_max=%d, level_min=%d, level_max=%d\n", level, bl_ext_config->dim_min, bl_ext_config->dim_max, bl_ext_config->level_min, bl_ext_config->level_max);
#if 0
    level = bl_ext_config->dim_min + ((level - bl_ext_config->level_min) * (bl_ext_config->dim_max - bl_ext_config->dim_min)) / (bl_ext_config->level_max - bl_ext_config->level_min);
#else
    level = (bl_ext_config->dim_min * 10) + ((level - bl_ext_config->level_min) * (bl_ext_config->dim_max - bl_ext_config->dim_min) * 10) / (bl_ext_config->level_max - bl_ext_config->level_min);
    /* Round off the original decimal point, then remove it by divide 10. */
    DBG_PRINT("decimal point=%d\n", (level%10));
    if ((level%10) > 4)
        level = (level / 10) + 1;
    else
        level = level / 10;
#endif
    DBG_PRINT("convert level to dim step=%d\n", level);

    /* Swap bits, from D4~D0 to D0~D4 */
    level = ((level&0x01)<<4)+((level&0x02)<<2)+(level&0x04)+((level&0x08)>>2)+((level&0x10)>>4);
    DBG_PRINT("bits swap=%d\n", level);

    /* OR enable bit for register 0x00 */
    level = (level << 2) | 0x80;
    DBG_PRINT("add enable bit=0x%x\n\n", level);

    if (bl_status) {
        tData[0] = 0x0;
        tData[1] = level;
        ret = aml_i2c_write(aml_mp3309c_i2c_client, tData, 2);
    }

    return ret;
}

static int bl_extern_power_on(void)
{
    unsigned char tData[3];
    int i=0, ending_flag=0;
    int ret=0;

    if (bl_ext_config->gpio_used > 0) {
        if (bl_ext_config->gpio_on==2)
            bl_extern_gpio_direction_input(bl_ext_config->gpio);
        else
            bl_extern_gpio_direction_output(bl_ext_config->gpio, bl_ext_config->gpio_on);
    }

    while (ending_flag == 0) {
        if (i2c_init_table[i][0] == 0xff) {    //special mark
            if (i2c_init_table[i][1] == 0xff) { //ending flag
                ending_flag = 1;
            }
            else { //delay flag
                mdelay(i2c_init_table[i][1]);
            }
        }
        else {
            tData[0]=i2c_init_table[i][0];
            tData[1]=i2c_init_table[i][1];
            ret = aml_i2c_write(aml_mp3309c_i2c_client, tData, 2);
        }
        i++;
    }
    bl_status = 1;
    bl_extern_set_level(bl_level);//recover bl level

    printk("%s\n", __FUNCTION__);
    return ret;
}

static int bl_extern_power_off(void)
{
    bl_status = 0;
    if (bl_ext_config->gpio_used > 0) {
        if (bl_ext_config->gpio_off==2)
            bl_extern_gpio_direction_input(bl_ext_config->gpio);
        else
        bl_extern_gpio_direction_output(bl_ext_config->gpio, bl_ext_config->gpio_off);
    }

    printk("%s\n", __FUNCTION__);
    return 0;
}

static int get_bl_extern_config(struct device_node* of_node, struct bl_extern_config_t *bl_ext_cfg)
{
    int ret = 0;
    struct aml_bl_extern_driver_t* bl_ext;

    ret = get_bl_extern_dt_data(of_node, bl_ext_cfg);
    if (ret) {
        printk("[error] %s: failed to get dt data\n", BL_EXTERN_NAME);
        return ret;
    }

    if (bl_ext_cfg->dim_min > 0xff)
        bl_ext_cfg->dim_min = 0xff;
    if (bl_ext_cfg->dim_max > 0xff)
        bl_ext_cfg->dim_max = 0xff;

    //bl extern driver update
    bl_ext = aml_bl_extern_get_driver();
    if (bl_ext) {
        bl_ext->type      = bl_ext_cfg->type;
        bl_ext->name      = bl_ext_cfg->name;
        bl_ext->power_on  = bl_extern_power_on;
        bl_ext->power_off = bl_extern_power_off;
        bl_ext->set_level = bl_extern_set_level;
    }
    else {
        printk("[error] %s get bl_extern_driver failed\n", bl_ext_cfg->name);
        ret = -1;
    }

    return ret;
}

struct bl_extern_config_t* get_bl_ext_conf(void) {
	return bl_ext_config;
}

static int aml_mp3309c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk("[error] %s: functionality check failed\n", __FUNCTION__);
        return -ENODEV;
    }
    else {
        aml_mp3309c_i2c_client = client;
    }

    printk("%s OK\n", __FUNCTION__);
    return 0;
}

static int aml_mp3309c_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id aml_mp3309c_i2c_id[] = {
    {BL_EXTERN_NAME, 0},
    { }
};
// MODULE_DEVICE_TABLE(i2c, aml_mp3309c_id);

static struct i2c_driver aml_mp3309c_i2c_driver = {
    .probe    = aml_mp3309c_i2c_probe,
    .remove   = aml_mp3309c_i2c_remove,
    .id_table = aml_mp3309c_i2c_id,
    .driver = {
        .name = BL_EXTERN_NAME,
        .owner =THIS_MODULE,
    },
};

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

static int aml_mp3309c_probe(struct platform_device *pdev)
{
    struct i2c_board_info i2c_info;
    struct i2c_board_info i2c_info_lcm;
    struct i2c_adapter *adapter;
    struct i2c_adapter *adapter_lcm;
    struct i2c_client *i2c_client;
    struct i2c_client *i2c_client_lcm;
    unsigned char buff[2] = {0};
    int ret = 0;

    if (bl_extern_driver_check()) {
        return -1;
    }
    if (bl_ext_config == NULL)
        bl_ext_config = kzalloc(sizeof(*bl_ext_config), GFP_KERNEL);
    if (bl_ext_config == NULL) {
        printk("[error] %s probe: failed to alloc data\n", BL_EXTERN_NAME);
        return -1;
    }

    pdev->dev.platform_data = bl_ext_config;

    // touch i2c
    memset(&i2c_info_lcm, 0, sizeof(i2c_info_lcm));

    adapter_lcm = i2c_get_adapter(2); // Touch IC use the I2C_B
    if (!adapter_lcm) {
        printk("[error] %s failed to get touch i2c adapter\n", BL_EXTERN_NAME);
        goto bl_extern_probe_failed;
    }

    strncpy(i2c_info_lcm.type, "touch_i2c", I2C_NAME_SIZE);
    i2c_info_lcm.addr = 0x38;//0x70 >> 1;
    i2c_info_lcm.flags = 0;
    if (i2c_info_lcm.addr > 0x7F)
        i2c_info_lcm.flags = 0x10;
    i2c_client_lcm = i2c_new_device(adapter_lcm, &i2c_info_lcm);
    if (!i2c_client_lcm) {
        printk("[error] %x :failed to new i2c device\n", i2c_info_lcm.addr);
        goto bl_extern_probe_failed;
    }
    else{
        DBG_PRINT("[info] %x: new i2c device succeed\n",i2c_info_lcm.addr);
    }

    if (!aml_touch_i2c_client) {
        ret = i2c_add_driver(&aml_touch_i2c_driver);
        if (ret) {
            printk("[error] lcd_extern probe: add i2c_driver failed\n");
            goto bl_extern_probe_failed;
        }
    }
    buff[0] = 0xA8;
    aml_i2c_read(aml_touch_i2c_client, buff, 1);
    printk("[info] touch id: 0x%X\n", buff[0]);
    if (((buff[0] & 0xC0) == 0x40) || ((buff[0] & 0xC0) == 0x80))
    {
        // EDT
        printk("[info] EDT \n");
        bl_ext_config->lcm = LCM_EDT;
    }
    else
    {
        // Truly
        printk("[info] Truly \n");
        bl_ext_config->lcm = LCM_Truly;
    }
    i2c_unregister_device(i2c_client_lcm);
    i2c_del_driver(&aml_touch_i2c_driver);

    ret = get_bl_extern_config(pdev->dev.of_node, bl_ext_config);
    if (ret) {
        goto bl_extern_probe_failed;
    }

    memset(&i2c_info, 0, sizeof(i2c_info));

    adapter = i2c_get_adapter(bl_ext_config->i2c_bus);
    if (!adapter) {
        printk("[error] %s£ºfailed to get i2c adapter\n", BL_EXTERN_NAME);
        goto bl_extern_probe_failed;
    }

    strncpy(i2c_info.type, bl_ext_config->name, I2C_NAME_SIZE);
    i2c_info.addr = bl_ext_config->i2c_addr;
    i2c_info.platform_data = bl_ext_config;
    i2c_info.flags=0;
    if(i2c_info.addr>0x7f)
        i2c_info.flags=0x10;
    i2c_client = i2c_new_device(adapter, &i2c_info);
    if (!i2c_client) {
        printk("[error] %s :failed to new i2c device\n", BL_EXTERN_NAME);
        goto bl_extern_probe_failed;
    }
    else{
        DBG_PRINT("[error] %s: new i2c device succeed\n",BL_EXTERN_NAME);
    }

    if (!aml_mp3309c_i2c_client) {
        ret = i2c_add_driver(&aml_mp3309c_i2c_driver);
        if (ret) {
            printk("[error] %s probe: add i2c_driver failed\n", BL_EXTERN_NAME);
            goto bl_extern_probe_failed;
        }
    }

    printk("%s ok\n", __FUNCTION__);
    return ret;

bl_extern_probe_failed:
    if (bl_ext_config) {
        kfree(bl_ext_config);
        bl_ext_config = NULL;
    }
    return -1;
}

static int aml_mp3309c_remove(struct platform_device *pdev)
{
    if (pdev->dev.platform_data)
        kfree (pdev->dev.platform_data);
    return 0;
}

#ifdef CONFIG_USE_OF
static const struct of_device_id aml_mp3309c_dt_match[]={
    {
        .compatible = "amlogic,bl_i2c_mp3309c",
    },
    {},
};
#else
#define aml_mp3309c_dt_match NULL
#endif

static struct platform_driver aml_mp3309c_driver = {
    .probe  = aml_mp3309c_probe,
    .remove = aml_mp3309c_remove,
    .driver = {
        .name  = BL_EXTERN_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_USE_OF
        .of_match_table = aml_mp3309c_dt_match,
#endif
    },
};

static int __init aml_mp3309c_init(void)
{
    int ret;
    DBG_PRINT("%s\n", __FUNCTION__);

    ret = platform_driver_register(&aml_mp3309c_driver);
    if (ret) {
        printk("[error] %s failed to register bl extern driver module\n", __FUNCTION__);
        return -ENODEV;
    }
    return ret;
}

static void __exit aml_mp3309c_exit(void)
{
    platform_driver_unregister(&aml_mp3309c_driver);
}

//late_initcall(aml_mp3309c_init);
module_init(aml_mp3309c_init);
module_exit(aml_mp3309c_exit);

MODULE_AUTHOR("AMLOGIC");
MODULE_DESCRIPTION("BL Extern driver for MP3309C");
MODULE_LICENSE("GPL");
