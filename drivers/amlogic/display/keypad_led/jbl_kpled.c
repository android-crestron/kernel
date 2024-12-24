/*
 * AMLOGIC backlight driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Author:  Wang Han <han.wang@amlogic.com>
 *  
 * Modify:  Evoke Zhang <evoke.zhang@amlogic.com>
 * Modify:  Chris Chris <chris_chen@jabil.com>
 * compatible dts
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/backlight.h>
#include <linux/slab.h>
#include <linux/amlogic/jbl_kpled.h>
#include <linux/workqueue.h>
#include <mach/power_gate.h>
#ifdef CONFIG_ARCH_MESON6
#include <mach/mod_gate.h>
#endif /* CONFIG_ARCH_MESON6 */
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include <linux/amlogic/vout/lcdoutc.h>

//#define MESON_LED_DEBUG
#ifdef MESON_LED_DEBUG
#define DPRINT(...) printk(KERN_INFO __VA_ARGS__)
#define DTRACE()    DPRINT(KERN_INFO "%s()\n", __FUNCTION__)
static const char* kpled_ctrl_method_table[]={
    "gpio",
    "pwm_negative",
    "pwm_positive",
    "pwm_combo",
    "extern",
    "null"
};
#else
#define DPRINT(...)
#define DTRACE()
#endif /* MESON_LED_DEBUG */


#define KPLED_LEVEL_MAX    		255
#define KPLED_LEVEL_MIN    		10
#define KPLED_LEVEL_OFF			1

#define KPLED_LEVEL_MID    		128
#define KPLED_LEVEL_MID_MAPPED		102

#define KPLED_LEVEL_DEFAULT						KPLED_LEVEL_MID
#define KPLED_NAME 								"kpled"
#define kpled_gpio_request(gpio) 				amlogic_gpio_request(gpio, KPLED_NAME)
#define kpled_gpio_free(gpio) 					amlogic_gpio_free(gpio, KPLED_NAME)
#define kpled_gpio_direction_input(gpio) 		amlogic_gpio_direction_input(gpio, KPLED_NAME)
#define kpled_gpio_direction_output(gpio, val) 	amlogic_gpio_direction_output(gpio, val, KPLED_NAME)
#define kpled_gpio_get_value(gpio) 				amlogic_get_value(gpio, KPLED_NAME)
#define kpled_gpio_set_value(gpio,val) 			amlogic_set_value(gpio, val, KPLED_NAME)
#define kpled_gpio_export(gpio, val) 			amlogic_gpio_export(gpio, val, KPLED_NAME)

#ifdef CONFIG_JBL_KEYPAD_LED_SUPPORT
/* for keypad led power */
typedef enum {
    KPLED_CTL_GPIO = 0,
    KPLED_CTL_PWM_NEGATIVE = 1,
    KPLED_CTL_PWM_POSITIVE = 2,
    KPLED_CTL_PWM_COMBO = 3,
    KPLED_CTL_EXTERN = 4,
    KPLED_CTL_MAX = 5,
} KPLED_Ctrl_Method_t;


typedef enum {
    KPLED_PWM_A = 0,
    KPLED_PWM_B,
    KPLED_PWM_C,
    KPLED_PWM_D,
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
    KPLED_PWM_E,
    KPLED_PWM_F,
#endif
    KPLED_PWM_MAX,
} KPLED_PWM_t;

#define  KPLED_NUM_MAX 5

typedef struct {
    unsigned level_default;
    unsigned level_mid;
    unsigned level_mid_mapping;
    unsigned level_min;
    unsigned level_max;
    unsigned short power_on_delay;
    unsigned char method;

    int gpio[KPLED_NUM_MAX];
    unsigned char gpio_on;
    unsigned char gpio_off;
    unsigned dim_max;
    unsigned dim_min;
    unsigned char pwm_port;
    unsigned char pwm_gpio_used;
    unsigned pwm_cnt;
    unsigned pwm_pre_div;
    unsigned pwm_max;
    unsigned pwm_min;

    unsigned combo_level_switch;
    unsigned char combo_high_port;
    unsigned char combo_high_method;
    unsigned char combo_low_port;
    unsigned char combo_low_method;
    unsigned combo_high_cnt;
    unsigned combo_high_pre_div;
    unsigned combo_high_duty_max;
    unsigned combo_high_duty_min;
    unsigned combo_low_cnt;
    unsigned combo_low_pre_div;
    unsigned combo_low_duty_max;
    unsigned combo_low_duty_min;

    struct pinctrl *p;
    struct workqueue_struct *workqueue;
    struct delayed_work kpled_delayed_work;
} Keypad_Led_Config_t;

static Keypad_Led_Config_t kpled_config = {
    .level_default = 128,
    .level_mid = 128,
    .level_mid_mapping = 128,
    .level_min = 10,
    .level_max = 255,
    .power_on_delay = 100,
    .method = KPLED_CTL_MAX,
};
static unsigned kpled_level = KPLED_LEVEL_DEFAULT;
static unsigned int kpled_real_status = 1;

#define KPLED_FIN_FREQ				(24 * 1000)

static DEFINE_MUTEX(kpled_power_mutex);
static void power_on_kpled(void)
{
    struct pinctrl_state *s;
    int ret;
    int i;

    mutex_lock(&kpled_power_mutex);

    DPRINT("%s: kpled_real_status=%u\n", __FUNCTION__, kpled_real_status);
    if (kpled_real_status == 1) {
        goto exit_power_on_kpled;
    }

    switch (kpled_config.method) {
        case KPLED_CTL_GPIO:
            //TBD
            break;
        case KPLED_CTL_PWM_NEGATIVE:
        case KPLED_CTL_PWM_POSITIVE:
            switch (kpled_config.pwm_port) {
                case KPLED_PWM_A:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, kpled_config.pwm_pre_div, 8, 7);  //pwm_a_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 4, 2);  //pwm_a_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 15, 1);  //pwm_a_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 0, 1);  //enable pwm_a
                    break;
                case KPLED_PWM_B:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, kpled_config.pwm_pre_div, 16, 7);  //pwm_b_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 6, 2);  //pwm_b_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 23, 1);  //pwm_b_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 1, 1);  //enable pwm_b
                    break;
                case KPLED_PWM_C:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, kpled_config.pwm_pre_div, 8, 7);  //pwm_c_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 4, 2);  //pwm_c_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 15, 1);  //pwm_c_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 0, 1);  //enable pwm_c
                    break;
                case KPLED_PWM_D:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, kpled_config.pwm_pre_div, 16, 7);  //pwm_d_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 6, 2);  //pwm_d_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 23, 1);  //pwm_d_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 1, 1);  //enable pwm_d
                    break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                case KPLED_PWM_E:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, kpled_config.pwm_pre_div, 8, 7);  //pwm_c_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 4, 2);  //pwm_c_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 15, 1);  //pwm_c_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 0, 1);  //enable pwm_c
                    break;
                case KPLED_PWM_F:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, kpled_config.pwm_pre_div, 16, 7);  //pwm_d_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 6, 2);  //pwm_d_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 23, 1);  //pwm_d_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 1, 1);  //enable pwm_d
                    break;
#endif
                default:
                    break;
            }

            if (IS_ERR(kpled_config.p)) {
                printk("set kpled pinmux error.\n");
                goto exit_power_on_kpled;
            }
            s = pinctrl_lookup_state(kpled_config.p, "default"); //select pinctrl
            if (IS_ERR(s)) {
                printk("set kpled pinmux error.\n");
                devm_pinctrl_put(kpled_config.p);
                goto exit_power_on_kpled;
            }

            ret = pinctrl_select_state(kpled_config.p, s); //set pinmux and lock pins
            if (ret < 0) {
                printk("set kpled pinmux error.\n");
                devm_pinctrl_put(kpled_config.p);
                goto exit_power_on_kpled;
            }
            mdelay(20);
            if (kpled_config.pwm_gpio_used) {
                for (i=0; i<KPLED_NUM_MAX; i++) {
                    if (kpled_config.gpio[i])
                        kpled_gpio_direction_output(kpled_config.gpio[i], kpled_config.gpio_on);
                }
            }
            break;
        case KPLED_CTL_PWM_COMBO:
            //TBD
            break;
        case KPLED_CTL_EXTERN:
            //TBD
            break;
        default:
            printk("wrong kpled control method\n");
            goto exit_power_on_kpled;
            break;
    }
    kpled_real_status = 1;
    printk("kpled power on\n");

exit_power_on_kpled:
    mutex_unlock(&kpled_power_mutex);
}

static void kpled_delayd_on(struct work_struct *work) //kpled_delayed_work
{
    power_on_kpled();
}

void kpled_power_on(void)
{
    DPRINT("%s: kpled_real_status=%s\n", __FUNCTION__, (kpled_real_status ? "ON" : "OFF"));
    if (kpled_config.method < KPLED_CTL_MAX) {
        power_on_kpled();
    }
    else {
        printk("wrong backlight control method\n");
    }

    DPRINT("kpled_power_on...\n");
}

void kpled_power_off(void)
{
    int i;

    mutex_lock(&kpled_power_mutex);

    DPRINT("%s: kpled_real_status=%u\n", __FUNCTION__, kpled_real_status);
    if (kpled_real_status == 0) {
        mutex_unlock(&kpled_power_mutex);
        return;
    }

    switch (kpled_config.method) {
        case KPLED_CTL_GPIO:
            //TBD
            break;
        case KPLED_CTL_PWM_NEGATIVE:
        case KPLED_CTL_PWM_POSITIVE:
            if (kpled_config.pwm_gpio_used) {
                for (i=0; i<KPLED_NUM_MAX; i++) {
                    if (kpled_config.gpio[i])
                        kpled_gpio_direction_output(kpled_config.gpio[i], kpled_config.gpio_off);
                }
            }
            switch (kpled_config.pwm_port) {
                case KPLED_PWM_A:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 0, 1);  //disable pwm_a
                    break;
                case KPLED_PWM_B:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 1, 1);  //disable pwm_b
                    break;
                case KPLED_PWM_C:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 0, 1);  //disable pwm_c
                    break;
                case KPLED_PWM_D:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 1, 1);  //disable pwm_d
                    break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                case KPLED_PWM_E:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 0, 1);  //disable pwm_c
                    break;
                case KPLED_PWM_F:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 1, 1);  //disable pwm_d
                    break;
#endif
                default:
                    break;
            }
            break;
        case KPLED_CTL_PWM_COMBO:
            //TBD
            break;
        case KPLED_CTL_EXTERN:
            //TBD
            break;
        default:
            break;
    }
    kpled_real_status = 0;
    printk("kpled power off\n");
    mutex_unlock(&kpled_power_mutex);
}

#if (MESON_CPU_TYPE != MESON_CPU_TYPE_MESON6TV)&&(MESON_CPU_TYPE != MESON_CPU_TYPE_MESON6TVD)
static DEFINE_MUTEX(kpled_level_mutex);
static void set_keypad_led_level(unsigned level)
{
    unsigned pwm_hi = 0, pwm_lo = 0;

    mutex_lock(&kpled_level_mutex);

    DPRINT("set_keypad_led_level: %u, last level: %u, kpled_real_status: %u\n", level, kpled_level, kpled_real_status);
    level = (level > kpled_config.level_max ? kpled_config.level_max : (level < kpled_config.level_min ? (level < KPLED_LEVEL_OFF ? 0 : kpled_config.level_min) : level));
    kpled_level = level;

    if (kpled_level == 0) {
        if (kpled_real_status == 1)
            kpled_power_off();
    }
    else {
        //mapping
        if (level > kpled_config.level_mid)
            level = ((level - kpled_config.level_mid) * (kpled_config.level_max - kpled_config.level_mid_mapping)) / (kpled_config.level_max - kpled_config.level_mid) + kpled_config.level_mid_mapping;
        else
            level = ((level - kpled_config.level_min) * (kpled_config.level_mid_mapping - kpled_config.level_min)) / (kpled_config.level_mid - kpled_config.level_min) + kpled_config.level_min;
        DPRINT("level mapping=%u\n", level);

        switch (kpled_config.method) {
            case KPLED_CTL_GPIO:
                //TBD
                break;
            case KPLED_CTL_PWM_NEGATIVE:
            case KPLED_CTL_PWM_POSITIVE:
                level = (kpled_config.pwm_max - kpled_config.pwm_min) * (level - kpled_config.level_min) / (kpled_config.level_max - kpled_config.level_min) + kpled_config.pwm_min;
                if (kpled_config.method == KPLED_CTL_PWM_NEGATIVE) {
                    pwm_hi = kpled_config.pwm_cnt - level;
                    pwm_lo = level;
                }
                else {
                    pwm_hi = level;
                    pwm_lo = kpled_config.pwm_cnt - level;
                }
                switch (kpled_config.pwm_port) {
                    case KPLED_PWM_A:
                        aml_write_reg32(P_PWM_PWM_A, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_B:
                        aml_write_reg32(P_PWM_PWM_B, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_C:
                        aml_write_reg32(P_PWM_PWM_C, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_D:
                        aml_write_reg32(P_PWM_PWM_D, (pwm_hi << 16) | (pwm_lo));
                        break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                    case KPLED_PWM_E:
                        aml_write_reg32(P_PWM_PWM_E, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case KPLED_PWM_F:
                        aml_write_reg32(P_PWM_PWM_F, (pwm_hi << 16) | (pwm_lo));
                        break;
#endif
                    default:
                        break;
                }
                break;
            case KPLED_CTL_PWM_COMBO:
                //TBD
                break;
            case KPLED_CTL_EXTERN:
                //TBD
                break;
            default:
                break;
        }
        if (kpled_real_status == 0)
            kpled_power_on();
    }
    mutex_unlock(&kpled_level_mutex);
}

unsigned get_keypad_led_level(void)
{
    DPRINT("%s: %d\n", __FUNCTION__, kpled_level);
    return kpled_level;
}
#endif
#endif

struct aml_kpled {
    const struct aml_kpled_platform_data   *pdata;
    struct platform_device          *pdev;
};

#ifdef CONFIG_USE_OF
static struct aml_kpled_platform_data meson_keypad_led_platform =
{
    //.power_on_kpled = power_on_keypad_led,
    //.power_off_kpled = power_off_keypad_led,
    .get_kpled_level = get_keypad_led_level,
    .set_kpled_level = set_keypad_led_level,
    .max_brightness = KPLED_LEVEL_MAX,
    .dft_brightness = KPLED_LEVEL_DEFAULT,
};

#define AMLOGIC_KPLED_DRV_DATA ((kernel_ulong_t)&meson_keypad_led_platform)

static const struct of_device_id keypad_led_dt_match[] = {
    {
        .compatible = "amlogic,keypad_led",
        .data = (void *)AMLOGIC_KPLED_DRV_DATA
    },
    {},
};
#else
#define keypad_led_dt_match NULL
#endif

#ifdef CONFIG_USE_OF
static inline struct aml_kpled_platform_data *kpled_get_driver_data(struct platform_device *pdev)
{
    const struct of_device_id *match;

    if(pdev->dev.of_node) {
        //DPRINT("***of_device: get backlight driver data.***\n");
        match = of_match_node(keypad_led_dt_match, pdev->dev.of_node);
        return (struct aml_kpled_platform_data *)match->data;
    }
    return NULL;
}
#endif

#ifdef CONFIG_JBL_KEYPAD_LED_SUPPORT
static inline int _get_keypad_led_config(struct platform_device *pdev)
{
    int ret=0;
    int val;
    const char *str;
    unsigned int kpled_para[3];
    unsigned pwm_freq=0, pwm_cnt, pwm_pre_div=0;
    int i;

    if (pdev->dev.of_node) {
        ret = of_property_read_u32_array(pdev->dev.of_node,"kpled_level_default_uboot_kernel", &kpled_para[0], 2);
        if(ret){
            printk("faild to get kpled_level_default_uboot_kernel\n");
            kpled_config.level_default = KPLED_LEVEL_DEFAULT;
        }
        else {
            kpled_config.level_default = kpled_para[1];
        }
        DPRINT("kpled level default kernel=%u\n", kpled_config.level_default);
        ret = of_property_read_u32_array(pdev->dev.of_node, "kpled_level_middle_mapping", &kpled_para[0], 2);
        if (ret) {
            printk("faild to get kpled_level_middle_mapping!\n");
            kpled_config.level_mid = KPLED_LEVEL_MID;
            kpled_config.level_mid_mapping = KPLED_LEVEL_MID_MAPPED;
        }
        else {
            kpled_config.level_mid = kpled_para[0];
            kpled_config.level_mid_mapping = kpled_para[1];
        }
        DPRINT("kpled level mid=%u, mid_mapping=%u\n", kpled_config.level_mid, kpled_config.level_mid_mapping);
        ret = of_property_read_u32_array(pdev->dev.of_node,"kpled_level_max_min", &kpled_para[0],2);
        if(ret){
            printk("faild to get kpled_level_max_min\n");
            kpled_config.level_min = KPLED_LEVEL_MIN;
            kpled_config.level_max = KPLED_LEVEL_MAX;
        }
        else {
            kpled_config.level_max = kpled_para[0];
            kpled_config.level_min = kpled_para[1];
        }
        DPRINT("kpled level max=%u, min=%u\n", kpled_config.level_max, kpled_config.level_min);

        ret = of_property_read_u32(pdev->dev.of_node, "kpled_power_on_delay", &val);
        if (ret) {
            printk("faild to get kpled_power_on_delay\n");
            kpled_config.power_on_delay = 100;
        }
        else {
            val = val & 0xffff;
            kpled_config.power_on_delay = (unsigned short)val;
        }
        DPRINT("kpled power_on_delay: %ums\n", kpled_config.power_on_delay);
        ret = of_property_read_u32(pdev->dev.of_node, "kpled_ctrl_method", &val);
        if (ret) {
            printk("faild to get kpled_ctrl_method\n");
            kpled_config.method = KPLED_CTL_MAX;
        }
        else {
            val = (val >= KPLED_CTL_MAX) ? KPLED_CTL_MAX : val;
            kpled_config.method = (unsigned char)val;
        }
        DPRINT("kpled control_method: %s(%u)\n", kpled_ctrl_method_table[kpled_config.method], kpled_config.method);

        if (kpled_config.method == KPLED_CTL_GPIO) {
#if 0
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_gpio_port_on_off", 0, &str);
            if (ret) {
                printk("faild to get bl_gpio_port_on_off!\n");
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
                str = "GPIOD_1";
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
                str = "GPIODV_28";
#endif
            }
            val = amlogic_gpio_name_map_num(str);
            if (val > 0) {
                ret = bl_gpio_request(val);
                if (ret) {
                    printk("faild to alloc bl gpio (%s)!\n", str);
                }
                kpled_config.gpio = val;                
                DPRINT("bl gpio = %s(%d)\n", str, kpled_config.gpio);
            }
            else {
                kpled_config.gpio = -1;
            }
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_gpio_port_on_off", 1, &str);
            if (ret) {
                printk("faild to get bl_gpio_port_on!\n");
                kpled_config.gpio_on = LCD_POWER_GPIO_OUTPUT_HIGH;
            }
            else {
                if (strcmp(str, "2") == 0)
                    kpled_config.gpio_on = LCD_POWER_GPIO_INPUT;
                else if(strcmp(str, "0") == 0)
                    kpled_config.gpio_on = LCD_POWER_GPIO_OUTPUT_LOW;
                else
                    kpled_config.gpio_on = LCD_POWER_GPIO_OUTPUT_HIGH;
            }
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_gpio_port_on_off", 2, &str);
            if (ret) {
                printk("faild to get bl_gpio_port_off!\n");
                kpled_config.gpio_off = LCD_POWER_GPIO_OUTPUT_LOW;
            }
            else {
                if (strcmp(str, "2") == 0)
                    kpled_config.gpio_off = LCD_POWER_GPIO_INPUT;
                else if(strcmp(str, "1") == 0)
                    kpled_config.gpio_off = LCD_POWER_GPIO_OUTPUT_HIGH;
                else
                    kpled_config.gpio_off = LCD_POWER_GPIO_OUTPUT_LOW;
            }
            DPRINT("bl gpio_on=%u, bl gpio_off=%u\n", kpled_config.gpio_on, kpled_config.gpio_off);
            ret = of_property_read_u32_array(pdev->dev.of_node,"bl_gpio_dim_max_min",&kpled_para[0],2);
            if (ret) {
                printk("faild to get bl_gpio_dim_max_min\n");
                kpled_config.dim_max = 0x0;
                kpled_config.dim_min = 0xf;
            }
            else {
                kpled_config.dim_max = kpled_para[0];
                kpled_config.dim_min = kpled_para[1];
            }
            DPRINT("bl dim max=%u, min=%u\n", kpled_config.dim_max, kpled_config.dim_min);
#endif
        }
        else if ((kpled_config.method == KPLED_CTL_PWM_NEGATIVE) || (kpled_config.method == KPLED_CTL_PWM_POSITIVE)) {
            ret = of_property_read_string_index(pdev->dev.of_node, "kpled_pwm_port_gpio_used", 1, &str);
            if (ret) {
                printk("faild to get kpled_pwm_port_gpio_used!\n");
                kpled_config.pwm_gpio_used = 0;
            }
            else {
                if (strncmp(str, "1", 1) == 0)
                    kpled_config.pwm_gpio_used = 1;
                else
                    kpled_config.pwm_gpio_used = 0;
                DPRINT("kpled_pwm gpio_used: %u\n", kpled_config.pwm_gpio_used);
            }
            if (kpled_config.pwm_gpio_used == 1) {
                ret = of_property_read_string(pdev->dev.of_node, "kpled_gpio_port_on_off", &str);
                if (ret) {
                    printk("faild to get kpled_gpio_port_on_off!\n");
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
                    str = "GPIOD_1";
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
                    str = "GPIODV_28";
#endif
                }
                for (i=0; i<KPLED_NUM_MAX; i++) {
                    val = amlogic_gpio_name_map_num(str);
                    if (val > 0) {
                        ret = kpled_gpio_request(val);
                        if (ret) {
                          printk("faild to alloc kpled gpio (%s)!\n", str);
                        }
                        ret = kpled_gpio_export(val, 1);
                        if (ret) {
                          printk("faild to export kpled gpio (%s)!\n", str);
                        }
                        kpled_config.gpio[i] = val;
                        DPRINT("kpled gpio[%d] = %s(%d)\n", i, str, kpled_config.gpio[i]);
                    }
                    else {
                        kpled_config.gpio[i] = -1;
                    }
                    str += strlen(str) + 1;
                }
                          ret = of_property_read_string_index(pdev->dev.of_node, "kpled_gpio_port_on_off", 5, &str);
            if (ret) {
                printk("faild to get kpled_gpio_port_on!\n");
                kpled_config.gpio_on = LCD_POWER_GPIO_OUTPUT_LOW;
            }
            else {
                if (strcmp(str, "2") == 0)
                    kpled_config.gpio_on = LCD_POWER_GPIO_INPUT;
                else if(strcmp(str, "0") == 0)
                    kpled_config.gpio_on = LCD_POWER_GPIO_OUTPUT_LOW;
                else
                    kpled_config.gpio_on = LCD_POWER_GPIO_OUTPUT_HIGH;
            }
            ret = of_property_read_string_index(pdev->dev.of_node, "kpled_gpio_port_on_off", 6, &str);
            if (ret) {
                printk("faild to get kpled_gpio_port_off!\n");
                kpled_config.gpio_off = LCD_POWER_GPIO_OUTPUT_HIGH;
            }
            else {
                if (strcmp(str, "2") == 0)
                    kpled_config.gpio_off = LCD_POWER_GPIO_INPUT;
                else if(strcmp(str, "1") == 0)
                    kpled_config.gpio_off = LCD_POWER_GPIO_OUTPUT_HIGH;
                else
                    kpled_config.gpio_off = LCD_POWER_GPIO_OUTPUT_LOW;
            }
            DPRINT("gpio_on=%u, gpio_off=%u\n", kpled_config.gpio_on, kpled_config.gpio_off);
          }
            ret = of_property_read_string_index(pdev->dev.of_node, "kpled_pwm_port_gpio_used", 0, &str);
            if (ret) {
                printk("faild to get kpled_pwm_port_gpio_used!\n");
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
                kpled_config.pwm_port = KPLED_PWM_D;
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
                kpled_config.pwm_port = KPLED_PWM_C;
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B)
                kpled_config.pwm_port = KPLED_PWM_MAX;
#endif
            }
            else {
                if (strcmp(str, "PWM_A") == 0)
                    kpled_config.pwm_port = KPLED_PWM_A;
                else if (strcmp(str, "PWM_B") == 0)
                    kpled_config.pwm_port = KPLED_PWM_B;
                else if (strcmp(str, "PWM_C") == 0)
                    kpled_config.pwm_port = KPLED_PWM_C;
                else if (strcmp(str, "PWM_D") == 0)
                    kpled_config.pwm_port = KPLED_PWM_D;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                else if (strcmp(str, "PWM_E") == 0)
                    kpled_config.pwm_port = KPLED_PWM_E;
                else if (strcmp(str, "PWM_F") == 0)
                    kpled_config.pwm_port = KPLED_PWM_F;
#endif
                else
                    kpled_config.pwm_port = KPLED_PWM_MAX;
                DPRINT("kpled pwm_port: %s(%u)\n", str, kpled_config.pwm_port);
            }
            ret = of_property_read_u32(pdev->dev.of_node,"kpled_pwm_freq",&val);
            if (ret) {
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
                pwm_freq = 1000;
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
                pwm_freq = 300000;
#endif
                printk("faild to get kpled_pwm_freq, default set to %u\n", pwm_freq);
            }
            else {
                pwm_freq = ((val >= (KPLED_FIN_FREQ * 500)) ? (KPLED_FIN_FREQ * 500) : val);
            }
            if (pwm_freq > 366 ) {
                for (i=0; i<0x7f; i++) {
                    pwm_pre_div = i;
                    pwm_cnt = KPLED_FIN_FREQ * 1000 / (pwm_freq * (pwm_pre_div + 1)) - 2;
                    if (pwm_cnt <= 0xffff)
                        break;
                }
            }
            else
                pwm_cnt = 0xffff;
            kpled_config.pwm_cnt = pwm_cnt;
            kpled_config.pwm_pre_div = pwm_pre_div;
            DPRINT("kpled pwm_frequency=%u, cnt=%u, div=%u\n", pwm_freq, kpled_config.pwm_cnt, kpled_config.pwm_pre_div);
            ret = of_property_read_u32_array(pdev->dev.of_node,"kpled_pwm_duty_max_min",&kpled_para[0],2);
            if (ret) {
                printk("faild to get kpled_pwm_duty_max_min\n");
                kpled_para[0] = 100;
                kpled_para[1] = 20;
            }
            kpled_config.pwm_max = (kpled_config.pwm_cnt * kpled_para[0] / 100);
            kpled_config.pwm_min = (kpled_config.pwm_cnt * kpled_para[1] / 100);
            DPRINT("kpled pwm_duty max=%u%%, min=%u%%\n", kpled_para[0], kpled_para[1]);
        }
        else if (kpled_config.method == KPLED_CTL_PWM_COMBO) {
#if 0
            ret = of_property_read_u32(pdev->dev.of_node,"bl_pwm_combo_high_low_level_switch",&val);
            if (ret) {
                printk("faild to get bl_pwm_combo_high_low_level_switch\n");
                val = kpled_config.level_mid;
            }
            if (val > kpled_config.level_mid)
                val = ((val - kpled_config.level_mid) * (kpled_config.level_max - kpled_config.level_mid_mapping)) / (kpled_config.level_max - kpled_config.level_mid) + kpled_config.level_mid_mapping;
            else
                val = ((val - kpled_config.level_min) * (kpled_config.level_mid_mapping - kpled_config.level_min)) / (kpled_config.level_mid - kpled_config.level_min) + kpled_config.level_min;
            kpled_config.combo_level_switch = val;
            DPRINT("bl pwm_combo level switch =%u\n", kpled_config.combo_level_switch);
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_pwm_combo_high_port_method", 0, &str);
            if (ret) {
                printk("faild to get bl_pwm_combo_high_port_method!\n");
                str = "PWM_C";
                kpled_config.combo_high_port = BL_PWM_C;
            }
            else {
                if (strcmp(str, "PWM_A") == 0)
                    kpled_config.combo_high_port = BL_PWM_A;
                else if (strcmp(str, "PWM_B") == 0)
                    kpled_config.combo_high_port = BL_PWM_B;
                else if (strcmp(str, "PWM_C") == 0)
                    kpled_config.combo_high_port = BL_PWM_C;
                else if (strcmp(str, "PWM_D") == 0)
                    kpled_config.combo_high_port = BL_PWM_D;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                else if (strcmp(str, "PWM_E") == 0)
                    kpled_config.pwm_port = BL_PWM_E;
                else if (strcmp(str, "PWM_F") == 0)
                    kpled_config.pwm_port = BL_PWM_F;
#endif
                else
                    kpled_config.combo_high_port = BL_PWM_MAX;
            }
            DPRINT("bl pwm_combo high port: %s(%u)\n", str, kpled_config.combo_high_port);
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_pwm_combo_high_port_method", 1, &str);
            if (ret) {
                printk("faild to get bl_pwm_combo_high_port_method!\n");
                str = "1";
                kpled_config.combo_high_method = BL_CTL_PWM_NEGATIVE;
            }
            else {
                if (strncmp(str, "1", 1) == 0)
                    kpled_config.combo_high_method = BL_CTL_PWM_NEGATIVE;
                else
                    kpled_config.combo_high_method = BL_CTL_PWM_POSITIVE;
            }
            DPRINT("bl pwm_combo high method: %s(%u)\n", kpled_ctrl_method_table[kpled_config.combo_high_method], kpled_config.combo_high_method);
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_pwm_combo_low_port_method", 0, &str);
            if (ret) {
                printk("faild to get bl_pwm_combo_low_port_method!\n");
                str = "PWM_D";
                kpled_config.combo_low_port = BL_PWM_D;
            }
            else {
                if (strcmp(str, "PWM_A") == 0)
                    kpled_config.combo_low_port = BL_PWM_A;
                else if (strcmp(str, "PWM_B") == 0)
                    kpled_config.combo_low_port = BL_PWM_B;
                else if (strcmp(str, "PWM_C") == 0)
                    kpled_config.combo_low_port = BL_PWM_C;
                else if (strcmp(str, "PWM_D") == 0)
                    kpled_config.combo_low_port = BL_PWM_D;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                else if (strcmp(str, "PWM_E") == 0)
                    kpled_config.pwm_port = BL_PWM_E;
                else if (strcmp(str, "PWM_F") == 0)
                    kpled_config.pwm_port = BL_PWM_F;
#endif
                else
                    kpled_config.combo_low_port = BL_PWM_MAX;
            }
            DPRINT("bl pwm_combo high port: %s(%u)\n", str, kpled_config.combo_low_port);
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_pwm_combo_low_port_method", 1, &str);
            if (ret) {
                printk("faild to get bl_pwm_combo_low_port_method!\n");
                str = "1";
                kpled_config.combo_low_method = BL_CTL_PWM_NEGATIVE;
            }
            else {
                if (strncmp(str, "1", 1) == 0)
                    kpled_config.combo_low_method = BL_CTL_PWM_NEGATIVE;
                else
                    kpled_config.combo_low_method = BL_CTL_PWM_POSITIVE;
            }
            DPRINT("bl pwm_combo low method: %s(%u)\n", kpled_ctrl_method_table[kpled_config.combo_low_method], kpled_config.combo_low_method);
            ret = of_property_read_u32_array(pdev->dev.of_node,"bl_pwm_combo_high_freq_duty_max_min",&kpled_para[0],3);
            if (ret) {
                printk("faild to get bl_pwm_combo_high_freq_duty_max_min\n");
                kpled_para[0] = 300000;  //freq=300k
                kpled_para[1] = 100;
                kpled_para[2] = 50;
            }
            pwm_freq = ((kpled_para[0] >= (FIN_FREQ * 500)) ? (FIN_FREQ * 500) : kpled_para[0]);
            for (i=0; i<0x7f; i++) {
                pwm_pre_div = i;
                pwm_cnt = FIN_FREQ * 1000 / (pwm_freq * (pwm_pre_div + 1)) - 2;
                if (pwm_cnt <= 0xffff)
                    break;
            }
            kpled_config.combo_high_cnt = pwm_cnt;
            kpled_config.combo_high_pre_div = pwm_pre_div;
            kpled_config.combo_high_duty_max = (kpled_config.combo_high_cnt * kpled_para[1] / 100);
            kpled_config.combo_high_duty_min = (kpled_config.combo_high_cnt * kpled_para[2] / 100);
            DPRINT("bl pwm_combo high freq=%uHz, duty_max=%u%%, duty_min=%u%%\n", pwm_freq, kpled_para[1], kpled_para[2]);
            ret = of_property_read_u32_array(pdev->dev.of_node,"bl_pwm_combo_low_freq_duty_max_min",&kpled_para[0],3);
            if (ret) {
                printk("faild to get bl_pwm_combo_low_freq_duty_max_min\n");
                kpled_para[0] = 1000;    //freq=1k
                kpled_para[1] = 100;
                kpled_para[2] = 50;
            }
            pwm_freq = ((kpled_para[0] >= (FIN_FREQ * 500)) ? (FIN_FREQ * 500) : kpled_para[0]);
            for (i=0; i<0x7f; i++) {
                pwm_pre_div = i;
                pwm_cnt = FIN_FREQ * 1000 / (pwm_freq * (pwm_pre_div + 1)) - 2;
                if (pwm_cnt <= 0xffff)
                    break;
            }
            kpled_config.combo_low_cnt = pwm_cnt;
            kpled_config.combo_low_pre_div = pwm_pre_div;
            kpled_config.combo_low_duty_max = (kpled_config.combo_low_cnt * kpled_para[1] / 100);
            kpled_config.combo_low_duty_min = (kpled_config.combo_low_cnt * kpled_para[2] / 100);
            DPRINT("bl pwm_combo low freq=%uHz, duty_max=%u%%, duty_min=%u%%\n", pwm_freq, kpled_para[1], kpled_para[2]);
#endif
        }

        //pinmux
        kpled_config.p = devm_pinctrl_get(&pdev->dev);
        if (IS_ERR(kpled_config.p))
            printk("get keypad led pinmux error.\n");
    }
    return ret;
}
#endif

//****************************
#ifdef CONFIG_JBL_KEYPAD_LED_SUPPORT
static struct class *kpled_debug_class = NULL;
static ssize_t kpled_status_read(struct class *class, struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "read kpled status: kpled_real_status=%s(%d), kpled_level=%d\n", (kpled_real_status ? "ON" : "OFF"), kpled_real_status, 
						 kpled_level);
}

static ssize_t kpled_show_brightness(struct class *class, struct class_attribute *attr, char *buf)
{
	unsigned brightness;

	brightness = get_keypad_led_level();
	return sprintf(buf, "%d\n", brightness);
}

static ssize_t kpled_store_brightness(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	int rc;
	unsigned brightness;

	brightness = simple_strtol(buf, NULL, 0);

	if (brightness > KPLED_LEVEL_MAX)
		rc = -EINVAL;
	else {
		DPRINT("set brightness to %d\n", brightness);
		kpled_level = brightness;
		set_keypad_led_level(brightness);
		rc = count;
	}

	return rc;
}

static ssize_t kpled_store_onoff(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	int argn;
	char * buf_work,*p,*para;
	char cmd;
	char * argv[3];
	int led_num;
	
	buf_work = kstrdup(buf, GFP_KERNEL);
	p = buf_work;
	
	for(argn = 0; argn < 3; argn++){
		para = strsep(&p," ");
		if(para == NULL)
			break;
		argv[argn] = para;
		DPRINT("argv[%d] = %s\n",argn, para);
	}
	
	if(argn < 1 || argn > 3)
		goto end;

	cmd = argv[0][3];
	switch (cmd){
	case '1':
		led_num = 0;
		break;
	case '2':
		led_num = 1;
		break;
	case '3':
		led_num = 2;
		break;
	case '4':
		led_num = 3;
		break;
	case '5':
		led_num = 4;
		break;
	default:
		printk("wrong control value, should be [led1, led2, led3, led4, led5].\n");
		rc = 1;
		break;
	}
	if (rc == 1)
		goto end;

	cmd = argv[1][0];
	switch (cmd) {
		case '0':
			kpled_gpio_direction_output(kpled_config.gpio[led_num], kpled_config.gpio_off);
			break;
		case '1':
			kpled_gpio_direction_output(kpled_config.gpio[led_num], kpled_config.gpio_on);
			break;
		default:
			printk("wrong control value, should be [0, 1].\n");
			rc = 1;
			break;
	}
	if (rc == 1)
		goto end;

	return count;

end:
	printk("error command!\n");
	kfree(buf_work);
	return -EINVAL;	
}

static struct class_attribute kpled_debug_class_attrs[] = {
	__ATTR(status,  S_IRUGO | S_IWUSR, kpled_status_read, NULL),
	__ATTR(brightness, S_IRUGO | S_IWUSR, kpled_show_brightness, kpled_store_brightness),
	__ATTR(onoff, S_IRUGO | S_IWUSR, NULL, kpled_store_onoff),
};

static int creat_kpled_attr(void)
{
    int i;

    kpled_debug_class = class_create(THIS_MODULE, "keypad_led");
    if(IS_ERR(kpled_debug_class)) {
        printk("create keypad_led debug class fail\n");
        return -1;
    }

    for(i=0;i<ARRAY_SIZE(kpled_debug_class_attrs);i++) {
        if (class_create_file(kpled_debug_class, &kpled_debug_class_attrs[i])) {
            printk("create keypad_led debug attribute %s fail\n", kpled_debug_class_attrs[i].attr.name);
        }
    }

    return 0;
}

static int remove_kpled_attr(void)
{
    int i;

    if (kpled_debug_class == NULL)
        return -1;

    for(i=0;i<ARRAY_SIZE(kpled_debug_class_attrs);i++) {
        class_remove_file(kpled_debug_class, &kpled_debug_class_attrs[i]);
    }
    class_destroy(kpled_debug_class);
    kpled_debug_class = NULL;

    return 0;
}
#endif
//****************************

static int aml_kpled_probe(struct platform_device *pdev)
{
    const struct aml_kpled_platform_data *pdata;
    struct aml_kpled *amlkpled;
    int retval;

    DTRACE();

    amlkpled = kzalloc(sizeof(struct aml_kpled), GFP_KERNEL);
    if (!amlkpled)
    {
        printk(KERN_ERR "%s() kzalloc error\n", __FUNCTION__);
        return -ENOMEM;
    }

    amlkpled->pdev = pdev;

#ifdef CONFIG_USE_OF
    pdata = kpled_get_driver_data(pdev);
#else
    pdata = pdev->dev.platform_data;
#endif
    if (!pdata) {
        printk(KERN_ERR "%s() missing platform data\n", __FUNCTION__);
        retval = -ENODEV;
        goto err;
    }

#ifdef CONFIG_USE_OF
	_get_keypad_led_config(pdev);
#endif

    amlkpled->pdata = pdata;

    DPRINT("%s() pdata->kpled_init=%p\n", __FUNCTION__, pdata->kpled_init);
    DPRINT("%s() pdata->power_on_kpled=%p\n", __FUNCTION__, pdata->power_on_kpled);
    DPRINT("%s() pdata->power_off_kpled=%p\n", __FUNCTION__, pdata->power_off_kpled);
    DPRINT("%s() pdata->set_kpled_level=%p\n", __FUNCTION__, pdata->set_kpled_level);
    DPRINT("%s() pdata->get_kpled_level=%p\n", __FUNCTION__, pdata->get_kpled_level);
    DPRINT("%s() pdata->max_brightness=%d\n", __FUNCTION__, pdata->max_brightness);
    DPRINT("%s() pdata->dft_brightness=%d\n", __FUNCTION__, pdata->dft_brightness);

    platform_set_drvdata(pdev, amlkpled);

    //init workqueue
    INIT_DELAYED_WORK(&kpled_config.kpled_delayed_work, kpled_delayd_on);
    //kpled_config.workqueue = create_singlethread_workqueue("bl_power_on_queue");
    kpled_config.workqueue = create_workqueue("kpled_power_on_queue");
    if (kpled_config.workqueue == NULL) {
        printk("can't create kpled work queue\n");
    }

    if (pdata->kpled_init)
        pdata->kpled_init();
    if (pdata->power_on_kpled)
        pdata->power_on_kpled();
    if (pdata->set_kpled_level)
        pdata->set_kpled_level(kpled_config.level_default);

#ifdef CONFIG_JBL_KEYPAD_LED_SUPPORT
    creat_kpled_attr();
#endif

    printk("aml kpled probe OK.\n");
    return 0;

err:
    kfree(amlkpled);
    return retval;
}

static int __exit aml_kpled_remove(struct platform_device *pdev)
{
    struct aml_kpled *amlkpled = platform_get_drvdata(pdev);

    DTRACE();

#ifdef CONFIG_JBL_KEYPAD_LED_SUPPORT
    remove_kpled_attr();
#endif

    if (kpled_config.workqueue)
        destroy_workqueue(kpled_config.workqueue);

    platform_set_drvdata(pdev, NULL);
    kfree(amlkpled);

    return 0;
}

static struct platform_driver aml_kpled_driver = {
    .driver = {
        .name = "aml-kpled",
        .owner = THIS_MODULE,
#ifdef CONFIG_USE_OF
        .of_match_table = keypad_led_dt_match,
#endif
    },
    .probe = aml_kpled_probe,
    .remove = __exit_p(aml_kpled_remove),
};

static int __init aml_kpled_init(void)
{
    DTRACE();
    if (platform_driver_register(&aml_kpled_driver)) {
        printk("failed to register kpled driver module\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit aml_kpled_exit(void)
{
    DTRACE();
    platform_driver_unregister(&aml_kpled_driver);
}

module_init(aml_kpled_init);
module_exit(aml_kpled_exit);

MODULE_DESCRIPTION("Meson Keypad LED Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amlogic, Inc.");
