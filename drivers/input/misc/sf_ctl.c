/**
 * The device control driver for Sunwave's fingerprint sensor.
 *
 * Copyright (C) 2016 Sunwave Corporation. <http://www.sunwavecorp.com>
 * Copyright (C) 2016 Langson L. <mailto: liangzh@sunwavecorp.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
**/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/ioctl.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/printk.h>
#include <linux/proc_fs.h>

#include "sf_ctl.h"

#include <linux/fb.h>
#include <linux/notifier.h>

#define MODULE_NAME "sf_ctl"
#define xprintk(level, fmt, args...) printk(level MODULE_NAME": "fmt, ##args)

#define SF_VDD_MIN_UV 2800000
#define SF_VDD_MAX_UV 2800000
#define PROC_NAME  "fp_info"

#define ANDROID_WAKELOCK	1

#if ANDROID_WAKELOCK
#include <linux/wakelock.h>
#endif

/**
 * Define the driver version string.
 * There is NO need to modify 'rXXXX_yyyymmdd', it should be updated automatically
 * by the building script (see the 'Driver-revision' section in 'build.sh').
 */
#define SF_DRV_VERSION "v0.9.2-r1_201711018"
#define LONGQI_HAL_COMPATIBLE 1	//powerking add for longqi 2017.01.16

struct sf_ctl_device {
    struct miscdevice miscdev;
    int reset_num;
    int irq_num;
    int pwr_num;
    struct regulator* vdd;
    u8 isOpen;
    u8 isInitialize;
    struct work_struct work_queue;
    struct input_dev* input;
    struct wake_lock wakelock;
    struct notifier_block notifier;
};

typedef enum {
    SF_PIN_STATE_PWR__ON,
    SF_PIN_STATE_PWR_OFF,
    SF_PIN_STATE_RST_SET,
    SF_PIN_STATE_RST_CLR,
    SF_PIN_STATE_INT_SET,

    /* Array size */
    SF_PIN_STATE_MAX
} sf_pin_state_t;

static int sf_ctl_device_power(bool on);
static int sf_ctl_device_reset(void);
static void sf_ctl_device_event(struct work_struct* ws);
static irqreturn_t sf_ctl_device_irq(int irq, void* dev_id);
static int sf_ctl_report_key_event(struct input_dev* input, sf_key_event_t* kevent);
static const char* sf_ctl_get_version(void);
static long sf_ctl_ioctl(struct file* filp, unsigned int cmd, unsigned long arg);
static int sf_ctl_open(struct inode* inode, struct file* filp);
static int sf_ctl_release(struct inode* inode, struct file* filp);
static int sf_ctl_init_gpio_pins(void);
static int sf_ctl_init_input(void);
static int __init sf_ctl_driver_init(void);
static void __exit sf_ctl_driver_exit(void);
static struct proc_dir_entry *proc_entry=NULL;

static struct file_operations sf_ctl_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = sf_ctl_ioctl,
    .open           = sf_ctl_open,
    .release        = sf_ctl_release,
};

static struct sf_ctl_device sf_ctl_dev = {
    .miscdev = {
        .minor  = MISC_DYNAMIC_MINOR,
        .name   = "sunwave_fp",
        .fops   = &sf_ctl_fops,
    }, 0,
};

static int sf_ctl_init_gpio(void)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    if (gpio_is_valid(sf_ctl_dev.reset_num)) {
        err = gpio_request(sf_ctl_dev.reset_num, "sf-reset");

        if (err) {
            xprintk(KERN_ERR, "Could not request reset gpio.\n");
            return err;
        }
    }
    else {
        xprintk(KERN_ERR, "not valid reset gpio\n");
        return -EIO;
    }

    if (gpio_is_valid(sf_ctl_dev.irq_num)) {
        err = gpio_request(sf_ctl_dev.irq_num,"sf-irq");

        if (err) {
            xprintk(KERN_ERR, "Could not request irq gpio.\n");
            gpio_free(sf_ctl_dev.reset_num);
            return err;
        }
    }
    else {
        xprintk(KERN_ERR, "not valid irq gpio\n");
        gpio_free(sf_ctl_dev.reset_num);
        return -EIO;
    }

    pinctrl_gpio_direction_input(sf_ctl_dev.irq_num);

    xprintk(KERN_DEBUG, "%s(..) ok! exit.\n", __FUNCTION__);
    sf_ctl_dev.isInitialize = 1;

    return err;
}


static int sf_ctl_free_gpio(void)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    sf_ctl_dev.isInitialize = 0;

    if (gpio_is_valid(sf_ctl_dev.irq_num)) {
        free_irq(gpio_to_irq(sf_ctl_dev.irq_num), (void*)&sf_ctl_dev);
        gpio_free(sf_ctl_dev.irq_num);
    }

	if (gpio_is_valid(sf_ctl_dev.reset_num)) {
        gpio_free(sf_ctl_dev.reset_num);
    }
    xprintk(KERN_DEBUG, "%s(..) ok! exit.\n", __FUNCTION__);

    return err;
}


static int sf_ctl_device_power(bool on)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    return err;
}

static int sf_ctl_device_reset(void)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    if (sf_ctl_dev.reset_num == 0) {
        xprintk(KERN_ERR, "sf_ctl_dev.reset_num is not get.\n");
        return -1;
    }

    gpio_direction_output(sf_ctl_dev.reset_num, 1);
    msleep(1);
    gpio_set_value(sf_ctl_dev.reset_num, 0);
    msleep(100);
    gpio_set_value(sf_ctl_dev.reset_num, 1);
    return err;
}

static void sf_ctl_device_event(struct work_struct* ws)
{
    struct sf_ctl_device* sf_ctl_dev =
        container_of(ws, struct sf_ctl_device, work_queue);
    char* uevent_env[2] = { "SPI_STATE=finger", NULL };
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    kobject_uevent_env(&sf_ctl_dev->miscdev.this_device->kobj,
                       KOBJ_CHANGE, uevent_env);
}

static irqreturn_t sf_ctl_device_irq(int irq, void* dev_id)
{
    struct sf_ctl_device* sf_ctl_dev = (struct sf_ctl_device*)dev_id;
    disable_irq_nosync(irq);
    xprintk(KERN_ERR, "%s(irq = %d, ..) toggled.\n", __FUNCTION__, irq);
    schedule_work(&sf_ctl_dev->work_queue);
#if ANDROID_WAKELOCK
	wake_lock_timeout(&sf_ctl_dev->wakelock, msecs_to_jiffies(5000));
#endif
    enable_irq(irq);
    return IRQ_HANDLED;
}

static int sf_ctl_report_key_event(struct input_dev* input, sf_key_event_t* kevent)
{
    int err = 0;
    unsigned int key_code = KEY_UNKNOWN;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);

    switch (kevent->key) {
		case SF_KEY_HOME:	key_code = KEY_HOME;   break;
		case SF_KEY_MENU:	key_code = KEY_MENU;   break;
		case SF_KEY_BACK:	key_code = KEY_BACK;   break;
		case SF_KEY_F11:	key_code = KEY_F19;    break; //click
		case SF_KEY_F12:	key_code = KEY_F20;    break; //double click
		case SF_KEY_ENTER:	key_code = KEY_F21;    break; //long press
		case SF_KEY_UP: 	key_code = KEY_UP;     break;
		case SF_KEY_LEFT:	key_code = KEY_LEFT;   break;
		case SF_KEY_RIGHT:	key_code = KEY_RIGHT;  break;
		case SF_KEY_DOWN:	key_code = KEY_DOWN;   break;
		case SF_KEY_WAKEUP: key_code = KEY_WAKEUP; break;

		default: break;
    }

    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    xprintk(KERN_DEBUG, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static const char* sf_ctl_get_version(void)
{
    static char version[SF_DRV_VERSION_LEN] = {'\0', };
    strncpy(version, SF_DRV_VERSION, SF_DRV_VERSION_LEN);
    version[SF_DRV_VERSION_LEN - 1] = '\0';
    return (const char*)version;
}

static int sf_ctl_init_irq(void)
{
    int err = 0;
    //struct device_node* dev_node = NULL;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    /* Register interrupt callback. */
    err = request_irq(gpio_to_irq(sf_ctl_dev.irq_num), sf_ctl_device_irq,
                      IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "sf-irq", (void*)&sf_ctl_dev);

    if (err) {
        xprintk(KERN_ERR, "request_irq(..) = %d.\n", err);
    }

    enable_irq_wake(gpio_to_irq(sf_ctl_dev.irq_num));
    return err;
}

////////////////////////////////////////////////////////////////////////////////
extern void sf_spi_platform_free(void);
static int proc_show_ver(struct seq_file *file,void *v)
{
	seq_printf(file,"[Vendor]sw9651,Sunwave\n");
	return 0;
}

static int proc_open(struct inode *inode,struct file *file)
{
	printk("sw9651 proc_open\n");
	single_open(file,proc_show_ver,NULL);
	return 0;
}

static const struct file_operations proc_file_ops = {
	.owner = THIS_MODULE,
	.open = proc_open,
	.read = seq_read,
	.release = single_release,
};

static int sunwave_fb_notifier_callback(struct notifier_block *self,
                                        unsigned long event, void *data)
{
    static char screen_status[64] = {'\0'};
    char *screen_env[2] = { screen_status, NULL };
    struct sf_ctl_device *sf_ctl_dev;
    struct fb_event *evdata = data;
    unsigned int blank;
    int retval = 0;

    if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
        return 0;
    }

    sf_ctl_dev = container_of(self, struct sf_ctl_device, notifier);
    blank = *(int *)evdata->data;

    switch (blank) {
        case FB_BLANK_UNBLANK:
            sprintf(screen_status, "SCREEN_STATUS=%s", "ON");
            kobject_uevent_env(&sf_ctl_dev->miscdev.this_device->kobj, KOBJ_CHANGE, screen_env);
            break;

        case FB_BLANK_POWERDOWN:
            sprintf(screen_status, "SCREEN_STATUS=%s", "OFF");
            kobject_uevent_env(&sf_ctl_dev->miscdev.this_device->kobj, KOBJ_CHANGE, screen_env);
            break;

        default:
            break;
    }
    return retval;
}

////////////////////////////////////////////////////////////////////////////////
// struct file_operations fields.

static long sf_ctl_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    struct miscdevice* dev = (struct miscdevice*)filp->private_data;
    struct sf_ctl_device* sf_ctl_dev =
        container_of(dev, struct sf_ctl_device, miscdev);
    int err = 0;
    sf_key_event_t kevent;
    xprintk(KERN_DEBUG, "%s(cmd = 0x%08x, ..)\n", __FUNCTION__, cmd);

    switch (cmd) {
        case SF_IOC_INIT_DRIVER: {
            xprintk(KERN_INFO, "SF_IOC_INIT_DRIVER.\n");
            sf_ctl_init_gpio();
    sf_ctl_dev->notifier.notifier_call = sunwave_fb_notifier_callback;
    fb_register_client(&sf_ctl_dev->notifier);
            break;
        }

        case SF_IOC_DEINIT_DRIVER: {
            xprintk(KERN_INFO, "SF_IOC_DEINIT_DRIVER.\n");
            sf_ctl_free_gpio();
    fb_unregister_client(&sf_ctl_dev->notifier);
            break;
        }

        case SF_IOC_RESET_DEVICE: {
            sf_ctl_device_reset();
            break;
        }

        case SF_IOC_ENABLE_IRQ: {
            // TODO:
            break;
        }

        case SF_IOC_DISABLE_IRQ: {
            // TODO:
            break;
        }

        case SF_IOC_REQUEST_IRQ: {
            xprintk(KERN_INFO, "SF_IOC_REQUEST_IRQ.\n");
			sf_ctl_init_irq();
	if (NULL == proc_entry)
	{
		proc_entry = proc_create(PROC_NAME, 0777, NULL, &proc_file_ops);
		if (NULL == proc_entry)
		{
			printk("sw9651 Couldn't create proc entry!");
			err = -ENOMEM;	
		}
		else
		{
			printk("sw9651 Create proc entry success!");
		}
	}

            break;
        }

        case SF_IOC_ENABLE_SPI_CLK: {
            // TODO:
            break;
        }

        case SF_IOC_DISABLE_SPI_CLK: {
            // TODO:
            break;
        }

        case SF_IOC_ENABLE_POWER: {
            // TODO:
            break;
        }

        case SF_IOC_DISABLE_POWER: {
            // TODO:
            break;
        }

        case SF_IOC_REPORT_KEY_EVENT: {
            if (copy_from_user(&kevent, (sf_key_event_t*)arg, sizeof(sf_key_event_t))) {
                xprintk(KERN_ERR, "copy_from_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            err = sf_ctl_report_key_event(sf_ctl_dev->input, &kevent);
            break;
        }

        case SF_IOC_SYNC_CONFIG: {
            // TODO:
            break;
        }

        case SF_IOC_GET_VERSION: {
            if (copy_to_user((void*)arg, sf_ctl_get_version(), SF_DRV_VERSION_LEN)) {
                xprintk(KERN_ERR, "copy_to_user(..) failed.\n");
                err = (-EFAULT);
                break;
            }

            break;
        }

        default:
            err = (-EINVAL);
            break;
    }

    return err;
}

static int sf_ctl_open(struct inode* inode, struct file* filp)
{
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    sf_ctl_dev.isOpen++;
    return 0;
}

static int sf_ctl_release(struct inode* inode, struct file* filp)
{
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    sf_ctl_dev.isOpen--;

    if ((!sf_ctl_dev.isOpen) && (sf_ctl_dev.isInitialize == 1)) {
        sf_ctl_free_gpio();
    }
    return 0;
}

////////////////////////////////////////////////////////////////////////////////

// see sf_spi.c
extern int  sf_spi_platform_init(void);
extern void sf_spi_platform_exit(void);

////////////////////////////////////////////////////////////////////////////////

static int sf_ctl_init_gpio_pins(void)
{
    int err = 0;
    struct device_node* dev_node = NULL;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    dev_node = of_find_compatible_node(NULL, NULL, "qcom,fingerprint");

    if (!dev_node) {
        xprintk(KERN_ERR, "of_find_compatible_node(..) failed.\n");
        return (-ENODEV);
    }

    sf_ctl_dev.reset_num = of_get_named_gpio(dev_node, "qcom,reset-gpio", 0);
    sf_ctl_dev.irq_num = of_get_named_gpio(dev_node, "qcom,irq-gpio", 0);

    xprintk(KERN_INFO, "reset_gpio_number = %d\n", sf_ctl_dev.reset_num);
    xprintk(KERN_INFO, "irq_gpio_number = %d\n", sf_ctl_dev.irq_num);

    xprintk(KERN_INFO, "=== Do not request gpio resources!!! ===\n");

    return err;
}

static int sf_ctl_init_input(void)
{
    int err = 0;
    xprintk(KERN_DEBUG, "%s(..) enter.\n", __FUNCTION__);
    sf_ctl_dev.input = input_allocate_device();

    if (!sf_ctl_dev.input) {
        xprintk(KERN_ERR, "input_allocate_device(..) failed.\n");
        return (-ENOMEM);
    }

    sf_ctl_dev.input->name = "sf-keys";
    __set_bit(EV_KEY  , sf_ctl_dev.input->evbit );
    __set_bit(KEY_HOME, sf_ctl_dev.input->keybit);
    __set_bit(KEY_MENU, sf_ctl_dev.input->keybit);
    __set_bit(KEY_BACK, sf_ctl_dev.input->keybit);
    __set_bit(KEY_F19, sf_ctl_dev.input->keybit);
    __set_bit(KEY_F20, sf_ctl_dev.input->keybit);
    __set_bit(KEY_F21, sf_ctl_dev.input->keybit);
    __set_bit(KEY_ENTER, sf_ctl_dev.input->evbit );
    __set_bit(KEY_UP, sf_ctl_dev.input->keybit);
    __set_bit(KEY_LEFT, sf_ctl_dev.input->keybit);
    __set_bit(KEY_RIGHT, sf_ctl_dev.input->keybit);
    __set_bit(KEY_DOWN, sf_ctl_dev.input->keybit);
    __set_bit(KEY_WAKEUP, sf_ctl_dev.input->keybit);

    err = input_register_device(sf_ctl_dev.input);

    if (err) {
        xprintk(KERN_ERR, "input_register_device(..) = %d.\n", err);
        input_free_device(sf_ctl_dev.input);
        sf_ctl_dev.input = NULL;
        return (-ENODEV);
    }

    xprintk(KERN_DEBUG, "%s(..) leave.\n", __FUNCTION__);
    return err;
}

static int __init sf_ctl_driver_init(void)
{
    int err = 0;
    sf_ctl_dev.isInitialize = 0;
    sf_ctl_dev.isOpen = 0;
    /* Initialize the GPIO pins. */
    err = sf_ctl_init_gpio_pins();

    if (err) {
        xprintk(KERN_ERR, "sf_ctl_init_gpio_pins failed with %d.\n", err);
        return err;
    }
	xprintk(KERN_INFO, "=== Do not sf_ctl_init_irq!!! ===\n");
    /* Initialize the input subsystem. */
    err = sf_ctl_init_input();

    if (err) {
        xprintk(KERN_ERR, "sf_ctl_init_input failed with %d.\n", err);
        //free_irq(sf_ctl_dev.irq_num, (void*)&sf_ctl_dev);
        return err;
    }

    err = sf_ctl_device_power(true);
    /* Register as a miscellaneous device. */
    err = misc_register(&sf_ctl_dev.miscdev);

    if (err) {
        xprintk(KERN_ERR, "misc_register(..) = %d.\n", err);
        input_unregister_device(sf_ctl_dev.input);
        //free_irq(sf_ctl_dev.irq_num, (void*)&sf_ctl_dev);
        return err;
    }

#if ANDROID_WAKELOCK
	wake_lock_init(&sf_ctl_dev.wakelock, WAKE_LOCK_SUSPEND, "sunwave_intr");
#endif

    INIT_WORK(&sf_ctl_dev.work_queue, sf_ctl_device_event);
    //err = sf_spi_platform_init();
    xprintk(KERN_INFO, "sunwave fingerprint device control driver registered.\n");
    xprintk(KERN_INFO, "driver version: '%s'.\n", sf_ctl_get_version());
    return err;
}

static void __exit sf_ctl_driver_exit(void)
{
    if (sf_ctl_dev.input) {
        input_unregister_device(sf_ctl_dev.input);
    }

    if (sf_ctl_dev.irq_num >= 0) {
		gpio_free(sf_ctl_dev.irq_num);
        free_irq(gpio_to_irq(sf_ctl_dev.irq_num), (void*)&sf_ctl_dev);
    }

	if (sf_ctl_dev.reset_num >= 0) {
        gpio_free(sf_ctl_dev.reset_num);
    }

    misc_deregister(&sf_ctl_dev.miscdev);
#if ANDROID_WAKELOCK
    wake_lock_destroy(&sf_ctl_dev.wakelock);
#endif
			remove_proc_entry(PROC_NAME,NULL);
    xprintk(KERN_INFO, "sunwave fingerprint device control driver released.\n");
}

module_init(sf_ctl_driver_init);
module_exit(sf_ctl_driver_exit);

MODULE_DESCRIPTION("The device control driver for Sunwave's fingerprint sensor.");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Langson L. <liangzh@sunwavecorp.com>");

