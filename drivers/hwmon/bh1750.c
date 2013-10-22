/*
 *  bh1750.c - Linux kernel modules for 3-Axis Orientation/Motion
 *  Detection Sensor 
 *
 *  Copyright (C) 2009-2010 Freescale Semiconductor Hong Kong Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>
#include <linux/earlysuspend.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend early_suspend;
static void bh1750_early_suspend(struct early_suspend *h);
static void bh1750_late_resume(struct early_suspend *h);
#endif
/*
 * Defines
 */
#define assert(expr)\
	if (!(expr)) {\
		printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);\
	}

#define BH1750_DRV_NAME	"bh1750"


#define I2C_RETRY_DELAY                 5
#define I2C_RETRIES                     5
#define MAX_FAILURE_COUNT	3
#define READMD			1
#define DEBUG				1
static u32 read_idx = 0;

#define BH1750_RETRY_COUNT	3
#define BH1750_RESET_INTV	10

/* register enum for bh1750 registers */
static struct device *hwmon_dev;
static struct i2c_client *bh1750_i2c_client;
static int ifsuspend = 0;

extern int dev_ver;
extern unsigned int key_state0;

static long gsensor_st = 0;
static long gsensor_disable = 0;

static int bh1750_i2c_rx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msgs[] = {
		{
			.addr	= bh1750_i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
//			.scl_rate = 200*1000,
//			.udelay = 100,
		}
	};

	for (i = 0; i < BH1750_RETRY_COUNT; i++) {
		if (i2c_transfer(bh1750_i2c_client->adapter, msgs, 1) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= BH1750_RETRY_COUNT) {
//		pr_err("%s: retry over %d\n", __FUNCTION__, BH1750_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int bh1750_i2c_tx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msg[] = {
		{
			.addr	= bh1750_i2c_client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};

	for (i = 0; i < BH1750_RETRY_COUNT; i++) {
		if (i2c_transfer(bh1750_i2c_client->adapter, msg, 1) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= BH1750_RETRY_COUNT) {
//		pr_err("%s: retry over %d\n", __FUNCTION__, BH1750_RETRY_COUNT);
		return -EIO;
	}
	return 0;
}



static int bh1750_get_data()
{
		char buf[3];
		int value;
	
    bh1750_i2c_rx_data(buf,2);
    value = buf[0]<< 8 | buf[1];
    value = value*10/12;
    
    return value;
}
/*
 * Initialization function
 */
static int bh1750_init_client(struct i2c_client *client)
{
	int err = 0;
	char buf[3];
	buf[0] = 0x10;
	bh1750_i2c_tx_data(buf,1);
	return err;
}

/***************************************************************
*
* read sensor data from bh1750
*
***************************************************************/ 				

static struct input_polled_dev *bh1750_idev;
#define POLL_INTERVAL		40
#define INPUT_FUZZ	16
#define INPUT_FLAT	16
#include <linux/input-polldev.h>
static void report_abs(void)
{
	int value = 0;
	value = bh1750_get_data();

    input_report_rel(bh1750_idev->input, REL_X, value);

//		printk("bl1750 value = %d +++++\n" , value );

	input_sync(bh1750_idev->input);
}

static void bh1750_dev_poll(struct input_polled_dev *dev)
{
  if(ifsuspend == 0)
	  report_abs();
} 
/////////////////////////end//////

/*
 * I2C init/probing/exit functions
 */
static int __devinit bh1750_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result;
	struct i2c_adapter *adapter;	
	struct input_dev *idev;
	
	ifsuspend = 0;

	bh1750_i2c_client = client;
  adapter = to_i2c_adapter(client->dev.parent);
	result = i2c_check_functionality(adapter, 
		I2C_FUNC_SMBUS_BYTE|I2C_FUNC_SMBUS_BYTE_DATA);
	assert(result);
	

  	/* Initialize the bh1750 chip */
  	result = bh1750_init_client(client);
  	assert(result==0);


	hwmon_dev = hwmon_device_register(&client->dev);
	assert(!(IS_ERR(hwmon_dev)));

	dev_info(&client->dev, "build time %s %s\n", __DATE__, __TIME__);

	/*input poll device register */
	bh1750_idev = input_allocate_polled_device();
	if (!bh1750_idev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		result = -ENOMEM;
		return result;
	}
	bh1750_idev->poll = bh1750_dev_poll;
	bh1750_idev->poll_interval = POLL_INTERVAL;
	idev = bh1750_idev->input;
	idev->name = BH1750_DRV_NAME;
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &client->dev;
//	idev->evbit[0] = BIT_MASK(EV_ABS);
	
	 input_set_capability(idev, EV_REL, REL_X);

   
	result = input_register_polled_device(bh1750_idev);
	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		return result;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	early_suspend.suspend = bh1750_early_suspend;
	early_suspend.resume = bh1750_late_resume;
	register_early_suspend(&early_suspend);
#endif

	return result;
}

static int __devexit bh1750_remove(struct i2c_client *client)
{
	int result=0;
	hwmon_device_unregister(hwmon_dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&early_suspend);
#endif
	return result;
}

static int bh1750_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int result=0;
	return result;
}

static int bh1750_resume(struct i2c_client *client)
{
	int result=0;
	return result;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bh1750_early_suspend(struct early_suspend *h)
{
  ifsuspend = 1;
}

static void bh1750_late_resume(struct early_suspend *h)
{
  ifsuspend = 0;
}
#endif

static const struct i2c_device_id bh1750_id[] = {
	{ BH1750_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bh1750_id);

static struct i2c_driver bh1750_driver = {
	.driver = {
		.name	= BH1750_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = bh1750_suspend,
	.resume	= bh1750_resume,
	.probe	= bh1750_probe,
	.remove	= __devexit_p(bh1750_remove),
	.id_table = bh1750_id,
};

static int __init bh1750_init(void)
{
	/* register driver */
	int res;

	res = i2c_add_driver(&bh1750_driver);
	if (res < 0){
		printk(KERN_INFO "add bh1750 i2c driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "add bh1750 i2c driver\n");

	return (res);
}

static void __exit bh1750_exit(void)
{
	printk(KERN_INFO "remove bh1750 i2c driver.\n");
	i2c_del_driver(&bh1750_driver);
}

MODULE_AUTHOR("Yang Yonghui <yonghui.yang@freescale.com>");
MODULE_DESCRIPTION("bh1750 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

module_init(bh1750_init);
module_exit(bh1750_exit);
