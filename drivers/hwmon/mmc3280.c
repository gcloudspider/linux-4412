/*
 *  mmc3280.c - Linux kernel modules for 3-Axis Orientation/Motion
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
static void mmc3280_early_suspend(struct early_suspend *h);
static void mmc3280_late_resume(struct early_suspend *h);
#endif
/*
 * Defines
 */
#define assert(expr)\
	if (!(expr)) {\
		printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);\
	}

#define MMC3280_DRV_NAME	"mmc3280"


#define I2C_RETRY_DELAY                 5
#define I2C_RETRIES                     5
#define MAX_FAILURE_COUNT	3
#define READMD			1
#define DEBUG				1
static u32 read_idx = 0;

#define MMC328X_RETRY_COUNT	3
#define MMC328X_RESET_INTV	10

#define MMC328X_I2C_ADDR		0x30

/* MMC328X register address */
#define MMC328X_REG_CTRL		0x07
#define MMC328X_REG_DATA		0x00
#define MMC328X_REG_DS			0x06

/* MMC328X control bit */
#define MMC328X_CTRL_TM			0x01
#define MMC328X_CTRL_RM			0x20

#define MMC328X_DELAY_TM	10	/* ms */
#define MMC328X_DELAY_RM	10	/* ms */
#define MMC328X_DELAY_STDN	1	/* ms */
/* register enum for mmc3280 registers */
static struct device *hwmon_dev;
static struct i2c_client *mmc3280_i2c_client;
static int ifsuspend = 0;

extern int dev_ver;
extern unsigned int key_state0;

static long gsensor_st = 0;
static long gsensor_disable = 0;

static int mmc328x_i2c_rx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msgs[] = {
		{
			.addr	= mmc3280_i2c_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
//			.scl_rate = 200*1000,
//			.udelay = 100,
		},
		{
			.addr	= mmc3280_i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
//			.scl_rate = 200*1000,
//			.udelay = 100,
		}
	};

	for (i = 0; i < MMC328X_RETRY_COUNT; i++) {
		if (i2c_transfer(mmc3280_i2c_client->adapter, msgs, 2) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MMC328X_RETRY_COUNT) {
	//	pr_err("%s: retry over %d\n", __FUNCTION__, MMC328X_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int mmc328x_i2c_tx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msg[] = {
		{
			.addr	= mmc3280_i2c_client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};

	for (i = 0; i < MMC328X_RETRY_COUNT; i++) {
		if (i2c_transfer(mmc3280_i2c_client->adapter, msg, 1) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MMC328X_RETRY_COUNT) {
	//	pr_err("%s: retry over %d\n", __FUNCTION__, MMC328X_RETRY_COUNT);
		return -EIO;
	}
	return 0;
}


static int mmc3280_get_data(struct i2c_client *client,short *x, short *y, short *z)
{
     int MD_times = 0;
     u8 data[6];

/* do RM every MMC328X_RESET_INTV times read */
		if (!(read_idx % MMC328X_RESET_INTV)) {
			/* RM */
			data[0] = MMC328X_REG_CTRL;
			data[1] = MMC328X_CTRL_RM;
			/* not check return value here, assume it always OK */
			mmc328x_i2c_tx_data(data, 2);
			/* wait external capacitor charging done for next RM */
			msleep(MMC328X_DELAY_RM);
		}
		/* send TM cmd before read */
		data[0] = MMC328X_REG_CTRL;
		data[1] = MMC328X_CTRL_TM;
		/* not check return value here, assume it always OK */
		mmc328x_i2c_tx_data(data, 2);
		/* wait TM done for coming data read */
		msleep(MMC328X_DELAY_TM);
#if READMD
		/* Read MD */
		data[0] = MMC328X_REG_DS;
		if (mmc328x_i2c_rx_data(data, 1) < 0) {
			return -EFAULT;
		}
		while (!(data[0] & 0x01)) {
			msleep(1);
			/* Read MD again*/
			data[0] = MMC328X_REG_DS;
			if (mmc328x_i2c_rx_data(data, 1) < 0) {
				return -EFAULT;
			}
			if (data[0] & 0x01) break;
			MD_times++;
			if (MD_times > 2) {
		#if DEBUG
				printk("TM not work!!");
		#endif
				return -EFAULT;
			}
		}
#endif		
		/* read xyz raw data */
		read_idx++;
		data[0] = MMC328X_REG_DATA;
		if (mmc328x_i2c_rx_data(data, 6) < 0) {
			return -EFAULT;
		}
		*x = data[1] << 8 | data[0];
		*y = data[3] << 8 | data[2];
		*z = data[5] << 8 | data[4];	
    return 0;
}
/*
 * Initialization function
 */
static int mmc3280_init_client(struct i2c_client *client)
{
	int err = 0;
	return err;
}

/***************************************************************
*
* read sensor data from mmc3280
*
***************************************************************/ 				

static struct input_polled_dev *mmc3280_idev;
#define POLL_INTERVAL		40
#define INPUT_FUZZ	16
#define INPUT_FLAT	16
#include <linux/input-polldev.h>
static void report_abs(void)
{
	short x_r = 0, y_r = 0, z_r = 0;
	mmc3280_get_data(mmc3280_i2c_client,&x_r,&y_r,&z_r);


    input_report_rel(mmc3280_idev->input, ABS_X, x_r);
    input_report_rel(mmc3280_idev->input, ABS_Y, y_r);
    input_report_rel(mmc3280_idev->input, ABS_Z, z_r);
//		printk("mmc3820 x=%4d, y=%4d, z=%4d ++++++\n" , x_r, y_r, z_r );

	input_sync(mmc3280_idev->input);
}

static void mmc3280_dev_poll(struct input_polled_dev *dev)
{
  if(ifsuspend == 0)
	  report_abs();
} 
/////////////////////////end//////

/*
 * I2C init/probing/exit functions
 */
static int __devinit mmc3280_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result;
	struct i2c_adapter *adapter;	
	struct input_dev *idev;
	
	ifsuspend = 0;

	mmc3280_i2c_client = client;
  adapter = to_i2c_adapter(client->dev.parent);
	result = i2c_check_functionality(adapter, 
		I2C_FUNC_SMBUS_BYTE|I2C_FUNC_SMBUS_BYTE_DATA);
	assert(result);
	
	if(dev_ver != 1)
	{ 
  	/* Initialize the mmc3280 chip */
  	result = mmc3280_init_client(client);
  	assert(result==0);
	}

	hwmon_dev = hwmon_device_register(&client->dev);
	assert(!(IS_ERR(hwmon_dev)));

	dev_info(&client->dev, "build time %s %s\n", __DATE__, __TIME__);

	/*input poll device register */
	mmc3280_idev = input_allocate_polled_device();
	if (!mmc3280_idev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		result = -ENOMEM;
		return result;
	}
	mmc3280_idev->poll = mmc3280_dev_poll;
	mmc3280_idev->poll_interval = POLL_INTERVAL;
	idev = mmc3280_idev->input;
	idev->name = MMC3280_DRV_NAME;
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &client->dev;
//	idev->evbit[0] = BIT_MASK(EV_ABS);
	
	 input_set_capability(idev, EV_REL, ABS_X);
   input_set_capability(idev, EV_REL, ABS_Y);
   input_set_capability(idev, EV_REL, ABS_Z);
   
	result = input_register_polled_device(mmc3280_idev);
	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		return result;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	early_suspend.suspend = mmc3280_early_suspend;
	early_suspend.resume = mmc3280_late_resume;
	register_early_suspend(&early_suspend);
#endif

	return result;
}

static int __devexit mmc3280_remove(struct i2c_client *client)
{
	int result=0;
	hwmon_device_unregister(hwmon_dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&early_suspend);
#endif
	return result;
}

static int mmc3280_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int result=0;
	return result;
}

static int mmc3280_resume(struct i2c_client *client)
{
	int result=0;
	return result;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mmc3280_early_suspend(struct early_suspend *h)
{
  ifsuspend = 1;
}

static void mmc3280_late_resume(struct early_suspend *h)
{
  ifsuspend = 0;
}
#endif

static const struct i2c_device_id mmc3280_id[] = {
	{ MMC3280_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mmc3280_id);

static struct i2c_driver mmc3280_driver = {
	.driver = {
		.name	= MMC3280_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = mmc3280_suspend,
	.resume	= mmc3280_resume,
	.probe	= mmc3280_probe,
	.remove	= __devexit_p(mmc3280_remove),
	.id_table = mmc3280_id,
};

static int __init mmc3280_init(void)
{
	/* register driver */
	int res;

	res = i2c_add_driver(&mmc3280_driver);
	if (res < 0){
		printk(KERN_INFO "add mmc3280 i2c driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "add mmc3280 i2c driver\n");

	return (res);
}

static void __exit mmc3280_exit(void)
{
	printk(KERN_INFO "remove mmc3280 i2c driver.\n");
	i2c_del_driver(&mmc3280_driver);
}

MODULE_AUTHOR("Yang Yonghui <yonghui.yang@freescale.com>");
MODULE_DESCRIPTION("mmc3280 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

module_init(mmc3280_init);
module_exit(mmc3280_exit);
