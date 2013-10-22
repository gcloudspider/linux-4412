/*
 *  l3g4200.c - Linux kernel modules for 3-Axis Orientation/Motion
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
static void l3g4200_early_suspend(struct early_suspend *h);
static void l3g4200_late_resume(struct early_suspend *h);
#endif
/*
 * Defines
 */
#define assert(expr)\
	if (!(expr)) {\
		printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);\
	}

#define L3G4200_DRV_NAME	"l3g4200d"
#define L3G4200_ID		0xD3

#define L3G4200_WHO_AM_I 0x0f
/** Register map */
#define L3G4200D_WHO_AM_I               0x0f
#define L3G4200D_CTRL_REG1              0x20
#define L3G4200D_CTRL_REG2              0x21
#define L3G4200D_CTRL_REG3              0x22
#define L3G4200D_CTRL_REG4              0x23
#define L3G4200D_CTRL_REG5              0x24

#define L3G4200D_REF_DATA_CAP           0x25
#define L3G4200D_OUT_TEMP               0x26
#define L3G4200D_STATUS_REG             0x27

#define L3G4200D_OUT_X_L                0x28
#define L3G4200D_OUT_X_H                0x29
#define L3G4200D_OUT_Y_L                0x2a
#define L3G4200D_OUT_Y_H                0x2b
#define L3G4200D_OUT_Z_L                0x2c
#define L3G4200D_OUT_Z_H                0x2d

#define L3G4200D_FIFO_CTRL              0x2e
#define L3G4200D_FIFO_SRC               0x2e

#define L3G4200D_INTERRUPT_CFG          0x30
#define L3G4200D_INTERRUPT_SRC          0x31
#define L3G4200D_INTERRUPT_THRESH_X_H   0x32
#define L3G4200D_INTERRUPT_THRESH_X_L   0x33
#define L3G4200D_INTERRUPT_THRESH_Y_H   0x34
#define L3G4200D_INTERRUPT_THRESH_Y_L   0x35
#define L3G4200D_INTERRUPT_THRESH_Z_H   0x36
#define L3G4200D_INTERRUPT_THRESH_Z_L   0x37
#define L3G4200D_INTERRUPT_DURATION     0x38

#define PM_MASK                         0x08
#define ENABLE_ALL_AXES                 0x07
#define ODR_MASK        0xF0
#define ODR100_BW25     0x10
#define ODR200_BW70     0x70
#define ODR400_BW110    0xB0
#define ODR800_BW110    0xF0

#define I2C_RETRY_DELAY                 5
#define I2C_RETRIES                     5
#define AUTO_INCREMENT                  0x80
#define L3G4200D_PU_DELAY               320

int ctrl_reg1 = 0x1f;      /* ODR100 */
int ctrl_reg2 = 0x00;
int ctrl_reg3 = 0x08;      /* Enable DRDY interrupt */
int ctrl_reg4 = 0xA0;      /* BDU enable, 2000 dps */
int ctrl_reg5 = 0x00;
int reference = 0x00;
int fifo_ctrl_reg = 0x00;
int int1_cfg = 0x00;
int int1_tsh_xh = 0x00;
int int1_tsh_xl = 0x00;
int int1_tsh_yh = 0x00;
int int1_tsh_yl = 0x00;
int int1_tsh_zh = 0x00;
int int1_tsh_zl = 0x00;
int int1_duration = 0x00;

/* register enum for l3g4200 registers */
static struct device *hwmon_dev;
static int test_test_mode = 1;
static struct i2c_client *l3g4200_i2c_client;
static int ifsuspend = 0;

extern int dev_ver;
extern unsigned int key_state0;

extern struct device *switch_dev;
static long gsensor_st = 0;
static long gsensor_disable = 0;

static int l3g4200d_i2c_read(struct i2c_client *client, u8 * buf, int len)
{
        int err;
        int tries = 0;
        struct i2c_msg msgs[] = {
                {
                        .addr = client->addr,
                        .flags = client->flags & I2C_M_TEN,
                        .len = 1,
                        .buf = buf,
                },
                {
                        .addr = client->addr,
                        .flags = (client->flags & I2C_M_TEN) | I2C_M_RD,
                        .len = len,
                        .buf = buf,
                },
        };

        do {
                err = i2c_transfer(client->adapter, msgs, 2);
                if (err != 2)
                        msleep_interruptible(I2C_RETRY_DELAY);
        } while ((err != 2) && (++tries < I2C_RETRIES));

        if (err != 2) {
                dev_err(&client->dev, "read transfer error\n");
                err = -EIO;
        } else {
                err = 0;
        }

        return err;
}

static int l3g4200d_i2c_write(struct i2c_client *client, u8 * buf, int len)
{
        int err;
        int tries = 0;
        struct i2c_msg msgs[] = {
                {
                        .addr = client->addr,
                        .flags = client->flags & I2C_M_TEN,
                        .len = len + 1,
                        .buf = buf,
                },
        };

        do {
                err = i2c_transfer(client->adapter, msgs, 1);
                if (err != 1)
                        msleep_interruptible(I2C_RETRY_DELAY);
        } while ((err != 1) && (++tries < I2C_RETRIES));

        if (err != 1) {
                dev_err(&client->dev, "write transfer error\n");
                err = -EIO;
        } else {
                err = 0;
        }

        return err;
}

static int l3g4200d_get_gyro_data(struct i2c_client *client,short *x, short *y, short *z)
{
        int err = -1;
        /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
        u8 gyro_data[6];

        gyro_data[0] = (AUTO_INCREMENT | L3G4200D_OUT_X_L);
        err = l3g4200d_i2c_read(client, gyro_data, 6);
        if (err < 0)
                return err;

        *x = (gyro_data[1] << 8) | gyro_data[0];
        *y = (gyro_data[3] << 8) | gyro_data[2];
        *z = (gyro_data[5] << 8) | gyro_data[4];
        return 0;
}
/*
 * Initialization function
 */
static int l3g4200_init_client(struct i2c_client *client)
{
	int err = -1;;

   u8 buf[8];

		buf[0] = L3G4200D_CTRL_REG1;
		buf[1] = 0xBf;
   err = l3g4200d_i2c_write(client, buf, 1);
   if (err < 0)
           return err;

		buf[0] = L3G4200D_CTRL_REG2;
		buf[1] = 0x00;
   err = l3g4200d_i2c_write(client, buf, 1);
   if (err < 0)
           return err;
           
		buf[0] = L3G4200D_CTRL_REG3;
		buf[1] = 0x08;
   err = l3g4200d_i2c_write(client, buf, 1);
   if (err < 0)
           return err;

		buf[0] = L3G4200D_CTRL_REG4;
		buf[1] = 0x30;
   err = l3g4200d_i2c_write(client, buf, 1);
   if (err < 0)
           return err;
           
		buf[0] = L3G4200D_CTRL_REG5;
		buf[1] = 0x00;
   err = l3g4200d_i2c_write(client, buf, 1);
   if (err < 0)
           return err;
                
		buf[0] = L3G4200D_FIFO_CTRL;  //stream
		buf[1] = 0x00 |1 << 6;
   err = l3g4200d_i2c_write(client, buf, 1);
   if (err < 0)
           return err;                     

	return err;
}

/***************************************************************
*
* read sensor data from l3g4200
*
***************************************************************/ 				

static struct input_polled_dev *l3g4200_idev;
#define POLL_INTERVAL		100
#define INPUT_FUZZ	16
#define INPUT_FLAT	16
#include <linux/input-polldev.h>
static void report_abs(void)
{
	short 	x, y, z, x_r, y_r, z_r, tmp;
	int result;

	l3g4200d_get_gyro_data(l3g4200_i2c_client,&x_r,&y_r,&z_r);

  
    input_report_rel(l3g4200_idev->input, REL_RX, x_r);
    input_report_rel(l3g4200_idev->input, REL_RY, y_r);
    input_report_rel(l3g4200_idev->input, REL_RZ, z_r);
//printk(" x=%4d, y=%4d, z=%4d ++++++\n" , x_r, y_r, z_r );
	input_sync(l3g4200_idev->input);
}

static void l3g4200_dev_poll(struct input_polled_dev *dev)
{
  if(gsensor_disable == 1)//ËøÆÁ¼ü
    return;
  if(ifsuspend == 0)
	  report_abs();
} 
/////////////////////////end//////

/*
 * I2C init/probing/exit functions
 */
static int __devinit l3g4200_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result;
	struct i2c_adapter *adapter;	
	struct input_dev *idev;
	
	ifsuspend = 0;

	l3g4200_i2c_client = client;
  adapter = to_i2c_adapter(client->dev.parent);
	result = i2c_check_functionality(adapter, 
		I2C_FUNC_SMBUS_BYTE|I2C_FUNC_SMBUS_BYTE_DATA);
	assert(result);
	
	if(dev_ver != 1)
	{
  	printk(KERN_INFO "check l3g4200 chip ID\n");
  	result = i2c_smbus_read_byte_data(client, L3G4200_WHO_AM_I)&0xFF;
  
  	if (L3G4200_ID != (result)) {	//compare the address value 
  		dev_err(&client->dev,"read chip ID 0x%x is not equal to 0x%x!\n", result,L3G4200_ID);
  		printk(KERN_INFO "read chip ID failed\n");
  		result = -EINVAL;
  		goto err_detach_client;
  	}
  
  	/* Initialize the l3g4200 chip */
  	result = l3g4200_init_client(client);
  	assert(result==0);
	}

	hwmon_dev = hwmon_device_register(&client->dev);
	assert(!(IS_ERR(hwmon_dev)));

	dev_info(&client->dev, "build time %s %s\n", __DATE__, __TIME__);

	/*input poll device register */
	l3g4200_idev = input_allocate_polled_device();
	if (!l3g4200_idev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		result = -ENOMEM;
		return result;
	}
	l3g4200_idev->poll = l3g4200_dev_poll;
	l3g4200_idev->poll_interval = POLL_INTERVAL;
	idev = l3g4200_idev->input;
	idev->name = L3G4200_DRV_NAME;
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &client->dev;
//	idev->evbit[0] = BIT_MASK(EV_ABS);
	
	 input_set_capability(idev, EV_REL, REL_RX);
   input_set_capability(idev, EV_REL, REL_RY);
   input_set_capability(idev, EV_REL, REL_RZ);
   
	result = input_register_polled_device(l3g4200_idev);
	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		return result;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	early_suspend.suspend = l3g4200_early_suspend;
	early_suspend.resume = l3g4200_late_resume;
	register_early_suspend(&early_suspend);
#endif

	return result;
	
err_detach_client:
	return result;
}

static int __devexit l3g4200_remove(struct i2c_client *client)
{
	int result=0;
	if(dev_ver != 1)
	{
  }

	hwmon_device_unregister(hwmon_dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&early_suspend);
#endif

	return result;
}

static int l3g4200_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int result=0;
	if(dev_ver != 1)
	{

  }
	return result;
}

static int l3g4200_resume(struct i2c_client *client)
{
	int result=0;
	if(dev_ver != 1)
	{

  }
	return result;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void l3g4200_early_suspend(struct early_suspend *h)
{
  ifsuspend = 1;
}

static void l3g4200_late_resume(struct early_suspend *h)
{
  ifsuspend = 0;
}
#endif

static const struct i2c_device_id l3g4200_id[] = {
	{ L3G4200_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, l3g4200_id);

static struct i2c_driver l3g4200_driver = {
	.driver = {
		.name	= L3G4200_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = l3g4200_suspend,
	.resume	= l3g4200_resume,
	.probe	= l3g4200_probe,
	.remove	= __devexit_p(l3g4200_remove),
	.id_table = l3g4200_id,
};

static int __init l3g4200_init(void)
{
	/* register driver */
	int res;

	res = i2c_add_driver(&l3g4200_driver);
	if (res < 0){
		printk(KERN_INFO "add l3g4200 i2c driver failed\n");
		return -ENODEV;
	}
	printk(KERN_INFO "add l3g4200 i2c driver\n");

	return (res);
}

static void __exit l3g4200_exit(void)
{
	printk(KERN_INFO "remove l3g4200 i2c driver.\n");
	i2c_del_driver(&l3g4200_driver);
}

MODULE_AUTHOR("Yang Yonghui <yonghui.yang@freescale.com>");
MODULE_DESCRIPTION("l3g4200 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

module_init(l3g4200_init);
module_exit(l3g4200_exit);
