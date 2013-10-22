/*
 *  apds9900.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
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
#include <linux/semaphore.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/hwmon.h>
#include <linux/err.h>
#include "sensor-input.h"
#include "apds9900.h"
/******************************************************************************************
******************************************************************************************/
struct delayed_work apds9900_dwork;
static struct semaphore apds9900_sem;
static struct i2c_client *g_client;
struct apds9900_data apds_data;
static int aTime = 0xDB; /* 100.64ms */
static int alsGain = 1;
//static float GA=0.48;
//static float COE_B=2.23;
//static float COE_C=0.7;
//static float COE_D=1.42;
static int DF=52;
/******************************************************************************************
******************************************************************************************/
static struct input_polled_dev *apds9900_idev;
static struct device *hwmon_dev;
#define POLL_INTERVAL		40
/*
 * Defines
 */
#define assert(expr)\
	if (!(expr)) {\
		printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);\
	}

static int apds9900_read_byte(u8 reg)
{
	int rc;      
	
	if( g_client == NULL )	/* Not initialize the client */	
		return -ENODEV;
	if(down_interruptible(&apds9900_sem))
		return -ENAVAIL;
	rc = i2c_smbus_read_byte_data(g_client,CMD_BYTE|reg);
	up(&apds9900_sem);
	return rc;
} 
static int apds9900_write_byte(u8 reg,int value)
{
	int rc;  
	if( g_client == NULL )	/* Not initialize the client */	
		return -ENODEV;
	if(down_interruptible(&apds9900_sem))
		return -ENAVAIL;
	rc = i2c_smbus_write_byte_data(g_client,CMD_BYTE|reg,value);
	up(&apds9900_sem);
	return rc;
}
static int apds9900_read_word(u8 reg)
{
	int rc;      
	
	if( g_client == NULL )	/* Not initialize the client */	
		return -ENODEV;
	if(down_interruptible(&apds9900_sem))
		return -ENAVAIL;
	rc = i2c_smbus_read_word_data(g_client,CMD_WORD|reg);
	up(&apds9900_sem);
	return rc;
} 
static int apds9900_write_word(u8 reg,int value)
{
	int rc;  
	if( g_client == NULL )	/* Not initialize the client */	
		return -ENODEV;
	if(down_interruptible(&apds9900_sem))
		return -ENAVAIL;
	rc = i2c_smbus_write_word_data(g_client,CMD_WORD|reg,value);
	up(&apds9900_sem);
	return rc;
}
static int apds9900_set_command(APDS_COMMAND command)
{
	int ret;
	int clearInt=0;
	if( g_client == NULL )	/* Not initialize the client */	
		return -ENODEV;
	switch(command)
  	{
       case APDS_CLR_PS_INT:
		    clearInt = CMD_CLR_PS_INT;
       case APDS_CLR_ALS_INT:
   		    clearInt = CMD_CLR_ALS_INT;
       case APDS_CLR_PS_ALS_INT:
		    clearInt = CMD_CLR_PS_ALS_INT;
	   default:
		    clearInt = CMD_CLR_PS_ALS_INT;
	}
	if(down_interruptible(&apds9900_sem))
		return -ENAVAIL;
	ret = i2c_smbus_write_byte(g_client, clearInt);
	up(&apds9900_sem);
    return ret;
}
static int apds9900_set_enable(int enable)
{
	int ret;
	ret = apds9900_write_byte(APDS9900_ENABLE_REG, enable);
	return ret;
}
static int apds9900_set_atime(int atime)
{
	int ret;
	ret = apds9900_write_byte(APDS9900_ATIME_REG, atime);
	return ret;
}

static int apds9900_set_ptime(int ptime)
{
	int ret;
	ret = apds9900_write_byte(APDS9900_PTIME_REG, ptime);
	return ret;
}

static int apds9900_set_wtime(int wtime)
{
	int ret;
	ret = apds9900_write_byte(APDS9900_WTIME_REG, wtime);
	return ret;
}

static int apds9900_set_ailt(int threshold)
{
	int ret;
	ret = apds9900_write_word(APDS9900_AILTL_REG, threshold);
	apds_data.ailt=threshold;
	return ret;
}

static int apds9900_set_aiht(int threshold)
{
	int ret;
	ret = apds9900_write_word(APDS9900_AIHTL_REG, threshold);
	apds_data.aiht=threshold;
	return ret;
}

static int apds9900_set_pilt(int threshold)
{
	int ret;
	ret =apds9900_write_word(APDS9900_PILTL_REG, threshold);
	apds_data.pilt=threshold;
	return ret;
}

static int apds9900_set_piht(int threshold)
{
	int ret;
	ret = apds9900_write_word(APDS9900_PIHTL_REG, threshold);
	apds_data.piht=threshold;
	return ret;
}

static int apds9900_set_pers(int pers)
{
	int ret;
	ret =apds9900_write_byte(APDS9900_PERS_REG,pers);
	return ret;
}
static int apds9900_set_ppcount(int ppcount)
{
	int ret;
	ret = apds9900_write_byte(APDS9900_PPCOUNT_REG, ppcount);
	return ret;
}

static int apds9900_set_control(int control)
{
	int ret;
	
	ret = apds9900_write_byte(APDS9900_CONTROL_REG, control);
	return ret;
}
static int LuxCalculation(unsigned int cdata, unsigned int irdata)
{
	int luxValue=0;
	int IAC1=0;
	int IAC2=0;
	int IAC=0;
	//IAC1 = (int) (cdata - (COE_B*irdata));
	//IAC2 = (int) ((COE_C*cdata) - (COE_D*irdata));
	IAC1 = (int) (cdata - (223*irdata/100));
	IAC2 = (int) ((7*cdata/10) - (142*irdata/100));
	if (IAC1 > IAC2)
		IAC = IAC1;
	else if (IAC1 <= IAC2)
		IAC = IAC2;
	else
		IAC = 0;
	//luxValue = (IAC*GA*DF)/((2.72*(256-aTime))*alsGain);
	luxValue = (IAC*48*DF/100)/((272*(256-aTime))/100*alsGain);
	return luxValue;
}
static void apds9900_report(struct input_dev *idev)
{
//	struct sensor_input_dev* sensor;
	int lux;
//	sensor = dev_get_drvdata(&idev->dev);
	apds_data.ps_data =	apds9900_read_word(APDS9900_PDATAL_REG);/*MAX=1023*/
	apds_data.cdata= apds9900_read_word(APDS9900_CDATAL_REG);/*MAX=65535*/
	apds_data.irdata= apds9900_read_word(APDS9900_IRDATAL_REG);/*MAX=65535*/
	lux=LuxCalculation(apds_data.cdata,apds_data.irdata);
	input_report_rel(apds9900_idev->input, ABS_X, apds_data.ps_data);
	input_report_rel(apds9900_idev->input, ABS_Y,     lux);

//	printk("apds9900:status = 0x%02x, Distance= %d, Lux = %d +++++++++++\n",apds9900_read_byte(0x13),apds_data.ps_data,lux);
	input_sync(apds9900_idev->input); 
}
static void apds9900_power_on(void)
{
	apds9900_set_enable(PON|AEN|PEN|WEN);
}
static inline void apds9900_power_off(void)
{
	apds9900_set_enable(0);//(~(PON|AEN|PEN|WEN|AIEN|PIEN));
}
static int apds9900_init_client(void)
{ 
    int ret;
	int ch0Data;
	int ch0lt, ch0ht;
	if((ret=apds9900_set_enable(0))<0)
		return ret;
	if((ret=apds9900_set_atime(0xDB))<0)
		return ret;
	if((ret=apds9900_set_ptime(0xFF))<0)
		return ret;
	if((ret=apds9900_set_wtime(0xFF))<0)
		return ret;
	if((ret=apds9900_set_ppcount(1))<0) /* 4-pulse */

		return ret;
	if((ret=apds9900_set_control(0x20))<0) /* 100mA, IR-diode, 1X PGAIN, 1X AGAIN */
		return ret;
	if((ret=apds9900_set_pers(0x22))<0)
		return ret; 

	ch0Data = apds9900_read_word(APDS9900_CDATAL_REG);
	ch0lt = (ch0Data * 80) /100;
	ch0ht = (ch0Data * 120 ) /100;
	if (ch0ht >= 65535) ch0ht = 65535;
	if((ret=apds9900_set_ailt(ch0lt))<0)
		return ret;
	if((ret=apds9900_set_aiht(ch0ht))<0)
		return ret;
	if((ret=apds9900_set_pilt(0))<0)
		return ret;
	if((ret=apds9900_set_piht(500))<0)
		return ret;
	apds9900_set_command(APDS_CLR_PS_ALS_INT);
	
//fighter++
	apds9900_power_on();
//fighter--	
	return 0;
}

static void apds9900_dev_poll(struct input_polled_dev *dev)
{
	 apds9900_report(apds9900_idev->input);
} 

#define	apds9900_i2c_suspend		NULL
#define	apds9900_i2c_resume		NULL
static int __devinit apds9900_i2c_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int device_id,ret;	
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct input_dev *idev;
	int result;
	
	g_client = client;
	sema_init(&apds9900_sem,1);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;
	device_id= apds9900_read_byte(APDS9900_ID_REG);
	if(device_id !=0x29)
	{
		g_client = NULL;
		printk("APDS9900 detected fail!");
		return -ENODEV;
	}
	printk("APDS9900 detected\n");
	ret=apds9900_init_client(); 
	if(ret<0)
	{
		g_client = NULL;
		return -ENODEV;
	}
	hwmon_dev = hwmon_device_register(&client->dev);
	assert(!(IS_ERR(hwmon_dev)));
	/*input poll device register */
	
	apds9900_idev = input_allocate_polled_device();
	if (!apds9900_idev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		result = -ENOMEM;
		return result;
	}
	apds9900_idev->poll = apds9900_dev_poll;
	apds9900_idev->poll_interval = POLL_INTERVAL;
	idev = apds9900_idev->input;
	idev->name = APDS9900_DRV_NAME;
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &client->dev;
//	idev->evbit[0] = BIT_MASK(EV_ABS);
	
	 input_set_capability(idev, EV_REL, ABS_X);
   input_set_capability(idev, EV_REL, ABS_Y);
   input_set_capability(idev, EV_REL, ABS_Z);
   
	result = input_register_polled_device(apds9900_idev);
	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		return result;
	}
	return 0;
}
static int apds9900_i2c_remove(struct i2c_client *client __maybe_unused)
{
	g_client = NULL;
	hwmon_device_unregister(hwmon_dev);
//	sensor_input_del("apds9900");
	return 0;
}
 

static const struct i2c_device_id apds9900_i2c_id[] = {
	{ "apds9900", 0 },
	{ }
};

static struct i2c_driver apds9900_i2c_driver = {
	.driver = {
		.name	= "apds9900",
	},
	.id_table 	= apds9900_i2c_id,
	.probe		= apds9900_i2c_probe,
	.remove		= apds9900_i2c_remove,
	.suspend	= apds9900_i2c_suspend,
	.resume		= apds9900_i2c_resume,
};

static int __devinit apds9900_i2c_init(void)
{
	return i2c_add_driver(&apds9900_i2c_driver);
}

static void __exit apds9900_i2c_exit(void)
{
	i2c_del_driver(&apds9900_i2c_driver);
}

MODULE_DESCRIPTION
    ("APDS9900  proximity (I2C) driver");
MODULE_AUTHOR("Bin Yang <bin.yang@marvell.com>");
MODULE_LICENSE("GPL");

module_init(apds9900_i2c_init);
module_exit(apds9900_i2c_exit);