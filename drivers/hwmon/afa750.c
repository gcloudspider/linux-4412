
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

#include	"afa750.h"


#define FUZZ								32
#define FLAT								32
#define RESUME_ENTRIES					20
#define I2C_RETRY_DELAY					5
#define I2C_RETRIES						3
#define G_MAX							16000	/** Maximum polled-device-reported g value */


#define RES_CTRL_REG1					0
#define RES_CTRL_REG2					1
#define RES_CTRL_REG3					2
#define RES_CTRL_REG4					3
#define RES_CTRL_REG5					4
#define RES_CTRL_REG6					5

#define RES_INT_CFG1						6
#define RES_INT_THS1						7
#define RES_INT_DUR1						8
#define RES_INT_CFG2						9
#define RES_INT_THS2						10
#define RES_INT_DUR2						11

#define RES_TT_CFG						12
#define RES_TT_THS						13
#define RES_TT_LIM						14
#define RES_TT_TLAT						15
#define RES_TT_TW						16

#define RES_TEMP_CFG_REG					17
#define RES_REFERENCE_REG				18
#define RES_FIFO_CTRL_REG				19

#define RESUME_ENTRIES					20



#define AFA750_REG_WHO_AM_I      			0x37 //RO
#define WHOAMI_AFA750_ACC				0x3D	

#define AFA750_REG_CTRL_PWR				0x03
#define AFA750_ACC_PM_OFF				0x02
#define AFA750_ACC_PM_ON					0x00


#define AFA750_REG_CTRL_FS				0x02
#define AFA750_ACC_FS_MASK				0x03
#define AFA750_ACC_G_2G 					0x00
#define AFA750_ACC_G_4G 					0x01
#define AFA750_ACC_G_8G 					0x02

#define SENSITIVITY_2G					1	/**	mg/LSB	*/
#define SENSITIVITY_4G					2	/**	mg/LSB	*/
#define SENSITIVITY_8G					4	/**	mg/LSB	*/
#define SENSITIVITY_16G					12	/**	mg/LSB	*/

#define AFA750_REG_CTRL_ODR				0x05
#define ODR2								0x08  /* 2Hz output data rate */
#define ODR3								0x07  /* 3Hz output data rate */
#define ODR7								0x06  /* 7Hz output data rate */
#define ODR13							0x05  /* 13Hz output data rate */
#define ODR25							0x04  /* 25Hz output data rate */
#define ODR50							0x03  /* 50Hz output data rate */
#define ODR100							0x02  /* 100Hz output data rate */
#define ODR200							0x01  /* 200Hz output data rate */
#define ODR400							0x00  /* 400Hz output data rate */


//extern void sprd_free_eic_irq(int irq);
extern struct device *switch_dev;
extern int dev_ver;

struct afa750_acc_platform_data 
{
	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x; //not use
	u8 axis_map_y; //not use
	u8 axis_map_z; //not use

	u8 negate_x; //not use
	u8 negate_y; //not use
	u8 negate_z; //not use

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	int gpio_int1;
	int gpio_int2;
};


struct afa750_acc_data {
	struct i2c_client *client;
	struct afa750_acc_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	int hw_working; //hw_working=-1 means not tested yet
	atomic_t enabled;
	int on_before_suspend;
	atomic_t position; 
	
	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	struct early_suspend early_suspend; 
};


static struct afa750_acc_platform_data afa750_plat_data = {
	.poll_interval = 100,
	.min_interval = 10,
	.g_range    = AFA750_ACC_G_8G, //+-8G 16bit 1g=4096
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x   = 0,
	.negate_y   = 0,
	.negate_z   = 1
};


struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} afa750_acc_odr_table[] = {
	{ 3, ODR400 },
	{ 5, ODR200 },
	{ 10, ODR100 },
	{ 20, ODR50 },
	{ 40, ODR25 },
	{ 80, ODR13 },
	{ 160, ODR7 },
	{ 320, ODR3},
	{ 640, ODR2 },
};

static const int afa750_position_map[][3][3] = 
{
	{{ 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1}}, /* top/upper-left */
	{{ 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1}}, /* top/upper-right */
	{{ 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1}}, /* top/lower-right */
	{{-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1}}, /* top/lower-left */
	{{ 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1}}, /* bottom/upper-right */
	{{-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1}}, /* bottom/upper-left */
	{{ 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1}}, /* bottom/lower-left */
	{{ 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1}}, /* bottom/lower-right */
};

struct afa750_acc_data*	afa750_acc_misc_data;
struct i2c_client*			afa750_i2c_client;
u16						global_acc_data[3];

//fighter++
static long gsensor_st = 0;
static long gsensor_disable = 0;

static int gsensor_st_get(struct device *dev, struct device_attribute *attr, char *buf)
{
  return sprintf(buf, "%ld\n", gsensor_st);
}

static int gsensor_st_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
  int ret;
  ret = strict_strtoul(buf, 0, &gsensor_st);
//  printk("set gsensor_st %ld\n", gsensor_st);
  return strnlen(buf, PAGE_SIZE);
}

static int gsensor_disable_get(struct device *dev, struct device_attribute *attr, char *buf)
{
//  printk("get gsensor_disable %ld\n", gsensor_disable);
  return sprintf(buf, "%ld\n", gsensor_disable);
}

static int gsensor_disable_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
  int ret;
  ret = strict_strtoul(buf, 0, &gsensor_disable);
//  printk("set gsensor_disable %ld\n", gsensor_disable);
  return strnlen(buf, PAGE_SIZE);
}

static DEVICE_ATTR(gsensor_st, 0666, gsensor_st_get, gsensor_st_set);
static DEVICE_ATTR(gsensor_disable, 0666, gsensor_disable_get, gsensor_disable_set);
//fighter--
static int afa750_acc_i2c_read(struct afa750_acc_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = 
	{
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf, 
		},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf, 
		},
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) 
	{
		GSENSOR_MSG("read transfer error\n");
		err = -EIO;
	} 
	else 
	{
		err = 0;
	}

	return err;
}


static int afa750_acc_i2c_write(struct afa750_acc_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = 
	{ 
		{ 
		.addr = acc->client->addr,
		.flags = acc->client->flags & I2C_M_TEN,
		.len = len + 1,
		.buf = buf, 
		}, 
	};
	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) 
	{
		GSENSOR_MSG("write transfer error\n");
		err = -EIO;
	}
	else 
	{
		err = 0;
	}

	return err;
}


static int afa750_acc_hw_init(struct afa750_acc_data *acc)
{
	int err = -1;
	u8 buf[7];

	GSENSOR_MSG("afa750_acc_hw_init");

	buf[0] = AFA750_REG_WHO_AM_I;
	buf[1] =0;
	err = afa750_acc_i2c_read(acc, buf, 1);
	if (err < 0)
		goto error_firstread;
	else
		acc->hw_working = 1;

	if (buf[0] != WHOAMI_AFA750_ACC) 
	{
		err = -1; /* choose the right coded error */
		goto error_unknown_device;
	}

	buf[0] = (AFA750_REG_CTRL_PWR);
	buf[1] = AFA750_ACC_PM_ON ; 
	err = afa750_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	acc->hw_initialized = 1;
	GSENSOR_MSG("afa750_acc_hw_init: hw init done\n");

	return 0;

error_firstread:
	acc->hw_working = 0;
	GSENSOR_MSG("Error reading WHO_AM_I\n");
	goto error1;
error_unknown_device:
	GSENSOR_MSG("device unknown. Expected: 0x%x, Replies: 0x%x\n", WHOAMI_AFA750_ACC, buf[0]);
error1:
	acc->hw_initialized = 0;
	GSENSOR_MSG("hw init error Reg=0x%x, value=0x%x: %d\n", buf[0],buf[1], err);
	return err;

}




static void afa750_acc_device_power_off(struct afa750_acc_data *acc)
{
	int err;
	u8 buf[2] = { AFA750_REG_CTRL_PWR, AFA750_ACC_PM_OFF };

	GSENSOR_MSG("afa750_acc_device_power_off");
	
	err = afa750_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		GSENSOR_MSG("soft power off failed: %d\n");

	if (acc->pdata->power_off) 
	{
	
		acc->pdata->power_off();
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized) 
	{
	
		acc->hw_initialized = 0;
	}

}

static int afa750_acc_device_power_on(struct afa750_acc_data *acc)
{
	int err = -1;

	GSENSOR_MSG("afa750_acc_device_power_on");
	
	if (acc->pdata->power_on) 
	{
		err = acc->pdata->power_on();
		if (err < 0) 
		{
			GSENSOR_MSG("power_on failed: %d\n");
			return err;
		}
	}

	if (!acc->hw_initialized) 
	{
		err = afa750_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) 
		{
			afa750_acc_device_power_off(acc);
			return err;
		}
	}
	return err;
}

static irqreturn_t afa750_acc_isr1(int irq, void *dev)
{
	struct mma8452_acc_data *acc = dev;

	if(afa750_acc_misc_data != acc)
		GSENSOR_MSG("afa750_acc_isr1: afa750_acc_misc_data failed");

	disable_irq_nosync(irq);
	queue_work(afa750_acc_misc_data->irq1_work_queue, &afa750_acc_misc_data->irq1_work);
	GSENSOR_MSG("mma8452_acc_isr1");	
	return IRQ_HANDLED;
}

static irqreturn_t afa750_acc_isr2(int irq, void *dev)
{
	struct mma8452_acc_data *acc = dev;

	if(afa750_acc_misc_data != acc )
		GSENSOR_MSG("afa750_acc_isr2: afa750_acc_misc_data failed");

	disable_irq_nosync(irq);
	queue_work(afa750_acc_misc_data->irq2_work_queue, &afa750_acc_misc_data->irq2_work);
	GSENSOR_MSG("mma8452_acc_isr2");	
	return IRQ_HANDLED;
}

static void afa750_acc_irq1_work_func(struct work_struct *work)
{
//	struct afa750_acc_data *acc =	container_of(work, struct afa750_acc_data, irq1_work);

	//read x y z
	GSENSOR_MSG("IRQ1 triggered\n");
	enable_irq(afa750_acc_misc_data->irq1);
}

static void afa750_acc_irq2_work_func(struct work_struct *work)
{
//	struct afa750_acc_data *acc =	container_of(work, struct afa750_acc_data, irq2_work);

	GSENSOR_MSG("IRQ2 triggered\n");
	enable_irq(afa750_acc_misc_data->irq2);
}




int afa750_acc_update_g_range(struct afa750_acc_data *acc, u8 new_g_range)
{
	int err;
	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = AFA750_ACC_FS_MASK ;
    
	GSENSOR_MSG("afa750_acc_update_g_range");
	switch (new_g_range) 
	{
		case AFA750_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		break;
		
		case AFA750_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		break;
		
		case AFA750_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		break;
		
		//case afa750_ACC_G_16G:
		//sensitivity = SENSITIVITY_16G;
		//break;
		
		default:
		GSENSOR_MSG("invalid g range requested: %d\n",new_g_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) 
	{
		buf[0] = AFA750_REG_CTRL_FS;  
		err = afa750_acc_i2c_read(acc, buf, 1);
		if (err < 0)
		goto error;
		init_val = buf[0];
		acc->resume_state[RES_CTRL_REG4] = init_val;
		new_val = new_g_range ;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf[1] = updated_val;
		buf[0] = AFA750_REG_CTRL_FS; 
		err = afa750_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG4] = updated_val;
		acc->sensitivity = sensitivity;
		GSENSOR_MSG("sensitivity %d g-range %d\n", sensitivity, new_g_range);
	}
	
	return 0;
error:
	GSENSOR_MSG("update g range failed Reg=0x%x, Value=0x%x: %d\n", buf[0], buf[1], err);
	return err;
}



int afa750_acc_update_odr(struct afa750_acc_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	for (i = ARRAY_SIZE(afa750_acc_odr_table) - 1; i >= 0; i--) 
	{
		if (afa750_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config[1] = afa750_acc_odr_table[i].mask;
	
	if (atomic_read(&acc->enabled)) 
	{
		config[0] = AFA750_REG_CTRL_ODR; 
		err = afa750_acc_i2c_write(acc, config, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG1] = config[1];
	}
	return 0;

error:
	GSENSOR_MSG("update odr failed reg=0x%x,Value=0x%x: %d\n", config[0], config[1], err);

	return err;
}


static int afa750_acc_get_acceleration_data(struct afa750_acc_data *acc, int *xyz)
{
	int err = -1;
	u8 acc_data[6];
	s16 hw_d[3] = { 0 };

	int raw[3], data[3];
	int pos;
	int i, j, temp;

	acc_data[0] = (0x10);
	err = afa750_acc_i2c_read(acc, acc_data, 6);
	
	if (err < 0)
	{
		GSENSOR_MSG("afa750_acc_get_acceleration_data: I2C read error %d\n", err);
		return err;
	}
	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])));
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) );
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) );

	
	pos = atomic_read(&acc->position);
	raw[0]  = hw_d[0];
	raw[1]  = hw_d[1];
	raw[2]  = hw_d[2];

	for (i = 0; i < 3; i++) 
	{
		data[i] = 0;
		for (j = 0; j < 3; j++) 
		{
			data[i] += raw[j] * afa750_position_map[pos][i][j];
		}
		temp = data[i] ;
		data[i] = temp;
	}

	xyz[0] = data[0] ;
	xyz[1] = data[1] ;
	xyz[2] = data[2] ;

//	GSENSOR_MSG("read x=%d, y=%d, z=%d\n", xyz[0], xyz[1], xyz[2]);
//	GSENSOR_MSG("poll interval %d\n", acc->pdata->poll_interval);
	return err;
}

static void afa750_acc_report_values(struct afa750_acc_data *acc, int *xyz)
{	
	unsigned int X=0,Y=0,Z=0;
	if(gsensor_disable == 1)//锁屏键
    return;
   //旋转
  xyz[0] >>= 2;
	xyz[1] >>= 2;
	xyz[2] >>= 2;

  if(gsensor_st == 0) //旋转0度
  {
#ifdef CONFIG_MACH_F8
	  X = xyz[1];
    Y = xyz[0];
    Z = xyz[2];
#else
   	X = -xyz[0];
   	Y = xyz[1];
   	Z = xyz[2];
#endif
  }else if(gsensor_st == 1) //旋转90度
  {
		X = -xyz[0];
  	Y = -xyz[1];
  	Z = xyz[2];
  }else if(gsensor_st == 2) //旋转180度
  {
	  X = -xyz[1];
  	Y = xyz[0];
  	Z = xyz[2];
  }else if(gsensor_st == 3) //旋转270度
  {
		X = xyz[0];
  	Y = -xyz[1];
  	Z = xyz[2];
  }

	input_report_abs(acc->input_dev, ABS_X,X);
	input_report_abs(acc->input_dev, ABS_Y,Y);
	input_report_abs(acc->input_dev, ABS_Z,Z);
	input_sync(acc->input_dev);
	global_acc_data[0] = xyz[0];
	global_acc_data[1] = xyz[1];
	global_acc_data[2] = xyz[2];
}

static int afa750_acc_enable(struct afa750_acc_data *acc)
{
	int err=0;

	GSENSOR_MSG("afa750_acc_enable");
	if (!atomic_cmpxchg(&acc->enabled, 0, 1))
	{
		err = afa750_acc_device_power_on(acc);
		if (err < 0)
		{
			atomic_set(&acc->enabled, 0);
			return err;
		}

		if (acc->hw_initialized)
		{
			if (acc->irq1 != 0) enable_irq(acc->irq1);
			if (acc->irq2 != 0) enable_irq(acc->irq2);
			GSENSOR_MSG("afa750_acc_enable: irq enabled\n");
		}
		schedule_delayed_work(&acc->input_work, msecs_to_jiffies(acc->pdata->poll_interval));
	}

	return 0;
}

static int afa750_acc_disable(struct afa750_acc_data *acc)
{
	GSENSOR_MSG("afa750_acc_disable");
	if (atomic_cmpxchg(&acc->enabled, 1, 0))
	{
		cancel_delayed_work_sync(&acc->input_work);
		afa750_acc_device_power_off(acc);
		if (acc->irq1 != 0) disable_irq_nosync(acc->irq1);
		if (acc->irq2 != 0) disable_irq_nosync(acc->irq2);
		GSENSOR_MSG("afa750_acc_disable: irq disabled\n");
	}
	return 0;
}

static void afa750_acc_input_work_func(struct work_struct *work)
{
	struct afa750_acc_data *acc;
	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work, struct afa750_acc_data,	input_work);
	mutex_lock(&acc->lock);
	err = afa750_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		GSENSOR_MSG("afa750_acc_input_work_func: get_acceleration_data failed\n");
	else
		afa750_acc_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}

#ifdef AFA750_OPEN_ENABLE
int afa750_acc_input_open(struct input_dev *input)
{
	struct afa750_acc_data *acc = input_get_drvdata(input);

	return afa750_acc_enable(acc);
}

void afa750_acc_input_close(struct input_dev *dev)
{
	struct afa750_acc_data *acc = input_get_drvdata(dev);

	afa750_acc_disable(acc);
}
#endif

static int afa750_acc_validate_pdata(struct afa750_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval, acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 || acc->pdata->axis_map_y > 2 || acc->pdata->axis_map_z > 2)
	{
		GSENSOR_MSG("invalid axis_map value x:%d y:%d z%d\n", acc->pdata->axis_map_x, acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1 || acc->pdata->negate_z > 1) {
		GSENSOR_MSG("invalid negate value x:%d y:%d z:%d\n", acc->pdata->negate_x, acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		GSENSOR_MSG( "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int afa750_acc_input_init(struct afa750_acc_data *acc)
{
	int err;
	// Polling rx data when the interrupt is not used.
	if (1/*acc->irq1 == 0 && acc->irq1 == 0*/)
	{
		INIT_DELAYED_WORK(&acc->input_work, afa750_acc_input_work_func);
	}

	acc->input_dev = input_allocate_device();
	if (!acc->input_dev)
	{
		err = -ENOMEM;
		GSENSOR_MSG("input device allocate failed\n");
		goto err0;
	}

	#ifdef AFA750_ACC_OPEN_ENABLE
	acc->input_dev->open = afa750_acc_input_open;
	acc->input_dev->close = afa750_acc_input_close;
	#endif

	input_set_drvdata(acc->input_dev, acc);
	set_bit(EV_ABS, acc->input_dev->evbit);
	//next is used for interruptA sources data if the case 
	set_bit(ABS_MISC, acc->input_dev->absbit);
	//next is used for interruptB sources data if the case
	set_bit(ABS_WHEEL, acc->input_dev->absbit);
	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	//next is used for interruptA sources data if the case 
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	//next is used for interruptB sources data if the case 
	input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);
//	acc->input_dev->name = "accelerometer";
		acc->input_dev->name = "mma8452";
		
	err = input_register_device(acc->input_dev);
	if (err) 
	{
		GSENSOR_MSG("unable to register input polled device %s\n", acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}



static ssize_t afa750_poll_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct afa750_acc_data *acc = i2c_get_clientdata(client);
	int interval=0;

	if(afa750_acc_misc_data != acc )
		GSENSOR_MSG("afa750_poll_show: afa750_acc_misc_data failed");

	interval = afa750_acc_misc_data->pdata->poll_interval;
	GSENSOR_MSG("afa750_poll_show: poll_interval=%d ",interval);
	
	return sprintf(buf, "%d\n", interval);
}


static ssize_t afa750_poll_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct afa750_acc_data *acc = i2c_get_clientdata(client);
//	struct input_dev *input_dev = acc->input_dev;
	unsigned int interval =0;
	
	interval = (unsigned int)simple_strtoul(buf, NULL, 10);
	if(afa750_acc_misc_data != acc )
		GSENSOR_MSG("afa750_poll_store: afa750_acc_misc_data failed");

	//GSENSOR_MSG("afa750_poll_store: Raw poll=%d(jiffies)",interval);
	//interval =  msecs_to_jiffies(interval);
	GSENSOR_MSG("afa750_poll_store: ms=%d",interval);
	afa750_acc_misc_data->pdata->poll_interval = max(interval, afa750_acc_misc_data->pdata->min_interval);
	//afa750_acc_update_odr(acc, acc->pdata->poll_interval);
	//GSENSOR_MSG("afa750_poll_store: ms1=%d",interval);
	return count;
}


static ssize_t afa750_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct afa750_acc_data *acc = i2c_get_clientdata(client);
//	struct input_dev *input_dev = acc->input_dev;
	unsigned int enable=0;

	#if 0// (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
	int error;
	error = kstrtouint(buf, 10, &enable);
	if (error < 0)
		return error;
	#else
	enable = (unsigned int)simple_strtoul(buf, NULL, 10);
	#endif
	GSENSOR_MSG("afa750_enable_store: enable=%d",enable);

	if(afa750_acc_misc_data != acc )
		GSENSOR_MSG("afa750_enable_store: afa750_acc_misc_data failed");

	if(enable)
	{
		afa750_acc_enable(afa750_acc_misc_data);
	}
	else
	{
//		afa750_acc_disable(afa750_acc_misc_data);
	}
	return count;
}



static ssize_t afa750_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct afa750_acc_data *acc = i2c_get_clientdata(client);
	int enable=0;

	if(afa750_acc_misc_data != acc )
		GSENSOR_MSG("afa750_enable_show: afa750_acc_misc_data failed");

	enable = atomic_read(&afa750_acc_misc_data->enabled);	
	GSENSOR_MSG("afa750_enable_show: enable=%d",enable);
	return sprintf(buf, "%d\n", enable);
}


static ssize_t afa750_accdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct afa750_acc_data *acc = i2c_get_clientdata(client);

	
	if(afa750_acc_misc_data != acc )
		GSENSOR_MSG("afa750_accdata_show: afa750_acc_misc_data failed");

	//GSENSOR_MSG("afa750_get_accdata:x=%d,y=%d, z=%d",global_acc_data[0],global_acc_data[1] ,global_acc_data[2] );
	if (!atomic_read(&afa750_acc_misc_data->enabled)) 
	{
		global_acc_data[0] =0;
		global_acc_data[1] =0;
		global_acc_data[2] =0;
	}
	return sprintf(buf, "%04x%04x%04x\n", global_acc_data[0],global_acc_data[1],global_acc_data[2]);
}


static ssize_t afa750_position_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct afa750_acc_data *acc = i2c_get_clientdata(client);
	int position = 0;
	
	if(afa750_acc_misc_data != acc )
		GSENSOR_MSG("afa750_position_show: afa750_acc_misc_data failed");

	position =  atomic_read(&afa750_acc_misc_data->position);
	GSENSOR_MSG("afa750_position_show: positon=%d",position); 
	return sprintf(buf, "%d\n", position);
}

static ssize_t afa750_position_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct afa750_acc_data *acc = i2c_get_clientdata(client);
	unsigned long position =0;
	
	if(afa750_acc_misc_data != acc )
		GSENSOR_MSG("afa750_position_store: afa750_acc_misc_data failed");
		
	position = simple_strtoul(buf, NULL,10);
	GSENSOR_MSG("afa750_position_store: positon=%d",position); 
	if ((position >= 0) && (position <= 7)) 
	{		
		atomic_set(&afa750_acc_misc_data->position, position);
	}
	return count;
}


static DEVICE_ATTR(poll,0666 , afa750_poll_show, afa750_poll_store); //poll
static DEVICE_ATTR(enable, 0666, afa750_enable_show, afa750_enable_store);
static DEVICE_ATTR(accdata, 0666, afa750_accdata_show, NULL);
static DEVICE_ATTR(position, 0666, afa750_position_show, afa750_position_store);

static struct attribute *afa750_attributes[] = {
	&dev_attr_poll.attr,
	&dev_attr_enable.attr,
	&dev_attr_accdata.attr,
	&dev_attr_position.attr,
	NULL
};

static struct attribute_group afa750_attribute_group = {
	.attrs = afa750_attributes
};



static void afa750_acc_input_cleanup(struct afa750_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static int afa750_acc_resume(struct i2c_client *client)
{
	struct afa750_acc_data *acc = i2c_get_clientdata(client);
	
	GSENSOR_MSG("afa750_acc_resume");
	if (acc != NULL && acc->on_before_suspend) 
	{
		acc->on_before_suspend = 0;
		return afa750_acc_enable(acc);
	}

	return 0;
}

static int afa750_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct afa750_acc_data *acc = i2c_get_clientdata(client);

	GSENSOR_MSG("afa750_acc_suspend");
	if (acc != NULL) 
	{
		if (atomic_read(&acc->enabled)) 
		{
			acc->on_before_suspend = 1;
			return afa750_acc_disable(acc);
		}
	}
	return 0;
}

static void afa750_early_suspend (struct early_suspend* es)
{
	GSENSOR_MSG("afa750_early_suspend");
	afa750_acc_suspend(afa750_i2c_client, (pm_message_t){.event=0});
}

static void afa750_early_resume (struct early_suspend* es)
{ 
	GSENSOR_MSG("afa750_early_resume");
	afa750_acc_resume(afa750_i2c_client);
}

static int afa750_acc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct afa750_acc_data *acc;
	int err = -1;
//	int tempvalue;
//	u8 buf[7];

	GSENSOR_MSG("afa750_acc_probe:enter");
//	if (client->dev.platform_data == NULL) 
//	{
//		GSENSOR_MSG("platform data is NULL. exiting.\n");
//		err = -ENODEV;
//		goto exit_check_functionality_failed;
//	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		GSENSOR_MSG("client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_BYTE |I2C_FUNC_SMBUS_BYTE_DATA |I2C_FUNC_SMBUS_WORD_DATA)) 
	{
		GSENSOR_MSG("client not smb-i2c capable:2\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}


	if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_I2C_BLOCK))
	{
		GSENSOR_MSG("client not smb-i2c capable:3\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}

	acc = kzalloc(sizeof(struct afa750_acc_data), GFP_KERNEL);
	if (acc == NULL) 
	{
		err = -ENOMEM;
		GSENSOR_MSG("failed to allocate memory for module data: %d\n", err);
		goto exit_alloc_data_failed;
	}
	afa750_acc_misc_data = acc;

	mutex_init(&acc->lock);
	
	mutex_lock(&acc->lock);

	acc->client = client;
	afa750_i2c_client = client;
	i2c_set_clientdata(client, acc);

//	if(AFA750_INT1)
//	acc->irq1 = sprd_alloc_eic_irq(AFA750_INT1);
//	if(AFA750_INT2)
//	acc->irq2 = sprd_alloc_eic_irq(AFA750_INT2);
	acc->irq1 = 0;
	acc->irq2 = 0;
//	GSENSOR_MSG("afa750_acc_probe: has set irq1 to irq: %d\n", acc->irq1);
//	GSENSOR_MSG("afa750_acc_probe: has set irq2 to irq: %d\n", acc->irq2);
	     
	if (acc->irq1 != 0) 
	{
		err = request_irq(acc->irq1, afa750_acc_isr1, IRQF_TRIGGER_HIGH, "afa750_acc_irq1", acc);
		if (err < 0) 
		{
			GSENSOR_MSG("afa750_acc_probe: request irq1 failed: %d\n", err);
			goto err_mutexunlockfreedata;
		}
		else
		{
			GSENSOR_MSG("afa750_acc_probe: request_irq1 = %d",acc->irq1);
			disable_irq_nosync(acc->irq1);//disable_irq(acc->irq1);
		}

		INIT_WORK(&acc->irq1_work, afa750_acc_irq1_work_func);
		acc->irq1_work_queue = create_singlethread_workqueue("afa750_acc_wq1");
		if (!acc->irq1_work_queue) 
		{
			err = -ENOMEM;
			GSENSOR_MSG("cannot create work queue1: %d\n", err);
			goto err_free_irq1;
		}
	}

	if (acc->irq2 != 0) 
	{
		err = request_irq(acc->irq2, afa750_acc_isr2, IRQF_TRIGGER_HIGH, "afa750_acc_irq2", acc);
		if (err < 0) 
		{
			GSENSOR_MSG("request irq2 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		else
		{
			GSENSOR_MSG("afa750_acc_probe: request_irq2 = %d",acc->irq2);
			disable_irq_nosync(acc->irq2);//disable_irq(acc->irq1);
		}
		
		INIT_WORK(&acc->irq2_work, afa750_acc_irq2_work_func);
		acc->irq2_work_queue = create_singlethread_workqueue("afa750_acc_wq2");
		if (!acc->irq2_work_queue) 
		{
			err = -ENOMEM;
			GSENSOR_MSG("cannot create work queue2: %d\n", err);
			goto err_free_irq2;
		}
	}

	/*
	buf[0] = AFA750_REG_WHO_AM_I;
	err = afa750_acc_i2c_read(acc, buf, 1);
	if (err < 0 || buf[0] != WHOAMI_AFA750_ACC)
	{
		GSENSOR_MSG("afa750_acc_probe: failed to read  afa750 chip id  unknown %d\n",buf[0]);
		goto err_destoyworkqueue2;
	}*/
		
	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) 
	{
		err = -ENOMEM;
		GSENSOR_MSG("failed to allocate memory for pdata: %d\n",err);
		goto err_destoyworkqueue2;
	}

//	memcpy(acc->pdata, client->dev.platform_data, sizeof(*acc->pdata));
	memcpy(acc->pdata, &afa750_plat_data, sizeof(*acc->pdata));
	
	err = afa750_acc_validate_pdata(acc);
	if (err < 0) 
	{
		GSENSOR_MSG("afa750_acc_probe: failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	i2c_set_clientdata(client, acc);
	
	if (acc->pdata->init) 
	{
		err = acc->pdata->init();
		if (err < 0) 
		{
			GSENSOR_MSG("afa750_acc_probe: init failed: %d\n", err);
			goto err2;
		}
	}

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	//acc->resume_state[RES_TT_TW] = 0x00;
	err = afa750_acc_device_power_on(acc);
	if (err < 0) 
	{
		GSENSOR_MSG("afa750_acc_probe: power on failed: %d\n");
		goto err2;  
	}

	/*
	atomic_set(&acc->enabled, 1);
	err = afa750_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) 
	{
		GSENSOR_MSG("update_g_range failed\n");
		goto  err_power_off;
	}

	err = afa750_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) 
	{
		GSENSOR_MSG("update_odr failed\n");
		goto  err_power_off;
	}
	*/
	
	err = afa750_acc_input_init(acc);
	if (err < 0) 
	{
		GSENSOR_MSG("afa750_acc_probe: input init failed\n");
		goto err_power_off;
	}

	afa750_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);
	atomic_set(&afa750_acc_misc_data->position, AFA750_DEFAULT_POSITION);

	acc->on_before_suspend = 0;
	
	acc->early_suspend.suspend = afa750_early_suspend;
	acc->early_suspend.resume  = afa750_early_resume;
	acc->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&acc->early_suspend);
	
	mutex_unlock(&acc->lock);

	err = sysfs_create_group(&acc->input_dev->dev.kobj, &afa750_attribute_group);
	if (err) 
	{
		GSENSOR_MSG("afa750_acc_probe: sysfs create failed: %d", err);	
		goto err_input_cleanup;
	}
	//fighter++
	afa750_acc_enable(acc);
	err = device_create_file(switch_dev, &dev_attr_gsensor_st);
	err = device_create_file(switch_dev, &dev_attr_gsensor_disable);
	//fighter--
	GSENSOR_MSG( "afa750_acc_probe: probed\n");
	printk("afa750_acc_probe: probed\n");
	return 0;

err_input_cleanup:
	afa750_acc_input_cleanup(acc);
err_power_off:
	afa750_acc_device_power_off(acc);
err2:
	if (acc->pdata->exit) acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_destoyworkqueue2:
	if(acc->irq2)
 	destroy_workqueue(acc->irq2_work_queue);
err_free_irq2:
	if (acc->irq2) free_irq(acc->irq2, acc);
err_destoyworkqueue1:
	if(acc->irq1)
	destroy_workqueue(acc->irq1_work_queue);
err_free_irq1:
	if (acc->irq1) free_irq(acc->irq1, acc);
err_mutexunlockfreedata:
//	if(acc->irq1)
//	sprd_free_eic_irq(acc->irq1);
//	if(acc->irq2)
//	sprd_free_eic_irq(acc->irq2);
	
	mutex_unlock(&acc->lock);
	i2c_set_clientdata(client, NULL);
	kfree(acc);
	acc = NULL;
	afa750_acc_misc_data = NULL;
exit_alloc_data_failed:
exit_check_functionality_failed:
	printk(" afa750_acc_probe Probe failed\n");
	return err;
}

static int __devexit afa750_acc_remove(struct i2c_client *client)
{
	struct afa750_acc_data *acc = i2c_get_clientdata(client);

	GSENSOR_MSG("afa750_acc_remove");
	
	if (acc != NULL) 
	{
		if (acc->irq1 != 0) free_irq(acc->irq1, acc);
		if (acc->irq2 != 0) free_irq(acc->irq2, acc);

//		if (acc->irq1 != 0) sprd_free_eic_irq(acc->irq1);
//		if (acc->irq2 != 0) sprd_free_eic_irq(acc->irq2);
		//if (acc->irq1 != 0) sprd_alloc_gpio_irq(acc->irq1);
		//if (acc->irq2 != 0) sprd_alloc_gpio_irq(acc->irq2);

		destroy_workqueue(acc->irq1_work_queue);
		destroy_workqueue(acc->irq2_work_queue);		
		afa750_acc_input_cleanup(acc);
		afa750_acc_device_power_off(acc);
		if (acc->pdata->exit)
			acc->pdata->exit();
		kfree(acc->pdata);
		kfree(acc);
	}
	device_remove_file(switch_dev, &dev_attr_gsensor_st);
  device_remove_file(switch_dev, &dev_attr_gsensor_disable);
	return 0;
}


static const struct i2c_device_id afa750_acc_id[] = {
	{AFA750_ACC_DEV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, afa750_acc_id);

struct i2c_driver afa750_acc_driver ={
	.driver = {
		.name = AFA750_ACC_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe = afa750_acc_probe,
	.remove = afa750_acc_remove,
	.id_table = afa750_acc_id,
};



#if I2C_BOARD_INFO_METHOD

static int __init afa750_acc_init(void)
{       

	GSENSOR_MSG("afa750_acc_init\n");
	return i2c_add_driver(&afa750_acc_driver);
}

static void __exit afa750_acc_exit(void)
{	
	GSENSOR_MSG("afa750_acc_exit\n");
	i2c_del_driver(&afa750_acc_driver);
	return;
}
#else

struct sprd_i2c_setup_data 
{
	unsigned i2c_bus;  //the same number as i2c->adap.nr in adapter probe function
	unsigned short i2c_address;
	int irq;
	char type[I2C_NAME_SIZE];
};

static struct sprd_i2c_setup_data afa750_setup={AFA750_I2C_BUS_NUM, AFA750_ACC_I2C_ADDR, 0, AFA750_ACC_DEV_NAME};

int sprd_add_i2c_device(struct sprd_i2c_setup_data *i2c_set_data, struct i2c_driver *driver)
{
	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int ret,err;


	GSENSOR_MSG("sprd_add_i2c_device: i2c_bus=%d; slave_address=0x%x; i2c_name=%s",i2c_set_data->i2c_bus,  i2c_set_data->i2c_address, i2c_set_data->type);

	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = i2c_set_data->i2c_address;
	strlcpy(info.type, i2c_set_data->type, I2C_NAME_SIZE);
	if(i2c_set_data->irq > 0)
		info.irq = i2c_set_data->irq;

	info.platform_data = &afa750_plat_data;

	adapter = i2c_get_adapter( i2c_set_data->i2c_bus);
	if (!adapter) {
		printk("%s: can't get i2c adapter %d\n",
			__func__,  i2c_set_data->i2c_bus);
		err = -ENODEV;
		goto err_driver;
	}

	client = i2c_new_device(adapter, &info);
	if (!client) {
		printk("%s:  can't add i2c device at 0x%x\n",
			__func__, (unsigned int)info.addr);
		err = -ENODEV;
		goto err_driver;
	}

	i2c_put_adapter(adapter);

	ret = i2c_add_driver(driver);
	if (ret != 0) {
		printk("%s: can't add i2c driver\n", __func__);
		err = -ENODEV;
		goto err_driver;
	}	

	return 0;

err_driver:
	return err;
}

void sprd_del_i2c_device(struct i2c_client *client, struct i2c_driver *driver)
{
	GSENSOR_MSG("sprd_del_i2c_device: slave_address=0x%x; i2c_name=%s", client->addr, client->name);
	i2c_unregister_device(client);
	i2c_del_driver(driver);
}

static int __init afa750_acc_init(void)
{
	int  ret = 0;        

    	GSENSOR_MSG("afa750_acc_init\n");

	//gpio_request(91, "afa750_sa0");
	//gpio_direction_output(91, 1);
	//gpio_set_value(91, 1);
	//gpio_free(91);
		
	afa750_setup.i2c_bus = AFA750_I2C_BUS_NUM;
	afa750_setup.i2c_address = AFA750_ACC_I2C_ADDR;
	strcpy (afa750_setup.type,AFA750_ACC_I2C_NAME);
	afa750_setup.irq = 0;
	return sprd_add_i2c_device(&afa750_setup, &afa750_acc_driver);
}

static void __exit afa750_acc_exit(void)
{	
	GSENSOR_MSG("afa750_acc_exit\n");
	if(afa750_i2c_client)
		sprd_del_i2c_device(afa750_i2c_client, &afa750_acc_driver);
}

#endif


module_init(afa750_acc_init);
module_exit(afa750_acc_exit);

MODULE_DESCRIPTION("afa750 accelerometer misc driver");
MODULE_AUTHOR("Afamicro");
MODULE_LICENSE("GPL");

