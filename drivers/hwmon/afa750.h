
/******************** Jerry xiang /2012-8*************************************
	

 ******************************************************************************/


#ifndef	__AFA750_H__
#define	__AFA750_H__

#include	<linux/ioctl.h>	/* For IOCTL macros */
#include	<linux/input.h>


#define	AFA750_DEBUG_MSG		0
#define	I2C_BOARD_INFO_METHOD	1
#if I2C_BOARD_INFO_METHOD
#else
#define	AFA750_I2C_BUS_NUM		3
#endif

#if AFA750_DEBUG_MSG
#define	GSENSOR_MSG(format, ...)	\
		printk(KERN_ERR "kylin: afa750 " format "\n", ## __VA_ARGS__)
#else
#define	GSENSOR_MSG(format, ...)
#endif

#define	AFA750_ACC_DEV_NAME					"afa750"

#define	AFA750_ACC_I2C_ADDR		 				0x3C
#define	AFA750_ACC_I2C_NAME						AFA750_ACC_DEV_NAME

#define	AFA750_INT1								0 //EIC_ID_0 // sprd_3rdparty_gpio_gint1_irq
#define	AFA750_INT2								0 //EIC_ID_1 //sprd_3rdparty_gpio_gint2_irq

#define	AFA750_DEFAULT_POSITION					3 



#endif	/* __AFA750_H__ */



