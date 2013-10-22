/*
 *  include <mach/apds9900.h>
 *  
 *  Copyright (C) 2006, Marvell Corporation.
 *  
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_APDS9900_H
#define __ASM_ARCH_APDS9900_H

#define APDS9900_DRV_NAME	"apds9900"
#define DRIVER_VERSION		"1.0.3"
#define APDS9900_ALS_THRESHOLD_HSYTERESIS	20	/* 20 = 20% */

#define APDS9900_ENABLE_REG	0x00
#define APDS9900_ATIME_REG	0x01
#define APDS9900_PTIME_REG	0x02
#define APDS9900_WTIME_REG	0x03
#define APDS9900_AILTL_REG	0x04
#define APDS9900_AILTH_REG	0x05
#define APDS9900_AIHTL_REG	0x06
#define APDS9900_AIHTH_REG	0x07
#define APDS9900_PILTL_REG	0x08
#define APDS9900_PILTH_REG	0x09
#define APDS9900_PIHTL_REG	0x0A
#define APDS9900_PIHTH_REG	0x0B
#define APDS9900_PERS_REG	0x0C
#define APDS9900_CONFIG_REG	0x0D
#define APDS9900_PPCOUNT_REG	0x0E
#define APDS9900_CONTROL_REG	0x0F
#define APDS9900_REV_REG	0x11
#define APDS9900_ID_REG		0x12
#define APDS9900_STATUS_REG	0x13
#define APDS9900_CDATAL_REG	0x14
#define APDS9900_CDATAH_REG	0x15
#define APDS9900_IRDATAL_REG	0x16
#define APDS9900_IRDATAH_REG	0x17
#define APDS9900_PDATAL_REG	0x18
#define APDS9900_PDATAH_REG	0x19

#define CMD_BYTE	0x80
#define CMD_WORD	0xA0
#define CMD_SPECIAL	0xE0

#define CMD_CLR_PS_INT	0xE5
#define CMD_CLR_ALS_INT	0xE6
#define CMD_CLR_PS_ALS_INT	0xE7

/********************************************
             APDS9900_ENABLE_REG
*********************************************/
#define PIEN   (1<<5)    /*Proximity Interrupt Enable*/
#define AIEN   (1<<4)    /*ALS Interrupt Enable*/
#define WEN    (1<<3)    /*Wait Enable*/
#define PEN    (1<<2)    /*Proximity Enable*/
#define AEN    (1<<1)    /*ALS Enable*/
#define PON    (1<<0)    /*Power ON*/

/********************************************
             APDS9900_REG_PERS
*********************************************/
#define PPERS  (0xF<<4)  /*Proximity interrupt persistence*/
#define APERS  (0xF<<0)  /*ALS interrupt persistence*/

/********************************************
             APDS9900_CONTROL_REG
*********************************************/
#define PDRIVE  (0x3<<6) /* LED Drive Strength:
                            00 -- 100mA
                            01 -- 50mA
                            10 -- 25mA
                            11 -- 12.5mA  */

/********************************************
             APDS9900_STATUS_REG
*********************************************/
#define PINT   (1<<5)  /*Proximity Interrupt*/
#define AINT   (1<<4)  /*ALS Interrupt*/
#define PVALID (1<<1)  /*PS Valid*/
#define AVALID (1<<0)  /*ALS Valid*/
/*
 * Structs
 */
typedef enum {
    APDS_CLR_PS_INT=0,
    APDS_CLR_ALS_INT=1,
    APDS_CLR_PS_ALS_INT=2,
}APDS_COMMAND;

typedef enum {
    PS_NEAR_TO_FAR=0,
    PS_FAR_TO_NEAR=1,
    PS_NO_MOVE=2,
}PS_STATUS;

typedef enum {
    ALS_DARK_TO_BRIGHT=0,
    ALS_BRIGHT_TO_DARK=1,
    ALS_NO_CHANGE=2,
}ALS_STATUS;

struct apds9900_data {
	unsigned int atime;
	/* PS parameters */
	PS_STATUS    ps_detection;		/* 0 = near-to-far; 1 = far-to-near */
	unsigned int pilt;	/* low data */
	unsigned int piht;	/* high data */
	unsigned int pilt_threshold;	/* low threshold */
	unsigned int piht_threshold;	/* high threshold */
	unsigned int ps_data;			/* to store PS data */
	/* ALS parameters */
	ALS_STATUS   als_detection;		/* 0 = near-to-far; 1 = far-to-near */
	unsigned int ailt;	/* low data */
	unsigned int aiht;	/* high data */
	unsigned int ailt_threshold;	/* low threshold */
	unsigned int aiht_threshold;	/* high threshold */
	unsigned int cdata;			/* to store ALS data */
	unsigned int irdata;			/* to store ALS data */
};

struct apds9900_platform_data {
	int	(*init_irq)(void);
	int	(*ack_irq)(void);
};
#endif /* __ASM_ARCH_APDS9900_H */