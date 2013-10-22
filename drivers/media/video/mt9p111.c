/*------------------------------------------------------------------------------
* (c) Copyright, Augusta Technology, Inc., 2006-present.
* (c) Copyright, Augusta Technology USA, Inc., 2006-present.
*  
* This software, document, web pages, or material (the "Work") is copyrighted 
* by its respective copyright owners.  The Work may be confidential and 
* proprietary.  The Work may be further protected by one or more patents and 
* be protected as a part of a trade secret package.
*   
* No part of the Work may be copied, photocopied, reproduced, translated, or 
* reduced to any electronic medium or machine-readable form, in whole or in 
* part, without prior written consent of the copyright owner. Any other 
* reproduction in any form without the permission of the copyright owner is 
* prohibited.
*   
* All Work are protected by the copyright laws of all relevant jurisdictions, 
* including protection under the United States copyright laws, and may not be 
* reproduced, distributed, transmitted, displayed, published, or broadcast 
* without the prior written permission of the copyright owner.
*
------------------------------------------------------------------------------*/
#include <linux/module.h>
#include <linux/errno.h> 
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h> 
#include <linux/time.h>
#include <mach/gpio.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/mixtile_camera.h>
#include <mach/gpio-mixtile4x12.h>

#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif

#include "mt9p111.h"

extern int mixtilev310_cam_b_power(int onoff);
extern void flash_led_power(int onoff, int mode);
static int initted = 0;
struct i2c_client *mt9p111_client = NULL;
#define PRINTK(x,y...) do{if(x<1) v4l_info(mt9p111_client, y);}while(0)

#define MT9P111_DRIVER_NAME "MT9P111"

static struct cam_res {
	const char		*name;
	unsigned long		width;
	unsigned long		height;
} cam_res_list[] = {
	{
		.name		= "640x480, VGA",
		.width		= 640,
		.height		= 480
	}, {
		.name		= "800x600, SVGA",
		.width		= 800,
		.height		= 600
	}, {
		.name		= "1280x720, HD720P",
		.width		= 1280,
		.height		= 720
	}, {
		.name		= "1280x960, SXGA",
		.width		= 1280,
		.height		= 960
	}, {
		.name		= "1600x1200, UXGA",
		.width		= 1600,
		.height		= 1200
	}, {
		.name		= "2048x1536, QXGA",
		.width		= 2048,
		.height		= 1536
	}, {
		.name		= "2592x1936, QSXGA",
		.width		= 2592,
		.height		= 1936
	}
};

#define N_RESOLUTIONS ARRAY_SIZE(cam_res_list)

static int mt9p111_i2c_rxdata(unsigned short saddr, unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = 2,
			.buf = rxdata,
		},
		{
			.addr = saddr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxdata,
		},
	};
	if (i2c_transfer(mt9p111_client->adapter, msgs, 2) < 0) {
		printk("mt9p111_i2c_rxdata failed!\n");
		return -EIO;
	}
	return 0;
}

static int32_t mt9p111_i2c_read(unsigned short saddr, unsigned short raddr, unsigned short *rdata, enum mt9p111_enum width)
{
	int32_t rc = 0;
	unsigned char buf[4];
	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));
	switch (width) {
  	case WORD_LEN:
  	{
  		buf[0] = (unsigned char)((raddr & 0xFF00)>>8);
  		buf[1] = (unsigned char)(raddr & 0x00FF);
  
  		rc = mt9p111_i2c_rxdata(saddr, buf, 2);
  		if (rc < 0)
  			return rc;
      
  		*rdata = (unsigned short)((buf[0] << 8) | buf[1]);
          //printk("word 0x%x  0x%x\n", raddr,*rdata);
  		break;
  	}

    case BYTE_LEN:
    {
  		buf[0] = (unsigned char)((raddr & 0xFF00)>>8);
  		buf[1] = (unsigned char)(raddr & 0x00FF);
  
  		rc = mt9p111_i2c_rxdata(saddr, buf, 1);
  		if (rc < 0)
  			return rc;
     
  		*rdata = (unsigned short)buf[0];// << 8 | buf[1];
  		break;
	  }

  	default:
  		break;
	}
	if (rc < 0)
		printk(KERN_ERR "mt9p111_i2c_read failed!\n");
	return rc;
}

static int32_t mt9p111_i2c_txdata(unsigned short saddr, unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
		 .addr = saddr,
		 .flags = 0,
		 .len = length,
		 .buf = txdata,
		 },
	};

	if (i2c_transfer(mt9p111_client->adapter, msg, 1) < 0) {
		printk("mt9p111_i2c_txdata failed\n");
		return -EIO;
	}

	return 0;
}

static int32_t mt9p111_i2c_write(unsigned short saddr, unsigned short waddr, unsigned short wdata, enum mt9p111_enum width)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	switch (width) {
	case WORD_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = (wdata & 0xFF00)>>8;
		buf[3] = (wdata & 0x00FF);

		rc = mt9p111_i2c_txdata(saddr, buf, 4);
	}
		break;

	case BYTE_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = (wdata & 0x00FF);
		rc = mt9p111_i2c_txdata(saddr, buf, 3);
	}
		break;

	default:
		break;
	}

	if (rc < 0)
		printk(KERN_ERR 
		"i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int32_t mt9p111_i2c_write_table(struct mt9p111_i2c_reg_conf const *reg_conf_tbl, int num_of_items_in_table)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num_of_items_in_table; i++) {
		rc = mt9p111_i2c_write(mt9p111_client->addr, reg_conf_tbl->waddr, reg_conf_tbl->wdata, reg_conf_tbl->width);
		if (rc < 0)
			break;
		if (reg_conf_tbl->mdelay_time != 0)
			msleep(reg_conf_tbl->mdelay_time);
		reg_conf_tbl++;
	}

	return rc;
}

static inline struct mt9p111_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9p111_info, sd);
}

static long mt9p111_reg_init(void)
{
	int rc = 0;
	PRINTK(1, "%s begin!\n", __func__);
	
	rc = mt9p111_i2c_write_table(&mt9p111_reg_init_tab[0], ARRAY_SIZE(mt9p111_reg_init_tab));
	if (rc < 0)
		return rc;

	PRINTK(1, "%s end!\n", __func__);
	return 0;
}

static int32_t mt9p111_set_sensor_mode(int mode)
{
	int32_t rc = 0;
	unsigned short ret, check_ret;

  PRINTK(1, "%s\n", __func__);
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		mt9p111_i2c_write_table(&mt9p111_reg_preview[0], ARRAY_SIZE(mt9p111_reg_preview));
		check_ret = 0x03;
		break;
	case SENSOR_SNAPSHOT_MODE:
		mt9p111_i2c_write_table(&mt9p111_reg_capture[0], ARRAY_SIZE(mt9p111_reg_capture));
		check_ret = 0x07;
		break;
	default:
		return -EINVAL;
	}
	
	do
	{
	  mt9p111_i2c_read(mt9p111_client->addr, 0x8405, &ret, BYTE_LEN);
    PRINTK(3, "ret=0x%x\n", ret);
	  if(ret == check_ret)
	  {
	    PRINTK(1, "%s success.\n", __func__);
	    break;
	  }
	  msleep(50);
	  rc++;
	}while(rc<=10);
	
	return 0;
}

static void mt9p111_set_brightness(int value)
{
  PRINTK(1, "%s %d\n", __func__, value);
	if((value>=EV_MINUS_4) && (value<=EV_PLUS_4))
	  mt9p111_i2c_write_table(brightness_tbl[value+4], brightness_tbl_sz[value+4]);
}

static void mt9p111_set_wb(int value)
{
  PRINTK(1, "%s %d\n", __func__, value);
	if((value>=WHITE_BALANCE_AUTO) && (value<=WHITE_BALANCE_FLUORESCENT))
	  mt9p111_i2c_write_table(wb_tbl[value-1], wb_tbl_sz[value-1]);
}

static void mt9p111_set_effect(int value)
{
  PRINTK(1, "%s %d\n", __func__, value);
	if((value>=IMAGE_EFFECT_NONE) && (value<=IMAGE_EFFECT_SHARPEN))
	  mt9p111_i2c_write_table(effect_tbl[value-1], effect_tbl_sz[value-1]);
}

static void mt9p111_set_iso(int value)
{
  PRINTK(1, "%s %d\n", __func__, value);
	if((value>=ISO_AUTO) && (value<=ISO_800))
	  mt9p111_i2c_write_table(iso_tbl[value], iso_tbl_sz[value]);
}

static void mt9p111_set_scene(int value)
{
  PRINTK(1, "%s %d\n", __func__, value);
	if((value>=SCENE_MODE_NONE) && (value<=SCENE_MODE_CANDLE_LIGHT))
	  mt9p111_i2c_write_table(scene_tbl[value-1], scene_tbl_sz[value-1]);
}

static void mt9p111_set_zoom(int value)
{
  PRINTK(1, "%s %d\n", __func__, value);
	if((value>=SCENE_MODE_NONE) && (value<=SCENE_MODE_CANDLE_LIGHT))
	  mt9p111_i2c_write_table(zoom_tbl[value], zoom_tbl_sz[value]);
}

static void mt9p111_set_flash(int value)
{
  switch(value)
  {
    case FLASH_MODE_ON:
      flash_led_power(1,1);
      break;
    case FLASH_MODE_OFF:
      flash_led_power(0,0);
      break;
  }
}

/* Auto Focus Trigger */
//static int32_t mt9p111_af_trigger(unsigned int value)
//{
//  uint16_t af_status;
//  uint32_t i;
//  int32_t rc;
//
//  PRINTK(1, "%s: entry\n", __func__);
//
//  /* AF trigger */
//  mt9p111_i2c_write(mt9p111_client->addr, 0x098E, 0xB006, WORD_LEN);
//  mt9p111_i2c_write(mt9p111_client->addr, 0xB006, 0x01, BYTE_LEN);
//
//  mdelay(50);
//  for (i = 0; i < 50; ++i)
//  {     
//    rc = mt9p111_i2c_write(mt9p111_client->addr, 0x098E, 0xB000, WORD_LEN);
//    if (rc < 0)
//    {
//      PRINTK(1, "%s: i2c_write error\n", __func__);
//      return rc;
//    }
//
//    af_status = 0x0000;
//    rc = mt9p111_i2c_read(mt9p111_client->addr, 0xB000, &af_status, WORD_LEN);  
//    if (rc < 0)
//    {
//      PRINTK(1, "%s: i2c_read error\n", __func__);
//      return rc;
//    }
//
////PRINTK(1, "%s: 0xB000=0x%x\n", __func__, af_status);
////    if (0x0010 == af_status)
//    if (0x8010 == af_status)
//    {
//      PRINTK(1, "%s: success\n", __func__);
//      return 0;
//    }
//    /* To improve efficiency of switch between preview and snapshot mode, decrease time delay from 20ms to 120ms */
//    mdelay(50);
//  }
//  return -EIO;
//}

static int32_t mt9p111_af_trigger(unsigned int value)
{
  //fighter++
  mt9p111_i2c_write(mt9p111_client->addr, 0x098E, 0xB006, WORD_LEN);
  mt9p111_i2c_write(mt9p111_client->addr, 0xB006, 0x01, BYTE_LEN);
	return 0;
 //fighter--
}

static int mt9p111_set_reflect(u32 value)
{
  int rc = 0;
  switch(value) {
  case 0:
  default:
	  rc = mt9p111_i2c_write_table(&mt9p111_reg_reflect_none[0], ARRAY_SIZE(mt9p111_reg_reflect_none));
	  break;
  case 1:	
	  rc = mt9p111_i2c_write_table(&mt9p111_reg_reflect_mirror[0], ARRAY_SIZE(mt9p111_reg_reflect_mirror));
	  break;
  case 2:
	  rc = mt9p111_i2c_write_table(&mt9p111_reg_reflect_flip[0], ARRAY_SIZE(mt9p111_reg_reflect_flip));
	  break;
  case 3:
	  rc = mt9p111_i2c_write_table(&mt9p111_reg_reflect_mirror_flip[0], ARRAY_SIZE(mt9p111_reg_reflect_mirror_flip));
	  break;		
  }
	return rc;
}

static int mt9p111_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int err = -EINVAL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

  PRINTK(1, "%s %d\n", __func__, ctrl->id - V4L2_CID_PRIVATE_BASE);

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_BRIGHTNESS:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_BRIGHTNESS\n", __func__);
    mt9p111_set_brightness(ctrl->value);
		cur_userset->brightness = ctrl->value;
		err = 0;
		break;

	case V4L2_CID_CAMERA_WHITE_BALANCE:
		dev_dbg(&client->dev, "%s: V4L2_CID_AUTO_WHITE_BALANCE\n", __func__);
		mt9p111_set_wb(ctrl->value);
    cur_userset->manual_wb = ctrl->value;
		err = 0;
		break;

	case V4L2_CID_CAMERA_EFFECT:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_EFFECT\n", __func__);
		mt9p111_set_wb(ctrl->value);
    cur_userset->effect = ctrl->value;
		err = 0;
		break;
	case V4L2_CID_CAMERA_ISO:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_ISO\n", __func__);
		mt9p111_set_iso(ctrl->value);
    cur_userset->iso = ctrl->value;
		err = 0;
		break;

	case V4L2_CID_CAMERA_SCENE_MODE:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_ISO\n", __func__);
		mt9p111_set_scene(ctrl->value);
    cur_userset->scene = ctrl->value;
		err = 0;
		break;

	case V4L2_CID_CAMERA_ZOOM:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_ZOOM\n", __func__);
		mt9p111_set_zoom(ctrl->value);
    cur_userset->zoom = ctrl->value;
		err = 0;
		break;

	case V4L2_CID_CAMERA_FLASH_MODE:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_FLASH_MODE\n", __func__);
		mt9p111_set_flash(ctrl->value);
    cur_userset->flash = ctrl->value;
		err = 0;
		break;
 
 	case V4L2_CID_CAMERA_SET_AUTO_FOCUS:
		dev_dbg(&client->dev, "%s: V4L2_CID_CAMERA_SET_AUTO_FOCUS\n", __func__);
		cur_userset->autofocus = mt9p111_af_trigger(ctrl->value);
		err = 0;
		break;
 
	default:
		dev_err(&client->dev, "%s: no such control no process %d\n", __func__, ctrl->id - V4L2_CID_PRIVATE_BASE);
		err = 0;
		break;
	}
	return err;
}

static int mt9p111_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

  switch (ctrl->id) {
    case V4L2_CID_CAMERA_EXIF_ISO:
      ctrl->value = cur_userset->iso;
      break;
    case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT:
      if(cur_userset->autofocus == 0)
        ctrl->value = 2;
      else
        ctrl->value = 0;
      break;

  	default:
  		dev_err(&client->dev, "%s: no such ctrl process %d\n", __func__, ctrl->id - V4L2_CID_PRIVATE_BASE);
  		break;
  }
	return 0;
}

static int mt9p111_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	struct mt9p111_info *state = to_state(sd);

	fsize->discrete.width = cam_res_list[state->framesize_index].width;
	fsize->discrete.height = cam_res_list[state->framesize_index].height;
  PRINTK(1, "%s width=%d height=%d\n", __func__, fsize->discrete.width, fsize->discrete.height);
	return 0;
}

static int mt9p111_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *f)
{
	int i;
	struct mt9p111_info *info = to_state(sd);
 	PRINTK(1, "%s requested res(%d, %d)\n", __func__, f->width, f->height);

	for(i = 0; i < N_RESOLUTIONS; i++) {
		if (f->width > cam_res_list[i].width - 20 &&	f->width < cam_res_list[i].width + 20 &&
			  f->height > cam_res_list[i].height - 20 &&	f->height < cam_res_list[i].height + 20)
			break;
	}

	if(i == N_RESOLUTIONS)
		return -EINVAL;

	f->width = cam_res_list[i].width;
	f->height = cam_res_list[i].height;
	info->framesize_index = i;
	return 0;
}

static int mt9p111_s_stream(struct v4l2_subdev *sd, int enable)
{
  PRINTK(1, "%s %d\n", __func__, enable);
	return 0;
}

static int mt9p111_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *f)
{
	int ret;
	struct mt9p111_info *info = to_state(sd);
	unsigned short width, height;

  if((f->height == cam_res_list[info->framesize_index].height) && (f->width == cam_res_list[info->framesize_index].width))
  {
    PRINTK(2, "same width and height , ignore\n");
    return 0;
  }

  PRINTK(1, "%s width = %d,height = %d\n",__func__, f->width, f->height);
	ret = mt9p111_try_fmt(sd, f);
	if (ret < 0)
		return ret;

	info->width = f->width;
	info->height = f->height;

  if(f->width<=800)
    mt9p111_set_sensor_mode(SENSOR_PREVIEW_MODE);
  else
    mt9p111_set_sensor_mode(SENSOR_SNAPSHOT_MODE);
  
  width = f->width;
  height = f->height;
  mt9p111_i2c_write(mt9p111_client->addr, 0x098e, 0x48c0, WORD_LEN);
  mt9p111_i2c_write(mt9p111_client->addr, 0xC8C0, width, WORD_LEN);
  mt9p111_i2c_write(mt9p111_client->addr, 0xC8C2, height, WORD_LEN);
//  mt9p111_i2c_write(mt9p111_client->addr, 0xC8AA, width, WORD_LEN);
//  mt9p111_i2c_write(mt9p111_client->addr, 0xC8AC, height, WORD_LEN);
  mt9p111_i2c_write(mt9p111_client->addr, 0x8404, 0x06, BYTE_LEN);
  if(width == 640)
    msleep(100);
	return 0;
}

static int mt9p111_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
  PRINTK(1, "%s \n",__func__);
  PRINTK(1, "%s type=%d\n",__func__, a->type);
  return 0;
}

static int mt9p111_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
  PRINTK(1, "%s \n",__func__);
  return 0;
}

static int mt9p111_g_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
  PRINTK(1, "%s \n",__func__);
  return 0;
}

static int mt9p111_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
  PRINTK(1, "%s \n",__func__);
  return 0;
}

static int mt9p111_detect(struct mt9p111_info *info)
{
	int ret = 0;
	unsigned short ver;

  ver = 0x0000;
//  mixtilev310_cam_b_power(1);
//  msleep(50);
	mt9p111_i2c_read(mt9p111_client->addr, 0x0000, &ver, WORD_LEN);
//  mixtilev310_cam_b_power(0);

  PRINTK(1, "ver = 0x%04x\n", ver);
	if (ver != 0x2880) {
    PRINTK(0, "mt9p111_detect failed\n");
		ret = -1;
	} else {
    PRINTK(0, "mt9p111_detect success\n");
  }
	return ret;
}

static int mt9p111_init(struct v4l2_subdev *sd, u32 val)
{
	struct mt9p111_info *info = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	
  if(initted == 0)
  {
  	if (mt9p111_detect(info)) {
  		dev_info(&client->dev, "mt9p111 sensor not found\n");
  		return -1;
  	}

  	mt9p111_reg_init();
  	mt9p111_set_sensor_mode(SENSOR_PREVIEW_MODE);
  	mt9p111_set_brightness(cur_userset->brightness);
  	mt9p111_set_wb(cur_userset->manual_wb);
  	mt9p111_set_effect(cur_userset->effect);
  	mt9p111_set_iso(cur_userset->iso);
  	mt9p111_set_scene(cur_userset->scene);
  	mt9p111_set_zoom(cur_userset->zoom);
  	mt9p111_set_reflect(0);
  	
  	info->width =  800;
  	info->height = 600;
  	info->framesize_index = 1;
    initted = 1;
  }
  else
    msleep(150);
	return 0;
}

static const struct v4l2_subdev_core_ops mt9p111_core_ops = {
	.init = mt9p111_init,
	.g_ctrl = mt9p111_g_ctrl,
	.s_ctrl = mt9p111_s_ctrl,
	.g_ext_ctrls = mt9p111_g_ext_ctrls,
};

static const struct v4l2_subdev_video_ops mt9p111_video_ops = {
	.enum_framesizes = mt9p111_enum_framesizes,
	.s_mbus_fmt = mt9p111_s_fmt,
	.s_stream = mt9p111_s_stream,
	.g_parm = mt9p111_g_parm,
	.s_parm = mt9p111_s_parm,
};

static const struct v4l2_subdev_ops mt9p111_ops = {
	.core = &mt9p111_core_ops,
	.video = &mt9p111_video_ops,
};

static int mt9p111_probe(struct i2c_client *client, const struct i2c_device_id *did)
{
	struct v4l2_subdev *sd;
	struct mt9p111_info *info;

	dev_info(&client->dev, "mt9p111 sensor probe enter\n");
	

	info = kzalloc(sizeof(struct mt9p111_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;

  cur_userset = &info->userset;
	sd = &info->sd;
	strcpy(sd->name, MT9P111_DRIVER_NAME);
	v4l2_i2c_subdev_init(sd, client, &mt9p111_ops);

	mt9p111_client = client;
	cur_userset->brightness = EV_DEFAULT;
	cur_userset->manual_wb = WHITE_BALANCE_AUTO;
	cur_userset->effect = IMAGE_EFFECT_NONE;
	cur_userset->iso = IS_ISO_AUTO;
	cur_userset->scene = SCENE_MODE_NONE;
	cur_userset->zoom = ZOOM_LEVEL_0;
	dev_info(&client->dev, "mt9p111 sensor probed\n");
	initted = 0;
	return 0;
}

static int mt9p111_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	
	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id mt9p111_id[] = {
	{ MT9P111_DRIVER_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mt9p111_id);

static struct i2c_driver mt9p111_i2c_driver = {
	.driver = {
		.name	= MT9P111_DRIVER_NAME,
	},
	.probe		= mt9p111_probe,
	.remove		= mt9p111_remove,
	.id_table	= mt9p111_id,
};

static int __init mt9p111_mod_init(void)
{
	return i2c_add_driver(&mt9p111_i2c_driver);
}

static void __exit mt9p111_mod_exit(void)
{
	i2c_del_driver(&mt9p111_i2c_driver);
}
module_init(mt9p111_mod_init);
module_exit(mt9p111_mod_exit);

MODULE_DESCRIPTION("mt9p111 camera driver");
MODULE_AUTHOR("Apollo Yang <Apollo5520@gmail.como>");
MODULE_LICENSE("GPL");
