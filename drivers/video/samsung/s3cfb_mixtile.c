#include "s3cfb.h"
#include <linux/gpio.h>
#include <linux/serial_core.h>
#include <plat/devs.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-serial.h>
#include <mach/gpio.h>
#include <mach/gpio-mixtile4x12.h>

extern struct s3cfb_lcd mixtile_lcd;
extern struct s3cfb_lcd mixtile_lcd_1080p;
extern struct s3cfb_lcd mixtile_lcd_720p;

extern int dev_ver;
/* name should be fixed as 's3cfb_set_lcd_info' */
void s3cfb_set_lcd_info(struct s3cfb_global *ctrl)
{
	mixtile_lcd.init_ldi = NULL;
if(gpio_get_value(EXYNOS4_GPX3(4))!=0)
//hdmi_dev->cur_preset = V4L2_DV_720P60;
  ctrl->lcd = &mixtile_lcd_720p;
else
  ctrl->lcd = &mixtile_lcd_1080p;
//  ctrl->lcd = &mixtile_lcd;
}

