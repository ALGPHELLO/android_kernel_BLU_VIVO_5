#ifndef __HW_CTL_DRVIER__

#define __HW_CTL_DRVIER__

#define HWCTL_DBG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>

#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/delay.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/string.h>
#include <mach/irqs.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/leds.h>



#define AW2013_LED_TYPE_TOTAL 3

#define AW2013_I2C_ADDR		0x45
#define AW2013_I2C_BUS 		2

#define BLUE_LED_ID		0
#define GREEN_LED_ID	1
#define RED_LED_ID		2
struct aw2013_led_data{
	int		level;
	bool    level_change;
	int 	blink;
	unsigned long	delay_on;
	unsigned long	delay_off;
	bool	blink_change;
};

struct cust_aw2013_led {
	char                 *name;
	void		(*brightness_set)(struct led_classdev *led_cdev,
					  enum led_brightness brightness);
	int		(*blink_set)(struct led_classdev *led_cdev,
				     unsigned long *delay_on,
				     unsigned long *delay_off);
	struct aw2013_led_data data;
};

#endif
