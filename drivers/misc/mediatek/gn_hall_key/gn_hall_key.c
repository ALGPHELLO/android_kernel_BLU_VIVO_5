/*
 * Copyright (C) 2010 MediaTek, Inc.
 *
 * Author: Terry Chang <terry.chang@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


/*kpd.h file path: ALPS/mediatek/kernel/include/linux */



#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/rtpm_prio.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/switch.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>




#define OPEN_HALL_KEY	_IO('k', 32)
#define CLOSE_HALL_KEY	_IO('k', 33)

#define HALL_TAG "gn_hall_key"
#define HALL_KEY_NAME	"gn_hall_key"

#define GN_HALL_KEY_OPEN 202
#define GN_HALL_KEY_CLOSE 203

#define GPIO_MHALL_EINT_PIN 4



enum hall_state
{
    HALL_CLOSE = 0,
    HALL_OPEN = 1
};

enum hall_key
{
    key_close = 0,
    key_open = 1
};

struct input_dev *hkey_input_dev;
static struct switch_dev hall_switch_dev;

int hall_key_irq=0;
struct pinctrl *pinctrl1_hallkey;
struct pinctrl_state *hallkey_eint_as_int, *hallkey_eint_output0, *hallkey_eint_output1;
static int key_on = key_close;
static int  gn_hall_flag = 0;
struct task_struct *gn_thread_hall = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter_hall);
static int gn_hall_eint_handler(void *data);



/*********************************************************************/
static ssize_t hall_show_state(struct device *dev,	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", key_on);
}

static ssize_t hall_store_state(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	if (buf != NULL && size != 0) {
		if (buf[0] == '0') {
			key_on = key_close;
			switch_set_state((struct switch_dev *)&hall_switch_dev, HALL_OPEN);
		} else {
			key_on = key_open;
		}
	}
	return size;
}
static DEVICE_ATTR(state, 0664, hall_show_state, hall_store_state);

/*Gionee luliya 20160413 add for hall callback for tp begin */
#if 0
extern void tpd_hall_key_callback(int state);
#endif
/*Gionee luliya 20160413 add for hall callback for tp end*/

static int gn_hall_eint_handler(void *data)
{
	u8 kpd_hallkey_state;
	u8 old_state;
	unsigned long flags = 0;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);

	do{
	    set_current_state(TASK_INTERRUPTIBLE);

        wait_event_interruptible(waiter_hall,gn_hall_flag!=0);

        gn_hall_flag = 0;


        set_current_state(TASK_RUNNING);
	    kpd_hallkey_state = gpio_get_value(GPIO_MHALL_EINT_PIN);
	    old_state = !kpd_hallkey_state;

	     //printk( "%s: kpd_hallkey_state = %d\n",__func__,kpd_hallkey_state);

		if(key_on == key_open)
		{
		    if(kpd_hallkey_state == 1) //hall open
		    {
		    	input_report_key(hkey_input_dev, GN_HALL_KEY_OPEN, 1);
		    	input_report_key(hkey_input_dev, GN_HALL_KEY_OPEN, 0);
				input_sync(hkey_input_dev);
		    	switch_set_state((struct switch_dev *)&hall_switch_dev, HALL_OPEN);
		    }
		    else	//hall close
		    {
		    	input_report_key(hkey_input_dev, GN_HALL_KEY_CLOSE, 1);
	    		input_report_key(hkey_input_dev, GN_HALL_KEY_CLOSE, 0);
				input_sync(hkey_input_dev);
	    		switch_set_state((struct switch_dev *)&hall_switch_dev, HALL_CLOSE);
	    	}
/*Gionee luliya 20160413 add for hall callback for tp begin */
#if 0
		tpd_hall_key_callback(kpd_hallkey_state);
#endif
/*Gionee luliya 20160413 add for hall callback for tp end */
		}
		
		
		
	old_state=!kpd_hallkey_state;
	flags = old_state?IRQF_TRIGGER_HIGH:IRQF_TRIGGER_LOW;
	
	irq_set_irq_type(hall_key_irq,flags);
	enable_irq(hall_key_irq);
	
	}while(!kthread_should_stop());
	
	return 0;
}



static irqreturn_t kpd_hall_eint_handler(unsigned irq, struct irq_desc *desc)
{


	disable_irq_nosync(hall_key_irq);
	
	gn_hall_flag = 1;
		
	wake_up_interruptible(&waiter_hall);

	return IRQ_HANDLED;
}


static int key_get_gpio_info(struct platform_device *pdev)
{
	int ret;

	printk("%s enter\n", __func__);
	pinctrl1_hallkey = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl1_hallkey)) {
		ret = PTR_ERR(pinctrl1_hallkey);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl1!\n");
		return ret;
	}
	hallkey_eint_as_int = pinctrl_lookup_state(pinctrl1_hallkey, "state_hallkey_as_int");
	if (IS_ERR(hallkey_eint_as_int)) {
		ret = PTR_ERR(hallkey_eint_as_int);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl state_hallkey_as_int!\n");
		return ret;
	}
	hallkey_eint_output0 = pinctrl_lookup_state(pinctrl1_hallkey, "state_hallkey_output0");
	if (IS_ERR(hallkey_eint_output0)) {
		ret = PTR_ERR(hallkey_eint_output0);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl state_hallkey_output0!\n");
		return ret;
	}
	hallkey_eint_output1 = pinctrl_lookup_state(pinctrl1_hallkey, "state_hallkey_output1");
	if (IS_ERR(hallkey_eint_output1)) {
		ret = PTR_ERR(hallkey_eint_output1);
		dev_err(&pdev->dev, "fwq Cannot find touch pinctrl state_hallkey_output1!\n");
		return ret;
	}
	printk("%s end!\n", __func__);
	return 0;
}



/*****************************************************************************************/


static DEFINE_MUTEX(key_set_gpio_mutex);


struct of_device_id mhall_key_of_match[] = {
	{ .compatible = "mediatek,mhall-eint", },
	{},
};



static int mhall_key_probe(struct platform_device *pdev)
{
	int error;
	struct device_node *node = NULL;
	hkey_input_dev = input_allocate_device();
	hkey_input_dev->name = HALL_KEY_NAME;

	error = input_register_device(hkey_input_dev);
	if (error) {
		printk(HALL_TAG "register input device failed (%d)\n",error);
		input_free_device(hkey_input_dev);
		return error;
	}

	__set_bit(EV_KEY, hkey_input_dev->evbit);

	__set_bit(GN_HALL_KEY_OPEN, hkey_input_dev->keybit);
	__set_bit(GN_HALL_KEY_CLOSE, hkey_input_dev->keybit);

	hall_switch_dev.name = HALL_KEY_NAME;
	hall_switch_dev.index = 0;
	hall_switch_dev.state = HALL_OPEN;

	error = switch_dev_register(&hall_switch_dev);
	if(error)
	{
		printk(HALL_TAG "register switch failed (%d)\n", error);
		input_unregister_device(hkey_input_dev);
		input_free_device(hkey_input_dev);
		return error;
	}

	key_on = key_open;

	gn_thread_hall = kthread_run(gn_hall_eint_handler, 0, hall_switch_dev.name);


	switch_set_state((struct switch_dev *)&hall_switch_dev, HALL_OPEN);	
	key_get_gpio_info(pdev);

	mutex_lock(&key_set_gpio_mutex);	
	pinctrl_select_state(pinctrl1_hallkey, hallkey_eint_as_int);	
	mutex_unlock(&key_set_gpio_mutex);
	
	
	//node = of_find_compatible_node(NULL, NULL, "mediatek,mhall-eint");
	node = of_find_matching_node(node, mhall_key_of_match);
	if (node) {
		switch_set_state((struct switch_dev *)&hall_switch_dev, HALL_CLOSE);
		hall_key_irq = irq_of_parse_and_map(node, 0);
		printk("%s hall_key_irq=%d\n",__func__,hall_key_irq);
		error = request_irq(hall_key_irq,
				  (irq_handler_t)kpd_hall_eint_handler,
				  IRQF_TRIGGER_HIGH,
				  "mhall_eint", NULL);

		if (error > 0) {
			error = -1;
			printk("hall_key request_irq IRQ LINE NOT AVAILABLE!.");
			return error;

		}
		
		
	}
		
	printk("%s end\n",__func__);
	
	return 0;
}


static int hall_key_probe(struct platform_device *pdev)
{
	return 0;
}
static int mhall_key_remove(struct platform_device *pdev)
{
	return 0;
}

static int hall_key_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_device hall_key_pdev = {
	.name	   = HALL_KEY_NAME,
	.id	       = -1,
};


static struct platform_driver hall_key_pdrv = {
	.probe = hall_key_probe,
	.remove = hall_key_remove,
	.driver = {
		.name = HALL_KEY_NAME,
		.owner = THIS_MODULE,

	},
};


static struct platform_driver mhall_key_pdrv = {
	.probe = mhall_key_probe,
	.remove = mhall_key_remove,
	.driver = {
		.name = "mhallkey",
		.owner = THIS_MODULE,
		.of_match_table = mhall_key_of_match,

	},
};
static int __init hall_key_mod_init(void)
{
	int error;
	
	error = platform_device_register(&hall_key_pdev);
	if (error) {
		printk(HALL_TAG "register device failed (%d)\n", error);
		return error;
	}


	error = device_create_file(&hall_key_pdev.dev, &dev_attr_state);
	if (error)
	{
		platform_device_unregister(&hall_key_pdev);
		printk(HALL_TAG "gn_hall_key_on_attr failed(%d)\n",error);
		return error;
	}

	error = platform_driver_register(&hall_key_pdrv);
	if (error) {
		platform_device_unregister(&hall_key_pdev);
		printk(HALL_TAG "register hall_key_pdrv driver failed (%d)\n", error);
		return error;
	}	
	
	error = platform_driver_register(&mhall_key_pdrv);
	if (error) {
		printk(HALL_TAG "register mhall_key_pdrv driver failed (%d)\n", error);
		return error;
	}	


	return 0;
}

/* should never be called */
static void __exit hall_key_mod_exit(void)
{
}
//late_initcall(hall_key_mod_init);
module_init(hall_key_mod_init);
module_exit(hall_key_mod_exit);

MODULE_AUTHOR("yangqb@gionee.com");
MODULE_DESCRIPTION("Gionee Hall Key v1.1");
MODULE_LICENSE("GPL");
