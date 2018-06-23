/*******************************************************************************
 * Filename:
 * ---------
 *  pi5usb30216.c
 *
 * Project:
 * --------
 *
 * Description:
 * ------------
 *
 * Author:
 * -------
 *  Yanjun, yanjun@gionee.com, 2016-05-05
 * 
 *******************************************************************************/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include "pi5usb30216.h"

#define BUF_LEN 16
#define REG_COUNT 4

static int int_flag = 0;
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int suspend_flag = 0;

struct pinctrl *pinctrl_usbc;
struct pinctrl_state *typec_en_low, *typec_en_high;
extern bool mt_disable_otgcharger(void);

int pi5usb_read(struct i2c_client *client, char *val_buf, int count)
{
	int      ret=0;
	
	if(NULL == client || NULL == val_buf || count > REG_COUNT || suspend_flag)
		return -1;

	ret = i2c_master_recv(client, val_buf, count);
	if (ret < 0)
    	{
		printk("%s read failed \n", __func__);
        	return -1;
    	}
#if 0
	printk("%s: read \n", __func__);
	for(ret  = 0; ret < count; ret++)
		printk(" 0x%x ", val_buf[ret]);
	printk("\n");
#endif
    	return 0;
}
//Only for write reg 0x2
int pi5usb_write(struct i2c_client *client, int writeData)
{
	char    write_data[2] = {0};
	int     ret=0;

	if(NULL == client || suspend_flag)
		return -1;
    	write_data[0] = 0;
    	write_data[1] = (char)(writeData & 0xFF);

    	ret = i2c_master_send(client, write_data, 2);
    	if (ret < 0)
    	{
		printk("%s write failed \n", __func__);
        	return -1;
    	}

    	return 0;
}

static void  pi5usb_reset_i2c(struct i2c_client *client)
{
	if(NULL == client)
	{
		printk("%s client is null \n", __func__);
		return ;
	}
	pi5usb_write(client, 1);
	msleep(30);
	pi5usb_write(client, 4);
	return ;
}

static int def_mode = 0x4;
static int current_mode = 0x4;
//When attached to a phone/notebook/tablet, our phone as a device
#ifdef CONFIG_MTK_BQ24296_SUPPORT
extern void bq24296_set_otg_config(unsigned int val);
#endif
static void pi5usb_try_snk(struct i2c_client *client)
{
	char reg_val[REG_COUNT] = "";
	if(NULL == client)
		return ;
	{
		//TrySNK for 700ms
		current_mode = 0;
		pi5usb_write(client, current_mode + 0x1);
		//TrySNK wait for VBUS
		msleep(700);
		pi5usb_read(client, reg_val, REG_COUNT);
		//if not attached.goto unattached SRC.
		if(0x0 == reg_val[3])
		{
			current_mode = 0x2;
			pi5usb_write(client, current_mode + 0x1);
			msleep(500);
			pi5usb_read(client, reg_val, REG_COUNT);
			if(0x5 == reg_val[3] || 0x6 == reg_val[3])
			{
				printk("%s: As a host \n", __func__);
			}
			else if(0 == reg_val[3])
			{
				current_mode = 0;
				pi5usb_write(client, current_mode + 0x1);
				msleep(500);
				pi5usb_read(client, reg_val, REG_COUNT);
				//Host attached
				if(0xa9 == reg_val[3]
	                           || 0xaa == reg_val[3]
        	                   || 0xc9 == reg_val[3]
                	           || 0xca == reg_val[3]
                        	   || 0xe9 == reg_val[3]
				   || 0xea == reg_val[3]	
                         	)
				{
#ifdef CONFIG_MTK_FAN5405_SUPPORT
                mt_disable_otgcharger();
#elif defined(CONFIG_MTK_BQ24296_SUPPORT)
                bq24296_set_otg_config(0);
#endif
	                                printk("%s: As a devices \n", __func__);
				}
				else	
					current_mode = def_mode;
			}
			else
				current_mode = def_mode;
		}//end tryWait SRC
		else
		{
#ifdef CONFIG_MTK_FAN5405_SUPPORT
        mt_disable_otgcharger();
#elif defined(CONFIG_MTK_BQ24296_SUPPORT)
        bq24296_set_otg_config(0);
#endif		
			printk("Try.SNK success. Attached to a host, as a device \n");
		}
	}//end trySNK flow
	return ;
}

static void pi5usb_process(void *data)
{
	struct i2c_client *client = (struct i2c_client *)data;
	char reg_val[REG_COUNT] = "";
	static int audio_accessory_flag = 0;
	static int debug_accessory_flag = 0;
	static int repeattimes = 0;
	static int otg_inserted = 0;

	if(NULL == client)
	{
		printk("%s : client point is null \n", __func__);
		return ;
	}

	pi5usb_write(client, current_mode + 0x1);//Disable interrupt
	msleep(30);//Delay 20ms ~ 45ms
	pi5usb_read(client, reg_val, REG_COUNT);
	if(0x0 == reg_val[3] || (0x80 == reg_val[3]) || 0x1 == reg_val[3])
	{
		current_mode = def_mode;
	}
	switch (reg_val[2])
	{
		case 0x1:
		{
			switch (reg_val[3])
			{
				case 0:
					repeattimes = 0;
					audio_accessory_flag = 0;
					debug_accessory_flag = 0;
					printk("No plugin \n");
					break;
				case 0x5:
				case 0x6:
					{
						repeattimes = 0;
						otg_inserted = 1;
						pi5usb_try_snk(client);
						//Device Plug in
						//printk("OTG Device plug in \n");
					}
					break;
				case 0x13:
					{
						repeattimes = 0;
						msleep(50);
						debug_accessory_flag = 1; ////确认为Debug accessory且无外部VBUS供电，开启VBUS供电
					}
					break;
				case 0xa8:
					break;
				case 0x0f:
					//Audio accessory plug in
					audio_accessory_flag = 1;
					msleep(50);
					break;
				case 0x93:
					break;
				case 0x8f:
					break;
				case 0xa9:
				case 0xaa:
				case 0xc9:
				case 0xca:
				case 0xe9:
				case 0xea:
					printk("Host plug in \n");
					break;
				default:
					break;
			}
		}	
			break;
		case 0x0:
			switch (reg_val[3])
			{
				case 0x0:
					repeattimes = 0;
					audio_accessory_flag = 0;
					debug_accessory_flag = 0;
					break;
				case 0x97:
					if(1 == audio_accessory_flag || 1 == debug_accessory_flag)
					{
						msleep(100);
					}else if(repeattimes >= 3)
					{
						repeattimes = 0;
						printk("VBUS in, no CC \n");
					}
					else
					{
						repeattimes++;
						msleep(240);
						pi5usb_write(client, 0x1);
						msleep(100);
				//		pi5usb_write(client, 0x4);
					}
					break;
				case 0x93:
					break;
				case 0x8f:
					break;
				case 0xa9:
				case 0xaa:
				case 0xc9:
				case 0xca:
				case 0xe9:
				case 0xea:
					printk("Host plug in \n");
					break;
				case 0x4:
				case 0x5:
				case 0x6:
				default:
					printk("%s %d \n", __func__, __LINE__);
					break;

			}
			break;
		case 0x02:
			audio_accessory_flag = 0;
			debug_accessory_flag = 0;
			repeattimes = 0;
			printk("Detach \n");
			break;
		default:	
			audio_accessory_flag = 0;
			debug_accessory_flag = 0;
			repeattimes = 0;
			break;
	}
	if(0x4 == reg_val[3])
	{
		pi5usb_reset_i2c(client);
	}else
	{
		msleep(20);
		pi5usb_write(client, current_mode);
	}
	return;
}

static irqreturn_t pi5usb_eint_interrupt_handler(int irqnum, void *data)
{
	int_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}
static int pi5usb_event_handler(void *data)
{
	do
    	{
        	set_current_state(TASK_INTERRUPTIBLE);
        	wait_event_interruptible(waiter, int_flag != 0);
        	set_current_state(TASK_RUNNING);
		int_flag = 0; //Must here ,because pi5usb_process may be have interrupts
		//read registers
		pi5usb_process(data);
        }while(!kthread_should_stop());
	return 0;
}
int eint_init(void){
	int retval = 0;
	u32 ints[2] = { 0, 0 };
	struct device_node *node;
	unsigned int debounce, gpiopin;
    	int irqnum;

	node = of_find_compatible_node(NULL, NULL, "mediatek,usb_typec");
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		debounce = ints[1];
		gpiopin = ints[0];
        printk("pi5usb eint debounce = %d gpiopin=%d\n", debounce, gpiopin);
		//gpio_set_debounce(gpiopin, debounce);
	}else{
        printk("pi5usb can not find interrupt pin\n");
    }

	irqnum = irq_of_parse_and_map(node, 0);

	printk("pi5usb request_irq irqnum=0x%x\n", irqnum);

	retval = request_irq(irqnum, pi5usb_eint_interrupt_handler, IRQF_TRIGGER_FALLING, "usb_typec_eint", NULL);
	if (retval != 0) {
		printk("pi5usb request_irq fail, ret %d, irqnum %d!!!\n", retval, irqnum);
	}

	return retval;
}

static void enable_pi5usb(void){
    pinctrl_select_state(pinctrl_usbc, typec_en_low);
    msleep(50);
}
static int pi5usb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
    thread = kthread_run(pi5usb_event_handler, (void *)client, "usb_typec");
    if (IS_ERR(thread)){
	err = PTR_ERR(thread);
    	printk("pi5usb failed to create kernel thread: %d\n", err);
    }
    if (eint_init()){
        printk("pi5usb %s can not get irq\n", __func__);
    }
    enable_pi5usb();
    pi5usb_reset_i2c(client);
    printk("%s end\n", __func__);
    return err;
}

static int usbc_pinctrl_probe(struct platform_device *pdev){
    int retval = 0;
    printk("pi5usb func=%s  line=%d\n", __func__, __LINE__);
    pinctrl_usbc = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(pinctrl_usbc)) {
            printk("pi5usb Cannot find usb pinctrl!\n");
    } 
    else 
    {
	    typec_en_low = pinctrl_lookup_state(pinctrl_usbc, "typec_en_low");
	    if (IS_ERR(typec_en_low))
	            printk("pi5usb Can *NOT* find pi5usb_en_low\n");
	    else
	            printk("pi5usb Find pi5usb_en_low\n");
	    typec_en_high = pinctrl_lookup_state(pinctrl_usbc, "typec_en_high");
	    if (IS_ERR(typec_en_high))
		    printk("pi5usb Can *NOT* find pi5usb_en_high\n");
            else
		    printk("pi5usb Find pi5usb_en_high\n");
    }
    return retval;
}

static int usbc_pinctrl_remove(struct platform_device *pdev){
	        return 0;
}

static const struct of_device_id usb_pinctrl_ids[] = {
       {.compatible = "mediatek,usb_typec",},
};

static struct platform_driver usbc_pinctrl_driver = 
{
        .probe = usbc_pinctrl_probe,
        .remove = usbc_pinctrl_remove,
        .driver = 
	{
        	.name = "usb_typec",
	        .of_match_table = usb_pinctrl_ids,
        },
};

static const struct of_device_id pi5usb_of_match[] = {
    {.compatible = "mediatek,type_c"},
    {},
};

static const struct i2c_device_id usbc_i2c_id[] = {
		{PI5USB_NAME, 0},
		{}
};

#ifdef CONFIG_PM_SLEEP
static int pi5usb_suspend(struct device *dev)
{
        //struct i2c_client *client = to_i2c_client(dev);
	//Low power mode 
	if(NULL != pinctrl_usbc)
		pinctrl_select_state(pinctrl_usbc, typec_en_high);
	suspend_flag = 1;
        return 0;
}

static int pi5usb_resume(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
	if(NULL != pinctrl_usbc)
		pinctrl_select_state(pinctrl_usbc, typec_en_low);
	
	//reset 
	if(NULL != client)
		pi5usb_reset_i2c(client);
	suspend_flag = 0;
        return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pi5usb_pm_ops, pi5usb_suspend, pi5usb_resume);

static struct i2c_driver pi5usb_i2c_driver = {
    .probe = pi5usb_i2c_probe,
    .driver = {
        .name   = PI5USB_NAME,
        .owner  = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP	
	.pm     = &pi5usb_pm_ops,
#endif	
	.of_match_table = pi5usb_of_match,
    },
    .id_table = usbc_i2c_id,
};

static int __init pi5usb_driver_init(void){
	if (!platform_driver_register(&usbc_pinctrl_driver))
	{
	        printk(KERN_ERR "pi5usb register usbc pinctrl succeed!!\n");
	}else{
		 printk(KERN_ERR "pi5usb register usbc pinctrl fail!!\n");
		 return -1;
	 }
	if (i2c_add_driver(&pi5usb_i2c_driver) != 0){
   	   	printk("%s i2c add failed \n", __func__);
   	   	return -1;	
   	}
   	return 0;
}

static void __exit pi5usb_driver_exit(void){
	printk("pi5usb init i2c board exit\n");
	return ;
}

module_init(pi5usb_driver_init);
module_exit(pi5usb_driver_exit);

MODULE_AUTHOR("Jun yan");
MODULE_DESCRIPTION("Gionee TYPEC driver");
MODULE_LICENSE("GPL");
