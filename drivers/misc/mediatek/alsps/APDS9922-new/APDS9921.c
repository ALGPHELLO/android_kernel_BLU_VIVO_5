/* drivers/hwmon/mt6516/amit/apds9921.c - APDS9921 ALS/PS driver
 * 
 * Author: Lee Kai Koon <kai-koon.lee@avagotech.com>
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
//#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>


#include <alsps.h>

//#define CONFIG_HAS_EARLYSUSPEND
#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
#include <asm/io.h>
#include <cust_alsps.h>
#include "APDS9921.h"
#include <linux/sched.h>

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define APDS9921_DEV_NAME     "APDS9921"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args) 

#define I2C_FLAG_WRITE 0
#define I2C_FLAG_READ	1

/*----------------------------------------------------------------------------*/
static struct i2c_client *apds9921_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id apds9921_i2c_id[] = {{APDS9921_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_apds9921={ I2C_BOARD_INFO("APDS9921", 0x53)};
/*----------------------------------------------------------------------------*/
static int apds9921_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int apds9921_i2c_remove(struct i2c_client *client);
static int apds9921_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int apds9921_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int apds9921_i2c_resume(struct i2c_client *client);
static int apds9921_remove(void);
static int apds9921_local_init(void);

static int set_psensor_threshold(int high,int low);
long apds9921_read_ps(struct i2c_client *client, u16 *data);
static int apds9921_init_flag =-1; // 0<==>OK -1 <==> fail
static struct alsps_init_info apds9921_init_info = {
	.name   = "APDS9921",
	.init   = apds9921_local_init,
	.uninit = apds9921_remove,
};

static DEFINE_MUTEX(apds9921_mutex);
struct PS_CALI_DATA_STRUCT
{
	int close;
	int far_away;
	int valid;
};

static struct PS_CALI_DATA_STRUCT ps_cali={100,80,1};

/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS = 1,
	CMC_BIT_PS  = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct apds9921_i2c_addr { /*define a series of i2c slave address*/
	u8 write_addr;  
	u8 ps_thd; /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/
struct apds9921_priv {
	struct alsps_hw *hw;
	struct i2c_client *client;
	struct work_struct eint_work;	

	/*i2c address group*/
	struct apds9921_i2c_addr addr;

	/*misc*/
	atomic_t    trace;
	u16 als_modulus;
	atomic_t i2c_retry;
	atomic_t als_suspend;
	atomic_t als_debounce; /*debounce time after enabling als*/
	atomic_t als_deb_on;   /*indicates if the debounce is on*/
	atomic_t als_deb_end;  /*the jiffies representing the end of debounce*/
	atomic_t ps_mask;      /*mask ps: always return far away*/
	atomic_t ps_debounce;  /*debounce time after enabling ps*/
	atomic_t ps_deb_on;    /*indicates if the debounce is on*/
	atomic_t ps_deb_end;   /*the jiffies representing the end of debounce*/
	atomic_t ps_suspend;
    
    struct device_node *irq_node;
	int irq;


	/*data*/
	u16 als;
	u16 ps;
	u8 _align;
	u16 als_level_num;
	u16 als_value_num;
	u32 als_level[C_CUST_ALS_LEVEL-1];
	u32 als_value[C_CUST_ALS_LEVEL];
	int ps_cali;

	atomic_t als_cmd_val;    /*the cmd value can't be read, stored in ram*/
	atomic_t ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
	atomic_t ps_thd_val_high;/*the cmd value can't be read, stored in ram*/
	atomic_t ps_thd_val_low; /*the cmd value can't be read, stored in ram*/
	ulong enable;            /*enable mask*/
	ulong pending_intr;      /*pending interrupt*/

	int first_int_enable;
	unsigned int    cali_noise;
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static int apds9921_get_ps_value(struct apds9921_priv *obj, u16 ps);
#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
        {.compatible = "mediatek,alsps"},
        {},

};
#endif

static struct i2c_driver apds9921_i2c_driver = {	
	.probe      = apds9921_i2c_probe,
	.remove     = apds9921_i2c_remove,
	.detect     = apds9921_i2c_detect,
	.suspend    = apds9921_i2c_suspend,
	.resume     = apds9921_i2c_resume,
	.id_table   = apds9921_i2c_id,
	.driver = {
		.name     = APDS9921_DEV_NAME,
	#ifdef CONFIG_OF
        .of_match_table = alsps_of_match,
    #endif 
    },
};

static struct apds9921_priv *apds9921_obj = NULL;
//unsigned int alsps_int_gpio_number = 0;

static unsigned int alsps_irq;
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
struct platform_device *alspsPltFmDev;
/* For alsp driver get cust info */
struct alsps_hw *get_cust_alsps(void)
{
        return &alsps_cust;

}

/*------------------------i2c function for 89-------------------------------------*/
int apds9921_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag)
{
	int res = 0;
	
	mutex_lock(&apds9921_mutex);

	switch (i2c_flag) {
		case I2C_FLAG_WRITE:
			client->addr &=I2C_MASK_FLAG;
			res = i2c_master_send(client, buf, count);
			client->addr &=I2C_MASK_FLAG;
			break;

		case I2C_FLAG_READ:
			client->addr &=I2C_MASK_FLAG;
			client->addr |=I2C_WR_FLAG;
			client->addr |=I2C_RS_FLAG;
			res = i2c_master_send(client, buf, count);
			client->addr &=I2C_MASK_FLAG;
			break;

		default:
			APS_LOG("apds9921_i2c_master_operate i2c_flag command not support!\n");
			break;
	}

	if (res <= 0) {
		goto EXIT_ERR;
	}

	mutex_unlock(&apds9921_mutex);
	return res;

EXIT_ERR:
	mutex_unlock(&apds9921_mutex);
	APS_ERR("apds9921_i2c_transfer fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
int apds9921_get_addr(struct alsps_hw *hw, struct apds9921_i2c_addr *addr)
{
	if(!hw || !addr) {
		return -EFAULT;
	}

	addr->write_addr= hw->i2c_addr[0];
	return 0;
}

/*----------------------------------------------------------------------------*/
static void apds9921_power(struct alsps_hw *hw, unsigned int on) 
{

}
/*----------------------------------------------------------------------------*/
static long apds9921_enable_als(struct i2c_client *client, int enable)
{
	struct apds9921_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];	  
	long res = 0;

	databuf[0]= APDS9921_DD_MAIN_CTRL_ADDR;
	res = apds9921_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	//APS_LOG("APDS9921_CMM_ENABLE als value = %x\n", databuf[0]);
	
	if (enable) {
		databuf[1] = databuf[0]|APDS9921_DD_ALS_EN;
		databuf[0] = APDS9921_DD_MAIN_CTRL_ADDR;
		//APS_LOG("APDS9921_DD_MAIN_CTRL_ADDR enable als value = %x\n", databuf[1]);
		res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			goto EXIT_ERR;
		}

		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
	} else {
		if (test_bit(CMC_BIT_PS, &obj->enable))
			databuf[1] = databuf[0]&~APDS9921_DD_ALS_EN;
		else
			databuf[1] = databuf[0]&~(APDS9921_DD_PRX_EN|APDS9921_DD_ALS_EN);

		databuf[0] = APDS9921_DD_MAIN_CTRL_ADDR;
		//APS_LOG("APDS9921_DD_MAIN_CTRL_ADDR disable als value = %x\n", databuf[1]);
		res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			goto EXIT_ERR;
		}
	}

	return 0;

EXIT_ERR:
	APS_ERR("apds9921_enable_als fail\n");
	return res;
}
static int apds9921_dynamic_calibrate(void)
{
    int ret = 0;
    int i = 0;
    u16 data = 0;
    int noise = 0;
    int max = 0;
    u16 data_total = 0;
    struct apds9921_priv *obj = apds9921_obj;

    APS_FUN(f);
    if (!obj) goto err;

    for (i = 0; i < 6; i++) {
        if (max++ > 10) {
            atomic_set(&obj->ps_thd_val_high,NOISE_MAX);
            atomic_set(&obj->ps_thd_val_low, NOISE_MAX);
            goto err;
        }
        msleep(10);
        ret = apds9921_read_ps(obj->client,&data);
        if (ret != 0) {
            if (i>0) i--;
            continue;
        }
        data_total += data;
//        APS_ERR("noise data = %d\n",data);
    }
    noise = data_total/6;
/*
    for (idx_table = 0; idx_table < obj->ps_cali_noise_num; idx_table++) {
        if (noise <= obj->ps_cali_noise[idx_table])
            break;
    }
    if (idx_table >= obj->ps_cali_noise_num) goto err;*/
    atomic_set(&obj->ps_thd_val_high,120 + noise);
    if (atomic_read(&obj->ps_thd_val_high) > NOISE_MAX) atomic_set(&obj->ps_thd_val_high,NOISE_MAX);

    atomic_set(&obj->ps_thd_val_low,100 + noise);
    if (atomic_read(&obj->ps_thd_val_low) > NOISE_MAX) atomic_set(&obj->ps_thd_val_low,NOISE_MAX);
	
	obj->cali_noise = noise;
	set_psensor_threshold(atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));
    APS_ERR("noise=%d, obj->ps_thd_val_low= %d , obj->ps_thd_val_high = %d\n", noise, atomic_read(&apds9921_obj->ps_thd_val_low), atomic_read(&apds9921_obj->ps_thd_val_high));
    return 0;
err:
    APS_ERR("apds9921_dynamic_calibrate fail!!!\n");
    return -1;
}
/*----------------------------------------------------------------------------*/
static long apds9921_enable_ps(struct i2c_client *client, int enable)
{
	struct apds9921_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	long res = 0;
	u8 main_ctrl=0;
	struct hwm_sensor_data sensor_data;

	databuf[0] = APDS9921_DD_MAIN_CTRL_ADDR;
	res = apds9921_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	APS_LOG("APDS9921_DD_MAIN_CTRL_ADDR ps value = %x\n", databuf[0]);

	if (enable) {
		main_ctrl = databuf[0]|APDS9921_DD_PRX_EN;
		databuf[1] = main_ctrl;
		databuf[0] = APDS9921_DD_MAIN_CTRL_ADDR;
//		APS_LOG("APDS9921_DD_MAIN_CTRL_ADDR enable ps value = %x\n", databuf[1]);	
		res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		if(apds9921_dynamic_calibrate() < 0)
			goto EXIT_ERR;
		set_bit(CMC_BIT_PS, &obj->enable);
    	enable_irq(obj->irq);
	} else {
		if(test_bit(CMC_BIT_ALS, &obj->enable))
			databuf[1] = databuf[0]&~APDS9921_DD_PRX_EN;
		else
			databuf[1] = databuf[0]&~(APDS9921_DD_PRX_EN|APDS9921_DD_ALS_EN);
		
		databuf[0] = APDS9921_DD_MAIN_CTRL_ADDR;
//		APS_LOG("APDS9921_DD_MAIN_CTRL_ADDR disable ps value = %x\n", databuf[1]);	
		res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		clear_bit(CMC_BIT_PS, &obj->enable);
    //	disable_irq(obj->irq);
	}
#ifdef CONFIG_GN_BSP_ALSPS_INTERRUPT_MODE                                                                                                                           
    apds9921_read_ps(obj->client, &obj->ps);
    sensor_data.values[0] = apds9921_get_ps_value(obj, obj->ps);  
    sensor_data.value_divide = 1;
    sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
    res = ps_report_interrupt_data(sensor_data.values[0]);                                                                                                          
    if(res)                                                                                                                                                         
    {   
		APS_ERR("call ps_report_interrupt_data fail = %ld\n", res);
        goto EXIT_ERR;
    }
#endif	
	return 0;
	
EXIT_ERR:
	APS_ERR("apds9921_enable_ps fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static int apds9921_check_intr(struct i2c_client *client) 
{
	int res;
	u8 buffer[2];
    APS_FUN();
    //if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
    //	return 0;
    //gpio_direction_input(alsps_int_gpio_number);
    //if (gpio_get_value(alsps_int_gpio_number) == 1)/*skip if no interrupt */
    //    return 0;

    buffer[0] = APDS9921_DD_MAIN_STATUS_ADDR;
	res = apds9921_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	return buffer[0] & APDS9921_DD_PRX_LOGICAL_STATUS?0:1;

EXIT_ERR:
	APS_ERR("apds9921_check_intr fail\n");
	return 1;
}

/*-----------------------------------------------------------------------------*/
static irqreturn_t apds9921_eint_func(int irq, void *dev_id)
{
	if (!apds9921_obj) {
		return IRQ_HANDLED;
	}
	schedule_work(&apds9921_obj->eint_work);
	disable_irq_nosync(apds9921_obj->irq);
    return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
int apds9921_setup_eint(struct i2c_client *client)
{
	struct apds9921_priv *obj = i2c_get_clientdata(client);        
    int ret;
    struct pinctrl *pinctrl;
  //  struct pinctrl_state *pins_default;
    struct pinctrl_state *pins_cfg;
    u32 ints[2] = {0, 0};

    alspsPltFmDev = get_alsps_platformdev();
    /* gpio setting */
    pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
    if (IS_ERR(pinctrl)) {
        ret = PTR_ERR(pinctrl);                                                                                                                                                                    
        APS_ERR("Cannot find alsps pinctrl!\n");
    }
    pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
    if (IS_ERR(pins_cfg)) {
        ret = PTR_ERR(pins_cfg);
        APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");
    }

    if(obj->irq_node) {
        of_property_read_u32_array(obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
        gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
        APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

        obj->irq = irq_of_parse_and_map(obj->irq_node, 0);
        alsps_irq = obj->irq;
        APS_LOG("obj->irq = %d\n", obj->irq);
        if (!obj->irq) {
            APS_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        if (request_irq(obj->irq, apds9921_eint_func, IRQF_TRIGGER_FALLING, "ALS-eint", NULL)) {
            APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;
        }
        APS_LOG("apds9921_setup_eint request_irq ok  !!!!!\n");
    } else {
        APS_ERR("null irq node!!\n");
        return -EINVAL;
    }
    return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9921_init_client(struct i2c_client *client)
{

	struct apds9921_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
    APS_FUN();
	databuf[0] = APDS9921_DD_MAIN_CTRL_ADDR;
	databuf[1] = 0;
	res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9921_DD_PRX_LED_ADDR;
	databuf[1] = APDS9921_DD_PRX_DEFAULT_LED_FREQ|APDS9921_DD_PRX_DEFAULT_LED_CURRENT;
	res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9921_DD_PRX_PULSES_ADDR;
	databuf[1] = APDS9921_DD_PRX_DEFAULT_PULSE;
	res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9921_DD_PRX_MEAS_RATE_ADDR;
	databuf[1] = 0x40|APDS9921_DD_PRX_DEFAULT_RES|APDS9921_DD_PRX_DEFAULT_MEAS_RATE;
	res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9921_DD_ALS_MEAS_RATE_ADDR;
	databuf[1] = APDS9921_DD_ALS_DEFAULT_RES|APDS9921_DD_ALS_DEFAULT_MEAS_RATE;
	res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9921_DD_ALS_GAIN_ADDR;
	databuf[1] = APDS9921_DD_ALS_DEFAULT_GAIN;
	res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9921_DD_INT_PERSISTENCE_ADDR;
	databuf[1] = APDS9921_DD_PRX_PERS_1;
	res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	databuf[0] = APDS9921_DD_INT_CFG_ADDR;
	if (obj->hw->polling_mode_ps == 1)
		databuf[1] = 0x10;
	
	if (obj->hw->polling_mode_ps == 0)
		databuf[1] = 0x11;
	
	res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	if (0 == obj->hw->polling_mode_ps) {
		if (0){//1 == ps_cali.valid) {
			databuf[0] = APDS9921_DD_PRX_THRES_LOW_0_ADDR;	
			databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
			res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if (res <= 0) {
				goto EXIT_ERR;
			}

			databuf[0] = APDS9921_DD_PRX_THRES_LOW_1_ADDR;	
			databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
			res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if (res <= 0) {
				goto EXIT_ERR;
			}
			
			databuf[0] = APDS9921_DD_PRX_THRES_UP_0_ADDR;	
			databuf[1] = (u8)(ps_cali.close & 0x00FF);
			res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if (res <= 0) {
				goto EXIT_ERR;
			}

			databuf[0] = APDS9921_DD_PRX_THRES_UP_1_ADDR;	
			databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
			res = apds9921_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0) {
				goto EXIT_ERR;
			}
		} else {
			 set_psensor_threshold(atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));
			//set_psensor_threshold(client);
		}
	}
    APS_LOG("apds9921_setup_eint start !!!!!\n");
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if ((res = apds9921_setup_eint(client))!=0) {
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
    APS_LOG("apds9921_setup_eint end  !!!!!\n");
	return APDS9921_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int apds9921_read_als(struct i2c_client *client, u16 *data)
{	 
	//struct apds9921_priv *obj = i2c_get_clientdata(client);
	u32 clr_value, als_value;	 
	u8 buffer[3];
	u16 als_time;
	u16 als_gain;
	int res = 0;

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	
	buffer[0]=APDS9921_DD_CLEAR_DATA_ADDR;
	res = apds9921_i2c_master_operate(client, buffer, 0x301, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	
	clr_value = buffer[0] | (buffer[1]<<8) | (buffer[2]<<16);
	//APS_LOG("clr_value=%d\n", clr_value);

	buffer[0]=APDS9921_DD_ALS_DATA_ADDR;
	res = apds9921_i2c_master_operate(client, buffer, 0x301, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	
	als_value = buffer[0] | (buffer[1]<<8) | (buffer[2]<<16);
	//APS_LOG("als_value=%d\n", als_value);

	buffer[0]=APDS9921_DD_ALS_MEAS_RATE_ADDR;
	res = apds9921_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	als_time = (buffer[0]>>4)&0x07;
#if 0
	if (als_time == 0)
		als_time = 400;
	else if (als_time == 1)
		als_time = 200;
	else if (als_time == 2)
		als_time = 100;
	else if (als_time == 3)
		als_time = 50;
	else
		als_time = 25;
#endif
	buffer[0]=APDS9921_DD_ALS_GAIN_ADDR;
	res = apds9921_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	als_gain = buffer[0]&0x07;
#if 0
	if (als_gain == 0)
		als_gain = 1;
	else if (als_gain == 1)
		als_gain = 3;
	else if (als_gain == 2)
		als_gain = 6;
	else if (als_gain == 3)
		als_gain = 9;
	else
		als_gain = 18;
#endif
	*data = (als_value*APDS9921_DD_LUX_FACTOR)/(als_time*als_gain);
	
	//APS_LOG("apds9921_read_als als_value_lux = %d\n", *data);
	return 0;	 

EXIT_ERR:
	APS_ERR("apds9921_read_als fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static int apds9921_get_als_value(struct apds9921_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	
	for (idx = 0; idx < obj->als_level_num; idx++) {
		if (als < obj->hw->als_level[idx]) {
			break;
		}
	}

	if (idx >= obj->als_value_num) {
		APS_ERR("apds9921_get_als_value exceed range\n"); 
		idx = obj->als_value_num - 1;
	}

	if (1 == atomic_read(&obj->als_deb_on)) {
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if (time_after(jiffies, endt)) {
			atomic_set(&obj->als_deb_on, 0);
		}

		if (1 == atomic_read(&obj->als_deb_on)) {
			invalid = 1;
		}
	}

	if (!invalid) {
#if defined(CONFIG_MTK_AAL_SUPPORT)
		int level_high = obj->hw->als_level[idx];
		int level_low = (idx > 0) ? obj->hw->als_level[idx-1] : 0;
		int level_diff = level_high - level_low;
		int value_high = obj->hw->als_value[idx];
		int value_low = (idx > 0) ? obj->hw->als_value[idx-1] : 0;
    int value_diff = value_high - value_low;
    int value = 0;

		if ((level_low >= level_high) || (value_low >= value_high))
			value = value_low;
		else
			value = (level_diff * value_low + (als - level_low) * value_diff + ((level_diff + 1) >> 1)) / level_diff;

		//APS_DBG("ALS: %d [%d, %d] => %d [%d, %d] \n", als, level_low, level_high, value, value_low, value_high);
		
		return value;
#endif			        

		//APS_ERR("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	} else {
		//APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
long apds9921_read_ps(struct i2c_client *client, u16 *data)
{
	u8 buffer[2];
	u16 ps_data;
	long res = 0;

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=APDS9921_DD_PRX_DATA_ADDR;
	res = apds9921_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	
	ps_data = ( buffer[0] | ( (buffer[1]<<8)&0x7FF ) );
	*data = ps_data; 
	return 0;    

EXIT_ERR:
	APS_ERR("apds9921_read_ps fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static int apds9921_get_ps_value(struct apds9921_priv *obj, u16 ps)
{
	int val;// mask = atomic_read(&obj->ps_mask);
	static int val_temp=1;
	if (ps  >= atomic_read(&obj->ps_thd_val_high)) {
		val = 0;  /*close*/
		val_temp = 0;
	} else if (ps  <= atomic_read(&obj->ps_thd_val_low)) {
		val = 1;  /*far away*/
		val_temp = 1;
	} else {
		val = val_temp;
	}	
	APS_LOG("apds9921_get_ps_value val  = %d\n",val);
	//APS_DBG("PS:  %05d => %05d\n", ps, val);
	return val;
}

/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t apds9921_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return 0;
	}
#if defined(CONFIG_GN_BSP_PS_CALIBRATE_INCALL)	
	res = scnprintf(buf, PAGE_SIZE, "(%d %d %d %d %d %d %d)\n", 
		atomic_read(&apds9921_obj->i2c_retry), atomic_read(&apds9921_obj->als_debounce), 
		atomic_read(&apds9921_obj->ps_mask), atomic_read(&apds9921_obj->ps_thd_val_high),atomic_read(&apds9921_obj->ps_thd_val_low), atomic_read(&apds9921_obj->ps_debounce), apds9921_obj->ps_thd_val_max);      
#else	
	res = scnprintf(buf, PAGE_SIZE, "(%d %d %d %d %d %d)\n", 
		atomic_read(&apds9921_obj->i2c_retry), atomic_read(&apds9921_obj->als_debounce), 
		atomic_read(&apds9921_obj->ps_mask), atomic_read(&apds9921_obj->ps_thd_val_high),atomic_read(&apds9921_obj->ps_thd_val_low), atomic_read(&apds9921_obj->ps_debounce));      
#endif
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t apds9921_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;
	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return 0;
	}
	
	if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	{ 
		atomic_set(&apds9921_obj->i2c_retry, retry);
		atomic_set(&apds9921_obj->als_debounce, als_deb);
		atomic_set(&apds9921_obj->ps_mask, mask);
		//atomic_set(&apds9921_obj->ps_thd_val, thres);        
		atomic_set(&apds9921_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t apds9921_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&apds9921_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t apds9921_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&apds9921_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t apds9921_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	
	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return 0;
	}
	if((res = apds9921_read_als(apds9921_obj->client, &apds9921_obj->als)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", apds9921_obj->als);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t apds9921_show_ps(struct device_driver *ddri, char *buf)
{
	int res;
	if(!apds9921_obj)
	{
		APS_ERR("cm3623_obj is null!!\n");
		return 0;
	}
	
	if((res = apds9921_read_ps(apds9921_obj->client, &apds9921_obj->ps)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "%d\n", apds9921_obj->ps);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t apds9921_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return 0;
	}
	
	if(apds9921_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			apds9921_obj->hw->i2c_num, apds9921_obj->hw->power_id, apds9921_obj->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	
	len += snprintf(buf+len, PAGE_SIZE-len, "REGS: %02X %02X %02lX %02lX\n", 
				atomic_read(&apds9921_obj->als_cmd_val), atomic_read(&apds9921_obj->ps_cmd_val)
				,apds9921_obj->enable, apds9921_obj->pending_intr);
	
	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&apds9921_obj->als_suspend), atomic_read(&apds9921_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct apds9921_priv *obj, const char* buf, size_t count, u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;        
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++; 
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t apds9921_show_reg(struct device_driver *ddri, char *buf)
{
	unsigned char i = 0;
    unsigned char databuf[2];
    int count  = 0;
	int res;

	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return 0;
	}
    for(i = 0;i < 40 ;i++) {
		    databuf[0]= i;
            res = apds9921_i2c_master_operate(apds9921_obj->client, databuf, 0x101, I2C_FLAG_READ); 
            if (res <= 0) {
                return 0;
            }
	count+= sprintf(buf+count,"[%x] = (%x)\n",i,databuf[0]);
    }
    return count;	
}
static ssize_t apds9921_store_reg(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned char databuf[2];
	u32 data[2];
	int res;
	if(!apds9921_obj)
    {
        APS_ERR("apds9921_obj is null!!\n");
        return 0;
    }else if(2 != read_int_from_buf(apds9921_obj,buf,count,data,2))
	{
        APS_ERR("invalid format:%s\n",buf);
        return 0;
	}
	databuf[0] = data[0];
	databuf[1] = data[1];
	res = apds9921_i2c_master_operate(apds9921_obj->client, databuf, 0x2, I2C_FLAG_WRITE); 
    if (res <= 0) {
    	return 0;
    }
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t apds9921_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < apds9921_obj->als_level_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", apds9921_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t apds9921_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(apds9921_obj->als_level, apds9921_obj->hw->als_level, sizeof(apds9921_obj->als_level));
	}
	else if(apds9921_obj->als_level_num != read_int_from_buf(apds9921_obj, buf, count, 
			apds9921_obj->hw->als_level, apds9921_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t apds9921_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < apds9921_obj->als_value_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", apds9921_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t apds9921_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(apds9921_obj->als_value, apds9921_obj->hw->als_value, sizeof(apds9921_obj->als_value));
	}
	else if(apds9921_obj->als_value_num != read_int_from_buf(apds9921_obj, buf, count, 
			apds9921_obj->hw->als_value, apds9921_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
static ssize_t apds9921_show_proximity_low(struct device_driver *ddri, char *buf)
{
	if (!apds9921_obj) {
        APS_ERR("apds9921_obj is null!!\n");
        return 0;
    }
    return snprintf(buf, PAGE_SIZE, "%d", atomic_read(&apds9921_obj->ps_thd_val_low));
}
static ssize_t apds9921_show_proximity_high(struct device_driver *ddri, char *buf)
{
	if (!apds9921_obj) {
        APS_ERR("apds9921_obj is null!!\n");
        return 0;
    }
    return snprintf(buf, PAGE_SIZE, "%d", atomic_read(&apds9921_obj->ps_thd_val_high));
}
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, apds9921_show_als, NULL);
static DRIVER_ATTR(pdata,   S_IWUSR | S_IRUGO, apds9921_show_ps, NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, apds9921_show_config,	apds9921_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, apds9921_show_alslv, apds9921_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, apds9921_show_alsval, apds9921_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, apds9921_show_trace,		apds9921_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, apds9921_show_status, NULL);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, apds9921_show_reg, apds9921_store_reg);
static DRIVER_ATTR(low_threshold,   S_IRUGO,    apds9921_show_proximity_low,  NULL);
static DRIVER_ATTR(high_threshold,  S_IRUGO,    apds9921_show_proximity_high,     NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *apds9921_attr_list[] = {
    &driver_attr_als,
    &driver_attr_pdata,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_reg,
    &driver_attr_low_threshold,
    &driver_attr_high_threshold,
};
/*----------------------------------------------------------------------------*/
static int apds9921_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(apds9921_attr_list)/sizeof(apds9921_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, apds9921_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", apds9921_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int apds9921_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(apds9921_attr_list)/sizeof(apds9921_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, apds9921_attr_list[idx]);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
//#define DEBUG_APDS9921
static void apds9921_eint_work(struct work_struct *work)
{
	struct apds9921_priv *obj = (struct apds9921_priv *)container_of(work, struct apds9921_priv, eint_work);
	int err;
	struct hwm_sensor_data sensor_data;
//	unsigned char databuf[2];
//	int res = 0;
    APS_FUN();

	err = apds9921_check_intr(obj->client);
	APS_ERR("apds9921_eint_work check intrs: %d\n", err);
		
		//get raw data
		apds9921_read_ps(obj->client, &obj->ps);
		APS_LOG("apds9921_eint_work rawdata ps=%d high=%d low=%d\n", obj->ps, atomic_read(&obj->ps_thd_val_high), atomic_read(&obj->ps_thd_val_low));
			
		sensor_data.values[0] = apds9921_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;	
			
		if(ps_report_interrupt_data(sensor_data.values[0]))
		{
			APS_ERR("call ps_report_interrupt_data fail \n");
		}	 
		if(sensor_data.values[0] == 1){
			if (obj->cali_noise > 20 && obj->ps < (obj->cali_noise - 20)) 
			{
				msleep(600);
				apds9921_dynamic_calibrate();
			}
		}	
   enable_irq(alsps_irq);
    return;

//EXIT_ERR:
//	enable_irq(alsps_irq);
//    APS_ERR("i2c_transfer error = %d\n", res);
//	return;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
/*
static void apds9921_check_ps_work(struct work_struct *work)
{
	//struct apds9921_priv *obj = (struct apds9921_priv *)container_of(work, struct apds9921_priv, check_ps_work);
	struct apds9921_priv *obj = apds9921_obj;
	struct hwm_sensor_data sensor_data;
	APS_FUN();

	if (test_bit(CMC_BIT_PS, &obj->enable)) {
		apds9921_read_ps(obj->client, &obj->ps);
		APS_LOG("apds9921_eint_work rawdata ps=%d high=%d low=%d\n", obj->ps, atomic_read(&obj->ps_thd_val_high), atomic_read(&obj->ps_thd_val_low));

		sensor_data.values[0] = apds9921_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM; 

		if(ps_report_interrupt_data(sensor_data.values[0]))
		{
			APS_ERR("call ps_report_interrupt_data fail \n");
		}	
	}

	return;
}*/
static int apds9921_open(struct inode *inode, struct file *file)
{
	file->private_data = apds9921_i2c_client;

	if (!file->private_data) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int apds9921_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static int set_psensor_threshold(int high,int low)
{
	struct apds9921_priv *obj = apds9921_obj;
	u8 databuf[2];    
	int res = 0;

	APS_ERR("set_psensor_threshold function high: %d, low:%d\n",high,low);

	databuf[0] = APDS9921_DD_PRX_THRES_LOW_0_ADDR;	
	databuf[1] = (u8)(low & 0x00FF);
	res = apds9921_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		return -1;
	}

	databuf[0] = APDS9921_DD_PRX_THRES_LOW_1_ADDR; 
	databuf[1] = (u8)((low & 0xFF00) >> 8);
	res = apds9921_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		return -1;
	}

	databuf[0] = APDS9921_DD_PRX_THRES_UP_0_ADDR; 
	databuf[1] = (u8)(high & 0x00FF);
	res = apds9921_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		return -1;
	}

	databuf[0] = APDS9921_DD_PRX_THRES_UP_1_ADDR;	
	databuf[1] = (u8)((high & 0xFF00) >> 8);;
	res = apds9921_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		return -1;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static long apds9921_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct apds9921_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	int ps_result;
	int ps_cali;
	int threshold[2];
	int ps_noise[8], i;
	struct PS_CALI_DATA_STRUCT ps_cali_temp={150,130,1};
	u16 data;

	switch (cmd) {
		case ALSPS_SET_PS_MODE:
			if (copy_from_user(&enable, ptr, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}

			if (enable) {
				if ((err = apds9921_enable_ps(obj->client, 1))) {
					APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			} else {
				if ((err = apds9921_enable_ps(obj->client, 0))) {
					APS_ERR("disable ps fail: %ld\n", err); 
					goto err_out;
				}

				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if (copy_to_user(ptr, &enable, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if ((err = apds9921_read_ps(obj->client, &obj->ps))) {
				goto err_out;
			}
			
			dat = apds9921_get_ps_value(obj, obj->ps);
			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if ((err = apds9921_read_ps(obj->client, &obj->ps))) {
				goto err_out;
			}
			
			dat = obj->ps;
			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
			break;              

		case ALSPS_SET_ALS_MODE:
			if (copy_from_user(&enable, ptr, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}

			if (enable) {
				if ((err = apds9921_enable_als(obj->client, 1))) {
					APS_ERR("enable als fail: %ld\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			} else {
				if ((err = apds9921_enable_als(obj->client, 0))) {
					APS_ERR("disable als fail: %ld\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if (copy_to_user(ptr, &enable, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if ((err = apds9921_read_als(obj->client, &obj->als))) {
				goto err_out;
			}

			dat = apds9921_get_als_value(obj, obj->als);
			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if ((err = apds9921_read_als(obj->client, &obj->als))) {
				goto err_out;
			}

			dat = obj->als;
			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}              
			break;

		/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
			if ((err = apds9921_read_ps(obj->client, &obj->ps))) {
				goto err_out;
			}

			if (obj->ps > atomic_read(&obj->ps_thd_val_high)) {
				ps_result = 0;
			} else ps_result = 1;
				
			if (copy_to_user(ptr, &ps_result, sizeof(ps_result))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_IOCTL_CLR_CALI:
			if (copy_from_user(&dat, ptr, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}

			if(dat == 0)
				obj->ps_cali = 0;
			break;

		case ALSPS_IOCTL_GET_CALI:
			ps_cali = obj->ps_cali ;
			if (copy_to_user(ptr, &ps_cali, sizeof(ps_cali))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_IOCTL_SET_CALI:
			if (copy_from_user(&ps_cali, ptr, sizeof(ps_cali))) {
				err = -EFAULT;
				goto err_out;
			}

			obj->ps_cali = ps_cali;
			break;

		case ALSPS_SET_PS_THRESHOLD:
			if (copy_from_user(threshold, ptr, sizeof(threshold))) {
				err = -EFAULT;
				goto err_out;
			}

			APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]); 
			atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
			atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm

			//set_psensor_threshold(obj->client);
				
			break;
				
		case ALSPS_GET_PS_THRESHOLD_HIGH:
			threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
			//APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]); 
			if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
				err = -EFAULT;
				goto err_out;
			}
			break;
				
		case ALSPS_GET_PS_THRESHOLD_LOW:
			threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
			//APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]); 
			if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

			/*------------------------------------------------------------------------------------------*/
		case ALSPS_SET_PS_CALI:
            break;
		case ALSPS_GET_PS_CALI:
            if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp))) {
                err = -EFAULT;
                goto err_out;
            }              
            break;

		case ALSPS_GET_PS_NOISE:
            //APS_LOG("%s command ALSPS_GET_PS_NOISE = 0x%0x4\n", __FUNCTION__, ALSPS_GET_PS_NOISE);
            if ((err = apds9921_enable_ps(obj->client, 1)))
            {
                APS_ERR("apds9921 ioctl enable ps fail: %ld\n", err);
            }
            for (i=0; i<8; i++)
            {
                apds9921_read_ps(obj->client, &data);
				ps_noise[i] = (int)data;
                msleep(50);
                APS_LOG("apds9960_read_ps noise[i]= %d\n", ps_noise[i]);
            }

            if ((err = apds9921_enable_ps(obj->client, 0)))
            {
                APS_ERR("apds9921 ioctl enable ps fail: %ld\n", err);
            }

            if (copy_to_user(ptr, ps_noise, sizeof(ps_noise)))
            {	
                err = -EFAULT;
                goto err_out;
            }	 
            break;
			
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}

/*----------------------------------------------------------------------------*/

#ifdef CONFIG_COMPAT
static long apds9921_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;
	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_ALSPS_SET_PS_MODE:
		err = file->f_op->unlocked_ioctl(file, ALSPS_SET_PS_MODE, (unsigned long)arg32);
		if (err)
			APS_ERR("ALSPS_SET_PS_MODE unlocked_ioctl failed.");


		break;

	case COMPAT_ALSPS_GET_PS_MODE:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_MODE, (unsigned long)arg32);
		if (err)
			APS_ERR("ALSPS_GET_PS_MODE unlocked_ioctl failed.");


		break;

	case COMPAT_ALSPS_GET_PS_DATA:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_DATA, (unsigned long)arg32);
		if (err)
			APS_ERR("ALSPS_GET_PS_DATA unlocked_ioctl failed.");


		break;

	case COMPAT_ALSPS_GET_PS_RAW_DATA:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_RAW_DATA, (unsigned long)arg32);
		if (err)
			APS_ERR("ALSPS_GET_PS_RAW_DATA unlocked_ioctl failed.");


		break;

	case COMPAT_ALSPS_SET_ALS_MODE:
		err = file->f_op->unlocked_ioctl(file, ALSPS_SET_ALS_MODE, (unsigned long)arg32);
		if (err)
			APS_ERR("ALSPS_SET_ALS_MODE unlocked_ioctl failed.");


		break;

	case COMPAT_ALSPS_GET_ALS_MODE:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_MODE, (unsigned long)arg32);
		if (err)
			APS_ERR("ALSPS_GET_ALS_MODE unlocked_ioctl failed.");


		break;

	case COMPAT_ALSPS_GET_ALS_DATA:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_DATA, (unsigned long)arg32);
		if (err)
			APS_ERR("ALSPS_GET_ALS_DATA unlocked_ioctl failed.");


		break;

	case COMPAT_ALSPS_GET_ALS_RAW_DATA:
		err = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_RAW_DATA, (unsigned long)arg32);
		if (err) 
			APS_ERR("ALSPS_GET_ALS_RAW_DATA unlocked_ioctl failed.");


		break;

	case COMPAT_ALSPS_IOCTL_SET_CALI:
		err = file->f_op->unlocked_ioctl(file, ALSPS_IOCTL_SET_CALI, (unsigned long)arg32);
		if (err) 
			APS_ERR("ALSPS_IOCTL_SET_CALI unlocked_ioctl failed.");

		break;

	case COMPAT_ALSPS_SET_PS_THRESHOLD:
		err = file->f_op->unlocked_ioctl(file, ALSPS_SET_PS_THRESHOLD, (unsigned long)arg32);
		if (err) 
			APS_ERR("ALSPS_SET_PS_THRESHOLD unlocked_ioctl failed.");

		break;
	
	default:
		APS_ERR("%s not supported = 0x%04x", __func__, cmd);
		err = -ENOIOCTLCMD;
		break;

	}
	return err;
}
#endif
/*----------------------------------------------------------------------------*/

static struct file_operations apds9921_fops = {
	.owner          = THIS_MODULE,
	.open           = apds9921_open,
	.release        = apds9921_release,
	.unlocked_ioctl = apds9921_unlocked_ioctl,

#ifdef CONFIG_COMPAT
	.compat_ioctl = apds9921_compat_ioctl,
#endif

};

/*----------------------------------------------------------------------------*/
static struct miscdevice apds9921_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "als_ps",
	.fops  = &apds9921_fops,
};

/*----------------------------------------------------------------------------*/
static int apds9921_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	APS_FUN();    
	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9921_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void apds9921_early_suspend(struct early_suspend *h) 
{
	/*early_suspend is only applied for ALS*/
	struct apds9921_priv *obj = container_of(h, struct apds9921_priv, early_drv);   
	int err;

	APS_FUN();    

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return;
	}
	
	atomic_set(&obj->als_suspend, 1);
	if (test_bit(CMC_BIT_ALS, &obj->enable)) {
		if ((err = apds9921_enable_als(obj->client, 0))) {
			APS_ERR("disable als fail: %d\n", err); 
		}
	}
}

/*----------------------------------------------------------------------------*/
static void apds9921_late_resume(struct early_suspend *h)
{
	/*early_suspend is only applied for ALS*/
	struct apds9921_priv *obj = container_of(h, struct apds9921_priv, early_drv);         
	int err;
	
	APS_FUN();

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if (test_bit(CMC_BIT_ALS, &obj->enable)) {
		if ((err = apds9921_enable_als(obj->client, 1))) {
			APS_ERR("enable als fail: %d\n", err);  
		}
	}
}
#endif
/*----------------------------------------------------------------------------*/
static int temp_als = 0;
static int ALS_FLAG = 0;

int apds9921_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int value;
	int err = 0;
	
	struct hwm_sensor_data* sensor_data;
	struct apds9921_priv *obj = (struct apds9921_priv *)self;
	
	//APS_FUN(f);
	switch (command) {
		case SENSOR_DELAY:
			if ((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			} else {	
				value = *(int *)buff_in;
				if (value) {
					if ((err = apds9921_enable_ps(obj->client, 1))) {
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}

					set_bit(CMC_BIT_PS, &obj->enable);

					if (!test_bit(CMC_BIT_ALS, &obj->enable)) {
						ALS_FLAG = 1;
						if ((err = apds9921_enable_als(obj->client, 1))) {
							APS_ERR("enable als fail: %d\n", err); 
							return -1;
						}
					}
				} else {
					if ((err = apds9921_enable_ps(obj->client, 0))) {
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}

					clear_bit(CMC_BIT_PS, &obj->enable);
					if (ALS_FLAG == 1) {
						if ((err = apds9921_enable_als(obj->client, 0))) {
							APS_ERR("disable als fail: %d\n", err); 
							return -1;
						}

						ALS_FLAG = 0;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			if ((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data))) {
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			} else {
				sensor_data = (struct hwm_sensor_data *)buff_out;	
				apds9921_read_ps(obj->client, &obj->ps);
				APS_ERR("apds9921_ps_operate als data=%d!\n",obj->als);
				sensor_data->values[0] = apds9921_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;

		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int apds9921_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* sensor_data;
	struct apds9921_priv *obj = (struct apds9921_priv *)self;

	switch (command) {
		case SENSOR_DELAY:
			if ((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if ((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			} else {
				value = *(int *)buff_in;				
				if (value) {
					if ((err = apds9921_enable_als(obj->client, 1))) {
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				} else {
					if ((err = apds9921_enable_als(obj->client, 0))) {
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if ((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data))) {
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			} else {
				sensor_data = (struct hwm_sensor_data *)buff_out;
				/*yucong MTK add for fixing known issue*/
				apds9921_read_als(obj->client, &obj->als);
				if (obj->als == 0) {
					sensor_data->values[0] = temp_als;				
				} else {
					u16 b[2];
					int i;

					for (i = 0;i < 2;i++) {
						apds9921_read_als(obj->client, &obj->als);
						b[i] = obj->als;
					}

					(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
					sensor_data->values[0] = apds9921_get_als_value(obj, obj->als);
					temp_als = sensor_data->values[0];
				}
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;

		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
	int res = 0;
	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return -1;
	}
	APS_LOG("apds9921_obj als enable value = %d\n", en);

    if(en)
	{
		if((res = apds9921_enable_als(apds9921_obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", res); 
			return -1;
		}
		set_bit(CMC_BIT_ALS, &apds9921_obj->enable);
		
	}
	else
	{
		if((res = apds9921_enable_als(apds9921_obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", res); 
			return -1;
		}
		clear_bit(CMC_BIT_ALS, &apds9921_obj->enable);
	}
    
	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_get_data(int* value, int* status)
{
	int err = 0;

	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return -1;
	}

	if((err = apds9921_read_als(apds9921_obj->client, &apds9921_obj->als)))
	{
		err = -1;
	}
	else
	{
		*value = apds9921_get_als_value(apds9921_obj, apds9921_obj->als);
		//*value = apds9921_obj->als;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
	int res = 0;
	if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return -1;
	}
	APS_LOG("apds9921_obj ps enable value = %d\n", en);

    if(en)
	{
		if((res = apds9921_enable_ps(apds9921_obj->client, 1)))
		{
			APS_ERR("enable ps fail: %d\n", res); 
			return -1;
		}
		set_bit(CMC_BIT_PS, &apds9921_obj->enable);
	}
	else
	{
		if((res = apds9921_enable_ps(apds9921_obj->client, 0)))
		{
			APS_ERR("disable ps fail: %d\n", res); 
			return -1;
		}
		clear_bit(CMC_BIT_PS, &apds9921_obj->enable);
	}
    
	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
    
	return 0;

}
/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_get_data(int* value, int* status)
{
    int err = 0;

    if(!apds9921_obj)
	{
		APS_ERR("apds9921_obj is null!!\n");
		return -1;
	}
    
    if((err = apds9921_read_ps(apds9921_obj->client, &apds9921_obj->ps)))
    {
        err = -1;
    }
    else
    {
        *value = apds9921_get_ps_value(apds9921_obj, apds9921_obj->ps);
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }
    
	return err;
}

/*----------------------------------------------------------------------------*/
static int apds9921_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, APDS9921_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9921_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct apds9921_priv *obj;
	//struct hwmsen_object obj_ps, obj_als;
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
    struct ps_data_path ps_data={0};
    int err = 0;
    
    APS_FUN();

    if (!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))) {
        err = -ENOMEM;
		goto exit_kzalloc;
	}

	memset(obj, 0, sizeof(*obj));
	apds9921_obj = obj;
	obj->hw = hw;
	apds9921_get_addr(obj->hw, &obj->addr);

	/* for interrup work mode support */
	INIT_WORK(&obj->eint_work, apds9921_eint_work);


	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 50);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  200);
	atomic_set(&obj->ps_thd_val_low,  180);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	obj->ps_cali = 0;
    
	apds9921_i2c_client = client;
    obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint"); 

	if (1 == obj->hw->polling_mode_ps) {
		//obj_ps.polling = 1;
	} else {
		//obj_ps.polling = 0;
	}
    APS_LOG("apds9921_init_client() start !!\n");
	if ((err = apds9921_init_client(client))) {
		goto exit_init_failed;
	}

	APS_LOG("apds9921_init_client() OK!\n");

	if ((err = misc_register(&apds9921_device))) {
		APS_ERR("apds9921_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	als_ctl.is_use_common_factory =false;
	ps_ctl.is_use_common_factory = false;
	/*------------------------apds9921 attribute file for debug--------------------------------------*/
	if((err = apds9921_create_attr(&apds9921_init_info.platform_diver_addr->driver)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------apds9921 attribute file for debug--------------------------------------*/

	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = true;
#else
    als_ctl.is_support_batch = false;
#endif
	
	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = true;
	//ps_ctl.is_ps_polling_run = true;
	//ps_ctl.is_polling_mode = true;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = true;
#else
    ps_ctl.is_support_batch = false;
#endif
	
	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
	
#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	obj->early_drv.suspend  = apds9921_early_suspend;
	obj->early_drv.resume   = apds9921_late_resume;  
	register_early_suspend(&obj->early_drv);
#endif

	apds9921_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
	return 0;
exit_create_attr_failed:
exit_sensor_obj_attach_fail:
	misc_deregister(&apds9921_device);
exit_misc_device_register_failed:
exit_init_failed:
	kfree(obj);
exit_kzalloc:
	apds9921_i2c_client = NULL;           
// MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	apds9921_init_flag = -1;
    return err;
}

/*----------------------------------------------------------------------------*/
static int apds9921_i2c_remove(struct i2c_client *client)
{
	int err;	
	if((err = apds9921_delete_attr(&(apds9921_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("apds9921_delete_attr fail: %d\n", err);
	} 
	
	if ((err = misc_deregister(&apds9921_device))) {
		APS_ERR("misc_deregister fail: %d\n", err);    
	}

    apds9921_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}


/*----------------------------------------------------------------------------*/
static int  apds9921_local_init(void)
{

	apds9921_power(hw, 1);

	if (i2c_add_driver(&apds9921_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}

	if (-1 == apds9921_init_flag) {
		return -1;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9921_remove(void)
{

	APS_FUN();    
	apds9921_power(hw, 0);    
	i2c_del_driver(&apds9921_i2c_driver);
	return 0;
}



/*----------------------------------------------------------------------------*/
static int __init apds9921_init(void)
{

    const char *name = "mediatek,apds9921";
    APS_FUN();
    hw =   get_alsps_dts_func(name, hw);
    if (!hw)
    APS_ERR("get dts info fail\n");
    alsps_driver_add(&apds9921_init_info); 

    return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit apds9921_exit(void)
{
	APS_FUN();
}

/*----------------------------------------------------------------------------*/
module_init(apds9921_init);
module_exit(apds9921_exit);

/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Lee Kai Koon");
MODULE_DESCRIPTION("APDS9921 driver");
MODULE_LICENSE("GPL");
