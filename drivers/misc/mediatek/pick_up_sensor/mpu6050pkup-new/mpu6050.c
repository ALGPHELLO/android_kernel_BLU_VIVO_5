/* MPU6050 motion sensor driver
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
#include <asm/io.h>

#include <cust_pkup.h>
#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
#include "mpu6050.h"
#include <hwmsen_helper.h>
#include <batch.h>
#include <pick_up.h>

#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

//#define POWER_NONE_MACRO MT65XX_POWER_NONE
static DEFINE_MUTEX(mpu6050_i2c_mutex);

/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/

static struct mpu6050_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;

#define MPU6050_DEV_NAME        "MPU6050PKUP"   /* name must different with gyro mpu6050 */
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id mpu6050_i2c_id[] = {{MPU6050_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_mpu6050={ I2C_BOARD_INFO(MPU6050_DEV_NAME, (MPU6050_I2C_SLAVE_ADDR>>1))};

#ifdef MPU6050_ACCESS_BY_GSE_I2C
extern int MPU6050_hwmsen_read_block(u8 addr, u8 *buf, u8 len);
extern int MPU6050_hwmsen_write_block(u8 addr, u8 *buf, u8 len);
#endif

/*----------------------------------------------------------------------------*/
static int mpu6050_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mpu6050_i2c_remove(struct i2c_client *client);
static int mpu6050_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);

static int mpu6050_local_init(void);
static int mpu6050_remove(void);
static int mpu6050_init_flag =-1; // 0<==>OK -1 <==> fail

static struct pkup_init_info mpu6050_init_info = {
		.name = "mpu6050pkup",
		.init = mpu6050_local_init,
		.uninit = mpu6050_remove,
};


/*----------------------------------------------------------------------------*/
struct mpu6050_i2c_data
{
    struct i2c_client *client;
	struct work_struct eint_work;
    struct pkup_hw *hw;

    struct device_node *irq_node;
    int irq;
    
    /*misc*/
    atomic_t                trace;
    atomic_t                suspend;

};

static const struct of_device_id pkup_of_match[] = {
    {.compatible = "mediatek,pkup"},
    {},

};
/*----------------------------------------------------------------------------*/
static struct i2c_driver mpu6050_i2c_driver = {
    .driver = {
        .name           = MPU6050_DEV_NAME,
        .of_match_table = pkup_of_match, 
    },
    .probe              = mpu6050_i2c_probe,
    .remove             = mpu6050_i2c_remove,
    .detect             = mpu6050_i2c_detect,
    .id_table = mpu6050_i2c_id,
};




//#define PKUP_TAG		"[PICK_UP]"
//#define PKUP_FUN(f)		printk(PKUP_TAG"%s\n", __func__)
//#define PKUP_ERR(fmt, args...)	printk(PKUP_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
//#define PKUP_LOG(fmt, args...)	printk(PKUP_TAG fmt, ##args)
//#define PKUP_VER(fmt, args...)  printk(PKUP_TAG"%s: "fmt, __func__, ##args) //((void)0)



//static unsigned int power_on = 0;
struct pkup_hw pkup_cust;
static struct pkup_hw *hw = &pkup_cust;
struct platform_device *pkupPltFmDev;

extern int MPU6050_gyro_power(void);
extern int MPU6050_gyro_mode(void);
extern int MPU6050_gse_power(void);
extern int MPU6050_gse_mode(void);

//int MPU6050_pkup_power( void)
//{
//    return(power_on);
//}
//EXPORT_SYMBOL(MPU6050_gse_power);

//int MPU6050_pkup_mode(void)
//{
//    return sensor_power;
//}
//EXPORT_SYMBOL(MPU6050_gse_mode);


//static int mpu6050_power_reset(struct i2c_client *client);




/*--------------------mpu6050 power control function----------------------------------*/
static void MPU6050_power(struct pkup_hw *hw, unsigned int on)
{
    /*
    if (hw->power_id != POWER_NONE_MACRO)        // have externel LDO
    {
        PKUP_LOG("power %s\n", on ? "on" : "off");
        if (power_on == on)  // power status not change
        {
            PKUP_LOG("ignore power control: %d\n", on);
        }
        else if (on) // power on
        {
            if (!hwPowerOn(hw->power_id, hw->power_vol, "MPU6050G"))
            {
                PKUP_ERR("power on fails!!\n");
            }
        }
        else    // power off
        {
            if (MPU6050_gyro_power() == false)
            {
                if (!hwPowerDown(hw->power_id, "MPU6050G"))
                {
                    PKUP_ERR("power off fail!!\n");
                }
            }
        }
    }
    power_on = on;
*/
}





//----------------------------------------------------------------------------//
static int MPU6050_SetPowerMode(struct i2c_client *client, bool enable)
{
    u8 databuf[2] = {0};    
    int res = 0;

    if (enable == sensor_power)
    {
        PKUP_LOG("Sensor power status is newest!\n");
        return MPU6050_SUCCESS;
    }

#ifdef MPU6050_ACCESS_BY_GSE_I2C
    if (MPU6050_hwmsen_read_block(MPU6050_REG_POWER_CTL, databuf, 0x01))
#else
    if (hwmsen_read_byte(client, MPU6050_REG_POWER_CTL, databuf))
#endif 
    {
        PKUP_ERR("read power ctl register err!\n");
        return MPU6050_ERR_I2C;
    }

    databuf[0] &= ~MPU6050_SLEEP;   
    if (enable == 0)
    {
        if ((MPU6050_gse_mode() == 0)&&(MPU6050_gyro_mode() == 0))
        {
            databuf[0] |= MPU6050_SLEEP;
        }
    }
    else
    {
        // do nothing
    }

#ifdef MPU6050_ACCESS_BY_GSE_I2C
    res = MPU6050_hwmsen_write_block(MPU6050_REG_POWER_CTL, databuf, 0x1);
#else
    databuf[1] = databuf[0];
    databuf[0] = MPU6050_REG_POWER_CTL;  
    res = i2c_master_send(client, databuf, 0x2);
#endif 

    if (res <= 0)
    {
        PKUP_LOG("set power mode failed!\n");
        return MPU6050_ERR_I2C;
    }
    else
    {
        PKUP_LOG("set power mode ok %d!\n", enable);
    }

    sensor_power = enable;

    return MPU6050_SUCCESS;    
}


//----------------------------------------------------------------------------//
static int MPU6050_SetLowPowerMode(struct i2c_client *client, bool enable)
{
    u8 databuf[2] = {0};
    int res = 0;
return 0;
#ifdef MPU6050_ACCESS_BY_GSE_I2C
    if (MPU6050_hwmsen_read_block(MPU6050_REG_POWER_CTL, databuf, 0x01))
#else
    if (hwmsen_read_byte(client, MPU6050_REG_POWER_CTL, databuf))
#endif
    {
        PKUP_ERR("read power ctl register err!\n");
        return MPU6050_ERR_I2C;
    }

    if (enable == 1) {
    	databuf[0] &= ~MPU6050_SLEEP;
		databuf[0] |= 0x28;
    } else {
    	databuf[0] |= MPU6050_SLEEP;
		databuf[0] &= ~0x28;
    }

#ifdef MPU6050_ACCESS_BY_GSE_I2C
    res = MPU6050_hwmsen_write_block(MPU6050_REG_POWER_CTL, databuf, 0x1);
#else
    databuf[1] = databuf[0];
    databuf[0] = MPU6050_REG_POWER_CTL;
    res = i2c_master_send(client, databuf, 0x2);
#endif
	
    if (res <= 0)
    {
        PKUP_LOG("set power mode failed!\n");
        return MPU6050_ERR_I2C;
    }
    else
    {
        PKUP_LOG("set power mode ok %x!\n", databuf[0]);
    }

	databuf[0] = 0x07;
#ifdef MPU6050_ACCESS_BY_GSE_I2C
	res = MPU6050_hwmsen_write_block(MPU6050_REG_POWER_CTL2, databuf, 0x1);
#else
	databuf[1] = databuf[0];
	databuf[0] = MPU6050_REG_POWER_CTL2;
	res = i2c_master_send(client, databuf, 0x2);
#endif

	if (res <= 0)
	{
		PKUP_LOG("set power ctl failed!\n");
		return MPU6050_ERR_I2C;
	}
	else
	{
		PKUP_LOG("set low power ctl ok %x!\n", databuf[0]);
	}

    return MPU6050_SUCCESS;
}

static int MPU6050_SetAccIntEnable(struct i2c_client *client, u8 intenable)
{
    u8 databuf[2];
    int res = 0;

    memset(databuf, 0, sizeof(u8)*2);
	memset(databuf, 0, sizeof(u8)*2);
	
#ifdef MPU6050_ACCESS_BY_GSE_I2C
		if (MPU6050_hwmsen_read_block(MPU6050_REG_ACC_INT_CTL, databuf, 0x01))
#else
		if (hwmsen_read_byte(client, MPU6050_REG_ACC_INT_CTL, databuf))
#endif
		{
			PKUP_ERR("read acc interrupt enable register err!\n");
			return MPU6050_ERR_I2C;
		}
	
		if (intenable == 1) {
			
			databuf[0] |= MPU6050_ACC_INT_EN;
		} else if (intenable == 0) {
			
			databuf[0] &= ~MPU6050_ACC_INT_EN;
		} else {
			//do nothing
		}
		
		//res = mpu_i2c_write_block(client, MPU6050_REG_INT_ENABLE, databuf, 0x1);
	
#ifdef MPU6050_ACCESS_BY_GSE_I2C
		res = MPU6050_hwmsen_write_block(MPU6050_REG_ACC_INT_CTL, databuf, 0x1);
#else
	
		databuf[1] = databuf[0];
		databuf[0] = MPU6050_REG_ACC_INT_CTL;
		res = i2c_master_send(client, databuf, 0x2);
#endif
	
		if (res < 0)
		{
			return MPU6050_ERR_I2C;
		} else {
			PKUP_LOG("set acc interrupt enable ok %x %x %d!\n", databuf[0], databuf[1], intenable);
		}
	
		return MPU6050_SUCCESS;

}
static int MPU6050_SetWOMEnable(struct i2c_client *client, u8 intenable)
{
    u8 databuf[2];
    int res = 0;

    memset(databuf, 0, sizeof(u8)*2);

#ifdef MPU6050_ACCESS_BY_GSE_I2C
    if (MPU6050_hwmsen_read_block(MPU6050_REG_INT_ENABLE, databuf, 0x01))
#else
    if (hwmsen_read_byte(client, MPU6050_REG_INT_ENABLE, databuf))
#endif
    {
        PKUP_ERR("read interrupt enable register err!\n");
        return MPU6050_ERR_I2C;
    }

	if (intenable == 1) {
    	databuf[0] |= MPU6050_WOM_EN;
	} else if (intenable == 0) {
		databuf[0] &= ~MPU6050_WOM_EN;
	} else {
		//do nothing
	}
	
    //res = mpu_i2c_write_block(client, MPU6050_REG_INT_ENABLE, databuf, 0x1);

#ifdef MPU6050_ACCESS_BY_GSE_I2C
    res = MPU6050_hwmsen_write_block(MPU6050_REG_INT_ENABLE, databuf, 0x1);
#else
    databuf[1] = databuf[0];
    databuf[0] = MPU6050_REG_INT_ENABLE;
    res = i2c_master_send(client, databuf, 0x2);
#endif

    if (res < 0)
    {
        return MPU6050_ERR_I2C;
    } else {
		PKUP_LOG("set interrupt enable ok %x %x %d!\n", databuf[0], databuf[1], intenable);
	}

	return MPU6050_SUCCESS;

}
/*----------------------------------------------------------------------------*/

static int mpu6050_check_WOM(struct i2c_client *client)
{
		u8 databuf[2];
	
#ifdef MPU6050_ACCESS_BY_GSE_I2C
			if (MPU6050_hwmsen_read_block(MPU6050_REG_INT_STATUS, databuf, 0x01))
#else
			if (hwmsen_read_byte(client, MPU6050_REG_INT_STATUS, databuf))
#endif
			{
				PKUP_ERR("read interrupt status register err!\n");
				return MPU6050_ERR_I2C;
			}
			PKUP_ERR("status = %x!\n", databuf[0]);
			if (databuf[0] & MPU6050_WOM_INT_STATES) {
				return 1;
			} else {
				return 0;
			}
	

}



static int mpu6050_interrupt_config(struct i2c_client *client)
{
	u8 databuf[2];
	int res = 0;

#ifdef MPU6050_ACCESS_BY_GSE_I2C
	if (MPU6050_hwmsen_read_block(MPU6050_REG_INT_CFG, databuf, 0x01))
#else
	if (hwmsen_read_byte(client, MPU6050_REG_INT_CFG, databuf))
#endif
	{
		PKUP_ERR("read interrupt cfg register err!\n");
		return MPU6050_ERR_I2C;
	}

	databuf[0] |= (MPU6050_ACTL_LOW|MPU6050_OPEN_DRAIN|MPU6050_LATCH_INT_EN|MPU6050_INT_ANYRD_2CLEAR);

#ifdef MPU6050_ACCESS_BY_GSE_I2C
	res = MPU6050_hwmsen_write_block(MPU6050_REG_INT_CFG, databuf, 0x1);
#else
	databuf[1] = databuf[0];
	databuf[0] = MPU6050_REG_INT_CFG;
	res = i2c_master_send(client, databuf, 0x2);
#endif

	if (res <= 0)
	{
		PKUP_LOG("set interrupt cfg failed!\n");
		return MPU6050_ERR_I2C;
	}
	else
	{
		PKUP_LOG("set interrupt cfg ok %x!\n", databuf[0]);
	}
	return 0;

}

/*----------------------------------------------------------------------------*/
static void mpu6050_eint_work(struct work_struct *work)
{
	struct mpu6050_i2c_data *obj = obj_i2c_data;

	PKUP_FUN();

	if (mpu6050_check_WOM(obj->client) ==1 ) {
		
		pkup_notify();
	}

	enable_irq(obj_i2c_data->irq);
}

static irqreturn_t mpu6050_eint_func(int irq, void *dev_id)
{
	struct mpu6050_i2c_data *obj = obj_i2c_data;
	PKUP_FUN();
	if (!obj) {
		return IRQ_HANDLED;
	}
    
    //int_top_time = sched_clock();
    schedule_work(&obj->eint_work);
    disable_irq_nosync(obj_i2c_data->irq);
    return IRQ_HANDLED;
}


static int mpu6050_eint_init(struct i2c_client *client)
{

    int ret;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_default;
    struct pinctrl_state *pins_cfg;
    u32 ints[2] = {0, 0};
    struct mpu6050_i2c_data *obj = i2c_get_clientdata(client);

    pkupPltFmDev = get_pkup_platformdev();
    /* gpio setting */
    pinctrl = devm_pinctrl_get(&pkupPltFmDev->dev);
    if (IS_ERR(pinctrl)) {
        ret = PTR_ERR(pinctrl);
        PKUP_ERR("Cannot find alsps pinctrl!\n");

    }
    pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
    if (IS_ERR(pins_default)) {
        ret = PTR_ERR(pins_default);
        PKUP_ERR("Cannot find alsps pinctrl default!\n");


    }

    pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
    if (IS_ERR(pins_cfg)) {
        ret = PTR_ERR(pins_cfg);
        PKUP_ERR("Cannot find alsps pinctrl pin_cfg!\n");


    }
    /* eint request */
    if (obj->irq_node) {
        of_property_read_u32_array(obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_request(ints[0], "pkup-sensor");
        gpio_set_debounce(ints[0], ints[1]);
        pinctrl_select_state(pinctrl, pins_cfg);
        PKUP_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

        obj->irq = irq_of_parse_and_map(obj->irq_node, 0);
        PKUP_LOG("obj->irq = %d\n", obj->irq);
        if (!obj->irq) {
            PKUP_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;

        }
        if (request_irq(obj->irq, mpu6050_eint_func, IRQF_TRIGGER_NONE, "pkup-eint", NULL)) {
            PKUP_ERR("IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;

        }
        enable_irq(obj->irq);

    } else {
        PKUP_ERR("null irq node!!\n");
        return -EINVAL;

    }

    return 0;
}

static int mpu6050_WOM_config(struct i2c_client *client, u8 threshold)
{
		u8 databuf[2];
		int res = 0;
	
	
		databuf[0] = threshold;
#ifdef MPU6050_ACCESS_BY_GSE_I2C
		res = MPU6050_hwmsen_write_block(MPU6050_REG_WON_THRES, databuf, 0x1);
#else
		databuf[1] = databuf[0];
		databuf[0] = MPU6050_REG_WON_THRES;
		res = i2c_master_send(client, databuf, 0x2);
#endif
	
		if (res <= 0)
		{
			PKUP_LOG("set WON threshold failed!\n");
			return MPU6050_ERR_I2C;
		}
		else
		{
			PKUP_LOG("set WON threshold ok %d!\n", databuf[0]);
		}

	
	return 0;
}

static int mpu6050_init_client(struct i2c_client *client)
{
    int ret = 0;	
	ret = mpu6050_eint_init(client);

    return ret;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    struct mpu6050_i2c_data *obj = obj_i2c_data;
    if (obj == NULL)
    {
        PKUP_ERR("i2c_data obj is null!!\n");
        return 0;
    }

    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct mpu6050_i2c_data *obj = obj_i2c_data;
    int trace;
    if (obj == NULL)
    {
        PKUP_ERR("i2c_data obj is null!!\n");
        return 0;
    }

    if (1 == sscanf(buf, "0x%x", &trace))
    {
        atomic_set(&obj->trace, trace);
    }
    else
    {
        PKUP_ERR("invalid content: '%s', length = %zu\n", buf, count);
    }

    return count;
}

static ssize_t show_reg_value(struct device_driver *ddri, char *buf)
{
    //ssize_t len = 0;    
    //struct mpu6050_i2c_data *obj = obj_i2c_data;
	u8 databuf[10] = {0};
	PKUP_FUN();

#ifdef MPU6050_ACCESS_BY_GSE_I2C
	if (MPU6050_hwmsen_read_block(MPU6050_REG_INT_CFG, &databuf[0], 0x01))
#else
	if (hwmsen_read_byte(obj->client, MPU6050_REG_INT_CFG, &databuf[0]))
#endif
	{
		PKUP_ERR("read MPU6050_REG_INT_CFG register err!\n");
		return MPU6050_ERR_I2C;
	}

#ifdef MPU6050_ACCESS_BY_GSE_I2C
	if (MPU6050_hwmsen_read_block(MPU6050_REG_INT_ENABLE, &databuf[1], 0x01))
#else
	if (hwmsen_read_byte(obj->client, MPU6050_REG_INT_ENABLE, &databuf[1]))
#endif
	{
		PKUP_ERR("read MPU6050_REG_INT_ENABLE register err!\n");
		return MPU6050_ERR_I2C;
	}

#ifdef MPU6050_ACCESS_BY_GSE_I2C
	if (MPU6050_hwmsen_read_block(MPU6050_REG_ACC_INT_CTL, &databuf[2], 0x01))
#else
	if (hwmsen_read_byte(obj->client, MPU6050_REG_ACC_INT_CTL, &databuf[2]))
#endif
	{
		PKUP_ERR("read MPU6050_REG_ACC_INT_CTL register err!\n");
		return MPU6050_ERR_I2C;
	}

#ifdef MPU6050_ACCESS_BY_GSE_I2C
	if (MPU6050_hwmsen_read_block(MPU6050_REG_POWER_CTL, &databuf[3], 0x01))
#else
	if (hwmsen_read_byte(obj->client, MPU6050_REG_POWER_CTL, &databuf[3]))
#endif
	{
		PKUP_ERR("read MPU6050_REG_ACC_INT_CTL register err!\n");
		return MPU6050_ERR_I2C;
	}


#ifdef MPU6050_ACCESS_BY_GSE_I2C
	if (MPU6050_hwmsen_read_block(MPU6050_REG_POWER_CTL2, &databuf[4], 0x01))
#else
	if (hwmsen_read_byte(obj->client, MPU6050_REG_POWER_CTL2, &databuf[4]))
#endif
	{
		PKUP_ERR("read MPU6050_REG_ACC_INT_CTL register err!\n");
		return MPU6050_ERR_I2C;
	}

#ifdef MPU6050_ACCESS_BY_GSE_I2C
	if (MPU6050_hwmsen_read_block(MPU6050_REG_DLPF, &databuf[5], 0x01))
#else
	if (hwmsen_read_byte(obj->client, MPU6050_REG_DLPF, &databuf[5]))
#endif
	{
		PKUP_ERR("read MPU6050_REG_ACC_INT_CTL register err!\n");
		return MPU6050_ERR_I2C;
	}

#ifdef MPU6050_ACCESS_BY_GSE_I2C
	if (MPU6050_hwmsen_read_block(MPU6050_REG_WON_THRES, &databuf[6], 0x01))
#else
	if (hwmsen_read_byte(obj->client, MPU6050_REG_WON_THRES, &databuf[6]))
#endif
	{
		PKUP_ERR("read MPU6050_REG_ACC_INT_CTL register err!\n");
		return MPU6050_ERR_I2C;
	}

#ifdef MPU6050_ACCESS_BY_GSE_I2C
	if (MPU6050_hwmsen_read_block(MPU6050_REG_LP_ACC_ODR, &databuf[7], 0x01))
#else
	if (hwmsen_read_byte(obj->client, MPU6050_REG_LP_ACC_ODR, &databuf[7]))
#endif
	{
		PKUP_ERR("read MPU6050_REG_ACC_INT_CTL register err!\n");
		return MPU6050_ERR_I2C;
	}


	sprintf(buf, "0x37 = %x \n0x38 = %x \n0x69 = %x \n0x6B = %x \n 0x6C =%x \n 0x1D = %x \n0x1F = %x \n0x1E = %x", 
		databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5], databuf[6], databuf[7]);

    return snprintf(buf, PAGE_SIZE, "%s\n", buf);
}

static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(reg,        S_IWUSR | S_IRUGO, show_reg_value,                NULL);

static struct driver_attribute *mpu6050_attr_list[] = {
    &driver_attr_trace,        /*trace log*/
	&driver_attr_reg,        /*trace reg*/

};

static int mpu6050_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(mpu6050_attr_list)/sizeof(mpu6050_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for (idx = 0; idx < num; idx++)
    {
        if (0 != (err = driver_create_file(driver, mpu6050_attr_list[idx])))
        {
            PKUP_ERR("driver_create_file (%s) = %d\n", mpu6050_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int mpu6050_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(mpu6050_attr_list)/sizeof(mpu6050_attr_list[0]));

    if (driver == NULL)
    {
        return -EINVAL;
    }

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, mpu6050_attr_list[idx]);
    }

    return err;
}

/*
static int mpu6050_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct mpu6050_i2c_data *obj = i2c_get_clientdata(client);
    int err = 0;
    PKUP_FUN();
	atomic_set(&obj->suspend, 1);
    return err;
}
static int mpu6050_resume(struct i2c_client *client)
{
    struct mpu6050_i2c_data *obj = i2c_get_clientdata(client);
    int err;
    PKUP_FUN();
	atomic_set(&obj->suspend, 0);
    return 0;
}
*/
static int mpu6050_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, MPU6050_DEV_NAME);
    return 0;
}



// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int mpu6050_open_report_data(int open)
{
	struct mpu6050_i2c_data *obj = obj_i2c_data;
	int res = 0;
	//u8 databuf[2];
	PKUP_FUN();
	
	if (open == 1) {
	/*wait vibrator stop */
		MPU6050_SetPowerMode(obj->client, true);
		mpu6050_interrupt_config(obj->client);
		mpu6050_WOM_config(obj->client, 100);	
		mpu6050_check_WOM(obj->client); 	//for clear interrupt	
        MPU6050_SetAccIntEnable(obj->client, 1); 
		MPU6050_SetWOMEnable(obj->client, 1);

	} else if (open == 0) {
		
        MPU6050_SetLowPowerMode(obj->client, 0);
		MPU6050_SetPowerMode(obj->client, false);
		MPU6050_SetAccIntEnable(obj->client, 0);
		MPU6050_SetWOMEnable(obj->client, 0);
	}
	return res;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
/*
static int mpu6050_enable_nodata(int en)
{
	PKUP_FUN();
	return 0;
}
*/

static int mpu6050_get_data(int *value , int *status)
{
	PKUP_FUN();
	return 0;
}


/*----------------------------------------------------------------------------*/
static int mpu6050_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mpu6050_i2c_data *obj;
	int err = 0;
	struct pkup_control_path ctl={0};
	struct pkup_data_path data={0};
	PKUP_FUN();

    if (!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(struct mpu6050_i2c_data));


    obj_i2c_data = obj;
    obj->client = client;
    obj->client->timing = 400;

    /* for interrup work mode support */
	INIT_WORK(&obj->eint_work, mpu6050_eint_work);
    atomic_set(&obj->trace, 0);
    atomic_set(&obj->suspend, 0);
    obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek,pkup-eint");


    i2c_set_clientdata(obj->client,obj);


    if ((err = mpu6050_init_client(obj->client)) != 0) {
        PKUP_ERR("mpu6050_init_client fail err = %d\n", err);
        goto exit_init_failed;
    }

    //ctl.is_use_common_factory = false;
    if ((err = mpu6050_create_attr(&(mpu6050_init_info.platform_diver_addr->driver))) != 0) {
        PKUP_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    ctl.open_report_data = mpu6050_open_report_data;
    err = pkup_register_control_path(&ctl);
    if(err != 0) {
        PKUP_ERR("register pick up control path err\n");
        goto out_probe;
    }

    data.get_data = mpu6050_get_data;
    err = pkup_register_data_path(&data);
    if(err != 0) {
        PKUP_ERR("register pick up data path err\n");
        goto out_probe;
    }
    mpu6050_init_flag = 0;
    PKUP_LOG("%s: OK\n", __func__);
    return 0;
out_probe:
    mpu6050_delete_attr(&(mpu6050_init_info.platform_diver_addr->driver));
exit_create_attr_failed:
exit_init_failed:
exit:
    kfree(obj);
    PKUP_ERR("%s: err = %d\n", __func__, err);
    mpu6050_init_flag =-1;
    return err;
}

/*----------------------------------------------------------------------------*/
static int mpu6050_i2c_remove(struct i2c_client *client)
{
    int err = 0;

    if ((err =  mpu6050_delete_attr(&(mpu6050_init_info.platform_diver_addr->driver))))
    {
        PKUP_ERR("mpu6050_delete_attr fail: %d\n", err);
    }

    //i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}
/*----------------------------------------------------------------------------*/

static int mpu6050_remove(void)
{
    PKUP_FUN();
    MPU6050_power(hw, 0);
    i2c_del_driver(&mpu6050_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/

static int  mpu6050_local_init(void)
{
	PKUP_FUN();
	//MPU6050_power(hw, 1);
	
    if(i2c_add_driver(&mpu6050_i2c_driver))
	{
		PKUP_ERR("add driver error 11\n");
		return -1;
	}
	
	PKUP_ERR("add driver ok 11\n");
	PKUP_FUN();
    
    if(-1 == mpu6050_init_flag)
	{
		PKUP_ERR("add driver error 22\n");
        return -1;
	}

	return 0;
}


/*----------------------------------------------------------------------------*/
static int __init mpu6050pkup_init(void)
{
    const char *name = "mediatek,mpu6050pkup";
    
    hw = get_pkup_dts_func(name, hw);
    if (!hw)
        PKUP_ERR("get dts info fail\n");

    pkup_driver_add(&mpu6050_init_info);
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit mpu6050pkup_exit(void)
{
    PKUP_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(mpu6050pkup_init);
module_exit(mpu6050pkup_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MPU6050 pick up driver");
MODULE_AUTHOR("tangjh@gionee.com");
