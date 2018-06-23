/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* drivers/hwmon/mt6516/amit/ltr559.c - LTR559 ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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
#include <linux/of.h>
#include <linux/of_address.h>                                                                                                                                
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include "cust_alsps_ltr559.h"
#include "alsps.h"
#include "gn_ltr559.h"

// Gionee lizhi 20150205 add for CR01444969 begin
#ifdef CONFIG_GN_BSP_MTK_DEVICE_CHECK
#include <linux/gn_device_check.h>
extern int gn_set_device_info(struct gn_device_info gn_dev_info);
#endif
// Gionee lizhi 20150205 add for CR01444969 begin end



#define POWER_NONE_MACRO MT65XX_POWER_NONE
#define COUNT_FOR_CALI 6

#define LTR559_I2C_ADDR_RAR	0 /*!< the index in obj->hw->i2c_addr: alert response address */
#define LTR559_I2C_ADDR_ALS	1 /*!< the index in obj->hw->i2c_addr: ALS address */
#define LTR559_I2C_ADDR_PS	2 /*!< the index in obj->hw->i2c_addr: PS address */
#define LTR559_DEV_NAME		"LTR559"

#define APS_TAG	"[ALS/PS] "
#define APS_FUN(f)      printk(KERN_ERR APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)	printk(KERN_ERR APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_DEBUG
#if defined(APS_DEBUG)
#define APS_LOG(fmt, args...)	printk(KERN_ERR APS_TAG "%s(%d):" fmt, __FUNCTION__, __LINE__, ##args)
#define APS_DBG(fmt, args...)	printk(KERN_ERR APS_TAG fmt, ##args)
#else
#define APS_LOG(fmt, args...)
#define APS_DBG(fmt, args...)
#endif


static struct i2c_client *ltr559_i2c_client = NULL;
static const struct i2c_device_id ltr559_i2c_id[] = {{LTR559_DEV_NAME,0},{}};

//struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &cust_alsps_hw;
struct platform_device *alspsPltFmDev;

static int ltr559_enable_ps(struct i2c_client *client, bool enable);  

static int ltr559_local_init(void);
static int ltr559_remove(void);
static int ltr559_init_flag = 0;
static int first_init = 0; 

static struct alsps_init_info ltr559_init_info = {
    .name = "psensor_ltr559",
    .init = ltr559_local_init,
    .uninit = ltr559_remove,
};
static struct wake_lock alsps_wakelock;

#if defined(CONFIG_GN_BSP_PS_STATIC_CALIBRATION)
struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int  valid;
};

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
#endif

typedef enum {
    CMC_TRC_APS_DATA	= 0x0002,
    CMC_TRC_EINT		= 0x0004,
    CMC_TRC_IOCTL		= 0x0008,
    CMC_TRC_I2C		= 0x0010,
    CMC_TRC_CVT_ALS		= 0x0020,
    CMC_TRC_CVT_PS		= 0x0040,
    CMC_TRC_DEBUG		= 0x8000,
} CMC_TRC;

typedef enum {
    CMC_BIT_ALS		= 1,
    CMC_BIT_PS		= 2,
} CMC_BIT;

struct ltr559_priv {
    struct alsps_hw *hw;
    struct i2c_client *client;
    struct work_struct eint_work;
    /*misc*/
    atomic_t	trace;
    atomic_t	i2c_retry;
    atomic_t	als_suspend;
    atomic_t	als_debounce;	/*debounce time after enabling als*/
    atomic_t	als_deb_on;	/*indicates if the debounce is on*/
    atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
    atomic_t	ps_mask;	/*mask ps: always return far away*/
    atomic_t	ps_debounce;	/*debounce time after enabling ps*/
    atomic_t	ps_deb_on;	/*indicates if the debounce is on*/
    atomic_t	ps_deb_end;	/*the jiffies representing the end of debounce*/
    atomic_t	ps_suspend;

    int		als;
    int		ps;
    u8		_align;
    u16		als_level_num;
    u16		als_value_num;
    u32		als_level[C_CUST_ALS_LEVEL-1];
    u32		als_value[C_CUST_ALS_LEVEL];

    bool		als_enable;	/*record current als status*/
    unsigned int	als_widow_loss;

    bool		ps_enable;	 /*record current ps status*/
    unsigned int	ps_thd_val;	 /*the cmd value can't be read, stored in ram*/

    unsigned int ps_thd_val_high;
    unsigned int ps_thd_val_low;

    ulong		enable;		 /*record HAL enalbe status*/
    ulong		pending_intr;	/*pending interrupt*/
    unsigned int	polling;

    unsigned int	ps_cali_noise[PS_CALI_NOISE_NUM];
    unsigned int	ps_cali_offset_high[PS_CALI_NOISE_NUM];
    unsigned int	ps_cali_offset_low[PS_CALI_NOISE_NUM];
    unsigned int	cali_index;
    unsigned int	cali_noise;

    unsigned int	ps_cali_noise_num;
    unsigned int	ps_cali_offset_high_num;
    unsigned int	ps_cali_offset_low_num;
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend	early_drv;
#endif
    struct device_node *irq_node;
    int     irq;

};

static struct ltr559_priv *ltr559_obj = NULL;

static int ltr559_get_ps_value(struct ltr559_priv *obj, int ps);
static int ltr559_get_als_value(struct ltr559_priv *obj, int als);

static int ltr559_dynamic_calibrate(void);

static int hwmsen_read_byte_sr(struct i2c_client *client, u8 addr, int *data)
{
    u8 buf;
    int ret = 0;
    struct i2c_client client_read = *client;

    client_read.addr = (client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG |I2C_RS_FLAG;
    buf = addr;
    ret = i2c_master_send(&client_read, (const char*)&buf, 1<<8 | 1);
    if (ret < 0) {
        APS_ERR("send command error!!\n");
        return -EFAULT;
    }

    *data = buf;
    client->addr = client->addr& I2C_MASK_FLAG;
    return 0;
}

int ltr559_get_timing(void)
{
    return 200;
}

int ltr559_read_data_als(struct i2c_client *client, int *data)
{
    struct ltr559_priv *obj = i2c_get_clientdata(client);
    int alsval_ch1_lo = 0;
    int alsval_ch1_hi = 0;
    int alsval_ch0_lo = 0;
    int alsval_ch0_hi = 0;
    int luxdata_int;
    int luxdata_flt;
    int ratio;
    int alsval_ch0;
    int alsval_ch1;

    int als_data_valid = 0;

	int read_count = 3;
als_data_try:
    if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH1_0, &alsval_ch1_lo)) {
        APS_ERR("reads als data (ch1 lo) = %d\n", alsval_ch1_lo);
        return -EFAULT;
    }
    if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH1_1, &alsval_ch1_hi)) {
        APS_ERR("reads aps data (ch1 hi) = %d\n", alsval_ch1_hi);
        return -EFAULT;
    }
    alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;
    //	APS_LOG("alsval_ch1_hi=%x alsval_ch1_lo=%x\n",alsval_ch1_hi,alsval_ch1_lo);


    if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH0_0, &alsval_ch0_lo)) {
        APS_ERR("reads als data (ch0 lo) = %d\n", alsval_ch0_lo);
        return -EFAULT;
    }
    if (hwmsen_read_byte_sr(client, APS_RO_ALS_DATA_CH0_1, &alsval_ch0_hi)) {
        APS_ERR("reads als data (ch0 hi) = %d\n", alsval_ch0_hi);
        return -EFAULT;
    }
    alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;
    //	APS_LOG("alsval_ch0_hi=%x alsval_ch0_lo=%x\n",alsval_ch0_hi,alsval_ch0_lo);

    // FIXME:
    // make sure we will not meet div0 error
    if ((alsval_ch0 + alsval_ch1) == 0) {
        APS_ERR("Both CH0 and CH1 are zero\n");
        ratio = 1000;
    } else {
        ratio = (alsval_ch1 * 1000) / (alsval_ch0 + alsval_ch1);
    }
    if (ratio < 450) {
        luxdata_flt = (17743 * alsval_ch0) + (11059 * alsval_ch1);
        luxdata_flt = luxdata_flt / 10000;
    } else if ((ratio >= 450) && (ratio < 640)) {
        luxdata_flt = (37725 * alsval_ch0) - (13363 * alsval_ch1);
        luxdata_flt = luxdata_flt / 10000;
    } else if ((ratio >= 640) && (ratio < 1000)) {
        luxdata_flt = (16900 * alsval_ch0) - (1690 * alsval_ch1);
        luxdata_flt = luxdata_flt / 10000;
    } else {
        luxdata_flt = 0;
        if(hwmsen_read_byte_sr(client,APS_RO_ALS_PS_STATUS,&als_data_valid)){
            APS_ERR("read als data valid flag wrong\n");
        }else if(als_data_valid&0x80){
            APS_ERR("ALS data is not ready\n");
            msleep(50);
			read_count --;
			if (read_count <= 0) {
			    luxdata_int = 1;
				*data = luxdata_int; 
				return 0;
			}
            goto als_data_try;
        }
    }

    if (luxdata_flt > 65535)
        luxdata_flt = 65534;

    if (luxdata_flt <0)
        luxdata_flt = 65534;

    //Gionee: mali 2012-06-14 modify the CR00623845 from the old data to the new data for mmi test end
    // convert float to integer;
    luxdata_int = luxdata_flt;
    if ((luxdata_flt - luxdata_int) > 0.5) {
        luxdata_int = luxdata_int + 1;
    } else {
        luxdata_int = luxdata_flt;
    }

    if (atomic_read(&obj->trace) & CMC_TRC_APS_DATA) {
        APS_DBG("ALS (CH0): 0x%04X\n", alsval_ch0);
        APS_DBG("ALS (CH1): 0x%04X\n", alsval_ch1);
        APS_DBG("ALS (Ratio): %d\n", ratio);
        APS_DBG("ALS: %d\n", luxdata_int);
    }

    *data = luxdata_int;



    return 0;
}

int ltr559_read_data_ps(struct i2c_client *client, int *data)
{
    int psval_lo = 0;
    int psval_hi = 0;
    int psdata = 0;

    if (hwmsen_read_byte_sr(client, APS_RO_PS_DATA_0, &psval_lo)) {
        APS_ERR("reads aps data = %d\n", psval_lo);
        return -EFAULT;
    }

    if (hwmsen_read_byte_sr(client, APS_RO_PS_DATA_1, &psval_hi)) {
        APS_ERR("reads aps hi data = %d\n", psval_hi);
        return -EFAULT;
    }

    psdata = ((psval_hi & 7) * 256) + psval_lo;
    *data = psdata;
    return 0;

}

int ltr559_init_device(struct i2c_client *client)
{
    int i = 0;
    int *reg_table = &regster_value_table[0];
    int part_id = 255;

    APS_FUN();

    if (hwmsen_read_byte_sr(client, APS_RO_PART_ID, &part_id)) {
        APS_ERR("ltr559 reads ps id error = %d\n", part_id);
        return -EFAULT;
    }
    APS_LOG("ltr559 reads ps id = %d\n", part_id);

    for (i = first_init; i < APS_REGISTER_COUNT; i++) {
        if (reg_table[i] & REG_WRITE_FLAG) {
            if (hwmsen_write_byte(client, APS_REGISTER_BASE+i, (unsigned char)reg_table[i])) goto err;
        }
    }
    first_init = 1;
    msleep(WAKEUP_DELAY);
    return 0;
err:
    APS_ERR("init device reg error!\n");
    return -EFAULT;
}

/*----------------------------------------------------------------------------*/
static void ltr559_power(struct alsps_hw *hw, unsigned int on)
{
}

static int ltr559_enable_als(struct i2c_client *client, bool enable)
{
    struct ltr559_priv *obj = i2c_get_clientdata(client);
    int err=0;
    int regdata=0;

    if (enable == obj->als_enable)
        return 0;

    if (hwmsen_read_byte_sr(client, APS_RW_ALS_CONTR, &regdata)) {
        APS_ERR("read APS_RW_ALS_CONTR register err!\n");
        return -1;
    }

    if (enable == 1) {
        APS_LOG("ALS(1): enable als only \n");
        regdata |= 0x01;
    } else {
        APS_LOG("ALS(1): disable als only \n");
        regdata &= 0xfe;
    }

    if (hwmsen_write_byte(client, APS_RW_ALS_CONTR, regdata)) {
        APS_LOG("ltr559_enable_als failed!\n");
        return -1;
    }
    obj->als_enable = enable;
    return err;
}

static int ltr559_enable_ps(struct i2c_client *client, bool enable)
{
    struct ltr559_priv *obj = i2c_get_clientdata(client);
    int err = 0;
    int regdata = 0;
    int regint = 0;
    struct hwm_sensor_data sensor_data;

    APS_LOG("enable:  %d, obj->ps_enable: %d\n",enable, obj->ps_enable);
    if (enable == obj->ps_enable)
        return 0;

    if (hwmsen_read_byte_sr(client, APS_RW_PS_CONTR, &regdata)) {
        APS_ERR("read APS_RW_PS_CONTR register err!\n");
        return -1;
    }

    if (enable == 1) {
        if ((err = ltr559_init_device(client))) {
            APS_ERR("ltr559_init_device init dev: %d\n", err);
            return err;
        }
        regdata |= 0b00000010;
        APS_LOG("PS(0): enable +++++++++++\n");
    } else {		
        APS_LOG("PS(0): disable -----------------\n");
        regdata &= 0b11111100;	// de-active the ps
    }

    if (hwmsen_write_byte(client, APS_RW_PS_CONTR, regdata)) {
        APS_ERR("ltr559_enable_ps failed!\n");
        goto err;
    }

    if (hwmsen_read_byte_sr(client, APS_RW_PS_CONTR, &regdata)) {
        APS_ERR("ltr559_read failed!\n");
        goto err;
    }

    if (regdata & 0x02) {

        msleep(50);
        if (hwmsen_read_byte_sr(obj->client, APS_RO_PS_DATA_1, &regint)) {
            APS_ERR("reads APS_RO_PS_DATA_1 data\n");
            return -1;
        }

        if (regint & 0x80) {
            sensor_data.values[0] = 1;
            sensor_data.value_divide = 1;
            sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
            APS_LOG("ltr559 read ps data exception = %d \n",sensor_data.values[0]);
            err = ps_report_interrupt_data(sensor_data.values[0]);
            if(err)
                APS_ERR("call ps_report_interrupt_data fail = %d\n", err);

            obj->ps_thd_val_high = 1250;
            obj->ps_thd_val_low = 1200;
            if (hwmsen_write_byte(obj->client, APS_RW_PS_THRES_UP_0, obj->ps_thd_val_high & 0xff)) goto err;
            if (hwmsen_write_byte(obj->client, APS_RW_PS_THRES_UP_1, (obj->ps_thd_val_high >> 8) & 0xff)) goto err;
            if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,obj->ps_thd_val_low & 0xff)) goto err;
            if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,(obj->ps_thd_val_low >> 8) & 0xff)) goto err;				

           // mt_eint_unmask(CUST_EINT_ALS_NUM);
            enable_irq(ltr559_obj->irq);
            APS_LOG("the ps data is overflow in enable\n");		   
            return 0;
        }

        if (ltr559_dynamic_calibrate() < 0)
            goto err;

    }
#ifdef CONFIG_GN_BSP_ALSPS_INTERRUPT_MODE
  //  mt_eint_unmask(CUST_EINT_ALS_NUM);
    enable_irq(ltr559_obj->irq);
    ltr559_read_data_ps(obj->client, &obj->ps);
    sensor_data.values[0] = ltr559_get_ps_value(obj, obj->ps);
    sensor_data.value_divide = 1;
    sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
    err = ps_report_interrupt_data(sensor_data.values[0]);
    if(err)
    {
        APS_ERR("call ps_report_interrupt_data fail = %d\n", err);
        return err;
    }
#endif

    obj->ps_enable = enable;
    return err;

err:
  //  mt_eint_unmask(CUST_EINT_ALS_NUM);
    enable_irq(ltr559_obj->irq);
    return -1;
}

static int ltr559_check_intr(struct i2c_client *client)
{
    struct ltr559_priv *obj = i2c_get_clientdata(client);
    int err;
    int data=0;

    err = hwmsen_read_byte_sr(client,APS_RO_ALS_PS_STATUS,&data);

    if (err) {
        APS_ERR("WARNING: read int status: %d\n", err);
        return -1;
    }

    if (data & 0x02)
        set_bit(CMC_BIT_PS, &obj->pending_intr);
    else
        clear_bit(CMC_BIT_PS, &obj->pending_intr);

   // if (atomic_read(&obj->trace) & CMC_TRC_DEBUG)
       // APS_LOG("check intr: 0x%u\n", obj->pending_intr);
    return 0;
}

/*----------------------------------------------------------------------------*/
void ltr559_eint_func(void)
{
    struct ltr559_priv *obj = ltr559_obj;
    if (!obj)
        return;

    schedule_work(&obj->eint_work);
    if (atomic_read(&obj->trace) & CMC_TRC_EINT)
        APS_LOG("eint: als/ps intrs\n");
}
static irqreturn_t ltr559_eint_handler(int irq, void *desc)
{
    ltr559_eint_func();
    disable_irq_nosync(ltr559_obj->irq);

    return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
static void ltr559_eint_work(struct work_struct *work)
{
    struct ltr559_priv *obj = (struct ltr559_priv *)container_of(work, struct ltr559_priv, eint_work);
    int err;
    int buf = 0;
    int idx_table = 0;
    struct hwm_sensor_data sensor_data;
    int temp_noise = 0;

    APS_FUN();
    memset(&sensor_data, 0, sizeof(sensor_data));
    //Qux
    wake_lock_timeout(&alsps_wakelock, HZ);


    err = ltr559_check_intr(obj->client);
    if (err) {
        APS_ERR("check intrs: %d\n", err);
        return;
    }

    if (hwmsen_read_byte_sr(obj->client, APS_RO_PS_DATA_1, &buf)) {
        APS_ERR("reads APS_RO_PS_DATA_1 data = %d\n", buf);
        return;
    }

    if (buf & 0x80) {
        sensor_data.values[0] = 1;
        sensor_data.value_divide = 1;
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
        APS_LOG("ltr559 read ps data exception = %d \n",sensor_data.values[0]);
        err = ps_report_interrupt_data(sensor_data.values[0]);
        if(err)
            APS_ERR("call ps_report_interrupt_data fail = %d\n", err);

        //mt_eint_unmask(CUST_EINT_ALS_NUM);
        enable_irq(ltr559_obj->irq);
        return;
    }

    if ((1 << CMC_BIT_PS) & obj->pending_intr) {
        err = ltr559_read_data_ps(obj->client, &obj->ps);
        if(err)
        {
            APS_ERR("ltr559 read ps data: %d\n", err);;
        }
        //map and store data to struct hwm_sensor_data
        sensor_data.values[0] = ltr559_get_ps_value(obj, obj->ps);

        if (sensor_data.values[0] == 0) {
            if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_0,0xff)) goto error_rw;
            if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_1,0x07)) goto error_rw;

            //Gionee mali add calibration for ltr559 2012-9-28 begin
#if defined(CONFIG_GN_BSP_PS_STATIC_CALIBRATION)
            if (ps_cali.valid == 0) {
                if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,ltr559_obj->ps_thd_val_low & 0xff)) goto error_rw;
                if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,(ltr559_obj->ps_thd_val_low >> 8) & 0xff)) goto error_rw;				
            } else if (ps_cali.valid == 1) {

                if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,ltr559_obj->ps_thd_val_low & 0xff)) goto error_rw;
                if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,(ltr559_obj->ps_thd_val_low >> 8) & 0xff)) goto error_rw;				
            }
#else
            if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,ltr559_obj->ps_thd_val_low & 0xff)) goto error_rw;
            if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,(ltr559_obj->ps_thd_val_low >> 8) & 0xff)) goto error_rw;
#endif
        } else if (sensor_data.values[0] == 1) {
#if defined(CONFIG_GN_BSP_PS_STATIC_CALIBRATION)
            if (ps_cali.valid == 0) {
                if (obj->cali_noise > 20 && obj->ps < (obj->cali_noise - 20)) {
                    for (idx_table = 0; idx_table < obj->ps_cali_noise_num; idx_table++) {
                        if (obj->ps <= obj->ps_cali_noise[idx_table])
                            break;
                    }
                    if (idx_table >= obj->ps_cali_noise_num) {
                        APS_ERR("%s: the cali_offset_table is error", __func__);
                        return ;
                    }
                    obj->ps_thd_val_high = obj->ps_cali_offset_high[idx_table] + obj->ps;
                    ltr559_obj->ps_thd_val_low = ltr559_obj->ps_cali_offset_low[idx_table] + ltr559_obj->ps;
                    if (ltr559_obj->ps_thd_val_low >= PS_MAX_VALUE)
                        ltr559_obj->ps_thd_val_low = PS_MAX_VALUE;
                }
                if (ltr559_obj->ps >= 20) {
                    temp_noise = ltr559_obj->ps - 20; 
                } else {
                    temp_noise = 0;
                }
                if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_0,obj->ps_thd_val_high & 0xff)) goto error_rw;
                if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_1,(obj->ps_thd_val_high>>8) & 0xff)) goto error_rw;		

                if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,temp_noise & 0xff)) goto error_rw;
                if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,(temp_noise >> 8) & 0xff))  goto error_rw;
                APS_ERR("%s: temp_noise:%d\n",__func__, temp_noise);

            } else if (ps_cali.valid == 1) {
                //gionee lizhi 20150520 modify for CR01484103 start
                if (obj->cali_noise > 20 && obj->ps < (obj->cali_noise - 20) && obj->ps > 50) {
                    //gionee lizhi 20150520 modify for CR01484103 end
                    for (idx_table = 0; idx_table < obj->ps_cali_noise_num; idx_table++) {
                        if (obj->ps <= obj->ps_cali_noise[idx_table])
                            break;
                    }
                    if (idx_table >= obj->ps_cali_noise_num) {
                        APS_ERR("%s: the cali_offset_table is error", __func__);
                        return ;
                    }
                    obj->ps_thd_val_high = obj->ps_cali_offset_high[idx_table] + obj->ps;


                    obj->ps_thd_val_low = obj->ps_cali_offset_low[idx_table] + obj->ps;
                    if (obj->ps_thd_val_low >= PS_MAX_VALUE)
                        obj->ps_thd_val_low = PS_MAX_VALUE;
                }
                if (obj->ps >= 20) {
                    temp_noise = obj->ps - 20; 
                } else {
                    temp_noise = 0;
                }

                if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_0,ltr559_obj->ps_thd_val_high & 0xff)) goto error_rw;
                if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_1,(ltr559_obj->ps_thd_val_high>>8) & 0xff)) goto error_rw;		
                if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,temp_noise & 0xff)) goto error_rw;
                if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,(temp_noise >> 8) & 0xff))  goto error_rw;
                APS_ERR("%s: temp_noise:%d\n",__func__, temp_noise);


            }
#else
            if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_0,ltr559_obj->ps_thd_val_high & 0xff)) goto error_rw;
            if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_UP_1,(ltr559_obj->ps_thd_val_high>>8) & 0xff)) goto error_rw;		
#endif
            //	if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,0x00)) goto error_rw;
            //	if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,0x00)) goto error_rw;
        }
        sensor_data.value_divide = 1;
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
        //let up layer to know
        APS_LOG("ltr559 read ps data = %d \n",sensor_data.values[0]);
        err = ps_report_interrupt_data(sensor_data.values[0]);
        if(err)
            APS_ERR("call ps_report_interrupt_data fail = %d\n", err);
    }

    //mt_eint_unmask(CUST_EINT_ALS_NUM);
    enable_irq(ltr559_obj->irq);
    return;
error_rw:
    APS_ERR("read or write alsps device reg error\n");
}

/*----------------------------------------------------------------------------*/
int ltr559_setup_eint(struct i2c_client *client)
{
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
    /*pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
    if (IS_ERR(pins_default)) {
        ret = PTR_ERR(pins_default);
        APS_ERR("Cannot find alsps pinctrl default!\n");

    }*/

    pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
    if (IS_ERR(pins_cfg)) {
        ret = PTR_ERR(pins_cfg);
        APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");

    }
    if (ltr559_obj->irq_node) {
        of_property_read_u32_array(ltr559_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_request(ints[0], "p-sensor");
        gpio_set_debounce(ints[0], ints[1]);
        pinctrl_select_state(pinctrl, pins_cfg);
        APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

        ltr559_obj->irq = irq_of_parse_and_map(ltr559_obj->irq_node, 0);
        APS_LOG("ltr559_obj->irq = %d\n", ltr559_obj->irq);
        if (!ltr559_obj->irq) {
            APS_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        if (request_irq(ltr559_obj->irq, ltr559_eint_handler, IRQF_TRIGGER_FALLING, "ALS-eint", NULL)) {
            APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;
        }
    } else {
        APS_ERR("null irq node!!\n");
        return -EINVAL;
    }
    return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr559_init_client(struct i2c_client *client)
{
    int err=0;
    APS_FUN();	

    if ((err = ltr559_init_device(client))) {
        APS_ERR("ltr559_init_device init dev: %d\n", err);
        return err;
    }

    if ((err = ltr559_setup_eint(client))) {
        APS_ERR("setup eint: %d\n", err);
        return err;
    }
    return err;
}
/******************************************************************************
 * Sysfs attributes
 *******************************************************************************/
static ssize_t ltr559_show_config(struct device_driver *ddri, char *buf)
{
    ssize_t res;

    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }

    res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n",
            atomic_read(&ltr559_obj->i2c_retry), atomic_read(&ltr559_obj->als_debounce),
            atomic_read(&ltr559_obj->ps_mask), ltr559_obj->ps_thd_val, atomic_read(&ltr559_obj->ps_debounce));
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_store_config(struct device_driver *ddri,const char *buf, size_t count)
{
    int retry, als_deb, ps_deb, mask, thres;
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }

    if (5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb)) {
        atomic_set(&ltr559_obj->i2c_retry, retry);
        atomic_set(&ltr559_obj->als_debounce, als_deb);
        atomic_set(&ltr559_obj->ps_mask, mask);
        ltr559_obj->ps_thd_val= thres;
        atomic_set(&ltr559_obj->ps_debounce, ps_deb);
    } else {
        APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
    }
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_trace(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }

    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ltr559_obj->trace));
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }

    if (1 == sscanf(buf, "0x%x", &trace))
        atomic_set(&ltr559_obj->trace, trace);
    else
        APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_als(struct device_driver *ddri, char *buf)
{
    u8 dat = 0;
    int res = 0;
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }
    res = ltr559_read_data_als(ltr559_obj->client, &ltr559_obj->als);
    if(res)
    {
        return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
    }
    else {
        dat = ltr559_obj->als;
        return snprintf(buf, PAGE_SIZE, "0x%04X\n", dat);
    }
}

static ssize_t ltr559_show_ps(struct device_driver *ddri, char *buf)
{
    int res = 0;
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }

    res = ltr559_read_data_ps(ltr559_obj->client, &ltr559_obj->ps);
    if(res)
    {
        return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
    } else {
        return snprintf(buf, PAGE_SIZE, "%d", ltr559_obj->ps);
    }
}

static ssize_t ltr559_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }

    if (ltr559_obj->hw) {

        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n",
                ltr559_obj->hw->i2c_num, ltr559_obj->hw->power_id, ltr559_obj->hw->power_vol);

    } else {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    }


    len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ltr559_obj->als_suspend), atomic_read(&ltr559_obj->ps_suspend));

    return len;
}

static ssize_t ltr559_show_cali_noise(struct device_driver *ddri, char *buf)
{

    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }
    return snprintf(buf, PAGE_SIZE, "%d", ltr559_obj->cali_noise);
}

static ssize_t ltr559_show_proximity_low(struct device_driver *ddri, char *buf)
{
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }
    return snprintf(buf, PAGE_SIZE, "%d", ltr559_obj->ps_thd_val_low);
}

static ssize_t ltr559_show_proximity_high(struct device_driver *ddri, char *buf)
{
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }
    return snprintf(buf, PAGE_SIZE, "%d", ltr559_obj->ps_thd_val_high);
}

static ssize_t ltr559_show_calitableindex(struct device_driver *ddri, char *buf)
{

    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }
    return snprintf(buf, PAGE_SIZE, "%d", ltr559_obj->cali_index);
}

/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct ltr559_priv *obj, const char* buf, size_t count,
        u32 data[], int len)
{
    int idx = 0;
    char *cur = (char*)buf, *end = (char*)(buf+count);

    while(idx < len)
    {
        while((cur < end) && IS_SPACE(*cur))
        {
            cur++;
        }

        if (1 != sscanf(cur, "%d", &data[idx]))
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
static ssize_t ltr559_show_reg(struct device_driver *ddri, char *buf)
{
    int i = 0;
    int bufdata;
    int count  = 0;

    if(!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }

    for(i = 0;i < 31 ;i++) {
        hwmsen_read_byte_sr(ltr559_obj->client,0x80+i,&bufdata);
        count+= sprintf(buf+count,"[%x] = (%x)\n",0x80+i,bufdata);
    }

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t ltr559_store_reg(struct device_driver *ddri,const char *buf,size_t count)
{

    u32 data[2];
    if(!ltr559_obj) {
        APS_ERR("ltr559_obj is null\n");
        return 0;
    }
    /*else if(2 != sscanf(buf,"%d %d",&addr,&data))*/
    else if(2 != read_int_from_buf(ltr559_obj,buf,count,data,2)) {
        APS_ERR("invalid format:%s\n",buf);
        return 0;
    }

    hwmsen_write_byte(ltr559_obj->client,data[0],data[1]);

    return count;
}

static ssize_t ltr559_show_calitable(struct device_driver *ddri, char *buf)
{
    int i = 0;
    int count  = 0;

    if(!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }

    count+= sprintf(buf+count,"index: noise ofs_low ofs_high\n");

    for(i = 0; i < sizeof(ltr559_obj->ps_cali_noise)/sizeof(ltr559_obj->ps_cali_noise[0]); i++)
        count+= sprintf(buf+count,"%d: %d %d %d\n",i, ltr559_obj->ps_cali_noise[i], ltr559_obj->ps_cali_offset_low[i], ltr559_obj->ps_cali_offset_high[i] );
    return count;
}

static ssize_t ltr559_store_offsethightable(struct device_driver *ddri,const char *buf,size_t count)
{
    u32 data[2];
    if(!ltr559_obj) {
        APS_ERR("ltr559_obj is null\n");
        return 0;
    } else if(2 != read_int_from_buf(ltr559_obj,buf,count,data,2)) {
        APS_ERR("invalid format:%s\n",buf);
        return 0;
    }

    ltr559_obj->ps_cali_offset_high[data[0]] = data[1];

    return count;
}

static ssize_t ltr559_store_offsetlowtable(struct device_driver *ddri,const char *buf,size_t count)
{

    int data = 0;
    int index = 0;
    if(!ltr559_obj) {
        APS_ERR("ltr559_obj is null\n");
        return 0;
    }
    if (2 != sscanf(buf, "%d %d", &index, &data))
        return 0;

    ltr559_obj->ps_cali_offset_low[index] = data;
    return count;
}

static ssize_t ltr559_store_noisetable(struct device_driver *ddri,const char *buf,size_t count)
{

    int data = 0;
    int index = 0;
    if(!ltr559_obj) {
        APS_ERR("ltr559_obj is null\n");
        return 0;
    }
    if (2 != sscanf(buf, "%d %d", &index, &data))
        return 0;

    ltr559_obj->ps_cali_noise[index] = data;
    return count;
}

static ssize_t ltr559_show_alslv(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    int idx;
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }

    for(idx = 0; idx < ltr559_obj->als_level_num; idx++) {
        len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr559_obj->hw->als_level[idx]);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_store_alslv(struct device_driver *ddri,const char *buf, size_t count)
{
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    } else if (!strcmp(buf, "def")) {
        memcpy(ltr559_obj->als_level, ltr559_obj->hw->als_level, sizeof(ltr559_obj->als_level));
    } else if (ltr559_obj->als_level_num != read_int_from_buf(ltr559_obj, buf, count,
                ltr559_obj->hw->als_level, ltr559_obj->als_level_num)) {
        APS_ERR("invalid format: '%s'\n", buf);
    }

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_alsval(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    int idx;
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }

    for(idx = 0; idx < ltr559_obj->als_value_num; idx++) {
        len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr559_obj->hw->als_value[idx]);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");

    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_store_alsval(struct device_driver *ddri,const char *buf, size_t count)
{
    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    } else if (!strcmp(buf, "def")) {
        memcpy(ltr559_obj->als_value, ltr559_obj->hw->als_value, sizeof(ltr559_obj->als_value));
    } else if (ltr559_obj->als_value_num != read_int_from_buf(ltr559_obj, buf, count,
                ltr559_obj->hw->als_value, ltr559_obj->als_value_num)) {
        APS_ERR("invalid format: '%s'\n", buf);
    }

    return count;
}

static ssize_t ltr559_show_name(struct device_driver *ddri, char *buf)
{

    if (!ltr559_obj) {
        APS_ERR("ltr559_obj is null!!\n");
        return 0;
    }
    return snprintf(buf, PAGE_SIZE, "%s", "ltr559");
}

static DRIVER_ATTR(als,	 S_IWUSR | S_IRUGO, ltr559_show_als,	NULL);
static DRIVER_ATTR(pdata, S_IWUSR | S_IRUGO, ltr559_show_ps,	NULL);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, ltr559_show_trace, ltr559_store_trace);
static DRIVER_ATTR(config, S_IWUSR | S_IRUGO, ltr559_show_config,ltr559_store_config);
static DRIVER_ATTR(alslv, S_IWUSR | S_IRUGO, ltr559_show_alslv, ltr559_store_alslv);
static DRIVER_ATTR(alsval, S_IWUSR | S_IRUGO, ltr559_show_alsval, ltr559_store_alsval);
static DRIVER_ATTR(status, S_IWUSR | S_IRUGO, ltr559_show_status, NULL);
static DRIVER_ATTR(reg,	 S_IRUGO | S_IWUSR, ltr559_show_reg, ltr559_store_reg);

static DRIVER_ATTR(cali_noise,	 	S_IRUGO, 	ltr559_show_cali_noise, 	NULL);
static DRIVER_ATTR(low_threshold,	S_IRUGO, 	ltr559_show_proximity_low, 	NULL);
static DRIVER_ATTR(high_threshold,	S_IRUGO, 	ltr559_show_proximity_high, 	NULL);

static DRIVER_ATTR(calitable,		S_IRUGO, 	ltr559_show_calitable, 		NULL);
static DRIVER_ATTR(offsetlowtable,	S_IWUSR, 	NULL, 				ltr559_store_offsetlowtable);
static DRIVER_ATTR(offsethightable,	S_IWUSR, 	NULL, 				ltr559_store_offsethightable);
static DRIVER_ATTR(noisetable,		S_IWUSR, 	NULL, 				ltr559_store_noisetable);
static DRIVER_ATTR(calitableindex,	S_IRUGO, 	ltr559_show_calitableindex, 	NULL);
static DRIVER_ATTR(name,		S_IRUGO, 	ltr559_show_name, 		NULL);

static struct driver_attribute *ltr559_attr_list[] = {
    &driver_attr_als,
    &driver_attr_pdata,
    &driver_attr_trace,
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_reg,

    &driver_attr_cali_noise,
    &driver_attr_low_threshold,
    &driver_attr_high_threshold,
    &driver_attr_calitable,
    &driver_attr_offsetlowtable,
    &driver_attr_offsethightable,
    &driver_attr_noisetable,
    &driver_attr_calitableindex,
    &driver_attr_name,
};

static int ltr559_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(ltr559_attr_list)/sizeof(ltr559_attr_list[0]));
    if (driver == NULL) {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++) {
        err = driver_create_file(driver, ltr559_attr_list[idx]);
        if(err)
        {
            APS_ERR("driver_create_file (%s) = %d\n", ltr559_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int ltr559_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(ltr559_attr_list)/sizeof(ltr559_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++) {
        driver_remove_file(driver, ltr559_attr_list[idx]);
    }

    return err;
}
/******************************************************************************
 * Function Configuration
 ******************************************************************************/

static int ltr559_get_als_value(struct ltr559_priv *obj, int als)
{
    int idx;
    int invalid = 0;
    for(idx = 0; idx < obj->als_level_num; idx++) {
        if (als <= obj->hw->als_level[idx]) {
            break;
        }
    }

    if (idx >= obj->als_value_num) {
        APS_ERR("exceed range\n");
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
//gionee lizhi 20160323 add for als value begin
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
//gionee lizhi 20160323 add for als value end

        if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS) {
            APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
        }
        return obj->hw->als_value[idx];
    } else {
        if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS) {
            APS_DBG("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
        }
        return -1;
    }
}

static int ltr559_get_ps_value(struct ltr559_priv *obj, int ps)
{
    int val= -1;
    static int val_temp = 1;

#if defined(CONFIG_GN_BSP_PS_STATIC_CALIBRATION)

    if (ps>=ltr559_obj->ps_thd_val_high) {
        val = 0;
        val_temp = 0;
    } else if (ps <= ltr559_obj->ps_thd_val_low) {
        val = 1;
        val_temp = 1;
    } else {
        val = val_temp;
    }
    APS_LOG(" ps=%d, ltr559_obj->ps_thd_val_low= %d , ltr559_obj->ps_thd_val_high = %d\n", ps, ltr559_obj->ps_thd_val_low, ltr559_obj->ps_thd_val_high);
#else
    if (ps>=ltr559_obj->ps_thd_val_high) {
        val = 0;
        val_temp = 0;
    } else if (ps <= ltr559_obj->ps_thd_val_low) {
        val = 1;
        val_temp = 1;
    } else {
        val = val_temp;
    }
    APS_LOG(" ps=%d, ltr559_obj->ps_thd_val_low= %d , ltr559_obj->ps_thd_val_high = %d\n", ps, ltr559_obj->ps_thd_val_low, ltr559_obj->ps_thd_val_high);
#endif
    return val;
}

static int ltr559_dynamic_calibrate(void)
{
    int ret = 0;
    int i = 0;
    int data = 0;
    int noise = 0;
    int max = 0;
    int idx_table = 0;
    unsigned long data_total = 0;
    struct ltr559_priv *obj = ltr559_obj;

    APS_FUN(f);
    if (!obj) goto err;

    for (i = 0; i < COUNT_FOR_CALI; i++) {
        if (max++ > 10) {
            obj->ps_thd_val_high = PS_MAX_VALUE;
            obj->ps_thd_val_low = PS_MAX_VALUE;

            obj->cali_index = obj->ps_cali_noise_num-1;
            obj->cali_noise = PS_MAX_VALUE;
            goto err;
        }
        msleep(10);
        ret = ltr559_read_data_ps(obj->client,&data);
        if (ret != 0) {
            if (i>0) i--;
            continue;
        }
        data_total += data;
        APS_ERR("noise data = %d\n",data);
    }
    noise = data_total/COUNT_FOR_CALI;

    for (idx_table = 0; idx_table < obj->ps_cali_noise_num; idx_table++) {
        if (noise <= obj->ps_cali_noise[idx_table])
            break;
    }
    if (idx_table >= obj->ps_cali_noise_num) goto err;

    obj->ps_thd_val_high = obj->ps_cali_offset_high[idx_table] + noise;
    if (obj->ps_thd_val_high > PS_MAX_VALUE) obj->ps_thd_val_high = PS_MAX_VALUE;

    obj->ps_thd_val_low = obj->ps_cali_offset_low[idx_table] + noise;
    if (obj->ps_thd_val_low > PS_MAX_VALUE) obj->ps_thd_val_low = PS_MAX_VALUE;

    obj->cali_index = idx_table;
    obj->cali_noise = noise;

    if (hwmsen_write_byte(obj->client, APS_RW_PS_THRES_UP_0, obj->ps_thd_val_high & 0xff)) goto err;
    if (hwmsen_write_byte(obj->client, APS_RW_PS_THRES_UP_1, (obj->ps_thd_val_high >> 8) & 0xff)) goto err;
    //	if (hwmsen_write_byte(obj->client, APS_RW_PS_THRES_LOW_0, 0x00)) goto err;
    //	if (hwmsen_write_byte(obj->client, APS_RW_PS_THRES_LOW_1, 0x00)) goto err;

    if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_0,obj->ps_thd_val_low & 0xff)) goto err;
    if (hwmsen_write_byte(obj->client,APS_RW_PS_THRES_LOW_1,(obj->ps_thd_val_low >> 8) & 0xff)) goto err;				
    APS_ERR(" ltr559_dynamic_calibrate end:noise=%d, obj->ps_thd_val_low= %d , obj->ps_thd_val_high = %d\n", obj->cali_noise, ltr559_obj->ps_thd_val_low, ltr559_obj->ps_thd_val_high);
    return 0;
err:
    APS_ERR("ltr559_dynamic_calibrate fail!!!\n");
    return -1;
}

//Gionee mali add calibration for ltr559 2012-9-28 begin
#if defined(CONFIG_GN_BSP_PS_STATIC_CALIBRATION)
static int ltr559_WriteCalibration(struct PS_CALI_DATA_STRUCT *data_cali)
{
    if (data_cali->valid ==1){
        ps_cali.close = data_cali->close; 
        ps_cali.far_away = data_cali->far_away;
        ps_cali.valid = 1;

        hwmsen_write_byte(ltr559_obj->client, 0x90, (ps_cali.close) & 0x00ff);
        hwmsen_write_byte(ltr559_obj->client, 0x91, ((ps_cali.close)>>8) & 0x07);
        hwmsen_write_byte(ltr559_obj->client, 0x92, (ps_cali.far_away)& 0x00ff);
        hwmsen_write_byte(ltr559_obj->client, 0x93, ((ps_cali.far_away)>>8)&0x07);
    } else {
        ps_cali.valid = 0; 

        hwmsen_write_byte(ltr559_obj->client, 0x90, PS_THRES_UP_0_VALUE);
        hwmsen_write_byte(ltr559_obj->client, 0x91, PS_THRES_UP_1_VALUE);
        hwmsen_write_byte(ltr559_obj->client, 0x92, PS_THRES_LOW_0_VALUE);
        hwmsen_write_byte(ltr559_obj->client, 0x93, PS_THRES_LOW_1_VALUE);
    }
    return 0;
}

static int ltr559_read_data_for_cali(struct i2c_client *client,struct PS_CALI_DATA_STRUCT *ps_data_cali)
{

    int ret=0;
    int i=0;
    int data[COUNT];
    int data_total=0;
    int noise = 0;
    int max = 0;
    int err = 0;
    int j = 0;
    struct hwm_sensor_data sensor_data;

    if (!ltr559_obj) { 
        APS_ERR("ltr559_obj is null!!\n");
        goto report_value;
    }

    // wait for register to be stable
    for (i = 0; i < COUNT; i++) {
        // wait for ps value be stable
        if (max++ > 50) {
            ps_cali.valid = 0;
            ps_data_cali->valid = 0;
            ps_data_cali->close = 0;
            ps_data_cali->far_away = 0;

            goto report_value;
        }

        msleep(10);
        ret=ltr559_read_data_ps(ltr559_obj->client,&data[i]);

        APS_ERR("the register of APS_RW_PS_CONTR data[i] is %x \n", data[i]);

        if (ret < 0) {
            i--;
            continue;
        }

        data_total+= data[i];
        if (data[i] == 0) {
            j++;
        }
    }

    if (data_total == 0) {
        ps_data_cali->close = NOISE_HIGH;
        ps_data_cali->far_away = NOISE_LOW;
        ps_data_cali->valid = 1;

        ps_cali.close = NOISE_HIGH;
        ps_cali.far_away = NOISE_LOW;
        ps_cali.valid = 1;
    } else {
        noise=data_total/(COUNT-j);
        if (noise > NOISE_MAX) {
            ps_cali.far_away = 0;
            ps_cali.close = 0;
            ps_cali.valid = 0;

            ps_data_cali->valid = 0;
            ps_data_cali->close = 0;
            ps_data_cali->far_away = 0;
        } else {
            ps_data_cali->close = noise + NOISE_HIGH; 
            ps_data_cali->far_away = noise + NOISE_LOW;
            ps_data_cali->valid = 1;

            ps_cali.close = noise + NOISE_HIGH;
            ps_cali.far_away = noise + NOISE_LOW;
            ps_cali.valid = 1;
        }

    }

    if (ps_cali.valid ==1) {
        hwmsen_write_byte(ltr559_obj->client, 0x90, (ps_cali.close) & 0x00ff);
        hwmsen_write_byte(ltr559_obj->client, 0x91, ((ps_cali.close)>>8) & 0x07);
        hwmsen_write_byte(ltr559_obj->client, 0x92, (ps_cali.far_away)& 0x00ff);
        hwmsen_write_byte(ltr559_obj->client, 0x93, ((ps_cali.far_away)>>8)&0x07);
    } else {
        hwmsen_write_byte(ltr559_obj->client, 0x90, PS_THRES_UP_0_VALUE);
        hwmsen_write_byte(ltr559_obj->client, 0x91, PS_THRES_UP_1_VALUE);
        hwmsen_write_byte(ltr559_obj->client, 0x92, PS_THRES_LOW_0_VALUE);
        hwmsen_write_byte(ltr559_obj->client, 0x93, PS_THRES_LOW_1_VALUE);
    }

report_value:
    sensor_data.values[0] = 1;
    sensor_data.value_divide = 1;
    sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
    err = ps_report_interrupt_data(sensor_data.values[0]);
    if(err)
    {
        APS_ERR("call ps_report_interrupt_data fail = %d\n", err);
    }

    return 0;
}
#endif
//Gionee mali add calibration for ltr559 2012-9-28 end

/******************************************************************************
 * Function Configuration
 ******************************************************************************/
static int ltr559_open(struct inode *inode, struct file *file)
{
    file->private_data = ltr559_i2c_client;

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int ltr559_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
/*----------------------------------------------------------------------------*/
static long ltr559_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct ltr559_priv *obj = i2c_get_clientdata(client);
    int err = 0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
#if defined(CONFIG_GN_BSP_PS_STATIC_CALIBRATION)
    int i = 0;
    int ps_noise[8];
    struct PS_CALI_DATA_STRUCT ps_cali_temp;
#endif
    APS_LOG("ltr559 cmd = %d \n", cmd);
    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if (copy_from_user(&enable, ptr, sizeof(enable))) {
                err = -EFAULT;
                goto err_out;
            }

            if (enable) {
                err = ltr559_enable_ps(obj->client, true);
                if(err)
                {
                    APS_ERR("enable ps fail: %d\n", err);
                    goto err_out;
                }
                set_bit(CMC_BIT_PS, &obj->enable);
            } else {
                err = ltr559_enable_ps(obj->client, false);
                if(err)
                {
                    APS_ERR("disable ps fail: %d\n", err);
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
            err = ltr559_read_data_ps(obj->client, &obj->ps);
            if(err)
            {
                goto err_out;
            }
            dat = ltr559_get_ps_value(obj, obj->ps);
            if (copy_to_user(ptr, &dat, sizeof(dat))) {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_RAW_DATA:
            err = ltr559_read_data_ps(obj->client, &obj->ps);
            if(err)
            {
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
                err = ltr559_enable_als(obj->client, true);
                if(err)
                {
                    APS_ERR("enable als fail: %d\n", err);
                    goto err_out;
                }
                set_bit(CMC_BIT_ALS, &obj->enable);
            } else {
                err = ltr559_enable_als(obj->client, false);
                if(err)
                {
                    APS_ERR("disable als fail: %d\n", err);
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
            err = ltr559_read_data_als(obj->client, &obj->als);
            if(err)
            {
                goto err_out;
            }

            dat = obj->als;
            if (copy_to_user(ptr, &dat, sizeof(dat))) {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_RAW_DATA:
            err = ltr559_read_data_als(obj->client, &obj->als);
            if(err)
            {
                goto err_out;
            }

            dat = obj->als;	// & 0x3f;//modified by mayq
            if (copy_to_user(ptr, &dat, sizeof(dat))) {
                err = -EFAULT;
                goto err_out;
            }
            break;

            //Gionee mali add calibration for ltr559 2012-9-28 begin
#if defined(CONFIG_GN_BSP_PS_STATIC_CALIBRATION)
        case ALSPS_SET_PS_CALI:
            /*case ALSPS_SET_PS_CALI:*/
            if (ptr == NULL) {
                APS_LOG("ptr == NULL\n");
                err = -EINVAL;
                break;    
            }

            if (copy_from_user(&ps_cali_temp, ptr, sizeof(ps_cali_temp))) {
                APS_LOG("copy_from_user\n");
                err = -EFAULT;
                break;    
            }

            ltr559_WriteCalibration(&ps_cali_temp);
            APS_LOG("111111  ALSPS_SET_PS_CALI %d,%d,%d\t",ps_cali_temp.close, ps_cali_temp.far_away,ps_cali_temp.valid);

            break;

        case ALSPS_GET_PS_CALI:

            err = ltr559_enable_ps(obj->client, true);
            if(err)
            {
                APS_ERR("ltr559 ioctl enable ps fail: %d\n", err);
            }

            msleep(50);

            err = ltr559_read_data_for_cali(obj->client, &ps_cali_temp);
            if(err){
                goto err_out;
            }

            err = ltr559_enable_ps(client, false);
            if(err)
            {
                APS_ERR("ltr559 ioctl disable ps fail: %d\n", err);
            }

            if (copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp))) {
                err = -EFAULT;
                goto err_out;
            }              
            APS_LOG("ltr559 ALSPS_GET_PS_CALI %d,%d,%d\t",ps_cali_temp.close, ps_cali_temp.far_away,ps_cali_temp.valid);
            break;

        case ALSPS_GET_PS_NOISE:
            err = ltr559_enable_ps(client, true);
            if(err)
            {
                APS_ERR("ltr559 ioctl disable ps fail: %d\n", err);
            }

            for (i = 0; i < 8; i++) {    
                err = ltr559_read_data_ps(obj->client, &ps_noise[i]);
                if(err)
                {    
                    i--; 
                    continue;
                }    
                APS_LOG("ltr559 get ps_noise = %x\n",ps_noise[i]);
                msleep(10);
            }    

            err = ltr559_enable_ps(client, false);
            if(err)
            {
                APS_ERR("ltr559 ioctl disable ps fail: %d\n", err);
            }


            if (copy_to_user(ptr, ps_noise, sizeof(ps_noise))) {
                err = -EFAULT;
                goto err_out;
            }
            break;

#endif
            //Gionee mali add calibration for ltr559 2012-9-28 end
        default:
            APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;
}
/*----------------------------------------------------------------------------*/
static struct file_operations ltr559_fops = {
    .owner = THIS_MODULE,
    .open = ltr559_open,
    .release = ltr559_release,
    .unlocked_ioctl = ltr559_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice ltr559_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &ltr559_fops,
};
/*----------------------------------------------------------------------------*/
static int ltr559_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct ltr559_priv *obj = i2c_get_clientdata(client);
    int err;
    APS_FUN();

    if (msg.event == PM_EVENT_SUSPEND)
    {
        if (!obj)
        {
            APS_ERR("null pointer!!\n");
            return -EINVAL;
        }
        atomic_set(&obj->als_suspend, 1);
        err = ltr559_enable_als(client, false);
        if(err)
        {
            APS_ERR("disable als: %d\n", err);
            return err;
        }

        //Gionee yanggy 2012-07-25 add for psensor interrupt mode begin
        /*==========this part is necessary when use polling mode==========*/
#ifndef CONFIG_GN_BSP_ALSPS_INTERRUPT_MODE
        atomic_set(&obj->ps_suspend, 1);
        err = ltr559_enable_ps(client, false);
        if(err)
        {
            APS_ERR("disable ps: %d\n", err);
            return err;
        }
#endif
        //Gionee yanggy 2012-07-25 add for psensor interrupt mode end
    }
    return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr559_i2c_resume(struct i2c_client *client)
{
    struct ltr559_priv *obj = i2c_get_clientdata(client);
    int err;
    APS_FUN();

    if (!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    atomic_set(&obj->als_suspend, 0);
    if (test_bit(CMC_BIT_ALS, &obj->enable))
    {
        err = ltr559_enable_als(client, true);
        if(err)
        {
            APS_ERR("enable als fail: %d\n", err);
        }
    }

    //Gionee yanggy 2012-07-25 add for psensor interrupt mode begin
    /*==========this part is necessary when use polling mode======*/
#ifndef CONFIG_GN_BSP_ALSPS_INTERRUPT_MODE
    atomic_set(&obj->ps_suspend, 0);
    if (test_bit(CMC_BIT_PS, &obj->enable))
    {
        err = ltr559_enable_ps(client, true);
        if(err)
        {
            APS_ERR("enable ps fail: %d\n", err);
        }
    }
#endif
    //Gionee yanggy 2012-07-25 add for psensor interrupt mode

    return 0;
}
#if 0
/*----------------------------------------------------------------------------*/
static void ltr559_early_suspend(struct early_suspend *h)
{
    /*early_suspend is only applied for ALS*/
    struct ltr559_priv *obj = container_of(h, struct ltr559_priv, early_drv);
    int err;
    APS_FUN();

    if (!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
    atomic_set(&obj->als_suspend, 1);
    if (err = ltr559_enable_als(obj->client, false))
    {
        APS_ERR("disable als fail: %d\n", err);
    }
}
/*----------------------------------------------------------------------------*/
static void ltr559_late_resume(struct early_suspend *h)
{
    /*early_suspend is only applied for ALS*/
    struct ltr559_priv *obj = container_of(h, struct ltr559_priv, early_drv);
    APS_FUN();

    if (!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 0);
    if (test_bit(CMC_BIT_ALS, &obj->enable))
    {
        if (ltr559_enable_als(obj->client, true))
        {
            APS_ERR("enable als fail: %d\n", err);
        }
    }

}
#endif
int ltr559_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    struct hwm_sensor_data* sensor_data;
    struct ltr559_priv *obj = (struct ltr559_priv *)self;

    //APS_FUN(f);
    //APS_LOG("ltr559_ps_operate command(1 delay,2 enable,3 getdata:):%d\n",command);
    switch (command)
    {
        case SENSOR_DELAY:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            // Do nothing
            break;

        case SENSOR_ENABLE:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                APS_LOG("SENSOR_ENABLE:%d\n",value);
                if (value)
                {
                    err = ltr559_enable_ps(obj->client, true);
                    if(err)
                    {
                        APS_ERR("enable ps fail: %d\n", err);
                        return -1;
                    }
                    set_bit(CMC_BIT_PS, &obj->enable);
                }
                else
                {
                    err = ltr559_enable_ps(obj->client, false);
                    if(err)
                    {
                        APS_ERR("disable ps fail: %d\n", err);
                        return -1;
                    }
                    clear_bit(CMC_BIT_PS, &obj->enable);
                }
            }
            break;

        case SENSOR_GET_DATA:
            if ((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data))) {
                APS_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            } else {
                if (obj->ps_enable != 1) {
                    err = -EINVAL;
                    return err;
                }

                sensor_data = (struct hwm_sensor_data *)buff_out;
                err = ltr559_read_data_ps(obj->client, &obj->ps);
                if(err)
                {
                    APS_ERR("SENSOR_GET_DATA^^^^^^^^^^!\n");
                    err = -1;
                    break;
                } else { 
                    while(-1 == ltr559_get_ps_value(obj, obj->ps)) {
                        ltr559_read_data_ps(obj->client, &obj->ps);
                        msleep(50);
                    }
                    sensor_data->values[0] = ltr559_get_ps_value(obj, obj->ps);
                    sensor_data->value_divide = 1;
                    sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                    APS_LOG("fwq get ps data =%d\n",sensor_data->values[0]);
                }
            }
            break;
        default:
            APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}
int ltr559_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    struct hwm_sensor_data* sensor_data;
    struct ltr559_priv *obj = (struct ltr559_priv *)self;

    //	APS_LOG("ltr559_als_operate command(1 delay,2 enable,3 getdata:):%d\n",command);
    switch (command)
    {
        case SENSOR_DELAY:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            // Do nothing
            break;

        case SENSOR_ENABLE:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                if (value)
                {
                    err = ltr559_enable_als(obj->client, true);
                    if(err)
                    {
                        APS_ERR("enable als fail: %d\n", err);
                        return -1;
                    }
                    set_bit(CMC_BIT_ALS, &obj->enable);
                }
                else
                {
                    err = ltr559_enable_als(obj->client, false);
                    if(err)
                    {
                        APS_ERR("disable als fail: %d\n", err);
                        return -1;
                    }
                    clear_bit(CMC_BIT_ALS, &obj->enable);
                }
            }
            break;

        case SENSOR_GET_DATA:
            if ((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
            {
                APS_ERR("ltr559_als_operate get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (struct hwm_sensor_data *)buff_out;

                err = ltr559_read_data_als(obj->client, &obj->als);
                if(err)
                {
                    err = -1;;
                }
                else
                {
                    while(-1 == ltr559_get_als_value(obj, obj->als))
                    {
                        ltr559_read_data_als(obj->client, &obj->als);
                        msleep(50);
                    }
                    sensor_data->values[0] = ltr559_get_als_value(obj, obj->als);
                    sensor_data->value_divide = 1;
                    sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                    //APS_LOG("ltr559_als_operate get als data =%d\n",sensor_data->values[0]);
                }
            }
            break;
        default:
            APS_ERR("light sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}


/*----------------------------------------------------------------------------*/
//static int ltr559_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
//{
//    strcpy(info->type, LTR559_DEV_NAME);
  //  return 0;
//}

static int als_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true                                                                                        
    return 0;
}

//if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
    int err = 0;
    struct ltr559_priv *obj = NULL;

    if(!ltr559_obj)
    {
        APS_ERR("ltr559_obj is null!!\n");
        return -1;
    }
    obj = ltr559_obj;
    if (en)
    {
        err = ltr559_enable_als(obj->client, true);
        if(err)
        {
            APS_ERR("enable als fail: %d\n", err);
            return -1;
        }
        set_bit(CMC_BIT_ALS, &obj->enable);                                                                                   
    }
    else
    {
        err = ltr559_enable_als(obj->client, false);
        if(err)
        {
            APS_ERR("disable als fail: %d\n", err);
            return -1;
        }
        clear_bit(CMC_BIT_ALS, &obj->enable);
    }
    return 0;
}

static int als_set_delay(u64 ns)
{
    return 0;
}

static int als_get_data(int* value, int* status)
{
    int err = 0;
    struct ltr559_priv *obj = NULL;

    if(!ltr559_obj)
    {
        APS_ERR("ltr559_obj is null!!\n");
        return -1;
    }
    obj = ltr559_obj;
    if((err = ltr559_read_data_als(obj->client, &obj->als)))
    {
        err = -1;
    }
    else
    {
        while(-1 == ltr559_get_als_value(obj, obj->als))                                                          
        {   
            ltr559_read_data_als(obj->client, &obj->als);                                                         
            msleep(50);                                                                                           
        }
        *value = ltr559_get_als_value(obj, obj->als);
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

    return err;
}

static int ps_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
    int err = 0;
    struct ltr559_priv *obj = NULL;

    if(!ltr559_obj)
    {
        APS_ERR("ltr559_obj is null!!\n");
        return -1;
    }
    obj = ltr559_obj;

    APS_LOG("ltr559_obj ps enable value = %d\n", en);

    if (en)
    {   
        err = ltr559_enable_ps(obj->client, true);
        if(err)
        {
            APS_ERR("enable ps fail: %d\n", err);
            return -1;
        }
        set_bit(CMC_BIT_PS, &obj->enable);
    }
    else
    {
        err = ltr559_enable_ps(obj->client, false);
        if(err)
        {
            APS_ERR("disable ps fail: %d\n", err);
            return -1;
        }
        clear_bit(CMC_BIT_PS, &obj->enable);
    }

    return 0;
}

static int ps_set_delay(u64 ns)
{
    return 0;
}

static int ps_get_data(int* value, int* status)
{
    int err = 0;
    struct ltr559_priv *obj = NULL;

    if(!ltr559_obj)
    {
        APS_ERR("ltr559_obj is null!!\n");
        return -1;
    }
    obj = ltr559_obj;
    err = ltr559_read_data_ps(obj->client, &obj->ps);
    if(err)
    {                                                                                      
        err = -1;
    }
    else
    { 
        while(-1 == ltr559_get_ps_value(obj, obj->ps)) {
            ltr559_read_data_ps(obj->client, &obj->ps);
            msleep(50);
        }
        *value = ltr559_get_ps_value(obj, obj->ps);
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

    return err;
}


/*----------------------------------------------------------------------------*/
static int ltr559_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ltr559_priv *obj;
    int err = 0;
    struct als_control_path als_ctl={0};
    struct als_data_path als_data={0};
    struct ps_control_path ps_ctl={0};
    struct ps_data_path ps_data={0};
	// Gionee lizhi 20150205 add for CR01444969 begin begin
#ifdef CONFIG_GN_BSP_MTK_DEVICE_CHECK
    struct gn_device_info gn_mydev_info_alspssensor;
#endif
    // Gionee lizhi 20150205 add for CR01444969 beginend
    APS_FUN();

    if (!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }
    memset(obj, 0, sizeof(*obj));
    ltr559_obj = obj;
    obj->hw = hw;
    ltr559_obj->polling = obj->hw->polling_mode;
    wake_lock_init(&alsps_wakelock, WAKE_LOCK_SUSPEND, "alsps_wake_lock");

    INIT_WORK(&obj->eint_work, ltr559_eint_work);
    obj->client = client;
    i2c_set_clientdata(client, obj);
    atomic_set(&obj->als_debounce, 300);
    atomic_set(&obj->als_deb_on, 0);
    atomic_set(&obj->als_deb_end, 0);
    atomic_set(&obj->ps_debounce, 200);
    atomic_set(&obj->als_suspend, 0);

    obj->ps_enable = 0;
    obj->als_enable = 0;
    obj->enable = 0;
    obj->pending_intr = 0;
    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));


    obj->ps_cali_noise_num =  sizeof(obj->hw->ps_cali_noise)/sizeof(obj->hw->ps_cali_noise[0]);
    obj->ps_cali_offset_high_num =  sizeof(obj->hw->ps_threshold_high)/sizeof(obj->hw->ps_threshold_high[0]);
    obj->ps_cali_offset_low_num =  sizeof(obj->hw->ps_threshold_low)/sizeof(obj->hw->ps_threshold_low[0]);

    BUG_ON(sizeof(obj->ps_cali_noise) != sizeof(obj->hw->ps_cali_noise));
    memcpy(obj->ps_cali_noise, obj->hw->ps_cali_noise, sizeof(obj->ps_cali_noise));

    BUG_ON(sizeof(obj->ps_cali_offset_high) != sizeof(obj->hw->ps_threshold_high));
    memcpy(obj->ps_cali_offset_high, obj->hw->ps_threshold_high, sizeof(obj->ps_cali_offset_high));

    BUG_ON(sizeof(obj->ps_cali_offset_low) != sizeof(obj->hw->ps_threshold_low));
    memcpy(obj->ps_cali_offset_low, obj->hw->ps_threshold_low, sizeof(obj->ps_cali_offset_low));

    atomic_set(&obj->i2c_retry, 3);

    ltr559_i2c_client = client;
    obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");
    err = ltr559_init_client(client);
    if(err)
    {
        goto exit_init_failed;
    }

    err = misc_register(&ltr559_device);
    if(err)
    {
        APS_ERR("ltr559_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    err = ltr559_create_attr(&(ltr559_init_info.platform_diver_addr->driver));
    if(err)
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    als_ctl.is_use_common_factory =false;                                                                                             
    ps_ctl.is_use_common_factory = false;

    als_ctl.open_report_data= als_open_report_data;
    als_ctl.enable_nodata = als_enable_nodata;
    als_ctl.set_delay  = als_set_delay;
    als_ctl.is_report_input_direct = false;
    als_ctl.is_support_batch = false;

    err = als_register_control_path(&als_ctl);
    if(err)
    {
        APS_ERR("register fail = %d\n", err);
        goto exit_create_attr_failed;
    }

    als_data.get_data = als_get_data;
    als_data.vender_div = 100;
    err = als_register_data_path(&als_data);    
    if(err)
    {
        APS_ERR("tregister fail = %d\n", err);
        goto exit_create_attr_failed;
    }

    ps_ctl.open_report_data= ps_open_report_data;
    ps_ctl.enable_nodata = ps_enable_nodata;
    ps_ctl.set_delay  = ps_set_delay;
    ps_ctl.is_report_input_direct = true;
    ps_ctl.is_support_batch = false;

    err = ps_register_control_path(&ps_ctl);
    if(err)
    {
        APS_ERR("register fail = %d\n", err);
        goto exit_create_attr_failed;
    }

    ps_data.get_data = ps_get_data;
    ps_data.vender_div = 100;
    err = ps_register_data_path(&ps_data);  
    if(err)
    {
        APS_ERR("tregister fail = %d\n", err);
        goto exit_create_attr_failed;
    }

    err = batch_register_support_info(ID_LIGHT,als_ctl.is_support_batch, 100, 0);
    if(err)
    {
        APS_ERR("register light batch support err = %d\n", err);
    }

    err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
    if(err)
    {
        APS_ERR("register proximity batch support err = %d\n", err);
    }
    // Gionee lizhi 20150205 add for CR01444969 begin begin
#ifdef CONFIG_GN_BSP_MTK_DEVICE_CHECK
    memset(&gn_mydev_info_alspssensor, 0, sizeof(struct gn_device_info));
    gn_mydev_info_alspssensor.gn_dev_type = GN_DEVICE_TYPE_LIGHT;
    sprintf(gn_mydev_info_alspssensor.name,"ltr559");
    gn_set_device_info(gn_mydev_info_alspssensor);
#endif
    // Gionee lizhi 20150205 add for CR01444969 beginend


    APS_LOG("%s: OK\n", __func__);
    ltr559_init_flag = 0;
    return 0;

exit_create_attr_failed:
    misc_deregister(&ltr559_device);
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(client);
    kfree(obj);
exit:
    wake_lock_destroy(&alsps_wakelock); //Qux
    ltr559_i2c_client = NULL;
    APS_ERR("%s: err = %d\n", __func__, err);

    ltr559_init_flag = -1;
    return err;
}
/*----------------------------------------------------------------------------*/
static int ltr559_i2c_remove(struct i2c_client *client)
{
    int err;

    err = ltr559_delete_attr(&(ltr559_init_info.platform_diver_addr->driver));
    if(err)
    {
        APS_ERR("ltr559_delete_attr fail: %d\n", err);
    }
    err = misc_deregister(&ltr559_device);
    if(err)
    {
        APS_ERR("misc_deregister fail: %d\n", err);
    }
    ltr559_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}
#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
    {.compatible = "mediatek,alsps"}, 
    {},
};
#endif
static struct i2c_driver ltr559_i2c_driver = {
    .probe		= ltr559_i2c_probe,
    .remove		= ltr559_i2c_remove,
 //   .detect		= ltr559_i2c_detect,
    .suspend	= ltr559_i2c_suspend,
    .resume		= ltr559_i2c_resume,
    .id_table	= ltr559_i2c_id,
    .driver = {
        //  .owner	= THIS_MODULE,
        .name	= LTR559_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = alsps_of_match,
#endif
    },
};

static int ltr559_local_init(void)
{
    ltr559_power(hw, 1);
    if (i2c_add_driver(&ltr559_i2c_driver)) {
        APS_ERR("add driver error\n");
        return -1;
    }
    if (-1 == ltr559_init_flag)
        return -1;

    return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr559_remove(void)
{
    ltr559_power(hw, 0);
    i2c_del_driver(&ltr559_i2c_driver);
    return 0;
}

static int __init ltr559_init(void)
{
//    const char *name = "mediatek,ltr559";
    APS_FUN();
  //  hw = get_alsps_dts_func(name, hw);
    if (!hw)
        APS_ERR("get dts info fail\n");
    alsps_driver_add(&ltr559_init_info);
    return 0;
}

static void __exit ltr559_exit(void)
{
    APS_FUN();
    wake_lock_destroy(&alsps_wakelock); //Qux
}

module_init(ltr559_init);
module_exit(ltr559_exit);

MODULE_AUTHOR("lizhi lizhi@gionee.com");
MODULE_DESCRIPTION("LTR559 light sensor & p sensor driver");
MODULE_LICENSE("GPL");
