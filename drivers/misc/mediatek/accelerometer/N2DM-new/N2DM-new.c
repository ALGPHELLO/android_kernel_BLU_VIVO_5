/* drivers/i2c/chips/n2dm.c - N2DM motion sensor driver
 *
 *
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
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/fb.h>

#include "N2DM-new.h"
#include <accel.h>
#include <cust_acc.h>
#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <hwmsen_helper.h>
/* Gionee BSP1 lizhi 20150713 add for motion-detect begin */
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
#include <linux/of.h>
#include <linux/of_address.h>                                                                                                                                
#include <linux/of_irq.h>
#include <linux/gpio.h>
#endif
/* Gionee BSP1 lizhi 20150713 add for motion-detect begin */

// Gionee lizhi 20150205 add for CR01444969 begin
#ifdef CONFIG_GN_BSP_MTK_DEVICE_CHECK
#include <linux/gn_device_check.h>
extern int gn_set_device_info(struct gn_device_info gn_dev_info);
#endif
// Gionee lizhi 20150205 add for CR01444969 begin end

#define DEBUG 1
/*----------------------------------------------------------------------------*/
#define CONFIG_N2DM_LOWPASS   /*apply low pass filter on output*/       
/*----------------------------------------------------------------------------*/
#define N2DM_AXIS_X          0
#define N2DM_AXIS_Y          1
#define N2DM_AXIS_Z          2
#define N2DM_AXES_NUM        3
#define N2DM_DATA_LEN        6
#define N2DM_DEV_NAME        "N2DM"
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id n2dm_i2c_id[] = {{N2DM_DEV_NAME,0},{}};
/*the adapter id will be available in customization*/

static int n2dm_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int n2dm_i2c_remove(struct i2c_client *client);
static int n2dm_suspend(struct i2c_client *client, pm_message_t msg);                                                                                      
static int n2dm_resume(struct i2c_client *client);

static int  n2dm_local_init(void);
static int  n2dm_remove(void);
static int n2dm_init_flag =-1; // 0<==>OK -1 <==> fail

static struct acc_init_info n2dm_init_info = {
    .name = "n2dm",
    .init = n2dm_local_init,
    .uninit = n2dm_remove,
};
/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
    ADX_TRC_EINT    = 0x20,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][N2DM_AXES_NUM];
    int sum[N2DM_AXES_NUM];
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
struct n2dm_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;

    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t				filter;
    s16                     cali_sw[N2DM_AXES_NUM+1];

    /*data*/
    s8                      offset[N2DM_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[N2DM_AXES_NUM+1];

    /* Gionee BSP1 lizhi 20150713 add for motion-detect begin */
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
    struct work_struct      n2dm_eint_work;
    bool                    pickup_enable;
	struct device_node *irq_node;
    int     irq;
#endif
    bool                    gsensor_enable;
    /* Gionee BSP1 lizhi 20150713 add for motion-detect end */

#if defined(CONFIG_N2DM_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif
	struct notifier_block fb_notifier; 
};
/*----------------------------------------------------------------------------*/
static const struct of_device_id accel_of_match[] = {
    {.compatible = "mediatek,gsensor"},
    {},
};

static struct i2c_driver n2dm_i2c_driver = {
    .driver = {
        //        .owner          = THIS_MODULE,
        .name           = N2DM_DEV_NAME,
        .of_match_table = accel_of_match,
    },
    .probe      		= n2dm_i2c_probe,
    .remove    			= n2dm_i2c_remove,
    .suspend            = n2dm_suspend,
    .resume             = n2dm_resume,
    .id_table = n2dm_i2c_id,
    //	.address_data = &n2dm_addr_data,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *n2dm_i2c_client = NULL;
static struct n2dm_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;
static struct GSENSOR_VECTOR3D gsensor_gain, gsensor_offset;
static DEFINE_MUTEX(n2dm_i2c_mutex);
static DEFINE_MUTEX(n2dm_op_mutex); 
//static char selftestRes[10] = {0};

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_INFO GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;
static struct data_resolution n2dm_data_resolution[] = {
    /* combination by {FULL_RES,RANGE}*/
    {{ 1, 0}, 1024},   // dataformat +/-2g  in 12-bit resolution;  { 1, 0} = 1.0 = (2*2*1000)/(2^12);  1024 = (2^12)/(2*2) 
    {{ 1, 9}, 512},   // dataformat +/-4g  in 12-bit resolution;  { 1, 9} = 1.9 = (2*4*1000)/(2^12);  512 = (2^12)/(2*4) 
    {{ 3, 9}, 256},   // dataformat +/-8g  in 12-bit resolution;  { 1, 0} = 1.0 = (2*8*1000)/(2^12);  1024 = (2^12)/(2*8) 
};
/*----------------------------------------------------------------------------*/
static struct data_resolution n2dm_offset_resolution = {{15, 6}, 64};

static int n2dm_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
    int err;
    struct i2c_msg msgs[2]={{0},{0}};

    mutex_lock(&n2dm_i2c_mutex);

    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len =1;
    msgs[0].buf = &beg;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len =len;
    msgs[1].buf = data;

    if (!client)
    {
        mutex_unlock(&n2dm_i2c_mutex);
        return -EINVAL;
    }
    else if (len > C_I2C_FIFO_SIZE) 
    {
        GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&n2dm_i2c_mutex);
        return -EINVAL;
    }
    err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
    //GSE_LOG(" lis_i2c_read_block return value  %d\n", err);
    if (err < 0) 
    {
        GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, err);
        err = -EIO;
    } 
    else 
    {
        err = 0;
    }
    mutex_unlock(&n2dm_i2c_mutex);
    return err; //if success will return 0

}
#if 0
static int n2dm_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int err, idx, num;
    char buf[C_I2C_FIFO_SIZE];
    err =0;
    mutex_lock(&n2dm_i2c_mutex);
    if (!client)
    {
        mutex_unlock(&n2dm_i2c_mutex);
        return -EINVAL;
    }
    else if (len >= C_I2C_FIFO_SIZE) 
    {        
        GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&n2dm_i2c_mutex);
        return -EINVAL;
    }    

    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
    {
        buf[num++] = data[idx];
    }

    err = i2c_master_send(client, buf, num);
    if (err < 0)
    {
        GSE_ERR("send command error!!\n");
        mutex_unlock(&n2dm_i2c_mutex);
        return -EFAULT;
    } 
    mutex_unlock(&n2dm_i2c_mutex);
    return err; //if success will return transfer lenth
}
#endif
static int n2dm_i2c_read_byte(struct i2c_client *client, u8 addr, u8 *data)
{
    u8 beg = addr;
    int err;   

    struct i2c_msg msgs[2] = {
        {
            .addr = client->addr,   .flags = 0,
            .len = 1,   .buf = &beg
        },
        {
            .addr = client->addr,   .flags = I2C_M_RD,
            .len = 1,   .buf = data,
        }
    };

    if (!client)
        return -EINVAL;

    err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
    if (err != 2) {
        GSE_ERR("i2c_transfer error: (%d %p) %d\n", addr, data, err);
        err = -EIO;
    }

    err = 0;

    return err;
}
static int n2dm_i2c_write_byte(struct i2c_client *client, u8 addr, u8 data)
{
    u8 buf[] = {addr, data};
    int ret = 0;

    ret = i2c_master_send(client, (const char *)buf, sizeof(buf));
    if (ret < 0) {
        GSE_ERR("send command error!!\n");
        return -EFAULT;
    }
    return 0;
}
static void dumpReg(struct i2c_client *client)
{
    int i=0;
    u8 addr = 0x20;
    u8 regdata=0;
    for(i=0; i<3 ; i++)
    {
        //dump all
        n2dm_i2c_read_byte(client,addr,&regdata);
        GSE_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
        addr++;
    }
}

/*--------------------ADXL power control function----------------------------------*/
static void N2DM_power(struct acc_hw *hw, unsigned int on) 
{
}
/*----------------------------------------------------------------------------*/
static int N2DM_SetDataResolution(struct n2dm_i2c_data *obj)
{
    int err = 0;
    u8  dat, reso;

    err = n2dm_i2c_read_byte(obj->client, N2DM_REG_CTL_REG4, &dat);
    if(err)
    {
        GSE_ERR("write data format fail!!\n");
        return err;
    }

    /*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
    reso  = (dat & 0x30)>>4;
    if(reso >= 0x3)
        reso = 0x2;


    if(reso < sizeof(n2dm_data_resolution)/sizeof(n2dm_data_resolution[0]))
    {        
        obj->reso = &n2dm_data_resolution[reso];
        return 0;
    }
    else
    {
        return -EINVAL;
    }
}
/*----------------------------------------------------------------------------*/
static int N2DM_ReadData(struct i2c_client *client, s16 data[N2DM_AXES_NUM])
{
    struct n2dm_i2c_data *priv = i2c_get_clientdata(client);        
    //	u8 addr = N2DM_REG_DATAX0;
    u8 buf[N2DM_DATA_LEN] = {0};
    int err = 0;

    if(NULL == client)
    {
        err = -EINVAL;
    }

    else
    {
        if(n2dm_i2c_read_block(client, N2DM_REG_OUT_X, buf, 0x01))
        {
            GSE_ERR("read  G sensor data register err!\n");
            return -1;
        }
        if(n2dm_i2c_read_block(client, N2DM_REG_OUT_X+1, &buf[1], 0x01))
        {
            GSE_ERR("read  G sensor data register err!\n");
            return -1;
        }

        data[N2DM_AXIS_X] = (s16)((buf[0]+(buf[1]<<8))>>4);
        if(n2dm_i2c_read_block(client, N2DM_REG_OUT_Y, &buf[2], 0x01))
        {
            GSE_ERR("read  G sensor data register err!\n");
            return -1;
        }
        if(n2dm_i2c_read_block(client, N2DM_REG_OUT_Y+1, &buf[3], 0x01))
        {
            GSE_ERR("read  G sensor data register err!\n");
            return -1;
        }

        data[N2DM_AXIS_Y] =  (s16)((s16)(buf[2] +( buf[3]<<8))>>4);

        if(n2dm_i2c_read_block(client, N2DM_REG_OUT_Z, &buf[4], 0x01))
        {
            GSE_ERR("read  G sensor data register err!\n");
            return -1;
        }

        if(n2dm_i2c_read_block(client, N2DM_REG_OUT_Z+1, &buf[5], 0x01))
        {
            GSE_ERR("read  G sensor data register err!\n");
            return -1;
        }

        data[N2DM_AXIS_Z] =(s16)((buf[4]+(buf[5]<<8))>>4);

        //GSE_LOG("[%08X %08X %08X %08x %08x %08x]\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);


        data[N2DM_AXIS_X] &= 0xfff;
        data[N2DM_AXIS_Y] &= 0xfff;
        data[N2DM_AXIS_Z] &= 0xfff;


        if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
        {
            GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[N2DM_AXIS_X], data[N2DM_AXIS_Y], data[N2DM_AXIS_Z],
                    data[N2DM_AXIS_X], data[N2DM_AXIS_Y], data[N2DM_AXIS_Z]);
        }

        if(data[N2DM_AXIS_X]&0x800)
        {
            data[N2DM_AXIS_X] = ~data[N2DM_AXIS_X];
            data[N2DM_AXIS_X] &= 0xfff;
            data[N2DM_AXIS_X]+=1;
            data[N2DM_AXIS_X] = -data[N2DM_AXIS_X];
        }
        if(data[N2DM_AXIS_Y]&0x800)
        {
            data[N2DM_AXIS_Y] = ~data[N2DM_AXIS_Y];
            data[N2DM_AXIS_Y] &= 0xfff;
            data[N2DM_AXIS_Y]+=1;
            data[N2DM_AXIS_Y] = -data[N2DM_AXIS_Y];
        }
        if(data[N2DM_AXIS_Z]&0x800)
        {
            data[N2DM_AXIS_Z] = ~data[N2DM_AXIS_Z];
            data[N2DM_AXIS_Z] &= 0xfff;
            data[N2DM_AXIS_Z]+=1;
            data[N2DM_AXIS_Z] = -data[N2DM_AXIS_Z];
        }

        if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
        {
            GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[N2DM_AXIS_X], data[N2DM_AXIS_Y], data[N2DM_AXIS_Z],
                    data[N2DM_AXIS_X], data[N2DM_AXIS_Y], data[N2DM_AXIS_Z]);
        }

#ifdef CONFIG_N2DM_LOWPASS
        if(atomic_read(&priv->filter))
        {
            if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
            {
                int idx, firlen = atomic_read(&priv->firlen);   
                if(priv->fir.num < firlen)
                {                
                    priv->fir.raw[priv->fir.num][N2DM_AXIS_X] = data[N2DM_AXIS_X];
                    priv->fir.raw[priv->fir.num][N2DM_AXIS_Y] = data[N2DM_AXIS_Y];
                    priv->fir.raw[priv->fir.num][N2DM_AXIS_Z] = data[N2DM_AXIS_Z];
                    priv->fir.sum[N2DM_AXIS_X] += data[N2DM_AXIS_X];
                    priv->fir.sum[N2DM_AXIS_Y] += data[N2DM_AXIS_Y];
                    priv->fir.sum[N2DM_AXIS_Z] += data[N2DM_AXIS_Z];
                    if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
                    {
                        GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
                                priv->fir.raw[priv->fir.num][N2DM_AXIS_X], priv->fir.raw[priv->fir.num][N2DM_AXIS_Y], priv->fir.raw[priv->fir.num][N2DM_AXIS_Z],
                                priv->fir.sum[N2DM_AXIS_X], priv->fir.sum[N2DM_AXIS_Y], priv->fir.sum[N2DM_AXIS_Z]);
                    }
                    priv->fir.num++;
                    priv->fir.idx++;
                }
                else
                {
                    idx = priv->fir.idx % firlen;
                    priv->fir.sum[N2DM_AXIS_X] -= priv->fir.raw[idx][N2DM_AXIS_X];
                    priv->fir.sum[N2DM_AXIS_Y] -= priv->fir.raw[idx][N2DM_AXIS_Y];
                    priv->fir.sum[N2DM_AXIS_Z] -= priv->fir.raw[idx][N2DM_AXIS_Z];
                    priv->fir.raw[idx][N2DM_AXIS_X] = data[N2DM_AXIS_X];
                    priv->fir.raw[idx][N2DM_AXIS_Y] = data[N2DM_AXIS_Y];
                    priv->fir.raw[idx][N2DM_AXIS_Z] = data[N2DM_AXIS_Z];
                    priv->fir.sum[N2DM_AXIS_X] += data[N2DM_AXIS_X];
                    priv->fir.sum[N2DM_AXIS_Y] += data[N2DM_AXIS_Y];
                    priv->fir.sum[N2DM_AXIS_Z] += data[N2DM_AXIS_Z];
                    priv->fir.idx++;
                    data[N2DM_AXIS_X] = priv->fir.sum[N2DM_AXIS_X]/firlen;
                    data[N2DM_AXIS_Y] = priv->fir.sum[N2DM_AXIS_Y]/firlen;
                    data[N2DM_AXIS_Z] = priv->fir.sum[N2DM_AXIS_Z]/firlen;
                    if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
                    {
                        GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
                                priv->fir.raw[idx][N2DM_AXIS_X], priv->fir.raw[idx][N2DM_AXIS_Y], priv->fir.raw[idx][N2DM_AXIS_Z],
                                priv->fir.sum[N2DM_AXIS_X], priv->fir.sum[N2DM_AXIS_Y], priv->fir.sum[N2DM_AXIS_Z],
                                data[N2DM_AXIS_X], data[N2DM_AXIS_Y], data[N2DM_AXIS_Z]);
                    }
                }
            }
        }	
#endif         
    }
    return err;
}
/*----------------------------------------------------------------------------*/
/*
   static int N2DM_ReadOffset(struct i2c_client *client, s8 ofs[N2DM_AXES_NUM])
   {    
   int err;

   return err;    
   }
   */
/*----------------------------------------------------------------------------*/
static int N2DM_ResetCalibration(struct i2c_client *client)
{
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);	

    memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
    return 0;     
}
/*----------------------------------------------------------------------------*/
static int N2DM_ReadCalibration(struct i2c_client *client, int dat[N2DM_AXES_NUM])
{
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);

    dat[obj->cvt.map[N2DM_AXIS_X]] = obj->cvt.sign[N2DM_AXIS_X]*obj->cali_sw[N2DM_AXIS_X];
    dat[obj->cvt.map[N2DM_AXIS_Y]] = obj->cvt.sign[N2DM_AXIS_Y]*obj->cali_sw[N2DM_AXIS_Y];
    dat[obj->cvt.map[N2DM_AXIS_Z]] = obj->cvt.sign[N2DM_AXIS_Z]*obj->cali_sw[N2DM_AXIS_Z];                        

    return 0;
}
/*----------------------------------------------------------------------------*/
/*
   static int N2DM_ReadCalibrationEx(struct i2c_client *client, int act[N2DM_AXES_NUM], int raw[N2DM_AXES_NUM])
   {  

   struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
   int err;
   int mul;

   if(err = N2DM_ReadOffset(client, obj->offset))
   {
   GSE_ERR("read offset fail, %d\n", err);
   return err;
   }    

   mul = obj->reso->sensitivity/n2dm_offset_resolution.sensitivity;
   raw[N2DM_AXIS_X] = obj->offset[N2DM_AXIS_X]*mul + obj->cali_sw[N2DM_AXIS_X];
   raw[N2DM_AXIS_Y] = obj->offset[N2DM_AXIS_Y]*mul + obj->cali_sw[N2DM_AXIS_Y];
   raw[N2DM_AXIS_Z] = obj->offset[N2DM_AXIS_Z]*mul + obj->cali_sw[N2DM_AXIS_Z];

   act[obj->cvt.map[N2DM_AXIS_X]] = obj->cvt.sign[N2DM_AXIS_X]*raw[N2DM_AXIS_X];
   act[obj->cvt.map[N2DM_AXIS_Y]] = obj->cvt.sign[N2DM_AXIS_Y]*raw[N2DM_AXIS_Y];
   act[obj->cvt.map[N2DM_AXIS_Z]] = obj->cvt.sign[N2DM_AXIS_Z]*raw[N2DM_AXIS_Z];                        

   return 0;
   }
   */
/*----------------------------------------------------------------------------*/
static int N2DM_WriteCalibration(struct i2c_client *client, int dat[N2DM_AXES_NUM])
{
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    int err = 0;
    //	int cali[N2DM_AXES_NUM];


    GSE_FUN();
    if(!obj || ! dat)
    {
        GSE_ERR("null ptr!!\n");
        return -EINVAL;
    }
    else
    {        
        s16 cali[N2DM_AXES_NUM];
        cali[obj->cvt.map[N2DM_AXIS_X]] = obj->cvt.sign[N2DM_AXIS_X]*obj->cali_sw[N2DM_AXIS_X];
        cali[obj->cvt.map[N2DM_AXIS_Y]] = obj->cvt.sign[N2DM_AXIS_Y]*obj->cali_sw[N2DM_AXIS_Y];
        cali[obj->cvt.map[N2DM_AXIS_Z]] = obj->cvt.sign[N2DM_AXIS_Z]*obj->cali_sw[N2DM_AXIS_Z]; 
        cali[N2DM_AXIS_X] += dat[N2DM_AXIS_X];
        cali[N2DM_AXIS_Y] += dat[N2DM_AXIS_Y];
        cali[N2DM_AXIS_Z] += dat[N2DM_AXIS_Z];

        obj->cali_sw[N2DM_AXIS_X] += obj->cvt.sign[N2DM_AXIS_X]*dat[obj->cvt.map[N2DM_AXIS_X]];
        obj->cali_sw[N2DM_AXIS_Y] += obj->cvt.sign[N2DM_AXIS_Y]*dat[obj->cvt.map[N2DM_AXIS_Y]];
        obj->cali_sw[N2DM_AXIS_Z] += obj->cvt.sign[N2DM_AXIS_Z]*dat[obj->cvt.map[N2DM_AXIS_Z]];
    } 

    return err;
}
/*----------------------------------------------------------------------------*/
#if 0
static int N2DM_CheckDeviceID(struct i2c_client *client)
{
    u8 databuf[10];    
    int res = 0;
    /*
       memset(databuf, 0, sizeof(u8)*10);    
       databuf[0] = N2DM_REG_DEVID;    

       res = i2c_master_send(client, databuf, 0x1);
       if(res <= 0)
       {
       goto exit_N2DM_CheckDeviceID;
       }

       udelay(500);

       databuf[0] = 0x0;        
       res = i2c_master_recv(client, databuf, 0x01);
       if(res <= 0)
       {
       goto exit_N2DM_CheckDeviceID;
       }


       if(databuf[0]!=N2DM_FIXED_DEVID)
       {
       return N2DM_ERR_IDENTIFICATION;
       }

exit_N2DM_CheckDeviceID:
if (res <= 0)
{
return N2DM_ERR_I2C;
}
*/
    return N2DM_SUCCESS;
    }
#endif
/*----------------------------------------------------------------------------*/
static int N2DM_SetPowerMode(struct i2c_client *client, bool enable)
{
    u8 databuf[2];    
    int res = 0;
    u8 addr = N2DM_REG_CTL_REG1;
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);

    //	printk("lizhi n2dm %s",__func__);
    if(enable == sensor_power)
    {
        //		GSE_LOG("Sensor power status is newest!\n");
        return N2DM_SUCCESS;
    }

    if(n2dm_i2c_read_byte(client, addr, &databuf[0]))
    {
        GSE_ERR("read power ctl register err!\n");
        return N2DM_ERR_I2C;
    }

    databuf[0] &= ~N2DM_MEASURE_MODE;

    if(enable) 
    {
        databuf[0] &=  ~N2DM_MEASURE_MODE;
    }
    else
    {
        databuf[0] |= N2DM_MEASURE_MODE;
    }
    databuf[1] = databuf[0];
    databuf[0] = N2DM_REG_CTL_REG1;


    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0)
    {
        GSE_LOG("set power mode failed!\n");
        return N2DM_ERR_I2C;
    }
    else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
    {
        GSE_LOG("set power mode ok %d!\n", databuf[1]);
    }

    sensor_power = enable;

    return N2DM_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int N2DM_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    u8 databuf[10];
    u8 addr = N2DM_REG_CTL_REG4;
    int res = 0;

    memset(databuf, 0, sizeof(u8)*10);

    if(n2dm_i2c_read_byte(client, addr, &databuf[0]))
    {
        GSE_ERR("read reg_ctl_reg1 register err!\n");
        return N2DM_ERR_I2C;
    }

    databuf[0] &= ~0x30;
    databuf[0] |=dataformat;


    databuf[0] |=0x08;	  //enable high resolution .  //malp

    databuf[1] = databuf[0];
    databuf[0] = N2DM_REG_CTL_REG4;


    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0)
    {
        return N2DM_ERR_I2C;
    }


    return N2DM_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int N2DM_SetBWRate(struct i2c_client *client, u8 bwrate)
{
    u8 databuf[10];
    u8 addr = N2DM_REG_CTL_REG1;
    int res = 0;

    memset(databuf, 0, sizeof(u8)*10);

    if(n2dm_i2c_read_byte(client, addr, &databuf[0]))
    {
        GSE_ERR("read reg_ctl_reg1 register err!\n");
        return N2DM_ERR_I2C;
    }

    databuf[0] &= ~0xF0;
    databuf[0] |= bwrate;

    databuf[1] = databuf[0];
    databuf[0] = N2DM_REG_CTL_REG1;

    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0)
    {
        return N2DM_ERR_I2C;
    }

    return N2DM_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
//enalbe data ready interrupt
static int N2DM_SetIntEnable(struct i2c_client *client, u8 intenable)
{
    u8 databuf[10];
    u8 addr = N2DM_REG_CTL_REG3;
    int res = 0;

    memset(databuf, 0, sizeof(u8)*10); 

    if(n2dm_i2c_read_byte(client, addr, &databuf[0]))
    {
        GSE_ERR("read reg_ctl_reg1 register err!\n");
        return N2DM_ERR_I2C;
    }

    databuf[0] = 0x00;
    databuf[1] = databuf[0];
    databuf[0] = N2DM_REG_CTL_REG3;

    res = i2c_master_send(client, databuf, 0x2);

    if(res <= 0)
    {
        return N2DM_ERR_I2C;
    }

    return N2DM_SUCCESS;    
}
/* Gionee BSP1 lizhi 20150713 add for motion-detect begin */
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
static int n2dm_set_pickup_enable(struct i2c_client *client, bool enable)
{
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    int res = 0;

    if(enable){
        res = n2dm_i2c_write_byte(client,0x30,0x20);//INT1_CFG:0x30,Enable ZH interrupt generation
        if(res != N2DM_SUCCESS)
        {
            return res;
        }
       enable_irq(obj->irq); 
    }else{
        res = n2dm_i2c_write_byte(client,0x30,0x00);//INT1_CFG:0x30,Disable ZH interrupt generation
        if(res != N2DM_SUCCESS)
        {
            return res;
        }
		disable_irq(obj->irq);
    }
    obj->pickup_enable = enable;
    return N2DM_SUCCESS; 
}
static int n2dm_init_pickup_detect(struct i2c_client *client)//init pickup-detect config for Interrupt ,but not enable
{
    int res = 0;

    res = n2dm_i2c_write_byte(client,N2DM_REG_CTL_REG1,0x57);//100Hz.enable x y z 
    if(res != N2DM_SUCCESS)
    {
        return res;
    }
    res = n2dm_i2c_write_byte(client,N2DM_REG_CTL_REG2,0xbf);//set High pass filter for eint 
    if(res != N2DM_SUCCESS)
    {
        return res;
    }

    res = n2dm_i2c_write_byte(client,N2DM_REG_CTL_REG3,0x40);//enable EINT1
    if(res != N2DM_SUCCESS)
    {
        return res;
    }
    res = n2dm_i2c_write_byte(client,N2DM_REG_CTL_REG4,0x08);//high resolution +/-2g FS
    if(res != N2DM_SUCCESS)
    {
        return res;
    }
    res = n2dm_i2c_write_byte(client,N2DM_REG_CTL_REG5,0x08);//latch Interrupt until read INT1_SRC
    if(res != N2DM_SUCCESS)
    {
        return res;
    }
    res = n2dm_i2c_write_byte(client,0x32,0x10);// INT1_THS:0x32,Threshold = 250mg
    if(res != N2DM_SUCCESS)
    {
        return res;
    }
    res = n2dm_i2c_write_byte(client,0x33,0x00);//INT1_DURATION:0x33,Duration = 0
    if(res != N2DM_SUCCESS)
    {
        return res;
    }
	return 0;
}

static void n2dm_eint_work_func(struct work_struct *work)
{
    struct hwm_sensor_data sensor_data;
    int err;
    struct n2dm_i2c_data *obj = obj_i2c_data;
    u8 data_status_reg = 0;
    //GSE_FUN();
    
    n2dm_set_pickup_enable(obj->client,false);
    if (obj == NULL) {
        GSE_ERR("%s obj is null", __func__);
    }

    err = n2dm_i2c_read_byte(obj->client, 0x31, &data_status_reg);//INT1_SRC:0x31
    // GSE_LOG("read the resister 0x31 = %x\n", data_status_reg);
    if (err != N2DM_SUCCESS) {
        GSE_ERR("read the resister 0x18 error\n");
        return;
    }
    sensor_data.values[0] = 1;
    sensor_data.value_divide = 1;
    sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
    err = hwmsen_get_interrupt_data(ID_PICK_UP_GESTURE, &sensor_data);
	if(err){
        GSE_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
        return;
    }
    if(atomic_read(&obj->trace) & ADX_TRC_EINT)
    {
        GSE_LOG("%s:int src %x,sensor_data: %d",__func__,data_status_reg,sensor_data.values[0]);
    }
    return;
}
static irqreturn_t pkup_eint_handler(int irq, void *desc)                                                                                            
{           
    schedule_work(&obj_i2c_data->n2dm_eint_work);
	//disable_irq_nosync(obj_i2c_data->irq);
	 
    return IRQ_HANDLED;
}
static int n2dm_setup_eint(struct i2c_client *client) 
{
    int ret;
	struct platform_device *gsensorPltFmDev;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_cfg;
    u32 ints[2] = {0, 0};

    gsensorPltFmDev = get_gsensor_platformdev();
    /* gpio setting */
    pinctrl = devm_pinctrl_get(&gsensorPltFmDev->dev);
    if (IS_ERR(pinctrl)) {
        ret = PTR_ERR(pinctrl);
        GSE_ERR("Cannot find gsensor pinctrl!\n");
    }

    pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
    if (IS_ERR(pins_cfg)) {
        ret = PTR_ERR(pins_cfg);
        GSE_ERR("Cannot find gsensor pinctrl pin_cfg!\n");
    }
    obj_i2c_data->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, gse_1-eint");
    if (obj_i2c_data->irq_node) {
        of_property_read_u32_array(obj_i2c_data->irq_node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_request(ints[0], "g-sensor");
        gpio_set_debounce(ints[0], ints[1]);
        pinctrl_select_state(pinctrl, pins_cfg);
        GSE_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

        obj_i2c_data->irq = irq_of_parse_and_map(obj_i2c_data->irq_node, 0);
        GSE_LOG("obj_i2c_data->irq = %d\n", obj_i2c_data->irq);
        if (!obj_i2c_data->irq) {
            GSE_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        if (request_irq(obj_i2c_data->irq, pkup_eint_handler, IRQF_TRIGGER_RISING, "GSE_1-eint", NULL)) {
            GSE_ERR("IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;
        }
    } else {
        GSE_ERR("null irq node!!\n");
        return -EINVAL;
    }
    return 0;
}
#endif
/* Gionee BSP1 lizhi 20150713 add for motion-detect end */

/* lizhi add for gsensor no use after hw reset 20141217@CR01422678 begin*/
static int N2DM_SetPowerDown(struct i2c_client *client) {
    int res = 0;
    u8 databuf[2] = {0, 0};
    // get ctrl_reg1 current value
    if(n2dm_i2c_read_byte(client, N2DM_REG_CTL_REG1, &databuf[0])) {
        GSE_ERR("read reg_ctl_reg1 register err!\n");
    }
    printk("lizhi n2dm N2DM_SetPowerDown\n");
    // clear ODR to 0x00
    databuf[0] &= ~0xF0;
    databuf[1] = databuf[0];
    databuf[0] = N2DM_REG_CTL_REG1;
    res = i2c_master_send(client, databuf, 0x2);
    if(res <= 0) {
        GSE_ERR("write reg_ctl_reg1 register err!\n");
    }
    msleep(20);
    return N2DM_SUCCESS;
}
/* lizhi add for gsensor no use after hw reset 20141217@CR01422678 end*/
/*----------------------------------------------------------------------------*/
static int N2DM_Init(struct i2c_client *client, int reset_cali)
{
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    int res = 0;
    printk("lizhi n2dm %s\n",__func__);
    /*
       res = N2DM_CheckDeviceID(client); 
       if(res != N2DM_SUCCESS)
       {
       return res;
       }	
       */
    // first clear reg1
    /* lizhi add for gsensor no use after hw reset 20141217@CR01422678 begin*/
    res = N2DM_SetPowerDown(client);
    if(res != N2DM_SUCCESS)
    {
        return res;
    }
    /* lizhi add for gsensor no use after hw reset 20141217@CR01422678 end*/
    res = n2dm_i2c_write_byte(client,N2DM_REG_CTL_REG1,0x07);
    if(res != N2DM_SUCCESS)
    {
        return res;
    }

    res = N2DM_SetPowerMode(client, false);
    if(res != N2DM_SUCCESS)
    {
        return res;
    }


    res = N2DM_SetBWRate(client, N2DM_BW_100HZ);//400 or 100 no other choice
    if(res != N2DM_SUCCESS )
    {
        return res;
    }
    res = n2dm_i2c_write_byte(client,N2DM_REG_CTL_REG2,0x00);//disable High pass filter 
    if(res != N2DM_SUCCESS)
    {
        return res;
    }

    res = N2DM_SetDataFormat(client, N2DM_RANGE_2G);//8g or 2G no oher choise
    if(res != N2DM_SUCCESS) 
    {
        return res;
    }
    gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

    res = N2DM_SetIntEnable(client, false);        
    if(res != N2DM_SUCCESS)
    {
        return res;
    }


    if(0 != reset_cali)
    { 
        //reset calibration only in power on
        res = N2DM_ResetCalibration(client);
        if(res != N2DM_SUCCESS)
        {
            return res;
        }
    }

#ifdef CONFIG_N2DM_LOWPASS
    memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

    return N2DM_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int N2DM_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
    u8 databuf[10];    

    memset(databuf, 0, sizeof(u8)*10);

    if((NULL == buf)||(bufsize<=30))
    {
        return -1;
    }

    if(NULL == client)
    {
        *buf = 0;
        return -2;
    }

    sprintf(buf, "N2DM Chip");
    return 0;
}
/*----------------------------------------------------------------------------*/
static int N2DM_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
    struct n2dm_i2c_data *obj = (struct n2dm_i2c_data*)i2c_get_clientdata(client);
    u8 databuf[20];
    int acc[N2DM_AXES_NUM];
    int res = 0;
    memset(databuf, 0, sizeof(u8)*10);

    if(NULL == buf)
    {
        return -1;
    }
    if(NULL == client)
    {
        *buf = 0;
        return -2;
    }

    if(!sensor_power)
    {
        res = N2DM_SetPowerMode(client, true);
        if(res)
        {
            GSE_ERR("Power on n2dm error %d!\n", res);
        }
        msleep(20);
    }

    if((res = N2DM_ReadData(client, obj->data)))
    {        
        GSE_ERR("I2C error: ret value=%d", res);
        return -3;
    }
    else
    {
        obj->data[N2DM_AXIS_X] += obj->cali_sw[N2DM_AXIS_X];
        obj->data[N2DM_AXIS_Y] += obj->cali_sw[N2DM_AXIS_Y];
        obj->data[N2DM_AXIS_Z] += obj->cali_sw[N2DM_AXIS_Z];

        /*remap coordinate*/
        acc[obj->cvt.map[N2DM_AXIS_X]] = obj->cvt.sign[N2DM_AXIS_X]*obj->data[N2DM_AXIS_X];
        acc[obj->cvt.map[N2DM_AXIS_Y]] = obj->cvt.sign[N2DM_AXIS_Y]*obj->data[N2DM_AXIS_Y];
        acc[obj->cvt.map[N2DM_AXIS_Z]] = obj->cvt.sign[N2DM_AXIS_Z]*obj->data[N2DM_AXIS_Z];

        //GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[N2DM_AXIS_X], acc[N2DM_AXIS_Y], acc[N2DM_AXIS_Z]);

        //Out put the mg
        acc[N2DM_AXIS_X] = acc[N2DM_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
        acc[N2DM_AXIS_Y] = acc[N2DM_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
        acc[N2DM_AXIS_Z] = acc[N2DM_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		


        sprintf(buf, "%04x %04x %04x", acc[N2DM_AXIS_X], acc[N2DM_AXIS_Y], acc[N2DM_AXIS_Z]);
        if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)//atomic_read(&obj->trace) & ADX_TRC_IOCTL
        {
            GSE_LOG("gsensor data: %s!\n", buf);
            dumpReg(client);
        }
    }

    return 0;
}
/*----------------------------------------------------------------------------*/
static int N2DM_ReadRawData(struct i2c_client *client, char *buf)
{
    struct n2dm_i2c_data *obj = (struct n2dm_i2c_data*)i2c_get_clientdata(client);
    int res = 0;

    if (!buf || !client)
    {
        return EINVAL;
    }

    if((res = N2DM_ReadData(client, obj->data)))
    {        
        GSE_ERR("I2C error: ret value=%d", res);
        return EIO;
    }
    else
    {
        sprintf(buf, "%04x %04x %04x", obj->data[N2DM_AXIS_X], 
                obj->data[N2DM_AXIS_Y], obj->data[N2DM_AXIS_Z]);

    }

    return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = n2dm_i2c_client;
    char strbuf[N2DM_BUFSIZE];
    if(NULL == client)
    {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }

    N2DM_ReadChipInfo(client, strbuf, N2DM_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = n2dm_i2c_client;
    char strbuf[N2DM_BUFSIZE];

    if(NULL == client)
    {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }
    N2DM_ReadSensorData(client, strbuf, N2DM_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = n2dm_i2c_client;
    struct n2dm_i2c_data *obj;
    int err, len = 0, mul;
    int tmp[N2DM_AXES_NUM];

    if(NULL == client)
    {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }

    obj = i2c_get_clientdata(client);

    if((err = N2DM_ReadCalibration(client, tmp)))
    {
        return -EINVAL;
    }
    else
    {    
        mul = obj->reso->sensitivity/n2dm_offset_resolution.sensitivity;
        len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
                obj->offset[N2DM_AXIS_X], obj->offset[N2DM_AXIS_Y], obj->offset[N2DM_AXIS_Z],
                obj->offset[N2DM_AXIS_X], obj->offset[N2DM_AXIS_Y], obj->offset[N2DM_AXIS_Z]);
        len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
                obj->cali_sw[N2DM_AXIS_X], obj->cali_sw[N2DM_AXIS_Y], obj->cali_sw[N2DM_AXIS_Z]);

        len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
                obj->offset[N2DM_AXIS_X]*mul + obj->cali_sw[N2DM_AXIS_X],
                obj->offset[N2DM_AXIS_Y]*mul + obj->cali_sw[N2DM_AXIS_Y],
                obj->offset[N2DM_AXIS_Z]*mul + obj->cali_sw[N2DM_AXIS_Z],
                tmp[N2DM_AXIS_X], tmp[N2DM_AXIS_Y], tmp[N2DM_AXIS_Z]);

        return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct i2c_client *client = n2dm_i2c_client;  
    int err, x, y, z;
    int dat[N2DM_AXES_NUM];

    if(!strncmp(buf, "rst", 3))
    {
        if((err = N2DM_ResetCalibration(client)))
        {
            GSE_ERR("reset offset err = %d\n", err);
        }	
    }
    else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
    {
        dat[N2DM_AXIS_X] = x;
        dat[N2DM_AXIS_Y] = y;
        dat[N2DM_AXIS_Z] = z;
        if((err = N2DM_WriteCalibration(client, dat)))
        {
            GSE_ERR("write calibration err = %d\n", err);
        }		
    }
    else
    {
        GSE_ERR("invalid format\n");
    }

    return count;
}
/*----------------------------------------------------------------------------*/

static ssize_t show_power_status(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = n2dm_i2c_client;
    struct n2dm_i2c_data *obj;
    u8 data;

    if(NULL == client)
    {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }

    obj = i2c_get_clientdata(client);
    n2dm_i2c_read_byte(client,N2DM_REG_CTL_REG1,&data);
    data &= 0x08;
    data = data>>3;
    return snprintf(buf, PAGE_SIZE, "%x\n", data);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_N2DM_LOWPASS
    struct i2c_client *client = n2dm_i2c_client;
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    if(atomic_read(&obj->firlen))
    {
        int idx, len = atomic_read(&obj->firlen);
        GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

        for(idx = 0; idx < len; idx++)
        {
            GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][N2DM_AXIS_X], obj->fir.raw[idx][N2DM_AXIS_Y], obj->fir.raw[idx][N2DM_AXIS_Z]);
        }

        GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[N2DM_AXIS_X], obj->fir.sum[N2DM_AXIS_Y], obj->fir.sum[N2DM_AXIS_Z]);
        GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[N2DM_AXIS_X]/len, obj->fir.sum[N2DM_AXIS_Y]/len, obj->fir.sum[N2DM_AXIS_Z]/len);
    }
    return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
    return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_N2DM_LOWPASS
    struct i2c_client *client = n2dm_i2c_client;  
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    int firlen;

    if(1 != sscanf(buf, "%d", &firlen))
    {
        GSE_ERR("invallid format\n");
    }
    else if(firlen > C_MAX_FIR_LENGTH)
    {
        GSE_ERR("exceeds maximum filter length\n");
    }
    else
    { 
        atomic_set(&obj->firlen, firlen);
        if(0 == firlen)//yucong fix build warning
        {
            atomic_set(&obj->fir_en, 0);
        }
        else
        {
            memset(&obj->fir, 0x00, sizeof(obj->fir));
            atomic_set(&obj->fir_en, 1);
        }
    }
#endif    
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    struct n2dm_i2c_data *obj = obj_i2c_data;
    if (obj == NULL)
    {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }

    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
    return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct n2dm_i2c_data *obj = obj_i2c_data;
    int trace;
    if (obj == NULL)
    {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }

    if(1 == sscanf(buf, "0x%x", &trace))
    {
        atomic_set(&obj->trace, trace);
    }	
    else
    {
        GSE_ERR("invalid content: '%s', length = %zd\n", buf, count);
    }

    return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;    
    struct n2dm_i2c_data *obj = obj_i2c_data;
    if (obj == NULL)
    {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }	

    if(obj->hw)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
                obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
    }
    else
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    }
    return len;    
}

static ssize_t store_status_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct n2dm_i2c_data *obj = obj_i2c_data;
    int dir;
    if (obj == NULL) {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }

    if(1 == sscanf(buf, "0x%x", &dir)) {
        obj->hw->direction = dir;
        GSE_ERR("direction: %d\n", obj->hw->direction);
        hwmsen_get_convert(obj->hw->direction, &obj->cvt);
    }

    return count;    
}

static ssize_t show_reg_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = n2dm_i2c_client; 
    struct n2dm_i2c_data *obj = obj_i2c_data;
    int i;
    u8 bufdata;
    int count = 0;
    if (obj == NULL) {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }

    for(i=0;i<55;i++){
        n2dm_i2c_read_byte(client,0x07+i,&bufdata);
        count+=sprintf(buf+count,"[%x] = [%x]\n",0x07+i,bufdata);
    }
    return count;
}
static ssize_t store_reg_value(struct device_driver *ddri,const char *buf,size_t count)
{
    struct i2c_client *client = n2dm_i2c_client;
    u32 data[2];
    if(2 != sscanf(buf,"0x%x 0x%x",&data[0],&data[1])){
        GSE_ERR("invalid format:%s\n",buf);
        return 0;
    }

    n2dm_i2c_write_byte(client,data[0],data[1]);

    return count;
}

#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
static ssize_t show_pickup_enable(struct device_driver *ddri, char *buf)
{
    struct n2dm_i2c_data *obj = obj_i2c_data;
    int count = 0;
    count = sprintf(buf,"%d\n",obj->pickup_enable);
    return count;
}
static ssize_t store_pickup_enable(struct device_driver *ddri,const char *buf,size_t count)
{
    struct n2dm_i2c_data *obj = obj_i2c_data;
    u32 enable = 0;
    int err = 0;
    struct hwm_sensor_data sensor_data;
    if(1 != sscanf(buf,"%d",&enable)){
        GSE_ERR("invalid format:%s\n",buf);
        return 0;
    }
    GSE_LOG("%s pickup sensor enable value=%d, pickup_enable =%d\n",__func__,enable,obj->pickup_enable);
    if(((enable == 0) && (false == obj->pickup_enable)) ||((enable == 1) && (true == obj->pickup_enable)))
    {
        GSE_LOG("pickup device have updated!\n");
    }
    else
    {
        obj->pickup_enable = (bool)enable;
        if(enable){
            sensor_data.values[0] = 0;
            sensor_data.value_divide = 1;
            sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

            err = hwmsen_get_interrupt_data(ID_PICK_UP_GESTURE, &sensor_data);
			if(err){
                GSE_ERR("%s call hwmsen_get_interrupt_data fail = %d\n", __func__, err);
                return err;
            }
        }
    }
	return count;
}
#endif
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(power,                S_IRUGO, show_power_status,          NULL);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,     S_IWUSR | S_IRUGO, show_status_value,        store_status_value);
static DRIVER_ATTR(reg,        S_IWUSR | S_IRUGO, show_reg_value,           store_reg_value);
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
static DRIVER_ATTR(pickup,     S_IWUSR | S_IRUGO, show_pickup_enable,       store_pickup_enable);
#endif
/*----------------------------------------------------------------------------*/
static struct driver_attribute *n2dm_attr_list[] = {
    &driver_attr_chipinfo,     /*chip information*/
    &driver_attr_sensordata,   /*dump sensor data*/
    &driver_attr_cali,         /*show calibration data*/
    &driver_attr_power,         /*show power reg*/
    &driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
    &driver_attr_trace,        /*trace log*/
    &driver_attr_status,        
    &driver_attr_reg,      /*for read/write reg*/
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
    &driver_attr_pickup,  /*pick enable control*/
#endif
};
/*----------------------------------------------------------------------------*/
static int n2dm_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(n2dm_attr_list)/sizeof(n2dm_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, n2dm_attr_list[idx])))
        {            
            GSE_ERR("driver_create_file (%s) = %d\n", n2dm_attr_list[idx]->attr.name, err);
            break;
        }
    }    
    return err;
}
/*----------------------------------------------------------------------------*/
static int n2dm_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(n2dm_attr_list)/sizeof(n2dm_attr_list[0]));

    if(driver == NULL)
    {
        return -EINVAL;
    }


    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, n2dm_attr_list[idx]);
    }


    return err;
}

/*----------------------------------------------------------------------------*/
int n2dm_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value, sample_delay;	
    struct n2dm_i2c_data *priv = (struct n2dm_i2c_data*)self;
    struct hwm_sensor_data* gsensor_data;
    char buff[N2DM_BUFSIZE];

    //GSE_FUN(f);
    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                GSE_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                if(value <= 5)
                {
                    sample_delay = N2DM_BW_200HZ;
                }
                else if(value <= 10)
                {
                    //mdf by malp
                    //sample_delay = ~N2DM_BW_100HZ;  
                    sample_delay = N2DM_BW_100HZ;
                }
                else
                {
                    //mdf by malp
                    //sample_delay = ~N2DM_BW_50HZ;
                    sample_delay = N2DM_BW_50HZ;
                }

                err = N2DM_SetBWRate(priv->client, sample_delay);
                if(err != N2DM_SUCCESS ) //0x2C->BW=100Hz
                {
                    GSE_ERR("Set delay parameter error!\n");
                }

                if(value >= 50)
                {
                    atomic_set(&priv->filter, 0);
                }
                else
                {					
                    priv->fir.num = 0;
                    priv->fir.idx = 0;
                    priv->fir.sum[N2DM_AXIS_X] = 0;
                    priv->fir.sum[N2DM_AXIS_Y] = 0;
                    priv->fir.sum[N2DM_AXIS_Z] = 0;
                    atomic_set(&priv->filter, 1);
                }
            }
            break;

        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                GSE_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                GSE_LOG("enable value=%d, sensor_power =%d\n",value,sensor_power);
                if(((value == 0) && (false == priv->gsensor_enable)) ||((value == 1) && (true == priv->gsensor_enable)))
                {
                    GSE_LOG("Gsensor device have updated!\n");
                }
                else
                {
                    priv->gsensor_enable = (bool)value;
                    err = N2DM_SetPowerMode( priv->client, (bool)value);
                }
            }
            break;

        case SENSOR_GET_DATA:
            if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
            {
                GSE_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                gsensor_data = (struct hwm_sensor_data *)buff_out;
                N2DM_ReadSensorData(priv->client, buff, N2DM_BUFSIZE);
                sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
                        &gsensor_data->values[1], &gsensor_data->values[2]);				
                gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
                gsensor_data->value_divide = 1000;
            }
            break;
        default:
            GSE_ERR("gsensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}
/* Gionee BSP1 lizhi 20150713 add for motion-detect begin */
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
int n2dm_pickup_detect_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;	
    struct n2dm_i2c_data *priv = (struct n2dm_i2c_data*)self;
    struct hwm_sensor_data sensor_data;	
    GSE_FUN();
    switch (command)
    {
        case SENSOR_DELAY:
            break;
        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                GSE_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {

                value = *(int *)buff_in;
                GSE_LOG("pickup sensor enable value=%d, pickup_enable =%d\n",value,priv->pickup_enable);
                if(((value == 0) && (false == priv->pickup_enable)) ||((value == 1) && (true == priv->pickup_enable)))
                {
                    GSE_LOG("pickup device have updated!\n");
                }
                else
                {
                    priv->pickup_enable = (bool)value;
                    if(value){
						n2dm_init_pickup_detect(priv->client);
        				n2dm_set_pickup_enable(priv->client, true);
                        sensor_data.values[0] = 0;
                        sensor_data.value_divide = 1;
                        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

                        err = hwmsen_get_interrupt_data(ID_PICK_UP_GESTURE, &sensor_data);
						if(err){
                            GSE_ERR("%s call hwmsen_get_interrupt_data fail = %d\n", __func__, err);
                            return err;
                        }
                    }
                }
            }
            break;

        default:
            GSE_ERR("pickup_detect operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}
/* Gionee BSP1 lizhi 20150713 add for motion-detect end */
#endif 
/****************************************************************************** 
 * Function Configuration
 ******************************************************************************/
static int n2dm_open(struct inode *inode, struct file *file)
{
    file->private_data = n2dm_i2c_client;

    if(file->private_data == NULL)
    {
        GSE_ERR("null pointer!!\n");
        return -EINVAL;
    }
    return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int n2dm_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
/*----------------------------------------------------------------------------*/

#ifdef CONFIG_COMPAT
static long n2dm_unlocked_compat_ioctl(struct file *file, unsigned int cmd,
        unsigned long arg)
{
    long err = 0;

    void __user *arg32 = compat_ptr(arg);

    if (!file->f_op || !file->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd)
    {
        case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }

            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
            if (err){
                GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
                return err;
            }
            break;
        case COMPAT_GSENSOR_IOCTL_SET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }

            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
            if (err){
                GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
                return err;
            }
            break;
        case COMPAT_GSENSOR_IOCTL_GET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }

            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
            if (err){
                GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
                return err;
            }
            break;
        case COMPAT_GSENSOR_IOCTL_CLR_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }

            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
            if (err){
                GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
                return err;
            }
            break;

        default:
            GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
            break;

    }

    return err;
}
#endif

static long n2dm_unlocked_ioctl(struct file *file, unsigned int cmd,
        unsigned long arg)

{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct n2dm_i2c_data *obj = (struct n2dm_i2c_data*)i2c_get_clientdata(client);	
    char strbuf[N2DM_BUFSIZE];
    void __user *data;
    struct SENSOR_DATA sensor_data;
    long err = 0;
    int cali[3];

    //GSE_FUN(f);
    if(_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if(_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }

    if(err)
    {
        GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch(cmd)
    {
        case GSENSOR_IOCTL_INIT:
            N2DM_Init(client, 0);			
            break;

        case GSENSOR_IOCTL_READ_CHIPINFO:
            data = (void __user *) arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;	  
            }

            N2DM_ReadChipInfo(client, strbuf, N2DM_BUFSIZE);
            if(copy_to_user(data, strbuf, strlen(strbuf)+1))
            {
                err = -EFAULT;
                break;
            }				 
            break;	  

        case GSENSOR_IOCTL_READ_SENSORDATA:
            data = (void __user *) arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;	  
            }
            N2DM_SetPowerMode(client,true);
            N2DM_ReadSensorData(client, strbuf, N2DM_BUFSIZE);
            if(copy_to_user(data, strbuf, strlen(strbuf)+1))
            {
                err = -EFAULT;
                break;	  
            }				 
            break;

        case GSENSOR_IOCTL_READ_GAIN:
            data = (void __user *) arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;	  
            }			

            if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D)))
            {
                err = -EFAULT;
                break;
            }				 
            break;

        case GSENSOR_IOCTL_READ_OFFSET:
            data = (void __user *) arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;	  
            }

            if(copy_to_user(data, &gsensor_offset, sizeof(struct GSENSOR_VECTOR3D)))
            {
                err = -EFAULT;
                break;
            }				 
            break;

        case GSENSOR_IOCTL_READ_RAW_DATA:
            data = (void __user *) arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;	  
            }
            N2DM_ReadRawData(client, strbuf);
            if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
            {
                err = -EFAULT;
                break;	  
            }
            break;	  

        case GSENSOR_IOCTL_SET_CALI:
            data = (void __user*)arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;	  
            }
            if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
            {
                err = -EFAULT;
                break;	  
            }
            if(atomic_read(&obj->suspend))
            {
                GSE_ERR("Perform calibration in suspend state!!\n");
                err = -EINVAL;
            }
            else
            {
                cali[N2DM_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
                cali[N2DM_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
                cali[N2DM_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
                err = N2DM_WriteCalibration(client, cali);			 
            }
            break;

        case GSENSOR_IOCTL_CLR_CALI:
            err = N2DM_ResetCalibration(client);
            break;

        case GSENSOR_IOCTL_GET_CALI:
            data = (void __user*)arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;	  
            }
            if((err = N2DM_ReadCalibration(client, cali)))
            {
                break;
            }

            sensor_data.x = cali[N2DM_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
            sensor_data.y = cali[N2DM_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
            sensor_data.z = cali[N2DM_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
            if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
            {
                err = -EFAULT;
                break;
            }		
            break;


        default:
            GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
            break;

    }

    return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations n2dm_fops = {
    .owner = THIS_MODULE,
    .open = n2dm_open,
    .release = n2dm_release,
    //.ioctl = n2dm_ioctl,
    .unlocked_ioctl = n2dm_unlocked_ioctl,

#ifdef CONFIG_COMPAT
    .compat_ioctl = n2dm_unlocked_compat_ioctl,
#endif

};
/*----------------------------------------------------------------------------*/
static struct miscdevice n2dm_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gsensor",
    .fops = &n2dm_fops,
};
/*----------------------------------------------------------------------------*/
static int n2dm_suspend(struct i2c_client *client, pm_message_t msg) 
{
/*
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);    
    int err = 0;
    GSE_FUN();    

    if(msg.event == PM_EVENT_SUSPEND)
    {   
        if(obj == NULL)
        {
            GSE_ERR("null pointer!!\n");
            return -EINVAL;
        }
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
		if(!obj->pickup_enable){
        	atomic_set(&obj->suspend, 1);
			err = N2DM_SetPowerMode(obj->client, false);
			if(err)
			{
				GSE_ERR("write power control fail!!\n");
				return -EINVAL;
			}
			N2DM_power(obj->hw, 0);
		}
#else
        atomic_set(&obj->suspend, 1);
        err = N2DM_SetPowerMode(obj->client, false);
        if(err)
        {
            GSE_ERR("write power control fail!!\n");
            return err;
        }        
        N2DM_power(obj->hw, 0);
#endif
    }
    return err;*/
	return 0;
}
/*----------------------------------------------------------------------------*/
static int n2dm_resume(struct i2c_client *client)
{
/*
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);        
    int err;
    GSE_FUN();

    if(obj == NULL)
    {
        GSE_ERR("null pointer!!\n");
        return -EINVAL;
    }

    N2DM_power(obj->hw, 1);
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
	err = N2DM_Init(obj->client, 0);
	if(err)
	{
		GSE_ERR("initialize client fail!!\n");
		return -EINVAL;        
	}
#endif
    err = N2DM_SetPowerMode(obj->client, obj->gsensor_enable);
    if(err)
    {
        GSE_ERR("initialize client fail! err code %d!\n", err);
        return err ;        
    }
    atomic_set(&obj->suspend, 0);
*/
    return 0;
}
/*Gionee BSP lizhi 20160524 add for CR01704330 begin replace for early_suspend*/
static void n2dm_lcm_off(void) 
{
   struct n2dm_i2c_data *obj = obj_i2c_data;   
   int err = 0; 

   GSE_FUN();    
	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
   		return;
	}
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
	printk("lizhi gsensor_enable %d,pickup_enable %d\n",obj->gsensor_enable,obj->pickup_enable);
	if(obj->pickup_enable){
		//n2dm_init_pickup_detect(obj->client);
		//n2dm_set_pickup_enable(obj->client, true);        
	}else{
		if((err = N2DM_SetPowerMode(obj->client, false)))
		{
			GSE_ERR("write power control fail!!\n");
			return;
		}
	}
#else
	if((err = N2DM_SetPowerMode(obj->client, false)))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}
#endif
	atomic_set(&obj->suspend, 1); 
	N2DM_power(obj->hw, 0);
}
static void n2dm_lcm_on(void)
{
	struct n2dm_i2c_data *obj = obj_i2c_data;         
	int err = 0;

	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
	printk("lizhi gsensor_enable %d,pickup_enable %d\n",obj->gsensor_enable,obj->pickup_enable);
	if(obj->pickup_enable){
		n2dm_set_pickup_enable(obj->client, false);
	}
	if((err = N2DM_Init(obj->client, 0)))
	{
		GSE_ERR("initialize client fail!!\n");
		return;        
	}
	if(obj->gsensor_enable){
		if((err = N2DM_SetPowerMode(obj->client, true)))
		{
			GSE_ERR("write power control fail!!\n");
			return;
		}
	}
#else
	if(obj->gsensor_enable){
		if((err = N2DM_SetPowerMode(obj->client, true)))
		{
			GSE_ERR("write power control fail!!\n");
			return;
		}
	}
#endif
	N2DM_power(obj->hw, 1);
	atomic_set(&obj->suspend, 0);    
}
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = NULL;
    int blank;
    GSE_FUN();
    evdata = data;
    if (event != FB_EVENT_BLANK)
        return 0;
    blank = *(int *)evdata->data;
    GSE_LOG("fb_notify(blank=%d)\n", blank);
    switch (blank) {
        case FB_BLANK_UNBLANK:
            n2dm_lcm_on();
            break;
        case FB_BLANK_POWERDOWN:
            n2dm_lcm_off();
            break;
        default:
            break;
    }
    return 0;
}
/*Gionee BSP lizhi 20160524 add for CR01704330 end replace for early_suspend*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int n2dm_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int n2dm_enable_nodata(int en)
{
    int res =0;
    bool power = false;

    if(1==en)
    {
        power = true;
    }
    if(0==en)
    {
        power = false;
    }
    res = N2DM_SetPowerMode(obj_i2c_data->client, power);
    if(res != N2DM_SUCCESS)
    {
        GSE_ERR("N2DM_SetPowerMode fail!\n");
        return -1;
    }
    GSE_LOG("n2dm_enable_nodata OK!\n");
    return 0;
}

static int n2dm_set_delay(u64 ns)
{
    int value =0;
    int sample_delay=0;
    int err;
    value = (int)ns/1000/1000;
    if(value <= 5)
    {
        sample_delay = N2DM_BW_200HZ;
    }
    else if(value <= 10)
    {
        sample_delay = N2DM_BW_100HZ;
    }
    else
    {
        sample_delay = N2DM_BW_50HZ;
    }
    mutex_lock(&n2dm_op_mutex);
    err = N2DM_SetBWRate(obj_i2c_data->client, sample_delay);
    if(err != N2DM_SUCCESS ) //0x2C->BW=100Hz
    {
        GSE_ERR("Set delay parameter error!\n");
    }
    mutex_unlock(&n2dm_op_mutex);
    if(value >= 50)
    {
        atomic_set(&obj_i2c_data->filter, 0);
    }
    else
    {					
        obj_i2c_data->fir.num = 0;
        obj_i2c_data->fir.idx = 0;
        obj_i2c_data->fir.sum[N2DM_AXIS_X] = 0;
        obj_i2c_data->fir.sum[N2DM_AXIS_Y] = 0;
        obj_i2c_data->fir.sum[N2DM_AXIS_Z] = 0;
        atomic_set(&obj_i2c_data->filter, 1);
    }

    GSE_LOG("n2dm_set_delay (%d)\n",value);
    return 0;
}

static int n2dm_get_data(int* x ,int* y,int* z, int* status)
{
    char buff[N2DM_BUFSIZE];
    N2DM_ReadSensorData(obj_i2c_data->client, buff, N2DM_BUFSIZE);

    sscanf(buff, "%x %x %x", x, y, z);		
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    return 0;
}


/*----------------------------------------------------------------------------*/
static int n2dm_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct i2c_client *new_client;
    struct n2dm_i2c_data *obj;
    struct acc_control_path ctl={0};
    struct acc_data_path data={0};
    /* Gionee BSP1 lizhi 20150713 add for motion-detect begin */
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
    struct hwmsen_object pickobj;
#endif
    /* Gionee BSP1 lizhi 20150713 add for motion-detect end */
    // Gionee lizhi 20150205 add for CR01444969 begin begin
#ifdef CONFIG_GN_BSP_MTK_DEVICE_CHECK
    struct gn_device_info gn_dev_info_accelerometer;
#endif
    // Gionee lizhi 20150205 add for CR01444969 beginend
    int err = 0;
    GSE_FUN();

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(struct n2dm_i2c_data));

    obj->hw = hw;

    if((err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
    {
        GSE_ERR("invalid direction: %d\n", obj->hw->direction);
        goto exit;
    }

    obj_i2c_data = obj;
    obj->client = client;
    new_client = obj->client;

    i2c_set_clientdata(new_client,obj);

    atomic_set(&obj->trace, 0);
    atomic_set(&obj->suspend, 0);

#ifdef CONFIG_N2DM_LOWPASS
    if(obj->hw->firlen > C_MAX_FIR_LENGTH)
    {
        atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
    }	
    else
    {
        atomic_set(&obj->firlen, obj->hw->firlen);
    }

    if(atomic_read(&obj->firlen) > 0)
    {
        atomic_set(&obj->fir_en, 1);
    }

#endif

    n2dm_i2c_client = new_client;	
    /* Gionee BSP1 lizhi 20150713 add for motion-detect begin */
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
    INIT_WORK(&obj->n2dm_eint_work, n2dm_eint_work_func);
#endif
    /* Gionee BSP1 lizhi 20150713 add for motion-detect end */

    if((err = N2DM_Init(new_client, 1)))
    {
        goto exit_init_failed;
    }

    if((err = misc_register(&n2dm_device)))
    {
        GSE_ERR("n2dm_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    if((err = n2dm_create_attr(&(n2dm_init_info.platform_diver_addr->driver))))
    {
        GSE_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    ctl.open_report_data= n2dm_open_report_data;
    ctl.enable_nodata = n2dm_enable_nodata;
    ctl.set_delay  = n2dm_set_delay;
    ctl.is_report_input_direct = false;
    ctl.is_support_batch = false;

    err = acc_register_control_path(&ctl);
    if(err)
    {
        GSE_ERR("register acc control path err\n");
        goto exit_kfree;
    }

    data.get_data = n2dm_get_data;
    data.vender_div = 1000;
    err = acc_register_data_path(&data);
    if(err)
    {
        GSE_ERR("register acc data path err\n");
        goto exit_kfree;
    }

    /* Gionee BSP1 lizhi 20150713 add for motion-detect begin */
#if defined(CONFIG_GN_BSP_PICK_UP_SENSOR)
    pickobj.self = obj;
    pickobj.polling = 0;
    pickobj.sensor_operate = n2dm_pickup_detect_operate;
    if((err = hwmsen_attach(ID_PICK_UP_GESTURE, &pickobj)))
    {
        GSE_ERR("attach fail = %d\n", err);
        goto exit_kfree;
    }
    n2dm_setup_eint(new_client);
    obj->pickup_enable = false;
#endif
    /* Gionee BSP1 lizhi 20150713 add for motion-detect end */
/*Gionee BSP lizhi 20160524 add for CR01704330 begin replace for early_suspend*/
	obj->fb_notifier.notifier_call = fb_notifier_callback;
    if (fb_register_client(&obj->fb_notifier)){
        printk("register fb_notifier fail!\n");
        goto exit_create_attr_failed;
    }
 /*Gionee BSP lizhi 20160524 add for CR01704330 end replace for early_suspend*/ 
    // Gionee lizhi 20150205 add for CR01444969 begin begin
#ifdef CONFIG_GN_BSP_MTK_DEVICE_CHECK
    memset(&gn_dev_info_accelerometer, 0, sizeof(struct gn_device_info));
    gn_dev_info_accelerometer.gn_dev_type = GN_DEVICE_TYPE_ACCELEROMETER;
    sprintf(gn_dev_info_accelerometer.name,"gn_n2dm");
    gn_set_device_info(gn_dev_info_accelerometer);
#endif
    // Gionee lizhi 20150205 add for CR01444969 beginend

    GSE_LOG("%s: OK\n", __func__);
    n2dm_init_flag = 0;    
    return 0;

exit_create_attr_failed:
    misc_deregister(&n2dm_device);
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(new_client);
exit_kfree:
    kfree(obj);
exit:
    GSE_ERR("%s: err = %d\n", __func__, err);
    n2dm_init_flag = -1;        
    return err;
}

/*----------------------------------------------------------------------------*/
static int n2dm_i2c_remove(struct i2c_client *client)
{
    int err = 0;	

    if((err = n2dm_delete_attr(&(n2dm_init_info.platform_diver_addr->driver))))
    {
        GSE_ERR("n2dm_delete_attr fail: %d\n", err);
    }

    if((err = misc_deregister(&n2dm_device)))
    {
        GSE_ERR("misc_deregister fail: %d\n", err);
    }
    n2dm_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}
/*----------------------------------------------------------------------------*/
static int  n2dm_remove(void)
{
    GSE_FUN();    
    N2DM_power(hw, 0);    
    i2c_del_driver(&n2dm_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/

static int  n2dm_local_init(void)
{
    GSE_FUN();
    N2DM_power(hw, 1);
    if(i2c_add_driver(&n2dm_i2c_driver))
    {
        GSE_ERR("add driver error\n");
        return -1;
    }
    if(-1 == n2dm_init_flag)
    {
        return -1;
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
static int __init n2dm_init(void)
{
    const char *name = "mediatek,n2dm";
    hw = get_accel_dts_func(name, hw);
    if(!hw)
        GSE_ERR("get dts info fail\n");
    acc_driver_add(&n2dm_init_info);
    return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit n2dm_exit(void)
{
    GSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(n2dm_init);
module_exit(n2dm_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("N2DM I2C driver");
MODULE_AUTHOR("lizhi@gionee.com");
