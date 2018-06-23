/*
 * Driver for CAM_CAL
 *
 *
 */
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>

#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "ofilm_s5k3m2otp.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>

//#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define CAM_CALDB pr_debug
#else
#define CAM_CALDB(x,...)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 0
extern u8 s5k3m2_ofilm_otp_Data[];   //add by chenqiang


/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "OFILM_CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
//static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, OV13853OTP_DEVICE_ID>>1)};

static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;

/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long CAM_CAL_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    int i = 0;
//	int j =0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
CAM_CALDB("[S24CAM_CAL] CAM_CAL_Ioctl\n");

	#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
	#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {   
		//get input structure address
                kfree(pBuff);
                CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
//    CAM_CALDB("[S24CAM_CAL] init Working buffer address 0x%8x  command is 0x%8x\n", (u32)pWorkingBuff, (u32)a_u4Command);

 
    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:    
            CAM_CALDB("[S24CAM_CAL] Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif            
            //i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[S24CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
#endif     

	     CAM_CALDB("chenqiang cpy otp data,offset:%x,length %d\n",ptempbuf->u4Offset,ptempbuf->u4Length);
       
    	 memcpy(pWorkingBuff,s5k3m2_ofilm_otp_Data+ptempbuf->u4Offset,ptempbuf->u4Length);
#if 1
	if(ptempbuf->u4Length<=10)
	{
	    for(i = 0; i< ptempbuf->u4Length ; i++)
	    {
	        CAM_CALDB("chenqiang cpy ofilm otp %s pWorkingBuff[%x]=0x%x\n",__func__,i,pWorkingBuff[i]);
	    }
	}else
	{
		for(i = 0; i<10 ; i++)
		{
			CAM_CALDB("chenqiang cpy ofilm otp %s pWorkingBuff[%x]=0x%x\n",__func__,i,pWorkingBuff[i]);
		}
	}
#endif

#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            
            break;
        default :
      	     CAM_CALDB("[S24CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        CAM_CALDB("[S24CAM_CAL] to user length %d \n", ptempbuf->u4Length);
//        CAM_CALDB("[S24CAM_CAL] to user  Working buffer address 0x%8x \n", (u32)pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            CAM_CALDB("[S24CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}

//Gionee <litao> <2014-01-07> add imx214 otp begin
#ifdef CONFIG_COMPAT

static int compat_get_otp_struct(
        COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
        stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uint_t i;
    compat_uptr_t p;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err = get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);
    return err;
}
/*
static int compat_put_otp_struct(
        COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
        stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    //compat_uint_t i;
    int err;

	err |= get_user(p, &data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
    return err;
}*/

static long CAM_CAL_Ioctl_Compat(
    struct file *filp, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
){

	long ret = 0;

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
    return -ENOTTY;

    switch (a_u4Command) {
	    case COMPAT_CAM_CALIOC_G_READ:
	    {
		    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
		    stCAM_CAL_INFO_STRUCT __user *data;
			
		    int err;

		    CAM_CALDB("leetao CAM_CAL_Ioctl_Compat COMPAT_CAM_CALIOC_G_READ\r\n");
		    data32 = compat_ptr(a_u4Param);
		    data = compat_alloc_user_space(sizeof(*data));
		    if (data == NULL)
		        return -EFAULT;

			err = compat_get_otp_struct(data32,data);
			if (err)
			{
				CAM_CALDB("leetao compat_get_otp_struct err=%d\r\n",err);
		        return err;
			}
		    ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
		    //err = compat_put_otp_struct(data32, data);

		    
		    //CAM_CALDB("leetao compat_put_otp_struct err=%d\r\n",err);
		    return ret;
		}
	}
	return ret;
}
#endif
//Gionee <litao> <2014-01-07> add imx214 otp end

static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("[S24CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[S24CAM_CAL] Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
    .unlocked_ioctl = CAM_CAL_Ioctl,
//Gionee <litao> <2014-01-07> add imx214 otp begin
#ifdef CONFIG_COMPAT
    .compat_ioctl = CAM_CAL_Ioctl_Compat,
#endif
//Gionee <litao> <2014-01-07> add imx214 otp end
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

	#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Allocate device no failed\n");

        return -EAGAIN;
    }
	#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Register device no failed\n");

        return -EAGAIN;
    }
	#endif

    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALDB("[S24CAM_CAL] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }
    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALDB("[S24CAM_CAL] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "OFILM_CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}


//////////////////////////////////////////////////////////////////////
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME,0},{}};   
#if 0 //test110314 Please use the same I2C Group ID as Sensor
static unsigned short force[] = {CAM_CAL_I2C_GROUP_ID, IMX214OTP_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#else
//static unsigned short force[] = {IMG_SENSOR_I2C_GROUP_ID, OV5647OTP_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#endif
//static const unsigned short * const forces[] = { force, NULL };              
//static struct i2c_client_address_data addr_data = { .forces = forces,}; 


#ifdef CONFIG_OF
    static const struct of_device_id CAM_CAL_i2c_of_ids[] = {
	{ .compatible = "mediatek,ofilm_cam_cal", },
	{}
    };
#endif


static struct i2c_driver CAM_CAL_i2c_driver = {
    .probe = CAM_CAL_i2c_probe,                                   
    .remove = CAM_CAL_i2c_remove,                           
//   .detect = CAM_CAL_i2c_detect, 
	.driver = {
		.name = CAM_CAL_DRVNAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = CAM_CAL_i2c_of_ids,
#endif
		},
    .id_table = CAM_CAL_i2c_id,                             
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, CAM_CAL_DRVNAME);
    return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {             
int i4RetValue = 0;
    CAM_CALDB("[S24CAM_CAL] 3m2 Attach I2C \n");
	pr_debug("CAM_CAL_i2c_probe \n");

//    spin_lock_init(&g_CAM_CALLock);

    //get sensor i2c client
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = S5K3M2OTP_DEVICE_ID>>1;
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    CAM_CALDB("[CAM_CAL] g_pstI2Cclient->addr = 0x%8x \n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterCAM_CALCharDrv();

    if(i4RetValue){
        CAM_CALDB("[S24CAM_CAL] register char device failed!\n");
        return i4RetValue;
    }


    CAM_CALDB("[S24CAM_CAL] Attached!! \n");
    return 0;                                                                                       
} 

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};


static int __init OFILM_CAM_CAL_i2C_init(void)
{
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALDB("failed to register S24CAM_CAL driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALDB("failed to register S24CAM_CAL driver, 2nd time\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit OFILM_CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(OFILM_CAM_CAL_i2C_init);
module_exit(OFILM_CAM_CAL_i2C_exit);

//MODULE_DESCRIPTION("OFILM_CAM_CAL driver");
//MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
//MODULE_LICENSE("GPL");
