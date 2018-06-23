/*
 * AudDrv_NXP.c
 *
 * Device Driver for sound speaker external PA
 * (PA IC set used YAMAHA NXPExtSpkC)
 * 
 * Authors:
 * Weiwei <weiwei@gionee.com>
 * date:2013.3.27
 *
 * (C) Copyright Gionee Communication Equipment Co., Ltd.ShenZhen
 * License: GNU GPL
 *
 *************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************
 */

#include <linux/input.h>	/* BUS_I2C */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/major.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/i2c-dev.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>

#include <linux/gpio.h>
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>


#include "AudDrv_NXP.h"





#define SY_TAG "[sound_NXP] "
#define SY_ERR(fmt, args...)  printk(KERN_ERR SY_TAG "%s %d : "fmt, __FUNCTION__, __LINE__, ##args )
#define SY_FUN(f)    printk(KERN_ERR SY_TAG"%s   %d \n", __FUNCTION__, __LINE__)

#define SY_DEBUG
#if defined(SY_DEBUG)
#define SY_LOG(fmt,args...)  printk(KERN_ERR SY_TAG "%s(%d):" fmt, __FUNCTION__, __LINE__, ##args)
#else
#define SY_LOG(arg...) 
#endif

#define I2C_MASTER_CLOCK       400
#define I2C_RETRY_CNT (3)
#define MTK_I2C_DMA_THRES (8)

#define G_BUFF_SIZE (1024)
#define RW_BUFFER_LENGTH (256)
#define I2C_MINORS	256

#define MAX_BUFFER_SIZE	255

#define GPIO_EXT_SPKAMP_ENABLE     (GPIO61 | 0x80000000) 

static struct i2c_client *NXPExtSpk_i2c_client = NULL;
//static u8 *I2CDMABuf_va = NULL;
//static u32 I2CDMABuf_pa = NULL;

static void *TfaI2CDMABuf_va;
static dma_addr_t TfaI2CDMABuf_pa;


//static struct i2c_board_info __initdata i2c_NXPExtSpk = { I2C_BOARD_INFO(NXPExtSpk_AUDIO_DEVICE, (NXPExtSpk_DEVICE_ADDR))};
static struct miscdevice AudDrv_NXPExtSpk_audio_device;
static int  gn_NXPExtSpk_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id);
static int  gn_NXPExtSpk_i2c_remove(struct i2c_client *client);

#if 1
static int nxp_i2c_master_recv(const struct i2c_client *client, char *buf, int count)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.timing = I2C_MASTER_CLOCK;
	//msg.flags = client->flags & I2C_M_TEN;
	msg.addr = client->addr ;
	//Gionee huangzhuolin 20160226 @ modify for CR01641892 begin
	msg.flags = I2C_M_RD;
	//Gionee huangzhuolin 20160226 @ modify for CR01641892 end
	msg.len = count;
	msg.ext_flag = client->ext_flag;
	msg.buf = (char *)buf;

	if (count <= 8)
		msg.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG );
	else
		msg.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG);


	ret = i2c_transfer(adap, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg received), return #bytes received,
	 * else error code.
	 */
	return (ret == 1) ? count : ret;

}

static int nxp_i2c_master_send(const struct i2c_client *client, const char *buf, int count)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;

	msg.timing = I2C_MASTER_CLOCK;
	msg.addr = client->addr ;

	if (count <= 8)
		msg.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG );
	else
		msg.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG);

	msg.flags = 0;

	msg.len = count;
	msg.buf = (char *)buf;
	//msg.ext_flag = client->ext_flag;
	ret = i2c_transfer(adap, &msg, 1);

	/*
	 * If everything went ok (i.e. 1 msg transmitted), return #bytes
	 * transmitted, else error code.
	 */
	return (ret == 1) ? count : ret;

}
#endif

static ssize_t i2cdev_read(struct file *fp,  char __user *data, size_t count, loff_t *offset)
{
	int i = 0;
	char *tmp;
	char *TfaI2CDMABuf = (char *)TfaI2CDMABuf_va;
	int ret = 0;

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	if (count <= 8)
		ret = nxp_i2c_master_recv(NXPExtSpk_i2c_client, tmp, count);
	else {
		ret = nxp_i2c_master_recv(NXPExtSpk_i2c_client, (char *)TfaI2CDMABuf_pa, count);
		for (i = 0; i < count; i++)
			tmp[i] = TfaI2CDMABuf[i];
	}

	if (ret >= 0)
		ret = copy_to_user(data, tmp, count) ? (-EFAULT) : ret;
	kfree(tmp);

	return ret;
}



static ssize_t i2cdev_write(struct file *fp, const char __user *data, size_t count, loff_t *offset)
{
	int i = 0;
	int ret;
	char *tmp;
	char *TfaI2CDMABuf = (char *)TfaI2CDMABuf_va;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;
	if (copy_from_user(tmp, data, count)) {
		kfree(tmp);
		return -EFAULT;
	}


	for (i = 0;  i < count; i++)
		TfaI2CDMABuf[i] = tmp[i];

	if (count <= 8)
		ret = nxp_i2c_master_send(NXPExtSpk_i2c_client, tmp, count);
	else
		ret = nxp_i2c_master_send(NXPExtSpk_i2c_client, (char *)TfaI2CDMABuf_pa, count);

	kfree(tmp);
	return ret;
}

static long i2cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = NXPExtSpk_i2c_client;
    //Gionee zhangliu 20150826 modify for CR01544616, begin
    #ifdef CONFIG_GN_BSP_AUDIO_SUPPORT
    //SY_LOG("ioctl, cmd=0x%02x, arg=0x%02lx\n",cmd, arg);
    #else
      //SY_LOG("ioctl, cmd=0x%02x, arg=0x%02lx\n",cmd, arg);
    #endif
    //Gionee zhangliu 20150826 modify for CR01544616, end
    switch (cmd) {
        case I2C_SLAVE:
        case I2C_SLAVE_FORCE:
            /* NOTE:  devices set up to work with "new style" drivers
             * can't use I2C_SLAVE, even when the device node is not
             * bound to a driver.  Only I2C_SLAVE_FORCE will work.
             *
             * Setting the PEC flag here won't affect kernel drivers,
             * which will be using the i2c_client node registered with
             * the driver model core.  Likewise, when that client has
             * the PEC flag already set, the i2c-dev driver won't see
             * (or use) this setting.
             */
            if ((arg > 0x3ff) ||
                    (((client->flags & I2C_M_TEN) == 0) && arg > 0x7f))
            {
                SY_LOG("ioctl return -EINVAL");
             //   return -EINVAL;
            }        
            client->addr = arg;
            return 0;
        default:
            /* NOTE:  returning a fault code here could cause trouble
             * in buggy userspace code.  Some old kernel bugs returned
             * zero in this case, and userspace code might accidentally
             * have depended on that bug.
             */
            //return -ENOTTY;
			return 0;
    }
    return 0;
}

static int i2cdev_open(struct inode *inode, struct file *file)
{
    SY_FUN();
    file->private_data = NXPExtSpk_i2c_client;
    return 0;
}

static int i2cdev_release(struct inode *inode, struct file *file)
{
    //struct i2c_client *client = file->private_data;
    SY_FUN();

    return 0;
}

static const struct i2c_device_id gn_NXPExtSpk_i2c_id[] = {
    {NXPExtSpk_AUDIO_DEVICE,0 },
    { }
};

static const struct of_device_id pa_of_match[] = {
	{.compatible = "mediatek,SMART_PA_TFA9890"},
	{},
};

MODULE_DEVICE_TABLE(i2c, gn_NXPExtSpk_i2c_id);


static struct i2c_driver gn_NXPExtSpk_i2c_driver = {
    .probe		= gn_NXPExtSpk_i2c_probe,
    .remove		= gn_NXPExtSpk_i2c_remove,
    /*
    .detect		= gn_NXPExtSpk_i2c_detect,
    .suspend 	= gn_NXPExtSpk_i2c_suspend,
    .resume		= gn_NXPExtSpk_i2c_resume,
    */
    .id_table 	= gn_NXPExtSpk_i2c_id,
    .driver	= {
        .name	= NXPExtSpk_AUDIO_DEVICE,
	    .of_match_table = pa_of_match,
    },
};



static int  gn_NXPExtSpk_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{ 
    int ret=0;
    SY_FUN();

    NXPExtSpk_i2c_client = client;
	printk("%s:: enter addriess= 0x%x\n",__func__,client->addr);
	mt_set_gpio_mode(GPIO_EXT_SPKAMP_ENABLE, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_EXT_SPKAMP_ENABLE, GPIO_DIR_OUT); 
	
	mt_set_gpio_out(GPIO_EXT_SPKAMP_ENABLE, GPIO_OUT_ZERO);
	msleep(2);
	mt_set_gpio_out(GPIO_EXT_SPKAMP_ENABLE, GPIO_OUT_ONE);
    msleep(2);
	mt_set_gpio_out(GPIO_EXT_SPKAMP_ENABLE, GPIO_OUT_ZERO);
    msleep(10);

    
	TfaI2CDMABuf_va = (char *)dma_alloc_coherent(&client->dev, 4096, (dma_addr_t *) &TfaI2CDMABuf_pa, GFP_KERNEL);
	if (TfaI2CDMABuf_va == NULL) {
		return -1;
	}

   
    // register MISC device
    if ((ret = misc_register(&AudDrv_NXPExtSpk_audio_device)))
    {
        printk("AudDrv_probe misc_register Fail:%d \n", ret);
        return ret;
    }

    return 0;
}
/*
static int gn_NXPExtSpk_i2c_suspend(struct device *dev)
{	
    SY_LOG("NXPExtSpk_i2c_suspend\n");
    return 0;
}

static int gn_NXPExtSpk_i2c_resume(struct device *dev)
{
    SY_LOG("NXPExtSpk_i2c_resume\n");  
    return 0;
}

static int gn_NXPExtSpk_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    SY_FUN();
    strcpy(info->type, NXPExtSpk_AUDIO_DEVICE);
    return 0;
}  */

static int  gn_NXPExtSpk_i2c_remove(struct i2c_client *client)
{
    //int err;
    NXPExtSpk_i2c_client = NULL;
	i2c_del_driver(&gn_NXPExtSpk_i2c_driver);
	if (TfaI2CDMABuf_va != NULL) {
		dma_free_coherent(&client->dev, 4096, TfaI2CDMABuf_va, TfaI2CDMABuf_pa);
		TfaI2CDMABuf_va = NULL;
		TfaI2CDMABuf_pa = 0;
	}
    return 0;
}


static const struct file_operations i2cdev_fops = {
    .owner		= THIS_MODULE,
    .llseek		= no_llseek,
    .read		= i2cdev_read,
    .write		= i2cdev_write,
    .unlocked_ioctl	= i2cdev_ioctl,
    .open		= i2cdev_open,
    .release	= i2cdev_release,
};

static struct miscdevice AudDrv_NXPExtSpk_audio_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "i2c-3",
    .fops = &i2cdev_fops,
};


static int gn_NXPExtSpk_probe(struct platform_device *pdev) 
{
    //int ret = 0;
    SY_FUN();
	printk("%s:: enter\n",__func__);

  

    if(i2c_add_driver(&gn_NXPExtSpk_i2c_driver)){
        SY_ERR("add i2c driver error\n");
		printk("%s:: NXP probe error - ----- \n",__func__);
        return -1;
    }
	return 0;
}

static int gn_NXPExtSpk_remove(struct platform_device *pdev)
{
    SY_ERR(" mtk sound driver remove! \n");  

    return 0;
}

static struct platform_driver gn_NXPExtSpk_driver = {
    .probe      = gn_NXPExtSpk_probe,
    .remove     = gn_NXPExtSpk_remove,    
    .driver     = {
        .name  ="sound_speaker",
    }
};


static struct platform_device gn_NXPExtSpk_device = {
    .name = "sound_speaker",
    .id = -1,
    .dev = {
    }
};


static __init int gn_NXPExtSpk_init(void)
{		
    SY_FUN();
	printk("%s:: enter\n",__func__);
    //int ret = 0;
    //i2c_register_board_info(4, &i2c_NXPExtSpk, 1);

    if(platform_driver_register(&gn_NXPExtSpk_driver)){
        SY_ERR("failed to register NXPExtSpk sound driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&gn_NXPExtSpk_device)){
        SY_ERR("failed to register NXPExtSpk sound device\n");
        return -ENODEV;
    }	

    return 0; 
}

static __exit void gn_NXPExtSpk_exit(void)
{		
    SY_FUN();
    platform_driver_unregister(&gn_NXPExtSpk_driver);
    platform_device_unregister(&gn_NXPExtSpk_device);

}

module_init(gn_NXPExtSpk_init);
module_exit(gn_NXPExtSpk_exit);

MODULE_DESCRIPTION("AudDrv_NXP");
MODULE_AUTHOR("Weiwei <weiwei@gionee.com>");
MODULE_LICENSE("GPL");


