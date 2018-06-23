/* Copyright Statement:
 * gionee songll 2013-10-17 add for CR00921405 begin
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
//#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>

//#include <mach/mt_reg_base.h>

#include <mach/irqs.h>
//#include <mach/eint.h>

#include <linux/gn_device_check.h>

#define GN_DEVICE_NAME	"gn_device_check"

struct gn_device_info dev_info[GN_DEVICE_TYPE_TOTAL];

extern u32 get_devinfo_with_index(u32 index);//chiva

int gn_set_device_info(struct gn_device_info gn_dev_info)
{
	strcpy(dev_info[gn_dev_info.gn_dev_type].name, gn_dev_info.name);
	strcpy(dev_info[gn_dev_info.gn_dev_type].vendor, gn_dev_info.vendor);
	strcpy(dev_info[gn_dev_info.gn_dev_type].version, gn_dev_info.version);
	return 0;
}
/*
static void dump_info(void)
{
	int i;
	for(i=0;i<GN_DEVICE_TYPE_TOTAL;i++)
	{
		GN_DEVICE_DEBUG("type=%d,name=%s,vendor=%s,version=%s\n",i,dev_info[i].name,dev_info[i].vendor,dev_info[i].version);
	}
}
*/
static ssize_t gn_device_name(struct device *dev,struct device_attribute *attr, char *buf_name)
{
	int len=0;
	#ifdef GN_DEVICE_LCD
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"LCD: %s\n", dev_info[GN_DEVICE_TYPE_LCD].name);
	#endif
        #ifdef GN_DEVICE_FINGER
        len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"Finger:%s\n", dev_info[GN_DEVICE_TYPE_FINGER].name);
        #endif

	#ifdef GN_DEVICE_ACCELEROMETER
	len+=snprintf(buf_name+len,GN_DEVICE_NAME_LEN, "G-sensor: %s\n", dev_info[GN_DEVICE_TYPE_ACCELEROMETER].name);
	#endif	
	#ifdef GN_DEVICE_TOUCHPANEL
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"TP: %s\n", dev_info[GN_DEVICE_TYPE_TP].name);
    	#endif
	#ifdef GN_DEVICE_MAGNETIC_FIELD
	len+=snprintf(buf_name+len,GN_DEVICE_NAME_LEN, "M-sensor: %s\n", dev_info[GN_DEVICE_TYPE_MAGNETIC_FIELD].name);
	#endif
	#ifdef GN_DEVICE_GYROSCOPE
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"GYRO: %s\n", dev_info[GN_DEVICE_TYPE_GYROSCOPE].name);
	#endif
	#ifdef GN_DEVICE_LIGHT
	len+=snprintf(buf_name+len,GN_DEVICE_NAME_LEN, "L/P-sensor:%s\n", dev_info[GN_DEVICE_TYPE_LIGHT].name);
	#endif
	#ifdef GN_DEVICE_PROXIMITY
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_PROXIMITY].name);
	#endif
	#ifdef GN_DEVICE_CAP_KEY
	len+=snprintf(buf_name+len, GN_DEVICE_NAME_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_CAP_KEY].name);
	#endif
//Gionee BSP1 yanggy 20140915 add for device check for CR01374728 begin
    #ifdef GN_DEVICE_MAIN_CAM
    len+=snprintf(buf_name+len,GN_DEVICE_NAME_LEN, "Main_CAM: %s\n",dev_info[GN_DEVICE_TYPE_MAIN_CAM].name);
    #endif
    #ifdef GN_DEVICE_SUB_CAM 
    len+=snprintf(buf_name+len,GN_DEVICE_NAME_LEN, "SUB_CAM: %s\n",dev_info[GN_DEVICE_TYPE_SUB_CAM].name);
    #endif
//Gionee BSP1 yanggy 20140915 add for device check for CR01374728 end
	return len;
}

static ssize_t gn_device_vendor(struct device *dev,struct device_attribute *attr, char *buf_vendor)
{
	int len=0;
	#ifdef GN_DEVICE_LCD
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_LCD].vendor);
	#endif
	#ifdef GN_DEVICE_FINGER
        len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_FINGER].vendor);
        #endif
	#ifdef GN_DEVICE_ACCELEROMETER
	len+=snprintf(buf_vendor+len,GN_DEVICE_VENDOR_LEN, "%s\n", dev_info[GN_DEVICE_TYPE_ACCELEROMETER].vendor);
	#endif
	#ifdef GN_DEVICE_TOUCHPANEL
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_TP].vendor);
	#endif
	#ifdef GN_DEVICE_MAGNETIC_FIELD
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_MAGNETIC_FIELD].vendor);
	#endif
	#ifdef GN_DEVICE_GYROSCOPE
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_GYROSCOPE].vendor);
	#endif
	#ifdef GN_DEVICE_LIGHT
	len+=snprintf(buf_vendor+len,GN_DEVICE_VENDOR_LEN, "%s\n", dev_info[GN_DEVICE_TYPE_LIGHT].vendor);
	#endif
	#ifdef GN_DEVICE_PROXIMITY
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_PROXIMITY].vendor);
	#endif
	#ifdef GN_DEVICE_CAP_KEY
	len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_CAP_KEY].vendor);
	#endif
//Gionee BSP1 yanggy 20140915 add for device check for CR01374728 begin
    #ifdef GN_DEVICE_MAIN_CAM 
    len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_MAIN_CAM].vendor);
    #endif
    #ifdef GN_DEVICE_SUB_CAM 
    len+=snprintf(buf_vendor+len, GN_DEVICE_VENDOR_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_SUB_CAM].vendor);
    #endif
//Gionee BSP1 yanggy 20140915 add for device check for CR01374728 end
	return len;	
}

static ssize_t gn_device_version(struct device *dev,struct device_attribute *attr, char *buf_version)
{	
	int len=0;
	#ifdef GN_DEVICE_LCD
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_LCD].version);
	#endif
	#ifdef GN_DEVICE_FINGER
        len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_FINGER].version);
        #endif
	#ifdef GN_DEVICE_ACCELEROMETER
	len+=snprintf(buf_version+len,GN_DEVICE_VERSION_LEN, "%s\n", dev_info[GN_DEVICE_TYPE_ACCELEROMETER].version);
	#endif
	#ifdef GN_DEVICE_TOUCHPANEL
	len+=snprintf(buf_version+len,GN_DEVICE_VERSION_LEN, "%s\n", dev_info[GN_DEVICE_TYPE_TP].version);
	#endif
	#ifdef GN_DEVICE_MAGNETIC_FIELD
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_MAGNETIC_FIELD].version);
	#endif
	#ifdef GN_DEVICE_GYROSCOPE
	len+=snprintf(buf_version+len,GN_DEVICE_VERSION_LEN, "%s\n", dev_info[GN_DEVICE_TYPE_GYROSCOPE].version);
	#endif
	#ifdef GN_DEVICE_LIGHT
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_LIGHT].version);
	#endif
	#ifdef GN_DEVICE_PROXIMITY
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_PROXIMITY].version);
	#endif
	#ifdef GN_DEVICE_CAP_KEY
	len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_CAP_KEY].version);
	#endif 
//Gionee BSP1 yanggy 20140915 add for device check for CR01374728 begin
    #ifdef GN_DEVICE_MAIN_CAM 
    len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_MAIN_CAM].version);
    #endif
    #ifdef GN_DEVICE_SUB_CAM 
    len+=snprintf(buf_version+len, GN_DEVICE_VERSION_LEN,"%s\n", dev_info[GN_DEVICE_TYPE_SUB_CAM].version);
    #endif
//Gionee BSP1 yanggy 20140915 add for device check for CR01374728 end
	return len;
}

//Gionee yanjun CR01655428 . Needn't write permission. begin
DEVICE_ATTR(name,     S_IRUSR | S_IRUGO, gn_device_name, NULL);
DEVICE_ATTR(vendor,     S_IRUSR | S_IRUGO, gn_device_vendor, NULL);
DEVICE_ATTR(version,     S_IRUSR | S_IRUGO, gn_device_version, NULL);
//Gionee yanjun CR01655428 . Needn't write permission. end

static struct device_attribute *gn_device_attr_list[] =
{
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_version,
};

static int gn_device_create_attr(struct device *dev) 
{
	int idx, err = 0;
	int num = (int)(sizeof(gn_device_attr_list)/sizeof(gn_device_attr_list[0]));
	GN_DEVICE_DEBUG("%s\n", __func__);
	if(!dev)
	{
		return -EINVAL;
	}	

	for(idx = 0; idx < num; idx++)
	{
		if((err = device_create_file(dev, gn_device_attr_list[idx])))
		{            
			GN_DEVICE_DEBUG("device_create_file (%s) = %d\n", gn_device_attr_list[idx]->attr.name, err);        
			break;
		}
	}

	return err;
}


static int gn_device_pdrv_probe(struct platform_device *pdev)
{
	int err=0;
	GN_DEVICE_DEBUG("%s\n", __func__);
	if(gn_device_create_attr(&pdev->dev) != 0)
	{
		GN_DEVICE_DEBUG("%s:unable to create attributes!!\n",__func__);
		goto exit_create_attr_failed;
	}

	return 0;
exit_create_attr_failed:
	GN_DEVICE_DEBUG("%s: err = %d\n", __func__, err);        
	return err;
}

/* should never be called */
static int gn_device_pdrv_remove(struct platform_device *pdev)
{
	GN_DEVICE_DEBUG("%s\n", __func__);
	return 0;
}

static struct platform_device gn_device_check_pdev = {
	.name	= "gn_device_check",
	.id	= -1,
};

#ifdef CONFIG_OF
static const struct of_device_id gn_device_check[] = {
	{ .compatible = "mediatek,gn_dev_check", },
	{},
};
#endif

static struct platform_driver gn_device_pdrv = {
	.probe		= gn_device_pdrv_probe,
	.remove		= gn_device_pdrv_remove,
	.driver		= {
		.name	= GN_DEVICE_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = gn_device_check,
#endif
	},
};

static int __init gn_device_mod_init(void)
{
	int ret;
	GN_DEVICE_DEBUG("%s\n", __func__);

	ret = platform_device_register(&gn_device_check_pdev);
	if (ret != 0){
		GN_DEVICE_DEBUG("register device failed (%d)\n", ret);
		return ret;
	}	

	ret = platform_driver_register(&gn_device_pdrv);
	if (ret) {
		GN_DEVICE_DEBUG("register driver failed (%d)\n", ret);
		return ret;
	}
	return 0;
}

/* should never be called */
static void __exit gn_device_mod_exit(void)
{
	GN_DEVICE_DEBUG("%s\n", __func__);
}

//module_init core_initcall
module_init(gn_device_mod_init);
module_exit(gn_device_mod_exit);

MODULE_AUTHOR("wanght <wanght@gionee.com>");
MODULE_DESCRIPTION("gn_device_check Driver v0.1");
MODULE_LICENSE("GPL");
//gionee songll 2013-10-17 add for CR00921405 end
