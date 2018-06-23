/* Copyright Statement:
 *
 * gionee wanght 2013-10-17 add for CR00921405 begin
 */

#ifndef __GN_DEVICE_CHECK_H__
#define __GN_DEVICE_CHECK_H__

//static int gn_device_debug_enable = 0;
#define GN_DEVICE_DEBUG(format, args...) do{ \
	if(0) \
	{\
		printk(KERN_EMERG format,##args);\
	}\
}while(0)

#define GN_DEVICE_NAME_LEN 50  //gionee yanggy modify for CR01243889
#define GN_DEVICE_VENDOR_LEN 20
#define GN_DEVICE_VERSION_LEN 20

enum gn_device_type
{
	GN_DEVICE_TYPE_LCD = 0,
	GN_DEVICE_TYPE_ACCELEROMETER, 
	GN_DEVICE_TYPE_TP, 
	//GN_DEVICE_TYPE_TP_IC,
	//GN_DEVICE_TYPE_TP_LCM,
	GN_DEVICE_TYPE_MAGNETIC_FIELD,
	GN_DEVICE_TYPE_GYROSCOPE,
	GN_DEVICE_TYPE_LIGHT ,
	GN_DEVICE_TYPE_PRESSURE,
	GN_DEVICE_TYPE_TEMPERATURE,
	GN_DEVICE_TYPE_PROXIMITY,
	GN_DEVICE_TYPE_CAP_KEY,
    GN_DEVICE_TYPE_MAIN_CAM,
    GN_DEVICE_TYPE_SUB_CAM,
    GN_DEVICE_TYPE_FINGER,
    GN_DEVICE_TYPE_FM,
	GN_DEVICE_TYPE_CPU,//chiva
	GN_DEVICE_TYPE_TOTAL, 
};

struct gn_device_info
{
	char     name[GN_DEVICE_NAME_LEN];				//gionee device name :such as akm8379
	char     vendor[GN_DEVICE_VENDOR_LEN];				//gionee device vendor:such as:sumsung
	char     version[GN_DEVICE_VERSION_LEN];			//gionee device version:such as:v1.01
	int	    gn_dev_type;								//enum gn_device_type
};

/* Gionee Qux. Declare for device drivers, once */
extern int gn_set_device_info(struct gn_device_info gn_dev_info);

//#ifdef CUSTOM_KERNEL_LCM
#define GN_DEVICE_LCD
//#endif

//#ifdef CUSTOM_KERNEL_ACCELEROMETER
#define GN_DEVICE_ACCELEROMETER
//#endif

//#ifdef CUSTOM_KERNEL_TOUCHPANEL
#define GN_DEVICE_TOUCHPANEL
//#endif

//#ifdef CUSTOM_KERNEL_MAGNETOMETER
#define GN_DEVICE_MAGNETIC_FIELD
//#endif

#ifdef CUSTOM_KERNEL_GYROSCOPE
//#define GN_DEVICE_GYROSCOPE
#endif

//#ifdef CUSTOM_KERNEL_ALSPS
#define GN_DEVICE_LIGHT
//#endif

//#ifdef CUSTOM_KERNEL_PRESSURE
//#define GN_DEVICE_PRESSURE
//#endif

//#ifdef CUSTOM_KERNEL_THERMOMETER
//#define GN_DEVICE_TEMPERATURE
//#endif

//#ifdef CUSTOM_KERNEL_PROXIMITY
#define GN_DEVICE_PROXIMITY
//#endif

//#ifdef CUSTOM_KERNEL_CAPKEY
//#define GN_DEVICE_CAP_KEY
//#endif

#define GN_DEVICE_MAIN_CAM    //yanggy modify for cam_dev check

#define GN_DEVICE_SUB_CAM     //yanggy modify for cam_dev check

//#ifdef CUSTOM_KERNEL_FM
//#define GN_DEVICE_FM
//#endif

//#define GN_DEVICE_CPU //chiva

//#define GN_DEVICE_FINGER //Gionee chenhaibing close for CR01714975

#endif // __GN_DEVICE_CHECK_H__

//gionee wanght 2013-10-17 add for CR00921405 end

