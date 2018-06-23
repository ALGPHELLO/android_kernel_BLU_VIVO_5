#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
//#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k3m2_ofilm_Sensor.h"

#define PFX1 "S5K3M2_ofilm_otp"

#define S5K3M2_OTP_DEBUG
#ifdef S5K3M2_OTP_DEBUG
#define SENSORDB(fmt,arg...) pr_debug(PFX1 "[%s] " fmt, __func__, ##arg)
//xlog_printk(ANDROID_LOG_DEBUG , __FUNCTION__, fmt, ##arg)
#else
#define SENSORDB(fmt,arg...)
#endif

#define S5K3M2_OFILM_OTP_SIZE  0x0CDF // 0x0763+1 

u8 s5k3m2_ofilm_otp_Data[S5K3M2_OFILM_OTP_SIZE];   //add by chenqiang

static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;

static kal_uint16 sensor_addr = 0xA0;
//static kal_uint16 sensor_init = 0;

static kal_uint16 golden_r = 0x25c;  //check with M/H,20150115,0x24b
static kal_uint16 golden_b = 0x267;  //check with M/H,20150115,0x23f
//static kal_uint16 golden_g = 0x400;  //check with M/H,20150115,0x????

static kal_uint16 current_r = 0;
static kal_uint16 current_b = 0;
static kal_uint16 current_g = 0;

/*
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, sensor_addr);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}*/

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	iWriteRegI2C(pusendcmd , 4, sensor_addr);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte=0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,sensor_addr);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	iWriteRegI2C(pusendcmd , 3, sensor_addr);
}


#define USHORT		unsigned short
#define UINT		unsigned int
#define BYTE   		unsigned char

#define Sleep(ms) mdelay(ms)

BYTE get_ofilm_module_id(void)
{
	BYTE flag = 0, i = 0;

	USHORT tempVal[20] = {0};

        flag=read_cmos_sensor_8(0x0000);
        if (flag != 0x01)
        {  
		SENSORDB("read ofilm module info flag fail");            
            	return 0;  // 
        }
	for(i = 0; i < 6; i++ )
	{
			tempVal[i] = read_cmos_sensor_8(0x0001 + i);
			SENSORDB("chenqiang ofilm module info addr = 0x%x: %x\n", 0x0001+i, tempVal[i]);

	}

	return tempVal[4];
}


static BYTE get_awb_value(void)
{
	//BYTE flag = 0, i = 0, j = 0;
	BYTE i = 0;

	USHORT tempVal[64] = {0}, rsum = 0, csum = 0;


	if((read_cmos_sensor_8(0x0a04 ) & 0xc0)== 0x40)
	{
		rsum = read_cmos_sensor_8(0x0a05);
		SENSORDB("yingjie0:0x0a04= %x: 0x0a05=%x\n", read_cmos_sensor_8(0x0a04), rsum);                

		for(i = 0; i < 14; i++ )
		{
			tempVal[i] = read_cmos_sensor_8(0x0a06 + i);
			SENSORDB("yingjie0: %x: %x\n", 0x0a06+ i, tempVal[i]);
			csum += tempVal[i];
			Sleep(5);
		}
	}
	else if((read_cmos_sensor_8(0x0a24 ) & 0xc0)== 0x40)
	{
		rsum = read_cmos_sensor_8(0x0a25);
		SENSORDB("yingjie1:0x0a24= %x: 0x0a25=%x\n", read_cmos_sensor_8(0x0a24), rsum);              
		for(i = 0; i < 14; i++ )
		{
			tempVal[i] = read_cmos_sensor_8(0x0a26 + i);
			SENSORDB("yingjie1: %x: %x\n", 0x0a26 + i, tempVal[i]);
			csum += tempVal[i];
			Sleep(5);
		}
	}



	csum = csum % 255 + 1;

	if(csum != rsum)
	{
		SENSORDB("check sum fail: %x, %x\n", csum, rsum);
		return 0;
	}

	SENSORDB("module id: %x\n", tempVal[0]);

	current_r = (tempVal[6] & 0x03) << 8 | tempVal[7];
	current_b = (tempVal[8] & 0x03) << 8 | tempVal[9];
	//current_g = (tempVal[10] & 0x03) << 8 | tempVal[11];
        current_g = 0x400;  //20150123

	SENSORDB("r: %x; g: %x; b: %x\n", current_r, current_g, current_b);

	return 1;
}

static bool otp_lsc_update(void)
{

	write_cmos_sensor_8(0x0B00,0x01);
	write_cmos_sensor(0x3058,0x0900);

	SENSORDB("lsc update pass\n");
	
	return 1;
}

static bool otp_wb_update(void)
{
	BYTE ret = 0;

	USHORT R_gain,B_gain,G_gain;	
	UINT G_gain_R, G_gain_B;

	ret = get_awb_value();

	if(ret == 0 || current_r == 0 || current_g == 0 || current_b == 0)
		return 0;

	if(current_b < golden_b)
	{
		if (current_r < golden_r)
		{
			G_gain = 0x100;
			B_gain = 0x100 * golden_b / current_b;
			R_gain = 0x100 * golden_r / current_r;
		}
		else
		{
			R_gain = 0x100;
			G_gain = 0x100 * current_r / golden_r;
			B_gain = G_gain * golden_b / current_b;
		}
	}
	else
	{
		if (current_r < golden_r)
		{
			B_gain = 0x100;
			G_gain = 0x100 * current_b / golden_b;
			R_gain = G_gain * golden_r / current_r;
		}
		else
		{
			G_gain_B = current_b * 0x100 / golden_b;
			G_gain_R = current_r * 0x100 / golden_r;
			
			if(G_gain_B > G_gain_R )
			{
				B_gain = 0x100;
				G_gain = G_gain_B;
				R_gain = G_gain_B * golden_r / current_r;
			}
			else
			{
				R_gain = 0x100;
				G_gain = G_gain_R;
				B_gain = G_gain_R * golden_b / current_b;
			}
		}
	}

	SENSORDB("r: %d, b: %d, g: %d\n", R_gain, B_gain, G_gain);
	
	write_cmos_sensor(0x020E, G_gain);//GR
	write_cmos_sensor(0x0210, R_gain);//R
	write_cmos_sensor(0x0212, B_gain);//B
	write_cmos_sensor(0x0214, G_gain);//GB

	write_cmos_sensor_8(0x3056, 0x01);//??

	SENSORDB("awb update pass\n");

	return 1;
}

bool s5k3m2_ofilm_otp_update(kal_uint16 address)
{
	BYTE ret = 0;

	sensor_addr = address;
	
	ret = otp_wb_update();

	if(ret == 1)
		otp_lsc_update();
	
	return 1;   
}

bool s5k3m2_ofilm_otp_read(void)
{
	unsigned int i;

	//BYTE ret = 0;
	sensor_addr = sensor_addr;

	memset(s5k3m2_ofilm_otp_Data,0,sizeof(s5k3m2_ofilm_otp_Data));
	for(i = 0; i<S5K3M2_OFILM_OTP_SIZE; i++)
	{		
		s5k3m2_ofilm_otp_Data[i]= read_cmos_sensor_8(i);
//		SENSORDB("chenqiang ofilm read_3m2_otp address=%x,data=%x\r\n",i,s5k3m2_ofilm_otp_Data[i]);
	}
	
	for(i = 6; i<(0x17+10); i++)  //add by malp
	{		
	SENSORDB("chenqiang ofilm read_3m2_otp address=%x,data=%x\r\n",i,s5k3m2_ofilm_otp_Data[i]);
	}
	
	return 1;   
}
//pdaf code begin
#define S5K3M2_OFILM_PDAF_PROC_SIZE  1404

#define S5K3M2_OFILM_PDAF_PROC_ADD 0x0763

static bool s5k3m2_read_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size )
{
	int i = 0;
	int j = 0;
 
	for(i = 0; i < S5K3M2_OFILM_PDAF_PROC_SIZE; i++) {
        	 data[j++] = s5k3m2_ofilm_otp_Data[S5K3M2_OFILM_PDAF_PROC_ADD + i];
			 SENSORDB("s5k3m2_read_eeprom address=%x,data=%x\r\n",i,s5k3m2_ofilm_otp_Data[i]);

    	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}
bool s5k3m2_ofilm_read_otp_pdaf_data( kal_uint16 addr, BYTE* data, kal_uint32 size)
{
	
	SENSORDB("sunny read_otp_pdaf_data enter");
//	if(!get_done || last_size != size || last_offset != addr) {
		//if(!_read_eeprom(addr, eeprom_data, size)){
		if(!s5k3m2_read_eeprom(addr, data, size)){
			get_done = 0;
			last_size = 0;
			last_offset = 0;
			SENSORDB("read_otp_pdaf_data fail");
			return false;
		}
//	}
	//memcpy(data, eeprom_data, size);
	SENSORDB("sunny read_otp_pdaf_data end");
    return true;
}
//pdaf code end
