/*************************************************************************************************
S5K5E2YA_OFILM_otp.c
---------------------------------------------------------
OTP Application file From Truly for S5K5E2YA_OFILM_
2013.01.14
---------------------------------------------------------
NOTE:
The modification is appended to initialization of image sensor. 
After sensor initialization, use the function , and get the id value.
bool ofilm_otp_wb_update(BYTE zone)
and
bool ofilm_otp_lenc_update(), 
then the calibration of AWB and LSC will be applied. 
After finishing the OTP written, we will provide you the ofilm_golden_rg and ofilm_golden_bg settings.
**************************************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>  
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

//#include <linux/xlog.h>
#include <linux/types.h>
#include "kd_camera_typedef.h"


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k5e2yamipiraw_Sensor.h"
#include "s5k5e2yamipiraw_Camera_Sensor_para.h"
#include "s5k5e2yamipiraw_CameraCustomized.h"

#define PFX "s5k5e2ya_otp"

#define S5K5E2YA_OFILM_DEBUG
#ifdef S5K5E2YA_OFILM_DEBUG
#define LOG_TAG (__FUNCTION__)
//#define SENSORDB(fmt,arg...) 
#define SENSORDB(fmt,arg...) pr_debug(PFX "[%s] " fmt, __func__, ##arg)
#else
#define SENSORDB(fmt,arg...)  
#endif

#define USHORT unsigned short
#define BYTE unsigned char
#define Sleep(ms) mdelay(ms)

#define OFILM_INVALID_OTP        0x01

#define OFILM_GAIN_DEFAULT       0x0100
#define OFILM_GAIN_GREEN1_ADDR   0x020E
#define OFILM_GAIN_BLUE_ADDR     0x0212
#define OFILM_GAIN_RED_ADDR      0x0210
#define OFILM_GAIN_GREEN2_ADDR   0x0214



#if 1

#define OFILM_GRG_TYPICAL   0x300     //0x31C        //0x3A0  0x2a1    //673,need check from module house,20140514
#define OFILM_BG_TYPICAL    0x2F5     //0x2E1       //0x2D8 0x23f    //575,need check from module house,20140514
#else

#define OFILM_GRG_TYPICAL 0x001    //673,need check from module house,20140514
#define OFILM_BG_TYPICAL 0x002    //575,need check from module house,20140514
#endif

kal_uint32 OFILM_tRG_Ratio_typical = OFILM_GRG_TYPICAL;
kal_uint32 OFILM_tBG_Ratio_typical = OFILM_BG_TYPICAL;


USHORT ofilm_golden_r;
USHORT ofilm_golden_gr;
USHORT ofilm_golden_gb;
USHORT ofilm_golden_b;

USHORT ofilm_current_r;
USHORT ofilm_current_gr;
USHORT ofilm_current_gb;
USHORT ofilm_current_b;

kal_uint32 ofilm_r_ratio = 0;
kal_uint32 ofilm_b_ratio = 0;

static kal_uint16 s5k5e2yamipiraw_ofilm_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    kdSetI2CSpeed(400);
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, 0x20);

	return get_byte;
}

static void s5k5e2yamipiraw_ofilm_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    kdSetI2CSpeed(400);
	iWriteRegI2C(pu_send_cmd, 3, 0x20);
}

/*************************************************************************************************
* Function    :  ofilm_start_read_otp
* Description :  before read otp , set the reading block setting  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  0, reading block setting err
                 1, reading block setting ok 
**************************************************************************************************/
bool ofilm_start_read_otp(BYTE zone)
{
	//BYTE val = 0;
	//int i;
   	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0x04);//make initial state
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A02, zone);   //Select the page to write by writing to 0xD0000A02 0x00~0x0F
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0x01);   //Enter read mode by writing 01h to 0xD0000A00
	
    Sleep(2);//wait time > 47us

	return 1;
}

/*************************************************************************************************
* Function    :  ofilm_stop_read_otp
* Description :  after read otp , stop and reset otp block setting  
**************************************************************************************************/
void ofilm_stop_read_otp(void)
{
   	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0x04);//make initial state
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0x00);//Disable NVM controller
}

/*************************************************************************************************
* Function    :  ofilm_get_otp_AWB_flag
* Description :  get otp AWB_WRITTEN_FLAG  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE], if 0x00 , this type has valid or empty otp data, otherwise, invalid otp data
**************************************************************************************************/
BYTE ofilm_get_otp_AWB_flag(BYTE zone)
{
    BYTE flag_AWB = 0x00;
    BYTE Flag1=0,Flag0=0;
    
    if(zone>3||zone<2)
    {
    	//SENSORDB("page error \n");
    	return flag_AWB;
    }
    ofilm_start_read_otp(zone);
    
    
    Flag0=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A23);
    Flag1=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A43);
    
    
    flag_AWB = Flag0|Flag1;
    
    ofilm_stop_read_otp();

    SENSORDB("flag_AWB[page %d] := 0x%02x \n", zone, flag_AWB );
	return flag_AWB;
}

/*************************************************************************************************
* Function    :  ofilm_get_otp_LSC_flag
* Description :  get otp LSC_WRITTEN_FLAG  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE], if 0x00 , this type has valid or empty otp data, otherwise, invalid otp data
**************************************************************************************************/
BYTE ofilm_get_otp_LSC_flag(BYTE zone)
{
    BYTE flag_LSC = 0x00;
    ofilm_start_read_otp(zone);
    flag_LSC = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A43);
    ofilm_stop_read_otp();

    SENSORDB("page[%d] flag_LSC := 0x%02x \n", zone,flag_LSC );
	return flag_LSC;
}

/*************************************************************************************************
* Function    :  ofilm_get_otp_module_id
* Description :  get otp MID value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : module ID data , TRULY ID is 0x0001            
**************************************************************************************************/
int ofilm_get_otp_module_id(int *id)
{
	kal_uint16 Temp=0;
	kal_uint16 PageCount;
	kal_uint16 GroupFlag=0,GroupFlag0=0,GroupFlag1=0;
	
	for(PageCount = 3; PageCount>=2; PageCount--)
	{
		SENSORDB("[S5K5E2YA_OFILM] [ofilm_get_otp_module_id] PageCount=%d\n", PageCount);	
		
		if(PageCount>2)
			{//group 4,3
				//otp start control set 
				s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X04); 
				s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A02, PageCount); // page 3
				s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X01);
				mdelay(5);//delay 5ms
				
				GroupFlag1 =0;
				GroupFlag0 =0;
				GroupFlag0 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a23);
				GroupFlag1 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a43);
		
				if((GroupFlag0 ==0x01) || (GroupFlag1 ==0x01))
					{
						if(GroupFlag1 ==0x01)
							{	//group 4 module ID
								Temp=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a24);

								GroupFlag =4;//page 3 ->group 4,3
							}
							else
								{
									//group 3 module ID
									Temp=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a04);
										
									GroupFlag =3;//page 3 ->group 4,3
								}
						break; //
					}
			}
			else
				{//group 2,1
					//otp start control set 
					s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X04); 
				  s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A02, PageCount); //page 2
					s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X01);
					mdelay(5);//delay 5ms
					
					GroupFlag1 =0;
					GroupFlag0 =0;
					GroupFlag1 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a43);
					GroupFlag0 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a23);
					
					if((GroupFlag0 ==0x01) || (GroupFlag1 ==0x01))
						{
							if(GroupFlag1 ==0x01)
							{	//group 2 module ID
								Temp=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a24);

								GroupFlag =2;//page 2 ->group 2,1
							}
							else
								{
									//group 1 module ID
									Temp=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a04);
	
									GroupFlag =1;//page 2 ->group 2,1
								}
									
							break;
						}	
				}
		//otp end control set 
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X04); 
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0x00); 
	}
	
        *id =(Temp&0xff);
    SENSORDB("Module ID = 0x%02x.\n",*id);
	
	if(GroupFlag==0)
		{
        SENSORDB("ofilm_get_otp_module_id error \n");
        return -1;
		}
	
	return 0; 
}


/*************************************************************************************************
* Function    :  ofilm_get_otp_date
* Description :  get otp date value    
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f    
**************************************************************************************************/
bool ofilm_get_otp_date(BYTE zone) 
{
	BYTE year  = 0;
	BYTE month = 0;
	BYTE day   = 0;

	ofilm_start_read_otp(zone);

	year  = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A05);
	month = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A06);
	day   = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A07);

	ofilm_stop_read_otp();

    SENSORDB("date=%02d.%02d.%02d \n", year,month,day);

	return 1;
}



/*************************************************************************************************
* Function    :  ofilm_get_otp_lens_id
* Description :  get otp LENS_ID value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : LENS ID data             
**************************************************************************************************/
BYTE ofilm_get_otp_lens_id(BYTE zone)
{
	BYTE lens_id = 0;

	ofilm_start_read_otp(zone);
	
	lens_id = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A08);
	ofilm_stop_read_otp();

    SENSORDB("Lens ID = 0x%02x.\n",lens_id);

	return lens_id;
}


/*************************************************************************************************
* Function    :  ofilm_get_otp_vcm_id
* Description :  get otp VCM_ID value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : VCM ID data             
**************************************************************************************************/
BYTE ofilm_get_otp_vcm_id(BYTE zone)
{
	BYTE vcm_id = 0;

	ofilm_start_read_otp(zone);
	
	vcm_id = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A09);

	ofilm_stop_read_otp();

    SENSORDB("VCM ID = 0x%02x.\n",vcm_id);

	return vcm_id;	
}


/*************************************************************************************************
* Function    :  ofilm_get_otp_driverIC_id
* Description :  get otp driverIC id value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : driver ID data             
**************************************************************************************************/
BYTE ofilm_get_otp_driverIC_id(BYTE zone)
{
	BYTE driverIC_id = 0;

	ofilm_start_read_otp(zone);
	
	driverIC_id = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A0A);

	ofilm_stop_read_otp();

	SENSORDB("Driver ID = 0x%02x.\n",driverIC_id);

	return driverIC_id;
}

/*************************************************************************************************
* Function    :  ofilm_get_light_id
* Description :  get otp environment light temperature value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                        other value : driver ID data     
			            BIT0:D65(6500K) EN
						BIT1:D50(5100K) EN
						BIT2:CWF(4000K) EN
						BIT3:A Light(2800K) EN
**************************************************************************************************/
BYTE ofilm_get_light_id(BYTE zone)
{
	BYTE light_id = 0;

	ofilm_start_read_otp(zone);	

	light_id = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A0B);

	ofilm_stop_read_otp();

	SENSORDB("Light ID: 0x%02x.\n",light_id);

	return light_id;
}


/*************************************************************************************************
* Function    :  ofilm_otp_lenc_update
* Description :  Update lens correction 
* Parameters  :  
* Return      :  [bool] 0 : OTP data fail 
                        1 : otp_lenc update success            
**************************************************************************************************/
bool ofilm_otp_lenc_update(void)
{
    BYTE  flag_LSC_page = 0;
    //unsigned int  flag_LSC = 0;
    //BYTE  temp = 0;
    flag_LSC_page = ofilm_get_otp_LSC_flag(15);
    //flag_LSC_page3 = ofilm_get_otp_LSC_flag(3);
    //flag_LSC = (flag_LSC_page2 | flag_LSC_page2);
	
    //
    if(1 == flag_LSC_page)
    {
        //LSC correct
        SENSORDB("LSC Auto Correct OK! \n");
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0B00,0x01);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3400,0x00);
    }
    //Load LSC SRAM data
    else
    {
        SENSORDB("LSC Load SRAM Data!!!!! \n");
        //Load
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x340f,0x81); //Truly Add 20140218 start
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0b00,0x01);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3400,0x00);
        Sleep(10);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3457,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3b4c,0x00);//Truly Add 20140218 end
		// ofilm-OJL5F08 LSC golden 
		//GR SEED WRITE REGION                       
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x8b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x29);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xdf);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x01);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xfe);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x02);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x0d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x03);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x10);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x27);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xec);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xba);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x05);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xf6);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x06);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x05);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xf6);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x07);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x16);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x9c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x08);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xdf);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x6d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x09);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xf9);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x1d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x0a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x1e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x55);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x0b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x39);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x11);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x0c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xfc);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x16);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x0d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xd7);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x94);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x0e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x31);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa2);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x18);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xac);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x10);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x56);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x11);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xd9);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xcf);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x12);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x05);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x53);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x13);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x2a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xea);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x14);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc5);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xaf);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x15);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xf1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa4);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x16);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x35);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x3a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x17);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x11);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x98);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x18);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xf1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x94);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x19);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xfa);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xbe);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x1a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x1a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x22);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x1b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xea);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa7);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x1c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x11);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x49);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x1d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x07);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x1e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xe5);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x1f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xdd);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x63);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x20);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x1d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x23);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x21);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x11);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xb5);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x22);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xcd);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x29);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x23);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		//R SEED WRITE REGION                        
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x90);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xcf);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x24);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe2);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc3);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x25);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x06);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x26);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x28);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x27);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x03);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xaa);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x28);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xf3);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xd7);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x29);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xbf);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x90);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x2a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xff);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x5e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x2b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x1c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x71);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x2c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xd7);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x59);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x2d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xea);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x2e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x17);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x4d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x2f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x37);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x5b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x30);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc0);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x31);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xd1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa7);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x32);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x34);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x58);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x33);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x10);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x4f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x34);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc6);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x35);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xdd);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x69);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x36);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xed);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x55);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x37);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x34);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x90);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x38);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc9);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x7f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x39);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe8);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xe2);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x3a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x41);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x44);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x3b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x14);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x41);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x3c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x07);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x3d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xed);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x48);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x3e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x18);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x95);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x3f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xff);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x36);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xf6);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x53);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x41);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x01);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x1a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x42);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x08);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa0);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x43);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe5);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x4d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x44);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x18);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xf2);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x45);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x08);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xd1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x46);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xdc);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x55);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x47);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		//B SEED WRITE REGION                        
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x80);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa7);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x48);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe5);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x3a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x49);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x08);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xe7);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x4a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x02);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x08);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x4b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x9d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x4c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xeb);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x8f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x4d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc8);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x72);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x4e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x02);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xee);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x4f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x18);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x41);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x50);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe2);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x6c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x51);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xef);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc7);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x52);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x27);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xb0);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x53);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x32);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x65);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x54);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc3);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x55);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xcb);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x47);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x56);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x35);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x76);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x57);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x1e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x9e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x58);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xb7);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x1f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x59);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xd4);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x5a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe9);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x3c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x5b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x3e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x5b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x5c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc5);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x5a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x5d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe6);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xd0);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x5e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x3b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x5f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x06);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x5f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x60);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x9a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x61);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xeb);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x8e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x62);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x10);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xf1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x63);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xf6);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x43);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x64);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x10);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x59);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x65);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa5);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x66);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xf1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x67);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe0);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x4a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x68);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x29);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x69);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x69);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x63);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x6a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc6);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x48);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x6b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		//GB SEED WRITE REGION                       
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x8d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xcf);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x6c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe0);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x3e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x6d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xec);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x6e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xff);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x45);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x6f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x15);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x3b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x70);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe6);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xf3);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x71);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc1);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x3d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x72);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x02);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x81);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x73);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x1b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xbe);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x74);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe0);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x58);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x75);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xea);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xbb);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x76);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x2e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xd5);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x77);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x3a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x24);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x78);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x07);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x65);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x79);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc9);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x14);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x7a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x33);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xce);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x7b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x2f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x14);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x7c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xa6);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x24);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x7d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xd8);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x76);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x7e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xf0);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xbe);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x7f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x7b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x80);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc5);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x25);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x81);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xd4);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x26);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x82);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x54);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xf0);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x83);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x12);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa6);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x84);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x07);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x56);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x85);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe8);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x90);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x86);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x18);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xec);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x87);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xfa);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x03);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x88);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x06);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x5a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x89);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x06);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xd0);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x8a);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x06);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x3d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x8b);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe0);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xf2);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x8c);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x21);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x23);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x8d);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x14);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xb5);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x8e);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc2);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x14);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x8f);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
		//ALPHA WRITE REGION                         
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x341A,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x341B,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x341C,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x341D,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x341E,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x341F,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3420,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3421,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3422,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3423,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3424,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3425,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3426,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3427,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3428,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3429,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x342A,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x342B,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x342C,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x342D,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x342E,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x342F,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3430,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3431,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3432,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3433,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3434,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3435,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3436,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3437,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3438,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3439,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x343A,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x343B,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x343C,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x343D,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x343E,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x343F,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3440,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3441,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3442,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3443,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3444,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3445,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3446,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3447,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3448,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3449,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x344A,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x344B,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x344C,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x344D,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x344E,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x344F,0x00);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3450,0x40);
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3451,0x00);

        //Add More
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3457,0x0c);
        Sleep(10);
    }
    SENSORDB("OTP LSC Update Finished! \n");
    return 1;
}







//#define S5K5E2Y_USE_AWB_OTP
#if 1


int S5K5E2YA_OFILM_MIPI_read_otp_wb(struct S5K5E2YA_OFILM_MIPI_otp_struct *otp)
{
	
	//kal_uint32 R_to_G, B_to_G, G_to_G;
	kal_uint16 PageCount;
	kal_uint16 GroupFlag=0,GroupFlag0=0,GroupFlag1=0;
	kal_uint16 H_Byte=0,L_Byte=0;
	kal_uint16 RValue_C=0,GValue_C=0,BValue_C=0,RValue_G=0,GValue_G=0,BValue_G=0;
	
	for(PageCount = 3; PageCount>=2; PageCount--)
	{
        SENSORDB("PageCount=%d\n", PageCount);	
		
		if(PageCount>2)
			{//group 4,3
				//otp start control set 
				s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X04); 
				s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A02, 0X03); // page 3
				s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X01);
				mdelay(5);//delay 5ms
				
				GroupFlag1 =0;
				GroupFlag0 =0;
				GroupFlag0 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a23);
				GroupFlag1 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a43);
				
				
				if((GroupFlag0 ==0x01) || (GroupFlag1 ==0x01))
					{
						if(GroupFlag1 ==0x01)
							{	//page3 group4 you xiao 
								//current module value
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2B);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2C);
								RValue_C =(H_Byte<<8)+L_Byte;
								
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2D);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2E);
								BValue_C =(H_Byte<<8)+L_Byte;
								
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2F);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a30);
								GValue_C =(H_Byte<<8)+L_Byte;
								
								// Golden value 
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a32);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a33);
								RValue_G =(H_Byte<<8)+L_Byte;

								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a34);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a35);
								BValue_G =(H_Byte<<8)+L_Byte;
								
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a36);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a37);
								GValue_G =(H_Byte<<8)+L_Byte;

								GroupFlag =4;
							}
							else
								{
									//page3 group3 you xiao 
									//current module value
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0B);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0C);
								RValue_C =(H_Byte<<8)+L_Byte;
								
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0D);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0E);
								BValue_C =(H_Byte<<8)+L_Byte;
								
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0F);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a10);
								GValue_C =(H_Byte<<8)+L_Byte;
								
								// Golden value
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a12);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a13);
								RValue_G =(H_Byte<<8)+L_Byte;

								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a14);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a15);
								BValue_G =(H_Byte<<8)+L_Byte;
								
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a16);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a17);
								GValue_G =(H_Byte<<8)+L_Byte;
									
								GroupFlag =3;
								}
						break; //
					}
				
			}
			else
				{//group 2,1
					//otp start control set 
					s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X04); 
				    s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A02, 0X02); //page 2
					s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X01);
					 
					mdelay(5);//delay 5 ms
					 GroupFlag1 =0;
					 GroupFlag0 =0;
					GroupFlag1 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a43);
					GroupFlag0 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a23);
					
					if((GroupFlag0 ==0x01) || (GroupFlag1 ==0x01))
						{
							if(GroupFlag1 ==0x01)
							{	//page2 group2 you xiao
							    //current awb value
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2B);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2C);
								RValue_C =(H_Byte<<8)+L_Byte;
								
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2D);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2E);
								BValue_C =(H_Byte<<8)+L_Byte;
								
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2F);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a30);
								GValue_C =(H_Byte<<8)+L_Byte;
								
								//golden awb value
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a32);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a33);
								RValue_G =(H_Byte<<8)+L_Byte;

								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a34);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a35);
								BValue_G =(H_Byte<<8)+L_Byte;
								
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a36);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a37);
								GValue_G =(H_Byte<<8)+L_Byte;

								GroupFlag =2;
							}
							else
								{
								//page2 group1 you xiao
								//current awb value
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0B);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0C);
								RValue_C =(H_Byte<<8)+L_Byte;
								
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0D);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0E);
								BValue_C =(H_Byte<<8)+L_Byte;
								
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0F);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a10);
								GValue_C =(H_Byte<<8)+L_Byte;
								
								//golden awb value 
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a12);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a13);
								RValue_G =(H_Byte<<8)+L_Byte;

								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a14);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a15);
								BValue_G =(H_Byte<<8)+L_Byte;
								
								H_Byte=0;
								L_Byte=0;
								H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a16);
								L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a17);
								GValue_G =(H_Byte<<8)+L_Byte;
									
								GroupFlag =1;
								}
									
							break;
						}	
				}
		//otp end control set 
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X04); 
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0x00); 
	}
	SENSORDB("GrouFlag =%d, RValue_C=0x%x, BValue_C=0x%x, GValue_C=0x%x \n"
            ,GroupFlag
            ,RValue_C
            ,BValue_C
            ,GValue_C);
	SENSORDB("GrouFlag =%d, RValue_G=0x%x, BValue_G=0x%x, GValue_G=0x%x \n"
            ,GroupFlag
            ,RValue_G
            ,BValue_G
            ,GValue_G);

		if((GroupFlag==0)||(RValue_C==0)||(BValue_C==0)||(GValue_C==0))
		{
        SENSORDB("otp AWB ERROR!");
			  otp->R_to_G = OFILM_tRG_Ratio_typical;
			  otp->B_to_G = OFILM_tBG_Ratio_typical;
			  otp->G_to_G = 0x400;
        return -1;
		}
		else
			{	
				otp->R_to_G=(RValue_C*1024/GValue_C);
				otp->B_to_G=(BValue_C*1024/GValue_C);
			    otp->G_to_G = 0x400;	
			}
		
	

    SENSORDB("GrouFlag =%d, otp->R_to_G=0x%x, otp->B_to_G=0x%x, otp->G_to_G=0x%x \n"
            ,GroupFlag
            ,otp->R_to_G
            ,otp->B_to_G
            ,otp->G_to_G);
    return 0;
}

void S5K5E2YA_OFILM_MIPI_algorithm_otp_wb1(struct S5K5E2YA_OFILM_MIPI_otp_struct *otp)
{
	kal_uint32 R_to_G, B_to_G, G_to_G;
	kal_uint32 R_Gain, B_Gain, G_Gain;
	kal_uint32 G_gain_R, G_gain_B;
	
	R_to_G = otp->R_to_G;
	B_to_G = otp->B_to_G;
	G_to_G = otp->G_to_G;

    SENSORDB("R_to_G=%d, B_to_G=%d, G_to_G=%d \n",R_to_G,B_to_G,G_to_G);

if(R_to_G < OFILM_tRG_Ratio_typical )
    {
        if(B_to_G < OFILM_tBG_Ratio_typical) 
        {
          R_Gain = 0x100 * OFILM_tRG_Ratio_typical / R_to_G;
          G_Gain = 0x100;
					B_Gain = 0x100 * OFILM_tBG_Ratio_typical / B_to_G;
        }
        else
        {
					R_Gain = 0x100 * (OFILM_tRG_Ratio_typical*B_to_G) / (OFILM_tBG_Ratio_typical*R_to_G);
					G_Gain = 0x100 *  B_to_G / OFILM_tBG_Ratio_typical;
					B_Gain = 0x100;
        }
    }
    else                      
    {		
        if(B_to_G < OFILM_tBG_Ratio_typical)
        {
          R_Gain = 0x100;
					G_Gain = 0x100 * R_to_G / OFILM_tRG_Ratio_typical;
					B_Gain = 0x100 * (OFILM_tBG_Ratio_typical*R_to_G) / (OFILM_tRG_Ratio_typical* B_to_G);
        } 
        else 
        {
        		G_gain_R = 0x100*R_to_G / OFILM_tRG_Ratio_typical;
            G_gain_B = 0x100*B_to_G / OFILM_tBG_Ratio_typical;
            if(G_gain_R > G_gain_B)						
            {						
              R_Gain = 0x100;
							G_Gain = 0x100 * R_to_G / OFILM_tRG_Ratio_typical;
							B_Gain = 0x100 * (OFILM_tBG_Ratio_typical*R_to_G) / (OFILM_tRG_Ratio_typical* B_to_G);
            } 
            else
            {		
            	R_Gain = 0x100 * (OFILM_tRG_Ratio_typical*B_to_G) / (OFILM_tBG_Ratio_typical*R_to_G);	
							G_Gain = 0x100 *  B_to_G / OFILM_tBG_Ratio_typical;
							B_Gain = 0x100;
            }
        }        
    }

	otp->R_Gain = R_Gain;
	otp->B_Gain = B_Gain;
	otp->G_Gain = G_Gain;

    SENSORDB("R_gain=0x%x, B_gain=0x%x, G_gain=0x%x \n",otp->R_Gain, otp->B_Gain, otp->G_Gain);
}



void S5K5E2YA_OFILM_MIPI_write_otp_wb(struct S5K5E2YA_OFILM_MIPI_otp_struct *otp)
{
	kal_uint16 R_GainH, B_GainH, G_GainH;
	kal_uint16 R_GainL, B_GainL, G_GainL;
	kal_uint32 temp;

	temp = otp->R_Gain;
	R_GainH = (temp & 0xff00)>>8;
	temp = otp->R_Gain;
	R_GainL = (temp & 0x00ff);

	temp = otp->B_Gain;
	B_GainH = (temp & 0xff00)>>8;
	temp = otp->B_Gain;
	B_GainL = (temp & 0x00ff);

	temp = otp->G_Gain;
	G_GainH = (temp & 0xff00)>>8;
	temp = otp->G_Gain;
	G_GainL = (temp & 0x00ff);

    SENSORDB("G_GainH = 0x%x,G_GainL = 0x%x, R_GainH = 0x%x, R_GainL = 0x%x, B_GainH = 0x%x, B_GainL = 0x%x \n"
            ,G_GainH
            ,G_GainL
            ,R_GainH
            ,R_GainL
            ,B_GainH
            ,B_GainL);

	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x020e, G_GainH);
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x020f, G_GainL);//GR
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0210, R_GainH);
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0211, R_GainL);
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0212, B_GainH);
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0213, B_GainL);
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0214, G_GainH);
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0215, G_GainL);//GB
}

void S5K5E2YA_OFILM_MIPI_update_wb_register_from_otp(void)
{
    struct S5K5E2YA_OFILM_MIPI_otp_struct current_otp;

    S5K5E2YA_OFILM_MIPI_read_otp_wb(&current_otp);
    S5K5E2YA_OFILM_MIPI_algorithm_otp_wb1(&current_otp);
    S5K5E2YA_OFILM_MIPI_write_otp_wb(&current_otp);
}
#endif






/*************************************************************************************************
* Function    :  ofilm_wb_gain_set
* Description :  Set WB ratio to register gain setting  512x
* Parameters  :  [int] ofilm_r_ratio : R ratio data compared with golden module R
                       ofilm_b_ratio : B ratio data compared with golden module B
* Return      :  [bool] 0 : set wb fail 
                        1 : WB set success            
**************************************************************************************************/

bool ofilm_wb_gain_set(void)
{
    USHORT R_GAIN;
    USHORT B_GAIN;
    USHORT Gr_GAIN;
    USHORT Gb_GAIN;
    USHORT G_GAIN;

    if(!ofilm_r_ratio || !ofilm_b_ratio)
    {
        SENSORDB("OTP WB ratio Data Err!\n");
        return 0;
    }
//s5k5e2yamipiraw_ofilm_write_cmos_sensor(OFILM_GAIN_GREEN1_ADDR, OFILM_GAIN_DEFAULT); //Green 1 default gain 1x
//s5k5e2yamipiraw_ofilm_write_cmos_sensor(OFILM_GAIN_GREEN2_ADDR, OFILM_GAIN_DEFAULT); //Green 2 default gain 1x
    if(ofilm_r_ratio >= 512 )
    {
        if(ofilm_b_ratio>=512) 
        {
            R_GAIN = (USHORT)(OFILM_GAIN_DEFAULT * ofilm_r_ratio / 512);						
            G_GAIN = OFILM_GAIN_DEFAULT;
            B_GAIN = (USHORT)(OFILM_GAIN_DEFAULT * ofilm_b_ratio / 512);
        }
        else
        {
            R_GAIN =  (USHORT)(OFILM_GAIN_DEFAULT*ofilm_r_ratio / ofilm_b_ratio );
            G_GAIN = (USHORT)(OFILM_GAIN_DEFAULT*512 / ofilm_b_ratio );
            B_GAIN = OFILM_GAIN_DEFAULT;    
        }
    }
    else                      
    {		
        if(ofilm_b_ratio >= 512)
        {
            R_GAIN = OFILM_GAIN_DEFAULT;
            G_GAIN = (USHORT)(OFILM_GAIN_DEFAULT*512 /ofilm_r_ratio);		
            B_GAIN =  (USHORT)(OFILM_GAIN_DEFAULT*ofilm_b_ratio / ofilm_r_ratio );
        } 
        else 
        {
            Gr_GAIN = (USHORT)(OFILM_GAIN_DEFAULT*512/ ofilm_r_ratio );						
            Gb_GAIN = (USHORT)(OFILM_GAIN_DEFAULT*512/ofilm_b_ratio );						
            if(Gr_GAIN >= Gb_GAIN)						
            {						
                R_GAIN = OFILM_GAIN_DEFAULT;						
                G_GAIN = (USHORT)(OFILM_GAIN_DEFAULT *512/ ofilm_r_ratio );						
                B_GAIN =  (USHORT)(OFILM_GAIN_DEFAULT*ofilm_b_ratio / ofilm_r_ratio );						
            } 
            else
            {						
                R_GAIN =  (USHORT)(OFILM_GAIN_DEFAULT*ofilm_r_ratio  / ofilm_b_ratio);						
                G_GAIN = (USHORT)(OFILM_GAIN_DEFAULT*512 / ofilm_b_ratio );						
                B_GAIN = OFILM_GAIN_DEFAULT;
            }
        }        
    }

    SENSORDB("WB Gain Set [R_GAIN=%d],[G_GAIN=%d],[B_GAIN=%d] \n",R_GAIN, G_GAIN, B_GAIN);
    s5k5e2yamipiraw_ofilm_write_cmos_sensor(OFILM_GAIN_RED_ADDR, R_GAIN);		
    s5k5e2yamipiraw_ofilm_write_cmos_sensor(OFILM_GAIN_BLUE_ADDR, B_GAIN);     
    s5k5e2yamipiraw_ofilm_write_cmos_sensor(OFILM_GAIN_GREEN1_ADDR, G_GAIN); //Green 1 default gain 1x		
    s5k5e2yamipiraw_ofilm_write_cmos_sensor(OFILM_GAIN_GREEN2_ADDR, G_GAIN); //Green 2 default gain 1x

	return 1;
}



/*************************************************************************************************
* Function    :  ofilm_get_otp_wb
* Description :  Get WB data    
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f      
**************************************************************************************************/
bool ofilm_get_otp_wb(BYTE zone)
{
	BYTE temph = 0;
	BYTE templ = 0;
	ofilm_golden_r = 0, ofilm_golden_gr = 0, ofilm_golden_gb = 0, ofilm_golden_b = 0;
	ofilm_current_r = 0, ofilm_current_gr = 0, ofilm_current_gb = 0, ofilm_current_b = 0;

    ofilm_start_read_otp(zone);

	temph = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A19);  
	templ = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A18);   
	ofilm_golden_r  = (USHORT)templ + ((USHORT)temph << 8);

	temph = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A1B);  
	templ = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A1A);   
	ofilm_golden_gr  = (USHORT)templ + ((USHORT)temph << 8);

	temph = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A1D);  
	templ = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A1C);   
	ofilm_golden_gb  = (USHORT)templ + ((USHORT)temph << 8);

	temph = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A1F);  
	templ = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A1E);   
	ofilm_golden_b  = (USHORT)templ + ((USHORT)temph << 8);

	temph = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A0B);  
	templ = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A0C);   
	ofilm_current_r  = (USHORT)templ + ((USHORT)temph << 8);

	temph = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A13);  
	templ = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A12);   
	ofilm_current_gr  = (USHORT)templ + ((USHORT)temph << 8);

	temph = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A15);  
	templ = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A14);   
	ofilm_current_gb  = (USHORT)templ + ((USHORT)temph << 8);

	temph = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A0D);  
	templ = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A0E);   
	ofilm_current_b  = (USHORT)templ + ((USHORT)temph << 8);

	ofilm_stop_read_otp();

    SENSORDB("[page %d]\n",zone);
    SENSORDB("ofilm_golden_r=%d,ofilm_golden_gr=%d,ofilm_golden_gb=%d,ofilm_golden_b=%d \n",ofilm_golden_r,ofilm_golden_gr,ofilm_golden_gb,ofilm_golden_b);
    SENSORDB("ofilm_current_r=%d,ofilm_current_gr=%d,ofilm_current_gb=%d,ofilm_current_b=%d \n",ofilm_current_r,ofilm_current_gr,ofilm_current_gb,ofilm_current_b);    
	return 1;
}


/*************************************************************************************************
* Function    :  ofilm_otp_wb_update
* Description :  Update WB correction 
* Return      :  [bool] 0 : OTP data fail 
                        1 : otp_WB update success            
**************************************************************************************************/
bool ofilm_otp_wb_update(BYTE zone)
{
	USHORT golden_g, current_g;


	if(!ofilm_get_otp_wb(zone))  // get wb data from otp
		return 0;

	golden_g = (ofilm_golden_gr + ofilm_golden_gb) / 2;
	current_g = (ofilm_current_gr + ofilm_current_gb) / 2;

	if(!golden_g || !current_g || !ofilm_golden_r || !ofilm_golden_b || !ofilm_current_r || !ofilm_current_b)
	{
		SENSORDB("WB update Err !%d--%d--%d--%d--%d--%d--\n",golden_g,current_g,ofilm_golden_r,ofilm_golden_b,ofilm_current_r,ofilm_current_b);
		//return 0;
	}


	ofilm_r_ratio = 512 * ofilm_golden_r * current_g /( golden_g * ofilm_current_r );
	ofilm_b_ratio = 512 * ofilm_golden_b * current_g /( golden_g * ofilm_current_b );
    SENSORDB("ofilm_r_ratio=%d, ofilm_b_ratio=%d \n",ofilm_r_ratio, ofilm_b_ratio);
	//ofilm_wb_gain_set();


	return 1;
}

/*************************************************************************************************
* Function    :  ofilm_otp_update()
* Description :  update otp data from otp , it otp data is valid, 
                 it include get ID and WB update function  
* Return      :  [bool] 0 : update fail
                        1 : update success
**************************************************************************************************/
bool ofilm_otp_update(void)
{
	BYTE zone = 0x02;//start from Page 2
	BYTE FLG = 0x00;
	int MID = 0x00;//LENS_ID= 0x00;
	int i;
	
	for(i=0;i<3;i++)
	{
		FLG = ofilm_get_otp_AWB_flag(zone);
		if(FLG == OFILM_INVALID_OTP)
			zone ++;
		else
			break;
	}
	if(i==3)
	{
		SENSORDB("No OTP Data or OTP data is invalid!!\n");
		return 0;
	}
    ofilm_get_otp_module_id(&MID);
//    ofilm_get_otp_date(zone);
//    LENS_ID=	ofilm_get_otp_lens_id(zone);
//    VCM_ID=	ofilm_get_otp_vcm_id(zone);
//    ofilm_get_otp_driverIC_id(zone);
//    ofilm_get_light_id(zone);
 #if 0   
	if(MID != TRULY_ID)
	{
		SENSORDB("Not Truly Modul!!!!\n");
		return 0;
	}
#endif
	return ofilm_otp_wb_update(zone);	
}

