#ifdef BUILD_LK
#include <stdio.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mt-plat/mt_gpio.h>
#include <mt-plat/upmu_common.h>
#include <mach/gpio_const.h>
#include <linux/time.h>
#include <linux/rtc.h>
#endif



#include "lcm_drv.h"



// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

//#define ESD_SUPPORT

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define PHYSICAL_WIDTH  (56)  // mm
#define PHYSICAL_HEIGHT (93)  // mm

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


#define LCM_ID        (0x40)
#define LCM_ID1       (0x10)
#define LCM_ID2       (0xa2)



//#define ESD_CHECK_EN

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------



#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)


#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER


struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};



#ifdef CONFIG_GN_MTK_BSP_LCD_ACL_SUPPORT
static unsigned char g_acl_state = 0;
static unsigned char g_hbm_state = 0;
void lcm_setacl_cmdq(unsigned int level);
#endif




static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	{0x11, 0, {}},
	{REGFLAG_DELAY, 20, {}},
	
	{0x53, 1, {0x20}},
	{0x51, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	{0x29, 0, {}},
	//{REGFLAG_DELAY, 10, {}},
	
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
	
	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};


#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    //{REGFLAG_DELAY, 10, {}},
    
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {
//	{0x53, 1, {0x20}},
	{0x51, 1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};



static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}




// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;	

	params->physical_width = PHYSICAL_WIDTH;
	params->physical_height = PHYSICAL_HEIGHT;

	params->dsi.mode = SYNC_EVENT_VDO_MODE;
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 5;  
	params->dsi.vertical_frontporch = 9; 
 	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 50;
	params->dsi.horizontal_frontporch = 150;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

//	params->dsi.intermediat_buffer_num = 2;

//	params->dsi.ssc_disable = 1;

	params->dsi.cont_clock = 1;
	params->dsi.PLL_CLOCK = 230;

	
#ifdef ESD_CHECK_EN
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
	#endif
}


static void lcm_init_power(void)
{

#ifdef BUILD_LK
		pmic_set_register_value(PMIC_RG_VGP1_VOSEL,7);	//7:3.3V 
		pmic_set_register_value(PMIC_RG_VGP1_EN,1);
#else
		hwPowerOn(MT6328_POWER_LDO_VGP1, VOL_3300, "LCM_DRV");
#endif	
}

#ifndef BUILD_LK
static bool first_suspend = TRUE;
#endif
static void lcm_suspend_power(void)
{
//        SET_RESET_PIN(1);
 //       MDELAY(5);
        SET_RESET_PIN(0);
        MDELAY(1);

#ifdef BUILD_LK
		pmic_set_register_value(PMIC_RG_VGP1_EN,0);
#else
		if(first_suspend == TRUE)
		{
			hwPowerOn(MT6328_POWER_LDO_VGP1, VOL_3300, "LCM_DRV");
			first_suspend = FALSE;
		}
		hwPowerDown(MT6328_POWER_LDO_VGP1, "LCM_DRV");	
#endif	
}

static void lcm_resume_power(void)
{
	lcm_init_power();
}


static void init_lcm_registers(void)
{
#if 0
	unsigned int data_array[16];

	//sleep out
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	//brightness control
	data_array[0]=0x00021500;
	data_array[1]=0x00002053;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00021500;
	data_array[1]=0x0000cc51;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(5);

	//display on
	data_array[0]= 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
	#endif 

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	
}

static void lcm_init(void)
{

	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(10);
	
	init_lcm_registers();

#ifdef CONFIG_GN_MTK_BSP_LCD_ACL_SUPPORT
	if(g_acl_state == 1)
	{
	    lcm_setacl_cmdq(1);
	}
#endif

}


static void lcm_suspend(void)
{
#if 0
	unsigned int data_array[16];

	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(40);

	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
#endif

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
	lcm_init();
}


static void lcm_setbacklight(unsigned int level)
{	
	if (level > 255)
		level = 255;
	if((level < 10)&&(level > 0))
		level = 2;
	else if((level >= 10)&&(g_hbm_state == 0))
	{
		level = level*230/245 - 7;
	}
	LCM_PRINT("lcm_set_backlight = %d\n", level);

	lcm_backlight_level_setting[0].para_list[0] = level;
	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}



//gionee mazuyu add for amoled specific function start 
#ifdef CONFIG_CUSTOM_GN_BSP_AMOLED_HBM_SUPPORT
static void lcm_set_hbm(unsigned int level)
{
	unsigned int cmd = 0x53;
	unsigned int mode = 0;
	
	if(level == 1)
		mode = 0x60;
	else
		mode = 0x20;

	LCM_PRINT("lcm_set_hbm = %d\n", level);

	//MDELAY(20); 
	dsi_set_cmdq_V2(cmd, 1, &mode, 1);
	//MDELAY(50); 
}
#endif

#ifdef CONFIG_CUSTOM_GN_BSP_AMOLED_ACL_SUPPORT
static void lcm_set_acl(unsigned int level)
{
	unsigned int cmd = 0x55;
	unsigned int mode = 0;
	
	if(level == 1)
	{
		mode = 0x01;
		g_acl_state = 1;
	}
	else
	{
		mode = 0x00;
		g_acl_state = 0;
	}

	LCM_PRINT("lcm_set_acl = %d\n", level);

	//MDELAY(20); 
	dsi_set_cmdq_V2(cmd, 1, &mode, 1);
	//MDELAY(50); 
}
#endif
//gionee mazuyu add for amoled specific function end


static unsigned int lcm_compare_id(void)
{

#if 0
	int i = 0;
	unsigned int id = 0;
	unsigned char buffer[3];
	unsigned int data_array[16];

	//Level 2 command access mode enable
	data_array[0]=0x00033902;
	data_array[1]=0x005a5af0;
	dsi_set_cmdq(data_array, 2, 1);

	//Level 3 command access mode enable
	data_array[0]=0x00033902;
	data_array[1]=0x005a5afc;
	dsi_set_cmdq(data_array, 2, 1);

	//read three times most in case of accidental error reading
	i = 0;
	do{
		//read ID command
		data_array[0]=0x00043902;	
		data_array[1]=0xa2104004;
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(30);

		read_reg_v2(0xDA, buffer, 1);
		id = buffer[0]; 
		i++;
	}while(id != LCM_ID && i < 3);


	//Level 2 command access mode disable
	data_array[0]=0x00033902;
	data_array[1]=0x00a5a5f0;
	dsi_set_cmdq(data_array, 2, 1);

	//Level 3 command access mode disable
	data_array[0]=0x00033902;
	data_array[1]=0x00a5a5fc;
	dsi_set_cmdq(data_array, 2, 1);

	LCM_PRINT("mazuyu___lcm ID1 = %d\n", id);

	return (LCM_ID == id)?1:0;
#endif

	return 1;

}


#if 0
static unsigned int lcm_esd_check(void)
{
	unsigned int result = TRUE;
	unsigned int data_array[16];
	unsigned char buffer[16] = {0};

	data_array[0] = 0x00013700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x0A, buffer, 1);
	if (buffer[0] == 0x9C)
		result = FALSE;

	return result;
}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();

	return TRUE;
}

#endif

// Write and read back 0x2A (CASET)
// For real video mode LCM, use other driver IC register
static unsigned int lcm_ata_check(unsigned char *buffer)
{
#if 0
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH/4;
	unsigned int x1 = FRAME_WIDTH*3/4;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];
	LCM_PRINT("ATA check size = 0x%x,0x%x,0x%x,0x%x\n",x0_MSB,x0_LSB,x1_MSB,x1_LSB);
	data_array[0]= 0x0005390A;//HS packet
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00043700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x2A, read_buf, 4);

	if ((read_buf[0] == x0_MSB) && (read_buf[1] == x0_LSB) 
			&& (read_buf[2] == x1_MSB) && (read_buf[3] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	x0 = 0;
	x1 = FRAME_WIDTH - 1;

	x0_MSB = ((x0>>8)&0xFF);
	x0_LSB = (x0&0xFF);
	x1_MSB = ((x1>>8)&0xFF);
	x1_LSB = (x1&0xFF);

	data_array[0]= 0x0005390A;//HS packet
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	return ret;
#endif

	return 1;
}


#ifdef CONFIG_GN_MTK_BSP_LCD_ACL_SUPPORT
void lcm_setacl_cmdq(unsigned int level)
{

	unsigned int cmd = 0x55;
//	unsigned int count =1;
	unsigned char value = 0;
	LCM_PRINT("lcm_setacl_cmdq level= %d\n", level);
	if(level == 1)
	{
		g_acl_state = 1;
		value = 0x02;
	//	dsi_set_cmdq_V2(cmd, count, &value, 1);
	}
	else
	{
		g_acl_state = 0;
		value = 0x00;
	//	dsi_set_cmdq_V2( cmd, count, &value, 1);
	}
	//MDELAY(20); 
	dsi_set_cmdq_V2(cmd, 1, &value, 1);
	//MDELAY(50); 

}

void lcm_sethbm_cmdq(unsigned int level)
{

//	unsigned int cmd = 0x53;
//	unsigned int count =1;
//	unsigned int value = 0;
	LCM_PRINT("lcm_sethbm_cmdq level= %d\n",level);
	if(level == 1)
	{
		g_hbm_state = 1;
		//value = 0x60;
	//	dsi_set_cmdq_V2(cmd, count, &value, 1);
	}
	else
	{
		g_hbm_state = 0;
		//value = 0x20;
	//	dsi_set_cmdq_V2(cmd, count, &value, 1);
	}
	//MDELAY(20); 
	//dsi_set_cmdq_V2(cmd, 1, &value, 1);
	//MDELAY(50); 

}
/*
void lcm_setpartical_cmdq(unsigned int level)
{
	LCM_PRINT("lcm_setpartical_cmdq level= %d\n",level);
	if(level == 1)
	{				
		unsigned int cmd = 0x12;
		unsigned int count =0;
		unsigned char paralist1[1] = {0x00};

		dsi_set_cmdq_V2(cmd, count, paralist1, 1);
		
		cmd = 0x30;
		count = 4;
		unsigned char paralist2[4] = {0x03,0x1F,0x04,0xFF};

		dsi_set_cmdq_V2(cmd, count, paralist2, 1);
	}
	else
	{		
		unsigned int cmd = 0x13;
		unsigned int count =0;
		unsigned char paralist[1] = {0x00};

		dsi_set_cmdq_V2(cmd, count, paralist, 1);
	}
}
*/
#endif


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER s6e8aa4_hd_dsi_vdo_samsung_lcm_drv =
{
	.name           = "s6e8aa4_hd_dsi_vdo_samsung",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.set_backlight  = lcm_setbacklight,
	.compare_id     = lcm_compare_id,
	
	.init_power		= lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,

	//gionee mazuyu add for amoled specific function start 
#ifdef CONFIG_GN_MTK_BSP_LCD_ACL_SUPPORT
	.set_hbm  		= lcm_sethbm_cmdq,
#endif

#ifdef CONFIG_GN_MTK_BSP_LCD_ACL_SUPPORT
	.set_acl  		= lcm_setacl_cmdq,
 #endif
 	//gionee mazuyu add for amoled specific function end
#if 0
	.esd_check      = lcm_esd_check,
	.esd_recover    = lcm_esd_recover,
#endif
	.ata_check      = lcm_ata_check,
};




