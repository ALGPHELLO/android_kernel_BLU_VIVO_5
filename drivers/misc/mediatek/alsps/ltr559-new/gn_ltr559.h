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

/* 
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
/*
 * Definitions for ltr559 als/ps sensor chip.
 */

/*
 * Change log
 *
 * Date		Author		Description
 * ----------------------------------------------------------------------------
 * 11/17/2011	chenqy		Initial modification from ltr502.
 * 01/06/2012	chenqy		Do not enable the als and ps by default.
 *
 */

#ifndef __LTR559_H__
#define __LTR559_H__

#include <linux/ioctl.h>
#include <linux/types.h>
#include <cust_alsps_ltr559.h>

#define LTR559_I2C_SLAVE_ADDR  0x23

/*REG address*/

#define APS_RW_ALS_CONTR		0x80 // ALS operation mode control SW reset
#define APS_RW_PS_CONTR			0x81 // PS operation mode control
#define APS_RW_PS_LED			0x82 // PS LED setting
#define APS_RW_PS_N_PULSES		0x83 // PS number of pulses
#define APS_RW_PS_MEAS_RATE		0x84 // PS measurement rate in active mode
#define APS_RW_ALS_MEAS_RATE		0x85 // ALS measurement rate in active mode
#define APS_RO_PART_ID			0x86 // Part Number ID and Revision ID
#define APS_RO_MANUFAC_ID		0x87 // Manufacturer ID
#define APS_RO_ALS_DATA_CH1_0		0x88 // ALS measurement CH1 data, lower byte
#define APS_RO_ALS_DATA_CH1_1		0x89 // ALS measurement CH1 data, upper byte
#define APS_RO_ALS_DATA_CH0_0		0x8A // ALS measurement CH0 data, lower byte
#define APS_RO_ALS_DATA_CH0_1		0x8B // ALS measurement CH0 data, upper byte
#define APS_RO_ALS_PS_STATUS		0x8C // ALS and PS new data status
#define APS_RO_PS_DATA_0		0x8D // PS measurement data, lower byte
#define APS_RO_PS_DATA_1		0x8E // PS measurement data, upper byte
#define APS_RW_INTERRUPT		0x8F // Interrupt settings
#define APS_RW_PS_THRES_UP_0		0x90 // PS interrupt upper threshold, lower byte
#define APS_RW_PS_THRES_UP_1		0x91 // PS interrupt upper threshold, upper byte
#define APS_RW_PS_THRES_LOW_0		0x92 // PS interrupt lower threshold, lower byte
#define APS_RW_PS_THRES_LOW_1		0x93 // PS interrupt lower threshold, upper byte
#define APS_RW_ALS_THRES_UP_0		0x97 // ALS interrupt upper threshold, lower byte
#define APS_RW_ALS_THRES_UP_1		0x98 // ALS interrupt upper threshold, upper byte
#define APS_RW_ALS_THRES_LOW_0		0x99 // ALS interrupt lower threshold, lower byte
#define APS_RW_ALS_THRES_LOW_1		0x9A // ALS interrupt lower threshold, upper byte
#define APS_RW_INTERRUPT_PERSIST	0x9E // ALS / PS Interrupt persist setting

/* Basic Operating Modes */
// FIXME:
// We should not enable the als and ps by default

// #define MODE_ALS_ON_Range1		0x0B
// #define MODE_ALS_ON_Range2		0x03
#define MODE_ALS_ON_Range1		(0x1 << 3)
#define MODE_ALS_ON_Range2		(0x0 << 3)
#define MODE_ALS_StdBy			0x00

// #define MODE_PS_ON_Gain1		0x03
// #define MODE_PS_ON_Gain4		0x07
// #define MODE_PS_ON_Gain8		0x0B
// #define MODE_PS_ON_Gain16		0x0F
#define MODE_PS_ON_Gain1		(0x0 << 2)
#define MODE_PS_ON_Gain4		(0x1 << 2)
#define MODE_PS_ON_Gain8		(0x2 << 2)
#define MODE_PS_ON_Gain16		(0x3 << 2)
#define MODE_PS_StdBy			0x00

#define PS_RANGE1			1
#define PS_RANGE2			2
#define PS_RANGE4			4
#define PS_RANGE8			8

#define ALS_RANGE1_320			1
#define ALS_RANGE2_64K			2

/* Power On response time in ms */
#define PON_DELAY			600
#define WAKEUP_DELAY			10

#define  NOISE_HIGH   150
#define  NOISE_LOW    145
#define  COUNT        5
#define  NOISE_MAX   0x600

#define PS_THRES_UP_0_VALUE     0x88
#define PS_THRES_UP_1_VALUE     0x02
#define PS_THRES_LOW_0_VALUE    0x80
#define PS_THRES_LOW_1_VALUE    0x02
#define PS_THRES_UP_VALUE      (PS_THRES_UP_1_VALUE<<8 | PS_THRES_UP_0_VALUE)
#define PS_THRES_LOW_VALUE     (PS_THRES_LOW_1_VALUE<<8 | PS_THRES_LOW_0_VALUE)

#define PS_MAX_VALUE		2047
#define PS_THRES_UP_1_VALUE     0x02



#define APS_REGISTER_COUNT	31
#define APS_REGISTER_BASE	0x80
#define REG_WRITE_FLAG (1<<8)
#define REG_DEFAULT_VALUE 0x00
static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,
    .polling_mode = 1,
    .polling_mode_ps = 0,
    .polling_mode_als = 1,
    .power_id   = 0xffff,    /*LDO is not used*/
    .power_vol  = 0,          /*LDO is not used*/
    .i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
#if 0
    .als_level  = {10,   20,   50,  100,  350,   500, 625, 1000,   5000,  7000,   9600,   10000, 26000,  30000, 65535},
    .als_value  = {1,     1,    1,  100,  100,   100, 100,  100,    100,  1000,  1000,    10000,  10000,  10000, 100000}, //-----the new data
#else // gionee lizhi 20150417 modify for AAL 
    .als_level  = {0,  5,  15,  50,  80,  150, 300,  700,  1400, 2800,  5600,  5600,  5600,  5600},
    .als_value  = {1,  49, 100, 200, 400, 750, 2000, 4000, 6500, 10000, 15000, 15000, 15000, 15000, 15000},
#endif
    .ps_threshold =PS_THRESHOLD,
    .ps_cali_noise={    150, 400, 800, 1547, 2047},
    .ps_threshold_high={170, 190, 200, 210,  220},
    .ps_threshold_low={ 160, 180, 190, 200,  210},
};

int regster_value_table[APS_REGISTER_COUNT]=
{  
    REG_DEFAULT_VALUE, //0x80
    REG_WRITE_FLAG | 0x20, //0x81
    REG_WRITE_FLAG | 0x7f, //0x82
    REG_WRITE_FLAG | 0x05, //0x83
    REG_WRITE_FLAG | 0x00, //0x84
    REG_WRITE_FLAG | 0x01, //0x85
    REG_DEFAULT_VALUE, //0x86
    REG_DEFAULT_VALUE, //0x87
    REG_DEFAULT_VALUE, //0x88
    REG_DEFAULT_VALUE, //0x89
    REG_DEFAULT_VALUE, //0x8a
    REG_DEFAULT_VALUE, //0x8b
    REG_DEFAULT_VALUE, //0x8c
    REG_DEFAULT_VALUE, //0x8d
    REG_DEFAULT_VALUE, //0x8e
    REG_WRITE_FLAG | 0x01, //0x8f
    REG_WRITE_FLAG | 0x88, //0x90
    REG_WRITE_FLAG | 0x02, //0x91
    REG_WRITE_FLAG | 0x80, //0x92
    REG_WRITE_FLAG | 0x02, //0x93
    REG_DEFAULT_VALUE, //0x94
    REG_DEFAULT_VALUE, //0x95
    REG_DEFAULT_VALUE, //0x96
    REG_DEFAULT_VALUE, //0x97
    REG_DEFAULT_VALUE, //0x98
    REG_DEFAULT_VALUE, //0x99
    REG_DEFAULT_VALUE, //0x9a
    REG_DEFAULT_VALUE, //0x9b
    REG_DEFAULT_VALUE, //0x9c
    REG_DEFAULT_VALUE, //0x9d
    REG_WRITE_FLAG | 0x40 //0x9e
};

#endif
