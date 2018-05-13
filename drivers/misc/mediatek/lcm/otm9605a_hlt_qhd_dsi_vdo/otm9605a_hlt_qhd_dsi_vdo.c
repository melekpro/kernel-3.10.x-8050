/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2013. All rights reserved.
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

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifdef BUILD_LK
#include <string.h>
#include <mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <linux/string.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#include "lcm_drv.h"
#include <cust_gpio_usage.h>
#if defined(BUILD_LK)
#define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif
#include <linux/regulator/consumer.h>
#include "../code/first/cfg.h"
#include <mach/upmu_hw.h>
extern U32 pmic_config_interface (U32 RegNum, U32 val, U32 MASK, U32 SHIFT);
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
// pixel
#define FRAME_WIDTH  			(540)
#define FRAME_HEIGHT 			(960)
// physical dimension
#define PHYSICAL_WIDTH        (68)
#define PHYSICAL_HIGHT         (121)
#define LCM_REGULATOR "/sys/class/graphics/fb0"

#define LCM_ID       (0x40)
#define LCM_DSI_CMD_MODE		0

#define REGFLAG_DELAY 0xAB
#define REGFLAG_END_OF_TABLE 0xAA // END OF REGISTERS MARKER

/*
 power +5V,-5v
*/
#define GPIO_PAVDD_EN_PIN (GPIO12 | 0x80000000)
//#define GPIO_NAVDD_EN_PIN (GPIO15 | 0x80000000)
#define GPIO_NAVDD_EN_PIN (GPIO21 | 0X80000000)
/*
RESET PIN
*/
#define GPIO_LCM_REST_PIN (GPIO70 | 0x80000000)

/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.9.16*/
#define GPIO_LCM_ID1 	  (GPIO16 | 0x80000000)
#define GPIO_LCM_ID2	  (GPIO20 | 0x80000000)
/* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2015.9.16*/

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V3(para_tbl, size, force_update)   	lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

static unsigned int need_set_lcm_addr = 1;

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table __attribute__ ((unused)) lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {
		unsigned cmd;
		
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
	LCM_PRINT("[LCD] push_table \n");
}
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy((void*)&lcm_util, (void*)util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS * params) 

{ 
	memset(params, 0, sizeof(LCM_PARAMS)); 

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

 //  params->physical_width=PHYSICAL_WIDTH;
 //  params->physical_height=PHYSICAL_HIGHT;

	// enable tearing-free
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine. 

	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;	
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST; 
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB; 
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888; 

	// Highly depends on LCD driver capability. 
	params->dsi.packet_size = 256; 
	// Video mode setting 
	params->dsi.intermediat_buffer_num = 2; 
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888; 

	params->dsi.vertical_sync_active = 1; 
	params->dsi.vertical_backporch = 16; 
	params->dsi.vertical_frontporch = 15; 
	params->dsi.vertical_active_line = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 4;
	params->dsi.horizontal_backporch				= 32;
	params->dsi.horizontal_frontporch				= 32;
	params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;
	params->dsi.noncont_clock=1;

	// Bit rate calculation
	//params->dsi.pll_div1=35;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
	//params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)

	/* ESD or noise interference recovery For video mode LCM only. */
	// Send TE packet to LCM in a period of n frames and check the response.
	//params->dsi.lcm_int_te_monitor = FALSE;
	//params->dsi.lcm_int_te_period = 1;		// Unit : frames

	// Need longer FP for more opportunity to do int. TE monitor applicably.
	//if(params->dsi.lcm_int_te_monitor)
	//	params->dsi.vertical_frontporch *= 2;

	// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.)
	//params->dsi.lcm_ext_te_monitor = FALSE;
	// Non-continuous clock
	//params->dsi.noncont_clock = TRUE;
	//params->dsi.noncont_clock_period = 2;	// Unit : frames

	// DSI MIPI Spec parameters setting
	/*params->dsi.HS_TRAIL = 6;
	params->dsi.HS_ZERO = 9;
	params->dsi.HS_PRPR = 5;
	params->dsi.LPX = 4;
	params->dsi.TA_SACK = 1;
	params->dsi.TA_GET = 20;
	params->dsi.TA_SURE = 6;
	params->dsi.TA_GO = 16;
	params->dsi.CLK_TRAIL = 5;
	params->dsi.CLK_ZERO = 18;
	params->dsi.LPX_WAIT = 1;
	params->dsi.CONT_DET = 0;
	params->dsi.CLK_HS_PRPR = 4;*/
	// Bit rate calculation
	params->dsi.PLL_CLOCK = 217;

	LCM_PRINT("[LCD] lcm_get_params \n");

}

static void init_lcm_registers(void)
{
	unsigned int data_array[32];

	dsi_set_cmdq_V3(lcm_initialization_setting_V3, sizeof(lcm_initialization_setting_V3) / sizeof(LCM_setting_table_V3), 1);
	LCM_PRINT("[LCD] init_lcm_registers \n");
}

static void init_lcm_registers_added(void)
{
	unsigned int data_array[1];
	MDELAY(5);
	data_array[0] = 0x00110500;	//sleep out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00290500;	//Display On
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
	LCM_PRINT("[LCD] init_lcm_registers_added \n");
}

static void init_lcm_registers_sleep(void)
{
	unsigned int data_array[1];

	MDELAY(10);
	data_array[0] = 0x00280500;	//Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(150);
	data_array[0] = 0x00100500;	//sleep in
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(150);
	LCM_PRINT("[LCD] init_lcm_registers_sleep \n");
}

/* vgp3 1.8v LDO enable */
static void ldo_1v8io_on(void)
{
#ifdef BUILD_UBOOT 
	#error "not implemeted"
#elif defined(BUILD_LK) 	
	// IOVCC 1.8v LDO on
	upmu_set_rg_vgp3_vosel(3); 
	upmu_set_rg_vgp3_en(1);
#else
	pmic_config_interface(MT6350_PMIC_RG_VGP3_VOSEL_ADDR, 0x6, MT6350_PMIC_RG_VGP3_STB_SEL_MASK, MT6350_PMIC_RG_VGP3_STB_SEL_SHIFT);
	pmic_config_interface(MT6350_PMIC_RG_VGP3_EN_ADDR, 0x1, MT6350_PMIC_RG_VGP3_EN_MASK, MT6350_PMIC_RG_VGP3_EN_SHIFT);
#if 0
	struct regulator * temp = NULL;
	int ret;
	temp=regulator_get(LCM_REGULATOR,MT6325_POWER_LDO_VGP3);
	ret = regulator_set_voltage(temp, 1800000, 1800000);
	ret = regulator_enable(temp);
	//hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "1V8_LCD_VIO_MTK_S");
	#endif
#endif 
}

/* vgp3 1.8v LDO disable */
static void ldo_1v8io_off(void)
{
#ifdef BUILD_UBOOT 
#error "not implemeted"
#elif defined(BUILD_LK) 	
	upmu_set_rg_vgp3_en(0);
#else
	pmic_config_interface(MT6350_PMIC_RG_VGP3_VOSEL_ADDR, 0x0, MT6350_PMIC_RG_VGP3_STB_SEL_MASK, MT6350_PMIC_RG_VGP3_STB_SEL_SHIFT);
	pmic_config_interface(MT6350_PMIC_RG_VGP3_EN_ADDR, 0x0, MT6350_PMIC_RG_VGP3_EN_MASK, MT6350_PMIC_RG_VGP3_EN_SHIFT);
#if 0
	struct regulator * temp = NULL;
	int ret;
	temp=regulator_get(LCM_REGULATOR,MT6325_POWER_LDO_VGP3);
	ret = regulator_set_voltage(temp, 1800000, 1800000);
	ret = regulator_disable(temp);

	//hwPowerDown(MT6323_POWER_LDO_VGP3, "1V8_LCD_VIO_MTK_S");
	#endif
#endif 
}

static void ldo_ext_3v0_on(void)
{
#if 1 // L80P_FIX
#ifdef BUILD_UBOOT 
	#error "not implemeted"
#elif defined(BUILD_LK)
	upmu_set_rg_vcam_af_vosel(5);
	upmu_set_rg_vcam_af_en(1);
#else
	//hwPowerOn(MT6323_POWER_LDO_VCAM_AF, VOL_2800, "2V8_LCD_VCC_MTK_S");
#endif
#else
	mt_set_gpio_mode(GPIO_LCM_PWR, GPIO_LCM_PWR_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_LCM_PWR, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCM_PWR, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ONE);
#endif
}

static void ldo_ext_3v0_off(void)
{
#if 1 // L80P_FIX
#ifdef BUILD_UBOOT 
	#error "not implemeted"
#elif defined(BUILD_LK)
	upmu_set_rg_vcam_af_en(0);
#else
	//hwPowerDown(MT6323_POWER_LDO_VCAM_AF, "2V8_LCD_VCC_MTK_S");
#endif
#else
	mt_set_gpio_mode(GPIO_LCM_PWR, GPIO_LCM_PWR_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_LCM_PWR, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCM_PWR, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ZERO);
#endif
}

/*
DSV power +5V,-5v
*/
/*
static void ldo_p5m5_dsv_3v0_on(void)
{
#if 0
	mt_set_gpio_mode(GPIO_DSV_EN, GPIO_DSV_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_DSV_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_DSV_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_DSV_EN, GPIO_OUT_ONE);
#else	
	mt_set_gpio_mode(GPIO_DSV_AVDD_EN, GPIO_DSV_AVDD_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_DSV_AVDD_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_DSV_AVDD_EN, GPIO_DIR_OUT);
	mt_set_gpio_mode(GPIO_DSV_AVEE_EN, GPIO_DSV_AVEE_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_DSV_AVEE_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_DSV_AVEE_EN, GPIO_DIR_OUT);
	
	mt_set_gpio_out(GPIO_DSV_AVDD_EN, GPIO_OUT_ONE);
	MDELAY(1);
	mt_set_gpio_out(GPIO_DSV_AVEE_EN, GPIO_OUT_ONE);
#endif
}*/
/*
static void ldo_p5m5_dsv_3v0_off(void)
{
#if 0
	mt_set_gpio_mode(GPIO_DSV_EN, GPIO_DSV_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_DSV_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_DSV_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_DSV_EN, GPIO_OUT_ZERO);
#else
	mt_set_gpio_mode(GPIO_DSV_AVDD_EN, GPIO_DSV_AVDD_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_DSV_AVDD_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_DSV_AVDD_EN, GPIO_DIR_OUT);
	mt_set_gpio_mode(GPIO_DSV_AVEE_EN, GPIO_DSV_AVEE_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_DSV_AVEE_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_DSV_AVEE_EN, GPIO_DIR_OUT);
	
	mt_set_gpio_out(GPIO_DSV_AVDD_EN, GPIO_OUT_ZERO);
	MDELAY(1);
	mt_set_gpio_out(GPIO_DSV_AVEE_EN, GPIO_OUT_ZERO);
#endif
}*/

/*
static void reset_lcd_module(unsigned char reset)
{
	mt_set_gpio_mode(GPIO_LCM_RST, GPIO_LCM_RST_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_LCM_RST, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);

   if(reset){
   	mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
      MDELAY(50);	
   }else{
   	mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
   }
}
   */

static void lcm_init(void)
{

	mt_set_gpio_mode(GPIO_LCM_REST_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_REST_PIN, GPIO_DIR_OUT);

        mt_set_gpio_mode(GPIO_NAVDD_EN_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_NAVDD_EN_PIN, GPIO_DIR_OUT);

        mt_set_gpio_mode(GPIO_PAVDD_EN_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_PAVDD_EN_PIN, GPIO_DIR_OUT);


	/*
		INIT ALL PIN STATE.
	*/
	mt_set_gpio_out(GPIO_LCM_REST_PIN,GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_PAVDD_EN_PIN,GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_NAVDD_EN_PIN,GPIO_OUT_ZERO);
	ldo_1v8io_off();
	MDELAY(10);
	 /*
		ldo  1.8 v enable
	 */
	 ldo_1v8io_on();
	 MDELAY(10);
	 /*PAVDD NAVDD 5v*/
	 mt_set_gpio_out(GPIO_PAVDD_EN_PIN,GPIO_OUT_ONE);
	 MDELAY(10);
	 mt_set_gpio_out(GPIO_NAVDD_EN_PIN,GPIO_OUT_ONE);
	 MDELAY(10);
	/* 
	/ reset pin High > delay 1ms > Low > delay 1ms > High
	/ LCD RESET PIN 
	*/	
	mt_set_gpio_out(GPIO_LCM_REST_PIN,GPIO_OUT_ONE);
	MDELAY(2);
	mt_set_gpio_out(GPIO_LCM_REST_PIN,GPIO_OUT_ZERO);
	MDELAY(15);
	mt_set_gpio_out(GPIO_LCM_REST_PIN,GPIO_OUT_ONE);
	MDELAY(50);

	init_lcm_registers();	//SET EXTC 

	//MDELAY(80);
	init_lcm_registers_added();	//Display On
	need_set_lcm_addr = 1;

	LCM_PRINT("[LCD] lcm_init \n");
}

static void lcm_suspend(void)
{
	init_lcm_registers_sleep();

	mt_set_gpio_mode(GPIO_LCM_REST_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_REST_PIN, GPIO_DIR_OUT);

        mt_set_gpio_mode(GPIO_NAVDD_EN_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_NAVDD_EN_PIN, GPIO_DIR_OUT);

        mt_set_gpio_mode(GPIO_PAVDD_EN_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_PAVDD_EN_PIN, GPIO_DIR_OUT);

	//power down reset
	mt_set_gpio_out(GPIO_LCM_REST_PIN,GPIO_OUT_ZERO);
	MDELAY(20);
	//power off NAVDD PAVDD
	mt_set_gpio_out(GPIO_NAVDD_EN_PIN,GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(GPIO_PAVDD_EN_PIN,GPIO_OUT_ZERO);
	MDELAY(10);
	//VCI/IOVCC off
	ldo_1v8io_off();
	LCM_PRINT("[LCD] lcm_suspend \n");
}

static void lcm_resume(void)
{
	lcm_init();
    need_set_lcm_addr = 1;
	LCM_PRINT("[LCD] lcm_resume \n");
}

static void lcm_esd_recover(void)
{
	lcm_suspend();
	lcm_resume();

	LCM_PRINT("[LCD] lcm_esd_recover \n");
}

static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	// need update at the first time
	if(need_set_lcm_addr)
	{
		data_array[0]= 0x00053902;
		data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
		data_array[2]= (x1_LSB);
		dsi_set_cmdq(data_array, 3, 1);
		
		data_array[0]= 0x00053902;
		data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
		data_array[2]= (y1_LSB);
		dsi_set_cmdq(data_array, 3, 1);		
		need_set_lcm_addr = 0;
	}
	
	data_array[0]= 0x002c3909;
   dsi_set_cmdq(data_array, 1, 0);
	LCM_PRINT("[LCD] lcm_update \n");	
}

static unsigned int lcm_compare_id(void)
{
#if 0
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];  
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(10);//Must over 6 ms
	array[0]=0x00043902;
	array[1]=0x8983FFB9;// page enable
	dsi_set_cmdq(array, 2, 1);
	MDELAY(10);
	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);
	read_reg_v2(0xDC, buffer, 2);
	id = buffer[0]; 
	LCM_PRINT("%s, id = 0x%08x\n", __func__, id);
	return (LCM_ID == id)?1:0;
	return 1;
#endif	

/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.9.16*/
	int gpio1_state = 0;
	int gpio2_state = 0;

	ldo_1v8io_on();

	mt_set_gpio_mode(GPIO_LCM_ID1, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCM_ID1, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_LCM_ID2, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_ID2, GPIO_DIR_IN);

	MDELAY(10);

	gpio1_state = mt_get_gpio_in(GPIO_LCM_ID1);
	gpio2_state = mt_get_gpio_in(GPIO_LCM_ID2);

	ldo_1v8io_off();
	if ((gpio1_state == 1) && (gpio2_state == 0))
	{
		LCM_PRINT("[LCD] otm9605a_hlt_qhd_dsi_vdo select!\n");
		return 1;
	}
	else
		return 0;
/* [PLATFORM]-Add-END by TCTSZ.lizhi.wu, 2015.9.16*/
}
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER otm9605a_hlt_qhd_dsi_vdo = {
	.name = "otm9605a_hlt_qhd_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	//.update = lcm_update,
#if (!defined(BUILD_UBOOT) && !defined(BUILD_LK))
	.esd_recover = lcm_esd_recover,
#endif
};
