#ifndef BUILD_LK
#include <linux/string.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
#include "../code/second/cfg.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------


#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)
#define LCM_ID_OTM1287A (0x1284)

#define LCM_DSI_CMD_MODE									0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

static unsigned int need_set_lcm_addr = 1;


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
    memcpy((void*)&lcm_util, (void*)util, sizeof(LCM_UTIL_FUNCS));
}


static void lcd_reset(unsigned char enabled)
{
    if (enabled) {
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    } else {	
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);    	
    }
}

static void lcm_get_params(LCM_PARAMS *params)
{
		memset((void*)params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
#endif
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_THREE_LANE;

		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 14;
		params->dsi.vertical_frontporch					= 16;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 6;
		params->dsi.horizontal_backporch				= 42;
		params->dsi.horizontal_frontporch				= 44;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.clk_lp_per_line_enable = 1;
		params->dsi.cont_clock = 0;

		params->dsi.esd_check_enable=1;
		params->dsi.customization_esd_check_enable=1;
		params->dsi.lcm_esd_check_table[0].cmd=0x0a;
		params->dsi.lcm_esd_check_table[0].count=0x1;
		params->dsi.lcm_esd_check_table[0].para_list[0]=0x9c;
		params->dsi.lcm_esd_check_table[1].cmd=0xac;
		params->dsi.lcm_esd_check_table[1].count=0x1;
		params->dsi.lcm_esd_check_table[1].para_list[0]=0x0;

		params->dsi.noncont_clock=1;
		params->dsi.PLL_CLOCK=250;
}

static void lcm_init(void)
{
#ifdef BUILD_LK
    printf("%s, LK \n", __func__);
#else
    printk("%s, kernel", __func__);
#endif

	lcd_reset(0);
	mt_set_gpio_out(GPIO_LCD_ENP_PIN, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_LCD_ENN_PIN, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO);
	MDELAY(50);
	
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);
	MDELAY(10);
	
	lcd_reset(1);
	MDELAY(5);
	lcd_reset(0);
	MDELAY(20);
	lcd_reset(1);
	MDELAY(5);
	
	mt_set_gpio_out(GPIO_LCD_ENP_PIN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_ENN_PIN, GPIO_OUT_ONE);
	MDELAY(20);
	lcd_reset(0);
	MDELAY(20);
	lcd_reset(1);
	MDELAY(130);


	push_table(lcm_vdo_initialization_setting, sizeof(lcm_vdo_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
#ifdef BUILD_LK
    printf("%s, LK \n", __func__);
#else
    printk("%s, kernel", __func__);
#endif

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

	lcd_reset(0);
	MDELAY(2);
	mt_set_gpio_out(GPIO_LCD_ENN_PIN, GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_LCD_ENP_PIN, GPIO_OUT_ZERO);
	MDELAY(5);
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO);
	MDELAY(10);

}


static void lcm_resume(void)
{
#ifdef BUILD_LK
	printf("%s, LK \n", __func__);
#else
	printk("%s, kernel", __func__);
#endif

	lcm_init();
}


#if 0
#ifndef BUILD_LK
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif

static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];
	int ret = 0;
	
	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0F, buffer, 1);
	if(buffer[0] != 0xc0)
	{
		printk("[LCM ERROR] [0x0F]=0x%02x\n", buffer[0]);
		ret++;
	}

	read_reg_v2(0x05, buffer, 1);
	if(buffer[0] != 0x00)
	{
		printk("[LCM ERROR] [0x05]=0x%02x\n", buffer[0]);
		ret++;
	}
	
	read_reg_v2(0x0A, buffer, 1);
	if((buffer[0]&0xf)!=0x0C)
	{
		printk("[LCM ERROR] [0x0A]=0x%02x\n", buffer[0]);
		ret++;
	}

	// return TRUE: need recovery
	// return FALSE: No need recovery
	if(ret)
	{
		return TRUE;
	}
	else
	{			 
		return FALSE;
	}
#else
	return FALSE;
 #endif
}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	lcm_resume();

	return TRUE;
}
#endif

#if 0
static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];  

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	
	SET_RESET_PIN(1);
	MDELAY(20); 

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; //we only need ID
    #ifdef BUILD_LK
		printf("%s, LK otm1282a debug: nt35590 id = 0x%08x\n", __func__, id);
    #else
		printk("%s, kernel otm1282a horse debug: otm1282a id = 0x%08x\n", __func__, id);
    #endif

    if(id == LCM_ID_OTM1282A)
    	return 1;
    else
        return 0;

}
#endif

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER otm1287a_tdt_wxga_dsi_vdo_lcm_drv = 
{
    .name			= "otm1287a_tdt_wxga_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	//.compare_id     = lcm_compare_id,
	//.esd_check   	= lcm_esd_check,
    //.esd_recover	= lcm_esd_recover,
};
