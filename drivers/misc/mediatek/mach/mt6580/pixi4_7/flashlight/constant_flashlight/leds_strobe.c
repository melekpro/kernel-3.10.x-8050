#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>



/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        pr_warning(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __FUNCTION__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


//#define STROBE_DEVICE_ID 0x30
#define STROBE_DEVICE_I2C_ADDR  0x63
#define I2C_STROBE_MAIN_CHANNEL 2


static struct work_struct workTimeOut;

#ifndef GPIO_CAMERA_FLASH_MODE_STRB_PIN
#define GPIO_CAMERA_FLASH_MODE_STRB_PIN GPIO83
#endif

#ifndef GPIO_CAMERA_FLASH_MODE_TRCH_PIN
#define GPIO_CAMERA_FLASH_MODE_TRCH_PIN GPIO84
#endif

#ifndef GPIO_CAMERA_FLASH_MODE_TX_PIN
#define GPIO_CAMERA_FLASH_MODE_TX_PIN GPIO19
#endif

#ifndef GPIO_CAMERA_FLASH_EN_PIN
#define GPIO_CAMERA_FLASH_EN_PIN GPIO1
#endif

#define FLASH_GPIO_EN          GPIO_CAMERA_FLASH_EN_PIN
#define FLASH_MODE_FLASH_MODE  GPIO_CAMERA_FLASH_MODE_STRB_PIN
#define FLASH_MODE_TORCH_MODE  GPIO_CAMERA_FLASH_MODE_TRCH_PIN
#define FLASH_MODE_TX_MODE     GPIO_CAMERA_FLASH_MODE_TX_PIN


#define KTD2682_REG_ChipID 0x0c
#define KTD2682_REG_FLASH_Current_Set_LED1 3
#define KTD2682_REG_FLASH_Current_Set_LED2 4
#define KTD2682_REG_TORCH_Current_Set_LED1 5
#define KTD2682_REG_TORCH_Current_Set_LED2 6

#define KTD2682_REG_LVP 2

#define KTD2682_REG_Flash_Timer 8
#define KTD2682_REG_Control 1
#define KTD2682_REG_Fault1 0xa
#define KTD2682_REG_Fault2 0xb
#define KTD2682_REG_Current_Boost 7



static int g_bLtVersion=0;
extern u32 gl_camera_index;

/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

static struct i2c_client *KTD2682_i2c_client = NULL;

int FL_Init(void);



struct KTD2682_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct KTD2682_chip_data {
	struct i2c_client *client;

	//struct led_classdev cdev_flash;
	//struct led_classdev cdev_torch;
	//struct led_classdev cdev_indicator;

	struct KTD2682_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

/* i2c access*/
/*
static int KTD2682_read_reg(struct i2c_client *client, u8 reg,u8 *val)
{
	int ret;
	struct KTD2682_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (ret < 0) {
		PK_ERR("failed reading at 0x%02x error %d\n",reg, ret);
		return ret;
	}
	*val = ret&0xff;

	return 0;
}*/

static int KTD2682_write_reg(struct i2c_client *client, u8 reg, u8 val)
{

	if (client == NULL){
		printk("%s error!\n",__func__);
		return -1;
	}
	int ret=0;
	struct KTD2682_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_ERR("failed writting at 0x%02x\n", reg);
	return ret;
}

static int KTD2682_read_reg(struct i2c_client *client, u8 reg)
{
	if (client == NULL){
		printk("%s error!\n",__func__);
		return -1;
	}
	int val=0;
	struct KTD2682_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}


static void KTD2682_chip_enable()
{
	if(mt_set_gpio_mode(FLASH_GPIO_EN,GPIO_MODE_00)){printk("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(FLASH_GPIO_EN,GPIO_DIR_OUT)){printk("[constant_flashlight] set gpio dir failed!! \n");}
	if(mt_set_gpio_out(FLASH_GPIO_EN,GPIO_OUT_ONE)){printk("[constant_flashlight] set gpio failed!! \n");}
	mdelay(1);
}

static void KTD2682_chip_disable()
{
	if(mt_set_gpio_mode(FLASH_GPIO_EN,GPIO_MODE_00)){printk("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(FLASH_GPIO_EN,GPIO_DIR_OUT)){printk("[constant_flashlight] set gpio dir failed!! \n");}
	if(mt_set_gpio_out(FLASH_GPIO_EN,GPIO_OUT_ZERO)){printk("[constant_flashlight] set gpio failed!! \n");}
	mdelay(1);
}

static int KTD2682_chip_init(struct KTD2682_chip_data *chip)
{

	KTD2682_chip_enable();
	//KTD2682 CHECK DEVICE ID
	if (KTD2682_read_reg(chip->client,KTD2682_REG_ChipID) != 0)
	{
		printk("KTD2682 check device ID error!\n");
		KTD2682_chip_disable();
		return -1;
	}

	KTD2682_chip_disable();
	return 0;
}

static int KTD2682_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct KTD2682_chip_data *chip;
	struct KTD2682_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	gl_camera_index = 0;
	PK_DBG("cuckoo KTD2682_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "KTD2682 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct KTD2682_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		printk("KTD2682 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct KTD2682_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	if(KTD2682_chip_init(chip)<0)
		goto err_chip_init;

	KTD2682_i2c_client = client;
	PK_DBG("KTD2682 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	printk("KTD2682 probe is failed \n");
	return -ENODEV;
}

static int KTD2682_remove(struct i2c_client *client)
{
	struct KTD2682_chip_data *chip = i2c_get_clientdata(client);

    if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define KTD2682_NAME "leds-KTD2682"
static const struct i2c_device_id KTD2682_id[] = {
	{KTD2682_NAME, 0},
	{}
};

static struct i2c_driver KTD2682_i2c_driver = {
	.driver = {
		.name  = KTD2682_NAME,
	},
	.probe	= KTD2682_probe,
	.remove   = KTD2682_remove,
	.id_table = KTD2682_id,
};

struct KTD2682_platform_data KTD2682_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_KTD2682={ I2C_BOARD_INFO(KTD2682_NAME, STROBE_DEVICE_I2C_ADDR), \
													.platform_data = &KTD2682_pdata,};

static int __init KTD2682_init(void)
{
	i2c_register_board_info(I2C_STROBE_MAIN_CHANNEL, &i2c_KTD2682, 1);


	return i2c_add_driver(&KTD2682_i2c_driver);
}

static void __exit KTD2682_exit(void)
{
	i2c_del_driver(&KTD2682_i2c_driver);
}


module_init(KTD2682_init);
module_exit(KTD2682_exit);

MODULE_DESCRIPTION("Flash driver for KTD2682");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

static int readReg(int reg)
{

    int val;
    val = KTD2682_read_reg(KTD2682_i2c_client, reg);
	PK_DBG("cuckoo read reg 0x%x val is:0x%x\n",reg,val);
    return (int)val;
}


int FL_Enable(void)
{
	//mode setting and out on
	
	KTD2682_chip_enable();
	/* main camera pre flash and main flash: g_duty = 1 and 0  sub camera pre flash and main flash: g_duty = 1 and 6*/
	if(g_duty != 1){
	    //PK_DBG("cuckoo  camera main flash FL_Enable line=%d\n",__LINE__);
		KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_FLASH_Current_Set_LED1, 0x1d);  // < 350ma
		KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_FLASH_Current_Set_LED2, 0x02);  //  < 50ma
		KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_Flash_Timer, 0x1d);   //300ms
		if (gl_camera_index == 0) 
			KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_Control, 0x0d);   //main camera flash mode
		else
			KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_Control, 0x0e);   //sub camera flash mode

	}else{
	    //PK_DBG("cuckoo camera pre flash FL_Enable line=%d\n",__LINE__);
		KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_TORCH_Current_Set_LED1, 0x4f);  // < 100.5ma
		KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_TORCH_Current_Set_LED2, 0x06);  //  < 20ma
		if (gl_camera_index == 0) 
			KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_Control, 0x09);   //main camera torch mode and Pre flash
		else
			KTD2682_write_reg(KTD2682_i2c_client,KTD2682_REG_Control, 0x0a);   //sub camera torch mode  and Pre flash

	}
	PK_DBG("cuckoo camera index: %d FL_Enable line=%d\n",gl_camera_index,__LINE__);
	return 0;

}



int FL_Disable(void)
{

	KTD2682_chip_disable();
	PK_DBG("cuckoo FL_Disable line=%d\n",__LINE__);
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG("cuckoo duty: %d FL_dim_duty line=%d\n",duty,__LINE__);
	g_duty = duty;
    return 0;
}




int FL_Init(void)
{

	if (KTD2682_i2c_client == NULL){
		printk("ERROR:KTD2682 I2C CLIENT IS NULL !\n");
		return -1;
	}
	PK_DBG(" cuckoo SUB camera index : %d FL_Init line=%d\n",gl_camera_index,__LINE__);
	return 0;
}


int FL_Uninit(void)
{
	PK_DBG(" cuckoo FL_UNInit line=%d\n",__LINE__);
	FL_Disable();
    //KTD2682_chip_disable();
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("KTD2682 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift,(int)arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",(int)arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
    		if(arg==1)
    		{

    		    int s;
    		    int ms;
    		    if(g_timeOutTimeMs>1000)
            	{
            		s = g_timeOutTimeMs/1000;
            		ms = g_timeOutTimeMs - s*1000;
            	}
            	else
            	{
            		s = 0;
            		ms = g_timeOutTimeMs;
            	}

				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( s, ms*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("cuckoo constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG("cuckoo  constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    gl_camera_index == 0; 
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{

    gl_camera_index == 1; 
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


