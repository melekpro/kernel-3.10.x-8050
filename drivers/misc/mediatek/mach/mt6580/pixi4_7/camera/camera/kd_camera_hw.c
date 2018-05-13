#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <linux/regulator/consumer.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mach/upmu_common.h>
#include <mach/upmu_common_sw.h>



/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_info( PFX  fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         pr_err( PFX  fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   pr_info( PFX  fmt, ##arg); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif

// test
#define TEST_SUBCAMERA_SLEEP_FOREVER	0

/*
#ifndef BOOL
typedef unsigned char BOOL;
#endif
*/

/* Mark: need to verify whether ISP_MCLK1_EN is required in here //Jessy @2014/06/04*/
extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);

//extern struct platform_device *camerahw_platform_device;
extern struct device *sensor_device;

struct regulator *regVCAMA = NULL;
struct regulator *regVCAMD = NULL;
struct regulator *regVCAMIO = NULL;
struct regulator *regVCAMAF = NULL;
struct regulator *regVCAMD_SUB = NULL;

bool _hwPowerOn(char * powerId, int powerVolt, struct regulator **regVCAM){
    bool ret = false;
	struct regulator * temp = NULL;
		PK_DBG("[_hwPowerOn]before get, powerId:%s, regVCAM: %p\n", powerId, *regVCAM );
	if(*regVCAM == NULL)
		*regVCAM = regulator_get(sensor_device,powerId);
	temp = *regVCAM;

	PK_DBG("[_hwPowerOn]after get, powerId:%s, regVCAM: %p\n", powerId, temp );


 	if(!IS_ERR(temp)){
        if(powerId != CAMERA_POWER_VCAM_IO && regulator_set_voltage(temp , powerVolt,powerVolt)!=0 ){
             PK_DBG("[_hwPowerOn]fail to regulator_set_voltage, powerVolt:%d, powerID: %s\n", powerVolt, powerId);
        }
        if(regulator_enable(temp)!= 0) {
            PK_DBG("[_hwPowerOn]fail to regulator_enable, powerVolt:%d, powerID: %s\n", powerVolt, powerId);
            //regulator_put(regVCAM);
            //regVCAM = NULL;
            return ret;
        }
        ret = true;
    }else
   		 PK_DBG("[_hwPowerOn]IS_ERR_OR_NULL regVCAM %s\n",powerId);

    return ret;
}

bool _hwPowerDown(char * powerId, struct regulator **regVCAM){
    bool ret = false;
	struct regulator * temp = *regVCAM;
    if(!IS_ERR(temp)){
		#if 1
        if(regulator_is_enabled(temp)) {
            PK_DBG("[_hwPowerDown]before disable %s is enabled\n", powerId);
        }
		#endif
		if(regulator_disable(temp)!=0)
            PK_DBG("[_hwPowerDown]fail to regulator_disable, powerID: %s\n", powerId);
		//for SMT stage, put seems always fail?
       // regulator_put(regVCAM);
       // regVCAM = NULL;
        ret = true;
    } else {
        PK_DBG("[_hwPowerDown]%s fail to power down  due to regVCAM == NULL regVCAM 0x%p\n", powerId,temp );
    }
    return ret;
}

u32 gl_camera_index;

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{

u32 pinSetIdx = 0;//default main sensor

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000

u32 pinSet[3][8] = {
                        //for main sensor
                     {  CAMERA_CMRST_PIN, // The reset pin of main sensor uses GPIO10 of mt6306, please call mt6306 API to set
                        CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,              /* ON state */
                        GPIO_OUT_ZERO,             /* OFF state */
                        CAMERA_CMPDN_PIN,
                        CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for sub sensor
                     {  CAMERA_CMRST1_PIN,
                        CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        CAMERA_CMPDN1_PIN,
                        CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for main_2 sensor
                     {  GPIO_CAMERA_INVALID,
                        GPIO_CAMERA_INVALID,   /* mode */
                        GPIO_OUT_ONE,               /* ON state */
                        GPIO_OUT_ZERO,              /* OFF state */
                        GPIO_CAMERA_INVALID,
                        GPIO_CAMERA_INVALID,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     }
                   };



    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }

	gl_camera_index = pinSetIdx;

	PK_DBG("[Power %s]pinSetIdx:%d, currSensorName: %s\n", (On ? "ON" : "OFF"), pinSetIdx, currSensorName);
	
    //power ON
    if (On) {

        printk("[Poweron]pinSetIdx:%d, currSensorName:%s\n", pinSetIdx, currSensorName);
        if (currSensorName && pinSetIdx == 0 && (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV5670_MIPI_RAW)))
        {
        	// disable inactive sensor
        	  ISP_MCLK1_EN(1);
    #if 0
            if (GPIO_CAMERA_INVALID != pinSet[1 - pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[1 - pinSetIdx][IDX_PS_CMPDN],pinSet[1 - pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[1 - pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[1 - pinSetIdx][IDX_PS_CMPDN],pinSet[1 - pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            
            if (GPIO_CAMERA_INVALID != pinSet[1 - pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[1 - pinSetIdx][IDX_PS_CMRST],pinSet[1 - pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[1 - pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[1 - pinSetIdx][IDX_PS_CMRST],pinSet[1 - pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }
#endif
			
			//First Power Pin low and Reset Pin Low
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
			}         
				   
			printk("ov5670 power on\n");

            //VCAM_A
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800, &regVCAMA))//avdd
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %s\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }

			mdelay(5);

				if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
			}
           //VCAM_IO
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))//dovdd
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }

			mdelay(5);

            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200, &regVCAMD))//dvdd
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %s \n", CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

			mdelay(5);

            //AF_VCC
      /*      if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,&regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }*/

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}

            }
           
        }
        else if (currSensorName && pinSetIdx == 0 && (0 == strcmp(SENSOR_DRVNAME_SP2508_RAW ,currSensorName))) 
        {

          ISP_MCLK1_EN(1);
		  
		           if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                   if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                   if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
			
					if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
					mdelay(1);
				printk("sp2508 power on\n");

				 if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800, &regVCAMA))
				  {
					  PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					  goto _kdCISModulePowerOn_exit_;
				  }
				   msleep(1);
				  if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
				  {
					  PK_DBG("[CAMERA SENSOR] Fail to enable IO power\n");
					  goto _kdCISModulePowerOn_exit_;
				  }
				  msleep(1);
				  if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800, &regVCAMD))
                 {
                      PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %s \n", CAMERA_POWER_VCAM_D);
                      goto _kdCISModulePowerOn_exit_;
                 }

				  msleep(1);
				  
				  mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ONE);
                  mdelay(100);
				  mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ZERO);
                  mdelay(1);
			     
				  if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

				  // wait power to be stable 
				  mdelay(5);		
		    }
		           
		    else if (currSensorName && pinSetIdx == 1 && (0 == strcmp(SENSOR_DRVNAME_GC2355_MIPI_RAW ,currSensorName))) 
        {

	
			    // add end
			    if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			   	  if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
			    }

			    //RST pin
			    if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
				    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
				    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			    }

			      mdelay(5);

            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }

			mdelay(1);

			printk("gc2355 power on \n");
			#if 0
            if(TRUE != _hwPowerOn(SUB_CAMERA_POWER_VCAM_D, VOL_1800, &regVCAMD))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %s \n", CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

			#else
			//pmic_set_register_value(PMIC_RG_VIBR_EN, 0);
			printk("xxxxxxxxxxxxxxxxxxxxx\n");
			pmic_set_register_value(PMIC_RG_VIBR_VOSEL,0x3);
			pmic_set_register_value(PMIC_RG_VIBR_EN, 1);
				#endif
				
			printk("gc2355 power off ffff \n");
             mdelay(1);
            if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800, &regVCAMA))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %s\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
			
			ISP_MCLK2_EN(1);

			      mdelay(1);

			     //enable active sensor
			     if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				     //PDN pin
				     if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				     if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				     if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			     }

			     mdelay(1);
			     //RST pin
			    if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST], pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
				    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
				    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			    }		
		    } 
		    else if (currSensorName && pinSetIdx == 1 && (0 == strcmp(SENSOR_DRVNAME_GC0310_MIPI_YUV ,currSensorName))) 
        {
          if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}        	

				  if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, &regVCAMIO))
				  {
					  PK_DBG("[CAMERA SENSOR] Fail to enable IO power\n");
					  goto _kdCISModulePowerOn_exit_;
				  }
                  printk("gc0310 power on\n");
				  msleep(10);

				  if(TRUE != _hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800, &regVCAMA))
				  {
					  PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					  goto _kdCISModulePowerOn_exit_;
				  }

          ISP_MCLK2_EN(1);			
					mdelay(10);
				  if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
          mdelay(10);
          if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN], pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}	
				  // wait power to be stable 
				  mdelay(10);		
		    }
    }
    else {//power OFF

        if (currSensorName && pinSetIdx == 0 && (0 == strcmp(currSensorName, SENSOR_DRVNAME_OV5670_MIPI_RAW))) {
            ISP_MCLK1_EN(0);
			      mdelay(1);
            //Set Power Pin low and Reset Pin Low
                     
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
            }
			printk("ov5670 power off \n");

			mdelay(5);
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_D, &regVCAMD))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %s \n",CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }
			mdelay(5);

		
			//VCAM_IO
			if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %s \n", CAMERA_POWER_VCAM_IO);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
			
			 if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

			mdelay(5);

				  //VCAM_A
            if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A, &regVCAMA)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%s) \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
             //AF_VCC
    /*        if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_AF, &regVCAMAF))
            {
                PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %s \n", CAMERA_POWER_VCAM_AF);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }*/

        }     
        else if (currSensorName && pinSetIdx == 0 && (0 == strcmp(SENSOR_DRVNAME_SP2508_RAW,currSensorName)))
        {
            ISP_MCLK1_EN(0);

			  if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

			  
				    if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {

					    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
				    }

					printk("sp2508 power off \n");
				mdelay(1);
					 if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_D,&regVCAMD))
                 {
                      PK_DBG("[CAMERA SENSOR] Fail to OFF a dvdd power\n");
                   
                 }
					 mdelay(1);


					  if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO))
				    {
					    PK_DBG("[CAMERA SENSOR] Fail to OFF d2 digital power\n");
				    }
					  mdelay(1);

				    if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A ,&regVCAMA)) {
					    PK_DBG("[CAMERA SENSOR] Fail to OFF a power\n");
				    }
				    mdelay(5);

					

				   
			
		    }
		    
		    else if (currSensorName && pinSetIdx == 1 && (0 == strcmp(SENSOR_DRVNAME_GC2355_MIPI_RAW,currSensorName)))
        {
           

			   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			   	  if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
			     }  
			   if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
				  }

			   mdelay(1);

			    if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			   	  if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
			    }            
          		mdelay(1);
          
				  if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
					    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
					    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
					    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
				  }
				    
             ISP_MCLK2_EN(0);
			 mdelay(1);
				    if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A ,&regVCAMA)) {
					    PK_DBG("[CAMERA SENSOR] Fail to OFF a power\n");
				    }
				    mdelay(1);
				    /*
				    if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_D, &regVCAMD))
				    {
					    PK_DBG("[CAMERA SENSOR] Fail to OFF d2 digital power\n");
				    }	
				    */
				    pmic_set_register_value(PMIC_RG_VIBR_EN,0); 	//dvdd
				    mdelay(1);
				    if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO, &regVCAMIO))
				    {
					    PK_DBG("[CAMERA SENSOR] Fail to OFF d2 digital power\n");
				    }
				    mdelay(1);
				    
			     if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			   	  if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
			     }   				    
	
		    }
		    else if (currSensorName && pinSetIdx == 1 && (0 == strcmp(SENSOR_DRVNAME_GC0310_MIPI_YUV ,currSensorName))) 
        {
        	
        	mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_OUT_ONE);
        	mdelay(1) ;
        	ISP_MCLK2_EN(0);
        	mdelay(1);
				printk("gc0310 power off \n");
				  if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_A, &regVCAMA))
				  {
					  PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
					  goto _kdCISModulePowerOn_exit_;
				  }	
        		msleep(1);
				  if(TRUE != _hwPowerDown(CAMERA_POWER_VCAM_IO , &regVCAMIO))
				  {
					  PK_DBG("[CAMERA SENSOR] Fail to enable IO power\n");
					  goto _kdCISModulePowerOn_exit_;
				  }
		    }       
    }
	PK_DBG("[Power %s]pinSetIdx:%d, currSensorName: %s Ends...\n", (On ? "ON" : "OFF"), pinSetIdx, currSensorName);

    return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;

}

EXPORT_SYMBOL(kdCISModulePowerOn);
