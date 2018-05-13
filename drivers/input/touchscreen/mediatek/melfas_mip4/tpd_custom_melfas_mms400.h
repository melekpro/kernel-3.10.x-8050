/*
 * MELFAS MMS400 Touchscreen for MediaTek
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 */

#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__
#include <pmic_drv.h>

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
//#define TPD_POWER_SOURCE	MT65XX_POWER_LDO_VGP2
#define TPD_POWER_SOURCE_CUSTOM PMIC_APP_CAP_TOUCH_VDD
#define TPD_I2C_BUS		1
#define TPD_I2C_ADDR		0x48
#define TPD_WAKEUP_TRIAL	60
#define TPD_WAKEUP_DELAY	100

//#define TPD_HAVE_TREMBLE_ELIMINATION
//#define TPD_HAVE_CALIBRATION

//#define TPD_LDO MT6328_POWER_LDO_VGP1


//Screen
//#define TPD_RES_X	1080
//#define TPD_RES_Y	1920
#define TPD_RES_X	540
#define TPD_RES_Y	960
#define LCD_X TPD_RES_X //480
#define LCD_Y TPD_RES_Y //854

//#define	TPD_XY_INVERT
//#define	TPD_X_INVERT
//#define	TPD_Y_INVERT

//Key
/* Define the virtual button mapping */
#define TPD_HAVE_BUTTON
#define TPD_BUTTON_HEIGH	(100)
#define TPD_KEY_COUNT		3
#define TPD_KEYS			{KEY_BACK, KEY_HOMEPAGE, KEY_MENU}
#define TPD_KEYS_DIM		{{80,850,160,TPD_BUTTON_HEIGH},{260,850,160,TPD_BUTTON_HEIGH},{440,850,160,TPD_BUTTON_HEIGH}}

/* Define the touch dimension */
#ifdef TPD_HAVE_BUTTON
#define TPD_TOUCH_HEIGH_RATIO	80
#define TPD_DISPLAY_HEIGH_RATIO	73
#endif

/* Define the 0D button mapping */
#ifdef TPD_HAVE_BUTTON
#define TPD_0D_BUTTON {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#else
#define TPD_0D_BUTTON {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#endif

#endif /* TOUCHPANEL_H__ */

