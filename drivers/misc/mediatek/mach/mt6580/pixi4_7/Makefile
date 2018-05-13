
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif

ifneq ($(CONFIG_ARM64), y)
obj-y   +=  camera/camera/
endif
obj-y   +=  flashlight/constant_flashlight/
obj-$(CONFIG_MTK_VIBRATOR)   +=  vibrator/
obj-$(CONFIG_MTK_LEDS)   +=  leds/
#obj-y   +=  keypad/
#obj-y   +=  sound/
obj-$(CONFIG_MTK_ACCDET)   +=  accdet/

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)

ifeq ($(CONFIG_MTK_BMA222E_NEW),y)
obj-y   +=  accelerometer/bma222E-new/
endif

ifeq ($(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER),y)
obj-y   +=  accelerometer/autoDetectAccelerometer/

else

ifeq ($(CONFIG_MTK_LIS2DE12),y)
obj-y   +=  accelerometer/lis2de12/
endif

ifeq ($(CONFIG_MTK_MXC4005),y)
obj-y   +=  accelerometer/mxc4005/
endif
endif

ifeq ($(CONFIG_MTK_STK3X1X),y)
obj-y   +=  alsps/stk3x1x/
endif


ifeq ($(CONFIG_MTK_S62X),y)
obj-y	+=  magnetometer/s62x/
endif
endif
