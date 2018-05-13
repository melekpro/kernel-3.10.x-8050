#
# Makefile for MELFAS MIP4 Touchscreen driver (Mediatek)
#

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(MTK_PROJECT_PATH_ROOT)/touchpanel/melfas_mip4/

#Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/melfas_mip4/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/


ifeq ($(CONFIG_MTK_PLATFORM),"mt6572")
    ccflags-y += -DMT6572
endif
ifeq ($(CONFIG_MTK_PLATFORM),"mt6582")
    ccflags-y += -DMT6582
endif


obj-$(CONFIG_TOUCHSCREEN_MTK_MELFAS_MMS438)	+= mip4.o mip4_mod.o mip4_debug.o mip4_fw_mms438.o
#obj-$(CONFIG_TOUCHSCREEN_MTK_MELFAS_MMS449)	+= mip4.o mip4_mod.o mip4_debug.o mip4_fw_mms438.o
#obj-$(CONFIG_TOUCHSCREEN_MTK_MELFAS_MMS458)	+= mip4.o mip4_mod.o mip4_debug.o mip4_fw_mms438.o
#obj-$(CONFIG_TOUCHSCREEN_MTK_MELFAS_MMS500)	+= mip4.o mip4_mod.o mip4_debug.o mip4_fw_mms500.o
#obj-$(CONFIG_TOUCHSCREEN_MTK_MELFAS_MCS8040L)	+= mip4.o mip4_mod.o mip4_debug.o mip4_fw_mcs8040l.o
#obj-$(CONFIG_TOUCHSCREEN_MTK_MELFAS_MIT200) 	+= mip4.o mip4_mod.o mip4_debug.o mip4_fw_mit200.o
#obj-$(CONFIG_TOUCHSCREEN_MTK_MELFAS_MIT300)	+= mip4.o mip4_mod.o mip4_debug.o mip4_fw_mit300.o

