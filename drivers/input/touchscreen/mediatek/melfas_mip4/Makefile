#
# Makefile for MELFAS MIP4 Touchscreen driver (Mediatek)
#

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(MTK_PROJECT_PATH_ROOT)/touchpanel/melfas_mip4/

#Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/melfas_mip4/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

ifeq ($(CONFIG_MTK_PLATFORM),"mt6582")
    ccflags-y += -DMT6582
endif

obj-y += melfas_mms400.o melfas_mms400_mod.o melfas_mms400_cmd.o melfas_mms400_test.o melfas_mms438_fw_update.o


