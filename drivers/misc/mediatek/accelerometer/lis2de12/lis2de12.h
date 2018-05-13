/* linux/drivers/hwmon/LIS2DE12.c
 *
 * (C) Copyright 2008 
 * MediaTek <www.mediatek.com>
 *
 * LIS2DE12 driver for MT6516
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef LIS2DE12_H
#define LIS2DE12_H
	 
#include <linux/ioctl.h>

#define LIS2DE12_I2C_SLAVE_ADDR		0x28//0x28<-> SD0=GND;0x29<-> SD0=High
	 
	 /* LIS2DE12 Register Map  (Please refer to LIS2DE12 Specifications) */
#define LIS2DE12_REG_CTL_REG1		0x20
#define LIS2DE12_REG_CTL_REG2		0x21
#define LIS2DE12_REG_CTL_REG3		0x22
#define LIS2DE12_REG_CTL_REG4		0x23
#define LIS2DE12_REG_DATAX0		    0x28
#define LIS2DE12_REG_OUT_X		    0x28
#define LIS2DE12_REG_OUT_Y		    0x2A
#define LIS2DE12_REG_OUT_Z		    0x2C



#define LIS2DE12_FIXED_DEVID			0xE5
	 
#define LIS2DE12_BW_200HZ			0x60
#define LIS2DE12_BW_100HZ			0x50 //400 or 100 on other choise //changed
#define LIS2DE12_BW_50HZ				0x40

#define	LIS2DE12_FULLRANG_LSB		0XFF
	 
#define LIS2DE12_MEASURE_MODE		0x08	//changed 
#define LIS2DE12_DATA_READY			0x07    //changed
	 
//#define LIS2DE12_FULL_RES			0x08
#define LIS2DE12_RANGE_2G			0x00
#define LIS2DE12_RANGE_4G			0x10
#define LIS2DE12_RANGE_8G			0x20 //8g or 2g no ohter choise//changed
//#define LIS2DE12_RANGE_16G			0x30 //8g or 2g no ohter choise//changed

#define LIS2DE12_SELF_TEST           0x10 //changed
	 
#define LIS2DE12_STREAM_MODE			0x80
#define LIS2DE12_SAMPLES_15			0x0F
	 
#define LIS2DE12_FS_8G_LSB_G			0x20
#define LIS2DE12_FS_4G_LSB_G			0x10
#define LIS2DE12_FS_2G_LSB_G			0x00
	 
#define LIS2DE12_LEFT_JUSTIFY		0x04
#define LIS2DE12_RIGHT_JUSTIFY		0x00
	 
	 
#define LIS2DE12_SUCCESS						0
#define LIS2DE12_ERR_I2C						-1
#define LIS2DE12_ERR_STATUS					-3
#define LIS2DE12_ERR_SETUP_FAILURE			-4
#define LIS2DE12_ERR_GETGSENSORDATA			-5
#define LIS2DE12_ERR_IDENTIFICATION			-6
	 
	 
	 
#define LIS2DE12_BUFSIZE				256
	 
#endif

