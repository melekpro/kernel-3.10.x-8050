/*
 * MELFAS MMS438/449/458 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * Firmware update functions
 *
 */

#include "melfas_mms400.h"

//ISC Info
#define ISC_PAGE_SIZE				128

//ISC Command
#define ISC_CMD_ERASE_ALL			{0xFB,0x4A,0x00,0x15,0x00,0x00}
#define ISC_CMD_ERASE_PAGE			{0xFB,0x4A,0x00,0x8F,0x00,0x00}
#define ISC_CMD_READ_PAGE			{0xFB,0x4A,0x00,0xC2,0x00,0x00}
#define ISC_CMD_WRITE_PAGE			{0xFB,0x4A,0x00,0xA5,0x00,0x00}
#define ISC_CMD_PROGRAM_PAGE		{0xFB,0x4A,0x00,0x54,0x00,0x00}
#define ISC_CMD_READ_STATUS		{0xFB,0x4A,0x36,0xC2,0x00,0x00}
#define ISC_CMD_EXIT				{0xFB,0x4A,0x00,0x66,0x00,0x00}

//ISC Status
#define ISC_STATUS_BUSY				0x96
#define ISC_STATUS_DONE				0xAD

/**
* Read ISC status
*/
static int mms_isc_read_status(struct mms_ts_info *info) {
    //struct i2c_client *client = info->client;
    u8 write_buf[6] =  ISC_CMD_READ_STATUS;
    u8 read_buf[4];
    u8 result = 0;
    int cnt = 100;
    int ret = 0;
    /*
    struct i2c_msg msg[2] = {
    	{
    		.addr = client->addr,
    		.flags = 0,
    		.buf = write_buf,
    		.len = 6,
    	}, {
    		.addr = client->addr,
    		.flags = I2C_M_RD,
    		.buf = &result,
    		.len = 1,
    	},
    };
    */

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    do {
        /*
        if(i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
        	dev_err(&info->client->dev, "%s [ERROR] i2c_transfer\n", __func__);
        	return -1;
        }
        */
        if(mms_i2c_read_normal(info, write_buf, 6, read_buf, 1)) {
            dev_err(&info->client->dev, "%s [ERROR] mms_i2c_read\n", __func__);
            ret = -1;
            goto ERROR;
        }
        result = read_buf[0];

        if(result == ISC_STATUS_DONE) {
            ret = 0;
            break;
        } else if(result == ISC_STATUS_BUSY) {
            ret = -1;
            msleep(1);
        } else {
            dev_err(&info->client->dev, "%s [ERROR] wrong value [0x%02X]\n", __func__, result);
            ret = -1;
            msleep(1);
        }
    } while (--cnt);

    if (!cnt) {
        dev_err(&info->client->dev, "%s [ERROR] count overflow - cnt [%d] status [0x%02X]\n", __func__, cnt, result);
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

    return ret;

ERROR:
    return ret;
}

/**
* Command : Erase Page
*/
static int mms_isc_erase_page(struct mms_ts_info *info, int offset) {
    u8 write_buf[6] = ISC_CMD_ERASE_PAGE;
    /*
    struct i2c_msg msg[1] = {
    	{
    		.addr = info->client->addr,
    		.flags = 0,
    		.buf = write_buf,
    		.len = 6,
    	},
    };
    */

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    write_buf[4] = (u8)(((offset) >> 8) & 0xFF );
    write_buf[5] = (u8)(((offset) >> 0) & 0xFF );
    /*
    if(i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)){
    	dev_err(&info->client->dev, "%s [ERROR] i2c_transfer\n", __func__);
    	goto ERROR;
    }
    */	if(mms_i2c_write_normal(info, write_buf, 6)) {
        dev_err(&info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
        goto ERROR;
    }

    if(mms_isc_read_status(info) != 0) {
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE] - Offset [0x%04X]\n", __func__, offset);

    return 0;

ERROR:
    return -1;
}

/**
* Command : Read Page
*/
static int mms_isc_read_page(struct mms_ts_info *info, int offset, u8 *data) {
    u8 write_buf[6] = ISC_CMD_READ_PAGE;
    /*
    struct i2c_msg msg[2] = {
    	{
    		.addr = info->client->addr,
    		.flags = 0,
    		.buf = write_buf,
    		.len = 6,
    	}, {
    		.addr = info->client->addr,
    		.flags = I2C_M_RD,
    		.buf = data,
    		.len = ISC_PAGE_SIZE,
    	},
    };
    */

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    write_buf[4] = (u8)(((offset) >> 8) & 0xFF );
    write_buf[5] = (u8)(((offset) >> 0) & 0xFF );
    /*
    if(i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)){
    	dev_err(&info->client->dev, "%s [ERROR] i2c_transfer\n", __func__);
    	goto ERROR;
    }
    */
    if(mms_i2c_read_dma(info, write_buf, 6, data, ISC_PAGE_SIZE)) {
        dev_err(&info->client->dev, "%s [ERROR] mms_i2c_read\n", __func__);
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE] - Offset [0x%04X]\n", __func__, offset);

    return 0;

ERROR:
    return -1;
}

#if 0
/**
* Command : Write Page
*/
static int mms_isc_write_page(struct mms_ts_info *info, int offset, const u8 *data, int length) {
    u8 write_buf[134] = ISC_CMD_WRITE_PAGE;

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    if( length > 128 ) {
        dev_err(&info->client->dev, "%s [ERROR] page length overflow\n", __func__);
        goto ERROR;
    }

    write_buf[4] = (u8)(((offset) >> 8) & 0xFF );
    write_buf[5] = (u8)(((offset) >> 0) & 0xFF );

    memcpy( &write_buf[6], data, length);

    if(i2c_master_send(info->client, write_buf, length + 6 ) != length + 6) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_master_send\n", __func__);
        goto ERROR;
    }

    if(mms_isc_read_status(info) != 0) {
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE] - Offset[0x%04X] Length[%d]\n", __func__, offset, length);

    return 0;

ERROR:
    return -1;
}
#endif

/**
* Command : Program Page
*/
static int mms_isc_program_page(struct mms_ts_info *info, int offset, const u8 *data, int length) {
    u8 write_buf[134] = ISC_CMD_PROGRAM_PAGE;

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    if( length > 128 ) {
        dev_err(&info->client->dev, "%s [ERROR] page length overflow\n", __func__);
        goto ERROR;
    }

    write_buf[4] = (u8)(((offset) >> 8) & 0xFF );
    write_buf[5] = (u8)(((offset) >> 0) & 0xFF );

    memcpy( &write_buf[6], data, length);

    /*
    if(i2c_master_send(info->client, write_buf, length+6 )!=length+6){
    	dev_err(&info->client->dev, "%s [ERROR] i2c_master_send\n", __func__);
    	goto ERROR;
    }
    */
    if(mms_i2c_write_dma(info, write_buf, length + 6)) {
        dev_err(&info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
        goto ERROR;
    }

    if(mms_isc_read_status(info) != 0) {
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE] - Offset[0x%04X] Length[%d]\n", __func__, offset, length);

    return 0;

ERROR:
    return -1;
}

/**
* Command : Exit ISC
*/
static int mms_isc_exit(struct mms_ts_info *info) {
    u8 write_buf[6] = ISC_CMD_EXIT;

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    /*
    if(i2c_master_send(info->client, write_buf, 6 )!=6){
    	dev_err(&info->client->dev, "%s [ERROR] i2c_master_send\n", __func__);
    	goto ERROR;
    }
    */
    if(mms_i2c_write_normal(info, write_buf, 6)) {
        dev_err(&info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

    return 0;

ERROR:
    return -1;
}

#if 0
/**
* Flash chip firmware (main function)
*/
int mms_flash_fw(struct mms_ts_info *info, const u8 *fw_data, size_t fw_size, bool force, bool section) {

    struct mms_bin_hdr *fw_hdr;
    struct mms_fw_img **img;
    struct i2c_client *client = info->client;
    int i;
    int retry = 3;
    int ret;
    int nRet;
    int nStartAddr;
    int nWriteLength;
    int nLast;
    int nOffset;
    int nTransferLength;
    int size;
    u8 *data;
    u8 *cpydata;

    int offset = sizeof(struct mms_bin_hdr);

    bool update_flag = false;
    bool update_flags[MMS_FW_MAX_SECT_NUM] = {false, };

    u16 ver_chip[MMS_FW_MAX_SECT_NUM];
    u16 ver_file[MMS_FW_MAX_SECT_NUM];

    int offsetStart = 0;
    u8 initData[ISC_PAGE_SIZE];
    memset(initData, 0xFF, sizeof(initData));

    dev_dbg(&client->dev, "%s [START]\n", __func__);


    //Read firmware file
    fw_hdr = (struct mms_bin_hdr *)fw_data;
    img = kzalloc(sizeof(*img) * fw_hdr->section_num, GFP_KERNEL);

    //Check firmware file
    if(memcmp(CHIP_FW_CODE, &fw_hdr->tag[4], 4)) {
        dev_err(&client->dev, "%s [ERROR] F/W file is not for %s\n", __func__, CHIP_NAME);

        nRet = fw_err_file_type;
        goto ERROR;
    }

    //Reboot chip
    mms_reboot(info);

    //Check chip firmware version
    while (retry--) {
        if (!mms_get_fw_version_u16(info, ver_chip)) {
            break;
        } else {
            mms_reboot(info);
        }
    }
    if (retry < 0) {
        dev_err(&client->dev, "%s [ERROR] cannot read chip firmware version\n", __func__);

        memset(ver_chip, 0xFFFF, sizeof(ver_chip));
        dev_info(&client->dev, "%s - Chip firmware version is set to [0xFFFF]\n", __func__);
    } else {
        dev_info(&client->dev, "%s - Chip firmware version [0x%04X 0x%04X 0x%04X 0x%04X]\n", __func__, ver_chip[0], ver_chip[1], ver_chip[2], ver_chip[3]);
    }

    //Set update flag
    dev_info(&client->dev, "%s - Firmware file info : Sections[%d] Offset[0x%08X] Length[0x%08X]\n", __func__, fw_hdr->section_num, fw_hdr->binary_offset, fw_hdr->binary_length);
    printk("%s - Firmware file info : Sections[%d] Offset[0x%08X] Length[0x%08X]\n", __func__, fw_hdr->section_num, fw_hdr->binary_offset, fw_hdr->binary_length);
    for (i = 0; i < fw_hdr->section_num; i++, offset += sizeof(struct mms_fw_img)) {
        img[i] = (struct mms_fw_img *)(fw_data + offset);
        ver_file[i] = img[i]->version;

        dev_info(&client->dev, "%s - Section info : Section[%d] Version[0x%04X] StartPage[%d] EndPage[%d] Offset[0x%08X] Length[0x%08X]\n", __func__, i, img[i]->version, img[i]->start_page, img[i]->end_page, img[i]->offset, img[i]->length);
        printk("%s - Section info : Section[%d] Version[0x%04X] StartPage[%d] EndPage[%d] Offset[0x%08X] Length[0x%08X]\n", __func__, i, img[i]->version, img[i]->start_page, img[i]->end_page, img[i]->offset, img[i]->length);
        //Compare section version
        if (ver_chip[i] != ver_file[i]) {
            //Set update flag
            update_flag = true;
            update_flags[i] = true;

            dev_info(&client->dev, "%s - Section [%d] is need to be updated. Version : Chip[0x%04X] File[0x%04X]\n", __func__, i, ver_chip[i], ver_file[i]);
            printk("%s - Section [%d] is need to be updated. Version : Chip[0x%04X] File[0x%04X]\n", __func__, i, ver_chip[i], ver_file[i]);
        }
    }

    //Set force update flag
    if(force == true) {
        update_flag = true;
        update_flags[0] = true;
        update_flags[1] = true;
        update_flags[2] = true;
        update_flags[3] = true;

        dev_info(&client->dev, "%s - Force update\n", __func__);
    }


    //Exit when up-to-date
    if(update_flag == false) {
        nRet = fw_err_uptodate;
        dev_dbg(&client->dev, "%s [DONE] Chip firmware is already up-to-date\n", __func__);
        goto EXIT;
    }

    //Set start addr offset
    if(section == true) {
        if(update_flags[0] == true) {
            //boot
            offsetStart = img[0]->start_page;
        } else if(update_flags[1] == true) {
            //core
            offsetStart = img[1]->start_page;
        } else if(update_flags[2] == true) {
            //custom
            offsetStart = img[2]->start_page;
        } else if(update_flags[3] == true) {
            //param
            offsetStart = img[3]->start_page;
        }
    } else {
        offsetStart = 0;
    }

    offsetStart = offsetStart * 1024;

    //Load firmware data
    data = kzalloc(sizeof(u8) * fw_hdr->binary_length, GFP_KERNEL);
    size = fw_hdr->binary_length;
    cpydata = kzalloc(ISC_PAGE_SIZE, GFP_KERNEL);

    //Check firmware size
    if( size % ISC_PAGE_SIZE != 0 ) {
        size += ( ISC_PAGE_SIZE - (size % ISC_PAGE_SIZE) );
    }

    nStartAddr = 0;
    nWriteLength = size;
    nLast = nStartAddr + nWriteLength;

    if( (nLast) % 8 != 0 ) {
        nRet = fw_err_file_type;
        dev_err(&client->dev, "%s [ERROR] Firmware size mismatch\n", __func__);
        goto ERROR;
    } else {
        memcpy(data, fw_data + fw_hdr->binary_offset, fw_hdr->binary_length);
    }

    //Set address
    nOffset = nStartAddr + nWriteLength - ISC_PAGE_SIZE;
    nTransferLength = ISC_PAGE_SIZE;

    //Erase first page
    dev_info(&client->dev, "%s - Erase first page : Offset[0x%04X]\n", __func__, offsetStart);
    printk("%s - Erase first page : Offset[0x%04X]\n", __func__, offsetStart);
    nRet = mms_isc_erase_page(info, offsetStart);
    if( nRet != 0 ) {
        dev_err(&client->dev, "%s [ERROR] clear first page failed\n", __func__);
        goto ERROR;
    }

    //Flash firmware
    dev_info(&client->dev, "%s - Start Download : Offset Start[0x%04X] End[0x%04X]\n", __func__, nOffset, offsetStart);
    printk("%s - Start Download : Offset Start[0x%04X] End[0x%04X]\n", __func__, nOffset, offsetStart);
    while( nOffset >= offsetStart ) {
        dev_info(&client->dev, "%s - Downloading : Offset[0x%04X]\n", __func__, nOffset);
        printk("%s - Downloading : Offset[0x%04X]\n", __func__, nOffset);

        //Program (erase and write) a page
        nRet = mms_isc_program_page(info, nOffset, &data[nOffset], nTransferLength);
        if( nRet != 0 ) {
            dev_err(&client->dev, "%s [ERROR] isc_program_page\n", __func__);
            goto ERROR;
        }

        //Verify (read and compare)
        if (mms_isc_read_page(info, nOffset, cpydata)) {
            dev_err(&client->dev, "%s [ERROR] mms_isc_read_page\n", __func__);
            goto ERROR;
        }

        if (memcmp(&data[nOffset], cpydata, ISC_PAGE_SIZE)) {
#if MMS_FW_UPDATE_DEBUG
            print_hex_dump(KERN_ERR, "Firmware Page Write : ", DUMP_PREFIX_OFFSET, 16, 1, data, ISC_PAGE_SIZE, false);
            print_hex_dump(KERN_ERR, "Firmware Page Read : ", DUMP_PREFIX_OFFSET, 16, 1, cpydata, ISC_PAGE_SIZE, false);
#endif
            dev_err(&client->dev, "%s [ERROR] verify page failed\n", __func__);

            ret = -1;
            goto ERROR;
        }

        nOffset -= nTransferLength;
    }

    //Exit ISC
    nRet = mms_isc_exit(info);
    if( nRet != 0 ) {
        dev_err(&client->dev, "%s [ERROR] mms_isc_exit\n", __func__);
        goto ERROR;
    }

    //Reboot chip
    mms_reboot(info);

    //Check chip firmware version
    if (mms_get_fw_version_u16(info, ver_chip)) {
        dev_err(&client->dev, "%s [ERROR] cannot read chip firmware version after flash\n", __func__);

        nRet = -1;
        goto ERROR;
    } else {
        for (i = 0; i < fw_hdr->section_num; i++) {
            if (ver_chip[i] != ver_file[i]) {
                dev_err(&client->dev, "%s [ERROR] version mismatch after flash. Section[%d] : Chip[0x%04X] != File[0x%04X]\n", __func__, i, ver_chip[i], ver_file[i]);

                nRet = -1;
                goto ERROR;
            }
        }
    }

    nRet = 0;
    dev_dbg(&client->dev, "%s [DONE]\n", __func__);
    dev_info(&client->dev, "Firmware update completed\n");
    goto EXIT;

ERROR:
    dev_err(&client->dev, "%s [ERROR]\n", __func__);
    dev_err(&client->dev, "Firmware update failed\n");
    goto EXIT;

EXIT:
    kfree(img);
    return nRet;

}
#endif

/**
* Firmware binary tail info
*/
struct mip_bin_tail {
    u8 tail_mark[4];
    char chip_name[4];
    u32 bin_start_addr;
    u32 bin_length;

    u16 ver_boot;
    u16 ver_core;
    u16 ver_app;
    u16 ver_param;
    u8 boot_start;
    u8 boot_end;
    u8 core_start;
    u8 core_end;
    u8 app_start;
    u8 app_end;
    u8 param_start;
    u8 param_end;

    u8 checksum_type;
    u8 hw_category;
    u16 param_id;
    u32 param_length;
    u32 build_date;
    u32 build_time;

    u32 reserved1;
    u32 reserved2;
    u16 reserved3;
    u16 tail_size;
    u32 crc;
} __attribute__ ((packed));

#define MIP_BIN_TAIL_MARK		{0x4D, 0x42, 0x54, 0x01}	// M B T 0x01
#define MIP_BIN_TAIL_SIZE		64

/**
* Flash chip firmware (main function) : BIN
*/
int mms_flash_fw(struct mms_ts_info *info, const u8 *fw_data, size_t fw_size, bool force, bool section) {
    struct i2c_client *client = info->client;
    struct mip_bin_tail *bin_info;
    int ret = 0;
    int retry = 3;
    u8 rbuf[ISC_PAGE_SIZE];
    int offset = 0;
    int offset_start = 0;
    int bin_size = 0;
    u8 *bin_data;
    u16 tail_size = 0;
    u8 tail_mark[4] = MIP_BIN_TAIL_MARK;
    u16 ver_chip[4];

    dev_err(&client->dev, "%s [START]\n", __func__);

    //Check tail size
    tail_size = (fw_data[fw_size - 5] << 8) | fw_data[fw_size - 6];
    if(tail_size != MIP_BIN_TAIL_SIZE) {
        dev_err(&client->dev, "%s [ERROR] wrong tail size [%d]\n", __func__, tail_size);
        ret = fw_err_file_type;
        goto ERROR;
    }

    //Check bin format
    if(memcmp(&fw_data[fw_size - tail_size], tail_mark, 4)) {
        dev_err(&client->dev, "%s [ERROR] wrong tail mark\n", __func__);
        ret = fw_err_file_type;
        goto ERROR;
    }

    //Read bin info
    bin_info = (struct mip_bin_tail *)&fw_data[fw_size - tail_size];

    dev_err(&client->dev, "%s - bin_info : bin_len[%d] hw_cat[0x%2X] date[%4X] time[%4X] tail_size[%d]\n", __func__, bin_info->bin_length, bin_info->hw_category, bin_info->build_date, bin_info->build_time, bin_info->tail_size);

#if MMS_FW_UPDATE_DEBUG
    print_hex_dump(KERN_ERR, MMS_DEVICE_NAME " Bin Info : ", DUMP_PREFIX_OFFSET, 16, 1, bin_info, tail_size, false);
#endif

    //Check chip code
    if(memcmp(bin_info->chip_name, CHIP_FW_CODE, 4)) {
        dev_err(&client->dev, "%s [ERROR] F/W file is not for %s\n", __func__, CHIP_NAME);
        ret = fw_err_file_type;
        goto ERROR;
    }

    //Check F/W version
    dev_info(&client->dev, "%s - F/W file version [0x%04X 0x%04X 0x%04X 0x%04X]\n", __func__, bin_info->ver_boot, bin_info->ver_core, bin_info->ver_app, bin_info->ver_param);

    if(force == true) {
        //Force update
        dev_err(&client->dev, "%s - Skip chip firmware version check\n", __func__);
    } else {
        //Read firmware version from chip
        while(retry--) {
            if(!mms_get_fw_version_u16(info, ver_chip)) {
                break;
            } else {
                mms_reboot(info);
            }
        }
        if(retry < 0) {
            dev_err(&client->dev, "%s [ERROR] Unknown chip firmware version\n", __func__);
        } else {
            dev_err(&client->dev, "%s - Chip firmware version [0x%04X 0x%04X 0x%04X 0x%04X]\n", __func__, ver_chip[0], ver_chip[1], ver_chip[2], ver_chip[3]);

            //Compare version
            if((ver_chip[0] == bin_info->ver_boot) && (ver_chip[1] == bin_info->ver_core) && (ver_chip[2] == bin_info->ver_app) && (ver_chip[3] == bin_info->ver_param)) {
                dev_err(&client->dev, "%s - Chip firmware is already up-to-date\n", __func__);
                ret = fw_err_uptodate;
                goto ERROR;
            }
        }
    }

    //Read bin data
    bin_size = bin_info->bin_length;
    bin_data = kzalloc(sizeof(u8) * (bin_size), GFP_KERNEL);
    memcpy(bin_data, fw_data, bin_size);

    //Erase first page
    offset = 0;
    dev_err(&client->dev, "%s - Erase first page : Offset[0x%04X]\n", __func__, offset);
    ret = mms_isc_erase_page(info, offset);
    if(ret != 0) {
        dev_err(&client->dev, "%s [ERROR] mip_isc_erase_page\n", __func__);
        ret = fw_err_download;
        goto ERROR;
    }

    //Program & Verify
    dev_err(&client->dev, "%s - Program & Verify\n", __func__);
    offset_start = 0;
    offset = bin_size - ISC_PAGE_SIZE;
    while(offset >= offset_start) {
        //Program page
        if(mms_isc_program_page(info, offset, &bin_data[offset], ISC_PAGE_SIZE)) {
            dev_err(&client->dev, "%s [ERROR] mip_isc_program_page : offset[0x%04X]\n", __func__, offset);
            ret = fw_err_download;
            goto ERROR;
        }
        dev_err(&client->dev, "%s - mip_isc_program_page : offset[0x%04X]\n", __func__, offset);

        //Verify page
        if(mms_isc_read_page(info, offset, rbuf)) {
            dev_err(&client->dev, "%s [ERROR] mip_isc_read_page : offset[0x%04X]\n", __func__, offset);
            ret = fw_err_download;
            goto ERROR;
        }
        dev_err(&client->dev, "%s - mip_isc_read_page : offset[0x%04X]\n", __func__, offset);

#if MMS_FW_UPDATE_DEBUG
        print_hex_dump(KERN_ERR, MMS_DEVICE_NAME " F/W File : ", DUMP_PREFIX_OFFSET, 16, 1, &bin_data[offset], ISC_PAGE_SIZE, false);
        print_hex_dump(KERN_ERR, MMS_DEVICE_NAME " F/W Chip : ", DUMP_PREFIX_OFFSET, 16, 1, rbuf, ISC_PAGE_SIZE, false);
#endif

        if(memcmp(rbuf, &bin_data[offset], ISC_PAGE_SIZE)) {
            dev_err(&client->dev, "%s [ERROR] Verify failed : offset[0x%04X]\n", __func__, offset);
            ret = fw_err_download;
            goto ERROR;
        }

        offset -= ISC_PAGE_SIZE;
    }

    //Exit ISC mode
    dev_err(&client->dev, "%s - Exit\n", __func__);
    mms_isc_exit(info);

    //Reset chip
    mms_reboot(info);

    //Check chip firmware version
    if(mms_get_fw_version_u16(info, ver_chip)) {
        dev_err(&client->dev, "%s [ERROR] Unknown chip firmware version\n", __func__);
        ret = fw_err_download;
        goto ERROR;
    } else {
        if((ver_chip[0] == bin_info->ver_boot) && (ver_chip[1] == bin_info->ver_core) && (ver_chip[2] == bin_info->ver_app) && (ver_chip[3] == bin_info->ver_param)) {
            dev_err(&client->dev, "%s - Version check OK\n", __func__);
        } else {
            dev_err(&client->dev, "%s [ERROR] Version mismatch after flash. Chip[0x%04X 0x%04X 0x%04X 0x%04X] File[0x%04X 0x%04X 0x%04X 0x%04X]\n", __func__, ver_chip[0], ver_chip[1], ver_chip[2], ver_chip[3], bin_info->ver_boot, bin_info->ver_core, bin_info->ver_app, bin_info->ver_param);
            ret = fw_err_download;
            goto ERROR;
        }
    }

    goto EXIT;

ERROR:
    //Reset chip
    mms_reboot(info);

    dev_err(&client->dev, "%s [ERROR]\n", __func__);

EXIT:
    dev_dbg(&client->dev, "%s [DONE]\n", __func__);

    return ret;
}

//hyeog added
/*
static int mms_flash_fw(const struct firmware *fw, struct i2c_client *client)
{
    int ret;
    int i;
    struct mms_bin_hdr *fw_hdr;
    struct mms_fw_img **img;

    u8 ver[MAX_SECTION_NUM];
    u8 target[MAX_SECTION_NUM];
    int offset = sizeof(struct mms_bin_hdr);
    struct mms_ext_hdr *ext;
    int retires = 3;
    bool update_flag = false;

    TPD_DMESG("mms_flash_fw \n");

    fw_hdr = (struct mms_bin_hdr *)fw->data;
    ext = (struct mms_ext_hdr *)(fw->data + fw_hdr->extention_offset);
    img = kzalloc(sizeof(*img) * fw_hdr->section_num, GFP_KERNEL);

    if (retires < 0) {
        TPD_DMESG("failed to obtain ver. info\n");
        memset(ver, 0xff, sizeof(ver));
    } else {
        print_hex_dump(KERN_INFO, "mms_ts fw ver : ", DUMP_PREFIX_NONE, 16, 1,
                ver, MAX_SECTION_NUM, false);
        ret = mms_isc_enter(client);
        if (ret <0)
        {
            TPD_DMESG("MMS-ISC, mms_isc_enter, %d, Failed\n",__LINE__);
            mms_reboot();
            goto out;
        }
        else
            TPD_DMESG("MMS-ISC, mms_isc_enter, %d, Sucess\n",__LINE__);
    }

    for (i = 0; i < fw_hdr->section_num; i++, offset += sizeof(struct mms_fw_img))
    {
        TPD_DMESG("MMS-ISC, section_num = %d, offset = %d \n",fw_hdr->section_num,sizeof(struct mms_fw_img));
        img[i] = (struct mms_fw_img *)(fw->data + offset);
        target[i] = img[i]->version;

        if (ver[img[i]->type] != target[i]) {
            TPD_DMESG("MMS-ISC, section [%d] is need to be updated. ver : 0x%x, bin : 0x%x\n",
                i, ver[i], target[i]);

            update_flag = true;

            if ((ret = mms_flash_section(client, img[i], fw->data + fw_hdr->binary_offset, ext))) {
                mms_reboot();
                goto out;
            }

        } else {
            TPD_DMESG("MMS-ISC, section [%d] is up to date\n", i);
        }
    }

    if (update_flag)
        ret = mms_isc_exit(client);

    if (ret < 0)
    {
        TPD_DMESG("MMS-ISC, mms_isc_exit, Failed\n");
        goto out;
    }
    else
    {
        TPD_DMESG("MMS-ISC, mms_isc_exit, Sucess\n");
    }


    msleep(5);

    mms_reboot();
    #if 0
    //update finished, check the firmware to confirm
    if (melfas_check_firmware(client)) {
        TPD_DMESG("failed to obtain version after flash\n");
        ret = -1;
        goto out;
    } else {
        for (i = 0; i < fw_hdr->section_num; i++) {
            if (ver[img[i]->type] != target[i]) {
                TPD_DMESG("version mismatch after flash. [%d] 0x%x != 0x%x\n",
                    i, ver[img[i]->type], target[i]);

                ret = -1;
                goto out;
            }
        }
    }
    #endif
    ret = 0;

out:
    kfree(img);
    if (isc_entered)//(update_flag)
    {
        ret = mms_isc_exit(client);
        if (ret < 0)
        {
            TPD_DMESG("MMS-ISC, mms_isc_exit, Failed\n");
        }
        else
        {
            TPD_DMESG("MMS-ISC, mms_isc_exit, Sucess\n");
        }
    }

    msleep(5);

    mms_reboot();

    //mms_ts_config(info);
    i2c_smbus_read_byte_data(client, MMS_TX_NUM);
	i2c_smbus_read_byte_data(client, MMS_RX_NUM);
    //release_firmware(fw);

    return ret;

}
*/




//int mms_fw_update_from_kernel(struct mms_ts_info *info)
void mms_fw_update_controller(struct mms_ts_info *info, const struct firmware *fw, struct i2c_client *client) {
    int retry = 3;
    int ret;

    printk("mms_fw_update_controller \n");

    do {
        ret = mms_flash_fw(info, fw->data, fw->size, false, true);
        //ret = mms_flash_fw(info, fw->data, fw->size, true, true);
        if(ret >= fw_err_none) {
            break;
        }
    } while (--retry);

    if (!retry) {
        printk("failed to flash firmware after retires\n");
    }
}
