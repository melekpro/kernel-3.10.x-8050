/*
 * MELFAS MIP4 Touchscreen for MediaTek
 *
 * Copyright (C) 2015 MELFAS Inc.
 *
 *
 * mip4.c : Main functions
 *
 *
 * Version : 2015.08.23
 *
 */

#include "mip4.h"

#if MIP_USE_WAKEUP_GESTURE
//struct wake_lock mip_wake_lock;
#endif

/**
* MTK
*/
extern struct tpd_device *tpd;
struct mip_ts_info *mip_info;
extern unsigned int gtp_fw_version;
extern unsigned int gtp_cfg_version;
extern int gesture_wakeup_flag;
static int mip_gesture_wakeup_flag;

#if defined (MIP_FLIP_COVER_SWITCH)
#define FLIP_COVER_MODE_ON 1
#define FLIP_COVER_MODE_OFF 0
#define FLIP_COVER_MODE_ADDR 0x061B
extern int smart_cover_flag;
extern void (*hall_state_charge_notify)(int state);
void mip_flip_cover_switch(int state);
extern void (*smart_cover_flag_charge_notify)(int flag);
void mip_flip_cover_flag_charge(int flag);
int mip_cover_state = 1;
#endif


#define MIP_I2C_SPEED 			300	//kHz
#define MIP_I2C_DMA_LENGTH 	9 	//Bytes
static u8 *i2c_dma_buf_va = NULL;
static u32 i2c_dma_buf_pa = 0;

struct task_struct *mtk_eint_thread = NULL;
int mtk_eint_flag = 0;
int mtk_eint_count = 0;
DECLARE_WAIT_QUEUE_HEAD(waiter);

int mip_tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;

#ifdef TPD_HAVE_BUTTON
int mip_tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif


/**
* Reboot chip
*
* Caution : IRQ must be disabled before mip_reboot and enabled after mip_reboot.
*/
void mip_reboot(struct mip_ts_info *info) {
    struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    i2c_lock_adapter(adapter);

    mip_power_off(info);
    mip_power_on(info);

    i2c_unlock_adapter(adapter);

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

/**
* I2C Read (Normal)
*/
int mip_i2c_read_normal(struct mip_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len) {
    int res;

    struct i2c_msg msg[] = {
        {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG),
            .flags = 0,
            .buf = write_buf,
            .len = write_len,
            .timing = MIP_I2C_SPEED
        }, {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG),
            .flags = I2C_M_RD,
            .buf = read_buf,
            .len = read_len,
            .timing = MIP_I2C_SPEED
        },
    };

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));

    if(res == ARRAY_SIZE(msg)) {
        goto DONE;
    } else if(res < 0) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n", __func__, res);
    } else if(res != ARRAY_SIZE(msg)) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, ARRAY_SIZE(msg), res);
    } else {
        dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n", __func__, res);
    }

    goto ERROR;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;

DONE:
    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* I2C Read (DMA)
*/
int mip_i2c_read_dma(struct mip_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len) {
    int res;
    int i = 0;

    struct i2c_msg msg[] = {
        {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG),
            .flags = 0,
            .buf = write_buf,
            .len = write_len,
            .timing = MIP_I2C_SPEED
        }, {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = I2C_M_RD,
            //.buf = read_buf,
            .buf = i2c_dma_buf_pa,
            .len = read_len,
            .timing = MIP_I2C_SPEED
        },
    };

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));

    if(res == ARRAY_SIZE(msg)) {
        for(i = 0; i < read_len; i++) {
            read_buf[i] = i2c_dma_buf_va[i];
        }
        goto DONE;
    } else if(res < 0) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n", __func__, res);
    } else if(res != ARRAY_SIZE(msg)) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, ARRAY_SIZE(msg), res);
    } else {
        dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n", __func__, res);
    }

    goto ERROR;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;

DONE:
    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* I2C Read
*/
int mip_i2c_read(struct mip_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len) {
    int retry = I2C_RETRY_COUNT;

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    //dev_dbg(&info->client->dev, "%s - write_len[%d] read_len[%d]\n", __func__, write_len, read_len);

    while(retry--) {
        if(read_len < MIP_I2C_DMA_LENGTH) {
            if(mip_i2c_read_normal(info, write_buf, write_len, read_buf, read_len) == 0) {
                goto EXIT;
            }
        } else {
            if(mip_i2c_read_dma(info, write_buf, write_len, read_buf, read_len) == 0) {
                goto EXIT;
            }
        }
    }

    goto ERROR;

ERROR:
#if RESET_ON_I2C_ERROR
    mip_reboot(info);
#endif
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;

EXIT:
    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* I2C Write (Normal)
*/
int mip_i2c_write_normal(struct mip_ts_info *info, char *write_buf, unsigned int write_len) {
    int res;

    struct i2c_msg msg[] = {
        {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG),
            .flags = 0,
            .buf = write_buf,
            .len = write_len,
            .timing = MIP_I2C_SPEED
        },
    };

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));

    if(res == ARRAY_SIZE(msg)) {
        goto DONE;
    } else if(res < 0) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n", __func__, res);
    } else if(res != ARRAY_SIZE(msg)) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, ARRAY_SIZE(msg), res);
    } else {
        dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n", __func__, res);
    }

    goto ERROR;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;

DONE:
    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* I2C Write (DMA)
*/
int mip_i2c_write_dma(struct mip_ts_info *info, char *write_buf, unsigned int write_len) {
    int res;
    int i = 0;

    struct i2c_msg msg[] = {
        {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = 0,
            //.buf = write_buf,
            .buf = i2c_dma_buf_pa,
            .len = write_len,
            .timing = MIP_I2C_SPEED
        },
    };

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    for(i = 0; i < write_len; i++) {
        i2c_dma_buf_va[i] = write_buf[i];
    }

    res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));

    if(res == ARRAY_SIZE(msg)) {
        goto DONE;
    } else if(res < 0) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n", __func__, res);
    } else if(res != ARRAY_SIZE(msg)) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, ARRAY_SIZE(msg), res);
    } else {
        dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n", __func__, res);
    }

    goto ERROR;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;

DONE:
    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* I2C Write
*/
int mip_i2c_write(struct mip_ts_info *info, char *write_buf, unsigned int write_len) {
    int retry = I2C_RETRY_COUNT;

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    //dev_dbg(&info->client->dev, "%s - write_len[%d]\n", __func__, write_len);

    while(retry--) {
        if(write_len < MIP_I2C_DMA_LENGTH) {
            if(mip_i2c_write_normal(info, write_buf, write_len) == 0) {
                goto EXIT;
            }
        } else {
            if(mip_i2c_write_dma(info, write_buf, write_len) == 0) {
                goto EXIT;
            }
        }
    }

    goto ERROR;

ERROR:
#if RESET_ON_I2C_ERROR
    mip_reboot(info);
#endif
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;

EXIT:
    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* Enable device
*/
int mip_enable(struct mip_ts_info *info) {
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    MIP_INFO("%s info->enabled = %d\n", __func__, info->enabled);
    if (info->enabled) {
        dev_err(&info->client->dev, "%s [ERROR] device already enabled\n", __func__);
        goto EXIT;
    }

    mip_power_on(info);

#if 1
    if(info->disable_esd == true) {
        //Disable ESD alert
        mip_disable_esd_alert(info);
    }
#endif

    mutex_lock(&info->lock);

    //enable_irq(info->client->irq);
    mip_irq_enable(info);
    info->enabled = true;

    mutex_unlock(&info->lock);

EXIT:
    dev_info(&info->client->dev, MIP_DEVICE_NAME" - Enabled\n");

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* Disable device
*/
int mip_disable(struct mip_ts_info *info) {
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    MIP_INFO("%s info->enabled = %d\n", __func__, info->enabled);
    if (!info->enabled) {
        dev_err(&info->client->dev, "%s [ERROR] device already disabled\n", __func__);
        goto EXIT;
    }

    mutex_lock(&info->lock);
    //disable_irq(info->client->irq);
    mip_irq_disable(info);
    mip_power_off(info);
    mutex_unlock(&info->lock);
    info->enabled = false;

EXIT:
    dev_info(&info->client->dev, MIP_DEVICE_NAME" - Disabled\n");

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

#if defined (MIP_FLIP_COVER_SWITCH)

void mip_flip_cover_read_reg(void) {
    u8 wbuf[16];
    u8 rbuf[16];
    int ret = 0;

    if(mip_info == NULL) {
        return;
    }

    wbuf[0] = 0x06;
    wbuf[1] = 0x1B;

    ret = mip_i2c_read(mip_info, wbuf, 2, rbuf, 1);
    MIP_INFO("%s ret = %d rbuf[0] = %d\n", __func__, ret, rbuf[0]);
 }

void mip_flip_cover_write_reg(int isEnable) {
    u8 wbuf[16];
    int ret = 0;

    if(mip_info == NULL) {
        return;
    }

    wbuf[0] = 0x06;
    wbuf[1] = 0x1B;
    wbuf[2] = isEnable;

    ret = mip_i2c_write(mip_info, wbuf, 3);
    MIP_INFO("%s ret = %d isEnable = %d\n", __func__, ret, isEnable);
}

void mip_flip_cover_update(void) {
    MIP_INFO("%s smart_cover_flag = %d mip_cover_state = %d\n", __func__, smart_cover_flag, mip_cover_state);
    if(smart_cover_flag == 0) {
       return;
    }

    if(mip_cover_state) {
       mip_flip_cover_write_reg(FLIP_COVER_MODE_OFF);
    } else {
       mip_flip_cover_write_reg(FLIP_COVER_MODE_ON);
    }

    mip_flip_cover_read_reg();
}

void mip_flip_cover_flag_charge(int flag) {
    MIP_INFO("%s flag = %d\n", __func__, flag);

    if(smart_cover_flag == 0) {
       mip_flip_cover_write_reg(0);
    } else if(smart_cover_flag == 1 && mip_cover_state == 0){
       mip_flip_cover_write_reg(1);
    }
    mip_flip_cover_read_reg();
}

void mip_flip_cover_switch(int state) {
    MIP_INFO("%s state = %d\n", __func__, state);
    mip_cover_state = state;
    mip_flip_cover_update();
}

#endif

#if MIP_USE_INPUT_OPEN_CLOSE
/**
* Open input device
*/
static int mip_input_open(struct input_dev *dev) {
    struct mip_ts_info *info = input_get_drvdata(dev);

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    if(info->init == true) {
        info->init = false;
    } else {
        mip_enable(info);
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

    return 0;
}

/**
* Close input device
*/
static void mip_input_close(struct input_dev *dev) {
    struct mip_ts_info *info = input_get_drvdata(dev);

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    mip_disable(info);

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

    return;
}
#endif

/**
* Get ready status
*/
int mip_get_ready_status(struct mip_ts_info *info) {
    u8 wbuf[16];
    u8 rbuf[16];
    int ret = 0;

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_READY_STATUS;
    if(mip_i2c_read(info, wbuf, 2, rbuf, 1)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_read\n", __func__);
        goto ERROR;
    }
    ret = rbuf[0];

    //check status
    if((ret == MIP_CTRL_STATUS_NONE) || (ret == MIP_CTRL_STATUS_LOG) || (ret == MIP_CTRL_STATUS_READY)) {
        //dev_dbg(&info->client->dev, "%s - status [0x%02X]\n", __func__, ret);
    } else {
        dev_err(&info->client->dev, "%s [ERROR] Unknown status [0x%02X]\n", __func__, ret);
        goto ERROR;
    }

    if(ret == MIP_CTRL_STATUS_LOG) {
        //skip log event
        wbuf[0] = MIP_R0_LOG;
        wbuf[1] = MIP_R1_LOG_TRIGGER;
        wbuf[2] = 0;
        if(mip_i2c_write(info, wbuf, 3)) {
            dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
        }
    }

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return ret;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}

/**
* Read chip firmware version
*/
int mip_get_fw_version(struct mip_ts_info *info, u8 *ver_buf) {
    u8 rbuf[8];
    u8 wbuf[2];
    int i;

    wbuf[0] = MIP_R0_INFO;
    wbuf[1] = MIP_R1_INFO_VERSION_BOOT;
    if(mip_i2c_read(info, wbuf, 2, rbuf, 8)) {
        goto ERROR;
    };

    for(i = 0; i < MIP_FW_MAX_SECT_NUM; i++) {
        ver_buf[0 + i * 2] = rbuf[1 + i * 2];
        ver_buf[1 + i * 2] = rbuf[0 + i * 2];
    }

    return 0;

ERROR:
    memset(ver_buf, 0xFF, sizeof(ver_buf));

    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}

/**
* Read chip firmware version for u16
*/
int mip_get_fw_version_u16(struct mip_ts_info *info, u16 *ver_buf_u16) {
    u8 rbuf[8];
    int i;

    if(mip_get_fw_version(info, rbuf)) {
        goto ERROR;
    }

    for(i = 0; i < MIP_FW_MAX_SECT_NUM; i++) {
        ver_buf_u16[i] = (rbuf[0 + i * 2] << 8) | rbuf[1 + i * 2];
    }

    return 0;

ERROR:
    memset(ver_buf_u16, 0xFFFF, sizeof(ver_buf_u16));

    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}

/**
* Read bin(file) firmware version
*/
int mip_get_fw_version_from_bin(struct mip_ts_info *info, u8 *ver_buf) {
    const struct firmware *fw;

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    request_firmware(&fw, FW_PATH_INTERNAL, &info->client->dev);

    if(!fw) {
        dev_err(&info->client->dev, "%s [ERROR] request_firmware\n", __func__);
        goto ERROR;
    }

    if(mip_bin_fw_version(info, fw->data, fw->size, ver_buf)) {
        memset(ver_buf, 0xFF, sizeof(ver_buf));
        dev_err(&info->client->dev, "%s [ERROR] mip_bin_fw_version\n", __func__);
        goto ERROR;
    }

    release_firmware(fw);

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Set power state
*/
int mip_set_power_state(struct mip_ts_info *info, u8 mode) {
    u8 wbuf[3];

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    dev_dbg(&info->client->dev, "%s - mode[%02X]\n", __func__, mode);

    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_POWER_STATE;
    wbuf[2] = mode;
    if(mip_i2c_write(info, wbuf, 3)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Set wake-up gesture type
*/
int mip_set_wakeup_gesture_type(struct mip_ts_info *info, u32 type) {
    u8 wbuf[6];

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    dev_dbg(&info->client->dev, "%s - type[%08X]\n", __func__, type);

    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_GESTURE_TYPE;
    wbuf[2] = (type >> 24) & 0xFF;
    wbuf[3] = (type >> 16) & 0xFF;
    wbuf[4] = (type >> 8) & 0xFF;
    wbuf[5] = type & 0xFF;
    if(mip_i2c_write(info, wbuf, 6)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Disable ESD alert
*/
int mip_disable_esd_alert(struct mip_ts_info *info) {
    u8 wbuf[4];
    u8 rbuf[4];

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_DISABLE_ESD_ALERT;
    wbuf[2] = 1;
    if(mip_i2c_write(info, wbuf, 3)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_write\n", __func__);
        goto ERROR;
    }

    if(mip_i2c_read(info, wbuf, 2, rbuf, 1)) {
        dev_err(&info->client->dev, "%s [ERROR] mip_i2c_read\n", __func__);
        goto ERROR;
    }

    if(rbuf[0] != 1) {
        dev_dbg(&info->client->dev, "%s [ERROR] failed\n", __func__);
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Alert event handler - ESD
*/
static int mip_alert_handler_esd(struct mip_ts_info *info, u8 *rbuf) {
    u8 frame_cnt = rbuf[1];

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    dev_dbg(&info->client->dev, "%s - frame_cnt[%d]\n", __func__, frame_cnt);

    if(frame_cnt == 0) {
        //sensor crack, not ESD
        info->esd_cnt++;
        dev_dbg(&info->client->dev, "%s - esd_cnt[%d]\n", __func__, info->esd_cnt);

        if(info->disable_esd == true) {
            mip_disable_esd_alert(info);
            info->esd_cnt = 0;
        } else if(info->esd_cnt > ESD_COUNT_FOR_DISABLE) {
            //Disable ESD alert
            if(mip_disable_esd_alert(info)) {
            } else {
                info->disable_esd = true;
                info->esd_cnt = 0;
            }
        } else {
            //Reset chip
            mip_reboot(info);
        }
    } else {
        //ESD detected
        //Reset chip
        mip_reboot(info);
        info->esd_cnt = 0;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

    //ERROR:
    //dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    //return 1;
}

/**
* Alert event handler - Wake-up
*/
static int mip_alert_handler_wakeup(struct mip_ts_info *info, u8 *rbuf) {
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    if(mip_wakeup_event_handler(info, rbuf)) {
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Alert event handler - Input type
*/
static int mip_alert_handler_inputtype(struct mip_ts_info *info, u8 *rbuf) {
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    //...

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

    //ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Interrupt handler
*/
static irqreturn_t mip_interrupt(int irq, void *dev_id) {
    struct mip_ts_info *info = dev_id;
    struct i2c_client *client = info->client;
    u8 wbuf[8];
    u8 rbuf[256];
    unsigned int size = 0;
    //int event_size = info->event_size;
    u8 category = 0;
    u8 alert_type = 0;

    dev_dbg(&client->dev, "%s [START]\n", __func__);

    //Read packet info
    wbuf[0] = MIP_R0_EVENT;
    wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
    if(mip_i2c_read(info, wbuf, 2, rbuf, 1)) {
        dev_err(&client->dev, "%s [ERROR] Read packet info\n", __func__);
        goto ERROR;
    }

    size = (rbuf[0] & 0x7F);
    category = ((rbuf[0] >> 7) & 0x1);
    dev_dbg(&client->dev, "%s - packet info : size[%d] category[%d]\n", __func__, size, category);

    //Check size
    if(size <= 0) {
        dev_err(&client->dev, "%s [ERROR] Packet size [%d]\n", __func__, size);
        goto EXIT;
    }

    //Read packet data
    wbuf[0] = MIP_R0_EVENT;
    wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
    if(mip_i2c_read(info, wbuf, 2, rbuf, size)) {
        dev_err(&client->dev, "%s [ERROR] Read packet data\n", __func__);
        goto ERROR;
    }

    //Event handler
    if(category == 0) {
        //Touch event
        info->esd_cnt = 0;

        mip_input_event_handler(info, size, rbuf);
    } else {
        //Alert event
        alert_type = rbuf[0];

        dev_dbg(&client->dev, "%s - alert type [%d]\n", __func__, alert_type);

        if(alert_type == MIP_ALERT_ESD) {
            //ESD detection
            if(mip_alert_handler_esd(info, rbuf)) {
                goto ERROR;
            }
        } else if(alert_type == MIP_ALERT_WAKEUP) {
            //Wake-up gesture
            if(mip_alert_handler_wakeup(info, rbuf)) {
                goto ERROR;
            }
        } else if(alert_type == MIP_ALERT_INPUT_TYPE) {
            //Input type changed
            if(mip_alert_handler_inputtype(info, rbuf)) {
                goto ERROR;
            }
        } else {
            dev_err(&client->dev, "%s [ERROR] Unknown alert type [%d]\n", __func__, alert_type);
            goto ERROR;
        }
    }

EXIT:
    dev_dbg(&client->dev, "%s [DONE]\n", __func__);
    return IRQ_HANDLED;

ERROR:
    if(RESET_ON_EVENT_ERROR) {
        dev_info(&client->dev, "%s - Reset on error\n", __func__);

        mip_disable(info);
        mip_clear_input(info);
        mip_enable(info);
    }

    dev_err(&client->dev, "%s [ERROR]\n", __func__);
    return IRQ_HANDLED;
}


/**
* Interrupt handler for MTK
*/
int mip_interrupt_mtk(void *data) {
    struct mip_ts_info *info = data;
    struct i2c_client *client = info->client;
    u8 wbuf[8];
    u8 rbuf[256];
    unsigned int size = 0;
    //int event_size = info->event_size;
    u8 category = 0;
    u8 alert_type = 0;

    struct sched_param param = {
        .sched_priority = RTPM_PRIO_TPD
    };
    sched_setscheduler(current, SCHED_RR, &param);

    dev_dbg(&client->dev, "%s [START]\n", __func__);

    do {
        dev_dbg(&client->dev, "%s - wait\n", __func__);

        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, mtk_eint_flag != 0);

        mtk_eint_flag = 0;
        dev_dbg(&client->dev, "%s - eint_count[%d]\n", __func__, mtk_eint_count);

        if(info->enabled == false) {
            dev_dbg(&client->dev, "%s - skip : enabled [false]\n", __func__);
            msleep(1);
            goto NEXT;
        }

        if(info->init == true) {
            info->init = false;
            dev_dbg(&client->dev, "%s - skip : init [false]\n", __func__);
            goto NEXT;
        }

        if(info->irq_pause == true) {
            dev_dbg(&client->dev, "%s - skip : irq_pause\n", __func__);
            goto NEXT;
        }

        set_current_state(TASK_RUNNING);
        dev_dbg(&client->dev, "%s - run\n", __func__);

        //Read packet info
        wbuf[0] = MIP_R0_EVENT;
        wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
        if(mip_i2c_read(info, wbuf, 2, rbuf, 1)) {
            dev_err(&client->dev, "%s [ERROR] Read packet info\n", __func__);
            goto ERROR;
        }

        size = (rbuf[0] & 0x7F);
        category = ((rbuf[0] >> 7) & 0x1);
        dev_dbg(&client->dev, "%s - packet info : size[%d] category[%d]\n", __func__, size, category);

        //Check size
        if(size <= 0) {
            dev_err(&client->dev, "%s [ERROR] Packet size [%d]\n", __func__, size);
            goto NEXT;
        }

        //Read packet data
        wbuf[0] = MIP_R0_EVENT;
        wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
        if(mip_i2c_read(info, wbuf, 2, rbuf, size)) {
            dev_err(&client->dev, "%s [ERROR] Read packet data\n", __func__);
            goto ERROR;
        }

        //Event handler
        if(category == 0) {
            //Touch event
            info->esd_cnt = 0;

            mip_input_event_handler(info, size, rbuf);
        } else {
            //Alert event
            alert_type = rbuf[0];

            dev_dbg(&client->dev, "%s - alert type [%d]\n", __func__, alert_type);

            if(alert_type == MIP_ALERT_ESD) {
                //ESD detection
                if(mip_alert_handler_esd(info, rbuf)) {
                    goto ERROR;
                }
            } else if(alert_type == MIP_ALERT_WAKEUP) {
                //Wake-up gesture
                if(mip_alert_handler_wakeup(info, rbuf)) {
                    goto ERROR;
                }
            } else if(alert_type == MIP_ALERT_INPUT_TYPE) {
                //Input type changed
                if(mip_alert_handler_inputtype(info, rbuf)) {
                    goto ERROR;
                }
            } else {
                dev_err(&client->dev, "%s [ERROR] Unknown alert type [%d]\n", __func__, alert_type);
                goto ERROR;
            }
        }

        goto NEXT;

ERROR:
        if(RESET_ON_EVENT_ERROR) {
            dev_info(&client->dev, "%s - Reset on error\n", __func__);

            mip_disable(info);
            mip_clear_input(info);
            mip_enable(info);
        }
        dev_err(&client->dev, "%s [ERROR]\n", __func__);

NEXT:
        mip_irq_enable(info);
        mtk_eint_count--;
    } while(!kthread_should_stop());

    info->irq_enabled = false;

    dev_dbg(&client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* Start interrupt handler for MTK
*/
int mip_interrupt_mtk_start(struct mip_ts_info *info) {
    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    info->irq_pause = false;

    //mtk_eint_thread = kthread_run(mip_interrupt_mtk, info, TPD_DEVICE);
    mtk_eint_thread = kthread_run(mip_interrupt_mtk, info, MIP_DEVICE_NAME);
    if(IS_ERR(mtk_eint_thread)) {
        dev_err(&info->client->dev, "%s [ERROR] kthread_run\n", __func__);
        goto ERROR;
    }

    info->irq_enabled = true;
    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    info->irq_enabled = false;
    dev_dbg(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Update firmware from kernel built-in binary
*/
int mip_fw_update_from_kernel(struct mip_ts_info *info) {
    const char *fw_name = FW_PATH_INTERNAL;
    const struct firmware *fw;
    int retires = 3;
    int ret = fw_err_none;

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    //Disable IRQ
    mutex_lock(&info->lock);
    //disable_irq(info->client->irq);
    mip_irq_disable(info);

    //Get firmware
    request_firmware(&fw, fw_name, &info->client->dev);

    if (!fw) {
        dev_err(&info->client->dev, "%s [ERROR] request_firmware\n", __func__);
        goto ERROR;
    }

    //Update firmware
    do {
        ret = mip_flash_fw(info, fw->data, fw->size, false, true);
        if(ret >= fw_err_none) {
            break;
        }
    } while (--retires);

    if (!retires) {
        dev_err(&info->client->dev, "%s [ERROR] mip_flash_fw failed\n", __func__);
        ret = fw_err_download;
    }

    release_firmware(fw);

    //Enable IRQ
    //enable_irq(info->client->irq);
    mip_irq_enable(info);
    mutex_unlock(&info->lock);

    if(ret < fw_err_none) {
        goto ERROR;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}

/**********************************auto update************************************/
const u8 MELFAS_wind_binary[] = {
#include "melfas_mip4.fw.h"
};

extern void mip_fw_update_controller(struct mip_ts_info *info, const struct firmware *fw,
                                     struct i2c_client *client);

extern void mip_kernel_fw_force_update_controller(struct mip_ts_info *info, const struct firmware *fw,
                                     struct i2c_client *client);

struct firmware melfas_fw = {
    .size = sizeof(MELFAS_wind_binary),
    .data = &MELFAS_wind_binary[0],
};

static int mip_fw_update(struct mip_ts_info *info, struct i2c_client *client) {
    mip_fw_update_controller(info, &melfas_fw, client);
    return 0;
}

static int mip_kenel_fw_update(struct mip_ts_info *info, struct i2c_client *client) {
    mip_kernel_fw_force_update_controller(info, &melfas_fw, client);
    return 0;
}
/********************************************************************************/


/**
* Update firmware from external storage
*/
int mip_fw_update_from_storage(struct mip_ts_info *info, char *path, bool force) {
    struct file *fp;
    mm_segment_t old_fs;
    size_t fw_size, nread;
    int ret = fw_err_none;

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    //Disable IRQ
    mutex_lock(&info->lock);
    //disable_irq(info->client->irq);
    mip_irq_disable(info);

    //Get firmware
    old_fs = get_fs();
    set_fs(KERNEL_DS);

    fp = filp_open(path, O_RDONLY, S_IRUSR);
    if (IS_ERR(fp)) {
        dev_err(&info->client->dev, "%s [ERROR] file_open - path[%s]\n", __func__, path);
        ret = fw_err_file_open;
        goto ERROR;
    }

    fw_size = fp->f_path.dentry->d_inode->i_size;
    if (0 < fw_size) {
        //Read firmware
        unsigned char *fw_data;
        fw_data = kzalloc(fw_size, GFP_KERNEL);
        nread = vfs_read(fp, (char __user *)fw_data, fw_size, &fp->f_pos);
        dev_dbg(&info->client->dev, "%s - path[%s] size[%u]\n", __func__, path, fw_size);

        if (nread != fw_size) {
            dev_err(&info->client->dev, "%s [ERROR] vfs_read - size[%d] read[%d]\n", __func__, fw_size, nread);
            ret = fw_err_file_read;
        } else {
            //Update firmware
            ret = mip_flash_fw(info, fw_data, fw_size, force, true);
        }

        kfree(fw_data);
    } else {
        dev_err(&info->client->dev, "%s [ERROR] fw_size [%d]\n", __func__, fw_size);
        ret = fw_err_file_read;
    }

    filp_close(fp, current->files);

ERROR:
    set_fs(old_fs);

    //Enable IRQ
    //enable_irq(info->client->irq);
    mip_irq_enable(info);
    mutex_unlock(&info->lock);

    if(ret < fw_err_none) {
        dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    } else {
        dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    }

    return ret;
}

static ssize_t mip_sys_fw_update(struct device *dev, struct device_attribute *attr, char *buf) {
    struct i2c_client *client = to_i2c_client(dev);
    struct mip_ts_info *info = i2c_get_clientdata(client);
    int result = 0;
    u8 data[255];
    int ret = 0;

    memset(info->print_buf, 0, PAGE_SIZE);

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    ret = mip_fw_update_from_storage(info, info->fw_path_ext, true);

    switch(ret) {
    case fw_err_none:
        sprintf(data, "F/W update success.\n");
        break;
    case fw_err_uptodate:
        sprintf(data, "F/W is already up-to-date.\n");
        break;
    case fw_err_download:
        sprintf(data, "F/W update failed : Download error\n");
        break;
    case fw_err_file_type:
        sprintf(data, "F/W update failed : File type error\n");
        break;
    case fw_err_file_open:
        sprintf(data, "F/W update failed : File open error [%s]\n", info->fw_path_ext);
        break;
    case fw_err_file_read:
        sprintf(data, "F/W update failed : File read error\n");
        break;
    default:
        sprintf(data, "F/W update failed.\n");
        break;
    }

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

    strcat(info->print_buf, data);
    result = snprintf(buf, PAGE_SIZE, "%s\n", info->print_buf);
    return result;
}
static DEVICE_ATTR(fw_update, 0666, mip_sys_fw_update, NULL);

/**
* Sysfs attr info
*/
static struct attribute *mip_attrs[] = {
    &dev_attr_fw_update.attr,
    NULL,
};

/**
* Sysfs attr group info
*/
static const struct attribute_group mip_attr_group = {
    .attrs = mip_attrs,
};

static int mip_read_test(struct mip_ts_info *info) {
    int ret = 0;
    u8 rbuf[64];
    ret = mip_get_fw_version(info, rbuf);
    printk("%s - mip_read_test F/W Version : %02X.%02X %02X.%02X %02X.%02X %02X.%02X\n",
           __func__, rbuf[0], rbuf[1], rbuf[2], rbuf[3],
           rbuf[4], rbuf[5], rbuf[6], rbuf[7]);
    return ret;
}


/**
* Initial config
*/
static int mip_init_config(struct mip_ts_info *info) {
    u8 wbuf[8];
    u8 rbuf[64];
    int ret = 0;

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    //Product name
    wbuf[0] = MIP_R0_INFO;
    wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;
    ret = mip_i2c_read(info, wbuf, 2, rbuf, 16);
    if(ret) {
        goto ERROR;
    }
    memcpy(info->product_name, rbuf, 16);
    printk("%s - product_name[%s]\n", __func__, info->product_name);

    //Firmware version
    ret = mip_get_fw_version(info, rbuf);
    if(ret) {
        printk("mip_get_fw_version err ret = %d\n", ret);
        goto ERROR;
    }
    memcpy(info->fw_version, rbuf, 8);
    printk("%s - F/W Version : %02X.%02X %02X.%02X %02X.%02X %02X.%02X\n", __func__, info->fw_version[0], info->fw_version[1], info->fw_version[2], info->fw_version[3], info->fw_version[4], info->fw_version[5], info->fw_version[6], info->fw_version[7]);

    gtp_fw_version = rbuf[0] << 24 | rbuf[1] << 16 | rbuf[2] << 8 | rbuf[3];
    gtp_cfg_version = rbuf[4] << 24 | rbuf[5] << 16 | rbuf[6] << 8 | rbuf[7];

    //Resolution
    wbuf[0] = MIP_R0_INFO;
    wbuf[1] = MIP_R1_INFO_RESOLUTION_X;
    ret = mip_i2c_read(info, wbuf, 2, rbuf, 7);
    if(ret) {
        goto ERROR;
    }


#if MIP_AUTOSET_RESOLUTION
    //Set resolution using chip info
    info->max_x = (rbuf[0]) | (rbuf[1] << 8);
    info->max_y = (rbuf[2]) | (rbuf[3] << 8);
#else
    //Set resolution using platform data
    info->max_x = info->pdata->max_x;
    info->max_y = info->pdata->max_y;
#endif
    dev_dbg(&info->client->dev, "%s - max_x[%d] max_y[%d]\n", __func__, info->max_x, info->max_y);

    //Node info
    info->node_x = rbuf[4];
    info->node_y = rbuf[5];
    info->node_key = rbuf[6];
    dev_dbg(&info->client->dev, "%s - node_x[%d] node_y[%d] node_key[%d]\n", __func__, info->node_x, info->node_y, info->node_key);

    //Key info
    if(info->node_key > 0) {
        //Enable touchkey
        info->key_enable = true;
        info->key_num = info->node_key;
    }

    //Protocol
#if MIP_AUTOSET_EVENT_FORMAT
    wbuf[0] = MIP_R0_EVENT;
    wbuf[1] = MIP_R1_EVENT_SUPPORTED_FUNC;
    mip_i2c_read(info, wbuf, 2, rbuf, 7);
    info->event_format = (rbuf[4]) | (rbuf[5] << 8);
    info->event_size = rbuf[6];
#else
    info->event_format = 0;
    info->event_size = 6;
#endif
    dev_dbg(&info->client->dev, "%s - event_format[%d] event_size[%d] \n", __func__, info->event_format, info->event_size);

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}

/**
* Initialize driver
*/
static int mip_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct mip_ts_info *info;
    struct input_dev *input_dev;
    int ret = 0;

    dev_dbg(&client->dev, "%s [START]\n", __func__);

    if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "%s [ERROR] i2c_check_functionality\n", __func__);
        ret = -EIO;
        goto I2C_CHECK_ERROR;
    }

    //Init info data
    info = kzalloc(sizeof(struct mip_ts_info), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (!info || !input_dev) {
        dev_err(&client->dev, "%s [ERROR]\n", __func__);
        ret = -ENOMEM;
        goto info_ERROR;
    }

    info->client = client;
    info->input_dev = input_dev;
    info->irq = -1;
    info->init = true;
    info->power_state = -1;
    info->fw_path_ext = kstrdup(FW_PATH_EXTERNAL, GFP_KERNEL);

    //Config GPIO
    mip_config_gpio(info);
    //Power on
#ifdef __USE_LINUX_REGULATOR_FRAMEWORK__     //hardy 20151217
    tpd->reg = regulator_get(tpd->tpd_dev, TPD_POWER_SOURCE_CUSTOM);
    if (IS_ERR(tpd->reg)) {
        ret = PTR_ERR(tpd->reg);
        goto info_ERROR;
        //return PTR_ERR(tpd->reg);
    }
    ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);
    ret += regulator_enable(tpd->reg);
    if (ret)
        printk("Power on ic error!\n");
#endif
    msleep(100);
    mip_reboot(info);

    ret = mip_read_test(info);
    if(ret) {
        goto READ_TEST_ERROR;
    }

    mutex_init(&info->lock);
    //Get platform data
#if MIP_USE_DEVICETREE
    if (client->dev.of_node) {
        info->pdata  = devm_kzalloc(&client->dev, sizeof(struct melfas_platform_data), GFP_KERNEL);
        if (!info->pdata) {
            dev_err(&client->dev, "%s [ERROR] pdata devm_kzalloc\n", __func__);
            //goto error_platform_data;
        }

        ret = mip_parse_devicetree(&client->dev, info);
        if (ret) {
            dev_err(&client->dev, "%s [ERROR] mip_parse_dt\n", __func__);
            goto ERROR;
        }
    } else
#endif
    {
        //info->pdata = client->dev.platform_data;
        mip_config_platform_data(client, info);

        if (info->pdata == NULL) {
            dev_err(&client->dev, "%s [ERROR] pdata is null\n", __func__);
            ret = -EINVAL;
            goto ERROR;
        }
    }

    //Init input device
    info->input_dev->name = "MELFAS_" CHIP_NAME "_Touchscreen";
    snprintf(info->phys, sizeof(info->phys), "%s/input1", info->input_dev->name);

    info->input_dev->phys = info->phys;
    info->input_dev->id.bustype = BUS_I2C;
    info->input_dev->dev.parent = &client->dev;

#if MIP_USE_INPUT_OPEN_CLOSE
    info->input_dev->open = mip_input_open;
    info->input_dev->close = mip_input_close;
#endif

    //Set info data
    input_set_drvdata(input_dev, info);
    i2c_set_clientdata(client, info);

    //Set I2C DMA
    if(i2c_dma_buf_va == NULL) {
        i2c_dma_buf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &i2c_dma_buf_pa, GFP_KERNEL);
    }
    if(!i2c_dma_buf_va) {
        dev_err(&client->dev, "%s [ERROR] dma_alloc_coherent\n", __func__);
        goto ERROR;
    }
    dev_dbg(&client->dev, "%s - dma_alloc_coherent : pa[0x%08X]\n", __func__, i2c_dma_buf_pa);

    //Firmware update
#if MIP_USE_AUTO_FW_UPDATE
    /*ret = mip_fw_update_from_kernel(info);
    if(ret){
    	dev_err(&client->dev, "%s [ERROR] mip_fw_update_from_kernel\n", __func__);
    }*/
    mip_fw_update(info, client);
    if(ret) {
        dev_err(&client->dev, "%s [ERROR] mip_fw_update_from_kernel\n", __func__);
        printk("%s [ERROR] mip_fw_update_from_kernel\n", __func__);
    }
#endif

    //Initial config
    ret = mip_init_config(info);
    if(ret) {
        printk("mip_init_config err ret = %d\n", ret);
        goto ERROR;
    }

    printk("gtp_fw_version 0x%x gtp_cfg_version = 0x%x\n", gtp_fw_version, gtp_cfg_version);
    //update tp fw error recover tp fw from kernel code
    if(gtp_fw_version == 0xFFFFFFFF || gtp_cfg_version == 0xFFFFFFFF) {
		mip_kenel_fw_update(info, client);
		//Initial config
        ret = mip_init_config(info);
        if(ret) {
            printk("mip_kenel_fw_update mip_init_config err ret = %d\n", ret);
            goto ERROR;
        }
    }

    //Config input interface
    mip_config_input(info);

    //Register input device
    ret = input_register_device(input_dev);
    if (ret) {
        dev_err(&client->dev, "%s [ERROR] input_register_device\n", __func__);
        ret = -EIO;
        goto ERROR;
    }

    mip_info = info;

    //Config IRQ
    mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, mip_tpd_interrupt_handler, 0);

    //Enable device
    mip_enable(info);

    //Set interrupt handler
    /*
    ret = request_threaded_irq(client->irq, NULL, mip_interrupt, IRQF_TRIGGER_LOW | IRQF_ONESHOT, MIP_DEVICE_NAME, info);
    //ret = request_threaded_irq(client->irq, NULL, mip_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, MIP_DEVICE_NAME, info);
    if (ret) {
    	dev_err(&client->dev, "%s [ERROR] request_threaded_irq\n", __func__);
    	goto ERROR;
    }

    //disable_irq(client->irq);
    mip_irq_disable(info);
    info->irq = client->irq;
    */
    mip_interrupt_mtk_start(info);

#if MIP_USE_WAKEUP_GESTURE
    //Wake-lock for wake-up gesture mode
    //wake_lock_init(&mip_wake_lock, WAKE_LOCK_SUSPEND, "mip_wake_lock");
#endif

#if 0
    //#ifdef CONFIG_HAS_EARLYSUSPEND
    //Config early suspend
    info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    info->early_suspend.suspend = mip_early_suspend;
    info->early_suspend.resume = mip_late_resume;
    register_early_suspend(&info->early_suspend);
    dev_dbg(&client->dev, "%s - register_early_suspend\n", __func__);
#endif

    //Enable device
    //mip_enable(info);

    tpd_load_status = 1;

#if defined (MIP_FLIP_COVER_SWITCH)
    hall_state_charge_notify = mip_flip_cover_switch;
    smart_cover_flag_charge_notify = mip_flip_cover_flag_charge;
#endif

#if MIP_USE_DEV
    //Create dev node (optional)
    if(mip_dev_create(info)) {
        dev_err(&client->dev, "%s [ERROR] mip_dev_create\n", __func__);
        ret = -EAGAIN;
        goto ERROR;
    }

    //Create dev
    info->class = class_create(THIS_MODULE, MIP_DEVICE_NAME);
    device_create(info->class, NULL, info->mip_dev, NULL, MIP_DEVICE_NAME);
#endif

#if MIP_USE_SYS
    //Create sysfs for test mode (optional)
    if (mip_sysfs_create(info)) {
        dev_err(&client->dev, "%s [ERROR] mip_sysfs_create\n", __func__);
        ret = -EAGAIN;
        goto ERROR;
    }
#endif

    //Create sysfs
    if (sysfs_create_group(&client->dev.kobj, &mip_attr_group)) {
        dev_err(&client->dev, "%s [ERROR] sysfs_create_group\n", __func__);
        ret = -EAGAIN;
        goto ERROR;
    }

    if (sysfs_create_link(NULL, &client->dev.kobj, MIP_DEVICE_NAME)) {
        dev_err(&client->dev, "%s [ERROR] sysfs_create_link\n", __func__);
        ret = -EAGAIN;
        goto ERROR;
    }

    dev_dbg(&client->dev, "%s [DONE]\n", __func__);
    dev_info(&client->dev, "MELFAS " CHIP_NAME " Touchscreen is initialized successfully.\n");
    return 0;

ERROR:

READ_TEST_ERROR:

    kfree(info);
#ifdef __USE_LINUX_REGULATOR_FRAMEWORK__   //hardy 20151217
    regulator_disable(tpd->reg);
    msleep(20);
    regulator_put(tpd->reg);
#endif

info_ERROR:
I2C_CHECK_ERROR:

    dev_dbg(&client->dev, "%s [ERROR]\n", __func__);
    dev_err(&client->dev, "MELFAS " CHIP_NAME " Touchscreen initialization failed.\n");
    return ret;
}

/**
* Remove driver
*/
static int mip_remove(struct i2c_client *client) {
    struct mip_ts_info *info = i2c_get_clientdata(client);

    //IRQ
    if (info->irq >= 0) {
        free_irq(info->irq, info);
    }

    //I2C DMA
    if(i2c_dma_buf_va) {
        dma_free_coherent(NULL, 4096, i2c_dma_buf_va, i2c_dma_buf_pa);
        i2c_dma_buf_va = NULL;
        i2c_dma_buf_pa = 0;
    }

#if MIP_USE_SYS
    mip_sysfs_remove(info);
#endif

    sysfs_remove_group(&info->client->dev.kobj, &mip_attr_group);
    sysfs_remove_link(NULL, MIP_DEVICE_NAME);
    kfree(info->print_buf);

#if MIP_USE_DEV
    device_destroy(info->class, info->mip_dev);
    class_destroy(info->class);
#endif

#if 0
    //#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&info->early_suspend);
#endif

    input_unregister_device(info->input_dev);

    kfree(info->fw_name);
    kfree(info);

    return 0;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
/**
* Device suspend event handler
*/
int mip_suspend(struct device *dev) {
    struct i2c_client *client = to_i2c_client(dev);
    struct mip_ts_info *info = i2c_get_clientdata(client);

    dev_dbg(&client->dev, "%s [START]\n", __func__);
    printk("%s [START]\n", __func__);

#if defined (MIP_FLIP_COVER_SWITCH)
    mip_flip_cover_update();
#endif

#if MIP_USE_WAKEUP_GESTURE
    mip_gesture_wakeup_flag = gesture_wakeup_flag;
    MIP_INFO("mip_gesture_wakeup_flag = %d\n", mip_gesture_wakeup_flag);
    if(mip_gesture_wakeup_flag) {
        info->wakeup_gesture_code = 0;
        //mip_set_wakeup_gesture_type(info, MIP_EVENT_GESTURE_ALL);
        mip_set_power_state(info, MIP_CTRL_POWER_LOW);

        info->nap_mode = true;
        dev_dbg(&info->client->dev, "%s - nap mode : on\n", __func__);

        //if(!wake_lock_active(&mip_wake_lock)) {
        //    wake_lock(&mip_wake_lock);
        //    dev_dbg(&info->client->dev, "%s - wake_lock\n", __func__);
        //}
    } else {
        MIP_INFO("MIP CTRL POWER LOW\n");
        mip_disable(info);
    }
#else
    mip_disable(info);
#endif

    mip_clear_input(info);
    dev_dbg(&client->dev, "%s [DONE]\n", __func__);
    printk("%s [DONE]\n", __func__);

    return 0;

}

/**
* Device resume event handler
*/
int mip_resume(struct device *dev) {
    struct i2c_client *client = to_i2c_client(dev);
    struct mip_ts_info *info = i2c_get_clientdata(client);
    int ret = 0;

    dev_dbg(&client->dev, "%s [START]\n", __func__);
    printk("%s [START]\n", __func__);

#if MIP_USE_WAKEUP_GESTURE
    printk("gesture_wakeup_flag = %d mip_gesture_wakeup_flag = %d\n", gesture_wakeup_flag, mip_gesture_wakeup_flag);
    if(mip_gesture_wakeup_flag) {
		mip_set_power_state(info, MIP_CTRL_POWER_ACTIVE);
        //if(wake_lock_active(&mip_wake_lock)) {
        //    wake_unlock(&mip_wake_lock);
        //    dev_dbg(&info->client->dev, "%s - wake_unlock\n", __func__);
        //}

        info->nap_mode = false;
        dev_dbg(&info->client->dev, "%s - nap mode : off\n", __func__);
    } else {
        mip_power_off(info);
        mip_power_on(info);
        mip_enable(info);
    }
#else
    mip_enable(info);
#endif

#if defined (MIP_FLIP_COVER_SWITCH)
    mip_flip_cover_update();
#endif

    dev_dbg(&client->dev, "%s [DONE]\n", __func__);
    printk("%s [DONE]\n", __func__);
    return ret;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
/**
* Early suspend handler
*/
void mip_early_suspend(struct early_suspend *h) {
    //struct mip_ts_info *info = container_of(h, struct mip_ts_info, early_suspend);
    struct mip_ts_info *info = mip_info;

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    printk("%s [START]\n", __func__);
    mip_suspend(&info->client->dev);

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    printk("%s [DONE]\n", __func__);
}

/**
* Late resume handler
*/
void mip_late_resume(struct early_suspend *h) {
    //struct mip_ts_info *info = container_of(h, struct mip_ts_info, early_suspend);
    struct mip_ts_info *info = mip_info;

    dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    printk("%s [START]\n", __func__);

    mip_resume(&info->client->dev);

    dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    printk("%s [DONE]\n", __func__);
}
#endif

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
/**
* PM info
*/
const struct dev_pm_ops mip_pm_ops = {
#if 0
    SET_SYSTEM_SLEEP_PM_OPS(mip_suspend, mip_resume)
#else
    .suspend	= mip_suspend,
    .resume = mip_resume,
#endif
};
#endif

#if MIP_USE_DEVICETREE
/**
* Device tree match table
*/
static const struct of_device_id mip_match_table[] = {
    { .compatible = "melfas,"MIP_DEVICE_NAME,},
    {},
};
MODULE_DEVICE_TABLE(of, mip_match_table);
#endif

/**
* I2C Device ID
*/
static const struct i2c_device_id mip_id[] = {
    {MIP_DEVICE_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, mip_id);

/**
* I2C detect
*/
static int mip_detect (struct i2c_client *client, struct i2c_board_info *info) {
    strcpy(info->type, TPD_DEVICE);
    return 0;
}

/**
* I2C driver info
*/
static struct i2c_driver mip_driver = {
    .id_table = mip_id,
    .probe = mip_probe,
    .remove = mip_remove,
    .detect = mip_detect,
    .driver = {
        .name = MIP_DEVICE_NAME,
        .owner = THIS_MODULE,
#if MIP_USE_DEVICETREE
        .of_match_table = mip_match_table,
#endif
        //
        //#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
        //		.pm 	= &mip_pm_ops,
        //#endif
    },
};

#if 0
/**
* Init driver
*/
static int __init mip_init(void) {
    return i2c_add_driver(&mip_driver);
}

/**
* Exit driver
*/
static void __exit mip_exit(void) {
    i2c_del_driver(&mip_driver);
}

module_init(mip_init);
module_exit(mip_exit);
#endif

/**
* MTK tpd interrupt
*/
void mip_tpd_interrupt_handler(void) {
    MIP_DEBUG("%s [START]\n", __func__);

    mtk_eint_count++;
    MIP_DEBUG("%s - eint_count[%d]\n", __func__, mtk_eint_count);

    mtk_eint_flag = 1;
    wake_up_interruptible(&waiter);

    MIP_DEBUG("%s [DONE]\n", __func__);
}

/**
* MTK tpd local init
*/
static int mip_tpd_local_init(void) {
    MIP_DEBUG("%s [START]\n", __func__);

    if (tpd_load_status == 1) {
        printk("mip === add touch panel driver already.\n");
        return -1;
    }

    if(i2c_add_driver(&mip_driver) != 0) {
        MIP_DEBUG("%s [ERROR] i2c_add_driver\n", __func__);
        goto ERROR;
    }

    if (tpd_load_status == 0) {
        i2c_del_driver(&mip_driver);
        printk("add error mip touch panel driver.\n");
        return -1;
    }

#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, mip_tpd_keys_local, mip_tpd_keys_dim_local);
#endif

    tpd_type_cap = 1;

    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;

ERROR:
    MIP_DEBUG("%s [ERROR]\n", __func__);
    return -1;
}

/**
* MTK tpd driver info
*/
static struct tpd_driver_t mip_tpd_driver = {
    .tpd_device_name = MIP_DEVICE_NAME,
    .tpd_local_init = mip_tpd_local_init,
    .suspend = mip_early_suspend,
    .resume = mip_late_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};

/**
* MTK I2C board info
*/
static struct i2c_board_info __initdata mip_board_info = {
    I2C_BOARD_INFO(MIP_DEVICE_NAME, TPD_I2C_ADDR),
};


/**
* MTK tpd driver init
*/
static int __init tpd_driver_init(void) {
    int ret = 0;

    MIP_DEBUG("%s [START]\n", __func__);

    mip_board_info.addr = TPD_I2C_ADDR;
    i2c_register_board_info(TPD_I2C_BUS, &mip_board_info, 1);

#ifdef TPD_HAVE_BUTTON
    mip_tpd_driver.tpd_have_button = 1;
#else
    mip_tpd_driver.tpd_have_button = 0;
#endif

    if(tpd_driver_add(&mip_tpd_driver) < 0) {
        MIP_DEBUG("%s [ERROR] tpd_driver_add\n", __func__);
    }

    MIP_DEBUG("%s [DONE]\n", __func__);
    return 0;

ERROR:
    MIP_DEBUG("%s [ERROR]\n", __func__);
    return -1;
}

/**
* MTK tpd driver exit
*/
static void __exit tpd_driver_exit(void) {
    MIP_DEBUG("%s [START]\n", __func__);

    //input_unregister_device(tpd->dev);
    if(tpd_driver_remove(&mip_tpd_driver)) {
        MIP_DEBUG("%s [ERROR] tpd_driver_remove\n", __func__);
    }

    MIP_DEBUG("%s [DONE]\n", __func__);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


MODULE_DESCRIPTION("MELFAS MIP4 Touchscreen for MediaTek");
MODULE_VERSION("2015.06.22");
MODULE_AUTHOR("Jee, SangWon <jeesw@melfas.com>");
MODULE_LICENSE("GPL");

