/*
 * MELFAS MMS400 Touchscreen for MediaTek
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * Main Functions
 *
 * This module is tested on the Google Android One (MediaTek MT6582) platform.
 *
 */

#include "melfas_mms400.h"

#if MMS_USE_NAP_MODE
struct wake_lock mms_wake_lock;
#endif

/**
* MTK
*/
struct mms_ts_info *mms_info;
extern struct tpd_device *tpd;

#define MMS_I2C_SPEED 300	//kHz
static u8 *i2c_dma_buf_va = NULL;
static dma_addr_t i2c_dma_buf_pa = 0;
static struct work_struct mms_resume_work;

struct task_struct *mms_eint_thread = NULL;
int mms_eint_flag = 0;
int mms_eint_count = 0;
DECLARE_WAIT_QUEUE_HEAD(waiter);

#ifdef TPD_HAVE_BUTTON
int mms_tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
int mms_tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
static unsigned int mms_wakeup_control_flag = 1;
/**
* Reboot chip
*
* Caution : IRQ must be disabled before mms_reboot and enabled after mms_reboot.
*/
void mms_reboot(struct mms_ts_info *info) {
    struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    i2c_lock_adapter(adapter);

    mms_power_off(info);
    mms_power_on(info);

    i2c_unlock_adapter(adapter);

    msleep(10);

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

/**
* Reboot chip
*
* Caution : IRQ must be disabled before mms_reboot and enabled after mms_reboot.
*/
void mms_reset(struct mms_ts_info *info) {
    struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    printk("[Melfas] %s() \n", __func__);
    i2c_lock_adapter(adapter);


    msleep(50);
    printk("[Melfas] %s() RST set to 0\n", __func__);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);

    msleep(100);

    printk("[Melfas] %s() RST set to 1\n", __func__);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

    i2c_unlock_adapter(adapter);

    msleep(50);

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

/**
* Normal I2C Read
*/
int mms_i2c_read_normal(struct mms_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len) {
    int res;

    struct i2c_msg msg[] = {
        {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG),
            .flags = 0,
            .buf = write_buf,
            .len = write_len,
            .timing = MMS_I2C_SPEED
        }, {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG),
            .flags = I2C_M_RD,
            .buf = read_buf,
            .len = read_len,
            .timing = MMS_I2C_SPEED
        },
    };

    ////dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    //res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));
    res = i2c_transfer(info->client->adapter, msg, 2);

    //if(res == ARRAY_SIZE(msg)){
    if(res == 2) {
        goto EXIT;
    } else if(res < 0) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n", __func__, res);
    }
    //else if(res != ARRAY_SIZE(msg)){
    else if(res != 2) {
        //dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, ARRAY_SIZE(msg), res);
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, 2, res);
    } else {
        dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n", __func__, res);
    }

    goto ERROR;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;

EXIT:
    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* DMA I2C Read
*/
int mms_i2c_read_dma(struct mms_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len) {

    int res;
    int i = 0;
    int retry = 0;

    struct i2c_msg msg[] = {
        {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG),
            .flags = 0,
            .buf = write_buf,
            .len = write_len,
            .timing = MMS_I2C_SPEED
        }, {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = I2C_M_RD,
            //.buf = read_buf,
            .buf = (u8 *)i2c_dma_buf_pa,
            .len = read_len,
            .timing = MMS_I2C_SPEED
        },
    };

    //		dev_dbg(&info->client->dev, "%s [START], msg[0] = %d, msg[1] = %d,msg[2] = %d \n", __func__,msg[0],msg[1],msg[2]);
    //	printk("%s \n",__func__);

    for (retry = 0; retry < 5; ++retry) {
        //res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));
        res = i2c_transfer(info->client->adapter, msg, 2);

        //if(res == ARRAY_SIZE(msg)){
        //		dev_dbg(&info->client->dev, "%s [START], res = %d \n", __func__,res);
        //		printk("%s res = %d \n",__func__,res);
        if(res == 2) {
            for(i = 0; i < read_len; i++) {
                read_buf[i] = i2c_dma_buf_va[i];
            }

            goto EXIT;
        } else if(res < 0) {
            dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n", __func__, res);
        }
        //else if(res != ARRAY_SIZE(msg)){
        else if(res != 2) {
            //dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, ARRAY_SIZE(msg), res);
            dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, 1, res);
        } else {
            dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n", __func__, res);
        }
    }
    goto ERROR;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;

EXIT:
    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    printk("%s [DONE]\n", __func__);
    return 0;

}

/**
* I2C Read
*/
int mms_i2c_read(struct mms_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len) {
    int retry = I2C_RETRY_COUNT;

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    //dev_dbg(&info->client->dev, "%s - write_len[%d] read_len[%d]\n", __func__, write_len, read_len);

    while(retry--) {
        if(read_len <= 8) {
            if(mms_i2c_read_normal(info, write_buf, write_len, read_buf, read_len) == 0) {
                goto EXIT;
            }
        } else {
            if(mms_i2c_read_dma(info, write_buf, write_len, read_buf, read_len) == 0) {
                goto EXIT;
            }
        }
    }

    goto ERROR;

ERROR:
    mms_reboot(info);
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;

EXIT:
    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

#if 0
/**
* I2C Read (Continue)
*/
int mms_i2c_read_next(struct mms_ts_info *info, char *read_buf, int start_idx, unsigned int read_len) {
    int retry = I2C_RETRY_COUNT;
    int res;
    u8 rbuf[read_len];

    while(retry--) {
        res = i2c_master_recv(info->client, rbuf, read_len);

        if(res == read_len) {
            goto DONE;
        } else if(res < 0) {
            dev_err(&info->client->dev, "%s [ERROR] i2c_master_recv - errno [%d]\n", __func__, res);
        } else if(res != read_len) {
            dev_err(&info->client->dev, "%s [ERROR] length mismatch - read[%d] result[%d]\n", __func__, read_len, res);
        } else {
            dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n", __func__, res);
        }
    }

    goto ERROR_REBOOT;

ERROR_REBOOT:
    mms_reboot(info);
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;

DONE:
    memcpy(&read_buf[start_idx], rbuf, read_len);

    return 0;
}
#endif

/**
* Normal I2C Write
*/
int mms_i2c_write_normal(struct mms_ts_info *info, char *write_buf, unsigned int write_len) {
    int res;

    struct i2c_msg msg[] = {
        {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG),
            .flags = 0,
            .buf = write_buf,
            .len = write_len,
            .timing = MMS_I2C_SPEED
        },
    };

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    //res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));
    res = i2c_transfer(info->client->adapter, msg, 1);

    //if(res == ARRAY_SIZE(msg)){
    if(res == 1) {
        goto EXIT;
    } else if(res < 0) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n", __func__, res);
    }
    //else if(res != ARRAY_SIZE(msg)){
    else if(res != 1) {
        //dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, ARRAY_SIZE(msg), res);
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, 1, res);
    } else {
        dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n", __func__, res);
    }

    goto ERROR;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;

EXIT:
    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* DMA I2C Write
*/
int mms_i2c_write_dma(struct mms_ts_info *info, char *write_buf, unsigned int write_len) {
    int res;
    int i = 0;

    struct i2c_msg msg[] = {
        {
            //.addr = info->client->addr,
            .addr = ((info->client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = 0,
            //.buf = write_buf,
            .buf = (u8 *)i2c_dma_buf_pa,
            .len = write_len,
            .timing = MMS_I2C_SPEED
        },
    };

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    for(i = 0; i < write_len; i++) {
        i2c_dma_buf_va[i] = write_buf[i];
    }

    //res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));
    res = i2c_transfer(info->client->adapter, msg, 1);

    //if(res == ARRAY_SIZE(msg)){
    if(res == 1) {
        goto EXIT;
    } else if(res < 0) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - errno[%d]\n", __func__, res);
    }
    //else if(res != ARRAY_SIZE(msg)){
    //	dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, ARRAY_SIZE(msg), res);
    else if(res != 1) {
        dev_err(&info->client->dev, "%s [ERROR] i2c_transfer - size[%d] result[%d]\n", __func__, 1, res);
    } else {
        dev_err(&info->client->dev, "%s [ERROR] unknown error [%d]\n", __func__, res);
    }

    goto ERROR;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;

EXIT:
    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* I2C Write
*/
int mms_i2c_write(struct mms_ts_info *info, char *write_buf, unsigned int write_len) {
    int retry = I2C_RETRY_COUNT;

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    //dev_dbg(&info->client->dev, "%s - write_len[%d]\n", __func__, write_len);

    while(retry--) {
        if(write_len <= 8) {
            if(mms_i2c_write_normal(info, write_buf, write_len) == 0) {
                goto EXIT;
            }
        } else {
            if(mms_i2c_write_dma(info, write_buf, write_len) == 0) {
                goto EXIT;
            }
        }
    }

    goto ERROR;

ERROR:
    mms_reboot(info);
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;

EXIT:
    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* Enable device
*/
int mms_enable(struct mms_ts_info *info) {
    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    printk("[Melfas] %s() info->enabled=%d\n", __func__, info->enabled);
    if (info->enabled) {
        dev_err(&info->client->dev, "%s [ERROR] device already enabled\n", __func__);
        //goto EXIT;
    }

    mutex_lock(&info->lock);

    mms_reboot(info);
    //	mms_power_on(info);

    //enable_irq(info->client->irq);
    mms_irq_enable(info);
    info->enabled = true;

    mutex_unlock(&info->lock);

    //Post-enable process
    if(info->disable_esd == true) {
        //Disable ESD alert
        mms_disable_esd_alert(info);
    }

EXIT:
    dev_info(&info->client->dev, MMS_DEVICE_NAME" Enabled\n");

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

/**
* Disable device
*/
int mms_disable(struct mms_ts_info *info) {
    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    if (!info->enabled) {
        dev_err(&info->client->dev, "%s [ERROR] device already disabled\n", __func__);
        goto EXIT;
    }

    mutex_lock(&info->lock);

    info->enabled = false;
    //disable_irq(info->client->irq);
    mms_irq_disable(info);

    msleep(5);

    //	mms_reboot(info);

    mms_power_off(info);

    mutex_unlock(&info->lock);

EXIT:
    dev_info(&info->client->dev, MMS_DEVICE_NAME" Disabled\n");

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;
}

#if MMS_USE_INPUT_OPEN_CLOSE
/**
* Open input device
*/
static int mms_input_open(struct input_dev *dev) {
    struct mms_ts_info *info = input_get_drvdata(dev);

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    if(info->init == true) {
        info->init = false;
    } else {
        mms_enable(info);
    }

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

    return 0;
}

/**
* Close input device
*/
static void mms_input_close(struct input_dev *dev) {
    struct mms_ts_info *info = input_get_drvdata(dev);

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    mms_disable(info);

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

    return;
}
#endif

/**
* Get ready status
*/
int mms_get_ready_status(struct mms_ts_info *info) {
    u8 wbuf[16];
    u8 rbuf[16];
    int ret = 0;

    ////dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_READY_STATUS;
    if(mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
        dev_err(&info->client->dev, "%s [ERROR] mms_i2c_read\n", __func__);
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
        if(mms_i2c_write(info, wbuf, 3)) {
            dev_err(&info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
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
int mms_get_fw_version(struct mms_ts_info *info, u8 *ver_buf) {
    u8 rbuf[8];
    u8 wbuf[2];
    int i;

    wbuf[0] = MIP_R0_INFO;
    wbuf[1] = MIP_R1_INFO_VERSION_BOOT;
    if(mms_i2c_read(info, wbuf, 2, rbuf, 8)) {
        goto ERROR;
    };

    for(i = 0; i < MMS_FW_MAX_SECT_NUM; i++) {
        ver_buf[0 + i * 2] = rbuf[1 + i * 2];
        ver_buf[1 + i * 2] = rbuf[0 + i * 2];
    }

    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

//add hschoi test
/**
* Read chip firmware version
*/
int mms_get_chip_id(struct mms_ts_info *info, u8 *ver_buf) {
    u8 rbuf[8];
    u8 wbuf[2];
    int i;

    wbuf[0] = MIP_R0_INFO;
    wbuf[1] = MIP_R1_INFO_IC_ID;
    if(mms_i2c_read(info, wbuf, 2, rbuf, 8)) {
        goto ERROR;
    };

    for(i = 0; i < MMS_FW_MAX_SECT_NUM; i++) {
        ver_buf[0 + i * 2] = rbuf[1 + i * 2];
        ver_buf[1 + i * 2] = rbuf[0 + i * 2];
    }

    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Read chip firmware version for u16
*/
int mms_get_fw_version_u16(struct mms_ts_info *info, u16 *ver_buf_u16) {
    u8 rbuf[8];
    int i;

    if(mms_get_fw_version(info, rbuf)) {
        goto ERROR;
    }

    for(i = 0; i < MMS_FW_MAX_SECT_NUM; i++) {
        ver_buf_u16[i] = (rbuf[0 + i * 2] << 8) | rbuf[1 + i * 2];
    }

    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Disable ESD alert
*/
int mms_disable_esd_alert(struct mms_ts_info *info) {
    u8 wbuf[4];
    u8 rbuf[4];

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_DISABLE_ESD_ALERT;
    wbuf[2] = 1;
    if(mms_i2c_write(info, wbuf, 3)) {
        dev_err(&info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
        goto ERROR;
    }

    if(mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
        dev_err(&info->client->dev, "%s [ERROR] mms_i2c_read\n", __func__);
        goto ERROR;
    }

    if(rbuf[0] != 1) {
        //dev_dbg(&info->client->dev, "%s [ERROR] failed\n", __func__);
        goto ERROR;
    }

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Enable gesture wakeup mode
*/

int mms_enable_gesture_wakeup_mode(struct mms_ts_info *info) {
    u8 wbuf[4];
    u8 rbuf[4];

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_POWER_STATE;
    wbuf[2] = 1;
    if(mms_i2c_write(info, wbuf, 3)) {
        dev_err(&info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
        goto ERROR;
    }

    if(mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
        dev_err(&info->client->dev, "%s [ERROR] mms_i2c_read\n", __func__);
        goto ERROR;
    }

    if(rbuf[0] != 1) {
        //dev_dbg(&info->client->dev, "%s [ERROR] failed\n", __func__);
        goto ERROR;
    }

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Enable normal active mode
*/

int mms_enable_nomal_active_mode(struct mms_ts_info *info) {
    u8 wbuf[4];
    u8 rbuf[4];

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_POWER_STATE;
    wbuf[2] = 0;
    if(mms_i2c_write(info, wbuf, 3)) {
        dev_err(&info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
        goto ERROR;
    }

    if(mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
        dev_err(&info->client->dev, "%s [ERROR] mms_i2c_read\n", __func__);
        goto ERROR;
    }

    if(rbuf[0] != 0) {
        //dev_dbg(&info->client->dev, "%s [ERROR] failed\n", __func__);
        goto ERROR;
    }

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Re-Calibration
*/

int mms_recalibration(struct mms_ts_info *info) {
    u8 wbuf[4];
    u8 rbuf[4];

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_RECALIBRATE;
    wbuf[2] = 1;
    if(mms_i2c_read(info, wbuf, 3, rbuf, 1)) {
        goto ERROR;
    };

    return 0;

ERROR:
    //dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Chracter Setting Bit
*/

#define SET_BIT(flag, g)                  ((flag) |= (0x1 << g))
#define CLEAR_BIT(flag, g)                ((flag) ^= (0x1 << g))
#define CHECK_BIT(flag, g)                (((flag) & (0x1 << g)) >> g)

int mms_gesture_type_setting(struct mms_ts_info *info, u8 *Gesture_buf) {
    u8 wbuf[8];
    u8 rbuf[8];
    u8 gesture_flag = rbuf[2];
    u8 gesture_setting = rbuf[0];
    u32 ulGestureType = 0;
    int i;

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    if(gesture_flag == 0x01) { //All Gesture
        if(gesture_setting == 0x00) {
            ulGestureType = 0x00000000; //all off
        } else {
            for (i = 0; i < 32; i++) {
                SET_BIT(ulGestureType, i); //all on
            }
        }
    } else if(gesture_flag == 0x02) { //Double-Tap
        if(gesture_setting == 0x00) {
            CLEAR_BIT(ulGestureType, 23); //D-Tap off
        } else {
            SET_BIT(ulGestureType, 23); //D-Tap On
        }

    } else if(gesture_flag == 0x03) { //Direction
        for (i = 0; i < 4; i++) {
            if (CHECK_BIT(gesture_setting, i) == true) {
                switch (i + 1) {
                case 1:
                    //Right
                    SET_BIT(ulGestureType, 19);
                    break;
                case 2:
                    //Left
                    SET_BIT(ulGestureType, 21);
                    break;
                case 3:
                    //Down
                    SET_BIT(ulGestureType, 20);
                    break;
                case 4:
                    //Up
                    SET_BIT(ulGestureType, 22);
                    break;
                }
            }

        }
    } else if(gesture_flag == 0x04) { //Character
        for (i = 0; i < 8; i++) {
            if (CHECK_BIT(gesture_setting, i) == true) {
                switch (i + 1) {
                case 1:
                    //V
                    SET_BIT(ulGestureType, 2); //normal V
                    SET_BIT(ulGestureType, 8); // 90degree V
                    SET_BIT(ulGestureType, 9); // 180degree V
                    break;
                case 2:
                    //C
                    SET_BIT(ulGestureType, 0);
                    break;
                case 3:
                    //E
                    SET_BIT(ulGestureType, 7);
                    break;
                case 4:
                    //W
                    SET_BIT(ulGestureType, 1);
                    break;
                case 5:
                    //M
                    SET_BIT(ulGestureType, 3);
                    break;
                case 6:
                    //S
                    SET_BIT(ulGestureType, 4);
                    break;
                case 7:
                    //Null
                    break;
                case 8:
                    //O
                    SET_BIT(ulGestureType, 6);
                    break;
                }
            }

        }

    } else {
        ulGestureType = 0xFFFFFFFF; //all on
    }

    wbuf[0] = MIP_R0_CTRL;
    wbuf[1] = MIP_R1_CTRL_GESTURE_TYPE;
    wbuf[2] = ulGestureType;
    if(mms_i2c_read(info, wbuf, 6, rbuf, 6)) {
        goto ERROR;
    };

    return 0;

ERROR:
    //dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}


/**
* Alert event handler - ESD
*/
static int mms_alert_handler_esd(struct mms_ts_info *info, u8 *rbuf) {
    u8 frame_cnt = rbuf[2];

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    //dev_dbg(&info->client->dev, "%s - frame_cnt[%d]\n", __func__, frame_cnt);

    if(frame_cnt == 0) {
        //sensor crack, not ESD
        info->esd_cnt++;
        //dev_dbg(&info->client->dev, "%s - esd_cnt[%d]\n", __func__, info->esd_cnt);

        if(info->disable_esd == true) {
            mms_disable_esd_alert(info);
        } else if(info->esd_cnt > ESD_COUNT_FOR_DISABLE) {
            //Disable ESD alert
            if(mms_disable_esd_alert(info)) {
            } else {
                info->disable_esd = true;
            }
        } else {
            //Reset chip
            mms_reboot(info);
        }
    } else {
        //ESD detected
        //Reset chip
        mms_reboot(info);
        info->esd_cnt = 0;
    }

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

    //ERROR:
    //dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    //return 1;
}

/**
* Alert event handler - Wake-up
*/
static int mms_alert_handler_wakeup(struct mms_ts_info *info, u8 *rbuf) {
    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    if(mms_wakeup_event_handler(info, rbuf)) {
        goto ERROR;
    }

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Interrupt handler
*/
//static irqreturn_t mms_interrupt(int irq, void *dev_id)
int mms_interrupt(void *data) {
    //struct mms_ts_info *info = dev_id;
    struct mms_ts_info *info = data;
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

    //dev_dbg(&client->dev, "%s [START]\n", __func__);

    do {
        //dev_dbg(&client->dev, "%s - wait\n", __func__);

        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, mms_eint_flag != 0);

        mms_eint_flag = 0;
        //dev_dbg(&client->dev, "%s - eint_count[%d]\n", __func__, mms_eint_count);

        if(info->enabled == false) {
            //dev_dbg(&client->dev, "%s - skip : enabled [false]\n", __func__);
            msleep(1);
            goto NEXT;
        }

        if(info->init == true) {
            info->init = false;
            //dev_dbg(&client->dev, "%s - skip : init [false]\n", __func__);
            goto NEXT;
        }

        if(info->irq_pause == true) {
            //dev_dbg(&client->dev, "%s - skip : irq_pause\n", __func__);
            goto NEXT;
        }

        set_current_state(TASK_RUNNING);
        //dev_dbg(&client->dev, "%s - run\n", __func__);
#if 0
        //Read first packet
        wbuf[0] = MIP_R0_EVENT;
        wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
        if(mms_i2c_read(info, wbuf, 2, rbuf, (1 + event_size))) {
            dev_err(&client->dev, "%s [ERROR] Read packet info\n", __func__);
            goto ERROR;
        }

        //dev_dbg(&client->dev, "%s - info [0x%02X]\n", __func__, rbuf[0]);

        //Check event
        size = (rbuf[0] & 0x7F);
        if(size <= 0) {
            dev_err(&client->dev, "%s [ERROR] packet size [%d]\n", __func__, size);
            goto NEXT;
        }
        //dev_dbg(&client->dev, "%s - packet size [%d]\n", __func__, size);

        category = ((rbuf[0] >> 7) & 0x1);
        if(category == 0) {
            //Touch event
            if(size > event_size) {
                //Read next packet
                /*
                if(mms_i2c_read_next(info, rbuf, (1 + event_size), (size - event_size))){
                	dev_err(&client->dev, "%s [ERROR] Read next packet\n", __func__);
                	goto ERROR;
                }
                */
            }

            info->esd_cnt = 0;

            mms_input_event_handler(info, size, rbuf);
        }
#else
        //Read packet info
        wbuf[0] = MIP_R0_EVENT;
        wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
        if(mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
            //if(mms_i2c_read(info, wbuf, 2, rbuf, (1 + event_size + 0))){
            dev_err(&client->dev, "%s [ERROR] Read packet info\n", __func__);
            goto ERROR;
        }

        //dev_dbg(&client->dev, "%s - info [0x%02X]\n", __func__, rbuf[0]);

        //Check event
        size = (rbuf[0] & 0x7F);
        if(size <= 0) {
            dev_err(&client->dev, "%s [ERROR] packet size [%d]\n", __func__, size);
            goto NEXT;
        }
        //dev_dbg(&client->dev, "%s - packet size [%d]\n", __func__, size);

        category = ((rbuf[0] >> 7) & 0x1);
        if(category == 0) {
            //Touch event
            wbuf[0] = MIP_R0_EVENT;
            wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
            if(mms_i2c_read(info, wbuf, 2, rbuf, size)) {
                dev_err(&client->dev, "%s [ERROR] Read touch event packet\n", __func__);
                goto ERROR;
            }

            info->esd_cnt = 0;

            mms_input_event_handler(info, size, rbuf);
        }
#endif
        else {
            //Alert event
            wbuf[0] = MIP_R0_EVENT;
            wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
            if(mms_i2c_read(info, wbuf, 2, rbuf, size)) {
                dev_err(&client->dev, "%s [ERROR] Read touch event packet\n", __func__);
                goto ERROR;
            }
            alert_type = rbuf[0];

            //dev_dbg(&client->dev, "%s - alert type [%d]\n", __func__, alert_type);
            printk("[Melfas]%s - alert type [%d]\n", __func__, alert_type);


            if(alert_type == MIP_ALERT_ESD) {
                //ESD detection
                if(mms_alert_handler_esd(info, rbuf)) {
                    goto ERROR;
                }
            } else if(alert_type == MIP_ALERT_WAKEUP) {
                //Wake-up gesture
                if(mms_alert_handler_wakeup(info, rbuf)) {
                    goto ERROR;
                }
            } else {
                dev_err(&client->dev, "%s [ERROR] Unknown alert type [%d]\n", __func__, alert_type);
                mms_reset(info); //modified by YJOH(MFS)   20150615
                goto ERROR;
            }
        }

        goto NEXT;

ERROR:
        //if(RESET_ON_EVENT_ERROR){
        dev_info(&client->dev, "%s - Reset on error\n", __func__);

        mms_disable(info);
        mms_clear_input(info);
        mms_enable(info);
        //}
        dev_err(&client->dev, "%s [ERROR]\n", __func__);

NEXT:
        mms_irq_enable(info);
        mms_eint_count--;
    } while(!kthread_should_stop());

    info->irq_enabled = false;

    //dev_dbg(&client->dev, "%s [DONE]\n", __func__);
    return 0;
}

int mms_interrupt_start(struct mms_ts_info *info) {
    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    info->irq_pause = false;

    //mms_eint_thread = kthread_run(mms_interrupt, info, TPD_DEVICE);
    mms_eint_thread = kthread_run(mms_interrupt, info, MMS_DEVICE_NAME);
    if(IS_ERR(mms_eint_thread)) {
        dev_err(&info->client->dev, "%s [ERROR] kthread_run\n", __func__);
        goto ERROR;
    }

    info->irq_enabled = true;
    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    info->irq_enabled = false;
    //dev_dbg(&info->client->dev, "%s [ERROR]\n", __func__);
    return 1;
}

/**
* Update firmware from kernel built-in binary
*/
int mms_fw_update_from_kernel(struct mms_ts_info *info) {
    const char *fw_name = INTERNAL_FW_PATH;
    const struct firmware *fw;
    int retry = 3;
    int ret;


    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    //Disable IRQ
    mutex_lock(&info->lock);
    //disable_irq(info->client->irq);
    mms_irq_disable(info);

    //Get firmware

    request_firmware(&fw, fw_name, &info->client->dev);


    if (!fw) {
        dev_err(&info->client->dev, "%s [ERROR] request_firmware\n", __func__);
        goto ERROR;
    }

    //Update fw
    do {
        //ret = mms_flash_fw(info, fw->data, fw->size, false, true);
        ret = mms_flash_fw(info, fw->data, fw->size, true, true);
        if(ret >= fw_err_none) {
            break;
        }
    } while (--retry);

    if (!retry) {
        dev_err(&info->client->dev, "%s [ERROR] mms_flash_fw failed\n", __func__);
        ret = -1;
    }

    release_firmware(fw);

    mutex_unlock(&info->lock);

    //Enable IRQ
    //enable_irq(info->client->irq);
    mms_irq_enable(info);

    if(ret < 0) {
        goto ERROR;
    }

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

ERROR:
    dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    return -1;
}
//hyeog added -s
const u8 MELFAS_wind_binary[] = {
#include "melfas_mms400.mfsb.h"
};

extern void mms_fw_update_controller(struct mms_ts_info *info, const struct firmware *fw,
                                     struct i2c_client *client);

struct firmware melfas_fw = {
    .size = sizeof(MELFAS_wind_binary),
    .data = &MELFAS_wind_binary[0],
};

static int melfas_fw_update(struct mms_ts_info *info, struct i2c_client *client) {

    mms_fw_update_controller(info, &melfas_fw, client);
}
//hyeog -e

/**
* Update firmware from external storage
*/
int mms_fw_update_from_storage(struct mms_ts_info *info, bool force) {
    struct file *fp;
    mm_segment_t old_fs;
    unsigned int fw_size, nread;
    int ret = 0;

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    //Disable IRQ
    mutex_lock(&info->lock);
    //disable_irq(info->client->irq);
    mms_irq_disable(info);

    //Get firmware
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    fp = filp_open(EXTERNAL_FW_PATH, O_RDONLY, S_IRUSR);
    if (IS_ERR(fp)) {
        dev_err(&info->client->dev, "%s [ERROR] file_open - path[%s]\n", __func__, EXTERNAL_FW_PATH);
        ret = fw_err_file_open;
        goto ERROR;
    }

    fw_size = fp->f_path.dentry->d_inode->i_size;
    if (0 < fw_size) {
        unsigned char *fw_data;
        fw_data = kzalloc(fw_size, GFP_KERNEL);
        nread = vfs_read(fp, (char __user *)fw_data, fw_size, &fp->f_pos);
        //dev_dbg(&info->client->dev, "%s - path [%s] size [%u]\n", __func__,EXTERNAL_FW_PATH, fw_size);

        if (nread != fw_size) {
            dev_err(&info->client->dev, "%s [ERROR] vfs_read - size[%d] read[%d]\n", __func__, fw_size, nread);
            ret = fw_err_file_read;
        } else {
            //Update fw
            ret = mms_flash_fw(info, fw_data, fw_size, force, true);
        }

        kfree(fw_data);
    } else {
        dev_err(&info->client->dev, "%s [ERROR] fw_size [%d]\n", __func__, fw_size);
        ret = fw_err_file_read;
    }

    filp_close(fp, current->files);

ERROR:
    set_fs(old_fs);

    mutex_unlock(&info->lock);

    //Enable IRQ
    //enable_irq(info->client->irq);
    mms_irq_enable(info);

    if(ret == 0) {
        //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    } else {
        dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    }

    return ret;
}

static ssize_t mms_sys_fw_update(struct device *dev, struct device_attribute *attr, char *buf) {
    struct i2c_client *client = to_i2c_client(dev);
    struct mms_ts_info *info = i2c_get_clientdata(client);
    int result = 0;
    u8 data[255];
    int ret = 0;

    memset(info->print_buf, 0, PAGE_SIZE);

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    ret = mms_fw_update_from_storage(info, true);

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
        sprintf(data, "F/W update failed : File open error [%s]\n", EXTERNAL_FW_PATH);
        break;
    case fw_err_file_read:
        sprintf(data, "F/W update failed : File read error\n");
        break;
    default:
        sprintf(data, "F/W update failed.\n");
        break;
    }

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);

    strcat(info->print_buf, data);
    result = snprintf(buf, PAGE_SIZE, "%s\n", info->print_buf);
    return result;
}
static DEVICE_ATTR(fw_update, 0666, mms_sys_fw_update, NULL);

/**
* Sysfs attr info
*/
static struct attribute *mms_attrs[] = {
    &dev_attr_fw_update.attr,
    NULL,
};

/**
* Sysfs attr group info
*/
static const struct attribute_group mms_attr_group = {
    .attrs = mms_attrs,
};

u8 mms_verify_chip_id(struct mms_ts_info *info) {
    u8 wbuf[8];
    u8 rbuf[64];

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    wbuf[0] = MIP_R0_INFO;
    wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;

    mms_i2c_read(info, wbuf, 2, rbuf, 16);


    mms_get_chip_id(info, rbuf);
    memcpy(info->fw_version, rbuf, 8);

    //	memcpy(info->product_name, rbuf, 16);
    //	dev_info(&info->client->dev, "%s - product_name[%s]\n", __func__, info->product_name);

    printk("[Melfas] %s - product_name[%s] info->fw_version[0] = 0x%x\n", __func__, info->product_name, info->fw_version[0]);

    return TRUE;
    //	if((info->fw_version[0]==0x36) &&(info->fw_version[1]==0x55))
    //		return TRUE;
    //	else
    //		return FALSE;
}

/**
* Initial config
*/
static int mms_init_config(struct mms_ts_info *info) {
    u8 wbuf[8];
    u8 rbuf[64];

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);

    wbuf[0] = MIP_R0_INFO;
    wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;

    mms_i2c_read(info, wbuf, 2, rbuf, 16);


    mms_get_chip_id(info, rbuf);
    memcpy(info->fw_version, rbuf, 8);

    //	memcpy(info->product_name, rbuf, 16);
    //	dev_info(&info->client->dev, "%s - product_name[%s]\n", __func__, info->product_name);

    printk("[Melfas] %s - product_name[%s]\n", __func__, info->product_name);
#if 0
    /***********************/
    //if((strcmp(info->fw_version,"438")) == 0)
    if(mms_verify_chip_id(info)) {
        printk("[darren] mms438 read id success\n");
        tpd_load_status = 1;
    } else {
        printk("[darren] mms438 read id fail\n");
        tpd_load_status = 0;
        return -1;
    }

    /************************/
#endif
    mms_get_fw_version(info, rbuf);
    memcpy(info->fw_version, rbuf, 8);
    dev_info(&info->client->dev, "%s - F/W Version : %02X.%02X %02X.%02X %02X.%02X %02X.%02X\n", __func__, info->fw_version[0], info->fw_version[1], info->fw_version[2], info->fw_version[3], info->fw_version[4], info->fw_version[5], info->fw_version[6], info->fw_version[7]);
    printk("[Melfas] %s - F/W Version : %02X.%02X %02X.%02X %02X.%02X %02X.%02X\n", __func__, info->fw_version[0], info->fw_version[1], info->fw_version[2], info->fw_version[3], info->fw_version[4], info->fw_version[5], info->fw_version[6], info->fw_version[7]);

    //hschoi test 1
    mms_get_chip_id(info, rbuf);
    memcpy(info->fw_version, rbuf, 8);
    dev_info(&info->client->dev, "hschoi %s - F/W Version : %02X.%02X %02X.%02X %02X.%02X %02X.%02X\n", __func__, info->fw_version[0], info->fw_version[1], info->fw_version[2], info->fw_version[3], info->fw_version[4], info->fw_version[5], info->fw_version[6], info->fw_version[7]);
    printk("[Melfas] %s - chip F/W Version : %02X.%02X %02X.%02X %02X.%02X %02X.%02X\n", __func__, info->fw_version[0], info->fw_version[1], info->fw_version[2], info->fw_version[3], info->fw_version[4], info->fw_version[5], info->fw_version[6], info->fw_version[7]);

    wbuf[0] = MIP_R0_INFO;
    wbuf[1] = MIP_R1_INFO_RESOLUTION_X;
    mms_i2c_read(info, wbuf, 2, rbuf, 7);

#if 1
    //Set resolution using chip info
    info->max_x = (rbuf[0]) | (rbuf[1] << 8);
    info->max_y = (rbuf[2]) | (rbuf[3] << 8);
#else
    //Set resolution using platform data
    info->max_x = info->pdata->max_x;
    info->max_y = info->pdata->max_y;
#endif
    dev_info(&info->client->dev, "%s - max_x[%d] max_y[%d]\n", __func__, info->max_x, info->max_y);


    info->node_x = rbuf[4];
    info->node_y = rbuf[5];
    info->node_key = rbuf[6];
    dev_info(&info->client->dev, "%s - node_x[%d] node_y[%d] node_key[%d]\n", __func__, info->node_x, info->node_y, info->node_key);


    if(info->node_key > 0) {
        //Enable touchkey
        info->tkey_enable = true;
    }

    /*
    wbuf[0] = MIP_R0_EVENT;
    wbuf[1] = MIP_R1_EVENT_SUPPORTED_FUNC;
    mms_i2c_read(info, wbuf, 2, rbuf, 7);
    info->event_size = rbuf[6];
    */
    info->event_size = 6;
    dev_info(&info->client->dev, "%s - event_size[%d] \n", __func__, info->event_size);

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
    return 0;

    //ERROR:
    //dev_err(&info->client->dev, "%s [ERROR]\n", __func__);
    //return 1;
}

/**
* Initialize driver
*/
extern struct tpd_device *tpd;
static void mms_resume_later_handle(struct work_struct *work) {
    printk("[Melfas] %s() ?=%p\n", __func__, &mms_info->client->dev);
    mms_resume(&mms_info->client->dev);
}

void mms_resume_enable(void) {
    schedule_work(&mms_resume_work);
}

static int mms_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct mms_ts_info *info;
    struct input_dev *input_dev;
    int ret = 0;

    //dev_dbg(&client->dev, "%s [START]\n", __func__);

    if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "%s [ERROR] i2c_check_functionality\n", __func__);
        ret = -EIO;
        goto ERROR;
    }

    ////dev_dbg(&client->dev, "%s - i2c_check_functionality\n", __func__);

    info = kzalloc(sizeof(struct mms_ts_info), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (!info || !input_dev) {
        dev_err(&client->dev, "%s [ERROR]\n", __func__);
        ret = -ENOMEM;
        goto ERROR;
    }

    info->client = client;
    info->input_dev = input_dev;
    info->irq = -1;
    //info->init = true;
    mutex_init(&info->lock);


    ////dev_dbg(&client->dev, "%s - input_allocate_device\n", __func__);

    //Get platform data
#if MMS_USE_DEVICETREE
    if (client->dev.of_node) {
        info->pdata  = devm_kzalloc(&client->dev, sizeof(struct mms_platform_data), GFP_KERNEL);
        if (!info->pdata) {
            dev_err(&client->dev, "%s [ERROR] pdata devm_kzalloc\n", __func__);
            goto ERROR;
        }

        ret = mms_parse_devicetree(&client->dev, info);
        if (ret) {
            dev_err(&client->dev, "%s [ERROR] mms_parse_dt\n", __func__);
            goto ERROR;
        }
    } else
#endif

    {
        //info->pdata = client->dev.platform_data;
        mms_config_platform_data(client, info);

        if (info->pdata == NULL) {
            dev_err(&client->dev, "%s [ERROR] pdata is null\n", __func__);
            ret = -EINVAL;
            goto ERROR;
        }
    }

    snprintf(info->phys, sizeof(info->phys), "%s/input0", dev_name(&client->dev));

    input_dev->name = "MELFAS_" CHIP_NAME "_Touchscreen";
    input_dev->phys = info->phys;
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &client->dev;
#if MMS_USE_INPUT_OPEN_CLOSE
    input_dev->open = mms_input_open;
    input_dev->close = mms_input_close;
#endif

    //Create device
    input_set_drvdata(input_dev, info);
    i2c_set_clientdata(client, info);

    ret = input_register_device(input_dev);
    if (ret) {
        dev_err(&client->dev, "%s [ERROR] input_register_device\n", __func__);
        ret = -EIO;
        goto ERROR;
    }

    //dev_dbg(&client->dev, "%s - input_register_device\n", __func__);


    //Config GPIO
    mms_config_gpio(info);

    // power up sequence
#ifdef __USE_LINUX_REGULATOR_FRAMEWORK__
    tpd->reg = regulator_get(tpd->tpd_dev, TPD_POWER_SOURCE_CUSTOM);
    if (IS_ERR(tpd->reg))
        return PTR_ERR(tpd->reg);
    ret = regulator_set_voltage(tpd->reg, 2800000, 2800000);
    ret += regulator_enable(tpd->reg);
    if (ret)
        printk("Power on ic error!\n");
#else
    hwPowerOn(TPD_POWER_SOURCE, VOL_2800, "TP");
#endif

    msleep(200);

    //Power on
    //	mms_power_on(info);
    mms_reboot(info);

    //Set I2C DMA
    if(i2c_dma_buf_va == NULL) {
        i2c_dma_buf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &i2c_dma_buf_pa, GFP_KERNEL);
        //i2c_dma_buf_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 4096, &i2c_dma_buf_pa, GFP_KERNEL);
        //	gpDMABuf_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
        //i2c_dma_buf_va = dma_alloc_coherent(NULL, 4096, &i2c_dma_buf_pa, GFP_KERNEL);
    }
    if(!i2c_dma_buf_va) {
        dev_err(&client->dev, "%s [ERROR] dma_alloc_coherent\n", __func__);
    }
    //dev_dbg(&client->dev, "%s - dma_alloc_coherent : pa[0x%08X]\n", __func__, i2c_dma_buf_pa);

    if(mms_verify_chip_id(info)) {
        printk("[darren] mms438 read id success\n");
        tpd_load_status = 1;
    } else {
        printk("[darren] mms438 read id fail\n");
        tpd_load_status = 0;
        goto ERROR;
    }


    //Firmware update
#if MMS_USE_AUTO_FW_UPDATE
    /*
    info->fw_name = kstrdup(INTERNAL_FW_PATH, GFP_KERNEL);
    ret = request_firmware_nowait(THIS_MODULE, true, fw_name, &client->dev, GFP_KERNEL, info, mms_fw_update_boot);
    if (ret) {
    	dev_err(&client->dev, "%s [ERROR] request_firmware_nowait\n", __func__);
    	ret = -EIO;
    	//goto ERROR;
    }
    */
    //	ret = mms_fw_update_from_kernel(info);
    melfas_fw_update(info, client);
    if(ret) {
        dev_err(&client->dev, "%s [ERROR] mms_fw_update_from_kernel\n", __func__);
        printk("%s [ERROR] mms_fw_update_from_kernel\n", __func__);
    }

#endif

    //Initial config
    mms_init_config(info);

    //Config input interface
    mms_config_input(info);


#if MMS_USE_CALLBACK
    //Config callback functions
    mms_config_callback(info);
#endif

    mms_info = info;

    //Enable device

    mms_enable(info);
    //	mms_reboot(info);
    //Set interrupt handler
    /*
    ret = request_threaded_irq(client->irq, NULL, mms_interrupt, IRQF_TRIGGER_LOW | IRQF_ONESHOT, MMS_DEVICE_NAME, info);
    //ret = request_threaded_irq(client->irq, NULL, mms_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, MMS_DEVICE_NAME, info);
    if (ret) {
    	dev_err(&client->dev, "%s [ERROR] request_threaded_irq\n", __func__);
    	goto ERROR;
    }

    //disable_irq(client->irq);
    mms_irq_disable(info);
    info->irq = client->irq;
    */
    mms_interrupt_start(info);

#if MMS_USE_NAP_MODE
    //Wake lock for nap mode
    wake_lock_init(&mms_wake_lock, WAKE_LOCK_SUSPEND, "mms_wake_lock");
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
    //Config early suspend
    //info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +1;
    //info->early_suspend.level =	EARLY_SUSPEND_LEVEL_DISABLE_FB-1;
    //info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN -1;
    info->early_suspend.level =	EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
    info->early_suspend.suspend = mms_early_suspend;
    info->early_suspend.resume = mms_late_resume_dummy;

    register_early_suspend(&info->early_suspend);

    //dev_dbg(&client->dev, "%s - register_early_suspend\n", __func__);
#endif

    INIT_WORK(&mms_resume_work, mms_resume_later_handle);

    //	tpd_load_status = 1;

#if MMS_USE_DEV_MODE
    //Create dev node (optional)
    if(mms_dev_create(info)) {
        dev_err(&client->dev, "%s [ERROR] mms_dev_create\n", __func__);
        ret = -EAGAIN;
        goto ERROR;
    }

    //Create dev
    info->class = class_create(THIS_MODULE, MMS_DEVICE_NAME);
    device_create(info->class, NULL, info->mms_dev, NULL, MMS_DEVICE_NAME);
#endif

#if MMS_USE_TEST_MODE
    //Create sysfs for test mode (optional)
    if (mms_sysfs_create(info)) {
        dev_err(&client->dev, "%s [ERROR] mms_sysfs_create\n", __func__);
        ret = -EAGAIN;
        goto ERROR;
    }
#endif

#if MMS_USE_CMD_MODE
    //Create sysfs for command mode (optional)
    if (mms_sysfs_cmd_create(info)) {
        dev_err(&client->dev, "%s [ERROR] mms_sysfs_cmd_create\n", __func__);
        ret = -EAGAIN;
        goto ERROR;
    }
#endif

    //Create sysfs
    if (sysfs_create_group(&client->dev.kobj, &mms_attr_group)) {
        dev_err(&client->dev, "%s [ERROR] sysfs_create_group\n", __func__);
        ret = -EAGAIN;
        goto ERROR;
    }

    if (sysfs_create_link(NULL, &client->dev.kobj, MMS_DEVICE_NAME)) {
        dev_err(&client->dev, "%s [ERROR] sysfs_create_link\n", __func__);
        ret = -EAGAIN;
        goto ERROR;
    }
    mx_tsp_device_create();
    //dev_dbg(&client->dev, "%s [DONE]\n", __func__);
    dev_info(&client->dev, "MELFAS " CHIP_NAME " Touchscreen is initialized successfully.\n");
    return 0;

ERROR:

#ifdef __USE_LINUX_REGULATOR_FRAMEWORK__
    regulator_disable(tpd->reg);
    msleep(20);
    regulator_put(tpd->reg);
#endif

    //dev_dbg(&client->dev, "%s [ERROR]\n", __func__);
    dev_err(&client->dev, "MELFAS " CHIP_NAME " Touchscreen initialization failed.\n");
    return ret;
}

/**
* Remove driver
*/
static int mms_remove(struct i2c_client *client) {
    struct mms_ts_info *info = i2c_get_clientdata(client);

    //dev_dbg(&client->dev, "%s [START]\n", __func__);

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

#if MMS_USE_CMD_MODE
    mms_sysfs_cmd_remove(info);
#endif

#if MMS_USE_TEST_MODE
    mms_sysfs_remove(info);
#endif

    sysfs_remove_group(&info->client->dev.kobj, &mms_attr_group);
    sysfs_remove_link(NULL, MMS_DEVICE_NAME);
    kfree(info->print_buf);

#if MMS_USE_DEV_MODE
    device_destroy(info->class, info->mms_dev);
    class_destroy(info->class);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
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


int mms_suspend(struct device *dev) {
    struct i2c_client *client = to_i2c_client(dev);
    struct mms_ts_info *info = i2c_get_clientdata(client);
    int ret = 0;

    printk("[Melfas] %s()\n", __func__);


#if MMS_USE_GESTURE_WAKEUP_MODE
    //mutex_lock(&info->lock);

    if(mms_wakeup_control_flag == 1) {
        /* // modify by melfas
            	      msleep(100);
                     mms_reset(info);
             	      msleep(100);
        */
        ret  = mms_enable_gesture_wakeup_mode(info);
        if(ret == 0) {
            //dev_err(&client->dev," ahe gesture close OK!!\n");
            printk("[HST] gesture open OK!!\n");
        } else {
            //dev_err(&client->dev," ahe gesture close fail!!\n");
            printk(" [HST] gesture open fail!!\n");
        }
    }

    if(mms_wakeup_control_flag == 0) {
        mms_disable(info);
    }
#else
    mms_disable(info);
#endif
    mms_clear_input(info);

    //dev_dbg(&client->dev, "%s [DONE]\n", __func__);

    return 0;

}


/**
* Device resume event handler
*/
int mms_resume(struct device *dev) {

    struct i2c_client *client = to_i2c_client(dev);
    struct mms_ts_info *info = i2c_get_clientdata(client);
    int ret = 0;
    printk("[HST] %s()\n", __func__);

    ret = mms_enable(info);
    if(ret == 0) {
        printk(" [HST] reset OK!!!!\n");
    } else {
        printk(" [HST] reset fail!!\n");
    }



#if MMS_USE_NAP_MODE
    if (wake_lock_active(&mms_wake_lock)) {
        wake_unlock(&mms_wake_lock);
        //dev_dbg(&client->dev, "%s - wake_unlock\n", __func__);
    }

    info->nap_mode = false;
    //dev_dbg(&client->dev, "%s - nap mode : off\n", __func__);
#endif

    //dev_dbg(&client->dev, "%s [DONE]\n", __func__);

    return ret;
}

int mms_resume_dummy(struct device *dev) {
    printk("[Melfas] %s()\n", __func__);
}

#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
/**
* Early suspend handler
*/

static void mms_early_suspend(struct early_suspend *h) {
    //struct mms_ts_info *info = container_of(h, struct mms_ts_info, early_suspend);
    struct mms_ts_info *info = mms_info;

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    printk("[HST] %s()\n", __func__);
    mms_suspend(&info->client->dev);

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}


/**
* Late resume handler
*/
static void mms_late_resume(struct early_suspend *h) {
    //struct mms_ts_info *info = container_of(h, struct mms_ts_info, early_suspend);
    struct mms_ts_info *info = mms_info;

    //dev_dbg(&info->client->dev, "%s [START]\n", __func__);
    printk("[HST] %s()\n", __func__);
    mms_resume(&info->client->dev);

    //dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

static void mms_late_resume_dummy(struct early_suspend *h) {
#if MMS_USE_GESTURE_WAKEUP_MODE
    printk("[HST] %s()\n", __func__);
#else
    struct mms_ts_info *info = mms_info;

    printk("[HST] %s()\n", __func__);
    mms_resume(&info->client->dev);
#endif
}
#endif

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
/**
* PM info
*/
const struct dev_pm_ops mms_pm_ops = {
#if 0
    SET_SYSTEM_SLEEP_PM_OPS(mms_suspend, mms_resume)
#else
    .suspend = mms_suspend,
    .resume = mms_resume_dummy,
#endif
};
#endif

#if MMS_USE_DEVICETREE
/**
* Device tree match table
*/
static const struct of_device_id mms_match_table[] = {
    { .compatible = "melfas,"MMS_DEVICE_NAME,},
    {},
};
MODULE_DEVICE_TABLE(of, mms_match_table);
#endif

/**
* I2C Device ID
*/
static const struct i2c_device_id mms_id[] = {
    {MMS_DEVICE_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, mms_id);

static int mms_detect (struct i2c_client *client, struct i2c_board_info *info) {
    strcpy(info->type, TPD_DEVICE);
    return 0;
}

/**
* I2C driver info
*/
static struct i2c_driver mms_driver = {
    .id_table	= mms_id,
    .probe = mms_probe,
    .remove = mms_remove,
    .detect = mms_detect,
#if 0 // !defined(CONFIG_HAS_EARLYSUSPEND) 
    .suspend = mms_early_suspend,
    .resume = mms_late_resume_dummy,
#endif

    .driver = {
        .name = MMS_DEVICE_NAME,
        //.owner = THIS_MODULE,
#if MMS_USE_DEVICETREE
        //.of_match_table = mms_match_table,
#endif
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
        //.pm = &mms_pm_ops,
#endif
    },
};

#if 0
/**
* Init driver
*/
static int __init mms_init(void) {
    return i2c_add_driver(&mms_driver);
}

/**
* Exit driver
*/
static void __exit mms_exit(void) {
    i2c_del_driver(&mms_driver);
}

module_init(mms_init);
module_exit(mms_exit);
#endif

void mms_tpd_interrupt_handler(void) {
    MMS_DEBUG("%s [START]\n", __func__);

    mms_eint_count++;
    MMS_DEBUG("%s - eint_count[%d]\n", __func__, mms_eint_count);

    mms_eint_flag = 1;
    wake_up_interruptible(&waiter);

    MMS_DEBUG("%s [DONE]\n", __func__);
}

/**
* tpd local init
*/
static int mms_tpd_local_init(void) {
    MMS_DEBUG("%s [START]\n", __func__);

    if (tpd_load_status == 1) {
        printk("mms === add touch panel driver already.\n");
        return -1;
    }

    if(i2c_add_driver(&mms_driver) != 0) {
        MMS_DEBUG("%s [ERROR] i2c_add_driver\n", __func__);
        goto ERROR;
    }

    if (tpd_load_status == 0) {
        printk("add error mms touch panel driver.\n");
        return -1;
    }

#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, mms_tpd_keys_local, mms_tpd_keys_dim_local);
#endif

    tpd_type_cap = 1;

    MMS_DEBUG("%s [DONE]\n", __func__);
    return 0;

ERROR:
    MMS_DEBUG("%s [ERROR]\n", __func__);
    return -1;
}

/**
* tpd driver info
*/
static struct tpd_driver_t mms_tpd_driver = {
    .tpd_device_name = MMS_DEVICE_NAME,
    .tpd_local_init = mms_tpd_local_init,

#if 0 // !defined(CONFIG_HAS_EARLYSUSPEND) 
    .suspend = mms_early_suspend,
    .resume = mms_late_resume,
#endif

#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};

/**
* I2C board info
*/
static struct i2c_board_info __initdata mms_board_info = {
    I2C_BOARD_INFO(MMS_DEVICE_NAME, TPD_I2C_ADDR),
};

/**
* Init driver
*/
static int __init mms_tpd_driver_init(void) {
    MMS_DEBUG("%s [START]\n", __func__);

    i2c_register_board_info(TPD_I2C_BUS, &mms_board_info, 1);

    if(tpd_driver_add(&mms_tpd_driver) < 0) {
        MMS_DEBUG("%s [ERROR] tpd_driver_add\n", __func__);
        printk("[HST] %s [ERROR] tpd_driver_add\n", __func__);
    }

    MMS_DEBUG("%s [DONE]\n", __func__);

    return 0;
}

/**
* Exit driver
*/
static void __exit mms_tpd_driver_exit(void) {
    MMS_DEBUG("%s [START]\n", __func__);

    //input_unregister_device(tpd->dev);
    if(tpd_driver_remove(&mms_tpd_driver)) {
        MMS_DEBUG("%s [ERROR] tpd_driver_remove\n", __func__);
    }

    MMS_DEBUG("%s [DONE]\n", __func__);
}

module_init(mms_tpd_driver_init);
module_exit(mms_tpd_driver_exit);

MODULE_DESCRIPTION("MELFAS MMS400 Touchscreen for MediaTek");
MODULE_VERSION("2014.12.16");
MODULE_AUTHOR("Jee SangWon <jeesw@melfas.com>");
MODULE_LICENSE("GPL");

