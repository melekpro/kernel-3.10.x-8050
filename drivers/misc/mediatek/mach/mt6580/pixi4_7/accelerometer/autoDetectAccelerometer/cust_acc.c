#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>

/*
cust_acc_hw1 this config is for gsensor lis2de12
*/
static struct acc_hw cust_acc_hw1 = {
    .i2c_num = 2,
    .direction = 4,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
    .is_batch_supported = false,
};

/*
cust_acc_hw2 this config is for gsensor mxc4005
*/

static struct acc_hw cust_acc_hw2 = {
    .i2c_num = 2,
    .direction = 6,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
    .is_batch_supported = false,
};

struct acc_hw* get_cust_acc_hw(void) 
{
    return &cust_acc_hw1;
}

struct acc_hw* get_cust_acc_hw_sub(int id) 
{
    if(id == 1) {
        return &cust_acc_hw1;
    } else if(id == 2){
        return &cust_acc_hw2;
    }
    return NULL;
}