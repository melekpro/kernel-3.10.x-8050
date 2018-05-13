#include <cust_eint.h>
#include <mach/mt_gpio.h>
#include <linux/sched.h>
#include <linux/rtpm_prio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <mach/mt_pm_ldo.h>
#include <cust_gpio_usage.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <mach/upmu_common.h>
#include <mach/eint.h>
#include <linux/switch.h>
#include "psensor.h"

struct work_struct *iqs128_work;
static struct workqueue_struct *iqs128_wq;
//[BUGFIX]-Add-BEGIN by TCTSZ.weihong.chen, 2014/05/26, it should be zero in case of  there is no psensor on board
static int iqs128_int_value = 0;
//[BUGFIX-Add-END by TCTSZ.weihong.chen
static void do_iqs128_work(struct work_struct *work)
{
    send_psensor_uevent(iqs128_int_value);
}

static void iqs128_int_handler(void)
{
    //	printk("<2>""---%s,%d\n",__func__,__LINE__);
    //[BUGFIX]-Add-BEGIN by TCTSZ.weihong.chen, 2014/05/26, faraway:0  closed:1
    iqs128_int_value = mt_get_gpio_in(IQS128_EINT_PIN);
    //[BUGFIX-Add-END by TCTSZ.weihong.chen
    queue_work(iqs128_wq, iqs128_work);
    //[BUGFIX]-Add-BEGIN by TCTSZ.weihong.chen, 2014/05/26, trigger condition should be depended on current gpio state
    mt_eint_set_polarity(IQS128_EINT_NUM,
                         mt_get_gpio_in(IQS128_EINT_PIN) ? MT_EINT_POL_NEG : MT_EINT_POL_POS);
    //[BUGFIX-Add-END by TCTSZ.weihong.chen
}
void iqs128_reset(void)
{
    mt_eint_mask(IQS128_EINT_NUM);
    pmic_set_register_value(PMIC_RG_VIBR_EN, 0);
    msleep(100);
    pmic_set_register_value(PMIC_RG_VIBR_VOSEL, 0x7); //modify VIBR TO 3.3V
    pmic_set_register_value(PMIC_RG_VIBR_EN, 1);
    mt_eint_unmask(IQS128_EINT_NUM);
}
static ssize_t psensor_reset_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;
    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;
    input = input > 0 ? 1 : 0;
    if(input)
        iqs128_reset();
    return count;
}

static ssize_t psensor_reset_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "iqs128 int value = %d\n",
                    mt_get_gpio_in(IQS128_EINT_PIN));
}

static DEVICE_ATTR(reset, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
                   psensor_reset_show, psensor_reset_store);

static struct device_attribute *psensor_attrs[] =
{
    &dev_attr_reset,
};

static  struct psensor_attrs iqs128_attrs =
{
    .attr = psensor_attrs,
    .num = sizeof( psensor_attrs) / sizeof(psensor_attrs[0]),
};

static int iqs128_init(void)
{
    printk("<2>""---enter %s----\n", __func__);
    //upmu_set_rg_vibr_vosel(0x7); //modify VIBR TO 3.3V			/* [PLATFORM]-Removed by TCTSZ.lizhi.wu, 2015.3.20*/
    //upmu_set_rg_vibr_en(1);

    mt_set_gpio_mode(IQS128_EINT_PIN, IQS128_EINT_PIN_M_EINT);
    mt_set_gpio_dir(IQS128_EINT_PIN, GPIO_DIR_IN);
    //	mt_set_gpio_pull_enable(IQS128_EINT_PIN, GPIO_PULL_ENABLE);		/* [PLATFORM]-Removed by TCTSZ.lizhi.wu, 2015.3.20*/
    //	mt_set_gpio_pull_select(IQS128_EINT_PIN, GPIO_PULL_UP);
    msleep(50);

    iqs128_work = kzalloc(sizeof(typeof(*iqs128_work)), GFP_KERNEL);
    if (!iqs128_work)
    {
        printk("create work queue error, line: %d\n", __LINE__);
        return -1;
    }

    INIT_WORK(iqs128_work, do_iqs128_work);

    iqs128_wq = create_singlethread_workqueue("iqs128_wq");
    if (!iqs128_wq)
    {
        kfree(iqs128_work);
        printk("create thread error, line: %d\n", __LINE__);
        return -1;
    }
    //[BUGFIX]-Add-BEGIN by TCTSZ.weihong.chen, 2014/05/26, do once to initial switch_state as '0'
    queue_work(iqs128_wq, iqs128_work);//do one time to initial
    //[BUGFIX-Add-END by TCTSZ.weihong.chen
    mt_eint_set_hw_debounce(IQS128_EINT_NUM, IQS128_EINT_DEBOUNCE_CN );
    mt_eint_registration(IQS128_EINT_NUM, mt_get_gpio_in(IQS128_EINT_PIN) ? EINTF_TRIGGER_FALLING : EINTF_TRIGGER_RISING, iqs128_int_handler, 1);

    mt_eint_unmask(IQS128_EINT_NUM);
    msleep(50);
    return 0;
}

struct psensor_driver_t iqs128_driver =
{
    .name = "iqs128",
    .init = iqs128_init,
    .attrs = &iqs128_attrs,
};
