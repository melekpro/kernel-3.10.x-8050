/*
psensor driver
*/
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/switch.h>
#include <linux/seq_file.h>

#include <mach/upmu_common.h>
#include <mach/mt_gpio.h>
#include <linux/proc_fs.h>
#include "psensor.h"

struct switch_dev *psensor_dev = NULL;

extern struct psensor_driver_t iqs128_driver;

struct psensor_driver_t *psensor_driver_list[] =
{
    &iqs128_driver,
};

static void psensor_create_attributes(struct device *dev,
                                      struct psensor_attrs *attrs)
{
    int num = attrs->num;

    for (; num > 0;)
        device_create_file(dev, attrs->attr[--num]);
}


int psensor_driver_num = sizeof(psensor_driver_list) / sizeof(int);

void send_psensor_uevent(enum PSENSOR_STATUS val)
{
    char *envp[2];
    char psensor[20];

    if(psensor_dev == NULL)
        return;
    psensor_dev->state = val;
    snprintf(psensor, sizeof(psensor), "SWITCH_STATE=%d", psensor_dev->state);
    envp[0] = psensor;
    envp[1] = NULL;

    if(!psensor_dev->state)
        kobject_uevent_env(&psensor_dev->dev->kobj, KOBJ_ADD, envp);
    else
        kobject_uevent_env(&psensor_dev->dev->kobj, KOBJ_REMOVE, envp);
}

/* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.3.20*/
struct proc_dir_entry *proc_psensor_dir = NULL;
static struct proc_dir_entry *proc_name = NULL;
u8 psensor_power_on  = 0;

ssize_t psensor_power_en_store(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{
    if (!strncmp(buf, "on", 2))
    {
        pmic_set_register_value(PMIC_RG_VIBR_VOSEL, 0x7); //modify VIBR TO 3.3V
        pmic_set_register_value(PMIC_RG_VIBR_EN, 1);

#if PSENSOR_EN_GPIO_MODE
        mt_set_gpio_out(IQS128_EN_PIN, IQS128_EN_PIN_ENABLE);
#endif

        msleep(1);
        mt_set_gpio_pull_enable(IQS128_EINT_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(IQS128_EINT_PIN, GPIO_PULL_UP);
        psensor_power_on = 1;
        printk(KERN_CRIT"%s: Psensor power on\n", __func__);
    }
    else if (!strncmp(buf, "off", 3))
    {
        mt_set_gpio_pull_enable(IQS128_EINT_PIN, GPIO_PULL_DISABLE);
        msleep(1);
        pmic_set_register_value(PMIC_RG_VIBR_EN, 1);

#if PSENSOR_EN_GPIO_MODE
        mt_set_gpio_out(IQS128_EN_PIN, IQS128_EN_PIN_DISABLE);
#endif

        psensor_power_on = 0;
        printk(KERN_CRIT"%s: Psensor power off\n", __func__);
    }

    return size;
}

static int psensor_power_en_show(struct seq_file *m, void *v)
{
    if (psensor_power_on == 1)
    {
        seq_printf(m, "on\n");
    }
    else if (psensor_power_on == 0)
    {
        seq_printf(m, "off\n");
    }
    return 0;
}

static int psensor_power_en_open(struct inode *inode, struct file *file)
{
    return single_open(file, psensor_power_en_show, NULL);
}

//static DEVICE_ATTR(power_en, S_IRUGO | S_IWUSR | S_IWGRP, psensor_power_en_show, psensor_power_en_store);
static const struct file_operations psensor_power_en_fops =
{
    .open = psensor_power_en_open,
    .write = psensor_power_en_store,
    .read = seq_read,
    .owner = THIS_MODULE,
};
/* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.3.20*/
static int __init psensor_init(void)
{
    int ret = 0, i = 0;

    printk("<2>""--------------enter %s PSENSOR_EN_GPIO_MODE %d -----------------\n", __func__, PSENSOR_EN_GPIO_MODE);

#if PSENSOR_EN_GPIO_MODE
    mt_set_gpio_mode(IQS128_EN_PIN, IQS128_EN_PIN_M);
    mt_set_gpio_dir(IQS128_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(IQS128_EN_PIN, IQS128_EN_PIN_DISABLE);
#endif

    psensor_dev = kzalloc(sizeof(struct switch_dev), GFP_KERNEL);
    if (psensor_dev == NULL)
    {
        printk("!!!!!!!!psensor dev alloc fail\n");
        return -1;
    }

    psensor_dev->name = "psensor";
    psensor_dev->state = PSENSOR_STATUS_FAR;

    ret = switch_dev_register(psensor_dev);
    if (ret < 0)
    {
        printk("register switch dev fail,ret=%d\n", ret);
        return -2;
    }

    send_psensor_uevent(PSENSOR_STATUS_FAR);

    for(i = 0; i < psensor_driver_num; i++)
    {
        if(psensor_driver_list[i] && psensor_driver_list[i]->init)
        {
            ret = psensor_driver_list[i]->init();
            printk("<2>""psensor %s loaded,ret=%d\n", psensor_driver_list[i]->name, ret);
        }
        if(psensor_driver_list[i]->attrs != NULL)
        {
            if (psensor_driver_list[i]->attrs->num)
                psensor_create_attributes(psensor_dev->dev, psensor_driver_list[i]->attrs);
        }
    }

    /* [PLATFORM]-Add-BEGIN by TCTSZ.lizhi.wu, 2015.3.20*/
    proc_psensor_dir = proc_mkdir("psensor", NULL);
    if (proc_psensor_dir == NULL)
    {
        printk(KERN_CRIT"create psensor file failed!\n");
    }
    proc_name = proc_create("power_en", 0666, proc_psensor_dir, &psensor_power_en_fops);
    if (proc_name == NULL)
    {
        printk(KERN_CRIT"create power_en file failed!\n");
    }

    /* [PLATFORM]-Add-End by TCTSZ.lizhi.wu, 2015.3.20*/
    printk("<2>""-------------- %s finish-----------------\n", __func__);
    return 0;
}

static void __exit psensor_exit(void)
{
    printk("%s[%d]: %s\n", __FILE__, __LINE__, __FUNCTION__);
}

late_initcall(psensor_init);
module_exit(psensor_exit);
MODULE_LICENSE("GPL");
