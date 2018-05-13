#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/printk.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <mach/eint.h>
#include <mach/mt_gpio.h>
#include <cust_eint.h>
#include "hall.h"

#define CUST_EINT_POLARITY_LOW 0
#define COVER_WINDOW

static u8 hall_state = !CUST_EINT_POLARITY_LOW;
static struct input_dev *hall_input = NULL;
static struct hall_data *hall = NULL;
static struct platform_device *hall_pdev = NULL;
extern unsigned int bl_brightness_hal;
extern u16 g_hall_status;

void (*hall_state_charge_notify)(int state) = NULL;
EXPORT_SYMBOL(hall_state_charge_notify);

#ifndef COVER_WINDOW
static void do_hall_work(struct work_struct *work)
{
    u8 old_state = hall_state;
    bool pressed;

    mt_eint_mask(CUST_EINT_MHALL_NUM);
    hall_state = !hall_state;
    pressed = (hall_state == !!CUST_EINT_POLARITY_LOW);

    if ((pressed == 1) && (bl_brightness_hal != 0)) {
        input_report_key(hall_input, KEY_POWER, pressed);
        input_sync(hall_input);
        input_report_key(hall_input, KEY_POWER, !pressed);
        input_sync(hall_input);
    }

    if ((pressed == 0) && (bl_brightness_hal == 0)) {
        input_report_key(hall_input, KEY_POWER, !pressed);
        input_sync(hall_input);
        input_report_key(hall_input, KEY_POWER, pressed);
        input_sync(hall_input);
    }

    g_hall_status = pressed;

    if(hall_state_charge_notify != NULL) {
        hall_state_charge_notify(g_hall_status);
    }

    mt_eint_set_polarity(CUST_EINT_MHALL_NUM, old_state);
    mt_eint_unmask(CUST_EINT_MHALL_NUM);
}
#else
static void do_hall_work(struct work_struct *work)
{
    u8 old_state = hall_state;
    unsigned int key_code;
    u8 is_covered = mt_get_gpio_in(GPIO_MHALL_EINT_PIN);

    printk(KERN_INFO"Detect hall status = %d\n", is_covered);
    mt_eint_mask(CUST_EINT_MHALL_NUM);
    hall_state = !hall_state;
    key_code = (is_covered ? KEY_UNCOVER : KEY_COVER);

    input_report_key(hall_input, key_code, 1);
    input_sync(hall_input);
    input_report_key(hall_input, key_code, 0);
    input_sync(hall_input);

    g_hall_status = is_covered;

    if(hall_state_charge_notify != NULL) {
        hall_state_charge_notify(g_hall_status);
    }

    mt_eint_set_polarity(CUST_EINT_MHALL_NUM, old_state);
    mt_eint_unmask(CUST_EINT_MHALL_NUM);
}
#endif

static void interrupt_hall_irq(void)
{
    queue_work(hall->hall_wq, &hall->hall_work);
}

static int hall_probe(struct platform_device *pdev)
{
    int rc = 0;
    hall = kmalloc(sizeof(struct hall_data), GFP_KERNEL);

    if (hall == NULL) {
        dev_err(&pdev->dev, "Unable to allocate memory\n");
        return -ENOMEM;
    }

    hall_input = input_allocate_device();

    if (hall_input == NULL) {
        dev_err(&pdev->dev, "Not enough memory!\n");
        rc = -ENOMEM;
        goto err_malloc_input_device;
    }

    hall_input->name = "hall";
#ifndef COVER_WINDOW
    input_set_capability(hall_input, EV_KEY, KEY_POWER);
#else
    input_set_capability(hall_input, EV_KEY, KEY_COVER);
    input_set_capability(hall_input, EV_KEY, KEY_UNCOVER);
#endif
    rc = input_register_device(hall_input);

    if (rc) {
        dev_err(&pdev->dev, "Failed to register hall input device!\n");
        goto err_register_input_device;
    }

    hall->hall_wq = create_singlethread_workqueue("hall_wq");

    if (hall->hall_wq == NULL) {
        dev_err(&pdev->dev, "Create hall workqueue failed!\n");
        rc = -EFAULT;
        goto err_create_workqueue;
    }

    INIT_WORK(&hall->hall_work, do_hall_work);
    mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
    //	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_ENABLE);
    //	mt_set_gpio_pull_select(GPIO_MHALL_EINT_PIN, GPIO_PULL_UP);
    mt_eint_registration(CUST_EINT_MHALL_NUM, EINTF_TRIGGER_LOW, interrupt_hall_irq, 0);
    mt_eint_unmask(CUST_EINT_MHALL_NUM);
    dev_info(&pdev->dev, "Hall Sensor Driver Probe Success!\n");
    return 0;
err_create_workqueue:
    input_unregister_device(hall_input);
err_register_input_device:
    input_free_device(hall_input);
err_malloc_input_device:
    kfree(hall);
    return rc;
}

static int hall_remove(struct platform_device *pdev)
{
    if (hall->hall_wq)
        destroy_workqueue(hall->hall_wq);
    if (hall_input) {
        input_unregister_device(hall_input);
        input_free_device(hall_input);
        hall_input = NULL;
    }
    kfree(hall);

    return 0;
}

static struct platform_driver hall_driver = {
    .probe = hall_probe,
    .remove = hall_remove,
    .driver = {
        .name = "hall_sensor",
        .owner = THIS_MODULE,
    }
};

static int __init hall_init(void)
{
    int ret = platform_driver_register(&hall_driver);

    if (ret)
        return ret;

    hall_pdev = platform_device_register_simple("hall_sensor", -1, NULL, 0);

    if (IS_ERR(hall_pdev)) {
        platform_driver_unregister(&hall_driver);
        return PTR_ERR(hall_pdev);
    }

    pr_info("Hall Sensor Driver Initializ Success.\n");
    return 0;
}

static void __init hall_exit(void)
{
    platform_device_unregister(hall_pdev);
    platform_driver_unregister(&hall_driver);
}

late_initcall(hall_init);
module_exit(hall_exit);
MODULE_LICENSE("GPL");

