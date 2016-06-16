#include <linux/module.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/leds.h>

#include <linux/platform_device.h>

#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

static volatile int hall_key_debug = 1;

struct hall_switch_data{
    struct input_dev *input_dev;
    struct delayed_work hall_work;
    struct workqueue_struct *hall_workqueue;
    int hall_irq;
    int hall_gpio_val;
    int intr_pin;
    int irq;
};
static int hall_gpio;
static irqreturn_t misc_hall_irq(int irq, void *data)
{
    struct hall_switch_data *hall_data = data;
    int gpio_value;

    if(hall_data == NULL)
        return 0;
	disable_irq_nosync(hall_data->hall_irq);
	gpio_value = gpio_get_value(hall_data->intr_pin);
    if(gpio_value){
        /*----hall far----*/
        if(hall_key_debug)
        printk("hall-switch %d,report:____________________________far!!!\n",hall_data->intr_pin);
        input_event(hall_data->input_dev, EV_KEY, KEY_SHOP, 1);
        input_sync(hall_data->input_dev);
        input_event(hall_data->input_dev, EV_KEY, KEY_SHOP, 0);
        input_sync(hall_data->input_dev);
    }else{
        /*----hall near----*/
        if(hall_key_debug)
        printk("hall-switch %d,report:_____________________________near!!!\n",hall_data->intr_pin);
        input_event(hall_data->input_dev, EV_KEY, KEY_SPORT, 1);
        input_sync(hall_data->input_dev);
        input_event(hall_data->input_dev, EV_KEY, KEY_SPORT, 0);
        input_sync(hall_data->input_dev);
    }
	enable_irq(hall_data->hall_irq);
    return IRQ_HANDLED;
}

static ssize_t hall_gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        int tmp = gpio_get_value(hall_gpio);
        return sprintf(buf, "%s\n", tmp==0?"0":"1");
}

static DEVICE_ATTR(hall_int_gpio, 0444, hall_gpio_show, NULL);
static struct attribute *hall_attributes[] = {
          &dev_attr_hall_int_gpio.attr,
         NULL,
};

static struct attribute_group hall_attr_group = {
        .attrs = hall_attributes,
};



static int hall_probe(struct platform_device *pdev)
{
        int retval = 0;

        int err = 0;
        struct hall_switch_data *hall_data;
        unsigned int irq_gpio_flags;
        struct device_node *np = pdev->dev.of_node;
        printk("%s\n", __func__);
        hall_data = kzalloc(sizeof(struct hall_switch_data), GFP_KERNEL);
        if (!hall_data){
            err = -ENOMEM;
            goto exit;
        }
   /*----Register to Input Device----*/
        hall_data->input_dev = input_allocate_device();
        if (hall_data->input_dev == NULL){
            err = -ENOMEM;
            printk("hall-switch: ____________________________Failed to allocate input device!!! \n");
            goto exit_kfree;
        }
        hall_data->input_dev->name = "hall-switch";
        set_bit(EV_SYN, hall_data->input_dev->evbit);
        set_bit(EV_KEY, hall_data->input_dev->evbit);
        set_bit(EV_ABS, hall_data->input_dev->evbit);
        set_bit(KEY_SPORT, hall_data->input_dev->keybit);
        input_set_capability(hall_data->input_dev, EV_KEY, KEY_SPORT);
        set_bit(KEY_SHOP, hall_data->input_dev->keybit);
        input_set_capability(hall_data->input_dev, EV_KEY, KEY_SHOP);
        retval = input_register_device(hall_data->input_dev);
        if(retval) {
            printk("hall-switch:____________________________Failed to register input device!!!\n");
            goto exit_register_input;
        }

       hall_data->intr_pin = of_get_named_gpio_flags(np, "hall,irq-gpio",
                                0, &irq_gpio_flags);
        if (hall_data->intr_pin < 0) {
         printk("%s,intr_pin failed\n",__func__);
         gpio_free(hall_data->intr_pin);
        }

        if (gpio_is_valid(hall_data->intr_pin)) {
                retval = gpio_request(hall_data->intr_pin, "hall_switch");
                if (retval) {
                        printk( "%s,irq gpio request failed",__func__);
                }

                retval = gpio_direction_input(hall_data->intr_pin);
                if (retval) {
                        printk("%s,set_direction for irq gpio failed\n",__func__);
                }
        }
	hall_gpio = hall_data->intr_pin;
    hall_data->hall_irq = gpio_to_irq(hall_data->intr_pin);    
    retval = request_irq(hall_data->hall_irq, misc_hall_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                      "misc_hall_irq", hall_data);
    if(retval <0)
    {
        printk("%s,request irq pin %d fail for gpio\n",__func__,retval);
        goto fail_free_intr_pin;
    }
    irq_set_irq_wake(hall_data->irq, 1);

        retval = sysfs_create_group(&hall_data->input_dev->dev.kobj, &hall_attr_group);
        if(retval) {
                printk(KERN_ERR "%s: Failed to create sysfs\n", __FUNCTION__);
                goto exit_sys;
        }
       printk("%s sysfs_create_group sucess\n", __func__);
        return retval;
exit_sys:
fail_free_intr_pin:
    gpio_free(hall_data->intr_pin);
exit_register_input:
     input_free_device(hall_data->input_dev);
     hall_data->input_dev = NULL;
exit_kfree:
     kfree(hall_data);
exit:
     return err;
}

#ifdef CONFIG_OF
static struct of_device_id hall_match_table[] = {
        { .compatible = "hall_switch",},
        { },
};
#else
#define rmi4_match_table NULL
#endif

static struct platform_driver msm_hall_driver = {
	.probe = hall_probe,
	.driver = {
		.name = "msm_hall_switch",
		.owner = THIS_MODULE,
                .of_match_table = hall_match_table,
	},
};
static int __init hall_init(void)
{

   return platform_driver_register(&msm_hall_driver);

}

static void __exit hall_exit(void)
{
	platform_driver_unregister(&msm_hall_driver);
}

late_initcall(hall_init);
module_exit(hall_exit);
MODULE_DESCRIPTION("Hall switch sensor driver");
MODULE_LICENSE("GPL");



