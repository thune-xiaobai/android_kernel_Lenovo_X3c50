/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/sort.h>
#include <linux/signal.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <net/sock.h>
#include <net/genetlink.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>

#ifndef CONFIG_OF
#include <linux/spi/fpc1020.h>
//#include <linux/spi/fpc1020_common.h>
#include <linux/spi/fpc1020_regs.h>
#include <linux/spi/fpc1020_input.h>
#include <linux/spi/fpc1020_capture.h>
#include <linux/spi/fpc1020_regulator.h>
#else
#include <linux/of.h>
#include "fpc1020.h"
#include "fpc1020_common.h"
#include "fpc1020_regs.h"
//#include "fpc1020_input.h"
//#include "fpc1020_capture.h"
#include "fpc1020_regulator.h"
#endif
/*[kernel,shaojc2, KLASSEN-898, add head file for compile errror],add*/
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
/*[kernel,shaojc2, KLASSEN-898, add head file for compile errror],add end*/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fingerprint Cards AB <tech@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 touch sensor driver.");

/*[kernel,shaojc2, KLASSEN-898, add for spi secure transfer],add*/
struct tz_test{
	fpc1020_data_t *fpc1020;
	struct spi_message spi_msg;
	struct spi_transfer spi_xfer;
	struct spi_device *spi;
	struct pinctrl 	*pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_sleep;
	struct clk 	*core_clk;
	struct clk 	*iface_clk;
	struct mutex lock;
	bool clocks_enabled;
	bool clocks_suspended;
	uint8_t *tx_buf;
	uint32_t qup_id;
	uint32_t probe_state;
	uint32_t secure_state;
	uint32_t max_spi_speed;
	uint32_t irq_event_enable;
	uint32_t irq_pin_level;
	uint32_t cs_pin_level;
	uint32_t pid;
	uint32_t sig_no;
	uint32_t work_mode;
	uint32_t camera_key;
}tz_test;

#define SPI_PINCTRL_STATE_DEFAULT 	"spi_default"
#define SPI_PINCTRL_STATE_SLEEP 	"spi_sleep"
#define FPC_TTW_HOLD_TIME 1000


/* -------------------------------------------------------------------- */
/* fpc1020 driver constants						*/
/* -------------------------------------------------------------------- */
#define FPC1020_CLASS_NAME                      "fpsensor"
#define FPC1020_WORKER_THREAD_NAME		"fpc1020_worker"

/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */
#define FPC1020_KEY_CAMERA 		KEY_CAMERA		/* 212*/


static int __init fpc1020_init(void);

static void __exit fpc1020_exit(void);

static int __devinit fpc1020_probe(struct spi_device *spi);

static int __devexit fpc1020_remove(struct spi_device *spi);

static int fpc1020_suspend(struct device *dev);

static int fpc1020_resume(struct device *dev);


static int __devinit fpc1020_reset_init(fpc1020_data_t *fpc1020,
		struct fpc1020_platform_data *pdata);

static int __devinit fpc1020_irq_init(fpc1020_data_t *fpc1020,
		struct fpc1020_platform_data *pdata);


static int __devinit fpc1020_get_of_pdata(struct device *dev,
		struct fpc1020_platform_data *pdata);


irqreturn_t fpc1020_irq_handler(int irq, void *_fpc1020);

/* -------------------------------------------------------------------- */
void __devexit fpc1020_input_destroy(fpc1020_data_t *fpc1020)
{
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if (fpc1020->input_dev != NULL)
		input_free_device(fpc1020->input_dev);
}

/* -------------------------------------------------------------------- */
/* External interface							*/
/* -------------------------------------------------------------------- */
module_init(fpc1020_init);
module_exit(fpc1020_exit);

static const struct dev_pm_ops fpc1020_pm = {
	.suspend = fpc1020_suspend,
	.resume = fpc1020_resume
};

#ifdef CONFIG_OF
static struct of_device_id fpc1020_of_match[] __devinitdata = {
	{ .compatible = "fpc,fpc1020", },
	{}
};

MODULE_DEVICE_TABLE(of, fpc1020_of_match);
#endif

static struct spi_driver fpc1020_driver = {
	.driver = {
		.name	= FPC1020_DEV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.pm     = &fpc1020_pm,
#ifdef CONFIG_OF
		.of_match_table = fpc1020_of_match,
#endif
	},
	.probe	= fpc1020_probe,
	.remove	= __devexit_p(fpc1020_remove)
};



static int UV_MODE = 0;

static int __init fpc1020_init(void)
{
	printk("fpc1020_init\n");
	if (spi_register_driver(&fpc1020_driver)){
		return -EINVAL;
	}
	
	return 0;
}

/* -------------------------------------------------------------------- */
static void __exit fpc1020_exit(void)
{
	printk(KERN_INFO "%s\n", __func__);
	spi_unregister_driver(&fpc1020_driver);
}

static int __devinit fpc1020_input_init(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->input_dev = input_allocate_device();

	if (!fpc1020->input_dev) {
		dev_err(&fpc1020->spi->dev, "Input_allocate_device failed.\n");
		error  = -ENOMEM;
	}

	if (!error) {
		fpc1020->input_dev->name = FPC1020_DEV_NAME;
		set_bit(EV_KEY,		fpc1020->input_dev->evbit);
		set_bit(FPC1020_KEY_CAMERA, fpc1020->input_dev->keybit);
		error = input_register_device(fpc1020->input_dev);
	}

	if (error) {
		dev_err(&fpc1020->spi->dev, "Input_register_device failed.\n");
		input_free_device(fpc1020->input_dev);
		fpc1020->input_dev = NULL;
	}

	return error;
}
static int spi_set_pinctrl(struct tz_test *tz_test,bool active)
{
	int ret;

	if(active){
		ret = pinctrl_select_state(tz_test->pinctrl,tz_test->pins_active);
	}else{
		ret = pinctrl_select_state(tz_test->pinctrl,tz_test->pins_sleep);
	}

	if(ret != 0)
		dev_err(&tz_test->spi->dev,"%s %d ret :0x%08x\n",__func__,__LINE__,ret);
	return ret;

}

static long msm_spi_clk_max_rate(struct clk *clk, unsigned long rate)
{
	long lowest_available, nearest_low, step_size, cur;
	long step_direction = -1;
	long guess = rate;
	int  max_steps = 10;

	cur =  clk_round_rate(clk, rate);
	if (cur == rate)
		return rate;

	/* if we got here then: cur > rate */
	lowest_available =  clk_round_rate(clk, 0);
	if (lowest_available > rate)
		return -EINVAL;

	step_size = (rate - lowest_available) >> 1;
	nearest_low = lowest_available;

	while (max_steps-- && step_size) {
		guess += step_size * step_direction;

		cur =  clk_round_rate(clk, guess);

		if ((cur < rate) && (cur > nearest_low))
			nearest_low = cur;

		/*
		 * if we stepped too far, then start stepping in the other
		 * direction with half the step size
		 */
		if (((cur > rate) && (step_direction > 0))
				|| ((cur < rate) && (step_direction < 0))) {
			step_direction = -step_direction;
			step_size >>= 1;
		}
	}
	return nearest_low;
}
static int spi_set_fabric(struct tz_test * tz_test, bool active)
{
	int ret = 0;
	int count;

#if 1
	struct spi_master *master = tz_test->spi->master;

	if(active){
		ret = master->prepare_transfer_hardware(master);
	}else{
		ret = master->unprepare_transfer_hardware(master);
	}
#endif
#if 1
	count = atomic_read(&tz_test->spi->dev.power.usage_count);
	dev_dbg(&tz_test->spi->dev,"%s %d power.usage_count : %d\n",__func__,__LINE__,count);
#endif
	if(ret != 0)
		dev_err(&tz_test->spi->dev,"%s %d ret :0x%08x\n",__func__,__LINE__,ret);

	return ret;
}

static int spi_set_clks(struct tz_test * tz_test,bool enable)
{
	int ret;
	long rate ;
	//mutex_lock(&tz_test->lock);
	
	if (enable == tz_test->clocks_enabled)
		goto out;
	rate =  msm_spi_clk_max_rate(tz_test->core_clk,tz_test->max_spi_speed);

	if (rate < 0) {
		dev_err(&tz_test->spi->dev,
				"%s: no match found for requested clock frequency:%d",
				__func__, tz_test->max_spi_speed);
		return -1;
	}

	if(enable){
		spi_set_fabric(tz_test,true);
	
                spi_set_pinctrl(tz_test,true);

		ret = clk_set_rate(tz_test->core_clk,rate);
		if(ret){
			dev_err(&tz_test->spi->dev,"%s:Error setting clk_rate : %d\n",
					__func__,tz_test->max_spi_speed);
		}	
		ret = clk_prepare_enable(tz_test->core_clk);
		if(ret){
			dev_err(&tz_test->spi->dev,"%s:Error enable core_clk\n",__func__);
		}	

		ret = clk_prepare_enable(tz_test->iface_clk);
		if(ret){
			dev_err(&tz_test->spi->dev,"%s:Error enable iface_clk\n",__func__);
		}
		tz_test->clocks_enabled = true;
	}else{
		spi_set_fabric(tz_test,false);
		spi_set_pinctrl(tz_test,false);
		clk_disable_unprepare(tz_test->iface_clk);
		clk_disable_unprepare(tz_test->core_clk);
		tz_test->clocks_enabled = false;
	}	
out:
	//mutex_unlock(&tz_test->lock);
	return ret;
}

static int device_prepare(fpc1020_data_t *fpc1020, bool enable)
{
	int rc;

	if (enable && !fpc1020->prepared) {
		fpc1020->prepared = true;
		gpio_set_value(fpc1020->reset_gpio, 0);
		usleep_range(100, 900);
		gpio_set_value(fpc1020->reset_gpio, 1);
		usleep_range(100, 100);

	} else if (!enable && fpc1020->prepared) {
		rc = 0;
		gpio_set_value(fpc1020->reset_gpio, 0);
		usleep_range(100, 900);
		fpc1020->prepared = false;
	} else {
		rc = 0;
	}
	return rc;
}

static ssize_t clk_enable_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return spi_set_clks(&tz_test, (*buf == '1')) ? : count;
}

static DEVICE_ATTR(clk_enable, S_IRUSR | S_IWUSR, NULL, clk_enable_set);


static ssize_t spi_prepare_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable")))
		rc = device_prepare(fpc1020, true);
	else if (!strncmp(buf, "disable", strlen("disable")))
		rc = device_prepare(fpc1020, false);
	else
		return -EINVAL;
	return rc ? rc : count;
}
static DEVICE_ATTR(spi_prepare, S_IRUSR | S_IWUSR, NULL, spi_prepare_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable"))) {
		fpc1020->wakeup_enabled = true;
		smp_wmb();
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		fpc1020->wakeup_enabled = false;
		smp_wmb();
	} else
		return -EINVAL;

	return count;
}
static DEVICE_ATTR(wakeup_enable, S_IRUSR | S_IWUSR, NULL, wakeup_enable_set);


/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *device,
			     struct device_attribute *attribute,
			     char* buffer)
{
	fpc1020_data_t *fpc1020 = dev_get_drvdata(device);
	int irq = gpio_get_value(fpc1020->irq_gpio);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}


/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *device,
			     struct device_attribute *attribute,
			     const char *buffer, size_t count)
{
	printk("%s\n", __func__);
	return count;
}

static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

static ssize_t camera_input_set(struct device *device,
			     struct device_attribute *attribute,
			     const char *buffer, size_t count)
{
        fpc1020_data_t *fpc1020 = dev_get_drvdata(device);
	printk("%s FPC1020_KEY_CAMERA set\n",__func__);
	input_report_key(fpc1020->input_dev,
			FPC1020_KEY_CAMERA, 1);
	input_report_key(fpc1020->input_dev,
			FPC1020_KEY_CAMERA, 0);

	input_sync(fpc1020->input_dev);
	return count;
}

static DEVICE_ATTR(camera_input, S_IWUSR, NULL, camera_input_set);

static struct attribute *attributes[] = {
	&dev_attr_spi_prepare.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	&dev_attr_camera_input.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

/*static int __devinit fpc1020_input_init(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->input_dev = input_allocate_device();

	if (!fpc1020->input_dev) {
		dev_err(&fpc1020->spi->dev, "Input_allocate_device failed.\n");
		error  = -ENOMEM;
	}

	if (!error) {
		fpc1020->input_dev->name = FPC1020_DEV_NAME;
	          input_set_capability(fpc1020->input_dev, EV_MSC,MSC_SCAN);
		error = input_register_device(fpc1020->input_dev);
	}

	if (error) {
		dev_err(&fpc1020->spi->dev, "Input_register_device failed.\n");
		input_free_device(fpc1020->input_dev);
		fpc1020->input_dev = NULL;
	}

	return error;
}*/


/*[KLASSEN,shaojc2,KLASSEN-898,add for spi secure transfer switch],add end*/
/* -------------------------------------------------------------------- */
static int __devinit fpc1020_probe(struct spi_device *spi)
{
	struct fpc1020_platform_data *fpc1020_pdata;
	struct fpc1020_platform_data pdata_of;
	struct device *dev = &spi->dev;
	int error = 0;
	fpc1020_data_t *fpc1020 = NULL;
	int irqf;
        int rc;

	fpc1020 = kzalloc(sizeof(*fpc1020), GFP_KERNEL);
	if (!fpc1020) {
		dev_err(&spi->dev,
				"failed to allocate memory for struct fpc1020_data\n");

		return -ENOMEM;
	}

	printk(KERN_INFO "%s\n", __func__);
        fpc1020->device = dev;
	spi_set_drvdata(spi, fpc1020);
	fpc1020->spi = spi;
	fpc1020->spi_freq_khz     = 1000u;
	fpc1020->spi_freq_reg_khz = 1000u;

	fpc1020->reset_gpio = -EINVAL;
	fpc1020->irq_gpio   = -EINVAL;
	fpc1020->cs_gpio    = -EINVAL;

	fpc1020->irq        = -EINVAL;

	fpc1020_pdata = spi->dev.platform_data;

	if (!fpc1020_pdata) {
		error = fpc1020_get_of_pdata(dev, &pdata_of);
		fpc1020_pdata = &pdata_of;

		if (error)
			goto err;
	}

	if (!fpc1020_pdata) {
		dev_err(&fpc1020->spi->dev,
				"spi->dev.platform_data is NULL.\n");
		error = -EINVAL;
		goto err;
	}

	fpc1020->wakeup_enabled = false;
	tz_test.clocks_enabled = false;
	tz_test.clocks_suspended = false;

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	//if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	//}
	mutex_init(&tz_test.lock);
	
	error = fpc1020_irq_init(fpc1020, fpc1020_pdata);
	if (error)
		goto err;

	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
       if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto err;
	}
	dev_dbg(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));

	wake_lock_init(&fpc1020->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");
	
	error = fpc1020_reset_init(fpc1020, fpc1020_pdata);
	if (error)
		goto err;

	//error = fpc1020_input_init(fpc1020);

	error= sysfs_create_group(&dev->kobj, &attribute_group);
	if (error) {
		dev_err(dev, "could not create sysfs\n");
		goto err;
	}

	error = fpc1020_input_init(fpc1020);
	if (error)
		goto err;
	
	tz_test.spi = spi;
	tz_test.probe_state = 0;
	tz_test.irq_event_enable = 0;
	tz_test.pid = -1;
	tz_test.sig_no = 40;
	tz_test.fpc1020 = fpc1020;
	tz_test.pinctrl = devm_pinctrl_get(&spi->dev);
	if(IS_ERR_OR_NULL(tz_test.pinctrl)){
		dev_err(&spi->dev,"%s : Failed to get pinctrl\n",__func__);
		tz_test.probe_state |= -1;
	}

	tz_test.pins_active = pinctrl_lookup_state(tz_test.pinctrl, SPI_PINCTRL_STATE_DEFAULT);
	if(IS_ERR_OR_NULL(tz_test.pins_active)){
		dev_err(&spi->dev,"%s : Failed to get pinctrl sate active\n",__func__);
		tz_test.probe_state |= -2;
	}

	tz_test.pins_sleep = pinctrl_lookup_state(tz_test.pinctrl, SPI_PINCTRL_STATE_SLEEP);
	if(IS_ERR_OR_NULL(tz_test.pins_sleep)){
		dev_err(&spi->dev,"%s : Failed to get pinctrl sate sleep\n",__func__);
		tz_test.probe_state |= -4;
	}

	tz_test.core_clk = clk_get(&spi->dev,"core_clk");
	if(IS_ERR(tz_test.core_clk)){
		dev_err(&spi->dev,"%s : Failed to get spi core_clk %p\n",__func__,tz_test.core_clk);
		tz_test.probe_state |= -8;
	}

	tz_test.iface_clk = clk_get(&spi->dev,"iface_clk");
	if(IS_ERR(tz_test.iface_clk)){
		dev_err(&spi->dev,"%s : Failed to get spi iface_clk %p\n",__func__,tz_test.iface_clk);
		tz_test.probe_state |= -16;
	}
	tz_test.secure_state = 0;
	printk(KERN_ALERT "%s %s probe state : 0x%08x\n",__FILE__,__func__,tz_test.probe_state);

	if(UV_MODE == 1) {
		fpc1020->spi_freq_khz     = 5000u;
		fpc1020->spi_freq_reg_khz = 5000u;
	}
	if (of_property_read_bool(dev->of_node, "fpc,enable-on-boot")) {
		dev_info(dev, "Enabling hardware\n");
		//(void)device_prepare(fpc1020, true);
		//(void)spi_set_clks(&tz_test,1);
	}
	return 0;

err:
	return error;
}


/* -------------------------------------------------------------------- */
static int __devexit fpc1020_remove(struct spi_device *spi)
{
	fpc1020_data_t *fpc1020 = spi_get_drvdata(spi);

	sysfs_remove_group(&spi->dev.kobj, &attribute_group);
	mutex_destroy(&tz_test.lock);
	wake_lock_destroy(&fpc1020->ttw_wl);
	dev_info(&spi->dev, "%s\n", __func__);

	return 0;
}


/* -------------------------------------------------------------------- */
static int fpc1020_suspend(struct device *dev)
{
	tz_test.clocks_suspended = tz_test.clocks_enabled;
	spi_set_clks(&tz_test, false);
	return 0;
}


/* -------------------------------------------------------------------- */
static int fpc1020_resume(struct device *dev)
{
	if (tz_test.clocks_suspended)
		spi_set_clks(&tz_test,true);	
	return 0;
}

/* -------------------------------------------------------------------- */
static int __devinit fpc1020_reset_init(fpc1020_data_t *fpc1020,
		struct fpc1020_platform_data *pdata)
{
	int error = 0;

	if (gpio_is_valid(pdata->reset_gpio)) {

		dev_info(&fpc1020->spi->dev,
				"Assign HW reset -> GPIO%d\n", pdata->reset_gpio);

		fpc1020->soft_reset_enabled = false;

		error = gpio_request(pdata->reset_gpio, "fpc1020_reset");

		if (error) {
			dev_err(&fpc1020->spi->dev,
					"gpio_request (reset) failed.\n");
			return error;
		}

		fpc1020->reset_gpio = pdata->reset_gpio;

		error = gpio_direction_output(fpc1020->reset_gpio, 1);

		if (error) {
			dev_err(&fpc1020->spi->dev,
					"gpio_direction_output(reset) failed.\n");
			return error;
		}
	} else {
		dev_info(&fpc1020->spi->dev, "Using soft reset\n");

		fpc1020->soft_reset_enabled = true;
	}

	return error;
}


/* -------------------------------------------------------------------- */
static int __devinit fpc1020_irq_init(fpc1020_data_t *fpc1020,
		struct fpc1020_platform_data *pdata)
{
	int error = 0;

	if (gpio_is_valid(pdata->irq_gpio)) {

		dev_info(&fpc1020->spi->dev,
				"Assign IRQ -> GPIO%d\n",
				pdata->irq_gpio);

		error = gpio_request(pdata->irq_gpio, "fpc1020_irq");

		if (error) {
			dev_err(&fpc1020->spi->dev,
					"gpio_request (irq) failed.\n");

			return error;
		}

		fpc1020->irq_gpio = pdata->irq_gpio;

		error = gpio_direction_input(fpc1020->irq_gpio);

		if (error) {
			dev_err(&fpc1020->spi->dev,
					"gpio_direction_input (irq) failed.\n");
			return error;
		}
	} else {
		return -EINVAL;
	}

	/*fpc1020->irq = gpio_to_irq(fpc1020->irq_gpio);

	if (fpc1020->irq < 0) {
		dev_err(&fpc1020->spi->dev, "gpio_to_irq failed.\n");
		error = fpc1020->irq;
		return error;
	}

	error = request_irq(fpc1020->irq, fpc1020_irq_handler,
			IRQF_TRIGGER_RISING, "fpc1020", fpc1020);

	if (error) {
		dev_err(&fpc1020->spi->dev,
				"request_irq %i failed.\n",
				fpc1020->irq);

		fpc1020->irq = -EINVAL;

		return error;
	}
	dev_dbg(&fpc1020->spi->dev, " fpc1020 irq : %d\n",fpc1020->irq);*/

	return error;
}


static int __devinit fpc1020_get_of_pdata(struct device *dev,
		struct fpc1020_platform_data *pdata)
{
	struct device_node *node = dev->of_node;
	int ret;
	const void *vddtx_prop = of_get_property(node, "fpc,vddtx_mv", NULL);
	const void *boost_prop =
		of_get_property(node, "fpc,txout_boost_enable", NULL);
	const void *hwid_prop =
		of_get_property(node, "fpc,force_hwid", NULL);
	if (node == NULL) {
		dev_err(dev, "%s: Could not find OF device node\n", __func__);
		goto of_err;
	}

	pdata->irq_gpio = of_get_named_gpio(node, "fpc,gpio_irq",   0);
	if(pdata->irq_gpio == 0) {
		printk("Failed to get irq gpio.\n");
	}
	pdata->reset_gpio = of_get_named_gpio(node, "fpc,gpio_reset",0);
	if(pdata->reset_gpio == 0) {
		printk("Failed to get reset gpio.\n");
	}
	pdata->cs_gpio = of_get_named_gpio(node, "fpc,gpio_cs",0);
	if(pdata->cs_gpio == 0) {
		printk("Failed to get cs gpio.\n");
	}
	pdata->id_gpio= of_get_named_gpio(node, "fpc,finger_id",0);
	if(pdata->id_gpio == 0) {
		printk("Failed to get id gpio.\n");
	}
	ret = gpio_request(pdata->id_gpio, "fpc_id");
	if(ret)
		printk("request fpc id error\n");
	ret = gpio_direction_input(pdata->id_gpio);
	if(gpio_get_value(pdata->id_gpio)==1){
		UV_MODE = 1;
		printk("fpc1020:UV = 1 \n");
		tz_test.max_spi_speed = 5 * 1000 * 1000;
	}
	else{
		UV_MODE = 0;
		printk("fpc1020:UV = 0\n");
		tz_test.max_spi_speed = 2 * 1000 * 1000;
	}

	printk("==========pdata->reset_gpio = %d\n",pdata->reset_gpio);
	pdata->external_supply_mv =
		(vddtx_prop != NULL) ? be32_to_cpup(vddtx_prop) : 0;

	pdata->txout_boost = (boost_prop != NULL) ? 1 : 0;

	pdata->force_hwid =
		(hwid_prop != NULL) ? be32_to_cpup(hwid_prop) : 0;

	return 0;

of_err:
	pdata->reset_gpio = -EINVAL;
	pdata->irq_gpio   = -EINVAL;
	pdata->cs_gpio    = -EINVAL;
	pdata->force_hwid = -EINVAL;

	pdata->external_supply_mv = 0;
	pdata->txout_boost = 0;

	return -ENODEV;
}




/* -------------------------------------------------------------------- */
irqreturn_t fpc1020_irq_handler(int irq, void *_fpc1020)
{
#if 0
	fpc1020_data_t *fpc1020 = _fpc1020;
	//if (gpio_get_value(fpc1020->irq_gpio)) {
		printk("1111111111111111111111111\n");
		//fpc1020_data_t *fpc1020 = _fpc1020;
		input_event(fpc1020->input_dev, EV_MSC, MSC_SCAN, ++fpc1020->irq_num);
		input_sync(fpc1020->input_dev);
		printk("%s %d\n", __func__, fpc1020->irq_num);
		return IRQ_HANDLED;
	//}
	//return IRQ_NONE;
#endif
#if 1
	fpc1020_data_t *fpc1020 = _fpc1020;
	printk("%s\n", __func__);

	/* Make sure 'wakeup_enabled' is updated before using it
	** since this is interrupt context (other thread...) */
	smp_rmb();

	if (fpc1020->wakeup_enabled) {
		wake_lock_timeout(&fpc1020->ttw_wl,
					msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	}

	sysfs_notify(&fpc1020->device->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
#endif

}

