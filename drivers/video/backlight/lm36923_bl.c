/*

 * Simple driver for multi-string single control backlight driver chips of
	
 * Texas Instruments.
	
 *
	
 * Copyright (C) 2015 Texas Instruments
	
 * Author: Daniel Jeong  <gshark.jeong@gmail.com>
	
 *         Ldd Mlp <ldd-mlp@list.ti.com>
	
 *
	
 * This program is free software; you can redistribute it and/or modify
	
 * it under the terms of the GNU General Public License version 2 as
	
 * published by the Free Software Foundation.
	
 *
	
 */
	
#include <linux/backlight.h>
	
#include <linux/delay.h>
	
#include <linux/err.h>
	
#include <linux/i2c.h>
	
#include <linux/module.h>
	
#include <linux/platform_data/ti-scbl.h>
	
#include <linux/regmap.h>
	
#include <linux/slab.h>
	
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
	
	
#define scbl_attr(_name, _show, _store)\
{\
	.attr = {\
		.name = _name,\
		.mode = S_IWUSR | S_IRUSR,\
	},\
	.show = _show,\
	.store = _store,\
}
	
	
struct scbl_reg_ctrl {
	
	unsigned int reg;
	
	unsigned int mask;
	
	unsigned int shift;
	
};
	
	
/*
	
 *brightness control 
	
 *@min : minimum brightness
	
 *@max : maximum brightness
	
 *@msb_shift : msb bit shfit
	
 *@lsb_mask  : lsb bit mask
	
 *@msb : register for the upper 8bits of the brightness code
	
 *@lsb : register for the lower 8bits of the brightness code
	
 */
	
struct scbl_brt_ctrl {
	
	unsigned int min;
	
	unsigned int max;
	
	unsigned int msb_shift;
	
	unsigned int lsb_mask;
	
	struct scbl_reg_ctrl msb;
	
	struct scbl_reg_ctrl lsb;
	
};
	
	
/*
	
 *fault data
	
 *@reg : fault register
	
 *@mask: mask bit of fault
	
 *@bl_fault : backlight fault mapped the bit
	
 */
	
struct scbl_fault {
	
	unsigned int reg;
	
	unsigned int mask;
	
	unsigned int bl_fault;
	
};
	
	
/*
	
 *single control backlight data
	
 *@name    : chip name
	
 *@num_led : number of led string
	
 *@en_chip : chip enable register
	
 *@en_led  : led string enable register
	
 *@brt     : brightness register
	
 *@fault   : fault register
	
 *@boot    : function that should be called before chip enable
	
 *@init    : function that should be called after chip enable
	
 *@reset   : reset function
	
 *@num_devnode   : number of device node to be created
	
 *@scbl_dev_attr : device node attribute
	
 */
	
struct scbl_data {
	
	char *name;
	
	
	int num_led;
	
	struct scbl_reg_ctrl en_chip;
	
	struct scbl_reg_ctrl en_led[SCBL_LED_MAX];
	
	
	struct scbl_brt_ctrl brt;
	
	struct scbl_fault fault[SCBL_FAULT_MAX];
	
	
	int (*boot)(struct regmap* regmap);
	
	int (*init)(struct regmap* regmap);
	
	int (*reset)(struct regmap* regmap);
	
	
	
	int num_devnode;
	
	struct device_attribute *scbl_dev_attr;

};
	
	
/*
	
 *chip data
	
 *@dev   : device
	
 *@regmap: register map
	
 *@bled  : backlight device
	
 *@data  : single control backlight data
	
 */
	
struct scbl_chip {
	
	struct device *dev;
	
	struct regmap *regmap;
	
	
	struct backlight_device *bled;
	
	const struct scbl_data *data; 
	
};
	
struct regmap* debug_regmap;
static int   level_old = 0;
/* chip reset */
	
static ssize_t scbl_reset_store(struct device *dev,
	
				  struct device_attribute *attr,
	
				  const char *buf, size_t size)
	
{
	
	struct scbl_chip *pchip = dev_get_drvdata(dev);
	
	const struct scbl_data *data = pchip->data; 
	
	u8 input;
	
	int ret;
	
    pr_info("==jinjt==%s enter \n",__func__);
	
	if (kstrtou8(buf, 0, &input))
	
		return -EINVAL;
	
	
	if (input == 1) {
	
		ret = data->reset(pchip->regmap);
	
		if (ret<0)
	
			return ret;
	
	}
	
	return size;
	
}
	
	
/* fault status */
	
static ssize_t scbl_fault_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct scbl_chip *pchip = dev_get_drvdata(dev);
	
	const struct scbl_data *data = pchip->data; 
	
	unsigned int reg_val, reg = 0xfff;
	
	s32 fault = 0x0000;
	
	int ret, icnt;
	
	
    pr_info("==jinjt==%s enter \n",__func__);
	for(icnt = 0; icnt < SCBL_FAULT_MAX; icnt++) {
	
		if(data->fault[icnt].mask == 0x00)
	
			continue;
	
		if(data->fault[icnt].reg != reg) {
	
			reg = data->fault[icnt].reg;
	
			ret = regmap_read(pchip->regmap, reg, &reg_val);
	
			if (ret < 0) {
	
				dev_err(pchip->dev, 
	
						"fail : i2c access to register.\n");
	
				return sprintf(buf, "%d\n", ret);
	
			}
	
		}
	
		if (reg_val & data->fault[icnt].mask)
	
			fault |= data->fault[icnt].bl_fault;		
	
	}
	
	return sprintf(buf, "%d\n", fault);
	
}
	
u8 dimming_enabled = 1;
static char tmp_dimming_value =0;
extern int  dimming_enable_count_for_set_bl;
void lm36923_set_dimming(int enable);
static ssize_t scbl_dimming_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
    pr_info("==jinjt==%s line=%d \n",__func__,__LINE__);	
	
	return sprintf(buf, "%d\n",dimming_enabled);
	
}
	
	
static ssize_t scbl_dimming_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
	
	
    pr_info("==aaaaaaaaaajinjt==%s line=%d \n",__func__,__LINE__);	
	
	if (kstrtou8(buf, 0, &dimming_enabled))
	
		return -EINVAL;
	
	return size;
	
}
	
u8 backlight_mode = 1;
//bit 0 normal mode
//bit 1 tempreture mode
//bit 2 outside mode
extern void backlight_mode_change(int mode);
static ssize_t lm36923_backlight_buf_chg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
    pr_info("==jinjt==%s line=%d \n",__func__,__LINE__);	
	
	return sprintf(buf, "%d\n",backlight_mode);
	
}
	
	
static ssize_t lm36923_backlight_buf_chg_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
	
	
	
	if (kstrtou8(buf, 0, &backlight_mode))
	
		return -EINVAL;
    pr_info("==aaaaaaaaaajinjt==%s line=%d backlight_mode=%d\n",__func__,__LINE__,backlight_mode);	
    backlight_mode_change(backlight_mode);
	return size;
	
}
/* chip enable control */
	
static ssize_t scbl_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	struct scbl_chip *pchip = dev_get_drvdata(dev);
	
	const struct scbl_data *data = pchip->data; 
	
	unsigned int reg_val;
	
	int ret;
	
    pr_info("==jinjt==%s line=%d \n",__func__,__LINE__);	
	
	ret = regmap_read(pchip->regmap, data->en_chip.reg, &reg_val);
	
	if (ret <0) {		
	
		return sprintf(buf, "%d\n", -EINVAL);
	
	}
	
	return sprintf(buf, "%d\n",(reg_val & data->en_chip.mask) >> data->en_chip.shift);
	
}
	
	
static ssize_t scbl_enable_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
	struct scbl_chip *pchip = dev_get_drvdata(dev);
	
	const struct scbl_data *data = pchip->data; 
	
	int ret;
	
	u8 input;
	
    pr_info("==jinjt==%s line=%d \n",__func__,__LINE__);	
	
	if (kstrtou8(buf, 0, &input))
	
		return -EINVAL;
	
	
	if (input == 0)
	
		ret = regmap_update_bits(pchip->regmap, data->en_chip.reg, data->en_chip.mask, 0x00);
	
	else
	
		ret = regmap_update_bits(pchip->regmap, data->en_chip.reg,data->en_chip.mask, 0x01 << data->en_chip.shift);
	
	if(ret <0)
	
		return ret;
	
	
	return size;
	
}
	
	
/* brightness control */
	
static int scbl_update_status(struct backlight_device *bl)
	
{
	
	struct scbl_chip *pchip = bl_get_data(bl);
	
	const struct scbl_data *data = pchip->data; 
	
	int ret = -EINVAL;
	
	int msb, lsb;

    /*pr_debug("==jinjt==%s line=%d \n",__func__,__LINE__);*/
	
	if (bl->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
	
		bl->props.brightness = 0;
	
    ret = gpio_get_value(985);
    if(ret == 0)
    {
        pr_err("%s jinjt lm36923 is disable\n",__func__);
        return 0;
    }
	
	/* lsb write first */
	
	if(data->brt.lsb_mask != 0x00) {
	
		msb = bl->props.brightness >> data->brt.msb_shift ;
	
		lsb = bl->props.brightness & data->brt.lsb_mask ;
	
	
		ret = regmap_update_bits(pchip->regmap, data->brt.lsb.reg,data->brt.lsb.mask, lsb << data->brt.lsb.shift);
	
	} else {
	
		msb = bl->props.brightness;
	
	}
	
	
	/* msb write */
	
	ret = regmap_update_bits(pchip->regmap, data->brt.msb.reg,data->brt.msb.mask, msb << data->brt.msb.shift);
	
	if (ret < 0)
	
		dev_err(pchip->dev, "fail : i2c access to register.\n");
	
	else
	
		ret = bl->props.brightness;
#if 0 
    ret = gpio_get_value(985);
    if(ret != 0)
    {
        pr_err("==jinjt==%s line=%d brightness=%d\n",__func__,__LINE__,ret);
        regmap_read(pchip->regmap,0x10,&ret);
        pr_err("==jinjt==%s [0x10]=0x%0x \n",__func__,ret);
        regmap_read(pchip->regmap,0x11,&ret);
        pr_err("==jinjt==%s [0x11]=0x%0x \n",__func__,ret);
        regmap_read(pchip->regmap,0x12,&ret);
        pr_err("==jinjt==%s [0x12]=0x%0x \n",__func__,ret);
        regmap_read(pchip->regmap,0x13,&ret);
        pr_err("==jinjt==%s [0x13]=0x%0x \n",__func__,ret);
        regmap_read(pchip->regmap,0x18,&ret);
        pr_err("==jinjt==%s [0x18]=0x%0x \n",__func__,ret);
        regmap_read(pchip->regmap,0x19,&ret);
        pr_err("==jinjt==%s [0x19]=0x%0x \n",__func__,ret);
    }
#endif
	return ret;
	
}
	
	
static int scbl_get_brightness(struct backlight_device *bl)
	
{
	
    pr_info("==jinjt==%s line=%d \n",__func__,__LINE__);	
	return bl->props.brightness;
	
}
	
	
static const struct backlight_ops scbl_bled_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = scbl_update_status,
	.get_brightness = scbl_get_brightness,
};
	
	
/*
	
 * LP36923 backlight chip data
	
 *  lp36923 is an ultra-compact, highly efficient, three string white-LED
	
 * driver.
	
 */
	
static struct device_attribute lm36923_dev_attr[] = {
	scbl_attr("fault", scbl_fault_show, NULL),
	scbl_attr("reset", NULL, scbl_reset_store),
	scbl_attr("enable", scbl_enable_show, scbl_enable_store),
	scbl_attr("dimming", scbl_dimming_show, scbl_dimming_store),
	scbl_attr("mode", lm36923_backlight_buf_chg_show, lm36923_backlight_buf_chg_store),
};
	
	
static int scbl_lm36923_reset(struct regmap *regmap)
{
    pr_info("==jinjt==%s enter \n",__func__);
	return regmap_update_bits(regmap, 0x01, 0x01, 0x01);
}
	
static int scbl_lm36923_boot(struct regmap *regmap)
{
	int rval=0;
    int ret;
    pr_info("==jinjt==%s enter \n",__func__);
	/*rval = regmap_update_bits(regmap, 0x10, 0x0f, 0x0f);*/
    /*regmap_read(regmap,0x10,&ret);*/
    /*pr_info("==jinjt==%s [0x10]=0x%0x \n",__func__,ret);*/
	/* 
	
	 * For glitch free operation, the following data should
	
	 * only be written while chine enable bit is 0
	
	 */
	
	/* mapping mode / brt mode / ramp / ramp rate / bl_adj polarity */
	
    rval |= regmap_write(regmap, 0x11, 0x01);
	/*rval |= regmap_write(regmap, 0x12, 0xf3);*/
	/*rval |= regmap_write(regmap, 0x13, 0x4f);*/
    rval |= regmap_write(regmap, 0x18, 0x04);
    rval |= regmap_write(regmap, 0x19, 0x35);
	
    regmap_read(regmap,0x11,&ret);
    pr_info("==jinjt==%s [0x11]=0x%0x \n",__func__,ret);
	/* pwm */
	
    regmap_read(regmap,0x12,&ret);
    pr_info("==jinjt==%s [0x12]=0x%0x \n",__func__,ret);
	
    regmap_read(regmap,0x13,&ret);
    pr_info("==jinjt==%s [0x13]=0x%0x \n",__func__,ret);
    regmap_read(regmap,0x18,&ret);
    pr_info("==jinjt==%s [0x18]=0x%0x \n",__func__,ret);
    regmap_read(regmap,0x19,&ret);
    pr_info("==jinjt==%s [0x19]=0x%0x \n",__func__,ret);

    /*rval |= regmap_update_bits(regmap, 0x10, 0x0f, 0x0f);*/
    return rval;
#if 0	
	/* auto frequency high threshold */

    rval |= regmap_write(regmap, 0x15, 0x00);
	
	/* auto frequency low threshold */
	
	rval |= regmap_write(regmap, 0x16, 0x00);
	
	/* backlight adjust threshold */
	
	rval |= regmap_write(regmap, 0x17, 0x00);
	
	/* chip and leds enable */
	
	rval |= regmap_update_bits(regmap, 0x10, 0x0f, 0x0f);
	
	return rval;
#endif	
}
	
	
static const struct scbl_data bl_lm36923 = {
	
	.name = LM36923_NAME,
	
	.num_led = 3,
	
	.en_chip = {.reg = 0x10, .mask = 0x01, .shift = 0},
	
	.en_led = {
	
		[0] = {.reg = 0x10, .mask = 0x02, .shift = 1},
	
		[1] = {.reg = 0x10, .mask = 0x04, .shift = 2},
	
		[2] = {.reg = 0x10, .mask = 0x08, .shift = 3},
	
	},
#if 0	
	.brt = {
	
		.min = 0x00, .max = 0x7ff,
	
		.msb_shift = 3, .lsb_mask = 0x07,
	
		.msb = {.reg = 0x19, .mask = 0xff, .shift = 0},
	
		.lsb = {.reg = 0x18, .mask = 0x07, .shift = 0},
	
	},
#endif	
	.fault = {
	
		[0] = {
	
			.reg = 0x1f, .mask = 0x01, 
	
			.bl_fault = SCBL_FAULT_OVER_VOLTAGE
	
		},	
	
		[1] = {
	
			.reg = 0x1f, .mask = 0x02,
	
			.bl_fault = SCBL_FAULT_OVER_CURRENT
	
		},	
	
		[2] = {
	
			.reg = 0x1f, .mask = 0x04,
	
			.bl_fault = SCBL_FAULT_THERMAL_SHDN
	
		},	
	
		[3] = {
	
			.reg = 0x1f, .mask = 0x08,
	
			.bl_fault = SCBL_FAULT_LED_SHORT
	
		},	
	
		[4] = {
	
			.reg = 0x1f, .mask = 0x10,
	
			.bl_fault = SCBL_FAULT_LED_OPEN
	
		},	
	
	},
	
	.boot = scbl_lm36923_boot,
	
	.init = NULL,
	
	.reset = scbl_lm36923_reset,
	
	.num_devnode = 5,
	
	.scbl_dev_attr = lm36923_dev_attr,
	
};
	
	
static const struct regmap_config scbl_regmap = {

	.reg_bits = 8,
	
	.val_bits = 8,
	
	.max_register = SCBL_REG_MAX,
	
};

/*void  lm36923_set_brightness(int level)*/
/*{*/
    /*int ret;*/
    /*unsigned char high_bl;*/
    /*unsigned char low_bl;*/
    /*low_bl = level & 0x03;*/
    /*high_bl = level >> 3;*/
    /*if(NULL == debug_regmap)*/
    /*{*/
        /*pr_err("debug regmap is NULL %s\n",__func__);*/
        /*return;*/
    /*}*/
    /*pr_err("%s level=%d low_bl=%d  high_bl=%d\n",__func__,level,low_bl,high_bl);*/
	/*ret |= regmap_write(debug_regmap, 0x18, low_bl);*/
	/*ret |= regmap_write(debug_regmap, 0x19, high_bl);*/
        /*pr_err("%s ret=%d\n",__func__,ret);*/
/*}*/
void lm36923_set_config(void)
{
    int ret;
    if(NULL == debug_regmap)
    {
        pr_err("debug regmap is NULL %s\n",__func__);
        return;
    }
    ret |= regmap_write(debug_regmap, 0x11, 0x51);
    /*ret |= regmap_write(debug_regmap, 0x12, 0xf3);*/
	/*ret |= regmap_write(debug_regmap, 0x13, 0x4f);*/
    ret |= regmap_write(debug_regmap, 0x18, 0x06);
	ret |= regmap_write(debug_regmap, 0x19, 0xcc);
    msleep(10);
    pr_info("%s ret=%d\n",__func__,ret);
}
void lm36923_set_dimming_by_level(int enable,char  level )
{
    int ret = 0;
    if(NULL == debug_regmap)
    {
        pr_err("debug regmap is NULL %s\n",__func__);
        return;
    }
    if(enable ==1)
      {
     	if (tmp_dimming_value != level)
     	{
        		ret |= regmap_write(debug_regmap, 0x11,  level);
		tmp_dimming_value = level;
	}
	
       }
     else{
		if (tmp_dimming_value != level)
		{
         			ret |= regmap_write(debug_regmap, 0x11, 0x51);
			tmp_dimming_value= 0x51;
		}
     	}
	 
    pr_err("%s ret=%d enabled %d\n",__func__,ret,enable);
}
void lm36923_set_bl(int level)
{
    int ret;
    int hi;
    int low;
    char  target_value;
    printk("lm36923_set_bl level =%d\n",level);
    if(NULL == debug_regmap)
    {
        pr_err("debug regmap is NULL %s\n",__func__);
        return;
    }
     if (level_old >level){
			if( (level_old-level> 500)||(level > 700))
				target_value =  0x5b;
			else if ( level > 400 )
				target_value =  0x5d;
			else  
				target_value =  0x5f;
			}
	 else  
	 	{
	 		if  (level -500 > level_old)
	 			target_value =  0x51;
			else 
				{	
					if(dimming_enable_count_for_set_bl == 2) 
						target_value =  0x5b;
					else 
						target_value =  0x5d;
				}
		}
	if ((level == 0)||(dimming_enable_count_for_set_bl == 1) ||(!dimming_enabled))
	 	target_value =  0x51;
	if (level == 0)
		tmp_dimming_value = 0;

	 lm36923_set_dimming_by_level(dimming_enabled ,target_value);
	level_old =  level;
	
    low=level&0x07;
    hi=level >>3;
    ret |= regmap_write(debug_regmap, 0x18, low);
    ret |= regmap_write(debug_regmap, 0x19, hi);
    /*regmap_read(debug_regmap,0x18,&ret);*/
//    pr_info("==jinjt==%s [0x18]=0x%0x \n",__func__,ret);
    /*regmap_read(debug_regmap,0x19,&ret);*/
//    pr_info("==jinjt==%s [0x19]=0x%0x \n",__func__,ret);
//    pr_err("%s ret=%d\n",__func__,ret);

     if (dimming_enable_count_for_set_bl == 1)
     	{
		int   delay_time =5*level/40;
		msleep (delay_time);
     	}
    
	 	
}
	
void lm36923_set_dimming(int enable)
{
    int ret;
    if(NULL == debug_regmap)
    {
        pr_err("debug regmap is NULL %s\n",__func__);
        return;
    }
    if(enable ==1)
        ret |= regmap_write(debug_regmap, 0x11, 0x5f);
    else
        ret |= regmap_write(debug_regmap, 0x11, 0x51);
    pr_err("%s ret=%d enable=%d\n",__func__,ret,enable);
}
static int scbl_probe(struct i2c_client *client, const struct i2c_device_id *devid)
	
{
	
	struct scbl_chip *pchip;
	
	struct backlight_properties props;
	
	int ret, icnt;
	
	
    pr_err("==jinjt==%s enter \n",__func__);	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
	
		dev_err(&client->dev, "fail : i2c functionality check.\n");
	
		return -EOPNOTSUPP;
	
	}
	
	
	pchip = devm_kzalloc(&client->dev, sizeof(struct scbl_chip), GFP_KERNEL);
	
	if (!pchip)
	
		return -ENOMEM;
	
	pchip->dev = &client->dev;
	
	pchip->regmap = devm_regmap_init_i2c(client, &scbl_regmap);
debug_regmap = pchip->regmap;	
	if (IS_ERR(pchip->regmap)) {
	
		ret = PTR_ERR(pchip->regmap);
	
		dev_err(pchip->dev, "fail : allocate i2c register map.\n");
	
		return ret;
	
	}
	
	
	pchip->data = (const struct scbl_data *)devid->driver_data;
	
	i2c_set_clientdata(client, pchip);
	
	
	props.brightness = 0;
	
	props.type = BACKLIGHT_RAW;
	
	props.max_brightness = pchip->data->brt.max;
	
    pr_err("==jinjt==%s line=%d \n",__func__,__LINE__);	
	/*pchip->bled = devm_backlight_device_register(pchip->dev,pchip->data->name,pchip->dev,pchip, &scbl_bled_ops, &props);*/
	pchip->bled = backlight_device_register(pchip->data->name,pchip->dev,pchip, &scbl_bled_ops, &props);
	if (IS_ERR(pchip->bled)) {
	
		dev_err(pchip->dev, "fail : backlight register.\n");
	
		ret = PTR_ERR(pchip->bled);
	
		return ret;
	
	}
	
    pr_err("==jinjt==%s line=%d \n",__func__,__LINE__);	
	
	for (icnt = 0; icnt < pchip->data->num_devnode; icnt++) {
	
		ret = device_create_file(&(pchip->bled->dev), &(pchip->data->scbl_dev_attr)[icnt]);
	
		if (ret < 0) {
	
			dev_err(pchip->dev, "fail : node create\n");
	
			goto err_out;
	
		}
	
	}
	
	/* chip boot sequence before chip enable */
    /*gpio_set_value(985, 1);*/
    /*msleep(100);*/


    scbl_lm36923_reset(pchip->regmap);
	if(pchip->data->boot != NULL) {
	
		ret = pchip->data->boot(pchip->regmap);
	
		if (ret < 0)
	
			goto err_out;
	
	}
	
	/* chip enable */
	
	ret = regmap_update_bits(pchip->regmap, pchip->data->en_chip.reg,pchip->data->en_chip.mask,0x01<<pchip->data->en_chip.shift);
	
	
	/* chip init sequence after chip enable */
	
	if(pchip->data->init != NULL) {
	
		ret = pchip->data->boot(pchip->regmap);
	
		if (ret < 0)
	
			goto err_out;
	
	}
	
	dev_info(pchip->dev, "[%s] initialized.\n",	pchip->data->name);
	
    pr_err("==jinjt==%s exit line=%d \n",__func__,__LINE__);	
	return 0;
	
	
err_out:
	
	while (--icnt >= 0)
		device_remove_file(&(pchip->bled->dev),&(pchip->data->scbl_dev_attr)[icnt]);
	
	dev_info(pchip->dev, "[%s] init failed.\n",	pchip->data->name);
	
	return ret;
	
}
	
	
static int scbl_remove(struct i2c_client *client)
	
{
	
	struct scbl_chip *pchip = i2c_get_clientdata(client);
	
	int icnt;
	
    pr_err("==jinjt==%s line=%d \n",__func__,__LINE__);	
	
	for (icnt = 0; icnt < pchip->data->num_devnode; icnt++)
	
		device_remove_file(&(pchip->bled->dev),&(pchip->data->scbl_dev_attr)[icnt]);
	
	return 0;
	
}
	
static struct of_device_id lm36923_of_match_table[] = { 
        {.compatible = "ti,lm36923",},  
        {}
};
MODULE_DEVICE_TABLE(of, lm36923_of_match_table);
	
static const struct i2c_device_id scbl_id[] = {
    {LM36923_NAME, (unsigned long)&bl_lm36923},
    /*{LM36923_NAME, 0},*/
	{}
};
	
	
MODULE_DEVICE_TABLE(i2c, scbl_id);
static struct i2c_driver scbl_i2c_driver = {
	
	.driver = {
		   .name = LM36923_NAME,
           /*.owner = THIS_MODULE,*/
           .of_match_table = lm36923_of_match_table,
		   },
	
	.probe = scbl_probe,
	
	.remove = scbl_remove,
	
	.id_table = scbl_id,
	
};
	
static int __init lm36923_dev_init(void)
{
    pr_err("==jinjt==%s line=%d \n",__func__,__LINE__);	
    return i2c_add_driver(&scbl_i2c_driver);
}
module_init(lm36923_dev_init);
/*module_i2c_driver(scbl_i2c_driver);*/

static void __exit lm36923_dev_exit(void)
{
    i2c_del_driver(&scbl_i2c_driver);
}
module_exit(lm36923_dev_exit);

	
MODULE_DESCRIPTION("Texas Instruments Single Control Backlight Driver");
	
MODULE_AUTHOR("Daniel Jeong <gshark.jeong@gmail.com>");
	
MODULE_AUTHOR("Ldd Mlp <ldd-mlp@list.ti.com>");
	
MODULE_LICENSE("GPL v2");
