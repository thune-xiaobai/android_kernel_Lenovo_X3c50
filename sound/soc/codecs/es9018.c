/*
 * es9018.c -- es9018 ALSA SoC audio driver
 *
 */
#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <trace/events/asoc.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mfd/arizona/core.h>

#define INPUT_CONFIG_SOURCE 1
#define I2S_BIT_FORMAT_MASK (0x03 << 6)
#define GENERAL_SETTINGS_REG 7
#define SOFT_MUTE_MASK (0x03)
#define MASTER_MODE_CONTROL 10
#define I2S_CLK_DIVID_MASK (0x03 << 5)
#define SOFT_START_SETTING 14
#define SOFT_START_MASK (0x01 << 7)
#define LEFT_CHANNEL_VOLUME_15 15
#define RIGHT_CHANNEL_VOLUME_16 16
#define MASTER_TRIM_VOLUME_17 17
#define MASTER_TRIM_VOLUME_18 18
#define MASTER_TRIM_VOLUME_19 19
#define MASTER_TRIM_VOLUME_20 20
#define HEADPHONE_AMPLIFIER_CONTROL 42
#define I2S_CLK_MODE_MASK (0x01 << 7)

#define SAMPLING_RATE_8KHZ      8000
#define SAMPLING_RATE_16KHZ     16000
#define SAMPLING_RATE_32KHZ     32000
#define SAMPLING_RATE_48KHZ     48000
#define SAMPLING_RATE_96KHZ     96000
#define SAMPLING_RATE_192KHZ    192000
#define SAMPLING_RATE_176P4KHZ 176400
#define SAMPLING_RATE_88P2KHZ 88200
#define SAMPLING_RATE_44P1KHZ 44100

#define CLOSE_ES9018_DELAY_TIME_MS 4000

#define FLORIDA_HP 1
#define ESS_HP 2
#define PWR_DOWN 0
#define SW_MUTE 3

/* codec private data */
struct es9018_data {
	int reset_gpio;
	int i2c_scl_gpio;
	int i2c_sda_gpio;
	int i2c_addr;
	int clk_441k;
	int clk_48k;
	int ldo_0;
	int ldo_1;
	int es_hifi_pa_pwr_en;
		/*
	  * WM8281: hifi_sw_pwr_en = 0
	  *
	  * ES9018:   hifi_sw_pwr_en = 1
	  *		     hifi_switch_sel = 1
	  *		     hifi_switch_mute = 1
	*/
#if 0
	int hifi_sw_pwr_en;
	int hifi_switch_sel;
	int hifi_switch_mute;
#endif
};

struct es9018_priv {
	struct snd_soc_codec *codec;
	struct i2c_client *i2c_client;
	struct es9018_data *es9018_data;
	struct delayed_work sleep_work;
	struct mutex power_lock;
	struct delayed_work close_es9018_dwork;
	bool work_stop_request;
    bool malfunction;
} es9018_priv;

struct es9018_reg {
	unsigned char num;
	unsigned char value;
};

enum {
	CRYSTAL_45M = 0,
	CRYSTAL_49M,
};

struct es9018_ldo_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *es9018_ldo_sus;
	struct pinctrl_state *es9018_ldo_act;
};
static struct es9018_ldo_pinctrl_info es9018_ldo_pinctrl_info;

/* We only include the analogue supplies here; the digital supplies
 * need to be available well before this driver can be probed.
 */
 #if 0
struct es9018_reg es9018_init_register[44] = {
	{0, 0x00},
	{1, 0x8c},  /* I2S input */
	{2, 0x18},
	{3, 0x10},
	{4, 0x00},
	{5, 0x78},
	{6, 0x4a},  /* 47= 32KHz ; 57=44.1KHz; 67=48KHz */
	{7, 0x80},
	{8, 0x10},
	{9, 0x22},  /* slave mode=0x22;  master mode= 0xa2 */
	{10, 0xe5},
	{11, 0x02},
	{12, 0x5a},
	{13, 0x00},
	{14, 0x8a},
	{15, 0x00},
	{16, 0x00},
	{17, 0xff},
	{18, 0xff},
	{19, 0xff},
	{20, 0x3f},
	{21, 0x00},
	{22, 0x00},
	{23, 0x00},
	{24, 0x00},
	{25, 0x00},
	{26, 0x00},
	{27, 0x00},
	{28, 0x00},
	{29, 0x00},
	{30, 0x00},
	{31, 0x00},
	{32, 0x00},
	{33, 0x00},
	{34, 0x00},
	{35, 0x00},
	{36, 0x00},
	{37, 0x00},
	{38, 0x00},
	{39, 0x00},
	{40, 0x00},
	{41, 0x04},
	{42, 0x43},
	{43, 0x00}
};
 #endif
//#define ESS_SLAVE_MODE
struct es9018_reg es9018_init_register_test[44] = {
	{0, 0x00},
	{1, 0x8c},  /* I2S input */
	{2, 0x18},
	{3, 0x10},
	{4, 0x00},
	{5, 0x68},
	{6, 0x4a},  /* 47= 32KHz ; 57=44.1KHz; 67=48KHz */
	{7, 0x80},
	{8, 0x10},
	{9, 0x22}, 
#ifdef ESS_SLAVE_MODE
	{10, 0x45}, /* bit 7: 0 when slave, 1 when master */
#else
	{10, 0xc5}, /* bit 7: 0 when slave, 1 when master */
#endif
	{11, 0x02},
	{12, 0x5a},
	{13, 0x00},
	{14, 0x8a},
	{15, 0x00},
	{16, 0x00},
	{17, 0xff},
	{18, 0xff},
	{19, 0xff},
	{20, 0x7f},
	{21, 0x00},
	{22, 0x00},
	{23, 0x00},
	{24, 0x00},
	{25, 0x00},
	{26, 0x00},
	{27, 0x00},
	{28, 0x00},
	{29, 0x00},
	{30, 0x00},
	{31, 0x00},
	{32, 0x00},
	{33, 0x00},
	{34, 0x00},
	{35, 0x00},
	{36, 0x00},
	{37, 0x00},
	{38, 0x00},
	{39, 0x00},
	{40, 0x00},
	{41, 0x04},
	{42, 0x43},
	{43, 0x00}
};

static const DECLARE_TLV_DB_SCALE(es9018_gain, 0, 50, 1);

static int es9018_register_dump = -1;
static struct es9018_priv *g_es9018_priv = NULL;
static int es_sample_rate = SAMPLING_RATE_48KHZ;
static int cur_es_sample_rate = SAMPLING_RATE_48KHZ;
static int ess_headphone_fix_volume = 0;
static int auto_ess_headphone_volume = 0;
static bool es9018_closed = true;
static u8 es9018_chip_id = 0;
static int hp_imp_compensate = 0;
static bool ess_auto_gain = 1;

static int es9018_write_reg(struct i2c_client *client, int reg, u8 value);
static int es9018_read_reg(struct i2c_client *client, int reg);
static int set_osc_src(int sample_rate);
static int __set_clk_div(int sample_rate);
static int ess_headphones_ena(bool enable);
//int es9018_sample_rate_set(int sample_rate);
static int change_osc_n_sample_rate(int sample_rate);
static void reset_gpio_L(void);
static void reset_gpio_H(void);
static int es9018_open(void);
static int es9018_close(void);
static void clk_gpio_48k_H(void);
extern int headphone_switch_set(int val);
extern int headphone_switch_state(void);

#define ES9018_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |	\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |	\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |	\
		SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)

#define ES9018_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE | \
		SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE | \
		SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE)


static int es9018_register_dump_parm_set(const char *val, struct kernel_param *kp)
{
	int i, reg_val;
	param_set_int(val, kp);

	if (1 == es9018_register_dump) {
		for (i=0; i<44; i++) {
			reg_val = es9018_read_reg(g_es9018_priv->i2c_client, i);
			pr_info("enter %s, regester[%d] = %#x\n",
				__func__, i, reg_val);
			udelay(20);
		}
	} else {
		pr_info("enter %s, line %d\n", __func__, __LINE__);
	}
	return 0;
}
module_param_call(es9018_register_dump, es9018_register_dump_parm_set,
			param_get_int, &es9018_register_dump, 0664);


static bool need_change_osc_src(int src_sample_rate, int dst_sample_rate)
{
	pr_info("%s src_sample_rate %d  dst_sample_rate %d\n",__func__, src_sample_rate, dst_sample_rate);
	switch (src_sample_rate) {
		case SAMPLING_RATE_192KHZ:
		case SAMPLING_RATE_96KHZ:
		case SAMPLING_RATE_48KHZ:
			switch (dst_sample_rate) {
				case SAMPLING_RATE_44P1KHZ:
				case SAMPLING_RATE_88P2KHZ:	
				case SAMPLING_RATE_176P4KHZ: 
					return true;
				default:
					return false;
			}
		case SAMPLING_RATE_44P1KHZ:
		case SAMPLING_RATE_88P2KHZ:	
		case SAMPLING_RATE_176P4KHZ: 
			switch (dst_sample_rate) {
				case SAMPLING_RATE_192KHZ:
				case SAMPLING_RATE_96KHZ:
				case SAMPLING_RATE_48KHZ:
					return true;
				default:
					return false;
			}
		default:
			return false;
	}
}

static int change_osc_n_sample_rate(int sample_rate)
{
	int i;
    	unsigned int val;
        //mute
        val = es9018_read_reg(g_es9018_priv->i2c_client, GENERAL_SETTINGS_REG);
    	es9018_write_reg(g_es9018_priv->i2c_client, GENERAL_SETTINGS_REG, val  |SOFT_MUTE_MASK);
        //set soft start
        val = es9018_read_reg(g_es9018_priv->i2c_client, SOFT_START_SETTING);
    	es9018_write_reg(g_es9018_priv->i2c_client,SOFT_START_SETTING, val & (~SOFT_START_MASK));
        msleep(200);

        reset_gpio_L();
	set_osc_src(sample_rate);
	udelay(1000);
	reset_gpio_H();
	udelay(1000);
	for (i=0; i<=30; i++) {
		es9018_write_reg(g_es9018_priv->i2c_client,
		es9018_init_register_test[i].num,
		es9018_init_register_test[i].value);
	}
	__set_clk_div(sample_rate);
	return 0;
}

static void close_es9018_fn(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct es9018_priv *es9018_priv;
	pr_info("%s +++", __func__);

	dwork = to_delayed_work(work);
	es9018_priv = container_of(dwork, struct es9018_priv, close_es9018_dwork);

    //only when ess is connected, we can mute
    if (headphone_switch_state() == ESS_HP) {
        //mute switch and then we can close 9018
        headphone_switch_set(SW_MUTE);
    }
	es9018_close();
	gpio_set_value(g_es9018_priv->es9018_data->es_hifi_pa_pwr_en, 0);
	g_es9018_priv->work_stop_request = true;
	pr_info("%s ---", __func__);
	return;
}

static int ess_headphones_ena(bool enable)
{
	pr_info("%s: enable %d\n", __func__, enable);
	if (enable){
		if (g_es9018_priv->work_stop_request == false) {
			g_es9018_priv->work_stop_request = true;	
			if (cancel_delayed_work_sync(&g_es9018_priv->close_es9018_dwork)) {
				pr_info("%s cancel work success",__func__);
			}
		}
		if (es9018_closed) {	
            //we do not have to mute, because it must have been mute when close or connect to 8281
			es9018_open();
			gpio_set_value(g_es9018_priv->es9018_data->es_hifi_pa_pwr_en, 1);
		} else if (cur_es_sample_rate != es_sample_rate) {
			if (need_change_osc_src(cur_es_sample_rate, es_sample_rate)){
				change_osc_n_sample_rate(es_sample_rate);
			} else {
				__set_clk_div(es_sample_rate);
			}
		}
		headphone_switch_set(ESS_HP);
	} else {
		if (g_es9018_priv->work_stop_request == true) {
			g_es9018_priv->work_stop_request = false;
            schedule_delayed_work(&g_es9018_priv->close_es9018_dwork,
                msecs_to_jiffies(CLOSE_ES9018_DELAY_TIME_MS));
		}
	}
	return 0;
}

static int ess_headphone_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = !__gpio_get_value(g_es9018_priv->es9018_data->es_hifi_pa_pwr_en);
	pr_info("%s: ess_headphone %ld\n", __func__,ucontrol->value.integer.value[0] );
	return 0;
}
static int ess_headphone_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	if (ucontrol->value.integer.value[0]) {
		pr_info("%s: connect ess_headphone\n", __func__);
		//ess_headphones_ena(1);
	} else {
		pr_info("%s: disconnect ess_headphone\n", __func__);
		ess_headphones_ena(0);
	}
	return 0;
}

int ess_set_hp_imp_compensate(int gain)
{
    pr_info("%s enter compensate = %d \n",__func__,gain);
    hp_imp_compensate = gain;
    return 0;
}

#if 0
static int ess_volume_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
        u8 reg_val;
	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				LEFT_CHANNEL_VOLUME_15);
	ucontrol->value.integer.value[0] = 255-reg_val;
	pr_info("%s: volume %ld\n", __func__,ucontrol->value.integer.value[0] );
	return 0;
}
#endif
void ess_auto_gain_enable(bool ena)
{
	pr_debug("%s %d\n",__func__,ena);
	ess_auto_gain = ena;
	return;
}

static int ess_volume_put_imp(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int invert = mc->invert;
	int err;
	int val;

    /* ess_headphone_fix_volume zero indicate common headset */
    if (ess_headphone_fix_volume > 0)
        val = ess_headphone_fix_volume;
    else {
    	val = auto_ess_headphone_volume;
        pr_debug("%s: use old value %d\n", __func__, auto_ess_headphone_volume);
    	if (ess_auto_gain) {
    		val += hp_imp_compensate;
    		dev_info(codec->dev,
    			 "SET GAIN %d according to impedance, moved %d step\n",
    			 val, hp_imp_compensate);
        } else {
        	dev_info(codec->dev,
        		 "SET GAIN %d no move as auto gain disabled\n",val);
        }
    }
    if (val > 255) val = 255;
    if (val < 0) val = 0;
        
	if (invert)
		val = max - val;
	val = val << shift;

	es9018_init_register_test[LEFT_CHANNEL_VOLUME_15].value = val;
    es9018_init_register_test[RIGHT_CHANNEL_VOLUME_16].value = val;
    if (!es9018_closed) {
        err = es9018_write_reg(g_es9018_priv->i2c_client, LEFT_CHANNEL_VOLUME_15, val);
        if (err < 0)
	        return err;
        err = es9018_write_reg(g_es9018_priv->i2c_client, RIGHT_CHANNEL_VOLUME_16, val);
        if (err < 0)
        	return err;
    }
	return 0;

}

static int ess_auto_volume_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
    auto_ess_headphone_volume = ucontrol->value.integer.value[0];

    return ess_volume_put_imp(kcontrol, ucontrol);
}

int ess_auto_volume_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = auto_ess_headphone_volume;

	return 0;
}

static int ess_headphone_fix_volume_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	ess_headphone_fix_volume = (int)ucontrol->value.integer.value[0];
	pr_debug("%s %d\n",__func__, ess_headphone_fix_volume);

    return ess_volume_put_imp(kcontrol, ucontrol);
}

static int ess_headphone_fix_volume_get(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = ess_headphone_fix_volume;
	return 0;
}

#if 0
int es9018_sample_rate_set(int sample_rate)
{
	es_sample_rate = sample_rate;
	return 0;
}
#endif
static void power_gpio_0_H(void)
{
    int ret;
    pr_info("zhougd--%s enter",__func__);
    ret = pinctrl_select_state(es9018_ldo_pinctrl_info.pinctrl,
    		es9018_ldo_pinctrl_info.es9018_ldo_act);
    if (ret < 0) {
    	pr_err("failed to configure the gpio\n");
    }
#if 0
	gpio_set_value(g_es9018_priv->es9018_data->ldo_0, 1);
	pr_info("zhougd--%s(): gpio_0_level = %d\n", __func__,
		__gpio_get_value(g_es9018_priv->es9018_data->ldo_0));
#endif
}

static void power_gpio_0_L(void)
{
	int ret;
	pr_info("zhougd--%s enter",__func__);
	ret = pinctrl_select_state(es9018_ldo_pinctrl_info.pinctrl,
			es9018_ldo_pinctrl_info.es9018_ldo_sus);
	if (ret < 0) {
		pr_err("failed to configure the gpio\n");
	}
#if 0
		gpio_set_value(g_es9018_priv->es9018_data->ldo_0, 0);
		pr_info("%s(): gpio_0_level = %d\n", __func__,
			__gpio_get_value(g_es9018_priv->es9018_data->ldo_0));
#endif
}
#if 0
static void power_gpio_1_H(void)
{
	if (__gpio_get_value(g_es9018_priv->es9018_data->ldo_1) == 0) {
		gpio_set_value(g_es9018_priv->es9018_data->ldo_1, 1);
		pr_info("%s(): gpio_1_level = %d\n", __func__,
			__gpio_get_value(g_es9018_priv->es9018_data->ldo_1));
	}
}

static void power_gpio_1_L(void)
{
	if (__gpio_get_value(g_es9018_priv->es9018_data->ldo_1) == 1) {
		gpio_set_value(g_es9018_priv->es9018_data->ldo_1, 0);
		pr_info("%s(): gpio_1_level = %d\n", __func__,
			__gpio_get_value(g_es9018_priv->es9018_data->ldo_1));
	}
}
#endif
static void clk_gpio_441k_H(void)
{
	if (__gpio_get_value(g_es9018_priv->es9018_data->clk_441k) == 0) {
		gpio_set_value(g_es9018_priv->es9018_data->clk_441k, 1);
		pr_info("%s(): clk_441k_level = %d\n", __func__,
			__gpio_get_value(g_es9018_priv->es9018_data->clk_441k));
	}
}

static void clk_gpio_441k_L(void)
{
	if (__gpio_get_value(g_es9018_priv->es9018_data->clk_441k) == 1) {
		gpio_set_value(g_es9018_priv->es9018_data->clk_441k, 0);
		pr_info("%s(): clk_441k_level = %d\n", __func__,
			__gpio_get_value(g_es9018_priv->es9018_data->clk_441k));
	}
}

static void clk_gpio_48k_H(void)
{
	if (__gpio_get_value(g_es9018_priv->es9018_data->clk_48k) == 0) {
		gpio_set_value(g_es9018_priv->es9018_data->clk_48k, 1);
		pr_info("%s(): clk_48k_level = %d\n", __func__,
			__gpio_get_value(g_es9018_priv->es9018_data->clk_48k));
	}
}

static void clk_gpio_48k_L(void)
{
	if (__gpio_get_value(g_es9018_priv->es9018_data->clk_48k) == 1) {
		gpio_set_value(g_es9018_priv->es9018_data->clk_48k, 0);
		pr_info("%s(): clk_441k_level = %d\n", __func__,
			__gpio_get_value(g_es9018_priv->es9018_data->clk_48k));
	}
}

static void reset_gpio_H(void)
{
	if (__gpio_get_value(g_es9018_priv->es9018_data->reset_gpio) == 0) {
		gpio_set_value(g_es9018_priv->es9018_data->reset_gpio, 1);
		pr_info("%s(): reset_gpio_level = %d\n", __func__,
			__gpio_get_value(g_es9018_priv->es9018_data->reset_gpio));
	}
}

static void reset_gpio_L(void)
{
	if (__gpio_get_value(g_es9018_priv->es9018_data->reset_gpio) == 1) {
		gpio_set_value(g_es9018_priv->es9018_data->reset_gpio, 0);
		pr_info("%s(): reset_gpio_level = %d\n", __func__,
			__gpio_get_value(g_es9018_priv->es9018_data->reset_gpio));
	}
}

static int set_osc_src(int sample_rate)
{
	pr_info("%s es_sample_rate %d",__func__,sample_rate);
	switch (sample_rate) {
	case SAMPLING_RATE_192KHZ:
	case SAMPLING_RATE_96KHZ:
	case SAMPLING_RATE_48KHZ:
		clk_gpio_441k_L();
		clk_gpio_48k_H();
		udelay(10000);
		break;
	case SAMPLING_RATE_44P1KHZ:
	case SAMPLING_RATE_88P2KHZ:	
	case SAMPLING_RATE_176P4KHZ:
		clk_gpio_48k_L();
		clk_gpio_441k_H();
		udelay(10000);
		break;
	default:
		break;
	}
	return 0;
}

static int __set_clk_div(int sample_rate)
{
	u8 reg_val;
	pr_info("%s es_sample_rate %d",__func__,sample_rate);
	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
			MASTER_MODE_CONTROL);
	reg_val &= ~(I2S_CLK_DIVID_MASK);
	switch (sample_rate) {
		case SAMPLING_RATE_192KHZ:
		case SAMPLING_RATE_176P4KHZ:
			break;
		case SAMPLING_RATE_96KHZ:
		case SAMPLING_RATE_88P2KHZ:
			reg_val |=  1 << 5;
			break;
		case SAMPLING_RATE_48KHZ:
		case SAMPLING_RATE_44P1KHZ:
			reg_val |=  2 << 5;
			break;
		default:
			pr_info("%s not a valid sample rate",__func__);
			return -1;
	}
	es9018_write_reg(g_es9018_priv->i2c_client,
				MASTER_MODE_CONTROL, reg_val);
	pr_info("%s reg_val %d",__func__,reg_val);
	cur_es_sample_rate = 	sample_rate;
	return 0;
}

static int es9018_open(void)
{
    int ret = 0;
	int i = 0;

	pr_info("%s +++\n", __func__);
	power_gpio_0_H();
	set_osc_src(es_sample_rate);
	//power_gpio_1_H();
	reset_gpio_L();
	udelay(1000);
	reset_gpio_H();
	udelay(1000);
    es9018_closed = false;
    if (g_es9018_priv && g_es9018_priv->malfunction)
        return 0;
	for (i = 0; i <= 30; i++) {
		ret = es9018_write_reg(g_es9018_priv->i2c_client,
				es9018_init_register_test[i].num,
				es9018_init_register_test[i].value);
	}
	__set_clk_div(es_sample_rate);
	pr_info("%s ---, ret:%d\n", __func__, ret);
	return ret;
}

static int es9018_close(void)
{
	pr_info("%s +++\n", __func__);
	power_gpio_0_L();
    clk_gpio_441k_L();
    clk_gpio_48k_L();
	reset_gpio_L();
	es9018_closed = true;

	pr_info("%s ---\n", __func__);
	return 0;
}

#if 0
static int es9018_get_i2s_length(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;

	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				INPUT_CONFIG_SOURCE);
	reg_val = reg_val >> 6;
	ucontrol->value.integer.value[0] = reg_val;

	pr_info("%s: i2s_length = 0x%x\n", __func__, reg_val);
	return 0;
}

static int es9018_set_i2s_length(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;

	pr_info("%s: ucontrol->value.integer.value[0]  = %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				INPUT_CONFIG_SOURCE);
	pr_info("zhougd--%s: reg_val = 0x%x\n", __func__, reg_val);
	reg_val &= ~(I2S_BIT_FORMAT_MASK);
	reg_val |=  ucontrol->value.integer.value[0] << 6;
	pr_info("zhougd--%s: reg_val = 0x%x\n", __func__, reg_val);
	es9018_write_reg(g_es9018_priv->i2c_client,
				INPUT_CONFIG_SOURCE, reg_val);
	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				INPUT_CONFIG_SOURCE);
	pr_info("zhougd--%s: reg_val = 0x%x\n", __func__, reg_val);
	return 0;
}
#endif

static int crystal_45M_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		__gpio_get_value(g_es9018_priv->es9018_data->clk_441k);

	return 0;
}

static int crystal_45M_set(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	int comp = ((struct soc_multi_mixer_control *)
		    kcontrol->private_value)->shift;

	if ( comp == CRYSTAL_45M) {
		if (ucontrol->value.integer.value[0] == 1) {
			pr_info("%s enter, enable 45M crystal\n", __func__);
			clk_gpio_441k_H();
			mdelay(10);
		} else {
			pr_info("%s enter, disable 45M crystal\n", __func__);
			clk_gpio_441k_L();
			mdelay(10);
		}
	}

	return 0;
}

static int crystal_49M_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		__gpio_get_value(g_es9018_priv->es9018_data->clk_48k);

	return 0;
}

static int crystal_49M_set(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	int comp = ((struct soc_multi_mixer_control *)
		    kcontrol->private_value)->shift;

	if ( comp == CRYSTAL_49M) {
		if (ucontrol->value.integer.value[0] == 1) {
			pr_info("%s enter, enable 49M crystal\n", __func__);
			clk_gpio_48k_H();
			mdelay(10);
		} else {
			pr_info("%s enter, disable 49M crystal\n", __func__);
			clk_gpio_48k_L();
			mdelay(10);
		}
	}

	return 0;
}

static int es9018_get_clk_divider(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;

	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				MASTER_MODE_CONTROL);
	reg_val = reg_val >> 5;
	ucontrol->value.integer.value[0] = reg_val;

	pr_info("%s: clk_div = 0x%x\n", __func__, reg_val);
	return 0;
}

static int es9018_set_clk_divider(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;

	pr_info("%s: ucontrol->value.integer.value[0]  = %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				MASTER_MODE_CONTROL);

	reg_val &= ~(I2S_CLK_DIVID_MASK);
	reg_val |=  ucontrol->value.integer.value[0] << 5;

	es9018_write_reg(g_es9018_priv->i2c_client,
				MASTER_MODE_CONTROL, reg_val);
	return 0;
}
#if 0
static int es9018_bypass_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		__gpio_get_value(g_es9018_priv->es9018_data->ldo_0);
	return 0;
}

static int es9018_bypass_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	if (ucontrol->value.integer.value[0] == 1) {
		pr_info("%s enter, line %d\n", __func__, __LINE__);
		power_gpio_0_H();
	} else {
		pr_info("%s enter, line %d\n", __func__, __LINE__);
		power_gpio_0_L();
	}
	return 0;
}

static int es9018_master_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				MASTER_MODE_CONTROL);
	pr_info("zhougd--%s enter, master state is 0x%x\n", __func__, reg_val);
	reg_val = reg_val & I2S_CLK_MODE_MASK;
	ucontrol->value.integer.value[0] = reg_val ? 1 : 0;
   
	return 0;
}

static int es9018_master_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				MASTER_MODE_CONTROL);
	reg_val &= ~(I2S_CLK_MODE_MASK);
	reg_val |=  ucontrol->value.integer.value[0]<<7;
	pr_info("zhougd--%s: reg_val 0x%x\n", __func__, reg_val);
	es9018_write_reg(g_es9018_priv->i2c_client,
				MASTER_MODE_CONTROL, reg_val);
	return 0;
}
#endif
static int es9018_softmute_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				GENERAL_SETTINGS_REG);
	pr_info("zhougd--%s enter, softmute state is 0x%x\n", __func__, reg_val);
	reg_val = reg_val & SOFT_MUTE_MASK;
	ucontrol->value.integer.value[0] = reg_val ? 1 : 0;
   
	return 0;
}
static int es9018_softmute_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				GENERAL_SETTINGS_REG);
	reg_val &= ~(SOFT_MUTE_MASK);
	reg_val |=  ucontrol->value.integer.value[0];
	reg_val |=  ucontrol->value.integer.value[0]<<1;
	pr_info("zhougd--%s: reg_val 0x%x\n", __func__, reg_val);
	es9018_write_reg(g_es9018_priv->i2c_client,
				GENERAL_SETTINGS_REG, reg_val);
	return 0;
}

static const char * const es9018_hifi_state_texts[] = {
	"Off", "On"
};

static const char * const es9018_i2s_length_texts[] = {
	"16bit", "24bit", "32bit", "32bit"
};

static const char * const es9018_clk_divider_texts[] = {
	"DIV4", "DIV8", "DIV16", "DIV16"
};

static const struct soc_enum es9018_hifi_state_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9018_hifi_state_texts),
		es9018_hifi_state_texts);

static const struct soc_enum es9018_i2s_length_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9018_i2s_length_texts),
		es9018_i2s_length_texts);

static const struct soc_enum es9018_clk_divider_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9018_clk_divider_texts),
		es9018_clk_divider_texts);

static struct snd_kcontrol_new es9018_digital_ext_snd_controls[] = {
	/* commit controls */
#if 0    
	SOC_ENUM_EXT("Es9018 I2s Length", es9018_i2s_length_enum,
			es9018_get_i2s_length, es9018_set_i2s_length),
	SOC_SINGLE_EXT("Es9018 Bypass Switch", SND_SOC_NOPM, 0, 0, 0,
			es9018_bypass_get, es9018_bypass_set),
        SOC_SINGLE_EXT("Es9018 Master", SND_SOC_NOPM, 0, 1, 0,
			es9018_master_get, es9018_master_set),
#endif
	SOC_ENUM_EXT("Es9018 CLK Divider", es9018_clk_divider_enum,
			es9018_get_clk_divider, es9018_set_clk_divider),
	SOC_SINGLE_EXT("Es9018 Crystal 45M Switch", SND_SOC_NOPM, CRYSTAL_45M,
			0, 0, crystal_45M_get, crystal_45M_set),
	SOC_SINGLE_EXT("Es9018 Crystal 49M Switch", SND_SOC_NOPM, CRYSTAL_49M,
			0, 0, crystal_49M_get, crystal_49M_set),
	SOC_SINGLE_EXT("Es9018 SoftMute State", SND_SOC_NOPM, 0, 1, 0,
			es9018_softmute_get, es9018_softmute_set),
	SOC_SINGLE_EXT("ESS_HEADPHONE", SND_SOC_NOPM, 0, 1, 0,                                                                                                                                                          
			ess_headphone_get, ess_headphone_put),
	SOC_DOUBLE_R_EXT_TLV("Es9018 Volume", LEFT_CHANNEL_VOLUME_15, RIGHT_CHANNEL_VOLUME_16, 0, 255, 1,
	        snd_soc_get_volsw, snd_soc_put_volsw, es9018_gain),
	SOC_DOUBLE_R_EXT_TLV("ESS Auto Volume", LEFT_CHANNEL_VOLUME_15, RIGHT_CHANNEL_VOLUME_16, 0, 255, 1,
	        ess_auto_volume_get, ess_auto_volume_put, es9018_gain),
	SOC_DOUBLE_R_EXT_TLV("ESS Fix Volume", LEFT_CHANNEL_VOLUME_15, RIGHT_CHANNEL_VOLUME_16, 0, 255, 1,
	        ess_headphone_fix_volume_get, ess_headphone_fix_volume_put, es9018_gain),
};

static int es9018_read_reg(struct i2c_client *client, int reg)
{
	int ret;
    if (es9018_closed) {
        pr_err("%s es9018 not powered, skip\n", __func__);
        return -1;
    } else {
        ret = i2c_smbus_read_byte_data(client, reg);
        if (ret < 0)
        	dev_err(&client->dev, "%s: reg %d err %d\n", __func__, reg, ret);
    }
	return ret;
}

static int es9018_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;
	
    if(es9018_closed) {
        pr_err("%s es9018 not powered, skip\n", __func__);
        return -1;
    } else {
        pr_info("zhougd--%s reg %d val 0x%x\n",__func__,reg,value);
        ret = i2c_smbus_write_byte_data(client, reg, value);
        if (ret < 0)
        	dev_err(&client->dev, "%s: err %d\n", __func__, ret);
    }
	return ret;
}

static int es9018_populate_get_pdata(struct device *dev,
		struct es9018_data *pdata)
{
	int ret;
	struct pinctrl *pinctrl;
	pr_info("zhougd--%s enter",__func__);

	
	pdata->reset_gpio = of_get_named_gpio(dev->of_node,
			"es9018,reset-gpio", 0);
	if (pdata->reset_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,reset-gpio", dev->of_node->full_name,
				pdata->reset_gpio);
		goto err;
	}
	pr_info("zhougd--%s: reset gpio %d", __func__, pdata->reset_gpio);
	ret = gpio_request(pdata->reset_gpio, "es9018_reset");
	if (ret < 0) {
		dev_err(dev, "%s(): es9018_reset request failed",
				__func__);
		goto reset_gpio_request_error;
	}
	ret = gpio_direction_output(pdata->reset_gpio, 0);
	if (ret < 0) {
		dev_err(dev, "%s(): es9018_reset set failed",
				__func__);
		goto reset_gpio_request_error;
	}

	pdata->clk_441k = of_get_named_gpio(dev->of_node,
			"es9018,es_45m_en", 0);
	if (pdata->clk_441k < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,es_45m_en", dev->of_node->full_name,
				pdata->clk_441k);
		goto es_45m_en_gpio_request_error;
	}
	pr_info("zhougd--%s: es_45m_en gpio %d", __func__, pdata->clk_441k);
	ret = gpio_request(pdata->clk_441k, "es_45m_en");
	if (ret < 0) {
		dev_err(dev, "%s(): es_45m_en request failed",
				__func__);
		goto es_45m_en_gpio_request_error;
	}
	ret = gpio_direction_output(pdata->clk_441k, 0);
	if (ret < 0) {
		dev_err(dev, "%s(): es_45m_en set failed",
				__func__);
		goto es_45m_en_gpio_request_error;
	}

	pdata->clk_48k = of_get_named_gpio(dev->of_node,
			"es9018,es_49m_en", 0);
	if (pdata->clk_48k < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,es_49m_en", dev->of_node->full_name,
				pdata->clk_48k);
		goto es_49m_en_gpio_request_error;
	}
	pr_info("zhougd--%s: es_49m_en gpio %d", __func__, pdata->clk_48k);
	ret = gpio_request(pdata->clk_48k, "clk_441k");
	if (ret < 0) {
		dev_err(dev, "%s(): es_49m_en request failed",
				__func__);
		goto es_49m_en_gpio_request_error;
	}
	ret = gpio_direction_output(pdata->clk_48k, 0);
	if (ret < 0) {
		dev_err(dev, "%s(): es_49m_en set failed",
				__func__);
		goto es_49m_en_gpio_request_error;
	}

	pdata->es_hifi_pa_pwr_en = of_get_named_gpio(dev->of_node,
			"es9018,es_hifi_pa_pwr_en-gpio", 0);
	if (pdata->es_hifi_pa_pwr_en < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,es_hifi_pa_pwr_en-gpio", dev->of_node->full_name,
				pdata->es_hifi_pa_pwr_en);
		goto es_hifi_pa_pwr_en_gpio_request_error;
	}
	pr_info("zhougd--%s: es_hifi_pa_pwr_en gpio %d", __func__, pdata->es_hifi_pa_pwr_en);
	ret = gpio_request(pdata->es_hifi_pa_pwr_en, "es_hifi_pa_pwr_en");
	if (ret < 0) {
		dev_err(dev, "%s(): es_hifi_pa_pwr_en request failed",
				__func__);
		goto es_hifi_pa_pwr_en_gpio_request_error;
	}
	ret = gpio_direction_output(pdata->es_hifi_pa_pwr_en, 0);
	if (ret < 0) {
		dev_err(dev, "%s(): es_hifi_pa_pwr_en set failed",
				__func__);
		goto es_hifi_pa_pwr_en_gpio_request_error;
	}
#if 0
	pdata->hifi_sw_pwr_en = of_get_named_gpio(dev->of_node,
			"es9018,hifi_sw_pwr_en-gpio", 0);
	if (pdata->hifi_sw_pwr_en < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,hifi_sw_pwr_en-gpio", dev->of_node->full_name,
				pdata->hifi_sw_pwr_en);
		goto hifi_sw_pwr_en_gpio_request_error;
	}
	pr_info("zhougd--%s: hifi_sw_pwr_en gpio %d", __func__, pdata->hifi_sw_pwr_en);
	ret = gpio_request(pdata->hifi_sw_pwr_en, "hifi_sw_pwr_en");
	if (ret < 0) {
		dev_err(dev, "%s(): hifi_sw_pwr_en request failed",
				__func__);
		goto hifi_sw_pwr_en_gpio_request_error;
	}
	ret = gpio_direction_output(pdata->hifi_sw_pwr_en, 1);
	if (ret < 0) {
		dev_err(dev, "%s(): hifi_sw_pwr_en set failed",
				__func__);
		goto hifi_sw_pwr_en_gpio_request_error;
	}

	pdata->hifi_switch_sel = of_get_named_gpio(dev->of_node,
			"es9018,hifi_switch_sel-gpio", 0);
	if (pdata->hifi_switch_sel < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,hifi_switch_sel-gpio", dev->of_node->full_name,
				pdata->hifi_switch_sel);
		goto hifi_switch_sel_gpio_request_error;
	}
	pr_info("zhougd--%s: hifi_switch_sel gpio %d", __func__, pdata->hifi_switch_sel);
	ret = gpio_request(pdata->hifi_switch_sel, "hifi_switch_sel");
	if (ret < 0) {
		dev_err(dev, "%s(): hifi_switch_sel request failed",
				__func__);
		goto hifi_switch_sel_gpio_request_error;
	}
	ret = gpio_direction_output(pdata->hifi_switch_sel, 0);
	if (ret < 0) {
		dev_err(dev, "%s(): hifi_switch_sel set failed",
				__func__);
		goto hifi_switch_sel_gpio_request_error;
	}

	pdata->hifi_switch_mute = of_get_named_gpio(dev->of_node,
			"es9018,hifi_switch_mute-gpio", 0);
	if (pdata->hifi_switch_sel < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,hifi_switch_mute-gpio", dev->of_node->full_name,
				pdata->hifi_switch_mute);
		goto hifi_switch_mute_gpio_request_error;
	}
	pr_info("zhougd--%s: hifi_switch_mute gpio %d", __func__, pdata->hifi_switch_mute);
	ret = gpio_request(pdata->hifi_switch_mute, "hifi_switch_mute");
	if (ret < 0) {
		dev_err(dev, "%s(): hifi_switch_mute request failed",
				__func__);
		goto hifi_switch_mute_gpio_request_error;
	}
	ret = gpio_direction_output(pdata->hifi_switch_mute, 0);
	if (ret < 0) {
		dev_err(dev, "%s(): hifi_switch_mute set failed",
				__func__);
		goto hifi_switch_mute_gpio_request_error;
	}	
#endif	
	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl)) {
		pr_err("%s: Unable to get pinctrl handle\n", __func__);
		return -EINVAL;
	}
	es9018_ldo_pinctrl_info.pinctrl = pinctrl;
	es9018_ldo_pinctrl_info.es9018_ldo_sus = pinctrl_lookup_state(pinctrl,
						"es9018_ldo_sus");
	if (IS_ERR(es9018_ldo_pinctrl_info.es9018_ldo_sus)) {
		pr_err("%s: Unable to get pinctrl disable handle\n",
							  __func__);
		return -EINVAL;
	}
	es9018_ldo_pinctrl_info.es9018_ldo_act = pinctrl_lookup_state(pinctrl,
						"es9018_ldo_act");
	if (IS_ERR(es9018_ldo_pinctrl_info.es9018_ldo_act)) {
		pr_err("%s: Unable to get pinctrl active handle\n",
							 __func__);
		return -EINVAL;
	}

	return 0;
#if 0
hifi_switch_mute_gpio_request_error:
	gpio_free(pdata->hifi_switch_mute);
	return ret;
hifi_switch_sel_gpio_request_error:
	gpio_free(pdata->hifi_switch_sel);
	return ret;
hifi_sw_pwr_en_gpio_request_error:
	gpio_free(pdata->hifi_sw_pwr_en);
	return ret;
#endif
es_hifi_pa_pwr_en_gpio_request_error:
	gpio_free(pdata->es_hifi_pa_pwr_en);
	return ret;
es_49m_en_gpio_request_error:
	gpio_free(pdata->clk_441k);
	return ret;
es_45m_en_gpio_request_error:
	gpio_free(pdata->clk_48k);
	return ret;
reset_gpio_request_error:
	gpio_free(pdata->reset_gpio);
	return ret;
err:
	devm_kfree(dev, pdata);
	return -1;
}

static unsigned int es9018_codec_read(struct snd_soc_codec *codec,
		unsigned int reg)
{
	struct es9018_priv *priv = codec->control_data;
	pr_info("%s reg %d",__func__,reg);
    if (priv->malfunction)
        return -EIO;
	return es9018_read_reg(priv->i2c_client, reg);
}

static int es9018_codec_write(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value)
{
	struct es9018_priv *priv = codec->control_data; 
	pr_info("%s reg %d val %d",__func__,reg,value);
    if (priv->malfunction)
        return -EIO;
	return es9018_write_reg(priv->i2c_client, reg, value);
}

static int es9018_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{
	int ret = 0;

	dev_dbg(codec->dev, "%s(codec, level = 0x%04x): entry\n",
			__func__, level);

	switch (level) {
		case SND_SOC_BIAS_ON:
			break;

		case SND_SOC_BIAS_PREPARE:
			break;

		case SND_SOC_BIAS_STANDBY:
			break;

		case SND_SOC_BIAS_OFF:
			break;
	}
	codec->dapm.bias_level = level;

	return ret;
}

static int es9018_suspend(struct snd_soc_codec *codec)
{
    pr_info("zhougd--%s \n",__func__);
    if (!g_es9018_priv->work_stop_request) {
        //the work queue is still active for disable codec,we have to stop queue and close codec before suspend
        pr_info("zhougd--%s close 9018\n",__func__);
        g_es9018_priv->work_stop_request = true;
        es9018_close();
        gpio_set_value(g_es9018_priv->es9018_data->es_hifi_pa_pwr_en,0);
    }
	es9018_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int es9018_resume(struct snd_soc_codec *codec)
{
    pr_info("zhougd--%s \n",__func__);
	es9018_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

static int es9018_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *codec_dai)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */
    pr_info("zhougd--%s \n",__func__);
	return 0;
}

static int es9018_mute(struct snd_soc_dai *dai, int mute)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */

	return 0;
}

static int es9018_set_clkdiv(struct snd_soc_dai *codec_dai,
				int div_id, int div)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */
    pr_info("zhougd--%s \n",__func__);
	return 0;
}

static int es9018_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */
    pr_info("zhougd--%s \n",__func__);
	return 0;
}


static int es9018_set_dai_fmt(struct snd_soc_dai *codec_dai,
				unsigned int fmt)
{
	/*struct snd_soc_codec *codec = codec_dai->codec; 
	struct es9018_priv *priv = codec->control_data;*/
    pr_info("zhougd--%s sample_rate %d \n",__func__,fmt);
    es_sample_rate = fmt;
    ess_headphones_ena(1);
	return 0;
}

static int es9018_set_fll(struct snd_soc_dai *codec_dai,
		int pll_id, int source, unsigned int freq_in,
		unsigned int freq_out)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */
    pr_info("zhougd--%s \n",__func__);
	return 0;
}

static int es9018_pcm_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *codec_dai)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */
    pr_info("zhougd--%s \n",__func__);
	return 0;
}

static const struct snd_soc_dai_ops es9018_dai_ops = {
	.hw_params	= es9018_pcm_hw_params,
	.digital_mute	= es9018_mute,
	.trigger	= es9018_pcm_trigger,
	.set_fmt	= es9018_set_dai_fmt,
	.set_sysclk	= es9018_set_dai_sysclk,
	.set_pll	= es9018_set_fll,
	.set_clkdiv	= es9018_set_clkdiv,
};

static struct snd_soc_dai_driver es9018_dai = {
	.name = "es9018-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ES9018_RATES,
		.formats = ES9018_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ES9018_RATES,
		.formats = ES9018_FORMATS,
	},
	.ops = &es9018_dai_ops,
};

static  int es9018_codec_probe(struct snd_soc_codec *codec)
{
	int rc = 0;
	
	struct es9018_priv *priv = snd_soc_codec_get_drvdata(codec);

	pr_debug("zhougd--%s: enter\n", __func__);

	priv->codec = codec;

	codec->control_data = snd_soc_codec_get_drvdata(codec);

	es9018_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

    if (!priv->malfunction) {
        rc = snd_soc_add_codec_controls(codec, es9018_digital_ext_snd_controls,
                ARRAY_SIZE(es9018_digital_ext_snd_controls));
        if (rc)
            dev_err(codec->dev, "%s(): es325_digital_snd_controls failed\n",
                __func__);
    }
	INIT_DELAYED_WORK(&g_es9018_priv->close_es9018_dwork, close_es9018_fn);

	return 0;
}

static int  es9018_codec_remove(struct snd_soc_codec *codec)
{
	struct es9018_priv *priv = snd_soc_codec_get_drvdata(codec);

	es9018_set_bias_level(codec, SND_SOC_BIAS_OFF);

	kfree(priv);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_es9018 = {
	.probe = es9018_codec_probe,
	.remove = es9018_codec_remove,
	.suspend = es9018_suspend,
	.resume = es9018_resume,
	.read = es9018_codec_read,
	.write = es9018_codec_write,
//	.set_bias_level = es9018_set_bias_level,
};

static int es9018_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct es9018_priv *priv;
	struct es9018_data *pdata;
	int ret = 0;

	pr_info("zhougd--%s enter, es9018_probe\n", __func__);
	
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct es9018_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = es9018_populate_get_pdata(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "Parsing DT failed(%d)", ret);
			return ret;
		}

	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct es9018_priv),
			GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "%s: no support for i2c read/write"
				"byte data\n", __func__);
        priv->malfunction = true;
//		return -EIO;
	}
	priv->i2c_client = client;
	priv->es9018_data = pdata;
	i2c_set_clientdata(client, priv);

	g_es9018_priv = priv;
	g_es9018_priv->work_stop_request = true;	

    if (!priv->malfunction) {
    	ret = es9018_open();
        if (ret < 0)
            priv->malfunction = true;
    	ret = es9018_read_reg(g_es9018_priv->i2c_client,64);
    	if (ret < 0) {
    		pr_info("%s: read es9018 chip status fail",__func__);
            priv->malfunction = true;
//    		return ret;
    	}
    	pr_info("zhougd2--%s: chip status is 0x%x",__func__,ret);
    	es9018_chip_id = ret & 0x1c;
    	es9018_close();
    }
	if (client->dev.of_node)
		dev_set_name(&client->dev, "%s", "es9018-codec");

	ret = snd_soc_register_codec(&client->dev, &soc_codec_dev_es9018,
			&es9018_dai, 1);

	return ret;
}

static int es9018_chip_status_get(char *val, struct kernel_param *kp)
{
	pr_info("zhougd2--%s: chip status is 0x%x",__func__,es9018_chip_id);
	return sprintf(val, "0x%x",es9018_chip_id);
}
module_param_call(es9018_chip_id,NULL, es9018_chip_status_get,
        &es9018_chip_id, 0644);

static int es9018_remove(struct i2c_client *client)
{
	return 0;
}

static struct of_device_id es9018_match_table[] = {
	{ .compatible = "dac,es9018-codec", },
	{}
};

static const struct i2c_device_id es9018_id[] = {
	{ "es9018", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, isa1200_id);

static struct i2c_driver es9018_i2c_driver = {
	.driver	= {
		.name	= "es9018-codec",
		.of_match_table = es9018_match_table,
	},
	.probe		= es9018_probe,
	.remove		= es9018_remove,
	.id_table	= es9018_id,
};

static int __init es9018_init(void)
{
return i2c_add_driver(&es9018_i2c_driver);
}

static void __exit es9018_exit(void)
{
	i2c_del_driver(&es9018_i2c_driver);
}

module_init(es9018_init);
module_exit(es9018_exit);

MODULE_DESCRIPTION("ASoC ES9018 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:es9018-codec");
