/*
 * es8323.c -- es8323 ALSA SoC audio driver
 *
 * Copyright 2009 Wolfson Microelectronics plc
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
//#define DEBUG 0
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include "es8323.h"

#define DEBUG_REG	(0)
#define SPK_CON 	RK30_PIN2_PD7
#define HP_DET          RK30_PIN2_PD4

static void det_function(struct work_struct *work);
static irqreturn_t det_handler(int irq, void *dev_id);
static DECLARE_DELAYED_WORK(det_work, det_function);
#define BIT_HEADSET_NO             (0 << 0)
#define BIT_HEADSET_NO_MIC      (1 << 1)
static int cur_headset = BIT_HEADSET_NO;
	
static u16 es8323_reg[] = {
	0x06, 0x1C, 0xC3, 0xFC,  /*  0 */
	0xC0, 0x00, 0x00, 0x7C,  /*  4 */
	0x80, 0x00, 0x00, 0x06,  /*  8 */
	0x00, 0x06, 0x30, 0x30,  /* 12 */
	0xC0, 0xC0, 0x38, 0xB0,  /* 16 */
	0x32, 0x06, 0x00, 0x00,  /* 20 */
	0x06, 0x30, 0xC0, 0xC0,  /* 24 */
	0x08, 0x06, 0x1F, 0xF7,  /* 28 */
	0xFD, 0xFF, 0x1F, 0xF7,  /* 32 */
	0xFD, 0xFF, 0x00, 0x38,  /* 36 */
	0x38, 0x38, 0x38, 0x38,  /* 40 */
	0x38, 0x00, 0x00, 0x00,  /* 44 */
	0x00, 0x00, 0x00, 0x00,  /* 48 */
	0x00, 0x00, 0x00, 0x00,  /* 52 */
};

struct es8323_priv {
	unsigned int sysclk;
	bool det_initalized;
	enum snd_soc_control_type control_type;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
};
struct snd_soc_codec *es8323_codec;

static unsigned int es8323_read(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	if (reg >= ARRAY_SIZE(es8323_reg)) {
		dev_err(codec->dev,"fail to read reg=0x%2x\n", reg);
		return -1;
	}
	
	return es8323_reg[reg];
}

static int es8323_write(struct snd_soc_codec *codec, unsigned int reg,
			     unsigned int value)
{
	u8 data[2];
	int ret;

	data[0] = reg;
	data[1] = value & 0x00ff;

	if (reg < ARRAY_SIZE(es8323_reg))
		es8323_reg[reg] = value;
	
	ret = codec->hw_write(codec->control_data, data, 2);
	if (ret == 2)
		return 0;

	dev_err(codec->dev,"fail to write reg=0x%2x, value=0x%2x\n", reg, value);
	if (ret < 0)
		return ret;
	else
		return -EIO;
}

 static int es8323_reset(struct snd_soc_codec *codec)
 {
 	return 0;
	snd_soc_write(codec, ES8323_CONTROL1, 0x80);
	return snd_soc_write(codec, ES8323_CONTROL1, 0x00);
 }

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:4;
	u8 usb:1;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{12288000, 8000, 1536, 0xa, 0x0},
	{11289600, 8000, 1408, 0x9, 0x0},
	{18432000, 8000, 2304, 0xc, 0x0},
	{16934400, 8000, 2112, 0xb, 0x0},
	{12000000, 8000, 1500, 0xb, 0x1},

	/* 11.025k */
	{11289600, 11025, 1024, 0x7, 0x0},
	{16934400, 11025, 1536, 0xa, 0x0},
	{12000000, 11025, 1088, 0x9, 0x1},

	/* 16k */
	{12288000, 16000, 768, 0x6, 0x0},
	{18432000, 16000, 1152, 0x8, 0x0},
	{12000000, 16000, 750, 0x7, 0x1},

	/* 22.05k */
	{11289600, 22050, 512, 0x4, 0x0},
	{16934400, 22050, 768, 0x6, 0x0},
	{12000000, 22050, 544, 0x6, 0x1},

	/* 32k */
	{12288000, 32000, 384, 0x3, 0x0},
	{18432000, 32000, 576, 0x5, 0x0},
	{12000000, 32000, 375, 0x4, 0x1},

	/* 44.1k */
	{11289600, 44100, 256, 0x2, 0x0},
	{16934400, 44100, 384, 0x3, 0x0},
	{12000000, 44100, 272, 0x3, 0x1},

	/* 48k */
	{12288000, 48000, 256, 0x2, 0x0},
	{18432000, 48000, 384, 0x3, 0x0},
	{12000000, 48000, 250, 0x2, 0x1},

	/* 88.2k */
	{11289600, 88200, 128, 0x0, 0x0},
	{16934400, 88200, 192, 0x1, 0x0},
	{12000000, 88200, 136, 0x1, 0x1},

	/* 96k */
	{12288000, 96000, 128, 0x0, 0x0},
	{18432000, 96000, 192, 0x1, 0x0},
	{12000000, 96000, 125, 0x0, 0x1},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}

	return -EINVAL;
}

/* The set of rates we can generate from the above for each SYSCLK */

static unsigned int rates_12288[] = {
	8000, 12000, 16000, 24000, 24000, 32000, 48000, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12288 = {
	.count	= ARRAY_SIZE(rates_12288),
	.list	= rates_12288,
};

static unsigned int rates_112896[] = {
	8000, 11025, 22050, 44100,
};

static struct snd_pcm_hw_constraint_list constraints_112896 = {
	.count	= ARRAY_SIZE(rates_112896),
	.list	= rates_112896,
};

static unsigned int rates_12[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
	48000, 88235, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12 = {
	.count	= ARRAY_SIZE(rates_12),
	.list	= rates_12,
};

static int hdmi_work_state = 0;
static void es8323_off_amp(bool on)
{	
	struct snd_soc_codec *codec = es8323_codec;
	int newlevel = (on) ? GPIO_LOW : GPIO_HIGH;
	int oldlevel = gpio_get_value(SPK_CON);

	if (hdmi_work_state == 1) {
		dev_dbg(codec->dev, "HDMI work %s\n", hdmi_work_state ? "on" : "off");
		return;
	}
	
	if (oldlevel == newlevel)
		return;

	gpio_direction_output(SPK_CON, newlevel);
	/* Fix bug
	 * Cause: Amplifier chip's delay will decay output */
	mdelay(20);
	dev_info(codec->dev, "%s %d\n", __func__, on);
}

void codec_set_spk(bool on)
{
	struct snd_soc_codec *codec;
	
	if (!es8323_codec)
		goto exit;

	codec = es8323_codec;
	if (on) {
		hdmi_work_state = 0;	
		if(codec->dapm.bias_level == SND_SOC_BIAS_ON && cur_headset == BIT_HEADSET_NO )
			gpio_direction_output(SPK_CON, GPIO_HIGH);
	} else {
		gpio_direction_output(SPK_CON, GPIO_LOW);
		hdmi_work_state = 1;
	}

exit:
	return ;
}

EXPORT_SYMBOL_GPL(codec_set_spk);

static int es8323_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct es8323_priv *es8323 = snd_soc_codec_get_drvdata(codec);

	switch (freq) {
	case 11289600:
	case 18432000:
	case 22579200:
	case 36864000:
		es8323->sysclk_constraints = &constraints_112896;
		es8323->sysclk = freq;
		return 0;

	case 12288000:
	case 16934400:
	case 24576000:
	case 33868800:
		es8323->sysclk_constraints = &constraints_12288;
		es8323->sysclk = freq;
		return 0;

	case 12000000:
	case 24000000:
		es8323->sysclk_constraints = &constraints_12;
		es8323->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

static int es8323_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 iface = 0;
	u8 adciface = 0;
	u8 daciface = 0;

	iface    = snd_soc_read(codec, ES8323_IFACE);
	adciface = snd_soc_read(codec, ES8323_ADC_IFACE);
	daciface = snd_soc_read(codec, ES8323_DAC_IFACE);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= 0x80;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		iface &= 0x7F;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		adciface &= 0xFC;
		daciface &= 0xF9;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		break;
	case SND_SOC_DAIFMT_DSP_B:
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		iface    &= 0xDF;
		adciface &= 0xDF;
		daciface &= 0xBF;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface    |= 0x20;
		adciface |= 0x20;
		daciface |= 0x40;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface    |= 0x20;
		adciface &= 0xDF;
		daciface &= 0xBF;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface    &= 0xDF;
		adciface |= 0x20;
		daciface |= 0x40;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, ES8323_IFACE, iface);
	snd_soc_write(codec, ES8323_ADC_IFACE, adciface);
	snd_soc_write(codec, ES8323_DAC_IFACE, daciface);
	return 0;
}

static int es8323_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{ 
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct es8323_priv *es8323 = snd_soc_codec_get_drvdata(codec);
	u16 srate = snd_soc_read(codec, ES8323_IFACE) & 0x80;
	u16 adciface = snd_soc_read(codec, ES8323_ADC_IFACE) & 0xE3;
	u16 daciface = snd_soc_read(codec, ES8323_DAC_IFACE) & 0xC7;
	int coeff;

	coeff = get_coeff(es8323->sysclk, params_rate(params));
	if (coeff < 0) {
		coeff = get_coeff(es8323->sysclk / 2, params_rate(params));
		srate |= 0x40;
	}
	if (coeff < 0) {
		dev_err(codec->dev,
			"Unable to configure sample rate %dHz with %dHz MCLK\n",
			params_rate(params), es8323->sysclk);
		return coeff;
	}

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		adciface |= 0x000C;
		daciface |= 0x0018;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		adciface |= 0x0004;
		daciface |= 0x0008;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		adciface |= 0x0010;
		daciface |= 0x0020;
		break;
	}

	/* set iface & srate*/
	snd_soc_write(codec, ES8323_DAC_IFACE, daciface);
	snd_soc_write(codec, ES8323_ADC_IFACE, adciface);

	if (coeff >= 0) {
		snd_soc_write(codec, ES8323_IFACE, srate);
		snd_soc_write(codec, ES8323_ADCCONTROL5, coeff_div[coeff].sr | (coeff_div[coeff].usb) << 4);
		snd_soc_write(codec, ES8323_DACCONTROL2, coeff_div[coeff].sr | (coeff_div[coeff].usb) << 4);
	}

	return 0;
}

static int es8323_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_dbg(codec->dev, "%s %d\n", __func__, mute);
	if (mute) {
		es8323_off_amp(true);
		snd_soc_write(codec, ES8323_DACCONTROL3, 0x06);
	} else {
		if (dai->playback_active) {
			snd_soc_write(codec, ES8323_DACCONTROL3, 0x02);
			if (cur_headset == BIT_HEADSET_NO) {
				snd_soc_write(codec, ES8323_DACCONTROL4, 0x00);
				snd_soc_write(codec, ES8323_DACCONTROL5, 0x02);
				es8323_off_amp(false);
			}else{
				snd_soc_write(codec, ES8323_DACCONTROL4, 0x08);
				snd_soc_write(codec, ES8323_DACCONTROL5, 0x08);
			}
		}
	}
	return 0;
}

static int es8323_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{        
	switch (level) {
	case SND_SOC_BIAS_ON:
		dev_dbg(codec->dev, "%s on\n", __func__);
		break;
	case SND_SOC_BIAS_PREPARE:
		dev_dbg(codec->dev, "%s prepare\n", __func__);
		snd_soc_write(codec, ES8323_ANAVOLMANAG, 0x7C);
		snd_soc_write(codec, ES8323_CHIPLOPOW1, 0x00);
		snd_soc_write(codec, ES8323_CHIPLOPOW2, 0x00);							
		snd_soc_write(codec, ES8323_ADCPOWER, 0x00);
		break;
	case SND_SOC_BIAS_STANDBY:
		dev_dbg(codec->dev, "%s standby\n", __func__);
		snd_soc_write(codec, ES8323_ADCPOWER, 0xFF);
		snd_soc_write(codec, ES8323_CHIPLOPOW1, 0xFF);
		snd_soc_write(codec, ES8323_CHIPLOPOW2, 0xFF);
		snd_soc_write(codec, ES8323_ANAVOLMANAG, 0x7B);
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF)
		{
			snd_soc_write(codec, ES8323_CHIPPOWER, 0x00);
			snd_soc_write(codec, ES8323_DACPOWER, 0x2C);
		}
		break;
	case SND_SOC_BIAS_OFF:
		dev_dbg(codec->dev, "%s off\n", __func__);
		snd_soc_write(codec, ES8323_ADCPOWER, 0xFF);
		snd_soc_write(codec, ES8323_DACPOWER, 0xC0);
		snd_soc_write(codec, ES8323_CHIPLOPOW1, 0xFF);
		snd_soc_write(codec, ES8323_CHIPLOPOW2, 0xFF);
		snd_soc_write(codec, ES8323_CHIPPOWER, 0xFF);
		snd_soc_write(codec, ES8323_ANAVOLMANAG, 0x7B);
		break;
	}
	codec->dapm.bias_level = level;
	
	return 0;
}


#define es8323_RATES SNDRV_PCM_RATE_8000_96000
#define es8323_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
							SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops es8323_ops = {
	.set_fmt = es8323_set_dai_fmt,
	.set_sysclk = es8323_set_dai_sysclk,
	.hw_params = es8323_hw_params,
	.digital_mute = es8323_digital_mute,
};

static struct snd_soc_dai_driver es8323_dai = {
	.name = "ES8323 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es8323_RATES,
		.formats = es8323_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es8323_RATES,
		.formats = es8323_FORMATS,
	 },
	.ops = &es8323_ops,
	.symmetric_rates = 1,
};

static bool det_initalized = false;
static void det_function(struct work_struct *work)
{
	struct snd_soc_codec *codec;
	unsigned long flags;
	int hp_gpio,irq;
	int state = BIT_HEADSET_NO;
	int ret;

	if (!es8323_codec)
		goto exit;

	codec = es8323_codec;
	hp_gpio = gpio_get_value(HP_DET);
	if (hp_gpio == GPIO_LOW) {
		state |= BIT_HEADSET_NO_MIC;
	}

	if (cur_headset != state){
		cur_headset = state;
		rk_headset_report(state);
		dev_info(codec->dev, "%s state = %d\n", __func__, state);
	}
	
	irq = gpio_to_irq(HP_DET);
	if (det_initalized)
		free_irq(irq, NULL);
	
	if (hp_gpio != gpio_get_value(HP_DET)) {
		schedule_delayed_work(&det_work, msecs_to_jiffies(200));
		det_initalized = false;
		pr_warning("Plug operation happened\n");
		goto exit;
	}
	if (hp_gpio == GPIO_HIGH) {
		snd_soc_write(codec, ES8323_DACCONTROL4, 0x00);
		snd_soc_write(codec, ES8323_DACCONTROL5, 0x01);
		
		snd_soc_write(codec, 0x1e, 0x25);
		snd_soc_write(codec, 0x1f, 0x5b);
		snd_soc_write(codec, 0x20, 0xe5);
		snd_soc_write(codec, 0x21, 0xdf);
		snd_soc_write(codec, 0x22, 0x08);
		snd_soc_write(codec, 0x23, 0x74);
		snd_soc_write(codec, 0x24, 0x6d);
		snd_soc_write(codec, 0x25, 0xbe);

		
	} else {//headphone in
		snd_soc_write(codec, 0x1e, 0x1f);
		snd_soc_write(codec, 0x1f, 0xf7);
		snd_soc_write(codec, 0x20, 0xfd);
		snd_soc_write(codec, 0x21, 0xff);
		snd_soc_write(codec, 0x22, 0x1f);
		snd_soc_write(codec, 0x23, 0xf7);
		snd_soc_write(codec, 0x24, 0xfd);
		snd_soc_write(codec, 0x25, 0xff);
		snd_soc_write(codec, ES8323_DACCONTROL4, 0x08);
		snd_soc_write(codec, ES8323_DACCONTROL5, 0x08);
	}
	flags = (hp_gpio == GPIO_HIGH) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	ret = request_irq(irq, det_handler, flags, "hp_det", NULL);
	if (ret < 0) {
		pr_err("request_irq(%d) failed\n", irq);
		goto exit;
	}
	det_initalized = true;

exit:
	return;
}

static irqreturn_t det_handler(int irq, void *dev_id)
{
	disable_irq_nosync(irq);
	schedule_delayed_work(&det_work, msecs_to_jiffies(300));
	return IRQ_HANDLED;
}

static void det_initalize(void)
{
#if !defined(CONFIG_RK_HEADSET_DET)
	if (gpio_request(HP_DET, "hp_det")) {
		pr_err("%s %d request error", __func__, __LINE__);
		return;
	}
	gpio_pull_updown(HP_DET, PullDisable);
	gpio_direction_input(HP_DET);
#endif
	det_initalized = false;
	schedule_delayed_work(&det_work, msecs_to_jiffies(100));
	return;
}

static int es8323_init_regs(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s\n", __func__);

	/* Power setting */
	snd_soc_write(codec, ES8323_MASTERMODE,0x00);  //ES8388 salve
	snd_soc_write(codec, ES8323_CHIPPOWER, 0x00);
	snd_soc_write(codec, ES8323_CONTROL1,0x36);
	snd_soc_write(codec, ES8323_CONTROL2,0x50);
	snd_soc_write(codec, ES8323_DACCONTROL21, 0x80);

	/* Clocks setting */
	snd_soc_write(codec, ES8323_ADCCONTROL5, 0x02);
	snd_soc_write(codec, ES8323_DACCONTROL1, 0x18);
	snd_soc_write(codec, ES8323_DACCONTROL2, 0x02);

	/* ADC setting */
	snd_soc_write(codec, ES8323_ADCCONTROL1,0xaa);  //ADC L/R PGA =  dB
	snd_soc_write(codec, ES8323_ADCCONTROL2,0xf0);  //ADC INPUT=LIN2/RIN2
	snd_soc_write(codec, ES8323_ADCCONTROL3,0x82);  //ADC INPUT=LIN2/RIN2
	snd_soc_write(codec, ES8323_ADCCONTROL4,0x4c);  //left data = left ADC, right data = left ADC, 16BIT
	snd_soc_write(codec, ES8323_ADCCONTROL8,0x00);  //LADCVOL = 0 dB
	snd_soc_write(codec, ES8323_ADCCONTROL9,0x00);  //RADCVOL = 0 dB

	/* ALC setting */
	snd_soc_write(codec, ES8323_ADCCONTROL10,0xea);  //ALC stereo, MAXGAIN = 23.5dB,  MINGAIN = 0dB
	snd_soc_write(codec, ES8323_ADCCONTROL11,0xc0);  //ALC target = -1.5 dB, hold time = 0 ms
	snd_soc_write(codec, ES8323_ADCCONTROL12,0x56);  //ALC decay time and attack time
	snd_soc_write(codec, ES8323_ADCCONTROL13,0x06);  //ALC mode(default)
	snd_soc_write(codec, ES8323_ADCCONTROL14,0x53);  //noise gate set = -61.5 dBFS, noise gate type = mute ADC, noise gate enable

	/* DAC setting */
	snd_soc_write(codec, ES8323_DACCONTROL4, 0x08); //LOL volume
	snd_soc_write(codec, ES8323_DACCONTROL5, 0x08); //LOR volume
	snd_soc_write(codec, ES8323_DACPOWER, 0x2C); //DAC power up
	snd_soc_write(codec, ES8323_DACCONTROL17,0xb8);  //left DAC to left mixer enable
	snd_soc_write(codec, ES8323_DACCONTROL20,0xb8);  //right DAC to right mixer enable
	snd_soc_write(codec, ES8323_DACCONTROL3,0x06);  //SOFT RAMP RATE=32LRCKS/STEP, DAC MUTE
	snd_soc_write(codec, ES8323_DACCONTROL26,0x1e);  //LOUT1 volume = 0 dB
	snd_soc_write(codec, ES8323_DACCONTROL27,0x1e);  //ROUT1 volume = 0 dB
	
	return 0;
}

static int es8323_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	es8323_off_amp(true);
	snd_soc_write(codec, ES8323_CONTROL2, 0x58);
	snd_soc_write(codec, ES8323_CONTROL1, 0x00);
	snd_soc_write(codec, ES8323_DACCONTROL21, 0x9C);
	es8323_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int es8323_resume(struct snd_soc_codec *codec)
{
	snd_soc_write(codec, ES8323_DACCONTROL21, 0x80);
	snd_soc_write(codec, ES8323_CONTROL1,0x36);
	snd_soc_write(codec, ES8323_CONTROL2,0x50);
	es8323_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	schedule_delayed_work(&det_work, msecs_to_jiffies(0));
	return 0;
}

static int es8323_probe(struct snd_soc_codec *codec)
{
	int ret = 0;

	codec->read  = es8323_read;
	codec->write = es8323_write;
	codec->hw_write = (hw_write_t)i2c_master_send;
	codec->control_data = container_of(codec->dev, struct i2c_client, dev);
	es8323_codec = codec;

	ret = gpio_request(SPK_CON, "spk_con");
	if (ret != 0) {
		pr_err("%s %d request error", __func__, __LINE__);
		goto err;
	}
	es8323_off_amp(true);

	ret = es8323_reset(codec);
	if (ret < 0) {
		dev_err(codec->dev,"fail to reset audio (%d)\n", ret);
		goto err;
	}
	
	es8323_init_regs(codec);
	es8323_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	codec->dapm.idle_bias_off = 0;
	det_initalize();
err:	
	return ret;
}

static int es8323_remove(struct snd_soc_codec *codec)
{
	es8323_off_amp(true);
	es8323_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_es8323 = {
	.probe =	es8323_probe,
	.remove = es8323_remove,
	.suspend = es8323_suspend,
	.resume = es8323_resume,
	.set_bias_level = es8323_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(es8323_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = es8323_reg,
	.reg_cache_step = 1,
};

#if defined(CONFIG_SPI_MASTER)
static int __devinit es8323_spi_probe(struct spi_device *spi)
{
	struct es8323_priv *es8323;
	int ret;

	es8323 = kzalloc(sizeof(struct es8323_priv), GFP_KERNEL);
	if (es8323 == NULL)
		return -ENOMEM;

	es8323->control_type = SND_SOC_SPI;
	spi_set_drvdata(spi, es8323);

	ret = snd_soc_register_codec(&spi->dev,
			&soc_codec_dev_es8323, &es8323_dai, 1);
	if (ret < 0)
		kfree(es8323);
	return ret;
}

static int __devexit es8323_spi_remove(struct spi_device *spi)
{
	snd_soc_unregister_codec(&spi->dev);
	kfree(spi_get_drvdata(spi));
	return 0;
}

static struct spi_driver es8323_spi_driver = {
	.driver = {
		.name	= "ES8323",
		.owner	= THIS_MODULE,
	},
	.probe		= es8323_spi_probe,
	.remove		= __devexit_p(es8323_spi_remove),
};
#endif /* CONFIG_SPI_MASTER */


#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

static void es8323_i2c_shutdown(struct i2c_client *i2c)
{
	struct snd_soc_codec *codec;

	if (!es8323_codec)
		goto err;

	es8323_off_amp(true);
	codec = es8323_codec;
	snd_soc_write(codec, ES8323_CONTROL2,0x58);
	snd_soc_write(codec, ES8323_CONTROL1,0x32);
	snd_soc_write(codec, ES8323_CHIPPOWER, 0xF3);
	snd_soc_write(codec, ES8323_DACPOWER, 0xC0);
	snd_soc_write(codec, ES8323_DACCONTROL26, 0x00);
	snd_soc_write(codec, ES8323_DACCONTROL27, 0x00);
	snd_soc_write(codec, ES8323_CONTROL1,0x30);
	snd_soc_write(codec, ES8323_CONTROL1,0x34);
err:
	return;
}

static __devinit int es8323_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct es8323_priv *es8323;
	struct i2c_adapter *adapter = to_i2c_adapter(i2c->dev.parent);
	char reg;
	char tmp;
	int ret = -1;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_warn(&adapter->dev,
		"I2C-Adapter doesn't support\n");
		return -EIO;
	}

	es8323 = kzalloc(sizeof(struct es8323_priv), GFP_KERNEL);
	if (es8323 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, es8323);
	es8323->control_type = SND_SOC_I2C;
	
	reg = ES8323_DACCONTROL18;
	ret = i2c_master_reg8_recv(i2c, reg, &tmp, 1 ,200 * 1000);
	if (ret < 0){
		pr_err("es8323 probe error\n");
		kfree(es8323);
		return ret;
	}

	ret =  snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_es8323, &es8323_dai, 1);
	if (ret < 0) {
		kfree(es8323);
		return ret;
	}

	return ret;
}

static __devexit int es8323_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id es8323_i2c_id[] = {
	{ "es8323", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es8323_i2c_id);

static struct i2c_driver es8323_i2c_driver = {
	.driver = {
		.name = "ES8323",
		.owner = THIS_MODULE,
	},
	.shutdown = es8323_i2c_shutdown,
	.probe =    es8323_i2c_probe,
	.remove =   __devexit_p(es8323_i2c_remove),
	.id_table = es8323_i2c_id,
};
#endif

static int __init es8323_init(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	return i2c_add_driver(&es8323_i2c_driver);
#endif
#if defined(CONFIG_SPI_MASTER)
	return spi_register_driver(&es8323_spi_driver);
#endif
}

static void __exit es8323_exit(void)
{
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	return i2c_del_driver(&es8323_i2c_driver);
#endif
#if defined(CONFIG_SPI_MASTER)
	return spi_unregister_driver(&es8323_spi_driver);
#endif
}

module_init(es8323_init);
module_exit(es8323_exit);

MODULE_DESCRIPTION("ASoC es8323 driver");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");




#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int proc_reg_show (struct seq_file *s, void *v)
{
	struct snd_soc_codec *codec = es8323_codec;
	#define MAX_REGS ES8323_MAX_REGS
	int i, j, k, offset;
	u8 val[MAX_REGS] = {0,};
	
	for (i = 0; i < MAX_REGS; i++)
		val[i] = es8323_read(codec, i);

	for (i = 0; i < MAX_REGS/16; i++) {
		offset = (i * 16) % 128;
		if (offset == 0) {
			seq_printf (s, "    ");
			for (k = 0; k < 16; k++)
				seq_printf (s, "   %x", k);
			seq_printf (s, "\n");
		}
		seq_printf (s, " %3x:", i * 16);
		for (j = 0; j < 16; j++) {
			seq_printf (s, "  %02x", val[i * 16 + j]);
		}
		seq_printf (s, "\n");
	}
	seq_printf (s, "\n");	
	return 0;
}

static int proc_reg_open (struct inode *inode, struct file *file)
{
	return single_open (file, proc_reg_show, NULL);
}

static const struct file_operations proc_reg_fops = {
	.open		= proc_reg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init codec_proc_init (void)
{
	proc_create ("es8323", 0, NULL, &proc_reg_fops);
	return 0;
}
late_initcall (codec_proc_init);
#endif /* CONFIG_PROC_FS */
