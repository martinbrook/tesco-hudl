/*
 * rk29_es8323.c  --  SoC audio for rockchip
 *
 * Driver for rockchip es8323 audio
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include "../codecs/es8323.h"
#include "rk29_pcm.h"
#include "rk29_i2s.h"

#include <mach/gpio.h>

#if 0
#define	DBG(x...)	printk(KERN_INFO x)
#else
#define	DBG(x...)
#endif

//static void *rk29_speaker = NULL;

static int rk29_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int pll_out = 0; 
	int ret;

	DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);    
	/*by Vincent Hsiung for EQ Vol Change*/
#define HW_PARAMS_FLAG_EQVOL_ON 0x21
#define HW_PARAMS_FLAG_EQVOL_OFF 0x22
	if ((params->flags == HW_PARAMS_FLAG_EQVOL_ON)||(params->flags == HW_PARAMS_FLAG_EQVOL_OFF))
	{
		ret = codec_dai->driver->ops->hw_params(substream, params, codec_dai); //by Vincent
		DBG("Enter::%s----%d\n",__FUNCTION__,__LINE__);
	}
	else
	{
		/* set codec DAI configuration */
#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE) 
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS); 
#endif	
#if defined (CONFIG_SND_RK29_CODEC_SOC_MASTER) 
		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM ); 
#endif
		if (ret < 0)
		return ret; 
		/* set cpu DAI configuration */
#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE) 
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
#endif	
#if defined (CONFIG_SND_RK29_CODEC_SOC_MASTER) 
		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);	
#endif		
		if (ret < 0)
			return ret;
	}

	switch(params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
		pll_out = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
		pll_out = 11289600;
		break;
	default:
		DBG("Enter:%s, %d, Error rate=%d\n",__FUNCTION__,__LINE__,params_rate(params));
		return -EINVAL;
		break;
	}
	DBG("Enter:%s, %d, rate=%d\n",__FUNCTION__,__LINE__,params_rate(params));

#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE)
	snd_soc_dai_set_sysclk(cpu_dai, 0, pll_out, 0);
	snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK, (pll_out/4)/params_rate(params)-1);
	snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, 3);
	snd_soc_dai_set_sysclk(codec_dai, 0, pll_out, 0);
#endif

	DBG("Enter:%s, %d, LRCK=%d\n",__FUNCTION__,__LINE__,(pll_out/4)/params_rate(params));
	return 0;
}


static struct snd_soc_ops rk29_ops = {
	  .hw_params = rk29_hw_params,
};

static struct snd_soc_dai_link rk29_dai = {
	.name = "ES8323",
	.stream_name = "ES8323 PCM",
	.codec_name = "ES8323.4-0010",
	.platform_name = "rockchip-audio",
#if defined(CONFIG_SND_RK29_SOC_I2S_8CH)	
	.cpu_dai_name = "rk29_i2s.0",
#elif defined(CONFIG_SND_RK29_SOC_I2S_2CH)
	.cpu_dai_name = "rk29_i2s.1",
#else
	.cpu_dai_name = "rk29_i2s.2",
#endif
	.codec_dai_name = "ES8323 HiFi",
	.ops = &rk29_ops,
};

static struct snd_soc_card snd_soc_card_rk29 = {
	.name = "RK29_ES8323",
	.dai_link = &rk29_dai,
	.num_links = 1,
};

static struct platform_device *rk29_snd_device;

static int __init audio_card_init(void)
{
	int ret =0;
	
	rk29_snd_device = platform_device_alloc("soc-audio", -1);
	if (!rk29_snd_device) {
		pr_err("platform device allocation failed\n");
		ret = -ENOMEM;
		return ret;
	}
	
	platform_set_drvdata(rk29_snd_device, &snd_soc_card_rk29);
	ret = platform_device_add(rk29_snd_device);
	if (ret) {
		pr_err("platform device add failed\n");
		platform_device_put(rk29_snd_device);
		return ret;
	}

	return ret;
}
static void __exit audio_card_exit(void)
{
    return platform_device_unregister(rk29_snd_device);
}

module_init(audio_card_init);
module_exit(audio_card_exit);
/* Module information */
MODULE_AUTHOR("rockchip");
MODULE_DESCRIPTION("ROCKCHIP i2s ASoC Interface");
MODULE_LICENSE("GPL");

