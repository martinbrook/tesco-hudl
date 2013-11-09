/*
 * rk29_aic325x.c  --  SoC audio for Rockchip
 *
 * Author: Jason <xjq@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include "rk29_pcm.h"
#include "rk29_i2s.h"
#include "../codecs/tlv320aic325x.h"


static int rk29_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int pll_out = 0; 
	int div_bclk,div_mclk;
	int ret;

	/* set codec DAI configuration */
#if defined (CONFIG_SND_RK29_CODEC_SOC_SLAVE) 
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
	 	     SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
#endif	
#if defined (CONFIG_SND_RK29_CODEC_SOC_MASTER) 			   
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		     SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
#endif
	if (ret < 0)
		dev_err(codec_dai->dev, "fail to set codec format\n");

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
		dev_err(cpu_dai->dev, "fail to set iis format\n");

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
			dev_err(cpu_dai->dev, "rate=%d\n",params_rate(params));
			return -EINVAL;
	}
 
	div_bclk=(pll_out/4)/params_rate(params)-1;
	div_mclk=3;
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, pll_out, 0);
	if(ret < 0) {
		dev_err(cpu_dai->dev, "failed to set cpu sysclk\n"); 
		goto exit;
	}
	
	dev_dbg(cpu_dai->dev, "mclk=%d bclk=%d lrclk=%d\n",
		           pll_out, pll_out/div_mclk, pll_out/div_bclk);
	snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK, div_bclk);
	snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, div_mclk);
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, pll_out, 0);
	if (ret < 0) {
		pr_err("failed to set codec sysclk\n"); 
	}
	
exit:
	return ret;
}

static struct snd_soc_ops rk29_ops = {
	.hw_params = rk29_hw_params,
};

static struct snd_soc_dai_link rk29_dai = {
	.name = "Legacy McBSP",
	.stream_name = "Multimedia",
#if defined(CONFIG_SND_RK29_SOC_I2S_8CH)	
	.cpu_dai_name = "rk29_i2s.0",
#elif defined(CONFIG_SND_RK29_SOC_I2S_2CH)
	.cpu_dai_name = "rk29_i2s.1",
#else	
	.cpu_dai_name = "rk29_i2s.2",
#endif
	.codec_dai_name = "tlv320aic325x-MM_EXT",
	.platform_name = "rockchip-audio",
	.codec_name = "AIC325x",
	.ops = &rk29_ops,
};

static struct snd_soc_card snd_soc_card_rk29 = {
	.name = "RK29_AIC325X",
	.dai_link = &rk29_dai,
	.num_links = 1,
};

static struct platform_device *rk29_snd_device;

static int __init audio_card_init(void)
{
	int ret;
	
	rk29_snd_device = platform_device_alloc("soc-audio", -1);
	if (!rk29_snd_device) {
		  pr_err("platform device allocation failed\n");
		  return -ENOMEM;
	}
	
	platform_set_drvdata(rk29_snd_device, &snd_soc_card_rk29);
	ret = platform_device_add(rk29_snd_device);
	if (ret) {
		pr_err("platform device add failed\n");
		platform_device_put(rk29_snd_device);
	}
	
	return ret;
}

static void __exit audio_card_exit(void)
{
	return platform_device_unregister(rk29_snd_device);
}

module_init(audio_card_init);
module_exit(audio_card_exit);

MODULE_AUTHOR("rockchip>");
MODULE_DESCRIPTION("ROCKCHIP i2s ASoC Interface");
MODULE_LICENSE("GPL");

