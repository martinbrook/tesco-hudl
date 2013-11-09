/*
* linux/sound/soc/codecs/tlv320aic3xxx-dsp.h
*
* Copyright (C) 2011 TI.com
*
* This package is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
* WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
* The TLV320AIC3XXX is a flexible, low-power, low-voltage stereo audio
* codec with digital microphone inputs and programmable outputs.
*
* History:
*
* Rev 0.1   TI         20-01-2011
	kevin-dang
*/

#ifndef _TLV320AIC3XXX_DSP_H
#define _TLV320AIC3XXX_DSP_H

//#define AIC3262_DSP
//#define AIC3262_SPI

#define AIC3254_DSP
#define AIC3254_I2C

#define AIC3XXX_IS_OPT_ITEM(arg)	(((unsigned int)arg) & 0x8000)
#define AIC3XXX_TO_OPT_ITEM(arg)	(arg | 0x8000)
#define AIC3XXX_IS_OPT_PATCH(arg)	(((unsigned int)arg) & 0x0800)
#define AIC3XXX_TO_OPT_PATCH(items)	(0x8800 | items)
#define AIC3XXX_PATCH_ITEMS(arg)	(((unsigned int)arg) & 0x00ff)

#define AIC3XXX_PATCH_ITEM_A	0x40000
#define AIC3XXX_PATCH_ITEM_D1	0x20000
#define AIC3XXX_PATCH_ITEM_D2	0x10000
#define AIC3XXX_TO_PATCH_ITEM(patch_t, items) (patch_t | items)
#define AIC3XXX_GET_PATCH_T(arg) (arg & 0x0F0000)
#define AIC3XXX_GET_PATCH_SIZE(arg) (arg & 0x0FFFF)

#pragma pack(push)
#pragma pack(1)
typedef struct aic_reg_raw {
    u8 reg_off;
    u8 reg_val;
}AIC_REG_RAW;
#pragma pack(pop)

typedef struct aic_reg {
	unsigned int reg;
	int val;
}AIC_REG;

struct aic3xxx_dsp_priv {
	struct snd_soc_codec *codec;
	struct mutex *io_lock;
	struct device *dev;
	int cur_mode;
	int cur_config;
	int gpio_spi_cs;
};

struct aic3xxx_reg_array
{
	int items; /* if AIC3XXX_IS_OPT_ITEM(items), it is an opt */
	const struct aic_reg_raw *opts;
};

struct aic3xxx_dsp_mode
{
	int need_sync;
	int need_pd_adc;
	int need_pd_dac;
	int mode;
	int config;
	const struct aic3xxx_reg_array reg_values[];
};

extern int aic3xxx_set_dsp_mode(struct aic3xxx_dsp_priv *aic3xxx, int mode, int config);

#endif /* _TLV320AIC3XXX_DSP_H */
