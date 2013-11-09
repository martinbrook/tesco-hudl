/*
* linux/sound/soc/codecs/tlv320aic3xxx-dsp.c
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
#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>
#include <sound/jack.h>
#include <linux/spi/spi.h>

#include "tlv320aic3xxx-dsp.h"

#define XDBG(fmt, args...)  printk(KERN_INFO "[XDBG] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)

#define DEFINE_DSP_MODES 1
#include "tlv320aic3xxx-dsp_modes.h"
#undef DEFINE_DSP_MODES

#ifdef AIC3262_SPI

#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <sound/soc.h>
#include <linux/gpio.h>

#define aic3xxx_spi_cs_en(val) do{ gpio_set_value(aic3xxx->gpio_spi_cs, val); udelay(1); }while(0)

int aic3xxx_spi_read_device_raw(struct aic3xxx_dsp_priv *aic3xxx, unsigned int reg,
	int bytes, void *dest)
{
	struct spi_device *spi = aic3xxx->codec->control_data;

	struct spi_message	message;
	struct spi_transfer	x[16];
	int ret;
	u8 buf[16];

	buf[0] = ((reg & 0x0FF) << 1) | (0x01) ;
	memset(x, 0, sizeof x);
	spi_message_init(&message);	
	x[0].len = 1;
	x[0].tx_buf = buf;
	x[1].len = bytes;
	x[1].rx_buf = dest ;

	spi_message_add_tail(&x[0], &message);
	spi_message_add_tail(&x[1], &message);

	aic3xxx_spi_cs_en(0);	
	ret = spi_sync(spi, &message);
	aic3xxx_spi_cs_en(1);	
	if (ret < 0)
	{
		XDBG("[codec] aic3xxx_spi_read reg=%x reade error\n", buf[0] >> 1);
		return ret;
	}
	return bytes;
}

static int aic3xxx_spi_write_device_raw(struct aic3xxx_dsp_priv *aic3xxx, unsigned int reg,
	int bytes, const void *src)
{
	struct spi_device *spi = aic3xxx->codec->control_data;
	int ret;

	u8 write_buf[bytes + 1];
	write_buf[0] = (reg & 0x0FF) << 1 ;
	memcpy(&write_buf[1], src, bytes);
	aic3xxx_spi_cs_en(0);	
	ret = spi_write(spi, write_buf, bytes + 1);
	aic3xxx_spi_cs_en(1);	
	if (ret < 0)
		return ret;

	return bytes;
}

static int aic3xxx_read_raw(struct aic3xxx_dsp_priv *aic3xxx, unsigned int reg,
	int bytes, void *dest)
{
	int ret;

	BUG_ON(bytes <= 0);

	reg &= 0x7F;

	ret = aic3xxx_spi_read_device_raw(aic3xxx, reg, bytes, dest);
	if (ret < 0)
		return ret;

	return ret;
}

static int aic3xxx_write_raw(struct aic3xxx_dsp_priv *aic3xxx, unsigned int reg,
	int bytes, const void *src)
{
	const u8 *buf = src;

	BUG_ON(bytes <= 0);
	
	if(reg > 0x7F)
	{
		XDBG("aic3xxx aic3xxx_write_raw error reg address %d\n", reg);
		return -1;
	}

	return aic3xxx_spi_write_device_raw(aic3xxx, reg, bytes, src);
}

#endif // AIC3262_SPI

#ifdef AIC3254_I2C

#include <linux/i2c.h>
#include <linux/delay.h>
#include <sound/soc.h>

static int aic3xxx_i2c_read_device_raw(struct aic3xxx_dsp_priv *aic3xxx, unsigned int reg,
	int bytes, void *dest)
{
	int ret;
	struct i2c_client *i2c = to_i2c_client(aic3xxx->dev);
	unsigned char offset = reg;
	ret = i2c_master_send(i2c, &offset, 1);
	if(ret < 0)
		return ret;
	if(ret != 1)
		return -EIO;
	ret = i2c_master_recv(i2c, dest, bytes);

	return ret;
}

static int aic3xxx_i2c_write_device_raw(struct aic3xxx_dsp_priv *aic3xxx, unsigned int reg,
	int bytes, const void *src)
{
	int ret;
	struct i2c_client *i2c = to_i2c_client(aic3xxx->dev);

	u8 write_buf[bytes + 1];
	write_buf[0] = (reg & 0x0FF);
	memcpy(&write_buf[1], src, bytes);
	
	ret = i2c_master_send(i2c, write_buf, bytes + 1);
	
	return ret;
}

static int aic3xxx_read_raw(struct aic3xxx_dsp_priv *aic3xxx, unsigned int reg,
	int bytes, void *dest)
{
	int ret;

	BUG_ON(bytes <= 0);

	reg &= 0x7F;

	ret = aic3xxx_i2c_read_device_raw(aic3xxx, reg, bytes, dest);
	if (ret < 0)
		return ret;

	return ret;
}

static int aic3xxx_write_raw(struct aic3xxx_dsp_priv *aic3xxx, unsigned int reg,
	int bytes, const void *src)
{
	const u8 *buf = src;

	BUG_ON(bytes <= 0);
	
	if(reg > 0x7F)
	{
		XDBG("aic3xxx aic3xxx_write_raw error reg address %d\n", reg);
		return -1;
	}

	return aic3xxx_i2c_write_device_raw(aic3xxx, reg, bytes, src);
}
#endif // AIC3254_I2C

static int aic3xxx_access_lock(struct aic3xxx_dsp_priv *aic3xxx)
{
	mutex_lock(aic3xxx->io_lock);
	return 0;
}

static int aic3xxx_access_unlock(struct aic3xxx_dsp_priv *aic3xxx)
{
	mutex_unlock(aic3xxx->io_lock);
	return 0;
}

static int aic3xxx_set_bits_raw(struct aic3xxx_dsp_priv *aic3xxx, unsigned int reg,
	unsigned char mask, unsigned char val)
{
	int i;
	unsigned char buf[4];
	i = aic3xxx_read_raw(aic3xxx, reg, 1, &buf[0]);
	if(i < 0) return i;
	//XDBG("aic3xxx_set_bits_raw %02X %02X %02X %02X\n", reg, buf[0], mask, val);
	buf[0] &= ~mask;
	buf[0] |= (val & mask);
	//XDBG("aic3xxx_set_bits_raw %02X %02X\n", reg, buf[0]);
	return aic3xxx_write_raw(aic3xxx, reg, 1, &buf[0]);
}

static int aic3xxx_write_regs_raw(struct aic3xxx_dsp_priv *aic3xxx, const struct aic_reg_raw *regs, int count)
{
	int i;
	unsigned char v[256];
	int addr;
	int dat_idx = 0;

	i = 0;
	while(i<count)
	{
		addr = regs[i].reg_off;
		v[dat_idx++] = regs[i].reg_val;
		i ++;
		addr ++;

		while(i<count)
		{
			if(regs[i].reg_off == addr)
			{
				v[dat_idx++] = regs[i].reg_val;
				i ++;
				addr ++;
			}
			else
			{
				break;
			}
		}
		if(dat_idx)
		{
			aic3xxx_write_raw(aic3xxx, addr - dat_idx, dat_idx, v);
			dat_idx = 0;
		}
	}
	return 0;
}

static unsigned char last_book;
static unsigned char last_page;
static int store_bp_count;
static int store_book_page(struct aic3xxx_dsp_priv *aic3xxx)
{
	unsigned char v = 0;
	if(!store_bp_count)
	{
		aic3xxx_read_raw(aic3xxx, 0, 1, &last_page);
		aic3xxx_write_raw(aic3xxx, 0x00, 1, &v);
		aic3xxx_read_raw(aic3xxx, 0x7f, 1, &last_book);
	}
	store_bp_count ++;
}

static int restore_book_page(struct aic3xxx_dsp_priv *aic3xxx)
{
	unsigned char v = 0;
	store_bp_count --;
	if(!store_bp_count)
	{
		aic3xxx_write_raw(aic3xxx, 0x00, 1, &v);
		aic3xxx_write_raw(aic3xxx, 0x7f, 1, &last_book);
		aic3xxx_write_raw(aic3xxx, 0x00, 1, &last_page);
	}
}

static int aic3xxx_dsp_set_book_page(struct aic3xxx_dsp_priv *aic3xxx, unsigned char book, unsigned char page)
{
	aic3xxx_write_raw(aic3xxx, 0x00, 1, &page);
	aic3xxx_write_raw(aic3xxx, 0x7F, 1, &book);
	return 0;
}

static unsigned char v40;
static unsigned char mute_count = 0;
static void set_mute(struct aic3xxx_dsp_priv *aic3xxx, int mute)
{
#if 0
	unsigned char v;
	if(mute)
	{
		if(mute_count)
		{
			mute_count ++;
			return;
		}

		aic3xxx_dsp_set_book_page(aic3xxx, 0, 0);

		aic3xxx_read_raw(aic3xxx, 0x40, 1, &v40);
		v = v40&(~0x0C);
		aic3xxx_write_raw(aic3xxx, 0x40, 1, &v);

		mute_count ++;
	}
	else
	{
		mute_count --;
		if(mute_count)
		{
			return;
		}
		aic3xxx_dsp_set_book_page(aic3xxx, 0, 0);
		aic3xxx_write_raw(aic3xxx, 0x40, 1, &v40);
	}
	aic3xxx_dsp_set_book_page(aic3xxx, 0, 0);
#endif
}

int aic3xxx_set_dsp_mode(struct aic3xxx_dsp_priv *aic3xxx, int mode, int config)
{
	int idx;
	int i;
	const struct aic3xxx_dsp_mode *pmode;
	unsigned char v[4];
	unsigned char r_bak[4];
	unsigned char run_status[2];
	int retry;
	int ready = 1;
	idx = 0;
	while(g_dsp_modes[idx])
	{
		pmode = g_dsp_modes[idx];
		if((pmode->mode == mode) && (pmode->config == config))
		{
			XDBG("aic3xxx_set_dsp_mode %d %d\n", mode, config);

			if((mode != aic3xxx->cur_mode) && (config < 255))
			{
				/* switch mode first */
				XDBG("aic3xxx try switch mode\n");
				if(0 == aic3xxx_set_dsp_mode(aic3xxx, mode, 255))
				{
					XDBG("aic3xxx switch mode failed\n");
					return 0;
				}
				XDBG("aic3xxx switch mode done\n");
				aic3xxx->cur_mode = mode;
			}
			aic3xxx->cur_config = config;

			aic3xxx_access_lock(aic3xxx);
			store_book_page(aic3xxx);
			if(config == 255)set_mute(aic3xxx, 1);
			if(pmode->need_sync)
			{
				aic3xxx_dsp_set_book_page(aic3xxx, 0, 0);
				aic3xxx_read_raw(aic3xxx, 0x3F, 1, &r_bak[0]);
				aic3xxx_read_raw(aic3xxx, 0x51, 1, &r_bak[1]);
				v[0] = 0x02; //r_bak[0] & 0x3F;
				v[1] = 0x02; //r_bak[1] & 0x3F;
				aic3xxx_write_raw(aic3xxx, 0x3F, 1, &v[0]);
				aic3xxx_write_raw(aic3xxx, 0x51, 1, &v[1]);
				i = 0;
				do{
					// wait adc off
					aic3xxx_read_raw(aic3xxx, 36, 1, &v[2]);
					i ++;
					if(v[2] & 0x44)
					{
						msleep(5);
						aic3xxx_write_raw(aic3xxx, 0x51, 1, &v[1]);
					} else {
						break;
					}
				}while(i<10);
				if((v[2] & 0x44))
				{
					ready = 0;
					XDBG("aic3xxx adc power off failed %02X\n", (v[2] & 0x44));
				}
				i = 0;
				do{
					// wait dac off
					aic3xxx_read_raw(aic3xxx, 37, 1, &v[2]);
					i ++;
					if(v[2] & 0x88)
					{
						msleep(5);
						aic3xxx_write_raw(aic3xxx, 0x3F, 1, &v[0]);
					} else {
						break;
					}
				}while(i<10);
				if((v[2] & 0x88))
				{
					ready = 0;
					XDBG("aic3xxx dac power off failed %02X\n", (v[2] & 0x88));
				}
				aic3xxx_read_raw(aic3xxx, 0x06, 1, &r_bak[2]);
				v[2] = r_bak[2] & 0x7F;
				aic3xxx_write_raw(aic3xxx, 0x06, 1, &v[2]);
			}
			else if(pmode->need_pd_adc || pmode->need_pd_dac)
			{
				aic3xxx_dsp_set_book_page(aic3xxx, 0, 0);
				if(pmode->need_pd_adc)
				{
					aic3xxx_read_raw(aic3xxx, 0x51, 1, &r_bak[1]);
					v[1] = 0x02; // r_bak[1] & 0x3F;
					aic3xxx_write_raw(aic3xxx, 0x51, 1, &v[1]);
				}
				if(pmode->need_pd_dac)
				{
					aic3xxx_read_raw(aic3xxx, 0x3F, 1, &r_bak[0]);
					v[0] = 0x02; // r_bak[0] & 0x3F;
					aic3xxx_write_raw(aic3xxx, 0x3F, 1, &v[0]);
				}
				if(pmode->need_pd_adc)
				{
					i = 0;
					do{
						// wait adc off
						aic3xxx_read_raw(aic3xxx, 36, 1, &v[2]);
						i ++;
						if(v[2] & 0x44)
						{
							msleep(5);
							aic3xxx_write_raw(aic3xxx, 0x51, 1, &v[1]);
						} else {
							break;
						}
					}while(i<10);
					if((v[2] & 0x44))
					{
						ready = 0;
						XDBG("aic3xxx adc power off failed %02X\n", (v[2] & 0x44));
					}
				}
				if(pmode->need_pd_dac)
				{
					i = 0;
					do{
						// wait dac off
						aic3xxx_read_raw(aic3xxx, 37, 1, &v[2]);
						i ++;
						if(v[2] & 0x88)
						{
							msleep(5);
							aic3xxx_write_raw(aic3xxx, 0x3F, 1, &v[0]);
						} else {
							break;
						}
					}while(i<10);
					if((v[2] & 0x88))
					{
						ready = 0;
						XDBG("aic3xxx dac power off failed %02X\n", (v[2] & 0x88));
					}
				}
				aic3xxx_read_raw(aic3xxx, 0x06, 1, &r_bak[2]);
				v[2] = r_bak[2] & 0x7F;
				aic3xxx_write_raw(aic3xxx, 0x06, 1, &v[2]);
			}

			if(ready)
			{
				aic3xxx_dsp_set_book_page(aic3xxx, 0, 0);
				aic3xxx_read_raw(aic3xxx, 36, 1, v);
				run_status[0] = v[0] & 0x44; // adc
				aic3xxx_read_raw(aic3xxx, 37, 1, v);
				run_status[1] = v[0] & 0x88; // dac

				i = 0;
				while(pmode->reg_values[i].items)
				{
					if(AIC3XXX_IS_OPT_ITEM(pmode->reg_values[i].items))
					{
						if(AIC3XXX_IS_OPT_PATCH(pmode->reg_values[i].items))
						{
							unsigned int swaps = 0;
							int pcnt = AIC3XXX_PATCH_ITEMS(pmode->reg_values[i].items);
							int j;
							for(j=0; j<pcnt; j++)
							{
								i++;
								swaps |= AIC3XXX_GET_PATCH_T(pmode->reg_values[i].items);
								aic3xxx_write_regs_raw(aic3xxx, pmode->reg_values[i].opts, AIC3XXX_GET_PATCH_SIZE(pmode->reg_values[i].items));
							}
							if(swaps & AIC3XXX_PATCH_ITEM_A)
							{
								if(run_status[0])
								{
#ifdef AIC3262_DSP
									aic3xxx_dsp_set_book_page(aic3xxx, 0x28, 0);
#endif
#ifdef AIC3254_DSP
									aic3xxx_dsp_set_book_page(aic3xxx, 0, 44);
#endif

									v[0] = 0x05;
									aic3xxx_write_raw(aic3xxx, 0x01, 1, &v[0]);
									retry = 6;
									do{
										aic3xxx_read_raw(aic3xxx, 0x01, 1, &v[0]);
										msleep(5);
									}while((v[0] & 0x01) && retry--);
								}
								else
								{
									swaps &= ~AIC3XXX_PATCH_ITEM_A;
								}
							}
							if(swaps & (AIC3XXX_PATCH_ITEM_D1 | AIC3XXX_PATCH_ITEM_D2))
							{
								if(run_status[1])
								{
									if(swaps & AIC3XXX_PATCH_ITEM_D1)
									{
#ifdef AIC3262_DSP
										aic3xxx_dsp_set_book_page(aic3xxx, 0x50, 0);
#endif
#ifdef AIC3254_DSP
										aic3xxx_dsp_set_book_page(aic3xxx, 0, 8);
#endif
										v[0] = 0x05;
										aic3xxx_write_raw(aic3xxx, 0x01, 1, &v[0]);
										retry = 6;
										do{
											aic3xxx_read_raw(aic3xxx, 0x01, 1, &v[0]);
											msleep(5);
										}while((v[0] & 0x01) && retry--);
									}
#ifdef AIC3262_DSP
									if(swaps & AIC3XXX_PATCH_ITEM_D2)
									{
										aic3xxx_dsp_set_book_page(aic3xxx, 0x52, 0);
										v[0] = 0x05;
										aic3xxx_write_raw(aic3xxx, 0x01, 1, &v[0]);
										retry = 6;
										do{
											aic3xxx_read_raw(aic3xxx, 0x01, 1, &v[0]);
											msleep(5);
										}while((v[0] & 0x01) && retry--);
									}
#endif
								}
								else
								{
									swaps &= ~(AIC3XXX_PATCH_ITEM_D1 | AIC3XXX_PATCH_ITEM_D2);
								}
							}
							i -= pcnt;
							for(j=0; j<pcnt; j++)
							{
								i++;
								if(swaps & AIC3XXX_GET_PATCH_T(pmode->reg_values[i].items))
								{
									aic3xxx_write_regs_raw(aic3xxx, pmode->reg_values[i].opts, AIC3XXX_GET_PATCH_SIZE(pmode->reg_values[i].items));
								}
							}
						}
					}
					else if(pmode->reg_values[i].opts)
					{
						aic3xxx_write_regs_raw(aic3xxx, pmode->reg_values[i].opts, pmode->reg_values[i].items);
					}
					i ++;
				}
			}

			if(pmode->need_sync)
			{
				aic3xxx_dsp_set_book_page(aic3xxx, 0, 0);
				aic3xxx_set_bits_raw(aic3xxx, 0x3F, 0xC0, r_bak[0]);
				aic3xxx_set_bits_raw(aic3xxx, 0x51, 0xC0, r_bak[1]);
				aic3xxx_set_bits_raw(aic3xxx, 0x06, 0x80, r_bak[2]);
			}
			else if(pmode->need_pd_adc || pmode->need_pd_dac)
			{
				aic3xxx_dsp_set_book_page(aic3xxx, 0, 0);
				if(pmode->need_pd_adc)
				{
					aic3xxx_set_bits_raw(aic3xxx, 0x51, 0xC0, r_bak[1]);
				}
				if(pmode->need_pd_dac)
				{
					aic3xxx_set_bits_raw(aic3xxx, 0x3F, 0xC0, r_bak[0]);
				}
				aic3xxx_set_bits_raw(aic3xxx, 0x06, 0x80, r_bak[2]);
			}
			if(config == 255)set_mute(aic3xxx, 0);
			restore_book_page(aic3xxx);
			aic3xxx_access_unlock(aic3xxx);
			XDBG("aic3xxx_set_dsp_mode %d %d done\n", mode, config);
			return ready;
		}
		idx++;
	}
	return -1;
}

