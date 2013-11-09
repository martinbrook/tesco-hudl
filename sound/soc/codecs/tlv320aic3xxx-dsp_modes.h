
#ifndef tlv320aic3xxx_dsp_modes_hdr
#define tlv320aic3xxx_dsp_modes_hdr

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))
#endif

#ifdef DEFINE_DSP_MODES
#include "tlv320aic3xxx-dsp_ch1m.h"

const static struct aic3xxx_dsp_mode ch1m_base = {
	.need_sync = 0,
	.need_pd_adc = 1,
	.need_pd_dac = 1,
	.mode = 0,
	.config = 255,
	.reg_values = {
		{ARRAY_SIZE(ch1m_base_pre), ch1m_base_pre},
		{ARRAY_SIZE(ch1m_base_a), ch1m_base_a},
		{ARRAY_SIZE(ch1m_base_d), ch1m_base_d},
		{ARRAY_SIZE(ch1m_base_post), ch1m_base_post},
		{ARRAY_SIZE(ch1m_patch_main), ch1m_patch_main},
		{0, 0},
	},
};

const static struct aic3xxx_dsp_mode ch1m_hp = {
	.need_sync = 0,
	.need_pd_adc = 0,
	.need_pd_dac = 0,
	.mode = 0,
	.config = 0,
	.reg_values = {
		{AIC3XXX_TO_OPT_PATCH(1), 0},
		{AIC3XXX_TO_PATCH_ITEM(AIC3XXX_PATCH_ITEM_D1, ARRAY_SIZE(ch1m_patch_main)), ch1m_patch_main},
		{0, 0},
	}
};

const static struct aic3xxx_dsp_mode ch1m_hp_srs = {
	.need_sync = 0,
	.need_pd_adc = 0,
	.need_pd_dac = 0,
	.mode = 0,
	.config = 1,
	.reg_values = {
		{AIC3XXX_TO_OPT_PATCH(1), 0},
		{AIC3XXX_TO_PATCH_ITEM(AIC3XXX_PATCH_ITEM_D1, ARRAY_SIZE(ch1m_patch_hp_srs)), ch1m_patch_hp_srs},
		{0, 0},
	}
};

const static struct aic3xxx_dsp_mode ch1m_spk = {
	.need_sync = 0,
	.need_pd_adc = 0,
	.need_pd_dac = 0,
	.mode = 0,
	.config = 2,
	.reg_values = {
		{AIC3XXX_TO_OPT_PATCH(1), 0},
		{AIC3XXX_TO_PATCH_ITEM(AIC3XXX_PATCH_ITEM_D1, ARRAY_SIZE(ch1m_patch_spk)), ch1m_patch_spk},
		{0, 0},
	}
};

static const struct aic3xxx_dsp_mode *g_dsp_modes[] = {
	&ch1m_base, &ch1m_hp, &ch1m_hp_srs, &ch1m_spk, 0,
};

#endif // DEFINE_DSP_MODES

#endif // tlv320aic3xxx_dsp_modes_hdr
