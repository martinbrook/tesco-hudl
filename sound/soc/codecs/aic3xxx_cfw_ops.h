#ifndef AIC3XXX_CFW_OPS_H_
#define AIC3XXX_CFW_OPS_H_

#include <linux/mutex.h>
#include <sound/soc.h>
#include <linux/cdev.h>

struct cfw_project;
struct aic3xxx_codec_ops;

struct cfw_state {
	struct cfw_project *pjt;
	struct aic3xxx_codec_ops  *ops;
	void *ops_obj;
	struct mutex mutex;
	int cur_mode_id;
	int cur_pll;
	int cur_mode;
	int cur_pfw;
	int cur_ovly;
	int cur_cfg;
	struct cdev cdev;
	int is_open;
};

int aic3xxx_cfw_init(struct cfw_state *ps, struct aic3xxx_codec_ops *ops,
		     void *ops_obj);
int aic3xxx_cfw_lock(struct cfw_state *ps, int lock);
int aic3xxx_cfw_reload(struct cfw_state *ps, void *pcfw, int n);
int aic3xxx_cfw_setmode(struct cfw_state *ps, int mode);
int aic3xxx_cfw_setmode_cfg(struct cfw_state *ps, int mode, int cfg);
int aic3xxx_cfw_setcfg(struct cfw_state *ps, int cfg);
int aic3xxx_cfw_transition(struct cfw_state *ps, char *ttype);
int aic3xxx_cfw_set_pll(struct cfw_state *ps, int asi);
int aic3xxx_cfw_control(struct cfw_state *ps, char *cname, int param);
int aic3xxx_cfw_add_controls(struct snd_soc_codec *codec, struct cfw_state *ps);
int aic3xxx_cfw_add_modes(struct snd_soc_codec *codec, struct cfw_state *ps);


#define AIC3XXX_COPS_MDSP_D  (0x00000003u)
#define AIC3XXX_COPS_MDSP_A  (0x00000030u)
#define AIC3XXX_COPS_MDSP_ALL (AIC3XXX_COPS_MDSP_D|AIC3XXX_COPS_MDSP_A)

#define AIC3XXX_ABUF_MDSP_D1 (0x00000001u)
#define AIC3XXX_ABUF_MDSP_D2 (0x00000002u)
#define AIC3XXX_ABUF_MDSP_A  (0x00000010u)
#define AIC3XXX_ABUF_MDSP_ALL \
		(AIC3XXX_ABUF_MDSP_D1|AIC3XXX_ABUF_MDSP_D2|AIC3XXX_ABUF_MDSP_A)

struct aic3xxx_codec_ops {
	int (*reg_read) (struct snd_soc_codec *p, unsigned int reg);
	int (*reg_write) (struct snd_soc_codec *p, unsigned int reg, unsigned char val);
	int (*set_bits) (struct snd_soc_codec *p, unsigned int reg,
			 unsigned char mask, unsigned char val);
	int (*bulk_read) (struct snd_soc_codec *p, unsigned int reg, int count, u8 *buf);
	int (*bulk_write) (struct snd_soc_codec *p, unsigned int reg,
			   int count, const u8 *buf);

	int (*lock) (struct snd_soc_codec *p);
	int (*unlock) (struct snd_soc_codec *p);
	int (*stop) (struct snd_soc_codec *p, int mask);
	int (*restore) (struct snd_soc_codec *p, int runstate);
	int (*bswap) (struct snd_soc_codec *p, int mask);
};


#endif
