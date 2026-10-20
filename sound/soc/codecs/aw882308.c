// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bring-up driver for the Awinic smart amplifiers with product id 0x2308
 * that drive the OnePlus 13 (dodge) speakers.
 *
 * Register layout per the GPL-2.0 aw882xx driver OnePlus publishes for this
 * phone (aw882xx_pid_2308_reg.h). This driver does no speaker protection:
 * it applies an optional register table from firmware, runs the amplifier
 * from register defaults otherwise, and caps the volume.
 *
 * Described in DT as "awinic,aw882308" (provisional) with reset-gpios and
 * sound-name-prefix.
 */
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <linux/bitfield.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define AW_ID		0x00	/* write 0x55aa: soft reset */
#define AW_SYSST	0x01
#define AW_SYSINT	0x02
#define AW_SYSINTM	0x03
#define AW_SYSCTRL	0x04
#define AW_SYSCTRL2	0x05
#define AW_I2SCTRL1	0x06
#define AW_REG_MAX	0x80

#define AW_CHIP_ID	0x2308
#define AW_SOFT_RESET	0x55aa

#define SYSST_PLLS	BIT(0)
#define SYSST_CLKS	BIT(4)
#define SYSST_SWS	BIT(8)
#define SYSST_BSTS	BIT(9)

#define SYSCTRL_PWDN	BIT(0)
#define SYSCTRL_AMPPD	BIT(1)
#define SYSCTRL_HMUTE	BIT(8)
#define SYSCTRL_I2STXEN	BIT(13)
#define SYSCTRL_ULS_HMUTE BIT(15)

#define SYSCTRL2_VOL	GENMASK(9, 0)	/* attenuation, ~0.094 dB/step */
#define AW_MUTE_VOL	1023

#define I2SCTRL1_CHSEL	GENMASK(11, 10)	/* 1 left, 2 right, 3 mono */
#define I2SCTRL1_I2SMD	GENMASK(9, 8)
#define I2SCTRL1_I2SFS	GENMASK(7, 6)	/* 0 16b, 1 20b, 2 24b, 3 32b */
#define I2SCTRL1_I2SBCK	GENMASK(5, 4)	/* 0 32fs, 1 48fs, 2 64fs */
#define I2SCTRL1_I2SSR	GENMASK(3, 0)	/* 8 = 48 kHz */

/* Attenuation floor: 0 would be full scale. 320 steps is about -30 dB. */
static unsigned int min_atten = 320;
module_param(min_atten, uint, 0444);
MODULE_PARM_DESC(min_atten, "Lowest attenuation the volume control allows (0..1023)");

static bool use_fw = true;
module_param(use_fw, bool, 0444);
MODULE_PARM_DESC(use_fw, "Apply aw882308-<addr>.bin register table if present");

struct aw882308 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset;
	unsigned int atten;	/* user attenuation, >= min_atten */
	unsigned int chsel;
	bool running;
};

static const struct regmap_config aw882308_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = AW_REG_MAX - 1,
	.cache_type = REGCACHE_NONE,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static int aw882308_wait_sysst(struct aw882308 *aw, unsigned int want)
{
	unsigned int st = 0;
	int i;

	for (i = 0; i < 50; i++) {
		regmap_read(aw->regmap, AW_SYSST, &st);
		if ((st & want) == want)
			return 0;
		usleep_range(2000, 2100);
	}
	dev_err(aw->dev, "SYSST 0x%04x, wanted 0x%04x\n", st, want);
	return -ETIMEDOUT;
}

static void aw882308_set_vol(struct aw882308 *aw, unsigned int v)
{
	regmap_update_bits(aw->regmap, AW_SYSCTRL2, SYSCTRL2_VOL, v);
}

static void aw882308_stop(struct aw882308 *aw)
{
	aw882308_set_vol(aw, AW_MUTE_VOL);
	regmap_update_bits(aw->regmap, AW_SYSCTRL, SYSCTRL_ULS_HMUTE | SYSCTRL_HMUTE,
			   SYSCTRL_ULS_HMUTE | SYSCTRL_HMUTE);
	usleep_range(5000, 5100);
	regmap_update_bits(aw->regmap, AW_SYSCTRL, SYSCTRL_I2STXEN, 0);
	regmap_update_bits(aw->regmap, AW_SYSCTRL, SYSCTRL_AMPPD, SYSCTRL_AMPPD);
	regmap_update_bits(aw->regmap, AW_SYSCTRL, SYSCTRL_PWDN, SYSCTRL_PWDN);
	aw->running = false;
}

static int aw882308_start(struct aw882308 *aw)
{
	unsigned int st, ctrl, ngcheck;
	int ret;

	regmap_update_bits(aw->regmap, AW_SYSCTRL, SYSCTRL_PWDN, 0);
	usleep_range(2000, 2100);
	ret = aw882308_wait_sysst(aw, SYSST_PLLS | SYSST_CLKS);
	if (ret)
		goto fail;

	regmap_update_bits(aw->regmap, AW_SYSCTRL, SYSCTRL_AMPPD, 0);
	usleep_range(1000, 1100);
	/* Boost finished, clocks stable, PLL locked. */
	ngcheck = SYSST_BSTS | SYSST_CLKS | SYSST_PLLS;
	ret = aw882308_wait_sysst(aw, ngcheck);
	if (ret)
		goto fail;

	regmap_write(aw->regmap, AW_SYSINT, 0xffff);	/* read-clear status */
	regmap_read(aw->regmap, AW_SYSINT, &st);
	regmap_update_bits(aw->regmap, AW_SYSCTRL, SYSCTRL_ULS_HMUTE, 0);
	aw882308_set_vol(aw, AW_MUTE_VOL);
	regmap_update_bits(aw->regmap, AW_SYSCTRL, SYSCTRL_HMUTE, 0);
	aw882308_set_vol(aw, aw->atten);
	aw->running = true;

	regmap_read(aw->regmap, AW_SYSST, &st);
	regmap_read(aw->regmap, AW_SYSCTRL, &ctrl);
	dev_dbg(aw->dev, "running: SYSST 0x%04x SYSCTRL 0x%04x atten %u\n",
		 st, ctrl, aw->atten);
	return 0;
fail:
	regmap_read(aw->regmap, AW_SYSCTRL, &ctrl);
	dev_err(aw->dev, "start failed, SYSCTRL 0x%04x\n", ctrl);
	aw882308_stop(aw);
	return ret;
}

static int aw882308_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct aw882308 *aw = snd_soc_component_get_drvdata(dai->component);

	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;
	if (mute) {
		if (aw->running)
			aw882308_stop(aw);
		return 0;
	}
	if (aw->running)
		return 0;
	return aw882308_start(aw);
}

static int aw882308_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	struct aw882308 *aw = snd_soc_component_get_drvdata(dai->component);
	unsigned int width = params_physical_width(params);
	unsigned int fs, bck, val;

	if (params_rate(params) != 48000) {
		dev_err(aw->dev, "only 48 kHz for now, got %u\n", params_rate(params));
		return -EINVAL;
	}
	switch (params_width(params)) {
	case 16:
		fs = 0;
		break;
	case 24:
		fs = 2;
		break;
	case 32:
		fs = 3;
		break;
	default:
		return -EINVAL;
	}
	switch (width * params_channels(params)) {
	case 32:
		bck = 0;
		break;
	case 48:
		bck = 1;
		break;
	case 64:
		bck = 2;
		break;
	default:
		dev_err(aw->dev, "unsupported frame of %u bits\n",
			width * params_channels(params));
		return -EINVAL;
	}
	val = FIELD_PREP(I2SCTRL1_CHSEL, aw->chsel) |
	      FIELD_PREP(I2SCTRL1_I2SMD, 0) |
	      FIELD_PREP(I2SCTRL1_I2SFS, fs) |
	      FIELD_PREP(I2SCTRL1_I2SBCK, bck) |
	      FIELD_PREP(I2SCTRL1_I2SSR, 8);
	regmap_update_bits(aw->regmap, AW_I2SCTRL1,
			   I2SCTRL1_CHSEL | I2SCTRL1_I2SMD | I2SCTRL1_I2SFS |
			   I2SCTRL1_I2SBCK | I2SCTRL1_I2SSR, val);
	regmap_read(aw->regmap, AW_I2SCTRL1, &val);
	dev_dbg(aw->dev, "I2SCTRL1 0x%04x\n", val);
	return 0;
}

static int aw882308_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)
		return -EINVAL;
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC)
		return -EINVAL;
	return 0;
}

static const struct snd_soc_dai_ops aw882308_dai_ops = {
	.hw_params = aw882308_hw_params,
	.set_fmt = aw882308_set_fmt,
	.mute_stream = aw882308_mute,
	.no_capture_mute = 1,
	/*
	 * The amps lock their PLL to BCLK. AudioReach starts the back-end
	 * graph, and so BCLK, in the CPU DAI trigger, after prepare, so
	 * unmute (and start) from trigger, once the CPU DAI has run.
	 */
	.mute_unmute_on_trigger = 1,
};

static struct snd_soc_dai_driver aw882308_dai = {
	.name = "aw882308-aif",
	.playback = {
		.stream_name = "Speaker Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &aw882308_dai_ops,
};

/* The control is a gain: 0 is the quietest allowed, max is min_atten. */
static int aw882308_vol_info(struct snd_kcontrol *kc, struct snd_ctl_elem_info *ui)
{
	ui->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	ui->count = 1;
	ui->value.integer.min = 0;
	ui->value.integer.max = AW_MUTE_VOL - min_atten;
	return 0;
}

static int aw882308_vol_get(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uc)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kc);
	struct aw882308 *aw = snd_soc_component_get_drvdata(c);

	uc->value.integer.value[0] = AW_MUTE_VOL - aw->atten;
	return 0;
}

static int aw882308_vol_put(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uc)
{
	struct snd_soc_component *c = snd_kcontrol_chip(kc);
	struct aw882308 *aw = snd_soc_component_get_drvdata(c);
	long v = uc->value.integer.value[0];

	if (v < 0 || v > AW_MUTE_VOL - min_atten)
		return -EINVAL;
	if (aw->atten == AW_MUTE_VOL - v)
		return 0;
	aw->atten = AW_MUTE_VOL - v;
	if (aw->running)
		aw882308_set_vol(aw, aw->atten);
	return 1;
}

static const struct snd_kcontrol_new aw882308_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Amp Volume",
		.info = aw882308_vol_info,
		.get = aw882308_vol_get,
		.put = aw882308_vol_put,
	},
};

static const struct snd_soc_dapm_widget aw882308_widgets[] = {
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route aw882308_routes[] = {
	{ "SPK", NULL, "Speaker Playback" },
};

static const struct snd_soc_component_driver aw882308_component = {
	.controls = aw882308_controls,
	.num_controls = ARRAY_SIZE(aw882308_controls),
	.dapm_widgets = aw882308_widgets,
	.num_dapm_widgets = ARRAY_SIZE(aw882308_widgets),
	.dapm_routes = aw882308_routes,
	.num_dapm_routes = ARRAY_SIZE(aw882308_routes),
};

/*
 * Optional register table, firmware "aw882308-<i2c addr>.bin": little-endian
 * u16 (reg, value) pairs, e.g. made from the phone's own ACF by
 * tools/aw-acf-regs.py. Never shipped in the repo.
 */
static void aw882308_apply_fw(struct aw882308 *aw, struct i2c_client *i2c)
{
	const struct firmware *fw;
	char name[32];
	unsigned int cur;
	size_t i;
	int n = 0;

	snprintf(name, sizeof(name), "aw882308-%02x.bin", i2c->addr);
	if (firmware_request_nowarn(&fw, name, aw->dev)) {
		dev_info(aw->dev, "no %s, running on register defaults\n", name);
		return;
	}
	for (i = 0; i + 4 <= fw->size; i += 4) {
		unsigned int reg = get_unaligned_le16(fw->data + i);
		unsigned int val = get_unaligned_le16(fw->data + i + 2);

		if (reg >= AW_REG_MAX || reg == AW_ID || reg == AW_SYSST ||
		    reg == AW_SYSINT)
			continue;
		if (reg == AW_SYSINTM)
			val = 0xffff;
		if (reg == AW_SYSCTRL) {
			/* Stay powered down and muted; no I/V feedback. */
			val |= SYSCTRL_PWDN | SYSCTRL_AMPPD | SYSCTRL_HMUTE |
			       SYSCTRL_ULS_HMUTE;
			val &= ~SYSCTRL_I2STXEN;
		}
		if (reg == AW_SYSCTRL2)
			val = (val & ~SYSCTRL2_VOL) | AW_MUTE_VOL;
		if (reg == AW_I2SCTRL1)
			aw->chsel = FIELD_GET(I2SCTRL1_CHSEL, val);
		regmap_write(aw->regmap, reg, val);
		n++;
	}
	release_firmware(fw);
	regmap_read(aw->regmap, AW_SYSCTRL, &cur);
	dev_info(aw->dev, "applied %d registers from %s, SYSCTRL 0x%04x chsel %u\n",
		 n, name, cur, aw->chsel);
}

static int aw882308_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct aw882308 *aw;
	unsigned int id = 0;
	int ret;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;
	aw->dev = dev;
	aw->atten = AW_MUTE_VOL;
	/* Stock: 0x34 plays the left channel, 0x35 the right. */
	aw->chsel = (i2c->addr & 1) ? 2 : 1;
	i2c_set_clientdata(i2c, aw);

	aw->regmap = devm_regmap_init_i2c(i2c, &aw882308_regmap);
	if (IS_ERR(aw->regmap))
		return PTR_ERR(aw->regmap);

	/* Active-low reset: request asserted, then release. */
	aw->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(aw->reset))
		return dev_err_probe(dev, PTR_ERR(aw->reset), "reset gpio\n");
	if (aw->reset) {
		usleep_range(1000, 1100);
		gpiod_set_value_cansleep(aw->reset, 0);
		usleep_range(2000, 2100);
	} else {
		dev_warn(dev, "no reset gpio\n");
	}

	ret = regmap_read(aw->regmap, AW_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "no answer on I2C\n");
	if (id != AW_CHIP_ID)
		return dev_err_probe(dev, -ENODEV, "chip id 0x%04x\n", id);

	regmap_write(aw->regmap, AW_ID, AW_SOFT_RESET);
	usleep_range(2000, 2100);
	if (use_fw)
		aw882308_apply_fw(aw, i2c);
	aw882308_stop(aw);

	/*
	 * min_atten is a module parameter; keep it inside the volume field
	 * so the control range cannot underflow.
	 */
	min_atten = min(min_atten, AW_MUTE_VOL);

	/* Start quiet: attenuation 700 of 1023, or min_atten if higher. */
	aw->atten = max(min_atten, 700u);
	dev_info(dev, "chip 0x%04x, chsel %u, atten floor %u\n", id, aw->chsel,
		 min_atten);

	return devm_snd_soc_register_component(dev, &aw882308_component,
					       &aw882308_dai, 1);
}

static void aw882308_remove(struct i2c_client *i2c)
{
	struct aw882308 *aw = i2c_get_clientdata(i2c);

	aw882308_stop(aw);
}

static const struct i2c_device_id aw882308_id[] = {
	{ "aw882308" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw882308_id);

static const struct of_device_id aw882308_of_match[] = {
	{ .compatible = "awinic,aw882308" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw882308_of_match);

static struct i2c_driver aw882308_driver = {
	.driver = {
		.name = "aw882308",
		.of_match_table = aw882308_of_match,
	},
	.probe = aw882308_probe,
	.remove = aw882308_remove,
	.id_table = aw882308_id,
};

module_i2c_driver(aw882308_driver);



MODULE_DESCRIPTION("Awinic 0x2308 speaker amplifier (bring-up)");
MODULE_LICENSE("GPL");
