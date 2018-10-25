/*
 * ak4558.c
 *
 * Copyright (C) Matt Gattis
 * Matt Gattis <gattis@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <sound/soc.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>

#define AK4558_PWR_MGMT 0x00
#define AK4558_PMADR (1u<<4)
#define AK4558_PMADL (1u<<3)
#define AK4558_PMDAR (1u<<2)
#define AK4558_PMDAL (1u<<1)
#define AK4558_RSTN (1u)

#define AK4558_PLL_CTRL 0X01
#define AK4558_PLL3 (1u<<4)
#define AK4558_PLL2 (1u<<3)
#define AK4558_PLL1 (1u<<2)
#define AK4558_PLL0 (1u<<1)
#define AK4558_PMPLL (1u)

#define AK4558_DAC_TDM 0X02
#define AK4558_SDS1 (1u<<1)
#define AK4558_SDS0 (1u)

#define AK4558_CTRL_1 0X03
#define AK4558_TDM1 (1u<<7)
#define AK4558_TDM0 (1u<<6)
#define AK4558_DIF2 (1u<<5)
#define AK4558_DIF1 (1u<<4)
#define AK4558_DIF0 (1u<<3)
#define AK4558_ATS1 (1u<<2)
#define AK4558_ATS0 (1u<<1)
#define AK4558_SMUTE (1u)

#define AK4558_CTRL_2 0X04
#define AK4558_MCKS1 (1u<<4)
#define AK4558_MCKS0 (1u<<3)
#define AK4558_DFS1 (1u<<2)
#define AK4558_DFS0 (1u<<1)
#define AK4558_ACKS (1u)

#define AK4558_MODE_CTRL 0X05
#define AK4558_FS3 (1u<<6)
#define AK4558_FS2 (1u<<5)
#define AK4558_FS1 (1u<<4)
#define AK4558_FS0 (1u<<3)
#define AK4558_BCKO1 (1u<<2)
#define AK4558_BCKO0 (1u<<1)
#define AK4558_LOPS (1u<<0)

#define AK4558_FLTR_SET 0x06
#define AK4558_FIRDA2 (1u<<7)
#define AK4558_FIRDA1 (1u<<6)
#define AK4558_FIRDA0 (1u<<5)
#define AK4558_SLDA (1u<<4)
#define AK4558_SDDA (1u<<3)
#define AK4558_SSLOW (1u<<2)
#define AK4558_DEM1 (1u<<1)
#define AK4558_DEM0 (1u)

#define AK4558_HPF_EN_FLTR_SET 0x07
#define AK4558_SLAD (1u<<3)
#define AK4558_SDAD (1u<<2)
#define AK4558_HPFER (1u<<1)
#define AK4558_HPFEL (1u)

#define AK4558_LOUT_VOL 0x08
#define AK4558_ROUT_VOL 0x09

struct ak4558_priv {
	struct regmap *regmap;
	struct gpio_desc *pdn_gpio;
};

static const struct snd_soc_dapm_widget ak4558_dapm_widgets[] = {
SND_SOC_DAPM_INPUT("AINL"),
SND_SOC_DAPM_INPUT("AINR"),

SND_SOC_DAPM_OUTPUT("AOUTL"),
SND_SOC_DAPM_OUTPUT("AOUTR"),
};

static const struct reg_default ak4558_reg_defaults[] = {
	{AK4558_PWR_MGMT, AK4558_RSTN},
	{AK4558_PLL_CTRL, AK4558_PLL1},
	{AK4558_CTRL_1, AK4558_ATS1 | AK4558_DIF0 | AK4558_DIF1},
	{AK4558_MODE_CTRL, AK4558_BCKO0 | AK4558_FS0 | AK4558_FS2},
	{AK4558_FLTR_SET, AK4558_DEM0 | AK4558_SDDA | AK4558_FIRDA0},
	{AK4558_HPF_EN_FLTR_SET, AK4558_HPFEL | AK4558_HPFER | AK4558_SDAD},
};

static bool ak4558_volatile(struct device *dev, unsigned int reg) {
	return false;
}

static int ak4558_reset(struct snd_soc_component *codec) {
	struct ak4558_priv *ak4558 = snd_soc_component_get_drvdata(codec);	
	gpiod_set_value(ak4558->pdn_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value(ak4558->pdn_gpio, 1);
	usleep_range(1000, 2000);
	snd_soc_component_update_bits(codec, AK4558_PWR_MGMT, AK4558_RSTN, 0);
	usleep_range(1000, 2000);
	snd_soc_component_update_bits(codec, AK4558_PWR_MGMT, AK4558_RSTN, 1);
	usleep_range(1000, 2000);
	return 0;
}


static int ak4558_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir) {
	struct snd_soc_component *codec = dai->component;
	//struct ak4558_priv *ak4558 = snd_soc_codec_get_drvdata(codec);
	if (freq != 96000) {
		dev_err(codec->dev, "set sysclk wants freq=%d clkid=%d dir=%d\n", freq, clk_id, dir);
		return -EINVAL;
	}
	
	return 0;
}


static int ak4558_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai) {
	struct snd_soc_component *codec = dai->component;
	//struct ak4558_priv *ak4558 = snd_soc_codec_get_drvdata(codec);
	int rate = params_rate(params);
	if (rate != 96000) {
		dev_err(codec->dev, "hw params wants rate %d\n", rate);
		return -EINVAL;
	}

	snd_soc_component_write(codec, AK4558_PLL_CTRL, 0x4);
	snd_soc_component_write(codec, AK4558_CTRL_1, 0x38);
	snd_soc_component_write(codec, AK4558_MODE_CTRL, 0x3a);
	snd_soc_component_write(codec, AK4558_FLTR_SET, 0x29);
	snd_soc_component_write(codec, AK4558_HPF_EN_FLTR_SET, 0x4);
	snd_soc_component_write(codec, AK4558_PWR_MGMT, 0x1f);
	snd_soc_component_write(codec, AK4558_PLL_CTRL, 0x5);
	
	return 0;
}

static int ak4558_set_fmt(struct snd_soc_dai *dai, unsigned int fmt) {
	struct snd_soc_component *codec = dai->component;
	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
		dev_err(codec->dev, "set fmt asked for master mask not cbs_cfs: %u\n", fmt);
		return -EINVAL;
	}
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S) {
		dev_err(codec->dev, "set fmt asked for fmt mask not i2s: %u\n", fmt);
		return -EINVAL;
	}
	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF) {
		dev_err(codec->dev, "set fmt asked for inv mask not nb_nf: %u\n", fmt);
		return -EINVAL;
	}
	snd_soc_component_write(codec, AK4558_PLL_CTRL, 0x4);
	snd_soc_component_write(codec, AK4558_CTRL_1, 0x38);
	snd_soc_component_write(codec, AK4558_MODE_CTRL, 0x3a);
	snd_soc_component_write(codec, AK4558_FLTR_SET, 0x29);
	snd_soc_component_write(codec, AK4558_HPF_EN_FLTR_SET, 0x4);
	snd_soc_component_write(codec, AK4558_PWR_MGMT, 0x1f);
	snd_soc_component_write(codec, AK4558_PLL_CTRL, 0x5);

	return 0;
}
	

static const struct snd_soc_dapm_route ak4558_dapm_routes[] = {
	{ "Capture", NULL, "AINL" },
	{ "Capture", NULL, "AINR" },

	{ "AOUTL", NULL, "Playback" },
	{ "AOUTR", NULL, "Playback" },
};

static const struct snd_soc_dai_ops ak4558_dai_ops = {
	.hw_params    = ak4558_hw_params,
	.set_sysclk   = ak4558_set_sysclk,
	.set_fmt      = ak4558_set_fmt,
};

static struct snd_soc_dai_driver ak4558_dai = {
	.name = "ak4558-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_96000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_96000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,
	},
	.symmetric_rates = 1,
	.ops = &ak4558_dai_ops,
};


static int ak4558_probe(struct snd_soc_component *codec) {
	struct ak4558_priv *ak4558 = snd_soc_component_get_drvdata(codec);
	u32 regval;
	
	dev_info(codec->dev, "initializing ak4558 chip\n");
	
	regmap_read(ak4558->regmap, AK4558_MODE_CTRL, &regval);
	dev_info(codec->dev, "before reset mode ctrl: %u\n", regval);

	ak4558_reset(codec);	

	regmap_read(ak4558->regmap, AK4558_MODE_CTRL, &regval);
	dev_info(codec->dev, "after reset mode ctrl: %u\n", regval);
	
	snd_soc_component_write(codec, AK4558_PLL_CTRL, 0x4);
	snd_soc_component_write(codec, AK4558_CTRL_1, 0x38);
	snd_soc_component_write(codec, AK4558_MODE_CTRL, 0x3a);
	snd_soc_component_write(codec, AK4558_FLTR_SET, 0x29);
	snd_soc_component_write(codec, AK4558_HPF_EN_FLTR_SET, 0x4);
	snd_soc_component_write(codec, AK4558_PWR_MGMT, 0x1f);
	snd_soc_component_write(codec, AK4558_PLL_CTRL, 0x5);

	regmap_read(ak4558->regmap, AK4558_MODE_CTRL, &regval);
	dev_info(codec->dev, "after reset mode ctrl: %u\n", regval);
	return 0;
}

	
static const struct snd_soc_component_driver soc_codec_dev_ak4558 = {
	.probe = ak4558_probe,
	.dapm_widgets		= ak4558_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(ak4558_dapm_widgets),
	.dapm_routes		= ak4558_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(ak4558_dapm_routes),
};

static const struct of_device_id ak4558_of_match[] = {
	{ .compatible = "akm,ak4558", },
	{ }
};

MODULE_DEVICE_TABLE(of, ak4558_of_match);

static const struct regmap_config ak4558_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AK4558_ROUT_VOL,
	.volatile_reg = ak4558_volatile,
	.reg_defaults = ak4558_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ak4558_reg_defaults),
	.cache_type = REGCACHE_NONE,
};

static int ak4558_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id) {
	int ret;

	struct ak4558_priv *ak4558;

	dev_info(&i2c->dev, "ak4558 i2c getting probed\n");

	ak4558 = devm_kzalloc(&i2c->dev, sizeof(struct ak4558_priv), GFP_KERNEL);
	if (ak4558 == NULL) return -ENOMEM;
	i2c_set_clientdata(i2c, ak4558);
	
	ak4558->regmap = devm_regmap_init_i2c(i2c, &ak4558_regmap);
	if (IS_ERR(ak4558->regmap)) {
		ret = PTR_ERR(ak4558->regmap);
		dev_err(&i2c->dev, "Failed to create regmap: %d\n", ret);
		return ret;
	}

	ak4558->pdn_gpio = devm_gpiod_get(&i2c->dev, "pdn", GPIOD_ASIS);
	if (IS_ERR(ak4558->pdn_gpio)) {
		ret = PTR_ERR(ak4558->pdn_gpio);
		dev_err(&i2c->dev, "could not find pdn gpio\n");
		return ret;
	}


	ret = snd_soc_register_component(&i2c->dev, &soc_codec_dev_ak4558, &ak4558_dai, 1);
	if (ret < 0) {
		dev_err(&i2c->dev, "error registering ak4558 codec: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ak4558_i2c_remove(struct i2c_client *client) {
	return 0;
}

static const struct i2c_device_id ak4558_i2c_id[] = {
	{ "ak4558", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ak4558_i2c_id);

static struct i2c_driver ak4558_i2c_driver = {
	.driver = {
		.name = "ak4558",
		.of_match_table = of_match_ptr(ak4558_of_match),
	},
	.probe	= ak4558_i2c_probe,
	.remove	= ak4558_i2c_remove,
	.id_table = ak4558_i2c_id,
};
module_i2c_driver(ak4558_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SoC AK4558 driver");
MODULE_AUTHOR("Matt Gattis <gattis@gmail.com>");
