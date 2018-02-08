/*
 * ASoC driver for Ohmbre Module (with an AK4558)
 * connected to a Raspberry Pi
 *
 * Author:      Matt Gattis <gattis@gmail.com>
 *	      Copyright 2017
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>


static struct snd_soc_dai_link snd_ohmbre_codec_dai = {
	.name		= "AK4885",
	.stream_name	= "AK4558 HiFi",
	.cpu_dai_name	= "bcm2708-i2s.0",
	.codec_dai_name	= "ak4558-hifi",
	.platform_name	= "bcm2708-i2s.0",
	.codec_name	= "ak4558.0-0013",
	.dai_fmt	= SND_SOC_DAIFMT_I2S
				| SND_SOC_DAIFMT_NB_NF
				| SND_SOC_DAIFMT_CBS_CFS,
};

/* audio machine driver */
static struct snd_soc_card snd_ohmbre_codec = {
	.name		= "snd_ohmbre_codec",
	.owner		= THIS_MODULE,
	.dai_link	= &snd_ohmbre_codec_dai,
	.num_links	= 1,
};

static int snd_ohmbre_codec_probe(struct platform_device *pdev)
{
	int ret = 0;

	snd_ohmbre_codec.dev = &pdev->dev;

	if (pdev->dev.of_node) {
		struct device_node *i2s_node;
		struct snd_soc_dai_link *dai = &snd_ohmbre_codec_dai;
		i2s_node = of_parse_phandle(pdev->dev.of_node,
				            "i2s-controller", 0);

		if (i2s_node) {
			dai->cpu_dai_name = NULL;
			dai->cpu_of_node = i2s_node;
			dai->platform_name = NULL;
			dai->platform_of_node = i2s_node;
		}
	}

	ret = snd_soc_register_card(&snd_ohmbre_codec);
	if (ret && ret != -EPROBE_DEFER)
		dev_err(&pdev->dev,
				"snd_soc_register_card() failed: %d\n", ret);

	return ret;
}


static int snd_ohmbre_codec_remove(struct platform_device *pdev)
{
	return snd_soc_unregister_card(&snd_ohmbre_codec);
}

static const struct of_device_id snd_ohmbre_codec_of_match[] = {
	{ .compatible = "ohmbre,ohmbre-codec", },
	{},
};
MODULE_DEVICE_TABLE(of, snd_ohmbre_codec_of_match);

static struct platform_driver snd_ohmbre_codec_driver = {
	.driver = {
		.name   = "snd-ohmbre-codec",
		.owner  = THIS_MODULE,
		.of_match_table = snd_ohmbre_codec_of_match,
	},
	.probe	  = snd_ohmbre_codec_probe,
	.remove	 = snd_ohmbre_codec_remove,
};

module_platform_driver(snd_ohmbre_codec_driver);

MODULE_AUTHOR("Florian Meier");
MODULE_DESCRIPTION("ASoC Driver for Ohmbre Codec (AK4558)");
MODULE_LICENSE("GPL");
