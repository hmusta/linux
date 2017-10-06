/*
 * ASoC Driver for HifiBerry Digi
 *
 * Author: Daniel Matuschek <info@crazy-audio.com>
 * based on the HifiBerry DAC driver by Florian Meier <florian.meier@koalo.de>
 *	Copyright 2013
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/gpio/consumer.h>

#include "../codecs/wm8804.h"

static short int auto_shutdown_output = 0;
module_param(auto_shutdown_output, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(auto_shutdown_output, "Shutdown SP/DIF output if playback is stopped");

#define CLK_44EN_RATE 22579200UL
#define CLK_48EN_RATE 24576000UL
#define SYSCLK 27000000L

static bool snd_rpi_hifiberry_is_digipro;
static struct gpio_desc *snd_rpi_hifiberry_clk44gpio;
static struct gpio_desc *snd_rpi_hifiberry_clk48gpio;

static int samplerate=44100;

static uint32_t snd_rpi_hifiberry_digi_enable_clock(int sample_rate)
{
	switch (sample_rate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
		gpiod_set_value_cansleep(snd_rpi_hifiberry_clk44gpio, 1);
		gpiod_set_value_cansleep(snd_rpi_hifiberry_clk48gpio, 0);
		return CLK_44EN_RATE;
	default:
		gpiod_set_value_cansleep(snd_rpi_hifiberry_clk48gpio, 1);
		gpiod_set_value_cansleep(snd_rpi_hifiberry_clk44gpio, 0);
		return CLK_48EN_RATE;
	}
}

static void snd_rpi_hifiberry_digi_proc_registers(struct snd_info_entry *entry,
                           struct snd_info_buffer *buffer)
{
	/* Dump wm8804 registers. 
	   See https://www.cirrus.com/en/pubs/proDatasheet/WM8804_v4.5.pdf */
	
	struct snd_soc_codec *codec = entry->private_data;
	unsigned int prescale, pll_n, pll_k, freq_mode, temp;
	const unsigned int freq_mode_divs[] = {2, 4, 8, 12};
	const char * req_freq[] = { "192", "96 or 88.2", "48 or 44.1", "32" }; 

	const char * iec60958_3_frequencies[] = 
		{  "44.1", "?",  "48", "32", /* 0 1 2 3 */
		      "?", "?",   "?",  "?", /* 4 5 6 7 */
		   "88.2", "?",  "96",  "?", /* 8 9 a b */
		  "176.4", "?", "192",  "?"  /* c d e f */ }; 
	unsigned int lock;
	unsigned int pll_clock;

	temp = snd_soc_read(codec, WM8804_PLL4);
	prescale = (temp >> 4) & 0x1;

	pll_n = temp & 0xf;
	pll_k = snd_soc_read(codec, WM8804_PLL1) | 
		(snd_soc_read(codec, WM8804_PLL2) << 8) |
		(snd_soc_read(codec, WM8804_PLL3) << 16);

	temp = snd_soc_read(codec, WM8804_PWRDN);
	snd_iprintf(buffer, "pwrdn:    0x%02x\n", temp);
	snd_iprintf(buffer, " pll      %s\n", temp & (1     ) ? "off" : "ON");
	snd_iprintf(buffer, " spdif rx %s\n", temp & (1 << 1) ? "off" : "ON");
	snd_iprintf(buffer, " spdif tx %s\n", temp & (1 << 2) ? "off" : "ON");
	snd_iprintf(buffer, " osc      %s\n", temp & (1 << 3) ? "off" : "ON");
	snd_iprintf(buffer, " aif      %s\n", temp & (1 << 4) ? "off" : "ON");
	snd_iprintf(buffer, " trio     %s\n", temp & (1 << 5) ? "off" : "ON");

	snd_iprintf(buffer, "prescale  %d\n", prescale);
	snd_iprintf(buffer, "pll_n     %d\n", pll_n);
	snd_iprintf(buffer, "pll_k     0x%06x\n", pll_k);
	freq_mode = snd_soc_read(codec, WM8804_PLL5) & 0x3;
	snd_iprintf(buffer, "freq mode 0x%02x\n", freq_mode);

	pll_clock = (u32) (SYSCLK * pll_n + ((SYSCLK * (u64)pll_k) >> 22) ) / 
		( (prescale ? 2 : 1) *  freq_mode_divs[freq_mode] ) ;

	snd_iprintf(buffer, "pll clock %d Hz\n", pll_clock);

	temp = snd_soc_read(codec, WM8804_SPDSTAT) & 0x7f;
	snd_iprintf(buffer, "spdstat:  0x%02x\n", temp );
	lock = (temp & (1 << 6 )) == 0;
	snd_iprintf(buffer, " lock     %s\n", lock ? "ON" : "off");
	if (lock)
	{
		snd_iprintf(buffer, " rec freq %s Khz\n", req_freq[(temp >> 4) & 0x3]);
	}
	snd_iprintf(buffer, "rxchan1:  0x%02x\n", snd_soc_read(codec, WM8804_RXCHAN1) );
	snd_iprintf(buffer, "rxchan2:  0x%02x\n", snd_soc_read(codec, WM8804_RXCHAN2) );
	snd_iprintf(buffer, "rxchan3:  0x%02x\n", snd_soc_read(codec, WM8804_RXCHAN3) );
	temp = snd_soc_read(codec, WM8804_RXCHAN4);
	snd_iprintf(buffer, "rxchan4:  0x%02x\n", temp );
	if (lock)
	{
		snd_iprintf(buffer, " ind freq %s Khz\n", iec60958_3_frequencies[temp & 0xf]);
	}

	temp = snd_soc_read(codec, WM8804_RXCHAN5);
	snd_iprintf(buffer, "rxchan5:  0x%02x\n", temp );
	if (lock)
	{
		snd_iprintf(buffer, " org freq %s Khz\n", iec60958_3_frequencies[(temp >> 4) & 0xf]);
	}

	snd_iprintf(buffer, "intstat:  0x%02x\n", snd_soc_read(codec, WM8804_INTSTAT) );
	snd_iprintf(buffer, "spdrx1:   0x%02x\n", snd_soc_read(codec, WM8804_SPDRX1) );	  
	snd_iprintf(buffer, "spdtx1:   0x%02x\n", snd_soc_read(codec, WM8804_SPDTX1) );
	snd_iprintf(buffer, "spdtx2:   0x%02x\n", snd_soc_read(codec, WM8804_SPDTX2) );
	snd_iprintf(buffer, "spdtx3:   0x%02x\n", snd_soc_read(codec, WM8804_SPDTX3) );
	snd_iprintf(buffer, "spdtx4:   0x%02x\n", snd_soc_read(codec, WM8804_SPDTX4) );
	snd_iprintf(buffer, "spdtx5:   0x%02x\n", snd_soc_read(codec, WM8804_SPDTX5) );

}


static int snd_rpi_hifiberry_digi_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_info_entry *entry;

	/* enable TX output */
	snd_soc_update_bits(codec, WM8804_PWRDN, 0x4, 0x0);
	
	/* disable SCMS */
	snd_soc_update_bits(codec, WM8804_SPDTX1, 0xff, 0x04);

	/* Initialize Digi+ Pro hardware */
	if (snd_rpi_hifiberry_is_digipro) {
		struct snd_soc_dai_link *dai = rtd->dai_link;

		dai->name = "HiFiBerry Digi+ Pro";
		dai->stream_name = "HiFiBerry Digi+ Pro HiFi";
	}

	if (! snd_card_proc_new(rtd->card->snd_card, "registers", &entry)) {
			snd_info_set_text_ops(entry, rtd->codec, snd_rpi_hifiberry_digi_proc_registers);
	}

	return 0;
}

static int snd_rpi_hifiberry_digi_startup(struct snd_pcm_substream *substream) {
	/* turn on digital output */
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	snd_soc_update_bits(codec, WM8804_PWRDN, 0x3c, 0x00);
	return 0;
}

static void snd_rpi_hifiberry_digi_shutdown(struct snd_pcm_substream *substream) {
	/* turn off output and input  */
	if (auto_shutdown_output) {
		/* turn off output and input */
		struct snd_soc_pcm_runtime *rtd = substream->private_data;
		struct snd_soc_codec *codec = rtd->codec;
		snd_soc_update_bits(codec, WM8804_PWRDN, 0x3f, 0x3f);
	}
}


static int snd_rpi_hifiberry_digi_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	int sysclk = SYSCLK; /* This is fixed on this board */

	long mclk_freq=0;
	int mclk_div=1;
	int sampling_freq=1;

	int ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) 
	{

		samplerate = params_rate(params);

		if (samplerate<=96000) {
			mclk_freq=samplerate*256;
			mclk_div=WM8804_MCLKDIV_256FS;
		} else {
			mclk_freq=samplerate*128;
			mclk_div=WM8804_MCLKDIV_128FS;
		}

		if (snd_rpi_hifiberry_is_digipro)
			sysclk = snd_rpi_hifiberry_digi_enable_clock(samplerate);
		
		switch (samplerate) {
			case 32000:
				sampling_freq=0x03;
				break;
			case 44100:
				sampling_freq=0x00;
				break;
			case 48000:
				sampling_freq=0x02;
				break;
			case 88200:
				sampling_freq=0x08;
				break;
			case 96000:
				sampling_freq=0x0a;
				break;
			case 176400:
				sampling_freq=0x0c;
				break;
			case 192000:
				sampling_freq=0x0e;
				break;
			default:
				dev_err(codec->dev,
				"Failed to set WM8804 SYSCLK, unsupported samplerate %d\n",
				samplerate);
		}

		snd_soc_dai_set_clkdiv(codec_dai, WM8804_MCLK_DIV, mclk_div);
		snd_soc_dai_set_pll(codec_dai, 0, 0, sysclk, mclk_freq);

		ret = snd_soc_dai_set_sysclk(codec_dai, WM8804_TX_CLKSRC_PLL,
						sysclk, SND_SOC_CLOCK_OUT);

		if (ret < 0) {
			dev_err(codec->dev,
			"Failed to set WM8804 SYSCLK: %d\n", ret);
			return ret;
		}

		/* Enable TX output */
		snd_soc_update_bits(codec, WM8804_PWRDN, 0x4, 0x0);

		/* Power on */
		snd_soc_update_bits(codec, WM8804_PWRDN, 0x19, 0);

		/* set sampling frequency status bits */
		snd_soc_update_bits(codec, WM8804_SPDTX4, 0x0f, sampling_freq);
	} else /* SNDRV_PCM_STREAM_CAPTURE */ {

		/* Table 31 S/PDIF Receive mode Initial Configuration Examples, for 27 Mhz 
	  	   and 32/44.1/48/88.2/96 Khz sample rates:
	   	   PRE-SCALE (WM8804_PLL4 << 4) & 0x1 -> 1
	       PLL_N     (WM8804_PLL4 &	0xf)      -> 6	
	       PLL_K     (WM8804_PLL3 << 16 | WM8804_PLL2 << 8 | WM8804_PLL1) -> 3f19e5 */

	    /* Write appropriate values for prescale, pll_n and pll_k */
		snd_soc_update_bits(codec, WM8804_PLL4, 0x1f, (1 << 4 ) | 6);
	
		snd_soc_update_bits(codec, WM8804_PLL1, 0xff, 0xe5);
		snd_soc_update_bits(codec, WM8804_PLL2, 0xff, 0x19);
		snd_soc_update_bits(codec, WM8804_PLL3, 0xff, 0x3f);

		/* Set PLL to fractional mode */
		snd_soc_update_bits(codec, WM8804_PLL5, 0x04, 0x04);

		/* Enable PLL */
		snd_soc_update_bits(codec, WM8804_PWRDN, 0x01, 0x0);

		/* Enable SPDIF receiver */
		snd_soc_update_bits(codec, WM8804_PWRDN, 0x02, 0x0);

		/* Power on*/
		snd_soc_update_bits(codec, WM8804_PWRDN, 0x18, 0x0);

	}
	return snd_soc_dai_set_bclk_ratio(cpu_dai,64);
}

/* machine stream operations */
static struct snd_soc_ops snd_rpi_hifiberry_digi_ops = {
	.hw_params = snd_rpi_hifiberry_digi_hw_params,
        .startup = snd_rpi_hifiberry_digi_startup,
        .shutdown = snd_rpi_hifiberry_digi_shutdown,
};

static struct snd_soc_dai_link snd_rpi_hifiberry_digi_dai[] = {
{
	.name		= "HifiBerry Digi",
	.stream_name	= "HifiBerry Digi HiFi",
	.cpu_dai_name	= "bcm2708-i2s.0",
	.codec_dai_name	= "wm8804-spdif",
	.platform_name	= "bcm2708-i2s.0",
	.codec_name	= "wm8804.1-003b",
	.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBM_CFM,
	.ops		= &snd_rpi_hifiberry_digi_ops,
	.init		= snd_rpi_hifiberry_digi_init,
},
};

/* audio machine driver */
static struct snd_soc_card snd_rpi_hifiberry_digi = {
	.name         = "snd_rpi_hifiberry_digi",
	.owner        = THIS_MODULE,
	.dai_link     = snd_rpi_hifiberry_digi_dai,
	.num_links    = ARRAY_SIZE(snd_rpi_hifiberry_digi_dai),
};

static int snd_rpi_hifiberry_digi_probe(struct platform_device *pdev)
{
	int ret = 0;

	snd_rpi_hifiberry_digi.dev = &pdev->dev;

	if (pdev->dev.of_node) {
	    struct device_node *i2s_node;
	    struct snd_soc_dai_link *dai = &snd_rpi_hifiberry_digi_dai[0];
	    i2s_node = of_parse_phandle(pdev->dev.of_node,
					"i2s-controller", 0);

	    if (i2s_node) {
		dai->cpu_dai_name = NULL;
		dai->cpu_of_node = i2s_node;
		dai->platform_name = NULL;
		dai->platform_of_node = i2s_node;
	    }

	    snd_rpi_hifiberry_is_digipro = 1;

	    snd_rpi_hifiberry_clk44gpio =
		devm_gpiod_get(&pdev->dev, "clock44", GPIOD_OUT_LOW);
	    if (IS_ERR(snd_rpi_hifiberry_clk44gpio))
		snd_rpi_hifiberry_is_digipro = 0;

	    snd_rpi_hifiberry_clk48gpio =
		devm_gpiod_get(&pdev->dev, "clock48", GPIOD_OUT_LOW);
	    if (IS_ERR(snd_rpi_hifiberry_clk48gpio))
		snd_rpi_hifiberry_is_digipro = 0;

	}

	ret = snd_soc_register_card(&snd_rpi_hifiberry_digi);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n", ret);

	return ret;
}

static int snd_rpi_hifiberry_digi_remove(struct platform_device *pdev)
{
	return snd_soc_unregister_card(&snd_rpi_hifiberry_digi);
}

static const struct of_device_id snd_rpi_hifiberry_digi_of_match[] = {
	{ .compatible = "hifiberry,hifiberry-digi", },
	{},
};
MODULE_DEVICE_TABLE(of, snd_rpi_hifiberry_digi_of_match);

static struct platform_driver snd_rpi_hifiberry_digi_driver = {
	.driver = {
		.name   = "snd-hifiberry-digi",
		.owner  = THIS_MODULE,
		.of_match_table = snd_rpi_hifiberry_digi_of_match,
	},
	.probe          = snd_rpi_hifiberry_digi_probe,
	.remove         = snd_rpi_hifiberry_digi_remove,
};

module_platform_driver(snd_rpi_hifiberry_digi_driver);

MODULE_AUTHOR("Daniel Matuschek <info@crazy-audio.com>");
MODULE_DESCRIPTION("ASoC Driver for HifiBerry Digi");
MODULE_LICENSE("GPL v2");
