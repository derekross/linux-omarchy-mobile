# SPDX-License-Identifier: GPL-2.0
# Google's AoC ALSA/ASoC driver, gs201 port
ccflags-y += -I$(src) -I$(src)/.. -DCONFIG_SOC_GS201=1

obj-$(CONFIG_GS201_AOC_ALSA)	+= aoc_alsa_dev.o aoc_alsa_dev_util.o
aoc_alsa_dev_util-y := aoc_alsa_path.o aoc_alsa_drv.o aoc_alsa_pcm.o \
		       aoc_alsa_voice.o aoc_alsa_compr.o aoc_alsa_ctl.o \
		       aoc_alsa_hw.o aoc_alsa_nohost.o aoc_alsa_incall.o \
		       aoc_alsa_voip.o aoc_alsa_stubs.o
aoc_alsa_dev-y := aoc_alsa_card.o
