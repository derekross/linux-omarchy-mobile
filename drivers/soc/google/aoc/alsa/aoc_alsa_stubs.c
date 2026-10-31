// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB audio and DisplayPort offload through the AoC are not built on this
 * kernel (they hook Android's USB audio driver); these stand in for them.
 * USB and DisplayPort audio work through the AP's own drivers instead.
 */
#include "aoc_alsa.h"

int aoc_usb_init(void)
{
	return 0;
}

void aoc_usb_exit(void)
{
}

int aoc_dp_init(void)
{
	return 0;
}

void aoc_dp_exit(void)
{
}

int aoc_set_usb_mem_config(struct aoc_chip *achip)
{
	return -EOPNOTSUPP;
}
