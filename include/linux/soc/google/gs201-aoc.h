/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_GOOGLE_GS201_AOC_H
#define _LINUX_SOC_GOOGLE_GS201_AOC_H

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/of.h>

#if IS_REACHABLE(CONFIG_GS201_AOC)
bool gs201_aoc_firmware_online(const struct device_node *np);
#else
static inline bool gs201_aoc_firmware_online(const struct device_node *np)
{
	return false;
}
#endif

/* Optional for other boards; a malformed dependency must not permit power. */
static inline int gs201_aoc_require_online(struct device *dev)
{
	struct device_node *np;

	if (!of_find_property(dev->of_node, "google,aoc", NULL))
		return 0;

	np = of_parse_phandle(dev->of_node, "google,aoc", 0);
	if (!np)
		return -EINVAL;
	if (!gs201_aoc_firmware_online(np)) {
		of_node_put(np);
		return -EPROBE_DEFER;
	}
	of_node_put(np);
	return 0;
}

#endif
