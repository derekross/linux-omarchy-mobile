/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef AOC_SENSOR_POWER_H
#define AOC_SENSOR_POWER_H

#include <linux/mutex.h>

struct device;
struct regulator;

#define AOC_SENSOR_SUPPLIES 2

struct aoc_sensor_power {
	struct device *dev;
	struct regulator *supplies[AOC_SENSOR_SUPPLIES];
	/* Serializes the consumer votes and their power sequence. */
	struct mutex lock;
	unsigned long enabled;
};

int aoc_sensor_power_init(struct device *dev, struct aoc_sensor_power *power);
int aoc_sensor_power_set(struct aoc_sensor_power *power, bool enable);

#endif
