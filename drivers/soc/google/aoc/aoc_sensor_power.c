// SPDX-License-Identifier: GPL-2.0-only
/* Ordered ownership of the sensor supplies used by the Zumapro AoC. */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>

#include "aoc_sensor_power.h"

static const char * const aoc_sensor_supply_names[AOC_SENSOR_SUPPLIES] = {
	"sensor-1v8", "sensor-3v3",
};

/* Stop if a higher rail cannot be disabled: do not remove its lower rail. */
static int aoc_sensor_power_disable(struct aoc_sensor_power *power,
				    unsigned long mask)
{
	int i, ret;

	for (i = AOC_SENSOR_SUPPLIES - 1; i >= 0; i--) {
		if (!(power->enabled & mask & BIT(i)))
			continue;
		ret = regulator_disable(power->supplies[i]);
		if (ret)
			return ret;
		power->enabled &= ~BIT(i);
	}

	return 0;
}

int aoc_sensor_power_set(struct aoc_sensor_power *power, bool enable)
{
	unsigned long added = 0;
	int i, ret, rollback;

	mutex_lock(&power->lock);
	if (!enable) {
		ret = aoc_sensor_power_disable(power, GENMASK(AOC_SENSOR_SUPPLIES - 1, 0));
		goto out;
	}

	/* Individual enables retain each provider's settling delay and order. */
	for (i = 0; i < AOC_SENSOR_SUPPLIES; i++) {
		if (power->enabled & BIT(i)) {
			ret = regulator_is_enabled(power->supplies[i]);
			if (ret <= 0) {
				if (!ret)
					ret = -EIO;
				goto undo;
			}
			continue;
		}
		ret = regulator_is_enabled(power->supplies[i]);
		if (ret < 0)
			goto undo;
		/* Normal get starts with no consumer vote, even if hardware is on. */
		ret = regulator_enable(power->supplies[i]);
		if (ret)
			goto undo;
		power->enabled |= BIT(i);
		added |= BIT(i);
		ret = regulator_is_enabled(power->supplies[i]);
		if (ret <= 0) {
			if (!ret)
				ret = -EIO;
			goto undo;
		}
	}
	ret = 0;
	goto out;

undo:
	rollback = aoc_sensor_power_disable(power, added);
	if (rollback)
		dev_err(power->dev, "sensor supply rollback failed: %d\n", rollback);
out:
	mutex_unlock(&power->lock);
	return ret;
}

static void aoc_sensor_power_release(void *data)
{
	struct aoc_sensor_power *power = data;
	int ret;

	ret = aoc_sensor_power_set(power, false);
	if (ret)
		dev_err(power->dev, "sensor supply release failed: %d\n", ret);
}

int aoc_sensor_power_init(struct device *dev, struct aoc_sensor_power *power)
{
	int i;

	power->dev = dev;
	mutex_init(&power->lock);
	/* Both suppliers must exist before any AoC probe side effects. */
	if (!device_property_present(dev, "sensor-1v8-supply") ||
	    !device_property_present(dev, "sensor-3v3-supply"))
		return dev_err_probe(dev, -EINVAL, "both sensor supplies are required\n");

	for (i = 0; i < AOC_SENSOR_SUPPLIES; i++) {
		/* Optional-get rejects a missing supplier instead of using a dummy. */
		power->supplies[i] =
			devm_regulator_get_optional(dev, aoc_sensor_supply_names[i]);
		if (IS_ERR(power->supplies[i]))
			return dev_err_probe(dev, PTR_ERR(power->supplies[i]),
					     "failed to get %s supply\n",
					     aoc_sensor_supply_names[i]);
	}

	/* Registered last, so owned votes are dropped before the handles. */
	return devm_add_action_or_reset(dev, aoc_sensor_power_release, power);
}
