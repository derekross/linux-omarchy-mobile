// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google gs101 thermal sensors through the ACPM firmware (APM).
 *
 * On gs101 the TMU belongs to the APM: the kernel asks the ACPM firmware for
 * temperatures over its TMU channel instead of touching the TMU registers.
 * This driver only reads and polls; it programs no thresholds and no
 * interrupts, so the firmware's own hardware trips stay as the bootloader
 * left them.
 */

#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

/*
 * The sensors: BIG, MID, LITTLE, G3D, ISP, TPU, and on zuma and zumapro AUR,
 * in the firmware's order.
 */
#define GS101_ACPM_TMU_MAX_SENSORS	7

struct gs101_tmu_data {
	unsigned int acpm_chan;		/* ACPM channel of the TMU service */
	unsigned int nr_sensors;
};

struct gs101_tmu;

struct gs101_tmu_sensor {
	struct gs101_tmu *tmu;
	u8 id;
};

struct gs101_tmu {
	struct acpm_handle *handle;
	unsigned int acpm_chan;
	struct gs101_tmu_sensor sensors[GS101_ACPM_TMU_MAX_SENSORS];
};

static int gs101_tmu_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct gs101_tmu_sensor *sensor = thermal_zone_device_priv(tz);
	struct acpm_handle *handle = sensor->tmu->handle;
	int celsius, ret;

	ret = handle->ops->tmu.read_temp(handle, sensor->tmu->acpm_chan,
					 sensor->id, &celsius);
	if (ret)
		return ret;

	*temp = celsius * 1000;
	return 0;
}

static const struct thermal_zone_device_ops gs101_tmu_ops = {
	.get_temp = gs101_tmu_get_temp,
};

static int gs101_tmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gs101_tmu_data *data;
	struct thermal_zone_device *tz;
	struct gs101_tmu *tmu;
	int i, ret, zones = 0;

	data = of_device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	tmu = devm_kzalloc(dev, sizeof(*tmu), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;
	tmu->acpm_chan = data->acpm_chan;

	tmu->handle = devm_acpm_get_by_node(dev, dev->parent->of_node);
	if (IS_ERR(tmu->handle))
		return dev_err_probe(dev, PTR_ERR(tmu->handle),
				     "failed to get the ACPM handle\n");

	ret = tmu->handle->ops->tmu.init(tmu->handle, tmu->acpm_chan);
	if (ret)
		return dev_err_probe(dev, ret, "ACPM TMU init failed\n");

	for (i = 0; i < data->nr_sensors; i++) {
		tmu->sensors[i].tmu = tmu;
		tmu->sensors[i].id = i;

		tz = devm_thermal_of_zone_register(dev, i, &tmu->sensors[i],
						   &gs101_tmu_ops);
		if (IS_ERR(tz)) {
			/* A sensor without a thermal zone in the DT */
			if (PTR_ERR(tz) == -ENODEV)
				continue;
			return dev_err_probe(dev, PTR_ERR(tz),
					     "sensor %d: no thermal zone\n", i);
		}
		zones++;
	}

	dev_info(dev, "%d thermal zones\n", zones);
	return 0;
}

static const struct gs101_tmu_data gs101_tmu_data = {
	.acpm_chan = 9,
	.nr_sensors = 6,
};

/* zuma's TMU service is zumapro's: channel 9, the same seven sensors. */
static const struct gs101_tmu_data zumapro_tmu_data = {
	.acpm_chan = 9,
	.nr_sensors = 7,
};

static const struct of_device_id gs101_tmu_of_match[] = {
	{ .compatible = "google,gs101-acpm-tmu", .data = &gs101_tmu_data },
	{ .compatible = "google,zuma-acpm-tmu", .data = &zumapro_tmu_data },
	{ .compatible = "google,zumapro-acpm-tmu", .data = &zumapro_tmu_data },
	{ }
};
MODULE_DEVICE_TABLE(of, gs101_tmu_of_match);

static struct platform_driver gs101_tmu_driver = {
	.probe = gs101_tmu_probe,
	.driver = {
		.name = "gs101-acpm-tmu",
		.of_match_table = gs101_tmu_of_match,
	},
};
module_platform_driver(gs101_tmu_driver);

MODULE_DESCRIPTION("Google gs101 thermal sensors through ACPM");
MODULE_LICENSE("GPL");
