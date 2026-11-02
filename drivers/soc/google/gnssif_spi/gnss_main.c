// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Samsung Electronics.
 *
 */
#include <linux/devcoredump.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/vmalloc.h>

#include "gnss_prj.h"
#include "gnss_spi.h"
#include "gnss_utils.h"

/* Hand a dump to devcoredump, which frees it. */
static void gnss_set_coredump(struct device *dev, void *dump, size_t len,
			      const char *reason)
{
	gif_err("GNSS coredump: %s (%zu bytes)\n", reason, len);
	dev_coredumpv(dev, dump, len, GFP_KERNEL);
}

static ssize_t coredump_store(struct device *dev, struct device_attribute *attr, const char *buf,
			      size_t count)
{
	int symbol = (int)';';
	const char *next = NULL;
	char *coredump = NULL;
	char *reason = NULL;
	int length = 0;
	static time64_t last_trigger_time;
	time64_t trigger_time = ktime_get_seconds();

	// Ignore the trigger time less than 30 seconds
	if (trigger_time - last_trigger_time < 30) {
		goto clean;
	}
	last_trigger_time = trigger_time;

	gif_err("Trigger Coredump string: %s\n", buf);
	next = strchr(buf, symbol);
	if (next) {
		length = (int)(next - buf);
		reason = kmalloc(length + 1, GFP_KERNEL);
		if (!reason) {
			gif_err("Allocate crash reason failed\n");
			goto clean;
		}
		memcpy(reason, buf, length);
		reason[length] = '\0';
		length = strlen(next + 1);
		coredump = vmalloc(length);
		if (!coredump) {
			gif_err("Allocate crash coredump failed\n");
			goto clean;
		}
		memcpy(coredump, next + 1, length);
		gnss_set_coredump(dev, coredump, length, reason);
		coredump = NULL;
	}
clean:
	kfree(reason);
	vfree(coredump);
	return count;
}

static DEVICE_ATTR_WO(coredump);

static struct attribute *gnss_attrs[] = {
	&dev_attr_coredump.attr,
	NULL
};
ATTRIBUTE_GROUPS(gnss);

static int parse_dt_common_pdata(struct device_node *np, struct gnss_pdata *pdata)
{
	/*
	 * Google's device tree names these in device,name and
	 * device_node_name. They describe no hardware, and the GNSS daemon
	 * opens /dev/gnss_ipc, so they are fixed here.
	 */
	pdata->name = "KEPLER";
	pdata->node_name = "gnss_ipc";

	gif_info("device name: %s node name: %s\n", pdata->name, pdata->node_name);
	return 0;
}

static struct gnss_pdata *gnss_if_parse_dt_pdata(struct device *dev)
{
	struct gnss_pdata *pdata;
	int ret = 0;

	pdata = devm_kzalloc(dev, sizeof(struct gnss_pdata), GFP_KERNEL);
	if (!pdata) {
		gif_err("gnss_pdata: alloc fail\n");
		return ERR_PTR(-ENOMEM);
	}
	dev->platform_data = pdata;

	ret = parse_dt_common_pdata(dev->of_node, pdata);
	if (ret != 0) {
		gif_err("Failed to parse common pdata from dt\n");
		goto parse_dt_pdata_err;
	}

	gif_info("DT parse complete!\n");
	return pdata;

parse_dt_pdata_err:
	if (pdata)
		devm_kfree(dev, pdata);
	dev->platform_data = NULL;

	return ERR_PTR(-EINVAL);
}

static void gnss_remove(struct platform_device *pdev)
{
	struct gnss_ctl *gc = platform_get_drvdata(pdev);
	struct link_device *ld = gc->iod->ld;

	/*
	 * The read work enables the interrupt again when it ends, so stop it
	 * before the interrupt is disabled for good. devm then releases the
	 * device node, the work queue and the interrupt.
	 */
	gif_disable_irq_sync(&gc->irq_gnss2ap_spi);
	cancel_delayed_work_sync(&ld->rx_dwork);
	gif_disable_irq_sync(&gc->irq_gnss2ap_spi);

	gpiod_set_value(gc->gpio_ap2gnss_spi.desc, 0);
}

static int gnss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gnss_pdata *pdata = dev->platform_data;
	struct gnss_ctl *gc;
	struct io_device *iod;
	struct link_device *ld;

	gif_info("Exynos GNSS interface driver probe begins\n");

	gif_info("%s: +++\n", pdev->name);

	if (!dev->of_node) {
		gif_err("No DT data!\n");
		return -ENODEV;
	}

	/*
	 * The core's requests are read over gnss_spi: without its device
	 * the read work would find nothing to read from while the request
	 * line stays high.
	 */
	if (!gnss_spi_bound())
		return dev_err_probe(dev, -EPROBE_DEFER, "no GNSS SPI device yet\n");

	pdata = gnss_if_parse_dt_pdata(dev);
	if (IS_ERR(pdata)) {
		gif_err("DT parse error!\n");
		return PTR_ERR(pdata);
	}

	/* The lines' controller may come later: keep -EPROBE_DEFER. */
	gc = create_ctl_device(pdev);
	if (IS_ERR(gc))
		return dev_err_probe(dev, PTR_ERR(gc), "%s: Could not create gc\n",
				     pdata->name);

	ld = create_link_device(pdev);
	if (IS_ERR(ld))
		return dev_err_probe(dev, PTR_ERR(ld), "%s: Could not create ld\n",
				     pdata->name);

	ld->gc = gc;

	iod = create_io_device(pdev, ld, gc, pdata);
	if (IS_ERR(iod))
		return dev_err_probe(dev, PTR_ERR(iod), "%s: Could not create iod\n",
				     pdata->name);

	platform_set_drvdata(pdev, gc);

	/* wa: to prevent wrong irq handling during probe */
	gif_enable_irq(&gc->irq_gnss2ap_spi);

	gif_info("%s: ---\n", pdev->name);

	return 0;
}

static const struct of_device_id gnss_dt_match[] = {
	{
		.compatible = "samsung,exynos-gnss",
	},
	{},
};
MODULE_DEVICE_TABLE(of, gnss_dt_match);

static struct platform_driver gnss_driver = {
		.probe = gnss_probe,
		.remove = gnss_remove,
		.driver = {
				.name = "gnss_interface",
				.dev_groups = gnss_groups,
				/* gnssd may hold the device node open */
				.suppress_bind_attrs = true,
#if IS_ENABLED(CONFIG_OF)
				.of_match_table = of_match_ptr(gnss_dt_match),
#endif
		},
};

module_platform_driver(gnss_driver);

MODULE_DESCRIPTION("Exynos GNSS interface driver");
MODULE_LICENSE("GPL");
