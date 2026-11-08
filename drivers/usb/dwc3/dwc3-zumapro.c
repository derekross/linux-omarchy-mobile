// SPDX-License-Identifier: GPL-2.0-only
/*
 * Tensor G4 peripheral USB: initialize the PHY before the first core read.
 * Probe structure based on dwc3-generic-plat.c:
 * Copyright (C) 2025 Ze Huang <huang.ze@linux.dev>
 */
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/usb/otg.h>

#include "glue.h"

struct dwc3_zumapro {
	struct dwc3 dwc;
	struct phy *usb2_phy;
	struct clk_bulk_data *clks;
};

static void dwc3_zumapro_phy_exit(void *data)
{
	struct dwc3_zumapro *zumapro = data;
	int ret;

	ret = phy_exit(zumapro->usb2_phy);
	if (ret)
		dev_warn(zumapro->dwc.dev, "Failed to exit USB2 PHY: %d\n", ret);
}

static int dwc3_zumapro_probe(struct platform_device *pdev)
{
	struct dwc3_probe_data probe_data = {
		.ignore_clocks_and_resets = true,
		.properties = DWC3_DEFAULT_PROPERTIES,
	};
	struct device *dev = &pdev->dev;
	struct dwc3_zumapro *zumapro;
	struct resource *res;
	int ret;

	/* Host/dual-role and suspend need their own power sequencing. */
	if (usb_get_dr_mode(dev) != USB_DR_MODE_PERIPHERAL)
		return dev_err_probe(dev, -EINVAL, "Only peripheral mode is supported\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	zumapro = devm_kzalloc(dev, sizeof(*zumapro), GFP_KERNEL);
	if (!zumapro)
		return -ENOMEM;
	zumapro->dwc.dev = dev;

	/* Check and enable all three clocks before accessing the PHY. */
	ret = devm_clk_bulk_get_all_enabled(dev, &zumapro->clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to check inherited USB clocks\n");
	if (ret != 3)
		return dev_err_probe(dev, -EINVAL, "Three USB clocks are required\n");

	zumapro->usb2_phy = devm_phy_get(dev, "usb2-phy");
	if (IS_ERR(zumapro->usb2_phy))
		return dev_err_probe(dev, PTR_ERR(zumapro->usb2_phy), "Failed to get USB2 PHY\n");

	/*
	 * DWC3 reads GSNPSID before its own generic PHY initialization. On
	 * Tensor, an isolated or reset link can hang that read. Initialize
	 * under checked clocks first and hold this reference until removal;
	 * core probe/remove balances its additional generic PHY reference.
	 */
	ret = phy_init(zumapro->usb2_phy);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to prepare USB link before core access\n");

	ret = devm_add_action_or_reset(dev, dwc3_zumapro_phy_exit, zumapro);
	if (ret)
		return ret;

	probe_data.dwc = &zumapro->dwc;
	probe_data.res = res;
	return dwc3_core_probe(&probe_data);
}

static void dwc3_zumapro_remove(struct platform_device *pdev)
{
	dwc3_core_remove(platform_get_drvdata(pdev));
}

static int dwc3_zumapro_suspend(struct device *dev)
{
	/* This staged path deliberately retains the firmware power state. */
	return -EBUSY;
}

static const struct dev_pm_ops dwc3_zumapro_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(dwc3_zumapro_suspend, NULL)
};

static const struct of_device_id dwc3_zumapro_match[] = {
	{ .compatible = "google,zumapro-dwc3" },
	{ }
};
MODULE_DEVICE_TABLE(of, dwc3_zumapro_match);

static struct platform_driver dwc3_zumapro_driver = {
	.probe = dwc3_zumapro_probe,
	.remove = dwc3_zumapro_remove,
	.driver = {
		.name = "dwc3-zumapro",
		.of_match_table = dwc3_zumapro_match,
		.pm = pm_sleep_ptr(&dwc3_zumapro_pm_ops),
	},
};
module_platform_driver(dwc3_zumapro_driver);

MODULE_DESCRIPTION("Tensor G4 peripheral USB with PHY-before-core ordering");
MODULE_LICENSE("GPL");
