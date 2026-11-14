// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * linux/drivers/media/platform/samsung/s5p-mfc/s5p_mfc_pm.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include "s5p_mfc_common.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_pm.h"

int s5p_mfc_init_pm(struct s5p_mfc_dev *dev)
{
	struct s5p_mfc_pm *pm = &dev->pm;
	int i;

	pm->num_clocks = dev->variant->num_clocks;
	pm->clk_names = dev->variant->clk_names;
	pm->device = &dev->plat_dev->dev;
	pm->clock_gate = NULL;

	/* clock control */
	for (i = 0; i < pm->num_clocks; i++) {
		if (dev->variant->clocks_optional)
			pm->clocks[i] = devm_clk_get_optional(pm->device,
							      pm->clk_names[i]);
		else
			pm->clocks[i] = devm_clk_get(pm->device,
						     pm->clk_names[i]);
		if (IS_ERR(pm->clocks[i])) {
			/* additional clocks are optional */
			if (i && PTR_ERR(pm->clocks[i]) == -ENOENT) {
				pm->clocks[i] = NULL;
				continue;
			}
			mfc_err("Failed to get clock: %s\n",
				pm->clk_names[i]);
			return PTR_ERR(pm->clocks[i]);
		}
	}

	if (dev->variant->use_clock_gating)
		pm->clock_gate = pm->clocks[0];

	pm_runtime_enable(pm->device);
	return 0;
}

void s5p_mfc_final_pm(struct s5p_mfc_dev *dev)
{
	pm_runtime_disable(dev->pm.device);
}

int s5p_mfc_clock_on(struct s5p_mfc_dev *dev)
{
	return clk_enable(dev->pm.clock_gate);
}

void s5p_mfc_clock_off(struct s5p_mfc_dev *dev)
{
	clk_disable(dev->pm.clock_gate);
}

int s5p_mfc_power_on(struct s5p_mfc_dev *dev)
{
	int i, ret = 0;

	ret = pm_runtime_resume_and_get(dev->pm.device);
	if (ret < 0)
		return ret;

	/* clock control */
	for (i = 0; i < dev->pm.num_clocks; i++) {
		ret = clk_prepare_enable(dev->pm.clocks[i]);
		if (ret < 0) {
			mfc_err("clock prepare failed for clock: %s\n",
				dev->pm.clk_names[i]);
			goto err;
		}
	}

	/*
	 * Run at the variant's rate while the MFC is in use. The bootloader
	 * leaves the GS201 MFC at 100 MHz, a quarter of what 4K needs.
	 */
	if (dev->variant->active_clk_rate && dev->pm.clocks[0]) {
		dev->pm.idle_clk_rate = clk_get_rate(dev->pm.clocks[0]);
		ret = clk_set_rate(dev->pm.clocks[0],
				   dev->variant->active_clk_rate);
		if (ret)
			mfc_err("Failed to set the MFC clock to %lu Hz: %d\n",
				dev->variant->active_clk_rate, ret);
		ret = 0;
	}

	/* prepare for software clock gating */
	clk_disable(dev->pm.clock_gate);

	return 0;
err:
	while (--i >= 0)
		clk_disable_unprepare(dev->pm.clocks[i]);
	pm_runtime_put(dev->pm.device);
	return ret;
}

int s5p_mfc_power_off(struct s5p_mfc_dev *dev)
{
	int i;

	/* finish software clock gating */
	clk_enable(dev->pm.clock_gate);

	if (dev->pm.idle_clk_rate) {
		clk_set_rate(dev->pm.clocks[0], dev->pm.idle_clk_rate);
		dev->pm.idle_clk_rate = 0;
	}

	for (i = 0; i < dev->pm.num_clocks; i++)
		clk_disable_unprepare(dev->pm.clocks[i]);

	return pm_runtime_put_sync(dev->pm.device);
}

