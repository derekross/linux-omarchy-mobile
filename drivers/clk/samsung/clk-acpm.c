// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos ACPM protocol based clock driver.
 *
 * Copyright 2025 Linaro Ltd.
 */

#include <linux/array_size.h>
#include <linux/clk-provider.h>
#include <linux/container_of.h>
#include <linux/device/devres.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/units.h>

struct acpm_clk {
	u32 id;
	struct clk_hw hw;
	unsigned int mbox_chan_id;
	struct acpm_handle *handle;
};

struct acpm_clk_variant {
	const char *name;
};

/**
 * struct acpm_clk_pair - one operating point of a two-domain clock.
 * @rate:	rate of the clock (the primary domain), in kHz.
 * @rate2:	rate of the secondary domain at that point, in kHz.
 */
struct acpm_clk_pair {
	u32 rate;
	u32 rate2;
};

/**
 * struct acpm_clk_paired - a clock that moves two DVFS domains together.
 * @name:	clock name.
 * @id:		DVFS domain whose rate is the clock's rate.
 * @id2:	DVFS domain set to the paired rate. Its rate is never below
 *		the primary domain's: it is raised first and lowered last.
 * @pairs:	operating points, in ascending order of @rate.
 * @nr_pairs:	number of operating points.
 * @max_khz:	optional cap on the rate, in kHz. If it points to 0, the
 *		clock is read-only.
 */
struct acpm_clk_paired {
	const char *name;
	u32 id;
	u32 id2;
	const struct acpm_clk_pair *pairs;
	unsigned int nr_pairs;
	const unsigned int *max_khz;
};

struct acpm_clk_driver_data {
	const struct acpm_clk_variant *clks;
	unsigned int nr_clks;
	const struct acpm_clk_paired *paired;
	unsigned int nr_paired;
	unsigned long flags;
	unsigned int mbox_chan_id;
};

struct acpm_clk_paired_hw {
	struct clk_hw hw;
	const struct acpm_clk_paired *data;
	unsigned int mbox_chan_id;
	struct acpm_handle *handle;
	struct device *dev;
};

#define to_acpm_clk_paired(clk) container_of(clk, struct acpm_clk_paired_hw, hw)

#define to_acpm_clk(clk) container_of(clk, struct acpm_clk, hw)

#define ACPM_CLK(cname)					\
	{						\
		.name		= cname,		\
	}

static const struct acpm_clk_variant gs101_acpm_clks[] = {
	ACPM_CLK("mif"),
	ACPM_CLK("int"),
	ACPM_CLK("cpucl0"),
	ACPM_CLK("cpucl1"),
	ACPM_CLK("cpucl2"),
	ACPM_CLK("g3d"),
	ACPM_CLK("g3dl2"),
	ACPM_CLK("tpu"),
	ACPM_CLK("intcam"),
	ACPM_CLK("tnr"),
	ACPM_CLK("cam"),
	ACPM_CLK("mfc"),
	ACPM_CLK("disp"),
	ACPM_CLK("bo"),
};

static const struct acpm_clk_driver_data acpm_clk_gs101 = {
	.clks = gs101_acpm_clks,
	.nr_clks = ARRAY_SIZE(gs101_acpm_clks),
	.mbox_chan_id = 0,
};

/*
 * gs201 GPU operating points: shader (G3D) rate and the top/L2 (G3DL2) rate
 * paired with it, in kHz. 202-848 MHz are Google's gs201 GPU DVFS table
 * (gpu_dvfs_table_v1 in gs201-gpu.dtsi; its v2 table needs 633 and 434 MHz
 * levels that the Pixel 7 ECT does not have). 151 MHz is the boot and resume
 * level of both domains in the ECT (MINMAX_G3D, MINMAX_G3DL2).
 */
static const struct acpm_clk_pair gs201_gpu_pairs[] = {
	{ 151000, 151000 },
	{ 202000, 302000 },
	{ 251000, 302000 },
	{ 302000, 470000 },
	{ 351000, 470000 },
	{ 400000, 603000 },
	{ 471000, 603000 },
	{ 510000, 750000 },
	{ 572000, 750000 },
	{ 701000, 885000 },
	{ 762000, 996000 },
	{ 848000, 996000 },
};

/*
 * Staged bring-up: the GPU clock never goes above this rate. 0 (the default)
 * makes the clock read-only: every request rounds to the lowest point and
 * none reaches the firmware. Set it at runtime through
 * /sys/module/clk_acpm/parameters/gs201_gpu_max_khz.
 */
static unsigned int gs201_gpu_max_khz;
module_param(gs201_gpu_max_khz, uint, 0644);
MODULE_PARM_DESC(gs201_gpu_max_khz,
		 "gs201: highest GPU shader rate in kHz, 0 = read-only (default)");

static const struct acpm_clk_paired gs201_acpm_paired[] = {
	{
		.name = "gpu",
		.id = 5,	/* G3D: shader cores */
		.id2 = 6,	/* G3DL2: top level and L2 */
		.pairs = gs201_gpu_pairs,
		.nr_pairs = ARRAY_SIZE(gs201_gpu_pairs),
		.max_khz = &gs201_gpu_max_khz,
	},
};

/* The DVFS domains are the same as gs101's, in the same order. */
static const struct acpm_clk_driver_data acpm_clk_gs201 = {
	.clks = gs101_acpm_clks,
	.nr_clks = ARRAY_SIZE(gs101_acpm_clks),
	.paired = gs201_acpm_paired,
	.nr_paired = ARRAY_SIZE(gs201_acpm_paired),
	/* Show the firmware's rate, not the last one asked for. */
	.flags = CLK_GET_RATE_NOCACHE,
	.mbox_chan_id = 0,
};

/*
 * Tensor G4 (zumapro) DVFS domains, in the ACPM firmware's order: there are
 * 17, with DSU, BCI, AOC and BW among them, so the indices differ from gs101.
 */
static const struct acpm_clk_variant zumapro_acpm_clks[] = {
	ACPM_CLK("mif"),
	ACPM_CLK("int"),
	ACPM_CLK("cpucl0"),
	ACPM_CLK("cpucl1"),
	ACPM_CLK("cpucl2"),
	ACPM_CLK("dsu"),
	ACPM_CLK("bci"),
	ACPM_CLK("g3d"),
	ACPM_CLK("g3dl2"),
	ACPM_CLK("tpu"),
	ACPM_CLK("intcam"),
	ACPM_CLK("tnr"),
	ACPM_CLK("cam"),
	ACPM_CLK("mfc"),
	ACPM_CLK("disp"),
	ACPM_CLK("aoc"),
	ACPM_CLK("bw"),
};

/*
 * zumapro GPU operating points: shader (G3D) rate and the top/L2 (G3DL2)
 * rate paired with it, in kHz, from the Pixel 9a's GPU DVFS table (its
 * gpu_dvfs_table_v2). 940 MHz is the top level (gpu_dvfs_max_freq in
 * Google's zumapro-gpu.dtsi).
 */
static const struct acpm_clk_pair zumapro_gpu_pairs[] = {
	{ 150000, 160000 },
	{ 302000, 386000 },
	{ 337000, 430000 },
	{ 376000, 531000 },
	{ 419000, 531000 },
	{ 467000, 659000 },
	{ 521000, 659000 },
	{ 580000, 749000 },
	{ 649000, 749000 },
	{ 723000, 900000 },
	{ 807000, 900000 },
	{ 850000, 900000 },
	{ 890000, 900000 },
	{ 940000, 950000 },
};

/* As gs201_gpu_max_khz: 0 (the default) leaves the GPU clock read-only. */
static unsigned int zumapro_gpu_max_khz;
module_param(zumapro_gpu_max_khz, uint, 0644);
MODULE_PARM_DESC(zumapro_gpu_max_khz,
		 "zumapro: highest GPU shader rate in kHz, 0 = read-only (default)");

static const struct acpm_clk_paired zumapro_acpm_paired[] = {
	{
		.name = "gpu",
		.id = 7,	/* G3D: shader cores */
		.id2 = 8,	/* G3DL2: top level and L2 */
		.pairs = zumapro_gpu_pairs,
		.nr_pairs = ARRAY_SIZE(zumapro_gpu_pairs),
		.max_khz = &zumapro_gpu_max_khz,
	},
};

static const struct acpm_clk_driver_data acpm_clk_zumapro = {
	.clks = zumapro_acpm_clks,
	.nr_clks = ARRAY_SIZE(zumapro_acpm_clks),
	.paired = zumapro_acpm_paired,
	.nr_paired = ARRAY_SIZE(zumapro_acpm_paired),
	/* Show the firmware's rate, not the last one asked for. */
	.flags = CLK_GET_RATE_NOCACHE,
	.mbox_chan_id = 0,
};

/*
 * Tensor G3 (zuma): the same 17 DVFS domains as zumapro (Google's
 * enum acpm_dvfs_id is zuma's), and the Pixel 8 Pro's GPU DVFS table (its
 * stock gpu_dvfs_table_v2) is the 9a's, pair for pair. Stock stops at
 * 890 MHz (gpu_dvfs_max_freq in Google's zuma-gpu.dtsi), so the 940 MHz
 * level is left out.
 */
static unsigned int zuma_gpu_max_khz;
module_param(zuma_gpu_max_khz, uint, 0644);
MODULE_PARM_DESC(zuma_gpu_max_khz,
		 "zuma: highest GPU shader rate in kHz, 0 = read-only (default)");

static const struct acpm_clk_paired zuma_acpm_paired[] = {
	{
		.name = "gpu",
		.id = 7,	/* G3D: shader cores */
		.id2 = 8,	/* G3DL2: top level and L2 */
		.pairs = zumapro_gpu_pairs,
		.nr_pairs = ARRAY_SIZE(zumapro_gpu_pairs) - 1,
		.max_khz = &zuma_gpu_max_khz,
	},
};

static const struct acpm_clk_driver_data acpm_clk_zuma = {
	.clks = zumapro_acpm_clks,
	.nr_clks = ARRAY_SIZE(zumapro_acpm_clks),
	.paired = zuma_acpm_paired,
	.nr_paired = ARRAY_SIZE(zuma_acpm_paired),
	/* Show the firmware's rate, not the last one asked for. */
	.flags = CLK_GET_RATE_NOCACHE,
	.mbox_chan_id = 0,
};

static unsigned long acpm_clk_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct acpm_clk *clk = to_acpm_clk(hw);

	return clk->handle->ops->dvfs.get_rate(clk->handle, clk->mbox_chan_id,
					       clk->id);
}

static int acpm_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate)
{
	struct acpm_clk *clk = to_acpm_clk(hw);

	return clk->handle->ops->dvfs.set_rate(clk->handle, clk->mbox_chan_id,
					       clk->id, rate);
}

static const struct clk_ops acpm_clk_ops = {
	.recalc_rate = acpm_clk_recalc_rate,
	.determine_rate = clk_determine_rate_noop,
	.set_rate = acpm_clk_set_rate,
};

static int acpm_clk_register(struct device *dev, struct acpm_clk *aclk,
			     const char *name, unsigned long flags)
{
	struct clk_init_data init = {};

	init.name = name;
	init.ops = &acpm_clk_ops;
	init.flags = flags;
	aclk->hw.init = &init;

	return devm_clk_hw_register(dev, &aclk->hw);
}

static unsigned long acpm_clk_paired_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	struct acpm_clk_paired_hw *clk = to_acpm_clk_paired(hw);

	return clk->handle->ops->dvfs.get_rate(clk->handle, clk->mbox_chan_id,
					       clk->data->id);
}

/* The highest operating point at or below @rate and the cap, else the lowest. */
static const struct acpm_clk_pair *
acpm_clk_paired_find(const struct acpm_clk_paired *data, unsigned long rate)
{
	const struct acpm_clk_pair *pair = &data->pairs[0];
	unsigned long khz = rate / HZ_PER_KHZ;
	unsigned int i;

	if (data->max_khz)
		khz = min_t(unsigned long, khz, READ_ONCE(*data->max_khz));

	for (i = 1; i < data->nr_pairs; i++)
		if (data->pairs[i].rate <= khz)
			pair = &data->pairs[i];

	return pair;
}

static int acpm_clk_paired_determine_rate(struct clk_hw *hw,
					  struct clk_rate_request *req)
{
	struct acpm_clk_paired_hw *clk = to_acpm_clk_paired(hw);

	req->rate = acpm_clk_paired_find(clk->data, req->rate)->rate *
		    HZ_PER_KHZ;

	return 0;
}

static int acpm_clk_paired_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	struct acpm_clk_paired_hw *clk = to_acpm_clk_paired(hw);
	const struct acpm_clk_paired *data = clk->data;
	const struct acpm_clk_pair *pair = acpm_clk_paired_find(data, rate);
	struct acpm_handle *handle = clk->handle;
	unsigned int chan = clk->mbox_chan_id;
	unsigned long cur;
	int ret;

	if (pair->rate * HZ_PER_KHZ != rate)
		return -EINVAL;

	/* Read-only: leave the firmware's state alone. */
	if (data->max_khz && !READ_ONCE(*data->max_khz)) {
		dev_dbg(clk->dev, "%s: read-only, %u kHz not sent\n",
			data->name, pair->rate);
		return 0;
	}

	cur = handle->ops->dvfs.get_rate(handle, chan, data->id);

	dev_dbg(clk->dev, "%s: %lu -> %u kHz (%u kHz on domain %u)\n",
		data->name, cur / HZ_PER_KHZ, pair->rate, pair->rate2,
		data->id2);

	/*
	 * Keep the secondary domain at or above the primary one throughout:
	 * raise it first when going up, lower it last when going down. If the
	 * second request fails, the domains are left in an allowed state.
	 */
	if (rate > cur) {
		ret = handle->ops->dvfs.set_rate(handle, chan, data->id2,
						 pair->rate2 * HZ_PER_KHZ);
		if (!ret)
			ret = handle->ops->dvfs.set_rate(handle, chan, data->id,
							 rate);
	} else {
		ret = handle->ops->dvfs.set_rate(handle, chan, data->id, rate);
		if (!ret)
			ret = handle->ops->dvfs.set_rate(handle, chan,
							 data->id2,
							 pair->rate2 * HZ_PER_KHZ);
	}

	if (ret)
		dev_err(clk->dev, "%s: setting %u kHz failed: %d\n",
			data->name, pair->rate, ret);

	return ret;
}

static const struct clk_ops acpm_clk_paired_ops = {
	.recalc_rate = acpm_clk_paired_recalc_rate,
	.determine_rate = acpm_clk_paired_determine_rate,
	.set_rate = acpm_clk_paired_set_rate,
};

static int acpm_clk_paired_register(struct device *dev,
				    struct acpm_clk_paired_hw *clk,
				    unsigned long flags)
{
	struct clk_init_data init = {};

	init.name = clk->data->name;
	init.ops = &acpm_clk_paired_ops;
	init.flags = flags;
	clk->hw.init = &init;

	return devm_clk_hw_register(dev, &clk->hw);
}

static int acpm_clk_probe(struct platform_device *pdev)
{
	const struct acpm_clk_driver_data *drv_data;
	struct acpm_clk_paired_hw *paired = NULL;
	struct acpm_handle *acpm_handle;
	struct clk_hw_onecell_data *clk_data;
	struct clk_hw **hws;
	struct device *dev = &pdev->dev;
	struct acpm_clk *aclks;
	unsigned int mbox_chan_id;
	int i, err, count;

	drv_data = (const struct acpm_clk_driver_data *)
		   platform_get_device_id(pdev)->driver_data;

	acpm_handle = devm_acpm_get_by_node(dev, dev->parent->of_node);
	if (IS_ERR(acpm_handle))
		return dev_err_probe(dev, PTR_ERR(acpm_handle),
				     "Failed to get acpm handle\n");

	count = drv_data->nr_clks + drv_data->nr_paired;
	mbox_chan_id = drv_data->mbox_chan_id;

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, count),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->num = count;
	hws = clk_data->hws;

	aclks = devm_kcalloc(dev, drv_data->nr_clks, sizeof(*aclks),
			     GFP_KERNEL);
	if (!aclks)
		return -ENOMEM;

	for (i = 0; i < drv_data->nr_clks; i++) {
		struct acpm_clk *aclk = &aclks[i];

		/*
		 * The code assumes the clock IDs start from zero,
		 * are sequential and do not have gaps.
		 */
		aclk->id = i;
		aclk->handle = acpm_handle;
		aclk->mbox_chan_id = mbox_chan_id;

		hws[i] = &aclk->hw;

		err = acpm_clk_register(dev, aclk, drv_data->clks[i].name,
					drv_data->flags);
		if (err)
			return dev_err_probe(dev, err,
					     "Failed to register clock\n");
	}

	/* Paired clocks follow the DVFS domains in the clock index space. */
	if (drv_data->nr_paired) {
		paired = devm_kcalloc(dev, drv_data->nr_paired,
				      sizeof(*paired), GFP_KERNEL);
		if (!paired)
			return -ENOMEM;
	}

	for (i = 0; i < drv_data->nr_paired; i++) {
		struct acpm_clk_paired_hw *clk = &paired[i];

		clk->data = &drv_data->paired[i];
		clk->handle = acpm_handle;
		clk->mbox_chan_id = mbox_chan_id;
		clk->dev = dev;

		hws[drv_data->nr_clks + i] = &clk->hw;

		err = acpm_clk_paired_register(dev, clk, drv_data->flags);
		if (err)
			return dev_err_probe(dev, err,
					     "Failed to register clock\n");
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					   clk_data);
}

static const struct platform_device_id acpm_clk_id[] = {
	{
		.name = "gs101-acpm-clk",
		.driver_data = (kernel_ulong_t)&acpm_clk_gs101,
	}, {
		.name = "gs201-acpm-clk",
		.driver_data = (kernel_ulong_t)&acpm_clk_gs201,
	}, {
		.name = "zuma-acpm-clk",
		.driver_data = (kernel_ulong_t)&acpm_clk_zuma,
	}, {
		.name = "zumapro-acpm-clk",
		.driver_data = (kernel_ulong_t)&acpm_clk_zumapro,
	},
	{ }
};
MODULE_DEVICE_TABLE(platform, acpm_clk_id);

static struct platform_driver acpm_clk_driver = {
	.driver	= {
		.name = "acpm-clocks",
	},
	.probe = acpm_clk_probe,
	.id_table = acpm_clk_id,
};
module_platform_driver(acpm_clk_driver);

MODULE_AUTHOR("Tudor Ambarus <tudor.ambarus@linaro.org>");
MODULE_DESCRIPTION("Samsung Exynos ACPM clock driver");
MODULE_LICENSE("GPL");
