// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 * Author: Tomasz Figa <t.figa@samsung.com>
 *
 * Clock driver for Exynos clock output
 */

#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>

#define DRV_NAME			"exynos-clkout"

#define EXYNOS_CLKOUT_NR_CLKS		1
#define EXYNOS_CLKOUT_PARENTS		32

#define EXYNOS_PMU_DEBUG_REG		0xa00
#define EXYNOS_CLKOUT_DISABLE_SHIFT	0
#define EXYNOS_CLKOUT_MUX_SHIFT		8
#define EXYNOS4_CLKOUT_MUX_MASK		0xf
#define EXYNOS5_CLKOUT_MUX_MASK		0x1f

/*
 * GS101 and GS201 have one PAD_CTRL_CLKOUTn register per output pad in the
 * PMU, written through the PMU regmap (the PMU is secured on these SoCs).
 * Bit 0 drives the clock onto the pad, bits 13:8 select the source; source 0
 * is the TCXO.
 */
#define GS101_PAD_CTRL_CLKOUT(n)	(0x3e80 + 4 * (n))
#define GS101_CLKOUT_SEL_MASK		GENMASK(13, 8)
#define GS101_CLKOUT_EN			BIT(0)

/*
 * Tensor G4 (zumapro) feeds the NFC controller's reference clock from a PMU
 * pad as well. With EXTPAD_EN set the PMU follows the controller's clock
 * request line and drives the clock onto the pad, which is how Google's
 * st21nfc driver runs it (pmu_clk_pad = <0x3e84>, st,extpad_control_en);
 * without it the Pixel 9a's ST54L does not ACK its I2C address. Nothing
 * else about this register is known, so its source field is left alone.
 */
#define ZUMAPRO_PAD_CTRL_NFC_CLK	0x3e84
#define ZUMAPRO_CLKOUT_EXTPAD_EN	BIT(16)

#define EXYNOS_CLKOUT_MAX_PADS		2

/* A pad's register, the bit that puts the clock on it and the source field. */
struct exynos_clkout_pad_desc {
	unsigned int reg;
	u32 en;
	u32 sel_mask;
};

struct exynos_clkout_pad {
	struct clk_hw hw;
	struct regmap *regmap;
	const struct exynos_clkout_pad_desc *desc;
	bool en_bringup;
};

#define to_exynos_clkout_pad(_hw) container_of(_hw, struct exynos_clkout_pad, hw)

/*
 * Google's sources disagree on bit 0: the NFC driver sets it to run the
 * clock, the clock framework code clears it. Until a board proves one,
 * let bring-up pick at run time.
 */
static bool gs101_pad_en_active_low;
module_param(gs101_pad_en_active_low, bool, 0644);
MODULE_PARM_DESC(gs101_pad_en_active_low,
		 "GS101/GS201 CLKOUT pads run with bit 0 clear (default: set)");

static void exynos_clkout_pad_trace(struct exynos_clkout_pad *pad,
				    const char *what)
{
	unsigned int val = 0;
	int ret;

	ret = regmap_read(pad->regmap, pad->desc->reg, &val);
	pr_debug("%s: %s, PAD_CTRL 0x%x = 0x%08x (%d)\n",
		 clk_hw_get_name(&pad->hw), what, pad->desc->reg, val, ret);
}

/* Only the GS101 pads' bit 0 is still being settled by bring-up. */
static bool exynos_clkout_pad_active_low(struct exynos_clkout_pad *pad)
{
	return pad->en_bringup && gs101_pad_en_active_low;
}

struct exynos_clkout {
	struct clk_gate gate;
	struct clk_mux mux;
	spinlock_t slock;
	void __iomem *reg;
	struct device_node *np;
	u32 pmu_debug_save;
	struct exynos_clkout_pad pads[EXYNOS_CLKOUT_MAX_PADS];
	struct clk_hw_onecell_data data;
};

struct exynos_clkout_variant {
	u32 mux_mask;
	const struct exynos_clkout_pad_desc *pads;
	unsigned int nr_pads;
	bool pad_en_bringup;
};

static const struct exynos_clkout_variant exynos_clkout_exynos4 = {
	.mux_mask	= EXYNOS4_CLKOUT_MUX_MASK,
};

static const struct exynos_clkout_variant exynos_clkout_exynos5 = {
	.mux_mask	= EXYNOS5_CLKOUT_MUX_MASK,
};

static const struct exynos_clkout_pad_desc exynos_clkout_gs101_pads[] = {
	{
		.reg		= GS101_PAD_CTRL_CLKOUT(0),
		.en		= GS101_CLKOUT_EN,
		.sel_mask	= GS101_CLKOUT_SEL_MASK,
	}, {
		.reg		= GS101_PAD_CTRL_CLKOUT(1),
		.en		= GS101_CLKOUT_EN,
		.sel_mask	= GS101_CLKOUT_SEL_MASK,
	},
};

static_assert(ARRAY_SIZE(exynos_clkout_gs101_pads) <= EXYNOS_CLKOUT_MAX_PADS);

static const struct exynos_clkout_variant exynos_clkout_gs101 = {
	.pads		= exynos_clkout_gs101_pads,
	.nr_pads	= ARRAY_SIZE(exynos_clkout_gs101_pads),
	.pad_en_bringup	= true,
};

static const struct exynos_clkout_pad_desc exynos_clkout_zumapro_pads[] = {
	{
		.reg		= ZUMAPRO_PAD_CTRL_NFC_CLK,
		.en		= ZUMAPRO_CLKOUT_EXTPAD_EN,
	},
};

static_assert(ARRAY_SIZE(exynos_clkout_zumapro_pads) <= EXYNOS_CLKOUT_MAX_PADS);

static const struct exynos_clkout_variant exynos_clkout_zumapro = {
	.pads		= exynos_clkout_zumapro_pads,
	.nr_pads	= ARRAY_SIZE(exynos_clkout_zumapro_pads),
};

static const struct of_device_id exynos_clkout_ids[] = {
	{
		.compatible = "google,gs101-pmu",
		.data = &exynos_clkout_gs101,
	}, {
		.compatible = "google,gs201-pmu",
		.data = &exynos_clkout_gs101,
	}, {
		.compatible = "google,zumapro-pmu",
		.data = &exynos_clkout_zumapro,
	}, {
		.compatible = "samsung,exynos3250-pmu",
		.data = &exynos_clkout_exynos4,
	}, {
		.compatible = "samsung,exynos4210-pmu",
		.data = &exynos_clkout_exynos4,
	}, {
		.compatible = "samsung,exynos4212-pmu",
		.data = &exynos_clkout_exynos4,
	}, {
		.compatible = "samsung,exynos4412-pmu",
		.data = &exynos_clkout_exynos4,
	}, {
		.compatible = "samsung,exynos5250-pmu",
		.data = &exynos_clkout_exynos5,
	}, {
		.compatible = "samsung,exynos5410-pmu",
		.data = &exynos_clkout_exynos5,
	}, {
		.compatible = "samsung,exynos5420-pmu",
		.data = &exynos_clkout_exynos5,
	}, {
		.compatible = "samsung,exynos5433-pmu",
		.data = &exynos_clkout_exynos5,
	}, { }
};

/*
 * Device will be instantiated as child of PMU device without its own
 * device node.  Therefore match compatibles against parent.
 */
static int exynos_clkout_match_parent_dev(struct device *dev,
					  const struct exynos_clkout_variant **variant)
{
	const struct of_device_id *match;

	if (!dev->parent) {
		dev_err(dev, "not instantiated from MFD\n");
		return -EINVAL;
	}

	/*
	 * 'exynos_clkout_ids' arrays is not the ids array matched by
	 * the dev->parent driver, so of_device_get_match_data() or
	 * device_get_match_data() cannot be used here.
	 */
	match = of_match_device(exynos_clkout_ids, dev->parent);
	if (!match) {
		dev_err(dev, "cannot match parent device\n");
		return -EINVAL;
	}
	*variant = match->data;

	return 0;
}

static int exynos_clkout_pad_enable(struct clk_hw *hw)
{
	struct exynos_clkout_pad *pad = to_exynos_clkout_pad(hw);
	const struct exynos_clkout_pad_desc *desc = pad->desc;
	int ret;

	/* Select the TCXO (where the pad has a source field) and drive it. */
	ret = regmap_update_bits(pad->regmap, desc->reg,
				 desc->sel_mask | desc->en,
				 exynos_clkout_pad_active_low(pad) ? 0 : desc->en);
	exynos_clkout_pad_trace(pad, "enable");

	return ret;
}

static void exynos_clkout_pad_disable(struct clk_hw *hw)
{
	struct exynos_clkout_pad *pad = to_exynos_clkout_pad(hw);

	regmap_update_bits(pad->regmap, pad->desc->reg, pad->desc->en,
			   exynos_clkout_pad_active_low(pad) ? pad->desc->en : 0);
	exynos_clkout_pad_trace(pad, "disable");
}

static int exynos_clkout_pad_is_enabled(struct clk_hw *hw)
{
	struct exynos_clkout_pad *pad = to_exynos_clkout_pad(hw);
	unsigned int val;

	if (regmap_read(pad->regmap, pad->desc->reg, &val))
		return 0;

	return !!(val & pad->desc->en) != exynos_clkout_pad_active_low(pad);
}

static const struct clk_ops exynos_clkout_pad_ops = {
	.enable		= exynos_clkout_pad_enable,
	.disable	= exynos_clkout_pad_disable,
	.is_enabled	= exynos_clkout_pad_is_enabled,
};

/*
 * GS101/GS201 and Tensor G4: one gate per pad, parented to mux input 0 (the
 * TCXO, named "clkout0" in the PMU node) where the PMU node names it; a pad
 * whose source the node does not describe is a root clock. The pads may be
 * in use by devices that Linux does not describe, so leave them as the
 * bootloader left them until a consumer asks for them.
 */
static int exynos_clkout_pad_probe(struct platform_device *pdev,
				   struct exynos_clkout *clkout,
				   const struct exynos_clkout_variant *variant)
{
	struct device *dev = &pdev->dev;
	struct clk_init_data init = {};
	const char *parent_name = NULL;
	struct regmap *regmap;
	struct clk *parent;
	int ret, i;

	regmap = syscon_node_to_regmap(clkout->np);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to get PMU regmap\n");

	/*
	 * Only a PMU node that names "clkout0" has a parent: without
	 * clock-names at all of_clk_get_by_name() returns -EINVAL, not
	 * -ENOENT, so look for the name first.
	 */
	parent = NULL;
	if (of_property_match_string(clkout->np, "clock-names", "clkout0") >= 0) {
		parent = of_clk_get_by_name(clkout->np, "clkout0");
		if (IS_ERR(parent))
			return dev_err_probe(dev, PTR_ERR(parent),
					     "failed to get TCXO clock\n");
		parent_name = __clk_get_name(parent);
	}

	clkout->data.num = variant->nr_pads;
	for (i = 0; i < variant->nr_pads; i++) {
		struct exynos_clkout_pad *pad = &clkout->pads[i];
		char name[] = "pmu-clkoutX";

		snprintf(name, sizeof(name), "pmu-clkout%d", i);
		init.name = name;
		init.ops = &exynos_clkout_pad_ops;
		init.parent_names = parent ? &parent_name : NULL;
		init.num_parents = parent ? 1 : 0;
		init.flags = CLK_IGNORE_UNUSED;

		pad->regmap = regmap;
		pad->desc = &variant->pads[i];
		pad->en_bringup = variant->pad_en_bringup;
		pad->hw.init = &init;

		ret = devm_clk_hw_register(dev, &pad->hw);
		if (ret) {
			clk_put(parent);
			return dev_err_probe(dev, ret,
					     "failed to register %s\n", name);
		}
		clkout->data.hws[i] = &pad->hw;
	}
	clk_put(parent);

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					   &clkout->data);
}

static int exynos_clkout_probe(struct platform_device *pdev)
{
	const char *parent_names[EXYNOS_CLKOUT_PARENTS];
	const struct exynos_clkout_variant *variant;
	struct clk *parents[EXYNOS_CLKOUT_PARENTS];
	struct exynos_clkout *clkout;
	int parent_count, ret, i;

	ret = exynos_clkout_match_parent_dev(&pdev->dev, &variant);
	if (ret)
		return ret;

	clkout = devm_kzalloc(&pdev->dev,
			      struct_size(clkout, data.hws,
					  variant->nr_pads ?: EXYNOS_CLKOUT_NR_CLKS),
			      GFP_KERNEL);
	if (!clkout)
		return -ENOMEM;

	clkout->np = pdev->dev.of_node;
	if (!clkout->np) {
		/*
		 * pdev->dev.parent was checked by exynos_clkout_match_parent_dev()
		 * so it is not NULL.
		 */
		clkout->np = pdev->dev.parent->of_node;
	}

	platform_set_drvdata(pdev, clkout);

	if (variant->nr_pads)
		return exynos_clkout_pad_probe(pdev, clkout, variant);

	spin_lock_init(&clkout->slock);

	parent_count = 0;
	for (i = 0; i < EXYNOS_CLKOUT_PARENTS; ++i) {
		char name[] = "clkoutXX";

		snprintf(name, sizeof(name), "clkout%d", i);
		parents[i] = of_clk_get_by_name(clkout->np, name);
		if (IS_ERR(parents[i])) {
			parent_names[i] = "none";
			continue;
		}

		parent_names[i] = __clk_get_name(parents[i]);
		parent_count = i + 1;
	}

	if (!parent_count)
		return -EINVAL;

	clkout->reg = of_iomap(clkout->np, 0);
	if (!clkout->reg) {
		ret = -ENODEV;
		goto clks_put;
	}

	clkout->gate.reg = clkout->reg + EXYNOS_PMU_DEBUG_REG;
	clkout->gate.bit_idx = EXYNOS_CLKOUT_DISABLE_SHIFT;
	clkout->gate.flags = CLK_GATE_SET_TO_DISABLE;
	clkout->gate.lock = &clkout->slock;

	clkout->mux.reg = clkout->reg + EXYNOS_PMU_DEBUG_REG;
	clkout->mux.mask = variant->mux_mask;
	clkout->mux.shift = EXYNOS_CLKOUT_MUX_SHIFT;
	clkout->mux.lock = &clkout->slock;

	clkout->data.num = EXYNOS_CLKOUT_NR_CLKS;
	clkout->data.hws[0] = clk_hw_register_composite(NULL, "clkout",
				parent_names, parent_count, &clkout->mux.hw,
				&clk_mux_ops, NULL, NULL, &clkout->gate.hw,
				&clk_gate_ops, CLK_SET_RATE_PARENT
				| CLK_SET_RATE_NO_REPARENT);
	if (IS_ERR(clkout->data.hws[0])) {
		ret = PTR_ERR(clkout->data.hws[0]);
		goto err_unmap;
	}

	ret = of_clk_add_hw_provider(clkout->np, of_clk_hw_onecell_get, &clkout->data);
	if (ret)
		goto err_clk_unreg;

	return 0;

err_clk_unreg:
	clk_hw_unregister(clkout->data.hws[0]);
err_unmap:
	iounmap(clkout->reg);
clks_put:
	for (i = 0; i < EXYNOS_CLKOUT_PARENTS; ++i)
		if (!IS_ERR(parents[i]))
			clk_put(parents[i]);

	dev_err(&pdev->dev, "failed to register clkout clock\n");

	return ret;
}

static void exynos_clkout_remove(struct platform_device *pdev)
{
	struct exynos_clkout *clkout = platform_get_drvdata(pdev);

	/* The GS101 pads are managed by devres. */
	if (!clkout->reg)
		return;

	of_clk_del_provider(clkout->np);
	clk_hw_unregister(clkout->data.hws[0]);
	iounmap(clkout->reg);
}

static int __maybe_unused exynos_clkout_suspend(struct device *dev)
{
	struct exynos_clkout *clkout = dev_get_drvdata(dev);

	if (!clkout->reg)
		return 0;

	clkout->pmu_debug_save = readl(clkout->reg + EXYNOS_PMU_DEBUG_REG);

	return 0;
}

static int __maybe_unused exynos_clkout_resume(struct device *dev)
{
	struct exynos_clkout *clkout = dev_get_drvdata(dev);

	if (!clkout->reg)
		return 0;

	writel(clkout->pmu_debug_save, clkout->reg + EXYNOS_PMU_DEBUG_REG);

	return 0;
}

static SIMPLE_DEV_PM_OPS(exynos_clkout_pm_ops, exynos_clkout_suspend,
			 exynos_clkout_resume);

static struct platform_driver exynos_clkout_driver = {
	.driver = {
		.name = DRV_NAME,
		.pm = &exynos_clkout_pm_ops,
	},
	.probe = exynos_clkout_probe,
	.remove = exynos_clkout_remove,
};
module_platform_driver(exynos_clkout_driver);

MODULE_AUTHOR("Krzysztof Kozlowski <krzk@kernel.org>");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa@gmail.com>");
MODULE_DESCRIPTION("Samsung Exynos clock output driver");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_LICENSE("GPL");
