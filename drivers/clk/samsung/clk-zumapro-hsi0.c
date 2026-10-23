// SPDX-License-Identifier: GPL-2.0-only
/*
 * Tensor G4 HSI0 clocks derived from Google's GPL CMUCAL tables:
 * cal-if/zuma/cmucal-{sfr,node}.c, Copyright 2022 Samsung Electronics.
 *
 * Shared USB, alternate and sysreg gates retain firmware ownership.
 * Only the USI2 leaf gates and divider are put under CCF control.
 * Firmware leaves HSI0 powered; Linux does not power it off in this stage.
 * Power-off, suspend and S2MPU context restoration are not qualified.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos-pmu.h>
#include <linux/string.h>

#include <dt-bindings/clock/google,zumapro.h>

#include "clk.h"
#include "clk-pll.h"

#define HSI0_GATE_MANUAL		BIT(20)
#define HSI0_GATE_ENABLED		BIT(21)
#define HSI0_GATE_AUTO		BIT(28)
#define HSI0_GATE_SAVE_MASK	(HSI0_GATE_MANUAL | HSI0_GATE_ENABLED | \
				 HSI0_GATE_AUTO)
#define HSI0_USI2_DIVIDER		0x181c

#define INPUT_OSC		(-1)
#define INPUT_NOC		(-2)
#define INPUT_PERI		(-3)
#define INPUT_ALT		(-4)
#define INPUT_TCXO		(-5)

#define CLK_HSI0_NR_CLKS		(CLK_GOUT_HSI0_USB_SUSPEND + 1)

enum hsi0_clock_kind {
	HSI0_PLL,
	HSI0_MUX,
	HSI0_GATE,
	HSI0_RO_GATE,
	HSI0_DIVIDER,
};

struct hsi0_clock {
	unsigned int id;
	const char *name;
	enum hsi0_clock_kind kind;
	int parents[2];
	u16 offset;
	u8 shift;
	u8 width;
	u8 hw_flags;
	unsigned long flags;
};

/* Parents precede children; no missing parent can feed clock registration. */
static const struct hsi0_clock hsi0_clocks[] = {
	{
		.id = CLK_FOUT_HSI0_USB,
		.name = "zumapro_fout_hsi0_usb",
		.kind = HSI0_PLL,
		.parents = { INPUT_OSC },
		.offset = 0x10c,
	},
	{
		.id = CLK_MOUT_HSI0_TCXO_USER,
		.name = "zumapro_mout_hsi0_tcxo_user",
		.kind = HSI0_MUX,
		.parents = { INPUT_OSC, INPUT_TCXO },
		.offset = 0x630,
		.shift = 4,
		.width = 1,
	},
	{
		.id = CLK_DOUT_HSI0_USB,
		.name = "zumapro_dout_hsi0_usb",
		.kind = HSI0_DIVIDER,
		.parents = { CLK_FOUT_HSI0_USB },
		.offset = 0x1804,
		.shift = 0,
		.width = 6,
		.hw_flags = CLK_DIVIDER_READ_ONLY,
	},
	{
		.id = CLK_MOUT_HSI0_USB20_REF,
		.name = "zumapro_mout_hsi0_usb20_ref",
		.kind = HSI0_MUX,
		.parents = { CLK_DOUT_HSI0_USB, CLK_MOUT_HSI0_TCXO_USER },
		.offset = 0x1004,
		.shift = 0,
		.width = 1,
	},
	{
		.id = CLK_MOUT_HSI0_NOC_USER,
		.name = "zumapro_mout_hsi0_noc_user",
		.kind = HSI0_MUX,
		.parents = { INPUT_OSC, INPUT_NOC },
		.offset = 0x620,
		.shift = 4,
		.width = 1,
	},
	{
		.id = CLK_GOUT_HSI0_ALT,
		.name = "zumapro_gout_hsi0_alt",
		.kind = HSI0_RO_GATE,
		.parents = { INPUT_ALT },
		.offset = 0x204c,
		.shift = 21,
		.width = 1,
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = CLK_MOUT_HSI0_ALT_USER,
		.name = "zumapro_mout_hsi0_alt_user",
		.kind = HSI0_MUX,
		.parents = { INPUT_OSC, CLK_GOUT_HSI0_ALT },
		.offset = 0x600,
		.shift = 4,
		.width = 1,
	},
	{
		.id = CLK_MOUT_HSI0_NOC_LOCAL,
		.name = "zumapro_mout_hsi0_noc_local",
		.kind = HSI0_MUX,
		.parents = { CLK_MOUT_HSI0_NOC_USER, CLK_MOUT_HSI0_ALT_USER },
		.offset = 0x1000,
		.shift = 0,
		.width = 1,
	},
	{
		.id = CLK_DOUT_HSI0_EUSB,
		.name = "zumapro_dout_hsi0_eusb",
		.kind = HSI0_DIVIDER,
		.parents = { CLK_MOUT_HSI0_NOC_LOCAL },
		.offset = 0x180c,
		.shift = 0,
		.width = 2,
		.hw_flags = CLK_DIVIDER_READ_ONLY,
	},
	{
		.id = CLK_GOUT_HSI0_EUSB_REF,
		.name = "zumapro_gout_hsi0_eusb_ref",
		.kind = HSI0_RO_GATE,
		.parents = { CLK_MOUT_HSI0_USB20_REF },
		.offset = 0x20dc,
		.shift = 21,
		.width = 1,
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = CLK_GOUT_HSI0_EUSB_BUS,
		.name = "zumapro_gout_hsi0_eusb_bus",
		.kind = HSI0_RO_GATE,
		.parents = { CLK_DOUT_HSI0_EUSB },
		.offset = 0x20d8,
		.shift = 21,
		.width = 1,
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = CLK_GOUT_HSI0_EUSB_CTRL,
		.name = "zumapro_gout_hsi0_eusb_ctrl",
		.kind = HSI0_RO_GATE,
		.parents = { CLK_MOUT_HSI0_NOC_LOCAL },
		.offset = 0x20b8,
		.shift = 21,
		.width = 1,
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = CLK_MOUT_HSI0_PERI_USER,
		.name = "zumapro_mout_hsi0_peri_user",
		.kind = HSI0_MUX,
		.parents = { INPUT_OSC, INPUT_PERI },
		.offset = 0x680,
		.shift = 4,
		.width = 1,
	},
	{
		.id = CLK_MOUT_HSI0_USI2_LOCAL,
		.name = "zumapro_mout_hsi0_usi2_local",
		.kind = HSI0_MUX,
		.parents = { CLK_MOUT_HSI0_PERI_USER, INPUT_OSC },
		.offset = 0x101c,
		.shift = 0,
		.width = 1,
	},
	{
		.id = CLK_GOUT_HSI0_USI2_SOURCE,
		.name = "zumapro_gout_hsi0_usi2_source",
		.kind = HSI0_GATE,
		.parents = { CLK_MOUT_HSI0_USI2_LOCAL },
		.offset = 0x2120,
		.shift = 21,
		.width = 1,
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = CLK_DOUT_HSI0_USI2,
		.name = "zumapro_dout_hsi0_usi2",
		.kind = HSI0_DIVIDER,
		.parents = { CLK_GOUT_HSI0_USI2_SOURCE },
		.offset = 0x181c,
		.shift = 0,
		.width = 4,
	},
	{
		.id = CLK_GOUT_HSI0_USI2_IP,
		.name = "zumapro_gout_hsi0_usi2_ip",
		.kind = HSI0_GATE,
		.parents = { CLK_DOUT_HSI0_USI2 },
		.offset = 0x20fc,
		.shift = 21,
		.width = 1,
		.flags = CLK_IGNORE_UNUSED | CLK_SET_RATE_PARENT,
	},
	{
		.id = CLK_GOUT_HSI0_USI2_PCLK,
		.name = "zumapro_gout_hsi0_usi2_pclk",
		.kind = HSI0_GATE,
		.parents = { CLK_MOUT_HSI0_NOC_LOCAL },
		.offset = 0x2100,
		.shift = 21,
		.width = 1,
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = CLK_GOUT_HSI0_SYSREG,
		.name = "zumapro_gout_hsi0_sysreg",
		.kind = HSI0_RO_GATE,
		.parents = { CLK_MOUT_HSI0_NOC_LOCAL },
		.offset = 0x208c,
		.shift = 21,
		.width = 1,
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = CLK_GOUT_HSI0_USB_SUBCTL,
		.name = "zumapro_gout_hsi0_usb_subctl",
		.kind = HSI0_RO_GATE,
		.parents = { CLK_MOUT_HSI0_NOC_LOCAL },
		.offset = 0x20a0,
		.shift = 21,
		.width = 1,
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = CLK_GOUT_HSI0_USB_LINK,
		.name = "zumapro_gout_hsi0_usb_link",
		.kind = HSI0_RO_GATE,
		.parents = { CLK_MOUT_HSI0_NOC_LOCAL },
		.offset = 0x20a4,
		.shift = 21,
		.width = 1,
		.flags = CLK_IGNORE_UNUSED,
	},
	{
		.id = CLK_GOUT_HSI0_USB_SUSPEND,
		.name = "zumapro_gout_hsi0_usb_suspend",
		.kind = HSI0_RO_GATE,
		.parents = { CLK_MOUT_HSI0_USB20_REF },
		.offset = 0x20e4,
		.shift = 21,
		.width = 1,
		.flags = CLK_IGNORE_UNUSED,
	},
};

static const u16 hsi0_owned_gates[] = { 0x2120, 0x20fc, 0x2100 };

struct zumapro_hsi0 {
	struct device *dev;
	void __iomem *base;
	struct clk_hw_onecell_data *data;
	struct clk_hw *inputs[5];
	struct samsung_clk_provider *provider;
	/* Serializes owned USI2 gate/divider updates and restoration. */
	spinlock_t lock;
	u32 saved_gates[ARRAY_SIZE(hsi0_owned_gates)];
	u32 saved_divider;
	bool controls_applied;
};

static int hsi0_check_power(void)
{
	struct platform_device *pdev;
	struct device_node *np;
	struct resource res;
	struct regmap *pmu;
	u32 status;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "google,zumapro-pmu");
	if (!np)
		return -ENODEV;
	ret = of_address_to_resource(np, 0, &res);
	if (ret || res.start != 0x15460000 || resource_size(&res) < 0x2a88) {
		of_node_put(np);
		return ret ? ret : -EINVAL;
	}
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -EPROBE_DEFER;
	device_lock(&pdev->dev);
	if (!device_is_bound(&pdev->dev) ||
	    strcmp(pdev->dev.driver->name, "exynos-pmu")) {
		ret = -EPROBE_DEFER;
		goto out;
	}
	/* Only use the secure map already registered by the bound PMU driver. */
	pmu = exynos_get_pmu_regmap();
	if (IS_ERR_OR_NULL(pmu)) {
		ret = pmu ? PTR_ERR(pmu) : -EPROBE_DEFER;
		goto out;
	}
	ret = regmap_read(pmu, 0x2a84, &status);
	if (!ret && !(status & BIT(0)))
		ret = -EOPNOTSUPP;
out:
	device_unlock(&pdev->dev);
	put_device(&pdev->dev);
	return ret;
}

static int hsi0_check_shared_gates(struct zumapro_hsi0 *ctx)
{
	struct of_phandle_args args;
	struct resource res;
	void __iomem *top;
	int ret;

	/* Inspect the existing TOP provider read-only; never claim its gates. */
	ret = of_parse_phandle_with_args(ctx->dev->of_node, "clocks",
					 "#clock-cells", 2, &args);
	if (ret)
		return ret;
	if (!of_device_is_compatible(args.np, "google,zumapro-cmu-top") ||
	    !of_device_is_available(args.np) || args.args_count != 1 ||
	    args.args[0] != CLK_DOUT_HSI0_PERI) {
		of_node_put(args.np);
		return -EINVAL;
	}
	ret = of_address_to_resource(args.np, 0, &res);
	of_node_put(args.np);
	if (ret)
		return ret;
	if (res.start != 0x26040000 || resource_size(&res) < 0x20c4)
		return -EINVAL;
	top = devm_ioremap(ctx->dev, res.start, resource_size(&res));
	if (!top)
		return -ENOMEM;
	if (!(readl(top + 0x20bc) & HSI0_GATE_ENABLED) ||
	    !(readl(top + 0x20c0) & HSI0_GATE_ENABLED) ||
	    !(readl(ctx->base + 0x204c) & HSI0_GATE_ENABLED) ||
	    !(readl(ctx->base + 0x208c) & HSI0_GATE_ENABLED))
		return -EOPNOTSUPP;
	return 0;
}

static void hsi0_restore_controls(void *data)
{
	struct zumapro_hsi0 *ctx = data;
	unsigned long flags;
	u32 val;
	int i;

	if (!ctx->controls_applied)
		return;
	spin_lock_irqsave(&ctx->lock, flags);
	val = readl(ctx->base + HSI0_USI2_DIVIDER);
	val = (val & ~0xf) | ctx->saved_divider;
	writel(val, ctx->base + HSI0_USI2_DIVIDER);
	for (i = ARRAY_SIZE(hsi0_owned_gates) - 1; i >= 0; i--) {
		val = readl(ctx->base + hsi0_owned_gates[i]);
		val = (val & ~HSI0_GATE_SAVE_MASK) | ctx->saved_gates[i];
		writel(val, ctx->base + hsi0_owned_gates[i]);
	}
	spin_unlock_irqrestore(&ctx->lock, flags);
}

static struct clk_hw *hsi0_parent(struct zumapro_hsi0 *ctx, int parent)
{
	if (parent < 0 && parent >= INPUT_TCXO)
		return ctx->inputs[-parent - 1];
	if (parent > 0 && parent < CLK_HSI0_NR_CLKS)
		return ctx->data->hws[parent];
	return NULL;
}

static int hsi0_ro_gate_is_enabled(struct clk_hw *hw)
{
	return clk_gate_ops.is_enabled(hw);
}

static int hsi0_ro_gate_prepare(struct clk_hw *hw)
{
	return hsi0_ro_gate_is_enabled(hw) ? 0 : -EOPNOTSUPP;
}

static const struct clk_ops hsi0_ro_gate_ops = {
	.prepare = hsi0_ro_gate_prepare,
	.is_enabled = hsi0_ro_gate_is_enabled,
};

static struct clk_hw *hsi0_register_ro_gate(struct zumapro_hsi0 *ctx,
					    const struct hsi0_clock *c,
					    const struct clk_hw *parent)
{
	struct clk_init_data init = { };
	struct clk_gate *gate;
	int ret;

	gate = devm_kzalloc(ctx->dev, sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);
	init.name = c->name;
	init.ops = &hsi0_ro_gate_ops;
	init.parent_hws = &parent;
	init.num_parents = 1;
	init.flags = c->flags;
	gate->hw.init = &init;
	gate->reg = ctx->base + c->offset;
	gate->bit_idx = c->shift;
	gate->lock = &ctx->lock;
	ret = devm_clk_hw_register(ctx->dev, &gate->hw);
	return ret ? ERR_PTR(ret) : &gate->hw;
}

static int hsi0_register_clock(struct zumapro_hsi0 *ctx,
			       const struct hsi0_clock *c)
{
	const struct clk_hw *parents[2];
	struct samsung_pll_clock pll = { };
	struct clk_hw *hw;
	void __iomem *reg = ctx->base + c->offset;
	int i;

	for (i = 0; i < (c->kind == HSI0_MUX ? 2 : 1); i++) {
		parents[i] = hsi0_parent(ctx, c->parents[i]);
		if (IS_ERR_OR_NULL(parents[i]))
			return -EINVAL;
	}
	switch (c->kind) {
	case HSI0_PLL:
		pll.id = c->id;
		pll.name = c->name;
		pll.parent_name = clk_hw_get_name(parents[0]);
		pll.type = pll_0822x;
		pll.con_offset = c->offset;
		pll.flags = CLK_GET_RATE_NOCACHE;
		/* No rate table: the common M/P/S operations remain read-only. */
		hw = devm_samsung_clk_register_pll(ctx->provider, &pll);
		break;
	case HSI0_RO_GATE:
		hw = hsi0_register_ro_gate(ctx, c, parents[0]);
		break;
	case HSI0_MUX:
		hw = devm_clk_hw_register_mux_parent_hws(ctx->dev, c->name,
							 parents, 2, 0, reg, c->shift,
							 c->width, CLK_MUX_READ_ONLY, &ctx->lock);
		break;
	case HSI0_GATE:
		hw = devm_clk_hw_register_gate_parent_hw(ctx->dev, c->name,
							 parents[0], c->flags, reg, c->shift,
							 0, &ctx->lock);
		break;
	case HSI0_DIVIDER:
		hw = devm_clk_hw_register_divider_parent_hw(ctx->dev, c->name,
							    parents[0], 0, reg, c->shift,
							    c->width, c->hw_flags, &ctx->lock);
		break;
	default:
		return -EINVAL;
	}
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	ctx->data->hws[c->id] = hw;
	return 0;
}

static int zumapro_hsi0_probe(struct platform_device *pdev)
{
	static const char * const inputs[] = { "oscclk", "noc", "peri", "alt", "tcxo" };
	struct device *dev = &pdev->dev;
	struct zumapro_hsi0 *ctx;
	struct resource *res;
	struct clk *clk;
	u32 val;
	int ret, i;

	ret = hsi0_check_power();
	if (ret)
		return dev_err_probe(dev, ret, "HSI0 power is unavailable\n");
	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dev = dev;
	spin_lock_init(&ctx->lock);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || res->start != 0x11000000 || resource_size(res) != 0x8000)
		return -EINVAL;
	ctx->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctx->base))
		return PTR_ERR(ctx->base);
	ctx->provider = devm_kzalloc(dev,
				     struct_size(ctx->provider, clk_data.hws, CLK_HSI0_NR_CLKS),
				     GFP_KERNEL);
	if (!ctx->provider)
		return -ENOMEM;
	ctx->provider->dev = dev;
	ctx->provider->reg_base = ctx->base;
	spin_lock_init(&ctx->provider->lock);
	ctx->data = &ctx->provider->clk_data;
	ctx->data->num = CLK_HSI0_NR_CLKS;
	for (i = 0; i < CLK_HSI0_NR_CLKS; i++)
		ctx->data->hws[i] = ERR_PTR(-ENOENT);
	for (i = 0; i < ARRAY_SIZE(inputs); i++) {
		clk = devm_clk_get_enabled(dev, inputs[i]);
		if (IS_ERR(clk))
			return dev_err_probe(dev, PTR_ERR(clk), "missing clock input\n");
		ctx->inputs[i] = __clk_get_hw(clk);
		if (!ctx->inputs[i])
			return -EINVAL;
	}
	ret = hsi0_check_shared_gates(ctx);
	if (ret)
		return dev_err_probe(dev, ret, "shared firmware gate is unavailable\n");
	/* Reject inactive or invalid PLL state before CCF evaluates its rate. */
	val = readl(ctx->base + 0x10c);
	if (!(val & BIT(31)) || !(val & BIT(29)) || !(val & GENMASK(13, 8)))
		return -EOPNOTSUPP;
	ctx->saved_divider = readl(ctx->base + HSI0_USI2_DIVIDER) & 0xf;
	for (i = 0; i < ARRAY_SIZE(hsi0_owned_gates); i++)
		ctx->saved_gates[i] = readl(ctx->base + hsi0_owned_gates[i]) &
				      HSI0_GATE_SAVE_MASK;
	/* Remove the provider/clocks before restoring controls, input references last. */
	ret = devm_add_action_or_reset(dev, hsi0_restore_controls, ctx);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(hsi0_clocks); i++) {
		ret = hsi0_register_clock(ctx, &hsi0_clocks[i]);
		if (ret)
			return dev_err_probe(dev, ret, "clock registration failed\n");
	}
	ctx->controls_applied = true;
	for (i = 0; i < ARRAY_SIZE(hsi0_owned_gates); i++) {
		val = readl(ctx->base + hsi0_owned_gates[i]);
		val = (val & ~HSI0_GATE_AUTO) | HSI0_GATE_MANUAL;
		writel(val, ctx->base + hsi0_owned_gates[i]);
		val = readl(ctx->base + hsi0_owned_gates[i]);
		if ((val & (HSI0_GATE_AUTO | HSI0_GATE_MANUAL)) !=
		    HSI0_GATE_MANUAL)
			return -EIO;
	}
	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, ctx->data);
	if (ret)
		return ret;
	dev_info(dev, "USB and USI2 clocks ready; firmware-owned gates preserved\n");
	return 0;
}

static const struct of_device_id zumapro_hsi0_match[] = {
	{ .compatible = "google,zumapro-cmu-hsi0" },
	{ }
};
MODULE_DEVICE_TABLE(of, zumapro_hsi0_match);

static struct platform_driver zumapro_hsi0_driver = {
	.probe = zumapro_hsi0_probe,
	.driver = {
		.name = "zumapro-hsi0-clk",
		.of_match_table = zumapro_hsi0_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(zumapro_hsi0_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Tensor G4 HSI0 USB and USI2 clock controller");
