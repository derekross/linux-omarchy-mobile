// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe host controller driver for channel 1 of the Google zumapro (Tensor G4)
 *
 * Channel 1 is a one-lane Synopsys DesignWare core in a Samsung wrapper, with
 * its PHY behind an isolation switch in the PMU. On the Pixel 9a it carries
 * the Wi-Fi chip. The sequence follows the root-complex driver Google builds
 * for zumapro, google-modules/soc/gs drivers/pci/controller/dwc/
 * pcie-exynos-rc.c and pcie-exynos-zuma-rc-cal.c ("PCIe CAL ver 0.2"), for
 * channel 1 with a Broadcom Wi-Fi endpoint, reduced to what the DesignWare
 * core does not do itself.
 *
 * Nothing in the wrapper, the PHY or the DBI may be accessed while the PHY is
 * isolated, so every register access happens between opening the isolation
 * in host_init and closing it again on the way down.
 *
 * Not handled yet: L1 substates and ASPM L1 exit (the I/A sequencer is not
 * programmed, so ASPM is kept off by clearing it from the root port's link
 * capabilities), INTx, the System MMU, and system suspend with the link
 * powered down.
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/pci-pwrctrl.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos-pmu.h>

#include "pcie-designware.h"

/* ELBI (sub-control) registers */
#define ZUMAPRO_ELBI_IRQ0			0x000
#define ZUMAPRO_ELBI_IRQ1			0x004
#define ZUMAPRO_ELBI_IRQ2			0x008
#define ZUMAPRO_ELBI_LTSSM_EN			0x054
#define ZUMAPRO_ELBI_XFER_PENDING		0x074
#define ZUMAPRO_ELBI_LTSSM_STATE		0x2c8
#define ZUMAPRO_ELBI_LTSSM_STATE_MASK		GENMASK(5, 0)
#define ZUMAPRO_ELBI_LINKDOWN_RST_SEL		0x3a0
#define ZUMAPRO_ELBI_LINKDOWN_RST_MANUAL	BIT(1)
#define ZUMAPRO_ELBI_SOFT_RESET			0x3a4
#define ZUMAPRO_ELBI_SOFT_PWR_RESET		BIT(1)
#define ZUMAPRO_ELBI_QCH_SEL			0x3a8
#define ZUMAPRO_ELBI_QCH_GATING_MASK		GENMASK(11, 0)
#define ZUMAPRO_ELBI_REQ_EXIT_L1_MODE		0x3bc
#define ZUMAPRO_ELBI_APP_REQ_EXIT_L1_MODE	BIT(0)
#define ZUMAPRO_ELBI_L1_REQ_NAK_MASTER		BIT(4)
#define ZUMAPRO_ELBI_MSTR_PEND_SEL_NAK		0x474
#define ZUMAPRO_ELBI_DBI_L1_EXIT_DISABLE	0x1078

/* Sub-system control; clearing SEL switches to the OSC clock, in SW mode */
#define ZUMAPRO_SOC_CLK_CTRL			0x4000
#define ZUMAPRO_SOC_CLK_CTRL_ON			0x15
#define ZUMAPRO_SOC_CLK_CTRL_SEL		GENMASK(1, 0)

/* Values Google's driver writes outside the CAL tables, for channel 1 */
#define ZUMAPRO_UDBG_C800			0xc800
#define ZUMAPRO_UDBG_C800_VAL			0x421
#define ZUMAPRO_PHY_032C			0x032c

/* DBI port logic */
#define ZUMAPRO_DBI_AUX_CLK_FREQ		0xb40
#define ZUMAPRO_DBI_AUX_CLK_26MHZ		0x1a

/* PMU PHY control */
#define ZUMAPRO_PMU_PHY_ISO_BYPASS		BIT(0)
#define ZUMAPRO_PMU_PHY_CONFIGURED		BIT(10)

/* What Google's driver waits after PERST# goes high, before training */
#define ZUMAPRO_PERST_DELAY_US			15000
#define ZUMAPRO_L0_TIMEOUT_US			100000
#define ZUMAPRO_PHY_POLL_TIMEOUT_US		10000

enum zumapro_pcie_blk {
	ZUMAPRO_BLK_ELBI,
	ZUMAPRO_BLK_SOC,
	ZUMAPRO_BLK_UDBG,
	ZUMAPRO_BLK_PHY,
	ZUMAPRO_BLK_PCS,
	ZUMAPRO_BLK_COUNT,
};

enum zumapro_pcie_opcode {
	ZUMAPRO_OP_WRITE,
	ZUMAPRO_OP_SET,
	ZUMAPRO_OP_CLEAR,
	ZUMAPRO_OP_DELAY_US,
	ZUMAPRO_OP_POLL_BIT,
};

/*
 * One step of a CAL table: write a value, set or clear bits, wait a number of
 * microseconds, or wait for one bit to read 1.
 */
struct zumapro_pcie_op {
	u8 op;
	u8 blk;
	u16 offset;
	u32 value;
};

#define ZP_WR(b, o, v)	{ ZUMAPRO_OP_WRITE, ZUMAPRO_BLK_##b, o, v }
#define ZP_SET(b, o, v)	{ ZUMAPRO_OP_SET, ZUMAPRO_BLK_##b, o, v }
#define ZP_CLR(b, o, v)	{ ZUMAPRO_OP_CLEAR, ZUMAPRO_BLK_##b, o, v }
#define ZP_POLL(b, o, v) { ZUMAPRO_OP_POLL_BIT, ZUMAPRO_BLK_##b, o, v }
#define ZP_DELAY(us)	{ ZUMAPRO_OP_DELAY_US, 0, 0, us }

/* exynos_pcie_rc_phy_all_pwrdn_clear() for channel 1 */
static const struct zumapro_pcie_op zumapro_pcie_phy_pwrdn_clear[] = {
	ZP_SET(UDBG, 0xc700, BIT(0)),
	ZP_DELAY(100),
	ZP_WR(PHY, 0x000c, 0x00),
	ZP_WR(PHY, 0x0928, 0x55),
	ZP_WR(PHY, 0x0580, 0x02),
	ZP_WR(PHY, 0x0208, 0x00),
	ZP_WR(PHY, 0x1044, 0x00),
	ZP_WR(PHY, 0x1048, 0x00),
	ZP_WR(PHY, 0x104c, 0x00),
	ZP_WR(PHY, 0x1050, 0x00),
	ZP_WR(PHY, 0x185c, 0x00),
	ZP_DELAY(10),
};

/* exynos_pcie_rc_pcie_phy_config() for channel 1, Broadcom Wi-Fi endpoint */
static const struct zumapro_pcie_op zumapro_pcie_phy_config[] = {
	ZP_WR(PHY, 0x032c, 0x03),
	ZP_SET(UDBG, 0xc700, BIT(0)),
	ZP_CLR(UDBG, 0xc700, BIT(1)),
	ZP_POLL(UDBG, 0xc734, 2),		/* external PLL lock */
	ZP_CLR(SOC, 0x4004, BIT(2)),
	ZP_SET(SOC, 0x4004, GENMASK(5, 4)),
	ZP_WR(ELBI, 0x0080, 0x4),		/* device type: root complex */
	ZP_WR(ELBI, 0x03a4, 0xf),
	ZP_WR(ELBI, 0x03a4, 0xd),
	ZP_DELAY(10),
	ZP_WR(ELBI, 0x03a4, 0xf),
	ZP_DELAY(10),
	ZP_WR(ELBI, 0x1404, 0x1),
	ZP_WR(ELBI, 0x1408, 0x1),
	ZP_WR(ELBI, 0x1400, 0x1),
	ZP_WR(ELBI, 0x1404, 0x0),
	ZP_WR(ELBI, 0x1408, 0x0),
	ZP_WR(ELBI, 0x1400, 0x0),
	ZP_WR(ELBI, 0x03d8, 0x1),
	ZP_WR(PHY, 0x0000, 0x88),
	ZP_WR(PHY, 0x001c, 0x66),
	ZP_WR(PHY, 0x01f4, 0x00),
	ZP_WR(PHY, 0x0514, 0x59),
	ZP_WR(PHY, 0x051c, 0x11),
	ZP_WR(PHY, 0x062c, 0x0e),
	ZP_WR(PHY, 0x0644, 0x22),
	ZP_WR(PHY, 0x0688, 0x03),
	ZP_WR(PHY, 0x06d4, 0x28),
	ZP_WR(PHY, 0x0788, 0x64),
	ZP_WR(PHY, 0x078c, 0x64),
	ZP_WR(PHY, 0x0790, 0x50),
	ZP_WR(PHY, 0x0794, 0x50),
	ZP_WR(PHY, 0x0944, 0x05),
	ZP_WR(PHY, 0x0948, 0x05),
	ZP_WR(PHY, 0x094c, 0x05),
	ZP_WR(PHY, 0x0950, 0x05),
	ZP_WR(PHY, 0x0590, 0x02),
	ZP_WR(PHY, 0x07f8, 0xb0),
	ZP_WR(PHY, 0x0730, 0x08),
	ZP_WR(PHY, 0x0344, 0xc0),
	ZP_WR(PHY, 0x0040, 0x04),
	ZP_WR(PHY, 0x0204, 0x03),
	ZP_WR(PHY, 0x02d4, 0x1f),
	ZP_WR(PHY, 0x0358, 0x10),
	ZP_WR(PHY, 0x0018, 0x01),
	ZP_WR(PHY, 0x0514, 0x5b),
	ZP_WR(PHY, 0x0608, 0x0c),
	ZP_WR(PHY, 0x060c, 0x0f),
	ZP_WR(PHY, 0x0610, 0x0f),
	ZP_WR(PHY, 0x0614, 0x0f),
	ZP_WR(PHY, 0x0618, 0x0f),
	ZP_WR(PHY, 0x0510, 0x80),
	ZP_WR(PHY, 0x0688, 0x03),
	ZP_WR(PHY, 0x0644, 0x23),
	ZP_WR(PHY, 0x0624, 0x11),
	ZP_WR(PHY, 0x0630, 0x0f),
	ZP_WR(PHY, 0x06d0, 0x53),
	ZP_WR(PHY, 0x1140, 0x04),
	ZP_WR(PHY, 0x1144, 0x04),
	ZP_WR(PHY, 0x1148, 0x04),
	ZP_WR(PHY, 0x114c, 0x02),
	ZP_WR(PHY, 0x1150, 0x00),
	ZP_WR(PHY, 0x1154, 0x00),
	ZP_WR(PHY, 0x1158, 0x00),
	ZP_WR(PHY, 0x115c, 0x00),
	ZP_WR(PHY, 0x12cc, 0x1c),
	ZP_WR(PHY, 0x12dc, 0x6c),
	ZP_WR(PHY, 0x130c, 0x29),
	ZP_WR(PHY, 0x13b4, 0x2f),
	ZP_WR(PHY, 0x1a64, 0x05),
	ZP_WR(PHY, 0x1a68, 0x05),
	ZP_WR(PHY, 0x1a84, 0x05),
	ZP_WR(PHY, 0x1a88, 0x05),
	ZP_WR(PHY, 0x1a98, 0x00),
	ZP_WR(PHY, 0x1a9c, 0x00),
	ZP_WR(PHY, 0x1aa8, 0x07),
	ZP_WR(PHY, 0x1ab8, 0x00),
	ZP_WR(PHY, 0x1abc, 0x00),
	ZP_WR(PHY, 0x1af8, 0x90),
	ZP_WR(PHY, 0x1b34, 0x03),
	ZP_WR(PHY, 0x1bb0, 0x03),
	ZP_WR(PHY, 0x1bb4, 0x03),
	ZP_WR(PHY, 0x1bc0, 0x06),
	ZP_WR(PHY, 0x1bc4, 0x06),
	ZP_WR(PHY, 0x1be8, 0x01),
	ZP_WR(PHY, 0x1bf8, 0x04),
	ZP_WR(PHY, 0x1c98, 0x00),
	ZP_WR(PHY, 0x1ca4, 0x81),
	/* Broadcom Wi-Fi endpoint */
	ZP_WR(PHY, 0x1b20, 0x02),
	ZP_WR(PHY, 0x1340, 0x23),
	ZP_WR(PHY, 0x002c, 0x3c),
	ZP_SET(PHY, 0x1350, BIT(0)),
	ZP_WR(PCS, 0x0190, 0x100b0604),
	ZP_WR(PCS, 0x0154, 0x000700d5),
	ZP_WR(PCS, 0x0100, 0x16400000),
	ZP_WR(PCS, 0x0104, 0x08600000),
	ZP_WR(PCS, 0x0114, 0x18500000),
	ZP_WR(PCS, 0x0124, 0x60700000),
	ZP_WR(PCS, 0x0174, 0x00000007),
	ZP_WR(PCS, 0x0178, 0x00000100),
	ZP_WR(PCS, 0x017c, 0x00000700),	/* Broadcom Wi-Fi endpoint */
	ZP_WR(PCS, 0x0110, 0x01600202),	/* Broadcom Wi-Fi endpoint */
	ZP_WR(ELBI, 0x1404, 0x1),
	ZP_DELAY(10),
	ZP_WR(ELBI, 0x1408, 0x1),
	ZP_WR(ELBI, 0x1400, 0x1),
	ZP_POLL(PHY, 0x0a80, 0),		/* PLL lock */
	ZP_POLL(PHY, 0x15c0, 4),		/* CDR lock */
	ZP_POLL(PHY, 0x140c, 7),		/* offset calibration done */
	ZP_WR(PHY, 0x032c, 0x00),
	ZP_WR(ELBI, 0x1078, 0x1),
};

/* exynos_pcie_rc_phy_all_pwrdn() for channel 1 */
static const struct zumapro_pcie_op zumapro_pcie_phy_pwrdn[] = {
	ZP_CLR(PHY, 0x0204, GENMASK(3, 2)),
	ZP_WR(PHY, 0x1044, 0x2a),
	ZP_WR(PHY, 0x1048, 0xaa),
	ZP_WR(PHY, 0x104c, 0xa8),
	ZP_WR(PHY, 0x1050, 0x80),
	ZP_WR(PHY, 0x185c, 0x0a),
	ZP_DELAY(1),
	ZP_WR(PHY, 0x0208, 0xff),
	ZP_DELAY(1),
	ZP_WR(PHY, 0x0580, 0x0a),
	ZP_WR(PHY, 0x0928, 0xaa),
	ZP_WR(PHY, 0x000c, 0x0a),
	ZP_DELAY(50),
	ZP_CLR(UDBG, 0xc700, BIT(0)),
	ZP_DELAY(10),
};

static const char * const zumapro_pcie_blk_names[ZUMAPRO_BLK_COUNT] = {
	[ZUMAPRO_BLK_ELBI] = "elbi",
	[ZUMAPRO_BLK_SOC] = "soc",
	[ZUMAPRO_BLK_UDBG] = "udbg",
	[ZUMAPRO_BLK_PHY] = "phy",
	[ZUMAPRO_BLK_PCS] = "pcs",
};

struct zumapro_pcie {
	struct dw_pcie pci;
	void __iomem *blk[ZUMAPRO_BLK_COUNT];
	struct clk_bulk_data *clks;
	int num_clks;
	struct regmap *pmu;
	u32 pmu_offset;
	struct gpio_desc *perst;
};

#define to_zumapro_pcie(x) container_of(x, struct zumapro_pcie, pci)

static int zumapro_pcie_run(struct zumapro_pcie *zp,
			    const struct zumapro_pcie_op *ops, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++) {
		const struct zumapro_pcie_op *o = &ops[i];
		void __iomem *reg = zp->blk[o->blk] + o->offset;
		u32 val;
		int ret;

		switch (o->op) {
		case ZUMAPRO_OP_WRITE:
			writel(o->value, reg);
			break;
		case ZUMAPRO_OP_SET:
			writel(readl(reg) | o->value, reg);
			break;
		case ZUMAPRO_OP_CLEAR:
			writel(readl(reg) & ~o->value, reg);
			break;
		case ZUMAPRO_OP_DELAY_US:
			udelay(o->value);
			break;
		case ZUMAPRO_OP_POLL_BIT:
			ret = readl_poll_timeout(reg, val, val & BIT(o->value),
						 10, ZUMAPRO_PHY_POLL_TIMEOUT_US);
			if (ret) {
				dev_err(zp->pci.dev,
					"%s+%#x bit %u did not set (%#010x)\n",
					zumapro_pcie_blk_names[o->blk],
					o->offset, o->value, val);
				return ret;
			}
			break;
		}
	}

	return 0;
}

static u32 zumapro_pcie_ltssm(struct zumapro_pcie *zp)
{
	return readl(zp->pci.elbi_base + ZUMAPRO_ELBI_LTSSM_STATE) &
	       ZUMAPRO_ELBI_LTSSM_STATE_MASK;
}

static enum dw_pcie_ltssm zumapro_pcie_get_ltssm(struct dw_pcie *pci)
{
	return zumapro_pcie_ltssm(to_zumapro_pcie(pci));
}

/*
 * As Google's driver: recovery, L0, L0s and L1 all count as up. An endpoint
 * in D3hot parks the link in L1, and with an L0-only check the DesignWare
 * core would refuse the configuration write that brings it back to D0.
 */
static bool zumapro_pcie_link_up(struct dw_pcie *pci)
{
	u32 ltssm = zumapro_pcie_ltssm(to_zumapro_pcie(pci));

	return ltssm >= DW_PCIE_LTSSM_RCVRY_LOCK && ltssm <= DW_PCIE_LTSSM_L1_IDLE;
}

static int zumapro_pcie_start_link(struct dw_pcie *pci)
{
	struct zumapro_pcie *zp = to_zumapro_pcie(pci);
	void __iomem *elbi = pci->elbi_base;
	u32 ltssm;
	int ret;

	writel(1, elbi + ZUMAPRO_ELBI_LTSSM_EN);

	/*
	 * Google's driver waits for L0, acknowledges the ELBI interrupt status
	 * and only then drops XFER_PENDING, which host_init raised before
	 * training. With it left raised, configuration works but every memory
	 * read of the endpoint returns all ones.
	 */
	ret = readl_poll_timeout(elbi + ZUMAPRO_ELBI_LTSSM_STATE, ltssm,
				 (ltssm & ZUMAPRO_ELBI_LTSSM_STATE_MASK) ==
				 DW_PCIE_LTSSM_L0, 10, ZUMAPRO_L0_TIMEOUT_US);
	if (ret)
		dev_dbg(pci->dev, "LTSSM %#x, not L0 yet\n",
			zumapro_pcie_ltssm(zp));

	writel(readl(elbi + ZUMAPRO_ELBI_IRQ0), elbi + ZUMAPRO_ELBI_IRQ0);
	writel(readl(elbi + ZUMAPRO_ELBI_IRQ1), elbi + ZUMAPRO_ELBI_IRQ1);
	writel(readl(elbi + ZUMAPRO_ELBI_IRQ2), elbi + ZUMAPRO_ELBI_IRQ2);
	writel(0, elbi + ZUMAPRO_ELBI_XFER_PENDING);

	return 0;
}

static void zumapro_pcie_stop_link(struct dw_pcie *pci)
{
	writel(0, pci->elbi_base + ZUMAPRO_ELBI_LTSSM_EN);
}

/* ASPM L1 exit needs the I/A sequencer, which is not programmed */
static void zumapro_pcie_disable_aspm(struct dw_pcie *pci)
{
	u8 offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	u32 val;

	dw_pcie_dbi_ro_wr_en(pci);
	val = dw_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCAP);
	val &= ~PCI_EXP_LNKCAP_ASPMS;
	dw_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCAP, val);
	dw_pcie_dbi_ro_wr_dis(pci);
}

/*
 * Undo host_init in the order of Google's exynos_pcie_rc_poweroff(), the PHY
 * isolation last: nothing here may be touched once it is closed.
 */
static void zumapro_pcie_power_off(struct zumapro_pcie *zp)
{
	struct dw_pcie *pci = &zp->pci;
	void __iomem *elbi = pci->elbi_base;
	void __iomem *soc = zp->blk[ZUMAPRO_BLK_SOC];
	u32 val;

	gpiod_set_value_cansleep(zp->perst, 1);
	writel(readl(soc + ZUMAPRO_SOC_CLK_CTRL) & ~ZUMAPRO_SOC_CLK_CTRL_SEL,
	       soc + ZUMAPRO_SOC_CLK_CTRL);
	writel(0, elbi + ZUMAPRO_ELBI_LTSSM_EN);

	val = readl(elbi + ZUMAPRO_ELBI_SOFT_RESET);
	writel(val & ~ZUMAPRO_ELBI_SOFT_PWR_RESET, elbi + ZUMAPRO_ELBI_SOFT_RESET);
	usleep_range(20, 40);
	writel(val | ZUMAPRO_ELBI_SOFT_PWR_RESET, elbi + ZUMAPRO_ELBI_SOFT_RESET);

	zumapro_pcie_run(zp, zumapro_pcie_phy_pwrdn,
			 ARRAY_SIZE(zumapro_pcie_phy_pwrdn));
	clk_bulk_disable_unprepare(zp->num_clks, zp->clks);
	pci_pwrctrl_power_off_devices(pci->dev);
	regmap_update_bits(zp->pmu, zp->pmu_offset, ZUMAPRO_PMU_PHY_ISO_BYPASS, 0);
}

static int zumapro_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct zumapro_pcie *zp = to_zumapro_pcie(pci);
	struct device *dev = pci->dev;
	void __iomem *elbi = pci->elbi_base;
	u32 val;
	int ret;

	ret = regmap_update_bits(zp->pmu, zp->pmu_offset,
				 ZUMAPRO_PMU_PHY_ISO_BYPASS,
				 ZUMAPRO_PMU_PHY_ISO_BYPASS);
	if (ret)
		return dev_err_probe(dev, ret, "failed to open the PHY isolation\n");

	ret = clk_bulk_prepare_enable(zp->num_clks, zp->clks);
	if (ret) {
		dev_err(dev, "failed to enable the clocks: %d\n", ret);
		goto err_isolate;
	}

	/* The endpoint's power enable, held until PERST# is released below */
	ret = pci_pwrctrl_power_on_devices(dev);
	if (ret) {
		dev_err_probe(dev, ret, "failed to power the endpoint on\n");
		goto err_clocks;
	}

	writel(ZUMAPRO_SOC_CLK_CTRL_ON, zp->blk[ZUMAPRO_BLK_SOC] + ZUMAPRO_SOC_CLK_CTRL);
	zumapro_pcie_run(zp, zumapro_pcie_phy_pwrdn_clear,
			 ARRAY_SIZE(zumapro_pcie_phy_pwrdn_clear));
	ret = zumapro_pcie_run(zp, zumapro_pcie_phy_config,
			       ARRAY_SIZE(zumapro_pcie_phy_config));
	if (ret)
		goto err_power_off;

	ret = regmap_update_bits(zp->pmu, zp->pmu_offset,
				 ZUMAPRO_PMU_PHY_CONFIGURED,
				 ZUMAPRO_PMU_PHY_CONFIGURED);
	if (ret) {
		dev_err(dev, "failed to mark the PHY configured: %d\n", ret);
		goto err_power_off;
	}
	writel(ZUMAPRO_UDBG_C800_VAL, zp->blk[ZUMAPRO_BLK_UDBG] + ZUMAPRO_UDBG_C800);
	writel(0, zp->blk[ZUMAPRO_BLK_PHY] + ZUMAPRO_PHY_032C);
	dw_pcie_writel_dbi(pci, GEN3_RELATED_OFF,
			   GEN3_RELATED_OFF_GEN3_EQ_DISABLE |
			   GEN3_RELATED_OFF_RXEQ_RGRDLESS_RXTS);

	gpiod_set_value_cansleep(zp->perst, 0);
	usleep_range(ZUMAPRO_PERST_DELAY_US, ZUMAPRO_PERST_DELAY_US + 2000);

	val = readl(elbi + ZUMAPRO_ELBI_REQ_EXIT_L1_MODE);
	val |= ZUMAPRO_ELBI_APP_REQ_EXIT_L1_MODE | ZUMAPRO_ELBI_L1_REQ_NAK_MASTER;
	writel(val, elbi + ZUMAPRO_ELBI_REQ_EXIT_L1_MODE);
	writel(ZUMAPRO_ELBI_LINKDOWN_RST_MANUAL, elbi + ZUMAPRO_ELBI_LINKDOWN_RST_SEL);
	writel(1, elbi + ZUMAPRO_ELBI_XFER_PENDING);
	val = readl(elbi + ZUMAPRO_ELBI_QCH_SEL) & ~ZUMAPRO_ELBI_QCH_GATING_MASK;
	writel(val, elbi + ZUMAPRO_ELBI_QCH_SEL);
	writel(1, elbi + ZUMAPRO_ELBI_MSTR_PEND_SEL_NAK);
	writel(1, elbi + ZUMAPRO_ELBI_DBI_L1_EXIT_DISABLE);

	dw_pcie_writel_dbi(pci, ZUMAPRO_DBI_AUX_CLK_FREQ, ZUMAPRO_DBI_AUX_CLK_26MHZ);
	zumapro_pcie_disable_aspm(pci);

	return 0;

err_power_off:
	zumapro_pcie_power_off(zp);
	return ret;

err_clocks:
	clk_bulk_disable_unprepare(zp->num_clks, zp->clks);
err_isolate:
	regmap_update_bits(zp->pmu, zp->pmu_offset, ZUMAPRO_PMU_PHY_ISO_BYPASS, 0);
	return ret;
}

static void zumapro_pcie_host_deinit(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	zumapro_pcie_power_off(to_zumapro_pcie(pci));
}

static const struct dw_pcie_host_ops zumapro_pcie_host_ops = {
	.init = zumapro_pcie_host_init,
	.deinit = zumapro_pcie_host_deinit,
};

static const struct dw_pcie_ops zumapro_pcie_dw_ops = {
	.link_up = zumapro_pcie_link_up,
	.get_ltssm = zumapro_pcie_get_ltssm,
	.start_link = zumapro_pcie_start_link,
	.stop_link = zumapro_pcie_stop_link,
};

static int zumapro_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct of_phandle_args args;
	struct zumapro_pcie *zp;
	int i, ret;

	zp = devm_kzalloc(dev, sizeof(*zp), GFP_KERNEL);
	if (!zp)
		return -ENOMEM;

	zp->pci.dev = dev;
	zp->pci.ops = &zumapro_pcie_dw_ops;
	zp->pci.pp.ops = &zumapro_pcie_host_ops;
	platform_set_drvdata(pdev, zp);

	/* Mapped here rather than by the DesignWare core, which would too */
	for (i = 0; i < ZUMAPRO_BLK_COUNT; i++) {
		const char *name = zumapro_pcie_blk_names[i];

		zp->blk[i] = devm_platform_ioremap_resource_byname(pdev, name);
		if (IS_ERR(zp->blk[i]))
			return PTR_ERR(zp->blk[i]);
	}
	zp->pci.elbi_base = zp->blk[ZUMAPRO_BLK_ELBI];

	zp->num_clks = devm_clk_bulk_get_all(dev, &zp->clks);
	if (zp->num_clks < 0)
		return dev_err_probe(dev, zp->num_clks, "failed to get the clocks\n");

	ret = of_parse_phandle_with_fixed_args(dev->of_node, "samsung,pmu-syscon",
					       1, 0, &args);
	if (ret)
		return dev_err_probe(dev, ret, "no samsung,pmu-syscon\n");
	zp->pmu_offset = args.args[0];
	of_node_put(args.np);

	zp->pmu = exynos_get_pmu_regmap_by_phandle(dev->of_node, "samsung,pmu-syscon");
	if (IS_ERR(zp->pmu))
		return dev_err_probe(dev, PTR_ERR(zp->pmu), "failed to get the PMU\n");

	/* PERST# asserted until the PHY is configured */
	zp->perst = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(zp->perst))
		return dev_err_probe(dev, PTR_ERR(zp->perst), "failed to get PERST#\n");

	ret = pci_pwrctrl_create_devices(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create pwrctrl devices\n");

	ret = dw_pcie_host_init(&zp->pci.pp);
	if (ret) {
		pci_pwrctrl_destroy_devices(dev);
		return dev_err_probe(dev, ret, "failed to initialize the host\n");
	}

	return 0;
}

static void zumapro_pcie_remove(struct platform_device *pdev)
{
	struct zumapro_pcie *zp = platform_get_drvdata(pdev);

	dw_pcie_host_deinit(&zp->pci.pp);
	pci_pwrctrl_destroy_devices(&pdev->dev);
}

static const struct of_device_id zumapro_pcie_of_match[] = {
	{ .compatible = "google,zumapro-pcie" },
	{ }
};
MODULE_DEVICE_TABLE(of, zumapro_pcie_of_match);

static struct platform_driver zumapro_pcie_driver = {
	.driver = {
		.name = "zumapro-pcie",
		.of_match_table = zumapro_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = zumapro_pcie_probe,
	.remove = zumapro_pcie_remove,
};
module_platform_driver(zumapro_pcie_driver);

MODULE_DESCRIPTION("Google zumapro PCIe host controller driver");
MODULE_LICENSE("GPL");
