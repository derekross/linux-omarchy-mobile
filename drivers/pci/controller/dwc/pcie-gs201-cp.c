// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google gs201 (Tensor G2) PCIe GEN4A_0 for the Samsung S5300 modem:
 * bring-up driver.
 *
 * The modem's link differs from the Wi-Fi link in pcie-gs201.c:
 *  - nothing is on the bus until the modem is powered through its control
 *    lines, and its ROM only trains while it waits for its boot image, so
 *    the link is brought up on request, never at probe;
 *  - the outbound iATU belongs to the secure monitor ("use-secure-atu" in
 *    Google's device tree): Linux asks for it with an SMC and then reaches
 *    the endpoint's configuration space through the fixed config window;
 *  - two lanes, Gen1 for the ROM, Gen3 later;
 *  - MSI to a fixed target in the modem's shared memory (the "bus" step).
 *
 * Sequences from Google's GPL driver for gs201
 * (google-modules/soc/gs/drivers/pci/controller/dwc-whi/pcie-exynos-rc.c:
 * exynos_pcie_rc_establish_link(), exynos_pcie_setup_rc(),
 * exynos_pcie_rc_set_iocc()) and the modem power lines from Google's CPIF
 * (google-modules/radio/samsung/s5300/modem_ctrl_s5100.c:
 * gpio_power_offon_cp(), gpio_power_off_cp()).
 *
 * This driver only does bring-up steps, one per write to debugfs
 * (gs201-pcie-cp/ctl), and only when loaded with allow_hw=1:
 *
 *   cp_on      modem power-on sequence on its control lines
 *   cp_off     modem power-off sequence
 *              (both only while the "modem" child node is available; when
 *              it is disabled, the modem driver owns the lines)
 *   link_up    PHY out of isolation, PHY and controller setup, PERST#
 *              release, LTSSM, wait for L0 (Gen1 unless "link_up 3")
 *   atu        the secure-ATU SMC (config and memory windows)
 *   id         read the endpoint's vendor/device ID through the config window
 *   bus        register the PCI host bridge and scan it (first time), with
 *              the DWC MSI controller aimed at the modem's MSI page; after
 *              a later link_up and atu, it re-arms MSI on the same bus
 *   link_down  PERST#, LTSSM off, controller soft power reset, PHY
 *              power-down, then the PHY isolated again in the PMU
 *
 * The bus is never removed while the module is loaded: as with Google's
 * driver, the endpoint's pci_dev outlives link_down/link_up cycles, and its
 * configuration space reads as absent while the link is down.
 *
 * The "status" file only reads GPIO levels and the PMU isolation bit, plus
 * controller registers once the PHY is out of isolation (an ELBI access
 * while it is isolated hangs the SoC, as found on the Wi-Fi link).
 * Copyright (C) 2021 Samsung Electronics Co., Ltd. (sequences)
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/phy/phy.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/soc/samsung/exynos-pmu.h>
#include <linux/uaccess.h>

#include "pcie-designware.h"

static bool allow_hw;
module_param(allow_hw, bool, 0444);
MODULE_PARM_DESC(allow_hw, "Allow the debugfs bring-up steps to touch the hardware (default: off)");

static bool watch_lines;
module_param(watch_lines, bool, 0444);
MODULE_PARM_DESC(watch_lines, "Log every edge on the modem's cp2ap lines, with its time (default: off)");

/* ELBI (sub-controller) registers, as in pcie-gs201.c */
#define CP_ELBI_IRQ0			0x000
#define CP_ELBI_IRQ1			0x004
#define CP_ELBI_IRQ2			0x008
#define CP_ELBI_IRQ0_EN			0x010
#define CP_ELBI_IRQ1_EN			0x014
#define CP_ELBI_IRQ2_EN			0x018
#define CP_ELBI_LTSSM_EN		0x054
#define CP_ELBI_DEVICE_TYPE		0x080
#define CP_ELBI_DEVICE_TYPE_RC		0x4
#define CP_ELBI_LTSSM_STATE		0x2c8
#define CP_ELBI_LTSSM_MASK		GENMASK(5, 0)
#define CP_ELBI_LINKDOWN_RST_SEL	0x3a0
#define CP_ELBI_LINKDOWN_RST_MANUAL	BIT(1)
#define CP_ELBI_SOFT_RESET		0x3a4
#define CP_ELBI_SOFT_PWR_RESET		BIT(1)
#define CP_ELBI_SOFT_NON_STICKY_RESET	BIT(3)
#define CP_ELBI_QCH_SEL			0x3a8
#define CP_ELBI_QCH_GATING_MASK		GENMASK(11, 0)
#define CP_ELBI_REQ_EXIT_L1_MODE	0x3bc
#define CP_ELBI_APP_REQ_EXIT_L1		BIT(0)
#define CP_ELBI_L1_REQ_NAK_MASTER	BIT(4)
#define CP_ELBI_MSTR_PEND_SEL_NAK	0x474
#define CP_ELBI_DBI_L1_EXIT_DISABLE	0x1078

/* PMU PHY control (PHY_CTRL_PCIE_GEN4_0): bit 0 set = isolation bypassed */
#define CP_PMU_PHY_BYPASS		BIT(0)

/* DBI */
#define CP_DBI_AUX_CLK_FREQ		0xb40
#define CP_DBI_AUX_CLK_26MHZ		26
#define CP_DBI_COHERENCY_CONTROL_3	0x8e8

/* SYSREG_HSI1 shareability control (Google: PCIE_SYSREG_HSI1_SHARABILITY_CTRL) */
#define CP_SYSREG_SHARABLE_MASK		GENMASK(1, 0)
#define CP_SYSREG_SHARABLE_ENABLE	0x3

/* Secure ATU: the outbound iATU is programmed by EL3 (Google's exynos_smc()) */
#define CP_SMC_SECURE_ATU_SETUP		0x820020d8

#define CP_PERST_DELAY_US		18000	/* modem: 18-20 ms, Wi-Fi 20 */
#define CP_LINK_POLL_US			10
#define CP_LINK_TIMEOUT_US		48000
#define CP_LINK_TRIES			3
#define CP_SPEED_SETTLE_US		3000	/* Gen1 -> Gen3 change */

enum cp_line {
	CP_PDA_ACTIVE,	/* ap2cp_pda_active, gph1-1 */
	CP_PWR_ON,	/* ap2cp_cp_pwr_on, gph1-2 */
	CP_NRESET,	/* ap2cp_nreset_n, gph1-3 */
	CP_WRST_N,	/* ap2cp_cp_wrst_n, gph1-6 */
	CP_PM_WRST_N,	/* ap2cp_pm_wrst_n, gpa0-3 */
	CP_WAKEUP,	/* ap2cp_wake_up, gph0-2 */
	CP_DUMP_NOTI,	/* ap2cp_dump_noti, gph0-3 */
	CP_NUM_OUTPUTS,
};

enum cp_input {
	CP_IN_WRST_N,		/* cp2ap_cp_wrst_n, gpa0-2 */
	CP_IN_PS_HOLD,		/* cp2ap_cp_ps_hold, gpa8-2 */
	CP_IN_WAKEUP,		/* cp2ap_wake_up, gpa8-3 */
	CP_IN_PHONE_ACTIVE,	/* cp2ap_phone_active, gpa8-4 */
	CP_NUM_INPUTS,
};

static const char * const cp_output_names[CP_NUM_OUTPUTS] = {
	"ap2cp-pda-active", "ap2cp-cp-pwr-on", "ap2cp-nreset", "ap2cp-cp-wrst",
	"ap2cp-pm-wrst", "ap2cp-wakeup", "ap2cp-dump-noti",
};

static const char * const cp_input_names[CP_NUM_INPUTS] = {
	"cp2ap-cp-wrst", "cp2ap-ps-hold", "cp2ap-wakeup", "cp2ap-phone-active",
};

struct gs201_pcie_cp {
	struct device *dev;
	struct dw_pcie pci;		/* only for the DBI helpers */
	void __iomem *elbi;
	void __iomem *cfg;
	struct regmap *sysreg;
	u32 sysreg_offset;
	struct regmap *pmu;
	u32 pmu_offset;
	struct phy *phy;
	struct gpio_desc *perst;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	bool has_lines;			/* "modem" child available: we own the lines */
	struct gpio_desc *out[CP_NUM_OUTPUTS];
	struct gpio_desc *in[CP_NUM_INPUTS];
	struct gs201_pcie_cp_watch {
		struct gs201_pcie_cp *cp;
		unsigned int line;
	} watch[CP_NUM_INPUTS];
	struct pci_host_bridge *bridge;
	struct resource msi_page;	/* memory-region: the modem's MSI page */
	struct dentry *debugfs;
	struct mutex lock;		/* serializes the steps */
	raw_spinlock_t cfg_lock;	/* config accesses against link changes */
	bool cfg_root_ok;		/* root port (DBI) accessible */
	bool cfg_ep_ok;			/* endpoint reachable (link up, ATU set) */
	u32 num_lanes;
	bool cp_powered;
	bool phy_inited;
	bool phy_on;
	bool link_up;
	bool atu_done;
	bool msi_ready;		/* MSI domain and chained handlers set up */
	bool msi_irqs_on;	/* their parent interrupts enabled */
};

static u32 cp_elbi_read(struct gs201_pcie_cp *cp, u32 reg)
{
	return readl(cp->elbi + reg);
}

static void cp_elbi_write(struct gs201_pcie_cp *cp, u32 reg, u32 val)
{
	writel(val, cp->elbi + reg);
}

static void cp_elbi_rmw(struct gs201_pcie_cp *cp, u32 reg, u32 clr, u32 set)
{
	cp_elbi_write(cp, reg, (cp_elbi_read(cp, reg) & ~clr) | set);
}

/* Google's mif_gpio_set_value(): raw level, then a delay in ms */
static void cp_line(struct gs201_pcie_cp *cp, enum cp_line l, int val,
		    unsigned int delay_ms)
{
	gpiod_direction_output_raw(cp->out[l], val);
	if (delay_ms)
		msleep(delay_ms);
}

/* gpio_power_off_cp(), CONFIG_CP_WRESET_WA unset as in pantah_defconfig */
static void cp_power_off(struct gs201_pcie_cp *cp)
{
	dev_info(cp->dev, "modem power-off sequence\n");
	cp_line(cp, CP_WAKEUP, 1, 10);
	cp_line(cp, CP_WAKEUP, 0, 0);
	cp_line(cp, CP_NRESET, 0, 0);
	cp_line(cp, CP_WRST_N, 0, 0);
	cp_line(cp, CP_PWR_ON, 0, 30);
	cp_line(cp, CP_PM_WRST_N, 0, 50);
	cp_line(cp, CP_PDA_ACTIVE, 0, 0);
	cp->cp_powered = false;
}

/* power_on_cp(): AP active, no dump request, then gpio_power_offon_cp() */
static void cp_power_on(struct gs201_pcie_cp *cp)
{
	cp_line(cp, CP_PDA_ACTIVE, 1, 0);
	cp_line(cp, CP_DUMP_NOTI, 0, 0);
	cp_power_off(cp);
	cp_line(cp, CP_PDA_ACTIVE, 1, 0);
	dev_info(cp->dev, "modem power-on sequence\n");
	cp_line(cp, CP_PM_WRST_N, 1, 10);
	cp_line(cp, CP_PWR_ON, 1, 10);
	cp_line(cp, CP_NRESET, 1, 10);
	cp_line(cp, CP_WRST_N, 1, 0);
	cp->cp_powered = true;
}

static void cp_set_iocc(struct gs201_pcie_cp *cp, bool enable)
{
	dw_pcie_writel_dbi(&cp->pci, CP_DBI_COHERENCY_CONTROL_3,
			   enable ? 0x10101010 : 0);
	if (cp->sysreg)
		regmap_update_bits(cp->sysreg, cp->sysreg_offset,
				   CP_SYSREG_SHARABLE_MASK,
				   enable ? CP_SYSREG_SHARABLE_ENABLE : 0);
}

/* exynos_pcie_setup_rc() for EP_SAMSUNG_MODEM, plus Google's EQ-off write */
static void cp_setup_dbi(struct gs201_pcie_cp *cp, unsigned int speed)
{
	struct dw_pcie *pci = &cp->pci;
	u8 cap;
	u32 val;

	dev_info(cp->dev, "DBI ID %#010x\n", dw_pcie_readl_dbi(pci, PCI_VENDOR_ID));

	dw_pcie_dbi_ro_wr_en(pci);

	/* Root port BAR0 and expansion ROM BAR off (DBI2 shadow, +0x100000) */
	dw_pcie_writel_dbi(pci, 0x100010, 0);
	dw_pcie_writel_dbi(pci, 0x100030, 0);

	dw_pcie_writew_dbi(pci, PCI_VENDOR_ID, PCI_VENDOR_ID_SAMSUNG);
	dw_pcie_writew_dbi(pci, PCI_DEVICE_ID, 0xecec);	/* EXYNOS + ch 0 */

	/* "EQ off", as Google writes before PERST# release */
	dw_pcie_writel_dbi(pci, GEN3_RELATED_OFF, 0x12000);

	dw_pcie_writel_dbi(pci, CP_DBI_AUX_CLK_FREQ, CP_DBI_AUX_CLK_26MHZ);

	cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	if (cap) {
		val = dw_pcie_readl_dbi(pci, cap + PCI_EXP_LNKCAP);
		val &= ~(PCI_EXP_LNKCAP_L1EL | PCI_EXP_LNKCAP_SLS);
		val |= FIELD_PREP(PCI_EXP_LNKCAP_L1EL, 7) | PCI_EXP_LNKCAP_SLS_8_0GB;
		dw_pcie_writel_dbi(pci, cap + PCI_EXP_LNKCAP, val);

		val = dw_pcie_readl_dbi(pci, cap + PCI_EXP_LNKCTL2);
		val &= ~PCI_EXP_LNKCTL2_TLS;
		val |= speed;
		dw_pcie_writel_dbi(pci, cap + PCI_EXP_LNKCTL2, val);

		val = dw_pcie_readl_dbi(pci, cap + PCI_EXP_DEVCTL2);
		val &= ~PCI_EXP_DEVCTL2_COMP_TIMEOUT;
		val |= 0x5;	/* range B, 16-55 ms */
		dw_pcie_writel_dbi(pci, cap + PCI_EXP_DEVCTL2, val);
	}

	dw_pcie_dbi_ro_wr_dis(pci);
}

/*
 * Configuration space. The root port is the controller's own (DBI); the
 * endpoint is reached through the window the secure monitor maps to bus 1,
 * device 0. Neither may be touched while the link is down: a DBI access
 * hangs the SoC while the PHY is isolated, and the window has no target.
 * cfg_lock makes the checks and the accesses atomic against link_down.
 */
static void cp_cfg_allow(struct gs201_pcie_cp *cp, bool root, bool ep)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&cp->cfg_lock, flags);
	cp->cfg_root_ok = root;
	cp->cfg_ep_ok = ep;
	raw_spin_unlock_irqrestore(&cp->cfg_lock, flags);
}

static void __iomem *cp_root_map_bus(struct pci_bus *bus, unsigned int devfn,
				     int where)
{
	struct gs201_pcie_cp *cp = bus->sysdata;

	if (devfn || !cp->cfg_root_ok)
		return NULL;
	return cp->pci.dbi_base + where;
}

static void __iomem *cp_child_map_bus(struct pci_bus *bus, unsigned int devfn,
				      int where)
{
	struct gs201_pcie_cp *cp = bus->sysdata;

	if (devfn || !pci_is_root_bus(bus->parent) || !cp->cfg_ep_ok)
		return NULL;
	return cp->cfg + where;
}

static int cp_cfg_read(struct pci_bus *bus, unsigned int devfn, int where,
		       int size, u32 *val)
{
	struct gs201_pcie_cp *cp = bus->sysdata;
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&cp->cfg_lock, flags);
	ret = pci_generic_config_read(bus, devfn, where, size, val);
	raw_spin_unlock_irqrestore(&cp->cfg_lock, flags);

	return ret;
}

static int cp_cfg_write(struct pci_bus *bus, unsigned int devfn, int where,
			int size, u32 val)
{
	struct gs201_pcie_cp *cp = bus->sysdata;
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&cp->cfg_lock, flags);
	ret = pci_generic_config_write(bus, devfn, where, size, val);
	raw_spin_unlock_irqrestore(&cp->cfg_lock, flags);

	return ret;
}

static struct pci_ops cp_root_ops = {
	.map_bus = cp_root_map_bus,
	.read = cp_cfg_read,
	.write = cp_cfg_write,
};

static struct pci_ops cp_child_ops = {
	.map_bus = cp_child_map_bus,
	.read = cp_cfg_read,
	.write = cp_cfg_write,
};

/*
 * MSI: the DWC iMSI-RX block, one GIC SPI per block of 32 vectors
 * ("msi0".."msi4", Google's "separated" MSI), aimed at the modem's MSI
 * page as Google's driver does (exynos_pcie_set_msi_ctrl_addr()). The
 * modem writes its boot stage into the same page, so it is reserved RAM.
 * The chained handlers read DBI, so their parent interrupts are disabled
 * before the PHY goes back into isolation and enabled again after the next
 * link_up.
 */
static void cp_msi_irqs(struct gs201_pcie_cp *cp, bool on)
{
	struct dw_pcie_rp *pp = &cp->pci.pp;
	unsigned int i;

	if (!cp->msi_ready || cp->msi_irqs_on == on)
		return;
	for (i = 0; i < MAX_MSI_CTRLS; i++) {
		if (pp->msi_irq[i] <= 0)
			continue;
		if (on)
			enable_irq(pp->msi_irq[i]);
		else
			disable_irq(pp->msi_irq[i]);
	}
	cp->msi_irqs_on = on;
}

static int cp_msi_setup(struct gs201_pcie_cp *cp)
{
	struct dw_pcie_rp *pp = &cp->pci.pp;
	int ret;

	if (!cp->msi_page.start) {
		dev_warn(cp->dev, "no MSI page (memory-region): no MSI\n");
		return 0;
	}
	if (!cp->msi_ready) {
		raw_spin_lock_init(&pp->lock);
		pp->use_imsi_rx = true;
		ret = dw_pcie_msi_host_init(pp);	/* handlers on, enabled */
		if (ret)
			return ret;
		pp->msi_data = cp->msi_page.start;
		cp->msi_ready = true;
		cp->msi_irqs_on = true;
		dev_info(cp->dev, "MSI: %u vectors, target %pa\n",
			 pp->num_vectors, &pp->msi_data);
	}
	dw_pcie_msi_init(pp);	/* the controller reset cleared these */
	cp_msi_irqs(cp, true);

	return 0;
}

static int cp_bus_report(struct pci_dev *pdev, void *data)
{
	struct gs201_pcie_cp *cp = data;

	dev_info(cp->dev, "%s %04x:%04x class %06x\n", pci_name(pdev),
		 pdev->vendor, pdev->device, pdev->class);
	return 0;
}

static int cp_bus(struct gs201_pcie_cp *cp)
{
	int ret;

	if (!cp->link_up || !cp->atu_done)
		return -ENOLINK;

	ret = cp_msi_setup(cp);
	if (ret)
		return ret;

	if (cp->bridge->bus) {
		dev_info(cp->dev, "bus already registered, MSI re-armed\n");
		return 0;
	}

	ret = pci_host_probe(cp->bridge);
	if (ret)
		return ret;
	pci_walk_bus(cp->bridge->bus, cp_bus_report, cp);

	return 0;
}

static void cp_bus_remove(struct gs201_pcie_cp *cp)
{
	if (!cp->bridge->bus)
		return;
	pci_lock_rescan_remove();
	pci_stop_root_bus(cp->bridge->bus);
	pci_remove_root_bus(cp->bridge->bus);
	pci_unlock_rescan_remove();
}

/*
 * Isolate the PHY again once it is powered down. The PHY driver's exit
 * leaves isolation off, because the Wi-Fi link's controller is accessed
 * after it (Google never isolates that PHY again either). The modem link
 * is different: it is down most of the time, and Google's driver isolates
 * every link that is down when the system suspends
 * (exynos_pcie_rc_suspend_noirq()). The write goes through the same
 * SMC-backed PMU regmap as the release in phy_init(). No controller
 * register is accessed after this until the next link_up.
 */
static void cp_phy_isolate(struct gs201_pcie_cp *cp)
{
	unsigned int val = 0;
	int ret;

	ret = regmap_update_bits(cp->pmu, cp->pmu_offset, CP_PMU_PHY_BYPASS, 0);
	if (!ret)
		ret = regmap_read(cp->pmu, cp->pmu_offset, &val);
	if (ret)
		dev_err(cp->dev, "PHY isolation: PMU %#x: %d\n", cp->pmu_offset, ret);
	else
		dev_info(cp->dev, "PHY isolated (PMU %#x = %#x)\n", cp->pmu_offset, val);
}

/*
 * exynos_pcie_rc_poweroff(): PERST#, LTSSM off, a soft power reset of the
 * controller, PHY power-down; then isolation, as on suspend.
 */
static void cp_link_down(struct gs201_pcie_cp *cp)
{
	bool was_inited = cp->phy_inited;
	u32 val;

	cp_cfg_allow(cp, false, false);
	cp_msi_irqs(cp, false);

	if (cp->phy_inited) {
		gpiod_direction_output(cp->perst, 1);
		cp_elbi_write(cp, CP_ELBI_LTSSM_EN, 0);
		cp_elbi_write(cp, CP_ELBI_IRQ0_EN, 0);
		cp_elbi_write(cp, CP_ELBI_IRQ1_EN, 0);
		cp_elbi_write(cp, CP_ELBI_IRQ2_EN, 0);
		val = cp_elbi_read(cp, CP_ELBI_SOFT_RESET);
		cp_elbi_write(cp, CP_ELBI_SOFT_RESET, val & ~CP_ELBI_SOFT_PWR_RESET);
		udelay(20);
		cp_elbi_write(cp, CP_ELBI_SOFT_RESET, val | CP_ELBI_SOFT_PWR_RESET);
	}
	if (cp->phy_on) {
		phy_power_off(cp->phy);
		cp->phy_on = false;
	}
	if (cp->phy_inited) {
		phy_exit(cp->phy);	/* clocks only: isolation stays off */
		cp->phy_inited = false;
	}
	/* Only undo what link_up did: an untouched PHY is still isolated. */
	if (was_inited)
		cp_phy_isolate(cp);
	cp->link_up = false;
	cp->atu_done = false;
}

/* exynos_pcie_rc_establish_link(), one try */
static int cp_link_try(struct gs201_pcie_cp *cp, unsigned int speed)
{
	u32 val;
	int ret;

	ret = phy_power_on(cp->phy);	/* power-down clear, CAL, lock checks */
	if (ret)
		return ret;
	cp->phy_on = true;

	val = cp_elbi_read(cp, CP_ELBI_SOFT_RESET);
	cp_elbi_write(cp, CP_ELBI_SOFT_RESET, val & ~CP_ELBI_SOFT_PWR_RESET);
	mdelay(1);
	cp_elbi_write(cp, CP_ELBI_SOFT_RESET, val | CP_ELBI_SOFT_PWR_RESET);
	phy_calibrate(cp->phy);

	cp_elbi_write(cp, CP_ELBI_DEVICE_TYPE, CP_ELBI_DEVICE_TYPE_RC);

	val = cp_elbi_read(cp, CP_ELBI_SOFT_RESET);
	cp_elbi_write(cp, CP_ELBI_SOFT_RESET, val | CP_ELBI_SOFT_NON_STICKY_RESET);
	usleep_range(10, 12);
	cp_elbi_write(cp, CP_ELBI_SOFT_RESET, val & ~CP_ELBI_SOFT_NON_STICKY_RESET);
	mdelay(1);
	cp_elbi_write(cp, CP_ELBI_SOFT_RESET, val | CP_ELBI_SOFT_NON_STICKY_RESET);

	gpiod_set_value_cansleep(cp->perst, 0);
	fsleep(CP_PERST_DELAY_US);

	cp_elbi_rmw(cp, CP_ELBI_REQ_EXIT_L1_MODE, 0,
		    CP_ELBI_APP_REQ_EXIT_L1 | CP_ELBI_L1_REQ_NAK_MASTER);
	cp_elbi_write(cp, CP_ELBI_LINKDOWN_RST_SEL, CP_ELBI_LINKDOWN_RST_MANUAL);
	cp_elbi_rmw(cp, CP_ELBI_QCH_SEL, CP_ELBI_QCH_GATING_MASK, 0);
	cp_elbi_write(cp, CP_ELBI_MSTR_PEND_SEL_NAK, 1);
	cp_elbi_write(cp, CP_ELBI_DBI_L1_EXIT_DISABLE, 1);

	cp_setup_dbi(cp, speed);
	cp_set_iocc(cp, true);

	cp_elbi_write(cp, CP_ELBI_LTSSM_EN, 1);
	ret = readl_poll_timeout(cp->elbi + CP_ELBI_LTSSM_STATE, val,
				 FIELD_GET(CP_ELBI_LTSSM_MASK, val) ==
				 DW_PCIE_LTSSM_L0,
				 CP_LINK_POLL_US, CP_LINK_TIMEOUT_US);
	if (ret)
		dev_warn(cp->dev, "no L0, LTSSM %#lx\n",
			 FIELD_GET(CP_ELBI_LTSSM_MASK, val));
	return ret;
}

static int cp_link_up(struct gs201_pcie_cp *cp, unsigned int speed)
{
	struct dw_pcie *pci = &cp->pci;
	unsigned int try;
	u16 lnksta;
	u8 cap;
	int ret;

	if (cp->has_lines && !cp->cp_powered)
		dev_warn(cp->dev, "modem not powered (cp_on): expect no link\n");

	if (!cp->phy_inited) {
		/* PERST# asserted and CLKREQ# in its function before the PHY */
		gpiod_direction_output(cp->perst, 1);
		if (cp->pins_active) {
			ret = pinctrl_select_state(cp->pinctrl, cp->pins_active);
			if (ret)
				return ret;
		}
		phy_set_bus_width(cp->phy, cp->num_lanes);
		ret = phy_init(cp->phy);	/* PMU isolation off, then PHY power-down */
		if (ret)
			return ret;
		cp->phy_inited = true;
	}

	for (try = 1; try <= CP_LINK_TRIES; try++) {
		ret = cp_link_try(cp, speed);
		if (!ret)
			break;
		gpiod_set_value_cansleep(cp->perst, 1);
		cp_elbi_write(cp, CP_ELBI_LTSSM_EN, 0);
		phy_power_off(cp->phy);
		cp->phy_on = false;
	}
	if (ret)
		return ret;

	fsleep(CP_SPEED_SETTLE_US);
	cap = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	lnksta = cap ? dw_pcie_readw_dbi(pci, cap + PCI_EXP_LNKSTA) : 0;
	dev_info(cp->dev, "link up: Gen%u x%u (target Gen%u), try %u\n",
		 (unsigned int)FIELD_GET(PCI_EXP_LNKSTA_CLS, lnksta),
		 (unsigned int)FIELD_GET(PCI_EXP_LNKSTA_NLW, lnksta), speed, try);

	/* Clear what training latched; no interrupts are enabled */
	cp_elbi_write(cp, CP_ELBI_IRQ0, cp_elbi_read(cp, CP_ELBI_IRQ0));
	cp_elbi_write(cp, CP_ELBI_IRQ1, cp_elbi_read(cp, CP_ELBI_IRQ1));
	cp_elbi_write(cp, CP_ELBI_IRQ2, cp_elbi_read(cp, CP_ELBI_IRQ2));
	cp->link_up = true;
	cp_cfg_allow(cp, true, false);

	return 0;
}

static int cp_secure_atu(struct gs201_pcie_cp *cp)
{
	struct arm_smccc_res res;

	arm_smccc_smc(CP_SMC_SECURE_ATU_SETUP, 0, 0, 0, 0, 0, 0, 0, &res);
	dev_info(cp->dev, "secure ATU SMC: %#lx\n", res.a0);
	if (res.a0)
		return -EIO;
	cp->atu_done = true;
	cp_cfg_allow(cp, true, true);

	return 0;
}

static int cp_read_id(struct gs201_pcie_cp *cp)
{
	u32 id, class;

	id = readl(cp->cfg + PCI_VENDOR_ID);
	class = readl(cp->cfg + PCI_CLASS_REVISION);
	dev_info(cp->dev, "endpoint %04x:%04x class %06x\n",
		 id & 0xffff, id >> 16, class >> 8);

	return (id & 0xffff) == PCI_VENDOR_ID_SAMSUNG ? 0 : -ENODEV;
}

static ssize_t cp_ctl_write(struct file *file, const char __user *ubuf,
			    size_t len, loff_t *ppos)
{
	struct gs201_pcie_cp *cp = file_inode(file)->i_private;
	char buf[32], *cmd;
	unsigned int speed = 1;
	int ret = 0;

	if (!allow_hw)
		return -EPERM;
	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	cmd = strim(buf);

	mutex_lock(&cp->lock);
	if (!strcmp(cmd, "cp_on") || !strcmp(cmd, "cp_off")) {
		if (!cp->has_lines) {
			ret = -ENODEV;	/* the modem driver owns the lines */
		} else if (cmd[4] == 'n') {
			cp_power_on(cp);
		} else {
			cp_link_down(cp);
			cp_power_off(cp);
		}
	} else if (!strncmp(cmd, "link_up", 7)) {
		if (cmd[7] == ' ' && kstrtouint(cmd + 8, 0, &speed))
			ret = -EINVAL;
		else if (speed < 1 || speed > 3)
			ret = -EINVAL;
		else
			ret = cp_link_up(cp, speed);
	} else if (!strcmp(cmd, "link_down")) {
		cp_link_down(cp);
	} else if (!strcmp(cmd, "atu")) {
		ret = cp->link_up ? cp_secure_atu(cp) : -ENOLINK;
	} else if (!strcmp(cmd, "id")) {
		ret = cp->atu_done ? cp_read_id(cp) : -ENOLINK;
	} else if (!strcmp(cmd, "bus")) {
		ret = cp_bus(cp);
	} else {
		ret = -EINVAL;
	}
	mutex_unlock(&cp->lock);

	if (ret)
		dev_err(cp->dev, "%s: %d\n", cmd, ret);
	return ret ?: len;
}

static const struct file_operations cp_ctl_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = cp_ctl_write,
	.llseek = noop_llseek,
};

static int cp_status_show(struct seq_file *s, void *unused)
{
	struct gs201_pcie_cp *cp = s->private;
	unsigned int i, iso = 0;

	mutex_lock(&cp->lock);
	if (!cp->has_lines)
		seq_puts(s, "modem lines: owned by the modem driver\n");
	for (i = 0; cp->has_lines && i < CP_NUM_OUTPUTS; i++)
		seq_printf(s, "%-20s %d\n", cp_output_names[i],
			   gpiod_get_raw_value_cansleep(cp->out[i]));
	for (i = 0; cp->has_lines && i < CP_NUM_INPUTS; i++)
		seq_printf(s, "%-20s %d\n", cp_input_names[i],
			   gpiod_get_raw_value_cansleep(cp->in[i]));
	seq_printf(s, "%-20s %d\n", "perst (asserted)",
		   gpiod_get_value_cansleep(cp->perst));
	regmap_read(cp->pmu, cp->pmu_offset, &iso);
	seq_printf(s, "PMU %#x: %#x (bit 0: 0 = PHY isolated)\n",
		   cp->pmu_offset, iso);
	seq_printf(s, "state: cp %d, phy %d/%d, link %d, atu %d\n",
		   cp->cp_powered, cp->phy_inited, cp->phy_on, cp->link_up,
		   cp->atu_done);
	if (cp->phy_inited)
		seq_printf(s, "LTSSM %#lx, IRQ0/1/2 %#x %#x %#x\n",
			   FIELD_GET(CP_ELBI_LTSSM_MASK,
				     cp_elbi_read(cp, CP_ELBI_LTSSM_STATE)),
			   cp_elbi_read(cp, CP_ELBI_IRQ0),
			   cp_elbi_read(cp, CP_ELBI_IRQ1),
			   cp_elbi_read(cp, CP_ELBI_IRQ2));
	mutex_unlock(&cp->lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cp_status);

/*
 * watch_lines: log each edge on a cp2ap input with the kernel's timestamp,
 * to line the modem's own state changes up with other events in the log
 * (the UFS errors seen around modem power changes in M2). The lines are
 * only read; requesting the interrupt switches the pin to its EINT
 * function, which is how stock uses all four.
 */
static irqreturn_t cp_watch_irq(int irq, void *data)
{
	struct gs201_pcie_cp_watch *w = data;
	struct gs201_pcie_cp *cp = w->cp;

	dev_info_ratelimited(cp->dev, "%s -> %d\n", cp_input_names[w->line],
			     gpiod_get_raw_value_cansleep(cp->in[w->line]));
	return IRQ_HANDLED;
}

static void cp_watch_lines(struct gs201_pcie_cp *cp)
{
	unsigned int i;
	int irq, ret;

	for (i = 0; i < CP_NUM_INPUTS; i++) {
		cp->watch[i].cp = cp;
		cp->watch[i].line = i;
		irq = gpiod_to_irq(cp->in[i]);
		if (irq < 0) {
			dev_warn(cp->dev, "%s: no interrupt (%d), not watched\n",
				 cp_input_names[i], irq);
			continue;
		}
		ret = devm_request_threaded_irq(cp->dev, irq, NULL, cp_watch_irq,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						cp_input_names[i], &cp->watch[i]);
		if (ret)
			dev_warn(cp->dev, "%s: cannot watch: %d\n",
				 cp_input_names[i], ret);
		else
			dev_info(cp->dev, "watching %s (now %d)\n",
				 cp_input_names[i],
				 gpiod_get_raw_value_cansleep(cp->in[i]));
	}
}

/*
 * The modem's lines: ours when the "modem" child node is available (the
 * debugfs steps cp_on/cp_off), otherwise the modem driver's, which then
 * describes them in its own node. Exactly one of the two may own them.
 */
static int cp_get_lines(struct gs201_pcie_cp *cp)
{
	struct device *dev = cp->dev;
	struct fwnode_handle *modem;
	unsigned int i;
	int ret;

	modem = device_get_named_child_node(dev, "modem");
	if (!modem)
		return 0;
	if (!fwnode_device_is_available(modem)) {
		fwnode_handle_put(modem);
		return 0;
	}

	/* All lines as the bootloader left them until a step drives them. */
	for (i = 0; i < CP_NUM_OUTPUTS; i++) {
		cp->out[i] = devm_fwnode_gpiod_get(dev, modem, cp_output_names[i],
						   GPIOD_ASIS, cp_output_names[i]);
		if (IS_ERR(cp->out[i])) {
			ret = PTR_ERR(cp->out[i]);
			goto err_node;
		}
	}
	for (i = 0; i < CP_NUM_INPUTS; i++) {
		cp->in[i] = devm_fwnode_gpiod_get(dev, modem, cp_input_names[i],
						  GPIOD_ASIS, cp_input_names[i]);
		if (IS_ERR(cp->in[i])) {
			ret = PTR_ERR(cp->in[i]);
			goto err_node;
		}
	}
	fwnode_handle_put(modem);
	cp->has_lines = true;

	return 0;

err_node:
	fwnode_handle_put(modem);
	return dev_err_probe(dev, ret, "modem line %s\n",
			     i < CP_NUM_OUTPUTS ? cp_output_names[i] : "input");
}

static int gs201_pcie_cp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct gs201_pcie_cp *cp;
	struct resource *res;
	int ret;

	cp = devm_kzalloc(dev, sizeof(*cp), GFP_KERNEL);
	if (!cp)
		return -ENOMEM;
	cp->dev = dev;
	mutex_init(&cp->lock);
	raw_spin_lock_init(&cp->cfg_lock);
	platform_set_drvdata(pdev, cp);

	/* Mappings only: nothing here touches the controller or the PHY. */
	cp->elbi = devm_platform_ioremap_resource_byname(pdev, "elbi");
	if (IS_ERR(cp->elbi))
		return PTR_ERR(cp->elbi);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	cp->pci.dbi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(cp->pci.dbi_base))
		return PTR_ERR(cp->pci.dbi_base);
	cp->pci.dev = dev;

	cp->cfg = devm_platform_ioremap_resource_byname(pdev, "config");
	if (IS_ERR(cp->cfg))
		return PTR_ERR(cp->cfg);

	cp->sysreg = syscon_regmap_lookup_by_phandle_args(np, "samsung,sysreg",
							   1, &cp->sysreg_offset);
	if (IS_ERR(cp->sysreg))
		return dev_err_probe(dev, PTR_ERR(cp->sysreg), "no sysreg\n");

	cp->pmu = exynos_get_pmu_regmap_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(cp->pmu))
		return dev_err_probe(dev, PTR_ERR(cp->pmu), "no PMU regmap\n");
	ret = of_property_read_u32_index(np, "samsung,pmu-syscon", 1,
					 &cp->pmu_offset);
	if (ret)
		return dev_err_probe(dev, ret, "no PMU offset\n");

	/* The bus: windows from "ranges", registered only by the bus step */
	cp->bridge = devm_pci_alloc_host_bridge(dev, 0);
	if (!cp->bridge)
		return -ENOMEM;
	cp->bridge->ops = &cp_root_ops;
	cp->bridge->child_ops = &cp_child_ops;
	cp->bridge->sysdata = cp;
	cp->pci.pp.bridge = cp->bridge;

	if (of_reserved_mem_region_to_resource(np, 0, &cp->msi_page))
		memset(&cp->msi_page, 0, sizeof(cp->msi_page));

	cp->num_lanes = 2;
	of_property_read_u32(np, "num-lanes", &cp->num_lanes);

	cp->phy = devm_phy_get(dev, "pcie");
	if (IS_ERR(cp->phy))
		return dev_err_probe(dev, PTR_ERR(cp->phy), "no PHY\n");

	/* As found: PERST# is set only by link_up/link_down. */
	cp->perst = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(cp->perst))
		return dev_err_probe(dev, PTR_ERR(cp->perst), "no PERST#\n");

	/* PCIe pins ("active", as stock) are selected by link_up only. */
	cp->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(cp->pinctrl)) {
		cp->pins_active = pinctrl_lookup_state(cp->pinctrl, "active");
		if (IS_ERR(cp->pins_active))
			cp->pins_active = NULL;
	}

	ret = cp_get_lines(cp);
	if (ret)
		return ret;

	if (watch_lines && cp->has_lines)
		cp_watch_lines(cp);

	cp->debugfs = debugfs_create_dir("gs201-pcie-cp", NULL);
	debugfs_create_file("status", 0400, cp->debugfs, cp, &cp_status_fops);
	debugfs_create_file("ctl", 0200, cp->debugfs, cp, &cp_ctl_fops);

	dev_info(dev, "modem PCIe ch0 bring-up: %u lanes, steps %s, modem lines %s\n",
		 cp->num_lanes, allow_hw ? "enabled" : "disabled (allow_hw=0)",
		 cp->has_lines ? "here" : "left to the modem driver");

	return 0;
}

static void gs201_pcie_cp_remove(struct platform_device *pdev)
{
	struct gs201_pcie_cp *cp = platform_get_drvdata(pdev);

	debugfs_remove_recursive(cp->debugfs);
	mutex_lock(&cp->lock);
	cp_bus_remove(cp);
	cp_link_down(cp);
	if (cp->msi_ready) {
		dw_pcie_free_msi(&cp->pci.pp);
		cp->msi_ready = false;
	}
	if (cp->cp_powered)
		cp_power_off(cp);
	mutex_unlock(&cp->lock);
}

static void gs201_pcie_cp_shutdown(struct platform_device *pdev)
{
	gs201_pcie_cp_remove(pdev);
}

static const struct of_device_id gs201_pcie_cp_of_match[] = {
	{ .compatible = "google,gs201-pcie-cp" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs201_pcie_cp_of_match);

static struct platform_driver gs201_pcie_cp_driver = {
	.probe = gs201_pcie_cp_probe,
	.remove = gs201_pcie_cp_remove,
	.shutdown = gs201_pcie_cp_shutdown,
	.driver = {
		.name = "gs201-pcie-cp",
		.of_match_table = gs201_pcie_cp_of_match,
	},
};
module_platform_driver(gs201_pcie_cp_driver);

MODULE_DESCRIPTION("Google gs201 PCIe GEN4A_0 (modem) bring-up");
MODULE_LICENSE("GPL");
