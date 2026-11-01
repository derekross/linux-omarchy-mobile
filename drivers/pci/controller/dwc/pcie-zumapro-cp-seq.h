/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Tensor G4 (zumapro) PCIe channel 0 (HSI1 GEN3A_0, the Pixel 9a's S5300
 * modem link, two lanes), for pcie-gs201-cp.c: the PHY register sequences
 * of Google's GPL root-complex CAL for the zuma family, as built for tegu
 * (google-modules/soc/gs @ 204078758737, drivers/pci/controller/dwc/
 * pcie-exynos-zuma-rc-cal.c, "PCIe CAL ver 0.2"): the `else` (channel 0)
 * branches of exynos_pcie_rc_phy_all_pwrdn_clear(),
 * exynos_pcie_rc_pcie_phy_config() and exynos_pcie_rc_phy_all_pwrdn().
 * Written out by hand and checked against the source statement by
 * statement.
 *
 * Blocks: ELBI, SOC (soc_ctrl), UDBG, PHY (PMA), PCS; offsets relative to
 * each block. POLL_SET waits for the bit to read 1, as the CAL's
 * check_exynos_pcie_reg_status() (10000 x 10 us, then logs and goes on).
 * POLL_CLR_STOP waits for the bit to read 0 (100 x 1 us) and, if it never
 * does, ends the table there, as the CAL returns early from the power-down.
 *
 * The lane loop of phy_config: the CAL adds i * 0x1000 to phy_base_regs on
 * each pass and never takes it back, so lane 1 is at PMA +0x1000 and every
 * PMA access after the loop (the PLL, CDR and offset-calibration polls and
 * the final 0x32C write) lands at +0x1000 too: 0x1A80, 0x25C0, 0x240C and
 * 0x132C. That is what stock runs; the table keeps it. The host's own
 * PMA 0x32C = 0 after the table (exynos_pcie_rc_establish_link) is at the
 * base and is not part of this table.
 */
#ifndef PCIE_ZUMAPRO_CP_SEQ_H
#define PCIE_ZUMAPRO_CP_SEQ_H

#include <linux/types.h>

enum zumapro_cp_blk { BLK0_ELBI, BLK0_SOC, BLK0_UDBG, BLK0_PHY, BLK0_PCS, BLK0_COUNT };
enum zumapro_cp_opcode {
	OP0_WRITE,
	OP0_RMW,		/* (old & ~clear) | value */
	OP0_DELAY_US,
	OP0_POLL_SET,		/* bit "value" reads 1 */
	OP0_POLL_CLR_STOP,	/* bit "value" reads 0, else stop the table */
	OP0_READ,		/* a read the CAL does for its log only */
};

struct zumapro_cp_op {
	u8 op;
	u8 blk;
	u16 offset;
	u32 value;	/* write value, RMW set mask, delay in us, poll bit */
	u32 clear;	/* RMW clear mask */
};

#define W0(b, o, v)	{ OP0_WRITE, BLK0_##b, (o), (v), 0 }
#define RMW0(b, o, c, s) { OP0_RMW, BLK0_##b, (o), (s), (c) }
#define DLY0(us)	{ OP0_DELAY_US, 0, 0, (us), 0 }
#define POLL0(b, o, bit) { OP0_POLL_SET, BLK0_##b, (o), (bit), 0 }

/* exynos_pcie_rc_phy_all_pwrdn_clear(), channel 0 */
static const struct zumapro_cp_op zumapro_cp_phy_pwrdn_clear[] = {
	RMW0(UDBG, 0xC700, 0x2, 0x0),	/* & ~(0x1 << 1) */
	DLY0(100),
	RMW0(PHY, 0x0600, 0x30, 0x0),	/* restore PLL lock signal: & ~(0x3 << 4) */
	W0(PHY, 0x000C, 0x00),
	W0(PHY, 0x0928, 0x55),
	W0(PHY, 0x0580, 0x02),
	W0(PHY, 0x0208, 0x00),
	W0(PHY, 0x0228, 0x00),
	W0(PHY, 0x1044, 0x00),
	W0(PHY, 0x1048, 0x00),
	W0(PHY, 0x104C, 0x00),
	W0(PHY, 0x1050, 0x00),
	W0(PHY, 0x185C, 0x00),
	W0(PHY, 0x2044, 0x00),
	W0(PHY, 0x2048, 0x00),
	W0(PHY, 0x204C, 0x00),
	W0(PHY, 0x2050, 0x00),
	W0(PHY, 0x285C, 0x00),
	DLY0(10),
};

/* PMA offset still applied after the loop: 0 + 1 * 0x1000 */
#define ZUMAPRO_CP_AFTER_LOOP	0x1000

/* exynos_pcie_rc_pcie_phy_config(), channel 0, num_lanes = 2 */
static const struct zumapro_cp_op zumapro_cp_phy_config[] = {
	W0(PHY, 0x032C, 0x3),		/* PHY input clock un-gating */
	RMW0(UDBG, 0xC710, 0x2, 0x0),	/* external PLL initialization */
	RMW0(UDBG, 0xC700, 0x2, 0x0),	/* override external PLL RESETB */
	POLL0(UDBG, 0xC704, 3),		/* ext_pll_lock */
	RMW0(SOC, 0x4004, 0x4, 0x0),	/* PHY input clock: external PLL */
	RMW0(SOC, 0x4004, 0x0, 0x30),
	W0(ELBI, 0x0080, 0x4),		/* device type */
	W0(ELBI, 0x03A4, 0xF),		/* soft_pwr_rst */
	W0(ELBI, 0x03A4, 0xD),
	DLY0(10),
	W0(ELBI, 0x03A4, 0xF),
	DLY0(10),
	W0(ELBI, 0x1404, 0x1),		/* PMA reset assert */
	W0(ELBI, 0x1408, 0x1),
	W0(ELBI, 0x1400, 0x1),
	W0(ELBI, 0x1404, 0x0),
	W0(ELBI, 0x1408, 0x0),
	W0(ELBI, 0x1400, 0x0),
	W0(ELBI, 0x03D8, 0x1),		/* PCIE_SLV_PEND_SEL_NAK */
	/* PMA common */
	W0(PHY, 0x0000, 0x88),
	W0(PHY, 0x001C, 0x66),
	W0(PHY, 0x01F4, 0x00),
	W0(PHY, 0x0510, 0x81),
	W0(PHY, 0x0514, 0x59),
	W0(PHY, 0x051C, 0x11),
	W0(PHY, 0x062C, 0x0E),
	W0(PHY, 0x0644, 0x22),
	W0(PHY, 0x0688, 0x03),
	W0(PHY, 0x06D4, 0x28),
	W0(PHY, 0x0788, 0x64),
	W0(PHY, 0x078C, 0x64),
	W0(PHY, 0x0790, 0x50),
	W0(PHY, 0x0794, 0x50),
	W0(PHY, 0x0944, 0x05),
	W0(PHY, 0x0948, 0x05),
	W0(PHY, 0x094C, 0x05),
	W0(PHY, 0x0950, 0x05),
	/* REFCLK 38.4 MHz to the external PLL path */
	W0(PHY, 0x0590, 0x02),
	W0(PHY, 0x07F8, 0xB0),
	W0(PHY, 0x0730, 0x08),
	W0(PHY, 0x0344, 0xC0),		/* delay at L1.2 */
	W0(PHY, 0x0040, 0x04),
	W0(PHY, 0x0204, 0x03),
	W0(PHY, 0x02D4, 0x1F),
	W0(PHY, 0x0358, 0x10),
	W0(PHY, 0x0018, 0x01),		/* ERIO on */
	/* lane */
	W0(PHY, 0x0514, 0x5B),
	W0(PHY, 0x0608, 0x0C),
	W0(PHY, 0x060C, 0x0F),
	W0(PHY, 0x0610, 0x0F),
	W0(PHY, 0x0614, 0x0F),
	W0(PHY, 0x0618, 0x0F),
	W0(PHY, 0x0510, 0x80),
	W0(PHY, 0x0688, 0x03),
	W0(PHY, 0x0644, 0x23),
	W0(PHY, 0x0624, 0x11),
	W0(PHY, 0x0630, 0x0F),		/* PLL margin for ERIO, Gen1/Gen2 */
	W0(PHY, 0x06D0, 0x53),
	/* lane 0 of the loop (i = 0: PMA base + 0) */
	W0(PHY, 0x1140, 0x04),
	W0(PHY, 0x1144, 0x04),
	W0(PHY, 0x1148, 0x04),
	W0(PHY, 0x114C, 0x02),
	W0(PHY, 0x1150, 0x00),
	W0(PHY, 0x1154, 0x00),
	W0(PHY, 0x1158, 0x00),
	W0(PHY, 0x115C, 0x00),
	W0(PHY, 0x12CC, 0x1C),
	W0(PHY, 0x12DC, 0x6C),
	W0(PHY, 0x130C, 0x29),
	W0(PHY, 0x13B4, 0x2F),
	W0(PHY, 0x1A64, 0x05),
	W0(PHY, 0x1A68, 0x05),
	W0(PHY, 0x1A84, 0x05),
	W0(PHY, 0x1A88, 0x05),
	W0(PHY, 0x1A98, 0x00),
	W0(PHY, 0x1A9C, 0x00),
	W0(PHY, 0x1AA8, 0x07),
	W0(PHY, 0x1AB8, 0x00),
	W0(PHY, 0x1ABC, 0x00),
	W0(PHY, 0x1AF8, 0x90),
	W0(PHY, 0x1B34, 0x03),
	W0(PHY, 0x1BB0, 0x03),
	W0(PHY, 0x1BB4, 0x03),
	W0(PHY, 0x1BC0, 0x06),
	W0(PHY, 0x1BC4, 0x06),
	W0(PHY, 0x1BE8, 0x01),
	W0(PHY, 0x1BF8, 0x04),
	W0(PHY, 0x1C98, 0x00),
	W0(PHY, 0x1CA4, 0x81),
	/* lane 1 (i = 1: the CAL adds 0x1000 to the PMA base) */
	W0(PHY, 0x2140, 0x04),
	W0(PHY, 0x2144, 0x04),
	W0(PHY, 0x2148, 0x04),
	W0(PHY, 0x214C, 0x02),
	W0(PHY, 0x2150, 0x00),
	W0(PHY, 0x2154, 0x00),
	W0(PHY, 0x2158, 0x00),
	W0(PHY, 0x215C, 0x00),
	W0(PHY, 0x22CC, 0x1C),
	W0(PHY, 0x22DC, 0x6C),
	W0(PHY, 0x230C, 0x29),
	W0(PHY, 0x23B4, 0x2F),
	W0(PHY, 0x2A64, 0x05),
	W0(PHY, 0x2A68, 0x05),
	W0(PHY, 0x2A84, 0x05),
	W0(PHY, 0x2A88, 0x05),
	W0(PHY, 0x2A98, 0x00),
	W0(PHY, 0x2A9C, 0x00),
	W0(PHY, 0x2AA8, 0x07),
	W0(PHY, 0x2AB8, 0x00),
	W0(PHY, 0x2ABC, 0x00),
	W0(PHY, 0x2AF8, 0x90),
	W0(PHY, 0x2B34, 0x03),
	W0(PHY, 0x2BB0, 0x03),
	W0(PHY, 0x2BB4, 0x03),
	W0(PHY, 0x2BC0, 0x06),
	W0(PHY, 0x2BC4, 0x06),
	W0(PHY, 0x2BE8, 0x01),
	W0(PHY, 0x2BF8, 0x04),
	W0(PHY, 0x2C98, 0x00),
	W0(PHY, 0x2CA4, 0x81),
	/* PCS */
	W0(PCS, 0x0190, 0x100B0604),
	W0(PCS, 0x0154, 0x000700D5),
	W0(PCS, 0x0100, 0x16400000),
	W0(PCS, 0x0104, 0x08600000),
	W0(PCS, 0x0114, 0x18500000),
	W0(PCS, 0x0124, 0x60700000),
	W0(PCS, 0x0174, 0x00000007),
	W0(PCS, 0x0178, 0x00000100),
	W0(PCS, 0x017C, 0x00000010),
	W0(ELBI, 0x1404, 0x1),		/* PMA reset release */
	DLY0(10),
	W0(ELBI, 0x1408, 0x1),
	W0(ELBI, 0x1400, 0x1),
	POLL0(PHY, ZUMAPRO_CP_AFTER_LOOP + 0x0A80, 0),	/* pll_lock */
	POLL0(PHY, ZUMAPRO_CP_AFTER_LOOP + 0x15C0, 4),	/* cdr_lock */
	POLL0(PHY, ZUMAPRO_CP_AFTER_LOOP + 0x140C, 7),	/* oc_done */
	RMW0(UDBG, 0xC710, 0x300, 0x300), /* ext PLL clock gating in L2 */
	W0(PHY, ZUMAPRO_CP_AFTER_LOOP + 0x032C, 0x0),
	W0(ELBI, 0x1078, 0x1),		/* L1 exit off by DBI */
};

/* exynos_pcie_rc_phy_all_pwrdn(), channel 0 */
static const struct zumapro_cp_op zumapro_cp_phy_pwrdn[] = {
	RMW0(PHY, 0x0204, 0xC, 0x0),	/* & ~(0x3 << 2) */
	W0(PCS, 0x0150, 0x300D9),	/* PLL off, bias on */
	DLY0(5),
	{ OP0_READ, BLK0_PCS, 0x0150, 0, 0 },
	W0(PHY, 0x1044, 0x2A),
	W0(PHY, 0x1048, 0xAA),
	W0(PHY, 0x104C, 0xA8),
	W0(PHY, 0x1050, 0x80),
	W0(PHY, 0x185C, 0x0A),
	DLY0(1),
	W0(PHY, 0x2044, 0x2A),
	W0(PHY, 0x2048, 0xAA),
	W0(PHY, 0x204C, 0xA8),
	W0(PHY, 0x2050, 0x80),
	W0(PHY, 0x285C, 0x0A),
	DLY0(1),
	W0(PHY, 0x0208, 0xFF),
	W0(PHY, 0x0228, 0xFF),
	DLY0(1),
	W0(PHY, 0x0580, 0x0A),
	W0(PHY, 0x0928, 0xAA),
	W0(PHY, 0x000C, 0x0A),
	/*
	 * Force PLL lock signal off. The CAL computes "phy_val & ~(3 << 4)"
	 * and then overwrites it with "phy_val | (2 << 4)", so only bit 5 is
	 * set and bit 4 stays as it was: that is what runs, so it is kept.
	 */
	RMW0(PHY, 0x0600, 0x0, 0x20),
	W0(PCS, 0x0150, 0x300DE),	/* PLL off, bias off */
	DLY0(5),
	{ OP0_READ, BLK0_PCS, 0x0150, 0, 0 },
	{ OP0_POLL_CLR_STOP, BLK0_PHY, 0x0260, 2, 0 },	/* PMA PLL disengaged */
	RMW0(UDBG, 0xC700, 0x0, 0x2),	/* external PLL off: | (0x1 << 1) */
	DLY0(10),
};

#undef W0
#undef RMW0
#undef DLY0
#undef POLL0

#endif /* PCIE_ZUMAPRO_CP_SEQ_H */
