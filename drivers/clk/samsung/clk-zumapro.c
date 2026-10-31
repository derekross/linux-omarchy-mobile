// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Tensor G4 storage and staged peripheral clock paths.
 *
 * Register layout and parents derived from Google's GPL CMUCAL tables in
 * cal-if/zuma/cmucal-{sfr,node}.c, Copyright 2022 Samsung Electronics Co., Ltd.
 * Google's G4 integration also uses cal-if/zuma; the name is intentional.
 * UFS, supply GPIO, the PCIe reference path and HSI0 (USB, eUSB2, the
 * touchscreen's USI2, the GNSS SPI's USI3) are described. PLLs, muxes
 * and dividers retain firmware settings; no PLL rate table is supplied.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include <dt-bindings/clock/google,zumapro.h>

#include "clk.h"
#include "clk-exynos-arm64.h"
#include "clk-pll.h"

/* CMU_TOP, 0x26040000. PLL CON3 is the read-only rate source. */
static const struct samsung_pll_clock top_plls[] __initconst = {
	PLL(pll_0517x, CLK_FOUT_SHARED0, "zumapro_shared0", "oscclk",
	    0x004, 0x14c, NULL),
	PLL(pll_0517x, CLK_FOUT_SHARED1, "zumapro_shared1", "oscclk",
	    0x008, 0x18c, NULL),
	PLL(pll_0518x, CLK_FOUT_SHARED2, "zumapro_shared2", "oscclk",
	    0x00c, 0x1cc, NULL),
	PLL(pll_0518x, CLK_FOUT_SHARED3, "zumapro_shared3", "oscclk",
	    0x010, 0x20c, NULL),
	PLL(pll_0518x, CLK_FOUT_SPARE, "zumapro_spare", "oscclk",
	    0x014, 0x24c, NULL),
};

static const struct samsung_fixed_factor_clock top_factors[] __initconst = {
	FFACTOR(CLK_SHARED0_D2, "zumapro_shared0_d2", "zumapro_shared0", 1, 2, 0),
	FFACTOR(CLK_SHARED0_D3, "zumapro_shared0_d3", "zumapro_shared0", 1, 3, 0),
	FFACTOR(CLK_SHARED1_D2, "zumapro_shared1_d2", "zumapro_shared1", 1, 2, 0),
	FFACTOR(CLK_SHARED1_D3, "zumapro_shared1_d3", "zumapro_shared1", 1, 3, 0),
	FFACTOR(CLK_SHARED0_D4, "zumapro_shared0_d4", "zumapro_shared0", 1, 4, 0),
	FFACTOR(CLK_SHARED1_D4, "zumapro_shared1_d4", "zumapro_shared1", 1, 4, 0),
	FFACTOR(CLK_SHARED2_D2, "zumapro_shared2_d2", "zumapro_shared2", 1, 2, 0),
	FFACTOR(CLK_SHARED3_D2, "zumapro_shared3_d2", "zumapro_shared3", 1, 2, 0),
};

PNAME(hsi2_noc_parents) = {
	"zumapro_shared0_d4", "zumapro_shared1_d4", "zumapro_shared2_d2",
	"zumapro_shared3_d2", "zumapro_spare", "oscclk", "oscclk",
	"oscclk",
};

PNAME(hsi2_ufs_parents) = {
	"oscclk", "zumapro_shared0_d4", "zumapro_shared2_d2", "zumapro_spare",
};

PNAME(hsi2_pcie_parents) = { "oscclk", "zumapro_shared2_d2" };

PNAME(peric0_noc_parents) = {
	"zumapro_shared0_d4", "zumapro_shared2_d2", "zumapro_shared3_d2",
	"zumapro_spare",
};

PNAME(g3d_glb_parents) = {
	"zumapro_shared0_d2", "zumapro_shared1_d2", "zumapro_shared2",
	"zumapro_shared3", "zumapro_shared0_d3", "zumapro_shared1_d3",
	"zumapro_shared0_d4", "zumapro_spare",
};

PNAME(g3d_switch_parents) = {
	"zumapro_shared2", "zumapro_shared0_d3", "zumapro_shared3",
	"zumapro_shared1_d3", "zumapro_shared0_d4", "zumapro_shared1_d4",
	"zumapro_spare", "zumapro_spare",
};

PNAME(cis_clk_parents) = {
	"oscclk", "zumapro_shared0_d3", "zumapro_shared1_d3",
	"zumapro_shared2_d2", "zumapro_shared3_d2", "zumapro_spare",
	"oscclk", "oscclk",
};

PNAME(ispfe_noc_parents) = {
	"zumapro_shared0_d3", "zumapro_shared3", "zumapro_shared1_d3",
	"zumapro_shared0_d4", "zumapro_shared1_d4", "zumapro_shared2_d2",
	"zumapro_spare", "oscclk",
};

static const struct samsung_mux_clock top_muxes[] __initconst = {
	MUX_F(CLK_MOUT_HSI2_NOC, "zumapro_mout_hsi2_noc", hsi2_noc_parents,
	      0x10b0, 0, 3, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI2_UFS, "zumapro_mout_hsi2_ufs", hsi2_ufs_parents,
	      0x10b8, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC0_NOC, "zumapro_mout_peric0_noc", peric0_noc_parents,
	      0x10f0, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_NOC, "zumapro_mout_hsi0_noc", hsi2_noc_parents,
	      0x1094, 0, 3, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_PERI, "zumapro_mout_hsi0_peri", peric0_noc_parents,
	      0x1098, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI2_PCIE, "zumapro_mout_hsi2_pcie", hsi2_pcie_parents,
	      0x10b4, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_MISC_NOC, "zumapro_mout_misc_noc", peric0_noc_parents,
	      0x10d0, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_GLB, "zumapro_mout_g3d_glb", g3d_glb_parents,
	      0x106c, 0, 3, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_NOCD, "zumapro_mout_g3d_nocd", g3d_glb_parents,
	      0x1070, 0, 3, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_SWITCH, "zumapro_mout_g3d_switch", g3d_switch_parents,
	      0x1074, 0, 3, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI1_NOC, "zumapro_mout_hsi1_noc", hsi2_noc_parents,
	      0x10a4, 0, 3, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC1_NOC, "zumapro_mout_peric1_noc", peric0_noc_parents,
	      0x10f8, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC1_IP, "zumapro_mout_peric1_ip", peric0_noc_parents,
	      0x10f4, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC0_IP, "zumapro_mout_peric0_ip", peric0_noc_parents,
	      0x10ec, 0, 2, 0, CLK_MUX_READ_ONLY),
	/* The sensor master clocks are the kernel's, as on gs101 and gs201. */
	MUX(CLK_MOUT_CIS_CLK0, "zumapro_mout_cis_clk0", cis_clk_parents,
	    0x1010, 0, 3),
	MUX(CLK_MOUT_CIS_CLK1, "zumapro_mout_cis_clk1", cis_clk_parents,
	    0x1014, 0, 3),
	MUX(CLK_MOUT_CIS_CLK2, "zumapro_mout_cis_clk2", cis_clk_parents,
	    0x1018, 0, 3),
	MUX(CLK_MOUT_CIS_CLK3, "zumapro_mout_cis_clk3", cis_clk_parents,
	    0x101c, 0, 3),
	MUX_F(CLK_MOUT_ISPFE_NOC, "zumapro_mout_ispfe_noc", ispfe_noc_parents,
	      0x10bc, 0, 3, 0, CLK_MUX_READ_ONLY),
};

/* The hardware order is mux -> gate -> divider. */
static const struct samsung_gate_clock top_gates[] __initconst = {
	GATE(CLK_GOUT_HSI2_NOC, "zumapro_gout_hsi2_noc", "zumapro_mout_hsi2_noc",
	     0x20d8, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI2_UFS, "zumapro_gout_hsi2_ufs", "zumapro_mout_hsi2_ufs",
	     0x20e0, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_NOC, "zumapro_gout_peric0_noc",
	     "zumapro_mout_peric0_noc", 0x2114, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI2_PCIE, "zumapro_gout_hsi2_pcie", "zumapro_mout_hsi2_pcie",
	     0x20dc, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_CIS_CLK0, "zumapro_gout_cis_clk0", "zumapro_mout_cis_clk0",
	     0x2038, 21, 0, 0),
	GATE(CLK_GOUT_CIS_CLK1, "zumapro_gout_cis_clk1", "zumapro_mout_cis_clk1",
	     0x203c, 21, 0, 0),
	GATE(CLK_GOUT_CIS_CLK2, "zumapro_gout_cis_clk2", "zumapro_mout_cis_clk2",
	     0x2040, 21, 0, 0),
	GATE(CLK_GOUT_CIS_CLK3, "zumapro_gout_cis_clk3", "zumapro_mout_cis_clk3",
	     0x2044, 21, 0, 0),
};

/* Preserve the inherited MISC gate; even TOP registration must not switch it. */
static const struct samsung_gate_clock top_ro_gates[] __initconst = {
	GATE(CLK_GOUT_MISC_NOC, "zumapro_gout_misc_noc", "zumapro_mout_misc_noc",
	     0x20f4, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_NOC, "zumapro_gout_hsi0_noc", "zumapro_mout_hsi0_noc",
	     0x20bc, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_PERI, "zumapro_gout_hsi0_peri", "zumapro_mout_hsi0_peri",
	     0x20c0, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_GLB, "zumapro_gout_g3d_glb", "zumapro_mout_g3d_glb",
	     0x2094, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_NOCD, "zumapro_gout_g3d_nocd", "zumapro_mout_g3d_nocd",
	     0x2098, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_SWITCH, "zumapro_gout_g3d_switch",
	     "zumapro_mout_g3d_switch", 0x209c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI1_NOC, "zumapro_gout_hsi1_noc", "zumapro_mout_hsi1_noc",
	     0x20cc, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC1_NOC, "zumapro_gout_peric1_noc",
	     "zumapro_mout_peric1_noc", 0x211c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC1_IP, "zumapro_gout_peric1_ip", "zumapro_mout_peric1_ip",
	     0x2118, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_IP, "zumapro_gout_peric0_ip", "zumapro_mout_peric0_ip",
	     0x2110, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_ISPFE_NOC, "zumapro_gout_ispfe_noc", "zumapro_mout_ispfe_noc",
	     0x20e4, 21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_div_clock top_dividers[] __initconst = {
	DIV_F(CLK_DOUT_HSI2_NOC, "zumapro_dout_hsi2_noc", "zumapro_gout_hsi2_noc",
	      0x18a8, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_HSI2_UFS, "zumapro_dout_hsi2_ufs", "zumapro_gout_hsi2_ufs",
	      0x18b0, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC0_NOC, "zumapro_dout_peric0_noc",
	      "zumapro_gout_peric0_noc", 0x18e8, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_HSI0_NOC, "zumapro_dout_hsi0_noc", "zumapro_gout_hsi0_noc",
	      0x188c, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_HSI0_PERI, "zumapro_dout_hsi0_peri",
	      "zumapro_gout_hsi0_peri", 0x1890, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_HSI2_PCIE, "zumapro_dout_hsi2_pcie", "zumapro_gout_hsi2_pcie",
	      0x18ac, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_MISC_NOC, "zumapro_dout_misc_noc", "zumapro_gout_misc_noc",
	      0x18c4, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_G3D_GLB, "zumapro_dout_g3d_glb", "zumapro_gout_g3d_glb",
	      0x1864, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_G3D_NOCD, "zumapro_dout_g3d_nocd", "zumapro_gout_g3d_nocd",
	      0x1868, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_G3D_SWITCH, "zumapro_dout_g3d_switch",
	      "zumapro_gout_g3d_switch", 0x186c, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_HSI1_NOC, "zumapro_dout_hsi1_noc", "zumapro_gout_hsi1_noc",
	      0x189c, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC1_NOC, "zumapro_dout_peric1_noc",
	      "zumapro_gout_peric1_noc", 0x18f0, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC1_IP, "zumapro_dout_peric1_ip", "zumapro_gout_peric1_ip",
	      0x18ec, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC0_IP, "zumapro_dout_peric0_ip", "zumapro_gout_peric0_ip",
	      0x18e4, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV(CLK_DOUT_CIS_CLK0, "zumapro_dout_cis_clk0", "zumapro_gout_cis_clk0",
	    0x1810, 0, 5),
	DIV(CLK_DOUT_CIS_CLK1, "zumapro_dout_cis_clk1", "zumapro_gout_cis_clk1",
	    0x1814, 0, 5),
	DIV(CLK_DOUT_CIS_CLK2, "zumapro_dout_cis_clk2", "zumapro_gout_cis_clk2",
	    0x1818, 0, 5),
	DIV(CLK_DOUT_CIS_CLK3, "zumapro_dout_cis_clk3", "zumapro_gout_cis_clk3",
	    0x181c, 0, 5),
	DIV(CLK_DOUT_ISPFE_NOC, "zumapro_dout_ispfe_noc", "zumapro_gout_ispfe_noc",
	    0x18b4, 0, 4),
};

/*
 * Saved and restored across suspend; the gates among them are also
 * put in manual mode at probe.
 */
static const unsigned long top_regs[] __initconst = {
	0x20d8, 0x20e0, 0x2114, 0x20dc,
	/* the sensor master clocks: muxes, dividers and gates */
	0x1010, 0x1014, 0x1018, 0x101c, 0x1810, 0x1814, 0x1818, 0x181c,
	0x2038, 0x203c, 0x2040, 0x2044,
	/* the camera front end's NoC divider */
	0x18b4,
};

static const struct samsung_cmu_info top_cmu __initconst = {
	.pll_clks = top_plls,
	.nr_pll_clks = ARRAY_SIZE(top_plls),
	.fixed_factor_clks = top_factors,
	.nr_fixed_factor_clks = ARRAY_SIZE(top_factors),
	.mux_clks = top_muxes,
	.nr_mux_clks = ARRAY_SIZE(top_muxes),
	.gate_clks = top_gates,
	.nr_gate_clks = ARRAY_SIZE(top_gates),
	.ro_gate_clks = top_ro_gates,
	.nr_ro_gate_clks = ARRAY_SIZE(top_ro_gates),
	.div_clks = top_dividers,
	.nr_div_clks = ARRAY_SIZE(top_dividers),
	.nr_clk_ids = CLK_DOUT_ISPFE_NOC + 1,
	.clk_regs = top_regs,
	.nr_clk_regs = ARRAY_SIZE(top_regs),
};

static void __init zumapro_top_init(struct device_node *np)
{
	exynos_arm64_register_cmu(NULL, np, &top_cmu);
}

CLK_OF_DECLARE(zumapro_cmu_top, "google,zumapro-cmu-top", zumapro_top_init);

/* CMU_HSI2, 0x13000000. The parent bus must be enabled before CMU access. */
PNAME(hsi2_noc_user_parents) = { "oscclk", "zumapro_dout_hsi2_noc" };
PNAME(hsi2_ufs_user_parents) = { "oscclk", "zumapro_dout_hsi2_ufs" };
PNAME(hsi2_pcie_user_parents) = { "oscclk", "zumapro_dout_hsi2_pcie" };

static const struct samsung_mux_clock hsi2_muxes[] __initconst = {
	MUX_F(CLK_MOUT_HSI2_NOC_USER, "zumapro_hsi2_noc_user", hsi2_noc_user_parents,
	      0x610, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI2_UFS_USER, "zumapro_hsi2_ufs_user", hsi2_ufs_user_parents,
	      0x630, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI2_PCIE_USER, "zumapro_hsi2_pcie_user",
	      hsi2_pcie_user_parents, 0x620, 4, 1, 0, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock hsi2_dividers[] __initconst = {
	DIV_F(CLK_DOUT_HSI2_NOCP, "zumapro_hsi2_nocp", "zumapro_hsi2_noc_user",
	      0x1800, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock hsi2_gates[] __initconst = {
	GATE(CLK_GOUT_UFS_ACLK, "zumapro_ufs_aclk", "zumapro_hsi2_noc_user",
	     0x210c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_UFS_UNIPRO, "zumapro_ufs_unipro", "zumapro_hsi2_ufs_user",
	     0x2110, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_UFS_FMP, "zumapro_ufs_fmp", "zumapro_hsi2_noc_user",
	     0x2114, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_QE_UFS_ACLK, "zumapro_qe_ufs_aclk", "zumapro_hsi2_noc_user",
	     0x20cc, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_QE_UFS_PCLK, "zumapro_qe_ufs_pclk", "zumapro_hsi2_noc_user",
	     0x20d0, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_SYSREG_HSI2, "zumapro_sysreg_hsi2", "zumapro_hsi2_noc_user",
	     0x20e8, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_GPIO_HSI2UFS, "zumapro_gpio_hsi2ufs", "zumapro_hsi2_noc_user",
	     0x2008, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_GPIO_HSI2, "zumapro_gpio_hsi2", "zumapro_hsi2_noc_user",
	     0x2060, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_CTL_APB, "zumapro_pcie1_ctl_apb",
	     "zumapro_hsi2_noc_user", 0x2094, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_DBI_ACLK, "zumapro_pcie1_dbi_aclk",
	     "zumapro_hsi2_noc_user", 0x2088, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_SLV_ACLK, "zumapro_pcie1_slv_aclk",
	     "zumapro_hsi2_noc_user", 0x2090, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_MSTR_ACLK, "zumapro_pcie1_mstr_aclk",
	     "zumapro_hsi2_noc_user", 0x208c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_PHY_UDBG_APB, "zumapro_pcie1_phy_udbg_apb",
	     "zumapro_hsi2_noc_user", 0x2098, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_PHY_PAL_APB, "zumapro_pcie1_phy_pal_apb",
	     "zumapro_hsi2_noc_user", 0x209c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_PHY_REF, "zumapro_pcie1_phy_ref",
	     "zumapro_hsi2_pcie_user", 0x201c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_PHY_BRIDGE_APB, "zumapro_pcie1_phy_bridge_apb",
	     "zumapro_hsi2_nocp", 0x2000, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_PHY_TEST_APB, "zumapro_pcie1_phy_test_apb",
	     "zumapro_hsi2_nocp", 0x2020, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_IA_CLK, "zumapro_pcie1_ia_clk",
	     "zumapro_hsi2_noc_user", 0x20a4, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_SSMT_IA_ACLK, "zumapro_pcie1_ssmt_ia_aclk",
	     "zumapro_hsi2_noc_user", 0x2038, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_SSMT_IA_PCLK, "zumapro_pcie1_ssmt_ia_pclk",
	     "zumapro_hsi2_noc_user", 0x203c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_SYSMMU_CLK, "zumapro_pcie1_sysmmu_clk",
	     "zumapro_hsi2_noc_user", 0x204c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PCIE1_PMMU_CLK, "zumapro_pcie1_pmmu_clk",
	     "zumapro_hsi2_noc_user", 0x2050, 21, CLK_IGNORE_UNUSED, 0),
};

static const unsigned long hsi2_regs[] __initconst = {
	0x2008, 0x2060, 0x20cc, 0x20d0, 0x20e8, 0x210c, 0x2110, 0x2114,
	0x2094, 0x2088, 0x2090, 0x208c, 0x2098, 0x209c, 0x201c, 0x2000,
	0x2020, 0x20a4, 0x2038, 0x203c, 0x204c, 0x2050,
};

static const struct samsung_cmu_info hsi2_cmu __initconst = {
	.mux_clks = hsi2_muxes,
	.nr_mux_clks = ARRAY_SIZE(hsi2_muxes),
	.div_clks = hsi2_dividers,
	.nr_div_clks = ARRAY_SIZE(hsi2_dividers),
	.gate_clks = hsi2_gates,
	.nr_gate_clks = ARRAY_SIZE(hsi2_gates),
	.nr_clk_ids = CLK_GOUT_PCIE1_PMMU_CLK + 1,
	.clk_regs = hsi2_regs,
	.nr_clk_regs = ARRAY_SIZE(hsi2_regs),
	.clk_name = "bus",
};

/*
 * CMU_G3D. Firmware sets it up and keeps its domain on: the PLLs, muxes and
 * dividers are read-only (the PLLs use the PLL0822x M/P/S layout, with no
 * rate table), and the GPU's gates are read-only gates. The GPU's own rate
 * changes go through the ACPM firmware, not this CMU.
 */
PNAME(g3d_switch_user_parents) = { "oscclk", "zumapro_dout_g3d_switch" };
PNAME(g3d_glb_user_parents) = { "oscclk", "zumapro_dout_g3d_glb" };
PNAME(g3d_nocd_user_parents) = { "oscclk", "zumapro_dout_g3d_nocd" };
PNAME(g3d_l2_glb_parents) = { "zumapro_fout_g3d_l2", "zumapro_mout_g3d_glb_user" };
PNAME(g3d_stacks_parents) = { "zumapro_fout_g3d", "zumapro_mout_g3d_switch_user" };
PNAME(g3d_top_parents) = {
	"zumapro_mout_g3d_stacks", "zumapro_mout_g3d_nocd_user",
	"zumapro_mout_g3d_l2_glb", "oscclk",
};

PNAME(g3d_coregroup_user_parents) = { "oscclk", "zumapro_dout_g3d_l2_glb" };
PNAME(g3d_stacks_user_parents) = { "oscclk", "zumapro_dout_g3d_stacks" };
PNAME(g3d_top_user_parents) = { "oscclk", "zumapro_dout_g3d_top" };

static const struct samsung_pll_clock g3d_plls[] __initconst = {
	PLL(pll_0822x, CLK_FOUT_G3D, "zumapro_fout_g3d", "oscclk",
	    0x000, 0x10c, NULL),
	PLL(pll_0822x, CLK_FOUT_G3D_L2, "zumapro_fout_g3d_l2", "oscclk",
	    0x008, 0x18c, NULL),
};

static const struct samsung_mux_clock g3d_muxes[] __initconst = {
	MUX_F(CLK_MOUT_G3D_SWITCH_USER, "zumapro_mout_g3d_switch_user",
	      g3d_switch_user_parents, 0x620, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_GLB_USER, "zumapro_mout_g3d_glb_user",
	      g3d_glb_user_parents, 0x600, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_NOCD_USER, "zumapro_mout_g3d_nocd_user",
	      g3d_nocd_user_parents, 0x610, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_L2_GLB, "zumapro_mout_g3d_l2_glb",
	      g3d_l2_glb_parents, 0x1000, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_STACKS, "zumapro_mout_g3d_stacks",
	      g3d_stacks_parents, 0x1004, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_TOP, "zumapro_mout_g3d_top",
	      g3d_top_parents, 0x1008, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_COREGROUP_USER, "zumapro_mout_g3d_coregroup_user",
	      g3d_coregroup_user_parents, 0x630, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_STACKS_USER, "zumapro_mout_g3d_stacks_user",
	      g3d_stacks_user_parents, 0x640, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_TOP_USER, "zumapro_mout_g3d_top_user",
	      g3d_top_user_parents, 0x650, 4, 1, 0, CLK_MUX_READ_ONLY),
};

/* CAL exposes no ratio field for these two paths: they are pass-through. */
static const struct samsung_fixed_factor_clock g3d_factors[] __initconst = {
	FFACTOR(CLK_DOUT_G3D_L2_GLB, "zumapro_dout_g3d_l2_glb",
		"zumapro_mout_g3d_l2_glb", 1, 1, 0),
	FFACTOR(CLK_DOUT_G3D_STACKS, "zumapro_dout_g3d_stacks",
		"zumapro_mout_g3d_stacks", 1, 1, 0),
};

static const struct samsung_div_clock g3d_dividers[] __initconst = {
	DIV_F(CLK_DOUT_G3D_TOP, "zumapro_dout_g3d_top", "zumapro_mout_g3d_top",
	      0x1814, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_G3D_NOCP, "zumapro_dout_g3d_nocp", "zumapro_dout_g3d_l2_glb",
	      0x1808, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock g3d_ro_gates[] __initconst = {
	GATE(CLK_GOUT_G3D_CMU, "zumapro_gout_g3d_cmu", "zumapro_dout_g3d_nocp",
	     0x2018, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_CORE, "zumapro_gout_g3d_core", "zumapro_mout_g3d_top_user",
	     0x201c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_COREGROUP, "zumapro_gout_g3d_coregroup",
	     "zumapro_mout_g3d_coregroup_user", 0x2020, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_STACKS, "zumapro_gout_g3d_stacks",
	     "zumapro_mout_g3d_stacks_user", 0x2024, 21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info g3d_cmu __initconst = {
	.pll_clks = g3d_plls,
	.nr_pll_clks = ARRAY_SIZE(g3d_plls),
	.mux_clks = g3d_muxes,
	.nr_mux_clks = ARRAY_SIZE(g3d_muxes),
	.fixed_factor_clks = g3d_factors,
	.nr_fixed_factor_clks = ARRAY_SIZE(g3d_factors),
	.div_clks = g3d_dividers,
	.nr_div_clks = ARRAY_SIZE(g3d_dividers),
	.ro_gate_clks = g3d_ro_gates,
	.nr_ro_gate_clks = ARRAY_SIZE(g3d_ro_gates),
	.nr_clk_ids = CLK_GOUT_G3D_STACKS + 1,
	.clk_name = "bus",
};

/*
 * CMU_APM. Firmware sets it up and keeps it on: the muxes and dividers are
 * read-only, the gates read-only gates. Its sources are the oscillator and
 * the always-on ALV PLL's dividers. The ALV PLL runs at 786.432 MHz, 32
 * times the 24.576 MHz oscillator (the CAL's nominal 787 MHz), so its
 * dividers are given as exact fractions of that; the external pad input
 * (pad_clk_apm) is not described, as on gs101.
 */
static const struct samsung_fixed_rate_clock apm_fixed_clks[] __initconst = {
	FRATE(CLK_APM_PLL_ALV_DIV2, "zumapro_apm_pll_alv_div2", NULL, 0, 393216000),
	FRATE(CLK_APM_PLL_ALV_DIV4, "zumapro_apm_pll_alv_div4", NULL, 0, 196608000),
	FRATE(CLK_APM_PLL_ALV_DIV8, "zumapro_apm_pll_alv_div8", NULL, 0, 98304000),
	FRATE(CLK_APM_PLL_ALV_DIV16, "zumapro_apm_pll_alv_div16", NULL, 0, 49152000),
};

PNAME(mout_apm_funcsrc_parents) = {
	"zumapro_apm_pll_alv_div2", "zumapro_apm_pll_alv_div4",
	"zumapro_apm_pll_alv_div16", "zumapro_apm_pll_alv_div8",
};

PNAME(mout_apm_func_parents) = {
	"oscclk", "zumapro_mout_apm_funcsrc", "pad_clk_apm", "oscclk",
};

static const struct samsung_mux_clock apm_muxes[] __initconst = {
	MUX_F(CLK_MOUT_APM_FUNCSRC, "zumapro_mout_apm_funcsrc",
	      mout_apm_funcsrc_parents, 0x1004, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_APM_FUNC, "zumapro_mout_apm_func", mout_apm_func_parents,
	      0x1000, 0, 2, 0, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock apm_dividers[] __initconst = {
	DIV_F(CLK_DOUT_APM_NOC, "zumapro_dout_apm_noc", "zumapro_gout_apm_func",
	      0x1808, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_APM_USI1_UART, "zumapro_dout_apm_usi1_uart",
	      "zumapro_gout_apm_func", 0x1818, 0, 7, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock apm_ro_gates[] __initconst = {
	GATE(CLK_GOUT_APM_FUNC, "zumapro_gout_apm_func", "zumapro_mout_apm_func",
	     0x20d4, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_CMU, "zumapro_gout_apm_cmu", "zumapro_dout_apm_noc",
	     0x2010, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_MAILBOX_AP, "zumapro_gout_apm_mailbox_ap",
	     "zumapro_dout_apm_noc", 0x211c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_INTMEM_ACLK, "zumapro_gout_apm_intmem_aclk",
	     "zumapro_dout_apm_noc", 0x2108, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_INTMEM_PCLK, "zumapro_gout_apm_intmem_pclk",
	     "zumapro_dout_apm_noc", 0x210c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_GPIO_CUSTOM_ALIVE, "zumapro_gout_apm_gpio_custom_alive",
	     "zumapro_dout_apm_noc", 0x2000, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_GPIO_ALIVE, "zumapro_gout_apm_gpio_alive",
	     "zumapro_dout_apm_noc", 0x20d8, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_GPIO_FAR_ALIVE, "zumapro_gout_apm_gpio_far_alive",
	     "zumapro_dout_apm_noc", 0x20dc, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_USI1_UART_IPCLK, "zumapro_gout_apm_usi1_uart_ipclk",
	     "zumapro_dout_apm_usi1_uart", 0x20f4, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_USI1_UART_PCLK, "zumapro_gout_apm_usi1_uart_pclk",
	     "zumapro_dout_apm_noc", 0x20f8, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_USI1_UART_RSTNSYNC, "zumapro_gout_apm_usi1_uart_rstnsync",
	     "zumapro_dout_apm_usi1_uart", 0x2148, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_SYSREG, "zumapro_gout_apm_sysreg", "zumapro_dout_apm_noc",
	     0x2168, 21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info apm_cmu __initconst = {
	.fixed_clks = apm_fixed_clks,
	.nr_fixed_clks = ARRAY_SIZE(apm_fixed_clks),
	.mux_clks = apm_muxes,
	.nr_mux_clks = ARRAY_SIZE(apm_muxes),
	.div_clks = apm_dividers,
	.nr_div_clks = ARRAY_SIZE(apm_dividers),
	.ro_gate_clks = apm_ro_gates,
	.nr_ro_gate_clks = ARRAY_SIZE(apm_ro_gates),
	.nr_clk_ids = CLK_GOUT_APM_SYSREG + 1,
};

/*
 * CMU_HSI1. Firmware keeps the domain powered and owns its setup: the muxes
 * and the ALT divider are read-only, the gates read-only gates. Only the
 * GPIO bank's clock is described; the PCIe channel 0 clocks are not.
 */
PNAME(mout_hsi1_noc_user_parents) = { "oscclk", "zumapro_dout_hsi1_noc" };
PNAME(mout_hsi1_alt_user_parents) = { "oscclk", "zumapro_dout_hsi1_alt" };
PNAME(mout_hsi1_noc_local_parents) = { "zumapro_mout_hsi1_noc_user",
				       "zumapro_mout_hsi1_alt_user" };

static const struct samsung_mux_clock hsi1_muxes[] __initconst = {
	MUX_F(CLK_MOUT_HSI1_NOC_USER, "zumapro_mout_hsi1_noc_user",
	      mout_hsi1_noc_user_parents, 0x600, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI1_ALT_USER, "zumapro_mout_hsi1_alt_user",
	      mout_hsi1_alt_user_parents, 0x620, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI1_NOC_LOCAL, "zumapro_mout_hsi1_noc_local",
	      mout_hsi1_noc_local_parents, 0x1000, 0, 1, 0, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock hsi1_dividers[] __initconst = {
	DIV_F(CLK_DOUT_HSI1_ALT, "zumapro_dout_hsi1_alt", "zumapro_gout_hsi1_alt",
	      0x1808, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock hsi1_ro_gates[] __initconst = {
	GATE(CLK_GOUT_HSI1_ALT, "zumapro_gout_hsi1_alt", "hsi1-alt",
	     0x2080, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_GPIO_HSI1, "zumapro_gout_gpio_hsi1", "zumapro_mout_hsi1_noc_local",
	     0x205c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI1_CMU, "zumapro_gout_hsi1_cmu", "zumapro_mout_hsi1_noc_local",
	     0x2004, 21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info hsi1_cmu __initconst = {
	.mux_clks = hsi1_muxes,
	.nr_mux_clks = ARRAY_SIZE(hsi1_muxes),
	.div_clks = hsi1_dividers,
	.nr_div_clks = ARRAY_SIZE(hsi1_dividers),
	.ro_gate_clks = hsi1_ro_gates,
	.nr_ro_gate_clks = ARRAY_SIZE(hsi1_ro_gates),
	.nr_clk_ids = CLK_GOUT_HSI1_CMU + 1,
	.clk_name = "bus",
};

/*
 * CMU_PERIC1. Firmware sets it up: the user muxes and the USI9 divider are
 * read-only, and the GPIO, sysreg and USI9 gates are read-only gates. USI9
 * is the I2C bus of the NFC controller and the haptics driver.
 */
PNAME(mout_peric1_noc_user_parents) = { "oscclk", "zumapro_dout_peric1_noc" };
PNAME(mout_peric1_usi9_user_parents) = { "oscclk", "zumapro_dout_peric1_ip" };
PNAME(mout_peric1_usi10_user_parents) = { "oscclk", "zumapro_dout_peric1_ip" };

static const struct samsung_mux_clock peric1_muxes[] __initconst = {
	MUX_F(CLK_MOUT_PERIC1_NOC_USER, "zumapro_mout_peric1_noc_user",
	      mout_peric1_noc_user_parents, 0x610, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC1_USI9_USER, "zumapro_mout_peric1_usi9_user",
	      mout_peric1_usi9_user_parents, 0x690, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC1_USI10_USER, "zumapro_mout_peric1_usi10_user",
	      mout_peric1_usi10_user_parents, 0x630, 4, 1, 0, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock peric1_dividers[] __initconst = {
	DIV_F(CLK_DOUT_PERIC1_USI9, "zumapro_dout_peric1_usi9",
	      "zumapro_mout_peric1_usi9_user", 0x1824, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC1_USI10, "zumapro_dout_peric1_usi10",
	      "zumapro_mout_peric1_usi10_user", 0x180c, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock peric1_ro_gates[] __initconst = {
	GATE(CLK_GOUT_GPIO_PERIC1, "zumapro_gout_gpio_peric1",
	     "zumapro_mout_peric1_noc_user", 0x2080, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC1_SYSREG, "zumapro_gout_peric1_sysreg",
	     "zumapro_mout_peric1_noc_user", 0x20a4, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC1_USI9_IPCLK, "zumapro_gout_peric1_usi9_ipclk",
	     "zumapro_dout_peric1_usi9", 0x2070, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC1_USI9_PCLK, "zumapro_gout_peric1_usi9_pclk",
	     "zumapro_mout_peric1_noc_user", 0x2074, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC1_USI9_RSTNSYNC, "zumapro_gout_peric1_usi9_rstnsync",
	     "zumapro_dout_peric1_usi9", 0x20a0, 21, CLK_IGNORE_UNUSED, 0),
};

/* USI10, the I2C bus of the Pixel 9a's OTG boost: gates Linux turns on */
static const struct samsung_gate_clock peric1_gates[] __initconst = {
	GATE(CLK_GOUT_PERIC1_USI10_IPCLK, "zumapro_gout_peric1_usi10_ipclk",
	     "zumapro_dout_peric1_usi10", 0x2040, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI10_PCLK, "zumapro_gout_peric1_usi10_pclk",
	     "zumapro_mout_peric1_noc_user", 0x2044, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI10_RSTNSYNC, "zumapro_gout_peric1_usi10_rstnsync",
	     "zumapro_dout_peric1_usi10", 0x2090, 21, 0, 0),
};

static const struct samsung_cmu_info peric1_cmu __initconst = {
	.mux_clks = peric1_muxes,
	.nr_mux_clks = ARRAY_SIZE(peric1_muxes),
	.div_clks = peric1_dividers,
	.nr_div_clks = ARRAY_SIZE(peric1_dividers),
	.ro_gate_clks = peric1_ro_gates,
	.nr_ro_gate_clks = ARRAY_SIZE(peric1_ro_gates),
	.gate_clks = peric1_gates,
	.nr_gate_clks = ARRAY_SIZE(peric1_gates),
	.nr_clk_ids = CLK_GOUT_PERIC1_USI10_RSTNSYNC + 1,
	.clk_name = "bus",
};

/* CMU_HSI0. Firmware keeps the domain powered; there is no local PM. */
PNAME(mout_hsi0_tcxo_user_parents) = { "oscclk", "hsi0-tcxo" };
PNAME(mout_hsi0_usb20_ref_parents) = { "zumapro_dout_hsi0_usb", "zumapro_mout_hsi0_tcxo_user" };
PNAME(mout_hsi0_noc_user_parents) = { "oscclk", "zumapro_dout_hsi0_noc" };
PNAME(mout_hsi0_alt_user_parents) = { "oscclk", "zumapro_gout_hsi0_alt" };
PNAME(mout_hsi0_noc_local_parents) = { "zumapro_mout_hsi0_noc_user", "zumapro_mout_hsi0_alt_user" };
PNAME(mout_hsi0_peri_user_parents) = { "oscclk", "zumapro_dout_hsi0_peri" };
PNAME(mout_hsi0_usi2_local_parents) = { "zumapro_mout_hsi0_peri_user", "oscclk" };
PNAME(mout_hsi0_usi3_local_parents) = { "zumapro_mout_hsi0_peri_user", "oscclk" };
PNAME(mout_hsi0_usi4_local_parents) = { "zumapro_mout_hsi0_peri_user", "oscclk" };

static const struct samsung_pll_clock hsi0_plls[] __initconst = {
	PLL(pll_0822x, CLK_FOUT_HSI0_USB, "zumapro_fout_hsi0_usb", "oscclk",
	    0x000, 0x10c, NULL),
};

static const struct samsung_mux_clock hsi0_muxes[] __initconst = {
	MUX_F(CLK_MOUT_HSI0_TCXO_USER, "zumapro_mout_hsi0_tcxo_user", mout_hsi0_tcxo_user_parents,
	      0x630, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_USB20_REF, "zumapro_mout_hsi0_usb20_ref", mout_hsi0_usb20_ref_parents,
	      0x1004, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_NOC_USER, "zumapro_mout_hsi0_noc_user", mout_hsi0_noc_user_parents,
	      0x620, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_ALT_USER, "zumapro_mout_hsi0_alt_user", mout_hsi0_alt_user_parents,
	      0x600, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_NOC_LOCAL, "zumapro_mout_hsi0_noc_local", mout_hsi0_noc_local_parents,
	      0x1000, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_PERI_USER, "zumapro_mout_hsi0_peri_user", mout_hsi0_peri_user_parents,
	      0x680, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_USI2_LOCAL, "zumapro_mout_hsi0_usi2_local",
	      mout_hsi0_usi2_local_parents,
	      0x101c, 0, 1, 0, CLK_MUX_READ_ONLY),
	nMUX(CLK_MOUT_HSI0_USI3_LOCAL, "zumapro_mout_hsi0_usi3_local",
	     mout_hsi0_usi3_local_parents, 0x1020, 0, 1),
	MUX_F(CLK_MOUT_HSI0_USI4_LOCAL, "zumapro_mout_hsi0_usi4_local",
	      mout_hsi0_usi4_local_parents,
	      0x1024, 0, 1, 0, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock hsi0_dividers[] __initconst = {
	DIV_F(CLK_DOUT_HSI0_USB, "zumapro_dout_hsi0_usb", "zumapro_fout_hsi0_usb",
	      0x1804, 0, 6, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_HSI0_EUSB, "zumapro_dout_hsi0_eusb", "zumapro_mout_hsi0_noc_local",
	      0x180c, 0, 2, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_HSI0_USI2, "zumapro_dout_hsi0_usi2", "zumapro_gout_hsi0_usi2_source",
	      0x181c, 0, 4, 0, 0),
	DIV_F(CLK_DOUT_HSI0_USI3, "zumapro_dout_hsi0_usi3", "zumapro_gout_hsi0_usi3_source",
	      0x1820, 0, 4, CLK_SET_RATE_PARENT, 0),
	DIV_F(CLK_DOUT_HSI0_USI4, "zumapro_dout_hsi0_usi4", "zumapro_gout_hsi0_usi4_source",
	      0x1824, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock hsi0_gates[] __initconst = {
	GATE(CLK_GOUT_HSI0_USI2_SOURCE, "zumapro_gout_hsi0_usi2_source",
	     "zumapro_mout_hsi0_usi2_local",
	     0x2120, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_USI2_IP, "zumapro_gout_hsi0_usi2_ip", "zumapro_dout_hsi0_usi2",
	     0x20fc, 21, CLK_IGNORE_UNUSED | CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_HSI0_USI2_PCLK, "zumapro_gout_hsi0_usi2_pclk", "zumapro_mout_hsi0_noc_local",
	     0x2100, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_USI3_SOURCE, "zumapro_gout_hsi0_usi3_source",
	     "zumapro_mout_hsi0_usi3_local",
	     0x2124, 21, CLK_IGNORE_UNUSED | CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_HSI0_USI3_IP, "zumapro_gout_hsi0_usi3_ip", "zumapro_dout_hsi0_usi3",
	     0x2104, 21, CLK_IGNORE_UNUSED | CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_HSI0_USI3_PCLK, "zumapro_gout_hsi0_usi3_pclk", "zumapro_mout_hsi0_noc_local",
	     0x2108, 21, CLK_IGNORE_UNUSED, 0),
};

/*
 * Firmware-owned gates are observed, never switched. A zero CG_VAL is an
 * unavailable clock, not permission to force it on. Runtime auto-gating
 * and power-off remain outside this firmware-on handoff.
 */
static const struct samsung_gate_clock hsi0_ro_gates[] __initconst = {
	GATE(CLK_GOUT_HSI0_USI4_SOURCE, "zumapro_gout_hsi0_usi4_source",
	     "zumapro_mout_hsi0_usi4_local",
	     0x2128, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_USI4_IP, "zumapro_gout_hsi0_usi4_ip", "zumapro_dout_hsi0_usi4",
	     0x210c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_USI4_PCLK, "zumapro_gout_hsi0_usi4_pclk",
	     "zumapro_mout_hsi0_noc_local",
	     0x2110, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_ALT, "zumapro_gout_hsi0_alt", "hsi0-alt",
	     0x204c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_EUSB_REF, "zumapro_gout_hsi0_eusb_ref", "zumapro_mout_hsi0_usb20_ref",
	     0x20dc, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_EUSB_BUS, "zumapro_gout_hsi0_eusb_bus", "zumapro_dout_hsi0_eusb",
	     0x20d8, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_EUSB_CTRL, "zumapro_gout_hsi0_eusb_ctrl", "zumapro_mout_hsi0_noc_local",
	     0x20b8, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_SYSREG, "zumapro_gout_hsi0_sysreg", "zumapro_mout_hsi0_noc_local",
	     0x208c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_USB_SUBCTL, "zumapro_gout_hsi0_usb_subctl",
	     "zumapro_mout_hsi0_noc_local",
	     0x20a0, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_USB_LINK, "zumapro_gout_hsi0_usb_link", "zumapro_mout_hsi0_noc_local",
	     0x20a4, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_USB_SUSPEND, "zumapro_gout_hsi0_usb_suspend",
	     "zumapro_mout_hsi0_usb20_ref",
	     0x20e4, 21, CLK_IGNORE_UNUSED, 0),
};

/* USI2 (SPI20, touchscreen) and USI3 (SPI21, GNSS) run in manual mode. */
static const unsigned long hsi0_regs[] __initconst = {
	0x2120, 0x20fc, 0x2100, 0x2124, 0x2104, 0x2108,
};

static const struct samsung_cmu_info hsi0_cmu __initconst = {
	.pll_clks = hsi0_plls,
	.nr_pll_clks = ARRAY_SIZE(hsi0_plls),
	.mux_clks = hsi0_muxes,
	.nr_mux_clks = ARRAY_SIZE(hsi0_muxes),
	.div_clks = hsi0_dividers,
	.nr_div_clks = ARRAY_SIZE(hsi0_dividers),
	.gate_clks = hsi0_gates,
	.nr_gate_clks = ARRAY_SIZE(hsi0_gates),
	.ro_gate_clks = hsi0_ro_gates,
	.nr_ro_gate_clks = ARRAY_SIZE(hsi0_ro_gates),
	.nr_clk_ids = CLK_GOUT_HSI0_USI4_PCLK + 1,
	.clk_regs = hsi0_regs,
	.nr_clk_regs = ARRAY_SIZE(hsi0_regs),
	.clk_name = "bus",
};

/*
 * CMU_PERIC0, 0x10800000. Only the UFS VCC GPIO clock is controlled. USI2
 * to USI6 (the cameras', the camera power chips' and the battery's I2C
 * buses) are firmware-owned: their user muxes and dividers are read-only,
 * their gates read-only gates, fed by CMU_TOP's PERIC0 IP clock.
 */
PNAME(peric0_noc_user_parents) = { "oscclk", "zumapro_dout_peric0_noc" };
PNAME(mout_peric0_usi_user_parents) = { "oscclk", "zumapro_dout_peric0_ip" };

static const struct samsung_mux_clock peric0_muxes[] __initconst = {
	MUX_F(CLK_MOUT_PERIC0_NOC_USER, "zumapro_peric0_noc_user",
	      peric0_noc_user_parents, 0x600, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC0_USI2_USER, "zumapro_mout_peric0_usi2_user",
	      mout_peric0_usi_user_parents, 0x660, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC0_USI3_USER, "zumapro_mout_peric0_usi3_user",
	      mout_peric0_usi_user_parents, 0x670, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC0_USI4_USER, "zumapro_mout_peric0_usi4_user",
	      mout_peric0_usi_user_parents, 0x680, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC0_USI5_USER, "zumapro_mout_peric0_usi5_user",
	      mout_peric0_usi_user_parents, 0x690, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC0_USI6_USER, "zumapro_mout_peric0_usi6_user",
	      mout_peric0_usi_user_parents, 0x6a0, 4, 1, 0, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock peric0_dividers[] __initconst = {
	DIV_F(CLK_DOUT_PERIC0_USI2, "zumapro_dout_peric0_usi2",
	      "zumapro_mout_peric0_usi2_user", 0x1814, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC0_USI3, "zumapro_dout_peric0_usi3",
	      "zumapro_mout_peric0_usi3_user", 0x1818, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC0_USI4, "zumapro_dout_peric0_usi4",
	      "zumapro_mout_peric0_usi4_user", 0x181c, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC0_USI5, "zumapro_dout_peric0_usi5",
	      "zumapro_mout_peric0_usi5_user", 0x1820, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC0_USI6, "zumapro_dout_peric0_usi6",
	      "zumapro_mout_peric0_usi6_user", 0x1824, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock peric0_gates[] __initconst = {
	GATE(CLK_GOUT_GPIO_PERIC0, "zumapro_gpio_peric0", "zumapro_peric0_noc_user",
	     0x20b8, 21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_gate_clock peric0_ro_gates[] __initconst = {
	GATE(CLK_GOUT_PERIC0_USI2_IPCLK, "zumapro_gout_peric0_usi2_ipclk",
	     "zumapro_dout_peric0_usi2", 0x2078, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI2_PCLK, "zumapro_gout_peric0_usi2_pclk",
	     "zumapro_peric0_noc_user", 0x207c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI2_RSTNSYNC, "zumapro_gout_peric0_usi2_rstnsync",
	     "zumapro_dout_peric0_usi2", 0x20d4, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI3_IPCLK, "zumapro_gout_peric0_usi3_ipclk",
	     "zumapro_dout_peric0_usi3", 0x2080, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI3_PCLK, "zumapro_gout_peric0_usi3_pclk",
	     "zumapro_peric0_noc_user", 0x2084, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI3_RSTNSYNC, "zumapro_gout_peric0_usi3_rstnsync",
	     "zumapro_dout_peric0_usi3", 0x20d8, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI4_IPCLK, "zumapro_gout_peric0_usi4_ipclk",
	     "zumapro_dout_peric0_usi4", 0x2088, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI4_PCLK, "zumapro_gout_peric0_usi4_pclk",
	     "zumapro_peric0_noc_user", 0x208c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI4_RSTNSYNC, "zumapro_gout_peric0_usi4_rstnsync",
	     "zumapro_dout_peric0_usi4", 0x20dc, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI5_IPCLK, "zumapro_gout_peric0_usi5_ipclk",
	     "zumapro_dout_peric0_usi5", 0x2090, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI5_PCLK, "zumapro_gout_peric0_usi5_pclk",
	     "zumapro_peric0_noc_user", 0x2094, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI5_RSTNSYNC, "zumapro_gout_peric0_usi5_rstnsync",
	     "zumapro_dout_peric0_usi5", 0x20e0, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI6_IPCLK, "zumapro_gout_peric0_usi6_ipclk",
	     "zumapro_dout_peric0_usi6", 0x2098, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI6_PCLK, "zumapro_gout_peric0_usi6_pclk",
	     "zumapro_peric0_noc_user", 0x209c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_USI6_RSTNSYNC, "zumapro_gout_peric0_usi6_rstnsync",
	     "zumapro_dout_peric0_usi6", 0x20e4, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_SYSREG, "zumapro_gout_peric0_sysreg",
	     "zumapro_peric0_noc_user", 0x20f0, 21, CLK_IGNORE_UNUSED, 0),
};

static const unsigned long peric0_regs[] __initconst = { 0x20b8 };

static const struct samsung_cmu_info peric0_cmu __initconst = {
	.mux_clks = peric0_muxes,
	.nr_mux_clks = ARRAY_SIZE(peric0_muxes),
	.div_clks = peric0_dividers,
	.nr_div_clks = ARRAY_SIZE(peric0_dividers),
	.gate_clks = peric0_gates,
	.nr_gate_clks = ARRAY_SIZE(peric0_gates),
	.ro_gate_clks = peric0_ro_gates,
	.nr_ro_gate_clks = ARRAY_SIZE(peric0_ro_gates),
	.nr_clk_ids = CLK_GOUT_PERIC0_SYSREG + 1,
	.clk_regs = peric0_regs,
	.nr_clk_regs = ARRAY_SIZE(peric0_regs),
	.clk_name = "bus",
};

/* CMU_MISC, 0x10010000. Read-only firmware-on watchdog bus paths. */
PNAME(misc_noc_user_parents) = { "oscclk", "zumapro_dout_misc_noc" };

static const struct samsung_mux_clock misc_muxes[] __initconst = {
	MUX_F(CLK_MOUT_MISC_NOC_USER, "zumapro_misc_noc_user", misc_noc_user_parents,
	      0x600, 4, 1, 0, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock misc_dividers[] __initconst = {
	DIV_F(CLK_DOUT_MISC_NOCP, "zumapro_misc_nocp", "zumapro_misc_noc_user",
	      0x1808, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock misc_ro_gates[] __initconst = {
	GATE(CLK_GOUT_WDT_CLUSTER0, "zumapro_wdt_cluster0", "zumapro_misc_nocp",
	     0x2164, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_WDT_CLUSTER1, "zumapro_wdt_cluster1", "zumapro_misc_nocp",
	     0x2168, 21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_cmu_info misc_cmu __initconst = {
	.mux_clks = misc_muxes,
	.nr_mux_clks = ARRAY_SIZE(misc_muxes),
	.div_clks = misc_dividers,
	.nr_div_clks = ARRAY_SIZE(misc_dividers),
	.ro_gate_clks = misc_ro_gates,
	.nr_ro_gate_clks = ARRAY_SIZE(misc_ro_gates),
	.nr_clk_ids = CLK_GOUT_WDT_CLUSTER1 + 1,
};

static int __init zumapro_cmu_probe(struct platform_device *pdev)
{
	const struct samsung_cmu_info *cmu = device_get_match_data(&pdev->dev);

	exynos_arm64_register_cmu(&pdev->dev, pdev->dev.of_node, cmu);
	return 0;
}

static const struct of_device_id zumapro_cmu_match[] = {
	{ .compatible = "google,zumapro-cmu-apm", .data = &apm_cmu },
	{ .compatible = "google,zumapro-cmu-g3d", .data = &g3d_cmu },
	{ .compatible = "google,zumapro-cmu-hsi0", .data = &hsi0_cmu },
	{ .compatible = "google,zumapro-cmu-hsi1", .data = &hsi1_cmu },
	{ .compatible = "google,zumapro-cmu-hsi2", .data = &hsi2_cmu },
	{ .compatible = "google,zumapro-cmu-peric0", .data = &peric0_cmu },
	{ .compatible = "google,zumapro-cmu-peric1", .data = &peric1_cmu },
	{ }
};

static struct platform_driver zumapro_cmu_driver __refdata = {
	.driver = {
		.name = "zumapro-cmu",
		.of_match_table = zumapro_cmu_match,
		.suppress_bind_attrs = true,
	},
	.probe = zumapro_cmu_probe,
};

static int __init zumapro_cmu_init(void)
{
	return platform_driver_register(&zumapro_cmu_driver);
}
core_initcall(zumapro_cmu_init);

static int __init zumapro_misc_probe(struct platform_device *pdev)
{
	struct clk *bus;

	/* The legacy helper continues after a bus error; do not use it. */
	bus = devm_clk_get_enabled(&pdev->dev, "bus");
	if (IS_ERR(bus))
		return dev_err_probe(&pdev->dev, PTR_ERR(bus),
				     "cannot retain MISC bus clock\n");

	/* No manual setup or save/restore list for firmware-owned clocks. */
	if (!samsung_cmu_register_one(pdev->dev.of_node, &misc_cmu))
		return -ENODEV;

	return 0;
}

static const struct of_device_id zumapro_misc_match[] = {
	{ .compatible = "google,zumapro-cmu-misc" },
	{ }
};

static struct platform_driver zumapro_misc_driver = {
	.driver = {
		.name = "zumapro-cmu-misc",
		.of_match_table = zumapro_misc_match,
		.suppress_bind_attrs = true,
	},
};

static int __init zumapro_misc_init(void)
{
	/* Init-only clock tables must not be used by a later deferred probe. */
	return platform_driver_probe(&zumapro_misc_driver, zumapro_misc_probe);
}
subsys_initcall(zumapro_misc_init);
