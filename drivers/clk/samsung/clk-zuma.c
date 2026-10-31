// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Tensor G3 (zuma) clocks for the touchscreen, its PMIC, the battery
 * bus and the GPU.
 *
 * Register layout and parents derived from Google's GPL CMUCAL tables in
 * cal-if/zuma/cmucal-{sfr,node}.c, Copyright 2022 Samsung Electronics Co., Ltd.
 * Only the paths from CMU_TOP to HSI0's USI2 and to CMU_G3D are described,
 * with CMU_APM's clocks for the APM mailbox, its SRAM and the always-on GPIO
 * banks, and the path from CMU_TOP to PERIC1's USI11 (the battery, charger
 * and USB-C I2C bus). PLLs, muxes and dividers keep their firmware settings,
 * except USI2's own divider, which sets the SPI rate. No PLL rate table is
 * supplied.
 */

#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include <dt-bindings/clock/google,zuma.h>

#include "clk.h"
#include "clk-exynos-arm64.h"
#include "clk-pll.h"

/* CMU_TOP, 0x26040000. PLL CON3 is the read-only rate source. */
static const struct samsung_pll_clock top_plls[] __initconst = {
	PLL(pll_0517x, CLK_FOUT_SHARED0, "zuma_shared0", "oscclk",
	    0x004, 0x14c, NULL),
	PLL(pll_0517x, CLK_FOUT_SHARED1, "zuma_shared1", "oscclk",
	    0x008, 0x18c, NULL),
	PLL(pll_0518x, CLK_FOUT_SHARED2, "zuma_shared2", "oscclk",
	    0x00c, 0x1cc, NULL),
	PLL(pll_0518x, CLK_FOUT_SHARED3, "zuma_shared3", "oscclk",
	    0x010, 0x20c, NULL),
	PLL(pll_0518x, CLK_FOUT_SPARE, "zuma_spare", "oscclk",
	    0x014, 0x24c, NULL),
};

static const struct samsung_fixed_factor_clock top_factors[] __initconst = {
	FFACTOR(CLK_SHARED0_D4, "zuma_shared0_d4", "zuma_shared0", 1, 4, 0),
	FFACTOR(CLK_SHARED1_D4, "zuma_shared1_d4", "zuma_shared1", 1, 4, 0),
	FFACTOR(CLK_SHARED2_D2, "zuma_shared2_d2", "zuma_shared2", 1, 2, 0),
	FFACTOR(CLK_SHARED3_D2, "zuma_shared3_d2", "zuma_shared3", 1, 2, 0),
	FFACTOR(CLK_SHARED0_D2, "zuma_shared0_d2", "zuma_shared0", 1, 2, 0),
	FFACTOR(CLK_SHARED0_D3, "zuma_shared0_d3", "zuma_shared0", 1, 3, 0),
	FFACTOR(CLK_SHARED1_D2, "zuma_shared1_d2", "zuma_shared1", 1, 2, 0),
	FFACTOR(CLK_SHARED1_D3, "zuma_shared1_d3", "zuma_shared1", 1, 3, 0),
};

PNAME(mout_peric1_parents) = {
	"zuma_shared0_d4", "zuma_shared2_d2", "zuma_shared3_d2", "zuma_spare",
};
PNAME(mout_hsi0_noc_parents) = {
	"zuma_shared0_d4", "zuma_shared1_d4", "zuma_shared2_d2",
	"zuma_shared3_d2", "zuma_spare", "oscclk", "oscclk", "oscclk",
};

PNAME(mout_hsi0_peri_parents) = {
	"zuma_shared0_d4", "zuma_shared2_d2", "zuma_shared3_d2", "zuma_spare",
};

/* NOCD takes the same sources as GLB. */
PNAME(mout_g3d_glb_parents) = {
	"zuma_shared0_d2", "zuma_shared1_d2", "zuma_shared2", "zuma_shared3",
	"zuma_shared0_d3", "zuma_shared1_d3", "zuma_shared0_d4", "zuma_spare",
};

PNAME(mout_g3d_switch_parents) = {
	"zuma_shared2", "zuma_shared0_d3", "zuma_shared3", "zuma_shared1_d3",
	"zuma_shared0_d4", "zuma_shared1_d4", "zuma_spare", "zuma_spare",
};

static const struct samsung_mux_clock top_muxes[] __initconst = {
	MUX_F(CLK_MOUT_HSI0_NOC, "zuma_mout_hsi0_noc", mout_hsi0_noc_parents,
	      0x1094, 0, 3, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_PERI, "zuma_mout_hsi0_peri", mout_hsi0_peri_parents,
	      0x1098, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC1_NOC, "zuma_mout_peric1_noc", mout_peric1_parents,
	      0x10f8, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC1_IP, "zuma_mout_peric1_ip", mout_peric1_parents,
	      0x10f4, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC0_NOC, "zuma_mout_peric0_noc", mout_peric1_parents,
	      0x10f0, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC0_IP, "zuma_mout_peric0_ip", mout_peric1_parents,
	      0x10ec, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_GLB, "zuma_mout_g3d_glb", mout_g3d_glb_parents,
	      0x106c, 0, 3, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_NOCD, "zuma_mout_g3d_nocd", mout_g3d_glb_parents,
	      0x1070, 0, 3, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_SWITCH, "zuma_mout_g3d_switch",
	      mout_g3d_switch_parents, 0x1074, 0, 3, 0, CLK_MUX_READ_ONLY),
};

/*
 * The hardware order is mux -> gate -> divider. The HSI0, PERIC0, PERIC1 and
 * G3D paths are firmware-owned.
 */
static const struct samsung_gate_clock top_ro_gates[] __initconst = {
	GATE(CLK_GOUT_HSI0_NOC, "zuma_gout_hsi0_noc", "zuma_mout_hsi0_noc",
	     0x20bc, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_PERI, "zuma_gout_hsi0_peri", "zuma_mout_hsi0_peri",
	     0x20c0, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC1_NOC, "zuma_gout_peric1_noc", "zuma_mout_peric1_noc",
	     0x211c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC1_IP, "zuma_gout_peric1_ip", "zuma_mout_peric1_ip",
	     0x2118, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_NOC, "zuma_gout_peric0_noc", "zuma_mout_peric0_noc",
	     0x2114, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_IP, "zuma_gout_peric0_ip", "zuma_mout_peric0_ip",
	     0x2110, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_GLB, "zuma_gout_g3d_glb", "zuma_mout_g3d_glb",
	     0x2094, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_NOCD, "zuma_gout_g3d_nocd", "zuma_mout_g3d_nocd",
	     0x2098, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_SWITCH, "zuma_gout_g3d_switch", "zuma_mout_g3d_switch",
	     0x209c, 21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_div_clock top_dividers[] __initconst = {
	DIV_F(CLK_DOUT_HSI0_NOC, "zuma_dout_hsi0_noc", "zuma_gout_hsi0_noc",
	      0x188c, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_HSI0_PERI, "zuma_dout_hsi0_peri", "zuma_gout_hsi0_peri",
	      0x1890, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC1_NOC, "zuma_dout_peric1_noc", "zuma_gout_peric1_noc",
	      0x18f0, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC1_IP, "zuma_dout_peric1_ip", "zuma_gout_peric1_ip",
	      0x18ec, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC0_NOC, "zuma_dout_peric0_noc", "zuma_gout_peric0_noc",
	      0x18e8, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC0_IP, "zuma_dout_peric0_ip", "zuma_gout_peric0_ip",
	      0x18e4, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_G3D_GLB, "zuma_dout_g3d_glb", "zuma_gout_g3d_glb",
	      0x1864, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_G3D_NOCD, "zuma_dout_g3d_nocd", "zuma_gout_g3d_nocd",
	      0x1868, 0, 4, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_G3D_SWITCH, "zuma_dout_g3d_switch", "zuma_gout_g3d_switch",
	      0x186c, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_cmu_info top_cmu __initconst = {
	.pll_clks = top_plls,
	.nr_pll_clks = ARRAY_SIZE(top_plls),
	.fixed_factor_clks = top_factors,
	.nr_fixed_factor_clks = ARRAY_SIZE(top_factors),
	.mux_clks = top_muxes,
	.nr_mux_clks = ARRAY_SIZE(top_muxes),
	.ro_gate_clks = top_ro_gates,
	.nr_ro_gate_clks = ARRAY_SIZE(top_ro_gates),
	.div_clks = top_dividers,
	.nr_div_clks = ARRAY_SIZE(top_dividers),
	.nr_clk_ids = CLK_DOUT_G3D_SWITCH + 1,
};

static void __init zuma_top_init(struct device_node *np)
{
	exynos_arm64_register_cmu(NULL, np, &top_cmu);
}

CLK_OF_DECLARE(zuma_cmu_top, "google,zuma-cmu-top", zuma_top_init);

/*
 * CMU_APM, 0x15400000. The always-on domain is firmware-owned, so it is
 * read-only. ALV runs at the oscillator times 32.
 */
static const struct samsung_fixed_rate_clock apm_fixed_clks[] __initconst = {
	FRATE(CLK_APM_PLL_ALV_DIV2, "zuma_apm_pll_alv_div2", NULL, 0, 393216000),
	FRATE(CLK_APM_PLL_ALV_DIV4, "zuma_apm_pll_alv_div4", NULL, 0, 196608000),
	FRATE(CLK_APM_PLL_ALV_DIV8, "zuma_apm_pll_alv_div8", NULL, 0, 98304000),
	FRATE(CLK_APM_PLL_ALV_DIV16, "zuma_apm_pll_alv_div16", NULL, 0, 49152000),
};

PNAME(mout_apm_funcsrc_parents) = {
	"zuma_apm_pll_alv_div2", "zuma_apm_pll_alv_div4",
	"zuma_apm_pll_alv_div16", "zuma_apm_pll_alv_div8",
};

PNAME(mout_apm_func_parents) = {
	"oscclk", "zuma_mout_apm_funcsrc", "pad_clk_apm", "oscclk",
};

static const struct samsung_mux_clock apm_muxes[] __initconst = {
	MUX_F(CLK_MOUT_APM_FUNCSRC, "zuma_mout_apm_funcsrc",
	      mout_apm_funcsrc_parents, 0x1004, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_APM_FUNC, "zuma_mout_apm_func", mout_apm_func_parents,
	      0x1000, 0, 2, 0, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock apm_dividers[] __initconst = {
	DIV_F(CLK_DOUT_APM_NOC, "zuma_dout_apm_noc", "zuma_gout_apm_func",
	      0x1808, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_APM_USI1_UART, "zuma_dout_apm_usi1_uart",
	      "zuma_gout_apm_func", 0x1818, 0, 7, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock apm_ro_gates[] __initconst = {
	GATE(CLK_GOUT_APM_FUNC, "zuma_gout_apm_func", "zuma_mout_apm_func",
	     0x20d4, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_MAILBOX_AP, "zuma_gout_apm_mailbox_ap",
	     "zuma_dout_apm_noc", 0x211c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_INTMEM_ACLK, "zuma_gout_apm_intmem_aclk",
	     "zuma_dout_apm_noc", 0x2108, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_INTMEM_PCLK, "zuma_gout_apm_intmem_pclk",
	     "zuma_dout_apm_noc", 0x210c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_GPIO_ALIVE, "zuma_gout_apm_gpio_alive",
	     "zuma_dout_apm_noc", 0x20d8, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_GPIO_FAR_ALIVE, "zuma_gout_apm_gpio_far_alive",
	     "zuma_dout_apm_noc", 0x20dc, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_GPIO_CUSTOM_ALIVE, "zuma_gout_apm_gpio_custom_alive",
	     "zuma_dout_apm_noc", 0x2000, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_USI1_UART_IPCLK, "zuma_gout_apm_usi1_uart_ipclk",
	     "zuma_dout_apm_usi1_uart", 0x20f4, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_USI1_UART_PCLK, "zuma_gout_apm_usi1_uart_pclk",
	     "zuma_dout_apm_noc", 0x20f8, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_USI1_UART_RSTNSYNC, "zuma_gout_apm_usi1_uart_rstnsync",
	     "zuma_dout_apm_usi1_uart", 0x2148, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_APM_SYSREG, "zuma_gout_apm_sysreg", "zuma_dout_apm_noc",
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

/* CMU_HSI0, 0x11000000. Firmware keeps the domain powered. */
PNAME(mout_hsi0_noc_user_parents) = { "oscclk", "zuma_dout_hsi0_noc" };

PNAME(mout_hsi0_alt_user_parents) = { "oscclk", "zuma_gout_hsi0_alt" };

PNAME(mout_hsi0_noc_local_parents) = {
	"zuma_mout_hsi0_noc_user", "zuma_mout_hsi0_alt_user",
};

PNAME(mout_hsi0_peri_user_parents) = { "oscclk", "zuma_dout_hsi0_peri" };

PNAME(mout_hsi0_usi2_parents) = { "zuma_mout_hsi0_peri_user", "oscclk" };

static const struct samsung_mux_clock hsi0_muxes[] __initconst = {
	MUX_F(CLK_MOUT_HSI0_NOC_USER, "zuma_mout_hsi0_noc_user",
	      mout_hsi0_noc_user_parents, 0x620, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_ALT_USER, "zuma_mout_hsi0_alt_user",
	      mout_hsi0_alt_user_parents, 0x600, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_NOC_LOCAL, "zuma_mout_hsi0_noc_local",
	      mout_hsi0_noc_local_parents, 0x1000, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_PERI_USER, "zuma_mout_hsi0_peri_user",
	      mout_hsi0_peri_user_parents, 0x680, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_USI2, "zuma_mout_hsi0_usi2", mout_hsi0_usi2_parents,
	      0x101c, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_HSI0_USI3, "zuma_mout_hsi0_usi3", mout_hsi0_usi2_parents,
	      0x1020, 0, 1, 0, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock hsi0_dividers[] __initconst = {
	DIV(CLK_DOUT_HSI0_USI2, "zuma_dout_hsi0_usi2",
	    "zuma_gout_hsi0_usi2_source", 0x181c, 0, 4),
	DIV(CLK_DOUT_HSI0_USI3, "zuma_dout_hsi0_usi3",
	    "zuma_gout_hsi0_usi3_source", 0x1820, 0, 4),
};

static const struct samsung_gate_clock hsi0_gates[] __initconst = {
	GATE(CLK_GOUT_HSI0_USI2_SOURCE, "zuma_gout_hsi0_usi2_source",
	     "zuma_mout_hsi0_usi2", 0x2120, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_USI2_IP, "zuma_gout_hsi0_usi2_ip",
	     "zuma_dout_hsi0_usi2", 0x20fc, 21,
	     CLK_IGNORE_UNUSED | CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_HSI0_USI2_PCLK, "zuma_gout_hsi0_usi2_pclk",
	     "zuma_mout_hsi0_noc_local", 0x2100, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_USI3_SOURCE, "zuma_gout_hsi0_usi3_source",
	     "zuma_mout_hsi0_usi3", 0x2124, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_HSI0_USI3_IP, "zuma_gout_hsi0_usi3_ip",
	     "zuma_dout_hsi0_usi3", 0x2104, 21,
	     CLK_IGNORE_UNUSED | CLK_SET_RATE_PARENT, 0),
	GATE(CLK_GOUT_HSI0_USI3_PCLK, "zuma_gout_hsi0_usi3_pclk",
	     "zuma_mout_hsi0_noc_local", 0x2108, 21, CLK_IGNORE_UNUSED, 0),
};

/* Firmware-owned gates are observed, never switched. */
static const struct samsung_gate_clock hsi0_ro_gates[] __initconst = {
	GATE(CLK_GOUT_HSI0_ALT, "zuma_gout_hsi0_alt", "hsi0-alt",
	     0x204c, 21, CLK_IGNORE_UNUSED, 0),
};

/* The USI2 and USI3 gates run in manual mode; the rest of HSI0 is left alone. */
static const unsigned long hsi0_regs[] __initconst = {
	0x2120, 0x20fc, 0x2100, 0x2124, 0x2104, 0x2108,
};

static const struct samsung_cmu_info hsi0_cmu __initconst = {
	.mux_clks = hsi0_muxes,
	.nr_mux_clks = ARRAY_SIZE(hsi0_muxes),
	.div_clks = hsi0_dividers,
	.nr_div_clks = ARRAY_SIZE(hsi0_dividers),
	.gate_clks = hsi0_gates,
	.nr_gate_clks = ARRAY_SIZE(hsi0_gates),
	.ro_gate_clks = hsi0_ro_gates,
	.nr_ro_gate_clks = ARRAY_SIZE(hsi0_ro_gates),
	.nr_clk_ids = CLK_GOUT_HSI0_USI3_PCLK + 1,
	.clk_regs = hsi0_regs,
	.nr_clk_regs = ARRAY_SIZE(hsi0_regs),
	.clk_name = "bus",
};

/*
 * CMU_PERIC1, 0x10c00000. Firmware sets it up: the user muxes and the USI9 and
 * USI11 dividers are read-only, and the GPIO and sysreg gates are read-only
 * gates. USI11 is the I2C bus of the MAX77759 gauge, charger and Type-C
 * controller and the MAX77759 PMIC; USI9 is the I2C bus of the battery pack's
 * EEPROM. Their gates are the ones Linux turns on, as for the Pixel 9a's
 * USI10; their auto/manual mode is left as firmware set it.
 */
PNAME(mout_peric1_noc_user_parents) = { "oscclk", "zuma_dout_peric1_noc" };
PNAME(mout_peric1_usi11_user_parents) = { "oscclk", "zuma_dout_peric1_ip" };
PNAME(mout_peric1_usi9_user_parents) = { "oscclk", "zuma_dout_peric1_ip" };
PNAME(mout_peric1_usi0_user_parents) = { "oscclk", "zuma_dout_peric1_ip" };

static const struct samsung_mux_clock peric1_muxes[] __initconst = {
	MUX_F(CLK_MOUT_PERIC1_NOC_USER, "zuma_mout_peric1_noc_user",
	      mout_peric1_noc_user_parents, 0x610, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC1_USI11_USER, "zuma_mout_peric1_usi11_user",
	      mout_peric1_usi11_user_parents, 0x640, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC1_USI9_USER, "zuma_mout_peric1_usi9_user",
	      mout_peric1_usi9_user_parents, 0x690, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC1_USI0_USER, "zuma_mout_peric1_usi0_user",
	      mout_peric1_usi0_user_parents, 0x620, 4, 1, 0, CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock peric1_dividers[] __initconst = {
	DIV_F(CLK_DOUT_PERIC1_USI11, "zuma_dout_peric1_usi11",
	      "zuma_mout_peric1_usi11_user", 0x1810, 0, 4, 0,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC1_USI9, "zuma_dout_peric1_usi9",
	      "zuma_mout_peric1_usi9_user", 0x1824, 0, 4, 0,
	      CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_PERIC1_USI0, "zuma_dout_peric1_usi0",
	      "zuma_mout_peric1_usi0_user", 0x1808, 0, 4, 0,
	      CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock peric1_ro_gates[] __initconst = {
	GATE(CLK_GOUT_GPIO_PERIC1, "zuma_gout_gpio_peric1",
	     "zuma_mout_peric1_noc_user", 0x2080, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC1_SYSREG, "zuma_gout_peric1_sysreg",
	     "zuma_mout_peric1_noc_user", 0x20a4, 21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_gate_clock peric1_gates[] __initconst = {
	GATE(CLK_GOUT_PERIC1_USI11_IPCLK, "zuma_gout_peric1_usi11_ipclk",
	     "zuma_dout_peric1_usi11", 0x2048, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI11_PCLK, "zuma_gout_peric1_usi11_pclk",
	     "zuma_mout_peric1_noc_user", 0x204c, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI11_RSTNSYNC, "zuma_gout_peric1_usi11_rstnsync",
	     "zuma_dout_peric1_usi11", 0x2094, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI9_IPCLK, "zuma_gout_peric1_usi9_ipclk",
	     "zuma_dout_peric1_usi9", 0x2070, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI9_PCLK, "zuma_gout_peric1_usi9_pclk",
	     "zuma_mout_peric1_noc_user", 0x2074, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI9_RSTNSYNC, "zuma_gout_peric1_usi9_rstnsync",
	     "zuma_dout_peric1_usi9", 0x20a0, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI0_IPCLK, "zuma_gout_peric1_usi0_ipclk",
	     "zuma_dout_peric1_usi0", 0x2038, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI0_PCLK, "zuma_gout_peric1_usi0_pclk",
	     "zuma_mout_peric1_noc_user", 0x203c, 21, 0, 0),
	GATE(CLK_GOUT_PERIC1_USI0_RSTNSYNC, "zuma_gout_peric1_usi0_rstnsync",
	     "zuma_dout_peric1_usi0", 0x208c, 21, 0, 0),
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
	.nr_clk_ids = CLK_GOUT_PERIC1_USI0_RSTNSYNC + 1,
	.clk_name = "bus",
};

/*
 * CMU_PERIC0, 0x10800000. Firmware sets it up, as CMU_PERIC1: USI14 is the
 * I2C bus of the rear flash LED. The user muxes and the divider are
 * read-only, the GPIO and sysreg gates are read-only gates.
 */
PNAME(mout_peric0_noc_user_parents) = { "oscclk", "zuma_dout_peric0_noc" };
PNAME(mout_peric0_usi14_user_parents) = { "oscclk", "zuma_dout_peric0_ip" };

static const struct samsung_mux_clock peric0_muxes[] __initconst = {
	MUX_F(CLK_MOUT_PERIC0_NOC_USER, "zuma_mout_peric0_noc_user",
	      mout_peric0_noc_user_parents, 0x600, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_PERIC0_USI14_USER, "zuma_mout_peric0_usi14_user",
	      mout_peric0_usi14_user_parents, 0x640, 4, 1, 0,
	      CLK_MUX_READ_ONLY),
};

static const struct samsung_div_clock peric0_dividers[] __initconst = {
	DIV_F(CLK_DOUT_PERIC0_USI14, "zuma_dout_peric0_usi14",
	      "zuma_mout_peric0_usi14_user", 0x180c, 0, 4, 0,
	      CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock peric0_ro_gates[] __initconst = {
	GATE(CLK_GOUT_GPIO_PERIC0, "zuma_gout_gpio_peric0",
	     "zuma_mout_peric0_noc_user", 0x20b8, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_PERIC0_SYSREG, "zuma_gout_peric0_sysreg",
	     "zuma_mout_peric0_noc_user", 0x20f0, 21, CLK_IGNORE_UNUSED, 0),
};

static const struct samsung_gate_clock peric0_gates[] __initconst = {
	GATE(CLK_GOUT_PERIC0_USI14_IPCLK, "zuma_gout_peric0_usi14_ipclk",
	     "zuma_dout_peric0_usi14", 0x2068, 21, 0, 0),
	GATE(CLK_GOUT_PERIC0_USI14_PCLK, "zuma_gout_peric0_usi14_pclk",
	     "zuma_mout_peric0_noc_user", 0x206c, 21, 0, 0),
	GATE(CLK_GOUT_PERIC0_USI14_RSTNSYNC, "zuma_gout_peric0_usi14_rstnsync",
	     "zuma_dout_peric0_usi14", 0x20cc, 21, 0, 0),
};

static const struct samsung_cmu_info peric0_cmu __initconst = {
	.mux_clks = peric0_muxes,
	.nr_mux_clks = ARRAY_SIZE(peric0_muxes),
	.div_clks = peric0_dividers,
	.nr_div_clks = ARRAY_SIZE(peric0_dividers),
	.ro_gate_clks = peric0_ro_gates,
	.nr_ro_gate_clks = ARRAY_SIZE(peric0_ro_gates),
	.gate_clks = peric0_gates,
	.nr_gate_clks = ARRAY_SIZE(peric0_gates),
	.nr_clk_ids = CLK_GOUT_PERIC0_USI14_RSTNSYNC + 1,
	.clk_name = "bus",
};

/*
 * CMU_G3D, 0x1ee00000. Firmware sets it up and keeps its domain on: the
 * PLLs, muxes and dividers are read-only (the PLLs use the PLL0822x M/P/S
 * layout, with no rate table), and the GPU's gates are read-only gates. The
 * GPU's own rate changes go through the ACPM firmware, not this CMU.
 */
PNAME(mout_g3d_switch_user_parents) = { "oscclk", "zuma_dout_g3d_switch" };

PNAME(mout_g3d_glb_user_parents) = { "oscclk", "zuma_dout_g3d_glb" };

PNAME(mout_g3d_nocd_user_parents) = { "oscclk", "zuma_dout_g3d_nocd" };

PNAME(mout_g3d_l2_glb_parents) = {
	"zuma_fout_g3d_l2", "zuma_mout_g3d_glb_user",
};

PNAME(mout_g3d_stacks_parents) = {
	"zuma_fout_g3d", "zuma_mout_g3d_switch_user",
};

PNAME(mout_g3d_top_parents) = {
	"zuma_mout_g3d_stacks", "zuma_mout_g3d_nocd_user",
	"zuma_mout_g3d_l2_glb", "oscclk",
};

PNAME(mout_g3d_coregroup_user_parents) = { "oscclk", "zuma_dout_g3d_l2_glb" };

PNAME(mout_g3d_stacks_user_parents) = { "oscclk", "zuma_dout_g3d_stacks" };

PNAME(mout_g3d_top_user_parents) = { "oscclk", "zuma_dout_g3d_top" };

static const struct samsung_pll_clock g3d_plls[] __initconst = {
	PLL(pll_0822x, CLK_FOUT_G3D, "zuma_fout_g3d", "oscclk",
	    0x000, 0x10c, NULL),
	PLL(pll_0822x, CLK_FOUT_G3D_L2, "zuma_fout_g3d_l2", "oscclk",
	    0x008, 0x18c, NULL),
};

static const struct samsung_mux_clock g3d_muxes[] __initconst = {
	MUX_F(CLK_MOUT_G3D_SWITCH_USER, "zuma_mout_g3d_switch_user",
	      mout_g3d_switch_user_parents, 0x620, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_GLB_USER, "zuma_mout_g3d_glb_user",
	      mout_g3d_glb_user_parents, 0x600, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_NOCD_USER, "zuma_mout_g3d_nocd_user",
	      mout_g3d_nocd_user_parents, 0x610, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_L2_GLB, "zuma_mout_g3d_l2_glb",
	      mout_g3d_l2_glb_parents, 0x1000, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_STACKS, "zuma_mout_g3d_stacks",
	      mout_g3d_stacks_parents, 0x1004, 0, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_TOP, "zuma_mout_g3d_top", mout_g3d_top_parents,
	      0x1008, 0, 2, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_COREGROUP_USER, "zuma_mout_g3d_coregroup_user",
	      mout_g3d_coregroup_user_parents, 0x630, 4, 1, 0,
	      CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_STACKS_USER, "zuma_mout_g3d_stacks_user",
	      mout_g3d_stacks_user_parents, 0x640, 4, 1, 0, CLK_MUX_READ_ONLY),
	MUX_F(CLK_MOUT_G3D_TOP_USER, "zuma_mout_g3d_top_user",
	      mout_g3d_top_user_parents, 0x650, 4, 1, 0, CLK_MUX_READ_ONLY),
};

/* CAL exposes no ratio field for these two paths: they are pass-through. */
static const struct samsung_fixed_factor_clock g3d_factors[] __initconst = {
	FFACTOR(CLK_DOUT_G3D_L2_GLB, "zuma_dout_g3d_l2_glb",
		"zuma_mout_g3d_l2_glb", 1, 1, 0),
	FFACTOR(CLK_DOUT_G3D_STACKS, "zuma_dout_g3d_stacks",
		"zuma_mout_g3d_stacks", 1, 1, 0),
};

static const struct samsung_div_clock g3d_dividers[] __initconst = {
	DIV_F(CLK_DOUT_G3D_TOP, "zuma_dout_g3d_top", "zuma_mout_g3d_top",
	      0x1814, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
	DIV_F(CLK_DOUT_G3D_NOCP, "zuma_dout_g3d_nocp", "zuma_dout_g3d_l2_glb",
	      0x1808, 0, 3, 0, CLK_DIVIDER_READ_ONLY),
};

static const struct samsung_gate_clock g3d_ro_gates[] __initconst = {
	GATE(CLK_GOUT_G3D_CMU, "zuma_gout_g3d_cmu", "zuma_dout_g3d_nocp",
	     0x2018, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_CORE, "zuma_gout_g3d_core", "zuma_mout_g3d_top_user",
	     0x201c, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_COREGROUP, "zuma_gout_g3d_coregroup",
	     "zuma_mout_g3d_coregroup_user", 0x2020, 21, CLK_IGNORE_UNUSED, 0),
	GATE(CLK_GOUT_G3D_STACKS, "zuma_gout_g3d_stacks",
	     "zuma_mout_g3d_stacks_user", 0x2024, 21, CLK_IGNORE_UNUSED, 0),
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

static int __init zuma_cmu_probe(struct platform_device *pdev)
{
	const struct samsung_cmu_info *cmu = device_get_match_data(&pdev->dev);

	exynos_arm64_register_cmu(&pdev->dev, pdev->dev.of_node, cmu);
	return 0;
}

static const struct of_device_id zuma_cmu_match[] = {
	{ .compatible = "google,zuma-cmu-apm", .data = &apm_cmu },
	{ .compatible = "google,zuma-cmu-g3d", .data = &g3d_cmu },
	{ .compatible = "google,zuma-cmu-hsi0", .data = &hsi0_cmu },
	{ .compatible = "google,zuma-cmu-peric0", .data = &peric0_cmu },
	{ .compatible = "google,zuma-cmu-peric1", .data = &peric1_cmu },
	{ }
};

static struct platform_driver zuma_cmu_driver __refdata = {
	.driver = {
		.name = "zuma-cmu",
		.of_match_table = zuma_cmu_match,
		.suppress_bind_attrs = true,
	},
	.probe = zuma_cmu_probe,
};

static int __init zuma_cmu_init(void)
{
	return platform_driver_register(&zuma_cmu_driver);
}
core_initcall(zuma_cmu_init);
