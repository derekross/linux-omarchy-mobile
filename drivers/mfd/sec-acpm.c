// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Google Inc
 * Copyright 2025 Linaro Ltd.
 *
 * Samsung S2MPG1x ACPM driver
 */

#include <linux/array_size.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/rtc.h>
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg11.h>
#include <linux/mfd/samsung/s2mpg12.h>
#include <linux/mfd/samsung/s2mpg13.h>
#include <linux/mfd/samsung/s2mpg14.h>
#include <linux/mfd/samsung/s2mpg15.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include "sec-core.h"

#define ACPM_ADDR_BITS       8
#define ACPM_MAX_BULK_DATA   8

struct sec_pmic_acpm_platform_data {
	int device_type;

	unsigned int acpm_chan_id;
	u8 speedy_channel;

	const struct regmap_config *regmap_cfg_common;
	const struct regmap_config *regmap_cfg_pmic;
	const struct regmap_config *regmap_cfg_rtc;
	const struct regmap_config *regmap_cfg_meter;
	const struct regmap_config *regmap_cfg_gpio;

	/* No interrupt line is wired up for this PMIC (yet). */
	bool no_irq;
};

static const struct regmap_range s2mpg10_common_registers[] = {
	regmap_reg_range(0x00, 0x02), /* CHIP_ID_M, INT, INT_MASK */
	regmap_reg_range(0x0a, 0x0c), /* Speedy control */
	regmap_reg_range(0x1a, 0x2a), /* Debug */
};

static const struct regmap_range s2mpg10_common_ro_registers[] = {
	regmap_reg_range(0x00, 0x01), /* CHIP_ID_M, INT */
	regmap_reg_range(0x28, 0x2a), /* Debug */
};

static const struct regmap_range s2mpg10_common_nonvolatile_registers[] = {
	regmap_reg_range(0x00, 0x00), /* CHIP_ID_M */
	regmap_reg_range(0x02, 0x02), /* INT_MASK */
	regmap_reg_range(0x0a, 0x0c), /* Speedy control */
};

static const struct regmap_range s2mpg10_common_precious_registers[] = {
	regmap_reg_range(0x01, 0x01), /* INT */
};

static const struct regmap_access_table s2mpg10_common_wr_table = {
	.yes_ranges = s2mpg10_common_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_common_registers),
	.no_ranges = s2mpg10_common_ro_registers,
	.n_no_ranges = ARRAY_SIZE(s2mpg10_common_ro_registers),
};

static const struct regmap_access_table s2mpg10_common_rd_table = {
	.yes_ranges = s2mpg10_common_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_common_registers),
};

static const struct regmap_access_table s2mpg10_common_volatile_table = {
	.no_ranges = s2mpg10_common_nonvolatile_registers,
	.n_no_ranges = ARRAY_SIZE(s2mpg10_common_nonvolatile_registers),
};

static const struct regmap_access_table s2mpg10_common_precious_table = {
	.yes_ranges = s2mpg10_common_precious_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_common_precious_registers),
};

static const struct regmap_config s2mpg10_regmap_config_common = {
	.name = "common",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG10_COMMON_SPD_DEBUG4,
	.wr_table = &s2mpg10_common_wr_table,
	.rd_table = &s2mpg10_common_rd_table,
	.volatile_table = &s2mpg10_common_volatile_table,
	.precious_table = &s2mpg10_common_precious_table,
	.num_reg_defaults_raw = S2MPG10_COMMON_SPD_DEBUG4 + 1,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_range s2mpg10_pmic_registers[] = {
	regmap_reg_range(0x00, 0xf6), /* All PMIC registers */
};

static const struct regmap_range s2mpg10_pmic_ro_registers[] = {
	regmap_reg_range(0x00, 0x05), /* INTx */
	regmap_reg_range(0x0c, 0x0f), /* STATUSx PWRONSRC OFFSRC */
	regmap_reg_range(0xc7, 0xc7), /* GPIO input */
};

static const struct regmap_range s2mpg10_pmic_nonvolatile_registers[] = {
	regmap_reg_range(0x06, 0x0b), /* INTxM */
};

static const struct regmap_range s2mpg10_pmic_precious_registers[] = {
	regmap_reg_range(0x00, 0x05), /* INTx */
};

static const struct regmap_access_table s2mpg10_pmic_wr_table = {
	.yes_ranges = s2mpg10_pmic_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_pmic_registers),
	.no_ranges = s2mpg10_pmic_ro_registers,
	.n_no_ranges = ARRAY_SIZE(s2mpg10_pmic_ro_registers),
};

static const struct regmap_access_table s2mpg10_pmic_rd_table = {
	.yes_ranges = s2mpg10_pmic_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_pmic_registers),
};

static const struct regmap_access_table s2mpg10_pmic_volatile_table = {
	.no_ranges = s2mpg10_pmic_nonvolatile_registers,
	.n_no_ranges = ARRAY_SIZE(s2mpg10_pmic_nonvolatile_registers),
};

static const struct regmap_access_table s2mpg10_pmic_precious_table = {
	.yes_ranges = s2mpg10_pmic_precious_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_pmic_precious_registers),
};

static const struct regmap_config s2mpg10_regmap_config_pmic = {
	.name = "pmic",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG10_PMIC_LDO_SENSE4,
	.wr_table = &s2mpg10_pmic_wr_table,
	.rd_table = &s2mpg10_pmic_rd_table,
	.volatile_table = &s2mpg10_pmic_volatile_table,
	.precious_table = &s2mpg10_pmic_precious_table,
	.num_reg_defaults_raw = S2MPG10_PMIC_LDO_SENSE4 + 1,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_range s2mpg10_rtc_registers[] = {
	regmap_reg_range(0x00, 0x2b), /* All RTC registers */
};

static const struct regmap_range s2mpg10_rtc_volatile_registers[] = {
	regmap_reg_range(0x01, 0x01), /* RTC_UPDATE */
	regmap_reg_range(0x05, 0x0c), /* Time / date */
};

static const struct regmap_access_table s2mpg10_rtc_rd_table = {
	.yes_ranges = s2mpg10_rtc_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_rtc_registers),
};

static const struct regmap_access_table s2mpg10_rtc_volatile_table = {
	.yes_ranges = s2mpg10_rtc_volatile_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_rtc_volatile_registers),
};

static const struct regmap_config s2mpg10_regmap_config_rtc = {
	.name = "rtc",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG10_RTC_OSC_CTRL,
	.rd_table = &s2mpg10_rtc_rd_table,
	.volatile_table = &s2mpg10_rtc_volatile_table,
	.num_reg_defaults_raw = S2MPG10_RTC_OSC_CTRL + 1,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_range s2mpg10_meter_registers[] = {
	regmap_reg_range(0x00, 0x21), /* Meter config */
	regmap_reg_range(0x40, 0x8a), /* Meter data */
	regmap_reg_range(0xee, 0xee), /* Offset */
	regmap_reg_range(0xf1, 0xf1), /* Trim */
};

static const struct regmap_range s2mpg10_meter_ro_registers[] = {
	regmap_reg_range(0x40, 0x8a), /* Meter data */
};

static const struct regmap_access_table s2mpg10_meter_wr_table = {
	.yes_ranges = s2mpg10_meter_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_meter_registers),
	.no_ranges = s2mpg10_meter_ro_registers,
	.n_no_ranges = ARRAY_SIZE(s2mpg10_meter_ro_registers),
};

static const struct regmap_access_table s2mpg10_meter_rd_table = {
	.yes_ranges = s2mpg10_meter_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_meter_registers),
};

static const struct regmap_access_table s2mpg10_meter_volatile_table = {
	.yes_ranges = s2mpg10_meter_ro_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg10_meter_ro_registers),
};

static const struct regmap_config s2mpg10_regmap_config_meter = {
	.name = "meter",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG10_METER_BUCK_METER_TRIM3,
	.wr_table = &s2mpg10_meter_wr_table,
	.rd_table = &s2mpg10_meter_rd_table,
	.volatile_table = &s2mpg10_meter_volatile_table,
	.num_reg_defaults_raw = S2MPG10_METER_BUCK_METER_TRIM3 + 1,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_range s2mpg11_common_registers[] = {
	regmap_reg_range(0x00, 0x02), /* CHIP_ID_S, INT, INT_MASK */
	regmap_reg_range(0x0a, 0x0c), /* Speedy control */
	regmap_reg_range(0x1a, 0x27), /* Debug */
};

static const struct regmap_range s2mpg11_common_ro_registers[] = {
	regmap_reg_range(0x00, 0x01), /* CHIP_ID_S, INT */
	regmap_reg_range(0x25, 0x27), /* Debug */
};

static const struct regmap_range s2mpg11_common_nonvolatile_registers[] = {
	regmap_reg_range(0x00, 0x00), /* CHIP_ID_S */
	regmap_reg_range(0x02, 0x02), /* INT_MASK */
	regmap_reg_range(0x0a, 0x0c), /* Speedy control */
};

static const struct regmap_range s2mpg11_common_precious_registers[] = {
	regmap_reg_range(0x01, 0x01), /* INT */
};

static const struct regmap_access_table s2mpg11_common_wr_table = {
	.yes_ranges = s2mpg11_common_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg11_common_registers),
	.no_ranges = s2mpg11_common_ro_registers,
	.n_no_ranges = ARRAY_SIZE(s2mpg11_common_ro_registers),
};

static const struct regmap_access_table s2mpg11_common_rd_table = {
	.yes_ranges = s2mpg11_common_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg11_common_registers),
};

static const struct regmap_access_table s2mpg11_common_volatile_table = {
	.no_ranges = s2mpg11_common_nonvolatile_registers,
	.n_no_ranges = ARRAY_SIZE(s2mpg11_common_nonvolatile_registers),
};

static const struct regmap_access_table s2mpg11_common_precious_table = {
	.yes_ranges = s2mpg11_common_precious_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg11_common_precious_registers),
};

static const struct regmap_config s2mpg11_regmap_config_common = {
	.name = "common",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG11_COMMON_SPD_DEBUG4,
	.wr_table = &s2mpg11_common_wr_table,
	.rd_table = &s2mpg11_common_rd_table,
	.volatile_table = &s2mpg11_common_volatile_table,
	.precious_table = &s2mpg11_common_precious_table,
	.num_reg_defaults_raw = S2MPG11_COMMON_SPD_DEBUG4 + 1,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_range s2mpg11_pmic_registers[] = {
	regmap_reg_range(0x00, 0x5a), /* All PMIC registers */
	regmap_reg_range(0x5c, 0xb7), /* All PMIC registers */
};

static const struct regmap_range s2mpg11_pmic_ro_registers[] = {
	regmap_reg_range(0x00, 0x05), /* INTx */
	regmap_reg_range(0x0c, 0x0d), /* STATUS OFFSRC */
	regmap_reg_range(0x98, 0x98), /* GPIO input */
};

static const struct regmap_range s2mpg11_pmic_nonvolatile_registers[] = {
	regmap_reg_range(0x06, 0x0b), /* INTxM */
};

static const struct regmap_range s2mpg11_pmic_precious_registers[] = {
	regmap_reg_range(0x00, 0x05), /* INTx */
};

static const struct regmap_access_table s2mpg11_pmic_wr_table = {
	.yes_ranges = s2mpg11_pmic_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg11_pmic_registers),
	.no_ranges = s2mpg11_pmic_ro_registers,
	.n_no_ranges = ARRAY_SIZE(s2mpg11_pmic_ro_registers),
};

static const struct regmap_access_table s2mpg11_pmic_rd_table = {
	.yes_ranges = s2mpg11_pmic_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg11_pmic_registers),
};

static const struct regmap_access_table s2mpg11_pmic_volatile_table = {
	.no_ranges = s2mpg11_pmic_nonvolatile_registers,
	.n_no_ranges = ARRAY_SIZE(s2mpg11_pmic_nonvolatile_registers),
};

static const struct regmap_access_table s2mpg11_pmic_precious_table = {
	.yes_ranges = s2mpg11_pmic_precious_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg11_pmic_precious_registers),
};

static const struct regmap_config s2mpg11_regmap_config_pmic = {
	.name = "pmic",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG11_PMIC_LDO_SENSE2,
	.wr_table = &s2mpg11_pmic_wr_table,
	.rd_table = &s2mpg11_pmic_rd_table,
	.volatile_table = &s2mpg11_pmic_volatile_table,
	.precious_table = &s2mpg11_pmic_precious_table,
	.num_reg_defaults_raw = S2MPG11_PMIC_LDO_SENSE2 + 1,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_range s2mpg11_meter_registers[] = {
	regmap_reg_range(0x00, 0x3e), /* Meter config */
	regmap_reg_range(0x40, 0x8a), /* Meter data */
	regmap_reg_range(0x8d, 0x9c), /* Meter data */
};

static const struct regmap_range s2mpg11_meter_ro_registers[] = {
	regmap_reg_range(0x40, 0x9c), /* Meter data */
};

static const struct regmap_access_table s2mpg11_meter_wr_table = {
	.yes_ranges = s2mpg11_meter_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg11_meter_registers),
	.no_ranges = s2mpg11_meter_ro_registers,
	.n_no_ranges = ARRAY_SIZE(s2mpg11_meter_ro_registers),
};

static const struct regmap_access_table s2mpg11_meter_rd_table = {
	.yes_ranges = s2mpg11_meter_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg11_meter_registers),
};

static const struct regmap_access_table s2mpg11_meter_volatile_table = {
	.yes_ranges = s2mpg11_meter_ro_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg11_meter_ro_registers),
};

static const struct regmap_config s2mpg11_regmap_config_meter = {
	.name = "meter",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG11_METER_LPF_DATA_NTC7_2,
	.wr_table = &s2mpg11_meter_wr_table,
	.rd_table = &s2mpg11_meter_rd_table,
	.volatile_table = &s2mpg11_meter_volatile_table,
	.num_reg_defaults_raw = S2MPG11_METER_LPF_DATA_NTC7_2 + 1,
	.cache_type = REGCACHE_FLAT,
};

/*
 * S2MPG12 (gs201 main PMIC): the RTC, the interrupt masks, and the LDO
 * control block. The common block can write only its IBI masks. The PM
 * block can write INT1M-INT5M and L1M_CTRL through LDO_CTRL2. Buck
 * control and SW_RESET stay outside that window. No caching: every
 * access goes to the PMIC.
 */
static const struct regmap_range s2mpg12_common_registers[] = {
	regmap_reg_range(0x00, 0x10), /* VGPIO, I3C, IBI, CHIPID, IBI masks */
	regmap_reg_range(0x25, 0x25), /* TEST_MODE2 */
};

static const struct regmap_range s2mpg12_common_wr_registers[] = {
	regmap_reg_range(S2MPG12_COMMON_IBIM1, S2MPG12_COMMON_IBIM2),
};

static const struct regmap_access_table s2mpg12_common_wr_table = {
	.yes_ranges = s2mpg12_common_wr_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg12_common_wr_registers),
};

static const struct regmap_access_table s2mpg12_common_rd_table = {
	.yes_ranges = s2mpg12_common_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg12_common_registers),
};

static const struct regmap_config s2mpg12_regmap_config_common = {
	.name = "common",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG12_COMMON_TEST_MODE2,
	.wr_table = &s2mpg12_common_wr_table,
	.rd_table = &s2mpg12_common_rd_table,
	.cache_type = REGCACHE_NONE,
};

/*
 * Interrupt masks, plus the LDO control block (L1M_CTRL through LDO_CTRL2).
 * Buck control (0x17-0x2a), LDO_CTRL3, and SW_RESET stay outside this
 * window. A rail with no DT constraints is never asked to change.
 */
static const struct regmap_range s2mpg12_pmic_wr_registers[] = {
	regmap_reg_range(S2MPG12_PMIC_INT1M, S2MPG12_PMIC_INT5M),
	regmap_reg_range(S2MPG12_PMIC_L1M_CTRL, S2MPG12_PMIC_LDO_CTRL2),
};

static const struct regmap_range s2mpg12_pmic_precious_registers[] = {
	regmap_reg_range(S2MPG12_PMIC_INT1, S2MPG12_PMIC_INT5), /* clear on read */
};

static const struct regmap_access_table s2mpg12_pmic_wr_table = {
	.yes_ranges = s2mpg12_pmic_wr_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg12_pmic_wr_registers),
};

static const struct regmap_access_table s2mpg12_pmic_precious_table = {
	.yes_ranges = s2mpg12_pmic_precious_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg12_pmic_precious_registers),
};

static const struct regmap_config s2mpg12_regmap_config_pmic = {
	.name = "pmic",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG12_PMIC_SW_RESET,
	.wr_table = &s2mpg12_pmic_wr_table,
	.precious_table = &s2mpg12_pmic_precious_table,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config s2mpg12_regmap_config_rtc = {
	.name = "rtc",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG12_RTC_LAST,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_range s2mpg12_meter_registers[] = {
	regmap_reg_range(S2MPG12_METER_INT1, S2MPG12_METER_INT2M),
};

static const struct regmap_range s2mpg12_meter_wr_registers[] = {
	regmap_reg_range(S2MPG12_METER_INT1M, S2MPG12_METER_INT2M),
};

static const struct regmap_access_table s2mpg12_meter_wr_table = {
	.yes_ranges = s2mpg12_meter_wr_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg12_meter_wr_registers),
};

static const struct regmap_access_table s2mpg12_meter_rd_table = {
	.yes_ranges = s2mpg12_meter_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg12_meter_registers),
};

static const struct regmap_config s2mpg12_regmap_config_meter = {
	.name = "meter",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG12_METER_INT2M,
	.wr_table = &s2mpg12_meter_wr_table,
	.rd_table = &s2mpg12_meter_rd_table,
	.cache_type = REGCACHE_NONE,
};

/*
 * S2MPG13 (gs201 sub-PMIC): only its LDOs and GPIOs so far. The common block
 * is read only. In the PM block only the LDO control registers can be
 * written, and in the GPIO block only the per-pin control registers. The
 * interrupt status registers (clear on read) are never read, so nothing here
 * can touch a buck converter or lose an interrupt. No caching.
 */
static const struct regmap_range s2mpg13_common_registers[] = {
	regmap_reg_range(S2MPG13_COMMON_VGPIO0, S2MPG13_COMMON_IBIM2),
};

static const struct regmap_access_table s2mpg13_common_rd_table = {
	.yes_ranges = s2mpg13_common_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg13_common_registers),
};

static const struct regmap_access_table s2mpg13_common_wr_table = { };

static const struct regmap_config s2mpg13_regmap_config_common = {
	.name = "common",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG13_COMMON_IBIM2,
	.wr_table = &s2mpg13_common_wr_table,
	.rd_table = &s2mpg13_common_rd_table,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_range s2mpg13_pmic_rd_registers[] = {
	regmap_reg_range(S2MPG13_PMIC_INT1M, S2MPG13_PMIC_INT4M),
	regmap_reg_range(S2MPG13_PMIC_L1S_CTRL, S2MPG13_PMIC_LDO_CTRL2),
};

static const struct regmap_range s2mpg13_pmic_wr_registers[] = {
	regmap_reg_range(S2MPG13_PMIC_L1S_CTRL, S2MPG13_PMIC_LDO_CTRL2),
};

static const struct regmap_access_table s2mpg13_pmic_rd_table = {
	.yes_ranges = s2mpg13_pmic_rd_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg13_pmic_rd_registers),
};

static const struct regmap_access_table s2mpg13_pmic_wr_table = {
	.yes_ranges = s2mpg13_pmic_wr_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg13_pmic_wr_registers),
};

static const struct regmap_config s2mpg13_regmap_config_pmic = {
	.name = "pmic",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG13_PMIC_LDO_CTRL2,
	.wr_table = &s2mpg13_pmic_wr_table,
	.rd_table = &s2mpg13_pmic_rd_table,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_range s2mpg13_gpio_rd_registers[] = {
	regmap_reg_range(S2MPG13_GPIO_INT1M, S2MPG13_GPIO_7_MONSEL),
};

static const struct regmap_range s2mpg13_gpio_wr_registers[] = {
	regmap_reg_range(S2MPG13_GPIO_0_SET, S2MPG13_GPIO_7_SET),
};

static const struct regmap_access_table s2mpg13_gpio_rd_table = {
	.yes_ranges = s2mpg13_gpio_rd_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg13_gpio_rd_registers),
};

static const struct regmap_access_table s2mpg13_gpio_wr_table = {
	.yes_ranges = s2mpg13_gpio_wr_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg13_gpio_wr_registers),
};

static const struct regmap_config s2mpg13_regmap_config_gpio = {
	.name = "gpio",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG13_GPIO_7_MONSEL,
	.wr_table = &s2mpg13_gpio_wr_table,
	.rd_table = &s2mpg13_gpio_rd_table,
	.cache_type = REGCACHE_NONE,
};

/* Expose only the touchscreen and eUSB2 supply controls. */
static const struct regmap_range s2mpg14_common_registers[] = {
	regmap_reg_range(S2MPG14_COMMON_CHIPID, S2MPG14_COMMON_CHIPID),
};

static const struct regmap_access_table s2mpg14_common_rd_table = {
	.yes_ranges = s2mpg14_common_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg14_common_registers),
};

static const struct regmap_range s2mpg14_common_no_writes[] = {
	regmap_reg_range(0, S2MPG14_COMMON_CHIPID),
};

static const struct regmap_access_table s2mpg14_common_wr_table = {
	.no_ranges = s2mpg14_common_no_writes,
	.n_no_ranges = ARRAY_SIZE(s2mpg14_common_no_writes),
};

static const struct regmap_config s2mpg14_regmap_config_common = {
	.name = "common",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG14_COMMON_CHIPID,
	.rd_table = &s2mpg14_common_rd_table,
	.wr_table = &s2mpg14_common_wr_table,
};

static const struct regmap_range s2mpg14_pmic_rd_registers[] = {
	regmap_reg_range(S2MPG14_PMIC_L4M_CTRL, S2MPG14_PMIC_L4M_CTRL),
	regmap_reg_range(S2MPG14_PMIC_L8M_CTRL, S2MPG14_PMIC_L9M_CTRL),
	regmap_reg_range(S2MPG14_PMIC_L19M_CTRL, S2MPG14_PMIC_L19M_CTRL),
	regmap_reg_range(S2MPG14_PMIC_L25M_CTRL, S2MPG14_PMIC_LDO_CTRL1),
};

static const struct regmap_range s2mpg14_pmic_wr_registers[] = {
	regmap_reg_range(S2MPG14_PMIC_L4M_CTRL, S2MPG14_PMIC_L4M_CTRL),
	regmap_reg_range(S2MPG14_PMIC_L9M_CTRL, S2MPG14_PMIC_L9M_CTRL),
	regmap_reg_range(S2MPG14_PMIC_L19M_CTRL, S2MPG14_PMIC_L19M_CTRL),
	regmap_reg_range(S2MPG14_PMIC_L25M_CTRL, S2MPG14_PMIC_LDO_CTRL1),
};

static const struct regmap_access_table s2mpg14_pmic_rd_table = {
	.yes_ranges = s2mpg14_pmic_rd_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg14_pmic_rd_registers),
};

static const struct regmap_access_table s2mpg14_pmic_wr_table = {
	.yes_ranges = s2mpg14_pmic_wr_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg14_pmic_wr_registers),
};

static const struct regmap_config s2mpg14_regmap_config_pmic = {
	.name = "pmic",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG14_PMIC_LDO_CTRL1,
	.rd_table = &s2mpg14_pmic_rd_table,
	.wr_table = &s2mpg14_pmic_wr_table,
};

/* The sub-PMIC's revision and the LDO controls its consumers use. */
static const struct regmap_range s2mpg15_common_registers[] = {
	regmap_reg_range(S2MPG15_COMMON_CHIPID, S2MPG15_COMMON_CHIPID),
};

static const struct regmap_access_table s2mpg15_common_rd_table = {
	.yes_ranges = s2mpg15_common_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg15_common_registers),
};

static const struct regmap_range s2mpg15_common_no_writes[] = {
	regmap_reg_range(0, S2MPG15_COMMON_CHIPID),
};

static const struct regmap_access_table s2mpg15_common_wr_table = {
	.no_ranges = s2mpg15_common_no_writes,
	.n_no_ranges = ARRAY_SIZE(s2mpg15_common_no_writes),
};

static const struct regmap_config s2mpg15_regmap_config_common = {
	.name = "common",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG15_COMMON_CHIPID,
	.rd_table = &s2mpg15_common_rd_table,
	.wr_table = &s2mpg15_common_wr_table,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_range s2mpg15_pmic_registers[] = {
	regmap_reg_range(S2MPG15_PMIC_L5S_CTRL, S2MPG15_PMIC_L5S_CTRL),
	regmap_reg_range(S2MPG15_PMIC_L7S_CTRL, S2MPG15_PMIC_L7S_CTRL),
	regmap_reg_range(S2MPG15_PMIC_L9S_CTRL, S2MPG15_PMIC_L11S_CTRL),
	regmap_reg_range(S2MPG15_PMIC_L12S_CTRL, S2MPG15_PMIC_L12S_CTRL),
	regmap_reg_range(S2MPG15_PMIC_L15S_CTRL, S2MPG15_PMIC_L15S_CTRL),
};

static const struct regmap_access_table s2mpg15_pmic_table = {
	.yes_ranges = s2mpg15_pmic_registers,
	.n_yes_ranges = ARRAY_SIZE(s2mpg15_pmic_registers),
};

static const struct regmap_config s2mpg15_regmap_config_pmic = {
	.name = "pmic",
	.reg_bits = ACPM_ADDR_BITS,
	.val_bits = 8,
	.max_register = S2MPG15_PMIC_L15S_CTRL,
	.rd_table = &s2mpg15_pmic_table,
	.wr_table = &s2mpg15_pmic_table,
	.cache_type = REGCACHE_NONE,
};

struct sec_pmic_acpm_shared_bus_context {
	struct acpm_handle *acpm;
	unsigned int acpm_chan_id;
	u8 speedy_channel;
};

enum sec_pmic_acpm_accesstype {
	SEC_PMIC_ACPM_ACCESSTYPE_COMMON = 0x00,
	SEC_PMIC_ACPM_ACCESSTYPE_PMIC = 0x01,
	SEC_PMIC_ACPM_ACCESSTYPE_RTC = 0x02,
	SEC_PMIC_ACPM_ACCESSTYPE_METER = 0x0a,
	SEC_PMIC_ACPM_ACCESSTYPE_WLWP = 0x0b,
	SEC_PMIC_ACPM_ACCESSTYPE_GPIO = 0x0c,
	SEC_PMIC_ACPM_ACCESSTYPE_TRIM = 0x0f,
};

struct sec_pmic_acpm_bus_context {
	struct sec_pmic_acpm_shared_bus_context *shared;
	enum sec_pmic_acpm_accesstype type;
};

static int sec_pmic_acpm_bus_write(void *context, const void *data,
				   size_t count)
{
	struct sec_pmic_acpm_bus_context *ctx = context;
	struct acpm_handle *acpm = ctx->shared->acpm;
	const struct acpm_pmic_ops *pmic_ops = &acpm->ops->pmic;
	size_t val_count = count - BITS_TO_BYTES(ACPM_ADDR_BITS);
	const u8 *d = data;
	const u8 *vals = &d[BITS_TO_BYTES(ACPM_ADDR_BITS)];
	u8 reg;

	if (val_count < 1 || val_count > ACPM_MAX_BULK_DATA)
		return -EINVAL;

	reg = d[0];

	return pmic_ops->bulk_write(acpm, ctx->shared->acpm_chan_id, ctx->type, reg,
				    ctx->shared->speedy_channel, val_count, vals);
}

static int sec_pmic_acpm_bus_read(void *context, const void *reg_buf, size_t reg_size,
				  void *val_buf, size_t val_size)
{
	struct sec_pmic_acpm_bus_context *ctx = context;
	struct acpm_handle *acpm = ctx->shared->acpm;
	const struct acpm_pmic_ops *pmic_ops = &acpm->ops->pmic;
	const u8 *r = reg_buf;
	u8 reg;

	if (reg_size != BITS_TO_BYTES(ACPM_ADDR_BITS) || !val_size ||
	    val_size > ACPM_MAX_BULK_DATA)
		return -EINVAL;

	reg = r[0];

	return pmic_ops->bulk_read(acpm, ctx->shared->acpm_chan_id, ctx->type, reg,
				   ctx->shared->speedy_channel, val_size, val_buf);
}

static int sec_pmic_acpm_bus_reg_update_bits(void *context, unsigned int reg, unsigned int mask,
					     unsigned int val)
{
	struct sec_pmic_acpm_bus_context *ctx = context;
	struct acpm_handle *acpm = ctx->shared->acpm;
	const struct acpm_pmic_ops *pmic_ops = &acpm->ops->pmic;

	return pmic_ops->update_reg(acpm, ctx->shared->acpm_chan_id, ctx->type, reg & 0xff,
				    ctx->shared->speedy_channel, val, mask);
}

static const struct regmap_bus sec_pmic_acpm_regmap_bus = {
	.write = sec_pmic_acpm_bus_write,
	.read = sec_pmic_acpm_bus_read,
	.reg_update_bits = sec_pmic_acpm_bus_reg_update_bits,
	.max_raw_read = ACPM_MAX_BULK_DATA,
	.max_raw_write = ACPM_MAX_BULK_DATA,
};

static struct regmap *sec_pmic_acpm_regmap_init(struct device *dev,
						struct sec_pmic_acpm_shared_bus_context *shared_ctx,
						enum sec_pmic_acpm_accesstype type,
						const struct regmap_config *cfg, bool do_attach)
{
	struct sec_pmic_acpm_bus_context *ctx;
	struct regmap *regmap;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->shared = shared_ctx;
	ctx->type = type;

	regmap = devm_regmap_init(dev, &sec_pmic_acpm_regmap_bus, ctx, cfg);
	if (IS_ERR(regmap))
		return dev_err_cast_probe(dev, regmap, "regmap init (%s) failed\n", cfg->name);

	if (do_attach) {
		int ret;

		ret = regmap_attach_dev(dev, regmap, cfg);
		if (ret)
			return dev_err_ptr_probe(dev, ret, "regmap attach (%s) failed\n",
						 cfg->name);
	}

	return regmap;
}

static int sec_pmic_acpm_probe(struct platform_device *pdev)
{
	struct regmap *regmap_common, *regmap_pmic, *regmap;
	const struct sec_pmic_acpm_platform_data *pdata;
	struct sec_pmic_acpm_shared_bus_context *shared_ctx;
	struct acpm_handle *acpm;
	struct device *dev = &pdev->dev;
	int ret, irq;

	pdata = device_get_match_data(dev);
	if (!pdata)
		return dev_err_probe(dev, -ENODEV, "unsupported device type\n");

	acpm = devm_acpm_get_by_node(dev, dev->parent->of_node);
	if (IS_ERR(acpm))
		return dev_err_probe(dev, PTR_ERR(acpm), "failed to get acpm\n");

	if (pdata->no_irq) {
		irq = 0;
	} else {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0)
			return irq;
	}

	shared_ctx = devm_kzalloc(dev, sizeof(*shared_ctx), GFP_KERNEL);
	if (!shared_ctx)
		return -ENOMEM;

	shared_ctx->acpm = acpm;
	shared_ctx->acpm_chan_id = pdata->acpm_chan_id;
	shared_ctx->speedy_channel = pdata->speedy_channel;

	regmap_common = sec_pmic_acpm_regmap_init(dev, shared_ctx, SEC_PMIC_ACPM_ACCESSTYPE_COMMON,
						  pdata->regmap_cfg_common, true);
	if (IS_ERR(regmap_common))
		return PTR_ERR(regmap_common);

	regmap_pmic = sec_pmic_acpm_regmap_init(dev, shared_ctx, SEC_PMIC_ACPM_ACCESSTYPE_PMIC,
						pdata->regmap_cfg_pmic, false);
	if (IS_ERR(regmap_pmic))
		return PTR_ERR(regmap_pmic);

	if (pdata->regmap_cfg_rtc) {
		regmap = sec_pmic_acpm_regmap_init(dev, shared_ctx, SEC_PMIC_ACPM_ACCESSTYPE_RTC,
						   pdata->regmap_cfg_rtc, true);
		if (IS_ERR(regmap))
			return PTR_ERR(regmap);
	}

	if (pdata->regmap_cfg_meter) {
		regmap = sec_pmic_acpm_regmap_init(dev, shared_ctx, SEC_PMIC_ACPM_ACCESSTYPE_METER,
						   pdata->regmap_cfg_meter, true);
		if (IS_ERR(regmap))
			return PTR_ERR(regmap);
	}

	if (pdata->regmap_cfg_gpio) {
		regmap = sec_pmic_acpm_regmap_init(dev, shared_ctx, SEC_PMIC_ACPM_ACCESSTYPE_GPIO,
						   pdata->regmap_cfg_gpio, true);
		if (IS_ERR(regmap))
			return PTR_ERR(regmap);
	}

	ret = sec_pmic_probe(dev, pdata->device_type, irq, regmap_pmic, NULL);
	if (ret)
		return ret;

	if (device_property_read_bool(dev, "wakeup-source"))
		devm_device_init_wakeup(dev);

	return 0;
}

static void sec_pmic_acpm_shutdown(struct platform_device *pdev)
{
	sec_pmic_shutdown(&pdev->dev);
}

static const struct sec_pmic_acpm_platform_data s2mpg10_data = {
	.device_type = S2MPG10,
	.acpm_chan_id = 2,
	.speedy_channel = 0,
	.regmap_cfg_common = &s2mpg10_regmap_config_common,
	.regmap_cfg_pmic = &s2mpg10_regmap_config_pmic,
	.regmap_cfg_rtc = &s2mpg10_regmap_config_rtc,
	.regmap_cfg_meter = &s2mpg10_regmap_config_meter,
};

static const struct sec_pmic_acpm_platform_data s2mpg11_data = {
	.device_type = S2MPG11,
	.acpm_chan_id = 2,
	.speedy_channel = 1,
	.regmap_cfg_common = &s2mpg11_regmap_config_common,
	.regmap_cfg_pmic = &s2mpg11_regmap_config_pmic,
	.regmap_cfg_meter = &s2mpg11_regmap_config_meter,
};

static const struct sec_pmic_acpm_platform_data s2mpg12_data = {
	.device_type = S2MPG12,
	.acpm_chan_id = 2,
	.speedy_channel = 0,
	.regmap_cfg_common = &s2mpg12_regmap_config_common,
	.regmap_cfg_pmic = &s2mpg12_regmap_config_pmic,
	.regmap_cfg_rtc = &s2mpg12_regmap_config_rtc,
	.regmap_cfg_meter = &s2mpg12_regmap_config_meter,
};

static const struct sec_pmic_acpm_platform_data s2mpg13_data = {
	.device_type = S2MPG13,
	.acpm_chan_id = 2,
	.speedy_channel = 1,
	.regmap_cfg_common = &s2mpg13_regmap_config_common,
	.regmap_cfg_pmic = &s2mpg13_regmap_config_pmic,
	.regmap_cfg_gpio = &s2mpg13_regmap_config_gpio,
	.no_irq = true,
};

static const struct sec_pmic_acpm_platform_data s2mpg14_data = {
	.device_type = S2MPG14,
	.acpm_chan_id = 2,
	.speedy_channel = 0,
	.regmap_cfg_common = &s2mpg14_regmap_config_common,
	.regmap_cfg_pmic = &s2mpg14_regmap_config_pmic,
	.no_irq = true,
};

static const struct sec_pmic_acpm_platform_data s2mpg15_data = {
	.device_type = S2MPG15,
	.acpm_chan_id = 2,
	.speedy_channel = 1,
	.regmap_cfg_common = &s2mpg15_regmap_config_common,
	.regmap_cfg_pmic = &s2mpg15_regmap_config_pmic,
	.no_irq = true,
};

static const struct of_device_id sec_pmic_acpm_of_match[] = {
	{ .compatible = "samsung,s2mpg10-pmic", .data = &s2mpg10_data, },
	{ .compatible = "samsung,s2mpg11-pmic", .data = &s2mpg11_data, },
	{ .compatible = "samsung,s2mpg12-pmic", .data = &s2mpg12_data, },
	{ .compatible = "samsung,s2mpg13-pmic", .data = &s2mpg13_data, },
	{ .compatible = "samsung,s2mpg14-pmic", .data = &s2mpg14_data, },
	{ .compatible = "samsung,s2mpg15-pmic", .data = &s2mpg15_data, },
	{ },
};
MODULE_DEVICE_TABLE(of, sec_pmic_acpm_of_match);

static struct platform_driver sec_pmic_acpm_driver = {
	.driver = {
		.name = "sec-pmic-acpm",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.pm = pm_sleep_ptr(&sec_pmic_pm_ops),
		.of_match_table = sec_pmic_acpm_of_match,
	},
	.probe = sec_pmic_acpm_probe,
	.shutdown = sec_pmic_acpm_shutdown,
};
module_platform_driver(sec_pmic_acpm_driver);

MODULE_AUTHOR("André Draszik <andre.draszik@linaro.org>");
MODULE_DESCRIPTION("ACPM driver for the Samsung S2MPG1x");
MODULE_LICENSE("GPL");
