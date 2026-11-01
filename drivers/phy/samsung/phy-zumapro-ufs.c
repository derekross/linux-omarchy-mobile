// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 *
 * Google Tensor G4 (zumapro) UFS PHY calibration.
 *
 * Derived from Google's GPL zuma/ufs-cal.h. The EVT0 and EVT1 PMA tables
 * are identical.
 */

#include <linux/iopoll.h>

#include "phy-samsung-ufs.h"

#define ZUMAPRO_LANE_STRIDE	0x800
#define ZUMAPRO_CAL_DONE		0xc74

/* Offsets in the Samsung PHY configuration macros are 32-bit words. */
static const struct samsung_ufs_phy_cfg zumapro_pre_init[] = {
	PHY_COMN_REG_CFG(0x050, 0x08, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x005, 0x19, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x00b, 0x44, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x00c, 0xc4, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x00d, 0xc3, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x00f, 0x88, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x016, 0x1a, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x019, 0x04, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x054, 0x88, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x067, 0x4c, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x068, 0x4c, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_OFFSET(0x201, 0x44, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x202, 0x44, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x203, 0x00, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x204, 0x18, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x205, 0xc0, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x207, 0x1c, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x2ec, 0x8c, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x27c, 0xd0, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x288, 0xfa, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x289, 0x60, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x234, 0x30, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x239, 0x05, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x23d, 0x05, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x24d, 0x1a, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x24e, 0x12, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x24f, 0x5e, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x259, 0x2a, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x260, 0x54, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x266, 0x54, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x273, 0x00, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x274, 0x00, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x2ab, 0x00, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x2ac, 0x02, PWR_MODE_ANY, 0x200),
	PHY_COMN_REG_CFG(0x050, 0x0c, PWR_MODE_ANY),
	PHY_COMN_REG_CFG(0x050, 0x00, PWR_MODE_ANY),
	END_UFS_PHY_CFG,
};

/* Google's post_h8_enter: after the link has entered Hibern8. */
static const struct samsung_ufs_phy_cfg zumapro_post_h8_enter[] = {
	PHY_TRSV_REG_CFG_OFFSET(0x27d, 0x08, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x280, 0x3a, PWR_MODE_ANY, 0x200),
	PHY_COMN_REG_CFG(0x000, 0x51, PWR_MODE_ANY),
	PHY_TRSV_REG_CFG_OFFSET(0x2d9, 0x30, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x2d9, 0x33, PWR_MODE_ANY, 0x200),
	END_UFS_PHY_CFG,
};

/* Google's pre_h8_exit: before the link leaves Hibern8. */
static const struct samsung_ufs_phy_cfg zumapro_pre_h8_exit[] = {
	PHY_COMN_REG_CFG(0x000, 0x11, PWR_MODE_ANY),
	PHY_DELAY_US_CFG(10),
	PHY_TRSV_REG_CFG_OFFSET(0x27d, 0x00, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x280, 0x30, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x2d9, 0x32, PWR_MODE_ANY, 0x200),
	PHY_TRSV_REG_CFG_OFFSET(0x2d9, 0x22, PWR_MODE_ANY, 0x200),
	END_UFS_PHY_CFG,
};

static const struct samsung_ufs_phy_cfg *zumapro_cfgs[CFG_TAG_MAX] = {
	[CFG_PRE_INIT] = zumapro_pre_init,
};

static const struct samsung_ufs_phy_cfg *zumapro_hibern8_cfgs[] = {
	[CFG_POST_HIBERN8_ENTER]	= zumapro_post_h8_enter,
	[CFG_PRE_HIBERN8_EXIT]		= zumapro_pre_h8_exit,
};

static const char * const zumapro_clks[] = { "ref_clk" };

static int zumapro_wait_for_cal(struct phy *phy, u8 lane)
{
	struct samsung_ufs_phy *ufs_phy = get_samsung_ufs_phy(phy);
	u32 offset = ZUMAPRO_CAL_DONE + lane * ZUMAPRO_LANE_STRIDE;
	u32 val;
	int ret;

	ret = readl_poll_timeout(ufs_phy->reg_pma + offset, val, val & BIT(0),
				 40, 4000);
	if (ret)
		dev_err(ufs_phy->dev, "lane %u calibration timed out\n", lane);

	return ret;
}

const struct samsung_ufs_phy_drvdata tensor_zumapro_ufs_phy = {
	.cfgs = zumapro_cfgs,
	.cfgs_hibern8 = zumapro_hibern8_cfgs,
	.isol = {
		.offset = 0x3ec0,
		.mask = BIT(0),
		.en = BIT(0),
	},
	.clk_list = zumapro_clks,
	.num_clks = ARRAY_SIZE(zumapro_clks),
	.wait_for_cal = zumapro_wait_for_cal,
};
