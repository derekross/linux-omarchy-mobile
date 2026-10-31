/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * S2MPG14 register definitions derived from Google's GPL Samsung driver.
 * Copyright (C) 2022 Samsung Electronics
 */
#ifndef __LINUX_MFD_S2MPG14_H
#define __LINUX_MFD_S2MPG14_H

#define S2MPG14_COMMON_CHIPID		0x0b
#define S2MPG14_PMIC_L4M_CTRL		0x2e
#define S2MPG14_PMIC_L8M_CTRL		0x32
#define S2MPG14_PMIC_L9M_CTRL		0x33
#define S2MPG14_PMIC_L19M_CTRL		0x3d
#define S2MPG14_PMIC_L25M_CTRL		0x43
#define S2MPG14_PMIC_LDO_CTRL1		0x44

/* GPIO registers (access type 0x0c): INT1-2 clear on read, then masks, status, SET */
enum s2mpg14_gpio_reg {
	S2MPG14_GPIO_INT1,
	S2MPG14_GPIO_INT2,
	S2MPG14_GPIO_INT1M,
	S2MPG14_GPIO_INT2M,
	S2MPG14_GPIO_STATUS,
	S2MPG14_GPIO_0_SET,
	S2MPG14_GPIO_5_SET = S2MPG14_GPIO_0_SET + 5,
};

#define S2MPG14_GPIO_NUM		6

#endif /* __LINUX_MFD_S2MPG14_H */
