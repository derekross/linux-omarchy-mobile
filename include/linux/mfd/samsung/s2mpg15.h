/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * S2MPG15 register definitions derived from Google's GPL Samsung driver.
 * Copyright (C) 2022 Samsung Electronics
 */
#ifndef __LINUX_MFD_S2MPG15_H
#define __LINUX_MFD_S2MPG15_H

#define S2MPG15_COMMON_CHIPID		0x0b
#define S2MPG15_PMIC_L5S_CTRL		0x33
#define S2MPG15_PMIC_L7S_CTRL		0x35
#define S2MPG15_PMIC_L9S_CTRL		0x37
#define S2MPG15_PMIC_L10S_CTRL		0x38
#define S2MPG15_PMIC_L11S_CTRL		0x39
#define S2MPG15_PMIC_L12S_CTRL		0x3a
#define S2MPG15_PMIC_L15S_CTRL		0x3d

/* GPIO registers (access type 0x0c): INT1-3 clear on read, then masks, status, SET */
enum s2mpg15_gpio_reg {
	S2MPG15_GPIO_INT1,
	S2MPG15_GPIO_INT2,
	S2MPG15_GPIO_INT3,
	S2MPG15_GPIO_INT1M,
	S2MPG15_GPIO_INT2M,
	S2MPG15_GPIO_INT3M,
	S2MPG15_GPIO_STATUS1,
	S2MPG15_GPIO_STATUS2,
	S2MPG15_GPIO_0_SET,
	S2MPG15_GPIO_9_SET = S2MPG15_GPIO_0_SET + 9,
};

#define S2MPG15_GPIO_NUM		10

#endif /* __LINUX_MFD_S2MPG15_H */
