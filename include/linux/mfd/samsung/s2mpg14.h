/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * S2MPG14 register definitions derived from Google's GPL Samsung driver.
 * Copyright (C) 2022 Samsung Electronics
 */
#ifndef __LINUX_MFD_S2MPG14_H
#define __LINUX_MFD_S2MPG14_H

#include <linux/bits.h>

#define S2MPG14_COMMON_CHIPID		0x0b
#define S2MPG14_COMMON_IBIM1		0x0f
/* S2MPG14_COMMON_IBIM1: in-band interrupt sources, 1 = masked */
#define S2MPG14_COMMON_IBIM1_PMIC	BIT(0)
#define S2MPG14_COMMON_IBIM1_METER	BIT(1)

/* The PMIC block's interrupt registers: INT1-INT5 clear on read, INT1M-INT5M mask */
#define S2MPG14_PMIC_INT1		0x00
#define S2MPG14_PMIC_INT5		0x04
#define S2MPG14_PMIC_INT1M		0x05
#define S2MPG14_PMIC_INT5M		0x09
#define S2MPG14_PMIC_STATUS2		0x0b
#define S2MPG14_PMIC_L4M_CTRL		0x2e
#define S2MPG14_PMIC_L8M_CTRL		0x32
#define S2MPG14_PMIC_L9M_CTRL		0x33
#define S2MPG14_PMIC_L19M_CTRL		0x3d
#define S2MPG14_PMIC_L25M_CTRL		0x43
#define S2MPG14_PMIC_LDO_CTRL1		0x44

/* S2MPG14_PMIC_STATUS2: the alarm flag sits where it does on S2MPG12 */
#define S2MPG14_STATUS2_RTCA0E		BIT(2)

/*
 * The RTC block has S2MPG10's register layout (S2MPG10_RTC_* in rtc.h):
 * CTRL at 0x00 through OSC_CTRL at 0x1b, then two nonces to 0x2b and
 * five scratch registers to 0x30.
 */

/*
 * The AP sees the PMIC's interrupt as a level through the VGPIO2AP sysreg
 * block, which latches it in INTC0_IPEND until written back.
 */
#define S2MPG14_VGPIO2AP_INTC0_IPEND	0x290

#endif /* __LINUX_MFD_S2MPG14_H */
