/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Maxim MAX77779 interface PMIC: the PMIC block's registers
 *
 * Addresses as in Google's max77779_v20240115_regs.h (google-modules/bms).
 */

#ifndef __LINUX_MFD_MAX77779_H
#define __LINUX_MFD_MAX77779_H

#define MAX77779_PMIC_ID		0x00
#define MAX77779_PMIC_ID_VALUE		0x79
#define MAX77779_PMIC_REVISION		0x01
#define MAX77779_PMIC_INTSRC_STS	0x22	/* write 1 to clear */
#define MAX77779_PMIC_INTB_MASK		0x24	/* 1 = masked */

#define MAX77779_SGPIO_INT		0xe8	/* write 1 to clear */
#define MAX77779_SGPIO_PU		0xe9
#define MAX77779_SGPIO_PD		0xea
#define MAX77779_SGPIO_CNFG(n)		(0xed + (n))

#define MAX77779_N_SGPIO		8

#endif /* __LINUX_MFD_MAX77779_H */
