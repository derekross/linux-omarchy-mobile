/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Register definition file for Samsung MFC V15.x Interface (FIMV) driver
 *
 * MFC v15.6 is found in the Google GS201 SoC. Its register map is the v8+
 * map used by v12; only the differences used by this driver are listed here.
 */

#ifndef _REGS_MFC_V15_H
#define _REGS_MFC_V15_H

#include <linux/sizes.h>
#include "regs-mfc-v12.h"

/* Firmware base for the non-secure firmware; 0x0054 is the secure one */
#define S5P_FIMV_RISC_NONSECURE_BASE_ADDR_V15	0x0068
#define S5P_FIMV_MFC_VERSION_V15		0xF028
#define S5P_FIMV_FW_STATUS_INFO_V15		0xF0A4
#define S5P_FIMV_FW_STATUS_READY_V15		BIT(0)

/* MFCv15 variant defines */
#define MFC_VERSION_V15			0xF0
#define MFC_HW_VERSION_V15		0x15060000
#define MFC_NUM_PORTS_V15		1
#define MAX_FW_SIZE_V15			(SZ_1M)
#define MAX_CPB_SIZE_V15		(7 * SZ_1M)

/*
 * The firmware rejects every buffer below its own base address, so the
 * firmware and all the driver's private buffers share one region with the
 * firmware at its start.
 */
#define MFC_MEM_SIZE_V15		(64 * SZ_1M)

#endif /*_REGS_MFC_V15_H*/
