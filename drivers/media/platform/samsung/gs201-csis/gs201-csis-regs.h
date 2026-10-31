/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google gs201 CSIS link and CSIS WDMA registers
 *
 * Sources, in order of confidence:
 *   [GPL]  Google's GPL gs201 device tree (lwis CSIS/DMA nodes: register
 *          offsets, interrupt mask/source pairs) and its lwis gs201 csi.h
 *          (interrupt bit names);
 *   [IMX]  the same receiver's older version in imx-mipi-csis.c, for link
 *          bits the GPL sources don't name;
 *   [HAL]  facts observed in the stock camera HAL's own register writes,
 *          described in CAMERA-PLAN.md section 4.
 * Unconfirmed values are marked [I] and are checked by the CAM3b/CAM4
 * runbooks before anything streams.
 */

#ifndef GS201_CSIS_REGS_H
#define GS201_CSIS_REGS_H

#include <linux/bitfield.h>
#include <linux/bits.h>

/* --- CSIS link (one 4 KiB window per link) --- */

#define CSIS_LINK_PHYS(n)		(0x1a440000 + (n) * 0x10000)	/* [GPL] */

#define CSIS_VERSION			0x00	/* 0x05040000 on gs201 */
#define CSIS_CMN_CTRL			0x04
#define CSIS_CMN_CTRL_CSI_EN		BIT(0)			/* [IMX][HAL] */
#define CSIS_CMN_CTRL_SW_RESET		BIT(1)			/* [IMX] */
#define CSIS_CMN_CTRL_LANE_NUMBER	GENMASK(9, 8)		/* [IMX][HAL] */
#define CSIS_CMN_CTRL_INTERLEAVE	GENMASK(11, 10)		/* [HAL]: 3 */
#define CSIS_CMN_CTRL_F13		GENMASK(15, 13)		/* [HAL]: 2, meaning unknown */
#define CSIS_CMN_CTRL_CPHY		BIT(21)			/* [HAL] */
#define CSIS_UPD_SDW			0x0c
#define CSIS_UPD_SDW_ALL		GENMASK(3, 0)		/* [HAL] ch0-3 */

/* Interrupt mask (1 = enabled) and source (write 1 to clear) [GPL] */
#define CSIS_INT_MSK0			0x10
#define CSIS_INT_SRC0			0x14
#define CSIS_INT_MSK1			0x18
#define CSIS_INT_SRC1			0x1c
#define CSIS_INT_MSK_FS			0x20	/* frame start, per VC */
#define CSIS_INT_SRC_FS			0x24
#define CSIS_INT_MSK_FE			0x28	/* frame end, per VC */
#define CSIS_INT_SRC_FE			0x2c

/* INT0: errors [GPL] */
#define CSIS_INT0_ERRORS		(GENMASK(6, 0) | GENMASK(23, 12))
#define  CSIS_INT0_ERR_WRONG_CONFIG	BIT(3)	/* a packet no channel takes */

/* INT1: frame events [GPL] */
#define CSIS_INT1_LINE_END		BIT(0)
#define CSIS_INT1_FRAME_END		BIT(1)
#define CSIS_INT1_FRAME_START		BIT(2)
#define CSIS_INT1_ERR_LOST_FE		BIT(3)
#define CSIS_INT1_ERR_LOST_FS		BIT(4)
#define CSIS_INT1_HRESOL_MISMATCH	BIT(5)
#define CSIS_INT1_VRESOL_MISMATCH	BIT(6)
#define CSIS_INT1_ERRORS		GENMASK(6, 3)

/* Per-channel ISP configuration and resolution [GPL offsets, HAL fields] */
#define CSIS_ISP_CONFIG_CH(n)		(0x40 + (n) * 0x10)
#define CSIS_ISPCFG_DATAFORMAT		GENMASK(7, 2)
#define CSIS_ISPCFG_DT_UNUSED		0x3f
#define CSIS_ISPCFG_PIXEL_MODE		GENMASK(13, 12)
#define CSIS_ISPCFG_PIXEL_MODE_QUAD	2
#define CSIS_ISPCFG_VC			GENMASK(20, 16)
#define CSIS_ISPCFG_VC_PARKED		15	/* unused channels: no VC a sensor sends */
#define CSIS_ISP_RESOL_CH(n)		(0x44 + (n) * 0x10)
#define CSIS_ISP_RESOL_V		GENMASK(31, 16)
#define CSIS_ISP_RESOL_H		GENMASK(15, 0)
#define CSIS_FRM_CNT_CH(n)		(0x500 + (n) * 4)	/* read only */
#define CSIS_LRTE_CONFIG		0x600
#define CSIS_LRTE_DEFAULT		0x7fff			/* [HAL] */
#define CSIS_LRTE_EPD_EN		BIT(31)
#define CSIS_LRTE_SPACERS_RESET		0x7fff7fff		/* both spacer fields */
#define CSIS_DBG_OPTION_SUITE		0x690
/*
 * [I] the last packet header the link received: word count (or frame
 * number) in 31:16, data identifier (VC 7:6, data type 5:0) in 15:8; read
 * 0x09d82b00 with the main camera's RAW10 lines (2520 bytes) arriving and
 * 0x00014100 after a VC 1 frame end
 */
#define CSIS_DBG_LAST_PKT		0x6a0
#define CSIS_DBG_OPTION_BIT23		BIT(23)			/* [HAL], meaning unknown */
#define CSIS_PHY_CMN_CTRL		0x704
#define CSIS_PHY_CMN_CTRL_ENABLE	GENMASK(4, 0)		/* [IMX] at 0x24 there */

/* --- CSIS DMA block: common page [HAL values, written once] --- */

#define CSIS_DMA_CMN_CTRL		0x08
#define CSIS_DMA_CMN_CTRL_SW_RESET	BIT(0)
#define CSIS_DMA_CMN_CTRL_VAL		0x2	/* IP processing */
#define CSIS_DMA_CMN_CLK_CTRL		0x14
#define CSIS_DMA_CMN_CLK_CTRL_VAL	0x1
#define CSIS_DMA_CMN_CFG_CSIS1		0x48
#define CSIS_DMA_CMN_CFG_CSIS1_VAL	0x100
#define CSIS_DMA_CMN_DEBUG_EN		0x100	/* [I] measured by another bring-up */
#define CSIS_DMA_CMN_DBG_VCNT(ctx)	(0x144 + 0x20 * (ctx))	/* frames */
#define CSIS_DMA_CMN_DBG_HDCNT(ctx)	(0x154 + 0x20 * (ctx))	/* lines 31:16, data 15:0 */
#define CSIS_DMA_CMN_QURGENT_EN		0xa0
#define CSIS_DMA_CMN_QURGENT_EN_VAL	0x3

/* --- CSIS WDMA context (4 KiB at 0x7000 + 0x1000 * ctx) --- */

/* per channel, at 0x100 * ch [HAL] */
#define CSIS_WDMA_CH(ch, reg)		((ch) * 0x100 + (reg))
#define CSIS_WDMA_CTRL			0x000
#define CSIS_WDMA_CTRL_EN		BIT(0)
#define CSIS_WDMA_CTRL_ON		0xf83	/* enable | bit 1 | [11:7] = 31 */
#define CSIS_WDMA_CTRL_OFF		0xf82
#define CSIS_WDMA_FMT			0x004
#define CSIS_WDMA_FMT_CODE		GENMASK(4, 0)
#define CSIS_WDMA_FMT_RAW8		0	/* 1 byte/pixel */
#define CSIS_WDMA_FMT_RAW16		6	/* 16-bit container */
#define CSIS_WDMA_FMT_RAW10		7	/* MIPI packed, 1.25 bytes/pixel */
#define CSIS_WDMA_SKIP			0x008
#define CSIS_WDMA_ADDR(n)		(0x010 + 4 * ((n) - 1))	/* n = 1..32 */
#define CSIS_WDMA_ADDR_BITS		32
#define CSIS_WDMA_RESOL			0x0a4
#define CSIS_WDMA_RESOL_H		GENMASK(14, 0)
#define CSIS_WDMA_RESOL_V		GENMASK(30, 16)	/* [I] */
#define CSIS_WDMA_STRIDE		0x0a8
#define CSIS_WDMA_FCNT_SEQ		0x0b0	/* bitmask of the slots in use */
/* per context [GPL, HAL] */
#define CSIS_WDMA_DATA_CTRL		0x400
#define CSIS_WDMA_INT_MSK		0x404
#define CSIS_WDMA_INT_SRC		0x408
#define CSIS_WDMA_FRO_FRAME_NUM		0x420
#define CSIS_WDMA_SBWC_CTRL		0x440
#define CSIS_WDMA_BW_LIMIT(n)		(0x560 + 4 * (n))	/* [HAL] names, meaning [I] */

/* WDMA interrupt bits [GPL] */
#define CSIS_WDMA_INT_LINE_END(ch)	BIT(0 + (ch))
#define CSIS_WDMA_INT_FS(ch)		BIT(4 + (ch))
#define CSIS_WDMA_INT_FE(ch)		BIT(8 + (ch))
#define CSIS_WDMA_INT_FIFO_FULL		BIT(12)
#define CSIS_WDMA_INT_LASTDATA_ERR	BIT(14)
#define CSIS_WDMA_INT_LASTADDR_ERR	BIT(15)
#define CSIS_WDMA_INT_ABORT_DONE	BIT(16)
#define CSIS_WDMA_INT_OVERLAP(ch)	BIT(20 + (ch))
#define CSIS_WDMA_INT_FRAME_DROP(ch)	BIT(24 + (ch))
#define CSIS_WDMA_INT_ERRORS		(CSIS_WDMA_INT_FIFO_FULL | \
					 CSIS_WDMA_INT_LASTDATA_ERR | \
					 CSIS_WDMA_INT_LASTADDR_ERR | \
					 GENMASK(27, 20))

/* --- EBUF (0x1a4c0000), between the links and the WDMA [GPL address] --- */

#define CSIS_EBUF_CTRL			0x00
#define CSIS_EBUF_CTRL_BYPASS		BIT(0)	/* [I] measured with the sensor streaming */

/* --- CSIS system registers (sysreg_csis, 0x1a420000) [HAL] --- */

/*
 * The stock software's "SC_CON0-2" ("PDP mux", 3 bits each). With all three
 * at 0 the WDMA gets frame and line syncs but no pixel data; any non-zero
 * value lets the data through [I, measured].
 */
#define CSIS_SYSREG_SC_CON(n)		(0x408 + 4 * (n))
#define CSIS_SYSREG_SC_CON_NUM		3
#define CSIS_SYSREG_SC_CON_PASS		1
/* WDMA slot n takes its input from link (value) */
#define CSIS_SYSREG_EBUF_IN(ebuf)	(0x430 + 4 * (ebuf))
/* stock sets bit (link + 8 * PDP channel) here, masked [HAL]; meaning [I] */
#define CSIS_SYSREG_LINK_SEL		0x488
/*
 * One bit per DC-PHY slave, named "reset" in Google's GPL DT; the PHY driver
 * owns it, this driver only logs it.
 */
#define CSIS_SYSREG_PHY_RESET		0x500

#endif /* GS201_CSIS_REGS_H */
