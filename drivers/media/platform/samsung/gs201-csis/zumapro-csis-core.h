/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * zumapro (Tensor G4) CSIS core: the image logical channel (LoCH) path that
 * writes a CSI-2 link's frames to memory. Offsets are from the ISP front
 * end's CSIS core page (isp-fe 0x1c300000 + 0x20000). Names follow the
 * Pixel 9a camera HAL's register table; the fields come from the HAL's own
 * value packers. The interrupt registers (image processing, error, abort
 * done, the aggregate, and LoCH 1 at 0x20824) are also in Google's GPL
 * zuma-isp-event-info.dtsi, and their bits in
 * dt-bindings/lwis/platform/zuma/isp_fe.h.
 */
#ifndef ZUMAPRO_CSIS_CORE_H
#define ZUMAPRO_CSIS_CORE_H

#include <linux/bits.h>

/* common page */
#define ZP_LOCH_INT_ISR			0x008c	/* bit n = LoCH n */
#define ZP_LOCH_INT_IMR			0x0094	/* 1 passes the LoCH's interrupt */
#define ZP_LOCH_EN			0x009c
#define ZP_LOCH_EN_SET			0x00a0
#define ZP_LOCH_EN_CLR			0x00a4
#define ZP_LOCH_ABORT			0x00ac
#define ZP_FRAME_ID_CLR			0x00b0

/* image LoCH n (0-6) */
#define ZP_IMG(n, r)			(0x0400 + 0x400 * (n) + (r))
#define ZP_IMG_ABORT_DONE_ISR		0x004
#define  ZP_IMG_ABORT_DONE		BIT(0)
#define ZP_IMG_WDMA_CONTEXT_ENABLE	0x018
#define ZP_IMG_WDMA_CONFIG_ENABLE	0x01c
#define ZP_IMG_WDMA_CONFIG_MISS		0x020
#define ZP_IMG_PROC_ISR			0x024
#define ZP_IMG_PROC_IER			0x02c
#define ZP_IMG_PROC_IMR			0x030
#define  ZP_IMG_PROC_SOF		BIT(0)
#define  ZP_IMG_PROC_EOF		BIT(1)
#define  ZP_IMG_PROC_SHADOW_UPDATE	BIT(3)
#define  ZP_IMG_PROC_DMA_DONE		BIT(4)
#define  ZP_IMG_PROC_FRAME_DONE		BIT(5)
#define ZP_IMG_ERR_ISR			0x038
#define ZP_IMG_ERR_IMR			0x044
#define  ZP_IMG_ERRORS			GENMASK(13, 0)
#define ZP_IMG_BATCH_INT_MSK_SOF	0x060
#define ZP_IMG_BATCH_INT_MSK_EOF	0x064
#define ZP_IMG_BATCH_INT_MSK_DMA_DONE	0x06c
#define ZP_IMG_BATCH_INT_MSK_FRAME_DONE	0x070
#define ZP_IMG_ISP_RESOL		0x074
#define  ZP_IMG_ISP_RESOL_W		GENMASK(15, 0)
#define  ZP_IMG_ISP_RESOL_H		GENMASK(31, 16)
#define ZP_IMG_AUTO_REC_SYNC_INTV0	0x078
#define ZP_IMG_AUTO_REC_SYNC_INTV1	0x07c
#define ZP_IMG_BATCH_CTL		0x088
#define ZP_IMG_CTL			0x08c
#define  ZP_IMG_CTL_DT			GENMASK(11, 8)
#define  ZP_IMG_CTL_LINK_CH		GENMASK(15, 12)
#define  ZP_IMG_CTL_LINK		GENMASK(27, 24)
#define ZP_IMG_FRAME_ID			0x09c
#define ZP_IMG_LOCH_EN_STATUS		0x0a8	/* the link in the low bits */
#define  ZP_IMG_LOCH_EN_STATUS_ACTIVE	BIT(8)
#define ZP_IMG_SQ_CONFIG_ID_LSB		0x108
#define ZP_IMG_SQ_CONFIG_ID_MSB		0x10c
#define ZP_IMG_SQ_BASE_ADDR_LSB		0x110
#define ZP_IMG_SQ_BASE_ADDR_MSB		0x114
#define ZP_IMG_BAYER_ENABLE		0x140
#define ZP_IMG_BAYER_CROP_EN		0x144
#define ZP_IMG_BAYER_MEM_FORMAT		0x150
#define ZP_IMG_BAYER_STRIDE		0x154
#define ZP_IMG_BATCH_ADDR_LSB(i)	(0x1c0 + 8 * (i))
#define ZP_IMG_BATCH_ADDR_MSB(i)	(0x1c4 + 8 * (i))
#define  ZP_IMG_ADDR_LSB_MASK		GENMASK(31, 5)
#define  ZP_IMG_ADDR_MSB_MASK		GENMASK(3, 0)
#define ZP_IMG_WDMA_DBG_INFO		0x2d0
#define ZP_IMG_AXI_RESP_ERR_STATUS	0x2d4
#define ZP_IMG_ACTIVITY_STATUS		0x2e4
#define ZP_IMG_DONE_FRAME_ID		0x2f0

/* ZP_IMG_CTL_DT, from the CSI-2 data type */
#define ZP_DT_RAW8			0
#define ZP_DT_RAW10			2
#define ZP_DT_RAW12			4
#define ZP_DT_RAW14			6

/* ZP_IMG_BAYER_MEM_FORMAT */
#define ZP_MEM_FMT_8BIT			0
#define ZP_MEM_FMT_RAW16		2
#define ZP_MEM_FMT_RAW10_PACKED		4	/* as on the CSI-2 bus: 4 pixels in 5 bytes */
#define ZP_MEM_FMT_RAW12_PACKED		5

#define ZP_ADDR_BITS			36

/* the link's CMN_CTRL bits 15:12, as the zumapro HAL writes them */
#define ZP_LINK_CMN_CTRL_F12		BIT(12)
#define ZP_LINK_PIXEL_MODE		3

#endif
