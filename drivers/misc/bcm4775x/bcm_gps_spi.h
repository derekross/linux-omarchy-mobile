/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2015 Broadcom Corporation
 *
 * The Broadcom GPS SPI driver: SSI framing for the BCM4775x GNSS receivers.
 * Based on Google's gs201 bcm_gps_spi.h.
 */

#ifndef __BCM_GPS_SPI_H__
#define __BCM_GPS_SPI_H__

#include <linux/atomic.h>
#include <linux/circ_buf.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "bbd.h"

#define DELAY_FOR_SYSTEM_OVERLOADED_MS	100
#define READ_SIZE_FOR_SYSTEM_OVERLOADED	1024

#define SSI_MODE_STREAM		0x00
#define SSI_MODE_DEBUG		0x80

#define SSI_MODE_HALF_DUPLEX	0x00
#define SSI_MODE_FULL_DUPLEX	0x40

#define SSI_WRITE_TRANS		0x00
#define SSI_READ_TRANS		0x20

#define SSI_PCKT_1B_LENGTH	0
#define SSI_PCKT_2B_LENGTH	0x10

#define SSI_FLOW_CONTROL_DISABLED	0
#define SSI_FLOW_CONTROL_ENABLED	0x08

#define HSI_F_MOSI_CTRL_CNT_SHIFT	0
#define HSI_F_MOSI_CTRL_CNT_MASK	0x07
#define HSI_F_MOSI_CTRL_SZE_SHIFT	3
#define HSI_F_MOSI_CTRL_SZE_MASK	0x18
#define HSI_F_MOSI_CTRL_PE_MASK		0x40

/* Size of the chip's receive buffer, bytes */
#define HSI_PZC_MAX_RX_BUFFER		6144

/* BCM477x host interface registers */
#define HSI_ERROR_STATUS		0x2c
#define HSI_STATUS			0x30
#define HSI_RNGDMA_RX_SW_ADDR_OFFSET	0x40104050

struct bcm_spi_strm_protocol {
	int pckt_len;
	int fc_len;
	int ctrl_len;
	unsigned char ctrl_byte;
	unsigned short frame_len;
} __packed;

#define BCM_SPI_READ_BUF_SIZE	(16 * 4096)
#define BCM_SPI_WRITE_BUF_SIZE	(16 * 4096)

#define MAX_SPI_DREG_FRAME_LEN	254
#define MAX_SPI_FRAME_LEN	(1024 * 8)

struct bcm_ssi_tx_frame {
	unsigned char cmd;
	unsigned char data[MAX_SPI_FRAME_LEN - 1];
} __packed;

struct bcm_ssi_rx_frame {
	unsigned char status;
	unsigned char data[MAX_SPI_FRAME_LEN - 1];
} __packed;

#define BCM_NUM_SUPPLIES	3

struct bcm_spi_priv {
	struct spi_device *spi;

	/* /dev/ttyBCM */
	struct miscdevice misc;
	bool busy;
	struct circ_buf read_buf;
	struct circ_buf write_buf;
	struct mutex rlock;			/* read_buf */
	struct mutex wlock;			/* write_buf */
	char _read_buf[BCM_SPI_READ_BUF_SIZE];
	char _write_buf[BCM_SPI_WRITE_BUF_SIZE];
	wait_queue_head_t poll_wait;

	/* Lines */
	struct gpio_desc *host_req;
	struct gpio_desc *mcu_req;
	struct gpio_desc *mcu_resp;
	struct gpio_desc *nstandby;
	int irq;

	/* Supplies, on while /dev/ttyBCM is open (or for a debugfs probe) */
	struct regulator_bulk_data supplies[BCM_NUM_SUPPLIES];
	struct mutex power_lock;
	bool powered;

	/* IRQ and its control */
	atomic_t irq_enabled;
	spinlock_t irq_lock;

	/* Work */
	struct work_struct rxtx_work;
	struct workqueue_struct *serial_wq;
	atomic_t suspending;

	/* SPI tx/rx frames */
	struct bcm_ssi_tx_frame *tx_buf;
	struct bcm_ssi_rx_frame *rx_buf;

	/* SPI streaming protocol */
	struct bcm_spi_strm_protocol tx_strm;
	struct bcm_spi_strm_protocol rx_strm;

	unsigned long packet_received;
	unsigned long skip_count;
	unsigned long last_tick;
	int ssi_tx_fail;
	int ssi_tx_pzc_retries;
	int ssi_tx_pzc_retry_delays;
	unsigned long rx_buffer_avail_bytes;

	struct bbd_debug dbg;
	struct bbd_device *bbd;
	struct dentry *debugfs;
};

/* bcm_gps_regs.c */
int bcm_dreg_write(struct bcm_spi_priv *priv, const char *id, u8 offset,
		   u8 *buf, u8 size);
int bcm_dreg_read(struct bcm_spi_priv *priv, const char *id, u8 offset,
		  u8 *buf, u8 size);
int bcm_ireg_write(struct bcm_spi_priv *priv, const char *id, u32 regaddr,
		   u32 regval);
int bcm_ireg_read(struct bcm_spi_priv *priv, const char *id, u32 regaddr,
		  u32 *regval, s32 n);

/* bcm_gps_spi.c */
int bcm_spi_sync(struct bcm_spi_priv *priv, void *tx_buf, void *rx_buf,
		 int len);

#endif /* __BCM_GPS_SPI_H__ */
