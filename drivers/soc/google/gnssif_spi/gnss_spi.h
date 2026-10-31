/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022, Samsung Electronics.
 *
 */

#ifndef __GNSS_SPI_H__
#define __GNSS_SPI_H__

#include <linux/spi/spi.h>
#include <linux/mutex.h>

#define DEFAULT_SPI_RX_SIZE	64
#define MAX_SPI_RX_SIZE	SZ_2K
#define DEFAULT_SPI_TX_SIZE	SZ_4K
/*
 * omamobile: stock sends 32-bit words with the vendor SPI driver's
 * "swap-mode" (byte and half-word swap), which puts bytes on the wire in
 * memory order; mainline spi-s3c64xx has no swap mode, and 8-bit words give
 * that order.
 */
#define SPI_BITS_PER_WORD	8

struct gnss_spi {
	struct spi_device *spi;
	struct mutex lock;
};

static struct gnss_spi gnss_if;

extern int gnss_spi_send(char *buff, unsigned int size, char *recv_buff);
extern int gnss_spi_recv(char *buff, unsigned int size);

#endif /* __GNSS_SPI_H__ */
