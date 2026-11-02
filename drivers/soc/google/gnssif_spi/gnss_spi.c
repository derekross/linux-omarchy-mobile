// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Samsung Electronics.
 *
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/swab.h>

#include "include/gnss.h"
#include "gnss_spi.h"

static struct gnss_spi gnss_if;

/* gnssif binds only once the SPI device is there. */
bool gnss_spi_bound(void)
{
	return !!READ_ONCE(gnss_if.spi);
}
EXPORT_SYMBOL_GPL(gnss_spi_bound);

/*
 * Swap the bytes of each 32-bit word. Google's SPI controller driver
 * sends GNSS's 32-bit words in its swap mode (swap-mode in the board's
 * device tree: bytes and half-words swapped on TX and RX), which puts
 * them on the wire in memory order. spi-s3c64xx has no swap mode and
 * shifts each word out most significant byte first, so swap here.
 */
static void gnss_spi_swab(char *buf, unsigned int size)
{
	u32 *w = (u32 *)buf;
	unsigned int i;

	for (i = 0; i < size / 4; i++)
		w[i] = swab32(w[i]);
}

/*
 * One message of transfers of GNSS_SPI_BLOCK bytes at most. The core's
 * data comes in 64-byte blocks, and Google's driver reads one block per
 * transfer. spi-s3c64xx moves a transfer that fills its 64-byte FIFO in
 * one piece, but cuts a longer one into 60-byte pieces, and on the Pixel
 * 9a the cuts lost data; so writes go in blocks too.
 */
static int gnss_spi_xfer(char *tx_buf, char *rx_buf, unsigned int size)
{
	unsigned int n = DIV_ROUND_UP(size, GNSS_SPI_BLOCK);
	struct spi_transfer *t;
	struct spi_message msg;
	unsigned int i, off;
	int ret;

	if (!gnss_spi_bound())
		return -ENODEV;
	if (!size || size % sizeof(u32))
		return -EINVAL;

	t = kcalloc(n, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	if (tx_buf)
		gnss_spi_swab(tx_buf, size);

	spi_message_init(&msg);
	for (i = 0, off = 0; i < n; i++, off += GNSS_SPI_BLOCK) {
		t[i].len = min(size - off, GNSS_SPI_BLOCK);
		t[i].tx_buf = tx_buf ? tx_buf + off : NULL;
		t[i].rx_buf = rx_buf ? rx_buf + off : NULL;
		t[i].bits_per_word = SPI_BITS_PER_WORD;
		spi_message_add_tail(&t[i], &msg);
	}

	ret = spi_sync(gnss_if.spi, &msg);
	if (ret < 0)
		gif_err("spi_sync() error:%d\n", ret);

	/* Give the caller its buffer back as it was. */
	if (tx_buf)
		gnss_spi_swab(tx_buf, size);
	if (rx_buf && !ret)
		gnss_spi_swab(rx_buf, size);

	kfree(t);

	return ret;
}

int gnss_spi_send(char *buff, unsigned int size, char *rx_buff)
{
	return gnss_spi_xfer(buff, rx_buff, size);
}
EXPORT_SYMBOL_GPL(gnss_spi_send);

int gnss_spi_recv(char *buff, unsigned int size)
{
	return gnss_spi_xfer(NULL, buff, size);
}
EXPORT_SYMBOL_GPL(gnss_spi_recv);

static int gnss_spi_probe(struct spi_device *spi)
{
	int ret = 0;

	gif_info("+++\n");
	mutex_init(&gnss_if.lock);

	if (spi_setup(spi)) {
		gif_err("ERR! spi_setup fail\n");
		ret = -EINVAL;
		goto err_setup;
	}
	spi_set_drvdata(spi, &gnss_if);
	WRITE_ONCE(gnss_if.spi, spi);
	gif_info("---\n");

	return 0;

err_setup:
	mutex_destroy(&gnss_if.lock);

	return ret;
}

static void gnss_spi_remove (struct spi_device *spi)
{
	struct gnss_spi *gnss_if = spi_get_drvdata(spi);

	WRITE_ONCE(gnss_if->spi, NULL);
	mutex_destroy(&gnss_if->lock);
}

static const struct of_device_id gnss_spi_dt_match[] = {
	{ .compatible = "samsung,gnss-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, gnss_spi_dt_match);

static const struct spi_device_id gnss_spi_ids[] = {
	{ "gnss-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, gnss_spi_ids);

static struct spi_driver gnss_spi_driver = {
	.id_table = gnss_spi_ids,
	.probe = gnss_spi_probe,
	.remove = gnss_spi_remove,
	.driver = {
		.name = "gnss_spi",
		.of_match_table = of_match_ptr(gnss_spi_dt_match),
		.suppress_bind_attrs = true,
	},
};
module_spi_driver(gnss_spi_driver);

MODULE_DESCRIPTION("Exynos SPI driver for GNSS communication");
MODULE_LICENSE("GPL");

