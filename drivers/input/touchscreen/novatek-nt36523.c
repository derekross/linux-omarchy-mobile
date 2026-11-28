// SPDX-License-Identifier: GPL-2.0-only
/*
 * Novatek NT36523N touchscreen driver (SPI)
 *
 * The NT36523N is the touch half of the touch and display driver chip in
 * the Google Pixel Tablet. It has no flash: after every reset the host
 * loads its firmware into the chip's SRAM. The protocol follows Google's
 * GPL Novatek driver (kernel/google-modules/touch/novatek, nt36xxx/, branch
 * android-gs-tangorpro-6.1-android16: nt36xxx.c for the framing, the
 * resets and the report, nt36xxx_fw_update.c for the download with the
 * chip's CRC check, nt36xxx_mem_map.h for the addresses):
 *   Copyright (C) 2010 - 2021 Novatek, Inc.
 *
 * The chip's memory is reached through a 128-byte window. One transfer
 * moves the window:
 *
 *	tx: 0xff | addr >> 15 | addr >> 7
 *
 * and the next one writes or reads from the window's offset onwards:
 *
 *	write tx: 0x80 | (addr & 0x7f) | data ...
 *	read  tx: addr & 0x7f | 0 ...
 *	read  rx:     ...     | dummy | data ...
 *
 * A transfer may run past the end of the window.
 *
 * The firmware image holds the code (ILM) and data (DLM) memories and a
 * few small sections, each with its SRAM address and its length. The host
 * gives the boot loader the ILM's and DLM's addresses, lengths and CRCs,
 * writes every section, sets "boot ready" and waits for the firmware to
 * report its reset state. The chip checks the CRCs itself.
 *
 * Once running, the firmware raises its interrupt line for each report,
 * read from the start of the event buffer: ten 6-byte points,
 *
 *	id (5 bits) | status (3) | x[11:4] | y[11:4] | x[3:0] | y[3:0] |
 *	width | pressure
 *
 * where an id of 0 is an empty slot and a status of 1 or 2 is a finger
 * on the screen.
 */

#include <linux/cache.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define NT36523_PAGE_CMD		0xff
#define NT36523_WRITE_BIT		0x80
#define NT36523_OFFSET_MASK		0x7f
/* A read returns the address byte's slot and one dummy byte first */
#define NT36523_READ_HDR		2

/* Addresses: NT36523N_memory_map and the tangorpro device tree */
#define NT36523_ENG_RST_ADDR		0x7fff80
#define NT36523_SWRST_ADDR		0x03f0fe
#define NT36523_SPI_RD_FAST_ADDR	0x03f310
#define NT36523_CHIP_ID_ADDR		0x03f004
#define NT36523_EVENT_BUF_ADDR		0x02fe00	/* NT36523 */
#define NT36523N_EVENT_BUF_ADDR		0x02fd00	/* NT36523N */
#define NT36523_BOOT_RDY_ADDR		0x03f10d
#define NT36523_TX_AUTO_COPY_ADDR	0x03f7e8
#define NT36523_SPI_DMA_TX_INFO_ADDR	0x03f7f1
#define NT36523_SPI_DMA_VAL_ADDR	0x03f7d0
#define NT36523_G_ILM_CRC_ADDR		0x03f100
#define NT36523_G_DLM_CRC_ADDR		0x03f104
#define NT36523_ILM_LEN_ADDR		0x03f118
#define NT36523_ILM_DES_ADDR		0x03f128
#define NT36523_DLM_DES_ADDR		0x03f12c
#define NT36523_DLM_LEN_ADDR		0x03f130
#define NT36523_CRC_FLAG_ADDR		0x03f133
#define NT36523_BLD_CRC_EN_ADDR		0x03f30e

/* Offsets in the event buffer */
#define NT36523_EVT_HOST_CMD		0x50
#define NT36523_EVT_RESET_STATE		0x60
#define NT36523_EVT_FW_INFO		0x78

#define NT36523_ENG_RST			0x5a
#define NT36523_SWRST_BOOTLOADER	0x69
#define NT36523_TX_AUTO_COPY_EN		0x69
#define NT36523_HOST_CMD_FW_CRC		0xae
#define NT36523_BLD_CRC_EN		BIT(7)
#define NT36523_RESET_STATE_INIT	0xa0
#define NT36523_RESET_STATE_MAX		0xaf

#define NT36523_CHIP_ID_LEN		6
#define NT36523_CHIP_ID_TRIES		5
#define NT36523_FW_INFO_LEN		38
#define NT36523_FW_INFO_TRIES		3
#define NT36523_DOWNLOAD_TRIES		3
#define NT36523_BOOT_POLLS		11
#define NT36523_DMA_POLLS		200

#define NT36523_MAX_TOUCHES		10
#define NT36523_POINT_LEN		6
#define NT36523_REPORT_LEN		65
/* Frames of the firmware's watchdog pattern before the host reloads it */
#define NT36523_WDT_FRAMES		10

/* Firmware image */
#define NT36523_FW_SECTOR		4096
#define NT36523_FW_END_FLAG_LEN		3
#define NT36523_FW_INFO_START		0x30
#define NT36523_FW_SECTION_LEN		0x10
#define NT36523_FW_MAX_PARTS		32
/* Google's driver writes at most 63 KiB per transfer */
#define NT36523_XFER_MAX		(63 * 1024)

#define NT36523_BUF_LEN			ALIGN(NT36523_READ_HDR + NT36523_REPORT_LEN, 8)

struct nt36523_part {
	u32 bin;	/* offset in the image */
	u32 sram;	/* address in the chip */
	u32 size;	/* length - 1, as the image and the boot loader count it */
	u32 crc;
};

struct nt36523_ts {
	struct spi_device *spi;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct touchscreen_properties prop;
	const struct firmware *fw;
	struct nt36523_part parts[NT36523_FW_MAX_PARTS];
	unsigned int num_parts;
	bool cascade;
	bool spi_dma_div;
	bool is_n;		/* NT36523N: other event buffer, SPI DMA divider */
	u32 evbuf;
	unsigned int wdt_frames;
	u16 width;
	u16 height;
	u8 *xfer;	/* firmware writes: NT36523_XFER_MAX + 1 bytes */
	u8 report[NT36523_REPORT_LEN];
	/* SPI buffers, DMA-safe: each on its own DMA alignment boundary */
	u8 tx[NT36523_BUF_LEN] __aligned(ARCH_DMA_MINALIGN);
	u8 rx[NT36523_BUF_LEN] __aligned(ARCH_DMA_MINALIGN);
};

static int nt36523_set_page(struct nt36523_ts *ts, u32 addr)
{
	ts->tx[0] = NT36523_PAGE_CMD;
	ts->tx[1] = (addr >> 15) & 0xff;
	ts->tx[2] = (addr >> 7) & 0xff;

	return spi_write(ts->spi, ts->tx, 3);
}

static int nt36523_write(struct nt36523_ts *ts, u32 addr, const void *data,
			 size_t len)
{
	int ret;

	if (len + 1 > sizeof(ts->tx))
		return -EINVAL;

	ret = nt36523_set_page(ts, addr);
	if (ret)
		return ret;

	ts->tx[0] = NT36523_WRITE_BIT | (addr & NT36523_OFFSET_MASK);
	memcpy(&ts->tx[1], data, len);

	return spi_write(ts->spi, ts->tx, len + 1);
}

static int nt36523_write_u8(struct nt36523_ts *ts, u32 addr, u8 val)
{
	return nt36523_write(ts, addr, &val, 1);
}

static int nt36523_read(struct nt36523_ts *ts, u32 addr, void *data,
			size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->tx,
		.rx_buf = ts->rx,
		.len = NT36523_READ_HDR + len,
	};
	int ret;

	if (xfer.len > sizeof(ts->rx))
		return -EINVAL;

	ret = nt36523_set_page(ts, addr);
	if (ret)
		return ret;

	memset(ts->tx, 0, xfer.len);
	ts->tx[0] = addr & NT36523_OFFSET_MASK;

	ret = spi_sync_transfer(ts->spi, &xfer, 1);
	if (ret)
		return ret;

	memcpy(data, &ts->rx[NT36523_READ_HDR], len);
	return 0;
}

static int nt36523_read_u8(struct nt36523_ts *ts, u32 addr, u8 *val)
{
	return nt36523_read(ts, addr, val, 1);
}

/* Restart the MCU in its boot loader */
static int nt36523_bootloader_reset(struct nt36523_ts *ts)
{
	int ret;

	ret = nt36523_write_u8(ts, NT36523_SWRST_ADDR, NT36523_SWRST_BOOTLOADER);
	if (ret)
		return ret;

	usleep_range(5000, 6000);

	/* Google's driver turns fast reads off after every boot loader reset */
	return nt36523_write_u8(ts, NT36523_SPI_RD_FAST_ADDR, 0);
}

/* The engineering reset goes out while the touch reset line is asserted */
static int nt36523_hw_reset(struct nt36523_ts *ts)
{
	int ret;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	usleep_range(1000, 2000);

	ret = nt36523_write_u8(ts, NT36523_ENG_RST_ADDR, NT36523_ENG_RST);
	if (ret)
		return ret;
	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	usleep_range(10000, 11000);

	return 0;
}

static int nt36523_identify(struct nt36523_ts *ts)
{
	u8 id[NT36523_CHIP_ID_LEN];
	unsigned int i;
	int ret;

	for (i = 0; i < NT36523_CHIP_ID_TRIES; i++) {
		ret = nt36523_bootloader_reset(ts);
		if (ret)
			return ret;

		ret = nt36523_read(ts, NT36523_CHIP_ID_ADDR, id, sizeof(id));
		if (ret)
			return ret;

		/*
		 * Bytes 1 and 2 are not part of the chip id. 0x17 is the
		 * NT36523N; 0x0a, 0x0b, 0x0c and 0x20 are NT36523 revisions,
		 * whose event buffer sits 0x100 higher (Google's nt36xxx
		 * trim table, NT36523_memory_map and NT36523N_memory_map).
		 */
		if (id[3] == 0x23 && id[4] == 0x65 && id[5] == 0x03) {
			switch (id[0]) {
			case 0x17:
				ts->is_n = true;
				ts->evbuf = NT36523N_EVENT_BUF_ADDR;
				return 0;
			case 0x0a:
			case 0x0b:
			case 0x0c:
			case 0x20:
				ts->is_n = false;
				ts->evbuf = NT36523_EVENT_BUF_ADDR;
				return 0;
			}
		}

		usleep_range(10000, 11000);
	}

	dev_err(&ts->spi->dev, "no NT36523 found (id %*ph)\n",
		(int)sizeof(id), id);
	return -ENODEV;
}

static int nt36523_parse_fw(struct nt36523_ts *ts)
{
	struct device *dev = &ts->spi->dev;
	const u8 *d = ts->fw->data;
	size_t fw_size = ts->fw->size;
	size_t size = 0;
	u32 hdr_end, hdr_lim, pos;
	unsigned int info, ovly, i;
	bool found_hdr = false;

	if (fw_size < NT36523_FW_SECTOR)
		return -EINVAL;

	/* The image ends in a sector whose last bytes are "NVT" or "MOD" */
	for (i = fw_size / NT36523_FW_SECTOR; i > 0; i--) {
		const u8 *flag = d + i * NT36523_FW_SECTOR - NT36523_FW_END_FLAG_LEN;

		if (!memcmp(flag, "NVT", NT36523_FW_END_FLAG_LEN) ||
		    !memcmp(flag, "MOD", NT36523_FW_END_FLAG_LEN)) {
			size = i * NT36523_FW_SECTOR;
			break;
		}
	}
	if (!size) {
		dev_err(dev, "firmware has no end flag\n");
		return -EINVAL;
	}

	/* The last sector starts with the version and its complement */
	if (d[size - NT36523_FW_SECTOR] + d[size - NT36523_FW_SECTOR + 1] != 0xff) {
		dev_err(dev, "firmware version check failed\n");
		return -EINVAL;
	}

	hdr_end = get_unaligned_le32(d);
	if (hdr_end < NT36523_FW_INFO_START || hdr_end > size) {
		dev_err(dev, "bad firmware header length %#x\n", hdr_end);
		return -EINVAL;
	}

	/*
	 * Info sections of 16 bytes follow from 0x30. In a cascaded image
	 * they fill the first half of the header, and the second header's
	 * own section sits at its end.
	 */
	ts->cascade = d[0x20] & BIT(1);
	ts->spi_dma_div = ts->cascade && (d[0x29] & BIT(0));
	hdr_lim = ts->cascade ? hdr_end / 2 : hdr_end;
	info = hdr_lim > NT36523_FW_INFO_START ?
	       DIV_ROUND_UP(hdr_lim - NT36523_FW_INFO_START, NT36523_FW_SECTION_LEN) : 0;
	if (ts->cascade)
		info++;
	ovly = (d[0x28] & BIT(4)) ? d[0x28] & 0x0f : 0;

	ts->num_parts = 2 + info + ovly;
	if (ts->num_parts > NT36523_FW_MAX_PARTS) {
		dev_err(dev, "too many firmware sections (%u)\n", ts->num_parts);
		return -EINVAL;
	}

	for (i = 0; i < ts->num_parts; i++) {
		struct nt36523_part *p = &ts->parts[i];

		if (i < 2) {
			/* ILM and DLM: image offset, SRAM address, length */
			p->bin = get_unaligned_le32(d + 12 * i);
			p->sram = get_unaligned_le32(d + 4 + 12 * i);
			p->size = get_unaligned_le32(d + 8 + 12 * i);
			p->crc = get_unaligned_le32(d + 0x18 + 4 * i);
		} else {
			if (i < 2 + info) {
				if (found_hdr && ts->cascade)
					pos = hdr_end - NT36523_FW_SECTION_LEN;
				else
					pos = NT36523_FW_INFO_START +
					      NT36523_FW_SECTION_LEN * (i - 2);
			} else {
				/* Overlay sections are listed at the DLM's start */
				pos = ts->parts[1].bin +
				      NT36523_FW_SECTION_LEN * (i - 2 - info);
			}
			if (pos > size - NT36523_FW_SECTION_LEN) {
				dev_err(dev, "firmware section %u out of range\n", i);
				return -EINVAL;
			}

			/* SRAM address, length, image offset, CRC */
			p->sram = get_unaligned_le32(d + pos);
			p->size = get_unaligned_le32(d + pos + 4);
			p->bin = get_unaligned_le32(d + pos + 8);
			p->crc = get_unaligned_le32(d + pos + 12);

			if (i < 2 + info && p->bin < hdr_end && p->size)
				found_hdr = true;
		}

		/* An empty section is reserved and never written */
		if (p->size && (p->bin >= fw_size || p->size >= fw_size - p->bin)) {
			dev_err(dev, "firmware section %u exceeds the image\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

static int nt36523_write_sram(struct nt36523_ts *ts, u32 sram, const u8 *data,
			      size_t len)
{
	size_t max = min_t(size_t, NT36523_XFER_MAX,
			   spi_max_transfer_size(ts->spi) - 1);
	int ret;

	while (len) {
		size_t chunk = min(len, max);

		ret = nt36523_set_page(ts, sram);
		if (ret)
			return ret;

		ts->xfer[0] = NT36523_WRITE_BIT | (sram & NT36523_OFFSET_MASK);
		memcpy(&ts->xfer[1], data, chunk);
		ret = spi_write(ts->spi, ts->xfer, chunk + 1);
		if (ret)
			return ret;

		sram += chunk;
		data += chunk;
		len -= chunk;
	}

	return 0;
}

/* Tell the boot loader where a memory goes, how long it is and its CRC */
static int nt36523_set_crc_bank(struct nt36523_ts *ts, u32 des_addr,
				u32 len_addr, u32 crc_addr,
				const struct nt36523_part *p)
{
	u8 buf[4];
	int ret;

	put_unaligned_le24(p->sram, buf);
	ret = nt36523_write(ts, des_addr, buf, 3);
	if (ret)
		return ret;

	put_unaligned_le24(p->size & 0x1ffff, buf);
	ret = nt36523_write(ts, len_addr, buf, 3);
	if (ret)
		return ret;

	put_unaligned_le32(p->crc, buf);
	return nt36523_write(ts, crc_addr, buf, 4);
}

static int nt36523_wait_dma(struct nt36523_ts *ts)
{
	unsigned int i;
	u8 val;
	int ret;

	for (i = 0; i < NT36523_DMA_POLLS; i++) {
		ret = nt36523_read_u8(ts, NT36523_SPI_DMA_TX_INFO_ADDR, &val);
		if (ret)
			return ret;
		if (!val)
			return 0;
		usleep_range(1000, 1100);
	}

	dev_err(&ts->spi->dev, "SPI DMA did not finish (%#04x)\n", val);
	return -ETIMEDOUT;
}

static int nt36523_wait_boot(struct nt36523_ts *ts)
{
	u8 state[5], crc_flags = 0;
	unsigned int i;
	int ret;

	for (i = 0; i < NT36523_BOOT_POLLS; i++) {
		ret = nt36523_read(ts, ts->evbuf + NT36523_EVT_RESET_STATE,
				   state, sizeof(state));
		if (ret)
			return ret;
		if (state[0] >= NT36523_RESET_STATE_INIT &&
		    state[0] <= NT36523_RESET_STATE_MAX)
			return 0;
		usleep_range(10000, 11000);
	}

	/* bit 0: ILM CRC failed, bit 1: DLM CRC failed, bit 2: CRC done */
	nt36523_read_u8(ts, NT36523_CRC_FLAG_ADDR, &crc_flags);
	dev_err(&ts->spi->dev,
		"firmware did not start (state %*ph, CRC flags %#04x)\n",
		(int)sizeof(state), state, crc_flags);
	return -ETIMEDOUT;
}

static int nt36523_download(struct nt36523_ts *ts)
{
	static const u8 dma_val[] = { 0x35, 0x32, 0xaa, 0x00 };
	static const u8 crc_cmd[] = { NT36523_HOST_CMD_FW_CRC, 0x00 };
	static const u8 zero[32];
	unsigned int i;
	u8 val;
	int ret;

	ret = nt36523_bootloader_reset(ts);
	if (ret)
		return ret;

	ret = nt36523_set_crc_bank(ts, NT36523_ILM_DES_ADDR, NT36523_ILM_LEN_ADDR,
				   NT36523_G_ILM_CRC_ADDR, &ts->parts[0]);
	if (ret)
		return ret;

	ret = nt36523_set_crc_bank(ts, NT36523_DLM_DES_ADDR, NT36523_DLM_LEN_ADDR,
				   NT36523_G_DLM_CRC_ADDR, &ts->parts[1]);
	if (ret)
		return ret;

	if (ts->cascade) {
		/* Only the NT36523N has the SPI DMA divider */
		if (ts->is_n && ts->spi_dma_div) {
			ret = nt36523_write(ts, ts->parts[1].sram, zero,
					    sizeof(zero));
			if (ret)
				return ret;

			ret = nt36523_write(ts, NT36523_SPI_DMA_VAL_ADDR, dma_val,
					    sizeof(dma_val));
			if (ret)
				return ret;
		}

		ret = nt36523_write_u8(ts, NT36523_TX_AUTO_COPY_ADDR,
				       NT36523_TX_AUTO_COPY_EN);
		if (ret)
			return ret;
	}

	for (i = 0; i < ts->num_parts; i++) {
		const struct nt36523_part *p = &ts->parts[i];

		if (!p->size)
			continue;

		ret = nt36523_write_sram(ts, p->sram, ts->fw->data + p->bin,
					 p->size + 1);
		if (ret)
			return ret;
	}

	if (ts->cascade) {
		ret = nt36523_wait_dma(ts);
		if (ret)
			return ret;
	}

	ret = nt36523_read_u8(ts, NT36523_BLD_CRC_EN_ADDR, &val);
	if (ret)
		return ret;

	ret = nt36523_write_u8(ts, NT36523_BLD_CRC_EN_ADDR,
			       val | NT36523_BLD_CRC_EN);
	if (ret)
		return ret;

	/* Clear the reset state, then have the firmware check its CRC */
	ret = nt36523_write(ts, ts->evbuf + NT36523_EVT_RESET_STATE,
			    zero, 6);
	if (ret)
		return ret;

	ret = nt36523_write(ts, ts->evbuf + NT36523_EVT_HOST_CMD,
			    crc_cmd, sizeof(crc_cmd));
	if (ret)
		return ret;

	ret = nt36523_write_u8(ts, NT36523_BOOT_RDY_ADDR, 1);
	if (ret)
		return ret;
	usleep_range(5000, 6000);

	return nt36523_wait_boot(ts);
}

static int nt36523_read_fw_info(struct nt36523_ts *ts)
{
	u8 info[NT36523_FW_INFO_LEN];
	unsigned int i;
	int ret;

	for (i = 0; i < NT36523_FW_INFO_TRIES; i++) {
		ret = nt36523_read(ts, ts->evbuf + NT36523_EVT_FW_INFO,
				   info, sizeof(info));
		if (ret)
			return ret;

		/* The version, then its complement */
		if (info[0] + info[1] == 0xff)
			break;
	}
	if (i == NT36523_FW_INFO_TRIES) {
		dev_err(&ts->spi->dev, "firmware info is broken (%*ph)\n",
			2, info);
		return -EIO;
	}

	ts->width = get_unaligned_be16(&info[4]);
	ts->height = get_unaligned_be16(&info[6]);

	dev_info(&ts->spi->dev,
		 "firmware %#04x, PID %#06x, %ux%u channels, %ux%u\n",
		 info[0], get_unaligned_le16(&info[34]), info[2], info[3],
		 ts->width, ts->height);
	return 0;
}

static int nt36523_load(struct nt36523_ts *ts)
{
	unsigned int i;
	int ret = -EIO;

	for (i = 0; i < NT36523_DOWNLOAD_TRIES; i++) {
		ret = nt36523_download(ts);
		if (!ret)
			return nt36523_read_fw_info(ts);
	}

	dev_err(&ts->spi->dev, "firmware download failed: %d\n", ret);
	return ret;
}

static bool nt36523_is_pattern(const u8 *r, u8 a, u8 b)
{
	unsigned int i;

	for (i = 0; i < NT36523_POINT_LEN; i++)
		if (r[i] != a && r[i] != b)
			return false;

	return true;
}

static irqreturn_t nt36523_irq(int irq, void *dev_id)
{
	struct nt36523_ts *ts = dev_id;
	const u8 *r = ts->report;
	unsigned int i;
	int ret;

	ret = nt36523_read(ts, ts->evbuf, ts->report,
			   sizeof(ts->report));
	if (ret) {
		dev_err_ratelimited(&ts->spi->dev, "report read failed: %d\n",
				    ret);
		return IRQ_HANDLED;
	}

	/* The firmware's handshake frame for the host's ESD check */
	if (nt36523_is_pattern(r, 0x77, 0x77))
		return IRQ_HANDLED;

	/*
	 * After a watchdog reset the firmware is gone and the boot loader
	 * sends 0xfd/0xfe frames. Load the firmware again, as Google's
	 * driver does, once it has kept doing so for a while.
	 */
	if (nt36523_is_pattern(r, 0xfd, 0xfe)) {
		if (++ts->wdt_frames > NT36523_WDT_FRAMES) {
			ts->wdt_frames = 0;
			dev_warn(&ts->spi->dev, "controller reset, reloading firmware\n");
			input_mt_sync_frame(ts->input);
			input_sync(ts->input);
			if (!nt36523_hw_reset(ts))
				nt36523_load(ts);
		}
		return IRQ_HANDLED;
	}
	ts->wdt_frames = 0;

	for (i = 0; i < NT36523_MAX_TOUCHES; i++) {
		const u8 *p = &r[i * NT36523_POINT_LEN];
		unsigned int id = p[0] >> 3;
		unsigned int status = p[0] & 0x07;
		unsigned int x, y;

		if (!id || id > NT36523_MAX_TOUCHES)
			continue;
		if (status != 1 && status != 2)
			continue;

		x = (p[1] << 4) | (p[3] >> 4);
		y = (p[2] << 4) | (p[3] & 0x0f);
		if (x > ts->prop.max_x || y > ts->prop.max_y)
			continue;

		input_mt_slot(ts->input, id - 1);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, p[4] ?: 1);
		input_report_abs(ts->input, ABS_MT_PRESSURE, p[5] ?: 1);
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
	return IRQ_HANDLED;
}

static void nt36523_release_fw(void *data)
{
	release_firmware(data);
}

static int nt36523_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct nt36523_ts *ts;
	const char *fw_name;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->spi = spi;
	spi_set_drvdata(spi, ts);

	ts->xfer = devm_kmalloc(dev, NT36523_XFER_MAX + 1, GFP_KERNEL);
	if (!ts->xfer)
		return -ENOMEM;

	ret = device_property_read_string(dev, "firmware-name", &fw_name);
	if (ret)
		return dev_err_probe(dev, ret, "no firmware-name\n");

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	/*
	 * The touch reset is only ever pulsed, as in Google's driver, and is
	 * left released on failure and on removal: the chip also drives the
	 * display, and nothing here knows what a held reset does to it.
	 */
	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "cannot get the reset GPIO\n");

	ret = devm_regulator_get_enable_optional(dev, "vio");
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "cannot enable vio\n");

	ret = request_firmware(&ts->fw, fw_name, dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot load %s\n", fw_name);

	ret = devm_add_action_or_reset(dev, nt36523_release_fw, (void *)ts->fw);
	if (ret)
		return ret;

	ret = nt36523_parse_fw(ts);
	if (ret)
		return ret;

	ret = nt36523_hw_reset(ts);
	if (ret)
		return ret;

	ret = nt36523_identify(ts);
	if (ret)
		return ret;

	ret = nt36523_load(ts);
	if (ret)
		return ret;

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;
	ts->input->name = ts->is_n ? "Novatek NT36523N" : "Novatek NT36523";
	ts->input->id.bustype = BUS_SPI;

	if (ts->width && ts->height) {
		input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0,
				     ts->width - 1, 0, 0);
		input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0,
				     ts->height - 1, 0, 0);
	} else {
		input_set_capability(ts->input, EV_ABS, ABS_MT_POSITION_X);
		input_set_capability(ts->input, EV_ABS, ABS_MT_POSITION_Y);
	}
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	/* The device tree's size, if given, wins over the firmware's */
	touchscreen_parse_properties(ts->input, true, &ts->prop);
	if (!ts->prop.max_x || !ts->prop.max_y)
		return dev_err_probe(dev, -EINVAL, "touchscreen size unknown\n");

	ret = input_mt_init_slots(ts->input, NT36523_MAX_TOUCHES,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ret = input_register_device(ts->input);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, nt36523_irq,
					IRQF_ONESHOT, "nt36523", ts);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request the interrupt\n");

	return 0;
}

static int nt36523_suspend(struct device *dev)
{
	struct nt36523_ts *ts = dev_get_drvdata(dev);

	disable_irq(ts->spi->irq);
	return 0;
}

static int nt36523_resume(struct device *dev)
{
	struct nt36523_ts *ts = dev_get_drvdata(dev);

	enable_irq(ts->spi->irq);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(nt36523_pm_ops, nt36523_suspend, nt36523_resume);

static const struct of_device_id nt36523_of_match[] = {
	{ .compatible = "novatek,nt36523-ts" },
	{ }
};
MODULE_DEVICE_TABLE(of, nt36523_of_match);

static const struct spi_device_id nt36523_spi_ids[] = {
	{ "nt36523-ts" },
	{ }
};
MODULE_DEVICE_TABLE(spi, nt36523_spi_ids);

static struct spi_driver nt36523_driver = {
	.driver = {
		.name = "novatek-nt36523",
		.of_match_table = nt36523_of_match,
		.pm = pm_sleep_ptr(&nt36523_pm_ops),
	},
	.id_table = nt36523_spi_ids,
	.probe = nt36523_probe,
};
module_spi_driver(nt36523_driver);

MODULE_DESCRIPTION("Novatek NT36523N touchscreen driver");
MODULE_LICENSE("GPL");
