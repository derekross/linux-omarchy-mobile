// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Samsung Electronics.
 *
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/swab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/platform_data/spi-s3c64xx.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/ktime.h>

#include "include/gnss.h"
#include "gnss_spi.h"


/*
 * omamobile test: 32-bit words, as stock, with each word byte-swapped in
 * software (stock: the vendor driver's swap-mode), so the bytes go on the
 * wire in memory order. CHPP pads its frames to a multiple of 4.
 */
/*
 * omamobile test (G3-A2): word32=1 sends 32-bit words with no software
 * swap, for a controller that does the swap itself (spi-s3c64xx with
 * Google's swap-mode programmed in SWAP_CFG: bytes and half-words of each
 * word swapped on TX and RX, so the bytes leave in memory order, as stock's
 * vendor driver does). Default off: the 8-bit path is unchanged.
 */
/*
 * word32=2: 32-bit words with the byte swap done here (swab32 on TX and RX),
 * for a controller without swap-mode: tests whether the DT property is needed.
 */
static int word32;
module_param(word32, int, 0444);
MODULE_PARM_DESC(word32, "1: 32-bit words, the controller swaps; 2: 32-bit words, swapped in software");

static inline unsigned int gnss_spi_bpw(void)
{
	return word32 ? 32 : SPI_BITS_PER_WORD;
}

static void gnss_spi_swab(void *buf, unsigned int size)
{
	u32 *w = buf;
	unsigned int i;

	if (word32 == 1 || (word32 != 2 && SPI_BITS_PER_WORD != 32))
		return;

	for (i = 0; i < size / 4; i++)
		w[i] = swab32(w[i]);
}

/*
 * omamobile test: mainline spi-s3c64xx cuts a PIO transfer of 64 bytes or
 * more into 63-byte pieces, which a 32-bit word can't be split into (the
 * -110 timeouts of the plain 32-bit test). Send 32-byte transfers of one
 * message instead (chip select held): each fits the polling path, as stock's
 * 64-byte reads do in Google's driver.
 */
/* piece: transfer size within one message (chip select held); 64K = one transfer */
static unsigned int piece = SZ_64K;
module_param(piece, uint, 0444);
#define GNSS_SPI_PIECE	piece

/* omamobile test: override the controller's feedback delay (FB_CLK_SEL) */
static int fb_delay = -1;
module_param(fb_delay, int, 0444);

/*
 * omamobile test: CMU_HSI0 USI3. The clock driver keeps the mux read-only
 * on its 399.36 MHz parent and the divider stops at /16, so the SPI runs at
 * 24.96 MHz / 4 = 6.24 MHz whatever it asks for. osc=1 moves the mux to
 * OSCCLK (24.576 MHz), so the divider the driver then picks gives a slower
 * clock; speed_hz sets the rate the driver asks for.
 */
#define CMU_HSI0_USI3_MUX	0x11001020
#define CMU_HSI0_USI3_DIV	0x11001820
static int osc;
module_param(osc, int, 0444);
static int speed_hz;
module_param(speed_hz, int, 0444);
/* div >= 0: hold the USI3 divider at this value (stock 12 MHz: osc=1, div=1) */
static int div = -1;
/* rx_cpha=1: read-only transfers in SPI mode 1, writes stay in mode 0 */
static int rx_cpha;
module_param(rx_cpha, int, 0444);
/*
 * omamobile test (G3-A3): spi_mode 0..3 replaces the device tree's CPOL and
 * CPHA for every transfer, for a clock-phase sweep (RX arrived 2, 4 and 6
 * bits late in G3-A2); -1 keeps the device tree's mode.
 */
static int spi_mode = -1;
module_param(spi_mode, int, 0444);
MODULE_PARM_DESC(spi_mode, "0..3: SPI mode (CPOL/CPHA) for all transfers; -1 keeps the DT's");
/*
 * realign=1: Kepler's data comes in exactly one bit late with feedback
 * delay 3 (MODEM-PLAN G3), so shift every received buffer left one bit.
 */
static int realign;
module_param(realign, int, 0444);

/*
 * realign=2: Kepler's data is on time at first and one bit late later, so
 * choose per read: a CHPP preamble (68 43) at bit 0 keeps the read as it
 * is, one at bit 1 shifts it; a read without one keeps the last choice.
 */
static bool gnss_spi_late;
/*
 * carry=1: the bit the shift drops at the end of a read (Kepler's last bit of
 * the block) is the first bit sampled in the next read, when that read lands
 * right after it in the same buffer (rx_work's skb).
 */
static int carry;
module_param(carry, int, 0444);
/* rx_gap_us: wait this long before each read-only transfer (Kepler's next block) */
static int rx_gap_us;
module_param(rx_gap_us, int, 0444);
/*
 * psr >= 0: the SPI block's own prescaler (CLK_CFG at 0x04: ENCLK bit 8,
 * PSR bits 7:0; SPICLK = IPCLK / (2 * (PSR + 1)) if the IP has it). The
 * driver never writes CLK_CFG for a CMU-clocked port. bench=1 times ten
 * 64-byte reads at probe (Kepler not running yet) to see the real clock.
 */
#define GNSS_SPI_REGS	0x111e0000
static int psr = -1;
module_param(psr, int, 0444);
static int bench;
module_param(bench, int, 0444);
static void __iomem *gnss_spi_regs;

/* tx_gap_us: wait this long before each write (Kepler pausing its stream) */
static int tx_gap_us;
module_param(tx_gap_us, int, 0444);
static u8 *gnss_spi_carry_at;
static int gnss_spi_k;	/* realign=3: Kepler's bit offset, 0-7 */

static int gnss_spi_preamble_k(const u8 *b, unsigned int n)
{
	unsigned int i, k;

	for (i = 0; i + 2 < n; i++) {
		u32 w = b[i] << 16 | b[i + 1] << 8 | b[i + 2];

		for (k = 0; k < 8; k++)
			if (((w >> (8 - k)) & 0xffff) == 0x6843)
				return k;
	}
	return -1;
}

static void gnss_spi_shlk(u8 *b, unsigned int n, int k)
{
	unsigned int i;

	if (!k)
		return;
	for (i = 0; i < n; i++)
		b[i] = (b[i] << k) | (i + 1 < n ? b[i + 1] >> (8 - k) : 0);
}

static int gnss_spi_preamble_at(const u8 *b, unsigned int n)
{
	unsigned int i;

	for (i = 0; i + 2 < n; i++) {
		u32 w = b[i] << 16 | b[i + 1] << 8 | b[i + 2];

		if ((w >> 8) == 0x6843)
			return 0;
		if (((w >> 7) & 0xffff) == 0x6843)
			return 1;
	}
	return -1;
}

static void gnss_spi_shl1(u8 *b, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		b[i] = (b[i] << 1) | (i + 1 < n ? b[i + 1] >> 7 : 0);
}
module_param(div, int, 0444);
static void __iomem *gnss_spi_div;

static void gnss_spi_hold_div(void)
{
	u32 v;
	int i;

	if (div < 0 || !gnss_spi_div)
		return;
	v = readl(gnss_spi_div);
	if ((v & 0xf) == div)
		return;
	writel((v & ~0xf) | div, gnss_spi_div);
	for (i = 0; i < 1000 && (readl(gnss_spi_div) & BIT(16)); i++)
		udelay(1);
	gif_info("usi3 div 0x%08x -> 0x%08x\n", v, readl(gnss_spi_div));
}

static void gnss_spi_cmu(void)
{
	void __iomem *mux, *div;

	mux = ioremap(CMU_HSI0_USI3_MUX, 4);
	div = ioremap(CMU_HSI0_USI3_DIV, 4);
	if (!mux || !div)
		goto out;
	gif_info("usi3 mux 0x%08x div 0x%08x\n", readl(mux), readl(div));
	if (osc) {
		writel(readl(mux) | 1, mux);
		gif_info("usi3 mux now 0x%08x\n", readl(mux));
	}
out:
	if (mux)
		iounmap(mux);
	if (div)
		iounmap(div);
}

static void __maybe_unused gnss_spi_cmu_div(void)
{
	void __iomem *div = ioremap(CMU_HSI0_USI3_DIV, 4);

	if (div) {
		gif_info("usi3 div 0x%08x\n", readl(div));
		iounmap(div);
	}
}

/* omamobile test: microseconds from chip select to the first clock */
static int cs_setup_us;
module_param(cs_setup_us, int, 0444);
/* cs_hold_us: keep chip select asserted this long after the last clock */
static int cs_hold_us;
module_param(cs_hold_us, int, 0444);

static int gnss_spi_xfer(const char *tx, char *rx, unsigned int size)
{
	unsigned int n = DIV_ROUND_UP(size, GNSS_SPI_PIECE), i, off;
	struct spi_transfer *t;
	struct spi_message msg;
	char *tmp = NULL;
	int ret;

	if (gnss_spi_bpw() == 32 && size % 4)
		return -EINVAL;
	if (tx) {
		tmp = kmemdup(tx, size, GFP_KERNEL);
		if (!tmp)
			return -ENOMEM;
		gnss_spi_swab(tmp, size);
	}
	t = kcalloc(n, sizeof(*t), GFP_KERNEL);
	if (!t) {
		kfree(tmp);
		return -ENOMEM;
	}
	spi_message_init(&msg);
	for (i = 0, off = 0; i < n; i++, off += GNSS_SPI_PIECE) {
		t[i].len = min(GNSS_SPI_PIECE, size - off);
		t[i].tx_buf = tmp ? tmp + off : NULL;
		t[i].rx_buf = rx ? rx + off : NULL;
		t[i].bits_per_word = gnss_spi_bpw();
		spi_message_add_tail(&t[i], &msg);
	}
	gnss_spi_hold_div();
	if (!tx && rx_gap_us > 0)
		usleep_range(rx_gap_us, rx_gap_us + 50);
	if (tx && tx_gap_us > 0)
		usleep_range(tx_gap_us, tx_gap_us + 100);
	if (rx_cpha)
		mutex_lock(&gnss_if.lock);
	if (rx_cpha) {
		/*
		 * spi-s3c64xx reconfigures on every transfer here (its
		 * cur_speed is the 6.24 MHz it gets, never the rate asked),
		 * and takes cur_mode from spi->mode when it does.
		 */
		if (tx)
			gnss_if.spi->mode &= ~SPI_CPHA;
		else
			gnss_if.spi->mode |= SPI_CPHA;
	}
	ret = spi_sync(gnss_if.spi, &msg);
	if (rx_cpha)
		mutex_unlock(&gnss_if.lock);
	if (ret < 0)
		gif_err("spi_sync() error:%d\n", ret);
	gnss_spi_hold_div();
	{
		static bool once;

		if (!once) {
			once = true;
			gnss_spi_cmu_div();
		}
	}
	if (rx && realign == 1 && ret >= 0)
		gnss_spi_shl1(rx, size);
	if (rx && realign == 3 && ret >= 0) {
		int at = gnss_spi_preamble_k(rx, size);

		if (at >= 0 && at != gnss_spi_k) {
			gif_info("realign: Kepler bit offset %d -> %d\n", gnss_spi_k, at);
			gnss_spi_k = at;
		}
		gnss_spi_shlk(rx, size, gnss_spi_k);
	}
	if (rx && realign == 4 && ret >= 0 && !tx && size) {
		/*
		 * realign=4: as 3, but carry across reads instead of dropping the
		 * last k bits: each output byte takes its high bits from the byte
		 * before it (the previous read's last raw byte for the first), so
		 * the stream stays continuous and byte-aligned, one byte later.
		 */
		static u8 prev;
		u8 *b = rx;
		int at = gnss_spi_preamble_k(b, size);
		unsigned int i;
		u8 last = b[size - 1];

		if (at >= 0 && at != gnss_spi_k) {
			gif_info("realign: Kepler bit offset %d -> %d\n", gnss_spi_k, at);
			gnss_spi_k = at;
		}
		if (gnss_spi_k) {
			for (i = size - 1; i > 0; i--)
				b[i] = (b[i - 1] << gnss_spi_k) | (b[i] >> (8 - gnss_spi_k));
			b[0] = (prev << gnss_spi_k) | (b[0] >> (8 - gnss_spi_k));
		}
		prev = last;
	}
	if (rx && realign == 2 && ret >= 0) {
		int at = gnss_spi_preamble_at(rx, size);
		u8 first = ((u8 *)rx)[0] >> 7;

		if (carry && !tx && gnss_spi_carry_at &&
		    gnss_spi_carry_at + 1 == (u8 *)rx && gnss_spi_late)
			*gnss_spi_carry_at |= first;
		gnss_spi_carry_at = tx ? NULL : (u8 *)rx + size - 1;

		if (at >= 0 && gnss_spi_late != at) {
			gnss_spi_late = at;
			gif_info("realign: Kepler data %s\n", at ? "one bit late" : "on time");
		}
		if (gnss_spi_late)
			gnss_spi_shl1(rx, size);
	}
	if (rx)
		gnss_spi_swab(rx, size);
	kfree(t);
	kfree(tmp);
	return ret;
}

static int gnss_spi_xfer(const char *tx, char *rx, unsigned int size);

/* realign=3 and txk=1: send writes k bits later, so they land on Kepler's byte boundaries */
static int txk;
module_param(txk, int, 0444);

static int gnss_spi_send_k(const char *tx, char *rx, unsigned int size, int k)
{
	u8 *t, *r = NULL;
	unsigned int i;
	int ret;

	t = kzalloc(size + 1, GFP_KERNEL);
	if (!t)
		return -ENOMEM;
	if (rx)
		r = kzalloc(size + 1, GFP_KERNEL);
	for (i = 0; i <= size; i++)
		t[i] = (i < size ? (u8)tx[i] >> k : 0) |
		       (i ? (u8)((u8)tx[i - 1] << (8 - k)) : 0);
	ret = gnss_spi_xfer((char *)t, r ? (char *)r : NULL, size + 1);
	if (r) {
		memcpy(rx, r, size);
		kfree(r);
	}
	kfree(t);
	return ret;
}

int gnss_spi_send(char *buff, unsigned int size, char *rx_buff)
{
	if (realign == 3 && txk && gnss_spi_k)
		return gnss_spi_send_k(buff, rx_buff, size, gnss_spi_k);
	return gnss_spi_xfer(buff, rx_buff, size);
}
EXPORT_SYMBOL(gnss_spi_send);

int gnss_spi_recv(char *buff, unsigned int size)
{
	return gnss_spi_xfer(NULL, buff, size);
}
EXPORT_SYMBOL(gnss_spi_recv);

static int gnss_spi_probe(struct spi_device *spi)
{
	int ret = 0;

	gif_info("+++\n");
	mutex_init(&gnss_if.lock);

	gnss_spi_cmu();
	if (div >= 0) {
		gnss_spi_div = ioremap(CMU_HSI0_USI3_DIV, 4);
		gnss_spi_hold_div();
	}
	if (speed_hz > 0)
		spi->max_speed_hz = speed_hz;
	if (spi_mode >= 0 && spi_mode <= 3)
		spi->mode = (spi->mode & ~(SPI_CPHA | SPI_CPOL)) | spi_mode;
	if (spi_setup(spi)) {
		gif_err("ERR! spi_setup fail\n");
		ret = -EINVAL;
		goto err_setup;
	}
	gif_info("spi mode 0x%x (CPOL %d, CPHA %d)%s\n", spi->mode,
		 !!(spi->mode & SPI_CPOL), !!(spi->mode & SPI_CPHA),
		 spi_mode >= 0 ? " from spi_mode" : "");
	if (fb_delay >= 0 && spi->controller_data) {
		struct s3c64xx_spi_csinfo *cs = spi->controller_data;

		gif_info("feedback delay %u -> %d\n", cs->fb_delay, fb_delay);
		cs->fb_delay = fb_delay;
	}
	if (psr >= 0 || bench) {
		ret = pm_runtime_resume_and_get(spi->controller->dev.parent);
		if (ret < 0)
			gif_err("controller resume failed %d\n", ret);
		else
			gnss_spi_regs = ioremap(GNSS_SPI_REGS, 0x40);
		ret = 0;
	}
	if (psr >= 0 && gnss_spi_regs) {
		gif_info("CLK_CFG 0x%08x\n", readl(gnss_spi_regs + 4));
		writel(BIT(8) | (psr & 0xff), gnss_spi_regs + 4);
		gif_info("CLK_CFG now 0x%08x\n", readl(gnss_spi_regs + 4));
	}
	if (cs_hold_us > 0) {
		spi->cs_hold.value = cs_hold_us;
		spi->cs_hold.unit = SPI_DELAY_UNIT_USECS;
		gif_info("cs_hold %d us\n", cs_hold_us);
	}
	if (cs_setup_us > 0) {
		spi->cs_setup.value = cs_setup_us;
		spi->cs_setup.unit = SPI_DELAY_UNIT_USECS;
		gif_info("cs_setup %d us\n", cs_setup_us);
	}
	if (bench) {
		char *buf = kzalloc(64, GFP_KERNEL);
		ktime_t t0;
		int i;

		if (buf) {
			gnss_if.spi = spi;
			t0 = ktime_get();
			for (i = 0; i < 10; i++)
				gnss_spi_xfer(NULL, buf, 64);
			gif_info("bench: ten 64-byte reads in %lld us (82 us each at 6.24 MHz, 164 at 3.12)\n",
				 ktime_us_delta(ktime_get(), t0));
			kfree(buf);
		}
	}
	spi_set_drvdata(spi, &gnss_if);
	gnss_if.spi = spi;
	gif_info("---\n");

	return 0;

err_setup:
	mutex_destroy(&gnss_if.lock);

	return ret;
}

static void gnss_spi_remove (struct spi_device *spi)
{
	struct gnss_spi *gnss_if = spi_get_drvdata(spi);

	mutex_destroy(&gnss_if->lock);
}

static const struct of_device_id gnss_spi_dt_match[] = {
	{ .compatible = "samsung,gnss-spi" },
	{},
};
MODULE_DEVICE_TABLE(of, gnss_spi_dt_match);

static struct spi_driver gnss_spi_driver = {
	.probe = gnss_spi_probe,
	.remove = gnss_spi_remove,
	.driver = {
		.name = "gnss_spi",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gnss_spi_dt_match),
		.suppress_bind_attrs = true,
	},
};
module_spi_driver(gnss_spi_driver);

MODULE_DESCRIPTION("Exynos SPI driver for GNSS communication");
MODULE_LICENSE("GPL");

