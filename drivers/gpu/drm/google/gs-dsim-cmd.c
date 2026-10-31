// SPDX-License-Identifier: GPL-2.0-only
/*
 * Command path for the Google gs201 MIPI DSI master (DSIM), for a display
 * the bootloader left running.
 *
 * On gs201 the bootloader brings up DECON0 -> DSIM0 -> the panel in DSI
 * command mode and leaves the pipeline scanning out its framebuffer, which
 * simpledrm then uses. This driver does not touch that pipeline. It only
 * registers a MIPI DSI host on DSIM0, so the panel driver can bind and send
 * DCS commands (brightness, display on/off) through the DSIM packet header
 * and payload FIFOs, which the hardware interleaves with frame transfers.
 *
 * Nothing is sent until the "allow_tx" module parameter is set, so the
 * driver can first be loaded to read the hardware state only.
 *
 * For the gs-decon KMS driver it also switches the refresh rate: the DSIM's
 * command-mode TE and underrun timers, and the panel through
 * drm_panel_set_refresh_rate(). It only does so on the link it has timer
 * values for (gs_dsim_rates[]), recognised from what the bootloader left.
 *
 * Register layout from Google's GPL display driver
 * (google-modules/display/samsung/cal_9845/regs-dsim.h), which gs201
 * shares with gs101.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <video/mipi_display.h>

#include "gs-dsim.h"

#define DSIM_VERSION			0x0000
#define DSIM_LINK_STATUS1		0x000c
#define DSIM_DPHY_STATUS		0x001c
#define DSIM_CLK_CTRL			0x0020
#define DSIM_TIMEOUT			0x0028
#define DSIM_ESCMODE			0x002c
#define DSIM_ESCMODE_CMD_LPDT		BIT(7)
#define DSIM_UNDERRUN_CTRL		0x0034
#define DSIM_CONFIG			0x004c
#define DSIM_INTSRC			0x0050
#define DSIM_INTSRC_RX_DATA_DONE	BIT(18)
#define DSIM_INTMSK			0x0054
#define DSIM_PKTHDR			0x0058
#define DSIM_PKTHDR_BTA			BIT(24)
#define DSIM_PAYLOAD			0x005c
#define DSIM_RXFIFO			0x0060
#define DSIM_FIFOCTRL			0x0068
#define DSIM_FIFOCTRL_EMPTY_RX		BIT(12)
#define DSIM_FIFOCTRL_FULL_PH_SFR	BIT(11)
#define DSIM_FIFOCTRL_EMPTY_PH_SFR	BIT(10)
#define DSIM_FIFOCTRL_FULL_PL_SFR	BIT(9)
#define DSIM_FIFOCTRL_EMPTY_PL_SFR	BIT(8)
#define DSIM_CMD_CONFIG			0x0080
#define DSIM_CMD_CONFIG_PKT_GO_EN	BIT(16)
#define DSIM_CMD_TE_CTRL0		0x0084
#define DSIM_CMD_TE_CTRL1		0x0088
#define DSIM_CMD_STATUS			0x008c

#define DSIM_MAX_PL_FIFO		2048
/* the vendor driver's MIPI_WR_TIMEOUT */
#define DSIM_TX_TIMEOUT_US		80000
/* the vendor driver's MIPI_RD_TIMEOUT */
#define DSIM_RX_TIMEOUT_US		100000
#define DSIM_MAX_RX_FIFO		64

static bool allow_tx;
module_param(allow_tx, bool, 0644);
MODULE_PARM_DESC(allow_tx, "Send DSI packets to the panel (default: off, read-only)");

/*
 * Command-mode timers per refresh rate, for the link the bootloader sets up
 * for the S6E3HC4 (4 lanes at 1346 Mbps, DSC 3:1, 1440x3120, TE idle
 * 350 us): the LP underrun reference from Google's dsim_calc_underrun(),
 * and the TE protect/timeout pair the bootloader programs at 60 Hz, halved
 * for 120 Hz (both scale with the frame time). DISPLAY-PLAN.md D5.2; the
 * 120 Hz values ran clean on cheetah (gate D5c).
 */
struct gs_dsim_rate {
	unsigned int hz;
	u32 underrun;
	u32 te_ctrl1;
};

/*
 * One set per panel and link, found at probe from the timers the
 * bootloader left (its 60 Hz). Within a set, 120 Hz has half the TE
 * timeouts and the underrun of Google's dsim_calc_underrun(): the 60 Hz
 * value less the frame time saved (1e9 / 101 * (1/60 - 1/120) ns, te_var
 * 1) at the link's word clock, in units of 100 word clocks.
 */
static const struct gs_dsim_rate gs_dsim_rates[][2] = {
	/* Pixel 7 Pro, S6E3HC4 */
	{
		{ .hz = 60, .underrun = 0x1f09, .te_ctrl1 = 0x33eb3b26 },
		{ .hz = 120, .underrun = 0x03ec, .te_ctrl1 = 0x19f51d93 },
	},
	/*
	 * Pixel 9a, tg4c: word clock about 68.75 MHz (1100 Mbit/s lanes). The
	 * 60 Hz pair is the bootloader's; the 120 Hz underrun is derived from
	 * that value as above, which leaves less LP time than Google's own
	 * formula with tg4c's parameters would (0x69c), but ran clean.
	 */
	{
		{ .hz = 60, .underrun = 0x18c8, .te_ctrl1 = 0x2a82306d },
		{ .hz = 120, .underrun = 0x02a0, .te_ctrl1 = 0x15411836 },
	},
};

struct gs_dsim {
	struct device *dev;
	void __iomem *regs;
	struct mipi_dsi_host host;
	/* serialises use of the header and payload FIFOs */
	struct mutex lock;

	/* protects the two below and serialises rate switches */
	struct mutex panel_lock;
	struct drm_panel *panel;
	/* the timers in use, NULL if they are not ones from gs_dsim_rates[] */
	const struct gs_dsim_rate *rate;
	/* the set the bootloader's timers matched */
	const struct gs_dsim_rate *rates;
};

static struct platform_driver gs_dsim_driver;

static inline struct gs_dsim *host_to_dsim(struct mipi_dsi_host *host)
{
	return container_of(host, struct gs_dsim, host);
}

static void gs_dsim_dump(struct gs_dsim *dsim)
{
	static const struct {
		const char *name;
		u32 offset;
	} regs[] = {
		{ "VERSION", DSIM_VERSION },
		{ "LINK_STATUS1", DSIM_LINK_STATUS1 },
		{ "DPHY_STATUS", DSIM_DPHY_STATUS },
		{ "CLK_CTRL", DSIM_CLK_CTRL },
		{ "ESCMODE", DSIM_ESCMODE },
		{ "CONFIG", DSIM_CONFIG },
		{ "INTSRC", DSIM_INTSRC },
		{ "INTMSK", DSIM_INTMSK },
		{ "FIFOCTRL", DSIM_FIFOCTRL },
		{ "CMD_CONFIG", DSIM_CMD_CONFIG },
		{ "CMD_TE_CTRL0", DSIM_CMD_TE_CTRL0 },
		{ "CMD_TE_CTRL1", DSIM_CMD_TE_CTRL1 },
		{ "CMD_STATUS", DSIM_CMD_STATUS },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++)
		dev_info(dsim->dev, "%-12s (0x%03x) = 0x%08x\n", regs[i].name,
			 regs[i].offset, readl(dsim->regs + regs[i].offset));
}

static int gs_dsim_wait_fifo(struct gs_dsim *dsim, u32 mask, bool set)
{
	u32 val;

	if (set)
		return readl_poll_timeout(dsim->regs + DSIM_FIFOCTRL, val,
					  (val & mask) == mask, 10,
					  DSIM_TX_TIMEOUT_US);

	return readl_poll_timeout(dsim->regs + DSIM_FIFOCTRL, val,
				  !(val & mask), 10, DSIM_TX_TIMEOUT_US);
}

static int gs_dsim_attach(struct mipi_dsi_host *host,
			  struct mipi_dsi_device *device)
{
	struct gs_dsim *dsim = host_to_dsim(host);
	struct drm_panel *panel;

	dev_info(dsim->dev, "%s attached: %u lanes, mode flags 0x%lx\n",
		 dev_name(&device->dev), device->lanes, device->mode_flags);

	/* the panel driver adds its drm_panel before attaching */
	panel = of_drm_find_panel(device->dev.of_node);
	if (IS_ERR(panel)) {
		dev_warn(dsim->dev, "no drm_panel for %s (%pe): no rate switching\n",
			 dev_name(&device->dev), panel);
		return 0;
	}

	mutex_lock(&dsim->panel_lock);
	drm_panel_put(dsim->panel);
	dsim->panel = panel;
	mutex_unlock(&dsim->panel_lock);

	return 0;
}

static int gs_dsim_detach(struct mipi_dsi_host *host,
			  struct mipi_dsi_device *device)
{
	struct gs_dsim *dsim = host_to_dsim(host);

	mutex_lock(&dsim->panel_lock);
	if (dsim->panel && dsim->panel->dev == &device->dev) {
		drm_panel_put(dsim->panel);
		dsim->panel = NULL;
	}
	mutex_unlock(&dsim->panel_lock);

	return 0;
}

static int gs_dsim_send_header(struct gs_dsim *dsim, u8 h0, u8 h1, u8 h2, bool bta)
{
	int ret = gs_dsim_wait_fifo(dsim, DSIM_FIFOCTRL_FULL_PH_SFR, false);

	if (ret)
		return ret;
	writel(h0 | h1 << 8 | h2 << 16 | (bta ? DSIM_PKTHDR_BTA : 0),
	       dsim->regs + DSIM_PKTHDR);
	return gs_dsim_wait_fifo(dsim, DSIM_FIFOCTRL_EMPTY_PH_SFR, true);
}

/*
 * A read, as the vendor driver's dsim_read_data(): set the maximum return
 * packet size, send the read request, wait for RX_DATA_DONE (polled here),
 * then parse the response from the RX FIFO. Lock held.
 */
static ssize_t gs_dsim_read(struct gs_dsim *dsim, const struct mipi_dsi_msg *msg)
{
	struct mipi_dsi_packet packet;
	u8 *rx_buf = msg->rx_buf;
	u32 rx, val, intmsk;
	size_t rx_size = 0, i = 0;
	int ret;

	if (msg->rx_len > DSIM_MAX_RX_FIFO)
		return -EINVAL;
	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret)
		return ret;

	/* the vendor's BTA and LP RX timeouts, if the bootloader left none */
	if (!readl(dsim->regs + DSIM_TIMEOUT))
		writel(0xff << 16 | 0xffff, dsim->regs + DSIM_TIMEOUT);

	/* let RX_DATA_DONE latch in INTSRC while the read runs (no handler: polled) */
	intmsk = readl(dsim->regs + DSIM_INTMSK);
	writel(intmsk & ~DSIM_INTSRC_RX_DATA_DONE, dsim->regs + DSIM_INTMSK);
	writel(DSIM_INTSRC_RX_DATA_DONE, dsim->regs + DSIM_INTSRC);
	ret = gs_dsim_send_header(dsim, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
				  msg->rx_len & 0xff, (msg->rx_len >> 8) & 0xff, false) ?:
	      gs_dsim_send_header(dsim, packet.header[0], packet.header[1],
				  packet.header[2], true);
	if (ret) {
		writel(intmsk, dsim->regs + DSIM_INTMSK);
		return ret;
	}

	ret = readl_poll_timeout(dsim->regs + DSIM_INTSRC, val,
				 val & DSIM_INTSRC_RX_DATA_DONE, 10, DSIM_RX_TIMEOUT_US);
	if (ret) {
		dev_err(dsim->dev, "read 0x%02x timed out: INTSRC 0x%08x FIFOCTRL 0x%08x CMD_STATUS 0x%08x LINK_STATUS1 0x%08x ESCMODE 0x%08x\n",
			packet.header[1], readl(dsim->regs + DSIM_INTSRC),
			readl(dsim->regs + DSIM_FIFOCTRL), readl(dsim->regs + DSIM_CMD_STATUS),
			readl(dsim->regs + DSIM_LINK_STATUS1), readl(dsim->regs + DSIM_ESCMODE));
		writel(intmsk, dsim->regs + DSIM_INTMSK);
		return ret;
	}
	writel(intmsk, dsim->regs + DSIM_INTMSK);
	writel(DSIM_INTSRC_RX_DATA_DONE, dsim->regs + DSIM_INTSRC);

	rx = readl(dsim->regs + DSIM_RXFIFO);
	switch (rx & 0xff) {
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
		if (msg->rx_len > 1)
			rx_buf[1] = (rx >> 16) & 0xff;
		fallthrough;
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		rx_buf[0] = (rx >> 8) & 0xff;
		rx_size = min_t(size_t, msg->rx_len, 2);
		break;
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
		rx_size = min_t(size_t, (rx >> 8) & 0xffff, msg->rx_len);
		while (i < rx_size) {
			size_t max = min_t(size_t, rx_size, i + 4);

			rx = readl(dsim->regs + DSIM_RXFIFO);
			for (; i < max; i++, rx >>= 8)
				rx_buf[i] = rx & 0xff;
		}
		break;
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		dev_warn(dsim->dev, "read 0x%02x: error report 0x%04x\n",
			 packet.header[1], (rx >> 8) & 0xffff);
		ret = -EIO;
		break;
	default:
		dev_warn(dsim->dev, "read 0x%02x: unexpected response 0x%08x\n",
			 packet.header[1], rx);
		ret = -EIO;
	}

	/* leave the RX FIFO empty for the next read */
	for (i = 0; i < DSIM_MAX_RX_FIFO / 4 &&
	     !(readl(dsim->regs + DSIM_FIFOCTRL) & DSIM_FIFOCTRL_EMPTY_RX); i++)
		readl(dsim->regs + DSIM_RXFIFO);

	return ret ?: rx_size;
}

static ssize_t gs_dsim_transfer(struct mipi_dsi_host *host,
				const struct mipi_dsi_msg *msg)
{
	struct gs_dsim *dsim = host_to_dsim(host);
	struct mipi_dsi_packet packet;
	const u8 *p, *end;
	u32 word;
	int ret;

	if (!READ_ONCE(allow_tx)) {
		dev_dbg_ratelimited(dsim->dev,
				    "not sending packet 0x%02x: allow_tx is off\n",
				    msg->type);
		return -EPERM;
	}

	if (msg->rx_len) {
		mutex_lock(&dsim->lock);
		/* as for writes below: with packet go on, nothing goes out */
		if (readl(dsim->regs + DSIM_CMD_CONFIG) & DSIM_CMD_CONFIG_PKT_GO_EN)
			ret = -EBUSY;
		else
			ret = gs_dsim_read(dsim, msg);
		mutex_unlock(&dsim->lock);
		return ret;
	}

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret)
		return ret;

	if (packet.payload_length > DSIM_MAX_PL_FIFO)
		return -EINVAL;

	mutex_lock(&dsim->lock);

	/*
	 * With packet go enabled the FIFOs hold packets until software
	 * releases them in the TE window. The bootloader is not expected to
	 * leave it on; if it did, a write here would never go out.
	 */
	if (readl(dsim->regs + DSIM_CMD_CONFIG) & DSIM_CMD_CONFIG_PKT_GO_EN) {
		dev_warn_ratelimited(dsim->dev, "packet go is enabled, not sending\n");
		ret = -EBUSY;
		goto out;
	}

	ret = gs_dsim_wait_fifo(dsim, DSIM_FIFOCTRL_FULL_PH_SFR, false);
	if (ret)
		goto timeout;

	/* payload first, packed little-endian into 32-bit words */
	p = packet.payload;
	end = p + packet.payload_length;
	while (p < end) {
		size_t i, n = min_t(size_t, 4, end - p);

		word = 0;
		for (i = 0; i < n; i++)
			word |= (u32)p[i] << (8 * i);
		writel(word, dsim->regs + DSIM_PAYLOAD);
		p += n;
	}

	writel(packet.header[0] | packet.header[1] << 8 |
	       packet.header[2] << 16, dsim->regs + DSIM_PKTHDR);

	if (packet.payload_length) {
		ret = gs_dsim_wait_fifo(dsim, DSIM_FIFOCTRL_EMPTY_PL_SFR, true);
		if (ret)
			goto timeout;
	}
	ret = gs_dsim_wait_fifo(dsim, DSIM_FIFOCTRL_EMPTY_PH_SFR, true);
	if (ret)
		goto timeout;

	ret = packet.size;
	goto out;

timeout:
	dev_err(dsim->dev, "timed out sending packet 0x%02x 0x%02x\n",
		packet.header[0], packet.header[1]);
	gs_dsim_dump(dsim);
out:
	mutex_unlock(&dsim->lock);
	return ret;
}

/* Refresh rate switching, for gs-decon */

static const struct gs_dsim_rate *gs_dsim_find_rate(struct gs_dsim *dsim,
						    unsigned int hz)
{
	int i;

	if (!dsim->rates)
		return NULL;
	for (i = 0; i < ARRAY_SIZE(gs_dsim_rates[0]); i++)
		if (dsim->rates[i].hz == hz)
			return &dsim->rates[i];

	return NULL;
}

static void gs_dsim_write_timers(struct gs_dsim *dsim,
				 const struct gs_dsim_rate *rate)
{
	mutex_lock(&dsim->lock);
	writel(rate->underrun, dsim->regs + DSIM_UNDERRUN_CTRL);
	writel(rate->te_ctrl1, dsim->regs + DSIM_CMD_TE_CTRL1);
	mutex_unlock(&dsim->lock);
	dsim->rate = rate;
}

/**
 * gs_dsim_get - look up the DSIM behind a DECON
 * @consumer: the DECON device, which gets a device link to the DSIM
 * @np: the DSIM's device node
 *
 * Return: the DSIM, to be released with gs_dsim_put(), or an ERR_PTR():
 * -EPROBE_DEFER until gs-dsim-cmd is bound to it and a panel has attached.
 */
struct gs_dsim *gs_dsim_get(struct device *consumer, struct device_node *np)
{
	struct platform_device *pdev;
	struct gs_dsim *dsim = NULL;
	bool has_panel = false;

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	device_lock(&pdev->dev);
	if (pdev->dev.driver == &gs_dsim_driver.driver &&
	    device_is_bound(&pdev->dev))
		dsim = platform_get_drvdata(pdev);
	device_unlock(&pdev->dev);

	if (dsim) {
		mutex_lock(&dsim->panel_lock);
		has_panel = dsim->panel;
		mutex_unlock(&dsim->panel_lock);
	}
	if (!has_panel) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	if (!device_link_add(consumer, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		put_device(&pdev->dev);
		return ERR_PTR(-EINVAL);
	}

	return dsim;
}
EXPORT_SYMBOL_GPL(gs_dsim_get);

void gs_dsim_put(struct gs_dsim *dsim)
{
	put_device(dsim->dev);
}
EXPORT_SYMBOL_GPL(gs_dsim_put);

/* The rate the link runs at, or 0 if its timers are not ones we know. */
unsigned int gs_dsim_refresh_rate(struct gs_dsim *dsim)
{
	guard(mutex)(&dsim->panel_lock);
	return dsim->rate ? dsim->rate->hz : 0;
}
EXPORT_SYMBOL_GPL(gs_dsim_refresh_rate);

bool gs_dsim_can_set_refresh_rate(struct gs_dsim *dsim, unsigned int hz)
{
	guard(mutex)(&dsim->panel_lock);
	if (dsim->rate && dsim->rate->hz == hz)
		return true;

	return dsim->rate && gs_dsim_find_rate(dsim, hz) && READ_ONCE(allow_tx) &&
	       dsim->panel && dsim->panel->funcs &&
	       dsim->panel->funcs->set_refresh_rate;
}
EXPORT_SYMBOL_GPL(gs_dsim_can_set_refresh_rate);

/**
 * gs_dsim_set_refresh_rate - switch the link and the panel to another rate
 * @dsim: the DSIM
 * @hz: the new rate
 *
 * Going up, the timers change first and then the panel, going down the
 * other way round, as Google's driver does: the panel takes its new rate at
 * its next frame, and timers for the higher rate also work for a frame at
 * the lower one. The caller holds whatever bandwidth the higher rate needs
 * across the switch, and calls this right after a frame has been sent, so
 * that the commands go out in the idle time before the next TE.
 *
 * Return: 0, or a negative error code with the link left at the old rate.
 */
int gs_dsim_set_refresh_rate(struct gs_dsim *dsim, unsigned int hz)
{
	const struct gs_dsim_rate *old, *new = gs_dsim_find_rate(dsim, hz);
	int ret;

	guard(mutex)(&dsim->panel_lock);
	old = dsim->rate;
	if (!old || !new)
		return -EINVAL;
	if (old == new)
		return 0;
	if (!dsim->panel)
		return -ENODEV;
	if (!READ_ONCE(allow_tx))
		return -EPERM;

	if (new->hz > old->hz) {
		gs_dsim_write_timers(dsim, new);
		ret = drm_panel_set_refresh_rate(dsim->panel, new->hz);
		if (ret) {
			/* the panel may have taken part of it: send the old rate */
			drm_panel_set_refresh_rate(dsim->panel, old->hz);
			gs_dsim_write_timers(dsim, old);
		}
	} else {
		ret = drm_panel_set_refresh_rate(dsim->panel, new->hz);
		if (ret)
			drm_panel_set_refresh_rate(dsim->panel, old->hz);
		else
			gs_dsim_write_timers(dsim, new);
	}

	if (ret)
		dev_err(dsim->dev, "switching from %u to %u Hz failed: %d\n",
			old->hz, new->hz, ret);
	else
		dev_dbg(dsim->dev, "%u Hz\n", new->hz);

	return ret;
}
EXPORT_SYMBOL_GPL(gs_dsim_set_refresh_rate);

/**
 * gs_dsim_panel_sleep - switch the panel off and put it to sleep
 * @dsim: the DSIM
 *
 * Display off, sleep in and reset (drm_panel_disable() and
 * drm_panel_unprepare()). The panel loses its registers; only
 * gs_dsim_panel_wake() brings it back. The caller has stopped DECON from
 * sending frames, and has the link at the panel's cold-start rate.
 *
 * Return: 0, or a negative error code with the panel left on.
 */
int gs_dsim_panel_sleep(struct gs_dsim *dsim)
{
	guard(mutex)(&dsim->panel_lock);
	if (!dsim->panel)
		return -ENODEV;
	if (!READ_ONCE(allow_tx))
		return -EPERM;

	drm_panel_disable(dsim->panel);
	drm_panel_unprepare(dsim->panel);
	if (dsim->panel->prepared) {
		dev_err(dsim->dev, "panel did not go to sleep\n");
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(gs_dsim_panel_sleep);

/**
 * gs_dsim_panel_wake - reset and initialise the panel after a sleep
 * @dsim: the DSIM
 * @enable: false for the cold init only (drm_panel_prepare()), true for
 *	display on and the backlight (drm_panel_enable())
 *
 * The cold init leaves the panel at 60 Hz, so the link timers go back to
 * 60 Hz as well. The caller sends a frame between the two steps, so that
 * display on shows it rather than whatever the panel's memory held.
 *
 * Return: 0, or a negative error code.
 */
int gs_dsim_panel_wake(struct gs_dsim *dsim, bool enable)
{
	const struct gs_dsim_rate *cold = gs_dsim_find_rate(dsim, 60);

	guard(mutex)(&dsim->panel_lock);
	if (!dsim->panel)
		return -ENODEV;

	if (enable) {
		drm_panel_enable(dsim->panel);
		return dsim->panel->enabled ? 0 : -EIO;
	}

	if (!READ_ONCE(allow_tx))
		return -EPERM;
	if (dsim->rate && dsim->rate != cold)
		gs_dsim_write_timers(dsim, cold);
	drm_panel_prepare(dsim->panel);
	if (!dsim->panel->prepared) {
		dev_err(dsim->dev, "panel cold init failed\n");
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(gs_dsim_panel_wake);

static const struct mipi_dsi_host_ops gs_dsim_host_ops = {
	.attach = gs_dsim_attach,
	.detach = gs_dsim_detach,
	.transfer = gs_dsim_transfer,
};

static int gs_dsim_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs_dsim *dsim;
	u32 underrun, te_ctrl1;
	int ret, i, j;

	dsim = devm_kzalloc(dev, sizeof(*dsim), GFP_KERNEL);
	if (!dsim)
		return -ENOMEM;

	dsim->dev = dev;
	ret = devm_mutex_init(dev, &dsim->lock);
	if (ret)
		return ret;
	ret = devm_mutex_init(dev, &dsim->panel_lock);
	if (ret)
		return ret;

	dsim->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsim->regs))
		return PTR_ERR(dsim->regs);

	platform_set_drvdata(pdev, dsim);

	/* Bootloader state, for the record. Reads only. */
	gs_dsim_dump(dsim);

	underrun = readl(dsim->regs + DSIM_UNDERRUN_CTRL);
	te_ctrl1 = readl(dsim->regs + DSIM_CMD_TE_CTRL1);
	for (i = 0; i < ARRAY_SIZE(gs_dsim_rates); i++)
		for (j = 0; j < ARRAY_SIZE(gs_dsim_rates[0]); j++)
			if (gs_dsim_rates[i][j].underrun == underrun &&
			    gs_dsim_rates[i][j].te_ctrl1 == te_ctrl1) {
				dsim->rates = gs_dsim_rates[i];
				dsim->rate = &gs_dsim_rates[i][j];
			}
	if (dsim->rate)
		dev_info(dev, "link timers for %u Hz\n", dsim->rate->hz);
	else
		dev_info(dev, "unknown link timers (UNDERRUN_CTRL 0x%08x): no rate switching\n",
			 underrun);

	dsim->host.dev = dev;
	dsim->host.ops = &gs_dsim_host_ops;
	ret = mipi_dsi_host_register(&dsim->host);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register DSI host\n");

	return 0;
}

static void gs_dsim_remove(struct platform_device *pdev)
{
	struct gs_dsim *dsim = platform_get_drvdata(pdev);

	mipi_dsi_host_unregister(&dsim->host);
	drm_panel_put(dsim->panel);
}

static const struct of_device_id gs_dsim_of_match[] = {
	{ .compatible = "google,gs201-dsim" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gs_dsim_of_match);

static struct platform_driver gs_dsim_driver = {
	.probe = gs_dsim_probe,
	.remove = gs_dsim_remove,
	.driver = {
		.name = "gs-dsim-cmd",
		.of_match_table = gs_dsim_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(gs_dsim_driver);

MODULE_DESCRIPTION("Google gs201 DSIM command path for a bootloader-initialised display");
MODULE_LICENSE("GPL");
