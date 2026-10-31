// SPDX-License-Identifier: GPL-2.0-only
/*
 * BOE NT37290 DSI command-mode AMOLED panel (Google Pixel 7 Pro, BOE variant)
 *
 * Copyright (c) 2021 Google LLC
 *
 * Command sequences from Google's GPL panel driver
 * (google-modules/display/samsung/panel/panel-boe-nt37290.c,
 * branch android14-gs-pixel-6.1),
 * restricted to DVT1 and later panels that ship in phones: the
 * PROTO1.1-only LHBM tail is dropped, the EVT1_1+ brightness select,
 * the DVT1 power/IRC settings and the EVT1+ MIPI termination and LHBM
 * calibration tables are taken unconditionally.
 *
 * The bootloader powers the panel on and leaves it running. When the
 * reset line is released at probe, the driver takes the panel over as it
 * is: it sends nothing until the brightness or the power state changes.
 *
 * WQHD only: the FHD scaled modes, the runtime resolution switch, AOD/
 * low-power modes and the idle (early exit) machinery are not ported.
 * Brightness is written to DBV 1:1 without the vendor's nonlinear band
 * conversion; userspace owns the perceived curve.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

/*
 * Normal-range brightness per the vendor brightness capability
 * (nt37290_brightness_capability): levels 3..3584. Levels 3585..4094
 * are the HBM band (600-1000 nits), which the vendor driver enters
 * through its HBM mode; leave that band unused for now. 0 turns the
 * pixels off.
 */
#define NT37290_MAX_BRIGHTNESS		3584
#define NT37290_DEFAULT_BRIGHTNESS	1023

/* Google's LOCAL_HBM_MAX_TIMEOUT_MS */
#define NT37290_LHBM_TIMEOUT_MS		3000

static bool allow_lhbm;
module_param(allow_lhbm, bool, 0644);
MODULE_PARM_DESC(allow_lhbm,
		 "Allow local high brightness mode for the fingerprint sensor");

#define NT37290_DDIC_ID_REG		0xac

/* Frame-rate control (2Fh): mode in bits 7:4, OSC2 FCON in bits 2:0. */
#define NT37290_DFC_MANUAL_120		0x00
#define NT37290_DFC_PANEL_LP		0x30

struct nt37290 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
	struct drm_dsc_config dsc;
	struct backlight_device *bl;
	/*
	 * Keeps multi-command sequences (0x6f offset writes) from
	 * interleaving and guards the panel state below.
	 */
	struct mutex lock;
	struct delayed_work lhbm_off_work;
	unsigned int lhbm_timeout_ms;
	/* last brightness written to the panel (the LHBM boost builds on it) */
	u16 dbv;
	/* out of reset and awake: commands reach the panel */
	bool prepared;
	bool display_off;
	bool pixel_off;
	bool lhbm;
};

static const struct regulator_bulk_data nt37290_supplies[] = {
	{ .supply = "vddi" },
	{ .supply = "vddd" },
	{ .supply = "vci" },
};

/*
 * Vendor PPS tables (PPS1 WQHD 1440x3120, PPS2 FHD 1080x2340), written
 * to registers 0x91/0x93 as Google's driver does - not the standard PPS
 * data type. PPS2 is loaded unconditionally, as in the vendor init.
 */
static void nt37290_dsc_init(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_dcs_write_seq_multi(ctx, 0x91,
				     0x89, 0x28, 0x00, 0x1e, 0xd2, 0x00, 0x02,
				     0x86, 0x03, 0x28, 0x00, 0x0a, 0x03, 0x97,
				     0x02, 0x8b, 0x10, 0xf0);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x93,
				     0x89, 0x28, 0x00, 0x1e, 0xd2, 0x00, 0x02,
				     0x25, 0x02, 0xc5, 0x00, 0x07, 0x03, 0x97,
				     0x03, 0x64, 0x10, 0xf0);
}

static inline struct nt37290 *to_nt37290(struct drm_panel *panel)
{
	return container_of(panel, struct nt37290, panel);
}

static int nt37290_set_lhbm(struct nt37290 *ctx, bool on);
static void nt37290_lhbm_stop(struct nt37290 *ctx);

#define nt37290_page(ctx, page) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, (page))

/* Cold init after reset (Google's nt37290_init_cmds, DVT1+/EVT1_1+ path). */
static void nt37290_init(struct mipi_dsi_multi_context *ctx)
{
	/* set for higher MIPI speed: 1346 Mbps */
	mipi_dsi_dcs_write_seq_multi(ctx, 0x1f, 0xf0);
	/* gamma curve */
	mipi_dsi_dcs_write_seq_multi(ctx, 0x26, 0x00);
	/* TE output line, a bare 0x35 as the vendor driver sends it */
	mipi_dsi_dcs_write_seq_multi(ctx, MIPI_DCS_SET_TEAR_ON);
	/* select brightness value (~142 nits, EVT1.1 and after) */
	mipi_dsi_dcs_write_seq_multi(ctx, 0x51, 0x08, 0x8d, 0x08, 0x8d,
				     0x0f, 0xfe);
	/* control brightness */
	mipi_dsi_dcs_write_seq_multi(ctx, 0x53, 0x20);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x5a, 0x01);
	/* change refresh frame to 1 after 2Ch command in skip mode */
	nt37290_page(ctx, 0x00);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xba, 0x00);
	/* dimming config: 32 frame */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb2, 0x19);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x05);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb2, 0x20);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x06);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb2, 0x20);

	/* CMD2 Page 1 */
	nt37290_page(ctx, 0x01);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xc5, 0x00, 0x0b, 0x0b, 0x0b);
	/* CMD2 Page 0: display driving voltage Vkeep 6.35V->5.95V (DVT1+) */
	nt37290_page(ctx, 0x00);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x14);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xc2, 0x00, 0x50);
	/* CMD2 Page 3: tune internal power on sequence */
	nt37290_page(ctx, 0x03);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x10);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb6, 0x1f);
	/* CMD2 Page 5: tune internal power on sequence */
	nt37290_page(ctx, 0x05);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb8, 0x04, 0x00, 0x00, 0x02);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x04, 0x00, 0x00, 0x02);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xba, 0x84, 0x00, 0x00, 0x02);
	/* CMD2 Page 3: tune internal power on sequence */
	nt37290_page(ctx, 0x03);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x12);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xd8, 0x14);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x34);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb6, 0x0f, 0x00, 0x3c, 0x00, 0x3c,
				     0x00, 0x3c, 0x00, 0x3c);
	/* CMD2 Page 8: IRC IP settings (DVT1+) */
	nt37290_page(ctx, 0x08);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x10);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb7, 0x08, 0x00, 0x08, 0x00, 0x08,
				     0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00,
				     0x08, 0x00, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x28);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb8, 0x08, 0x00, 0x08, 0x00, 0x08,
				     0x00);
	/* CMD2 Page 5: remove long TE2 pulse */
	nt37290_page(ctx, 0x05);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb6, 0x06, 0x03);

	/* CMD3 Page 0 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x1b);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf4, 0x55);
	/* CMD3 Page 1 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xff, 0xaa, 0x55, 0xa5, 0x81);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x12);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf5, 0x00);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x09);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf9, 0x10);
	/* CMD3 Page 2: MIPI termination resistor 90 ohm (EVT1+) */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xff, 0xaa, 0x55, 0xa5, 0x82);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf2, 0x37);
	/* CMD3 Page 3 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xff, 0xaa, 0x55, 0xa5, 0x83);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x14);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf8, 0x0d);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf9, 0x06);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xfa, 0x06);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xfb, 0x06);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0x06);
	/* CMD3 Page 4 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xff, 0xaa, 0x55, 0xa5, 0x84);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x1c);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf8, 0x3a);

	mipi_dsi_dcs_exit_sleep_mode_multi(ctx);
	mipi_dsi_msleep(ctx, 120);

	mipi_dsi_dcs_set_column_address_multi(ctx, 0, 1440 - 1);
	mipi_dsi_dcs_set_page_address_multi(ctx, 0, 3120 - 1);
}

/*
 * DSC scaling off (1440x3120), row address and the WQHD PPS selection,
 * sent on every enable as in Google's nt37290_enable().
 */
static void nt37290_dsc_wqhd(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_dcs_write_seq_multi(ctx, 0x8f, 0x00);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x2b, 0x00, 0x00, 0x0c, 0x2f);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x90, 0x03, 0x03);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x03, 0x00);
}

/*
 * Refresh rate: Google's nt37290_update_panel_feat() in manual mode,
 * early exit off, OSC2 index 0. 120 Hz runs in manual DFC mode, 60 Hz in
 * panel-low-power mode with manual frame insertion, as the vendor does.
 * The panel takes the new rate at the next frame (0x2c stream write).
 * Followed by the TE2 update (Google's nt37290_update_te2(), changeable
 * option, default timing rising 0, falling 48).
 */
static void nt37290_write_rate(struct mipi_dsi_multi_context *ctx,
			       unsigned int hz)
{
	if (hz == 120) {
		/* DFC mode manual */
		mipi_dsi_dcs_write_seq_multi(ctx, 0x2f,
					     NT37290_DFC_MANUAL_120);
		/* 120Hz gamma band */
		mipi_dsi_dcs_write_seq_multi(ctx, 0x26, 0x00);
		/* restore TE timing (no shift) */
		mipi_dsi_dcs_write_seq_multi(ctx, 0x44, 0x00, 0x00);
	} else {
		/* DFC mode panel low power */
		mipi_dsi_dcs_write_seq_multi(ctx, 0x2f, NT37290_DFC_PANEL_LP);
		/* early exit off */
		mipi_dsi_dcs_write_seq_multi(ctx, 0x5a, 0x01);
		/* manual frame insertion: 60 Hz */
		nt37290_page(ctx, 0x00);
		mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x1c);
		mipi_dsi_dcs_write_seq_multi(ctx, 0xba, 0x91, 0x01, 0x01, 0x00,
					     0x01, 0x01, 0x01, 0x00);
		mipi_dsi_dcs_write_seq_multi(ctx, 0x2c);
		/* OSC2 clock index 0 */
		mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x03);
		mipi_dsi_dcs_write_seq_multi(ctx, 0xc0, 0x20);
		/* TE shift 8.2 ms */
		mipi_dsi_dcs_write_seq_multi(ctx, 0x44, 0x00, 0x01);
	}

	/* TE2: changeable option, default timing (CMD2 Page 3) */
	nt37290_page(ctx, 0x03);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xc3, 0x02);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x6f, 0x04);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xc3, 0x02);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xc4, 0x00, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x10, 0x30);
}

static void nt37290_reset(struct nt37290 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int nt37290_prepare(struct drm_panel *panel)
{
	struct nt37290 *ctx = to_nt37290(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	int ret;

	guard(mutex)(&ctx->lock);
	ret = regulator_bulk_enable(ARRAY_SIZE(nt37290_supplies), ctx->supplies);
	if (ret < 0)
		return ret;
	/* the vendor driver waits 5 ms between VDDI and VDDD/VCI */
	usleep_range(5000, 6000);

	nt37290_reset(ctx);

	nt37290_init(&dsi_ctx);
	nt37290_dsc_init(&dsi_ctx);
	/* WQHD output selection and the enable-path DSC sequence */
	nt37290_dsc_wqhd(&dsi_ctx);
	/* the vendor driver cold-starts at 60 Hz */
	nt37290_write_rate(&dsi_ctx, 60);
	if (dsi_ctx.accum_err) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(nt37290_supplies),
				       ctx->supplies);
		return dsi_ctx.accum_err;
	}

	/* the reset cleared these; display on comes with enable */
	ctx->prepared = true;
	ctx->display_off = true;
	ctx->pixel_off = false;

	return 0;
}

static int nt37290_enable(struct drm_panel *panel)
{
	struct nt37290 *ctx = to_nt37290(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	if (!dsi_ctx.accum_err)
		ctx->display_off = false;

	return dsi_ctx.accum_err;
}

static int nt37290_disable(struct drm_panel *panel)
{
	struct nt37290 *ctx = to_nt37290(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	nt37290_lhbm_stop(ctx);
	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	if (!dsi_ctx.accum_err)
		ctx->display_off = true;

	return dsi_ctx.accum_err;
}

static int nt37290_unprepare(struct drm_panel *panel)
{
	struct nt37290 *ctx = to_nt37290(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	nt37290_lhbm_stop(ctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 10);
	mipi_dsi_dcs_nop_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	/* the panel loses its registers: only prepare() brings it back */
	ctx->prepared = false;
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(nt37290_supplies), ctx->supplies);

	return 0;
}

/*
 * The display controller switches between these with
 * drm_panel_set_refresh_rate(), keeping the panel on.
 */
static int nt37290_set_refresh_rate(struct drm_panel *panel, unsigned int hz)
{
	struct nt37290 *ctx = to_nt37290(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	if (hz != 60 && hz != 120)
		return -EINVAL;

	/* always sent in full: the caller may not know the panel's state */
	guard(mutex)(&ctx->lock);
	if (!ctx->prepared)
		return -ENODEV;
	nt37290_write_rate(&dsi_ctx, hz);

	return dsi_ctx.accum_err;
}

#define NT37290_MODE(hz, flags) {					\
	.clock = (1440 + 80 + 24 + 36) * (3120 + 12 + 4 + 14) * (hz) / 1000, \
	.hdisplay = 1440,						\
	.hsync_start = 1440 + 80,					\
	.hsync_end = 1440 + 80 + 24,					\
	.htotal = 1440 + 80 + 24 + 36,					\
	.vdisplay = 3120,						\
	.vsync_start = 3120 + 12,					\
	.vsync_end = 3120 + 12 + 4,					\
	.vtotal = 3120 + 12 + 4 + 14,					\
	.width_mm = 71,							\
	.height_mm = 155,						\
	.type = DRM_MODE_TYPE_DRIVER | (flags),				\
}

/* 1440x3120 at 60 Hz (the vendor driver's base rate) and 120 Hz */
static const struct drm_display_mode nt37290_modes[] = {
	NT37290_MODE(60, DRM_MODE_TYPE_PREFERRED),
	NT37290_MODE(120, 0),
};

static int nt37290_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(nt37290_modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &nt37290_modes[i]);
		if (!mode)
			return i;
		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}
	connector->display_info.width_mm = nt37290_modes[0].width_mm;
	connector->display_info.height_mm = nt37290_modes[0].height_mm;

	return i;
}

static const struct drm_panel_funcs nt37290_panel_funcs = {
	.prepare = nt37290_prepare,
	.enable = nt37290_enable,
	.disable = nt37290_disable,
	.unprepare = nt37290_unprepare,
	.get_modes = nt37290_get_modes,
	.set_refresh_rate = nt37290_set_refresh_rate,
};

/*
 * Local high brightness mode: the calibration tables were loaded at cold
 * init (the LHBM on setting of Google's driver, circle centre x=720,
 * y=2361 in panel pixels). Switching runs through the vendor sequence:
 * the LHBM DBV (4x the last normal level) goes to the page-0 registers,
 * then the FPS gamma timing, FPS mode, and the 0x85 LHBM on command.
 * Off is the mirror image. Called with ctx->lock held, on a lit panel.
 */
static int nt37290_set_lhbm(struct nt37290 *ctx, bool on)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	if (on) {
		/*
		 * LHBM DBV value write: 4x the last normal level, into
		 * the page-0 registers. The bytes are runtime values, so
		 * the sequence goes through the buffer write, not the
		 * constant-only write_seq_multi.
		 */
		u16 level = ctx->dbv * 4;
		u8 val1 = level >> 8;
		u8 val2 = level & 0xff;
		u8 dbv_payload[] = { 0xdf, val1, val2, val1, val2,
				     val1, val2 };

		nt37290_page(&dsi_ctx, 0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x4c);
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, dbv_payload,
						ARRAY_SIZE(dbv_payload));
		/* FPS gamma timing, enter FPS mode */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2f, 0x02);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x87, 0x01);
		/* LHBM on */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x85);
	} else {
		/* LHBM off */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x86);
		/* exit FPS mode, normal gamma timing */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x87, 0x00);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2f, 0x00);
	}
	if (!dsi_ctx.accum_err)
		ctx->lhbm = on;

	return dsi_ctx.accum_err;
}

/*
 * Turn LHBM off before the panel blanks, dims to 0 or sleeps. Called with
 * ctx->lock held. If the off command fails the state is dropped anyway:
 * display off, pixel off and sleep all end the circle.
 */
static void nt37290_lhbm_stop(struct nt37290 *ctx)
{
	if (!ctx->lhbm)
		return;
	cancel_delayed_work(&ctx->lhbm_off_work);
	if (ctx->prepared)
		nt37290_set_lhbm(ctx, false);
	ctx->lhbm = false;
	if (ctx->bl)
		sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
}

static void nt37290_lhbm_off_work(struct work_struct *work)
{
	struct nt37290 *ctx = container_of(to_delayed_work(work),
					   struct nt37290, lhbm_off_work);

	guard(mutex)(&ctx->lock);
	if (ctx->lhbm) {
		dev_info(&ctx->dsi->dev, "local HBM timed out\n");
		nt37290_lhbm_stop(ctx);
	}
}

static ssize_t local_hbm_mode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct nt37290 *ctx = bl_get_data(to_backlight_device(dev));

	return sysfs_emit(buf, "%d\n", ctx->lhbm);
}

static ssize_t local_hbm_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct nt37290 *ctx = bl_get_data(bl);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;
	if (on && !allow_lhbm)
		return -EPERM;

	guard(mutex)(&ctx->lock);
	if (!on) {
		nt37290_lhbm_stop(ctx);
		return count;
	}
	/* as Google's driver: only on a lit panel */
	if (!ctx->prepared || ctx->display_off || ctx->pixel_off ||
	    backlight_is_blank(bl) || !backlight_get_brightness(bl))
		return -EPERM;
	if (ctx->lhbm) {
		/* re-enabling restarts the timeout */
		mod_delayed_work(system_wq, &ctx->lhbm_off_work,
				 msecs_to_jiffies(ctx->lhbm_timeout_ms));
		return count;
	}

	ret = nt37290_set_lhbm(ctx, true);
	if (ret)
		return ret;
	queue_delayed_work(system_wq, &ctx->lhbm_off_work,
			   msecs_to_jiffies(ctx->lhbm_timeout_ms));
	dev_dbg(dev, "local HBM on\n");

	return count;
}
static DEVICE_ATTR_RW(local_hbm_mode);

static ssize_t local_hbm_max_timeout_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct nt37290 *ctx = bl_get_data(to_backlight_device(dev));

	return sysfs_emit(buf, "%u\n", ctx->lhbm_timeout_ms);
}

static ssize_t local_hbm_max_timeout_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct nt37290 *ctx = bl_get_data(bl);
	unsigned int ms;
	int ret;

	ret = kstrtouint(buf, 0, &ms);
	if (ret)
		return ret;
	/* never unbounded: the circle is far above normal brightness */
	if (!ms || ms > NT37290_LHBM_TIMEOUT_MS)
		return -EINVAL;

	guard(mutex)(&ctx->lock);
	ctx->lhbm_timeout_ms = ms;

	return count;
}
static DEVICE_ATTR_RW(local_hbm_max_timeout);

static struct attribute *nt37290_bl_attrs[] = {
	&dev_attr_local_hbm_mode.attr,
	&dev_attr_local_hbm_max_timeout.attr,
	NULL
};

static const struct attribute_group nt37290_bl_group = {
	.attrs = nt37290_bl_attrs,
};

/*
 * Brightness 0 turns all pixels off (0x22) instead of writing DBV 0, as
 * the template driver does; the next non-zero level turns them back on
 * (0x13). A blanked backlight (bl_power) runs the display off sequence,
 * which keeps the panel out of sleep with its registers intact, and
 * unblanking runs the display on sequence. DBV is written 1:1 without
 * the vendor's nonlinear band conversion.
 */
static int nt37290_bl_update_locked(struct backlight_device *bl)
{
	struct nt37290 *ctx = bl_get_data(bl);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	/*
	 * In reset or asleep: keep the new state, enable() applies it
	 * through backlight_enable().
	 */
	if (!ctx->prepared)
		return 0;

	if (backlight_is_blank(bl) || !brightness)
		nt37290_lhbm_stop(ctx);

	if (backlight_is_blank(bl)) {
		if (!ctx->display_off) {
			mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
			if (!dsi_ctx.accum_err)
				ctx->display_off = true;
		}
		return dsi_ctx.accum_err;
	}

	if (ctx->display_off) {
		mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
		if (dsi_ctx.accum_err)
			return dsi_ctx.accum_err;
		ctx->display_off = false;
	}

	if (!brightness) {
		if (!ctx->pixel_off) {
			mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x22);
			if (!dsi_ctx.accum_err)
				ctx->pixel_off = true;
		}
		return dsi_ctx.accum_err;
	}

	if (ctx->pixel_off) {
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx,
					     MIPI_DCS_ENTER_NORMAL_MODE);
		if (dsi_ctx.accum_err)
			return dsi_ctx.accum_err;
		ctx->pixel_off = false;
	}

	/* DBV, most significant byte first */
	ret = mipi_dsi_dcs_set_display_brightness_large(ctx->dsi, brightness);
	if (!ret)
		ctx->dbv = brightness;

	return ret;
}

static int nt37290_bl_update_status(struct backlight_device *bl)
{
	struct nt37290 *ctx = bl_get_data(bl);

	guard(mutex)(&ctx->lock);
	return nt37290_bl_update_locked(bl);
}

static const struct backlight_ops nt37290_bl_ops = {
	.update_status = nt37290_bl_update_status,
};

static int nt37290_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = NT37290_DEFAULT_BRIGHTNESS,
		.max_brightness = NT37290_MAX_BRIGHTNESS,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};
	struct backlight_device *bl;
	struct nt37290 *ctx;
	bool handover;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct nt37290, panel,
				   &nt37290_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_mutex_init(dev, &ctx->lock);
	if (ret)
		return ret;

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(nt37290_supplies),
					    nt37290_supplies, &ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	/*
	 * Keep the line as the bootloader left it: driving it here would
	 * reset a running panel and blank the screen.
	 */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset-gpios\n");

	ret = gpiod_get_value_cansleep(ctx->reset_gpio);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read reset-gpios\n");
	handover = !ret;

	if (handover) {
		/* Hold the rails the bootloader turned on. */
		ret = regulator_bulk_enable(ARRAY_SIZE(nt37290_supplies),
					    ctx->supplies);
		if (ret < 0)
			return dev_err_probe(dev, ret, "failed to enable regulators\n");
		ret = gpiod_direction_output(ctx->reset_gpio, 0);
		/*
		 * The bootloader left it prepared and enabled: let a display
		 * driver's drm_panel_disable()/unprepare() reach it.
		 */
		ctx->prepared = true;
		ctx->panel.prepared = true;
		ctx->panel.enabled = true;
	} else {
		ret = gpiod_direction_output(ctx->reset_gpio, 1);
	}
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to set up reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	/* Two 720x30 slices, 8 bpc, 8 bpp, as in the vendor PPS. */
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_height = 30;
	ctx->dsc.slice_width = 720;
	ctx->dsc.slice_count = 2;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4;
	ctx->dsc.block_pred_enable = true;
	dsi->dsc = &ctx->dsc;

	ctx->panel.prepare_prev_first = true;

	bl = devm_backlight_device_register(dev, dev_name(dev), dev, ctx,
					    &nt37290_bl_ops, &props);
	if (IS_ERR(bl))
		return dev_err_probe(dev, PTR_ERR(bl),
				     "failed to register backlight\n");
	ctx->bl = bl;
	ctx->panel.backlight = bl;
	/* after the backlight, so the timer is cancelled before it goes away */
	ctx->lhbm_timeout_ms = NT37290_LHBM_TIMEOUT_MS;
	ret = devm_delayed_work_autocancel(dev, &ctx->lhbm_off_work,
					   nt37290_lhbm_off_work);
	if (ret)
		return ret;
	ret = devm_device_add_group(&bl->dev, &nt37290_bl_group);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	/*
	 * Google's driver reads the DDIC ID (panel_id_reg 0xac) to pick the
	 * panel revision; a phone always carries a production panel, and the
	 * wrong panel answers nothing, so the DT compatible decides here.
	 */
	dev_info(dev, "%s\n", handover ? "taken over from the bootloader" :
		 "held in reset");

	return 0;
}

static void nt37290_remove(struct mipi_dsi_device *dsi)
{
	struct nt37290 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	/* End a lit circle before devres tears down the LHBM attributes. */
	scoped_guard(mutex, &ctx->lock)
		nt37290_lhbm_stop(ctx);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id nt37290_of_match[] = {
	{ .compatible = "boe,nt37290" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nt37290_of_match);

static struct mipi_dsi_driver nt37290_driver = {
	.probe = nt37290_probe,
	.remove = nt37290_remove,
	.driver = {
		.name = "panel-boe-nt37290",
		.of_match_table = nt37290_of_match,
	},
};
module_mipi_dsi_driver(nt37290_driver);

MODULE_DESCRIPTION("DRM driver for the BOE NT37290 DSI command-mode panel");
MODULE_LICENSE("GPL");
