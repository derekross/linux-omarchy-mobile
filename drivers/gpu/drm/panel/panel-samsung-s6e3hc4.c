// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S6E3HC4 DSI command-mode AMOLED panel (Google Pixel 7 Pro)
 *
 * Command sequences from Google's GPL panel driver
 * (google-modules/display/samsung/panel/panel-samsung-s6e3hc4.c),
 * restricted to the PVT and later panels that ship in phones.
 *
 * The bootloader powers the panel on and leaves it running. When the
 * reset line is released at probe, the driver takes the panel over as it
 * is: it sends nothing until the brightness or the power state changes.
 *
 * Local high brightness mode (LHBM) lights a bright circle over the
 * under-display fingerprint sensor. It is controlled through the backlight
 * device's local_hbm_mode attribute, named as in Google's driver, and
 * switches itself off after local_hbm_max_timeout milliseconds.
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

/* Normal range; 2048..4095 are the high brightness mode (HBM) levels. */
#define S6E3HC4_MAX_BRIGHTNESS		2047
#define S6E3HC4_DEFAULT_BRIGHTNESS	1023

#define S6E3HC4_WRCTRLD_BCTRL		0x20
#define S6E3HC4_WRCTRLD_HBM		0xc0
#define S6E3HC4_WRCTRLD_LOCAL_HBM	0x10

/* Google's LOCAL_HBM_MAX_TIMEOUT_MS */
#define S6E3HC4_LHBM_TIMEOUT_MS		3000

static bool allow_lhbm;
module_param(allow_lhbm, bool, 0644);
MODULE_PARM_DESC(allow_lhbm,
		 "Allow local high brightness mode for the fingerprint sensor");

struct s6e3hc4 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
	struct drm_dsc_config dsc;
	struct backlight_device *bl;
	/*
	 * Keeps multi-command sequences (0xb0 offsets) from interleaving and
	 * guards the panel state below.
	 */
	struct mutex lock;
	struct delayed_work lhbm_off_work;
	unsigned int lhbm_timeout_ms;
	/* out of reset and sleep: commands reach the panel */
	bool prepared;
	bool display_off;
	bool pixel_off;
	bool lhbm;
};

static const struct regulator_bulk_data s6e3hc4_supplies[] = {
	{ .supply = "vddi" },
	{ .supply = "vddd" },
	{ .supply = "vci" },
};

static const u8 s6e3hc4_wqhd_pps[sizeof(struct drm_dsc_picture_parameter_set)] = {
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x0c, 0x30,
	0x05, 0xa0, 0x00, 0x34, 0x02, 0xd0, 0x02, 0xd0,
	0x02, 0x00, 0x02, 0x68, 0x00, 0x20, 0x05, 0xc6,
	0x00, 0x0a, 0x00, 0x0c, 0x01, 0xe2, 0x01, 0x78,
	0x18, 0x00, 0x10, 0xf0, 0x03, 0x0c, 0x20, 0x00,
	0x06, 0x0b, 0x0b, 0x33, 0x0e, 0x1c, 0x2a, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7b,
	0x7d, 0x7e, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xbe, 0x19, 0xfc, 0x19, 0xfa, 0x19, 0xf8,
	0x1a, 0x38, 0x1a, 0x78, 0x1a, 0xb6, 0x2a, 0xf6,
	0x2b, 0x34, 0x2b, 0x74, 0x3b, 0x74, 0x6b, 0xf4,
};

static inline struct s6e3hc4 *to_s6e3hc4(struct drm_panel *panel)
{
	return container_of(panel, struct s6e3hc4, panel);
}

static int s6e3hc4_set_lhbm(struct s6e3hc4 *ctx, bool on);
static void s6e3hc4_lhbm_stop(struct s6e3hc4 *ctx);

#define s6e3hc4_unlock_lvl2(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0x5a, 0x5a)
#define s6e3hc4_lock_lvl2(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0xa5, 0xa5)
#define s6e3hc4_sync_begin(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xe4, 0x00, 0x2c, 0x2c, 0xa2, 0x00, 0x00)
#define s6e3hc4_sync_end(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xe4, 0x00, 0x2c, 0x2c, 0x82, 0x00, 0x00)
#define s6e3hc4_freq_update(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf7, 0x0f)

static void s6e3hc4_reset(struct s6e3hc4 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

/* Cold init after reset, for PVT panels (Google's s6e3hc4_init_cmds). */
static void s6e3hc4_init(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_dcs_exit_sleep_mode_multi(ctx);
	mipi_dsi_msleep(ctx, 10);

	s6e3hc4_unlock_lvl2(ctx);
	/* disable black insertion */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xbb, 0x0b, 0x04, 0x04, 0x41, 0x3d, 0x0c);
	/* delete toggle */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x58, 0x94);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x94, 0x0c, 0x60, 0x0c, 0x60);
	s6e3hc4_sync_begin(ctx);
	/* VLIN1 7.9 V */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x12, 0xb1);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb1, 0x08);
	/* VGH 7.4 V */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x0e, 0xf4);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf4, 0x18, 0x18, 0x18, 0x18, 0x18);
	/* VREG 4.5 V */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x31, 0xf4);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00);
	s6e3hc4_sync_end(ctx);
	s6e3hc4_lock_lvl2(ctx);
	mipi_dsi_msleep(ctx, 110);

	/* TE on, a bare 0x35 as the vendor driver sends it */
	mipi_dsi_dcs_write_seq_multi(ctx, MIPI_DCS_SET_TEAR_ON);

	s6e3hc4_unlock_lvl2(ctx);
	/* AOD SAP settings */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x04, 0xf6);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf6, 0x5a);
	/* enable SP */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x02, 0xf3, 0x68);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x68, 0x09, 0x09, 0x09, 0x0a, 0x0a, 0x0a);
	/* FFC: 165 MHz, 1% tolerance */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x36, 0xc5);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xc5, 0x11, 0x10, 0x50, 0x05, 0x4e, 0x74);
	/* NS transition flicker setting */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xab, 0x83);
	/* TSP sync auto mode */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x3c, 0xb9);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x19, 0x09);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x40, 0xb9);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x30, 0x03);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x05, 0xf2);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf2, 0xc8, 0xc0);
	/* frame insertion count */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x10, 0xbd);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xbd, 0x00);
	s6e3hc4_freq_update(ctx);
	s6e3hc4_lock_lvl2(ctx);

	mipi_dsi_dcs_set_column_address_multi(ctx, 0, 1440 - 1);
	mipi_dsi_dcs_set_page_address_multi(ctx, 0, 3120 - 1);
}

/*
 * Operating features and refresh rate: Google's s6e3hc4_set_panel_feat()
 * with every feature written, as the vendor driver does on each enable,
 * for HS operation, HBM off, IRC on and a manual (fixed) refresh rate.
 * 120 Hz runs with early exit and a fixed TE, 60 Hz without early exit
 * and with a changeable TE, as in Google's s6e3hc4_update_refresh_mode().
 * The panel takes the new rate at the next frame (0xf7 0x0f).
 */
static void s6e3hc4_write_rate(struct mipi_dsi_multi_context *ctx,
			       unsigned int hz)
{
	const bool early_exit = hz == 120;

	s6e3hc4_unlock_lvl2(ctx);
	/* TE width, HS */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x04, 0xb9);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x0c, 0x42, 0x00, 0x1b,
				     0x0c, 0x42, 0x00, 0x1b,
				     0x0c, 0x42, 0x00, 0x1b);
	/* TE: fixed (0x51) with early exit, changeable (0x04) without */
	if (early_exit)
		mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x51);
	else
		mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x04);
	/* TE2: the same option, rising edge 0x10, falling edge 0x30 */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x42, 0xf2);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf2, 0x0d);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x01, 0xb9);
	if (early_exit) {
		mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x51);
		mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x22, 0xb9);
		mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x00, 0x10, 0x00, 0x30,
					     0x00, 0x10, 0x00, 0x30);
	} else {
		mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x04);
		mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x1e, 0xb9);
		mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x00, 0x10, 0x00, 0x30);
	}
	/* IRC on */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x01, 0x9b, 0x92);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x92, 0x21);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x02, 0xf3, 0x68);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x68, 0x09, 0x09, 0x09, 0x0a, 0x0a, 0x0a);
	/* HS operation */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf2, 0x01);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x60, 0x00);
	/* early exit on/off */
	if (early_exit)
		mipi_dsi_dcs_write_seq_multi(ctx, 0xbd, 0x21, 0x01, 0x83, 0x03, 0x03);
	else
		mipi_dsi_dcs_write_seq_multi(ctx, 0xbd, 0x21, 0x81, 0x83, 0x03, 0x03);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x1e, 0xbd);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xbd, 0x00, 0x00, 0x00, 0x02,
				     0x00, 0x06, 0x00, 0x16);
	/* manual frame rate: 0x00 = 120 Hz, 0x01 = 60 Hz */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xbd, 0x21);
	if (early_exit)
		mipi_dsi_dcs_write_seq_multi(ctx, 0x60, 0x00);
	else
		mipi_dsi_dcs_write_seq_multi(ctx, 0x60, 0x01);
	s6e3hc4_freq_update(ctx);
	s6e3hc4_lock_lvl2(ctx);
}

/* DSC on, then the vendor PPS (DSI data type 0x0a), verbatim. */
static void s6e3hc4_dsc(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_dcs_write_seq_multi(ctx, 0x9d, 0x01);
	mipi_dsi_picture_parameter_set_multi(ctx,
		(const struct drm_dsc_picture_parameter_set *)s6e3hc4_wqhd_pps);
}

static void s6e3hc4_display_on(struct mipi_dsi_multi_context *ctx)
{
	s6e3hc4_unlock_lvl2(ctx);
	s6e3hc4_sync_begin(ctx);
	/* AMP type change */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x4f, 0xf4);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf4, 0x70);
	/* VREG 6.8 V */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x31, 0xf4);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf4, 0x17, 0x17, 0x17, 0x17, 0x17);
	s6e3hc4_sync_end(ctx);
	s6e3hc4_lock_lvl2(ctx);
	mipi_dsi_dcs_set_display_on_multi(ctx);
}

static void s6e3hc4_display_off(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_dcs_set_display_off_multi(ctx);
	s6e3hc4_unlock_lvl2(ctx);
	s6e3hc4_sync_begin(ctx);
	/* VREG 4.5 V */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x31, 0xf4);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00);
	/* AMP type change */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, 0x00, 0x4f, 0xf4);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf4, 0x50);
	s6e3hc4_sync_end(ctx);
	s6e3hc4_lock_lvl2(ctx);
}

static int s6e3hc4_prepare(struct drm_panel *panel)
{
	struct s6e3hc4 *ctx = to_s6e3hc4(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	int ret;

	guard(mutex)(&ctx->lock);
	ret = regulator_bulk_enable(ARRAY_SIZE(s6e3hc4_supplies), ctx->supplies);
	if (ret < 0)
		return ret;
	/* the vendor driver waits 5 ms between VDDI and VDDD/VCI */
	usleep_range(5000, 6000);

	s6e3hc4_reset(ctx);

	s6e3hc4_dsc(&dsi_ctx);
	s6e3hc4_init(&dsi_ctx);
	s6e3hc4_unlock_lvl2(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x0c);	/* WQHD output */
	s6e3hc4_lock_lvl2(&dsi_ctx);
	/* the vendor driver cold-starts at 60 Hz */
	s6e3hc4_write_rate(&dsi_ctx, 60);
	/* normal brightness control, HBM off */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     S6E3HC4_WRCTRLD_BCTRL);
	if (dsi_ctx.accum_err) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(s6e3hc4_supplies), ctx->supplies);
		return dsi_ctx.accum_err;
	}

	/* the reset cleared these; display on comes with enable */
	ctx->prepared = true;
	ctx->display_off = true;
	ctx->pixel_off = false;

	return 0;
}

static int s6e3hc4_enable(struct drm_panel *panel)
{
	struct s6e3hc4 *ctx = to_s6e3hc4(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	s6e3hc4_display_on(&dsi_ctx);
	if (!dsi_ctx.accum_err)
		ctx->display_off = false;

	return dsi_ctx.accum_err;
}

static int s6e3hc4_disable(struct drm_panel *panel)
{
	struct s6e3hc4 *ctx = to_s6e3hc4(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	s6e3hc4_lhbm_stop(ctx);
	s6e3hc4_display_off(&dsi_ctx);
	if (!dsi_ctx.accum_err)
		ctx->display_off = true;

	return dsi_ctx.accum_err;
}

static int s6e3hc4_unprepare(struct drm_panel *panel)
{
	struct s6e3hc4 *ctx = to_s6e3hc4(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	s6e3hc4_lhbm_stop(ctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 10);
	mipi_dsi_dcs_nop_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);

	/* the panel loses its registers: only prepare() brings it back */
	ctx->prepared = false;
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(s6e3hc4_supplies), ctx->supplies);

	return 0;
}

/*
 * The display controller switches between these with
 * drm_panel_set_refresh_rate(), keeping the panel on.
 */
static int s6e3hc4_set_refresh_rate(struct drm_panel *panel, unsigned int hz)
{
	struct s6e3hc4 *ctx = to_s6e3hc4(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	if (hz != 60 && hz != 120)
		return -EINVAL;

	/* always sent in full: the caller may not know the panel's state */
	guard(mutex)(&ctx->lock);
	if (!ctx->prepared)
		return -ENODEV;
	s6e3hc4_write_rate(&dsi_ctx, hz);

	return dsi_ctx.accum_err;
}

#define S6E3HC4_MODE(hz, flags) {					\
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

/* 1440x3120 at 60 Hz (the vendor driver's preferred mode) and 120 Hz */
static const struct drm_display_mode s6e3hc4_modes[] = {
	S6E3HC4_MODE(60, DRM_MODE_TYPE_PREFERRED),
	S6E3HC4_MODE(120, 0),
};

static int s6e3hc4_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s6e3hc4_modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &s6e3hc4_modes[i]);
		if (!mode)
			return i;
		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}
	connector->display_info.width_mm = s6e3hc4_modes[0].width_mm;
	connector->display_info.height_mm = s6e3hc4_modes[0].height_mm;

	return i;
}

static const struct drm_panel_funcs s6e3hc4_panel_funcs = {
	.prepare = s6e3hc4_prepare,
	.enable = s6e3hc4_enable,
	.disable = s6e3hc4_disable,
	.unprepare = s6e3hc4_unprepare,
	.get_modes = s6e3hc4_get_modes,
	.set_refresh_rate = s6e3hc4_set_refresh_rate,
};

/*
 * Local HBM: Google's s6e3hc4_lhbm_extra_cmds, then WRCTRLD with the LHBM
 * bit. The circle is centred on the sensor (x 0x2d0, y 0x939 in panel
 * pixels) with a 6 mm radius. Off is WRCTRLD back to normal. Called with
 * ctx->lock held, on a prepared panel.
 */
static int s6e3hc4_set_lhbm(struct s6e3hc4 *ctx, bool on)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	if (on) {
		s6e3hc4_unlock_lvl2(&dsi_ctx);
		/* area set */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x02, 0x1e, 0x92);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x92, 0x20, 0x88, 0x71,
					     0x39, 0x8a, 0x01);
		/* centre position */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x02, 0x24, 0x92);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x92, 0x2d, 0x09, 0x39);
		/* circle size */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x02, 0x27, 0x92);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x92, 0x78);
		s6e3hc4_lock_lvl2(&dsi_ctx);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
					     S6E3HC4_WRCTRLD_BCTRL |
					     S6E3HC4_WRCTRLD_LOCAL_HBM);
	} else {
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
					     S6E3HC4_WRCTRLD_BCTRL);
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
static void s6e3hc4_lhbm_stop(struct s6e3hc4 *ctx)
{
	if (!ctx->lhbm)
		return;
	cancel_delayed_work(&ctx->lhbm_off_work);
	if (ctx->prepared)
		s6e3hc4_set_lhbm(ctx, false);
	ctx->lhbm = false;
	if (ctx->bl)
		sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
}

static void s6e3hc4_lhbm_off_work(struct work_struct *work)
{
	struct s6e3hc4 *ctx = container_of(to_delayed_work(work),
					   struct s6e3hc4, lhbm_off_work);

	guard(mutex)(&ctx->lock);
	if (ctx->lhbm) {
		dev_info(&ctx->dsi->dev, "local HBM timed out\n");
		s6e3hc4_lhbm_stop(ctx);
	}
}

static ssize_t local_hbm_mode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct s6e3hc4 *ctx = bl_get_data(to_backlight_device(dev));

	return sysfs_emit(buf, "%d\n", ctx->lhbm);
}

static ssize_t local_hbm_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct s6e3hc4 *ctx = bl_get_data(bl);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;
	if (on && !allow_lhbm)
		return -EPERM;

	guard(mutex)(&ctx->lock);
	if (!on) {
		s6e3hc4_lhbm_stop(ctx);
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

	ret = s6e3hc4_set_lhbm(ctx, true);
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
	struct s6e3hc4 *ctx = bl_get_data(to_backlight_device(dev));

	return sysfs_emit(buf, "%u\n", ctx->lhbm_timeout_ms);
}

static ssize_t local_hbm_max_timeout_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct s6e3hc4 *ctx = bl_get_data(to_backlight_device(dev));
	unsigned int ms;
	int ret;

	ret = kstrtouint(buf, 0, &ms);
	if (ret)
		return ret;
	/* never unbounded: the circle is far above normal brightness */
	if (!ms || ms > S6E3HC4_LHBM_TIMEOUT_MS)
		return -EINVAL;

	guard(mutex)(&ctx->lock);
	ctx->lhbm_timeout_ms = ms;

	return count;
}
static DEVICE_ATTR_RW(local_hbm_max_timeout);

static struct attribute *s6e3hc4_bl_attrs[] = {
	&dev_attr_local_hbm_mode.attr,
	&dev_attr_local_hbm_max_timeout.attr,
	NULL
};

static const struct attribute_group s6e3hc4_bl_group = {
	.attrs = s6e3hc4_bl_attrs,
};

/*
 * Brightness 0 turns all pixels off (0x22) instead of writing DBV 0, as
 * the vendor driver does; the next non-zero level turns them back on
 * (0x13). A blanked backlight (bl_power) runs the display off sequence,
 * which keeps the panel out of sleep with its registers intact, and
 * unblanking runs the display on sequence.
 */
static int s6e3hc4_bl_update_locked(struct backlight_device *bl)
{
	struct s6e3hc4 *ctx = bl_get_data(bl);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u16 brightness = backlight_get_brightness(bl);

	/*
	 * In reset or asleep: keep the new state, enable() applies it
	 * through backlight_enable().
	 */
	if (!ctx->prepared)
		return 0;

	if (backlight_is_blank(bl) || !brightness)
		s6e3hc4_lhbm_stop(ctx);

	if (backlight_is_blank(bl)) {
		if (!ctx->display_off) {
			s6e3hc4_display_off(&dsi_ctx);
			if (!dsi_ctx.accum_err)
				ctx->display_off = true;
		}
		return dsi_ctx.accum_err;
	}

	if (ctx->display_off) {
		s6e3hc4_display_on(&dsi_ctx);
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
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_ENTER_NORMAL_MODE);
		if (dsi_ctx.accum_err)
			return dsi_ctx.accum_err;
		ctx->pixel_off = false;
	}

	/* DBV, most significant byte first */
	return mipi_dsi_dcs_set_display_brightness_large(ctx->dsi, brightness);
}

static int s6e3hc4_bl_update_status(struct backlight_device *bl)
{
	struct s6e3hc4 *ctx = bl_get_data(bl);

	guard(mutex)(&ctx->lock);
	return s6e3hc4_bl_update_locked(bl);
}

static const struct backlight_ops s6e3hc4_bl_ops = {
	.update_status = s6e3hc4_bl_update_status,
};

static int s6e3hc4_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = S6E3HC4_DEFAULT_BRIGHTNESS,
		.max_brightness = S6E3HC4_MAX_BRIGHTNESS,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};
	struct backlight_device *bl;
	struct s6e3hc4 *ctx;
	bool handover;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct s6e3hc4, panel,
				   &s6e3hc4_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_mutex_init(dev, &ctx->lock);
	if (ret)
		return ret;

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(s6e3hc4_supplies),
					    s6e3hc4_supplies, &ctx->supplies);
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
		ret = regulator_bulk_enable(ARRAY_SIZE(s6e3hc4_supplies),
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

	/* Two 720x52 slices, 8 bpc, 8 bpp, as in the vendor PPS. */
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_height = 52;
	ctx->dsc.slice_width = 720;
	ctx->dsc.slice_count = 2;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4;
	ctx->dsc.block_pred_enable = true;
	dsi->dsc = &ctx->dsc;

	ctx->panel.prepare_prev_first = true;

	bl = devm_backlight_device_register(dev, dev_name(dev), dev, ctx,
					    &s6e3hc4_bl_ops, &props);
	if (IS_ERR(bl))
		return dev_err_probe(dev, PTR_ERR(bl),
				     "failed to register backlight\n");
	ctx->bl = bl;
	ctx->panel.backlight = bl;
	/* after the backlight, so the timer is cancelled before it goes away */
	ctx->lhbm_timeout_ms = S6E3HC4_LHBM_TIMEOUT_MS;
	ret = devm_delayed_work_autocancel(dev, &ctx->lhbm_off_work,
					   s6e3hc4_lhbm_off_work);
	if (ret)
		return ret;
	ret = devm_device_add_group(&bl->dev, &s6e3hc4_bl_group);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	dev_info(dev, "%s\n", handover ? "taken over from the bootloader" :
		 "held in reset");

	return 0;
}

static void s6e3hc4_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3hc4 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id s6e3hc4_of_match[] = {
	{ .compatible = "samsung,s6e3hc4" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s6e3hc4_of_match);

static struct mipi_dsi_driver s6e3hc4_driver = {
	.probe = s6e3hc4_probe,
	.remove = s6e3hc4_remove,
	.driver = {
		.name = "panel-samsung-s6e3hc4",
		.of_match_table = s6e3hc4_of_match,
	},
};
module_mipi_dsi_driver(s6e3hc4_driver);

MODULE_DESCRIPTION("DRM driver for the Samsung S6E3HC4 DSI command-mode panel");
MODULE_LICENSE("GPL");
