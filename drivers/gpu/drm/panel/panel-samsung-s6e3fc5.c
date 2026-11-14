// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S6E3FC5 DSI command-mode AMOLED panel (Google Pixel 7 Pro)
 *
 * Command sequences from Google's GPL panel drive
 * (google-modules/display/samsung/panel/panel-samsung-s6e3fc5.c),
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
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

/* Normal range; 2048..4095 are the high brightness mode (HBM) levels. */
#define S6E3FC5_MAX_BRIGHTNESS		2047
#define S6E3FC5_DEFAULT_BRIGHTNESS	1023

#define S6E3FC5_WRCTRLD_BCTRL		0x20
#define S6E3FC5_WRCTRLD_HBM		0xc0
#define S6E3FC5_WRCTRLD_LOCAL_HBM	0x10

/* Google's LOCAL_HBM_MAX_TIMEOUT_MS */
#define S6E3FC5_LHBM_TIMEOUT_MS		3000

static bool allow_lhbm;
module_param(allow_lhbm, bool, 0644);
MODULE_PARM_DESC(allow_lhbm,
		 "Allow local high brightness mode for the fingerprint sensor");

struct s6e3fc5 {
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

static const struct regulator_bulk_data s6e3fc5_supplies[] = {
	{ .supply = "vddio" },
	{ .supply = "avdd" },
};

static inline struct s6e3fc5 *to_s6e3fc5(struct drm_panel *panel)
{
	return container_of(panel, struct s6e3fc5, panel);
}

static int s6e3fc5_set_lhbm(struct s6e3fc5 *ctx, bool on);
static void s6e3fc5_lhbm_stop(struct s6e3fc5 *ctx);

#define s6e3fc5_unlock_lvl2(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0x5a, 0x5a)
#define s6e3fc5_lock_lvl2(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0xa5, 0xa5)
#define s6e3fc5_sync_begin(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xe4, 0x00, 0x2c, 0x2c, 0xa2, 0x00, 0x00)
#define s6e3fc5_sync_end(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xe4, 0x00, 0x2c, 0x2c, 0x82, 0x00, 0x00)
#define s6e3fc5_freq_update(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf7, 0x0f)

static void s6e3fc5_reset(struct s6e3fc5 *ctx)
{
	if (!ctx->reset_gpio)
		return; /* shared pad: the bootloader owns it */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

/* Cold init after reset, for PVT panels (Google's s6e3fc5_init_cmds). */
static void s6e3fc5_init(struct mipi_dsi_multi_context *ctx)
{
	/* Google's s6e3fc5_init_cmds: sleep out, TE on, 1080x2400 window. */
	mipi_dsi_dcs_exit_sleep_mode_multi(ctx);
	mipi_dsi_msleep(ctx, 120);
	mipi_dsi_dcs_write_seq_multi(ctx, MIPI_DCS_SET_TEAR_ON, 0x00);
	mipi_dsi_dcs_set_column_address_multi(ctx, 0, 1080 - 1);
	mipi_dsi_dcs_set_page_address_multi(ctx, 0, 2400 - 1);
}

/*
 * Operating features and refresh rate: Google's s6e3fc5_set_panel_feat()
 * with every feature written, as the vendor driver does on each enable,
 * for HS operation, HBM off, IRC on and a manual (fixed) refresh rate.
 * 120 Hz runs with early exit and a fixed TE, 60 Hz without early exit
 * and with a changeable TE, as in Google's s6e3fc5_update_refresh_mode().
 * The panel takes the new rate at the next frame (0xf7 0x0f).
 */
static void s6e3fc5_write_rate(struct mipi_dsi_multi_context *ctx,
			       unsigned int hz)
{
	/* Google's s6e3fc5_change_frequency: test key, 0x60, update. */
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0x5a, 0x5a);
	if (hz == 90)
		mipi_dsi_dcs_write_seq_multi(ctx, 0x60, 0x00, 0x00);
	else
		mipi_dsi_dcs_write_seq_multi(ctx, 0x60, 0x08, 0x00);
	s6e3fc5_freq_update(ctx);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0xa5, 0xa5);
}

/* DSC on, then the PPS generated from the lynx DSC configuration. */
static void s6e3fc5_dsc(struct mipi_dsi_multi_context *ctx)
{
	struct s6e3fc5 *p = mipi_dsi_get_drvdata(ctx->dsi);
	struct drm_dsc_picture_parameter_set pps;

	drm_dsc_pps_payload_pack(&pps, &p->dsc);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x9d, 0x01);
	mipi_dsi_picture_parameter_set_multi(ctx, &pps);
}

static void s6e3fc5_display_on(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_dcs_set_display_on_multi(ctx);
}

static void s6e3fc5_display_off(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_dcs_set_display_off_multi(ctx);
}

static int s6e3fc5_prepare(struct drm_panel *panel)
{
	struct s6e3fc5 *ctx = to_s6e3fc5(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	int ret;

	guard(mutex)(&ctx->lock);
	ret = regulator_bulk_enable(ARRAY_SIZE(s6e3fc5_supplies), ctx->supplies);
	if (ret < 0)
		return ret;
	/* the vendor driver waits 5 ms between VDDI and VDDD/VCI */
	usleep_range(5000, 6000);

	s6e3fc5_reset(ctx);

	s6e3fc5_dsc(&dsi_ctx);
	s6e3fc5_init(&dsi_ctx);
	s6e3fc5_unlock_lvl2(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x0c);	/* WQHD output */
	s6e3fc5_lock_lvl2(&dsi_ctx);
	/* the vendor driver cold-starts at 60 Hz */
	s6e3fc5_write_rate(&dsi_ctx, 60);
	/* normal brightness control, HBM off */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     S6E3FC5_WRCTRLD_BCTRL);
	if (dsi_ctx.accum_err) {
		if (ctx->reset_gpio)
			gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(s6e3fc5_supplies), ctx->supplies);
		return dsi_ctx.accum_err;
	}

	/* the reset cleared these; display on comes with enable */
	ctx->prepared = true;
	ctx->display_off = true;
	ctx->pixel_off = false;

	return 0;
}

static int s6e3fc5_enable(struct drm_panel *panel)
{
	struct s6e3fc5 *ctx = to_s6e3fc5(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	s6e3fc5_display_on(&dsi_ctx);
	if (!dsi_ctx.accum_err)
		ctx->display_off = false;

	return dsi_ctx.accum_err;
}

static int s6e3fc5_disable(struct drm_panel *panel)
{
	struct s6e3fc5 *ctx = to_s6e3fc5(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	s6e3fc5_lhbm_stop(ctx);
	s6e3fc5_display_off(&dsi_ctx);
	if (!dsi_ctx.accum_err)
		ctx->display_off = true;

	return dsi_ctx.accum_err;
}

static int s6e3fc5_unprepare(struct drm_panel *panel)
{
	struct s6e3fc5 *ctx = to_s6e3fc5(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	s6e3fc5_lhbm_stop(ctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 10);
	mipi_dsi_dcs_nop_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);

	/* the panel loses its registers: only prepare() brings it back */
	ctx->prepared = false;
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(s6e3fc5_supplies), ctx->supplies);

	return 0;
}

/*
 * The display controller switches between these with
 * drm_panel_set_refresh_rate(), keeping the panel on.
 */
static int s6e3fc5_set_refresh_rate(struct drm_panel *panel, unsigned int hz)
{
	struct s6e3fc5 *ctx = to_s6e3fc5(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	if (hz != 60 && hz != 90)
		return -EINVAL;

	/* always sent in full: the caller may not know the panel's state */
	guard(mutex)(&ctx->lock);
	if (!ctx->prepared)
		return -ENODEV;
	s6e3fc5_write_rate(&dsi_ctx, hz);

	return dsi_ctx.accum_err;
}

/* Pixel 7a: 1080x2400, 32/12/26 horizontal and 12/4/26 vertical blanks. */
#define S6E3FC5_MODE(clk, flags) {					\
	.clock = (clk),							\
	.hdisplay = 1080,						\
	.hsync_start = 1080 + 32,					\
	.hsync_end = 1080 + 32 + 12,					\
	.htotal = 1080 + 32 + 12 + 26,					\
	.vdisplay = 2400,						\
	.vsync_start = 2400 + 12,					\
	.vsync_end = 2400 + 12 + 4,					\
	.vtotal = 2400 + 12 + 4 + 26,					\
	.width_mm = 70,							\
	.height_mm = 149,						\
	.type = DRM_MODE_TYPE_DRIVER | (flags),				\
}

/* 1080x2400 at 60 Hz (the vendor driver's preferred mode) and 90 Hz */
static const struct drm_display_mode s6e3fc5_modes[] = {
	S6E3FC5_MODE(168498, DRM_MODE_TYPE_PREFERRED),
	S6E3FC5_MODE(252747, 0),
};

static int s6e3fc5_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s6e3fc5_modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &s6e3fc5_modes[i]);
		if (!mode)
			return i;
		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}
	connector->display_info.width_mm = s6e3fc5_modes[0].width_mm;
	connector->display_info.height_mm = s6e3fc5_modes[0].height_mm;

	return i;
}

static const struct drm_panel_funcs s6e3fc5_panel_funcs = {
	.prepare = s6e3fc5_prepare,
	.enable = s6e3fc5_enable,
	.disable = s6e3fc5_disable,
	.unprepare = s6e3fc5_unprepare,
	.get_modes = s6e3fc5_get_modes,
	.set_refresh_rate = s6e3fc5_set_refresh_rate,
};

/*
 * Local HBM: Google's s6e3fc5_lhbm_extra_cmds, then WRCTRLD with the LHBM
 * bit. The circle is centred on the sensor (x 0x2d0, y 0x939 in panel
 * pixels) with a 6 mm radius. Off is WRCTRLD back to normal. Called with
 * ctx->lock held, on a prepared panel.
 */
static int s6e3fc5_set_lhbm(struct s6e3fc5 *ctx, bool on)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	if (on) {
		s6e3fc5_unlock_lvl2(&dsi_ctx);
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
		s6e3fc5_lock_lvl2(&dsi_ctx);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
					     S6E3FC5_WRCTRLD_BCTRL |
					     S6E3FC5_WRCTRLD_LOCAL_HBM);
	} else {
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
					     S6E3FC5_WRCTRLD_BCTRL);
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
static void s6e3fc5_lhbm_stop(struct s6e3fc5 *ctx)
{
	if (!ctx->lhbm)
		return;
	cancel_delayed_work(&ctx->lhbm_off_work);
	if (ctx->prepared)
		s6e3fc5_set_lhbm(ctx, false);
	ctx->lhbm = false;
	if (ctx->bl)
		sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
}

static void s6e3fc5_lhbm_off_work(struct work_struct *work)
{
	struct s6e3fc5 *ctx = container_of(to_delayed_work(work),
					   struct s6e3fc5, lhbm_off_work);

	guard(mutex)(&ctx->lock);
	if (ctx->lhbm) {
		dev_info(&ctx->dsi->dev, "local HBM timed out\n");
		s6e3fc5_lhbm_stop(ctx);
	}
}

static ssize_t local_hbm_mode_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct s6e3fc5 *ctx = bl_get_data(to_backlight_device(dev));

	return sysfs_emit(buf, "%d\n", ctx->lhbm);
}

static ssize_t local_hbm_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct s6e3fc5 *ctx = bl_get_data(bl);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;
	if (on && !allow_lhbm)
		return -EPERM;

	guard(mutex)(&ctx->lock);
	if (!on) {
		s6e3fc5_lhbm_stop(ctx);
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

	ret = s6e3fc5_set_lhbm(ctx, true);
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
	struct s6e3fc5 *ctx = bl_get_data(to_backlight_device(dev));

	return sysfs_emit(buf, "%u\n", ctx->lhbm_timeout_ms);
}

static ssize_t local_hbm_max_timeout_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct s6e3fc5 *ctx = bl_get_data(to_backlight_device(dev));
	unsigned int ms;
	int ret;

	ret = kstrtouint(buf, 0, &ms);
	if (ret)
		return ret;
	/* never unbounded: the circle is far above normal brightness */
	if (!ms || ms > S6E3FC5_LHBM_TIMEOUT_MS)
		return -EINVAL;

	guard(mutex)(&ctx->lock);
	ctx->lhbm_timeout_ms = ms;

	return count;
}
static DEVICE_ATTR_RW(local_hbm_max_timeout);

static struct attribute *s6e3fc5_bl_attrs[] = {
	&dev_attr_local_hbm_mode.attr,
	&dev_attr_local_hbm_max_timeout.attr,
	NULL
};

static const struct attribute_group s6e3fc5_bl_group = {
	.attrs = s6e3fc5_bl_attrs,
};

/*
 * Brightness 0 turns all pixels off (0x22) instead of writing DBV 0, as
 * the vendor driver does; the next non-zero level turns them back on
 * (0x13). A blanked backlight (bl_power) runs the display off sequence,
 * which keeps the panel out of sleep with its registers intact, and
 * unblanking runs the display on sequence.
 */
static int s6e3fc5_bl_update_locked(struct backlight_device *bl)
{
	struct s6e3fc5 *ctx = bl_get_data(bl);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u16 brightness = backlight_get_brightness(bl);

	/*
	 * In reset or asleep: keep the new state, enable() applies it
	 * through backlight_enable().
	 */
	if (!ctx->prepared)
		return 0;

	if (backlight_is_blank(bl) || !brightness)
		s6e3fc5_lhbm_stop(ctx);

	if (backlight_is_blank(bl)) {
		if (!ctx->display_off) {
			s6e3fc5_display_off(&dsi_ctx);
			if (!dsi_ctx.accum_err)
				ctx->display_off = true;
		}
		return dsi_ctx.accum_err;
	}

	if (ctx->display_off) {
		s6e3fc5_display_on(&dsi_ctx);
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

static int s6e3fc5_bl_update_status(struct backlight_device *bl)
{
	struct s6e3fc5 *ctx = bl_get_data(bl);

	guard(mutex)(&ctx->lock);
	return s6e3fc5_bl_update_locked(bl);
}

static const struct backlight_ops s6e3fc5_bl_ops = {
	.update_status = s6e3fc5_bl_update_status,
};

static int s6e3fc5_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = S6E3FC5_DEFAULT_BRIGHTNESS,
		.max_brightness = S6E3FC5_MAX_BRIGHTNESS,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};
	struct backlight_device *bl;
	struct s6e3fc5 *ctx;
	bool handover;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct s6e3fc5, panel,
				   &s6e3fc5_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_mutex_init(dev, &ctx->lock);
	if (ret)
		return ret;

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(s6e3fc5_supplies),
					    s6e3fc5_supplies, &ctx->supplies);
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

	/*
	 * gpiod_get_value_cansleep() asks the controller for the line
	 * direction, which the shared-proxy GPIO path rejects. Take the
	 * bootloader's running panel over directly instead.
	 */
	ret = gpiod_direction_output(ctx->reset_gpio, 0);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to set up reset-gpios\n");
	handover = true;

	if (handover) {
		/* Hold the rails the bootloader turned on. */
		ret = regulator_bulk_enable(ARRAY_SIZE(s6e3fc5_supplies),
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

	/* Pixel 7a: 1080x2400, two 540x48 slices, 8 bpc, 8 bpp. */
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.pic_width = 1080;
	ctx->dsc.pic_height = 2400;
	ctx->dsc.slice_height = 48;
	ctx->dsc.slice_width = 540;
	ctx->dsc.slice_count = 2;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4;
	ctx->dsc.block_pred_enable = true;
	ctx->dsc.rc_model_size = 8192;
	ctx->dsc.initial_xmit_delay = 512;
	ctx->dsc.convert_rgb = true;
	ctx->dsc.mux_word_size = 48;
	ret = drm_dsc_compute_rc_parameters(&ctx->dsc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to compute DSC parameters\n");
	dsi->dsc = &ctx->dsc;

	ctx->panel.prepare_prev_first = true;

	bl = devm_backlight_device_register(dev, dev_name(dev), dev, ctx,
					    &s6e3fc5_bl_ops, &props);
	if (IS_ERR(bl))
		return dev_err_probe(dev, PTR_ERR(bl),
				     "failed to register backlight\n");
	ctx->bl = bl;
	ctx->panel.backlight = bl;
	/* after the backlight, so the timer is cancelled before it goes away */
	ctx->lhbm_timeout_ms = S6E3FC5_LHBM_TIMEOUT_MS;
	ret = devm_delayed_work_autocancel(dev, &ctx->lhbm_off_work,
					   s6e3fc5_lhbm_off_work);
	if (ret)
		return ret;
	ret = devm_device_add_group(&bl->dev, &s6e3fc5_bl_group);
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

static void s6e3fc5_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3fc5 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id s6e3fc5_of_match[] = {
	{ .compatible = "samsung,s6e3fc5" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s6e3fc5_of_match);

static struct mipi_dsi_driver s6e3fc5_driver = {
	.probe = s6e3fc5_probe,
	.remove = s6e3fc5_remove,
	.driver = {
		.name = "panel-samsung-s6e3fc5",
		.of_match_table = s6e3fc5_of_match,
	},
};
module_mipi_dsi_driver(s6e3fc5_driver);

MODULE_DESCRIPTION("DRM driver for the Samsung S6E3FC5 DSI command-mode panel");
MODULE_LICENSE("GPL");
