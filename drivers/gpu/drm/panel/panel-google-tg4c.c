// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google TG4C DSI command-mode AMOLED panel (Google Pixel 9a)
 *
 * Command sequences from Google's MIT-licensed panel driver
 * (google-modules/display/panels/tegu/panel-gs-tg4c.c).
 *
 * The bootloader powers the panel on, initialises it and leaves it running
 * at 60 Hz. The driver takes it over as it is and never resets it or
 * touches its supplies: it puts it to sleep and wakes it (sleep in/out,
 * the panel keeps its registers), switches it between 60 and 120 Hz and
 * sets its brightness. A panel without the bootloader's initialisation is
 * not handled.
 *
 * The Pixel 9a comes with one of three panels, so the driver reads the
 * panel's ID before its first command, and a panel that is not a TG4C
 * gets no commands.
 */

#include <linux/backlight.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#include <video/mipi_display.h>

#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

/*
 * Google's normal range, up to 1200 nits; the levels above it are high
 * brightness mode and are left alone.
 */
#define TG4C_MAX_BRIGHTNESS	3628
#define TG4C_DEFAULT_BRIGHTNESS	1829
#define TG4C_LHBM_TIMEOUT_MS	3000
/* Google's LHBM gamma set thresholds */
#define TG4C_LHBM_GAMMASET1	3628
#define TG4C_LHBM_GAMMASET2	2774
#define TG4C_LHBM_GAMMASET3	2186

/* local high brightness mode (the fingerprint circle): off unless allowed */
static bool allow_lhbm;
module_param(allow_lhbm, bool, 0644);
MODULE_PARM_DESC(allow_lhbm, "Allow local high brightness mode (the fingerprint circle); default off");

/*
 * The circle's brightness, as Google's GPL panel-gs-tg4c.c sets it after its
 * first-stage overdrive: on CMD2 page 2, the LHBM gamma set for the current
 * brightness, with the panel's own per-band LHBM values left as they are.
 */
static const u8 tg4c_cmd2_page2[] = { 0xF0, 0x55, 0xAA, 0x52, 0x08, 0x02 };

/*
 * The panel's ID as Google's gs_panel reads it: MIPI DCS ID1 (the module's
 * manufacturer), ID2 (its version), ID3 (the module/driver ID), and the first
 * byte of the device descriptor block. The bootloader appends the same four
 * bytes, last first, to exynos_drm.panel_name: google-tg4c.2d008004 on the
 * Pixel 9a's TG4C, so ID1 0x04 and ID3 0x00.
 */
static const u8 tg4c_id_regs[] = { 0xda, 0xdb, 0xdc, MIPI_DCS_READ_DDB_START };
#define TG4C_ID1		0x04
#define TG4C_ID3		0x00

enum tg4c_id {
	TG4C_ID_UNREAD,
	TG4C_ID_TG4C,
	TG4C_ID_OTHER,
};

struct tg4c {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	/* serialises commands from the panel and backlight paths */
	struct mutex lock;
	/* out of sleep: commands and frames reach the panel */
	bool awake;
	bool display_on;
	unsigned int hz;	/* the rate last set: LHBM needs 120 */
	bool lhbm;
	struct delayed_work lhbm_off;
	u16 brightness;
	enum tg4c_id id;
};

static inline struct tg4c *to_tg4c(struct drm_panel *panel)
{
	return container_of(panel, struct tg4c, panel);
}

/*
 * Lock held. Google's tegu device tree lists three panels for the Pixel 9a
 * (TG4A, TG4B and TG4C), whose commands differ: TG4B's rate switch, for
 * one, sends more than TG4C's. A panel whose manufacturer or module ID is
 * not the TG4C's is another panel. ID2 changes with the panel's build and
 * is not checked.
 *
 * The ID is read before the first command rather than at probe, because
 * gs-dsim-cmd refuses reads as well as writes (-EPERM) until its allow_tx
 * parameter is set, which can come after the panel has probed. Until then
 * nothing is sent anyway; once the read fails otherwise, the panel is
 * driven as before the ID was read.
 *
 * Return: false if the panel is another panel.
 */
static bool tg4c_check_id(struct tg4c *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	u8 id[ARRAY_SIZE(tg4c_id_regs)];
	unsigned int i;
	ssize_t ret;

	if (ctx->id != TG4C_ID_UNREAD)
		return ctx->id == TG4C_ID_TG4C;

	for (i = 0; i < ARRAY_SIZE(tg4c_id_regs); i++) {
		ret = mipi_dsi_dcs_read(ctx->dsi, tg4c_id_regs[i], &id[i], 1);
		if (ret == -EPERM)
			return true;
		if (ret != 1) {
			dev_warn(dev, "panel ID not read (%zd): taken for a TG4C\n", ret);
			ctx->id = TG4C_ID_TG4C;
			return true;
		}
	}

	if (id[0] != TG4C_ID1 || id[2] != TG4C_ID3) {
		dev_err(dev, "panel ID %*ph is not a TG4C's: sending it no commands\n",
			(int)sizeof(id), id);
		ctx->id = TG4C_ID_OTHER;
		return false;
	}

	dev_info(dev, "panel ID %*ph\n", (int)sizeof(id), id);
	ctx->id = TG4C_ID_TG4C;
	return true;
}

static void tg4c_write_rate(struct mipi_dsi_multi_context *ctx, unsigned int hz)
{
	if (hz == 120)
		mipi_dsi_dcs_write_seq_multi(ctx, 0x2f, 0x00);
	else
		mipi_dsi_dcs_write_seq_multi(ctx, 0x2f, 0x02);
}

/* lock held: Google's tg4c_set_local_hbm_gamma, on CMD2 page 2 */
static void tg4c_lhbm_prepare(struct tg4c *ctx, struct mipi_dsi_multi_context *dsi_ctx)
{
	u16 br = ctx->brightness;

	mipi_dsi_dcs_write_buffer_multi(dsi_ctx, tg4c_cmd2_page2, sizeof(tg4c_cmd2_page2));
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x6f, 0x07);
	if (br > TG4C_LHBM_GAMMASET1)
		mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x87, 0x01);
	else if (br > TG4C_LHBM_GAMMASET2)
		mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x87, 0x02);
	else if (br > TG4C_LHBM_GAMMASET3)
		mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x87, 0x03);
	else
		mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0x87, 0x04);
}

/* lock held */
static int tg4c_lhbm_set(struct tg4c *ctx, bool on)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	if (on == ctx->lhbm)
		return 0;
	if (on && (!ctx->awake || !ctx->display_on || ctx->hz != 120))
		return -EBUSY;
	if (on) {
		tg4c_lhbm_prepare(ctx, &dsi_ctx);
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x87, 0x05);
	} else {
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x87, 0x00);
	}
	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;
	ctx->lhbm = on;
	/* the timer is the current circle's: an earlier one must not end it */
	if (on)
		mod_delayed_work(system_wq, &ctx->lhbm_off,
				 msecs_to_jiffies(TG4C_LHBM_TIMEOUT_MS));
	else
		cancel_delayed_work(&ctx->lhbm_off);
	return 0;
}

static void tg4c_lhbm_timeout(struct work_struct *work)
{
	struct tg4c *ctx = container_of(to_delayed_work(work), struct tg4c, lhbm_off);

	guard(mutex)(&ctx->lock);
	if (ctx->awake)
		tg4c_lhbm_set(ctx, false);
	else
		ctx->lhbm = false;
}

static int tg4c_prepare(struct drm_panel *panel)
{
	struct tg4c *ctx = to_tg4c(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	if (ctx->awake)
		return 0;
	if (!tg4c_check_id(ctx))
		return -ENODEV;
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	/* the DSI host wakes the link at 60 Hz */
	tg4c_write_rate(&dsi_ctx, 60);
	if (!dsi_ctx.accum_err) {
		ctx->awake = true;
		ctx->hz = 60;
	}

	return dsi_ctx.accum_err;
}

static int tg4c_enable(struct drm_panel *panel)
{
	struct tg4c *ctx = to_tg4c(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	if (!tg4c_check_id(ctx))
		return -ENODEV;
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	if (!dsi_ctx.accum_err)
		ctx->display_on = true;

	return dsi_ctx.accum_err;
}

static int tg4c_disable(struct drm_panel *panel)
{
	struct tg4c *ctx = to_tg4c(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	if (!tg4c_check_id(ctx))
		return -ENODEV;
	tg4c_lhbm_set(ctx, false);
	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 100);
	if (!dsi_ctx.accum_err)
		ctx->display_on = false;

	return dsi_ctx.accum_err;
}

static int tg4c_unprepare(struct drm_panel *panel)
{
	struct tg4c *ctx = to_tg4c(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	guard(mutex)(&ctx->lock);
	if (!tg4c_check_id(ctx))
		return -ENODEV;
	ctx->lhbm = false;	/* sleep ends it */
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);
	if (!dsi_ctx.accum_err)
		ctx->awake = false;

	return dsi_ctx.accum_err;
}

static int tg4c_set_refresh_rate(struct drm_panel *panel, unsigned int hz)
{
	struct tg4c *ctx = to_tg4c(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	if (hz != 60 && hz != 120)
		return -EINVAL;

	guard(mutex)(&ctx->lock);
	if (!ctx->awake || !tg4c_check_id(ctx))
		return -ENODEV;
	/* LHBM fails away from 120 Hz: end it first (Google's tg4c does too) */
	if (hz != 120)
		tg4c_lhbm_set(ctx, false);
	tg4c_write_rate(&dsi_ctx, hz);
	if (!dsi_ctx.accum_err)
		ctx->hz = hz;

	return dsi_ctx.accum_err;
}

#define TG4C_MODE(hz, flags) {						\
	.clock = (1080 + 32 + 12 + 16) * (2424 + 12 + 4 + 15) * (hz) / 1000, \
	.hdisplay = 1080,						\
	.hsync_start = 1080 + 32,					\
	.hsync_end = 1080 + 32 + 12,					\
	.htotal = 1080 + 32 + 12 + 16,					\
	.vdisplay = 2424,						\
	.vsync_start = 2424 + 12,					\
	.vsync_end = 2424 + 12 + 4,					\
	.vtotal = 2424 + 12 + 4 + 15,					\
	.width_mm = 64,							\
	.height_mm = 145,						\
	.type = DRM_MODE_TYPE_DRIVER | (flags),				\
}

/* Google's tg4c modes: 60 Hz (preferred, the bootloader's) and 120 Hz */
static const struct drm_display_mode tg4c_modes[] = {
	TG4C_MODE(60, DRM_MODE_TYPE_PREFERRED),
	TG4C_MODE(120, 0),
};

static int tg4c_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tg4c_modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &tg4c_modes[i]);
		if (!mode)
			return i;
		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	return i;
}

static const struct drm_panel_funcs tg4c_panel_funcs = {
	.prepare = tg4c_prepare,
	.enable = tg4c_enable,
	.disable = tg4c_disable,
	.unprepare = tg4c_unprepare,
	.get_modes = tg4c_get_modes,
	.set_refresh_rate = tg4c_set_refresh_rate,
};

static int tg4c_bl_update_status(struct backlight_device *bl)
{
	struct tg4c *ctx = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);

	guard(mutex)(&ctx->lock);
	ctx->brightness = brightness;
	/* asleep: the level is kept and sent with the next update */
	if (!ctx->awake)
		return 0;
	if (!tg4c_check_id(ctx))
		return -ENODEV;

	return mipi_dsi_dcs_set_display_brightness_large(ctx->dsi, brightness);
}

static const struct backlight_ops tg4c_bl_ops = {
	.update_status = tg4c_bl_update_status,
};

static ssize_t local_hbm_mode_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct tg4c *ctx = bl_get_data(to_backlight_device(dev));

	return sysfs_emit(buf, "%d\n", ctx->lhbm);
}

static ssize_t local_hbm_mode_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct tg4c *ctx = bl_get_data(bl);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;
	if (on && (!READ_ONCE(allow_lhbm) || !backlight_get_brightness(bl)))
		return -EPERM;

	guard(mutex)(&ctx->lock);
	if (!tg4c_check_id(ctx))
		return -ENODEV;
	ret = tg4c_lhbm_set(ctx, on);
	return ret ?: count;
}
static DEVICE_ATTR_RW(local_hbm_mode);

/* the circle turns itself off after this long (ms), as Google's panels report */
static ssize_t local_hbm_max_timeout_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	return sysfs_emit(buf, "%d\n", TG4C_LHBM_TIMEOUT_MS);
}
static DEVICE_ATTR_RO(local_hbm_max_timeout);

static struct attribute *tg4c_bl_attrs[] = {
	&dev_attr_local_hbm_mode.attr,
	&dev_attr_local_hbm_max_timeout.attr,
	NULL
};
ATTRIBUTE_GROUPS(tg4c_bl);

static int tg4c_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = TG4C_DEFAULT_BRIGHTNESS,
		.max_brightness = TG4C_MAX_BRIGHTNESS,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};
	struct backlight_device *bl;
	struct tg4c *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct tg4c, panel, &tg4c_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_mutex_init(dev, &ctx->lock);
	if (ret)
		return ret;

	ctx->dsi = dsi;
	ctx->hz = 60;
	ctx->brightness = TG4C_DEFAULT_BRIGHTNESS;
	INIT_DELAYED_WORK(&ctx->lhbm_off, tg4c_lhbm_timeout);
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	/* the bootloader left it out of sleep with the display on */
	ctx->awake = true;
	ctx->display_on = true;
	ctx->panel.prepared = true;
	ctx->panel.enabled = true;
	ctx->panel.prepare_prev_first = true;

	/* the ID now, before any command, if the host allows reads yet */
	scoped_guard(mutex, &ctx->lock)
		tg4c_check_id(ctx);

	bl = devm_backlight_device_register(dev, dev_name(dev), dev, ctx,
					    &tg4c_bl_ops, &props);
	if (IS_ERR(bl))
		return dev_err_probe(dev, PTR_ERR(bl), "failed to register backlight\n");
	ctx->panel.backlight = bl;
	ret = sysfs_create_groups(&bl->dev.kobj, tg4c_bl_groups);
	if (ret)
		return dev_err_probe(dev, ret, "LHBM attribute\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		sysfs_remove_groups(&bl->dev.kobj, tg4c_bl_groups);
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	return 0;
}

static void tg4c_remove(struct mipi_dsi_device *dsi)
{
	struct tg4c *ctx = mipi_dsi_get_drvdata(dsi);

	/* no new writes, then the circle off, then nothing can queue its timer */
	sysfs_remove_groups(&ctx->panel.backlight->dev.kobj, tg4c_bl_groups);
	scoped_guard(mutex, &ctx->lock)
		tg4c_lhbm_set(ctx, false);
	cancel_delayed_work_sync(&ctx->lhbm_off);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id tg4c_of_match[] = {
	{ .compatible = "google,tg4c" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tg4c_of_match);

static struct mipi_dsi_driver tg4c_driver = {
	.probe = tg4c_probe,
	.remove = tg4c_remove,
	.driver = {
		.name = "panel-google-tg4c",
		.of_match_table = tg4c_of_match,
	},
};
module_mipi_dsi_driver(tg4c_driver);

MODULE_DESCRIPTION("Google TG4C DSI command mode panel");
MODULE_LICENSE("GPL");
