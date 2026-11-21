// SPDX-License-Identifier: GPL-2.0-only
// Author: Eralp Çelebi <eralp.celebi.personal@gmail.com>
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
// Copyright (c) 2022, The Linux Foundation. All rights reserved.

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct ss_notch_fhd_ea8074 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
};

static const struct regulator_bulk_data ss_notch_fhd_ea8074_supplies[] = {
	{ .supply = "vci" },
	{ .supply = "vddio" },
};

static inline
struct ss_notch_fhd_ea8074 *to_ss_notch_fhd_ea8074(struct drm_panel *panel)
{
	return container_of(panel, struct ss_notch_fhd_ea8074, panel);
}

static void ss_notch_fhd_ea8074_reset(struct ss_notch_fhd_ea8074 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int ss_notch_fhd_ea8074_on(struct ss_notch_fhd_ea8074 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 10000, 11000);

	mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0x0000, 0x08c7);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xef, 0xf0, 0x31, 0x00, 0x33, 0x31, 0x14, 0x35);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbb, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x4f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x6b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf7, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb1, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd5, 0x02, 0x17, 0x54, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd2, 0x9f, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd2, 0x70);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd2, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);

	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx,
				       MIPI_DSI_DCS_TEAR_MODE_VBLANK);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_POWER_SAVE, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 110);

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int ss_notch_fhd_ea8074_off(struct ss_notch_fhd_ea8074 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 17000, 18000);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x49);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x4f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xcb, 0xc8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf7, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_usleep_range(&dsi_ctx, 17000, 18000);

	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int ss_notch_fhd_ea8074_prepare(struct drm_panel *panel)
{
	struct ss_notch_fhd_ea8074 *ctx = to_ss_notch_fhd_ea8074(panel);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ss_notch_fhd_ea8074_supplies),
				    ctx->supplies);
	if (ret < 0)
		return ret;

	/*
	 * The panel bridge pre-enables the DSI host first, so initialize the
	 * panel in .enable after reset.
	 */
	ss_notch_fhd_ea8074_reset(ctx);
	return 0;
}

static int ss_notch_fhd_ea8074_enable(struct drm_panel *panel)
{
	struct ss_notch_fhd_ea8074 *ctx = to_ss_notch_fhd_ea8074(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = ss_notch_fhd_ea8074_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ss_notch_fhd_ea8074_disable(struct drm_panel *panel)
{
	struct ss_notch_fhd_ea8074 *ctx = to_ss_notch_fhd_ea8074(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	/*
	 * Send DCS off while the DSI host is powered; disable runs before the
	 * DSI bridge post-disable.
	 */
	ret = ss_notch_fhd_ea8074_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	return 0;
}

static int ss_notch_fhd_ea8074_unprepare(struct drm_panel *panel)
{
	struct ss_notch_fhd_ea8074 *ctx = to_ss_notch_fhd_ea8074(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	return regulator_bulk_disable(ARRAY_SIZE(ss_notch_fhd_ea8074_supplies),
				      ctx->supplies);
}

static const struct drm_display_mode ss_notch_fhd_ea8074_mode = {
	.clock = (1080 + 56 + 18 + 56) * (2248 + 26 + 12 + 24) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 56,
	.hsync_end = 1080 + 56 + 18,
	.htotal = 1080 + 56 + 18 + 56,
	.vdisplay = 2248,
	.vsync_start = 2248 + 26,
	.vsync_end = 2248 + 26 + 12,
	.vtotal = 2248 + 26 + 12 + 24,
	.width_mm = 68,
	.height_mm = 142,
};

static int ss_notch_fhd_ea8074_get_modes(struct drm_panel *panel,
					 struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &ss_notch_fhd_ea8074_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static int ss_notch_fhd_ea8074_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	int brightness, ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	brightness = backlight_get_brightness(bl);
	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret;
}

static const struct backlight_ops ss_notch_fhd_ea8074_bl_ops = {
	.update_status = ss_notch_fhd_ea8074_bl_update_status,
};

static struct backlight_device *
ss_notch_fhd_ea8074_create_backlight(struct mipi_dsi_device *dsi)
{
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 511,
		.max_brightness = 1023,
	};

	return devm_backlight_device_register(&dsi->dev, dev_name(&dsi->dev),
					      &dsi->dev, dsi,
					      &ss_notch_fhd_ea8074_bl_ops,
					      &props);
}

static const struct drm_panel_funcs ss_notch_fhd_ea8074_panel_funcs = {
	.prepare = ss_notch_fhd_ea8074_prepare,
	.enable = ss_notch_fhd_ea8074_enable,
	.disable = ss_notch_fhd_ea8074_disable,
	.unprepare = ss_notch_fhd_ea8074_unprepare,
	.get_modes = ss_notch_fhd_ea8074_get_modes,
};

static int ss_notch_fhd_ea8074_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ss_notch_fhd_ea8074 *ctx;
	int ret;
	struct backlight_device *backlight;

	ctx = devm_drm_panel_alloc(dev, struct ss_notch_fhd_ea8074, panel,
				   &ss_notch_fhd_ea8074_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(dev,
					    ARRAY_SIZE(ss_notch_fhd_ea8074_supplies),
					    ss_notch_fhd_ea8074_supplies,
					    &ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/*
	 * EA8074 is a command-mode panel; video-mode flags make DCS init fail.
	 */
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;
	ctx->panel.prepare_prev_first = true;
	backlight = ss_notch_fhd_ea8074_create_backlight(dsi);
	if (IS_ERR(backlight))
		return dev_err_probe(dev, PTR_ERR(backlight),
				     "Failed to create backlight\n");
	ctx->panel.backlight = backlight;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void ss_notch_fhd_ea8074_remove(struct mipi_dsi_device *dsi)
{
	struct ss_notch_fhd_ea8074 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ss_notch_fhd_ea8074_of_match[] = {
	{ .compatible = "samsung,ea8074" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ss_notch_fhd_ea8074_of_match);

static struct mipi_dsi_driver ss_notch_fhd_ea8074_driver = {
	.probe = ss_notch_fhd_ea8074_probe,
	.remove = ss_notch_fhd_ea8074_remove,
	.driver = {
		.name = "panel-ss-notch-fhd-ea8074",
		.of_match_table = ss_notch_fhd_ea8074_of_match,
	},
};
module_mipi_dsi_driver(ss_notch_fhd_ea8074_driver);

MODULE_AUTHOR("Eralp Çelebi <eralp.celebi.personal@gmail.com>");
MODULE_DESCRIPTION("DRM driver for the Samsung EA8074 Panel");
MODULE_LICENSE("GPL");
