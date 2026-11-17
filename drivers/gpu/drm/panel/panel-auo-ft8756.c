// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Omarchy Mobile contributors
 *
 * AUO FT8756 1080x2300 DSI video-mode panel, as fitted to the Motorola
 * Moto G Power (2020) (sofia).
 *
 * The timing and the command sequences come from Motorola's own device tree
 * (arch/arm64/boot/dts/qcom/dsi-panel-mot-auo-ft8756-fhd-video.dtsi in
 * MotorolaMobilityLLC/kernel-msm, tag MMI-QPE30.79-41), which is the same
 * panel the stock bootloader reports as "auo_ft8756_fhd_vid".
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct auo_ft8756 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
};

static inline
struct auo_ft8756 *to_auo_ft8756(struct drm_panel *panel)
{
	return container_of(panel, struct auo_ft8756, panel);
}

static void auo_ft8756_reset(struct auo_ft8756 *ctx)
{
	/*
	 * The vendor tree's reset sequence, <1 4>, <0 1>, <1 11>: line
	 * levels with the delays in milliseconds. The line is active low, so
	 * the levels are release, assert, release.
	 */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(4000, 4000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 1000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(11);
}

/*
 * Motorola's qcom,mdss-dsi-on-command and qcom,mdss-dsi-off-command lists.
 * Each entry is a header of type, bits, channel, ack, wait, flags and
 * length, followed by that many payload bytes, and the whole entry is one
 * write to the panel. The types are the DSI data types: 0x15 is a DCS
 * short write and 0x39 a DCS long write.
 */
static const u8 auo_ft8756_on_cmd[] = {
	39, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
	39, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04, 0xff, 0x87, 0x56, 0x01,
	39, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x80,
	39, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0x87, 0x56,
	15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x51, 0xa3,
	15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x53, 0x24,
	15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x55, 0x01,
	15, 0x01, 0x00, 0x00, 0x60, 0x00, 0x02, 0x11, 0x00,
	15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x29, 0x00,
};

static const u8 auo_ft8756_off_cmd[] = {
	15, 0x01, 0x00, 0x00, 0x32, 0x00, 0x02, 0x28, 0x00,
	15, 0x01, 0x00, 0x00, 0x50, 0x00, 0x02, 0x10, 0x00,
	39, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
	39, 0x01, 0x00, 0x00, 0x00, 0x00, 0x05, 0xf7, 0x5a, 0xa5, 0x95, 0x27,
};

/* The header of one entry: type, bits, channel, ack, wait, flags, length. */
#define AUO_FT8756_CMD_HDR_LEN	7

static int auo_ft8756_send_cmds(struct auo_ft8756 *ctx, const u8 *cmds,
				size_t len)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	size_t off = 0;

	while (off + AUO_FT8756_CMD_HDR_LEN <= len) {
		const u8 *hdr = &cmds[off];
		u8 type = hdr[0];
		u8 wait = hdr[4];
		u8 plen = hdr[6];
		const u8 *payload = &cmds[off + AUO_FT8756_CMD_HDR_LEN];

		if (off + AUO_FT8756_CMD_HDR_LEN + plen > len)
			return -EINVAL;

		switch (type) {
		case 0x15:
			/* DCS short write: the command and one parameter */
			mipi_dsi_dcs_write(ctx->dsi, payload[0],
					   payload + 1, plen - 1);
			break;
		case 0x39:
			/* DCS long write: the command and its parameters */
			mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, payload,
						       plen);
			break;
		default:
			mipi_dsi_generic_write(ctx->dsi, payload, plen);
			break;
		}

		if (dsi_ctx.accum_err < 0)
			return dsi_ctx.accum_err;

		if (wait)
			mipi_dsi_msleep(&dsi_ctx, wait);

		off += AUO_FT8756_CMD_HDR_LEN + plen;
	}

	return 0;
}

static int auo_ft8756_enable(struct auo_ft8756 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	int ret;

	ret = auo_ft8756_send_cmds(ctx, auo_ft8756_on_cmd,
				   sizeof(auo_ft8756_on_cmd));
	if (ret < 0)
		return ret;

	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int auo_ft8756_disable(struct auo_ft8756 *ctx)
{
	return auo_ft8756_send_cmds(ctx, auo_ft8756_off_cmd,
				   sizeof(auo_ft8756_off_cmd));
}

static int auo_ft8756_prepare(struct drm_panel *panel)
{
	struct auo_ft8756 *ctx = to_auo_ft8756(panel);
	int ret;

	auo_ft8756_reset(ctx);

	ret = auo_ft8756_enable(ctx);
	if (ret < 0) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	return 0;
}

static int auo_ft8756_unprepare(struct drm_panel *panel)
{
	struct auo_ft8756 *ctx = to_auo_ft8756(panel);

	auo_ft8756_disable(ctx);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

/*
 * The timings from the vendor tree: 1080x2300 at 60 Hz, with a horizontal
 * front porch of 16, back porch 43, pulse width 6, and a vertical front
 * porch of 8, back porch 32, pulse width 4.
 */
static const struct drm_display_mode auo_ft8756_mode = {
	.clock = (1080 + 16 + 43 + 6) * (2300 + 8 + 32 + 4) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 16,
	.hsync_end = 1080 + 16 + 6,
	.htotal = 1080 + 16 + 43 + 6,
	.vdisplay = 2300,
	.vsync_start = 2300 + 8,
	.vsync_end = 2300 + 8 + 4,
	.vtotal = 2300 + 8 + 32 + 4,
};

static int auo_ft8756_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &auo_ft8756_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs auo_ft8756_panel_funcs = {
	.prepare = auo_ft8756_prepare,
	.unprepare = auo_ft8756_unprepare,
	.get_modes = auo_ft8756_get_modes,
};

static const struct of_device_id auo_ft8756_of_match[] = {
	{ .compatible = "auo,ft8756" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, auo_ft8756_of_match);

static int auo_ft8756_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct auo_ft8756 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct auo_ft8756, panel,
				   &auo_ft8756_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void auo_ft8756_remove(struct mipi_dsi_device *dsi)
{
	struct auo_ft8756 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static struct mipi_dsi_driver auo_ft8756_driver = {
	.probe = auo_ft8756_probe,
	.remove = auo_ft8756_remove,
	.driver = {
		.name = "panel-auo-ft8756",
		.of_match_table = auo_ft8756_of_match,
	},
};
module_mipi_dsi_driver(auo_ft8756_driver);

MODULE_AUTHOR("Omarchy Mobile contributors");
MODULE_DESCRIPTION("AUO FT8756 1080x2300 DSI video mode panel");
MODULE_LICENSE("GPL");
