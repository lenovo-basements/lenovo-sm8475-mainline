// SPDX-License-Identifier: GPL-2.0-only
/*
 * Lenovo Legion Y700 (2023) NT36523N dual C-PHY panel
 *
 * DRM and the host DSC configuration describe the complete 1600-pixel
 * picture. Each physical C-PHY link carries one 800-pixel DSC slice.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

#include "panel-lenovo-asphalt-init.h"

static unsigned int refresh_rate = 144;
module_param(refresh_rate, uint, 0444);
MODULE_PARM_DESC(refresh_rate, "Fixed panel refresh rate: 144 (default) or 120 Hz");

enum asphalt_supply {
	ASPHALT_VDDIO,
	ASPHALT_VDDPOS,
	ASPHALT_VDDNEG,
	ASPHALT_NUM_SUPPLIES,
};

struct asphalt_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	struct drm_dsc_config dsc;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[ASPHALT_NUM_SUPPLIES];
	enum drm_panel_orientation orientation;
};

static const struct drm_display_mode asphalt_modes[] = {
	{
		.clock = 691524,
		.hdisplay = 1600,
		.hsync_start = 1600 + 2 * 137,
		.hsync_end = 1600 + 2 * (137 + 20),
		.htotal = 2 * (800 + 137 + 20 + 54),
		.vdisplay = 2560,
		.vsync_start = 2560 + 30,
		.vsync_end = 2560 + 30 + 4,
		.vtotal = 2560 + 30 + 4 + 256,
	}, {
		/*
		 * The stock 144 Hz mode changes HFP, keeping the C-PHY clock.
		 * DSC transports 267 pixel clocks per link. HFP137 at 120 Hz
		 * becomes 137 - floor(478 * 24 / 144) = 58; the vendor then
		 * requires an odd 144 Hz HFP, hence 57 and a link total of 398.
		 * This uncompressed DRM clock becomes 163476041 Hz per link
		 * after MSM's DSC/bonded adjustment, matching the fixed clock.
		 */
		.clock = 764805,
		.hdisplay = 1600,
		.hsync_start = 1600 + 2 * 57,
		.hsync_end = 1600 + 2 * (57 + 20),
		.htotal = 2 * (800 + 57 + 20 + 54),
		.vdisplay = 2560,
		.vsync_start = 2560 + 30,
		.vsync_end = 2560 + 30 + 4,
		.vtotal = 2560 + 30 + 4 + 256,
	},
};

static struct asphalt_panel *to_asphalt_panel(struct drm_panel *panel)
{
	return container_of(panel, struct asphalt_panel, panel);
}

/*
 * Preserve the vendor packet type: even two-byte register writes are DCS
 * long packets. Send each command to both links before its settling delay.
 */
static int asphalt_write(struct asphalt_panel *ctx, u8 type,
			 const u8 *data, size_t size, bool lpm)
{
	unsigned int i;
	ssize_t ret;

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		struct mipi_dsi_device *dsi = ctx->dsi[i];
		struct mipi_dsi_msg msg = {
			.channel = dsi->channel,
			.type = type,
			.flags = lpm ? MIPI_DSI_MSG_USE_LPM : 0,
			.tx_buf = data,
			.tx_len = size,
		};

		if (!dsi->host->ops || !dsi->host->ops->transfer)
			return -EOPNOTSUPP;

		ret = dsi->host->ops->transfer(dsi->host, &msg);
		if (ret < 0)
			return ret;
		/* Some hosts return the padded DMA packet length. */
		if ((size_t)ret < size)
			return -EIO;
	}

	return 0;
}

static int asphalt_init_sequence(struct asphalt_panel *ctx)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(asphalt_panel_init); i++) {
		const struct asphalt_panel_cmd *cmd = &asphalt_panel_init[i];

		ret = asphalt_write(ctx, MIPI_DSI_DCS_LONG_WRITE,
				    asphalt_panel_payload + cmd->offset,
				    cmd->size, true);
		if (ret) {
			dev_err(ctx->panel.dev, "init command %u failed: %d\n", i, ret);
			return ret;
		}

		if (cmd->delay_ms)
			msleep(cmd->delay_ms);
	}

	return 0;
}

static int asphalt_power_on_144(struct asphalt_panel *ctx)
{
	static const u8 commands[][2] = {
		{ 0xff, 0x10 }, { 0xb2, 0x80 }, { 0xb3, 0x12 },
	};
	unsigned int i;
	int ret;

	/* The vendor 0x40 header byte batches commands; its wait byte is zero. */
	for (i = 0; i < ARRAY_SIZE(commands); i++) {
		ret = asphalt_write(ctx, MIPI_DSI_DCS_LONG_WRITE,
				    commands[i], sizeof(commands[i]), true);
		if (ret)
			return ret;
	}

	return 0;
}

static int asphalt_enable_touch_144(struct asphalt_panel *ctx)
{
	static const u8 commands[][2] = {
		{ 0xff, 0x27 }, { 0x78, 0x80 }, { 0xff, 0x10 },
	};
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(commands); i++) {
		ret = asphalt_write(ctx, MIPI_DSI_DCS_LONG_WRITE,
				    commands[i], sizeof(commands[i]), true);
		if (ret)
			return ret;
	}

	return 0;
}

static int asphalt_send_pps(struct asphalt_panel *ctx)
{
	struct drm_dsc_config sink_dsc = ctx->dsc;
	struct drm_dsc_picture_parameter_set pps;

	/*
	 * Each physical sink receives one slice. Keep the full-picture host
	 * config intact: DPU uses its two slices to allocate both DSC channels.
	 */
	sink_dsc.pic_width = sink_dsc.slice_width;
	sink_dsc.slice_count = 1;
	drm_dsc_pps_payload_pack(&pps, &sink_dsc);

	return asphalt_write(ctx, MIPI_DSI_PICTURE_PARAMETER_SET,
			     (const u8 *)&pps, sizeof(pps), true);
}

static void asphalt_reset(struct asphalt_panel *ctx)
{
	/* Active-low reset: physical low, high, low, high. */
	gpiod_set_value_cansleep(ctx->reset, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset, 0);
	usleep_range(11000, 12000);
}

static void asphalt_power_off(struct asphalt_panel *ctx)
{
	gpiod_set_value_cansleep(ctx->reset, 1);
	usleep_range(3000, 4000);
	regulator_disable(ctx->supplies[ASPHALT_VDDNEG].consumer);
	regulator_disable(ctx->supplies[ASPHALT_VDDPOS].consumer);
	regulator_disable(ctx->supplies[ASPHALT_VDDIO].consumer);
}

static int asphalt_prepare(struct drm_panel *panel)
{
	struct asphalt_panel *ctx = to_asphalt_panel(panel);
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ctx->supplies); i++) {
		ret = regulator_enable(ctx->supplies[i].consumer);
		if (ret) {
			while (i--)
				regulator_disable(ctx->supplies[i].consumer);
			return ret;
		}
		if (i == ASPHALT_VDDIO)
			msleep(20);
	}
	usleep_range(10000, 11000);

	asphalt_reset(ctx);

	ret = asphalt_init_sequence(ctx);
	if (ret)
		goto power_off;

	if (refresh_rate == 144) {
		ret = asphalt_power_on_144(ctx);
		if (ret) {
			dev_err(panel->dev, "144 Hz power-on commands failed: %d\n", ret);
			goto power_off;
		}
	}

	/* Vendor register 0x90 already enables DSC. PPS follows the on sequence. */
	ret = asphalt_send_pps(ctx);
	if (ret) {
		dev_err(panel->dev, "PPS transfer failed: %d\n", ret);
		goto power_off;
	}

	return 0;

power_off:
	asphalt_power_off(ctx);
	return ret;
}

static int asphalt_enable(struct drm_panel *panel)
{
	struct asphalt_panel *ctx = to_asphalt_panel(panel);
	int ret;

	if (refresh_rate != 144)
		return 0;

	/* Allow a frame interval before releasing the panel touch gate. */
	usleep_range(7000, 8000);
	ret = asphalt_enable_touch_144(ctx);
	if (ret)
		return dev_err_probe(panel->dev, ret, "144 Hz touch-enable commands failed\n");
	/* Followers start their firmware only after this callback completes. */
	usleep_range(7000, 8000);

	return 0;
}

static int asphalt_off_sequence(struct asphalt_panel *ctx)
{
	static const u8 display_off[] = { MIPI_DCS_SET_DISPLAY_OFF, 0 };
	static const u8 enter_sleep[] = { MIPI_DCS_ENTER_SLEEP_MODE, 0 };
	int ret, err;

	/* The captured off packets are DCS short writes sent in HS mode. */
	err = asphalt_write(ctx, MIPI_DSI_DCS_SHORT_WRITE,
			    display_off, sizeof(display_off), false);
	usleep_range(5000, 6000);
	ret = asphalt_write(ctx, MIPI_DSI_DCS_SHORT_WRITE,
			    enter_sleep, sizeof(enter_sleep), false);
	msleep(70);

	return err ?: ret;
}

static int asphalt_unprepare(struct drm_panel *panel)
{
	struct asphalt_panel *ctx = to_asphalt_panel(panel);
	int ret;

	ret = asphalt_off_sequence(ctx);
	if (ret)
		dev_err(panel->dev, "off sequence failed: %d\n", ret);

	/* Power-off completes even if a powered-down panel no longer responds. */
	asphalt_power_off(ctx);
	return 0;
}

static int asphalt_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &asphalt_modes[refresh_rate == 144]);
	if (!mode)
		return -ENOMEM;

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);
	connector->display_info.width_mm = 118;
	connector->display_info.height_mm = 190;
	connector->display_info.bpc = 8;

	return 1;
}

static enum drm_panel_orientation asphalt_get_orientation(struct drm_panel *panel)
{
	return to_asphalt_panel(panel)->orientation;
}

static const struct drm_panel_funcs asphalt_panel_funcs = {
	.prepare = asphalt_prepare,
	.enable = asphalt_enable,
	.unprepare = asphalt_unprepare,
	.get_modes = asphalt_get_modes,
	.get_orientation = asphalt_get_orientation,
};

static int asphalt_probe(struct mipi_dsi_device *dsi)
{
	static const struct mipi_dsi_device_info secondary_info = {
		.type = "asphalt-secondary",
		.channel = 0,
	};
	struct device *dev = &dsi->dev;
	struct device_node *secondary;
	struct mipi_dsi_host *host;
	struct asphalt_panel *ctx;
	unsigned int i;
	int ret;

	if (refresh_rate != 120 && refresh_rate != 144)
		return dev_err_probe(dev, -EINVAL, "refresh_rate must be 120 or 144\n");

	ctx = devm_drm_panel_alloc(dev, struct asphalt_panel, panel,
				   &asphalt_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->supplies[ASPHALT_VDDIO].supply = "vddio";
	ctx->supplies[ASPHALT_VDDIO].init_load_uA = 200000;
	ctx->supplies[ASPHALT_VDDPOS].supply = "vddpos";
	ctx->supplies[ASPHALT_VDDNEG].supply = "vddneg";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get panel supplies\n");

	secondary = of_graph_get_remote_node(dev->of_node, 1, -1);
	if (!secondary)
		return dev_err_probe(dev, -ENODEV, "missing secondary DSI endpoint\n");
	host = of_find_mipi_dsi_host_by_node(secondary);
	of_node_put(secondary);
	if (!host)
		return dev_err_probe(dev, -EPROBE_DEFER, "secondary DSI host not ready\n");

	ctx->dsi[0] = dsi;
	ctx->dsi[1] = devm_mipi_dsi_device_register_full(dev, host, &secondary_info);
	if (IS_ERR(ctx->dsi[1]))
		return dev_err_probe(dev, PTR_ERR(ctx->dsi[1]),
				     "failed to register secondary DSI device\n");

	ret = drm_of_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get panel orientation\n");

	ctx->panel.prepare_prev_first = true;
	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get backlight\n");

	/* Acquire reset after dependencies so deferral preserves the boot display. */
	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset), "failed to get panel reset\n");

	ctx->dsc = (struct drm_dsc_config) {
		.dsc_version_major = 1,
		.dsc_version_minor = 1,
		.pic_width = 1600,
		.pic_height = 2560,
		.slice_width = 800,
		.slice_height = 20,
		.slice_count = 2,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
		.convert_rgb = true,
		.line_buf_depth = 9,
	};
	drm_dsc_set_const_params(&ctx->dsc);
	drm_dsc_set_rc_buf_thresh(&ctx->dsc);
	ret = drm_dsc_setup_rc_params(&ctx->dsc, DRM_DSC_1_1_PRE_SCR);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DSC rate control\n");
	ctx->dsc.initial_scale_value = drm_dsc_initial_scale_value(&ctx->dsc);
	ret = drm_dsc_compute_rc_parameters(&ctx->dsc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to compute DSC parameters\n");

	mipi_dsi_set_drvdata(dsi, ctx);
	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		ctx->dsi[i]->lanes = 3;
		ctx->dsi[i]->format = MIPI_DSI_FMT_RGB888;
		ctx->dsi[i]->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
					  MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;
		ctx->dsi[i]->dsc = &ctx->dsc;
	}

	/* The final attach may bind the DRM master, which must already find us. */
	ret = devm_drm_panel_add(dev, &ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register panel\n");
	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		ret = devm_mipi_dsi_attach(dev, ctx->dsi[i]);
		if (ret)
			return dev_err_probe(dev, ret, "DSI%u attach failed\n", i);
	}

	return 0;
}

static void asphalt_shutdown(struct mipi_dsi_device *dsi)
{
	struct asphalt_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (ctx->panel.enabled)
		drm_panel_disable(&ctx->panel);
	if (ctx->panel.prepared)
		drm_panel_unprepare(&ctx->panel);
}

static const struct of_device_id asphalt_of_match[] = {
	{ .compatible = "lenovo,tb320fc-nt36523n" },
	{ }
};
MODULE_DEVICE_TABLE(of, asphalt_of_match);

static struct mipi_dsi_driver asphalt_driver = {
	.probe = asphalt_probe,
	.remove = asphalt_shutdown,
	.shutdown = asphalt_shutdown,
	.driver = {
		.name = "panel-lenovo-asphalt",
		.of_match_table = asphalt_of_match,
	},
};
module_mipi_dsi_driver(asphalt_driver);

MODULE_DESCRIPTION("Lenovo asphalt NT36523N dual C-PHY DSC panel");
MODULE_LICENSE("GPL");
