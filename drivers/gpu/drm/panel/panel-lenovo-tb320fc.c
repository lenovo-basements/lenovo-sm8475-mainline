// SPDX-License-Identifier: GPL-2.0-only
/* Experimental TB320FC normal-panel C-PHY/DSC support.
 * Board sequence comes from its captured running ZUI 17.0.339 DT.
 * Power is retained across blanking until shared TDDI reset is coordinated.
 */
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <video/mipi_display.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include "panel-lenovo-tb320fc-init.h"

struct tb320fc_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	struct drm_dsc_config dsc;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[3];
	bool powered;
	struct mutex status_lock;
	bool status_attempted;
	int power_mode_result[2];
	u8 power_mode[2];
};

/* Downstream timings are per link; DRM describes the complete panel. */
static const struct drm_display_mode tb320fc_mode = {
	.clock = 691524,
	.hdisplay = 1600,
	.hsync_start = 1600 + 2 * 137,
	.hsync_end = 1600 + 2 * (137 + 20),
	.htotal = 2 * (800 + 137 + 20 + 54),
	.vdisplay = 2560,
	.vsync_start = 2560 + 30,
	.vsync_end = 2560 + 30 + 4,
	.vtotal = 2560 + 30 + 4 + 256,
	.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
};

static struct tb320fc_panel *to_tb320fc(struct drm_panel *panel)
{
	return container_of(panel, struct tb320fc_panel, panel);
}

/* Preserve the captured DCS-long packet type, including 2-byte commands.
 * With sync-dual-dsi, the MSM manager queues host0 and broadcasts on host1.
 */
static int tb320fc_write(struct tb320fc_panel *ctx, const u8 *data, size_t size)
{
	int i;
	ssize_t ret;

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		struct mipi_dsi_msg msg = {
			.channel = ctx->dsi[i]->channel,
			.type = MIPI_DSI_DCS_LONG_WRITE,
			.flags = MIPI_DSI_MSG_USE_LPM,
			.tx_buf = data,
			.tx_len = size,
		};

		ret = ctx->dsi[i]->host->ops->transfer(ctx->dsi[i]->host, &msg);
		if (ret < 0)
			return ret;
		/* MSM reports its padded DMA packet length on the sending host. */
		if (ret < size)
			return -EIO;
	}
	return 0;
}

static int tb320fc_prepare(struct drm_panel *panel)
{
	struct tb320fc_panel *ctx = to_tb320fc(panel);
	struct drm_dsc_picture_parameter_set pps;
	int ret, i;

	if (!ctx->powered) {
		/* VDDIO, positive bias, negative bias; outputs are already live.
		 * GPIO-controlled bias rails are pinned on in this prototype DT.
		 */
		for (i = 0; i < ARRAY_SIZE(ctx->supplies); i++) {
			ret = regulator_enable(ctx->supplies[i].consumer);
			if (ret) {
				while (i--)
					regulator_disable(ctx->supplies[i].consumer);
				return ret;
			}
			if (i == 0)
				msleep(20); /* stock VDDIO post-on delay */
		}
		msleep(10); /* stock bias settling delay */

		/* Active-low descriptor: physical 0,1,0,1 for 10,10,10,11 ms. */
		ret = gpiod_direction_output(ctx->reset, 1);
		if (ret) {
			regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
			return ret;
		}
		msleep(10);
		gpiod_set_value_cansleep(ctx->reset, 0);
		msleep(10);
		gpiod_set_value_cansleep(ctx->reset, 1);
		msleep(10);
		gpiod_set_value_cansleep(ctx->reset, 0);
		msleep(11);
		ctx->powered = true;
		dev_info(panel->dev, "TB320FC initial reset complete; keeping TDDI power on\n");
	}

	for (i = 0; i < ARRAY_SIZE(tb320fc_panel_init); i++) {
		const struct tb320fc_panel_cmd *cmd = &tb320fc_panel_init[i];

		ret = tb320fc_write(ctx, tb320fc_panel_payload + cmd->offset, cmd->size);
		if (ret) {
			dev_err(panel->dev, "TB320FC init command %d failed: %d\n", i, ret);
			return ret;
		}
		if (cmd->delay_ms)
			msleep(cmd->delay_ms);
	}

	/* Downstream mode_set uses the full picture width (default divisor 1),
	 * while each DSI link carries one 800-pixel slice per line.
	 */
	if (ctx->dsc.pic_width != 1600 || ctx->dsc.pic_height != 2560 ||
	    ctx->dsc.slice_chunk_size != 800)
		return -EINVAL;

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		ret = mipi_dsi_picture_parameter_set(ctx->dsi[i], &pps);
		if (ret < 0)
			return ret;
	}
	dev_info(panel->dev, "TB320FC 1600x2560@120 init and DSC PPS sent\n");
	return 0;
}

static int tb320fc_disable(struct drm_panel *panel)
{
	struct tb320fc_panel *ctx = to_tb320fc(panel);
	int i, ret, err = 0;

	/* Captured off commands use HS, with 5 ms then 70 ms delays. */
	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		ctx->dsi[i]->mode_flags &= ~MIPI_DSI_MODE_LPM;
		ret = mipi_dsi_dcs_set_display_off(ctx->dsi[i]);
		if (ret < 0 && !err)
			err = ret;
	}
	msleep(5);
	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		ret = mipi_dsi_dcs_enter_sleep_mode(ctx->dsi[i]);
		if (ret < 0 && !err)
			err = ret;
		ctx->dsi[i]->mode_flags |= MIPI_DSI_MODE_LPM;
	}
	msleep(70);
	return err;
}

static int tb320fc_unprepare(struct drm_panel *panel)
{
	/* Do not assert reset or disable supplies: the touch MCU shares TDDI
	 * power, and its SRAM firmware was loaded after the initial modeset.
	 * Full power-off/resume is deliberately deferred in this prototype.
	 */
	return 0;
}

static int tb320fc_enable(struct drm_panel *panel)
{
	struct tb320fc_panel *ctx = to_tb320fc(panel);
	int i;

	/* One captured-stock ESD status command (DCS 0x0a) per link, before
	 * the SD-root touch service can reset/upload the MCU. Read failures
	 * are diagnostic evidence, not a reason to retry init or power-cycle.
	 */
	mutex_lock(&ctx->status_lock);
	if (!ctx->status_attempted) {
		ctx->status_attempted = true;
		for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
			ctx->power_mode_result[i] =
				mipi_dsi_dcs_get_power_mode(ctx->dsi[i], &ctx->power_mode[i]);
			dev_info(panel->dev, "TB320FC power status DSI%d: ret=%d value=%#02x (stock 0x9c)\n",
				 i, ctx->power_mode_result[i], ctx->power_mode[i]);
		}
	}
	mutex_unlock(&ctx->status_lock);
	return 0;
}

static ssize_t tb320fc_power_mode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct tb320fc_panel *ctx = mipi_dsi_get_drvdata(to_mipi_dsi_device(dev));
	ssize_t count;

	/* Cached boot evidence only: reading sysfs sends no further commands. */
	mutex_lock(&ctx->status_lock);
	count = sysfs_emit(buf, "queried=%u dsi0_ret=%d dsi0_value=%#02x dsi1_ret=%d dsi1_value=%#02x expected=0x9c\n",
			 ctx->status_attempted, ctx->power_mode_result[0], ctx->power_mode[0],
			 ctx->power_mode_result[1], ctx->power_mode[1]);
	mutex_unlock(&ctx->status_lock);
	return count;
}
static DEVICE_ATTR_RO(tb320fc_power_mode);

static struct attribute *tb320fc_attrs[] = {
	&dev_attr_tb320fc_power_mode.attr,
	NULL,
};
ATTRIBUTE_GROUPS(tb320fc);

static int tb320fc_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode = drm_mode_duplicate(connector->dev, &tb320fc_mode);

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

static const struct drm_panel_funcs tb320fc_funcs = {
	.prepare = tb320fc_prepare,
	.enable = tb320fc_enable,
	.disable = tb320fc_disable,
	.unprepare = tb320fc_unprepare,
	.get_modes = tb320fc_get_modes,
};

static int tb320fc_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct tb320fc_panel *ctx;
	struct device_node *secondary;
	struct mipi_dsi_host *host;
	const struct mipi_dsi_device_info info = { .type = "tb320fc-secondary", .channel = 0 };
	int i, ret;

	ctx = devm_drm_panel_alloc(dev, struct tb320fc_panel, panel,
				 &tb320fc_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);
	mutex_init(&ctx->status_lock);
	ctx->power_mode_result[0] = -ENODATA;
	ctx->power_mode_result[1] = -ENODATA;

	ctx->supplies[0].supply = "vddio";
	ctx->supplies[1].supply = "vddpos";
	ctx->supplies[2].supply = "vddneg";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get panel supplies\n");

	/* Do not reset the firmware display while resolving dependencies. */
	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset), "failed to get reset\n");

	secondary = of_graph_get_remote_node(dev->of_node, 1, -1);
	if (!secondary)
		return -ENODEV;
	host = of_find_mipi_dsi_host_by_node(secondary);
	of_node_put(secondary);
	if (!host)
		return dev_err_probe(dev, -EPROBE_DEFER, "secondary DSI host not ready\n");

	ctx->dsi[0] = dsi;
	ctx->dsi[1] = devm_mipi_dsi_device_register_full(dev, host, &info);
	if (IS_ERR(ctx->dsi[1]))
		return dev_err_probe(dev, PTR_ERR(ctx->dsi[1]), "secondary DSI device failed\n");

	ctx->dsc = (struct drm_dsc_config) {
		.dsc_version_major = 1, .dsc_version_minor = 1,
		.pic_width = 1600, .pic_height = 2560,
		.slice_width = 800, .slice_height = 20, .slice_count = 2,
		.bits_per_component = 8, .bits_per_pixel = 8 << 4,
		.block_pred_enable = true,
	};

	ctx->panel.prepare_prev_first = true;
	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "backlight not ready\n");

	mipi_dsi_set_drvdata(dsi, ctx);
	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		ctx->dsi[i]->lanes = 3;
		ctx->dsi[i]->format = MIPI_DSI_FMT_RGB888;
		ctx->dsi[i]->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
					 MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;
		ctx->dsi[i]->dsc = &ctx->dsc;
	}

	/* Attaching the final host can synchronously bind MSM's component
	 * master. Its bridge lookup must already find this panel. Use devres
	 * so a deferred/failed attach removes the registry entry again.
	 */
	ret = devm_drm_panel_add(dev, &ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "panel registration failed\n");

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		ret = devm_mipi_dsi_attach(dev, ctx->dsi[i]);
		if (ret)
			return dev_err_probe(dev, ret, "DSI%d attach failed\n", i);
	}
	return 0;
}

static void tb320fc_remove(struct mipi_dsi_device *dsi)
{
	struct tb320fc_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (ctx->powered)
		regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
}

static const struct of_device_id tb320fc_of_match[] = {
	{ .compatible = "lenovo,tb320fc-nt36523n" },
	{ }
};
MODULE_DEVICE_TABLE(of, tb320fc_of_match);

static struct mipi_dsi_driver tb320fc_driver = {
	.probe = tb320fc_probe,
	.remove = tb320fc_remove,
	.driver = {
		.name = "panel-lenovo-tb320fc",
		.of_match_table = tb320fc_of_match,
		.dev_groups = tb320fc_groups,
	},
};
module_mipi_dsi_driver(tb320fc_driver);
MODULE_DESCRIPTION("Experimental Lenovo TB320FC dual C-PHY DSC panel");
MODULE_LICENSE("GPL");
