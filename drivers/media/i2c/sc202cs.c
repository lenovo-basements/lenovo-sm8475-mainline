// SPDX-License-Identifier: GPL-2.0
/*
 * SmartSens SC202CS 2MP Image Sensor Driver
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define SC202CS_REG_CHIP_ID		CCI_REG16(0x3107)
#define SC202CS_CHIP_ID			0xeb52

#define GC02M1_REG_CHIP_ID		CCI_REG8(0xf0)
#define GC02M1_CHIP_ID			0x02

#define SC202CS_DEFAULT_CLK_FREQ	(24 * HZ_PER_MHZ)
#define SC202CS_DATA_LANES		1

static const char * const sc202cs_supply_names[] = {
	"dovdd",
	"avdd",
	"dvdd",
};

struct sc202cs {
	struct device *dev;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(sc202cs_supply_names)];
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
};

static inline struct sc202cs *to_sc202cs(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc202cs, sd);
}

static int sc202cs_power_on(struct device *dev)
{
	struct sc202cs *sc202cs = dev_get_drvdata(dev);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sc202cs_supply_names),
				    sc202cs->supplies);
	if (ret < 0) {
		dev_err(sc202cs->dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(sc202cs->xclk);
	if (ret < 0) {
		regulator_bulk_disable(ARRAY_SIZE(sc202cs_supply_names),
				       sc202cs->supplies);
		dev_err(sc202cs->dev, "clk prepare enable failed\n");
		return ret;
	}

	/* Pulse reset pin */
	gpiod_set_value_cansleep(sc202cs->reset_gpio, 1);
	usleep_range(5000, 10000);
	gpiod_set_value_cansleep(sc202cs->reset_gpio, 0);
	usleep_range(10000, 15000);

	return 0;
}

static int sc202cs_power_off(struct device *dev)
{
	struct sc202cs *sc202cs = dev_get_drvdata(dev);

	gpiod_set_value_cansleep(sc202cs->reset_gpio, 1);
	clk_disable_unprepare(sc202cs->xclk);
	regulator_bulk_disable(ARRAY_SIZE(sc202cs_supply_names),
			       sc202cs->supplies);

	return 0;
}

static int sc202cs_identify_module(struct sc202cs *sc202cs)
{
	u64 val = 0;
	int ret;

	/* Try SmartSens SC202CS (0x3107 == 0xeb52) */
	ret = cci_read(sc202cs->regmap, SC202CS_REG_CHIP_ID, &val, NULL);
	if (!ret && ((val & 0xffff) == SC202CS_CHIP_ID)) {
		dev_info(sc202cs->dev, "found SmartSens SC202CS sensor (chip ID 0x%04llx)\n", val);
		return 0;
	}

	/* Try GalaxyCore GC02M1 fallback */
	ret = cci_read(sc202cs->regmap, GC02M1_REG_CHIP_ID, &val, NULL);
	if (!ret && ((val & 0xff) == GC02M1_CHIP_ID)) {
		dev_info(sc202cs->dev, "found GalaxyCore GC02M1 sensor (chip ID 0x%02llx)\n", val);
		return 0;
	}

	if (ret) {
		dev_warn(sc202cs->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	dev_err(sc202cs->dev, "sensor chip id mismatch (read 0x%llx, expected 0x%04x)\n",
		val, SC202CS_CHIP_ID);
	return -ENXIO;
}

static int sc202cs_parse_fwnode(struct sc202cs *sc202cs)
{
	struct fwnode_handle *endpoint;
	struct device *dev = sc202cs->dev;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	fwnode_handle_put(endpoint);
	return 0;
}

static const struct v4l2_subdev_ops sc202cs_subdev_ops = {
};

static const struct media_entity_operations sc202cs_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int sc202cs_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sc202cs *sc202cs;
	int i, ret;

	sc202cs = devm_kzalloc(dev, sizeof(*sc202cs), GFP_KERNEL);
	if (!sc202cs)
		return -ENOMEM;

	sc202cs->dev = dev;

	ret = sc202cs_parse_fwnode(sc202cs);
	if (ret)
		return ret;

	sc202cs->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sc202cs->regmap))
		return dev_err_probe(dev, PTR_ERR(sc202cs->regmap),
				     "failed to init CCI\n");

	for (i = 0; i < ARRAY_SIZE(sc202cs_supply_names); i++)
		sc202cs->supplies[i].supply = sc202cs_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sc202cs_supply_names),
				      sc202cs->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	sc202cs->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sc202cs->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sc202cs->reset_gpio),
				     "failed to get reset-gpios\n");

	sc202cs->xclk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(sc202cs->xclk))
		return dev_err_probe(dev, PTR_ERR(sc202cs->xclk),
				     "failed to get xclk\n");

	v4l2_i2c_subdev_init(&sc202cs->sd, client, &sc202cs_subdev_ops);
	dev_set_drvdata(dev, sc202cs);

	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		dev_err(dev, "failed to resume runtime PM: %d\n", ret);
		pm_runtime_disable(dev);
		return ret;
	}

	ret = sc202cs_identify_module(sc202cs);
	if (ret) {
		dev_warn(dev, "sensor not responding yet (%d), deferring probe\n", ret);
		pm_runtime_put_sync_suspend(dev);
		pm_runtime_disable(dev);
		return -EPROBE_DEFER;
	}

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	v4l2_ctrl_handler_init(&sc202cs->ctrl_handler, 4);
	sc202cs->sd.ctrl_handler = &sc202cs->ctrl_handler;

	sc202cs->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sc202cs->pad.flags = MEDIA_PAD_FL_SOURCE;
	sc202cs->sd.entity.ops = &sc202cs_subdev_entity_ops;
	sc202cs->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&sc202cs->sd.entity, 1, &sc202cs->pad);
	if (ret < 0) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto err_rpm;
	}

	ret = v4l2_async_register_subdev_sensor(&sc202cs->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register subdev sensor: %d\n", ret);
		goto err_media_cleanup;
	}

	dev_info(dev, "SmartSens SC202CS probe complete\n");
	return 0;

err_media_cleanup:
	media_entity_cleanup(&sc202cs->sd.entity);
err_rpm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	return ret;
}

static void sc202cs_remove(struct i2c_client *client)
{
	struct sc202cs *sc202cs = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(&sc202cs->sd);
	media_entity_cleanup(&sc202cs->sd.entity);
	v4l2_ctrl_handler_free(&sc202cs->ctrl_handler);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		sc202cs_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id sc202cs_of_match[] = {
	{ .compatible = "smartsens,sc202cs" },
	{}
};
MODULE_DEVICE_TABLE(of, sc202cs_of_match);

static DEFINE_RUNTIME_DEV_PM_OPS(sc202cs_pm_ops,
				 sc202cs_power_off,
				 sc202cs_power_on,
				 NULL);

static struct i2c_driver sc202cs_i2c_driver = {
	.driver = {
		.of_match_table = sc202cs_of_match,
		.pm = pm_ptr(&sc202cs_pm_ops),
		.name  = "sc202cs",
	},
	.probe  = sc202cs_probe,
	.remove = sc202cs_remove,
};
module_i2c_driver(sc202cs_i2c_driver);

MODULE_DESCRIPTION("SmartSens SC202CS Camera Driver");
MODULE_LICENSE("GPL");
