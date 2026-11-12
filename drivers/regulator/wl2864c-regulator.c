// SPDX-License-Identifier: GPL-2.0-only
/*
 * Will Semiconductor WL2864C camera PMIC: seven I2C controlled LDOs.
 *
 * Register layout as used by the Lenovo TB320FC camera stack
 * (cam_power_dev.c):
 *   0x01        control, 0x2f selects the higher current limit
 *   0x03..0x09  LDO1..LDO7 output voltage
 *   0x0e        enable, bit n = LDO n + 1
 * LDO1/LDO2 output 600 mV + 12.5 mV * code (0x28 = 1.1 V, 0x30 = 1.2 V),
 * LDO3..LDO7 output 1200 mV + 12.5 mV * code (0x80 = 2.8 V).
 */
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>

#define WL2864C_REG_CHIP_REV		0x00
#define WL2864C_REG_CTRL		0x01
#define WL2864C_REG_VOUT(n)		(0x03 + (n))
#define WL2864C_REG_ENABLE		0x0e
#define WL2864C_REG_MAX			0x0f

#define WL2864C_CTRL_HIGH_ILIMIT	0x2f

#define WL2864C_NUM_LDOS		7
#define WL2864C_ENABLE_TIME_US		1000

static const struct regulator_ops wl2864c_ldo_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

/* low-voltage LDOs: 0.6 V .. 1.8 V, high-voltage LDOs: 1.2 V .. 4.3 V */
#define WL2864C_LDO(_n, _supply, _min, _max)				\
	{								\
		.name = "ldo" #_n,					\
		.of_match = of_match_ptr("ldo" #_n),			\
		.regulators_node = of_match_ptr("regulators"),		\
		.supply_name = _supply,					\
		.id = (_n) - 1,						\
		.type = REGULATOR_VOLTAGE,				\
		.owner = THIS_MODULE,					\
		.ops = &wl2864c_ldo_ops,				\
		.min_uV = _min,						\
		.uV_step = 12500,					\
		.n_voltages = ((_max) - (_min)) / 12500 + 1,		\
		.vsel_reg = WL2864C_REG_VOUT((_n) - 1),			\
		.vsel_mask = 0xff,					\
		.enable_reg = WL2864C_REG_ENABLE,			\
		.enable_mask = BIT((_n) - 1),				\
		.enable_time = WL2864C_ENABLE_TIME_US,			\
	}

static const struct regulator_desc wl2864c_ldos[WL2864C_NUM_LDOS] = {
	WL2864C_LDO(1, "vin1", 600000, 1800000),
	WL2864C_LDO(2, "vin1", 600000, 1800000),
	WL2864C_LDO(3, "vin2", 1200000, 4300000),
	WL2864C_LDO(4, "vin2", 1200000, 4300000),
	WL2864C_LDO(5, "vin2", 1200000, 4300000),
	WL2864C_LDO(6, "vin2", 1200000, 4300000),
	WL2864C_LDO(7, "vin2", 1200000, 4300000),
};

static const struct regmap_config wl2864c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = WL2864C_REG_MAX,
};

static int wl2864c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { };
	struct regmap *regmap;
	unsigned int rev;
	int i, ret;

	/* I2C interface supply; the LDO inputs are requested per LDO */
	ret = devm_regulator_get_enable(dev, "vddio");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable vddio\n");
	usleep_range(1000, 2000);

	regmap = devm_regmap_init_i2c(client, &wl2864c_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "failed to init regmap\n");

	ret = regmap_read(regmap, WL2864C_REG_CHIP_REV, &rev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read chip revision\n");

	ret = regmap_write(regmap, WL2864C_REG_CTRL, WL2864C_CTRL_HIGH_ILIMIT);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set current limit\n");

	config.dev = dev;
	config.regmap = regmap;

	for (i = 0; i < WL2864C_NUM_LDOS; i++) {
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(dev, &wl2864c_ldos[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     wl2864c_ldos[i].name);
	}

	dev_dbg(dev, "WL2864C revision 0x%02x\n", rev);

	return 0;
}

static const struct of_device_id wl2864c_of_match[] = {
	{ .compatible = "willsemi,wl2864c" },
	{ }
};
MODULE_DEVICE_TABLE(of, wl2864c_of_match);

static const struct i2c_device_id wl2864c_id[] = {
	{ "wl2864c" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wl2864c_id);

static struct i2c_driver wl2864c_driver = {
	.driver = {
		.name = "wl2864c",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = wl2864c_of_match,
	},
	.probe = wl2864c_probe,
	.id_table = wl2864c_id,
};
module_i2c_driver(wl2864c_driver);

MODULE_DESCRIPTION("Will Semiconductor WL2864C camera PMIC regulator driver");
MODULE_LICENSE("GPL");
