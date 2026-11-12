// SPDX-License-Identifier: GPL-2.0
/*
 * SmartSens SC202CS 2MP image sensor driver
 *
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

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define SC202CS_REG_SOFT_RESET		CCI_REG8(0x0103)
#define SC202CS_REG_STREAM		CCI_REG8(0x0100)

#define SC202CS_REG_CHIP_ID		CCI_REG16(0x3107)
#define SC202CS_CHIP_ID			0xeb52

/* GC02M1 is only used by PRC variant, not supported by this driver */
#define GC02M1_REG_CHIP_ID		CCI_REG8(0xf0)
#define GC02M1_CHIP_ID			0x02

/* Frame length, 15 bits: 0x320e[6:0] high, 0x320f low */
#define SC202CS_REG_VTS			CCI_REG16(0x320e)
#define SC202CS_VTS_MAX			0x7fff

/* Exposure in lines, 16 bits: 0x3e00[3:0], 0x3e01, 0x3e02[7:4] */
#define SC202CS_REG_EXPOSURE		CCI_REG24(0x3e00)
#define SC202CS_EXPOSURE_MIN		1
#define SC202CS_EXPOSURE_MARGIN		6

/* Analogue gain: power-of-two coarse steps in 0x3e09, fine gain in 0x3e07 */
#define SC202CS_REG_AGAIN_FINE		CCI_REG8(0x3e07)
#define SC202CS_REG_AGAIN_COARSE	CCI_REG8(0x3e09)
#define SC202CS_AGAIN_UNIT		128	/* control value for 1x */
#define SC202CS_AGAIN_FINE_MAX		0xfe
#define SC202CS_AGAIN_MIN		SC202CS_AGAIN_UNIT
#define SC202CS_AGAIN_MAX		(16 * SC202CS_AGAIN_FINE_MAX)

#define SC202CS_WIDTH			1600
#define SC202CS_HEIGHT			1200
#define SC202CS_HTS			1920
#define SC202CS_VTS_DEFAULT		1250
#define SC202CS_PIXEL_RATE		72000000
#define SC202CS_LINK_FREQ		360000000	/* 720 Mbps, RAW10, 1 lane */
#define SC202CS_MCLK_FREQ		(24 * HZ_PER_MHZ)
#define SC202CS_DATA_LANES		1

/* Bayer order not verified yet; flip red/blue in a test capture to check */
#define SC202CS_MBUS_CODE		MEDIA_BUS_FMT_SBGGR10_1X10

static const char * const sc202cs_supply_names[] = {
	"dovdd",
	"avdd",
	"dvdd",
};

static const s64 sc202cs_link_freqs[] = {
	SC202CS_LINK_FREQ,
};

static const struct cci_reg_sequence sc202cs_init_regs[] = {
	{ CCI_REG8(0x36e9), 0x80 },
	{ CCI_REG8(0x36eb), 0x0c },
	{ CCI_REG8(0x36e9), 0x24 },
	{ CCI_REG8(0x301f), 0x01 },
	{ CCI_REG8(0x303f), 0x81 },
	{ CCI_REG8(0x3208), 0x06 },
	{ CCI_REG8(0x3209), 0x40 },
	{ CCI_REG8(0x320a), 0x04 },
	{ CCI_REG8(0x320b), 0xb0 },
	{ CCI_REG8(0x3211), 0x04 },
	{ CCI_REG8(0x3213), 0x04 },
	{ CCI_REG8(0x3215), 0x11 },
	{ CCI_REG8(0x3220), 0x13 },
	{ CCI_REG8(0x3250), 0x00 },
	{ CCI_REG8(0x3301), 0xff },
	{ CCI_REG8(0x3304), 0x68 },
	{ CCI_REG8(0x3306), 0x40 },
	{ CCI_REG8(0x3308), 0x08 },
	{ CCI_REG8(0x3309), 0xa8 },
	{ CCI_REG8(0x330b), 0xb0 },
	{ CCI_REG8(0x330c), 0x18 },
	{ CCI_REG8(0x330d), 0xff },
	{ CCI_REG8(0x330e), 0x20 },
	{ CCI_REG8(0x331e), 0x59 },
	{ CCI_REG8(0x331f), 0x99 },
	{ CCI_REG8(0x3333), 0x10 },
	{ CCI_REG8(0x335e), 0x06 },
	{ CCI_REG8(0x335f), 0x08 },
	{ CCI_REG8(0x3364), 0x1f },
	{ CCI_REG8(0x336c), 0xce },
	{ CCI_REG8(0x337c), 0x02 },
	{ CCI_REG8(0x337d), 0x0a },
	{ CCI_REG8(0x337e), 0x80 },
	{ CCI_REG8(0x338f), 0xa0 },
	{ CCI_REG8(0x3390), 0x01 },
	{ CCI_REG8(0x3391), 0x03 },
	{ CCI_REG8(0x3392), 0x1f },
	{ CCI_REG8(0x3393), 0xff },
	{ CCI_REG8(0x3394), 0xff },
	{ CCI_REG8(0x3395), 0xff },
	{ CCI_REG8(0x33a2), 0x04 },
	{ CCI_REG8(0x33ad), 0x0c },
	{ CCI_REG8(0x33b1), 0x20 },
	{ CCI_REG8(0x33b3), 0x38 },
	{ CCI_REG8(0x33f9), 0x40 },
	{ CCI_REG8(0x33fb), 0x48 },
	{ CCI_REG8(0x33fc), 0x0f },
	{ CCI_REG8(0x33fd), 0x1f },
	{ CCI_REG8(0x349f), 0x03 },
	{ CCI_REG8(0x34a6), 0x03 },
	{ CCI_REG8(0x34a7), 0x1f },
	{ CCI_REG8(0x34a8), 0x38 },
	{ CCI_REG8(0x34a9), 0x30 },
	{ CCI_REG8(0x34ab), 0xb0 },
	{ CCI_REG8(0x34ad), 0xb0 },
	{ CCI_REG8(0x34f8), 0x1f },
	{ CCI_REG8(0x34f9), 0x20 },
	{ CCI_REG8(0x3630), 0xa0 },
	{ CCI_REG8(0x3631), 0x92 },
	{ CCI_REG8(0x3632), 0x64 },
	{ CCI_REG8(0x3633), 0x43 },
	{ CCI_REG8(0x3637), 0x49 },
	{ CCI_REG8(0x363a), 0x85 },
	{ CCI_REG8(0x363c), 0x0f },
	{ CCI_REG8(0x3650), 0x31 },
	{ CCI_REG8(0x3670), 0x0d },
	{ CCI_REG8(0x3674), 0xc0 },
	{ CCI_REG8(0x3675), 0xa0 },
	{ CCI_REG8(0x3676), 0xa0 },
	{ CCI_REG8(0x3677), 0x92 },
	{ CCI_REG8(0x3678), 0x96 },
	{ CCI_REG8(0x3679), 0x9a },
	{ CCI_REG8(0x367c), 0x03 },
	{ CCI_REG8(0x367d), 0x0f },
	{ CCI_REG8(0x367e), 0x01 },
	{ CCI_REG8(0x367f), 0x0f },
	{ CCI_REG8(0x3698), 0x83 },
	{ CCI_REG8(0x3699), 0x86 },
	{ CCI_REG8(0x369a), 0x8c },
	{ CCI_REG8(0x369b), 0x94 },
	{ CCI_REG8(0x36a2), 0x01 },
	{ CCI_REG8(0x36a3), 0x03 },
	{ CCI_REG8(0x36a4), 0x07 },
	{ CCI_REG8(0x36ae), 0x0f },
	{ CCI_REG8(0x36af), 0x1f },
	{ CCI_REG8(0x36bd), 0x22 },
	{ CCI_REG8(0x36be), 0x22 },
	{ CCI_REG8(0x36bf), 0x22 },
	{ CCI_REG8(0x36d0), 0x01 },
	{ CCI_REG8(0x370f), 0x02 },
	{ CCI_REG8(0x3721), 0x6c },
	{ CCI_REG8(0x3722), 0x8d },
	{ CCI_REG8(0x3725), 0xc5 },
	{ CCI_REG8(0x3727), 0x14 },
	{ CCI_REG8(0x3728), 0x04 },
	{ CCI_REG8(0x37b7), 0x04 },
	{ CCI_REG8(0x37b8), 0x04 },
	{ CCI_REG8(0x37b9), 0x06 },
	{ CCI_REG8(0x37bd), 0x07 },
	{ CCI_REG8(0x37be), 0x0f },
	{ CCI_REG8(0x3901), 0x02 },
	{ CCI_REG8(0x3903), 0x40 },
	{ CCI_REG8(0x3905), 0x8d },
	{ CCI_REG8(0x3907), 0x00 },
	{ CCI_REG8(0x3908), 0x41 },
	{ CCI_REG8(0x391f), 0x41 },
	{ CCI_REG8(0x3933), 0x80 },
	{ CCI_REG8(0x3934), 0x02 },
	{ CCI_REG8(0x3937), 0x6f },
	{ CCI_REG8(0x393a), 0x01 },
	{ CCI_REG8(0x393d), 0x01 },
	{ CCI_REG8(0x393e), 0xc0 },
	{ CCI_REG8(0x39dd), 0x41 },
	{ CCI_REG8(0x3e00), 0x00 },
	{ CCI_REG8(0x3e01), 0x4d },
	{ CCI_REG8(0x3e02), 0xc0 },
	{ CCI_REG8(0x3e09), 0x00 },
	{ CCI_REG8(0x4509), 0x28 },
	{ CCI_REG8(0x450d), 0x61 },
	{ CCI_REG8(0x4800), 0x64 },
	{ CCI_REG8(0x4819), 0x09 },
	{ CCI_REG8(0x481b), 0x05 },
	{ CCI_REG8(0x481d), 0x13 },
	{ CCI_REG8(0x481f), 0x04 },
	{ CCI_REG8(0x4821), 0x0a },
	{ CCI_REG8(0x4823), 0x05 },
	{ CCI_REG8(0x4825), 0x04 },
	{ CCI_REG8(0x4827), 0x04 },
	{ CCI_REG8(0x4829), 0x08 },
	{ CCI_REG8(0x5000), 0x06 },
	{ CCI_REG8(0x5900), 0x01 },
	{ CCI_REG8(0x5901), 0x00 },
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
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
};

static inline struct sc202cs *to_sc202cs(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc202cs, sd);
}

static int sc202cs_set_gain(struct sc202cs *sc202cs, u32 gain)
{
	static const u8 coarse_codes[] = { 0x00, 0x01, 0x03, 0x07, 0x0f };
	unsigned int step = 0;
	u32 fine;
	int ret = 0;

	while (step < ARRAY_SIZE(coarse_codes) - 1 &&
	       gain >= (SC202CS_AGAIN_UNIT << (step + 1)))
		step++;

	fine = clamp_t(u32, gain >> step, SC202CS_AGAIN_UNIT,
		       SC202CS_AGAIN_FINE_MAX);

	cci_write(sc202cs->regmap, SC202CS_REG_AGAIN_COARSE,
		  coarse_codes[step], &ret);
	cci_write(sc202cs->regmap, SC202CS_REG_AGAIN_FINE, fine, &ret);

	return ret;
}

static int sc202cs_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc202cs *sc202cs = container_of(ctrl->handler, struct sc202cs,
					       ctrl_handler);
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		u32 exposure_max = SC202CS_HEIGHT + ctrl->val -
				   SC202CS_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(sc202cs->exposure,
					       sc202cs->exposure->minimum,
					       exposure_max,
					       sc202cs->exposure->step,
					       min_t(s64,
						     sc202cs->exposure->default_value,
						     exposure_max));
		if (ret)
			return ret;
	}

	if (!pm_runtime_get_if_active(sc202cs->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(sc202cs->regmap, SC202CS_REG_EXPOSURE,
				ctrl->val << 4, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = sc202cs_set_gain(sc202cs, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(sc202cs->regmap, SC202CS_REG_VTS,
				SC202CS_HEIGHT + ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(sc202cs->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc202cs_ctrl_ops = {
	.s_ctrl = sc202cs_set_ctrl,
};

static int sc202cs_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct sc202cs *sc202cs = to_sc202cs(sd);
	int ret;

	ret = pm_runtime_resume_and_get(sc202cs->dev);
	if (ret)
		return ret;

	ret = cci_write(sc202cs->regmap, SC202CS_REG_SOFT_RESET, 0x01, NULL);
	if (ret)
		goto err_rpm_put;
	usleep_range(2000, 3000);

	cci_write(sc202cs->regmap, SC202CS_REG_STREAM, 0x00, &ret);
	cci_multi_reg_write(sc202cs->regmap, sc202cs_init_regs,
			    ARRAY_SIZE(sc202cs_init_regs), &ret);
	if (ret)
		goto err_rpm_put;

	ret = __v4l2_ctrl_handler_setup(&sc202cs->ctrl_handler);
	if (ret)
		goto err_rpm_put;

	ret = cci_write(sc202cs->regmap, SC202CS_REG_STREAM, 0x01, NULL);
	if (ret)
		goto err_rpm_put;

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(sc202cs->dev);
	return ret;
}

static int sc202cs_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct sc202cs *sc202cs = to_sc202cs(sd);
	int ret;

	ret = cci_write(sc202cs->regmap, SC202CS_REG_STREAM, 0x00, NULL);
	pm_runtime_put_autosuspend(sc202cs->dev);

	return ret;
}

static void sc202cs_fill_format(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = SC202CS_WIDTH;
	fmt->height = SC202CS_HEIGHT;
	fmt->code = SC202CS_MBUS_CODE;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int sc202cs_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = SC202CS_MBUS_CODE;

	return 0;
}

static int sc202cs_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != SC202CS_MBUS_CODE)
		return -EINVAL;

	fse->min_width = SC202CS_WIDTH;
	fse->max_width = SC202CS_WIDTH;
	fse->min_height = SC202CS_HEIGHT;
	fse->max_height = SC202CS_HEIGHT;

	return 0;
}

static int sc202cs_set_format(struct v4l2_subdev *sd,
			      const struct v4l2_subdev_client_info *ci,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *format)
{
	sc202cs_fill_format(&format->format);
	*v4l2_subdev_state_get_format(state, 0) = format->format;

	return 0;
}

static int sc202cs_get_selection(struct v4l2_subdev *sd,
				 const struct v4l2_subdev_client_info *ci,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = SC202CS_WIDTH;
		sel->r.height = SC202CS_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int sc202cs_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	sc202cs_fill_format(v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops sc202cs_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops sc202cs_pad_ops = {
	.enum_mbus_code = sc202cs_enum_mbus_code,
	.enum_frame_size = sc202cs_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = sc202cs_set_format,
	.get_selection = sc202cs_get_selection,
	.enable_streams = sc202cs_enable_streams,
	.disable_streams = sc202cs_disable_streams,
};

static const struct v4l2_subdev_ops sc202cs_subdev_ops = {
	.video = &sc202cs_video_ops,
	.pad = &sc202cs_pad_ops,
};

static const struct v4l2_subdev_internal_ops sc202cs_internal_ops = {
	.init_state = sc202cs_init_state,
};

static int sc202cs_init_controls(struct sc202cs *sc202cs)
{
	struct v4l2_ctrl_handler *hdl = &sc202cs->ctrl_handler;
	const struct v4l2_ctrl_ops *ops = &sc202cs_ctrl_ops;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	u32 vblank_def = SC202CS_VTS_DEFAULT - SC202CS_HEIGHT;
	u32 exposure_max = SC202CS_VTS_DEFAULT - SC202CS_EXPOSURE_MARGIN;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 9);
	if (ret)
		return ret;

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      sc202cs_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, SC202CS_PIXEL_RATE,
			  SC202CS_PIXEL_RATE, 1, SC202CS_PIXEL_RATE);

	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK,
				 SC202CS_HTS - SC202CS_WIDTH,
				 SC202CS_HTS - SC202CS_WIDTH, 1,
				 SC202CS_HTS - SC202CS_WIDTH);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sc202cs->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
					    vblank_def,
					    SC202CS_VTS_MAX - SC202CS_HEIGHT,
					    1, vblank_def);

	sc202cs->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					      SC202CS_EXPOSURE_MIN,
					      exposure_max, 1, exposure_max);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN, SC202CS_AGAIN_MIN,
			  SC202CS_AGAIN_MAX, 1, SC202CS_AGAIN_MIN);

	ret = v4l2_fwnode_device_parse(sc202cs->dev, &props);
	if (ret)
		goto err_free;

	v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	sc202cs->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int sc202cs_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sc202cs *sc202cs = to_sc202cs(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sc202cs_supply_names),
				    sc202cs->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable regulators\n");

	ret = clk_prepare_enable(sc202cs->xclk);
	if (ret) {
		regulator_bulk_disable(ARRAY_SIZE(sc202cs_supply_names),
				       sc202cs->supplies);
		return dev_err_probe(dev, ret, "failed to enable clock\n");
	}

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(sc202cs->reset_gpio, 0);
	usleep_range(10000, 15000);

	return 0;
}

static int sc202cs_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sc202cs *sc202cs = to_sc202cs(sd);

	gpiod_set_value_cansleep(sc202cs->reset_gpio, 1);
	clk_disable_unprepare(sc202cs->xclk);
	regulator_bulk_disable(ARRAY_SIZE(sc202cs_supply_names),
			       sc202cs->supplies);

	return 0;
}

static int sc202cs_identify_module(struct sc202cs *sc202cs)
{
	u64 val;
	int ret;

	ret = cci_read(sc202cs->regmap, SC202CS_REG_CHIP_ID, &val, NULL);
	if (!ret && val == SC202CS_CHIP_ID)
		return 0;

	if (!cci_read(sc202cs->regmap, GC02M1_REG_CHIP_ID, &val, NULL) &&
	    (val & 0xff) == GC02M1_CHIP_ID)
		return dev_err_probe(sc202cs->dev, -ENODEV,
				     "GalaxyCore GC02M1 module variant is not supported\n");

	if (ret)
		return dev_err_probe(sc202cs->dev, ret,
				     "failed to read chip id\n");

	return dev_err_probe(sc202cs->dev, -ENXIO,
			     "unexpected chip id 0x%llx (expected 0x%04x)\n",
			     val, SC202CS_CHIP_ID);
}

static int sc202cs_check_hwcfg(struct sc202cs *sc202cs)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long link_freq_bitmap;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(sc202cs->dev), NULL);
	if (!ep)
		return dev_err_probe(sc202cs->dev, -EPROBE_DEFER,
				     "waiting for fwnode graph endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(sc202cs->dev, ret,
				     "failed to parse endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != SC202CS_DATA_LANES) {
		ret = dev_err_probe(sc202cs->dev, -EINVAL,
				    "%u data lanes not supported\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto out;
	}

	ret = v4l2_link_freq_to_bitmap(sc202cs->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       sc202cs_link_freqs,
				       ARRAY_SIZE(sc202cs_link_freqs),
				       &link_freq_bitmap);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int sc202cs_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sc202cs *sc202cs;
	unsigned long freq;
	int i, ret;

	sc202cs = devm_kzalloc(dev, sizeof(*sc202cs), GFP_KERNEL);
	if (!sc202cs)
		return -ENOMEM;

	sc202cs->dev = dev;

	ret = sc202cs_check_hwcfg(sc202cs);
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

	sc202cs->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(sc202cs->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sc202cs->reset_gpio),
				     "failed to get reset gpio\n");

	sc202cs->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sc202cs->xclk))
		return dev_err_probe(dev, PTR_ERR(sc202cs->xclk),
				     "failed to get clock\n");

	freq = clk_get_rate(sc202cs->xclk);
	if (freq != SC202CS_MCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "unsupported clock rate %lu\n", freq);

	v4l2_i2c_subdev_init(&sc202cs->sd, client, &sc202cs_subdev_ops);

	ret = sc202cs_power_on(dev);
	if (ret)
		return ret;

	ret = sc202cs_identify_module(sc202cs);
	if (ret)
		goto err_power_off;

	ret = sc202cs_init_controls(sc202cs);
	if (ret)
		goto err_power_off;

	sc202cs->sd.internal_ops = &sc202cs_internal_ops;
	sc202cs->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sc202cs->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sc202cs->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sc202cs->sd.entity, 1, &sc202cs->pad);
	if (ret)
		goto err_ctrls;

	sc202cs->sd.state_lock = sc202cs->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&sc202cs->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&sc202cs->sd);
	if (ret)
		goto err_rpm;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	return 0;

err_rpm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&sc202cs->sd);
err_entity:
	media_entity_cleanup(&sc202cs->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&sc202cs->ctrl_handler);
err_power_off:
	sc202cs_power_off(dev);
	return ret;
}

static void sc202cs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc202cs *sc202cs = to_sc202cs(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sc202cs->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		sc202cs_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(sc202cs_pm_ops, sc202cs_power_off,
				 sc202cs_power_on, NULL);

static const struct of_device_id sc202cs_of_match[] = {
	{ .compatible = "smartsens,sc202cs" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc202cs_of_match);

static struct i2c_driver sc202cs_i2c_driver = {
	.driver = {
		.name = "sc202cs",
		.of_match_table = sc202cs_of_match,
		.pm = pm_ptr(&sc202cs_pm_ops),
	},
	.probe = sc202cs_probe,
	.remove = sc202cs_remove,
};
module_i2c_driver(sc202cs_i2c_driver);

MODULE_DESCRIPTION("SmartSens SC202CS image sensor driver");
MODULE_LICENSE("GPL");
