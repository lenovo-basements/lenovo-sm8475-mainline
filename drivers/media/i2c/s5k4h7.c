// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5K4H7 8MP CMOS image sensor driver
 *
 * Copyright (c) 2026 Antigravity / Linux Mainline Porting Team
 * Based on Linaro S5K3M5 / S5KJN1 drivers by Vladimir Zapolskiy.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define S5K4H7_LINK_FREQ_336MHZ		(336ULL * HZ_PER_MHZ)
#define S5K4H7_LINK_FREQ_207MHZ		(207ULL * HZ_PER_MHZ)
#define S5K4H7_MCLK_FREQ_24MHZ		(24 * HZ_PER_MHZ)
#define S5K4H7_DATA_LANES		4

/* Register map compliant with MIPI CCS / Samsung ISOCELL standard */
#define S5K4H7_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K4H7_CHIP_ID			0x487b

#define S5K4H7_REG_CTRL_MODE		CCI_REG8(0x0100)
#define S5K4H7_MODE_STREAMING		BIT(0)

#define S5K4H7_REG_ORIENTATION		CCI_REG8(0x0101)
#define S5K4H7_HFLIP			BIT(0)
#define S5K4H7_VFLIP			BIT(1)

#define S5K4H7_REG_SW_RESET		CCI_REG8(0x0103)
#define S5K4H7_SW_RESET			0x01

#define S5K4H7_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K4H7_EXPOSURE_MIN		4
#define S5K4H7_EXPOSURE_STEP		1
#define S5K4H7_EXPOSURE_MARGIN		16

#define S5K4H7_REG_AGAIN		CCI_REG16(0x0204)
#define S5K4H7_AGAIN_MIN		1
#define S5K4H7_AGAIN_MAX		16
#define S5K4H7_AGAIN_STEP		1
#define S5K4H7_AGAIN_DEFAULT		1
#define S5K4H7_AGAIN_SHIFT		5

#define S5K4H7_REG_VTS			CCI_REG16(0x0340)
#define S5K4H7_VTS_MAX			0xffff

#define S5K4H7_REG_HTS			CCI_REG16(0x0342)
#define S5K4H7_REG_X_ADDR_START		CCI_REG16(0x0344)
#define S5K4H7_REG_Y_ADDR_START		CCI_REG16(0x0346)
#define S5K4H7_REG_X_ADDR_END		CCI_REG16(0x0348)
#define S5K4H7_REG_Y_ADDR_END		CCI_REG16(0x034a)
#define S5K4H7_REG_X_OUTPUT_SIZE	CCI_REG16(0x034c)
#define S5K4H7_REG_Y_OUTPUT_SIZE	CCI_REG16(0x034e)

#define S5K4H7_REG_TEST_PATTERN		CCI_REG16(0x0600)

#define to_s5k4h7(_sd)			container_of(_sd, struct s5k4h7, sd)

static const s64 s5k4h7_link_freq_menu[] = {
	S5K4H7_LINK_FREQ_336MHZ,
	S5K4H7_LINK_FREQ_207MHZ,
};

static const u32 s5k4h7_mbus_formats[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

struct s5k4h7_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5k4h7_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u32 exposure;

	const struct s5k4h7_reg_list reg_list;
};

static const char * const s5k4h7_test_pattern_menu[] = {
	"Disabled",
	"Solid colour",
	"Colour bars",
	"Fade to grey colour bars",
	"PN9",
};

static const char * const s5k4h7_supply_names[] = {
	"dovdd",	/* Digital I/O (1.8V) */
	"avdd",		/* Analog power (2.8V) */
	"dvdd",		/* Digital core (1.2V) */
};

#define S5K4H7_NUM_SUPPLIES	ARRAY_SIZE(s5k4h7_supply_names)

struct s5k4h7 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[S5K4H7_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;

	const struct s5k4h7_mode *mode;
};

/* S5K4H7 Global Init Register Sequence */
static const struct cci_reg_sequence s5k4h7_global_setting[] = {
	{ CCI_REG8(0x0a02), 0x01 },
	{ CCI_REG8(0x0b05), 0x01 },
	{ CCI_REG8(0x3074), 0x06 },
	{ CCI_REG8(0x3075), 0x2f },
	{ CCI_REG8(0x308a), 0x20 },
	{ CCI_REG8(0x308b), 0x08 },
	{ CCI_REG8(0x308c), 0x05 },
	{ CCI_REG8(0x3081), 0x07 },
	{ CCI_REG8(0x307b), 0x85 },
	{ CCI_REG8(0x307a), 0x0a },
	{ CCI_REG8(0x3079), 0x0a },
	{ CCI_REG8(0x306e), 0x71 },
	{ CCI_REG8(0x306f), 0x28 },
	{ CCI_REG8(0x301f), 0x9a },
	{ CCI_REG8(0x306b), 0x1f },
	{ CCI_REG8(0x3091), 0x79 },
	{ CCI_REG8(0x30c4), 0xff },
	{ CCI_REG8(0x3200), 0x3f },
	{ CCI_REG8(0x306a), 0x16 },
	{ CCI_REG8(0x30b0), 0xff },
	{ CCI_REG8(0x306d), 0x87 },
	{ CCI_REG8(0x3080), 0x7f },
	{ CCI_REG8(0x3929), 0x3f },
	{ CCI_REG8(0x3084), 0x16 },
	{ CCI_REG8(0x3070), 0x0f },
	{ CCI_REG8(0x3b45), 0x01 },
	{ CCI_REG8(0x30c2), 0x05 },
	{ CCI_REG8(0x3069), 0x87 },
	{ CCI_REG8(0x3924), 0x7f },
	{ CCI_REG8(0x3925), 0xfd },
	{ CCI_REG8(0x3c08), 0xff },
	{ CCI_REG8(0x3c09), 0xff },
	{ CCI_REG8(0x3c31), 0xff },
	{ CCI_REG8(0x3c32), 0xff },
};

/* 3264x2448 @ 30fps Full Resolution Mode */
static const struct cci_reg_sequence s5k4h7_3264x2448_regs[] = {
	{ S5K4H7_REG_X_ADDR_START, 0x0000 },
	{ S5K4H7_REG_Y_ADDR_START, 0x0000 },
	{ S5K4H7_REG_X_ADDR_END, 3263 },
	{ S5K4H7_REG_Y_ADDR_END, 2447 },
	{ S5K4H7_REG_X_OUTPUT_SIZE, 3264 },
	{ S5K4H7_REG_Y_OUTPUT_SIZE, 2448 },
	{ S5K4H7_REG_HTS, 3536 },
	{ S5K4H7_REG_VTS, 2496 },
};

/* 1632x1224 @ 30fps Binned Mode */
static const struct cci_reg_sequence s5k4h7_1632x1224_regs[] = {
	{ S5K4H7_REG_X_ADDR_START, 0x0000 },
	{ S5K4H7_REG_Y_ADDR_START, 0x0000 },
	{ S5K4H7_REG_X_ADDR_END, 3263 },
	{ S5K4H7_REG_Y_ADDR_END, 2447 },
	{ S5K4H7_REG_X_OUTPUT_SIZE, 1632 },
	{ S5K4H7_REG_Y_OUTPUT_SIZE, 1224 },
	{ S5K4H7_REG_HTS, 3536 },
	{ S5K4H7_REG_VTS, 2496 },
};

static const struct s5k4h7_mode s5k4h7_supported_modes[] = {
	{
		.width = 3264,
		.height = 2448,
		.hts = 3536,
		.vts = 2496,
		.exposure = 2000,
		.reg_list = {
			.regs = s5k4h7_3264x2448_regs,
			.num_regs = ARRAY_SIZE(s5k4h7_3264x2448_regs),
		},
	},
	{
		.width = 1632,
		.height = 1224,
		.hts = 3536,
		.vts = 2496,
		.exposure = 2000,
		.reg_list = {
			.regs = s5k4h7_1632x1224_regs,
			.num_regs = ARRAY_SIZE(s5k4h7_1632x1224_regs),
		},
	},
};

static inline u32 s5k4h7_get_format_code(struct s5k4h7 *s5k4h7)
{
	unsigned int index = 0;

	if (s5k4h7->vflip->val)
		index |= BIT(1);
	if (s5k4h7->hflip->val)
		index |= BIT(0);

	return s5k4h7_mbus_formats[index];
}

static int s5k4h7_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4h7 *s5k4h7 = container_of(ctrl->handler, struct s5k4h7,
					     ctrl_handler);
	const struct s5k4h7_mode *mode = s5k4h7->mode;
	s64 exposure_max;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		return 0;
	case V4L2_CID_VBLANK:
		exposure_max = mode->height + ctrl->val - S5K4H7_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(s5k4h7->exposure,
					 s5k4h7->exposure->minimum,
					 exposure_max,
					 s5k4h7->exposure->step,
					 s5k4h7->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_active(s5k4h7->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_AGAIN,
				ctrl->val << S5K4H7_AGAIN_SHIFT, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_VTS,
				mode->height + ctrl->val, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5k4h7->regmap, S5K4H7_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(s5k4h7->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k4h7_ctrl_ops = {
	.s_ctrl = s5k4h7_set_ctrl,
};

static int s5k4h7_init_controls(struct s5k4h7 *s5k4h7)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	const struct s5k4h7_mode *mode = s5k4h7->mode;
	s64 exposure_max, hblank, vblank;
	u64 pixel_rate_val;
	int ret;

	ctrl_hdlr = &s5k4h7->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	s5k4h7->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
						   &s5k4h7_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(s5k4h7_link_freq_menu) - 1,
						   0, s5k4h7_link_freq_menu);
	if (s5k4h7->link_freq)
		s5k4h7->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Pixel rate: 336MHz * 2 * 4 lanes / 10 bpp = 268.8 Mpix/s */
	pixel_rate_val = 268800000ULL;
	s5k4h7->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       pixel_rate_val, pixel_rate_val, 1,
					       pixel_rate_val);

	hblank = mode->hts - mode->width;
	s5k4h7->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					   V4L2_CID_HBLANK,
					   hblank, hblank, 1, hblank);
	if (s5k4h7->hblank)
		s5k4h7->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank = mode->vts - mode->height;
	s5k4h7->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					   V4L2_CID_VBLANK,
					   vblank, S5K4H7_VTS_MAX - mode->height, 1,
					   vblank);

	exposure_max = mode->vts - S5K4H7_EXPOSURE_MARGIN;
	s5k4h7->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K4H7_EXPOSURE_MIN, exposure_max,
					     S5K4H7_EXPOSURE_STEP,
					     mode->exposure);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN,
			  S5K4H7_AGAIN_MIN, S5K4H7_AGAIN_MAX,
			  S5K4H7_AGAIN_STEP, S5K4H7_AGAIN_DEFAULT);

	s5k4h7->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	s5k4h7->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &s5k4h7_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5k4h7_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k4h7_test_pattern_menu) - 1,
				     0, 0, s5k4h7_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(s5k4h7->dev, "failed to add controls: %d\n", ret);
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ret;
	}

	s5k4h7->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static int s5k4h7_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	const struct s5k4h7_reg_list *reg_list = &s5k4h7->mode->reg_list;
	int ret;

	ret = pm_runtime_resume_and_get(s5k4h7->dev);
	if (ret < 0)
		return ret;

	/* Software Reset */
	ret = cci_write(s5k4h7->regmap, S5K4H7_REG_SW_RESET, S5K4H7_SW_RESET, NULL);
	if (ret)
		goto error;

	usleep_range(5 * USEC_PER_MSEC, 10 * USEC_PER_MSEC);

	/* Standby mode before programming */
	ret = cci_write(s5k4h7->regmap, S5K4H7_REG_CTRL_MODE, 0x00, NULL);
	if (ret)
		goto error;

	/* Global settings */
	ret = cci_multi_reg_write(s5k4h7->regmap, s5k4h7_global_setting,
				  ARRAY_SIZE(s5k4h7_global_setting), NULL);
	if (ret)
		goto error;

	/* Mode-specific settings */
	ret = cci_multi_reg_write(s5k4h7->regmap, reg_list->regs,
				  reg_list->num_regs, NULL);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(s5k4h7->sd.ctrl_handler);
	if (ret)
		goto error;

	/* Start Streaming */
	ret = cci_write(s5k4h7->regmap, S5K4H7_REG_CTRL_MODE,
			S5K4H7_MODE_STREAMING |
			(s5k4h7->vflip->val ? S5K4H7_VFLIP : 0) |
			(s5k4h7->hflip->val ? S5K4H7_HFLIP : 0), NULL);
	if (ret)
		goto error;

	return 0;

error:
	dev_err(s5k4h7->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(s5k4h7->dev);

	return ret;
}

static int s5k4h7_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	int ret;

	ret = cci_write(s5k4h7->regmap, S5K4H7_REG_CTRL_MODE, 0x00, NULL);
	if (ret)
		dev_err(s5k4h7->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_mark_last_busy(s5k4h7->dev);
	pm_runtime_put_autosuspend(s5k4h7->dev);

	return ret;
}

static int s5k4h7_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	const struct s5k4h7_mode *mode;
	s64 exposure_max, hblank, vblank;

	mode = v4l2_find_nearest_size(s5k4h7_supported_modes,
				      ARRAY_SIZE(s5k4h7_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = s5k4h7_get_format_code(s5k4h7);
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		goto set_format;

	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(s5k4h7->hblank, hblank, hblank, 1, hblank);

	vblank = mode->vts - mode->height;
	__v4l2_ctrl_modify_range(s5k4h7->vblank, vblank,
				 S5K4H7_VTS_MAX - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5k4h7->vblank, vblank);

	exposure_max = mode->vts - S5K4H7_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(s5k4h7->exposure, S5K4H7_EXPOSURE_MIN,
				 exposure_max, S5K4H7_EXPOSURE_STEP,
				 mode->exposure);
	__v4l2_ctrl_s_ctrl(s5k4h7->exposure, mode->exposure);

	if (s5k4h7->sd.ctrl_handler->error)
		return s5k4h7->sd.ctrl_handler->error;

	s5k4h7->mode = mode;

set_format:
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5k4h7_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	if (code->index > 0)
		return -EINVAL;

	code->code = s5k4h7_get_format_code(s5k4h7);

	return 0;
}

static int s5k4h7_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	if (fse->index >= ARRAY_SIZE(s5k4h7_supported_modes))
		return -EINVAL;

	if (fse->code != s5k4h7_get_format_code(s5k4h7))
		return -EINVAL;

	fse->min_width = s5k4h7_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k4h7_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5k4h7_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = s5k4h7->mode->width;
		sel->r.height = s5k4h7->mode->height;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5k4h7_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = s5k4h7->mode->width,
			.height = s5k4h7->mode->height,
		},
	};

	s5k4h7_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5k4h7_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k4h7_pad_ops = {
	.set_fmt = s5k4h7_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5k4h7_get_selection,
	.enum_mbus_code = s5k4h7_enum_mbus_code,
	.enum_frame_size = s5k4h7_enum_frame_size,
	.enable_streams = s5k4h7_enable_streams,
	.disable_streams = s5k4h7_disable_streams,
};

static const struct v4l2_subdev_ops s5k4h7_subdev_ops = {
	.video = &s5k4h7_video_ops,
	.pad = &s5k4h7_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k4h7_internal_ops = {
	.init_state = s5k4h7_init_state,
};

static int s5k4h7_identify_sensor(struct s5k4h7 *s5k4h7)
{
	u64 val;
	int ret;

	ret = cci_read(s5k4h7->regmap, S5K4H7_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(s5k4h7->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != S5K4H7_CHIP_ID) {
		dev_err(s5k4h7->dev, "chip id mismatch: expected 0x%04x, got 0x%04llx\n",
			S5K4H7_CHIP_ID, val);
		return -ENODEV;
	}

	dev_info(s5k4h7->dev, "Found Samsung S5K4H7 image sensor (0x%04llx)\n", val);
	return 0;
}

static int s5k4h7_check_hwcfg(struct s5k4h7 *s5k4h7)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5k4h7->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus = {
			.mipi_csi2 = {
				.num_data_lanes = S5K4H7_DATA_LANES,
			},
		},
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K4H7_DATA_LANES) {
		dev_err(s5k4h7->dev, "unsupported number of CSI data lanes: %u\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
	}

	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5k4h7_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	int ret;

	ret = regulator_bulk_enable(S5K4H7_NUM_SUPPLIES, s5k4h7->supplies);
	if (ret)
		return ret;

	ret = clk_prepare_enable(s5k4h7->mclk);
	if (ret)
		goto disable_regulators;

	gpiod_set_value_cansleep(s5k4h7->reset_gpio, 0);
	usleep_range(10 * USEC_PER_MSEC, 15 * USEC_PER_MSEC);

	return 0;

disable_regulators:
	regulator_bulk_disable(S5K4H7_NUM_SUPPLIES, s5k4h7->supplies);

	return ret;
}

static int s5k4h7_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	gpiod_set_value_cansleep(s5k4h7->reset_gpio, 1);
	clk_disable_unprepare(s5k4h7->mclk);
	regulator_bulk_disable(S5K4H7_NUM_SUPPLIES, s5k4h7->supplies);

	return 0;
}

static int s5k4h7_probe(struct i2c_client *client)
{
	struct s5k4h7 *s5k4h7;
	unsigned int i;
	int ret;

	s5k4h7 = devm_kzalloc(&client->dev, sizeof(*s5k4h7), GFP_KERNEL);
	if (!s5k4h7)
		return -ENOMEM;

	s5k4h7->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5k4h7->sd, client, &s5k4h7_subdev_ops);
	s5k4h7->sd.internal_ops = &s5k4h7_internal_ops;

	s5k4h7->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k4h7->regmap))
		return dev_err_probe(s5k4h7->dev, PTR_ERR(s5k4h7->regmap),
				      "failed to initialize CCI regmap\n");

	ret = s5k4h7_check_hwcfg(s5k4h7);
	if (ret)
		return dev_err_probe(s5k4h7->dev, ret,
				      "failed to check hardware config\n");

	s5k4h7->mclk = devm_clk_get(s5k4h7->dev, "mclk");
	if (IS_ERR(s5k4h7->mclk))
		s5k4h7->mclk = devm_clk_get_optional(s5k4h7->dev, NULL);
	if (IS_ERR(s5k4h7->mclk))
		return dev_err_probe(s5k4h7->dev, PTR_ERR(s5k4h7->mclk),
				      "failed to get mclk\n");

	s5k4h7->reset_gpio = devm_gpiod_get_optional(s5k4h7->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5k4h7->reset_gpio))
		return dev_err_probe(s5k4h7->dev, PTR_ERR(s5k4h7->reset_gpio),
				      "failed to get reset GPIO\n");

	for (i = 0; i < S5K4H7_NUM_SUPPLIES; i++)
		s5k4h7->supplies[i].supply = s5k4h7_supply_names[i];

	ret = devm_regulator_bulk_get(s5k4h7->dev, S5K4H7_NUM_SUPPLIES,
				      s5k4h7->supplies);
	if (ret)
		return dev_err_probe(s5k4h7->dev, ret,
				      "failed to get regulators\n");

	pm_runtime_set_suspended(s5k4h7->dev);
	pm_runtime_enable(s5k4h7->dev);
	ret = pm_runtime_resume_and_get(s5k4h7->dev);
	if (ret < 0) {
		dev_err(s5k4h7->dev, "failed to resume runtime PM: %d\n", ret);
		pm_runtime_disable(s5k4h7->dev);
		return ret;
	}

	ret = s5k4h7_identify_sensor(s5k4h7);
	if (ret) {
		dev_warn(s5k4h7->dev, "sensor not responding yet (%d), deferring probe\n", ret);
		pm_runtime_put_sync_suspend(s5k4h7->dev);
		pm_runtime_disable(s5k4h7->dev);
		return -EPROBE_DEFER;
	}

	pm_runtime_set_autosuspend_delay(s5k4h7->dev, 1000);
	pm_runtime_use_autosuspend(s5k4h7->dev);
	pm_runtime_mark_last_busy(s5k4h7->dev);
	pm_runtime_put_autosuspend(s5k4h7->dev);

	s5k4h7->mode = &s5k4h7_supported_modes[0];

	ret = s5k4h7_init_controls(s5k4h7);
	if (ret)
		goto err_rpm;

	s5k4h7->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k4h7->pad.flags = MEDIA_PAD_FL_SOURCE;
	s5k4h7->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&s5k4h7->sd.entity, 1, &s5k4h7->pad);
	if (ret) {
		dev_err_probe(s5k4h7->dev, ret, "failed to init media entity pad\n");
		goto err_ctrls;
	}

	ret = v4l2_subdev_init_finalize(&s5k4h7->sd);
	if (ret < 0) {
		dev_err_probe(s5k4h7->dev, ret, "failed to finalize v4l2 subdev\n");
		goto err_entity;
	}

	ret = v4l2_async_register_subdev_sensor(&s5k4h7->sd);
	if (ret < 0) {
		dev_err_probe(s5k4h7->dev, ret, "failed to register V4L2 sensor subdev\n");
		goto err_subdev;
	}

	return 0;

err_subdev:
	v4l2_subdev_cleanup(&s5k4h7->sd);
err_entity:
	media_entity_cleanup(&s5k4h7->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(s5k4h7->sd.ctrl_handler);
err_rpm:
	pm_runtime_disable(s5k4h7->dev);
	if (!pm_runtime_status_suspended(s5k4h7->dev))
		s5k4h7_power_off(s5k4h7->dev);
	pm_runtime_set_suspended(s5k4h7->dev);
	return ret;
}

static void s5k4h7_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5k4h7->dev);

	if (!pm_runtime_status_suspended(s5k4h7->dev))
		s5k4h7_power_off(s5k4h7->dev);
	pm_runtime_set_suspended(s5k4h7->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(s5k4h7_pm_ops,
				 s5k4h7_power_off,
				 s5k4h7_power_on,
				 NULL);

static const struct of_device_id s5k4h7_of_match[] = {
	{ .compatible = "samsung,s5k4h7" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k4h7_of_match);

static struct i2c_driver s5k4h7_i2c_driver = {
	.driver = {
		.name = "s5k4h7",
		.pm = pm_ptr(&s5k4h7_pm_ops),
		.of_match_table = s5k4h7_of_match,
	},
	.probe = s5k4h7_probe,
	.remove = s5k4h7_remove,
};

module_i2c_driver(s5k4h7_i2c_driver);

MODULE_AUTHOR("Antigravity / Mainline Linux Team");
MODULE_DESCRIPTION("Samsung S5K4H7 image sensor driver");
MODULE_LICENSE("GPL");
