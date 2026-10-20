// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/bits.h>
#include <linux/iopoll.h>
#include <linux/pm_domain.h>
#include <linux/reset.h>

#include "iris_instance.h"
#include "iris_vpu_common.h"

#include "iris_vpu_register_defines.h"

/* SM8450 runs the video hardware domain under software control. */
static int iris_vpu2_sm8450_set_hwmode(struct iris_core *core)
{
	return dev_pm_genpd_set_hwmode(core->pmdomain_tbl->pd_devs[IRIS_HW_POWER_DOMAIN], false);
}

static void iris_vpu2_power_off_hw(struct iris_core *core)
{
	u32 value, i;
	int ret;

	if (!(readl(core->reg_base + WRAPPER_CORE_POWER_STATUS) & BIT(1)))
		goto disable_power;

	value = readl(core->reg_base + WRAPPER_CORE_CLOCK_CONFIG);
	if (value)
		writel(CORE_CLK_RUN, core->reg_base + WRAPPER_CORE_CLOCK_CONFIG);

	for (i = 0; i < core->iris_platform_data->num_vpp_pipe; i++) {
		ret = readl_poll_timeout(core->reg_base + VCODEC_SS_IDLE_STATUSN + 4 * i,
					 value, value & BIT(22), 2000, 20000);
		if (ret) {
			dev_err(core->dev, "VPU2 pipe %u idle timeout: %#x\n", i, value);
			goto disable_power;
		}
	}

	if (core->iris_platform_data->no_aon)
		goto reset_bridge;

	writel(VIDEO_NOC_RESET_REQ, core->reg_base + AON_WRAPPER_MVP_NOC_RESET_REQ);
	ret = readl_poll_timeout(core->reg_base + AON_WRAPPER_MVP_NOC_RESET_ACK,
				 value, (value & VIDEO_NOC_RESET_REQ) == VIDEO_NOC_RESET_REQ,
				 200, 2000);
	/* Release the request even if asserting the reset times out. */
	writel(0, core->reg_base + AON_WRAPPER_MVP_NOC_RESET_REQ);
	if (ret) {
		dev_err(core->dev, "VPU2 NoC reset assertion timeout: %#x\n", value);
		goto disable_power;
	}

	ret = readl_poll_timeout(core->reg_base + AON_WRAPPER_MVP_NOC_RESET_ACK,
				 value, !(value & VIDEO_NOC_RESET_REQ), 200, 2000);
	if (ret) {
		dev_err(core->dev, "VPU2 NoC reset release timeout: %#x\n", value);
		goto disable_power;
	}

reset_bridge:
	writel(CORE_BRIDGE_SW_RESET | CORE_BRIDGE_HW_RESET_DISABLE,
	       core->reg_base + CPU_CS_AHB_BRIDGE_SYNC_RESET);
	writel(CORE_BRIDGE_HW_RESET_DISABLE, core->reg_base + CPU_CS_AHB_BRIDGE_SYNC_RESET);
	writel(0, core->reg_base + CPU_CS_AHB_BRIDGE_SYNC_RESET);

disable_power:
	iris_vpu_power_off_hw(core);
}

static u64 iris_vpu2_calc_freq(struct iris_inst *inst, size_t data_size)
{
	struct platform_inst_caps *caps = inst->core->iris_platform_data->inst_caps;
	struct v4l2_format *inp_f = inst->fmt_src;
	u32 mbs_per_second, mbpf, height, width;
	unsigned long vpp_freq, vsp_freq;
	u32 fps = inst->frame_rate;

	width = max(inp_f->fmt.pix_mp.width, inst->crop.width);
	height = max(inp_f->fmt.pix_mp.height, inst->crop.height);

	mbpf = NUM_MBS_PER_FRAME(height, width);
	mbs_per_second = mbpf * fps;

	vpp_freq = mbs_per_second * caps->mb_cycles_vpp;

	/* 21 / 20 is overhead factor */
	vpp_freq += vpp_freq / 20;
	vsp_freq = mbs_per_second * caps->mb_cycles_vsp;

	/* 10 / 7 is overhead factor */
	vsp_freq += ((fps * data_size * 8) * 10) / 7;

	return max(vpp_freq, vsp_freq);
}

static void iris_vpu2_sm8450_program_bootup_registers(struct iris_core *core)
{
	u32 value;

	/* Match the SM8450 core-clock preset after the domains power on. */
	value = readl(core->reg_base + WRAPPER_CORE_CLOCK_CONFIG);
	writel(value & ~(BIT(0) | BIT(4)), core->reg_base + WRAPPER_CORE_CLOCK_CONFIG);
}

const struct vpu_ops iris_vpu2_ops = {
	.power_off_hw = iris_vpu_power_off_hw,
	.power_on_hw = iris_vpu_power_on_hw,
	.power_off_controller = iris_vpu_power_off_controller,
	.power_on_controller = iris_vpu_power_on_controller,
	.calc_freq = iris_vpu2_calc_freq,
	.set_hwmode = iris_vpu_set_hwmode,
};

const struct vpu_ops iris_vpu2_sm8450_ops = {
	.power_off_hw = iris_vpu2_power_off_hw,
	.power_on_hw = iris_vpu_power_on_hw,
	.power_off_controller = iris_vpu_power_off_controller,
	.power_on_controller = iris_vpu_power_on_controller,
	.calc_freq = iris_vpu2_calc_freq,
	.program_bootup_registers = iris_vpu2_sm8450_program_bootup_registers,
	.set_hwmode = iris_vpu2_sm8450_set_hwmode,
};
