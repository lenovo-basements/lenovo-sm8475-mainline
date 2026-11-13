/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2015-2018, The Linux Foundation. All rights reserved.
 */

#ifndef _DPU_HW_DSPP_H
#define _DPU_HW_DSPP_H

struct dpu_hw_dspp;

/**
 * struct dpu_hw_pcc_coeff - PCC coefficient structure for each color
 *                            component.
 * @r: red coefficient.
 * @g: green coefficient.
 * @b: blue coefficient.
 */

struct dpu_hw_pcc_coeff {
	__u32 r;
	__u32 g;
	__u32 b;
};

/**
 * struct dpu_hw_pcc_cfg - pcc feature structure
 * @r: red coefficients.
 * @g: green coefficients.
 * @b: blue coefficients.
 */
struct dpu_hw_pcc_cfg {
	struct dpu_hw_pcc_coeff r;
	struct dpu_hw_pcc_coeff g;
	struct dpu_hw_pcc_coeff b;
};

#define DPU_GAMMA_LUT_SIZE 1024
#define PGC_TBL_LEN 512
#define PGC_8B_ROUND BIT(0)

/**
 * struct dpu_hw_gc_lut - gc lut feature structure
 * @flags: flags for the feature values can be:
 *         - PGC_8B_ROUND
 * @c0: color0 component lut
 * @c1: color1 component lut
 * @c2: color2 component lut
 */
struct dpu_hw_gc_lut {
	__u64 flags;
	__u32 c0[PGC_TBL_LEN];
	__u32 c1[PGC_TBL_LEN];
	__u32 c2[PGC_TBL_LEN];
};

struct dpu_hw_ctl;
struct dpu_hw_reg_dma;

#define DPU_IGC_LUT_SIZE	257	/* 256 nodes + end point */
#define DPU_GAMUT3D_NODES	17	/* 17x17x17 */
#define DPU_GAMUT3D_SIZE	(DPU_GAMUT3D_NODES * DPU_GAMUT3D_NODES * DPU_GAMUT3D_NODES)

/**
 * struct dpu_hw_igc_lut - DSPP IGC (1D, "degamma") LUT, 12-bit entries
 * @c: [0] red, [1] green, [2] blue; entry i is the output for input i/256
 */
struct dpu_hw_igc_lut {
	u16 c[3][DPU_IGC_LUT_SIZE];
};

/**
 * struct dpu_hw_gamut3d_lut - DSPP 17^3 3D LUT, 12-bit entries
 * @rgb: node r + 17 * g + 289 * b, components in r, g, b order
 */
struct dpu_hw_gamut3d_lut {
	u16 rgb[DPU_GAMUT3D_SIZE][3];
};

/**
 * struct dpu_hw_dspp_ops - interface to the dspp hardware driver functions
 * Caller must call the init function to get the dspp context for each dspp
 * Assumption is these functions will be called after clocks are enabled
 */
struct dpu_hw_dspp_ops {
	/**
	 * @setup_pcc: setup_pcc - setup dspp pcc
	 * @ctx: Pointer to dspp context
	 * @cfg: Pointer to configuration
	 */
	void (*setup_pcc)(struct dpu_hw_dspp *ctx, struct dpu_hw_pcc_cfg *cfg);

	/**
	 * setup_gc - setup dspp gc
	 * @ctx: Pointer to dspp context
	 * @gc_lut: Pointer to lut content
	 */
	void (*setup_gc)(struct dpu_hw_dspp *ctx, struct dpu_hw_gc_lut *gc_lut);

	/**
	 * setup_igc - queue a DSPP IGC update on the LUTDMA engine
	 * @ctx: Pointer to dspp context (owner of the descriptor buffer)
	 * @rd: LUTDMA engine (the IGC LUT has no MMIO data port)
	 * @ctl: CTL whose next kickoff triggers the update
	 * @dspp_mask: DPU_REG_DMA_BLK_DSPP() bits of every DSPP to load; one
	 *	packet is broadcast to all of them (as downstream does)
	 * @lut: LUT, or NULL to disable IGC
	 * Return: 0 on success, negative errno otherwise
	 */
	int (*setup_igc)(struct dpu_hw_dspp *ctx, struct dpu_hw_reg_dma *rd,
			 struct dpu_hw_ctl *ctl, u32 dspp_mask,
			 const struct dpu_hw_igc_lut *lut);

	/**
	 * setup_gamut3d - queue a DSPP 3D LUT update on the LUTDMA engine
	 * @ctx: Pointer to dspp context (owner of the descriptor buffer)
	 * @rd: LUTDMA engine (the 3D LUT has no MMIO data port)
	 * @ctl: CTL whose next kickoff triggers the update
	 * @dspp_mask: DPU_REG_DMA_BLK_DSPP() bits of every DSPP to load
	 * @lut: LUT, or NULL to disable the 3D LUT
	 * Return: 0 on success, negative errno otherwise
	 */
	int (*setup_gamut3d)(struct dpu_hw_dspp *ctx, struct dpu_hw_reg_dma *rd,
			     struct dpu_hw_ctl *ctl, u32 dspp_mask,
			     const struct dpu_hw_gamut3d_lut *lut);
};

/**
 * struct dpu_hw_dspp - dspp description
 * @base: Hardware block base structure
 * @hw: Block hardware details
 * @idx: DSPP index
 * @cap: Pointer to layer_cfg
 * @ops: Pointer to operations possible for this DSPP
 */
struct dpu_hw_dspp {
	struct dpu_hw_blk base;
	struct dpu_hw_blk_reg_map hw;

	/* dspp */
	int idx;
	const struct dpu_dspp_cfg *cap;

	/* Ops */
	struct dpu_hw_dspp_ops ops;
};

/**
 * to_dpu_hw_dspp - convert base object dpu_hw_base to container
 * @hw: Pointer to base hardware block
 * return: Pointer to hardware block container
 */
static inline struct dpu_hw_dspp *to_dpu_hw_dspp(struct dpu_hw_blk *hw)
{
	return container_of(hw, struct dpu_hw_dspp, base);
}

struct dpu_hw_dspp *dpu_hw_dspp_init(struct drm_device *dev,
				     const struct dpu_dspp_cfg *cfg,
				     void __iomem *addr);

#endif /*_DPU_HW_DSPP_H */

