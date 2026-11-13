// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2015-2018, The Linux Foundation. All rights reserved.
 */

#include <drm/drm_managed.h>

#include "dpu_hwio.h"
#include "dpu_hw_catalog.h"
#include "dpu_hw_lm.h"
#include "dpu_hw_dspp.h"
#include "dpu_kms.h"
#include "dpu_hw_reg_dma.h"


/* DSPP_PCC */
#define PCC_EN BIT(0)
#define PCC_DIS 0
#define PCC_RED_R_OFF 0x10
#define PCC_RED_G_OFF 0x1C
#define PCC_RED_B_OFF 0x28
#define PCC_GREEN_R_OFF 0x14
#define PCC_GREEN_G_OFF 0x20
#define PCC_GREEN_B_OFF 0x2C
#define PCC_BLUE_R_OFF 0x18
#define PCC_BLUE_G_OFF 0x24
#define PCC_BLUE_B_OFF 0x30

/* DSPP_IGC v4 / 3D gamut v4.3: LUT data only via the LUTDMA LUTBUS */
#define IGC_OPMODE_OFF		0x4
#define IGC_EN			BIT(8)
#define IGC_TRANS_SIZE		3	/* dwords per LUTBUS transfer */
/* transfer sizes are 12 and 24 bytes: roundup(), ALIGN() needs a power of 2 */
#define IGC_DATA_BYTES		roundup(DPU_IGC_LUT_SIZE * 3 * sizeof(u16), \
					IGC_TRANS_SIZE * sizeof(u32))
#define GAMUT_EN		BIT(0)	/* mode 17 (0 << 2), no scale/offset map */
#define GAMUT_TBL_NUM		4
#define GAMUT_TBL_LEN		1229	/* 4 * 1229 >= 17^3 */
#define GAMUT_TRANS_SIZE	6
#define GAMUT_DATA_BYTES	roundup(GAMUT_TBL_NUM * GAMUT_TBL_LEN * 3 * sizeof(u16), \
					GAMUT_TRANS_SIZE * sizeof(u32))
/* descriptor overhead: decode-select, LUT header, 3 no-ops, op mode write */
#define LUT_DESC_OVERHEAD	(8 + 8 + 12 + 8)

/* DSPP_GC */
#define GC_EN BIT(0)
#define GC_DIS 0
#define GC_8B_ROUND_EN BIT(1)
#define GC_LUT_SWAP_OFF 0x1c
#define GC_C0_OFF 0x4
#define GC_C1_OFF 0xc
#define GC_C2_OFF 0x14
#define GC_C0_INDEX_OFF 0x8
#define GC_C1_INDEX_OFF 0x10
#define GC_C2_INDEX_OFF 0x18

static void dpu_setup_dspp_pcc(struct dpu_hw_dspp *ctx,
		struct dpu_hw_pcc_cfg *cfg)
{

	u32 base;

	if (!ctx) {
		DRM_ERROR("invalid ctx %p\n", ctx);
		return;
	}

	base = ctx->cap->sblk->pcc.base;

	if (!base) {
		DRM_ERROR("invalid ctx %p pcc base 0x%x\n", ctx, base);
		return;
	}

	if (!cfg) {
		DRM_DEBUG_DRIVER("disable pcc feature\n");
		DPU_REG_WRITE(&ctx->hw, base, PCC_DIS);
		return;
	}

	DPU_REG_WRITE(&ctx->hw, base + PCC_RED_R_OFF, cfg->r.r);
	DPU_REG_WRITE(&ctx->hw, base + PCC_RED_G_OFF, cfg->r.g);
	DPU_REG_WRITE(&ctx->hw, base + PCC_RED_B_OFF, cfg->r.b);

	DPU_REG_WRITE(&ctx->hw, base + PCC_GREEN_R_OFF, cfg->g.r);
	DPU_REG_WRITE(&ctx->hw, base + PCC_GREEN_G_OFF, cfg->g.g);
	DPU_REG_WRITE(&ctx->hw, base + PCC_GREEN_B_OFF, cfg->g.b);

	DPU_REG_WRITE(&ctx->hw, base + PCC_BLUE_R_OFF, cfg->b.r);
	DPU_REG_WRITE(&ctx->hw, base + PCC_BLUE_G_OFF, cfg->b.g);
	DPU_REG_WRITE(&ctx->hw, base + PCC_BLUE_B_OFF, cfg->b.b);

	DPU_REG_WRITE(&ctx->hw, base, PCC_EN);
}

static void dpu_setup_dspp_gc(struct dpu_hw_dspp *ctx,
		struct dpu_hw_gc_lut *gc_lut)
{
	int i = 0;
	u32 base, reg;

	if (!ctx) {
		DRM_ERROR("invalid ctx\n");
		return;
	}

	base = ctx->cap->sblk->gc.base;

	if (!base) {
		DRM_ERROR("invalid ctx %p gc base\n", ctx);
		return;
	}

	if (!gc_lut) {
		DRM_DEBUG_DRIVER("disable gc feature\n");
		DPU_REG_WRITE(&ctx->hw, base, GC_DIS);
		return;
	}

	DPU_REG_WRITE(&ctx->hw, base + GC_C0_INDEX_OFF, 0);
	DPU_REG_WRITE(&ctx->hw, base + GC_C1_INDEX_OFF, 0);
	DPU_REG_WRITE(&ctx->hw, base + GC_C2_INDEX_OFF, 0);

	for (i = 0; i < PGC_TBL_LEN; i++) {
		DPU_REG_WRITE(&ctx->hw, base + GC_C0_OFF, gc_lut->c0[i]);
		DPU_REG_WRITE(&ctx->hw, base + GC_C1_OFF, gc_lut->c1[i]);
		DPU_REG_WRITE(&ctx->hw, base + GC_C2_OFF, gc_lut->c2[i]);
	}

	DPU_REG_WRITE(&ctx->hw, base + GC_LUT_SWAP_OFF, BIT(0));

	reg = GC_EN | ((gc_lut->flags & PGC_8B_ROUND) ? GC_8B_ROUND_EN : 0);
	DPU_REG_WRITE(&ctx->hw, base, reg);
}

static int dpu_setup_dspp_lutdma(struct dpu_hw_dspp *ctx,
				 struct dpu_hw_reg_dma *rd,
				 struct dpu_hw_ctl *ctl, u32 dspp_mask,
				 enum dpu_reg_dma_buf_slot slot, u32 lutbus_blk,
				 u32 trans_size, const u16 *data, u32 data_bytes,
				 u32 opmode_off, u32 opmode)
{
	struct dpu_reg_dma_buf *buf;
	int ret;

	buf = dpu_reg_dma_get_buf(rd, slot, ctx->idx - DSPP_0,
				  data_bytes + LUT_DESC_OVERHEAD);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	dpu_reg_dma_buf_reset(buf);
	ret = dpu_reg_dma_decode_sel(buf, dspp_mask);
	if (!ret && data)
		ret = dpu_reg_dma_write_lut(buf, lutbus_blk, 0, trans_size,
					    data, data_bytes);
	if (!ret)
		ret = dpu_reg_dma_write(buf, opmode_off, data ? opmode : 0);
	/* LUT RAM is single-buffered: SB queue when there is one */
	if (!ret)
		ret = dpu_reg_dma_has_sb(rd) ? dpu_reg_dma_queue_sb(rd, ctl, buf) :
					       dpu_reg_dma_queue(rd, ctl, buf);
	return ret;
}

static int dpu_setup_dspp_igc(struct dpu_hw_dspp *ctx,
			      struct dpu_hw_reg_dma *rd, struct dpu_hw_ctl *ctl,
			      u32 dspp_mask, const struct dpu_hw_igc_lut *lut)
{
	u16 *data = NULL;
	int i, ret;

	if (lut) {
		data = kzalloc(IGC_DATA_BYTES, GFP_KERNEL);
		if (!data)
			return -ENOMEM;
		/*
		 * Per entry: G, B, R (measured with writeback; same order as
		 * the VIG IGC). 12-bit values MSB-aligned to 16 bits.
		 */
		for (i = 0; i < DPU_IGC_LUT_SIZE; i++) {
			data[3 * i + 0] = (lut->c[1][i] & 0xfff) << 4;
			data[3 * i + 1] = (lut->c[2][i] & 0xfff) << 4;
			data[3 * i + 2] = (lut->c[0][i] & 0xfff) << 4;
		}
	}

	ret = dpu_setup_dspp_lutdma(ctx, rd, ctl, dspp_mask, DPU_REG_DMA_BUF_IGC,
				    DPU_REG_DMA_LUTBUS_IGC, IGC_TRANS_SIZE,
				    data, IGC_DATA_BYTES,
				    ctx->cap->sblk->igc.base + IGC_OPMODE_OFF,
				    IGC_EN);
	kfree(data);
	return ret;
}

static int dpu_setup_dspp_gamut3d(struct dpu_hw_dspp *ctx,
				  struct dpu_hw_reg_dma *rd,
				  struct dpu_hw_ctl *ctl, u32 dspp_mask,
				  const struct dpu_hw_gamut3d_lut *lut)
{
	u16 *data = NULL;
	int j, t, ret;

	if (lut) {
		data = kvzalloc(GAMUT_DATA_BYTES, GFP_KERNEL);
		if (!data)
			return -ENOMEM;
		/*
		 * Node L = r + 17g + 289b lives in table L & 3 at entry L >> 2;
		 * the stream is entry-major, the 4 tables interleaved.
		 */
		for (j = 0; j < GAMUT_TBL_LEN; j++) {
			for (t = 0; t < GAMUT_TBL_NUM; t++) {
				int node = 4 * j + t;
				u16 *d = &data[3 * (GAMUT_TBL_NUM * j + t)];

				if (node >= DPU_GAMUT3D_SIZE)
					continue;
				d[0] = (lut->rgb[node][0] & 0xfff) << 4;
				d[1] = (lut->rgb[node][1] & 0xfff) << 4;
				d[2] = (lut->rgb[node][2] & 0xfff) << 4;
			}
		}
	}

	ret = dpu_setup_dspp_lutdma(ctx, rd, ctl, dspp_mask, DPU_REG_DMA_BUF_GAMUT,
				    DPU_REG_DMA_LUTBUS_GAMUT, GAMUT_TRANS_SIZE,
				    data, GAMUT_DATA_BYTES,
				    ctx->cap->sblk->gamut3d.base, GAMUT_EN);
	kvfree(data);
	return ret;
}

/**
 * dpu_hw_dspp_init() - Initializes the DSPP hw driver object.
 * should be called once before accessing every DSPP.
 * @dev:  Corresponding device for devres management
 * @cfg:  DSPP catalog entry for which driver object is required
 * @addr: Mapped register io address of MDP
 * Return: pointer to structure or ERR_PTR
 */
struct dpu_hw_dspp *dpu_hw_dspp_init(struct drm_device *dev,
				     const struct dpu_dspp_cfg *cfg,
				     void __iomem *addr)
{
	struct dpu_hw_dspp *c;

	if (!addr)
		return ERR_PTR(-EINVAL);

	c = drmm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	c->hw.blk_addr = addr + cfg->base;
	c->hw.log_mask = DPU_DBG_MASK_DSPP;

	/* Assign ops */
	c->idx = cfg->id;
	c->cap = cfg;
	if (c->cap->sblk->pcc.base)
		c->ops.setup_pcc = dpu_setup_dspp_pcc;
	if (c->cap->sblk->gc.base)
		c->ops.setup_gc = dpu_setup_dspp_gc;
	if (c->cap->sblk->igc.version == 0x40000)
		c->ops.setup_igc = dpu_setup_dspp_igc;
	if (c->cap->sblk->gamut3d.version == 0x40003)
		c->ops.setup_gamut3d = dpu_setup_dspp_gamut3d;

	return c;
}
