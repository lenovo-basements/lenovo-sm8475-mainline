/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#ifndef _DPU_HW_REG_DMA_H
#define _DPU_HW_REG_DMA_H

#include <linux/types.h>

struct drm_device;
struct drm_gem_object;
struct drm_gpuvm;
struct dpu_hw_ctl;
struct dpu_reg_dma_cfg;

/* decode-select block bits */
#define DPU_REG_DMA_BLK_DSPP(n)		BIT(17 + (n))	/* n = DSPP_x - DSPP_0 */
#define DPU_REG_DMA_BLK_DSPP_IGC	BIT(21)

/* per-DSPP descriptor buffer slots owned by the engine */
enum dpu_reg_dma_buf_slot {
	DPU_REG_DMA_BUF_IGC,
	DPU_REG_DMA_BUF_GAMUT,
	DPU_REG_DMA_BUF_MAX
};

#define DPU_REG_DMA_MAX_DSPP		4

/* LUTBUS block selectors */
#define DPU_REG_DMA_LUTBUS_IGC		0
#define DPU_REG_DMA_LUTBUS_GAMUT	1

/**
 * struct dpu_reg_dma_buf - descriptor buffer the engine reads from memory
 * @bo:     backing GEM object
 * @iova:   device address (aligned to 256 bytes)
 * @vaddr:  CPU address of @iova
 * @size:   usable size in bytes
 * @index:  bytes written so far
 * @has_decode_sel: a decode-select descriptor was written
 * @has_write: at least one register/LUT write was written
 */
struct dpu_reg_dma_buf {
	struct drm_gem_object *bo;
	u64 iova;
	u32 *vaddr;
	u32 size;
	u32 index;
	bool has_decode_sel;
	bool has_write;
};

/**
 * struct dpu_hw_reg_dma - LUTDMA engine
 * @base:     mapped "regdma" region + DB queue offset
 * @sb:       mapped "regdma" region + SB queue offset, NULL without one
 * @caps:     catalog entry
 * @dev:      drm device for GEM allocations
 * @vm:       KMS address space the engine fetches through
 * @last_cmd: small buffer holding the DB "last command" descriptor
 * @last_cmd_sb: same for the SB queue
 * @pending:  bit per CTL (idx - CTL_0) with queued, untriggered packets
 * @bufs:     lazily allocated per-DSPP feature buffers, freed with the engine
 */
struct dpu_hw_reg_dma {
	void __iomem *base;
	void __iomem *sb;
	const struct dpu_reg_dma_cfg *caps;
	struct drm_device *dev;
	struct drm_gpuvm *vm;
	struct dpu_reg_dma_buf *last_cmd;
	struct dpu_reg_dma_buf *last_cmd_sb;
	unsigned long pending;
	struct dpu_reg_dma_buf *bufs[DPU_REG_DMA_BUF_MAX][DPU_REG_DMA_MAX_DSPP];
};

struct dpu_hw_reg_dma *dpu_hw_reg_dma_init(struct drm_device *dev,
					   struct drm_gpuvm *vm,
					   void __iomem *regdma,
					   const struct dpu_reg_dma_cfg *cfg);
void dpu_hw_reg_dma_destroy(struct dpu_hw_reg_dma *rd);

struct dpu_reg_dma_buf *dpu_reg_dma_buf_alloc(struct dpu_hw_reg_dma *rd,
					      u32 size);
void dpu_reg_dma_buf_free(struct dpu_hw_reg_dma *rd,
			  struct dpu_reg_dma_buf *buf);
void dpu_reg_dma_buf_reset(struct dpu_reg_dma_buf *buf);
struct dpu_reg_dma_buf *dpu_reg_dma_get_buf(struct dpu_hw_reg_dma *rd,
					    enum dpu_reg_dma_buf_slot slot,
					    unsigned int dspp, u32 size);

int dpu_reg_dma_decode_sel(struct dpu_reg_dma_buf *buf, u32 blk_mask);
int dpu_reg_dma_write(struct dpu_reg_dma_buf *buf, u32 off, u32 val);
int dpu_reg_dma_write_lut(struct dpu_reg_dma_buf *buf, u32 block_sel,
			  u32 table_sel, u32 trans_size,
			  const void *data, u32 len);

int dpu_reg_dma_queue(struct dpu_hw_reg_dma *rd, struct dpu_hw_ctl *ctl,
		      struct dpu_reg_dma_buf *buf);
int dpu_reg_dma_flush(struct dpu_hw_reg_dma *rd, struct dpu_hw_ctl *ctl,
		      bool wait);

int dpu_reg_dma_queue_sb(struct dpu_hw_reg_dma *rd, struct dpu_hw_ctl *ctl,
			 struct dpu_reg_dma_buf *buf);
int dpu_reg_dma_last_cmd_sb(struct dpu_hw_reg_dma *rd, struct dpu_hw_ctl *ctl);

static inline bool dpu_reg_dma_has_sb(const struct dpu_hw_reg_dma *rd)
{
	return rd && rd->sb;
}

#endif /* _DPU_HW_REG_DMA_H */
