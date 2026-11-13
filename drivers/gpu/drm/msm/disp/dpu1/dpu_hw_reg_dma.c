// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 *
 * LUTDMA ("reg-DMA") engine. The engine fetches a buffer of
 * register-write descriptors from memory and applies them, which is the only
 * way to load LUT data into blocks without an MMIO data port (DSPP IGC v4,
 * DSPP 3D gamut v4.3). Register layout follows the downstream SDE reg-DMA v1
 * driver, version 2.0 (= v1 base + v1.1/v1.2/v2 offset deltas).
 *
 * Two queue blocks with the same register layout: DB (queue 0, SW trigger,
 * writes double-buffered registers) and SB (queue 1, triggered by the CTL
 * DSPP_SB flush). LUT RAM is single-buffered, so DSPP IGC and 3D LUT data
 * go through SB, as downstream does: the hardware then writes the RAM at a
 * point of the frame where it is safe to.
 */

#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <drm/drm_print.h>

#include "msm_gem.h"
#include "dpu_hw_catalog.h"
#include "dpu_hw_ctl.h"
#include "dpu_hw_reg_dma.h"

/* engine registers, relative to the DB queue block (v2 values) */
#define REG_DMA_OPMODE			0x004
#define REG_DMA_CTL0_QUEUE0_CMD0	0x014	/* + 0x10 per CTL: iova, +4: cmd */
#define REG_DMA_CTL0_QUEUE1_CMD0	0x01c	/* same, SB block */
#define REG_DMA_INTR_STATUS		0x160
#define REG_DMA_INTR_4_STATUS		0x170	/* error status */
#define REG_DMA_INTR_CLEAR		0x1a0
#define REG_DMA_ERROR_CLEAR_MASK	(BIT(0) | BIT(1) | BIT(2) | BIT(16) | \
					 BIT(17) | BIT(18) | BIT(19))

/* SW trigger for queue 0, a CTL register */
#define CTL_REG_DMA_TRIGGER		0x0d4

/* descriptor opcodes */
#define DECODE_SEL_WORD			0x180ac114	/* abs single write, regdma+0x114 */
#define SINGLE_REG_WRITE_OPCODE		BIT(28)
#define LUTBUS_WRITE_OPCODE		(BIT(30) | BIT(29))
#define NO_OP_OPCODE			0
#define MAX_RELATIVE_OFF		(BIT(21) - 1)
#define LUTBUS_TABLE_SEL_MASK		0x10000
#define LUTBUS_BLOCK_SEL_MASK		0xffff
#define LUTBUS_TRANS_SZ_MASK		0xff0000
#define LUTBUS_LUT_SIZE_MASK		0x3fff

/* queue command word */
#define CMD_LAST_COMMAND		BIT(24)
#define CMD_WRITE			BIT(22)
#define MAX_DWORDS			(BIT(14) - 1)

#define ADDR_ALIGN			256
#define LAST_CMD_BUF_SZ			(sizeof(u32) * 128)

static DEFINE_SPINLOCK(reg_dma_lock);

/* DB queue 0 "trigger done" bits in INTR_STATUS, indexed by idx - CTL_0 */
static const u32 trigger_done_mask[] = {
	BIT(16), BIT(17), BIT(18), BIT(19), BIT(25), BIT(26),
};

/* SB queue 1 "trigger done" bits, in the SB block's INTR_STATUS */
static const u32 sb_done_mask[] = {
	BIT(21), BIT(22), BIT(23), BIT(24), BIT(27), BIT(28),
};

struct dpu_reg_dma_buf *dpu_reg_dma_buf_alloc(struct dpu_hw_reg_dma *rd,
					      u32 size)
{
	struct dpu_reg_dma_buf *buf;
	void *vaddr;
	u64 iova, aligned;

	if (!size || size / sizeof(u32) > MAX_DWORDS)
		return ERR_PTR(-EINVAL);

	buf = kzalloc_obj(*buf);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	vaddr = msm_gem_kernel_new(rd->dev, size + ADDR_ALIGN, MSM_BO_WC,
				   rd->vm, &buf->bo, &iova);
	if (IS_ERR(vaddr)) {
		kfree(buf);
		return ERR_CAST(vaddr);
	}

	/* the queue register only takes 32 address bits */
	aligned = ALIGN(iova, ADDR_ALIGN);
	if (upper_32_bits(aligned + size)) {
		DRM_ERROR("reg_dma: iova %llx above 4G\n", iova);
		msm_gem_kernel_put(buf->bo, rd->vm);
		kfree(buf);
		return ERR_PTR(-ERANGE);
	}

	buf->iova = aligned;
	buf->vaddr = vaddr + (aligned - iova);
	buf->size = size;
	return buf;
}

void dpu_reg_dma_buf_free(struct dpu_hw_reg_dma *rd,
			  struct dpu_reg_dma_buf *buf)
{
	if (IS_ERR_OR_NULL(buf))
		return;
	msm_gem_kernel_put(buf->bo, rd->vm);
	kfree(buf);
}

/**
 * dpu_reg_dma_get_buf() - per-DSPP feature buffer, allocated on first use
 * @rd:   engine
 * @slot: feature
 * @dspp: DSPP_x - DSPP_0
 * @size: minimum size in bytes (only used for the first allocation)
 *
 * The buffer is reused for every update of that feature. Reuse is safe
 * because the engine consumes it right after the trigger at the next
 * kickoff, long before the following commit can rewrite it.
 */
struct dpu_reg_dma_buf *dpu_reg_dma_get_buf(struct dpu_hw_reg_dma *rd,
					    enum dpu_reg_dma_buf_slot slot,
					    unsigned int dspp, u32 size)
{
	struct dpu_reg_dma_buf **pbuf;

	if (!rd || slot >= DPU_REG_DMA_BUF_MAX || dspp >= DPU_REG_DMA_MAX_DSPP)
		return ERR_PTR(-EINVAL);

	pbuf = &rd->bufs[slot][dspp];
	if (!*pbuf) {
		struct dpu_reg_dma_buf *buf = dpu_reg_dma_buf_alloc(rd, size);

		if (IS_ERR(buf))
			return buf;
		*pbuf = buf;
	} else if ((*pbuf)->size < size) {
		return ERR_PTR(-EINVAL);
	}
	return *pbuf;
}

void dpu_reg_dma_buf_reset(struct dpu_reg_dma_buf *buf)
{
	buf->index = 0;
	buf->has_decode_sel = false;
	buf->has_write = false;
}

static u32 *buf_reserve(struct dpu_reg_dma_buf *buf, u32 bytes)
{
	u32 *loc;

	if (buf->size - buf->index < bytes) {
		DRM_ERROR("reg_dma: buffer full (%u/%u, need %u)\n",
			  buf->index, buf->size, bytes);
		return NULL;
	}
	loc = (u32 *)((u8 *)buf->vaddr + buf->index);
	buf->index += bytes;
	return loc;
}

/**
 * dpu_reg_dma_decode_sel() - select the hardware blocks later writes target
 * @buf:      descriptor buffer
 * @blk_mask: DPU_REG_DMA_BLK_* bits
 */
int dpu_reg_dma_decode_sel(struct dpu_reg_dma_buf *buf, u32 blk_mask)
{
	u32 *loc = buf_reserve(buf, 2 * sizeof(u32));

	if (!loc)
		return -ENOSPC;
	loc[0] = DECODE_SEL_WORD;
	loc[1] = blk_mask;
	buf->has_decode_sel = true;
	return 0;
}

/**
 * dpu_reg_dma_write() - single register write
 * @buf: descriptor buffer
 * @off: register offset relative to the decode-selected block
 * @val: value
 */
int dpu_reg_dma_write(struct dpu_reg_dma_buf *buf, u32 off, u32 val)
{
	u32 *loc;

	if (!buf->has_decode_sel || (off & 0x3) || off > MAX_RELATIVE_OFF)
		return -EINVAL;
	loc = buf_reserve(buf, 2 * sizeof(u32));
	if (!loc)
		return -ENOSPC;
	loc[0] = SINGLE_REG_WRITE_OPCODE | off;
	loc[1] = val;
	buf->has_write = true;
	return 0;
}

/**
 * dpu_reg_dma_write_lut() - LUTBUS block write
 * @buf:        descriptor buffer
 * @block_sel:  DPU_REG_DMA_LUTBUS_*
 * @table_sel:  table index (3D gamut only)
 * @trans_size: LUTBUS transfer size in dwords
 * @data:       LUT payload, @len bytes, multiple of trans_size dwords
 * @len:        payload size in bytes
 */
int dpu_reg_dma_write_lut(struct dpu_reg_dma_buf *buf, u32 block_sel,
			  u32 table_sel, u32 trans_size,
			  const void *data, u32 len)
{
	u32 *loc;

	if (!buf->has_decode_sel || !trans_size || !len ||
	    len % (trans_size * sizeof(u32)))
		return -EINVAL;
	loc = buf_reserve(buf, 2 * sizeof(u32) + len + 3 * sizeof(u32));
	if (!loc)
		return -ENOSPC;
	loc[0] = LUTBUS_WRITE_OPCODE |
		 ((table_sel << 16) & LUTBUS_TABLE_SEL_MASK) |
		 (block_sel & LUTBUS_BLOCK_SEL_MASK);
	loc[1] = ((trans_size << 16) & LUTBUS_TRANS_SZ_MASK) |
		 ((len / (trans_size * sizeof(u32))) & LUTBUS_LUT_SIZE_MASK);
	memcpy(&loc[2], data, len);
	/*
	 * HW limitation: residual LUT data plus the next opcode must exceed
	 * four dwords, so pad with three no-ops (as downstream does).
	 */
	loc = (u32 *)((u8 *)&loc[2] + len);
	loc[0] = NO_OP_OPCODE;
	loc[1] = NO_OP_OPCODE;
	loc[2] = NO_OP_OPCODE;
	buf->has_write = true;
	return 0;
}

static int ctl_index(const struct dpu_hw_ctl *ctl)
{
	int i = ctl->idx - CTL_0;

	return (i >= 0 && i < ARRAY_SIZE(trigger_done_mask)) ? i : -EINVAL;
}

/* caller holds reg_dma_lock; @blk is the DB or SB block, @q0 its CTL0 queue */
static void queue_buf(void __iomem *blk, u32 q0, int i,
		      struct dpu_reg_dma_buf *buf, bool last)
{
	u32 cmd = CMD_WRITE | ((buf->index / sizeof(u32)) & MAX_DWORDS);
	u32 qoff = q0 + i * 0x10;
	u32 err;

	if (last)
		cmd |= CMD_LAST_COMMAND;

	writel_relaxed(BIT(0), blk + REG_DMA_OPMODE);
	err = readl_relaxed(blk + REG_DMA_INTR_4_STATUS);
	if (err) {
		DRM_DEBUG_DRIVER("reg_dma: error status %#x, clearing\n", err);
		writel_relaxed(REG_DMA_ERROR_CLEAR_MASK,
			       blk + REG_DMA_INTR_CLEAR + 4 * sizeof(u32));
	}

	/* descriptors (WC memory) visible before the engine is pointed at them */
	wmb();
	writel_relaxed(lower_32_bits(buf->iova), blk + qoff);
	writel(cmd, blk + qoff + 4);
}

/**
 * dpu_reg_dma_queue() - queue a descriptor buffer on a CTL's DB queue
 * @rd:  engine
 * @ctl: CTL whose queue to use
 * @buf: descriptor buffer, with a decode-select and at least one write
 *
 * The packets only take effect after dpu_reg_dma_flush() on the same CTL.
 */
int dpu_reg_dma_queue(struct dpu_hw_reg_dma *rd, struct dpu_hw_ctl *ctl,
		      struct dpu_reg_dma_buf *buf)
{
	unsigned long flags;
	int i = ctl_index(ctl);

	if (i < 0)
		return i;
	if (!buf->has_decode_sel || !buf->has_write || !buf->index)
		return -EINVAL;

	spin_lock_irqsave(&reg_dma_lock, flags);
	queue_buf(rd->base, REG_DMA_CTL0_QUEUE0_CMD0, i, buf, false);
	__set_bit(i, &rd->pending);
	spin_unlock_irqrestore(&reg_dma_lock, flags);
	return 0;
}

/**
 * dpu_reg_dma_flush() - trigger the packets queued on @ctl
 * @rd:   engine
 * @ctl:  CTL whose DB queue to trigger
 * @wait: busy-wait (atomic-safe, 20 ms max) for the trigger-done status
 *
 * No-op when nothing is queued for @ctl, so it is cheap to call on every
 * kickoff. Safe in atomic context.
 */
int dpu_reg_dma_flush(struct dpu_hw_reg_dma *rd, struct dpu_hw_ctl *ctl,
		      bool wait)
{
	unsigned long flags;
	u32 mask, val;
	int i = ctl_index(ctl), ret = 0;

	if (!rd || i < 0)
		return 0;

	spin_lock_irqsave(&reg_dma_lock, flags);
	if (!test_bit(i, &rd->pending))
		goto out;

	dpu_reg_dma_buf_reset(rd->last_cmd);
	dpu_reg_dma_decode_sel(rd->last_cmd, 0);
	rd->last_cmd->has_write = true;

	mask = trigger_done_mask[i];
	queue_buf(rd->base, REG_DMA_CTL0_QUEUE0_CMD0, i, rd->last_cmd, true);
	writel_relaxed(mask, rd->base + REG_DMA_INTR_CLEAR);
	/* last command queued before the trigger */
	wmb();
	DPU_REG_WRITE(&ctl->hw, CTL_REG_DMA_TRIGGER, BIT(0));
	__clear_bit(i, &rd->pending);

	if (wait) {
		ret = readl_poll_timeout_atomic(rd->base + REG_DMA_INTR_STATUS,
						val, val & mask, 10, 20000);
		if (ret)
			DRM_ERROR("reg_dma: ctl %d trigger timeout, status %#x\n",
				  ctl->idx - CTL_0, val);
	}
out:
	spin_unlock_irqrestore(&reg_dma_lock, flags);
	return ret;
}

/**
 * dpu_reg_dma_queue_sb() - queue a descriptor buffer on a CTL's SB queue
 * @rd:  engine
 * @ctl: CTL whose queue to use
 * @buf: descriptor buffer, with a decode-select and at least one write
 *
 * Follow with dpu_reg_dma_last_cmd_sb() once all buffers of the commit are
 * queued; the CTL DSPP_SB flush of the DSPPs they target then runs them.
 */
int dpu_reg_dma_queue_sb(struct dpu_hw_reg_dma *rd, struct dpu_hw_ctl *ctl,
			 struct dpu_reg_dma_buf *buf)
{
	unsigned long flags;
	int i = ctl_index(ctl);

	if (!dpu_reg_dma_has_sb(rd))
		return -EOPNOTSUPP;
	if (i < 0)
		return i;
	if (!buf->has_decode_sel || !buf->has_write || !buf->index)
		return -EINVAL;

	spin_lock_irqsave(&reg_dma_lock, flags);
	queue_buf(rd->sb, REG_DMA_CTL0_QUEUE1_CMD0, i, buf, false);
	spin_unlock_irqrestore(&reg_dma_lock, flags);
	return 0;
}

/**
 * dpu_reg_dma_last_cmd_sb() - close the SB queue of @ctl for this commit
 * @rd:  engine
 * @ctl: CTL whose SB queue to close
 *
 * The packets run on the next CTL flush that carries DPU_DSPP_SB for the
 * DSPPs they target (there is no SW trigger for SB).
 */
int dpu_reg_dma_last_cmd_sb(struct dpu_hw_reg_dma *rd, struct dpu_hw_ctl *ctl)
{
	unsigned long flags;
	int i = ctl_index(ctl);

	if (!dpu_reg_dma_has_sb(rd))
		return -EOPNOTSUPP;
	if (i < 0)
		return i;

	spin_lock_irqsave(&reg_dma_lock, flags);
	dpu_reg_dma_buf_reset(rd->last_cmd_sb);
	dpu_reg_dma_decode_sel(rd->last_cmd_sb, 0);
	rd->last_cmd_sb->has_write = true;
	writel_relaxed(sb_done_mask[i], rd->sb + REG_DMA_INTR_CLEAR);
	queue_buf(rd->sb, REG_DMA_CTL0_QUEUE1_CMD0, i, rd->last_cmd_sb, true);
	spin_unlock_irqrestore(&reg_dma_lock, flags);
	return 0;
}

struct dpu_hw_reg_dma *dpu_hw_reg_dma_init(struct drm_device *dev,
					   struct drm_gpuvm *vm,
					   void __iomem *regdma,
					   const struct dpu_reg_dma_cfg *cfg)
{
	struct dpu_hw_reg_dma *rd;

	if (!regdma || !cfg || !vm)
		return ERR_PTR(-ENODEV);

	rd = kzalloc_obj(*rd);
	if (!rd)
		return ERR_PTR(-ENOMEM);

	rd->base = regdma + cfg->base;
	rd->caps = cfg;
	rd->dev = dev;
	rd->vm = vm;

	rd->last_cmd = dpu_reg_dma_buf_alloc(rd, LAST_CMD_BUF_SZ);
	if (IS_ERR(rd->last_cmd)) {
		void *err = rd->last_cmd;

		kfree(rd);
		return err;
	}

	if (cfg->sb_base) {
		rd->last_cmd_sb = dpu_reg_dma_buf_alloc(rd, LAST_CMD_BUF_SZ);
		if (IS_ERR(rd->last_cmd_sb)) {
			void *err = rd->last_cmd_sb;

			dpu_reg_dma_buf_free(rd, rd->last_cmd);
			kfree(rd);
			return err;
		}
		rd->sb = regdma + cfg->sb_base;
	}

	DRM_DEBUG_DRIVER("reg_dma: v%#x ready, last_cmd iova %#llx\n",
			 cfg->version, rd->last_cmd->iova);
	return rd;
}

void dpu_hw_reg_dma_destroy(struct dpu_hw_reg_dma *rd)
{
	int s, d;

	if (IS_ERR_OR_NULL(rd))
		return;
	for (s = 0; s < DPU_REG_DMA_BUF_MAX; s++)
		for (d = 0; d < DPU_REG_DMA_MAX_DSPP; d++)
			dpu_reg_dma_buf_free(rd, rd->bufs[s][d]);
	dpu_reg_dma_buf_free(rd, rd->last_cmd);
	dpu_reg_dma_buf_free(rd, rd->last_cmd_sb);
	kfree(rd);
}
