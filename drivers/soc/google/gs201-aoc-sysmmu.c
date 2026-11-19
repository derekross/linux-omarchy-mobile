// SPDX-License-Identifier: GPL-2.0-only
/*
 * Static translation table for the gs201 and zuma AoC System MMU
 *
 * The AoC (audio and sensor DSP) reaches DRAM and a few AP peripherals
 * through its own Samsung System MMU (v8, "ASOC", at 0x1a090000; only the
 * AoC uses it). The AoC firmware addresses them at fixed AoC addresses and
 * carries the map in its image header, as 1 MiB-granular windows, e.g. its
 * 48 MiB carve-out at 0x94000000 seen at 0x98000000. This driver installs
 * those windows as first-level sections and nothing else; any other access
 * faults and is logged with its VID.
 *
 * The block sits in BLK_AOC, whose registers abort while it is powered
 * down, so nothing here touches a register on its own: the AoC driver calls
 * aoc_sysmmu_dump()/aoc_sysmmu_enable() once BLK_AOC is on.
 *
 * zuma (Tensor G3) has a version 9 block instead (sysmmu_aoc at 0x19090000),
 * with a different register map: it is programmed through the v9 functions
 * below, and windows that are not whole 1 MiB sections get 4 KiB pages.
 *
 * Register map, table format and enable sequence follow Google's driver,
 * google-modules/soc/gs/drivers/iommu/samsung-iommu.[ch] (sysmmu_reg_set,
 * __sysmmu_enable(), __sysmmu_enable_vid(), the v8 fault handler).
 *
 * The AoC's traffic arrives on VID 0. "vids" can translate others too (only
 * the AoC is behind this System MMU, so that is harmless); fault lines name
 * the VID. Faults stall the access until the interrupt handler clears them,
 * as Google's driver configures this block. QoS and the TLB setup are the
 * stock device tree's (sysmmu_asoc: qos 15, default_tlb, tlb_property).
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 */

#include <linux/bits.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/soc/google/gs201-aoc-sysmmu.h>

#define REG_MMU_CTRL			0x0000
#define REG_MMU_CFG			0x0004
#define REG_MMU_STATUS			0x0008
#define REG_MMU_VERSION			0x0034
#define REG_INT_STATUS			0x0060
#define REG_INT_CLEAR			0x0064
#define REG_MMU_CAPA0			0x0870
#define REG_MMU_CAPA1			0x0874

/* Without VM support (sysmmu_reg_set[REG_IDX_DEFAULT]) */
#define REG_FLPT_BASE			0x000c
#define REG_ALL_INV			0x0010
#define REG_FAULT_VA			0x0070
#define REG_FAULT_INFO			0x0078

/* With VM support (sysmmu_reg_set[REG_IDX_VM]) */
#define REG_CTRL_VM(n)			(0x8000 + (n) * 0x1000)
#define REG_CFG_VM(n)			(0x8004 + (n) * 0x1000)
#define REG_FLPT_BASE_VM(n)		(0x800c + (n) * 0x1000)
#define REG_ALL_INV_VM(n)		(0x8010 + (n) * 0x1000)
#define REG_FAULT_VA_VM(n)		(0x1000 + (n) * 0x10)
#define REG_FAULT_INFO_VM(n)		(0x1004 + (n) * 0x10)

#define CTRL_MMU_ENABLE			BIT(0)
#define CTRL_MMU_BLOCK			BIT(1)
#define CTRL_FAULT_STALL_MODE		BIT(3)
#define CTRL_VID_ENABLE			BIT(0)
#define CAPA1_VCR_ENABLED		BIT(14)

#define MMU_RAW_VER(reg)		(((reg) >> 17) & 0x7fff)
#define MMU_MAJ_VER(val)		((val) >> 11)
#define MMU_MIN_VER(val)		(((val) >> 4) & 0x7f)

#define NUM_LV1ENTRIES			SZ_64K
#define LV1_TABLE_SIZE			(NUM_LV1ENTRIES * sizeof(u32))
#define SECT_ORDER			20
#define SECT_SIZE			BIT(SECT_ORDER)
#define PG_ENT_SHIFT			4
#define LV1_SECT			2
#define PT_BASE_SHIFT			12
#define MAX_VIDS			8

#define CFG_QOS_OVRRIDE			BIT(11)
#define CFG_QOS(n)			(((n) & 0xfU) << 7)
#define CAPA1_NUM_TLB(reg)		(((reg) >> 4) & 0xff)
#define REG_TLB_CFG(n)			(0x2000 + (n) * 0x20 + 0x4)
#define REG_TLB_MATCH_CFG(n)		(0x2000 + (n) * 0x20 + 0x8)
#define REG_TLB_MATCH_ID(n)		(0x2000 + (n) * 0x20 + 0x14)
#define TLB_CFG_MASK(reg)		((reg) & (GENMASK(7, 5) | GENMASK(3, 2) | GENMASK(1, 1)))
#define TLB_MATCH_CFG_MASK(reg)		((reg) & (GENMASK(31, 16) | GENMASK(9, 8)))

/* Stock sysmmu_asoc: qos, sysmmu,default_tlb, sysmmu,tlb_property */
#define ASOC_QOS			15
#define ASOC_DEFAULT_TLB		0x0a
static const struct {
	u32 index, cfg, match_cfg, match_id;
} asoc_tlb[] = {
	{ 1, 0x0a, 0x300, 0x300010 },
	{ 2, 0x0a, 0x300, 0x3c0008 },
	{ 3, 0x0a, 0x300, 0x3c000c },
	{ 4, 0x0a, 0x300, 0x3c0000 },
};

static unsigned int vids = BIT(0);
module_param(vids, uint, 0444);
MODULE_PARM_DESC(vids, "Bitmask of VIDs to translate (default: VID 0, which the AoC uses)");

static bool dump_faults;
module_param(dump_faults, bool, 0644);
MODULE_PARM_DESC(dump_faults, "Include VID 0's fault registers in register dumps (not yet read on this block)");

static bool empty;
module_param(empty, bool, 0444);
MODULE_PARM_DESC(empty, "Negative test: install an empty table, so every AoC access faults");

struct aoc_sysmmu_data {
	bool v9;	/* zuma's version 9 block, with its own register map */
};

struct aoc_sysmmu {
	struct device *dev;
	void __iomem *base;
	int irq;
	u32 *table;
	dma_addr_t table_dma;
	u32 vid_mask;
	bool vm;
	bool irq_on;
	bool enabled;
	bool v9;
	u32 *l2[8];		/* v9: second-level tables (4 KiB pages) */
	dma_addr_t l2_dma[8];
	unsigned int n_l2;
	u32 saved_ctrl;
	u32 saved_ctrl_vm[MAX_VIDS];
};

void aoc_sysmmu_dump(struct aoc_sysmmu *mmu, const char *when)
{
	u32 ver, capa1;
	unsigned int v;

	/*
	 * zuma's block is a v9 System MMU: CAPA0/CAPA1, CFG and the interrupt
	 * registers of the gs201 (v8) map are not in it, and reading them
	 * reset the phone (2026-11-04). Only CTRL, STATUS and VERSION, which
	 * Google's samsung-iommu-v9.c also reads, are touched, one per line.
	 */
	if (mmu->v9) {
		dev_info(mmu->dev, "%s: v9 read, version register\n", when);
		ver = readl(mmu->base + REG_MMU_VERSION);
		dev_info(mmu->dev, "%s: version %#x\n", when, ver);
		dev_info(mmu->dev, "%s: ctrl %#x\n", when,
			 readl(mmu->base + REG_MMU_CTRL));
		dev_info(mmu->dev, "%s: status %#x\n", when,
			 readl(mmu->base + REG_MMU_STATUS));
		return;
	}

	ver = readl(mmu->base + REG_MMU_VERSION);
	capa1 = readl(mmu->base + REG_MMU_CAPA1);

	mmu->vm = capa1 & CAPA1_VCR_ENABLED;
	dev_info(mmu->dev,
		 "%s: version %#x (v%u.%u) ctrl %#x cfg %#x status %#x int %#x capa0 %#x capa1 %#x (%s)\n",
		 when, ver, MMU_MAJ_VER(MMU_RAW_VER(ver)),
		 MMU_MIN_VER(MMU_RAW_VER(ver)), readl(mmu->base + REG_MMU_CTRL),
		 readl(mmu->base + REG_MMU_CFG), readl(mmu->base + REG_MMU_STATUS),
		 readl(mmu->base + REG_INT_STATUS),
		 readl(mmu->base + REG_MMU_CAPA0), capa1,
		 mmu->vm ? "8 VIDs" : "no VM");

	if (!mmu->vm) {
		dev_info(mmu->dev, "%s: table at %#llx\n", when,
			 (u64)readl(mmu->base + REG_FLPT_BASE) << PT_BASE_SHIFT);
		return;
	}
	for (v = 0; v < MAX_VIDS; v++)
		dev_info(mmu->dev, "%s: VID %u ctrl %#x cfg %#x table at %#llx\n",
			 when, v, readl(mmu->base + REG_CTRL_VM(v)),
			 readl(mmu->base + REG_CFG_VM(v)),
			 (u64)readl(mmu->base + REG_FLPT_BASE_VM(v)) << PT_BASE_SHIFT);
	if (dump_faults)
		dev_info(mmu->dev, "%s: VID 0 fault va %#x info %#x\n", when,
			 readl(mmu->base + REG_FAULT_VA_VM(0)),
			 readl(mmu->base + REG_FAULT_INFO_VM(0)));
}
EXPORT_SYMBOL_GPL(aoc_sysmmu_dump);

static irqreturn_t aoc_sysmmu_irq(int irq, void *data)
{
	struct aoc_sysmmu *mmu = data;
	u32 status, info;
	unsigned int v;
	u64 addr;

	status = readl(mmu->base + REG_INT_STATUS);
	if (!status)
		return IRQ_NONE;

	/* Four interrupt bits per VID: PTW access, page fault, multi-hit, access */
	v = __ffs(status) / 4;
	if (mmu->vm) {
		addr = readl(mmu->base + REG_FAULT_VA_VM(v));
		info = readl(mmu->base + REG_FAULT_INFO_VM(v));
	} else {
		addr = readl(mmu->base + REG_FAULT_VA);
		info = readl(mmu->base + REG_FAULT_INFO);
	}
	addr |= (u64)((info >> 28) & 0xf) << 32;

	dev_err_ratelimited(mmu->dev,
			    "blocked %s at %#llx (int %#x, VID %u, type %u, AxID %#x)\n",
			    (info & BIT(20)) ? "WRITE" : "READ", addr, status,
			    v, __ffs(status) % 4, info & 0xffff);

	writel(status, mmu->base + REG_INT_CLEAR);
	return IRQ_HANDLED;
}

/*
 * zuma's AoC System MMU is Samsung's v9 block (Google: samsung-iommu-v9.c,
 * zuma-sysmmu.dtsi sysmmu_aoc). Only VID 0 is used (vmid_mask 1), page table
 * entries get the AP read and write bits, and the streams are the ones of
 * the stock device tree: pmmu0, the default stream and 29 SID matches.
 */
#define V9_REG_CTRL_VM			0x8000
#define V9_REG_ALL_INV_VM		0x8010
#define V9_REG_FLPT_BASE_VM		0x8404
#define V9_REG_ATTR_VM			0x8408
#define V9_REG_PMMU_INDICATOR		0x2ffc
#define V9_REG_STREAM_CFG(n)		(0x2000 + (n) * 0x10)
#define V9_REG_STREAM_MATCH_CFG(n)	(0x2004 + (n) * 0x10)
#define V9_REG_STREAM_SID_VALUE(n)	(0x2008 + (n) * 0x10)
#define V9_REG_STREAM_SID_MASK(n)	(0x200c + (n) * 0x10)
#define V9_CTRL_ENABLE			BIT(0)
#define V9_ATTR_USE_AP			BIT(2)
#define V9_ATTR_QOS_OVERRIDE		BIT(11)
#define V9_ATTR_QOS(n)			(((n) & 0xf) << 7)
#define V9_ATTR_PT_CACHE_MASK		GENMASK(19, 16)
#define V9_ATTR_PT_CACHE_NORMAL_NC	(0x2 << 16)
#define V9_PTE_AP_READ			BIT(4)
#define V9_PTE_AP_WRITE			BIT(5)
/* 0x10b: S2_EN, BL1, PREF_PRED, STLB_PREF and PTLB_PREF */
#define V9_STREAM_CFG(stlb)		(((stlb) << 24) | 0x10b)
#define V9_MATCH_DIR_RW			0x301
#define V9_QOS				15

struct v9_stream {
	u8 index;
	u8 stlb;
	u8 sid;
};

static const struct v9_stream v9_streams[] = {
	{ 1, 1, 0x02 }, { 2, 0, 0x03 }, { 3, 0, 0x04 }, { 4, 0, 0x05 },
	{ 5, 0, 0x06 }, { 6, 0, 0x09 }, { 7, 0, 0x0a }, { 8, 0, 0x0b },
	{ 9, 0, 0x0c }, { 10, 0, 0x0d }, { 11, 0, 0x0e }, { 12, 0, 0x20 },
	{ 13, 0, 0x21 }, { 14, 0, 0x22 }, { 15, 0, 0x23 }, { 16, 0, 0x24 },
	{ 17, 0, 0x25 }, { 18, 0, 0x26 }, { 19, 0, 0x27 }, { 20, 0, 0x28 },
	{ 21, 0, 0x30 }, { 22, 0, 0x31 }, { 23, 0, 0x32 }, { 24, 0, 0x33 },
	{ 25, 0, 0x34 }, { 26, 0, 0x35 }, { 27, 0, 0x36 }, { 28, 0, 0x37 },
	{ 29, 0, 0x38 },
};

static void aoc_sysmmu_program_v9(struct aoc_sysmmu *mmu)
{
	unsigned int i;
	u32 attr;

	writel(mmu->table_dma >> PT_BASE_SHIFT, mmu->base + V9_REG_FLPT_BASE_VM);
	attr = readl(mmu->base + V9_REG_ATTR_VM);
	attr &= ~V9_ATTR_PT_CACHE_MASK;
	attr |= V9_ATTR_PT_CACHE_NORMAL_NC | V9_ATTR_USE_AP;
	attr &= ~V9_ATTR_QOS(0xf);
	attr |= V9_ATTR_QOS_OVERRIDE | V9_ATTR_QOS(V9_QOS);
	writel(attr, mmu->base + V9_REG_ATTR_VM);

	writel(0, mmu->base + V9_REG_PMMU_INDICATOR);
	readl(mmu->base + V9_REG_PMMU_INDICATOR);
	writel(V9_STREAM_CFG(0), mmu->base + V9_REG_STREAM_CFG(0));
	for (i = 0; i < ARRAY_SIZE(v9_streams); i++) {
		const struct v9_stream *st = &v9_streams[i];

		writel(V9_STREAM_CFG(st->stlb), mmu->base + V9_REG_STREAM_CFG(st->index));
		writel(V9_MATCH_DIR_RW, mmu->base + V9_REG_STREAM_MATCH_CFG(st->index));
		writel(st->sid, mmu->base + V9_REG_STREAM_SID_VALUE(st->index));
		writel(0x3f, mmu->base + V9_REG_STREAM_SID_MASK(st->index));
	}
	writel(readl(mmu->base + V9_REG_CTRL_VM) | V9_CTRL_ENABLE,
	       mmu->base + V9_REG_CTRL_VM);
	writel(1, mmu->base + V9_REG_ALL_INV_VM);
}

#define V9_SLPD_FLAG		1
#define V9_SPAGE_FLAG		2
#define V9_SLPD_AP_READ		BIT(2)
#define V9_SLPD_AP_WRITE	BIT(3)

/* v9: unmap and free the second-level tables */
static void aoc_sysmmu_free_l2(struct aoc_sysmmu *mmu)
{
	unsigned int i;

	for (i = 0; i < mmu->n_l2; i++) {
		dma_unmap_single(mmu->dev, mmu->l2_dma[i], PAGE_SIZE, DMA_TO_DEVICE);
		free_page((unsigned long)mmu->l2[i]);
	}
	mmu->n_l2 = 0;
}

/* v9: map @w with 4 KiB pages in second-level tables (256 entries per 1 MiB) */
static int aoc_sysmmu_fill_small(struct aoc_sysmmu *mmu,
				 const struct aoc_sysmmu_window *w, unsigned int idx)
{
	u32 off;

	for (off = 0; off < w->size; off += SZ_4K) {
		u32 iova = w->iova + off;
		u32 *l1 = &mmu->table[iova >> SECT_ORDER];
		u32 *l2;

		if (!*l1) {
			dma_addr_t dma;

			if (mmu->n_l2 >= ARRAY_SIZE(mmu->l2))
				return -ENOSPC;
			l2 = (u32 *)get_zeroed_page(GFP_KERNEL);
			if (!l2)
				return -ENOMEM;
			dma = dma_map_single(mmu->dev, l2, PAGE_SIZE, DMA_TO_DEVICE);
			if (dma_mapping_error(mmu->dev, dma) || dma != virt_to_phys(l2)) {
				if (!dma_mapping_error(mmu->dev, dma))
					dma_unmap_single(mmu->dev, dma, PAGE_SIZE, DMA_TO_DEVICE);
				free_page((unsigned long)l2);
				return -EINVAL;
			}
			mmu->l2[mmu->n_l2] = l2;
			mmu->l2_dma[mmu->n_l2] = dma;
			mmu->n_l2++;
			*l1 = (u32)(dma >> PG_ENT_SHIFT) | V9_SLPD_FLAG;
		} else if ((*l1 & 7) != V9_SLPD_FLAG) {
			dev_err(mmu->dev, "window %u overlaps a section at %#x\n", idx, iova);
			return -EINVAL;
		}
		l2 = phys_to_virt((phys_addr_t)(*l1 & ~0x3fU) << PG_ENT_SHIFT);
		l2 += (iova & (SECT_SIZE - 1)) >> 12;
		if (*l2) {
			dev_err(mmu->dev, "window %u overlaps at %#x\n", idx, iova);
			return -EINVAL;
		}
		*l2 = (u32)((w->pa + off) >> PG_ENT_SHIFT) | V9_SPAGE_FLAG |
		      V9_SLPD_AP_READ | V9_SLPD_AP_WRITE;
	}
	dev_info(mmu->dev, "window %u: %#010x -> %pa, %u KiB in 4 KiB pages\n",
		 idx, w->iova, &w->pa, w->size >> 10);
	return 0;
}

static int aoc_sysmmu_fill(struct aoc_sysmmu *mmu,
			   const struct aoc_sysmmu_window *win, unsigned int n)
{
	unsigned int i;
	u32 off;

	aoc_sysmmu_free_l2(mmu);
	memset(mmu->table, 0, LV1_TABLE_SIZE);
	if (empty) {
		dev_info(mmu->dev, "empty table: every AoC access will fault\n");
		return 0;
	}

	for (i = 0; i < n; i++) {
		const struct aoc_sysmmu_window *w = &win[i];

		/* v9: a window that is not 1 MiB sections is mapped with 4 KiB pages */
		if (mmu->v9 && (!IS_ALIGNED(w->iova, SECT_SIZE) ||
				!IS_ALIGNED(w->pa, SECT_SIZE) ||
				!IS_ALIGNED(w->size, SECT_SIZE)) && w->size &&
		    IS_ALIGNED(w->iova, SZ_4K) && IS_ALIGNED(w->pa, SZ_4K) &&
		    IS_ALIGNED(w->size, SZ_4K)) {
			int ret = aoc_sysmmu_fill_small(mmu, w, i);

			if (ret)
				return ret;
			continue;
		}
		if (!IS_ALIGNED(w->iova, SECT_SIZE) ||
		    !IS_ALIGNED(w->pa, SECT_SIZE) ||
		    !IS_ALIGNED(w->size, SECT_SIZE) || !w->size ||
		    w->pa + w->size > BIT_ULL(36) ||
		    (u64)w->iova + w->size > BIT_ULL(32)) {
			dev_err(mmu->dev, "window %u (%#x -> %pa, %#x) is not 1 MiB sections\n",
				i, w->iova, &w->pa, w->size);
			return -EINVAL;
		}
		for (off = 0; off < w->size; off += SECT_SIZE) {
			u32 *ent = &mmu->table[(w->iova + off) >> SECT_ORDER];

			if (*ent) {
				dev_err(mmu->dev, "window %u overlaps at %#x\n",
					i, w->iova + off);
				return -EINVAL;
			}
			*ent = (u32)((w->pa + off) >> PG_ENT_SHIFT) | LV1_SECT |
			       (mmu->v9 ? (V9_PTE_AP_READ | V9_PTE_AP_WRITE) : 0);
		}
		dev_info(mmu->dev, "window %u: %#010x -> %pa, %u MiB\n",
			 i, w->iova, &w->pa, w->size >> SECT_ORDER);
	}
	for (i = 0; i < mmu->n_l2; i++)
		dma_sync_single_for_device(mmu->dev, mmu->l2_dma[i], PAGE_SIZE, DMA_TO_DEVICE);
	return 0;
}

/* Write the configuration and the table base; the table is built. */
static void aoc_sysmmu_program(struct aoc_sysmmu *mmu)
{
	unsigned int v, ntlb;
	u32 cfg, ctrl;

	if (mmu->v9) {
		aoc_sysmmu_program_v9(mmu);
		return;
	}

	/*
	 * As __sysmmu_enable(): block, QoS and TLBs (__sysmmu_init_config()),
	 * then per VID table base, flush, enable with stall on fault, then
	 * enable.
	 */
	ctrl = readl(mmu->base + REG_MMU_CTRL);
	writel(ctrl | CTRL_MMU_BLOCK, mmu->base + REG_MMU_CTRL);
	cfg = readl(mmu->base + REG_MMU_CFG);
	cfg &= ~CFG_QOS(0xf);
	cfg |= CFG_QOS_OVRRIDE | CFG_QOS(ASOC_QOS);
	ntlb = CAPA1_NUM_TLB(readl(mmu->base + REG_MMU_CAPA1));
	writel(TLB_CFG_MASK(ASOC_DEFAULT_TLB), mmu->base + REG_TLB_CFG(0));
	for (v = 0; v < ARRAY_SIZE(asoc_tlb); v++) {
		u32 i = asoc_tlb[v].index;

		if (i >= ntlb)
			continue;
		writel(TLB_CFG_MASK(asoc_tlb[v].cfg), mmu->base + REG_TLB_CFG(i));
		writel(TLB_MATCH_CFG_MASK(asoc_tlb[v].match_cfg),
		       mmu->base + REG_TLB_MATCH_CFG(i));
		writel(asoc_tlb[v].match_id, mmu->base + REG_TLB_MATCH_ID(i));
	}
	writel(cfg, mmu->base + REG_MMU_CFG);

	if (mmu->vm) {
		for (v = 0; v < MAX_VIDS; v++) {
			u32 vctrl;

			if (!(BIT(v) & mmu->vid_mask))
				continue;
			writel(mmu->table_dma >> PT_BASE_SHIFT,
			       mmu->base + REG_FLPT_BASE_VM(v));
			writel(1, mmu->base + REG_ALL_INV_VM(v));
			vctrl = readl(mmu->base + REG_CTRL_VM(v));
			writel(vctrl | CTRL_FAULT_STALL_MODE | CTRL_VID_ENABLE,
			       mmu->base + REG_CTRL_VM(v));
		}
	} else {
		writel(mmu->table_dma >> PT_BASE_SHIFT, mmu->base + REG_FLPT_BASE);
		writel(1, mmu->base + REG_ALL_INV);
	}
	writel((ctrl & ~CTRL_MMU_BLOCK) | CTRL_MMU_ENABLE, mmu->base + REG_MMU_CTRL);
}

/**
 * aoc_sysmmu_lost() - has a block reset cleared the configuration?
 *
 * BLK_AOC must be on. True if translation is enabled in software but the
 * registers no longer say so (global enable, VID enable or table base).
 */
bool aoc_sysmmu_lost(struct aoc_sysmmu *mmu)
{
	if (!mmu->enabled)
		return false;
	if (mmu->v9)
		return !(readl(mmu->base + V9_REG_CTRL_VM) & V9_CTRL_ENABLE) ||
		       readl(mmu->base + V9_REG_FLPT_BASE_VM) !=
		       (u32)(mmu->table_dma >> PT_BASE_SHIFT);
	if (!(readl(mmu->base + REG_MMU_CTRL) & CTRL_MMU_ENABLE))
		return true;
	if (!mmu->vm)
		return readl(mmu->base + REG_FLPT_BASE) !=
		       (u32)(mmu->table_dma >> PT_BASE_SHIFT);
	return !(readl(mmu->base + REG_CTRL_VM(0)) & CTRL_VID_ENABLE) ||
	       readl(mmu->base + REG_FLPT_BASE_VM(0)) !=
	       (u32)(mmu->table_dma >> PT_BASE_SHIFT);
}
EXPORT_SYMBOL_GPL(aoc_sysmmu_lost);

/* Program the registers again after a block reset. BLK_AOC must be on. */
void aoc_sysmmu_reprogram(struct aoc_sysmmu *mmu)
{
	if (!mmu->enabled)
		return;
	aoc_sysmmu_program(mmu);
	aoc_sysmmu_dump(mmu, "reprogrammed");
}
EXPORT_SYMBOL_GPL(aoc_sysmmu_reprogram);

/**
 * aoc_sysmmu_enable() - install @n windows and turn translation on
 *
 * BLK_AOC must be on. Returns 0 or a negative error (nothing enabled).
 */
int aoc_sysmmu_enable(struct aoc_sysmmu *mmu,
		      const struct aoc_sysmmu_window *win, unsigned int n)
{
	unsigned int v;
	int ret;

	if (mmu->enabled)
		return -EBUSY;

	aoc_sysmmu_dump(mmu, "before");
	mmu->vid_mask = mmu->vm ? (vids & 0xff) : BIT(0);
	if (!mmu->vid_mask)
		return -EINVAL;

	if (!mmu->table) {
		/* 256 KiB, naturally aligned (Google: "FLPD must be aligned in 256KB") */
		mmu->table = (u32 *)__get_free_pages(GFP_KERNEL,
						     get_order(LV1_TABLE_SIZE));
		if (!mmu->table)
			return -ENOMEM;
	}
	ret = aoc_sysmmu_fill(mmu, win, n);
	if (ret)
		return ret;

	/* The table walker is not coherent: map it for the device (cleans) */
	mmu->table_dma = dma_map_single(mmu->dev, mmu->table, LV1_TABLE_SIZE,
					DMA_TO_DEVICE);
	if (dma_mapping_error(mmu->dev, mmu->table_dma))
		return -ENOMEM;
	if (mmu->table_dma != virt_to_phys(mmu->table)) {
		dev_err(mmu->dev, "table bounced to %pad\n", &mmu->table_dma);
		dma_unmap_single(mmu->dev, mmu->table_dma, LV1_TABLE_SIZE,
				 DMA_TO_DEVICE);
		return -EINVAL;
	}

	/* v9: the fault registers are not mapped yet, so no interrupt handler */
	if (!mmu->v9 && !mmu->irq_on) {
		ret = request_irq(mmu->irq, aoc_sysmmu_irq, 0, dev_name(mmu->dev),
				  mmu);
		if (ret) {
			dma_unmap_single(mmu->dev, mmu->table_dma, LV1_TABLE_SIZE,
					 DMA_TO_DEVICE);
			return ret;
		}
		mmu->irq_on = true;
	}

	mmu->saved_ctrl = readl(mmu->base + REG_MMU_CTRL);
	if (mmu->vm)
		for (v = 0; v < MAX_VIDS; v++)
			mmu->saved_ctrl_vm[v] = readl(mmu->base + REG_CTRL_VM(v));
	aoc_sysmmu_program(mmu);
	mmu->enabled = true;

	aoc_sysmmu_dump(mmu, "enabled");
	dev_info(mmu->dev, "translating VIDs %#x, table at %pad\n",
		 mmu->vid_mask, &mmu->table_dma);
	return 0;
}
EXPORT_SYMBOL_GPL(aoc_sysmmu_enable);

/* Restore the registers as found. BLK_AOC must be on. */
void aoc_sysmmu_disable(struct aoc_sysmmu *mmu)
{
	unsigned int v;

	if (!mmu->enabled)
		return;
	if (mmu->v9) {
		writel(readl(mmu->base + V9_REG_CTRL_VM) & ~V9_CTRL_ENABLE,
		       mmu->base + V9_REG_CTRL_VM);
		writel(0, mmu->base + V9_REG_FLPT_BASE_VM);
		writel(1, mmu->base + V9_REG_ALL_INV_VM);
		dma_unmap_single(mmu->dev, mmu->table_dma, LV1_TABLE_SIZE,
				 DMA_TO_DEVICE);
		aoc_sysmmu_free_l2(mmu);
		mmu->enabled = false;
		return;
	}
	writel(mmu->saved_ctrl | CTRL_MMU_BLOCK, mmu->base + REG_MMU_CTRL);
	if (mmu->vm)
		for (v = 0; v < MAX_VIDS; v++)
			if (BIT(v) & mmu->vid_mask)
				writel(mmu->saved_ctrl_vm[v],
				       mmu->base + REG_CTRL_VM(v));
	writel(mmu->saved_ctrl, mmu->base + REG_MMU_CTRL);
	if (mmu->irq_on) {
		free_irq(mmu->irq, mmu);
		mmu->irq_on = false;
	}
	dma_unmap_single(mmu->dev, mmu->table_dma, LV1_TABLE_SIZE,
			 DMA_TO_DEVICE);
	mmu->enabled = false;
}
EXPORT_SYMBOL_GPL(aoc_sysmmu_disable);

/**
 * aoc_sysmmu_get() - the System MMU a client's "google,sysmmu" names
 *
 * Return: the System MMU (device reference held), or an ERR_PTR();
 * -EPROBE_DEFER until it has probed.
 */
struct aoc_sysmmu *aoc_sysmmu_get(struct device *client)
{
	struct platform_device *pdev;
	struct device_node *np;
	struct aoc_sysmmu *mmu;

	np = of_parse_phandle(client->of_node, "google,sysmmu", 0);
	if (!np)
		return ERR_PTR(-ENODEV);
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	mmu = platform_get_drvdata(pdev);
	if (!mmu) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	return mmu;
}
EXPORT_SYMBOL_GPL(aoc_sysmmu_get);

void aoc_sysmmu_put(struct aoc_sysmmu *mmu)
{
	put_device(mmu->dev);
}
EXPORT_SYMBOL_GPL(aoc_sysmmu_put);

static int aoc_sysmmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct aoc_sysmmu_data *data;
	struct aoc_sysmmu *mmu;
	int ret;

	mmu = devm_kzalloc(dev, sizeof(*mmu), GFP_KERNEL);
	if (!mmu)
		return -ENOMEM;
	mmu->dev = dev;
	data = of_device_get_match_data(dev);
	mmu->v9 = data && data->v9;

	/* Mapped only; BLK_AOC may be off, so no register is read here */
	mmu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mmu->base))
		return PTR_ERR(mmu->base);

	mmu->irq = platform_get_irq(pdev, 0);
	if (mmu->irq < 0)
		return mmu->irq;

	ret = dma_set_mask(dev, DMA_BIT_MASK(36));
	if (ret)
		return ret;

	platform_set_drvdata(pdev, mmu);
	dev_info(dev, "ready (registers untouched until BLK_AOC is on)%s\n",
		 empty ? ", empty table (negative test)" : "");
	return 0;
}

static void aoc_sysmmu_remove(struct platform_device *pdev)
{
	struct aoc_sysmmu *mmu = platform_get_drvdata(pdev);

	/* The AoC driver disables translation before it lets go */
	WARN_ON(mmu->enabled);
	aoc_sysmmu_free_l2(mmu);
	if (mmu->table)
		free_pages((unsigned long)mmu->table, get_order(LV1_TABLE_SIZE));
}

static const struct aoc_sysmmu_data gs201_aoc_sysmmu_data = {
	.v9 = false,
};

static const struct aoc_sysmmu_data zuma_aoc_sysmmu_data = {
	.v9 = true,
};

static const struct of_device_id aoc_sysmmu_of_match[] = {
	{ .compatible = "google,gs201-aoc-sysmmu", .data = &gs201_aoc_sysmmu_data },
	{ .compatible = "google,zuma-aoc-sysmmu", .data = &zuma_aoc_sysmmu_data },
	{ }
};
MODULE_DEVICE_TABLE(of, aoc_sysmmu_of_match);

static struct platform_driver aoc_sysmmu_driver = {
	.probe = aoc_sysmmu_probe,
	.remove = aoc_sysmmu_remove,
	.driver = {
		.name = "gs201-aoc-sysmmu",
		.of_match_table = aoc_sysmmu_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(aoc_sysmmu_driver);

MODULE_DESCRIPTION("gs201 AoC System MMU static translation windows");
MODULE_LICENSE("GPL");
