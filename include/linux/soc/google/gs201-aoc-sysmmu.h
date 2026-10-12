/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_SOC_GOOGLE_GS201_AOC_SYSMMU_H
#define __LINUX_SOC_GOOGLE_GS201_AOC_SYSMMU_H

#include <linux/types.h>

struct device;
struct aoc_sysmmu;

/* One translation window: AoC address -> physical, 1 MiB granular */
struct aoc_sysmmu_window {
	u32 iova;
	phys_addr_t pa;
	u32 size;
};

struct aoc_sysmmu *aoc_sysmmu_get(struct device *client);
void aoc_sysmmu_put(struct aoc_sysmmu *mmu);

/* All of these touch BLK_AOC registers: call only while BLK_AOC is on */
void aoc_sysmmu_dump(struct aoc_sysmmu *mmu, const char *when);
int aoc_sysmmu_enable(struct aoc_sysmmu *mmu,
		      const struct aoc_sysmmu_window *win, unsigned int n);
void aoc_sysmmu_disable(struct aoc_sysmmu *mmu);

#endif /* __LINUX_SOC_GOOGLE_GS201_AOC_SYSMMU_H */
