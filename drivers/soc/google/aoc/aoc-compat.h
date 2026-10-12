/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Stand-ins for the vendor kernel interfaces Google's AoC driver uses, on a
 * mainline kernel without them.
 *
 *  - debug-snapshot: no snapshot or emergency-reboot facility; log instead.
 *  - exynos-cpupm power modes: the AP never enters system power modes
 *    here, so there is nothing to hold off while the AoC boots.
 *  - exynos-pmu-if: PMU reads go through the kernel's PMU regmap elsewhere;
 *    the AoC restart path that wants them is disabled.
 *  - acpm_ipc_ctrl: the AoC reset-completion channel (ACPM 13) is not
 *    available yet, so crash recovery is off (aoc_disable_restart).
 *  - sscoredump: no coredump consumer.
 */
#ifndef __AOC_COMPAT_H
#define __AOC_COMPAT_H

#include <linux/errno.h>
#include <linux/printk.h>
#include <linux/types.h>

struct device;
struct device_node;
struct platform_device;

/* soc/google/debug-snapshot.h */
static inline void dbg_snapshot_emergency_reboot(const char *str)
{
	pr_err("aoc: %s (no emergency reboot on this kernel)\n", str);
}

static inline void *dbg_snapshot_expire_watchdog(void)
{
	return NULL;
}

/* soc/google/exynos-cpupm.h */
#define POWERMODE_TYPE_SYSTEM	0
static inline void disable_power_mode(int cpu, int type) { }
static inline void enable_power_mode(int cpu, int type) { }

/* soc/google/exynos-pmu-if.h */
static inline int exynos_pmu_read(unsigned int offset, unsigned int *val)
{
	*val = 0;
	return -ENODEV;
}

/* soc/google/acpm_ipc_ctrl.h */
typedef void (*ipc_callback)(unsigned int *cmd, unsigned int size);
static inline int acpm_ipc_request_channel(struct device_node *np,
					   ipc_callback handler,
					   unsigned int *id, unsigned int *size)
{
	return -ENODEV;
}

static inline void acpm_ipc_release_channel(struct device_node *np,
					    unsigned int id)
{
}

/* linux/platform_data/sscoredump.h */
#define SSCD_NAME "sscoredump"
struct sscd_segment {
	void *addr;
	u64 size;
	u64 flags;
	void *paddr;
	void *vaddr;
};

struct sscd_platform_data {
	int (*sscd_report)(struct platform_device *pdev,
			   struct sscd_segment *segs, u16 nsegs, u64 flags,
			   const char *crash_info);
};
#define SSCD_FLAGS_ELFARM64HDR	0x1

/* gs-chipid: values read from the chip id block (gs201-aoc.c) */
u32 gs_chipid_get_revision(void);
u32 gs_chipid_get_type(void);
u32 gs_chipid_get_product_id(void);

#endif /* __AOC_COMPAT_H */
