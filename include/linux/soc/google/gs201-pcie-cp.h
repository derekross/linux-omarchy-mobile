/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google gs201 PCIe GEN4A_0 (the Samsung S5300 modem's link): the calls the
 * modem driver makes into the link driver (pcie-gs201-cp).
 *
 * There is one modem link, so the calls take no handle. Each returns
 * -ENODEV (or does nothing) while the link driver is not bound.
 */
#ifndef __LINUX_SOC_GOOGLE_GS201_PCIE_CP_H
#define __LINUX_SOC_GOOGLE_GS201_PCIE_CP_H

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/types.h>

/* Link faults, reported to the registered client and kept until cleared */
#define GS201_PCIE_CP_EVENT_LINKDOWN		BIT(0)
#define GS201_PCIE_CP_EVENT_CPL_TIMEOUT		BIT(1)

/**
 * struct gs201_pcie_cp_event - a client's fault callback
 * @events: the GS201_PCIE_CP_EVENT_* bits the client wants
 * @callback: called from a workqueue with the one event that happened
 * @priv: the client's data
 */
struct gs201_pcie_cp_event {
	u32 events;
	void (*callback)(struct gs201_pcie_cp_event *ev, u32 event);
	void *priv;
};

#if IS_ENABLED(CONFIG_PCIE_GS201_CP)
int gs201_pcie_cp_poweron(unsigned int speed);
void gs201_pcie_cp_poweroff(void);
bool gs201_pcie_cp_link_ok(void);
int gs201_pcie_cp_check_msi_addr(u64 addr);
void gs201_pcie_cp_set_perst_level(int level);
int gs201_pcie_cp_l1ss(bool enable);
int gs201_pcie_cp_register_event(struct gs201_pcie_cp_event *ev);
void gs201_pcie_cp_deregister_event(struct gs201_pcie_cp_event *ev);
u32 gs201_pcie_cp_faults(void);
void gs201_pcie_cp_clear_faults(u32 events);
void gs201_pcie_cp_force_linkdown(void);
void gs201_pcie_cp_prepare_recovery(void);
void gs201_pcie_cp_dump(void);
#else
static inline int gs201_pcie_cp_poweron(unsigned int speed) { return -ENODEV; }
static inline void gs201_pcie_cp_poweroff(void) { }
static inline bool gs201_pcie_cp_link_ok(void) { return false; }
static inline int gs201_pcie_cp_check_msi_addr(u64 addr) { return -ENODEV; }
static inline void gs201_pcie_cp_set_perst_level(int level) { }
static inline int gs201_pcie_cp_l1ss(bool enable) { return -ENODEV; }
static inline int gs201_pcie_cp_register_event(struct gs201_pcie_cp_event *ev)
{
	return -ENODEV;
}

static inline void gs201_pcie_cp_deregister_event(struct gs201_pcie_cp_event *ev) { }
static inline u32 gs201_pcie_cp_faults(void) { return 0; }
static inline void gs201_pcie_cp_clear_faults(u32 events) { }
static inline void gs201_pcie_cp_force_linkdown(void) { }
static inline void gs201_pcie_cp_prepare_recovery(void) { }
static inline void gs201_pcie_cp_dump(void) { }
#endif

#endif /* __LINUX_SOC_GOOGLE_GS201_PCIE_CP_H */
