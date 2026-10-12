// SPDX-License-Identifier: GPL-2.0
/*
 * The exynos_pcie_*() calls CPIF makes into Google's root complex driver,
 * mapped onto the gs201 modem link driver (pcie-gs201-cp). There is one
 * modem link, channel 0. The calls only the left-out parts use (BTL's
 * outbound window, the packet processor's exclusive MSI vectors) fail.
 */

#include <linux/interrupt.h>
#include <linux/printk.h>
#include <linux/soc/google/gs201-pcie-cp.h>
#include <linux/types.h>
#include <linux/exynos-pci-noti.h>

#include "modem_prj.h"
#include "s51xx_pcie.h"

#define CP_PCIE_CH	0

static bool cp_ch_ok(int ch_num)
{
	if (ch_num == CP_PCIE_CH)
		return true;
	pr_err_once("cpif: PCIe channel %d does not exist, only %d\n",
		    ch_num, CP_PCIE_CH);
	return false;
}

static void cp_compat_event(struct gs201_pcie_cp_event *ev, u32 event)
{
	struct exynos_pcie_register_event *reg = ev->priv;

	reg->notify.event = event & GS201_PCIE_CP_EVENT_LINKDOWN ?
			    EXYNOS_PCIE_EVENT_LINKDOWN :
			    EXYNOS_PCIE_EVENT_CPL_TIMEOUT;
	reg->notify.user = reg->user;
	if (reg->mode == EXYNOS_PCIE_TRIGGER_CALLBACK && reg->callback)
		reg->callback(&reg->notify);
	else if (reg->completion)
		complete(reg->completion);
}

static struct gs201_pcie_cp_event cp_compat_ev = {
	.callback = cp_compat_event,
};

int exynos_pcie_register_event(struct exynos_pcie_register_event *reg)
{
	if (!reg || !reg->user)
		return -ENODEV;
	cp_compat_ev.events = 0;
	if (reg->events & EXYNOS_PCIE_EVENT_LINKDOWN)
		cp_compat_ev.events |= GS201_PCIE_CP_EVENT_LINKDOWN;
	if (reg->events & EXYNOS_PCIE_EVENT_CPL_TIMEOUT)
		cp_compat_ev.events |= GS201_PCIE_CP_EVENT_CPL_TIMEOUT;
	cp_compat_ev.priv = reg;
	return gs201_pcie_cp_register_event(&cp_compat_ev);
}

int exynos_pcie_deregister_event(struct exynos_pcie_register_event *reg)
{
	if (cp_compat_ev.priv != reg)
		return -ENODEV;
	gs201_pcie_cp_deregister_event(&cp_compat_ev);
	cp_compat_ev.priv = NULL;
	return 0;
}

void exynos_pcie_rc_register_dump(int ch_num)
{
	gs201_pcie_cp_dump();
}

void exynos_pcie_rc_dump_all_status(int ch_num)
{
	gs201_pcie_cp_dump();
}

void exynos_pcie_rc_print_msi_register(int ch_num)
{
	gs201_pcie_cp_dump();
}

/* BTL only: not ported */
int exynos_pcie_rc_set_outbound_atu(int ch_num, u32 target_addr, u32 offset,
				    u32 size)
{
	return -EOPNOTSUPP;
}

bool exynos_pcie_rc_get_cpl_timeout_state(int ch_num)
{
	return gs201_pcie_cp_faults() & GS201_PCIE_CP_EVENT_CPL_TIMEOUT;
}

void exynos_pcie_rc_set_cpl_timeout_state(int ch_num, bool recovery)
{
	if (!recovery)
		gs201_pcie_cp_clear_faults(GS201_PCIE_CP_EVENT_CPL_TIMEOUT);
}

bool exynos_pcie_rc_get_sudden_linkdown_state(int ch_num)
{
	return gs201_pcie_cp_faults() & GS201_PCIE_CP_EVENT_LINKDOWN;
}

void exynos_pcie_rc_set_sudden_linkdown_state(int ch_num, bool recovery)
{
	if (!recovery)
		gs201_pcie_cp_clear_faults(GS201_PCIE_CP_EVENT_LINKDOWN);
}

void exynos_pcie_rc_force_linkdown_work(int ch_num)
{
	gs201_pcie_cp_force_linkdown();
}

int exynos_pcie_rc_chk_link_status(int ch_num)
{
	return cp_ch_ok(ch_num) && gs201_pcie_cp_link_ok();
}

int exynos_pcie_rc_l1ss_ctrl(int enable, int id, int ch_num)
{
	return cp_ch_ok(ch_num) ? gs201_pcie_cp_l1ss(enable) : -ENODEV;
}

/* The width is the modem's to choose, as in Google's driver */
int exynos_pcie_poweron(int ch_num, int spd, int width)
{
	return cp_ch_ok(ch_num) ? gs201_pcie_cp_poweron(spd) : -ENODEV;
}

int exynos_pcie_poweroff(int ch_num)
{
	if (!cp_ch_ok(ch_num))
		return -ENODEV;
	gs201_pcie_cp_poweroff();
	return 0;
}

void exynos_pcie_set_perst_gpio(int ch_num, bool on)
{
	if (cp_ch_ok(ch_num))
		gs201_pcie_cp_set_perst_level(on);
}

void exynos_pcie_set_ready_cto_recovery(int ch_num)
{
	if (cp_ch_ok(ch_num))
		gs201_pcie_cp_prepare_recovery();
}

/* The packet processor's exclusive vectors: not ported */
int register_separated_msi_vector(int ch_num, irq_handler_t handler,
				  void *context, int *irq_num)
{
	return -EOPNOTSUPP;
}

/* The target is the link's memory-region; check that both nodes agree. */
int exynos_pcie_set_msi_ctrl_addr(int num, u64 msi_ctrl_addr)
{
	return cp_ch_ok(num) ? gs201_pcie_cp_check_msi_addr(msi_ctrl_addr) :
			       -ENODEV;
}
