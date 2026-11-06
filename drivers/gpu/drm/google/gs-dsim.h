/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Interface between the gs201 DECON driver and the DSIM command path: the
 * DECON driver owns the CRTC and its mode, the DSIM driver owns the link
 * timers and the panel behind it.
 */
#ifndef __GS_DSIM_H__
#define __GS_DSIM_H__

#include <linux/types.h>

struct device;
struct device_node;
struct drm_connector;
struct gs_dsim;

struct gs_dsim *gs_dsim_get(struct device *consumer, struct device_node *np);
void gs_dsim_put(struct gs_dsim *dsim);

unsigned int gs_dsim_refresh_rate(struct gs_dsim *dsim);
bool gs_dsim_can_set_refresh_rate(struct gs_dsim *dsim, unsigned int hz);
int gs_dsim_set_refresh_rate(struct gs_dsim *dsim, unsigned int hz);
int gs_dsim_panel_get_modes(struct gs_dsim *dsim,
			    struct drm_connector *connector);

int gs_dsim_panel_sleep(struct gs_dsim *dsim);
int gs_dsim_panel_wake(struct gs_dsim *dsim, bool enable);

#endif /* __GS_DSIM_H__ */
