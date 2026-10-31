/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Device Tree binding constants for the clocks of the Google Tensor G3
 * (zuma) ACPM firmware.
 */

#ifndef _DT_BINDINGS_CLOCK_GOOGLE_ZUMA_ACPM_H
#define _DT_BINDINGS_CLOCK_GOOGLE_ZUMA_ACPM_H

/* DVFS domains, numbered as in the firmware messages (enum acpm_dvfs_id). */
#define ZUMA_CLK_ACPM_DVFS_MIF				0
#define ZUMA_CLK_ACPM_DVFS_INT				1
#define ZUMA_CLK_ACPM_DVFS_CPUCL0			2
#define ZUMA_CLK_ACPM_DVFS_CPUCL1			3
#define ZUMA_CLK_ACPM_DVFS_CPUCL2			4
#define ZUMA_CLK_ACPM_DVFS_DSU				5
#define ZUMA_CLK_ACPM_DVFS_BCI				6
#define ZUMA_CLK_ACPM_DVFS_G3D				7
#define ZUMA_CLK_ACPM_DVFS_G3DL2			8
#define ZUMA_CLK_ACPM_DVFS_TPU				9
#define ZUMA_CLK_ACPM_DVFS_INTCAM			10
#define ZUMA_CLK_ACPM_DVFS_TNR				11
#define ZUMA_CLK_ACPM_DVFS_CAM				12
#define ZUMA_CLK_ACPM_DVFS_MFC				13
#define ZUMA_CLK_ACPM_DVFS_DISP				14
#define ZUMA_CLK_ACPM_DVFS_AOC				15
#define ZUMA_CLK_ACPM_DVFS_BW				16

/*
 * The GPU as one clock: its rate is the shader (G3D) rate, and setting it
 * also moves the top/L2 domain (G3DL2) to the level paired with it.
 */
#define ZUMA_CLK_ACPM_GPU				17

#endif /* _DT_BINDINGS_CLOCK_GOOGLE_ZUMA_ACPM_H */
