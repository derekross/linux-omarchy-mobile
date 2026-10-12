/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2014 Broadcom Corporation
 *
 * The BBD (Broadcom Bridge Driver): the /dev/bbd_* nodes the stock lhd
 * daemon opens next to /dev/ttyBCM. Based on Google's gs201 bbd.c/bbd.h,
 * without the Samsung sensor-hub (SSP) glue and the 1 Hz debug statistics.
 */

#ifndef __BBD_H__
#define __BBD_H__

#include <linux/cdev.h>
#include <linux/circ_buf.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>

union long_union_t {
	u8 uc[sizeof(u32)];
	u32 ul;
} __packed __aligned(4);

union short_union_t {
	u8  uc[sizeof(u16)];
	u16 us;
} __packed __aligned(4);

enum {
	BBD_MINOR_SHMD = 0,	/* not created, as on stock */
	BBD_MINOR_SENSOR,
	BBD_MINOR_CONTROL,
	BBD_MINOR_PATCH,
	BBD_MINOR_PWRSTAT,
	BBD_DEVICE_INDEX
};

/* Control strings lhd writes to /dev/bbd_control */
#define BBD_CTRL_DEBUG_ON	"BBD:DEBUG=1"
#define BBD_CTRL_DEBUG_OFF	"BBD:DEBUG=0"
#define SSI_DEBUG_ON		"SSI:DEBUG=1"
#define SSI_DEBUG_OFF		"SSI:DEBUG=0"
#define PZC_DEBUG_ON		"PZC:DEBUG=1"
#define PZC_DEBUG_OFF		"PZC:DEBUG=0"
#define RNG_DEBUG_ON		"RNG:DEBUG=1"
#define RNG_DEBUG_OFF		"RNG:DEBUG=0"
#define GPSD_CORE_ON		"GPSD:CORE_ON"
#define GPSD_CORE_OFF		"GPSD:CORE_OFF"

#define BBD_BUFF_SIZE		(4096 * 2)

struct gnss_pwrstats {
	bool gps_on;
	u64 gps_on_cnt;
	u64 gps_on_duration;
	u64 gps_on_entry;
	u64 gps_on_exit;
	u64 gps_off_cnt;
	u64 gps_off_duration;
	u64 gps_off_entry;
	u64 gps_off_exit;
};

struct bbd_device;

struct bbd_cdev_priv {
	struct bbd_device *bbd;
	struct cdev cdev;
	struct device *dev;
	dev_t devno;
	bool busy;
	struct circ_buf read_buf;		/* lhd reads from BBD */
	struct mutex lock;			/* protects read_buf, pwrstats */
	char _read_buf[BBD_BUFF_SIZE];
	char write_buf[BBD_BUFF_SIZE];		/* lhd writes into BBD */
	wait_queue_head_t poll_wait;
	struct gnss_pwrstats pwrstats;		/* BBD_MINOR_PWRSTAT only */
};

/* Debug switches lhd can flip through /dev/bbd_control */
struct bbd_debug {
	bool bbd;
	bool ssi;
	bool pzc;
	bool rng;
};

struct bbd_device {
	struct class *class;
	dev_t dev_num;
	struct bbd_cdev_priv priv[BBD_DEVICE_INDEX];
	struct bbd_debug *dbg;			/* owned by the SPI driver */
};

struct bbd_device *bbd_init(struct device *parent, struct bbd_debug *dbg);
void bbd_exit(struct bbd_device *bbd);

#endif /* __BBD_H__ */
