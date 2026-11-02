/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022, Samsung Electronics.
 *
 */

#ifndef __GNSS_IF_H__
#define __GNSS_IF_H__

#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#define MAX_NAME_LEN		64

struct gnss_pdata {
	char	*name;	/* device name (ex. KEPLER) */
	char	*node_name;	/* device node (ex. gnss_ipc) */

	int	irq_gnss2ap_spi;
	int	irq_gnss2ap_wdt;

	struct gnss_io_t	*iodev;
	struct gnss_link_dev	*link_dev;

};

struct gnss_irq {
		spinlock_t lock;
		unsigned int num;
		char name[MAX_NAME_LEN];
		unsigned long flags;
		bool active;
		bool registered;
};

struct gnss_gpio {
	struct gpio_desc *desc;
	const char *label;
};

#define LOG_TAG "gif: "
#define CALLEE  (__func__)
#define CALLER  (__builtin_return_address(0))

#define gif_err_limited(fmt, ...) \
		 printk_ratelimited(KERN_ERR LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define gif_err(fmt, ...) \
		pr_err(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define gif_debug(fmt, ...) \
		pr_debug(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define gif_info(fmt, ...) \
		pr_info(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

#endif /* __GNSS_IF_H__ */
