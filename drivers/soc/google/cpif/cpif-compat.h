/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Stand-ins for the vendor kernel interfaces Google's CPIF uses, on a
 * mainline kernel without them.
 *
 *  - misc/logbuffer: Google's per-driver ring log has no consumer here;
 *    the messages CPIF puts in it go to the kernel log at debug level.
 */
#ifndef __CPIF_COMPAT_H
#define __CPIF_COMPAT_H

#include <linux/err.h>
#include <linux/printk.h>

/* misc/logbuffer.h */
struct logbuffer;

static inline struct logbuffer *logbuffer_register(const char *name)
{
	return ERR_PTR(-ENODEV);
}

#define logbuffer_log(log, fmt, ...)				\
	do {								\
		(void)(log);						\
		pr_debug("cpif: " fmt "\n", ##__VA_ARGS__);		\
	} while (0)

#endif /* __CPIF_COMPAT_H */
