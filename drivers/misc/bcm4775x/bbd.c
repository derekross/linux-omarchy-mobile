// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2014 Broadcom Corporation
 *
 * The BBD (Broadcom Bridge Driver)
 *
 * Google's gs201 bbd.c, reduced to what the stock lhd daemon uses on a
 * Pixel 7 Pro: /dev/bbd_control (control strings in), /dev/bbd_patch (an
 * empty embedded patch, as on stock: lhd loads the patch file itself over
 * /dev/ttyBCM), /dev/bbd_pwrstat (GNSS on/off statistics) and
 * /dev/bbd_sensor (no sensor hub here, so it never has data).
 *
 * Changes from Google's: the control strings are matched with strstr() on a
 * NUL-terminated copy (Google's strcmp() chain was inverted, so every string
 * took the first branch); the whole chrdev region is freed on exit and on
 * error; no PM notifier (there was nothing left for it to do without the
 * 1 Hz statistics timer).
 */

#define pr_fmt(fmt) "bbd: " fmt

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>

#include "bbd.h"

static const char * const bbd_dev_name[BBD_DEVICE_INDEX] = {
	[BBD_MINOR_SHMD]	= "bbd_shmd",
	[BBD_MINOR_SENSOR]	= "bbd_sensor",
	[BBD_MINOR_CONTROL]	= "bbd_control",
	[BBD_MINOR_PATCH]	= "bbd_patch",
	[BBD_MINOR_PWRSTAT]	= "bbd_pwrstat",
};

static void bbd_log_hex(struct bbd_device *bbd, const char *prefix,
			const void *buf, size_t len)
{
	if (likely(!bbd->dbg->bbd))
		return;

	print_hex_dump(KERN_INFO, prefix, DUMP_PREFIX_NONE, 32, 1, buf, len,
		       false);
}

static void bbd_pwrstat_switch(struct bbd_device *bbd, bool on)
{
	struct bbd_cdev_priv *p = &bbd->priv[BBD_MINOR_PWRSTAT];
	struct gnss_pwrstats *s = &p->pwrstats;
	u64 now = ktime_to_us(ktime_get_boottime());

	mutex_lock(&p->lock);
	if (on) {
		s->gps_on = true;
		s->gps_on_cnt++;
		s->gps_on_entry = now;
		s->gps_off_exit = now;
		s->gps_off_duration += s->gps_off_exit - s->gps_off_entry;
	} else {
		s->gps_on = false;
		s->gps_off_cnt++;
		s->gps_off_entry = now;
		s->gps_on_exit = now;
		s->gps_on_duration += s->gps_on_exit - s->gps_on_entry;
	}
	mutex_unlock(&p->lock);
}

/* Handles one control string from lhd; @buf is NUL-terminated. */
static void bbd_control(struct bbd_device *bbd, const char *buf)
{
	struct bbd_debug *dbg = bbd->dbg;

	if (dbg->bbd)
		pr_info("control: %s\n", buf);

	if (strstr(buf, BBD_CTRL_DEBUG_ON))
		dbg->bbd = true;
	else if (strstr(buf, BBD_CTRL_DEBUG_OFF))
		dbg->bbd = false;
	else if (strstr(buf, SSI_DEBUG_ON))
		dbg->ssi = true;
	else if (strstr(buf, SSI_DEBUG_OFF))
		dbg->ssi = false;
	else if (strstr(buf, PZC_DEBUG_ON))
		dbg->pzc = true;
	else if (strstr(buf, PZC_DEBUG_OFF))
		dbg->pzc = false;
	else if (strstr(buf, RNG_DEBUG_ON))
		dbg->rng = true;
	else if (strstr(buf, RNG_DEBUG_OFF))
		dbg->rng = false;
	else if (strstr(buf, GPSD_CORE_ON))
		bbd_pwrstat_switch(bbd, true);
	else if (strstr(buf, GPSD_CORE_OFF))
		bbd_pwrstat_switch(bbd, false);
	/* Anything else (BBD:PassThru, ESW:READY, ...) needs nothing here. */
}

static int bbd_common_open(struct inode *inode, struct file *filp)
{
	struct bbd_cdev_priv *p = container_of(inode->i_cdev,
					       struct bbd_cdev_priv, cdev);
	unsigned int minor = iminor(inode);

	if (minor >= BBD_DEVICE_INDEX)
		return -ENODEV;

	if (p->busy && minor != BBD_MINOR_CONTROL)
		return -EBUSY;

	p->busy = true;
	mutex_lock(&p->lock);
	p->read_buf.head = p->read_buf.tail = 0;
	mutex_unlock(&p->lock);
	filp->private_data = p;

	return 0;
}

static int bbd_common_release(struct inode *inode, struct file *filp)
{
	struct bbd_cdev_priv *p = filp->private_data;

	p->busy = false;

	return 0;
}

static ssize_t bbd_common_read(struct file *filp, char __user *buf,
			       size_t size, loff_t *ppos)
{
	struct bbd_cdev_priv *p = filp->private_data;
	struct circ_buf *circ = &p->read_buf;
	size_t rd_size = 0;

	mutex_lock(&p->lock);
	while (size > 0 && CIRC_CNT(circ->head, circ->tail, BBD_BUFF_SIZE)) {
		size_t cnt_to_end = CIRC_CNT_TO_END(circ->head, circ->tail,
						    BBD_BUFF_SIZE);
		size_t copied = min(cnt_to_end, size);

		if (copy_to_user(buf + rd_size, circ->buf + circ->tail,
				 copied)) {
			mutex_unlock(&p->lock);
			return -EFAULT;
		}
		bbd_log_hex(p->bbd, "bbd r ", circ->buf + circ->tail, copied);

		size -= copied;
		rd_size += copied;
		circ->tail = (circ->tail + copied) & (BBD_BUFF_SIZE - 1);
	}
	mutex_unlock(&p->lock);

	return rd_size;
}

static __poll_t bbd_common_poll(struct file *filp, poll_table *wait)
{
	struct bbd_cdev_priv *p = filp->private_data;
	struct circ_buf *circ = &p->read_buf;
	__poll_t mask = 0;

	poll_wait(filp, &p->poll_wait, wait);

	if (CIRC_CNT(circ->head, circ->tail, BBD_BUFF_SIZE))
		mask |= EPOLLIN | EPOLLRDNORM;

	return mask;
}

static ssize_t bbd_control_write(struct file *filp, const char __user *buf,
				 size_t size, loff_t *ppos)
{
	struct bbd_cdev_priv *p = filp->private_data;

	if (size >= BBD_BUFF_SIZE)
		return -EINVAL;

	mutex_lock(&p->lock);
	if (copy_from_user(p->write_buf, buf, size)) {
		mutex_unlock(&p->lock);
		return -EFAULT;
	}
	/* lhd does not terminate its strings */
	p->write_buf[size] = '\0';
	bbd_log_hex(p->bbd, "bbd w ", p->write_buf, size);
	mutex_unlock(&p->lock);

	bbd_control(p->bbd, p->write_buf);

	return size;
}

/* Stock embeds no patch: lhd reads EOF here and loads its patch file. */
static ssize_t bbd_patch_read(struct file *filp, char __user *buf,
			      size_t size, loff_t *ppos)
{
	return 0;
}

#define BBD_MAX_PWRSTAT_SIZE 512

static ssize_t bbd_pwrstat_read(struct file *filp, char __user *buf,
				size_t size, loff_t *ppos)
{
	struct bbd_cdev_priv *p = filp->private_data;
	struct gnss_pwrstats *s = &p->pwrstats;
	u64 now, on_dur, off_dur;
	char *out;
	ssize_t ret;
	int n = 0;

	out = kmalloc(BBD_MAX_PWRSTAT_SIZE, GFP_KERNEL);
	if (!out)
		return -ENOMEM;

	mutex_lock(&p->lock);
	now = ktime_to_us(ktime_get_boottime());
	on_dur = s->gps_on_duration;
	off_dur = s->gps_off_duration;
	if (s->gps_on)
		on_dur += now - s->gps_on_entry;
	else
		off_dur += now - s->gps_off_entry;

	n += scnprintf(out + n, BBD_MAX_PWRSTAT_SIZE - n,
		       "GPS_ON:\ncount: 0x%llx\nduration_usec: 0x%llx\n"
		       "last_entry_timestamp_usec: 0x%llx\n"
		       "last_exit_timestamp_usec: 0x%llx\n",
		       s->gps_on_cnt, on_dur, s->gps_on_entry, s->gps_on_exit);
	n += scnprintf(out + n, BBD_MAX_PWRSTAT_SIZE - n,
		       "GPS_OFF:\ncount: 0x%llx\nduration_usec: 0x%llx\n"
		       "last_entry_timestamp_usec: 0x%llx\n"
		       "last_exit_timestamp_usec: 0x%llx\n",
		       s->gps_off_cnt, off_dur, s->gps_off_entry,
		       s->gps_off_exit);
	mutex_unlock(&p->lock);

	ret = simple_read_from_buffer(buf, size, ppos, out, n);
	kfree(out);

	return ret;
}

static const struct file_operations bbd_fops[BBD_DEVICE_INDEX] = {
	[BBD_MINOR_SENSOR] = {
		.owner		= THIS_MODULE,
		.open		= bbd_common_open,
		.release	= bbd_common_release,
		.read		= bbd_common_read,
		.poll		= bbd_common_poll,
	},
	[BBD_MINOR_CONTROL] = {
		.owner		= THIS_MODULE,
		.open		= bbd_common_open,
		.release	= bbd_common_release,
		.read		= bbd_common_read,
		.write		= bbd_control_write,
		.poll		= bbd_common_poll,
	},
	[BBD_MINOR_PATCH] = {
		.owner		= THIS_MODULE,
		.open		= bbd_common_open,
		.release	= bbd_common_release,
		.read		= bbd_patch_read,
	},
	[BBD_MINOR_PWRSTAT] = {
		.owner		= THIS_MODULE,
		.open		= bbd_common_open,
		.release	= bbd_common_release,
		.read		= bbd_pwrstat_read,
	},
};

static void bbd_remove_cdevs(struct bbd_device *bbd, int last)
{
	int minor;

	for (minor = last; minor > BBD_MINOR_SHMD; minor--) {
		struct bbd_cdev_priv *p = &bbd->priv[minor];

		device_destroy(bbd->class, p->devno);
		cdev_del(&p->cdev);
	}
}

struct bbd_device *bbd_init(struct device *parent, struct bbd_debug *dbg)
{
	struct bbd_device *bbd;
	int minor, ret;

	bbd = kvzalloc(sizeof(*bbd), GFP_KERNEL);
	if (!bbd)
		return ERR_PTR(-ENOMEM);

	bbd->dbg = dbg;

	ret = alloc_chrdev_region(&bbd->dev_num, 0, BBD_DEVICE_INDEX, "bbd");
	if (ret)
		goto err_free;

	bbd->class = class_create("bbd");
	if (IS_ERR(bbd->class)) {
		ret = PTR_ERR(bbd->class);
		goto err_region;
	}

	/* Minor 0 (bbd_shmd) is not created, as on stock. */
	for (minor = BBD_MINOR_SENSOR; minor < BBD_DEVICE_INDEX; minor++) {
		struct bbd_cdev_priv *p = &bbd->priv[minor];

		p->bbd = bbd;
		p->devno = MKDEV(MAJOR(bbd->dev_num), minor);
		p->read_buf.buf = p->_read_buf;
		init_waitqueue_head(&p->poll_wait);
		mutex_init(&p->lock);

		cdev_init(&p->cdev, &bbd_fops[minor]);
		p->cdev.owner = THIS_MODULE;
		ret = cdev_add(&p->cdev, p->devno, 1);
		if (ret)
			goto err_cdevs;

		p->dev = device_create(bbd->class, parent, p->devno, bbd,
				       "%s", bbd_dev_name[minor]);
		if (IS_ERR(p->dev)) {
			ret = PTR_ERR(p->dev);
			cdev_del(&p->cdev);
			goto err_cdevs;
		}
	}

	return bbd;

err_cdevs:
	bbd_remove_cdevs(bbd, minor - 1);
	class_destroy(bbd->class);
err_region:
	unregister_chrdev_region(bbd->dev_num, BBD_DEVICE_INDEX);
err_free:
	kvfree(bbd);
	return ERR_PTR(ret);
}

void bbd_exit(struct bbd_device *bbd)
{
	bbd_remove_cdevs(bbd, BBD_DEVICE_INDEX - 1);
	class_destroy(bbd->class);
	unregister_chrdev_region(bbd->dev_num, BBD_DEVICE_INDEX);
	kvfree(bbd);
}
