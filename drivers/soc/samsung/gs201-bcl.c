// SPDX-License-Identifier: GPL-2.0-only
/*
 * Battery current limiter (BCL) listener for Google Tensor G2 (gs201) phones.
 *
 * Three lines tell the SoC that the battery can't keep up:
 *
 *  - vdroop1: the MAX77759's SYS_UVLO1 (system voltage under its first level),
 *    active high;
 *  - vdroop2: the MAX77759's SYS_UVLO2 or BAT_OILO (second voltage level, or
 *    battery over-current), which share one pin, active high;
 *  - smpl-warn: the S2MPG12's SMPL_WARN (battery under its warning voltage),
 *    active low.
 *
 * While a line is asserted this driver caps the CPU clusters (cpufreq QoS) and
 * the GPU (devfreq QoS) at the levels Google's BCL uses on these phones
 * (freq_qos in gs201-cloudripper-bcl.dtsi), polls the line every poll_ms and
 * lifts the caps once it has been clear for hold_ms. A line still asserted
 * after stuck_ms keeps its caps but is reported once and read only once a
 * second from then on: if a pin turns out to stay latched until the PMIC's
 * interrupt bits are cleared, which this driver doesn't do, the phone stays
 * capped but doesn't poll every few milliseconds until it reboots.
 *
 * Without the MAX77759's interrupt register, vdroop2 can't be told apart
 * from BAT_OILO, so it takes the stricter value of the two rows.
 *
 * It writes nothing to either PMIC: the thresholds are what the PMICs and
 * the bootloader left. It doesn't drive the clusters' hardware clock
 * dividers either, so it covers droops that last milliseconds, not
 * microsecond transients.
 */

#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

enum gs201_bcl_line {
	BCL_SMPL_WARN,
	BCL_VDROOP1,
	BCL_VDROOP2,
	BCL_NR_LINES,
};

#define BCL_NR_CLUSTERS	3

static const char * const gs201_bcl_line_names[BCL_NR_LINES] = {
	[BCL_SMPL_WARN]	= "smpl-warn",
	[BCL_VDROOP1]	= "vdroop1",
	[BCL_VDROOP2]	= "vdroop2",
};

/* The first CPU of each cluster: little, mid, big. */
static const unsigned int gs201_bcl_cluster_cpu[BCL_NR_CLUSTERS] = { 0, 4, 6 };

/* Caps in kHz: the CPU clusters (little, mid, big), then the GPU. */
struct gs201_bcl_caps {
	u32 cpu[BCL_NR_CLUSTERS];
	u32 gpu;
};

static const struct gs201_bcl_caps gs201_bcl_caps[BCL_NR_LINES] = {
	/* smpl_gm */
	[BCL_SMPL_WARN]	= { { 1803000, 1999000, 2252000 }, 572000 },
	/* vdroop1 */
	[BCL_VDROOP1]	= { { 1803000, 1999000, 2252000 }, 848000 },
	/* batoilo's CPU caps with vdroop2's GPU cap: the stricter of both */
	[BCL_VDROOP2]	= { {  930000, 1197000, 1426000 }, 572000 },
};

static unsigned int poll_ms = 5;
module_param(poll_ms, uint, 0644);
MODULE_PARM_DESC(poll_ms, "How often an asserted line is read again, in ms (default 5)");

static unsigned int hold_ms = 20;
module_param(hold_ms, uint, 0644);
MODULE_PARM_DESC(hold_ms, "How long a line must stay clear before its caps go, in ms (default 20)");

static unsigned int stuck_ms = 5000;
module_param(stuck_ms, uint, 0644);
MODULE_PARM_DESC(stuck_ms, "After how long an asserted line is reported and read only once a second, in ms (default 5000)");

#define BCL_STUCK_POLL_MS	1000

static bool dry_run;
module_param(dry_run, bool, 0644);
MODULE_PARM_DESC(dry_run, "Count and log events but apply no caps (default off)");

struct gs201_bcl;

struct gs201_bcl_line_data {
	struct gs201_bcl *bcl;
	enum gs201_bcl_line id;
	struct gpio_desc *gpiod;
	int irq;
	bool simulated;		/* forced asserted through debugfs */
	bool active;		/* its caps are in force */
	bool stuck;		/* asserted for longer than stuck_ms */
	unsigned long clear_since;	/* jiffies it was first seen clear */
	unsigned int count;	/* times it went active */
	u64 active_ms;		/* total time active, in ms */
	ktime_t since;		/* when it last went active */
};

struct gs201_bcl {
	struct device *dev;
	struct mutex lock;	/* lines, caps and the QoS requests */
	struct gs201_bcl_line_data line[BCL_NR_LINES];
	struct cpufreq_policy *policy[BCL_NR_CLUSTERS];
	struct freq_qos_request cpu_req[BCL_NR_CLUSTERS];
	struct device *gpu;
	struct dev_pm_qos_request gpu_req;
	struct gs201_bcl_caps applied;
	struct delayed_work poll;
	struct dentry *debugfs;
};

static bool gs201_bcl_asserted(struct gs201_bcl_line_data *l)
{
	if (l->simulated)
		return true;
	if (!l->gpiod)
		return false;

	return gpiod_get_value_cansleep(l->gpiod) > 0;
}

/* Re-read every line, then apply the strictest caps of the active ones. */
static void gs201_bcl_update(struct gs201_bcl *bcl)
{
	struct gs201_bcl_caps caps;
	bool any_active = false, any_moving = false;
	int i, c;

	lockdep_assert_held(&bcl->lock);

	for (c = 0; c < BCL_NR_CLUSTERS; c++)
		caps.cpu[c] = FREQ_QOS_MAX_DEFAULT_VALUE;
	caps.gpu = PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE;

	for (i = 0; i < BCL_NR_LINES; i++) {
		struct gs201_bcl_line_data *l = &bcl->line[i];

		if (gs201_bcl_asserted(l)) {
			l->clear_since = 0;
			if (!l->active) {
				l->active = true;
				l->count++;
				l->since = ktime_get();
				dev_info_ratelimited(bcl->dev, "%s asserted%s\n",
						     gs201_bcl_line_names[i],
						     dry_run ? " (dry run)" : "");
			}
			if (!l->stuck && stuck_ms &&
			    ktime_ms_delta(ktime_get(), l->since) >= stuck_ms) {
				l->stuck = true;
				dev_warn(bcl->dev,
					 "%s still asserted after %u ms, reading it once a second\n",
					 gs201_bcl_line_names[i], stuck_ms);
			}
			if (!l->stuck)
				any_moving = true;
		} else if (l->active) {
			any_moving = true;
			if (!l->clear_since)
				l->clear_since = jiffies ?: 1;
			if (time_after_eq(jiffies, l->clear_since +
					  msecs_to_jiffies(hold_ms))) {
				l->active = false;
				l->stuck = false;
				l->clear_since = 0;
				l->active_ms += ktime_ms_delta(ktime_get(), l->since);
			}
		}

		if (!l->active)
			continue;

		any_active = true;
		for (c = 0; c < BCL_NR_CLUSTERS; c++)
			caps.cpu[c] = min(caps.cpu[c], gs201_bcl_caps[i].cpu[c]);
		caps.gpu = min(caps.gpu, gs201_bcl_caps[i].gpu);
	}

	if (dry_run) {
		for (c = 0; c < BCL_NR_CLUSTERS; c++)
			caps.cpu[c] = FREQ_QOS_MAX_DEFAULT_VALUE;
		caps.gpu = PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE;
	}

	for (c = 0; c < BCL_NR_CLUSTERS; c++) {
		if (caps.cpu[c] != bcl->applied.cpu[c] &&
		    freq_qos_update_request(&bcl->cpu_req[c], caps.cpu[c]) >= 0)
			bcl->applied.cpu[c] = caps.cpu[c];
	}
	if (bcl->gpu && caps.gpu != bcl->applied.gpu &&
	    dev_pm_qos_update_request(&bcl->gpu_req, caps.gpu) >= 0)
		bcl->applied.gpu = caps.gpu;

	if (any_active)
		mod_delayed_work(system_highpri_wq, &bcl->poll,
				 msecs_to_jiffies(any_moving ? poll_ms ?: 1 :
						  BCL_STUCK_POLL_MS));
}

static void gs201_bcl_poll(struct work_struct *work)
{
	struct gs201_bcl *bcl = container_of(to_delayed_work(work),
					     struct gs201_bcl, poll);

	mutex_lock(&bcl->lock);
	gs201_bcl_update(bcl);
	mutex_unlock(&bcl->lock);
}

static irqreturn_t gs201_bcl_irq(int irq, void *data)
{
	struct gs201_bcl_line_data *l = data;

	mutex_lock(&l->bcl->lock);
	gs201_bcl_update(l->bcl);
	mutex_unlock(&l->bcl->lock);

	return IRQ_HANDLED;
}

static int gs201_bcl_status_show(struct seq_file *s, void *unused)
{
	struct gs201_bcl *bcl = s->private;
	int i;

	mutex_lock(&bcl->lock);
	for (i = 0; i < BCL_NR_LINES; i++) {
		struct gs201_bcl_line_data *l = &bcl->line[i];
		u64 ms = l->active_ms;

		if (l->active)
			ms += ktime_ms_delta(ktime_get(), l->since);
		seq_printf(s, "%-9s level %d%s active %d%s count %u active_ms %llu\n",
			   gs201_bcl_line_names[i],
			   l->gpiod ? gpiod_get_value_cansleep(l->gpiod) : -1,
			   l->simulated ? " (simulated)" : "", l->active,
			   l->stuck ? " (stuck)" : "", l->count, ms);
	}
	seq_printf(s, "caps_khz  cpu %u %u %u gpu %u%s\n",
		   bcl->applied.cpu[0], bcl->applied.cpu[1], bcl->applied.cpu[2],
		   bcl->applied.gpu, dry_run ? " (dry run)" : "");
	mutex_unlock(&bcl->lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(gs201_bcl_status);

/* "<line> 1" forces a line asserted, "<line> 0" lets it follow its pin. */
static ssize_t gs201_bcl_simulate(struct file *file, const char __user *ubuf,
				  size_t len, loff_t *ppos)
{
	struct gs201_bcl *bcl = file->private_data;
	char buf[32], name[16];
	int i, on;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	if (sscanf(buf, "%15s %d", name, &on) != 2)
		return -EINVAL;

	i = match_string(gs201_bcl_line_names, BCL_NR_LINES, name);
	if (i < 0)
		return i;

	mutex_lock(&bcl->lock);
	bcl->line[i].simulated = on;
	gs201_bcl_update(bcl);
	mutex_unlock(&bcl->lock);

	return len;
}

static const struct file_operations gs201_bcl_simulate_fops = {
	.owner	= THIS_MODULE,
	.open	= simple_open,
	.write	= gs201_bcl_simulate,
	.llseek	= noop_llseek,
};

static struct device *gs201_bcl_find_gpu(struct gs201_bcl *bcl)
{
	struct device_node *np;
	struct platform_device *pdev;

	np = of_parse_phandle(bcl->dev->of_node, "google,gpu", 0);
	if (!np)
		return NULL;
	pdev = of_find_device_by_node(np);
	of_node_put(np);

	return pdev ? &pdev->dev : ERR_PTR(-EPROBE_DEFER);
}

static void gs201_bcl_release(struct gs201_bcl *bcl)
{
	int c;

	for (c = 0; c < BCL_NR_CLUSTERS; c++) {
		if (freq_qos_request_active(&bcl->cpu_req[c]))
			freq_qos_remove_request(&bcl->cpu_req[c]);
		if (bcl->policy[c])
			cpufreq_cpu_put(bcl->policy[c]);
		bcl->policy[c] = NULL;
	}
	if (bcl->gpu) {
		if (dev_pm_qos_request_active(&bcl->gpu_req))
			dev_pm_qos_remove_request(&bcl->gpu_req);
		put_device(bcl->gpu);
		bcl->gpu = NULL;
	}
}

static int gs201_bcl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs201_bcl *bcl;
	struct dentry *dir;
	int i, c, ret;

	bcl = devm_kzalloc(dev, sizeof(*bcl), GFP_KERNEL);
	if (!bcl)
		return -ENOMEM;

	bcl->dev = dev;
	mutex_init(&bcl->lock);
	INIT_DELAYED_WORK(&bcl->poll, gs201_bcl_poll);
	platform_set_drvdata(pdev, bcl);

	for (i = 0; i < BCL_NR_LINES; i++) {
		struct gs201_bcl_line_data *l = &bcl->line[i];

		l->bcl = bcl;
		l->id = i;
		l->gpiod = devm_gpiod_get_optional(dev, gs201_bcl_line_names[i],
						   GPIOD_IN);
		if (IS_ERR(l->gpiod))
			return dev_err_probe(dev, PTR_ERR(l->gpiod),
					     "%s gpio\n", gs201_bcl_line_names[i]);
	}

	for (c = 0; c < BCL_NR_CLUSTERS; c++) {
		bcl->policy[c] = cpufreq_cpu_get(gs201_bcl_cluster_cpu[c]);
		if (!bcl->policy[c]) {
			ret = -EPROBE_DEFER;
			goto err_release;
		}
		ret = freq_qos_add_request(&bcl->policy[c]->constraints,
					   &bcl->cpu_req[c], FREQ_QOS_MAX,
					   FREQ_QOS_MAX_DEFAULT_VALUE);
		if (ret < 0)
			goto err_release;
		bcl->applied.cpu[c] = FREQ_QOS_MAX_DEFAULT_VALUE;
	}

	bcl->gpu = gs201_bcl_find_gpu(bcl);
	if (IS_ERR(bcl->gpu)) {
		ret = PTR_ERR(bcl->gpu);
		bcl->gpu = NULL;
		goto err_release;
	}
	if (bcl->gpu) {
		ret = dev_pm_qos_add_request(bcl->gpu, &bcl->gpu_req,
					     DEV_PM_QOS_MAX_FREQUENCY,
					     PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE);
		if (ret < 0)
			goto err_release;
		bcl->applied.gpu = PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE;
	} else {
		dev_warn(dev, "no GPU: only the CPUs are capped\n");
	}

	for (i = 0; i < BCL_NR_LINES; i++) {
		struct gs201_bcl_line_data *l = &bcl->line[i];
		unsigned long flags;

		if (!l->gpiod)
			continue;
		l->irq = gpiod_to_irq(l->gpiod);
		if (l->irq < 0) {
			ret = dev_err_probe(dev, l->irq, "%s irq\n",
					    gs201_bcl_line_names[i]);
			goto err_irqs;
		}
		/* Only the asserting edge: the poll sees the line clear. */
		flags = gpiod_is_active_low(l->gpiod) ? IRQF_TRIGGER_FALLING :
							IRQF_TRIGGER_RISING;
		ret = request_threaded_irq(l->irq, NULL, gs201_bcl_irq,
					   flags | IRQF_ONESHOT,
					   gs201_bcl_line_names[i], l);
		if (ret) {
			dev_err_probe(dev, ret, "%s irq %d\n",
				      gs201_bcl_line_names[i], l->irq);
			l->irq = 0;
			goto err_irqs;
		}
	}

	dir = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("status", 0444, dir, bcl, &gs201_bcl_status_fops);
	debugfs_create_file("simulate", 0200, dir, bcl, &gs201_bcl_simulate_fops);
	bcl->debugfs = dir;

	/* A line already asserted raised no edge. */
	mutex_lock(&bcl->lock);
	gs201_bcl_update(bcl);
	mutex_unlock(&bcl->lock);

	dev_info(dev, "listening: %s%s%s%s\n",
		 bcl->line[BCL_SMPL_WARN].gpiod ? "smpl-warn " : "",
		 bcl->line[BCL_VDROOP1].gpiod ? "vdroop1 " : "",
		 bcl->line[BCL_VDROOP2].gpiod ? "vdroop2 " : "",
		 dry_run ? "(dry run)" : "");
	return 0;

err_irqs:
	for (i = 0; i < BCL_NR_LINES; i++)
		if (bcl->line[i].irq > 0)
			free_irq(bcl->line[i].irq, &bcl->line[i]);
	/* A line that fired before a later request failed queued the poll. */
	cancel_delayed_work_sync(&bcl->poll);
err_release:
	gs201_bcl_release(bcl);
	return ret;
}

static void gs201_bcl_remove(struct platform_device *pdev)
{
	struct gs201_bcl *bcl = platform_get_drvdata(pdev);
	int i;

	debugfs_remove_recursive(bcl->debugfs);
	for (i = 0; i < BCL_NR_LINES; i++)
		if (bcl->line[i].irq > 0)
			free_irq(bcl->line[i].irq, &bcl->line[i]);
	cancel_delayed_work_sync(&bcl->poll);
	gs201_bcl_release(bcl);
}

static const struct of_device_id gs201_bcl_of_match[] = {
	{ .compatible = "google,gs201-bcl" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs201_bcl_of_match);

static struct platform_driver gs201_bcl_driver = {
	.probe		= gs201_bcl_probe,
	.remove		= gs201_bcl_remove,
	.driver		= {
		.name		= "gs201-bcl",
		.of_match_table	= gs201_bcl_of_match,
	},
};
module_platform_driver(gs201_bcl_driver);

MODULE_DESCRIPTION("Google Tensor G2 battery current limiter listener");
MODULE_LICENSE("GPL");
