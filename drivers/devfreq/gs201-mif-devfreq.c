// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google gs201 memory interface (MIF) frequency scaling.
 *
 * The MIF clock (DRAM controller and PHY) is an ACPM DVFS domain. Left
 * alone it stays at the rate the bootloader sets, 2028 MHz. gs201 has no
 * mainline driver for its bus traffic counters, so this driver estimates
 * the bandwidth needed from what the bus masters are doing: each CPU
 * cluster asks for a MIF rate that goes with its frequency, but only while
 * one of its CPUs is busy, and the GPU asks for one that goes with its
 * clock while it is powered. The highest request wins; going up is
 * immediate, going down waits for GS201_MIF_HOLD_MS.
 *
 * Without the busy condition the little cores, which ramp up for every
 * display frame while mostly idle, keep the MIF at 2028 MHz most of the
 * time.
 *
 * Other consumers of the MIF clock keep their floors: the display
 * controller asks for a minimum with clk_set_min_rate() at high refresh
 * rates, and the clock core never sets a rate below it.
 */

#include <linux/array_size.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/devfreq.h>
#include <linux/devfreq-governor.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/units.h>

/* A cluster asks for MIF only if its busiest CPU was busy this share, %. */
#define GS201_MIF_BUSY_PCT	40
/* Time the requests must stay lower before the rate goes down. */
#define GS201_MIF_HOLD_MS	200
/* How often the cached MIF rate is read back from the firmware. */
#define GS201_MIF_RESYNC_MS	1000

/**
 * struct gs201_mif_vote - one row of a "from this rate up" table
 * @from_khz:	the bus master's rate, in kHz
 * @mif_khz:	the lowest MIF rate it needs from there up, in kHz
 */
struct gs201_mif_vote {
	unsigned int from_khz;
	unsigned int mif_khz;
};

/**
 * struct gs201_mif_cluster - a CPU cluster and its MIF table
 * @first_cpu:	first CPU of the cluster
 * @nr_cpus:	number of CPUs in the cluster
 * @votes:	MIF needed for each cluster frequency, ascending
 * @nr_votes:	number of rows in @votes
 */
struct gs201_mif_cluster {
	unsigned int first_cpu;
	unsigned int nr_cpus;
	const struct gs201_mif_vote *votes;
	unsigned int nr_votes;
};

/*
 * CPU frequency to MIF rate, from the memory latency tables of the vendor
 * kernel's gs201 device tree (gs_memlat_cl0/1/2).
 */
static const struct gs201_mif_vote gs201_cl0_votes[] = {
	{   10000,  421000 }, {  300001,  546000 }, {  574001,  676000 },
	{  738001,  845000 }, {  930001, 1014000 }, { 1098001, 1352000 },
	{ 1197001, 1539000 }, { 1328001, 1716000 }, { 1598001, 2028000 },
	{ 1803001, 2535000 },
};

static const struct gs201_mif_vote gs201_cl1_votes[] = {
	{   10000,  421000 }, {  400001,  546000 }, {  553001,  676000 },
	{  696001,  845000 }, {  799001, 1014000 }, { 1024001, 1716000 },
	{ 1491001, 2028000 }, { 1836001, 2288000 }, { 2130001, 2730000 },
};

static const struct gs201_mif_vote gs201_cl2_votes[] = {
	{   10000,  845000 }, {  100001, 1014000 }, {  851001, 1352000 },
	{ 1106001, 1539000 }, { 1277001, 1716000 }, { 1582001, 2028000 },
	{ 1826001, 2288000 }, { 2048001, 2730000 }, { 2252001, 3172000 },
};

static const struct gs201_mif_cluster gs201_clusters[] = {
	{ 0, 4, gs201_cl0_votes, ARRAY_SIZE(gs201_cl0_votes) },
	{ 4, 2, gs201_cl1_votes, ARRAY_SIZE(gs201_cl1_votes) },
	{ 6, 2, gs201_cl2_votes, ARRAY_SIZE(gs201_cl2_votes) },
};

/*
 * GPU shader clock to MIF rate. The vendor GPU table only pins MIF for its
 * top levels and relies on bus counters for the rest; these rows are
 * measured instead: glmark2 lost 7% with the GPU at 572 MHz and the MIF at
 * 1352 MHz, and lost nothing with them.
 */
static const struct gs201_mif_vote gs201_gpu_votes[] = {
	{ 251000, 1352000 }, { 351000, 1539000 }, { 510000, 2028000 },
	{ 848000, 3172000 },
};

struct gs201_mif {
	struct devfreq *devfreq;
	struct clk *mif_clk;
	struct clk *gpu_clk;
	struct device *gpu_dev;
	unsigned long boot_rate;
	unsigned long hold_until;
	unsigned long next_resync;
	u64 *prev_idle;
	u64 *prev_wall;
};

static unsigned int gs201_mif_vote(const struct gs201_mif_vote *v,
				   unsigned int n, unsigned int khz)
{
	unsigned int mif = 0, i;

	for (i = 0; i < n; i++)
		if (khz >= v[i].from_khz)
			mif = v[i].mif_khz;

	return mif;
}

/* Busy share of the busiest CPU of @c since the last call, in %. */
static unsigned int gs201_mif_cluster_busy(struct gs201_mif *mif,
					   const struct gs201_mif_cluster *c)
{
	unsigned int cpu, busy = 0;

	for (cpu = c->first_cpu; cpu < c->first_cpu + c->nr_cpus; cpu++) {
		u64 wall, idle, dw, di;

		if (cpu >= nr_cpu_ids)
			break;
		idle = get_cpu_idle_time(cpu, &wall, 0);
		dw = wall - mif->prev_wall[cpu];
		di = idle - mif->prev_idle[cpu];

		mif->prev_wall[cpu] = wall;
		mif->prev_idle[cpu] = idle;
		if (dw && di < dw)
			busy = max_t(unsigned int, busy,
				     div64_u64(100 * (dw - di), dw));
	}

	return busy;
}

static unsigned int gs201_mif_gpu_khz(struct gs201_mif *mif)
{
	/* A suspended GPU keeps its last rate but makes no traffic. */
	if (!mif->gpu_clk || !mif->gpu_dev ||
	    pm_runtime_status_suspended(mif->gpu_dev))
		return 0;

	return clk_get_rate(mif->gpu_clk) / HZ_PER_KHZ;
}

static int gs201_mif_get_target(struct devfreq *devfreq, unsigned long *freq)
{
	struct gs201_mif *mif = dev_get_drvdata(devfreq->dev.parent);
	unsigned long now = jiffies, want;
	unsigned int khz = 0, i;

	for (i = 0; i < ARRAY_SIZE(gs201_clusters); i++) {
		const struct gs201_mif_cluster *c = &gs201_clusters[i];

		if (gs201_mif_cluster_busy(mif, c) < GS201_MIF_BUSY_PCT)
			continue;
		khz = max(khz, gs201_mif_vote(c->votes, c->nr_votes,
					      cpufreq_quick_get(c->first_cpu)));
	}
	khz = max(khz, gs201_mif_vote(gs201_gpu_votes,
				      ARRAY_SIZE(gs201_gpu_votes),
				      gs201_mif_gpu_khz(mif)));

	want = (unsigned long)khz * HZ_PER_KHZ;
	if (want >= devfreq->previous_freq)
		mif->hold_until = now + msecs_to_jiffies(GS201_MIF_HOLD_MS);
	else if (time_before(now, mif->hold_until))
		want = devfreq->previous_freq;

	/* The devfreq core rounds up to an OPP and clamps to min/max. */
	*freq = want;

	return 0;
}

static int gs201_mif_governor_event(struct devfreq *devfreq,
				    unsigned int event, void *data)
{
	switch (event) {
	case DEVFREQ_GOV_START:
		devfreq_monitor_start(devfreq);
		break;
	case DEVFREQ_GOV_STOP:
		devfreq_monitor_stop(devfreq);
		break;
	case DEVFREQ_GOV_UPDATE_INTERVAL:
		devfreq_update_interval(devfreq, data);
		break;
	case DEVFREQ_GOV_SUSPEND:
		devfreq_monitor_suspend(devfreq);
		break;
	case DEVFREQ_GOV_RESUME:
		devfreq_monitor_resume(devfreq);
		break;
	}

	return 0;
}

static struct devfreq_governor gs201_mif_governor = {
	.name = "gs201_mif",
	.attrs = DEVFREQ_GOV_ATTR_POLLING_INTERVAL |
		 DEVFREQ_GOV_ATTR_TIMER,
	.flags = DEVFREQ_GOV_FLAG_IMMUTABLE,
	.get_target_freq = gs201_mif_get_target,
	.event_handler = gs201_mif_governor_event,
};

static int gs201_mif_target(struct device *dev, unsigned long *freq,
			    u32 flags)
{
	struct gs201_mif *mif = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);

	/*
	 * The clock has CLK_GET_RATE_NOCACHE, but clk_set_rate() compares
	 * with the cached rate: refresh it now and then, in case the
	 * firmware changed the rate over a suspend.
	 */
	if (time_after_eq(jiffies, mif->next_resync)) {
		clk_get_rate(mif->mif_clk);
		mif->next_resync = jiffies +
				   msecs_to_jiffies(GS201_MIF_RESYNC_MS);
	}

	/*
	 * Asked every period: when the display controller drops its floor
	 * the rate stays where the floor had it, and this brings it down.
	 * While the floor holds, or the rate is already right, the clock
	 * core returns without asking the firmware.
	 */
	return clk_set_rate(mif->mif_clk, *freq);
}

static struct devfreq_dev_profile gs201_mif_profile = {
	.polling_ms = 50,
	.target = gs201_mif_target,
};

static void gs201_mif_restore(void *data)
{
	struct gs201_mif *mif = data;

	/* Refresh the cached rate first: clk_set_rate() compares with it. */
	clk_get_rate(mif->mif_clk);
	clk_set_rate(mif->mif_clk, mif->boot_rate);
}

static void gs201_mif_put_gpu(void *data)
{
	put_device(data);
}

static int gs201_mif_get_gpu(struct device *dev, struct gs201_mif *mif)
{
	struct platform_device *pdev;
	struct device_node *np;
	int ret;

	mif->gpu_clk = devm_clk_get_optional(dev, "gpu");
	if (IS_ERR(mif->gpu_clk))
		return dev_err_probe(dev, PTR_ERR(mif->gpu_clk),
				     "failed to get the GPU clock\n");

	np = of_parse_phandle(dev->of_node, "google,gpu", 0);
	if (!np)
		return 0;
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -EPROBE_DEFER;

	mif->gpu_dev = &pdev->dev;
	ret = devm_add_action_or_reset(dev, gs201_mif_put_gpu, mif->gpu_dev);
	if (ret)
		return ret;

	return 0;
}

static int gs201_mif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct devfreq *devfreq;
	struct dev_pm_opp *opp;
	struct gs201_mif *mif;
	unsigned long rate;
	int ret;

	mif = devm_kzalloc(dev, sizeof(*mif), GFP_KERNEL);
	if (!mif)
		return -ENOMEM;
	platform_set_drvdata(pdev, mif);

	mif->prev_idle = devm_kcalloc(dev, nr_cpu_ids, sizeof(u64), GFP_KERNEL);
	mif->prev_wall = devm_kcalloc(dev, nr_cpu_ids, sizeof(u64), GFP_KERNEL);
	if (!mif->prev_idle || !mif->prev_wall)
		return -ENOMEM;

	mif->mif_clk = devm_clk_get(dev, "mif");
	if (IS_ERR(mif->mif_clk))
		return dev_err_probe(dev, PTR_ERR(mif->mif_clk),
				     "failed to get the MIF clock\n");

	ret = gs201_mif_get_gpu(dev, mif);
	if (ret)
		return ret;

	ret = devm_pm_opp_of_add_table(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add the OPP table\n");

	mif->boot_rate = clk_get_rate(mif->mif_clk);
	rate = mif->boot_rate;
	opp = devfreq_recommended_opp(dev, &rate, 0);
	if (IS_ERR(opp))
		return dev_err_probe(dev, PTR_ERR(opp), "no OPP for %lu Hz\n",
				     mif->boot_rate);
	dev_pm_opp_put(opp);
	gs201_mif_profile.initial_freq = rate;
	mif->next_resync = jiffies;

	/* Leave the MIF at the boot rate when the driver goes away. */
	ret = devm_add_action_or_reset(dev, gs201_mif_restore, mif);
	if (ret)
		return ret;

	ret = devm_devfreq_add_governor(dev, &gs201_mif_governor);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add the governor\n");

	devfreq = devm_devfreq_add_device(dev, &gs201_mif_profile,
					  gs201_mif_governor.name, NULL);
	if (IS_ERR(devfreq))
		return dev_err_probe(dev, PTR_ERR(devfreq),
				     "failed to add the devfreq device\n");
	mif->devfreq = devfreq;

	return 0;
}

/*
 * Restart at the rate the bootloader left, as stock does with its
 * reboot_freq: a reboot never runs the devm action above. Stop the governor
 * first so its next period doesn't lower the rate again.
 */
static void gs201_mif_shutdown(struct platform_device *pdev)
{
	struct gs201_mif *mif = platform_get_drvdata(pdev);

	if (!mif || !mif->devfreq)
		return;

	devfreq_suspend_device(mif->devfreq);
	gs201_mif_restore(mif);
}

static const struct of_device_id gs201_mif_of_match[] = {
	{ .compatible = "google,gs201-mif" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs201_mif_of_match);

static struct platform_driver gs201_mif_driver = {
	.probe = gs201_mif_probe,
	.shutdown = gs201_mif_shutdown,
	.driver = {
		.name = "gs201-mif-devfreq",
		.of_match_table = gs201_mif_of_match,
	},
};
module_platform_driver(gs201_mif_driver);

MODULE_DESCRIPTION("Google gs201 memory interface frequency scaling");
MODULE_LICENSE("GPL");
