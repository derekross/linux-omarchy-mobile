// SPDX-License-Identifier: GPL-2.0-only
/*
 * Power meters of the Samsung S2MPG12 and S2MPG13 PMICs (Google gs201)
 *
 * Each PMIC has twelve meter channels. A channel samples one of the PMIC's
 * own bucks or LDOs, or the voltage across an external shunt resistor, and
 * adds the power of every sample into a 41-bit accumulator. Setting ASYNC_RD
 * latches all accumulators and the 20-bit sample count into readable
 * registers and clears them. So, between two latches,
 *
 *   average power = acc * resolution / count
 *   energy        = acc * resolution / sample rate
 *
 * The meter can get stuck with its sample count saturated, and then only
 * its software reset in the MT_TRIM block brings it back. So the driver
 * resets it before configuring it, and again whenever a latch shows a
 * saturated count or more samples than the elapsed time allows.
 *
 * The register layout and the per-rail resolutions come from Google's GPL
 * sources (s2mpg1x-meter.h, s2mpg12/13-powermeter.c and odpm.c).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/devm-helpers.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

/* The same meter registers (type 0xa) on S2MPG12 and S2MPG13 */
#define S2MPG12_METER_CTRL1		0x08
#define   S2MPG12_METER_EN		BIT(0)
#define   S2MPG12_EXT_METER_EN		BIT(1)
#define   S2MPG12_INT_SAMP_RATE		GENMASK(4, 2)
#define S2MPG12_METER_CTRL2		0x09
#define   S2MPG12_ASYNC_RD		BIT(7)
#define   S2MPG12_EXT_CHANNEL_EN	GENMASK(5, 3)
#define S2MPG12_METER_CTRL4		0x0b	/* channels 0-7, 0: power */
#define S2MPG12_METER_CTRL5		0x0c
#define   S2MPG12_ACC_MODE_HI		GENMASK(3, 0)	/* channels 8-11 */
#define S2MPG12_METER_BUCKEN1		0x0f	/* and BUCKEN2 */
#define S2MPG12_METER_MUXSEL(ch)	(0x11 + (ch))
#define   S2MPG12_MUXSEL		GENMASK(6, 0)
#define S2MPG12_METER_ACC_DATA(ch)	(0x63 + 6 * (ch))
#define S2MPG12_METER_ACC_COUNT		0xab

/* MT_TRIM block (type 0xe): the register differs, the reset bit does not */
#define S2MPG12_MT_TRIM_METER_RST	BIT(7)
#define S2MPG12_METER_RST_DELAY_US	2

#define S2MPG12_METER_CHANNELS		12
#define S2MPG12_ACC_BITS		41
#define S2MPG12_COUNT_BITS		20

/* 125 Hz: the rate Google uses, and ACC_COUNT then lasts 139 minutes */
#define S2MPG12_INT_125HZ		4
#define S2MPG12_RATE_HZ			125
#define S2MPG12_FLUSH_PERIOD		(30 * 60 * HZ)

#define S2MPG12_INTERVAL_MIN_MS		100
#define S2MPG12_INTERVAL_MAX_MS		60000
#define S2MPG12_INTERVAL_DEF_MS		1000

/* Multiplexer inputs: bucks at 0x01-0x10, LDOn at 0x20 + n */
#define BUCK(n)		(n)
#define BUCKD		0x0b
#define BUCKA		0x0c
#define BUCKC		0x0d
#define BUCKBOOST	0x10
#define LDO(n)		(0x20 + (n))
#define VSEN_C(n)	(0x5c + (n))	/* external shunts, n = 0..2 */
#define MUX_TABLE_SIZE	0x40

/* Power resolution in picowatts per LSB (Google's mW/LSB * 1e9) */
#define CMS		6105006
#define CMD		12210012
#define CMT		18315018
#define VM		12210012
#define BB		10989011
#define DVS_NLDO_150	457875
#define PLDO_150	915751
#define NLDO_300	915751
#define PLDO_300	1831502
#define NLDO_450	1373626
#define PLDO_600	3663004
#define DVS_NLDO_800	1221001
#define NLDO_800	2442002
#define PLDO_800	4884005
#define NLDO_1200	1418251

/*
 * External channels: the resolution is VRAIL (2.1978021 mV/LSB) times
 * VSHUNT (0.7935698 mA/LSB per 10 mOhm) times the trim (8), divided by the
 * shunt resistance: this constant is pW/LSB times micro-ohms.
 */
#define EXT_PW_UOHM		139528749835ULL
#define EXT_SHUNT_MIN_UOHM	1000

static const u32 s2mpg12_res_pw[MUX_TABLE_SIZE] = {
	[BUCK(1)] = CMS,	[BUCK(2)] = CMT,	[BUCK(3)] = CMD,
	[BUCK(4)] = CMS,	[BUCK(5)] = CMT,	[BUCK(6)] = CMS,
	[BUCK(7)] = CMS,	[BUCK(8)] = CMS,	[BUCK(9)] = CMS,
	[BUCK(10)] = CMT,
	[LDO(1)] = NLDO_300,	[LDO(2)] = PLDO_800,	[LDO(3)] = NLDO_300,
	[LDO(4)] = PLDO_150,	[LDO(5)] = DVS_NLDO_150, [LDO(6)] = NLDO_450,
	[LDO(7)] = DVS_NLDO_800, [LDO(8)] = NLDO_300,	[LDO(9)] = PLDO_150,
	[LDO(10)] = PLDO_150,	[LDO(11)] = DVS_NLDO_800, [LDO(12)] = DVS_NLDO_800,
	[LDO(13)] = DVS_NLDO_800, [LDO(14)] = PLDO_150,	[LDO(15)] = DVS_NLDO_800,
	[LDO(16)] = NLDO_800,	[LDO(17)] = DVS_NLDO_800, [LDO(18)] = PLDO_150,
	[LDO(19)] = DVS_NLDO_800, [LDO(20)] = PLDO_150,	[LDO(21)] = PLDO_300,
	[LDO(22)] = DVS_NLDO_800, [LDO(23)] = PLDO_150,	[LDO(24)] = PLDO_600,
	[LDO(25)] = PLDO_150,	[LDO(26)] = PLDO_300,	[LDO(27)] = PLDO_150,
	[LDO(28)] = NLDO_450,
};

static const u32 s2mpg13_res_pw[MUX_TABLE_SIZE] = {
	[BUCK(1)] = CMT,	[BUCK(2)] = CMT,	[BUCK(3)] = CMS,
	[BUCK(4)] = CMS,	[BUCK(5)] = CMS,	[BUCK(6)] = CMS,
	[BUCK(7)] = VM,		[BUCK(8)] = CMS,	[BUCK(9)] = CMS,
	[BUCK(10)] = CMS,	[BUCKD] = VM,		[BUCKA] = VM,
	[BUCKC] = CMS,		[BUCKBOOST] = BB,
	[LDO(1)] = DVS_NLDO_800, [LDO(2)] = NLDO_800,	[LDO(3)] = NLDO_800,
	[LDO(4)] = PLDO_150,	[LDO(5)] = PLDO_150,	[LDO(6)] = PLDO_150,
	[LDO(7)] = PLDO_150,	[LDO(8)] = NLDO_1200,	[LDO(9)] = DVS_NLDO_150,
	[LDO(10)] = PLDO_150,	[LDO(11)] = PLDO_150,	[LDO(12)] = PLDO_300,
	[LDO(13)] = PLDO_150,	[LDO(14)] = PLDO_300,	[LDO(15)] = PLDO_150,
	[LDO(16)] = PLDO_150,	[LDO(17)] = PLDO_150,	[LDO(18)] = PLDO_150,
	[LDO(19)] = PLDO_150,	[LDO(20)] = PLDO_150,	[LDO(21)] = NLDO_800,
	[LDO(22)] = PLDO_150,	[LDO(23)] = DVS_NLDO_800, [LDO(24)] = DVS_NLDO_800,
	[LDO(25)] = DVS_NLDO_800, [LDO(26)] = NLDO_300,	[LDO(27)] = PLDO_300,
	[LDO(28)] = PLDO_150,
};

struct s2mpg12_meter_variant {
	const char *name;
	const u32 *res_pw;
	unsigned int mt_trim_reg;
};

struct s2mpg12_meter_channel {
	const char *label;
	u64 res_pw;		/* picowatts per LSB */
	u64 energy_uj;
	u32 energy_rem_pj;
	u64 power_uw;		/* average over the last latch window */
	u8 mux;
	bool used;
};

struct s2mpg12_meter {
	struct device *dev;
	struct regmap *regmap;
	struct regmap *mt_trim;
	unsigned int mt_trim_reg;
	struct mutex lock;	/* the latch and the channel totals */
	struct delayed_work flush;
	ktime_t last;		/* boot time of the last latch */
	unsigned int interval_ms;
	struct s2mpg12_meter_channel ch[S2MPG12_METER_CHANNELS];
};

static bool s2mpg12_meter_is_ext(u32 mux)
{
	return mux >= VSEN_C(0) && mux <= VSEN_C(2);
}

/* Bit in BUCKEN1/2 that enables metering of a buck, or -1 */
static int s2mpg12_meter_bucken_bit(u8 mux)
{
	if (mux >= BUCK(1) && mux <= BUCKA)
		return mux - 1;
	if (mux == BUCKC)	/* shares BUCKA's bit in Google's gs201 DT */
		return BUCKA - 1;
	if (mux == BUCKBOOST)
		return 12;
	return -1;
}

static void s2mpg12_meter_account(struct s2mpg12_meter_channel *ch, u64 acc,
				  u32 count)
{
	u64 pj;

	pj = mul_u64_u64_div_u64(acc, ch->res_pw, S2MPG12_RATE_HZ);
	pj += ch->energy_rem_pj;
	ch->energy_uj += div_u64_rem(pj, 1000000, &ch->energy_rem_pj);
	if (count)
		ch->power_uw = mul_u64_u64_div_u64(acc, ch->res_pw,
						   (u64)count * 1000000);
}

/* Latch the accumulators and the count, which also clears them, and read them */
static int s2mpg12_meter_snapshot(struct s2mpg12_meter *m, u64 *acc, u32 *count)
{
	unsigned int val;
	u8 buf[6];
	int i, ret;

	ret = regmap_set_bits(m->regmap, S2MPG12_METER_CTRL2, S2MPG12_ASYNC_RD);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(m->regmap, S2MPG12_METER_CTRL2, val,
				       !(val & S2MPG12_ASYNC_RD), 500, 20000);
	if (ret)
		return ret;

	for (i = 0; i < S2MPG12_METER_CHANNELS; i++) {
		if (!m->ch[i].used)
			continue;
		ret = regmap_bulk_read(m->regmap, S2MPG12_METER_ACC_DATA(i),
				       buf, sizeof(buf));
		if (ret)
			return ret;
		acc[i] = ((u64)get_unaligned_le16(&buf[4]) << 32 |
			  get_unaligned_le32(buf)) &
			 GENMASK_ULL(S2MPG12_ACC_BITS - 1, 0);
	}

	ret = regmap_bulk_read(m->regmap, S2MPG12_METER_ACC_COUNT, buf, 3);
	if (ret)
		return ret;
	*count = get_unaligned_le24(buf) & GENMASK(S2MPG12_COUNT_BITS - 1, 0);

	return 0;
}

/*
 * The meter software reset: pulse the reset bit in MT_TRIM, then turn the
 * meter back on. The configuration is written again afterwards.
 */
static int s2mpg12_meter_reset(struct s2mpg12_meter *m)
{
	int ret;

	ret = regmap_clear_bits(m->mt_trim, m->mt_trim_reg,
				S2MPG12_MT_TRIM_METER_RST);
	if (ret)
		return ret;
	ret = regmap_set_bits(m->mt_trim, m->mt_trim_reg,
			      S2MPG12_MT_TRIM_METER_RST);
	if (ret)
		return ret;

	fsleep(S2MPG12_METER_RST_DELAY_US);

	return regmap_set_bits(m->regmap, S2MPG12_METER_CTRL1, S2MPG12_METER_EN);
}

static int s2mpg12_meter_configure(struct s2mpg12_meter *m)
{
	struct regmap *regmap = m->regmap;
	unsigned int meter_en = S2MPG12_METER_EN;
	u16 bucken = 0;
	u8 buf[2];
	int i, bit, ret;

	ret = regmap_update_bits(regmap, S2MPG12_METER_CTRL1,
				 S2MPG12_INT_SAMP_RATE,
				 FIELD_PREP(S2MPG12_INT_SAMP_RATE,
					    S2MPG12_INT_125HZ));
	if (ret)
		return ret;

	for (i = 0; i < S2MPG12_METER_CHANNELS; i++) {
		if (!m->ch[i].used)
			continue;
		ret = regmap_update_bits(regmap, S2MPG12_METER_MUXSEL(i),
					 S2MPG12_MUXSEL, m->ch[i].mux);
		if (ret)
			return ret;
		bit = s2mpg12_meter_bucken_bit(m->ch[i].mux);
		if (bit >= 0)
			bucken |= BIT(bit);
		if (s2mpg12_meter_is_ext(m->ch[i].mux))
			meter_en |= S2MPG12_EXT_METER_EN;
	}

	put_unaligned_le16(bucken, buf);
	ret = regmap_bulk_write(regmap, S2MPG12_METER_BUCKEN1, buf, sizeof(buf));
	if (ret)
		return ret;

	/* Accumulate power, not current, on every channel */
	ret = regmap_write(regmap, S2MPG12_METER_CTRL4, 0);
	if (ret)
		return ret;
	ret = regmap_clear_bits(regmap, S2MPG12_METER_CTRL5,
				S2MPG12_ACC_MODE_HI);
	if (ret)
		return ret;

	/* The external channel enables only change with the ext meter off */
	if (meter_en & S2MPG12_EXT_METER_EN) {
		ret = regmap_clear_bits(regmap, S2MPG12_METER_CTRL1,
					S2MPG12_EXT_METER_EN);
		if (ret)
			return ret;
		ret = regmap_set_bits(regmap, S2MPG12_METER_CTRL2,
				      S2MPG12_EXT_CHANNEL_EN);
		if (ret)
			return ret;
	}

	return regmap_set_bits(regmap, S2MPG12_METER_CTRL1, meter_en);
}

/* Reset and configure the meter, and start a new latch window */
static int s2mpg12_meter_restart(struct s2mpg12_meter *m)
{
	u64 acc[S2MPG12_METER_CHANNELS];
	u32 count;
	int ret;

	ret = s2mpg12_meter_reset(m);
	if (ret)
		return ret;
	ret = s2mpg12_meter_configure(m);
	if (ret)
		return ret;

	m->last = ktime_get_boottime();

	return s2mpg12_meter_snapshot(m, acc, &count);
}

/*
 * A stuck meter shows a saturated count, or more samples than the time
 * since the last latch allows: the datasheet gives the internal clock a
 * 12.5% tolerance, and two more samples (16 ms) cover a slow latch.
 */
static bool s2mpg12_meter_stuck(struct s2mpg12_meter *m, u32 count,
				ktime_t now)
{
	s64 elapsed_ms = ktime_ms_delta(now, m->last);

	if (count == GENMASK(S2MPG12_COUNT_BITS - 1, 0))
		return true;

	return count > div_s64(elapsed_ms * S2MPG12_RATE_HZ * 9, 8 * MSEC_PER_SEC) + 2;
}

/* Latch the accumulators and add them up; restart a stuck meter instead */
static int s2mpg12_meter_latch(struct s2mpg12_meter *m)
{
	u64 acc[S2MPG12_METER_CHANNELS];
	ktime_t now;
	u32 count;
	int i, ret;

	lockdep_assert_held(&m->lock);

	now = ktime_get_boottime();
	ret = s2mpg12_meter_snapshot(m, acc, &count);
	if (ret)
		return ret;

	if (s2mpg12_meter_stuck(m, count, now)) {
		dev_warn_ratelimited(m->dev,
				     "meter stuck (%u samples in %lld ms), resetting it\n",
				     count, ktime_ms_delta(now, m->last));
		return s2mpg12_meter_restart(m);
	}

	for (i = 0; i < S2MPG12_METER_CHANNELS; i++)
		if (m->ch[i].used)
			s2mpg12_meter_account(&m->ch[i], acc[i], count);

	m->last = now;

	return 0;
}

static int s2mpg12_meter_refresh(struct s2mpg12_meter *m)
{
	lockdep_assert_held(&m->lock);

	if (ktime_ms_delta(ktime_get_boottime(), m->last) < m->interval_ms)
		return 0;

	return s2mpg12_meter_latch(m);
}

/* ACC_COUNT saturates after 139 minutes: latch well before that */
static void s2mpg12_meter_flush(struct work_struct *work)
{
	struct s2mpg12_meter *m = container_of(to_delayed_work(work),
					       struct s2mpg12_meter, flush);
	int ret;

	mutex_lock(&m->lock);
	ret = s2mpg12_meter_latch(m);
	mutex_unlock(&m->lock);
	if (ret)
		dev_warn(m->dev, "latching the accumulators failed: %d\n", ret);

	queue_delayed_work(system_power_efficient_wq, &m->flush,
			   S2MPG12_FLUSH_PERIOD);
}

static int s2mpg12_meter_read(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct s2mpg12_meter *m = dev_get_drvdata(dev);
	int ret;

	guard(mutex)(&m->lock);

	switch (type) {
	case hwmon_chip:
		if (attr != hwmon_chip_update_interval)
			return -EOPNOTSUPP;
		*val = m->interval_ms;
		return 0;
	case hwmon_power:
		if (attr != hwmon_power_average)
			return -EOPNOTSUPP;
		ret = s2mpg12_meter_refresh(m);
		if (ret)
			return ret;
		*val = min_t(u64, m->ch[channel].power_uw, LONG_MAX);
		return 0;
	case hwmon_energy64:
		if (attr != hwmon_energy_input)
			return -EOPNOTSUPP;
		ret = s2mpg12_meter_refresh(m);
		if (ret)
			return ret;
		*(s64 *)val = min_t(u64, m->ch[channel].energy_uj, S64_MAX);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int s2mpg12_meter_read_string(struct device *dev,
				     enum hwmon_sensor_types type, u32 attr,
				     int channel, const char **str)
{
	struct s2mpg12_meter *m = dev_get_drvdata(dev);

	*str = m->ch[channel].label;

	return 0;
}

static int s2mpg12_meter_write(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, long val)
{
	struct s2mpg12_meter *m = dev_get_drvdata(dev);

	if (type != hwmon_chip || attr != hwmon_chip_update_interval)
		return -EOPNOTSUPP;

	guard(mutex)(&m->lock);
	m->interval_ms = clamp_val(val, S2MPG12_INTERVAL_MIN_MS,
				   S2MPG12_INTERVAL_MAX_MS);

	return 0;
}

static umode_t s2mpg12_meter_is_visible(const void *data,
					enum hwmon_sensor_types type,
					u32 attr, int channel)
{
	const struct s2mpg12_meter *m = data;

	if (type == hwmon_chip)
		return attr == hwmon_chip_update_interval ? 0644 : 0;

	if (channel >= S2MPG12_METER_CHANNELS || !m->ch[channel].used)
		return 0;

	return 0444;
}

#define S2MPG12_P	(HWMON_P_AVERAGE | HWMON_P_LABEL)
#define S2MPG12_E	(HWMON_E_INPUT | HWMON_E_LABEL)

static const struct hwmon_channel_info * const s2mpg12_meter_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_UPDATE_INTERVAL),
	HWMON_CHANNEL_INFO(power,
			   S2MPG12_P, S2MPG12_P, S2MPG12_P, S2MPG12_P,
			   S2MPG12_P, S2MPG12_P, S2MPG12_P, S2MPG12_P,
			   S2MPG12_P, S2MPG12_P, S2MPG12_P, S2MPG12_P),
	HWMON_CHANNEL_INFO(energy64,
			   S2MPG12_E, S2MPG12_E, S2MPG12_E, S2MPG12_E,
			   S2MPG12_E, S2MPG12_E, S2MPG12_E, S2MPG12_E,
			   S2MPG12_E, S2MPG12_E, S2MPG12_E, S2MPG12_E),
	NULL
};

static const struct hwmon_ops s2mpg12_meter_ops = {
	.is_visible = s2mpg12_meter_is_visible,
	.read = s2mpg12_meter_read,
	.read_string = s2mpg12_meter_read_string,
	.write = s2mpg12_meter_write,
};

static const struct hwmon_chip_info s2mpg12_meter_chip_info = {
	.ops = &s2mpg12_meter_ops,
	.info = s2mpg12_meter_info,
};

static int s2mpg12_meter_parse(struct s2mpg12_meter *m,
			       const struct s2mpg12_meter_variant *variant)
{
	struct device *dev = m->dev;
	struct s2mpg12_meter_channel *ch;
	u32 reg, mux, shunt;
	int ret;

	/* Boards that do not describe the meter leave it alone */
	if (!dev_fwnode(dev))
		return -ENODEV;

	device_for_each_child_node_scoped(dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret || reg >= S2MPG12_METER_CHANNELS)
			return dev_err_probe(dev, -EINVAL, "%pfw: bad reg\n",
					     child);
		ch = &m->ch[reg];
		if (ch->used)
			return dev_err_probe(dev, -EINVAL,
					     "channel %u described twice\n", reg);

		ret = fwnode_property_read_u32(child, "samsung,source", &mux);
		if (ret)
			return dev_err_probe(dev, ret, "%pfw: no samsung,source\n",
					     child);

		if (s2mpg12_meter_is_ext(mux)) {
			ret = fwnode_property_read_u32(child,
						       "shunt-resistor-micro-ohms",
						       &shunt);
			if (ret || shunt < EXT_SHUNT_MIN_UOHM)
				return dev_err_probe(dev, -EINVAL,
						     "%pfw: bad shunt resistor\n",
						     child);
			ch->res_pw = div_u64(EXT_PW_UOHM, shunt);
		} else if (mux < MUX_TABLE_SIZE && variant->res_pw[mux]) {
			ch->res_pw = variant->res_pw[mux];
		} else {
			return dev_err_probe(dev, -EINVAL,
					     "%pfw: unsupported source %#x\n",
					     child, mux);
		}

		if (fwnode_property_read_string(child, "label", &ch->label))
			ch->label = devm_kasprintf(dev, GFP_KERNEL, "%s_ch%u",
						   variant->name, reg);
		if (!ch->label)
			return -ENOMEM;

		ch->mux = mux;
		ch->used = true;
	}

	for (reg = 0; reg < S2MPG12_METER_CHANNELS; reg++)
		if (m->ch[reg].used)
			return 0;

	return dev_err_probe(dev, -ENODEV, "no meter channels described\n");
}

static int s2mpg12_meter_probe(struct platform_device *pdev)
{
	const struct s2mpg12_meter_variant *variant;
	struct device *dev = &pdev->dev;
	struct s2mpg12_meter *m;
	struct device *hwmon;
	int ret;

	/* matched by the cell's DT node (and so autoloaded), or by its name */
	variant = device_get_match_data(&pdev->dev);
	if (!variant)
		variant = (const void *)platform_get_device_id(pdev)->driver_data;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->dev = dev;
	m->interval_ms = S2MPG12_INTERVAL_DEF_MS;

	ret = devm_mutex_init(dev, &m->lock);
	if (ret)
		return ret;

	m->regmap = dev_get_regmap(dev->parent, "meter");
	if (!m->regmap)
		return dev_err_probe(dev, -ENODEV, "no meter regmap\n");

	m->mt_trim = dev_get_regmap(dev->parent, "mt-trim");
	if (!m->mt_trim)
		return dev_err_probe(dev, -ENODEV, "no mt-trim regmap\n");
	m->mt_trim_reg = variant->mt_trim_reg;

	ret = s2mpg12_meter_parse(m, variant);
	if (ret)
		return ret;

	/*
	 * Reset the meter, which a previous boot may have left stuck,
	 * configure it, and throw away whatever was accumulated before.
	 */
	mutex_lock(&m->lock);
	ret = s2mpg12_meter_restart(m);
	mutex_unlock(&m->lock);
	if (ret)
		return dev_err_probe(dev, ret, "failed to start the meter\n");

	ret = devm_delayed_work_autocancel(dev, &m->flush, s2mpg12_meter_flush);
	if (ret)
		return ret;
	queue_delayed_work(system_power_efficient_wq, &m->flush,
			   S2MPG12_FLUSH_PERIOD);

	hwmon = devm_hwmon_device_register_with_info(dev, variant->name, m,
						     &s2mpg12_meter_chip_info,
						     NULL);

	return PTR_ERR_OR_ZERO(hwmon);
}

static const struct s2mpg12_meter_variant s2mpg12_variant = {
	.name = "s2mpg12",
	.res_pw = s2mpg12_res_pw,
	.mt_trim_reg = 0x29,
};

static const struct s2mpg12_meter_variant s2mpg13_variant = {
	.name = "s2mpg13",
	.res_pw = s2mpg13_res_pw,
	.mt_trim_reg = 0x34,
};

static const struct platform_device_id s2mpg12_meter_id[] = {
	{ "s2mpg12-meter", (kernel_ulong_t)&s2mpg12_variant },
	{ "s2mpg13-meter", (kernel_ulong_t)&s2mpg13_variant },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpg12_meter_id);

/*
 * The mfd cells carry the meter's DT node, so the device's modalias is the OF
 * one: without this table the module isn't loaded by itself.
 */
static const struct of_device_id s2mpg12_meter_of_match[] = {
	{ .compatible = "samsung,s2mpg12-meter", .data = &s2mpg12_variant },
	{ .compatible = "samsung,s2mpg13-meter", .data = &s2mpg13_variant },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpg12_meter_of_match);

static struct platform_driver s2mpg12_meter_driver = {
	.driver = {
		.name = "s2mpg12-meter",
		.of_match_table = s2mpg12_meter_of_match,
	},
	.probe = s2mpg12_meter_probe,
	.id_table = s2mpg12_meter_id,
};
module_platform_driver(s2mpg12_meter_driver);

MODULE_DESCRIPTION("Samsung S2MPG12/S2MPG13 power meter driver");
MODULE_LICENSE("GPL");
