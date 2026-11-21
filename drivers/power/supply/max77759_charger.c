// SPDX-License-Identifier: GPL-2.0-only
/*
 * max77759_charger.c - Battery charger driver for MAX77759 charger device.
 *
 * Copyright 2025 Google LLC.
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/devm-helpers.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/linear_range.h>
#include <linux/math64.h>
#include <linux/mfd/max77759.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/string_choices.h>
#include <linux/workqueue.h>

/* Default values for Fast Charge Current & Float Voltage */
#define CHG_CC_DEFAULT_UA			2266770
#define CHG_FV_DEFAULT_MV			4300

#define MAX_NUM_RETRIES				3
#define PSY_WORK_RETRY_DELAY_MS			10

/* USB 3.x default power, the most a port must supply without a Type-C advertisement */
#define CHG_DEFAULT_ILIM_MAX_UA			900000

static unsigned int default_ilim_ua;
module_param(default_ilim_ua, uint, 0444);
MODULE_PARM_DESC(default_ilim_ua,
		 "Input current limit in uA for a source that advertises no current: used from probe until a tcpm-source reports, and for Type-C default power. 0 (default) keeps charging off in both cases. At most 900000.");

/*
 * Battery temperature guard: the step-charging policy of the stock phone
 * (Google's google,battery node on gs201 phones: chg-temp-limits,
 * chg-cv-limits, chg-cc-limits). Off by default.
 *
 * The temperature bands are the same on every board; the voltage tiers and
 * the currents come from a table per board, see guard_select_table().
 */
static bool temp_guard;
module_param(temp_guard, bool, 0644);
MODULE_PARM_DESC(temp_guard,
		 "Limit charging by battery temperature as the stock phone does: no charge below 0 or from 55 C, lower float voltage and current outside 10-46 C. Reads the battery power supply named by guard_psy every 30 s while awake: s2idle stops the guard, and the charge thresholds with it. Default off.");

static char guard_psy[32] = "max170xx_battery";
module_param_string(guard_psy, guard_psy, sizeof(guard_psy), 0444);
MODULE_PARM_DESC(guard_psy, "Battery power supply whose temp and voltage_now the guard reads");

#define STOP_RESUME_GAP			5	/* resume at end - 5 when start is 0 */

#define GUARD_PERIOD_MS			30000
#define GUARD_RETRY_MS			2000	/* while there is no reading */
#define GUARD_HYST_DECIC		20	/* 2.0 C into a band before relaxing */
#define GUARD_NBANDS			6
#define GUARD_MAX_TIERS			4

/* Band i is [guard_temp_decic[i], guard_temp_decic[i + 1]), in 0.1 C */
static const int guard_temp_decic[GUARD_NBANDS + 1] = {
	0, 100, 200, 420, 460, 480, 550
};

/**
 * struct guard_table - step-charging table of one battery
 * @ntiers: number of voltage tiers in use, at most GUARD_MAX_TIERS
 * @cv_mv: tier j charges up to cv_mv[j]
 * @cc_pct: charge current per band and tier, in percent of the design
 *	    capacity; 0 means the band does not charge in that tier
 */
struct guard_table {
	int ntiers;
	u32 cv_mv[GUARD_MAX_TIERS];
	u8 cc_pct[GUARD_NBANDS][GUARD_MAX_TIERS];
};

/* Google's gs201 table, for every board without a table of its own */
static const struct guard_table guard_table_default = {
	.ntiers = 3,
	.cv_mv = { 4200, 4300, 4450 },
	.cc_pct = {
		{  30, 10,  0 },	/*  0-10 C */
		{  50, 30, 30 },	/* 10-20 C */
		{ 100, 70, 50 },	/* 20-42 C */
		{  80, 50, 50 },	/* 42-46 C */
		{  50, 50,  0 },	/* 46-48 C */
		{  30,  0,  0 },	/* 48-55 C */
	},
};

/*
 * Pixel 8 Pro (husky): the google,battery node in Google's
 * kernel/devices/google/shusky, branch android-gs-shusky-6.1-android16:
 * temperature and voltage limits from dts/zuma-shusky-battery.dtsi,
 * the 4950 mAh and the six rows from dts/zuma-husky-battery.dtsi.
 */
static const struct guard_table guard_table_husky = {
	.ntiers = 4,
	.cv_mv = { 4200, 4300, 4350, 4480 },
	.cc_pct = {
		{  50,  30,   0,  0 },	/*  0-10 C */
		{  70,  30,  30, 30 },	/* 10-20 C */
		{ 120, 120, 120, 50 },	/* 20-42 C */
		{ 100, 100, 100, 50 },	/* 42-46 C */
		{  50,  50,   0,  0 },	/* 46-48 C */
		{  30,   0,   0,  0 },	/* 48-55 C */
	},
};

#define FOREACH_IRQ(S)			\
	S(AICL),			\
	S(CHGIN),			\
	S(CHG),				\
	S(INLIM),			\
	S(BAT_OILO),			\
	S(CHG_STA_CC),			\
	S(CHG_STA_CV),			\
	S(CHG_STA_TO),			\
	S(CHG_STA_DONE)

#define GENERATE_ENUM(e)		e
#define GENERATE_STRING(s)		#s

enum {
	FOREACH_IRQ(GENERATE_ENUM)
};

static const char *const chgr_irqs_str[] = {
	FOREACH_IRQ(GENERATE_STRING)
};

#define NUM_IRQS			ARRAY_SIZE(chgr_irqs_str)

/* Fast charge current limits (in uA) */
static const struct linear_range chgcc_limit_ranges[] = {
	LINEAR_RANGE(133330, 0x0, 0x2, 0),
	LINEAR_RANGE(200000, 0x3, 0x3C, 66670),
};

/* Charge Termination Voltage Limits (in mV) */
static const struct linear_range chg_cv_prm_ranges[] = {
	LINEAR_RANGE(3800, 0x38, 0x39, 100),
	LINEAR_RANGE(4000, 0x0, 0x32, 10),
};

/* USB input current limits (in uA) */
static const struct linear_range chgin_ilim_ranges[] = {
	LINEAR_RANGE(100000, 0x3, 0x7F, 25000),
};

struct max77759_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct regulator_dev *chgin_otg_rdev;
	struct notifier_block nb;
	struct power_supply *tcpm_psy;
	struct delayed_work psy_work;
	struct mutex retry_lock; /* Protects psy_work_retry_cnt */
	u32 psy_work_retry_cnt;
	int irqs[NUM_IRQS];
	struct mutex lock; /* protects the state below */
	enum max77759_chgr_mode mode;
	bool inhibit;	/* charge suspended by the temperature guard */
	bool stop_inhibit;	/* charge suspended at the end threshold */
	const char *bat_psy;	/* capacity source, NULL: guard_psy */

	/* Charge thresholds, in percent, run by the guard work */
	int end_threshold;
	int start_threshold;
	bool apply_failed;	/* the chip did not hold the guard's limits */

	/* Temperature guard */
	struct delayed_work guard_work;
	bool guard_active;
	int guard_band;		/* -1: charging inhibited */
	bool guard_no_reading;	/* inhibited for want of a reading */
	int guard_tier;
	const struct guard_table *guard_table;
	u32 fv_max_mv;		/* limits from the battery description */
	u32 cc_max_ua;
	u32 design_uah;
};

static inline int unlock_prot_regs(struct max77759_charger *chg, bool unlock)
{
	return regmap_update_bits(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_06,
				  MAX77759_CHGR_REG_CHG_CNFG_06_CHGPROT, unlock
				  ? MAX77759_CHGR_REG_CHG_CNFG_06_CHGPROT : 0);
}

static int charger_input_valid(struct max77759_charger *chg)
{
	u32 val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77759_CHGR_REG_CHG_INT_OK, &val);
	if (ret)
		return ret;

	/*
	 * CHGIN alone says whether the input is there. CHG_OK clears when a
	 * charge is done (and on charger faults), which the status property
	 * already reports.
	 */
	return !!(val & MAX77759_CHGR_REG_CHG_INT_CHGIN);
}

static int get_online(struct max77759_charger *chg)
{
	u32 val;
	int ret;

	ret = charger_input_valid(chg);
	if (ret <= 0)
		return ret;

	ret = regmap_read(chg->regmap, MAX77759_CHGR_REG_CHG_DETAILS_02, &val);
	if (ret)
		return ret;

	guard(mutex)(&chg->lock);

	return (val & MAX77759_CHGR_REG_CHG_DETAILS_02_CHGIN_STS) &&
		(chg->mode == MAX77759_CHGR_MODE_CHG_BUCK_ON);
}

static int get_status(struct max77759_charger *chg)
{
	u32 val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77759_CHGR_REG_CHG_DETAILS_01, &val);
	if (ret)
		return ret;

	switch (FIELD_GET(MAX77759_CHGR_REG_CHG_DETAILS_01_CHG_DTLS, val)) {
	case MAX77759_CHGR_CHG_DTLS_PREQUAL:
	case MAX77759_CHGR_CHG_DTLS_CC:
	case MAX77759_CHGR_CHG_DTLS_CV:
	case MAX77759_CHGR_CHG_DTLS_TO:
		return POWER_SUPPLY_STATUS_CHARGING;
	case MAX77759_CHGR_CHG_DTLS_DONE:
		return POWER_SUPPLY_STATUS_FULL;
	case MAX77759_CHGR_CHG_DTLS_TIMER_FAULT:
	case MAX77759_CHGR_CHG_DTLS_SUSP_BATT_THM:
	case MAX77759_CHGR_CHG_DTLS_OFF_WDOG_TIMER:
	case MAX77759_CHGR_CHG_DTLS_SUSP_JEITA:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case MAX77759_CHGR_CHG_DTLS_OFF:
		/*
		 * Buck only with the cable in, while the temperature guard or
		 * a charge threshold holds the charge: not charging, the
		 * battery is not what feeds the phone. With the cable out it
		 * is discharging, whatever the flags say.
		 */
		if (charger_input_valid(chg) <= 0)
			return POWER_SUPPLY_STATUS_DISCHARGING;
		scoped_guard(mutex, &chg->lock) {
			if (chg->mode == MAX77759_CHGR_MODE_CHG_BUCK_ON &&
			    (chg->inhibit || chg->stop_inhibit))
				return POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		return POWER_SUPPLY_STATUS_DISCHARGING;
	default:
		break;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static int get_charge_type(struct max77759_charger *chg)
{
	u32 val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77759_CHGR_REG_CHG_DETAILS_01, &val);
	if (ret)
		return ret;

	switch (FIELD_GET(MAX77759_CHGR_REG_CHG_DETAILS_01_CHG_DTLS, val)) {
	case MAX77759_CHGR_CHG_DTLS_PREQUAL:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case MAX77759_CHGR_CHG_DTLS_CC:
	case MAX77759_CHGR_CHG_DTLS_CV:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case MAX77759_CHGR_CHG_DTLS_TO:
		return POWER_SUPPLY_CHARGE_TYPE_STANDARD;
	case MAX77759_CHGR_CHG_DTLS_DONE:
	case MAX77759_CHGR_CHG_DTLS_TIMER_FAULT:
	case MAX77759_CHGR_CHG_DTLS_SUSP_BATT_THM:
	case MAX77759_CHGR_CHG_DTLS_OFF_WDOG_TIMER:
	case MAX77759_CHGR_CHG_DTLS_SUSP_JEITA:
	case MAX77759_CHGR_CHG_DTLS_OFF:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		break;
	}

	return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
}

static int get_chg_health(struct max77759_charger *chg)
{
	u32 val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77759_CHGR_REG_CHG_DETAILS_00, &val);
	if (ret)
		return ret;

	switch (FIELD_GET(MAX77759_CHGR_REG_CHG_DETAILS_00_CHGIN_DTLS, val)) {
	case MAX77759_CHGR_CHGIN_DTLS_VBUS_UNDERVOLTAGE:
	case MAX77759_CHGR_CHGIN_DTLS_VBUS_MARGINAL_VOLTAGE:
		return POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
	case MAX77759_CHGR_CHGIN_DTLS_VBUS_OVERVOLTAGE:
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	case MAX77759_CHGR_CHGIN_DTLS_VBUS_VALID:
		return POWER_SUPPLY_HEALTH_GOOD;
	default:
		break;
	}

	return POWER_SUPPLY_HEALTH_UNKNOWN;
}

static int get_batt_health(struct max77759_charger *chg)
{
	u32 val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77759_CHGR_REG_CHG_DETAILS_01, &val);
	if (ret)
		return ret;

	switch (FIELD_GET(MAX77759_CHGR_REG_CHG_DETAILS_01_BAT_DTLS, val)) {
	case MAX77759_CHGR_BAT_DTLS_NO_BATT_CHG_SUSP:
		return POWER_SUPPLY_HEALTH_NO_BATTERY;
	case MAX77759_CHGR_BAT_DTLS_DEAD_BATTERY:
		return POWER_SUPPLY_HEALTH_DEAD;
	case MAX77759_CHGR_BAT_DTLS_BAT_CHG_TIMER_FAULT:
		return POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
	case MAX77759_CHGR_BAT_DTLS_BAT_OKAY:
	case MAX77759_CHGR_BAT_DTLS_BAT_ONLY_MODE:
		return POWER_SUPPLY_HEALTH_GOOD;
	case MAX77759_CHGR_BAT_DTLS_BAT_UNDERVOLTAGE:
		return POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
	case MAX77759_CHGR_BAT_DTLS_BAT_OVERVOLTAGE:
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	case MAX77759_CHGR_BAT_DTLS_BAT_OVERCURRENT:
		return POWER_SUPPLY_HEALTH_OVERCURRENT;
	default:
		break;
	}

	return POWER_SUPPLY_HEALTH_UNKNOWN;
}

static int get_health(struct max77759_charger *chg)
{
	int ret;

	ret = get_online(chg);
	if (ret < 0)
		return ret;

	if (ret) {
		ret = get_chg_health(chg);
		if (ret < 0 || ret != POWER_SUPPLY_HEALTH_GOOD)
			return ret;
	}

	return get_batt_health(chg);
}

static int get_fast_charge_current(struct max77759_charger *chg)
{
	u32 regval, val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_02, &regval);
	if (ret)
		return ret;

	regval = FIELD_GET(MAX77759_CHGR_REG_CHG_CNFG_02_CHGCC, regval);
	ret = linear_range_get_value_array(chgcc_limit_ranges,
					   ARRAY_SIZE(chgcc_limit_ranges),
					   regval, &val);
	return ret ? ret : val;
}

/*
 * The fast charge current step at or below @cc_ua, never one above it. A
 * request below the lowest step or above the highest (4000190 uA) is
 * -EINVAL, not clamped: a battery node without
 * constant-charge-current-max-microamp leaves -EINVAL in the field, which
 * must stay a failed probe and not become the top current.
 */
static int cc_selector(u32 cc_ua, u32 *sel)
{
	const struct linear_range *top =
		&chgcc_limit_ranges[ARRAY_SIZE(chgcc_limit_ranges) - 1];
	bool found;

	if (cc_ua > linear_range_get_max_value(top) ||
	    linear_range_get_selector_low_array(chgcc_limit_ranges,
						ARRAY_SIZE(chgcc_limit_ranges),
						cc_ua, sel, &found))
		return -EINVAL;

	return 0;
}

static int set_fast_charge_current_limit(struct max77759_charger *chg,
					 u32 cc_max_ua)
{
	u32 regval;

	if (cc_selector(cc_max_ua, &regval))
		return -EINVAL;

	return regmap_update_bits(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_02,
				  MAX77759_CHGR_REG_CHG_CNFG_02_CHGCC, regval);
}

static int get_float_voltage(struct max77759_charger *chg)
{
	u32 regval, val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_04, &regval);
	if (ret)
		return ret;

	regval = FIELD_GET(MAX77759_CHGR_REG_CHG_CNFG_04_CHG_CV_PRM, regval);
	ret = linear_range_get_value_array(chg_cv_prm_ranges,
					   ARRAY_SIZE(chg_cv_prm_ranges),
					   regval, &val);

	return ret ? ret : val;
}

/*
 * The float voltage step at or below @fv_mv, never one above it. Between
 * the two ranges (3900 to 4000 mV) that is 3900 mV. A request below the
 * lowest step or above the highest is -EINVAL.
 */
static int fv_selector(u32 fv_mv, u32 *sel)
{
	const struct linear_range *top =
		&chg_cv_prm_ranges[ARRAY_SIZE(chg_cv_prm_ranges) - 1];
	bool found;

	if (fv_mv > linear_range_get_max_value(top) ||
	    linear_range_get_selector_low_array(chg_cv_prm_ranges,
						ARRAY_SIZE(chg_cv_prm_ranges),
						fv_mv, sel, &found))
		return -EINVAL;

	return 0;
}

static int set_float_voltage_limit(struct max77759_charger *chg, u32 fv_mv)
{
	u32 regval;

	if (fv_selector(fv_mv, &regval))
		return -EINVAL;

	return regmap_update_bits(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_04,
				  MAX77759_CHGR_REG_CHG_CNFG_04_CHG_CV_PRM,
				  regval);
}

static int get_input_current_limit(struct max77759_charger *chg)
{
	u32 regval, val;
	int ret;

	ret = regmap_read(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_09, &regval);
	if (ret)
		return ret;

	regval = FIELD_GET(MAX77759_CHGR_REG_CHG_CNFG_09_CHGIN_ILIM, regval);
	regval = umax(regval, chgin_ilim_ranges[0].min_sel);

	ret = linear_range_get_value_array(chgin_ilim_ranges,
					   ARRAY_SIZE(chgin_ilim_ranges),
					   regval, &val);

	return ret ? ret : val;
}

static int set_input_current_limit(struct max77759_charger *chg, int ilim_ua)
{
	u32 regval;

	if (ilim_ua < 0)
		return -EINVAL;

	linear_range_get_selector_within(chgin_ilim_ranges, ilim_ua, &regval);

	return regmap_update_bits(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_09,
				  MAX77759_CHGR_REG_CHG_CNFG_09_CHGIN_ILIM,
				  regval);
}

/* The float voltage the chip gets for a request, as fv_selector() picks it */
static int fv_quantize(u32 fv_mv, u32 *out_mv)
{
	u32 sel;

	if (fv_selector(fv_mv, &sel))
		return -EINVAL;
	return linear_range_get_value_array(chg_cv_prm_ranges,
					    ARRAY_SIZE(chg_cv_prm_ranges),
					    sel, out_mv);
}

/*
 * The fast charge current the chip gets for a request: the step at or
 * below it, as cc_selector() picks it
 */
static int cc_quantize(u32 cc_ua, u32 *out_ua)
{
	u32 sel;

	if (cc_selector(cc_ua, &sel))
		return -EINVAL;
	return linear_range_get_value_array(chgcc_limit_ranges,
					    ARRAY_SIZE(chgcc_limit_ranges),
					    sel, out_ua);
}

/*
 * Write the float voltage, then the fast charge current, and read both
 * back from the chip (the registers are volatile, so this is no cached
 * value). -EIO when the chip does not hold what was written. *@held_fv and
 * *@held_cc are what it holds, each -1 when it was not read back.
 */
static int charger_apply_limits(struct max77759_charger *chg, u32 fv_mv,
				u32 cc_ua, int *held_fv, int *held_cc)
{
	u32 want_fv, want_cc;
	int ret, got_fv, got_cc;

	*held_fv = -1;
	*held_cc = -1;

	ret = fv_quantize(fv_mv, &want_fv);
	if (!ret)
		ret = cc_quantize(cc_ua, &want_cc);
	if (!ret)
		ret = set_float_voltage_limit(chg, fv_mv);
	if (!ret)
		ret = set_fast_charge_current_limit(chg, cc_ua);
	if (ret)
		return ret;

	got_fv = get_float_voltage(chg);
	got_cc = get_fast_charge_current(chg);
	if (got_fv >= 0)
		*held_fv = got_fv;
	if (got_cc >= 0)
		*held_cc = got_cc;
	if (got_fv < 0)
		return got_fv;
	if (got_cc < 0)
		return got_cc;
	if (got_fv != (int)want_fv || got_cc != (int)want_cc)
		return -EIO;

	return 0;
}

static const enum power_supply_property max77759_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

/*
 * The charge at which a threshold stop resumes: the start threshold, or
 * end - STOP_RESUME_GAP when start is 0 or not below the end.
 */
static int stop_resume(int end, int start)
{
	return start && start < end ? start : max(end - STOP_RESUME_GAP, 0);
}

static int max77759_charger_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *pval)
{
	struct max77759_charger *chg = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = get_online(chg);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = charger_input_valid(chg);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = get_status(chg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = get_charge_type(chg);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = get_health(chg);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = get_fast_charge_current(chg);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = get_float_voltage(chg);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = get_input_current_limit(chg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		/* The value in force: 75 for an end of 80 and a start of 0 */
		scoped_guard(mutex, &chg->lock)
			ret = chg->end_threshold >= 100 ? chg->start_threshold :
			      stop_resume(chg->end_threshold,
					  chg->start_threshold);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		scoped_guard(mutex, &chg->lock)
			ret = chg->end_threshold;
		break;
	default:
		ret = -EINVAL;
	}

	pval->intval = ret;
	return ret < 0 ? ret : 0;
}

/*
 * A write of one threshold, in percent. Each is checked on its own so the
 * order of two writes does not matter: an end at or below the start takes
 * the start back to 0, and a start at or above the end is kept but
 * stop_resume() then resumes at end - STOP_RESUME_GAP.
 */
static int stop_set_threshold(bool is_end, int val, int *end, int *start)
{
	if (is_end) {
		if (val < 1 || val > 100)
			return -EINVAL;
		*end = val;
		if (*start >= val)
			*start = 0;
	} else {
		if (val < 0 || val >= 100)
			return -EINVAL;
		*start = val;
	}
	return 0;
}

static int max77759_charger_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *pval)
{
	struct max77759_charger *chg = power_supply_get_drvdata(psy);

	if (psp != POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD &&
	    psp != POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD)
		return -EINVAL;

	guard(mutex)(&chg->lock);

	/* The guard work takes them at its next run */
	return stop_set_threshold(psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
				  pval->intval, &chg->end_threshold,
				  &chg->start_threshold);
}

static int max77759_charger_property_is_writeable(struct power_supply *psy,
						  enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD ||
	       psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD;
}

static const struct power_supply_desc max77759_charger_desc = {
	.name = "max77759-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = max77759_charger_props,
	.num_properties = ARRAY_SIZE(max77759_charger_props),
	.get_property = max77759_charger_get_property,
	.set_property = max77759_charger_set_property,
	.property_is_writeable = max77759_charger_property_is_writeable,
};

/* The mode the hardware gets: buck only while either inhibit holds */
static int charger_write_mode(struct max77759_charger *chg,
			      enum max77759_chgr_mode mode)
{
	if (mode == MAX77759_CHGR_MODE_CHG_BUCK_ON &&
	    (chg->inhibit || chg->stop_inhibit))
		mode = MAX77759_CHGR_MODE_BUCK_ON;

	return regmap_update_bits(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_00,
				  MAX77759_CHGR_REG_CHG_CNFG_00_MODE, mode);
}

static int charger_set_mode(struct max77759_charger *chg,
			    enum max77759_chgr_mode mode)
{
	int ret;

	guard(mutex)(&chg->lock);

	if (chg->mode == mode)
		return 0;

	if ((mode == MAX77759_CHGR_MODE_CHG_BUCK_ON ||
	     mode == MAX77759_CHGR_MODE_OTG_BOOST_ON) &&
	    chg->mode != MAX77759_CHGR_MODE_OFF) {
		dev_err(chg->dev, "Invalid mode transition from %d to %d\n",
			chg->mode, mode);
		return -EINVAL;
	}

	ret = charger_write_mode(chg, mode);
	if (ret)
		return ret;

	chg->mode = mode;
	return 0;
}

static int enable_chgin_otg(struct regulator_dev *rdev)
{
	struct max77759_charger *chg = rdev_get_drvdata(rdev);

	return charger_set_mode(chg, MAX77759_CHGR_MODE_OTG_BOOST_ON);
}

static int disable_chgin_otg(struct regulator_dev *rdev)
{
	struct max77759_charger *chg = rdev_get_drvdata(rdev);

	return charger_set_mode(chg, MAX77759_CHGR_MODE_OFF);
}

static int chgin_otg_status(struct regulator_dev *rdev)
{
	struct max77759_charger *chg = rdev_get_drvdata(rdev);

	guard(mutex)(&chg->lock);

	return chg->mode == MAX77759_CHGR_MODE_OTG_BOOST_ON;
}

static const struct regulator_ops chgin_otg_reg_ops = {
	.enable = enable_chgin_otg,
	.disable = disable_chgin_otg,
	.is_enabled = chgin_otg_status,
};

static const struct regulator_desc chgin_otg_reg_desc = {
	.name = "chgin-otg",
	.of_match = of_match_ptr("chgin-otg-regulator"),
	.owner = THIS_MODULE,
	.ops = &chgin_otg_reg_ops,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static irqreturn_t irq_handler(int irq, void *data)
{
	struct max77759_charger *chg = data;

	power_supply_changed(chg->psy);

	return IRQ_HANDLED;
}

static irqreturn_t bat_oilo_irq_handler(int irq, void *data)
{
	struct max77759_charger *chg = data;

	dev_warn_ratelimited(chg->dev,
			     "Battery over-current threshold crossed\n");

	return irq_handler(irq, data);
}

static int max77759_init_irqhandler(struct max77759_charger *chg)
{
	struct device *dev = chg->dev;
	irq_handler_t thread_fn;
	char *name;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(chgr_irqs_str); i++) {
		ret = platform_get_irq_byname(to_platform_device(dev),
					      chgr_irqs_str[i]);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "Failed to get irq resource for %s\n",
					     chgr_irqs_str[i]);

		chg->irqs[i] = ret;
		name = devm_kasprintf(dev, GFP_KERNEL, "%s:%s", dev_name(dev),
				      chgr_irqs_str[i]);
		if (!name)
			return dev_err_probe(dev, -ENOMEM,
					     "Failed to allocate space for irqname: %s\n",
					     chgr_irqs_str[i]);

		if (i == BAT_OILO)
			thread_fn = bat_oilo_irq_handler;
		else
			thread_fn = irq_handler;

		ret = devm_request_threaded_irq(dev, chg->irqs[i], NULL,
						thread_fn, 0, name, chg);
		if (ret)
			return ret;
	}

	return 0;
}

static int max77759_charger_init(struct max77759_charger *chg)
{
	struct power_supply_battery_info *info;
	u32 regval, fast_chg_curr, fv;
	int ret;

	ret = regmap_read(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_00, &regval);
	if (ret)
		return ret;

	chg->mode = FIELD_GET(MAX77759_CHGR_REG_CHG_CNFG_00_MODE, regval);
	ret = charger_set_mode(chg, MAX77759_CHGR_MODE_OFF);
	if (ret)
		return ret;

	if (power_supply_get_battery_info(chg->psy, &info)) {
		fv = CHG_FV_DEFAULT_MV;
		fast_chg_curr = CHG_CC_DEFAULT_UA;
	} else {
		fv = info->constant_charge_voltage_max_uv / 1000;
		fast_chg_curr = info->constant_charge_current_max_ua;
		if (info->charge_full_design_uah > 0)
			chg->design_uah = info->charge_full_design_uah;
	}
	chg->fv_max_mv = fv;
	chg->cc_max_ua = fast_chg_curr;

	ret = set_fast_charge_current_limit(chg, fast_chg_curr);
	if (ret)
		return ret;

	ret = set_float_voltage_limit(chg, fv);
	if (ret)
		return ret;

	ret = unlock_prot_regs(chg, true);
	if (ret)
		return ret;

	/* Disable wireless charging input */
	ret = regmap_update_bits(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_12,
				 MAX77759_CHGR_REG_CHG_CNFG_12_WCINSEL, 0);
	if (ret)
		goto relock;

	ret = regmap_update_bits(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_18,
				 MAX77759_CHGR_REG_CHG_CNFG_18_WDTEN, 0);
	if (ret)
		goto relock;

	/*
	 * Disable the fast-charge safety timer. At a low input limit a full
	 * charge outlasts it, and the charger then stops in a timer fault
	 * with the cable still in, until it is replugged or reset.
	 */
	ret = regmap_update_bits(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_01,
				 MAX77759_CHGR_REG_CHG_CNFG_01_FCHGTIME, 0);
	if (ret)
		goto relock;

	return unlock_prot_regs(chg, false);

relock:
	(void)unlock_prot_regs(chg, false);
	return ret;
}

static void psy_work_item(struct work_struct *work)
{
	struct max77759_charger *chg =
		container_of(work, struct max77759_charger, psy_work.work);
	union power_supply_propval current_limit, online;
	int ret;

	ret = power_supply_get_property(chg->tcpm_psy,
					POWER_SUPPLY_PROP_CURRENT_MAX,
					&current_limit);
	if (ret) {
		dev_err(chg->dev,
			"Failed to get CURRENT_MAX psy property, ret=%d\n",
			ret);
		goto err;
	}

	ret = power_supply_get_property(chg->tcpm_psy, POWER_SUPPLY_PROP_ONLINE,
					&online);
	if (ret) {
		dev_err(chg->dev,
			"Failed to get ONLINE psy property, ret=%d\n",
			ret);
		goto err;
	}

	/*
	 * A source with Type-C default power (or a TCPC that cannot tell)
	 * reports no current limit; charge at the configured default then.
	 */
	if (online.intval && !current_limit.intval)
		current_limit.intval = default_ilim_ua;

	if (online.intval && current_limit.intval) {
		ret = set_input_current_limit(chg, current_limit.intval);
		if (ret) {
			dev_err(chg->dev,
				"Unable to set current limit, ret=%d\n", ret);
			goto err;
		}

		charger_set_mode(chg, MAX77759_CHGR_MODE_CHG_BUCK_ON);
	} else {
		charger_set_mode(chg, MAX77759_CHGR_MODE_OFF);
	}

	scoped_guard(mutex, &chg->retry_lock) {
		if (chg->psy_work_retry_cnt)
			dev_dbg(chg->dev,
				"chg psy_work succeeded after %u tries\n",
				chg->psy_work_retry_cnt);
		chg->psy_work_retry_cnt = 0;
	}

	return;

err:
	charger_set_mode(chg, MAX77759_CHGR_MODE_OFF);
	scoped_guard(mutex, &chg->retry_lock) {
		if (chg->psy_work_retry_cnt >= MAX_NUM_RETRIES) {
			dev_err(chg->dev, "chg psy work failed, giving up\n");
			return;
		}

		++chg->psy_work_retry_cnt;
		dev_dbg(chg->dev, "Retrying %u/%u chg psy_work\n",
			chg->psy_work_retry_cnt, MAX_NUM_RETRIES);
		schedule_delayed_work(&chg->psy_work,
				      msecs_to_jiffies(PSY_WORK_RETRY_DELAY_MS));
	}
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *data)
{
	struct max77759_charger *chg = container_of(nb, struct max77759_charger,
						    nb);
	static const char *psy_name = "tcpm-source";
	struct power_supply *psy = data;

	/*
	 * A guard waiting for its first reading takes it as soon as the
	 * battery has one, rather than at its next retry.
	 */
	if (evt == PSY_EVENT_PROP_CHANGED && READ_ONCE(chg->guard_no_reading) &&
	    !strcmp(psy->desc->name, guard_psy))
		mod_delayed_work(system_wq, &chg->guard_work, 0);

	if (!strnstr(psy->desc->name, psy_name, strlen(psy_name)) ||
	    evt != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	chg->tcpm_psy = psy;
	scoped_guard(mutex, &chg->retry_lock)
		chg->psy_work_retry_cnt = 0;

	schedule_delayed_work(&chg->psy_work, 0);

	return NOTIFY_OK;
}

static void max_tcpci_unregister_psy_notifier(void *nb)
{
	power_supply_unreg_notifier(nb);
}

/*
 * Without a Type-C port controller nothing ever reports a source, so start
 * charging at the default input current limit right away, as the
 * bootloader left it (CHG_BUCK_ON).
 */
static int max77759_charger_start_default(struct max77759_charger *chg)
{
	int ret;

	ret = set_input_current_limit(chg, default_ilim_ua);
	if (ret)
		return ret;

	return charger_set_mode(chg, MAX77759_CHGR_MODE_CHG_BUCK_ON);
}

/* On unbind, leave charging on instead of in whatever state we had. */
static void max77759_charger_leave_charging(void *data)
{
	struct max77759_charger *chg = data;

	scoped_guard(mutex, &chg->lock) {
		if (chg->mode == MAX77759_CHGR_MODE_OTG_BOOST_ON ||
		    chg->mode == MAX77759_CHGR_MODE_CHG_BUCK_ON)
			return;
	}

	charger_set_mode(chg, MAX77759_CHGR_MODE_OFF);
	charger_set_mode(chg, MAX77759_CHGR_MODE_CHG_BUCK_ON);
}

static int guard_classify(int decic)
{
	int i;

	if (decic < guard_temp_decic[0] || decic >= guard_temp_decic[GUARD_NBANDS])
		return -1;
	for (i = 0; i < GUARD_NBANDS - 1; i++)
		if (decic < guard_temp_decic[i + 1])
			break;
	return i;
}

/* Charging allowed in a band, for comparing how restrictive two bands are */
static unsigned int guard_band_score(const struct guard_table *tbl, int band)
{
	return band < 0 ? 0 : tbl->cc_pct[band][0];
}

/*
 * Move to a more restrictive band at once; to a less restrictive one only
 * when the temperature is GUARD_HYST_DECIC inside it on both sides.
 */
static int guard_next_band(const struct guard_table *tbl, int cur, int decic)
{
	int band = guard_classify(decic);

	/* First reading (cur -2): take it as it is */
	if (cur < -1 || band == cur ||
	    guard_band_score(tbl, band) < guard_band_score(tbl, cur))
		return band;
	if (guard_classify(decic - GUARD_HYST_DECIC) == band &&
	    guard_classify(decic + GUARD_HYST_DECIC) == band)
		return band;
	return cur;
}

/* Highest tier of a band that may charge at all */
static int guard_top_tier(const struct guard_table *tbl, int band)
{
	int t;

	for (t = tbl->ntiers - 1; t > 0; t--)
		if (tbl->cc_pct[band][t])
			break;
	return t;
}

/*
 * Tiers only rise within a band, as the voltage does while charging: from
 * @tier, step up past every tier whose voltage the battery has reached, up
 * to the highest tier the band charges in.
 */
static int guard_next_tier(const struct guard_table *tbl, int band, int tier,
			   int uv)
{
	while (tier < tbl->ntiers - 1 && uv >= tbl->cv_mv[tier] * 1000)
		tier++;
	return min(tier, guard_top_tier(tbl, band));
}

/*
 * Float voltage and charge current for a band and tier, both capped by the
 * limits from the battery description.
 */
static void guard_limits(const struct guard_table *tbl, int band, int tier,
			 u32 design_uah, u32 fv_max_mv, u32 cc_max_ua,
			 u32 *fv_mv, u32 *cc_ua)
{
	u32 cc;

	cc = div_u64((u64)design_uah * tbl->cc_pct[band][tier], 100);
	*fv_mv = min(tbl->cv_mv[guard_top_tier(tbl, band)], fv_max_mv);
	*cc_ua = min(cc, cc_max_ua);
}

static const struct guard_table *guard_select_table(void)
{
	if (of_machine_is_compatible("google,zuma-husky"))
		return &guard_table_husky;
	return &guard_table_default;
}

static int guard_read(int *decic, int *uv)
{
	union power_supply_propval val;
	struct power_supply *psy;
	int ret;

	psy = power_supply_get_by_name(guard_psy);
	if (!psy)
		return -ENODEV;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (!ret) {
		*decic = val.intval;
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
						&val);
		*uv = val.intval;
	}
	power_supply_put(psy);

	return ret;
}

/* Set one of the two inhibit flags; the mode follows the two together */
static void charger_set_inhibit(struct max77759_charger *chg, bool *flag,
				bool inhibit)
{
	bool was, old;

	guard(mutex)(&chg->lock);

	was = chg->inhibit || chg->stop_inhibit;
	old = *flag;
	*flag = inhibit;
	/* A failed write keeps the old flag, so the next call tries again */
	if (was != (chg->inhibit || chg->stop_inhibit) &&
	    chg->mode == MAX77759_CHGR_MODE_CHG_BUCK_ON &&
	    charger_write_mode(chg, chg->mode))
		*flag = old;
}

static void guard_set_inhibit(struct max77759_charger *chg, bool inhibit)
{
	charger_set_inhibit(chg, &chg->inhibit, inhibit);
}

/*
 * Charging only on limits the chip is seen to hold: inhibit when it does
 * not take them. Returns 0 when it does.
 */
static int guard_apply(struct max77759_charger *chg, u32 fv_mv, u32 cc_ua)
{
	int held_fv, held_cc;
	int ret = charger_apply_limits(chg, fv_mv, cc_ua, &held_fv, &held_cc);

	if (ret && !chg->apply_failed)
		dev_err(chg->dev,
			"chip did not take float %u mV, charge %u uA (%d), it holds %d mV, %d uA: charging inhibited\n",
			fv_mv, cc_ua, ret, held_fv, held_cc);
	chg->apply_failed = ret;
	guard_set_inhibit(chg, ret);
	return ret;
}

/*
 * Back to the limits from the battery description, charging allowed. If
 * the chip does not take them, stay inhibited and try again next run.
 */
static void guard_release(struct max77759_charger *chg)
{
	if (guard_apply(chg, chg->fv_max_mv, chg->cc_max_ua))
		return;
	chg->guard_active = false;
	chg->guard_band = -2;
	dev_info(chg->dev, "temperature guard off\n");
}

/*
 * Whether charging stays stopped: stop at @soc >= @end, resume at
 * @soc <= stop_resume(). An end of 100 never stops.
 */
static bool stop_next(bool stopped, int soc, int end, int start)
{
	if (end >= 100)
		return false;
	if (!stopped)
		return soc >= end;
	return soc > stop_resume(end, start);
}

static int stop_read_soc(struct max77759_charger *chg, int *soc)
{
	union power_supply_propval val;
	struct power_supply *psy;
	int ret;

	psy = power_supply_get_by_name(chg->bat_psy ?: guard_psy);
	if (!psy)
		return -ENODEV;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	power_supply_put(psy);
	if (!ret)
		*soc = val.intval;
	return ret;
}

/*
 * The charge thresholds. At 100 / 0 this reads nothing and writes
 * nothing.
 */
static void stop_work(struct max77759_charger *chg)
{
	int end, start, soc = -1;
	bool stopped;

	scoped_guard(mutex, &chg->lock) {
		end = chg->end_threshold;
		start = chg->start_threshold;
	}

	stopped = chg->stop_inhibit;
	if (end >= 100)
		stopped = false;
	else if (!stop_read_soc(chg, &soc))
		stopped = stop_next(stopped, soc, end, start);

	if (stopped != chg->stop_inhibit && soc >= 0)
		dev_info(chg->dev, "charge %s at %d %%\n",
			 stopped ? "stopped" : "resumed", soc);
	else if (stopped != chg->stop_inhibit)
		dev_info(chg->dev, "charge resumed: no limit\n");
	charger_set_inhibit(chg, &chg->stop_inhibit, stopped);
}

static void guard_work_item(struct work_struct *work)
{
	struct max77759_charger *chg =
		container_of(work, struct max77759_charger, guard_work.work);
	const struct guard_table *tbl = chg->guard_table;
	int decic = 0, uv = 0, band, tier;
	unsigned int next_ms = GUARD_PERIOD_MS;
	bool read_ok;
	u32 fv, cc;

	/*
	 * The charge thresholds first: a stop they call for is in place
	 * before the guard below can clear its own inhibit.
	 */
	stop_work(chg);

	if (!temp_guard) {
		WRITE_ONCE(chg->guard_no_reading, false);
		if (chg->guard_active)
			guard_release(chg);
		goto out;
	}

	if (!chg->guard_active) {
		chg->guard_active = true;
		chg->guard_band = -2;
		dev_info(chg->dev, "temperature guard on (%s)\n", guard_psy);
	}

	/*
	 * No reading, no charging. A missing reading is usually the gauge
	 * still starting (it answers once its model load is done), so look
	 * again soon instead of a guard period later.
	 */
	read_ok = !guard_read(&decic, &uv);
	WRITE_ONCE(chg->guard_no_reading, !read_ok);
	if (!read_ok)
		next_ms = GUARD_RETRY_MS;
	band = read_ok ? guard_next_band(tbl, chg->guard_band, decic) : -1;

	if (band < 0) {
		if (chg->guard_band != -1 && !read_ok)
			dev_warn(chg->dev, "no reading from %s: charging inhibited\n",
				 guard_psy);
		else if (chg->guard_band != -1)
			dev_info(chg->dev, "battery %s%d.%d C: charging inhibited\n",
				 decic < 0 ? "-" : "", abs(decic) / 10,
				 abs(decic) % 10);
		chg->guard_band = -1;
		guard_set_inhibit(chg, true);
		goto out;
	}

	/* A new band starts again from the lowest tier */
	if (band != chg->guard_band)
		chg->guard_tier = 0;
	tier = guard_next_tier(tbl, band, chg->guard_tier, uv);
	guard_limits(tbl, band, tier, chg->design_uah, chg->fv_max_mv,
		     chg->cc_max_ua, &fv, &cc);

	if (guard_apply(chg, fv, cc))
		goto out;

	if (band != chg->guard_band || tier != chg->guard_tier)
		dev_info(chg->dev,
			 "battery %s%d.%d C, %d mV: band %d-%d C, float %u mV, charge %u uA\n",
			 decic < 0 ? "-" : "", abs(decic) / 10, abs(decic) % 10,
			 uv / 1000,
			 guard_temp_decic[band] / 10, guard_temp_decic[band + 1] / 10,
			 fv, cc);
	chg->guard_band = band;
	chg->guard_tier = tier;
out:
	schedule_delayed_work(&chg->guard_work, msecs_to_jiffies(next_ms));
}

static void max77759_charger_log_state(struct max77759_charger *chg,
				       const char *when)
{
	int fv = get_float_voltage(chg);
	int cc = get_fast_charge_current(chg);
	int ilim = get_input_current_limit(chg);
	u32 cnfg_00 = 0;

	regmap_read(chg->regmap, MAX77759_CHGR_REG_CHG_CNFG_00, &cnfg_00);
	dev_info(chg->dev,
		 "%s: mode %#lx, float voltage %d mV, fast charge %d uA, input limit %d uA\n",
		 when, FIELD_GET(MAX77759_CHGR_REG_CHG_CNFG_00_MODE, cnfg_00),
		 fv, cc, ilim);
}

/* On unbind, no charge threshold holds charging off any more */
static void max77759_charger_clear_stop(void *data)
{
	struct max77759_charger *chg = data;

	charger_set_inhibit(chg, &chg->stop_inhibit, false);
}

static int max77759_charger_probe(struct platform_device *pdev)
{
	struct regulator_config chgin_otg_reg_cfg = { };
	struct power_supply_config psy_cfg = { };
	struct device *dev = &pdev->dev;
	struct max77759_charger *chg;
	int ret;

	device_set_of_node_from_dev(dev, dev->parent);
	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	platform_set_drvdata(pdev, chg);
	chg->dev = dev;
	chg->end_threshold = 100;	/* 100 / 0: no charge limit */
	chg->regmap = dev_get_regmap(dev->parent, "charger");
	if (!chg->regmap)
		return dev_err_probe(dev, -ENODEV, "Missing regmap\n");

	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize lock\n");

	ret = devm_mutex_init(dev, &chg->retry_lock);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to initialize retry_lock\n");

	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.drv_data = chg;
	chg->psy = devm_power_supply_register(dev, &max77759_charger_desc,
					      &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy),
				     "Failed to register psy\n");

	if (default_ilim_ua > CHG_DEFAULT_ILIM_MAX_UA) {
		dev_warn(dev, "default_ilim_ua %u too high, using %u\n",
			 default_ilim_ua, CHG_DEFAULT_ILIM_MAX_UA);
		default_ilim_ua = CHG_DEFAULT_ILIM_MAX_UA;
	}

	max77759_charger_log_state(chg, "found");

	ret = max77759_charger_init(chg);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to initialize max77759 charger\n");

	if (default_ilim_ua) {
		ret = devm_add_action_or_reset(dev,
					       max77759_charger_leave_charging,
					       chg);
		if (ret)
			return ret;

		ret = max77759_charger_start_default(chg);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to start charging\n");
	}

	/*
	 * devm undoes in reverse: on unbind the guard work is cancelled,
	 * then a threshold stop is cleared, and only then is charging
	 * left on.
	 */
	ret = devm_add_action_or_reset(dev, max77759_charger_clear_stop, chg);
	if (ret)
		return ret;

	chgin_otg_reg_cfg.dev = dev;
	chgin_otg_reg_cfg.driver_data = chg;
	chgin_otg_reg_cfg.of_node = dev_of_node(dev);
	chg->chgin_otg_rdev = devm_regulator_register(dev, &chgin_otg_reg_desc,
						      &chgin_otg_reg_cfg);
	if (IS_ERR(chg->chgin_otg_rdev))
		return dev_err_probe(dev, PTR_ERR(chg->chgin_otg_rdev),
				     "Failed to register chgin otg regulator\n");

	ret = devm_delayed_work_autocancel(dev, &chg->psy_work, psy_work_item);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize psy work\n");

	/*
	 * Before the notifier, which can kick the guard work: devm then
	 * unregisters the notifier before it cancels the work.
	 */
	chg->guard_band = -2;
	chg->guard_table = guard_select_table();
	ret = devm_delayed_work_autocancel(dev, &chg->guard_work, guard_work_item);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize guard work\n");

	chg->nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&chg->nb);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Unable to register psy notifier\n");

	ret = devm_add_action_or_reset(dev, max_tcpci_unregister_psy_notifier,
				       &chg->nb);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add devm action to unregister psy notifier\n");

	ret = max77759_init_irqhandler(chg);
	if (ret)
		return ret;

	/* 4926 mAh: the battery in our device tree, if it doesn't say */
	if (!chg->design_uah)
		chg->design_uah = 4926000;
	schedule_delayed_work(&chg->guard_work, 0);

	max77759_charger_log_state(chg, "probed");

	return 0;
}

static const struct platform_device_id max77759_charger_id[] = {
	{ .name = "max77759-charger" },
	{ }
};
MODULE_DEVICE_TABLE(platform, max77759_charger_id);

static struct platform_driver max77759_charger_driver = {
	.driver = {
		.name = "max77759-charger",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = max77759_charger_probe,
	.id_table = max77759_charger_id,
};
module_platform_driver(max77759_charger_driver);

#if IS_ENABLED(CONFIG_CHARGER_MAX77759_KUNIT_TEST)
#include "max77759_charger_test.c"
#endif

MODULE_AUTHOR("Amit Sunil Dhamne <amitsd@google.com>");
MODULE_DESCRIPTION("Maxim MAX77759 charger driver");
MODULE_LICENSE("GPL");
