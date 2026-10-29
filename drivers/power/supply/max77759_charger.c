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
 */
static bool temp_guard;
module_param(temp_guard, bool, 0644);
MODULE_PARM_DESC(temp_guard,
		 "Limit charging by battery temperature as the stock phone does: no charge below 0 or from 55 C, lower float voltage and current outside 10-46 C. Reads the battery power supply named by guard_psy every 30 s. Default off.");

static char guard_psy[32] = "max170xx_battery";
module_param_string(guard_psy, guard_psy, sizeof(guard_psy), 0444);
MODULE_PARM_DESC(guard_psy, "Battery power supply whose temp and voltage_now the guard reads");

#define GUARD_PERIOD_MS			30000
#define GUARD_HYST_DECIC		20	/* 2.0 C into a band before relaxing */
#define GUARD_NBANDS			6
#define GUARD_NTIERS			3

/* Band i is [guard_temp_decic[i], guard_temp_decic[i + 1]), in 0.1 C */
static const int guard_temp_decic[GUARD_NBANDS + 1] = {
	0, 100, 200, 420, 460, 480, 550
};

/* Tier j charges up to guard_cv_mv[j] */
static const u32 guard_cv_mv[GUARD_NTIERS] = { 4200, 4300, 4450 };

/* Charge current per band and tier, in percent of the design capacity */
static const u8 guard_cc_pct[GUARD_NBANDS][GUARD_NTIERS] = {
	{  30, 10,  0 },	/*  0-10 C */
	{  50, 30, 30 },	/* 10-20 C */
	{ 100, 70, 50 },	/* 20-42 C */
	{  80, 50, 50 },	/* 42-46 C */
	{  50, 50,  0 },	/* 46-48 C */
	{  30,  0,  0 },	/* 48-55 C */
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

	/* Temperature guard */
	struct delayed_work guard_work;
	bool guard_active;
	int guard_band;		/* -1: charging inhibited */
	int guard_tier;
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

	return (val & MAX77759_CHGR_REG_CHG_INT_CHG) &&
		(val & MAX77759_CHGR_REG_CHG_INT_CHGIN);
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

static int set_fast_charge_current_limit(struct max77759_charger *chg,
					 u32 cc_max_ua)
{
	bool found;
	u32 regval;

	linear_range_get_selector_high_array(chgcc_limit_ranges,
					     ARRAY_SIZE(chgcc_limit_ranges),
					     cc_max_ua, &regval, &found);
	if (!found)
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

static int set_float_voltage_limit(struct max77759_charger *chg, u32 fv_mv)
{
	u32 regval;
	bool found;

	linear_range_get_selector_high_array(chg_cv_prm_ranges,
					     ARRAY_SIZE(chg_cv_prm_ranges),
					     fv_mv, &regval, &found);
	if (!found)
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

static const enum power_supply_property max77759_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

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
	default:
		ret = -EINVAL;
	}

	pval->intval = ret;
	return ret < 0 ? ret : 0;
}

static const struct power_supply_desc max77759_charger_desc = {
	.name = "max77759-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = max77759_charger_props,
	.num_properties = ARRAY_SIZE(max77759_charger_props),
	.get_property = max77759_charger_get_property,
};

/* The mode the hardware gets: buck only while the guard inhibits charging */
static int charger_write_mode(struct max77759_charger *chg,
			      enum max77759_chgr_mode mode)
{
	if (mode == MAX77759_CHGR_MODE_CHG_BUCK_ON && chg->inhibit)
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
static unsigned int guard_band_score(int band)
{
	return band < 0 ? 0 : guard_cc_pct[band][0];
}

/*
 * Move to a more restrictive band at once; to a less restrictive one only
 * when the temperature is GUARD_HYST_DECIC inside it on both sides.
 */
static int guard_next_band(int cur, int decic)
{
	int band = guard_classify(decic);

	/* First reading (cur -2): take it as it is */
	if (cur < -1 || band == cur ||
	    guard_band_score(band) < guard_band_score(cur))
		return band;
	if (guard_classify(decic - GUARD_HYST_DECIC) == band &&
	    guard_classify(decic + GUARD_HYST_DECIC) == band)
		return band;
	return cur;
}

/* Highest tier of a band that may charge at all */
static int guard_top_tier(int band)
{
	int t;

	for (t = GUARD_NTIERS - 1; t > 0; t--)
		if (guard_cc_pct[band][t])
			break;
	return t;
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

static void guard_set_inhibit(struct max77759_charger *chg, bool inhibit)
{
	guard(mutex)(&chg->lock);

	if (chg->inhibit == inhibit)
		return;
	chg->inhibit = inhibit;
	if (chg->mode == MAX77759_CHGR_MODE_CHG_BUCK_ON)
		charger_write_mode(chg, chg->mode);
}

/* Back to the limits from the battery description, charging allowed */
static void guard_release(struct max77759_charger *chg)
{
	set_float_voltage_limit(chg, chg->fv_max_mv);
	set_fast_charge_current_limit(chg, chg->cc_max_ua);
	guard_set_inhibit(chg, false);
	chg->guard_active = false;
	chg->guard_band = -2;
	dev_info(chg->dev, "temperature guard off\n");
}

static void guard_work_item(struct work_struct *work)
{
	struct max77759_charger *chg =
		container_of(work, struct max77759_charger, guard_work.work);
	int decic = 0, uv = 0, band, tier;
	bool read_ok;
	u32 fv, cc;

	if (!temp_guard) {
		if (chg->guard_active)
			guard_release(chg);
		goto out;
	}

	if (!chg->guard_active) {
		chg->guard_active = true;
		chg->guard_band = -2;
		dev_info(chg->dev, "temperature guard on (%s)\n", guard_psy);
	}

	/* No reading, no charging */
	read_ok = !guard_read(&decic, &uv);
	band = read_ok ? guard_next_band(chg->guard_band, decic) : -1;

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

	/* Tiers only rise within a band, as the voltage does while charging */
	if (band != chg->guard_band)
		chg->guard_tier = 0;
	tier = chg->guard_tier;
	while (tier < GUARD_NTIERS - 1 && uv >= guard_cv_mv[tier] * 1000)
		tier++;
	tier = min(tier, guard_top_tier(band));

	fv = min(guard_cv_mv[guard_top_tier(band)], chg->fv_max_mv);
	cc = div_u64((u64)chg->design_uah * guard_cc_pct[band][tier], 100);
	cc = min(cc, chg->cc_max_ua);

	if (band != chg->guard_band || tier != chg->guard_tier)
		dev_info(chg->dev,
			 "battery %s%d.%d C, %d mV: band %d-%d C, float %u mV, charge %u uA\n",
			 decic < 0 ? "-" : "", abs(decic) / 10, abs(decic) % 10,
			 uv / 1000,
			 guard_temp_decic[band] / 10, guard_temp_decic[band + 1] / 10,
			 fv, cc);

	set_float_voltage_limit(chg, fv);
	set_fast_charge_current_limit(chg, cc);
	guard_set_inhibit(chg, false);
	chg->guard_band = band;
	chg->guard_tier = tier;
out:
	schedule_delayed_work(&chg->guard_work, msecs_to_jiffies(GUARD_PERIOD_MS));
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
	chg->guard_band = -2;
	ret = devm_delayed_work_autocancel(dev, &chg->guard_work, guard_work_item);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize guard work\n");
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

MODULE_AUTHOR("Amit Sunil Dhamne <amitsd@google.com>");
MODULE_DESCRIPTION("Maxim MAX77759 charger driver");
MODULE_LICENSE("GPL");
