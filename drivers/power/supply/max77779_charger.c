// SPDX-License-Identifier: GPL-2.0-only
/*
 * Maxim MAX77779 battery charger: status, charge inhibit and USB OTG VBUS.
 *
 * The charger runs under its own (OTP) defaults. This driver reports what
 * it is doing and can stop charging: inhibit-charge puts it in buck-only
 * mode, so the battery is no longer charged and USB powers the system,
 * though with the battery near full the battery still supplies part of the
 * load. For host mode it supplies VBUS (its chgin-otg regulator). Only the
 * mode is written; the interrupt registers (clear on read) are never read,
 * and the state is polled.
 *
 * Register fields from Google's driver (google-modules/bms at 183509aff980:
 * max77779_charger.c, GPL, and max77779_v20240115_regs.h).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/workqueue.h>

#define MAX77779_CHG_INT_OK		0xb6
#define  MAX77779_CHGIN_OK		BIT(6)
#define  MAX77779_WCIN_OK		BIT(5)
#define  MAX77779_INPUT_OK		(MAX77779_CHGIN_OK | MAX77779_WCIN_OK)
#define MAX77779_CHG_DETAILS_01		0xb8
#define  MAX77779_CHG_DTLS		GENMASK(3, 0)
#define MAX77779_CHG_CNFG_00		0xbc
#define  MAX77779_MODE			GENMASK(3, 0)
#define  MAX77779_MODE_OFF		0x0
#define  MAX77779_MODE_BUCK		0x4
#define  MAX77779_MODE_CHG_BUCK		0x5
#define MAX77779_CHG_CNFG_02		0xbe
#define  MAX77779_CHGCC			GENMASK(5, 0)
#define MAX77779_CHG_CNFG_09		0xc5
#define  MAX77779_CHGIN_ILIM		GENMASK(6, 0)

#define MAX77779_POLL_MS		10000

/* CHG_DETAILS_01.CHG_DTLS */
enum {
	MAX77779_DTLS_DEAD_BATTERY = 0x0,
	MAX77779_DTLS_FAST_CC = 0x1,
	MAX77779_DTLS_FAST_CV = 0x2,
	MAX77779_DTLS_TOP_OFF = 0x3,
	MAX77779_DTLS_DONE = 0x4,
};

struct max77779_charger {
	struct device *dev;
	struct regmap *map;
	struct power_supply *psy;
	struct delayed_work poll;
	/* serialises the mode changes */
	struct mutex lock;
	unsigned int last;
	int behaviour;
	bool held;
	/* the OTG boost that sources VBUS: optional, external */
	struct regulator *boost;
	bool otg;
};

static bool max77779_charger_readable(struct device *dev, unsigned int reg)
{
	return reg >= MAX77779_CHG_INT_OK && reg <= MAX77779_CHG_CNFG_09;
}

static bool max77779_charger_writeable(struct device *dev, unsigned int reg)
{
	return reg == MAX77779_CHG_CNFG_00;
}

static const struct regmap_config max77779_charger_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77779_CHG_CNFG_09,
	.readable_reg = max77779_charger_readable,
	.writeable_reg = max77779_charger_writeable,
	.cache_type = REGCACHE_NONE,
};

/* CHG_CNFG_00 is not behind CHGPROT (unlike CNFG_01, 03, 07-08 and 13 on). */
static int max77779_charger_set_mode(struct max77779_charger *chg,
				     unsigned int mode)
{
	return regmap_update_bits(chg->map, MAX77779_CHG_CNFG_00,
				  MAX77779_MODE, mode);
}

static int max77779_charger_status(struct max77779_charger *chg, int *status)
{
	unsigned int ok, dtls;
	int ret;

	ret = regmap_read(chg->map, MAX77779_CHG_INT_OK, &ok);
	if (ret)
		return ret;
	ret = regmap_read(chg->map, MAX77779_CHG_DETAILS_01, &dtls);
	if (ret)
		return ret;

	if (!(ok & MAX77779_INPUT_OK))
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (chg->held)
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		switch (FIELD_GET(MAX77779_CHG_DTLS, dtls)) {
		case MAX77779_DTLS_DEAD_BATTERY:
		case MAX77779_DTLS_FAST_CC:
		case MAX77779_DTLS_FAST_CV:
		case MAX77779_DTLS_TOP_OFF:
			*status = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case MAX77779_DTLS_DONE:
			*status = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}

	return 0;
}

static int max77779_charger_charge_type(struct max77779_charger *chg, int *type)
{
	unsigned int ok, dtls;
	int ret;

	ret = regmap_read(chg->map, MAX77779_CHG_INT_OK, &ok);
	if (ret)
		return ret;
	ret = regmap_read(chg->map, MAX77779_CHG_DETAILS_01, &dtls);
	if (ret)
		return ret;

	if (!(ok & MAX77779_INPUT_OK) || chg->held) {
		*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		return 0;
	}

	switch (FIELD_GET(MAX77779_CHG_DTLS, dtls)) {
	case MAX77779_DTLS_DEAD_BATTERY:
		*type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case MAX77779_DTLS_FAST_CC:
		*type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case MAX77779_DTLS_FAST_CV:
	case MAX77779_DTLS_TOP_OFF:
		*type = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
		break;
	default:
		*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	return 0;
}

static int max77779_charger_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct max77779_charger *chg = power_supply_get_drvdata(psy);
	unsigned int v;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		/* While it sources VBUS, CHGIN carries our own boost: no input. */
		if (READ_ONCE(chg->otg)) {
			val->intval = 0;
			return 0;
		}
		ret = regmap_read(chg->map, MAX77779_CHG_INT_OK, &v);
		if (ret)
			return ret;
		val->intval = !!(v & MAX77779_INPUT_OK);
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		if (READ_ONCE(chg->otg)) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			return 0;
		}
		return max77779_charger_status(chg, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		return max77779_charger_charge_type(chg, &val->intval);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		/* CHGIN_ILIM: 0 off, 1-3 100 mA, then 125 mA in 25 mA steps */
		ret = regmap_read(chg->map, MAX77779_CHG_CNFG_09, &v);
		if (ret)
			return ret;
		v = FIELD_GET(MAX77779_CHGIN_ILIM, v);
		val->intval = !v ? 0 : v < 4 ? 100000 : 125000 + (v - 4) * 25000;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		/* CHGCC as Google's driver reads it: 0 off, 1-2 133 mA, 4 A max */
		ret = regmap_read(chg->map, MAX77779_CHG_CNFG_02, &v);
		if (ret)
			return ret;
		v = FIELD_GET(MAX77779_CHGCC, v);
		if (!v)
			val->intval = 0;
		else if (v < 3)
			val->intval = 133000;
		else if (v >= 0x3c)
			val->intval = 4000000;
		else
			val->intval = 133000 + (v - 2) * 66670;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = chg->behaviour;
		return 0;
	default:
		return -EINVAL;
	}
}

static int max77779_charger_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *val)
{
	struct max77779_charger *chg = power_supply_get_drvdata(psy);
	bool hold;
	int ret;

	if (psp != POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR)
		return -EINVAL;
	if (val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
	    val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE)
		return -EINVAL;

	hold = val->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;

	scoped_guard(mutex, &chg->lock) {
		if (chg->otg)
			return -EBUSY;
		if (hold != chg->held) {
			ret = max77779_charger_set_mode(chg, hold ?
							MAX77779_MODE_BUCK :
							MAX77779_MODE_CHG_BUCK);
			if (ret)
				return ret;
			chg->held = hold;
		}
		chg->behaviour = val->intval;
	}

	power_supply_changed(chg->psy);
	return 0;
}

static int max77779_charger_property_is_writeable(struct power_supply *psy,
						  enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR;
}

static const enum power_supply_property max77779_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
};

static const struct power_supply_desc max77779_charger_desc = {
	.name = "max77779-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = max77779_charger_props,
	.num_properties = ARRAY_SIZE(max77779_charger_props),
	.get_property = max77779_charger_get_property,
	.set_property = max77779_charger_set_property,
	.property_is_writeable = max77779_charger_property_is_writeable,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
			     BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
};

/*
 * VBUS for a device on the USB-C port, from an external boost (the Pixel 9a's
 * RT9470): the charger is put in standby first and then the boost is enabled,
 * as Google's ext-otg-only use case does. The charger's own boost mode (0xa)
 * is not used: nothing here has run it.
 */
static int max77779_charger_otg_enable(struct regulator_dev *rdev)
{
	struct max77779_charger *chg = rdev_get_drvdata(rdev);
	unsigned int v;
	int ret;

	if (!chg->boost)
		return -EOPNOTSUPP;

	guard(mutex)(&chg->lock);
	/* Something else already drives VBUS: don't boost into it. */
	ret = regmap_read(chg->map, MAX77779_CHG_INT_OK, &v);
	if (ret)
		return ret;
	if (v & MAX77779_INPUT_OK)
		return -EBUSY;

	ret = max77779_charger_set_mode(chg, MAX77779_MODE_OFF);
	if (ret)
		return ret;

	ret = regulator_enable(chg->boost);
	if (ret) {
		max77779_charger_set_mode(chg, chg->held ? MAX77779_MODE_BUCK :
					  MAX77779_MODE_CHG_BUCK);
		return ret;
	}
	WRITE_ONCE(chg->otg, true);

	return 0;
}

static int max77779_charger_otg_disable(struct regulator_dev *rdev)
{
	struct max77779_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	guard(mutex)(&chg->lock);
	ret = regulator_disable(chg->boost);
	if (ret)
		return ret;
	ret = max77779_charger_set_mode(chg, chg->held ? MAX77779_MODE_BUCK :
					MAX77779_MODE_CHG_BUCK);
	if (ret)
		return ret;
	WRITE_ONCE(chg->otg, false);

	return 0;
}

static int max77779_charger_otg_is_enabled(struct regulator_dev *rdev)
{
	struct max77779_charger *chg = rdev_get_drvdata(rdev);

	guard(mutex)(&chg->lock);
	return chg->otg;
}

static const struct regulator_ops max77779_charger_otg_ops = {
	.enable = max77779_charger_otg_enable,
	.disable = max77779_charger_otg_disable,
	.is_enabled = max77779_charger_otg_is_enabled,
};

static const struct regulator_desc max77779_charger_otg_desc = {
	.name = "chgin-otg",
	.of_match = "chgin-otg-regulator",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &max77779_charger_otg_ops,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

/* Tell the gauge and userspace when the input or the charge state changes. */
static void max77779_charger_poll(struct work_struct *work)
{
	struct max77779_charger *chg =
		container_of(to_delayed_work(work), struct max77779_charger, poll);
	unsigned int ok = 0, dtls = 0, now;

	if (!regmap_read(chg->map, MAX77779_CHG_INT_OK, &ok) &&
	    !regmap_read(chg->map, MAX77779_CHG_DETAILS_01, &dtls)) {
		now = (ok & MAX77779_INPUT_OK) << 8 |
		      FIELD_GET(MAX77779_CHG_DTLS, dtls);
		if (now != chg->last) {
			chg->last = now;
			power_supply_changed(chg->psy);
		}
	}

	queue_delayed_work(system_freezable_wq, &chg->poll,
			   msecs_to_jiffies(MAX77779_POLL_MS));
}

/* Leave the charger charging, as it is normally found. */
static void max77779_charger_restore(void *data)
{
	struct max77779_charger *chg = data;

	if (chg->otg)
		regulator_disable(chg->boost);
	if (chg->held || chg->otg)
		max77779_charger_set_mode(chg, MAX77779_MODE_CHG_BUCK);
}

static void max77779_charger_cancel_poll(void *data)
{
	struct max77779_charger *chg = data;

	cancel_delayed_work_sync(&chg->poll);
}

static int max77779_charger_probe(struct i2c_client *client)
{
	struct power_supply_config cfg = {};
	struct regulator_config rcfg = {};
	struct regulator_dev *rdev;
	struct device *dev = &client->dev;
	struct max77779_charger *chg;
	unsigned int mode;
	int ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	i2c_set_clientdata(client, chg);
	chg->behaviour = POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
	chg->last = ~0U;
	INIT_DELAYED_WORK(&chg->poll, max77779_charger_poll);

	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return ret;

	chg->map = devm_regmap_init_i2c(client, &max77779_charger_regmap);
	if (IS_ERR(chg->map))
		return PTR_ERR(chg->map);

	ret = regmap_read(chg->map, MAX77779_CHG_CNFG_00, &mode);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the mode\n");
	/*
	 * An inhibit survives a warm reboot (the PMIC stays powered): start
	 * from the mode found, so auto restores charging. Leaving inhibit
	 * always goes back to charging and buck.
	 */
	mode = FIELD_GET(MAX77779_MODE, mode);
	if (mode == MAX77779_MODE_BUCK) {
		chg->held = true;
		chg->behaviour = POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
	} else if (mode != MAX77779_MODE_CHG_BUCK) {
		dev_warn(dev, "found in mode %#x, not charging and buck\n", mode);
	}

	chg->boost = devm_regulator_get_optional(dev, "otg-boost");
	if (IS_ERR(chg->boost)) {
		if (PTR_ERR(chg->boost) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(chg->boost),
					     "failed to get the OTG boost\n");
		chg->boost = NULL;
	}

	/*
	 * Unwound in reverse: the VBUS regulator gone, poll stopped, supply
	 * gone, then the boost off and charging back, and only then the boost
	 * put.
	 */
	ret = devm_add_action_or_reset(dev, max77779_charger_restore, chg);
	if (ret)
		return ret;

	cfg.drv_data = chg;
	cfg.fwnode = dev_fwnode(dev);
	chg->psy = devm_power_supply_register(dev, &max77779_charger_desc, &cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy),
				     "failed to register the power supply\n");

	ret = devm_add_action_or_reset(dev, max77779_charger_cancel_poll, chg);
	if (ret)
		return ret;

	rcfg.dev = dev;
	rcfg.driver_data = chg;
	rdev = devm_regulator_register(dev, &max77779_charger_otg_desc, &rcfg);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev),
				     "failed to register the OTG regulator\n");

	queue_delayed_work(system_freezable_wq, &chg->poll, 0);
	return 0;
}

/* Don't let an inhibit outlive the system: reboot with charging on. */
static void max77779_charger_shutdown(struct i2c_client *client)
{
	struct max77779_charger *chg = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chg->poll);
	max77779_charger_restore(chg);
}

static const struct of_device_id max77779_charger_of_match[] = {
	{ .compatible = "maxim,max77779-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77779_charger_of_match);

static const struct i2c_device_id max77779_charger_id[] = {
	{ "max77779-charger" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77779_charger_id);

static struct i2c_driver max77779_charger_driver = {
	.driver = {
		.name = "max77779-charger",
		.of_match_table = max77779_charger_of_match,
	},
	.probe = max77779_charger_probe,
	.shutdown = max77779_charger_shutdown,
	.id_table = max77779_charger_id,
};
module_i2c_driver(max77779_charger_driver);

MODULE_DESCRIPTION("Maxim MAX77779 battery charger status and charge inhibit");
MODULE_LICENSE("GPL");
