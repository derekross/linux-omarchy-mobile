// SPDX-License-Identifier: GPL-2.0-only
/* Read-only MAX77779 fuel-gauge measurements; retain the firmware model. */

#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define MAX77779_STATUS		0x00
#define MAX77779_REPCAP		0x06
#define MAX77779_REPSOC		0x07
#define MAX77779_CONFIG		0x0c
#define MAX77779_FULLCAPREP	0x10
#define MAX77779_TTE		0x11	/* 5.625 s, 0xffff unknown */
#define MAX77779_DESIGNCAP	0x18
#define MAX77779_AVGVCELL		0x19
#define MAX77779_VCELL		0x1a
#define MAX77779_TEMP		0x1b	/* thermistor, signed 1/256 degC */
#define MAX77779_CURRENT	0x1c	/* signed 1.5625 uV / Rsense */
#define MAX77779_AVGCURRENT	0x1d
#define MAX77779_TTF		0x20
#define MAX77779_DEVNAME		0x21
#define MAX77779_FSTAT		0x3d
#define MAX77779_POR		BIT(1)
#define MAX77779_ABSENT		BIT(3)
#define MAX77779_NOT_READY	GENMASK(1, 0)

/* The PMIC function of the same chip, a second I2C address */
#define MAX77779_PMIC_ADDR	0x66
#define MAX77779_PMIC_FW_REV	0x62
#define MAX77779_PMIC_FW_SUB_REV	0x63

struct max77779_battery {
	struct regmap *map;
	struct power_supply *supply;
	struct delayed_work poll;
	u32 rsense;
	bool temp_valid;
};

static bool max77779_readable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX77779_STATUS:
	case MAX77779_REPCAP:
	case MAX77779_REPSOC:
	case MAX77779_CONFIG:
	case MAX77779_FULLCAPREP:
	case MAX77779_DESIGNCAP:
	case MAX77779_AVGVCELL:
	case MAX77779_VCELL:
	case MAX77779_DEVNAME:
	case MAX77779_FSTAT:
	case MAX77779_TEMP:
	case MAX77779_CURRENT:
	case MAX77779_AVGCURRENT:
	case MAX77779_TTE:
	case MAX77779_TTF:
		return true;
	default:
		return false;
	}
}

static bool max77779_writeable(struct device *dev, unsigned int reg)
{
	return false;
}

static const struct regmap_config max77779_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.max_register = MAX77779_FSTAT,
	.readable_reg = max77779_readable,
	.writeable_reg = max77779_writeable,
	.cache_type = REGCACHE_NONE,
};

/*
 * After the pack ran flat the gauge restarts with POR set and no model
 * reloaded (nothing on Linux reloads it). It still measures and, once FSTAT
 * says the data are ready, reports state of charge from its power-on
 * configuration: less exact than with Google's model, far better than no
 * battery at all. Nothing is written to the gauge either way.
 */
static bool trust_por = true;
module_param(trust_por, bool, 0644);
MODULE_PARM_DESC(trust_por, "Report the gauge after a power-on reset with no model reloaded (default on)");

static int max77779_ready(struct max77779_battery *battery, u32 *status)
{
	u32 fstat;
	int ret;

	ret = regmap_read(battery->map, MAX77779_STATUS, status);
	if (ret)
		return ret;
	ret = regmap_read(battery->map, MAX77779_FSTAT, &fstat);
	if (ret)
		return ret;
	/* Never clear POR, reload a model, or publish uninitialised data. */
	if (fstat & MAX77779_NOT_READY)
		return -EAGAIN;
	if ((*status & MAX77779_POR) && !trust_por)
		return -EAGAIN;
	return 0;
}

static int max77779_get_property(struct power_supply *supply,
				 enum power_supply_property property,
				 union power_supply_propval *value)
{
	struct max77779_battery *battery = power_supply_get_drvdata(supply);
	s64 ibat;
	u64 charge;
	u32 status, raw;
	unsigned int reg;
	int ret;

	ret = max77779_ready(battery, &status);
	if (ret)
		return ret;
	switch (property) {
	case POWER_SUPPLY_PROP_PRESENT:
		value->intval = !(status & MAX77779_ABSENT);
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		if (!battery->temp_valid)
			return -ENODATA;
		ret = regmap_read(battery->map, MAX77779_TEMP, &reg);
		if (ret)
			return ret;
		/* tenths of a degree from the signed 1/256 degC register */
		value->intval = ((s16)reg * 10) / 256;
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		/*
		 * The charger is a separate device: take its status when one
		 * supplies this gauge, else report it as unknown.
		 */
		if (power_supply_get_property_from_supplier(supply, POWER_SUPPLY_PROP_STATUS,
							    value))
			value->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		value->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		value->strval = "MAX77779";
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		reg = MAX77779_REPSOC;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		reg = MAX77779_VCELL;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		reg = MAX77779_AVGVCELL;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		reg = MAX77779_REPCAP;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		reg = MAX77779_FULLCAPREP;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		reg = MAX77779_DESIGNCAP;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		reg = MAX77779_CURRENT;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		reg = MAX77779_AVGCURRENT;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		reg = MAX77779_TTE;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		reg = MAX77779_TTF;
		break;
	default:
		return -EINVAL;
	}
	if (status & MAX77779_ABSENT)
		return -ENODATA;
	/*
	 * After a power-on reset with no model reloaded, the charges and the
	 * time estimates come from the gauge's default pack, not this one:
	 * withhold them and keep the percentage, voltage and current.
	 */
	if (status & MAX77779_POR) {
		switch (property) {
		case POWER_SUPPLY_PROP_CHARGE_NOW:
		case POWER_SUPPLY_PROP_CHARGE_FULL:
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
			return -ENODATA;
		default:
			break;
		}
	}
	ret = regmap_read(battery->map, reg, &raw);
	if (ret)
		return ret;
	switch (property) {
	case POWER_SUPPLY_PROP_CAPACITY:
		if ((raw >> 8) > 100)
			return -ERANGE;
		value->intval = raw >> 8;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		value->intval = div_u64((u64)raw * 78125, 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		/* positive while charging, as power_supply expects */
		ibat = div_s64((s64)(s16)raw * 1562500, battery->rsense);
		if (ibat > INT_MAX || ibat < INT_MIN)
			return -ERANGE;
		value->intval = ibat;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		/* the gauge reads all ones when it has no estimate */
		if (raw == 0xffff)
			return -ENODATA;
		value->intval = raw * 5625 / 1000;
		break;
	default:
		/* 5 uVh / Rsense, with Rsense expressed in micro-ohms. */
		charge = div_u64((u64)raw * 5000000, battery->rsense);
		if (charge > INT_MAX)
			return -ERANGE;
		value->intval = charge;
		break;
	}
	return 0;
}

static const enum power_supply_property max77779_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
};

static const struct power_supply_desc max77779_supply = {
	.name = "max77779-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = max77779_properties,
	.num_properties = ARRAY_SIZE(max77779_properties),
	.get_property = max77779_get_property,
};

static void max77779_poll(struct work_struct *work)
{
	struct max77779_battery *battery;

	battery = container_of(to_delayed_work(work), struct max77779_battery, poll);
	power_supply_changed(battery->supply);
	queue_delayed_work(system_freezable_wq, &battery->poll, 30 * HZ);
}

static void max77779_cancel_poll(void *data)
{
	struct max77779_battery *battery = data;

	cancel_delayed_work_sync(&battery->poll);
}

/*
 * Google's driver reports a fixed 22 C instead of TEMP on firmware revision
 * 1 below subrevision 15. The revision is in the chip's PMIC function, at a
 * second address on the same bus; read it without claiming that address, so
 * a PMIC driver can still bind there. Unknown firmware: no temperature.
 */
static bool max77779_temp_valid(struct i2c_client *client)
{
	union i2c_smbus_data rev, sub;

	if (i2c_smbus_xfer(client->adapter, MAX77779_PMIC_ADDR, 0, I2C_SMBUS_READ,
			   MAX77779_PMIC_FW_REV, I2C_SMBUS_BYTE_DATA, &rev) ||
	    i2c_smbus_xfer(client->adapter, MAX77779_PMIC_ADDR, 0, I2C_SMBUS_READ,
			   MAX77779_PMIC_FW_SUB_REV, I2C_SMBUS_BYTE_DATA, &sub))
		return false;
	dev_dbg(&client->dev, "Firmware %u.%u\n", rev.byte, sub.byte);
	return rev.byte > 1 || (rev.byte == 1 && sub.byte >= 15);
}

static int max77779_probe(struct i2c_client *client)
{
	struct power_supply_config config = {};
	struct device *dev = &client->dev;
	struct max77779_battery *battery;
	u32 raw;
	int ret;

	battery = devm_kzalloc(dev, sizeof(*battery), GFP_KERNEL);
	if (!battery)
		return -ENOMEM;
	ret = device_property_read_u32(dev, "shunt-resistor-micro-ohms", &battery->rsense);
	if (ret)
		return dev_err_probe(dev, ret, "Sense resistor is required\n");
	if (battery->rsense < 100 || battery->rsense > 100000)
		return -EINVAL;
	battery->map = devm_regmap_init_i2c(client, &max77779_regmap);
	if (IS_ERR(battery->map))
		return PTR_ERR(battery->map);
	ret = regmap_read(battery->map, MAX77779_DEVNAME, &raw);
	if (ret)
		return ret;
	if ((raw >> 8) != 0x51 && (raw >> 8) != 0x62 && (raw >> 8) != 0x63)
		return -ENODEV;
	ret = max77779_ready(battery, &raw);
	if (ret)
		return dev_err_probe(dev, ret, "Firmware model is not ready\n");
	if (raw & MAX77779_POR)
		dev_warn(dev, "Power-on reset, no model reloaded: state of charge from the gauge's defaults\n");
	battery->temp_valid = max77779_temp_valid(client);
	config.drv_data = battery;
	config.fwnode = dev_fwnode(dev);
	battery->supply = devm_power_supply_register(dev, &max77779_supply, &config);
	if (IS_ERR(battery->supply))
		return PTR_ERR(battery->supply);
	INIT_DELAYED_WORK(&battery->poll, max77779_poll);
	ret = devm_add_action_or_reset(dev, max77779_cancel_poll, battery);
	if (ret)
		return ret;
	queue_delayed_work(system_freezable_wq, &battery->poll, 30 * HZ);
	dev_info(dev, "Read-only fuel gauge; firmware model retained; temperature %s\n",
		 battery->temp_valid ? "reported" : "withheld (firmware unknown or uncalibrated)");
	return 0;
}

static const struct of_device_id max77779_of_match[] = {
	{ .compatible = "maxim,max77779-fg" },
	{}
};
MODULE_DEVICE_TABLE(of, max77779_of_match);

static const struct i2c_device_id max77779_ids[] = {
	{ "max77779-battery" },
	{}
};
MODULE_DEVICE_TABLE(i2c, max77779_ids);

static struct i2c_driver max77779_driver = {
	.driver = {
		.name = "max77779-battery",
		.of_match_table = max77779_of_match,
	},
	.probe = max77779_probe,
	.id_table = max77779_ids,
};
module_i2c_driver(max77779_driver);

MODULE_DESCRIPTION("Read-only MAX77779 fuel gauge with a retained firmware model");
MODULE_LICENSE("GPL");
