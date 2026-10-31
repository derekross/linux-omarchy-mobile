// SPDX-License-Identifier: GPL-2.0+
//
// Fuel gauge driver for Maxim 17042 / 8966 / 8997
//  Note that Maxim 8966 and 8997 are mfd and this is its subdevice.
//
// Copyright (C) 2011 Samsung Electronics
// MyungJoo Ham <myungjoo.ham@samsung.com>
//
// This driver is based on max17040_battery.c

#include <linux/acpi.h>
#include <linux/devm-helpers.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/power/max17042_battery.h>
#include <linux/of.h>
#include <linux/regmap.h>

/* Status register bits */
#define STATUS_POR_BIT         (1 << 1)
#define STATUS_BST_BIT         (1 << 3)
#define STATUS_DSOCI_BIT       (1 << 7)
#define STATUS_VMN_BIT         (1 << 8)
#define STATUS_TMN_BIT         (1 << 9)
#define STATUS_SMN_BIT         (1 << 10)
#define STATUS_BI_BIT          (1 << 11)
#define STATUS_VMX_BIT         (1 << 12)
#define STATUS_TMX_BIT         (1 << 13)
#define STATUS_SMX_BIT         (1 << 14)
#define STATUS_BR_BIT          (1 << 15)

/* Interrupt mask bits */
#define CFG_ALRT_BIT_ENBL	(1 << 2)
#define CFG2_DSOCI_BIT_ENBL	(1 << 7)

#define VFSOC0_LOCK		0x0000
#define VFSOC0_UNLOCK		0x0080
#define MODEL_UNLOCK1	0X0059
#define MODEL_UNLOCK2	0X00C4
#define MODEL_LOCK1		0X0000
#define MODEL_LOCK2		0X0000

#define dQ_ACC_DIV	0x4
#define dP_ACC_100	0x1900
#define dP_ACC_200	0x3200

#define MAX17042_VMAX_TOLERANCE		50 /* 50 mV */

#define MAX17042_CRITICAL_SOC		0x03

#define MAX17042_CURRENT_LSB		1562500ll /* 1.5625µV/Rsense */
#define MAX17042_CAPACITY_LSB		5000000ll /* 5.0µVH/Rsense */
#define MAX17042_TIME_LSB		5625 / 1000 /* s */
#define MAX17042_VOLTAGE_LSB		625 / 8 /* µV */
#define MAX17042_RESISTANCE_LSB		1 / 4096 /* Ω */
#define MAX17042_TEMPERATURE_LSB	1 / 256 /* °C */

#define MAX17055_DQACC_DIV		32
#define MAX17055_DPACC_FACTOR		44138
#define MAX17055_DPACC_VCHG_FACTOR	51200
#define MAX17055_FSTAT_DNR_BIT		BIT(0)
#define MAX17055_VCHG_THRESHOLD_UV	4275000
#define MAX17055_DNR_POLL_US		10000
#define MAX17055_DNR_TIMEOUT_US		2000000
#define MAX17055_INIT_RETRY_DELAY_MS	10000
#define MAX17055_REFRESH_POLL_US		10000
#define MAX17055_REFRESH_TIMEOUT_US	1000000

#define MAX77759_FG_CONFIG2_LDMDL	BIT(5)
#define MAX77759_FG_MODEL_LOCKED	0xffff
#define MAX77759_FG_CGAIN_DEFAULT	0x0400
#define MAX77759_FG_CV_HALFTIME		0x0600
#define MAX77759_FG_LDMDL_POLL_US	50000
#define MAX77759_FG_LDMDL_TIMEOUT_US	1000000
#define MAX77759_FG_LOAD_RETRIES	3

/*
 * Register values that come with a MAX77759 ("M5") battery model, in the
 * order of the "maxim,fg-params" property.
 */
struct max77759_fg_params {
	u16 iavg_empty;
	u16 relax_cfg;
	u16 learn_cfg;
	u16 config;
	u16 config2;
	u16 full_soc_thr;
	u16 full_cap_rep;
	u16 design_cap;
	u16 dpacc;
	u16 dqacc;
	u16 full_cap_nom;
	u16 v_empty;
	u16 qrtable00;
	u16 qrtable10;
	u16 qrtable20;
	u16 qrtable30;
	u16 rcomp0;
	u16 tempco;
	u16 ichg_term;
	u16 tgain;
	u16 toff;
	u16 curve;
	u16 misc_cfg;
	u16 at_rate;
	u16 convg_cfg;
	u16 filter_cfg;
	u16 task_period;
	u16 cgain;
};

struct max77759_fg_model {
	u16 table[MAX17042_CHARACTERIZATION_DATA_SIZE];
	union {
		struct max77759_fg_params params;
		u16 words[sizeof(struct max77759_fg_params) / sizeof(u16)];
	};
};

struct max17042_chip {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *battery;
	enum max170xx_chip_type chip_type;
	struct max17042_config_data *config_data;
	struct max77759_fg_model *m5_model;
	struct delayed_work work;
	int    irq;
	int    task_period;
	bool   enable_current_sense;
	bool   enable_por_init;
	bool   enable_vchg_override;
	bool   init_complete;
	bool   hib_restore_pending;
	u16    hib_cfg;
	unsigned int r_sns;
	int    vmin;	/* in millivolts */
	int    vmax;	/* in millivolts */
	int    temp_min;	/* in tenths of degree Celsius */
	int    temp_max;	/* in tenths of degree Celsius */
};

static enum power_supply_property max17042_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_TEMP_MIN,
	POWER_SUPPLY_PROP_TEMP_MAX,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	// these two have to be at the end on the list
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
};

static int max17042_get_temperature(struct max17042_chip *chip, int *temp)
{
	int ret;
	u32 data;
	struct regmap *map = chip->regmap;

	ret = regmap_read(map, MAX17042_TEMP, &data);
	if (ret < 0)
		return ret;

	*temp = sign_extend32(data, 15);
	/* The value is converted into deci-centigrade scale */
	*temp = *temp * 10 * MAX17042_TEMPERATURE_LSB;
	return 0;
}

static int max17042_get_status(struct max17042_chip *chip, int *status)
{
	int ret, charge_full, charge_now;
	int current_now;
	bool supplied, full;
	u32 data;

	ret = power_supply_am_i_supplied(chip->battery);
	if (ret == 0) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	/*
	 * A negative value means no supplier is described, e.g. a charger the
	 * bootloader left running that no driver owns. The gauge's own current
	 * sense still tells which way charge flows, so use it when it's there.
	 */
	supplied = ret > 0;
	if (!supplied && !chip->enable_current_sense) {
		*status = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	}

	/*
	 * The MAX170xx has builtin end-of-charge detection and will update
	 * FullCAP to match RepCap when it detects end of charging.
	 *
	 * When this cycle the battery gets charged to a higher (calculated)
	 * capacity than the previous cycle then FullCAP will get updated
	 * continuously once end-of-charge detection kicks in, so allow the
	 * 2 to differ a bit.
	 */

	ret = regmap_read(chip->regmap, MAX17042_FullCAP, &charge_full);
	if (ret < 0)
		return ret;

	ret = regmap_read(chip->regmap, MAX17042_RepCap, &charge_now);
	if (ret < 0)
		return ret;

	full = (charge_full - charge_now) <= MAX17042_FULL_THRESHOLD;
	if (supplied && full) {
		*status = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	/*
	 * Even though we are supplied, we may still be discharging if the
	 * supply is e.g. only delivering 5V 0.5A. Check current if available.
	 */
	if (!chip->enable_current_sense) {
		*status = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	ret = regmap_read(chip->regmap, MAX17042_Current, &data);
	if (ret < 0)
		return ret;

	current_now = sign_extend32(data, 15);

	if (current_now > 0)
		*status = POWER_SUPPLY_STATUS_CHARGING;
	else if (full && current_now == 0)
		*status = POWER_SUPPLY_STATUS_FULL;
	else
		*status = POWER_SUPPLY_STATUS_DISCHARGING;

	return 0;
}

static int max17042_get_battery_health(struct max17042_chip *chip, int *health)
{
	int temp, vavg, vbatt, ret;
	u32 val;

	ret = regmap_read(chip->regmap, MAX17042_AvgVCELL, &val);
	if (ret < 0)
		goto health_error;

	/* bits [0-3] unused */
	vavg = val * MAX17042_VOLTAGE_LSB;
	/* Convert to millivolts */
	vavg /= 1000;

	ret = regmap_read(chip->regmap, MAX17042_VCELL, &val);
	if (ret < 0)
		goto health_error;

	/* bits [0-3] unused */
	vbatt = val * MAX17042_VOLTAGE_LSB;
	/* Convert to millivolts */
	vbatt /= 1000;

	if (vavg < chip->vmin) {
		*health = POWER_SUPPLY_HEALTH_DEAD;
		goto out;
	}

	if (vbatt > size_add(chip->vmax, MAX17042_VMAX_TOLERANCE)) {
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		goto out;
	}

	ret = max17042_get_temperature(chip, &temp);
	if (ret < 0)
		goto health_error;

	if (temp < chip->temp_min) {
		*health = POWER_SUPPLY_HEALTH_COLD;
		goto out;
	}

	if (temp > chip->temp_max) {
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
		goto out;
	}

	*health = POWER_SUPPLY_HEALTH_GOOD;

out:
	return 0;

health_error:
	return ret;
}

static int max17042_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17042_chip *chip = power_supply_get_drvdata(psy);
	struct regmap *map = chip->regmap;
	int ret;
	u32 data;
	u64 data64;

	if (!READ_ONCE(chip->init_complete))
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = max17042_get_status(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = regmap_read(map, MAX17042_STATUS, &data);
		if (ret < 0)
			return ret;

		if (data & MAX17042_STATUS_BattAbsent)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = regmap_read(map, MAX17042_Cycles, &data);
		if (ret < 0)
			return ret;

		val->intval = data;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = regmap_read(map, MAX17042_MinMaxVolt, &data);
		if (ret < 0)
			return ret;

		val->intval = data >> 8;
		val->intval *= 20000; /* Units of LSB = 20mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		ret = regmap_read(map, MAX17042_MinMaxVolt, &data);
		if (ret < 0)
			return ret;

		val->intval = (data & 0xff) * 20000; /* Units of 20mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17042)
			ret = regmap_read(map, MAX17042_V_empty, &data);
		else
			ret = regmap_read(map, MAX17047_V_empty, &data);
		if (ret < 0)
			return ret;

		val->intval = data >> 7;
		val->intval *= 10000; /* Units of LSB = 10mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = regmap_read(map, MAX17042_VCELL, &data);
		if (ret < 0)
			return ret;

		val->intval = data * MAX17042_VOLTAGE_LSB;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = regmap_read(map, MAX17042_AvgVCELL, &data);
		if (ret < 0)
			return ret;

		val->intval = data * MAX17042_VOLTAGE_LSB;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = regmap_read(map, MAX17042_OCVInternal, &data);
		if (ret < 0)
			return ret;

		val->intval = data * MAX17042_VOLTAGE_LSB;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (chip->enable_current_sense)
			ret = regmap_read(map, MAX17042_RepSOC, &data);
		else
			ret = regmap_read(map, MAX17042_VFSOC, &data);
		if (ret < 0)
			return ret;

		val->intval = data >> 8;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = regmap_read(map, MAX17042_DesignCap, &data);
		if (ret < 0)
			return ret;

		data64 = data * MAX17042_CAPACITY_LSB;
		data64 *= chip->task_period;
		do_div(data64, MAX17042_DEFAULT_TASK_PERIOD);
		do_div(data64, chip->r_sns);
		val->intval = data64;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = regmap_read(map, MAX17042_FullCAP, &data);
		if (ret < 0)
			return ret;

		data64 = data * MAX17042_CAPACITY_LSB;
		data64 *= chip->task_period;
		do_div(data64, MAX17042_DEFAULT_TASK_PERIOD);
		do_div(data64, chip->r_sns);
		val->intval = data64;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = regmap_read(map, MAX17042_RepCap, &data);
		if (ret < 0)
			return ret;

		data64 = data * MAX17042_CAPACITY_LSB;
		data64 *= chip->task_period;
		do_div(data64, MAX17042_DEFAULT_TASK_PERIOD);
		do_div(data64, chip->r_sns);
		val->intval = data64;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = regmap_read(map, MAX17042_QH, &data);
		if (ret < 0)
			return ret;

		data64 = sign_extend64(data, 15) * MAX17042_CAPACITY_LSB;
		data64 *= chip->task_period;
		data64 = div_s64(data64, MAX17042_DEFAULT_TASK_PERIOD);
		val->intval = div_s64(data64, chip->r_sns);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = max17042_get_temperature(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = regmap_read(map, MAX17042_TALRT_Th, &data);
		if (ret < 0)
			return ret;
		/* LSB is Alert Minimum. In deci-centigrade */
		val->intval = sign_extend32(data & 0xff, 7) * 10;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = regmap_read(map, MAX17042_TALRT_Th, &data);
		if (ret < 0)
			return ret;
		/* MSB is Alert Maximum. In deci-centigrade */
		val->intval = sign_extend32(data >> 8, 7) * 10;
		break;
	case POWER_SUPPLY_PROP_TEMP_MIN:
		val->intval = chip->temp_min;
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		val->intval = chip->temp_max;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = max17042_get_battery_health(chip, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (chip->enable_current_sense) {
			ret = regmap_read(map, MAX17042_Current, &data);
			if (ret < 0)
				return ret;

			data64 = sign_extend64(data, 15) * MAX17042_CURRENT_LSB;
			val->intval = div_s64(data64, chip->r_sns);
		} else {
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		if (chip->enable_current_sense) {
			ret = regmap_read(map, MAX17042_AvgCurrent, &data);
			if (ret < 0)
				return ret;

			data64 = sign_extend64(data, 15) * MAX17042_CURRENT_LSB;
			val->intval = div_s64(data64, chip->r_sns);
		} else {
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = regmap_read(map, MAX17042_ICHGTerm, &data);
		if (ret < 0)
			return ret;

		data64 = data * MAX17042_CURRENT_LSB;
		val->intval = div_s64(data64, chip->r_sns);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = regmap_read(map, MAX17042_TTE, &data);
		if (ret < 0)
			return ret;

		/* when charging, the value is not meaningful */
		if (data == U16_MAX)
			return -ENODATA;

		val->intval = data * MAX17042_TIME_LSB;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		if (chip->chip_type != MAXIM_DEVICE_TYPE_MAX17055 &&
		    chip->chip_type != MAXIM_DEVICE_TYPE_MAX77759)
			return -EINVAL;

		ret = regmap_read(map, MAX17055_TTF, &data);
		if (ret < 0)
			return ret;

		/* when discharging, the value is not meaningful */
		if (data == U16_MAX)
			return -ENODATA;

		val->intval = data * MAX17042_TIME_LSB;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int max17042_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
	struct max17042_chip *chip = power_supply_get_drvdata(psy);
	struct regmap *map = chip->regmap;
	int ret = 0;
	u32 data;
	int8_t temp;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = regmap_read(map, MAX17042_TALRT_Th, &data);
		if (ret < 0)
			return ret;

		/* Input in deci-centigrade, convert to centigrade */
		temp = val->intval / 10;
		/* force min < max */
		if (temp >= (int8_t)(data >> 8))
			temp = (int8_t)(data >> 8) - 1;
		/* Write both MAX and MIN ALERT */
		data = (data & 0xff00) + temp;
		ret = regmap_write(map, MAX17042_TALRT_Th, data);
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = regmap_read(map, MAX17042_TALRT_Th, &data);
		if (ret < 0)
			return ret;

		/* Input in Deci-Centigrade, convert to centigrade */
		temp = val->intval / 10;
		/* force max > min */
		if (temp <= (int8_t)(data & 0xff))
			temp = (int8_t)(data & 0xff) + 1;
		/* Write both MAX and MIN ALERT */
		data = (data & 0xff) + (temp << 8);
		ret = regmap_write(map, MAX17042_TALRT_Th, data);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int max17042_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

static int max17042_write_verify_reg(struct regmap *map, u8 reg, u32 value)
{
	int retries = 8;
	int ret;
	u32 read_value;

	do {
		ret = regmap_write(map, reg, value);
		regmap_read(map, reg, &read_value);
		if (read_value != value) {
			ret = -EIO;
			retries--;
		}
	} while (retries && read_value != value);

	if (ret < 0)
		pr_err("%s: err %d\n", __func__, ret);

	return ret;
}

static int max17055_write_verify_reg(struct regmap *map, u8 reg, u32 value)
{
	u32 read_value;
	int ret;

	ret = regmap_write(map, reg, value);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	ret = regmap_read(map, reg, &read_value);
	if (ret)
		return ret;

	return read_value == value ? 0 : -EIO;
}

static inline void max17042_override_por(struct regmap *map,
					 u8 reg, u16 value)
{
	if (value)
		regmap_write(map, reg, value);
}

static inline void max17042_unlock_model(struct max17042_chip *chip)
{
	struct regmap *map = chip->regmap;

	regmap_write(map, MAX17042_MLOCKReg1, MODEL_UNLOCK1);
	regmap_write(map, MAX17042_MLOCKReg2, MODEL_UNLOCK2);
}

static inline void max17042_lock_model(struct max17042_chip *chip)
{
	struct regmap *map = chip->regmap;

	regmap_write(map, MAX17042_MLOCKReg1, MODEL_LOCK1);
	regmap_write(map, MAX17042_MLOCKReg2, MODEL_LOCK2);
}

static inline void max17042_write_model_data(struct max17042_chip *chip,
					u8 addr, int size)
{
	struct regmap *map = chip->regmap;
	int i;

	for (i = 0; i < size; i++)
		regmap_write(map, addr + i,
			chip->config_data->cell_char_tbl[i]);
}

static inline void max17042_read_model_data(struct max17042_chip *chip,
					u8 addr, u16 *data, int size)
{
	struct regmap *map = chip->regmap;
	int i;
	u32 tmp;

	for (i = 0; i < size; i++) {
		regmap_read(map, addr + i, &tmp);
		data[i] = (u16)tmp;
	}
}

static inline int max17042_model_data_compare(struct max17042_chip *chip,
					u16 *data1, u16 *data2, int size)
{
	int i;

	if (memcmp(data1, data2, size)) {
		dev_err(chip->dev, "%s compare failed\n", __func__);
		for (i = 0; i < size; i++)
			dev_info(chip->dev, "0x%x, 0x%x",
				data1[i], data2[i]);
		dev_info(chip->dev, "\n");
		return -EINVAL;
	}
	return 0;
}

static int max17042_init_model(struct max17042_chip *chip)
{
	int ret;
	int table_size = ARRAY_SIZE(chip->config_data->cell_char_tbl);
	u16 *temp_data;

	temp_data = kcalloc(table_size, sizeof(*temp_data), GFP_KERNEL);
	if (!temp_data)
		return -ENOMEM;

	max17042_unlock_model(chip);
	max17042_write_model_data(chip, MAX17042_MODELChrTbl,
				table_size);
	max17042_read_model_data(chip, MAX17042_MODELChrTbl, temp_data,
				table_size);

	ret = max17042_model_data_compare(
		chip,
		chip->config_data->cell_char_tbl,
		temp_data,
		table_size);

	max17042_lock_model(chip);
	kfree(temp_data);

	return ret;
}

static int max17042_verify_model_lock(struct max17042_chip *chip)
{
	int i;
	int table_size = ARRAY_SIZE(chip->config_data->cell_char_tbl);
	u16 *temp_data;
	int ret = 0;

	temp_data = kcalloc(table_size, sizeof(*temp_data), GFP_KERNEL);
	if (!temp_data)
		return -ENOMEM;

	max17042_read_model_data(chip, MAX17042_MODELChrTbl, temp_data,
				table_size);
	for (i = 0; i < table_size; i++)
		if (temp_data[i])
			ret = -EINVAL;

	kfree(temp_data);
	return ret;
}

static void max17042_write_config_regs(struct max17042_chip *chip)
{
	struct max17042_config_data *config = chip->config_data;
	struct regmap *map = chip->regmap;

	regmap_write(map, MAX17042_CONFIG, config->config);
	regmap_write(map, MAX17042_LearnCFG, config->learn_cfg);
	regmap_write(map, MAX17042_FilterCFG,
			config->filter_cfg);
	regmap_write(map, MAX17042_RelaxCFG, config->relax_cfg);
	if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17047 ||
			chip->chip_type == MAXIM_DEVICE_TYPE_MAX17050 ||
			chip->chip_type == MAXIM_DEVICE_TYPE_MAX17055 ||
			chip->chip_type == MAXIM_DEVICE_TYPE_MAX77759)
		regmap_write(map, MAX17047_FullSOCThr,
						config->full_soc_thresh);
}

static void  max17042_write_custom_regs(struct max17042_chip *chip)
{
	struct max17042_config_data *config = chip->config_data;
	struct regmap *map = chip->regmap;

	max17042_write_verify_reg(map, MAX17042_RCOMP0, config->rcomp0);
	max17042_write_verify_reg(map, MAX17042_TempCo,	config->tcompc0);
	max17042_write_verify_reg(map, MAX17042_ICHGTerm, config->ichgt_term);
	if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17042) {
		regmap_write(map, MAX17042_EmptyTempCo,	config->empty_tempco);
		max17042_write_verify_reg(map, MAX17042_K_empty0,
					config->kempty0);
	} else {
		max17042_write_verify_reg(map, MAX17047_QRTbl00,
						config->qrtbl00);
		max17042_write_verify_reg(map, MAX17047_QRTbl10,
						config->qrtbl10);
		max17042_write_verify_reg(map, MAX17047_QRTbl20,
						config->qrtbl20);
		max17042_write_verify_reg(map, MAX17047_QRTbl30,
						config->qrtbl30);
	}
}

static void max17042_update_capacity_regs(struct max17042_chip *chip)
{
	struct max17042_config_data *config = chip->config_data;
	struct regmap *map = chip->regmap;

	max17042_write_verify_reg(map, MAX17042_FullCAP,
				config->fullcap);
	regmap_write(map, MAX17042_DesignCap, config->design_cap);
	max17042_write_verify_reg(map, MAX17042_FullCAPNom,
				config->fullcapnom);
}

static void max17042_reset_vfsoc0_reg(struct max17042_chip *chip)
{
	unsigned int vfSoc;
	struct regmap *map = chip->regmap;

	regmap_read(map, MAX17042_VFSOC, &vfSoc);
	regmap_write(map, MAX17042_VFSOC0Enable, VFSOC0_UNLOCK);
	max17042_write_verify_reg(map, MAX17042_VFSOC0, vfSoc);
	regmap_write(map, MAX17042_VFSOC0Enable, VFSOC0_LOCK);
}

static void max17042_load_new_capacity_params(struct max17042_chip *chip)
{
	u32 full_cap0, rep_cap, dq_acc, vfSoc;
	u32 rem_cap;

	struct max17042_config_data *config = chip->config_data;
	struct regmap *map = chip->regmap;

	regmap_read(map, MAX17042_FullCAP0, &full_cap0);
	regmap_read(map, MAX17042_VFSOC, &vfSoc);

	/* fg_vfSoc needs to shifted by 8 bits to get the
	 * perc in 1% accuracy, to get the right rem_cap multiply
	 * full_cap0, fg_vfSoc and devide by 100
	 */
	rem_cap = ((vfSoc >> 8) * full_cap0) / 100;
	max17042_write_verify_reg(map, MAX17042_RemCap, rem_cap);

	rep_cap = rem_cap;
	max17042_write_verify_reg(map, MAX17042_RepCap, rep_cap);

	/* Write dQ_acc to 200% of Capacity and dP_acc to 200% */
	dq_acc = config->fullcap / dQ_ACC_DIV;
	max17042_write_verify_reg(map, MAX17042_dQacc, dq_acc);
	max17042_write_verify_reg(map, MAX17042_dPacc, dP_ACC_200);

	max17042_write_verify_reg(map, MAX17042_FullCAP,
			config->fullcap);
	regmap_write(map, MAX17042_DesignCap,
			config->design_cap);
	max17042_write_verify_reg(map, MAX17042_FullCAPNom,
			config->fullcapnom);
	/* Update SOC register with new SOC */
	regmap_write(map, MAX17042_RepSOC, vfSoc);
}

/*
 * Block write all the override values coming from config_data.
 * This function MUST be called before the POR initialization procedure
 * specified by maxim.
 */
static inline void max17042_override_por_values(struct max17042_chip *chip)
{
	struct regmap *map = chip->regmap;
	struct max17042_config_data *config = chip->config_data;

	max17042_override_por(map, MAX17042_TGAIN, config->tgain);
	max17042_override_por(map, MAX17042_TOFF, config->toff);
	max17042_override_por(map, MAX17042_CGAIN, config->cgain);
	max17042_override_por(map, MAX17042_COFF, config->coff);

	max17042_override_por(map, MAX17042_VALRT_Th, config->valrt_thresh);
	max17042_override_por(map, MAX17042_TALRT_Th, config->talrt_thresh);
	max17042_override_por(map, MAX17042_SALRT_Th,
						config->soc_alrt_thresh);
	max17042_override_por(map, MAX17042_CONFIG, config->config);
	max17042_override_por(map, MAX17042_SHDNTIMER, config->shdntimer);

	if (chip->chip_type != MAXIM_DEVICE_TYPE_MAX17055) {
		max17042_override_por(map, MAX17042_DesignCap,
				      config->design_cap);
		max17042_override_por(map, MAX17042_ICHGTerm,
				      config->ichgt_term);
	}

	max17042_override_por(map, MAX17042_AtRate, config->at_rate);
	max17042_override_por(map, MAX17042_LearnCFG, config->learn_cfg);
	max17042_override_por(map, MAX17042_FilterCFG, config->filter_cfg);
	max17042_override_por(map, MAX17042_RelaxCFG, config->relax_cfg);
	max17042_override_por(map, MAX17042_MiscCFG, config->misc_cfg);

	max17042_override_por(map, MAX17042_FullCAP, config->fullcap);
	max17042_override_por(map, MAX17042_FullCAPNom, config->fullcapnom);
	if (chip->chip_type != MAXIM_DEVICE_TYPE_MAX17055) {
		max17042_override_por(map, MAX17042_dQacc, config->dqacc);
		max17042_override_por(map, MAX17042_dPacc, config->dpacc);
	}

	max17042_override_por(map, MAX17042_RCOMP0, config->rcomp0);
	max17042_override_por(map, MAX17042_TempCo, config->tcompc0);

	if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17042) {
		max17042_override_por(map, MAX17042_MaskSOC, config->masksoc);
		max17042_override_por(map, MAX17042_SOC_empty, config->socempty);
		max17042_override_por(map, MAX17042_V_empty, config->vempty);
		max17042_override_por(map, MAX17042_EmptyTempCo, config->empty_tempco);
		max17042_override_por(map, MAX17042_K_empty0, config->kempty0);
	}

	if ((chip->chip_type == MAXIM_DEVICE_TYPE_MAX17042) ||
	    (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17047) ||
	    (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17050) ||
	    (chip->chip_type == MAXIM_DEVICE_TYPE_MAX77759)) {
		max17042_override_por(map, MAX17042_IAvg_empty, config->iavg_empty);
		max17042_override_por(map, MAX17042_TempNom, config->temp_nom);
		max17042_override_por(map, MAX17042_TempLim, config->temp_lim);
		max17042_override_por(map, MAX17042_FCTC, config->fctc);
	}

	if ((chip->chip_type == MAXIM_DEVICE_TYPE_MAX17047) ||
	    (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17050) ||
	    (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17055) ||
	    (chip->chip_type == MAXIM_DEVICE_TYPE_MAX77759)) {
		max17042_override_por(map, MAX17047_V_empty, config->vempty);
	}

	if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17055 &&
	    !chip->enable_vchg_override)
		max17042_override_por(map, MAX17055_ModelCfg, config->model_cfg);
}

static int max17055_override_battery_values(struct max17042_chip *chip)
{
	struct max17042_config_data *config = chip->config_data;
	struct regmap *map = chip->regmap;
	unsigned int design_cap;
	unsigned int model_cfg;
	unsigned int dqacc;
	u64 dpacc;
	int ret;

	if (config->design_cap) {
		ret = max17055_write_verify_reg(map, MAX17042_DesignCap,
						config->design_cap);
		if (ret)
			return ret;
	}

	if (config->dqacc) {
		ret = max17055_write_verify_reg(map, MAX17042_dQacc,
						config->dqacc);
		if (ret)
			return ret;
	}

	if (config->ichgt_term) {
		ret = max17055_write_verify_reg(map, MAX17042_ICHGTerm,
						config->ichgt_term);
		if (ret)
			return ret;
	}

	if (chip->enable_vchg_override) {
		ret = regmap_update_bits(map, MAX17055_ModelCfg,
					 MAX17055_MODELCFG_VCHG_BIT,
					 config->model_cfg &
					 MAX17055_MODELCFG_VCHG_BIT);
		if (ret)
			return ret;

		usleep_range(1000, 2000);
	}

	if (!config->design_cap && !config->dqacc &&
	    !chip->enable_vchg_override)
		return 0;

	ret = regmap_read(map, MAX17042_DesignCap, &design_cap);
	if (ret)
		return ret;

	ret = regmap_read(map, MAX17042_dQacc, &dqacc);
	if (ret)
		return ret;

	ret = regmap_read(map, MAX17055_ModelCfg, &model_cfg);
	if (ret)
		return ret;
	if (chip->enable_vchg_override &&
	    (model_cfg & MAX17055_MODELCFG_VCHG_BIT) !=
	    (config->model_cfg & MAX17055_MODELCFG_VCHG_BIT))
		return -EIO;

	if (!design_cap || !dqacc)
		return -ERANGE;

	dpacc = (u64)dqacc *
		(model_cfg & MAX17055_MODELCFG_VCHG_BIT ?
		 MAX17055_DPACC_VCHG_FACTOR : MAX17055_DPACC_FACTOR);
	do_div(dpacc, design_cap);
	if (dpacc > U16_MAX)
		return -ERANGE;

	return max17055_write_verify_reg(map, MAX17042_dPacc, (u16)dpacc);
}

static int max17055_restore_hibernate(struct max17042_chip *chip)
{
	int restore_hib_ret;
	int soft_wakeup_ret;

	soft_wakeup_ret = regmap_write(chip->regmap, MAX17055_SoftWakeup, 0);
	restore_hib_ret = max17055_write_verify_reg(chip->regmap,
						    MAX17055_HibCfg,
						    chip->hib_cfg);
	if (!soft_wakeup_ret && !restore_hib_ret)
		chip->hib_restore_pending = false;

	return soft_wakeup_ret ?: restore_hib_ret;
}

static int max17055_init_chip(struct max17042_chip *chip)
{
	struct regmap *map = chip->regmap;
	unsigned int hib_cfg;
	unsigned int model_cfg;
	unsigned int fstat;
	int restore_ret;
	int ret;

	if (chip->hib_restore_pending) {
		ret = max17055_restore_hibernate(chip);
		if (ret)
			return ret;
	}

	ret = regmap_read_poll_timeout(map, MAX17042_FSTAT, fstat,
				       !(fstat & MAX17055_FSTAT_DNR_BIT),
				       MAX17055_DNR_POLL_US,
				       MAX17055_DNR_TIMEOUT_US);
	if (ret)
		return ret;

	ret = regmap_read(map, MAX17055_HibCfg, &hib_cfg);
	if (ret)
		return ret;

	chip->hib_cfg = hib_cfg;
	chip->hib_restore_pending = true;

	ret = regmap_write(map, MAX17055_SoftWakeup, 0x0090);
	if (ret)
		goto restore_hibernate;

	ret = max17055_write_verify_reg(map, MAX17055_HibCfg, 0);
	if (ret)
		goto restore_hibernate;

	ret = regmap_write(map, MAX17055_SoftWakeup, 0);
	if (ret)
		goto restore_hibernate;

	max17042_override_por_values(chip);

	ret = max17055_override_battery_values(chip);
	if (ret)
		goto restore_hibernate;

	ret = regmap_write_bits(map, MAX17055_ModelCfg,
				MAX17055_MODELCFG_REFRESH_BIT,
				MAX17055_MODELCFG_REFRESH_BIT);
	if (ret)
		goto restore_hibernate;

	ret = regmap_read_poll_timeout(map, MAX17055_ModelCfg, model_cfg,
				       !(model_cfg &
					 MAX17055_MODELCFG_REFRESH_BIT),
				       MAX17055_REFRESH_POLL_US,
				       MAX17055_REFRESH_TIMEOUT_US);

restore_hibernate:
	restore_ret = max17055_restore_hibernate(chip);
	if (restore_ret)
		return restore_ret;

	return ret;
}

static int max17042_init_chip(struct max17042_chip *chip)
{
	struct regmap *map = chip->regmap;
	int ret;

	if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17055) {
		ret = max17055_init_chip(chip);
		if (ret)
			return ret;
	} else {
		max17042_override_por_values(chip);

		/* Allow signal debouncing and initial SOC reporting. */
		msleep(500);

		max17042_write_config_regs(chip);

		/* write cell characterization data */
		ret = max17042_init_model(chip);
		if (ret) {
			dev_err(chip->dev, "%s init failed\n",
				__func__);
			return -EIO;
		}

		ret = max17042_verify_model_lock(chip);
		if (ret) {
			dev_err(chip->dev, "%s lock verify failed\n",
				__func__);
			return -EIO;
		}
		/* write custom parameters */
		max17042_write_custom_regs(chip);

		/* update capacity params */
		max17042_update_capacity_regs(chip);

		/* delay must be atleast 350mS to allow VFSOC
		 * to be calculated from the new configuration
		 */
		msleep(350);

		/* reset vfsoc0 reg */
		max17042_reset_vfsoc0_reg(chip);

		/* load new capacity params */
		max17042_load_new_capacity_params(chip);
	}

	/* Init complete, Clear the POR bit */
	return regmap_clear_bits(map, MAX17042_STATUS, STATUS_POR_BIT);
}

static void max17042_set_soc_threshold(struct max17042_chip *chip, u16 off)
{
	struct regmap *map = chip->regmap;
	u32 soc, soc_tr;

	/* program interrupt thresholds such that we should
	 * get interrupt for every 'off' perc change in the soc
	 */
	if (chip->enable_current_sense)
		regmap_read(map, MAX17042_RepSOC, &soc);
	else
		regmap_read(map, MAX17042_VFSOC, &soc);
	soc >>= 8;
	soc_tr = (soc + off) << 8;
	if (off < soc)
		soc_tr |= soc - off;
	regmap_write(map, MAX17042_SALRT_Th, soc_tr);
}

static void max17042_set_critical_soc_threshold(struct max17042_chip *chip)
{
	struct regmap *map = chip->regmap;
	u32 soc;

	if (chip->enable_current_sense)
		regmap_read(map, MAX17042_RepSOC, &soc);
	else
		regmap_read(map, MAX17042_VFSOC, &soc);

	regmap_write(map, MAX17042_SALRT_Th,
		     ((soc >> 8) >= MAX17042_CRITICAL_SOC) ?
		     0xff00 + MAX17042_CRITICAL_SOC : 0xff00);
}

/*
 * The MAX17055 and the MAX77759 raise Status.dSOCi on every whole-percent
 * change of the SOC, so they don't need a SALRT window around the current SOC.
 */
static bool max17042_has_dsoci(struct max17042_chip *chip)
{
	return chip->chip_type == MAXIM_DEVICE_TYPE_MAX17055 ||
	       chip->chip_type == MAXIM_DEVICE_TYPE_MAX77759;
}

static void max17042_enable_soc_alerts(struct max17042_chip *chip)
{
	if (max17042_has_dsoci(chip)) {
		regmap_update_bits(chip->regmap, MAX17055_Config2,
				   CFG2_DSOCI_BIT_ENBL,
				   CFG2_DSOCI_BIT_ENBL);
		max17042_set_critical_soc_threshold(chip);
		return;
	}

	max17042_set_soc_threshold(chip, 1);
}

static void max17042_suspend_soc_alerts(struct max17042_chip *chip)
{
	if (!max17042_has_dsoci(chip))
		return;

	regmap_update_bits(chip->regmap, MAX17055_Config2,
			   CFG2_DSOCI_BIT_ENBL, 0);
	max17042_set_critical_soc_threshold(chip);
}

/*
 * The MAX77759 releases ALRT only at a fuel-gauge update after Status was
 * cleared, up to one task period later. The interrupt is level-triggered and
 * oneshot, so it would be taken again every few milliseconds until then, each
 * time with Status already clear. Keep it masked until the gauge has updated.
 */
static void max17042_alert_holdoff(struct max17042_chip *chip)
{
	unsigned int period_ms;

	if (chip->chip_type != MAXIM_DEVICE_TYPE_MAX77759)
		return;

	/* TaskPeriod counts 1/32768 s; allow half a period of margin */
	period_ms = DIV_ROUND_UP(READ_ONCE(chip->task_period) * 1000, 32768);
	msleep(period_ms + period_ms / 2);
}

static irqreturn_t max17042_thread_handler(int id, void *dev)
{
	struct max17042_chip *chip = dev;
	u32 val;
	int ret;

	ret = regmap_read(chip->regmap, MAX17042_STATUS, &val);
	if (ret)
		return IRQ_HANDLED;

	/* Nothing latched: not our interrupt, or ALRT not released yet */
	if (!val) {
		max17042_alert_holdoff(chip);
		return IRQ_NONE;
	}

	if ((val & STATUS_SMN_BIT) || (val & STATUS_SMX_BIT) ||
	    (val & STATUS_DSOCI_BIT)) {
		dev_dbg(chip->dev, "SOC threshold INTR\n");
		max17042_enable_soc_alerts(chip);
	}

	/* we implicitly handle all alerts via power_supply_changed */
	regmap_clear_bits(chip->regmap, MAX17042_STATUS,
			  0xFFFF & ~(STATUS_POR_BIT | STATUS_BST_BIT));

	power_supply_changed(chip->battery);
	max17042_alert_holdoff(chip);
	return IRQ_HANDLED;
}

static int max77759_fg_write_verify(struct regmap *map, unsigned int reg,
				    unsigned int value)
{
	unsigned int read_value;
	int ret;

	ret = regmap_write(map, reg, value);
	if (ret)
		return ret;

	ret = regmap_read(map, reg, &read_value);
	if (ret)
		return ret;

	return read_value == value ? 0 : -EIO;
}

static int max77759_fg_lock_model(struct regmap *map, bool lock)
{
	int ret;

	ret = regmap_write(map, MAX17042_MLOCKReg1, lock ? MODEL_LOCK1 : MODEL_UNLOCK1);
	if (ret)
		return ret;

	return regmap_write(map, MAX17042_MLOCKReg2, lock ? MODEL_LOCK2 : MODEL_UNLOCK2);
}

static int max77759_fg_check_table(struct regmap *map, const u16 *expected)
{
	unsigned int val;
	int i, ret;

	for (i = 0; i < MAX17042_CHARACTERIZATION_DATA_SIZE; i++) {
		ret = regmap_read(map, MAX17042_MODELChrTbl + i, &val);
		if (ret)
			return ret;
		if (val != (expected ? expected[i] : MAX77759_FG_MODEL_LOCKED))
			return -EIO;
	}

	return 0;
}

/* Write the characterization table, verify it, and lock it again. */
static int max77759_fg_load_table(struct max17042_chip *chip)
{
	const u16 *table = chip->m5_model->table;
	struct regmap *map = chip->regmap;
	int retries = MAX77759_FG_LOAD_RETRIES;
	int i, ret, lock_ret;

	do {
		ret = max77759_fg_lock_model(map, false);
		for (i = 0; !ret && i < MAX17042_CHARACTERIZATION_DATA_SIZE; i++)
			ret = regmap_write(map, MAX17042_MODELChrTbl + i, table[i]);
		if (!ret)
			ret = max77759_fg_check_table(map, table);
	} while (ret && --retries);

	/* A locked table reads back as all ones. */
	retries = MAX77759_FG_LOAD_RETRIES;
	do {
		lock_ret = max77759_fg_lock_model(map, true);
		if (!lock_ret)
			lock_ret = max77759_fg_check_table(map, NULL);
	} while (lock_ret && --retries);

	if (ret)
		dev_err(chip->dev, "cannot write the model table: %d\n", ret);
	if (lock_ret)
		dev_err(chip->dev, "cannot lock the model table: %d\n", lock_ret);

	return ret ?: lock_ret;
}

struct max77759_fg_reg_write {
	u8 reg;
	bool verify;
	size_t offset;
};

#define M5_REG(_reg, _verify, _field) \
	{ _reg, _verify, offsetof(struct max77759_fg_params, _field) }

/* Order and read-back checks as in the ModelGauge m5 load procedure. */
static const struct max77759_fg_reg_write max77759_fg_config_regs[] = {
	M5_REG(MAX17042_LearnCFG,	true,	learn_cfg),
	M5_REG(MAX17042_CONFIG,		false,	config),
	M5_REG(MAX17055_Config2,	false,	config2),
	M5_REG(MAX17047_FullSOCThr,	false,	full_soc_thr),
	M5_REG(MAX17042_FullCAP0,	true,	full_cap_rep),
	M5_REG(MAX17042_DesignCap,	true,	design_cap),
	M5_REG(MAX17042_dPacc,		true,	dpacc),
	M5_REG(MAX17042_dQacc,		true,	dqacc),
	M5_REG(MAX17042_FullCAPNom,	true,	full_cap_nom),
	M5_REG(MAX17047_V_empty,	false,	v_empty),
	M5_REG(MAX17047_QRTbl00,	true,	qrtable00),
	M5_REG(MAX17047_QRTbl10,	true,	qrtable10),
	M5_REG(MAX17047_QRTbl20,	true,	qrtable20),
	M5_REG(MAX17047_QRTbl30,	true,	qrtable30),
	M5_REG(MAX17042_RCOMP0,		true,	rcomp0),
	M5_REG(MAX17042_TempCo,		true,	tempco),
	M5_REG(MAX17042_TaskPeriod,	false,	task_period),
	M5_REG(MAX17042_ICHGTerm,	false,	ichg_term),
	M5_REG(MAX17042_TGAIN,		false,	tgain),
	M5_REG(MAX17042_TOFF,		false,	toff),
	M5_REG(MAX17042_MiscCFG,	false,	misc_cfg),
};

static int max77759_fg_write_reg(struct regmap *map, const struct max77759_fg_params *p,
				 const struct max77759_fg_reg_write *w)
{
	u16 value = *(const u16 *)((const u8 *)p + w->offset);

	if (w->verify)
		return max77759_fg_write_verify(map, w->reg, value);

	return regmap_write(map, w->reg, value);
}

static int max77759_fg_write_params(struct max17042_chip *chip)
{
	const struct max77759_fg_params *p = &chip->m5_model->params;
	struct regmap *map = chip->regmap;
	unsigned int vfsoc;
	int i, ret, lock_ret;

	ret = max77759_fg_write_verify(map, MAX17042_RepCap, 0);
	if (!ret)
		ret = max77759_fg_write_verify(map, MAX17042_RelaxCFG, p->relax_cfg);
	if (ret)
		return ret;

	/* VFSOC0Enable doubles as the unlock for the extra configuration. */
	ret = max77759_fg_write_verify(map, MAX17042_VFSOC0Enable, VFSOC0_UNLOCK);
	if (!ret)
		ret = regmap_read(map, MAX17042_VFSOC, &vfsoc);
	if (!ret)
		ret = max77759_fg_write_verify(map, MAX17042_VFSOC0, vfsoc);

	for (i = 0; !ret && i < ARRAY_SIZE(max77759_fg_config_regs); i++)
		ret = max77759_fg_write_reg(map, p, &max77759_fg_config_regs[i]);

	if (!ret)
		ret = max77759_fg_write_verify(map, MAX17042_VFSOC0Enable, VFSOC0_UNLOCK);
	if (!ret)
		ret = regmap_write(map, MAX17042_AtRate, p->at_rate);
	if (!ret)
		ret = max77759_fg_write_verify(map, MAX17055_CVMixCap,
					       p->full_cap_nom * 75 / 100);
	if (!ret)
		ret = max77759_fg_write_verify(map, MAX17055_CVHalfTime,
					       MAX77759_FG_CV_HALFTIME);
	if (!ret)
		ret = regmap_write(map, MAX17055_ConvgCfg, p->convg_cfg);

	lock_ret = max77759_fg_write_verify(map, MAX17042_VFSOC0Enable, VFSOC0_LOCK);
	if (ret || lock_ret)
		return ret ?: lock_ret;

	/* Not part of the model proper. */
	ret = regmap_write(map, MAX17055_Curve, p->curve);
	if (!ret)
		ret = regmap_write(map, MAX17042_FilterCFG, p->filter_cfg);
	if (!ret)
		ret = regmap_write(map, MAX17042_CGAIN, p->cgain);

	return ret;
}

static int max77759_fg_load_params(struct max17042_chip *chip)
{
	struct regmap *map = chip->regmap;
	unsigned int val;
	int ret;

	ret = max77759_fg_write_params(chip);
	if (ret) {
		dev_err(chip->dev, "cannot write the model parameters: %d\n", ret);
		return ret;
	}

	ret = regmap_set_bits(map, MAX17055_Config2, MAX77759_FG_CONFIG2_LDMDL);
	if (ret)
		return ret;

	/* The gauge clears LdMdl when done, usually within 400 ms. */
	ret = regmap_read_poll_timeout(map, MAX17055_Config2, val,
				       !(val & MAX77759_FG_CONFIG2_LDMDL),
				       MAX77759_FG_LDMDL_POLL_US,
				       MAX77759_FG_LDMDL_TIMEOUT_US);
	if (!ret)
		ret = regmap_read_poll_timeout(map, MAX17042_RepCap, val, val,
					       MAX77759_FG_LDMDL_POLL_US,
					       MAX77759_FG_LDMDL_TIMEOUT_US);
	if (ret)
		dev_err(chip->dev, "model load did not complete: %d\n", ret);

	return ret;
}

/*
 * The gauge recalculates FullCapRep from the model. If it drifts from
 * FullCapNom by more than the model itself does (plus 1% of the design
 * capacity), the load did not take.
 */
static bool max77759_fg_caps_ok(struct max17042_chip *chip)
{
	const struct max77759_fg_params *p = &chip->m5_model->params;
	unsigned int rep, nom;
	int threshold;

	if (regmap_read(chip->regmap, MAX17042_FullCAP0, &rep) ||
	    regmap_read(chip->regmap, MAX17042_FullCAPNom, &nom))
		return false;

	threshold = abs(p->full_cap_nom - p->full_cap_rep) + p->design_cap / 100;

	dev_dbg(chip->dev, "FullCapNom %#x FullCapRep %#x (threshold %d)\n",
		nom, rep, threshold);

	return abs((int)nom - (int)rep) <= threshold;
}

static int max77759_fg_init_chip(struct max17042_chip *chip)
{
	struct regmap *map = chip->regmap;
	unsigned int val;
	int ret, i;

	ret = regmap_read_poll_timeout(map, MAX17042_FSTAT, val,
				       !(val & MAX17055_FSTAT_DNR_BIT),
				       MAX17055_DNR_POLL_US,
				       MAX17055_DNR_TIMEOUT_US);
	if (ret)
		return ret;

	ret = regmap_read(map, MAX17055_Config2, &val);
	if (ret)
		return ret;
	if (val & MAX77759_FG_CONFIG2_LDMDL)
		return -EBUSY;

	ret = max77759_fg_load_table(chip);
	if (ret)
		return ret;

	msleep(500);

	for (i = 0; i < MAX77759_FG_LOAD_RETRIES; i++) {
		ret = max77759_fg_load_params(chip);
		if (ret)
			return ret;
		if (max77759_fg_caps_ok(chip))
			break;
	}
	if (i == MAX77759_FG_LOAD_RETRIES)
		dev_warn(chip->dev, "FullCapRep differs from FullCapNom after the model load\n");

	ret = regmap_read(map, MAX17042_TaskPeriod, &val);
	if (ret)
		return ret;
	chip->task_period = val;

	ret = regmap_clear_bits(map, MAX17042_STATUS, STATUS_POR_BIT);
	if (ret)
		return ret;

	dev_info(chip->dev, "battery model loaded (DesignCap %#x, TaskPeriod %#x)\n",
		 chip->m5_model->params.design_cap, val);

	return 0;
}

static void max17042_init_worker(struct work_struct *work)
{
	struct max17042_chip *chip = container_of(to_delayed_work(work),
				struct max17042_chip, work);
	int ret = 0;

	if (chip->m5_model) {
		/*
		 * On failure, keep POR set so that the next probe retries, and
		 * keep reporting from the gauge's default model.
		 */
		ret = max77759_fg_init_chip(chip);
		if (ret)
			dev_err(chip->dev, "battery model load failed: %d\n", ret);
		goto done;
	}

	/* Initialize registers according to values from config_data */
	if (chip->enable_por_init && chip->config_data)
		ret = max17042_init_chip(chip);

	if (ret) {
		if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17055) {
			dev_warn_ratelimited(chip->dev,
				"initialization failed: %d, retrying\n", ret);
			schedule_delayed_work(&chip->work,
				msecs_to_jiffies(MAX17055_INIT_RETRY_DELAY_MS));
		} else {
			dev_err(chip->dev, "initialization failed: %d\n", ret);
		}
		return;
	}

done:
	WRITE_ONCE(chip->init_complete, true);
	power_supply_changed(chip->battery);
}

#ifdef CONFIG_OF
static int max77759_fg_parse_model(struct max17042_chip *chip)
{
	struct device_node *np = chip->dev->of_node;
	struct max77759_fg_model *model;
	int count, ret;

	if (chip->chip_type != MAXIM_DEVICE_TYPE_MAX77759 ||
	    !of_property_present(np, "maxim,fg-model"))
		return 0;

	model = devm_kzalloc(chip->dev, sizeof(*model), GFP_KERNEL);
	if (!model)
		return -ENOMEM;

	count = of_property_count_u16_elems(np, "maxim,fg-model");
	if (count != ARRAY_SIZE(model->table))
		return dev_err_probe(chip->dev, -EINVAL,
				     "maxim,fg-model needs %zu values\n",
				     ARRAY_SIZE(model->table));

	ret = of_property_read_u16_array(np, "maxim,fg-model", model->table,
					 ARRAY_SIZE(model->table));
	if (ret)
		return dev_err_probe(chip->dev, ret, "invalid maxim,fg-model\n");

	/* CGain is optional and comes last. */
	model->params.cgain = MAX77759_FG_CGAIN_DEFAULT;
	count = of_property_count_u16_elems(np, "maxim,fg-params");
	if (count != ARRAY_SIZE(model->words) &&
	    count != ARRAY_SIZE(model->words) - 1)
		return dev_err_probe(chip->dev, -EINVAL,
				     "maxim,fg-params needs %zu or %zu values\n",
				     ARRAY_SIZE(model->words) - 1,
				     ARRAY_SIZE(model->words));

	ret = of_property_read_u16_array(np, "maxim,fg-params", model->words, count);
	if (ret)
		return dev_err_probe(chip->dev, ret, "invalid maxim,fg-params\n");

	if (!model->params.design_cap || !model->params.task_period)
		return dev_err_probe(chip->dev, -EINVAL,
				     "maxim,fg-params: zero DesignCap or TaskPeriod\n");

	chip->m5_model = model;

	return 0;
}

static int max17042_parse_dt(struct max17042_chip *chip)
{
	struct device_node *np = chip->dev->of_node;
	u32 prop;

	/*
	 * Require current sense resistor value to be specified for
	 * current-sense functionality to be enabled at all.
	 * maxim,rsns-microohm is the property name used by older DTs and kept
	 * for compatibility.
	 */
	if ((of_property_read_u32(np, "shunt-resistor-micro-ohms",
				  &prop) == 0) ||
	    (of_property_read_u32(np, "maxim,rsns-microohm", &prop) == 0)) {
		chip->r_sns = prop;
		chip->enable_current_sense = true;
	}

	if (of_property_read_s32(np, "maxim,cold-temp", &chip->temp_min))
		chip->temp_min = INT_MIN;
	if (of_property_read_s32(np, "maxim,over-heat-temp", &chip->temp_max))
		chip->temp_max = INT_MAX;
	if (of_property_read_u32(np, "maxim,dead-volt", &prop))
		chip->vmin = INT_MIN;
	else
		chip->vmin = prop;
	if (of_property_read_u32(np, "maxim,over-volt", &prop))
		chip->vmax = INT_MAX;
	else
		chip->vmin = prop;

	return max77759_fg_parse_model(chip);
}
#endif

static int max17042_init_defaults(struct max17042_chip *chip)
{
	int ret, misc_cfg;

	/*
	 * The MAX17047 gets used on x86 where we might not have DT, assume
	 * the firmware will already have initialized the fuel-gauge and provide
	 * default values for the non init bits to make things work.
	 */

	ret = regmap_read(chip->regmap, MAX17042_MiscCFG, &misc_cfg);
	if (ret < 0)
		return ret;

	/* If bits 0-1 are set to 3 then only Voltage readings are used */
	chip->enable_current_sense = (misc_cfg & 0x3) != 0x3;

	chip->vmin = MAX17042_DEFAULT_VMIN;
	chip->vmax = MAX17042_DEFAULT_VMAX;
	chip->temp_min = MAX17042_DEFAULT_TEMP_MIN;
	chip->temp_max = MAX17042_DEFAULT_TEMP_MAX;

	return 0;
}

static int max17042_apply_battery_properties(struct max17042_chip *chip,
					     struct power_supply_battery_info *info)
{
	struct max17042_config_data *config;
	struct device *dev = chip->dev;
	bool have_design_cap;
	bool have_ichgt_term;
	bool have_vchg;
	u16 design_cap = 0;
	u16 ichgt_term = 0;
	u16 dqacc = 0;
	u16 model_cfg = 0;
	u64 data64;

	if (!info || chip->chip_type != MAXIM_DEVICE_TYPE_MAX17055)
		return 0;

	have_design_cap = chip->enable_current_sense &&
		info->charge_full_design_uah > 0;
	have_ichgt_term = chip->enable_current_sense &&
		info->charge_term_current_ua > 0;
	have_vchg = info->voltage_max_design_uv >= 0;
	if (!have_design_cap && !have_ichgt_term && !have_vchg)
		return 0;

	if (have_design_cap) {
		data64 = (u64)info->charge_full_design_uah * chip->r_sns;
		do_div(data64, MAX17042_CAPACITY_LSB);
		if (!data64)
			return dev_err_probe(dev, -ERANGE,
					     "battery design capacity is too small for sense resistor\n");
		if (data64 > U16_MAX)
			return dev_err_probe(dev, -ERANGE,
					     "battery design capacity exceeds register range\n");

		design_cap = (u16)data64;
		dqacc = design_cap / MAX17055_DQACC_DIV;
		if (!dqacc)
			return dev_err_probe(dev, -ERANGE,
					     "battery design capacity is too small for EZ config\n");
	}

	if (have_ichgt_term) {
		data64 = (u64)info->charge_term_current_ua * chip->r_sns;
		do_div(data64, MAX17042_CURRENT_LSB);
		if (!data64)
			return dev_err_probe(dev, -ERANGE,
					     "charge termination current is too small for sense resistor\n");
		if (data64 > S16_MAX)
			return dev_err_probe(dev, -ERANGE,
					     "charge termination current exceeds positive register range\n");

		ichgt_term = (u16)data64;
	}

	if (have_vchg) {
		if (!info->voltage_max_design_uv)
			return dev_err_probe(dev, -EINVAL,
					     "battery design voltage must be positive\n");

		if (info->voltage_max_design_uv > MAX17055_VCHG_THRESHOLD_UV)
			model_cfg = MAX17055_MODELCFG_VCHG_BIT;
	}

	config = chip->config_data;
	if (!config) {
		config = devm_kzalloc(dev, sizeof(*config), GFP_KERNEL);
		if (!config)
			return -ENOMEM;
	}

	if (have_design_cap) {
		config->design_cap = design_cap;
		config->dqacc = dqacc;
	}
	if (have_ichgt_term)
		config->ichgt_term = ichgt_term;
	if (have_vchg) {
		config->model_cfg &= ~MAX17055_MODELCFG_VCHG_BIT;
		config->model_cfg |= model_cfg;
		chip->enable_vchg_override = true;
	}

	chip->config_data = config;
	chip->enable_por_init = true;

	return 0;
}

static int max17042_init_battery(struct power_supply *psy)
{
	struct max17042_chip *chip = power_supply_get_drvdata(psy);

	return max17042_apply_battery_properties(chip, psy->battery_info);
}

static const struct regmap_config max17042_regmap_config = {
	.name = "max17042",
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};

static const struct regmap_range max77759_fg_registers[] = {
	regmap_reg_range(MAX17042_STATUS, MAX77759_MixAtFull),
	regmap_reg_range(MAX17042_VFSOC0Enable, MAX17042_VFSOC0Enable),
	regmap_reg_range(MAX17042_MLOCKReg1, MAX17042_MLOCKReg2),
	regmap_reg_range(MAX17042_MODELChrTbl, MAX17055_TimerH),
	regmap_reg_range(MAX77759_IIn, MAX77759_IIn),
	regmap_reg_range(MAX17055_AtQResidual, MAX17055_AtAvCap),
	regmap_reg_range(MAX17042_OCVInternal, MAX17042_OCVInternal),
	regmap_reg_range(MAX17042_VFSOC, MAX17042_VFSOC),
};

static const struct regmap_range max77759_fg_ro_registers[] = {
	regmap_reg_range(MAX17042_FSTAT, MAX17042_FSTAT),
	regmap_reg_range(MAX17042_OCVInternal, MAX17042_OCVInternal),
	regmap_reg_range(MAX17042_VFSOC, MAX17042_VFSOC),
};

static const struct regmap_access_table max77759_fg_write_table = {
	.yes_ranges = max77759_fg_registers,
	.n_yes_ranges = ARRAY_SIZE(max77759_fg_registers),
	.no_ranges = max77759_fg_ro_registers,
	.n_no_ranges = ARRAY_SIZE(max77759_fg_ro_registers),
};

static const struct regmap_access_table max77759_fg_rd_table = {
	.yes_ranges = max77759_fg_registers,
	.n_yes_ranges = ARRAY_SIZE(max77759_fg_registers),
};

static const struct regmap_config max77759_fg_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xff,
	.wr_table = &max77759_fg_write_table,
	.rd_table = &max77759_fg_rd_table,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.cache_type = REGCACHE_NONE,
};

static const struct power_supply_desc max17042_psy_desc = {
	.name		= "max170xx_battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= max17042_get_property,
	.set_property	= max17042_set_property,
	.property_is_writeable	= max17042_property_is_writeable,
	.external_power_changed	= power_supply_changed,
	.init		= max17042_init_battery,
	.properties	= max17042_battery_props,
	.num_properties	= ARRAY_SIZE(max17042_battery_props),
};

static const struct power_supply_desc max17042_no_current_sense_psy_desc = {
	.name		= "max170xx_battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= max17042_get_property,
	.set_property	= max17042_set_property,
	.property_is_writeable	= max17042_property_is_writeable,
	.init		= max17042_init_battery,
	.properties	= max17042_battery_props,
	.num_properties	= ARRAY_SIZE(max17042_battery_props) - 2,
};

static int max17042_probe(struct i2c_client *client, struct device *dev, int irq,
			  enum max170xx_chip_type chip_type)
{
	struct i2c_adapter *adapter = client->adapter;
	const struct power_supply_desc *max17042_desc = &max17042_psy_desc;
	const struct regmap_config *regmap_config;
	struct power_supply_config psy_cfg = {};
	struct max17042_chip *chip;
	int ret;
	u32 val;
	bool use_default_config = false;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->chip_type = chip_type;

	if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX77759)
		regmap_config = &max77759_fg_regmap_cfg;
	else
		regmap_config = &max17042_regmap_config;
	chip->regmap = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "Failed to initialize regmap\n");

#ifdef CONFIG_OF
	if (dev->of_node) {
		ret = max17042_parse_dt(chip);
		if (ret)
			return ret;
	} else
#endif
	{
		ret = max17042_init_defaults(chip);
		if (ret)
			return ret;
		use_default_config = true;
	}

	dev_set_drvdata(dev, chip);
	psy_cfg.drv_data = chip;
	psy_cfg.fwnode = dev_fwnode(dev);

	/* When current is not measured,
	 * CURRENT_NOW and CURRENT_AVG properties should be invisible. */
	if (!chip->enable_current_sense)
		max17042_desc = &max17042_no_current_sense_psy_desc;

	if (chip->r_sns == 0)
		chip->r_sns = MAX17042_DEFAULT_SNS_RESISTOR;

	if (!chip->enable_current_sense) {
		regmap_write(chip->regmap, MAX17042_CGAIN, 0x0000);
		regmap_write(chip->regmap, MAX17042_MiscCFG, 0x0003);
		regmap_write(chip->regmap, MAX17042_LearnCFG, 0x0007);
	}

	chip->task_period = MAX17042_DEFAULT_TASK_PERIOD;
	if (chip->chip_type == MAXIM_DEVICE_TYPE_MAX77759) {
		ret = regmap_read(chip->regmap, MAX17042_TaskPeriod, &val);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to read task period\n");
		chip->task_period = val;
	}
	dev_dbg(dev, "task period: %#.4x (%d)\n", chip->task_period,
		chip->task_period);

	chip->battery = devm_power_supply_register(dev, max17042_desc,
						   &psy_cfg);
	if (IS_ERR(chip->battery))
		return dev_err_probe(dev, PTR_ERR(chip->battery),
				     "failed: power supply register\n");

	/*
	 * Some firmwares do not set FullSOCThr, Enable End-of-Charge Detection
	 * when the voltage FG reports 95%, as recommended in the datasheet.
	 */
	if (use_default_config &&
	    (chip->chip_type == MAXIM_DEVICE_TYPE_MAX17047 ||
	     chip->chip_type == MAXIM_DEVICE_TYPE_MAX17050))
		regmap_write(chip->regmap, MAX17047_FullSOCThr,
			     MAX17042_BATTERY_FULL << 8);

	if (irq) {
		unsigned int flags = IRQF_ONESHOT | IRQF_SHARED | IRQF_PROBE_SHARED;

		ret = devm_request_threaded_irq(dev, irq,
						NULL,
						max17042_thread_handler, flags,
						chip->battery->desc->name,
						chip);
		if (!ret) {
			regmap_update_bits(chip->regmap, MAX17042_CONFIG,
					CFG_ALRT_BIT_ENBL,
					CFG_ALRT_BIT_ENBL);
			max17042_enable_soc_alerts(chip);
		} else {
			irq = 0;
			if (ret != -EBUSY)
				dev_err(dev, "Failed to get IRQ\n");
		}
	}
	/* Not able to update the charge threshold when exceeded? -> disable */
	if (!irq)
		regmap_write(chip->regmap, MAX17042_SALRT_Th, 0xff00);

	chip->irq = irq;

	ret = regmap_read(chip->regmap, MAX17042_STATUS, &val);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read status\n");

	if (val & STATUS_POR_BIT) {
		ret = devm_delayed_work_autocancel(dev, &chip->work,
						   max17042_init_worker);
		if (ret)
			return ret;
		schedule_delayed_work(&chip->work, 0);
	} else {
		WRITE_ONCE(chip->init_complete, true);
	}

	return 0;
}

static int max17042_i2c_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
	const struct acpi_device_id *acpi_id = NULL;
	struct device *dev = &client->dev;
	enum max170xx_chip_type chip_type;

	if (id) {
		chip_type = id->driver_data;
	} else {
		acpi_id = acpi_match_device(dev->driver->acpi_match_table, dev);
		if (!acpi_id)
			return -ENODEV;

		chip_type = acpi_id->driver_data;
	}

	return max17042_probe(client, dev, client->irq, chip_type);
}

static int max17042_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct i2c_client *i2c;
	const struct platform_device_id *id;
	int irq;

	i2c = to_i2c_client(pdev->dev.parent);
	if (!i2c)
		return -EINVAL;

	device_set_of_node_from_dev(dev, dev->parent);

	id = platform_get_device_id(pdev);
	irq = platform_get_irq(pdev, 0);

	return max17042_probe(i2c, dev, irq, id->driver_data);
}

static int max17042_suspend(struct device *dev)
{
	struct max17042_chip *chip = dev_get_drvdata(dev);

	/*
	 * disable the irq and enable irq_wake
	 * capability to the interrupt line.
	 */
	if (chip->irq) {
		disable_irq(chip->irq);
		max17042_suspend_soc_alerts(chip);
		enable_irq_wake(chip->irq);
	}

	return 0;
}

static int max17042_resume(struct device *dev)
{
	struct max17042_chip *chip = dev_get_drvdata(dev);

	if (chip->irq) {
		disable_irq_wake(chip->irq);
		enable_irq(chip->irq);
		/* re-arm runtime SOC alerts */
		max17042_enable_soc_alerts(chip);
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(max17042_pm_ops, max17042_suspend,
				max17042_resume);

#ifdef CONFIG_ACPI
static const struct acpi_device_id max17042_acpi_match[] = {
	{ "MAX17047", MAXIM_DEVICE_TYPE_MAX17047 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, max17042_acpi_match);
#endif

#ifdef CONFIG_OF
/*
 * Device may be instantiated through parent MFD device and device matching is done
 * through platform_device_id.
 *
 * However if device's DT node contains proper clock compatible and driver is
 * built as a module, then the *module* matching will be done trough DT aliases.
 * This requires of_device_id table.  In the same time this will not change the
 * actual *device* matching so do not add .of_match_table.
 */
static const struct of_device_id max17042_dt_match[] __used = {
	{ .compatible = "maxim,max17042",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17042 },
	{ .compatible = "maxim,max17047",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17047 },
	{ .compatible = "maxim,max17050",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17050 },
	{ .compatible = "maxim,max17055",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17055 },
	{ .compatible = "maxim,max77705-battery",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17047 },
	{ .compatible = "maxim,max77759-fg",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX77759 },
	{ .compatible = "maxim,max77849-battery",
		.data = (void *) MAXIM_DEVICE_TYPE_MAX17047 },
	{ },
};
MODULE_DEVICE_TABLE(of, max17042_dt_match);
#endif

static const struct i2c_device_id max17042_id[] = {
	{ .name = "max17042", .driver_data = MAXIM_DEVICE_TYPE_MAX17042 },
	{ .name = "max17047", .driver_data = MAXIM_DEVICE_TYPE_MAX17047 },
	{ .name = "max17050", .driver_data = MAXIM_DEVICE_TYPE_MAX17050 },
	{ .name = "max17055", .driver_data = MAXIM_DEVICE_TYPE_MAX17055 },
	{ .name = "max77759-fg", .driver_data = MAXIM_DEVICE_TYPE_MAX77759 },
	{ .name = "max77849-battery", .driver_data = MAXIM_DEVICE_TYPE_MAX17047 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17042_id);

static const struct platform_device_id max17042_platform_id[] = {
	{ "max17042", MAXIM_DEVICE_TYPE_MAX17042 },
	{ "max17047", MAXIM_DEVICE_TYPE_MAX17047 },
	{ "max17050", MAXIM_DEVICE_TYPE_MAX17050 },
	{ "max17055", MAXIM_DEVICE_TYPE_MAX17055 },
	{ "max77705-battery", MAXIM_DEVICE_TYPE_MAX17047 },
	{ "max77849-battery", MAXIM_DEVICE_TYPE_MAX17047 },
	{ }
};
MODULE_DEVICE_TABLE(platform, max17042_platform_id);

static struct i2c_driver max17042_i2c_driver = {
	.driver	= {
		.name	= "max17042",
		.acpi_match_table = ACPI_PTR(max17042_acpi_match),
		.of_match_table = of_match_ptr(max17042_dt_match),
		.pm	= pm_ptr(&max17042_pm_ops),
	},
	.probe		= max17042_i2c_probe,
	.id_table	= max17042_id,
};

static struct platform_driver max17042_platform_driver = {
	.driver	= {
		.name	= "max17042",
		.acpi_match_table = ACPI_PTR(max17042_acpi_match),
		.pm	= pm_ptr(&max17042_pm_ops),
	},
	.probe		= max17042_platform_probe,
	.id_table	= max17042_platform_id,
};

static int __init max17042_init(void)
{
	int ret;

	ret = platform_driver_register(&max17042_platform_driver);
	if (ret)
		return ret;

	ret = i2c_add_driver(&max17042_i2c_driver);
	if (ret) {
		platform_driver_unregister(&max17042_platform_driver);
		return ret;
	}

	return 0;
}
module_init(max17042_init);

static void __exit max17042_exit(void)
{
	i2c_del_driver(&max17042_i2c_driver);
	platform_driver_unregister(&max17042_platform_driver);
}
module_exit(max17042_exit);

MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_DESCRIPTION("MAX17042 Fuel Gauge");
MODULE_LICENSE("GPL");
