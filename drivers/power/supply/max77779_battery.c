// SPDX-License-Identifier: GPL-2.0-only
/*
 * MAX77779 fuel-gauge measurements. The gauge keeps its battery model; after a
 * power-on reset the driver loads the model from the device tree, if it has one.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define MAX77779_STATUS		0x00
#define MAX77779_HIBCFG		0x02
#define MAX77779_REPCAP		0x06
#define MAX77779_REPSOC		0x07
#define MAX77779_CONFIG		0x0c
#define MAX77779_FULLCAPREP	0x10
#define MAX77779_TTE		0x11	/* 5.625 s, 0xffff unknown */
#define MAX77779_CYCLES		0x17	/* 25 % of a cycle */
#define MAX77779_DESIGNCAP	0x18
#define MAX77779_AVGVCELL		0x19
#define MAX77779_VCELL		0x1a
#define MAX77779_TEMP		0x1b	/* thermistor, signed 1/256 degC */
#define MAX77779_CURRENT	0x1c	/* signed 1.5625 uV / Rsense */
#define MAX77779_AVGCURRENT	0x1d
#define MAX77779_TTF		0x20
#define MAX77779_DEVNAME		0x21
#define MAX77779_FSTAT		0x3d
#define MAX77779_BOOT_CHECK	0x40
#define MAX77779_DPACC		0x46
#define MAX77779_MODEL		0x80	/* OCV0, start of the model table */
#define MAX77779_MODEL_END	0x9f
#define MAX77779_CONFIG2	0xab
#define MAX77779_FG_INT_STS	0xe0
#define MAX77779_USR		0xff
#define MAX77779_POR		BIT(1)
#define MAX77779_ABSENT		BIT(3)
#define MAX77779_NOT_READY	GENMASK(1, 0)
#define MAX77779_DNR		BIT(0)
#define MAX77779_BOOT_OK	0x82
#define MAX77779_LDMDL		BIT(15)
#define MAX77779_USR_LOCKS	GENMASK(3, 1)	/* NLOCK, VLOCK, RLOCK */

/* The gauge's non-volatile shadow registers, at a second I2C address */
#define MAX77779_NVM_ADDR	0x37
#define MAX77779_N_ICHGTERM	0x9c
#define MAX77779_N_FILTERCFG	0x9d
#define MAX77779_N_MODELCFG	0x9e
#define MAX77779_N_LEARNCFG	0x9f
#define MAX77779_N_QRTABLE00	0xa0
#define MAX77779_N_QRTABLE10	0xa1
#define MAX77779_N_QRTABLE20	0xa2
#define MAX77779_N_QRTABLE30	0xa3
#define MAX77779_N_CYCLES	0xa4
#define MAX77779_N_FULLCAPNOM	0xa5
#define MAX77779_N_RCOMP0	0xa6
#define MAX77779_N_TEMPCO	0xa7
#define MAX77779_N_FULLCAPREP	0xa9
#define MAX77779_N_MISCCFG	0xb2
#define MAX77779_N_DESIGNCAP	0xb3
#define MAX77779_N_RELAXCFG	0xb6
#define MAX77779_N_HIBCFG	0xbb
#define MAX77779_N_FULLSOCTHR	0xbc
#define MAX77779_N_NVCFG0	0xc0
#define MAX77779_N_VEMPTY	0xc6
#define MAX77779_N_THERMCFG	0xca
#define MAX77779_N_MODELCFG_REFRESH	BIT(15)

/* The PMIC function of the same chip, a second I2C address */
#define MAX77779_PMIC_ADDR	0x66
#define MAX77779_PMIC_FW_REV	0x62
#define MAX77779_PMIC_FW_SUB_REV	0x63

#define MAX77779_MODEL_WORDS	(MAX77779_MODEL_END - MAX77779_MODEL + 1)
#define MAX77779_HIBCFG_DEFAULT	0x0909
#define MAX77779_POLL_MS	10
#define MAX77779_POLL_TRIES	20
#define MAX77779_LOAD_ATTEMPTS	3

/*
 * Register values that come with a battery model, in the order of the
 * "maxim,fg-params" property and of Google's max77779_custom_parameters.
 * Most go to the non-volatile shadow registers (n...).
 */
struct max77779_fg_params {
	u16 nvcfg0;
	u16 relaxcfg;
	u16 learncfg;
	u16 config;
	u16 config2;
	u16 fullsocthr;
	u16 fullcaprep;
	u16 designcap;
	u16 dpacc;
	u16 fullcapnom;
	u16 v_empty;
	u16 qrtable00;
	u16 qrtable10;
	u16 qrtable20;
	u16 qrtable30;
	u16 rcomp0;
	u16 tempco;
	u16 ichgterm;
	u16 misccfg;
	u16 modelcfg;	/* not written by the load */
	u16 thermcfg;
	u16 filtercfg;
};

struct max77779_fg_model {
	u16 table[MAX77779_MODEL_WORDS];
	union {
		struct max77779_fg_params params;
		u16 words[sizeof(struct max77779_fg_params) / sizeof(u16)];
	};
};

struct max77779_battery {
	struct device *dev;
	struct regmap *map;
	struct power_supply *supply;
	struct delayed_work poll;
	struct max77779_fg_model *model;
	u32 rsense;
	u8 fw_rev;
	u8 fw_sub_rev;
	bool fw_valid;
	bool temp_valid;
};

static bool max77779_readable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX77779_STATUS:
	case MAX77779_HIBCFG:
	case MAX77779_REPCAP:
	case MAX77779_REPSOC:
	case MAX77779_CONFIG:
	case MAX77779_FULLCAPREP:
	case MAX77779_CYCLES:
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
	case MAX77779_BOOT_CHECK:
	case MAX77779_DPACC:
	case MAX77779_MODEL ... MAX77779_MODEL_END:
	case MAX77779_CONFIG2:
	case MAX77779_FG_INT_STS:
	case MAX77779_USR:
		return true;
	default:
		return false;
	}
}

/* Only the model load writes, and only while the gauge reports POR. */
static bool max77779_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX77779_HIBCFG:
	case MAX77779_REPCAP:
	case MAX77779_CONFIG:
	case MAX77779_DPACC:
	case MAX77779_MODEL ... MAX77779_MODEL_END:
	case MAX77779_CONFIG2:
	case MAX77779_FG_INT_STS:
	case MAX77779_USR:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config max77779_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.max_register = MAX77779_USR,
	.readable_reg = max77779_readable,
	.writeable_reg = max77779_writeable,
	.cache_type = REGCACHE_NONE,
};

static bool max77779_nvm_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX77779_N_ICHGTERM ... MAX77779_N_TEMPCO:
	case MAX77779_N_FULLCAPREP:
	case MAX77779_N_MISCCFG:
	case MAX77779_N_DESIGNCAP:
	case MAX77779_N_RELAXCFG:
	case MAX77779_N_HIBCFG:
	case MAX77779_N_FULLSOCTHR:
	case MAX77779_N_NVCFG0:
	case MAX77779_N_VEMPTY:
	case MAX77779_N_THERMCFG:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config max77779_nvm_regmap = {
	.name = "nvm",
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.max_register = MAX77779_N_THERMCFG,
	.readable_reg = max77779_nvm_reg,
	.writeable_reg = max77779_nvm_reg,
	.cache_type = REGCACHE_NONE,
};

/*
 * After the pack ran flat the gauge restarts with POR set. Without a model in
 * the device tree nothing reloads one; the gauge still measures and, once
 * FSTAT says the data are ready, reports state of charge from its power-on
 * configuration: less exact than with Google's model, far better than no
 * battery at all.
 */
static bool trust_por = true;
module_param(trust_por, bool, 0644);
MODULE_PARM_DESC(trust_por, "Report the gauge after a power-on reset with no model reloaded (default on)");

static bool load_model = true;
module_param(load_model, bool, 0444);
MODULE_PARM_DESC(load_model, "Load the device tree's battery model after a power-on reset (default on)");

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
	/* Never publish uninitialised data. */
	if (fstat & MAX77779_NOT_READY)
		return -EAGAIN;
	if ((*status & MAX77779_POR) && !trust_por)
		return -EAGAIN;
	return 0;
}

/*
 * The model load follows Google's max77779_load_gauge_model() ("model loading
 * procedure version 0.2.1", max77779_fg_model.c in kernel/google-modules/bms),
 * step for step. Each write is read back; where Google checks the value read
 * back, so does this driver, the same way.
 */

/* Read back a write that Google does not check, after Google's 10 ms wait. */
static int max77779_check(struct regmap *map, unsigned int reg, u16 value)
{
	unsigned int val;
	int i, ret;

	for (i = 0; i < 3; i++) {
		usleep_range(MAX77779_POLL_MS * 1000, MAX77779_POLL_MS * 1000 + 100);
		ret = regmap_read(map, reg, &val);
		if (!ret && val == value)
			return 0;
	}
	return ret ?: -EIO;
}

static int max77779_write_check(struct max77779_battery *battery, struct regmap *map,
				unsigned int reg, u16 value)
{
	int ret;

	ret = regmap_write(map, reg, value);
	if (!ret)
		ret = max77779_check(map, reg, value);
	if (ret)
		dev_err(battery->dev, "%s %#04x: %#06x not written: %d\n",
			map == battery->map ? "register" : "nvm", reg, value, ret);
	return ret;
}

/*
 * A RAM register Google writes without a check, and which the gauge itself
 * may change: read it back and report a difference, but go on as Google does.
 */
static int max77779_write_note(struct max77779_battery *battery, unsigned int reg, u16 value)
{
	int ret;

	ret = regmap_write(battery->map, reg, value);
	if (ret) {
		dev_err(battery->dev, "register %#04x: %#06x not written: %d\n", reg, value, ret);
		return ret;
	}
	if (max77779_check(battery->map, reg, value))
		dev_warn(battery->dev, "register %#04x: wrote %#06x, reads back otherwise\n",
			 reg, value);
	return 0;
}

/* Google's maxfg_regmap_writeverify(): write, wait 10 ms, read; three tries */
static int max77779_write_verify(struct regmap *map, unsigned int reg, u16 value)
{
	unsigned int val;
	int i, ret;

	for (i = 0; i < 3; i++) {
		ret = regmap_write(map, reg, value);
		if (ret)
			continue;
		usleep_range(MAX77779_POLL_MS * 1000, MAX77779_POLL_MS * 1000 + 100);
		ret = regmap_read(map, reg, &val);
		if (!ret && val == value)
			return 0;
	}
	return -EIO;
}

static int max77779_usr_write(struct regmap *map, u8 val)
{
	const struct reg_sequence seq[] = {
		{ .reg = MAX77779_USR, .def = val },
		{ .reg = MAX77779_USR, .def = val },
	};

	return regmap_multi_reg_write(map, seq, ARRAY_SIZE(seq));
}

/*
 * Lock or unlock all three register sections (RAM, function, NVM) in USR.
 * Like Google, keep the low byte and write it twice under one regmap lock.
 */
static int max77779_usr_lock(struct max77779_battery *battery, bool lock)
{
	unsigned int usr;
	u8 val;
	int ret;

	ret = regmap_read(battery->map, MAX77779_USR, &usr);
	if (ret)
		return ret;
	val = lock ? usr | MAX77779_USR_LOCKS : usr & ~MAX77779_USR_LOCKS;
	ret = max77779_usr_write(battery->map, val);
	if (!ret)
		ret = regmap_read(battery->map, MAX77779_USR, &usr);
	if (!ret && (usr & MAX77779_USR_LOCKS) != (lock ? MAX77779_USR_LOCKS : 0))
		ret = -EIO;
	if (ret)
		dev_err(battery->dev, "cannot %s the gauge registers: %d\n",
			lock ? "lock" : "unlock", ret);
	return ret;
}

/* Step 3.4.1: RepCap to 0, then the table, read back as a whole. */
static int max77779_write_table(struct max77779_battery *battery)
{
	const struct max77779_fg_model *model = battery->model;
	u16 back[MAX77779_MODEL_WORDS];
	int ret;

	ret = regmap_write(battery->map, MAX77779_REPCAP, 0);
	if (!ret)
		ret = regmap_bulk_write(battery->map, MAX77779_MODEL, model->table,
					MAX77779_MODEL_WORDS);
	if (!ret)
		ret = regmap_bulk_read(battery->map, MAX77779_MODEL, back, MAX77779_MODEL_WORDS);
	if (!ret && memcmp(back, model->table, sizeof(back)))
		ret = -ERANGE;
	if (ret)
		dev_err(battery->dev, "cannot write the model table: %d\n", ret);
	return ret;
}

struct max77779_nvm_write {
	u8 reg;
	size_t offset;
};

#define MAX77779_NVM_W(_reg, _field) \
	{ _reg, offsetof(struct max77779_fg_params, _field) }

/* Step 3.6, after dPAcc and nFullCapNom: in Google's order */
static const struct max77779_nvm_write max77779_nvm_writes[] = {
	MAX77779_NVM_W(MAX77779_N_ICHGTERM,	ichgterm),
	MAX77779_NVM_W(MAX77779_N_VEMPTY,		v_empty),
	MAX77779_NVM_W(MAX77779_N_RCOMP0,		rcomp0),
	MAX77779_NVM_W(MAX77779_N_TEMPCO,		tempco),
};

static const struct max77779_nvm_write max77779_nvm_writes_qr[] = {
	MAX77779_NVM_W(MAX77779_N_QRTABLE00,	qrtable00),
	MAX77779_NVM_W(MAX77779_N_QRTABLE10,	qrtable10),
	MAX77779_NVM_W(MAX77779_N_QRTABLE20,	qrtable20),
	MAX77779_NVM_W(MAX77779_N_QRTABLE30,	qrtable30),
};

static const struct max77779_nvm_write max77779_nvm_writes_tail[] = {
	MAX77779_NVM_W(MAX77779_N_FULLSOCTHR,	fullsocthr),
	MAX77779_NVM_W(MAX77779_N_MISCCFG,		misccfg),
	MAX77779_NVM_W(MAX77779_N_THERMCFG,	thermcfg),
	MAX77779_NVM_W(MAX77779_N_NVCFG0,		nvcfg0),
	MAX77779_NVM_W(MAX77779_N_FILTERCFG,	filtercfg),
};

static int max77779_nvm_list(struct max77779_battery *battery, struct regmap *nvm,
			     const struct max77779_nvm_write *list, size_t n)
{
	const struct max77779_fg_params *p = &battery->model->params;
	size_t i;
	int ret = 0;

	for (i = 0; !ret && i < n; i++)
		ret = max77779_write_check(battery, nvm, list[i].reg,
					   *(const u16 *)((const u8 *)p + list[i].offset));
	return ret;
}

/* Step 3.6: the custom parameters */
static int max77779_write_params(struct max77779_battery *battery, struct regmap *nvm,
				 u16 hibcfg)
{
	const struct max77779_fg_params *p = &battery->model->params;
	struct regmap *map = battery->map;
	unsigned int val;
	int i, ret;

	ret = max77779_write_check(battery, nvm, MAX77779_N_DESIGNCAP, p->designcap);
	if (!ret)
		ret = max77779_write_check(battery, nvm, MAX77779_N_FULLCAPREP, p->fullcaprep);
	if (ret)
		return ret;

	/* Google writes dPAcc up to three times, reading it back 2 ms later. */
	for (i = 0; i < 3; i++) {
		ret = regmap_write(map, MAX77779_DPACC, p->dpacc);
		if (ret)
			continue;
		usleep_range(2000, 2500);
		ret = regmap_read(map, MAX77779_DPACC, &val);
		if (!ret && val == p->dpacc)
			break;
		ret = -EIO;
	}
	if (ret) {
		dev_err(battery->dev, "dPAcc not written: %d\n", ret);
		return ret;
	}

	ret = max77779_write_check(battery, nvm, MAX77779_N_FULLCAPNOM, p->fullcapnom);
	if (!ret)
		ret = max77779_nvm_list(battery, nvm, max77779_nvm_writes,
					ARRAY_SIZE(max77779_nvm_writes));
	/* Google's default state: no learned cycles to restore */
	if (!ret)
		ret = max77779_write_check(battery, nvm, MAX77779_N_CYCLES, 0);
	if (!ret)
		ret = max77779_nvm_list(battery, nvm, max77779_nvm_writes_qr,
					ARRAY_SIZE(max77779_nvm_writes_qr));
	if (!ret)
		ret = max77779_write_check(battery, nvm, MAX77779_N_HIBCFG, hibcfg);
	/* Google: clear nModelCfg.Refresh on firmware below 2.6 */
	if (!ret && (battery->fw_rev < 2 || (battery->fw_rev == 2 && battery->fw_sub_rev < 6))) {
		ret = regmap_read(nvm, MAX77779_N_MODELCFG, &val);
		if (!ret)
			ret = max77779_write_check(battery, nvm, MAX77779_N_MODELCFG,
						   val & ~MAX77779_N_MODELCFG_REFRESH);
	}
	if (!ret) {
		ret = max77779_write_verify(nvm, MAX77779_N_LEARNCFG, p->learncfg);
		if (ret)
			dev_err(battery->dev, "nLearnCfg not written: %d\n", ret);
	}
	if (!ret)
		ret = max77779_write_check(battery, nvm, MAX77779_N_RELAXCFG, p->relaxcfg);
	if (!ret)
		ret = max77779_write_note(battery, MAX77779_CONFIG, p->config);
	if (!ret)
		ret = max77779_nvm_list(battery, nvm, max77779_nvm_writes_tail,
					ARRAY_SIZE(max77779_nvm_writes_tail));
	return ret;
}

/* Steps 3.6.1 to 4.1: start the load, wait for it, restore Config2, clear POR */
static int max77779_run_load(struct max77779_battery *battery)
{
	struct regmap *map = battery->map;
	unsigned int val = 0;
	int i, ret;

	ret = regmap_read(map, MAX77779_CONFIG2, &val);
	if (!ret)
		ret = regmap_write(map, MAX77779_CONFIG2, val | MAX77779_LDMDL);
	if (ret)
		return ret;
	for (i = 0; i < MAX77779_POLL_TRIES; i++) {
		ret = regmap_read(map, MAX77779_CONFIG2, &val);
		if (!ret && !(val & MAX77779_LDMDL))
			break;
		usleep_range(MAX77779_POLL_MS * 1000, MAX77779_POLL_MS * 1000 + 100);
	}
	if (i == MAX77779_POLL_TRIES) {
		dev_err(battery->dev, "model load did not complete (Config2 %#06x)\n", val);
		return -ETIMEDOUT;
	}

	ret = max77779_write_note(battery, MAX77779_CONFIG2, battery->model->params.config2);
	if (ret)
		return ret;
	/* Google: registers resynchronise for 200 ms on firmware below 3.8 */
	if (battery->fw_rev < 3 || (battery->fw_rev == 3 && battery->fw_sub_rev < 8))
		msleep(200);

	for (i = 0; i < 10; i++) {
		ret = regmap_write(map, MAX77779_FG_INT_STS, MAX77779_POR);
		msleep(100);
		if (!ret)
			ret = regmap_read(map, MAX77779_FG_INT_STS, &val);
		if (!ret && !(val & MAX77779_POR))
			return 0;
	}
	dev_err(battery->dev, "cannot clear POR (FG_INT_STS %#06x): %d\n", val, ret);
	return ret ?: -ETIMEDOUT;
}

static int max77779_load_once(struct max77779_battery *battery, struct regmap *nvm)
{
	unsigned int val, hibcfg;
	int i, ret, lock_ret;

	/* Never write unless the gauge reports POR, a battery and ready data. */
	ret = regmap_read(battery->map, MAX77779_STATUS, &val);
	if (ret)
		return ret;
	if (!(val & MAX77779_POR))
		return 0;
	if (val & MAX77779_ABSENT)
		return -ENODEV;

	/* Step 1: the gauge's RISC-V core has started */
	for (i = 0; i < MAX77779_POLL_TRIES; i++) {
		ret = regmap_read(battery->map, MAX77779_BOOT_CHECK, &val);
		if (!ret && (val & MAX77779_BOOT_OK) == MAX77779_BOOT_OK)
			break;
		msleep(MAX77779_POLL_MS);
	}
	if (i == MAX77779_POLL_TRIES) {
		dev_err(battery->dev, "gauge core not ready\n");
		return -ETIMEDOUT;
	}

	/* Step 2: FStat.DNR clear */
	for (i = 0; i < MAX77779_POLL_TRIES; i++) {
		ret = regmap_read(battery->map, MAX77779_FSTAT, &val);
		if (!ret && !(val & MAX77779_DNR))
			break;
		msleep(MAX77779_POLL_MS);
	}
	if (i == MAX77779_POLL_TRIES) {
		dev_err(battery->dev, "gauge data not ready\n");
		return -ETIMEDOUT;
	}

	/* Step 3.1: unlock, and hold hibernation off for the load */
	ret = max77779_usr_lock(battery, false);
	if (ret)
		goto lock;
	ret = regmap_read(battery->map, MAX77779_HIBCFG, &hibcfg);
	if (!ret)
		ret = max77779_write_note(battery, MAX77779_HIBCFG, 0);
	if (!ret)
		ret = max77779_write_table(battery);
	if (!ret)
		ret = max77779_write_params(battery, nvm,
					    hibcfg ? hibcfg : MAX77779_HIBCFG_DEFAULT);
	if (!ret)
		ret = max77779_run_load(battery);

	/* Step 4.2 (Google returns without locking if POR stays set; lock anyway) */
lock:
	for (i = 0; i < 3; i++) {
		lock_ret = max77779_usr_lock(battery, true);
		if (!lock_ret)
			break;
	}
	return ret ?: lock_ret;
}

/*
 * Load the model while the gauge reports POR, up to three times a second
 * apart; a failed load leaves POR set and the gauge reported as without a
 * model. The NVM address is claimed only for the load.
 */
static void max77779_load_model(struct max77779_battery *battery, struct i2c_client *client)
{
	struct i2c_client *nvm_client;
	struct regmap *nvm;
	unsigned int status;
	int i, ret;

	if (!battery->model)
		return;
	ret = regmap_read(battery->map, MAX77779_STATUS, &status);
	if (ret || !(status & MAX77779_POR))
		return;
	if (!load_model) {
		dev_info(battery->dev, "load_model=0: battery model not loaded\n");
		return;
	}
	if (!battery->fw_valid || (!battery->fw_rev && !battery->fw_sub_rev)) {
		dev_err(battery->dev, "firmware revision unknown: battery model not loaded\n");
		return;
	}

	nvm_client = i2c_new_dummy_device(client->adapter, MAX77779_NVM_ADDR);
	if (IS_ERR(nvm_client)) {
		dev_err(battery->dev, "NVM address unavailable: %pe\n", nvm_client);
		return;
	}
	nvm = regmap_init_i2c(nvm_client, &max77779_nvm_regmap);
	if (IS_ERR(nvm)) {
		ret = PTR_ERR(nvm);
		dev_err(battery->dev, "NVM regmap: %pe\n", nvm);
		goto out;
	}

	for (i = 1; i <= MAX77779_LOAD_ATTEMPTS; i++) {
		dev_info(battery->dev, "power-on reset: loading the battery model (attempt %d)\n",
			 i);
		ret = max77779_load_once(battery, nvm);
		if (!ret)
			break;
		dev_err(battery->dev, "battery model load failed: %d\n", ret);
		if (i < MAX77779_LOAD_ATTEMPTS)
			msleep(1000);
	}
	regmap_exit(nvm);
out:
	i2c_unregister_device(nvm_client);

	if (ret)
		return;
	/*
	 * Status follows FG_INT_STS on the gauge's next update, about 0.2 s
	 * on the 9a: wait for its POR bit to clear, so that the probe's check
	 * below doesn't take the loaded gauge for one with no model.
	 */
	for (i = 0; i < 10; i++) {
		ret = regmap_read(battery->map, MAX77779_STATUS, &status);
		if (ret || !(status & MAX77779_POR))
			break;
		msleep(100);
	}
	if (!ret)
		dev_info(battery->dev, "battery model loaded (DesignCap %#06x, Status %#06x)\n",
			 battery->model->params.designcap, status);
}

static int max77779_parse_model(struct max77779_battery *battery)
{
	struct device *dev = battery->dev;
	struct max77779_fg_model *model;
	int count, ret;

	if (!device_property_present(dev, "maxim,fg-model"))
		return 0;

	model = devm_kzalloc(dev, sizeof(*model), GFP_KERNEL);
	if (!model)
		return -ENOMEM;

	count = device_property_count_u16(dev, "maxim,fg-model");
	if (count != ARRAY_SIZE(model->table)) {
		dev_err(dev, "maxim,fg-model needs %zu values\n", ARRAY_SIZE(model->table));
		return -EINVAL;
	}
	ret = device_property_read_u16_array(dev, "maxim,fg-model", model->table, count);
	if (ret)
		return ret;

	count = device_property_count_u16(dev, "maxim,fg-params");
	if (count != ARRAY_SIZE(model->words)) {
		dev_err(dev, "maxim,fg-params needs %zu values\n", ARRAY_SIZE(model->words));
		return -EINVAL;
	}
	ret = device_property_read_u16_array(dev, "maxim,fg-params", model->words, count);
	if (ret)
		return ret;
	if (!model->params.designcap || !model->params.fullcapnom) {
		dev_err(dev, "maxim,fg-params: zero DesignCap or FullCapNom\n");
		return -EINVAL;
	}

	battery->model = model;
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
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		reg = MAX77779_CYCLES;
		break;
	default:
		return -EINVAL;
	}
	if (status & MAX77779_ABSENT)
		return -ENODATA;
	/*
	 * After a power-on reset with no model reloaded, the charges and the
	 * time estimates come from the gauge's default pack, not this one,
	 * and the cycle count restarted from zero: withhold them and keep
	 * the percentage, voltage and current.
	 */
	if (status & MAX77779_POR) {
		switch (property) {
		case POWER_SUPPLY_PROP_CHARGE_NOW:
		case POWER_SUPPLY_PROP_CHARGE_FULL:
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		case POWER_SUPPLY_PROP_CYCLE_COUNT:
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
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		/* whole cycles from quarter cycles, rounded as Google does */
		value->intval = DIV_ROUND_CLOSEST(raw * 25, 100);
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
	POWER_SUPPLY_PROP_CYCLE_COUNT,
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
 * The firmware revision is in the chip's PMIC function, at a second address
 * on the same bus; read it without claiming that address, so a PMIC driver
 * can still bind there. Google's driver reports a fixed 22 C instead of TEMP
 * on revision 1 below subrevision 15, and its model load depends on the
 * revision too. Unknown firmware: no temperature and no model load.
 */
static bool max77779_temp_valid(struct max77779_battery *battery, struct i2c_client *client)
{
	union i2c_smbus_data rev, sub;

	if (i2c_smbus_xfer(client->adapter, MAX77779_PMIC_ADDR, 0, I2C_SMBUS_READ,
			   MAX77779_PMIC_FW_REV, I2C_SMBUS_BYTE_DATA, &rev) ||
	    i2c_smbus_xfer(client->adapter, MAX77779_PMIC_ADDR, 0, I2C_SMBUS_READ,
			   MAX77779_PMIC_FW_SUB_REV, I2C_SMBUS_BYTE_DATA, &sub))
		return false;
	battery->fw_rev = rev.byte;
	battery->fw_sub_rev = sub.byte;
	battery->fw_valid = true;
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
	battery->dev = dev;
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
	battery->temp_valid = max77779_temp_valid(battery, client);
	/* A bad model is reported and ignored: the battery is still reported. */
	ret = max77779_parse_model(battery);
	if (ret == -ENOMEM)
		return ret;
	max77779_load_model(battery, client);
	ret = max77779_ready(battery, &raw);
	if (ret)
		return dev_err_probe(dev, ret, "Firmware model is not ready\n");
	if (raw & MAX77779_POR)
		dev_warn(dev, "Power-on reset, no model reloaded: state of charge from the gauge's defaults\n");
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
	dev_info(dev, "Fuel gauge; firmware %u.%u; device tree battery model %s; temperature %s\n",
		 battery->fw_rev, battery->fw_sub_rev, battery->model ? "present" : "absent",
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

MODULE_DESCRIPTION("MAX77779 fuel gauge, with the battery model loaded after a power-on reset");
MODULE_LICENSE("GPL");
