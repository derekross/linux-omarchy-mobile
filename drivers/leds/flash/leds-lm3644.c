// SPDX-License-Identifier: GPL-2.0
/*
 * TI LM3644 / LM3644TT dual-channel flash LED driver, torch mode only
 *
 * The chip sits in shutdown while its HWEN pin is low. This driver raises
 * HWEN only while the torch is lit, so every switch-on starts from the
 * register reset values, and it never enters flash (strobe) mode: no flash
 * timeout is involved. An optional external strobe GPIO is held inactive.
 * Brightness steps are the chip's torch current codes, capped by the
 * board's led-max-microamp.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define LM3644_REG_ENABLE		0x01
#define LM3644_REG_IVFM			0x02
#define LM3644_REG_LED1_FLASH		0x03
#define LM3644_REG_LED2_FLASH		0x04
#define LM3644_REG_LED1_TORCH		0x05
#define LM3644_REG_LED2_TORCH		0x06
#define LM3644_REG_BOOST		0x07
#define LM3644_REG_TIMING		0x08
#define LM3644_REG_TEMP			0x09
#define LM3644_REG_FLAGS1		0x0a
#define LM3644_REG_FLAGS2		0x0b
#define LM3644_REG_DEVICE_ID		0x0c
#define LM3644_REG_LAST_FLASH		0x0d

#define LM3644_ENABLE_LED1		BIT(0)
#define LM3644_ENABLE_LED2		BIT(1)
#define LM3644_ENABLE_MODE_TORCH	(0x2 << 2)
#define LM3644_ENABLE_RESET		0x80
#define LM3644_TORCH_CODE_MAX		0x7f
#define LM3644_LED1_TORCH_RESET		0xbf
#define LM3644_LED2_TORCH_RESET		0x3f

#define LM3644_ID_LM3644		0x02
#define LM3644_ID_LM3644TT		0x04

/* HWEN high to I2C ready; the datasheet's t_on is well below this */
#define LM3644_STARTUP_US		2000

struct lm3644_variant {
	const char *name;
	u32 torch_base_ua;	/* torch current at code 0 */
	u32 torch_step_ua;	/* added per code */
};

static const struct lm3644_variant lm3644_variant = {
	.name = "LM3644", .torch_base_ua = 977, .torch_step_ua = 1400,
};

static const struct lm3644_variant lm3644tt_variant = {
	.name = "LM3644TT", .torch_base_ua = 2550, .torch_step_ua = 2910,
};

struct lm3644 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *hwen;
	struct led_classdev cdev;
	const struct lm3644_variant *variant;
	struct mutex lock;	/* serializes switching */
	u8 enable_bits;		/* LED1/LED2 enables, from led-sources */
	bool lit;
};

static const struct regmap_config lm3644_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = LM3644_REG_LAST_FLASH,
	.cache_type = REGCACHE_NONE,
};

static void lm3644_power(struct lm3644 *lm, bool on)
{
	gpiod_set_value_cansleep(lm->hwen, on);
	if (on)
		usleep_range(LM3644_STARTUP_US, LM3644_STARTUP_US + 500);
}

/* Read and log (and so clear) the fault flags */
static void lm3644_check_flags(struct lm3644 *lm)
{
	unsigned int f1, f2;

	if (regmap_read(lm->regmap, LM3644_REG_FLAGS1, &f1) ||
	    regmap_read(lm->regmap, LM3644_REG_FLAGS2, &f2))
		return;

	if (f1 || f2)
		dev_warn(lm->dev, "fault flags %#04x %#04x\n", f1, f2);
}

static int lm3644_torch_on(struct lm3644 *lm, unsigned int code)
{
	int ret;

	if (!lm->lit)
		lm3644_power(lm, true);

	/* LED2's override bit is cleared: each channel gets its own code */
	ret = regmap_write(lm->regmap, LM3644_REG_LED1_TORCH, code);
	if (!ret)
		ret = regmap_write(lm->regmap, LM3644_REG_LED2_TORCH, code);
	if (!ret)
		ret = regmap_write(lm->regmap, LM3644_REG_ENABLE,
				   LM3644_ENABLE_MODE_TORCH | lm->enable_bits);
	if (ret) {
		regmap_write(lm->regmap, LM3644_REG_ENABLE, 0);
		lm3644_power(lm, false);
		lm->lit = false;
		return ret;
	}

	lm->lit = true;
	lm3644_check_flags(lm);

	return 0;
}

static void lm3644_torch_off(struct lm3644 *lm)
{
	if (!lm->lit)
		return;

	lm3644_check_flags(lm);
	regmap_write(lm->regmap, LM3644_REG_ENABLE, 0);
	lm3644_power(lm, false);
	lm->lit = false;
}

static int lm3644_brightness_set(struct led_classdev *cdev,
				 enum led_brightness brightness)
{
	struct lm3644 *lm = container_of(cdev, struct lm3644, cdev);
	int ret = 0;

	mutex_lock(&lm->lock);
	if (brightness)
		ret = lm3644_torch_on(lm, brightness - 1);
	else
		lm3644_torch_off(lm);
	mutex_unlock(&lm->lock);

	return ret;
}

/*
 * Identify the part with HWEN freshly raised, and refuse a part whose reset
 * values don't match the register map this driver writes.
 */
static int lm3644_identify(struct lm3644 *lm)
{
	unsigned int id, en, t1, t2;
	int ret;

	lm3644_power(lm, true);

	ret = regmap_read(lm->regmap, LM3644_REG_DEVICE_ID, &id);
	if (ret) {
		dev_err(lm->dev, "no answer with HWEN high: %d\n", ret);
		goto out;
	}

	ret = regmap_read(lm->regmap, LM3644_REG_ENABLE, &en);
	if (!ret)
		ret = regmap_read(lm->regmap, LM3644_REG_LED1_TORCH, &t1);
	if (!ret)
		ret = regmap_read(lm->regmap, LM3644_REG_LED2_TORCH, &t2);
	if (ret)
		goto out;

	switch (id) {
	case LM3644_ID_LM3644:
		lm->variant = &lm3644_variant;
		break;
	case LM3644_ID_LM3644TT:
		lm->variant = &lm3644tt_variant;
		break;
	default:
		dev_err(lm->dev, "unknown device ID %#04x\n", id);
		ret = -ENODEV;
		goto out;
	}

	if (en != LM3644_ENABLE_RESET || t1 != LM3644_LED1_TORCH_RESET ||
	    t2 != LM3644_LED2_TORCH_RESET) {
		dev_err(lm->dev, "unexpected reset values %#04x %#04x %#04x\n",
			en, t1, t2);
		ret = -ENODEV;
		goto out;
	}

	dev_info(lm->dev, "%s (ID %#04x)\n", lm->variant->name, id);
out:
	lm3644_power(lm, false);

	return ret;
}

static void lm3644_shutdown(void *data)
{
	struct lm3644 *lm = data;

	mutex_lock(&lm->lock);
	lm3644_torch_off(lm);
	mutex_unlock(&lm->lock);
}

static int lm3644_probe(struct i2c_client *client)
{
	struct led_init_data init_data = {};
	struct gpio_desc *strobe;
	struct device *dev = &client->dev;
	struct fwnode_handle *child;
	u32 sources[2], max_ua, max_code;
	struct lm3644 *lm;
	int ret, n, i;

	lm = devm_kzalloc(dev, sizeof(*lm), GFP_KERNEL);
	if (!lm)
		return -ENOMEM;

	lm->dev = dev;
	i2c_set_clientdata(client, lm);
	ret = devm_mutex_init(dev, &lm->lock);
	if (ret)
		return ret;

	lm->regmap = devm_regmap_init_i2c(client, &lm3644_regmap);
	if (IS_ERR(lm->regmap))
		return PTR_ERR(lm->regmap);

	/* Keep external flash triggering inactive before raising HWEN. */
	strobe = devm_gpiod_get_optional(dev, "strobe", GPIOD_OUT_LOW);
	if (IS_ERR(strobe))
		return dev_err_probe(dev, PTR_ERR(strobe), "strobe GPIO\n");

	lm->hwen = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(lm->hwen))
		return dev_err_probe(dev, PTR_ERR(lm->hwen), "HWEN GPIO\n");

	child = device_get_next_child_node(dev, NULL);
	if (!child)
		return dev_err_probe(dev, -EINVAL, "no LED node\n");

	n = fwnode_property_count_u32(child, "led-sources");
	if (n < 1 || n > 2) {
		ret = dev_err_probe(dev, -EINVAL, "led-sources: 1 or 2 entries\n");
		goto put;
	}
	fwnode_property_read_u32_array(child, "led-sources", sources, n);
	for (i = 0; i < n; i++) {
		if (sources[i] < 1 || sources[i] > 2) {
			ret = dev_err_probe(dev, -EINVAL, "led-sources: 1 (LED1) or 2 (LED2)\n");
			goto put;
		}
		lm->enable_bits |= sources[i] == 2 ? LM3644_ENABLE_LED2 : LM3644_ENABLE_LED1;
	}

	ret = fwnode_property_read_u32(child, "led-max-microamp", &max_ua);
	if (ret) {
		dev_err_probe(dev, ret, "led-max-microamp is required\n");
		goto put;
	}

	ret = lm3644_identify(lm);
	if (ret)
		goto put;

	if (max_ua < lm->variant->torch_base_ua) {
		ret = dev_err_probe(dev, -EINVAL, "led-max-microamp below %u\n",
				    lm->variant->torch_base_ua);
		goto put;
	}
	max_code = min((max_ua - lm->variant->torch_base_ua) /
		       lm->variant->torch_step_ua, LM3644_TORCH_CODE_MAX);

	lm->cdev.max_brightness = max_code + 1;
	lm->cdev.brightness_set_blocking = lm3644_brightness_set;
	lm->cdev.flags = LED_CORE_SUSPENDRESUME;
	init_data.fwnode = child;
	init_data.devicename = NULL;
	init_data.default_label = "torch";

	ret = devm_add_action_or_reset(dev, lm3644_shutdown, lm);
	if (ret)
		goto put;

	ret = devm_led_classdev_register_ext(dev, &lm->cdev, &init_data);
	if (ret) {
		dev_err_probe(dev, ret, "LED registration\n");
		goto put;
	}

	dev_info(dev, "torch: %u steps, %u-%u uA per channel\n",
		 max_code + 1, lm->variant->torch_base_ua,
		 lm->variant->torch_base_ua + max_code * lm->variant->torch_step_ua);
put:
	fwnode_handle_put(child);

	return ret;
}

static void lm3644_i2c_shutdown(struct i2c_client *client)
{
	struct lm3644 *lm = i2c_get_clientdata(client);

	if (lm)
		lm3644_shutdown(lm);
}

static const struct of_device_id lm3644_of_match[] = {
	{ .compatible = "ti,lm3644" },
	{ }
};
MODULE_DEVICE_TABLE(of, lm3644_of_match);

static const struct i2c_device_id lm3644_id[] = {
	{ "lm3644" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3644_id);

static struct i2c_driver lm3644_driver = {
	.driver = {
		.name = "lm3644",
		.of_match_table = lm3644_of_match,
	},
	.probe = lm3644_probe,
	.shutdown = lm3644_i2c_shutdown,
	.id_table = lm3644_id,
};
module_i2c_driver(lm3644_driver);

MODULE_DESCRIPTION("TI LM3644 flash LED driver (torch mode)");
MODULE_LICENSE("GPL");
