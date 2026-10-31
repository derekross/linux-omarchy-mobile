// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bring-up only, not for upstream: the Pixel 9a's RT9470 (run by Google's
 * rt9471 driver) as an OTG boost only, a regulator on its OTG_EN bit
 * (FUNCTION 0x02 bit 1), which the MAX77779 charger enables to give a USB-C
 * device VBUS. Probe only reads (the device id, and the JEITA and watchdog
 * settings it finds), except for a boost a previous boot left on, which it
 * reports and turns off. Enabling turns off JEITA (0x0C bit 7; stock runs
 * the RT9470 with it off, tegu's zumapro-buck-charge-rt9471.dtsi leaves
 * en-jeita out), keeps the I2C watchdog at stock's 40 s (TOP 0x01 bits 1:0,
 * wdt = <40>) and sets OTG_EN; disabling clears OTG_EN and puts JEITA and
 * the watchdog back as probe found them.
 *
 * The watchdog is serviced as stock does it: the chip raises its WDT
 * interrupt (IRQ3 bit 5) before the timer runs out and the handler kicks it
 * (TOP bit 2). The interrupt line is the MAX77779 PMIC's SGPIO6, taken as a
 * GPIO (int-gpios, so the pull-up stock sets on it comes with the request)
 * and turned into an interrupt. Without the line, a kernel timer kicks the
 * watchdog every 20 s instead, so a crash still ends the boost within 40 s
 * either way. The chip enable (CEB, the PMIC's SGPIO7) is enable-gpios,
 * asserted at probe. Shutdown turns the boost off.
 *
 * The charger part of the chip (charging, BC1.2, its other interrupts) is
 * left alone. Registers from mainline rt9471.c and Google's GPL
 * rt9471_charger.h and rt9471_charger.c (google-modules/bms at
 * 183509aff980).
 */

#include <linux/bitfield.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define RT9471_REG_OTGCFG	0x00
#define RT9471_REG_TOP		0x01
#define  RT9471_WDT		GENMASK(1, 0)
#define  RT9471_WDT_40S		1
#define  RT9471_WDT_RST		BIT(2)
#define RT9471_REG_FUNC		0x02
#define  RT9471_OTGEN		BIT(1)
#define RT9471_REG_INFO		0x0b
#define RT9471_REG_JEITA	0x0c
#define  RT9471_JEITA_EN	BIT(7)
#define  RT9471_DEVID		GENMASK(6, 3)
#define RT9471_REG_IRQ3		0x23	/* write 1 to clear */
#define RT9471_REG_MASK3	0x33	/* 1 = masked */
#define  RT9471_IRQ3_WDT	BIT(5)	/* interrupt 29 */

#define RT9471_KICK_MS		20000	/* half the 40 s watchdog */

static bool rt9471_boost_writeable(struct device *dev, unsigned int reg)
{
	return reg == RT9471_REG_FUNC || reg == RT9471_REG_TOP ||
	       reg == RT9471_REG_JEITA || reg == RT9471_REG_IRQ3 ||
	       reg == RT9471_REG_MASK3;
}

static const struct regmap_config rt9471_boost_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x33,
	.writeable_reg = rt9471_boost_writeable,
	.cache_type = REGCACHE_NONE,
};

struct rt9471_boost {
	struct device *dev;
	struct regmap *map;
	struct gpio_desc *enable;
	int irq;
	struct delayed_work kick;
	unsigned int jeita;	/* JEITA_EN as probe found it */
	unsigned int wdt;	/* the watchdog bits as probe found them */
	unsigned int mask3;	/* MASK3 as probe found it */
};

static int rt9471_boost_kick(struct rt9471_boost *b)
{
	return regmap_update_bits(b->map, RT9471_REG_TOP, RT9471_WDT_RST, RT9471_WDT_RST);
}

static void rt9471_boost_kick_work(struct work_struct *work)
{
	struct rt9471_boost *b = container_of(to_delayed_work(work),
					      struct rt9471_boost, kick);

	if (rt9471_boost_kick(b))
		dev_err_ratelimited(b->dev, "cannot kick the watchdog\n");
	schedule_delayed_work(&b->kick, msecs_to_jiffies(RT9471_KICK_MS));
}

static irqreturn_t rt9471_boost_irq(int irq, void *data)
{
	struct rt9471_boost *b = data;
	unsigned int irq3;

	if (regmap_read(b->map, RT9471_REG_IRQ3, &irq3))
		return IRQ_NONE;
	if (!(irq3 & RT9471_IRQ3_WDT))
		return IRQ_NONE;

	rt9471_boost_kick(b);
	regmap_write(b->map, RT9471_REG_IRQ3, RT9471_IRQ3_WDT);
	return IRQ_HANDLED;
}

static int rt9471_boost_enable(struct regulator_dev *rdev)
{
	struct rt9471_boost *b = rdev_get_drvdata(rdev);
	int ret;

	ret = regmap_update_bits(b->map, RT9471_REG_JEITA, RT9471_JEITA_EN, 0) ?:
	      regmap_update_bits(b->map, RT9471_REG_TOP, RT9471_WDT,
				 FIELD_PREP(RT9471_WDT, RT9471_WDT_40S)) ?:
	      rt9471_boost_kick(b);
	if (ret)
		return ret;

	if (b->irq > 0) {
		ret = regmap_write(b->map, RT9471_REG_IRQ3, RT9471_IRQ3_WDT) ?:
		      regmap_update_bits(b->map, RT9471_REG_MASK3, RT9471_IRQ3_WDT, 0);
		if (ret)
			return ret;
	} else {
		schedule_delayed_work(&b->kick, msecs_to_jiffies(RT9471_KICK_MS));
	}

	return regmap_update_bits(b->map, RT9471_REG_FUNC, RT9471_OTGEN, RT9471_OTGEN);
}

static int rt9471_boost_off(struct rt9471_boost *b)
{
	cancel_delayed_work_sync(&b->kick);
	return regmap_update_bits(b->map, RT9471_REG_FUNC, RT9471_OTGEN, 0) ?:
	       regmap_update_bits(b->map, RT9471_REG_MASK3, RT9471_IRQ3_WDT,
				  b->mask3 & RT9471_IRQ3_WDT) ?:
	       regmap_update_bits(b->map, RT9471_REG_JEITA, RT9471_JEITA_EN, b->jeita) ?:
	       regmap_update_bits(b->map, RT9471_REG_TOP, RT9471_WDT, b->wdt);
}

static int rt9471_boost_disable(struct regulator_dev *rdev)
{
	return rt9471_boost_off(rdev_get_drvdata(rdev));
}

static const struct regulator_ops rt9471_boost_ops = {
	.enable = rt9471_boost_enable,
	.disable = rt9471_boost_disable,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc rt9471_boost_desc = {
	.name = "rt9471-otg-boost",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &rt9471_boost_ops,
	.fixed_uV = 5000000,
	.n_voltages = 1,
	.enable_reg = RT9471_REG_FUNC,
	.enable_mask = RT9471_OTGEN,
	.enable_time = 5000,
};

static int rt9471_boost_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config cfg = {};
	struct regulator_dev *rdev;
	struct gpio_desc *int_gpio;
	struct rt9471_boost *b;
	struct regmap *map;
	unsigned int info, func;
	int ret;

	b = devm_kzalloc(dev, sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;
	b->dev = dev;
	INIT_DELAYED_WORK(&b->kick, rt9471_boost_kick_work);

	/* CEB: asserted (low) keeps the chip on; stock drives it from the PMIC */
	b->enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(b->enable))
		return dev_err_probe(dev, PTR_ERR(b->enable), "cannot get the enable line\n");

	map = devm_regmap_init_i2c(client, &rt9471_boost_regmap);
	if (IS_ERR(map))
		return PTR_ERR(map);
	b->map = map;

	ret = regmap_read(map, RT9471_REG_INFO, &info);
	if (ret)
		return dev_err_probe(dev, ret, "no answer\n");
	/* RT9470 0x9, RT9470D 0xa, RT9471 0xd, RT9471D 0xe */
	switch (FIELD_GET(RT9471_DEVID, info)) {
	case 0x9: case 0xa: case 0xd: case 0xe:
		break;
	default:
		return dev_err_probe(dev, -ENODEV, "unknown id %#x\n", info);
	}
	dev_info(dev, "id %#lx\n", FIELD_GET(RT9471_DEVID, info));

	ret = regmap_read(map, RT9471_REG_JEITA, &b->jeita) ?:
	      regmap_read(map, RT9471_REG_TOP, &b->wdt) ?:
	      regmap_read(map, RT9471_REG_MASK3, &b->mask3) ?:
	      regmap_read(map, RT9471_REG_FUNC, &func);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read JEITA and the watchdog\n");
	b->jeita &= RT9471_JEITA_EN;
	b->wdt &= RT9471_WDT;
	dev_info(dev, "JEITA %s, watchdog %s (%#x)\n",
		 b->jeita ? "on" : "off", b->wdt ? "on" : "off", b->wdt);

	if (func & RT9471_OTGEN) {
		dev_warn(dev, "the boost was left on (a reset in host mode?); turning it off\n");
		ret = regmap_update_bits(map, RT9471_REG_FUNC, RT9471_OTGEN, 0);
		if (ret)
			return dev_err_probe(dev, ret, "cannot turn the boost off\n");
	}

	/* the WDT interrupt: stock's PLC_INT_L on the PMIC's SGPIO6, pulled up */
	int_gpio = devm_gpiod_get_optional(dev, "int", GPIOD_IN);
	if (IS_ERR(int_gpio))
		return dev_err_probe(dev, PTR_ERR(int_gpio), "cannot get the interrupt line\n");
	if (int_gpio) {
		b->irq = gpiod_to_irq(int_gpio);
		if (b->irq < 0)
			return dev_err_probe(dev, b->irq, "the interrupt line has no interrupt\n");
		ret = devm_request_threaded_irq(dev, b->irq, NULL, rt9471_boost_irq,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						dev_name(dev), b);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request the interrupt\n");
		dev_info(dev, "watchdog serviced from interrupt %d\n", b->irq);
	} else {
		dev_info(dev, "no interrupt line: a timer kicks the watchdog\n");
	}

	i2c_set_clientdata(client, b);
	cfg.dev = dev;
	cfg.regmap = map;
	cfg.driver_data = b;
	cfg.of_node = dev->of_node;
	cfg.init_data = of_get_regulator_init_data(dev, dev->of_node,
						   &rt9471_boost_desc);
	rdev = devm_regulator_register(dev, &rt9471_boost_desc, &cfg);
	return PTR_ERR_OR_ZERO(rdev);
}

static void rt9471_boost_shutdown(struct i2c_client *client)
{
	struct rt9471_boost *b = i2c_get_clientdata(client);

	if (rt9471_boost_off(b))
		dev_err(&client->dev, "cannot turn the boost off at shutdown\n");
}

static const struct of_device_id rt9471_boost_of_match[] = {
	{ .compatible = "omamobile,tegu-rt9471-boost" },
	{ }
};
MODULE_DEVICE_TABLE(of, rt9471_boost_of_match);

static struct i2c_driver rt9471_boost_driver = {
	.driver = {
		.name = "tegu-rt9471-boost",
		.of_match_table = rt9471_boost_of_match,
	},
	.probe = rt9471_boost_probe,
	.shutdown = rt9471_boost_shutdown,
};
module_i2c_driver(rt9471_boost_driver);

MODULE_DESCRIPTION("RT9471 OTG boost for the Pixel 9a's USB-C port");
MODULE_LICENSE("GPL");
