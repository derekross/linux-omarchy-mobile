// SPDX-License-Identifier: GPL-2.0-only
//
// GPIO driver for the Maxim MAX77779 interface PMIC's SGPIO block
//
// The MAX77779 (Google Pixel 9a) is a set of I2C blocks on one chip: the
// charger, the fuel gauge, the Type-C port controller and, at 0x66, the
// PMIC block that owns the chip's interrupt line and eight SGPIO pins. The
// pins carry the board's charger side signals (the RT9470 boost's interrupt
// and enable, the wireless charger's lines). This driver binds to the PMIC
// block and exposes the SGPIOs as a GPIO chip with interrupts. On stock the
// gauge, the charger, the I2C master and VIMON interrupt through this block
// as well (only the Type-C port controller has its own line); on this tree
// their nodes have no interrupts, so those sources stay masked here.

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>

#define MAX77779_PMIC_ID		0x00
#define MAX77779_PMIC_ID_VALUE		0x79
#define MAX77779_PMIC_REVISION		0x01
#define MAX77779_PMIC_OTP_REVISION	0x02
#define MAX77779_PMIC_INTSRC_STS	0x22	/* write 1 to clear */
#define MAX77779_PMIC_INTSRC_GPIO	BIT(5)
#define MAX77779_PMIC_INTB_MASK		0x24	/* 1 = masked */

#define MAX77779_SGPIO_INT		0xe8	/* write 1 to clear */
#define MAX77779_SGPIO_PU		0xe9
#define MAX77779_SGPIO_PD		0xea
#define MAX77779_SGPIO_CNFG(n)		(0xed + (n))
#define MAX77779_SGPIO_CNFG_DATA	BIT(0)
#define MAX77779_SGPIO_CNFG_VGPI_EN	BIT(1)
#define MAX77779_SGPIO_CNFG_MODE	GENMASK(3, 2)
#define MAX77779_SGPIO_MODE_INPUT	1
#define MAX77779_SGPIO_MODE_OUTPUT	2
#define MAX77779_SGPIO_CNFG_DBNC	GENMASK(5, 4)
#define MAX77779_SGPIO_CNFG_IRQ		GENMASK(7, 6)
#define MAX77779_SGPIO_IRQ_NONE		0
#define MAX77779_SGPIO_IRQ_FALLING	1
#define MAX77779_SGPIO_IRQ_RISING	2
#define MAX77779_SGPIO_IRQ_BOTH		3
#define MAX77779_VGPI_CNFG		0xfb

#define MAX77779_N_SGPIO		8

struct max77779_gpio {
	struct regmap *map;
	struct gpio_chip gc;
	int parent_irq;
	struct mutex irq_lock;	/* the irq bus: trigger, mask and wake changes */
	u8 irq_trig[MAX77779_N_SGPIO];
	unsigned long irq_masked;
	unsigned long irq_changed;
	unsigned long irq_wake;
	bool parent_wake;
};

static const struct regmap_range max77779_pmic_volatile_ranges[] = {
	regmap_reg_range(MAX77779_PMIC_ID, MAX77779_PMIC_OTP_REVISION),
	regmap_reg_range(MAX77779_PMIC_INTSRC_STS, MAX77779_PMIC_INTSRC_STS),
	regmap_reg_range(MAX77779_SGPIO_INT, MAX77779_SGPIO_INT),
	/* CNFGx carries the pin's live level in DATA */
	regmap_reg_range(MAX77779_SGPIO_CNFG(0), MAX77779_SGPIO_CNFG(7)),
};

static const struct regmap_access_table max77779_pmic_volatile_table = {
	.yes_ranges = max77779_pmic_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77779_pmic_volatile_ranges),
};

static const struct regmap_config max77779_pmic_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77779_VGPI_CNFG,
	.volatile_table = &max77779_pmic_volatile_table,
	.cache_type = REGCACHE_NONE,
};

static int max77779_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct max77779_gpio *chip = gpiochip_get_data(gc);
	unsigned int cnfg;
	int ret;

	ret = regmap_read(chip->map, MAX77779_SGPIO_CNFG(offset), &cnfg);
	if (ret)
		return ret;

	switch (FIELD_GET(MAX77779_SGPIO_CNFG_MODE, cnfg)) {
	case MAX77779_SGPIO_MODE_INPUT:
		return GPIO_LINE_DIRECTION_IN;
	case MAX77779_SGPIO_MODE_OUTPUT:
		return GPIO_LINE_DIRECTION_OUT;
	default:
		/* 0: the pin is off; 3: reserved */
		return -ENODEV;
	}
}

static int max77779_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct max77779_gpio *chip = gpiochip_get_data(gc);

	return regmap_update_bits(chip->map, MAX77779_SGPIO_CNFG(offset),
				  MAX77779_SGPIO_CNFG_MODE,
				  FIELD_PREP(MAX77779_SGPIO_CNFG_MODE,
					     MAX77779_SGPIO_MODE_INPUT));
}

static int max77779_gpio_direction_output(struct gpio_chip *gc,
					  unsigned int offset, int value)
{
	struct max77779_gpio *chip = gpiochip_get_data(gc);

	return regmap_update_bits(chip->map, MAX77779_SGPIO_CNFG(offset),
				  MAX77779_SGPIO_CNFG_MODE | MAX77779_SGPIO_CNFG_DATA,
				  FIELD_PREP(MAX77779_SGPIO_CNFG_MODE,
					     MAX77779_SGPIO_MODE_OUTPUT) |
				  FIELD_PREP(MAX77779_SGPIO_CNFG_DATA, !!value));
}

static int max77779_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct max77779_gpio *chip = gpiochip_get_data(gc);
	unsigned int cnfg;
	int ret;

	ret = regmap_read(chip->map, MAX77779_SGPIO_CNFG(offset), &cnfg);
	if (ret)
		return ret;

	return !!(cnfg & MAX77779_SGPIO_CNFG_DATA);
}

static int max77779_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct max77779_gpio *chip = gpiochip_get_data(gc);

	return regmap_update_bits(chip->map, MAX77779_SGPIO_CNFG(offset),
				  MAX77779_SGPIO_CNFG_DATA,
				  FIELD_PREP(MAX77779_SGPIO_CNFG_DATA, !!value));
}

static int max77779_gpio_set_config(struct gpio_chip *gc, unsigned int offset,
				    unsigned long config)
{
	struct max77779_gpio *chip = gpiochip_get_data(gc);
	unsigned int pu = 0, pd = 0;
	int ret;

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_BIAS_PULL_UP:
		pu = BIT(offset);
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		pd = BIT(offset);
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		break;
	default:
		return -ENOTSUPP;
	}

	ret = regmap_update_bits(chip->map, MAX77779_SGPIO_PU, BIT(offset), pu);
	if (ret)
		return ret;

	return regmap_update_bits(chip->map, MAX77779_SGPIO_PD, BIT(offset), pd);
}

static void max77779_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_gpio *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	set_bit(hwirq, &chip->irq_masked);
	set_bit(hwirq, &chip->irq_changed);
	gpiochip_disable_irq(gc, hwirq);
}

static void max77779_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_gpio *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	gpiochip_enable_irq(gc, hwirq);
	clear_bit(hwirq, &chip->irq_masked);
	set_bit(hwirq, &chip->irq_changed);
}

static int max77779_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_gpio *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	/*
	 * The SGPIOs detect edges. A level request takes the edge into the
	 * level, and the handler re-checks the pin afterwards: while the level
	 * holds, the interrupt isn't cleared and fires again.
	 */
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_LEVEL_HIGH:
		chip->irq_trig[hwirq] = MAX77779_SGPIO_IRQ_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_LEVEL_LOW:
		chip->irq_trig[hwirq] = MAX77779_SGPIO_IRQ_FALLING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		chip->irq_trig[hwirq] = MAX77779_SGPIO_IRQ_BOTH;
		break;
	default:
		return -EINVAL;
	}
	set_bit(hwirq, &chip->irq_changed);

	return 0;
}

static int max77779_gpio_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_gpio *chip = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	/* forwarded to the chip's line at bus_sync_unlock */
	if (on)
		set_bit(hwirq, &chip->irq_wake);
	else
		clear_bit(hwirq, &chip->irq_wake);

	return 0;
}

static void max77779_gpio_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_gpio *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->irq_lock);
}

static void max77779_gpio_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct max77779_gpio *chip = gpiochip_get_data(gc);
	unsigned long changed = chip->irq_changed;
	unsigned int offset;
	int ret;

	for_each_set_bit(offset, &changed, MAX77779_N_SGPIO) {
		u8 sel = test_bit(offset, &chip->irq_masked) ?
			 MAX77779_SGPIO_IRQ_NONE : chip->irq_trig[offset];

		ret = regmap_update_bits(chip->map, MAX77779_SGPIO_CNFG(offset),
					 MAX77779_SGPIO_CNFG_IRQ,
					 FIELD_PREP(MAX77779_SGPIO_CNFG_IRQ, sel));
		if (ret)
			dev_err(gc->parent, "SGPIO%u: can't set the trigger: %d\n",
				offset, ret);
		else
			clear_bit(offset, &chip->irq_changed);
	}

	/*
	 * A level line that was masked had its status bit cleared by the
	 * handler; if the level still holds now that it is unmasked, the pin
	 * won't produce a new edge, so run the handler once more: it checks
	 * every unmasked level line's pin.
	 */
	if (chip->parent_irq > 0 && (changed & ~chip->irq_masked))
		irq_wake_thread(chip->parent_irq, chip);

	/* the chip has one line: it wakes while any pin asks for it */
	if (chip->parent_irq > 0 && !!chip->irq_wake != chip->parent_wake) {
		chip->parent_wake = !!chip->irq_wake;
		irq_set_irq_wake(chip->parent_irq, chip->parent_wake);
	}

	mutex_unlock(&chip->irq_lock);
}

static void max77779_gpio_irq_print_chip(struct irq_data *d, struct seq_file *p)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);

	seq_puts(p, dev_name(gc->parent));
}

static const struct irq_chip max77779_gpio_irq_chip = {
	.irq_mask		= max77779_gpio_irq_mask,
	.irq_unmask		= max77779_gpio_irq_unmask,
	.irq_set_type		= max77779_gpio_irq_set_type,
	.irq_set_wake		= max77779_gpio_irq_set_wake,
	.irq_bus_lock		= max77779_gpio_irq_bus_lock,
	.irq_bus_sync_unlock	= max77779_gpio_irq_bus_sync_unlock,
	.irq_print_chip		= max77779_gpio_irq_print_chip,
	.flags			= IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

/* The pin of a level-typed line, as the handler sees it: active or not */
static bool max77779_gpio_level_active(struct max77779_gpio *chip,
				       unsigned int offset, unsigned int type)
{
	unsigned int cnfg;
	int ret;

	ret = regmap_read(chip->map, MAX77779_SGPIO_CNFG(offset), &cnfg);
	if (ret)
		return false;

	if (type & IRQ_TYPE_LEVEL_LOW)
		return !(cnfg & MAX77779_SGPIO_CNFG_DATA);
	return cnfg & MAX77779_SGPIO_CNFG_DATA;
}

static irqreturn_t max77779_gpio_irq_handler(int irq, void *data)
{
	struct max77779_gpio *chip = data;
	struct gpio_chip *gc = &chip->gc;
	unsigned long pending, masked, levels = 0;
	unsigned int types[MAX77779_N_SGPIO] = {};
	bool handled = false;
	unsigned int src, sts;
	unsigned int offset;
	u8 ack;
	int ret;

	ret = regmap_read(chip->map, MAX77779_PMIC_INTSRC_STS, &src);
	if (ret) {
		dev_err_ratelimited(gc->parent, "can't read INTSRC_STS: %d\n", ret);
		return IRQ_NONE;
	}

	/*
	 * The SGPIO status is read on every run, not only when INTSRC_STS
	 * names the GPIO source: an unmask re-runs the thread to re-check a
	 * level line whose bit was acknowledged while it was masked.
	 */
	ret = regmap_read(chip->map, MAX77779_SGPIO_INT, &sts);
	if (ret) {
		dev_err_ratelimited(gc->parent, "can't read SGPIO_INT: %d\n", ret);
		return IRQ_NONE;
	}

	/*
	 * Masked pins are acknowledged and not dispatched; their level, if it
	 * holds, is checked again when they are unmasked. An unmasked level
	 * line whose pin is active is pending even with its status bit clear
	 * (an unmask after a masked period).
	 */
	masked = READ_ONCE(chip->irq_masked);
	ack = sts & masked;
	pending = sts & ~masked;
	for_each_clear_bit(offset, &masked, MAX77779_N_SGPIO) {
		unsigned int virq = irq_find_mapping(gc->irq.domain, offset);
		struct irq_data *d = virq ? irq_get_irq_data(virq) : NULL;
		unsigned int type = d ? irqd_get_trigger_type(d) : IRQ_TYPE_NONE;

		if (!(type & IRQ_TYPE_LEVEL_MASK))
			continue;
		types[offset] = type;
		levels |= BIT(offset);
		if (max77779_gpio_level_active(chip, offset, type))
			pending |= BIT(offset);
	}

	for_each_set_bit(offset, &pending, MAX77779_N_SGPIO) {
		unsigned int virq = irq_find_mapping(gc->irq.domain, offset);

		/* a pin nobody requested has no mapping */
		if (virq)
			handle_nested_irq(virq);
		handled = true;
		/* a held level fires again: its bit stays until the pin drops */
		if (!(levels & BIT(offset)) ||
		    !max77779_gpio_level_active(chip, offset, types[offset]))
			ack |= BIT(offset);
	}
	if (ack)
		regmap_write(chip->map, MAX77779_SGPIO_INT, ack);

	/* the other sources are masked; acknowledge whatever is set */
	if (src)
		regmap_write(chip->map, MAX77779_PMIC_INTSRC_STS, src);

	return IRQ_RETVAL(handled || src);
}

static int max77779_gpio_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max77779_gpio *chip;
	struct gpio_irq_chip *girq;
	unsigned int id, rev, i;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->map = devm_regmap_init_i2c(client, &max77779_pmic_regmap_config);
	if (IS_ERR(chip->map))
		return dev_err_probe(dev, PTR_ERR(chip->map), "can't init the regmap\n");

	ret = regmap_read(chip->map, MAX77779_PMIC_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "can't read the ID\n");
	if (id != MAX77779_PMIC_ID_VALUE)
		return dev_err_probe(dev, -ENODEV, "ID 0x%02x is not a MAX77779\n", id);
	ret = regmap_read(chip->map, MAX77779_PMIC_REVISION, &rev);
	if (ret)
		return dev_err_probe(dev, ret, "can't read the revision\n");
	dev_info(dev, "MAX77779 PMIC revision 0x%02x\n", rev);

	ret = devm_mutex_init(dev, &chip->irq_lock);
	if (ret)
		return ret;

	/*
	 * Every pin's trigger off (OTP or the bootloader may have left one
	 * set, and a latched bit on a pin nobody requested would keep the
	 * line low), everything masked and acknowledged.
	 */
	chip->irq_masked = GENMASK(MAX77779_N_SGPIO - 1, 0);
	chip->irq_changed = chip->irq_masked;
	ret = regmap_write(chip->map, MAX77779_PMIC_INTB_MASK, 0xff);
	if (ret)
		return dev_err_probe(dev, ret, "can't mask the interrupts\n");
	for (i = 0; i < MAX77779_N_SGPIO; i++) {
		ret = regmap_update_bits(chip->map, MAX77779_SGPIO_CNFG(i),
					 MAX77779_SGPIO_CNFG_IRQ,
					 FIELD_PREP(MAX77779_SGPIO_CNFG_IRQ,
						    MAX77779_SGPIO_IRQ_NONE));
		if (ret)
			return dev_err_probe(dev, ret, "can't clear SGPIO%u's trigger\n", i);
	}
	chip->irq_changed = 0;
	regmap_write(chip->map, MAX77779_SGPIO_INT, 0xff);
	regmap_write(chip->map, MAX77779_PMIC_INTSRC_STS, 0xff);

	chip->gc.base = -1;
	chip->gc.label = dev_name(dev);
	chip->gc.parent = dev;
	chip->gc.owner = THIS_MODULE;
	chip->gc.can_sleep = true;
	chip->gc.ngpio = MAX77779_N_SGPIO;
	chip->gc.get_direction = max77779_gpio_get_direction;
	chip->gc.direction_input = max77779_gpio_direction_input;
	chip->gc.direction_output = max77779_gpio_direction_output;
	chip->gc.get = max77779_gpio_get;
	chip->gc.set = max77779_gpio_set;
	chip->gc.set_config = max77779_gpio_set_config;

	girq = &chip->gc.irq;
	gpio_irq_chip_set_chip(girq, &max77779_gpio_irq_chip);
	/* the parent interrupt is handled here */
	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_simple_irq;
	girq->threaded = true;

	ret = devm_gpiochip_add_data(dev, &chip->gc, chip);
	if (ret)
		return dev_err_probe(dev, ret, "can't add the GPIO chip\n");

	if (client->irq <= 0) {
		dev_info(dev, "no interrupt line: the SGPIOs don't interrupt\n");
		return 0;
	}
	chip->parent_irq = client->irq;

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					max77779_gpio_irq_handler,
					IRQF_ONESHOT, dev_name(dev), chip);
	if (ret)
		return dev_err_probe(dev, ret, "can't request the interrupt\n");

	/*
	 * Only the SGPIO source reaches the line. The gauge, the charger, the
	 * I2C master and VIMON interrupt through this block on stock too; on
	 * this tree their nodes have no interrupts, so they stay masked.
	 */
	ret = regmap_write(chip->map, MAX77779_PMIC_INTB_MASK,
			   (u8)~MAX77779_PMIC_INTSRC_GPIO);
	if (ret)
		return dev_err_probe(dev, ret, "can't unmask the SGPIO source\n");

	return 0;
}

static const struct of_device_id max77779_gpio_of_match[] = {
	{ .compatible = "maxim,max77779-pmic" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77779_gpio_of_match);

static const struct i2c_device_id max77779_gpio_i2c_id[] = {
	{ "max77779-pmic" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77779_gpio_i2c_id);

static struct i2c_driver max77779_gpio_driver = {
	.driver = {
		.name = "max77779-pmic",
		.of_match_table = max77779_gpio_of_match,
	},
	.probe = max77779_gpio_probe,
	.id_table = max77779_gpio_i2c_id,
};
module_i2c_driver(max77779_gpio_driver);

MODULE_AUTHOR("W4HO <87742609+W4HO-ham@users.noreply.github.com>");
MODULE_DESCRIPTION("GPIO driver for the Maxim MAX77779 PMIC's SGPIOs");
MODULE_LICENSE("GPL");
