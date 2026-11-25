// SPDX-License-Identifier: GPL-2.0-only
//
// GPIO driver for the Maxim MAX77779 interface PMIC's SGPIOs
//
// The MAX77779 (Google Pixel 9a) is a set of I2C blocks on one chip. Its
// PMIC block at 0x66 (drivers/mfd/max77779.c) owns the chip's interrupt
// line and holds eight SGPIO pins, which carry the board's charger side
// signals (the RT9470 boost's interrupt and enable, the wireless charger's
// lines). This driver is a cell of the PMIC block: it exposes the SGPIOs as
// a GPIO chip and is the interrupt controller for the eight pins, on the
// SGPIO source of the block's interrupt controller.

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/max77779.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>

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

struct max77779_gpio {
	struct regmap *map;
	struct gpio_chip gc;
	int parent_irq;
	struct mutex irq_lock;	/* the irq bus: trigger, mask and wake changes */
	struct mutex handler_lock;	/* one handler run at a time */
	struct work_struct recheck;
	u8 irq_trig[MAX77779_N_SGPIO];
	unsigned long irq_masked;
	unsigned long irq_changed;
	unsigned long irq_wake;
	bool parent_wake;
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
	unsigned int on, off;
	int ret;

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_BIAS_PULL_UP:
		on = MAX77779_SGPIO_PU;
		off = MAX77779_SGPIO_PD;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		on = MAX77779_SGPIO_PD;
		off = MAX77779_SGPIO_PU;
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		ret = regmap_clear_bits(chip->map, MAX77779_SGPIO_PU, BIT(offset));
		if (ret)
			return ret;
		return regmap_clear_bits(chip->map, MAX77779_SGPIO_PD, BIT(offset));
	default:
		return -ENOTSUPP;
	}

	/* the opposite pull goes off first: the pin never has both */
	ret = regmap_clear_bits(chip->map, off, BIT(offset));
	if (ret)
		return ret;

	return regmap_set_bits(chip->map, on, BIT(offset));
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

	/* forwarded to the parent interrupt at bus_sync_unlock */
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
	 * every unmasked level line's pin. The parent interrupt is nested in
	 * the PMIC block's thread, which can't be woken from here, so the
	 * run goes to a work item.
	 */
	if (changed & ~chip->irq_masked)
		schedule_work(&chip->recheck);

	/* the SGPIOs have one parent interrupt: it wakes while any pin asks */
	if (!!chip->irq_wake != chip->parent_wake) {
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
	if (ret) {
		dev_err_ratelimited(chip->gc.parent, "SGPIO%u: can't read the level: %d\n",
				    offset, ret);
		return false;
	}

	if (type & IRQ_TYPE_LEVEL_LOW)
		return !(cnfg & MAX77779_SGPIO_CNFG_DATA);
	return cnfg & MAX77779_SGPIO_CNFG_DATA;
}

static bool max77779_gpio_handle(struct max77779_gpio *chip)
{
	struct gpio_chip *gc = &chip->gc;
	unsigned long pending, masked, levels = 0;
	unsigned int types[MAX77779_N_SGPIO] = {};
	unsigned int sts, offset;
	u8 ack;
	int ret;

	guard(mutex)(&chip->handler_lock);

	/*
	 * The SGPIO status is read on every run, not only on the source's
	 * interrupt: an unmask re-runs the handler to re-check a level line
	 * whose bit was acknowledged while it was masked.
	 */
	ret = regmap_read(chip->map, MAX77779_SGPIO_INT, &sts);
	if (ret) {
		dev_err_ratelimited(gc->parent, "can't read SGPIO_INT: %d\n", ret);
		return false;
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
		/* a held level fires again: its bit stays until the pin drops */
		if (!(levels & BIT(offset)) ||
		    !max77779_gpio_level_active(chip, offset, types[offset]))
			ack |= BIT(offset);
	}
	if (ack) {
		ret = regmap_write(chip->map, MAX77779_SGPIO_INT, ack);
		if (ret)
			dev_err_ratelimited(gc->parent,
					    "can't acknowledge SGPIO_INT 0x%02x: %d\n",
					    ack, ret);
	}

	return pending || sts;
}

static irqreturn_t max77779_gpio_irq_handler(int irq, void *data)
{
	return IRQ_RETVAL(max77779_gpio_handle(data));
}

static void max77779_gpio_recheck(struct work_struct *work)
{
	struct max77779_gpio *chip = container_of(work, struct max77779_gpio,
						  recheck);

	max77779_gpio_handle(chip);
}

static void max77779_gpio_cancel_recheck(void *data)
{
	struct max77779_gpio *chip = data;

	cancel_work_sync(&chip->recheck);
}

static int max77779_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max77779_gpio *chip;
	struct gpio_irq_chip *girq;
	unsigned int i;
	int irq, ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->map = dev_get_regmap(dev->parent, NULL);
	if (!chip->map)
		return dev_err_probe(dev, -ENODEV, "missing the PMIC block's regmap\n");

	irq = platform_get_irq_byname_optional(pdev, "SGPIO");
	if (irq < 0 && irq != -ENXIO)
		return dev_err_probe(dev, irq, "can't get the SGPIO interrupt\n");

	ret = devm_mutex_init(dev, &chip->irq_lock);
	if (ret)
		return ret;
	ret = devm_mutex_init(dev, &chip->handler_lock);
	if (ret)
		return ret;
	INIT_WORK(&chip->recheck, max77779_gpio_recheck);

	/*
	 * Every pin's trigger off (OTP or the bootloader may have left one
	 * set, and a latched bit on a pin nobody requested would hold the
	 * source), and every status bit acknowledged, before the source is
	 * unmasked.
	 */
	chip->irq_masked = GENMASK(MAX77779_N_SGPIO - 1, 0);
	for (i = 0; i < MAX77779_N_SGPIO; i++) {
		ret = regmap_update_bits(chip->map, MAX77779_SGPIO_CNFG(i),
					 MAX77779_SGPIO_CNFG_IRQ,
					 FIELD_PREP(MAX77779_SGPIO_CNFG_IRQ,
						    MAX77779_SGPIO_IRQ_NONE));
		if (ret)
			return dev_err_probe(dev, ret, "can't clear SGPIO%u's trigger\n", i);
	}
	ret = regmap_write(chip->map, MAX77779_SGPIO_INT,
			   GENMASK(MAX77779_N_SGPIO - 1, 0));
	if (ret)
		return dev_err_probe(dev, ret, "can't acknowledge SGPIO_INT\n");

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

	if (irq > 0) {
		chip->parent_irq = irq;

		girq = &chip->gc.irq;
		gpio_irq_chip_set_chip(girq, &max77779_gpio_irq_chip);
		/* the parent interrupt is handled here */
		girq->parent_handler = NULL;
		girq->num_parents = 0;
		girq->parents = NULL;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_simple_irq;
		girq->threaded = true;
	}

	ret = devm_gpiochip_add_data(dev, &chip->gc, chip);
	if (ret)
		return dev_err_probe(dev, ret, "can't add the GPIO chip\n");

	/* registered after the chip: at removal the work stops before it goes */
	ret = devm_add_action_or_reset(dev, max77779_gpio_cancel_recheck, chip);
	if (ret)
		return ret;

	if (irq <= 0) {
		dev_info(dev, "no parent interrupt: the SGPIOs don't interrupt\n");
		return 0;
	}

	ret = devm_request_threaded_irq(dev, irq, NULL, max77779_gpio_irq_handler,
					IRQF_ONESHOT, dev_name(dev), chip);
	if (ret)
		return dev_err_probe(dev, ret, "can't request the SGPIO interrupt\n");

	return 0;
}

static const struct of_device_id max77779_gpio_of_match[] = {
	{ .compatible = "maxim,max77779-sgpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77779_gpio_of_match);

static const struct platform_device_id max77779_gpio_platform_id[] = {
	{ .name = "max77779-sgpio" },
	{ }
};
MODULE_DEVICE_TABLE(platform, max77779_gpio_platform_id);

static struct platform_driver max77779_gpio_driver = {
	.driver = {
		.name = "max77779-sgpio",
		.of_match_table = max77779_gpio_of_match,
	},
	.probe = max77779_gpio_probe,
	.id_table = max77779_gpio_platform_id,
};
module_platform_driver(max77779_gpio_driver);

MODULE_AUTHOR("W4HO <87742609+W4HO-ham@users.noreply.github.com>");
MODULE_DESCRIPTION("GPIO driver for the Maxim MAX77779 PMIC's SGPIOs");
MODULE_LICENSE("GPL");
