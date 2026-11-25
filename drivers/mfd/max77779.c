// SPDX-License-Identifier: GPL-2.0-only
//
// Core driver for the Maxim MAX77779 interface PMIC's PMIC block
//
// The MAX77779 (Google Pixel 9a) is a set of I2C blocks on one chip: the
// charger, the fuel gauge, the Type-C port controller, the battery voltage
// and current monitor (VIMON), an I2C master and, at 0x66, the PMIC block.
// The PMIC block owns the chip's interrupt line (INTB) and collects the
// other blocks' interrupts as sources in INTSRC_STS, masked in INTB_MASK.
// This driver is the interrupt controller for those sources; the blocks
// at their own addresses name their source in it. The block's eight SGPIOs
// interrupt through source 5 and are added as a child device.

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77779.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>

/* The interrupt sources: their bits in INTSRC_STS and INTB_MASK */
enum {
	MAX77779_PMIC_INT_TCPC,
	MAX77779_PMIC_INT_FG,
	MAX77779_PMIC_INT_CHGR,
	MAX77779_PMIC_INT_I2CM,
	MAX77779_PMIC_INT_BATTVIMON,
	MAX77779_PMIC_INT_GPIO,
	MAX77779_PMIC_INT_VDROOP,
	MAX77779_PMIC_INT_PMICTOP,
};

/* What probe and the interrupt handlers read */
static const struct regmap_range max77779_pmic_rd_ranges[] = {
	regmap_reg_range(MAX77779_PMIC_ID, MAX77779_PMIC_REVISION),
	regmap_reg_range(MAX77779_PMIC_INTSRC_STS, MAX77779_PMIC_INTSRC_STS),
	regmap_reg_range(MAX77779_PMIC_INTB_MASK, MAX77779_PMIC_INTB_MASK),
	regmap_reg_range(MAX77779_SGPIO_INT, MAX77779_SGPIO_PD),
	regmap_reg_range(MAX77779_SGPIO_CNFG(0), MAX77779_SGPIO_CNFG(7)),
};

/* The source status and mask, and the SGPIOs (not the AGPIO pulls) */
static const struct regmap_range max77779_pmic_wr_ranges[] = {
	regmap_reg_range(MAX77779_PMIC_INTSRC_STS, MAX77779_PMIC_INTSRC_STS),
	regmap_reg_range(MAX77779_PMIC_INTB_MASK, MAX77779_PMIC_INTB_MASK),
	regmap_reg_range(MAX77779_SGPIO_INT, MAX77779_SGPIO_PD),
	regmap_reg_range(MAX77779_SGPIO_CNFG(0), MAX77779_SGPIO_CNFG(7)),
};

static const struct regmap_access_table max77779_pmic_rd_table = {
	.yes_ranges = max77779_pmic_rd_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77779_pmic_rd_ranges),
};

static const struct regmap_access_table max77779_pmic_wr_table = {
	.yes_ranges = max77779_pmic_wr_ranges,
	.n_yes_ranges = ARRAY_SIZE(max77779_pmic_wr_ranges),
};

/* No cache: the status registers and CNFGx's live level change under us */
static const struct regmap_config max77779_pmic_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX77779_SGPIO_CNFG(7),
	.rd_table = &max77779_pmic_rd_table,
	.wr_table = &max77779_pmic_wr_table,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_irq max77779_pmic_irqs[] = {
	REGMAP_IRQ_REG(MAX77779_PMIC_INT_TCPC, 0, BIT(MAX77779_PMIC_INT_TCPC)),
	REGMAP_IRQ_REG(MAX77779_PMIC_INT_FG, 0, BIT(MAX77779_PMIC_INT_FG)),
	REGMAP_IRQ_REG(MAX77779_PMIC_INT_CHGR, 0, BIT(MAX77779_PMIC_INT_CHGR)),
	REGMAP_IRQ_REG(MAX77779_PMIC_INT_I2CM, 0, BIT(MAX77779_PMIC_INT_I2CM)),
	REGMAP_IRQ_REG(MAX77779_PMIC_INT_BATTVIMON, 0,
		       BIT(MAX77779_PMIC_INT_BATTVIMON)),
	REGMAP_IRQ_REG(MAX77779_PMIC_INT_GPIO, 0, BIT(MAX77779_PMIC_INT_GPIO)),
	REGMAP_IRQ_REG(MAX77779_PMIC_INT_VDROOP, 0, BIT(MAX77779_PMIC_INT_VDROOP)),
	REGMAP_IRQ_REG(MAX77779_PMIC_INT_PMICTOP, 0, BIT(MAX77779_PMIC_INT_PMICTOP)),
};

/*
 * Every source starts masked, and a status bit left by the bootloader is
 * acknowledged before the line is requested. A source's bit is cleared
 * before its handler runs; the blocks behind it keep their own status.
 */
static const struct regmap_irq_chip max77779_pmic_irq_chip = {
	.name = "max77779-pmic",
	.status_base = MAX77779_PMIC_INTSRC_STS,
	.mask_base = MAX77779_PMIC_INTB_MASK,
	.ack_base = MAX77779_PMIC_INTSRC_STS,
	.init_ack_masked = true,
	.num_regs = 1,
	.irqs = max77779_pmic_irqs,
	.num_irqs = ARRAY_SIZE(max77779_pmic_irqs),
};

static const struct resource max77779_sgpio_resources[] = {
	DEFINE_RES_IRQ_NAMED(MAX77779_PMIC_INT_GPIO, "SGPIO"),
};

static const struct mfd_cell max77779_pmic_cells[] = {
	MFD_CELL_OF("max77779-sgpio", max77779_sgpio_resources, NULL, 0, 0,
		    "maxim,max77779-sgpio"),
};

/* Without the chip's line the SGPIOs still work, with no interrupts */
static const struct mfd_cell max77779_pmic_cells_no_irq[] = {
	MFD_CELL_OF("max77779-sgpio", NULL, NULL, 0, 0,
		    "maxim,max77779-sgpio"),
};

static int max77779_pmic_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regmap_irq_chip_data *irq_data;
	struct regmap *map;
	unsigned int id, rev;
	int ret;

	map = devm_regmap_init_i2c(client, &max77779_pmic_regmap_config);
	if (IS_ERR(map))
		return dev_err_probe(dev, PTR_ERR(map), "can't init the regmap\n");

	ret = regmap_read(map, MAX77779_PMIC_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "can't read the ID\n");
	if (id != MAX77779_PMIC_ID_VALUE)
		return dev_err_probe(dev, -ENODEV, "ID 0x%02x is not a MAX77779\n", id);
	ret = regmap_read(map, MAX77779_PMIC_REVISION, &rev);
	if (ret)
		return dev_err_probe(dev, ret, "can't read the revision\n");
	dev_info(dev, "MAX77779 PMIC revision 0x%02x\n", rev);

	if (client->irq <= 0) {
		dev_info(dev, "no interrupt line: the sources don't interrupt\n");
		return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO,
					    max77779_pmic_cells_no_irq,
					    ARRAY_SIZE(max77779_pmic_cells_no_irq),
					    NULL, 0, NULL);
	}

	ret = devm_regmap_add_irq_chip(dev, map, client->irq, IRQF_ONESHOT, 0,
				       &max77779_pmic_irq_chip, &irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "can't add the interrupt controller\n");

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO,
				    max77779_pmic_cells,
				    ARRAY_SIZE(max77779_pmic_cells), NULL, 0,
				    regmap_irq_get_domain(irq_data));
}

static const struct of_device_id max77779_pmic_of_match[] = {
	{ .compatible = "maxim,max77779-pmic" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77779_pmic_of_match);

static const struct i2c_device_id max77779_pmic_i2c_id[] = {
	{ "max77779-pmic" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77779_pmic_i2c_id);

static struct i2c_driver max77779_pmic_driver = {
	.driver = {
		.name = "max77779-pmic",
		.of_match_table = max77779_pmic_of_match,
	},
	.probe = max77779_pmic_probe,
	.id_table = max77779_pmic_i2c_id,
};
module_i2c_driver(max77779_pmic_driver);

MODULE_AUTHOR("W4HO <87742609+W4HO-ham@users.noreply.github.com>");
MODULE_DESCRIPTION("Maxim MAX77779 PMIC block core driver");
MODULE_LICENSE("GPL");
