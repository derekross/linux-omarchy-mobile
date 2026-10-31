// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI TUSB2E11 eUSB2 to USB 2.0 repeater
 *
 * The repeater takes its role (host or peripheral) from the eUSB2 link, so
 * the driver only powers it, enables its port and writes the board's
 * signal tuning. The registers and the 3 ms ready time follow Google's GPL
 * eusb_repeater driver for the Pixel 9a.
 */

#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define TUSB2E11_UART_PORT1		0x50
#define TUSB2E11_U_TX_ADJUST_PORT1	0x70
#define TUSB2E11_U_HS_TX_PRE_EMPH_P1	0x71
#define TUSB2E11_U_RX_ADJUST_PORT1	0x72
#define TUSB2E11_U_DISCONNECT_SQUELCH	0x73
#define TUSB2E11_E_HS_TX_PRE_EMPH_P1	0x77
#define TUSB2E11_E_TX_ADJUST_PORT1	0x78
#define TUSB2E11_E_RX_ADJUST_PORT1	0x79
#define TUSB2E11_REV_ID			0xb0
#define TUSB2E11_GLOBAL_CONFIG		0xb2
#define  TUSB2E11_GLOBAL_DISABLE_P1	BIT(6)

/* time from power-up until the repeater accepts I2C and RAP requests */
#define TUSB2E11_READY_US		3000

static const struct {
	const char *prop;
	u8 reg;
} tusb2e11_tunes[] = {
	{ "ti,uart-port1", TUSB2E11_UART_PORT1 },
	{ "ti,usb2-tx-adjust", TUSB2E11_U_TX_ADJUST_PORT1 },
	{ "ti,usb2-hs-tx-pre-emphasis", TUSB2E11_U_HS_TX_PRE_EMPH_P1 },
	{ "ti,usb2-rx-adjust", TUSB2E11_U_RX_ADJUST_PORT1 },
	{ "ti,usb2-disconnect-squelch", TUSB2E11_U_DISCONNECT_SQUELCH },
	{ "ti,eusb2-hs-tx-pre-emphasis", TUSB2E11_E_HS_TX_PRE_EMPH_P1 },
	{ "ti,eusb2-tx-adjust", TUSB2E11_E_TX_ADJUST_PORT1 },
	{ "ti,eusb2-rx-adjust", TUSB2E11_E_RX_ADJUST_PORT1 },
};

struct tusb2e11 {
	struct regmap *regmap;
	struct regulator *vdd;
	struct phy *phy;
	/* the board's tuning, one value per tusb2e11_tunes entry */
	u8 tune[ARRAY_SIZE(tusb2e11_tunes)];
	unsigned long tune_set;
};

static int tusb2e11_init(struct phy *phy)
{
	struct tusb2e11 *tusb = phy_get_drvdata(phy);
	unsigned int i;
	int ret;

	ret = regulator_enable(tusb->vdd);
	if (ret)
		return ret;
	fsleep(TUSB2E11_READY_US);

	ret = regmap_clear_bits(tusb->regmap, TUSB2E11_GLOBAL_CONFIG,
				TUSB2E11_GLOBAL_DISABLE_P1);
	if (ret)
		goto err;

	for_each_set_bit(i, &tusb->tune_set, ARRAY_SIZE(tusb2e11_tunes)) {
		ret = regmap_write(tusb->regmap, tusb2e11_tunes[i].reg,
				   tusb->tune[i]);
		if (ret)
			goto err;
	}

	return 0;

err:
	regulator_disable(tusb->vdd);
	return ret;
}

static int tusb2e11_exit(struct phy *phy)
{
	struct tusb2e11 *tusb = phy_get_drvdata(phy);

	regmap_set_bits(tusb->regmap, TUSB2E11_GLOBAL_CONFIG,
			TUSB2E11_GLOBAL_DISABLE_P1);

	return regulator_disable(tusb->vdd);
}

static const struct phy_ops tusb2e11_ops = {
	.init = tusb2e11_init,
	.exit = tusb2e11_exit,
	.owner = THIS_MODULE,
};

static const struct regmap_config tusb2e11_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static int tusb2e11_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct phy_provider *provider;
	struct tusb2e11 *tusb;
	unsigned int i;
	u32 val;

	tusb = devm_kzalloc(dev, sizeof(*tusb), GFP_KERNEL);
	if (!tusb)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(tusb2e11_tunes); i++) {
		if (of_property_read_u32(dev->of_node, tusb2e11_tunes[i].prop, &val))
			continue;
		if (val > 0xff)
			return dev_err_probe(dev, -EINVAL, "%s out of range\n",
					     tusb2e11_tunes[i].prop);
		tusb->tune[i] = val;
		__set_bit(i, &tusb->tune_set);
	}

	tusb->regmap = devm_regmap_init_i2c(client, &tusb2e11_regmap_config);
	if (IS_ERR(tusb->regmap))
		return PTR_ERR(tusb->regmap);

	tusb->vdd = devm_regulator_get(dev, "vdd3v3");
	if (IS_ERR(tusb->vdd))
		return dev_err_probe(dev, PTR_ERR(tusb->vdd), "no vdd3v3 supply\n");

	tusb->phy = devm_phy_create(dev, dev->of_node, &tusb2e11_ops);
	if (IS_ERR(tusb->phy))
		return dev_err_probe(dev, PTR_ERR(tusb->phy), "failed to create PHY\n");
	phy_set_drvdata(tusb->phy, tusb);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id tusb2e11_of_match[] = {
	{ .compatible = "ti,tusb2e11" },
	{ }
};
MODULE_DEVICE_TABLE(of, tusb2e11_of_match);

static struct i2c_driver tusb2e11_driver = {
	.driver = {
		.name = "tusb2e11",
		.of_match_table = tusb2e11_of_match,
	},
	.probe = tusb2e11_probe,
};
module_i2c_driver(tusb2e11_driver);

MODULE_DESCRIPTION("TI TUSB2E11 eUSB2 repeater");
MODULE_LICENSE("GPL");
