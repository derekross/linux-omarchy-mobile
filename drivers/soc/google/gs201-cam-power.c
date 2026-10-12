// SPDX-License-Identifier: GPL-2.0-only
/*
 * gs201 camera module power sequencing and ID reads, for bring-up
 *
 * Before there is a sensor driver, this binds to a camera module's I2C
 * device (sensor or module EEPROM) and, from debugfs, powers it up and down
 * in the order and with the delays the stock device tree gives, and reads
 * single identification registers. It is the minimum needed to learn which
 * sensor sits behind each I2C address and that its rails come up.
 *
 * Device tree:
 *   google,supply-names            supplies in power-up order; each one is
 *                                  "<name>-supply"
 *   google,supply-on-delays-us     wait after enabling each supply
 *   google,supply-off-delays-us    wait after disabling each supply
 *   clocks (optional)              the sensor master clock, enabled as is:
 *                                  its rate is logged, never set
 *   pinctrl "on"/"off" (optional)  master clock pad states
 *   reset-gpios (optional)         asserted = sensor held in reset
 *   google,mclk-delay-us           wait after the clock starts
 *   google,reset-delay-us          wait after reset is released
 *   google,reset-before-mclk       release reset before starting the clock
 *   google,readable-registers      16-bit register addresses that "read"
 *                                  may read; nothing else is ever read
 *   google,reg-value-bytes         1 (default) or 2
 *
 * Nothing is powered at probe; reset and the clock pad are not touched
 * until the first power-up. debugfs, per device, under gs201-cam-power/:
 *   power  write 1 to power up, 0 to power down; read the state
 *   read   write one allowed register address, read back its value
 *   ack    read: one single-byte read with no register address written
 *          first, reporting only whether the device answered
 *
 * It never writes to the I2C device: a register read sends the register
 * address and then reads, which changes no register. Powered devices are
 * powered down when the driver unbinds.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#define CAM_POWER_MAX_SUPPLIES		8
#define CAM_POWER_MAX_REGS		16
/* Stock's wait after reset and after the clock at power-down */
#define CAM_POWER_STEP_US		1000

struct cam_power {
	struct device *dev;
	struct i2c_client *client;
	struct mutex lock;		/* power state and I2C access */

	unsigned int num_supplies;
	const char *supply_names[CAM_POWER_MAX_SUPPLIES];
	struct regulator *supplies[CAM_POWER_MAX_SUPPLIES];
	u32 on_delay_us[CAM_POWER_MAX_SUPPLIES];
	u32 off_delay_us[CAM_POWER_MAX_SUPPLIES];

	struct clk *mclk;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_on;
	struct pinctrl_state *pins_off;
	struct gpio_desc *reset;
	u32 mclk_delay_us;
	u32 reset_delay_us;
	bool reset_before_mclk;

	unsigned int num_regs;
	u32 regs[CAM_POWER_MAX_REGS];
	u32 val_bytes;

	bool powered;
	int last_reg;
	int last_val;
	struct dentry *debugfs;
};

static struct dentry *cam_power_debugfs_root;

static void cam_power_mclk_off(struct cam_power *cp)
{
	clk_disable_unprepare(cp->mclk);
	if (cp->pins_off)
		pinctrl_select_state(cp->pinctrl, cp->pins_off);
}

static int cam_power_mclk_on(struct cam_power *cp)
{
	int ret;

	if (cp->pins_on) {
		ret = pinctrl_select_state(cp->pinctrl, cp->pins_on);
		if (ret)
			return ret;
	}

	ret = clk_prepare_enable(cp->mclk);
	if (ret) {
		if (cp->pins_off)
			pinctrl_select_state(cp->pinctrl, cp->pins_off);
		return ret;
	}

	if (cp->mclk)
		dev_info(cp->dev, "mclk on at %lu Hz\n", clk_get_rate(cp->mclk));
	fsleep(cp->mclk_delay_us);

	return 0;
}

static void cam_power_release_reset(struct cam_power *cp)
{
	if (!cp->reset)
		return;
	gpiod_set_value_cansleep(cp->reset, 0);
	fsleep(cp->reset_delay_us);
}

static void cam_power_supplies_off(struct cam_power *cp, int from)
{
	int i;

	for (i = from; i >= 0; i--) {
		regulator_disable(cp->supplies[i]);
		dev_info(cp->dev, "%s off\n", cp->supply_names[i]);
		fsleep(cp->off_delay_us[i]);
	}
}

static int cam_power_up(struct cam_power *cp)
{
	int i, ret;

	if (cp->powered)
		return 0;

	/* Hold the sensor in reset while its rails come up. */
	if (cp->reset) {
		ret = gpiod_direction_output(cp->reset, 1);
		if (ret)
			return ret;
	}

	for (i = 0; i < cp->num_supplies; i++) {
		ret = regulator_enable(cp->supplies[i]);
		if (ret) {
			dev_err(cp->dev, "%s enable failed: %d\n",
				cp->supply_names[i], ret);
			cam_power_supplies_off(cp, i - 1);
			return ret;
		}
		dev_info(cp->dev, "%s on\n", cp->supply_names[i]);
		fsleep(cp->on_delay_us[i]);
	}

	if (cp->reset_before_mclk)
		cam_power_release_reset(cp);

	ret = cam_power_mclk_on(cp);
	if (ret) {
		dev_err(cp->dev, "mclk enable failed: %d\n", ret);
		if (cp->reset)
			gpiod_set_value_cansleep(cp->reset, 1);
		cam_power_supplies_off(cp, cp->num_supplies - 1);
		return ret;
	}

	if (!cp->reset_before_mclk)
		cam_power_release_reset(cp);

	cp->powered = true;
	dev_info(cp->dev, "powered up\n");

	return 0;
}

static void cam_power_down(struct cam_power *cp)
{
	if (!cp->powered)
		return;

	if (cp->reset) {
		gpiod_set_value_cansleep(cp->reset, 1);
		fsleep(CAM_POWER_STEP_US);
	}

	cam_power_mclk_off(cp);
	fsleep(CAM_POWER_STEP_US);

	cam_power_supplies_off(cp, cp->num_supplies - 1);

	cp->powered = false;
	dev_info(cp->dev, "powered down\n");
}

static int cam_power_read_reg(struct cam_power *cp, u16 reg, u32 *val)
{
	u8 addr[2] = { reg >> 8, reg & 0xff };
	u8 data[2] = { };
	struct i2c_msg msgs[2] = {
		{
			.addr = cp->client->addr,
			.len = sizeof(addr),
			.buf = addr,
		}, {
			.addr = cp->client->addr,
			.flags = I2C_M_RD,
			.len = cp->val_bytes,
			.buf = data,
		},
	};
	int ret;

	ret = i2c_transfer(cp->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;

	*val = cp->val_bytes == 2 ? (data[0] << 8 | data[1]) : data[0];

	return 0;
}

static int cam_power_get(void *data, u64 *val)
{
	struct cam_power *cp = data;

	*val = cp->powered;

	return 0;
}

static int cam_power_set(void *data, u64 val)
{
	struct cam_power *cp = data;
	int ret = 0;

	mutex_lock(&cp->lock);
	if (val)
		ret = cam_power_up(cp);
	else
		cam_power_down(cp);
	mutex_unlock(&cp->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(cam_power_fops, cam_power_get, cam_power_set, "%llu\n");

static ssize_t cam_power_read_write(struct file *file, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct cam_power *cp = file_inode(file)->i_private;
	unsigned int reg;
	u32 val;
	int i, ret;

	ret = kstrtouint_from_user(ubuf, count, 0, &reg);
	if (ret)
		return ret;

	for (i = 0; i < cp->num_regs; i++)
		if (cp->regs[i] == reg)
			break;
	if (i == cp->num_regs) {
		dev_err(cp->dev, "register 0x%04x is not in google,readable-registers\n",
			reg);
		return -EPERM;
	}

	mutex_lock(&cp->lock);
	if (!cp->powered) {
		ret = -EAGAIN;
		goto out;
	}

	dev_info(cp->dev, "reading register 0x%04x\n", reg);
	ret = cam_power_read_reg(cp, reg, &val);
	if (ret) {
		dev_err(cp->dev, "register 0x%04x read failed: %d\n", reg, ret);
		goto out;
	}

	dev_info(cp->dev, "register 0x%04x = 0x%0*x\n", reg, cp->val_bytes * 2,
		 val);
	cp->last_reg = reg;
	cp->last_val = val;
	ret = count;
out:
	mutex_unlock(&cp->lock);
	return ret;
}

static int cam_power_read_show(struct seq_file *s, void *unused)
{
	struct cam_power *cp = s->private;

	mutex_lock(&cp->lock);
	if (cp->last_reg < 0)
		seq_puts(s, "none\n");
	else
		seq_printf(s, "0x%04x = 0x%0*x\n", cp->last_reg,
			   cp->val_bytes * 2, cp->last_val);
	mutex_unlock(&cp->lock);

	return 0;
}

static int cam_power_read_open(struct inode *inode, struct file *file)
{
	return single_open(file, cam_power_read_show, inode->i_private);
}

static const struct file_operations cam_power_read_fops = {
	.owner = THIS_MODULE,
	.open = cam_power_read_open,
	.read = seq_read,
	.write = cam_power_read_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int cam_power_ack_show(struct seq_file *s, void *unused)
{
	struct cam_power *cp = s->private;
	u8 byte;
	int ret;

	mutex_lock(&cp->lock);
	if (!cp->powered) {
		mutex_unlock(&cp->lock);
		return -EAGAIN;
	}

	/* A current-address read: no register address is written. */
	ret = i2c_master_recv(cp->client, &byte, 1);
	mutex_unlock(&cp->lock);

	if (ret == 1)
		seq_puts(s, "ack\n");
	else
		seq_printf(s, "no answer (%d)\n", ret);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cam_power_ack);

static int cam_power_parse(struct cam_power *cp)
{
	struct device *dev = cp->dev;
	int i, n, ret;

	n = device_property_string_array_count(dev, "google,supply-names");
	if (n < 1 || n > CAM_POWER_MAX_SUPPLIES)
		return dev_err_probe(dev, -EINVAL, "need 1-%d google,supply-names\n",
				     CAM_POWER_MAX_SUPPLIES);
	cp->num_supplies = n;

	ret = device_property_read_string_array(dev, "google,supply-names",
						cp->supply_names, n);
	if (ret < 0)
		return ret;

	if (device_property_count_u32(dev, "google,supply-on-delays-us") != n ||
	    device_property_count_u32(dev, "google,supply-off-delays-us") != n)
		return dev_err_probe(dev, -EINVAL,
				     "need one on and one off delay per supply\n");

	ret = device_property_read_u32_array(dev, "google,supply-on-delays-us",
					     cp->on_delay_us, n);
	if (ret)
		return ret;
	ret = device_property_read_u32_array(dev, "google,supply-off-delays-us",
					     cp->off_delay_us, n);
	if (ret)
		return ret;

	for (i = 0; i < n; i++) {
		cp->supplies[i] = devm_regulator_get(dev, cp->supply_names[i]);
		if (IS_ERR(cp->supplies[i]))
			return dev_err_probe(dev, PTR_ERR(cp->supplies[i]),
					     "supply %s\n", cp->supply_names[i]);
	}

	device_property_read_u32(dev, "google,mclk-delay-us", &cp->mclk_delay_us);
	device_property_read_u32(dev, "google,reset-delay-us", &cp->reset_delay_us);
	cp->reset_before_mclk = device_property_read_bool(dev, "google,reset-before-mclk");

	cp->val_bytes = 1;
	device_property_read_u32(dev, "google,reg-value-bytes", &cp->val_bytes);
	if (cp->val_bytes != 1 && cp->val_bytes != 2)
		return dev_err_probe(dev, -EINVAL, "google,reg-value-bytes must be 1 or 2\n");

	n = device_property_count_u32(dev, "google,readable-registers");
	if (n > CAM_POWER_MAX_REGS)
		return dev_err_probe(dev, -EINVAL, "too many readable registers\n");
	if (n > 0) {
		ret = device_property_read_u32_array(dev, "google,readable-registers",
						     cp->regs, n);
		if (ret)
			return ret;
		cp->num_regs = n;
	}

	cp->mclk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(cp->mclk))
		return dev_err_probe(dev, PTR_ERR(cp->mclk), "mclk\n");

	/* Not driven until the first power-up. */
	cp->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(cp->reset))
		return dev_err_probe(dev, PTR_ERR(cp->reset), "reset\n");

	cp->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(cp->pinctrl)) {
		if (PTR_ERR(cp->pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		cp->pinctrl = NULL;
	}
	if (cp->pinctrl) {
		cp->pins_on = pinctrl_lookup_state(cp->pinctrl, "on");
		cp->pins_off = pinctrl_lookup_state(cp->pinctrl, "off");
		if (IS_ERR(cp->pins_on) || IS_ERR(cp->pins_off)) {
			cp->pins_on = NULL;
			cp->pins_off = NULL;
		}
	}

	return 0;
}

static int cam_power_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cam_power *cp;
	int ret;

	cp = devm_kzalloc(dev, sizeof(*cp), GFP_KERNEL);
	if (!cp)
		return -ENOMEM;

	cp->dev = dev;
	cp->client = client;
	cp->last_reg = -1;
	mutex_init(&cp->lock);
	i2c_set_clientdata(client, cp);

	ret = cam_power_parse(cp);
	if (ret)
		return ret;

	cp->debugfs = debugfs_create_dir(dev_name(dev), cam_power_debugfs_root);
	debugfs_create_file_unsafe("power", 0600, cp->debugfs, cp, &cam_power_fops);
	debugfs_create_file("read", 0600, cp->debugfs, cp, &cam_power_read_fops);
	debugfs_create_file("ack", 0400, cp->debugfs, cp, &cam_power_ack_fops);

	dev_info(dev, "%u supplies, %u readable registers, %s, %s; not powered\n",
		 cp->num_supplies, cp->num_regs,
		 cp->mclk ? "mclk" : "no mclk",
		 cp->reset ? "reset" : "no reset");

	return 0;
}

static void cam_power_remove(struct i2c_client *client)
{
	struct cam_power *cp = i2c_get_clientdata(client);

	debugfs_remove_recursive(cp->debugfs);

	mutex_lock(&cp->lock);
	cam_power_down(cp);
	mutex_unlock(&cp->lock);
}

static const struct of_device_id cam_power_of_match[] = {
	{ .compatible = "google,gs201-camera-power-test" },
	{ }
};
MODULE_DEVICE_TABLE(of, cam_power_of_match);

static struct i2c_driver cam_power_driver = {
	.driver = {
		.name = "gs201-cam-power",
		.of_match_table = cam_power_of_match,
	},
	.probe = cam_power_probe,
	.remove = cam_power_remove,
};

static int __init cam_power_init(void)
{
	int ret;

	cam_power_debugfs_root = debugfs_create_dir("gs201-cam-power", NULL);

	ret = i2c_add_driver(&cam_power_driver);
	if (ret)
		debugfs_remove_recursive(cam_power_debugfs_root);

	return ret;
}
module_init(cam_power_init);

static void __exit cam_power_exit(void)
{
	i2c_del_driver(&cam_power_driver);
	debugfs_remove_recursive(cam_power_debugfs_root);
}
module_exit(cam_power_exit);

MODULE_DESCRIPTION("gs201 camera module power sequencing and ID reads (bring-up)");
MODULE_LICENSE("GPL");
