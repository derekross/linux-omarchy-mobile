// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Samsung Electronics.
 *
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

#include "include/gnss.h"
#include "gnss_prj.h"
#include "gnss_utils.h"

/* handling interrupt by BETP messages from kepler through SPI */
static irqreturn_t kepler_spi_irq_handler(int irq, void *data)
{
	struct gnss_ctl *gc = data;
	struct link_device *ld = gc->iod->ld;

	gif_disable_irq_nosync(&gc->irq_gnss2ap_spi);

	if (atomic_read(&gc->wait_rdy)) {
		atomic_set(&gc->wait_rdy, 0);
		complete_all(&gc->gnss_rdy_cmpl);
		return IRQ_HANDLED;
	}

	if (atomic_read(&gc->rx_in_progress) == 0) {
		queue_delayed_work(ld->rx_wq, &ld->rx_dwork, 0);
	}

	return IRQ_HANDLED;
}

static int kepler_spi_status(struct gnss_ctl *gc)
{
	return gpiod_get_value(gc->gpio_gnss2ap_spi.desc);
}

static void kepler_send_betp_int(struct gnss_ctl *gc, int value)
{
	gif_debug("About to Set ap2gnss_spi value to %d\n", value);
	gpiod_set_value(gc->gpio_ap2gnss_spi.desc, value);
	gif_debug("complete to set ap2gnss_spi value to %d\n", value);
}

static int kepler_betp_int_status(struct gnss_ctl *gc)
{
	return gpiod_get_value(gc->gpio_ap2gnss_spi.desc);
}

static void gnss_set_ops(struct gnss_ctl *gc)
{
	gc->ops.gnss_spi_status = kepler_spi_status;
	gc->ops.gnss_send_betp_int = kepler_send_betp_int;
	gc->ops.gnss_betp_int_status = kepler_betp_int_status;
}

static int init_ctl_device(struct gnss_ctl *gc, struct gnss_pdata *pdata)
{
	int ret;

	gnss_set_ops(gc);

	dev_set_drvdata(gc->dev, gc);


	/*
	 * omamobile: the lines as GPIO descriptors from the node's
	 * gnss2ap-spi-gpios and ap2gnss-spi-gpios (stock: of_get_named_gpio()
	 * on gpio_gnss2ap_spi and gpio_ap2gnss_spi), released by devm.
	 */
	gc->gpio_gnss2ap_spi.label = "GNSS2AP_SPI";
	gc->gpio_gnss2ap_spi.desc = devm_gpiod_get(gc->dev, "gnss2ap-spi", GPIOD_IN);
	if (IS_ERR(gc->gpio_gnss2ap_spi.desc)) {
		gif_err("Can't get gnss2ap-spi-gpios (%ld)\n",
			PTR_ERR(gc->gpio_gnss2ap_spi.desc));
		return PTR_ERR(gc->gpio_gnss2ap_spi.desc);
	}
	ret = gpiod_to_irq(gc->gpio_gnss2ap_spi.desc);
	if (ret < 0) {
		gif_err("gnss2ap-spi has no interrupt (%d)\n", ret);
		return ret;
	}
	gc->irq_gnss2ap_spi.num = ret;

	/* Disable the interrupt until gnss_main.c probe function finish */
	gif_configure_irq(&gc->irq_gnss2ap_spi, gc->irq_gnss2ap_spi.num,
			"kepler_spi_irq_handler", IRQF_TRIGGER_HIGH | IRQF_NO_AUTOEN);

	ret = gif_request_irq(&gc->irq_gnss2ap_spi, kepler_spi_irq_handler, gc);
	if (ret) {
		gif_err("Request irq fail - kepler_spi_irq_handler(%d)\n", ret);
		return ret;
	}

	ret = enable_irq_wake(gc->irq_gnss2ap_spi.num);
	if (ret)	/* omamobile: not fatal, the line still interrupts */
		gif_err("enable_irq_wake failed (%d): no wake from GNSS\n", ret);

	gc->gpio_ap2gnss_spi.label = "AP2GNSS_SPI";
	gc->gpio_ap2gnss_spi.desc = devm_gpiod_get(gc->dev, "ap2gnss-spi", GPIOD_OUT_LOW);
	if (IS_ERR(gc->gpio_ap2gnss_spi.desc)) {
		ret = PTR_ERR(gc->gpio_ap2gnss_spi.desc);
		gif_err("Can't get ap2gnss-spi-gpios (%d)\n", ret);
		goto free_gpio_irq;
	}

	init_completion(&gc->gnss_rdy_cmpl);

	atomic_set(&gc->wait_rdy, 0);
	atomic_set(&gc->tx_in_progress, 0);
	atomic_set(&gc->rx_in_progress, 0);

	return 0;

free_gpio_irq:
	free_irq(gc->irq_gnss2ap_spi.num, gc);	/* stock passed &gc */

	return ret;

}

struct gnss_ctl *create_ctl_device(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gnss_pdata *pdata = pdev->dev.platform_data;
	struct gnss_ctl *gc;
	int ret;

	gc = devm_kzalloc(dev, sizeof(struct gnss_ctl), GFP_KERNEL);
	if (!gc)
		return NULL;
	gc->dev = dev;
	gc->pdata = pdata;
	gc->name = pdata->name;

	ret = init_ctl_device(gc, pdata);
	if (ret) {
		gif_err("%s: init_ctl_device fail (err %d)\n",
				pdata->name, ret);
		devm_kfree(dev, gc);
		return NULL;
	}

	return gc;
}
