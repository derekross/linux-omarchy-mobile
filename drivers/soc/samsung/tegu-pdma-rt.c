// SPDX-License-Identifier: GPL-2.0-only
/*
 * Runtime test (MODEM-PLAN G3): register stock's pdma1 (PL330 at
 * 0x10130000, SPI 806) as an AMBA device so the built-in pl330 driver
 * probes it, then run one 256-byte DMA memcpy through it and compare.
 * Proves the controller works from Linux before a DT node probes it at boot.
 */
#include <linux/amba/bus.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/slab.h>

static struct amba_device *adev;
static struct clk_hw *hw;

static bool filter(struct dma_chan *chan, void *param)
{
	return chan->device->dev == param;
}

static int __init rt_init(void)
{
	struct irq_fwspec fw = { 0 };
	struct device_node *gic;
	struct dma_chan *ch;
	dma_cap_mask_t mask;
	void *src, *dst;
	dma_addr_t s, d;
	struct dma_async_tx_descriptor *tx;
	dma_cookie_t ck;
	int ret, irq, i;

	gic = of_find_compatible_node(NULL, NULL, "arm,gic-v3");
	if (!gic)
		return -ENODEV;
	fw.fwnode = of_fwnode_handle(gic);
	fw.param_count = 3;
	fw.param[0] = 0;	/* SPI */
	fw.param[1] = 806;
	fw.param[2] = IRQ_TYPE_LEVEL_HIGH;
	irq = irq_create_fwspec_mapping(&fw);
	of_node_put(gic);
	if (irq <= 0)
		return -EINVAL;

	hw = clk_hw_register_fixed_rate(NULL, "tegu-pdma1-apb", NULL, 0, 200000000);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	clk_hw_register_clkdev(hw, "apb_pclk", "10130000.pdma");

	adev = amba_device_alloc("10130000.pdma", 0x10130000, 0x1000);
	if (!adev)
		return -ENOMEM;
	adev->irq[0] = irq;
	adev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	adev->dev.dma_mask = &adev->dev.coherent_dma_mask;
	ret = amba_device_add(adev, &iomem_resource);
	if (ret) {
		pr_err("tegu-pdma-rt: amba_device_add %d\n", ret);
		amba_device_put(adev);
		adev = NULL;
		return ret;
	}
	msleep(200);
	pr_info("tegu-pdma-rt: device added, driver %s\n",
		adev->dev.driver ? adev->dev.driver->name : "(none)");
	if (!adev->dev.driver)
		return 0;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	ch = dma_request_channel(mask, filter, &adev->dev);
	if (!ch) {
		pr_err("tegu-pdma-rt: no memcpy channel\n");
		return 0;
	}
	src = dma_alloc_coherent(ch->device->dev, 256, &s, GFP_KERNEL);
	dst = dma_alloc_coherent(ch->device->dev, 256, &d, GFP_KERNEL);
	if (!src || !dst)
		goto out;
	for (i = 0; i < 256; i++)
		((u8 *)src)[i] = i ^ 0x5a;
	memset(dst, 0, 256);
	tx = dmaengine_prep_dma_memcpy(ch, d, s, 256, DMA_PREP_INTERRUPT);
	if (!tx) {
		pr_err("tegu-pdma-rt: prep failed\n");
		goto out;
	}
	ck = dmaengine_submit(tx);
	dma_async_issue_pending(ch);
	ret = dma_sync_wait(ch, ck);
	pr_info("tegu-pdma-rt: memcpy status %d, %s (src %pad dst %pad)\n", ret,
		memcmp(src, dst, 256) ? "MISMATCH" : "data matches", &s, &d);
out:
	if (src)
		dma_free_coherent(ch->device->dev, 256, src, s);
	if (dst)
		dma_free_coherent(ch->device->dev, 256, dst, d);
	dma_release_channel(ch);
	return 0;
}

static void __exit rt_exit(void)
{
	if (adev)
		amba_device_unregister(adev);
}

module_init(rt_init);
module_exit(rt_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Runtime test of the Pixel 9a's pdma1 (PL330)");
