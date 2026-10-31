// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only probe (MODEM-PLAN G3, DMA for the GNSS SPI): read the PL330
 * PrimeCell IDs of stock's pdma1 (0x10130000) once, one register per read,
 * to see whether the block answers Linux before a DT node probes it.
 */
#include <linux/io.h>
#include <linux/module.h>

static int __init pdma_probe_init(void)
{
	void __iomem *b = ioremap(0x10130000, 0x1000);
	u32 p0, p1, c0, c1;

	if (!b)
		return -ENOMEM;
	p0 = readl(b + 0xfe0);
	p1 = readl(b + 0xfe4);
	c0 = readl(b + 0xff0);
	c1 = readl(b + 0xff4);
	pr_info("tegu-pdma-probe: periph 0x%02x 0x%02x cell 0x%02x 0x%02x\n",
		p0 & 0xff, p1 & 0xff, c0 & 0xff, c1 & 0xff);
	iounmap(b);
	return -EAGAIN;	/* nothing to keep loaded */
}
module_init(pdma_probe_init);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Read-only PL330 ID probe for the Pixel 9a's pdma1");
