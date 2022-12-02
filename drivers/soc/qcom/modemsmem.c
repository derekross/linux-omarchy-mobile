// SPDX-License-Identifier: GPL-2.0-only
/*
 * SoC information for the modem in shared memory
 *
 * Copyright (c) 2018 Google Inc.
 * Copyright (c) 2022-2024, Richard Acayan. All rights reserved.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/socinfo.h>
#include <linux/types.h>

#define MODEM_SMEM_VERSION	0

#define PLAT_VER_TO_MAJOR_ID(v)	(((v) >> 16) & 0xff)
#define PLAT_VER_TO_MINOR_ID(v)	((v) & 0xff)

/*
 * The item's layout, as the modem firmware reads it: Google's
 * struct modem_smem_type from their downstream modemsmem.h
 */
struct modem_smem_info {
	__le32 version;
	__le32 modem_flag;
	__le32 major_id;
	__le32 minor_id;
	__le32 subtype;
	__le32 platform;
	__le32 efs_magic;
	__le32 ftm_magic;
};

static int modemsmem_write_socinfo(struct modem_smem_info *info)
{
	struct socinfo *socinfo;
	u32 plat_ver;

	socinfo = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_HW_SW_BUILD_ID, NULL);
	if (IS_ERR(socinfo))
		return PTR_ERR(socinfo);

	/* The platform fields are there from socinfo format 0.6 */
	if (le32_to_cpu(socinfo->fmt) < SOCINFO_VERSION(0, 6))
		return -EOPNOTSUPP;

	plat_ver = le32_to_cpu(socinfo->plat_ver);

	info->version = cpu_to_le32(MODEM_SMEM_VERSION);
	info->major_id = cpu_to_le32(PLAT_VER_TO_MAJOR_ID(plat_ver));
	info->minor_id = cpu_to_le32(PLAT_VER_TO_MINOR_ID(plat_ver));
	info->platform = socinfo->hw_plat;
	info->subtype = socinfo->hw_plat_subtype;

	return 0;
}

static int modemsmem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct modem_smem_info *info;
	size_t size;
	u32 item;
	int ret;

	ret = of_property_read_u32(dev->of_node, "qcom,smem", &item);
	if (ret)
		return dev_err_probe(dev, ret, "no SMEM item\n");

	ret = qcom_smem_alloc(QCOM_SMEM_HOST_ANY, item, sizeof(*info));
	if (ret && ret != -EEXIST)
		return dev_err_probe(dev, ret, "failed to allocate SMEM item %u\n", item);

	info = qcom_smem_get(QCOM_SMEM_HOST_ANY, item, &size);
	if (IS_ERR(info))
		return dev_err_probe(dev, PTR_ERR(info), "failed to get SMEM item %u\n", item);
	if (size < sizeof(*info))
		return dev_err_probe(dev, -EINVAL, "SMEM item %u is too small\n", item);

	ret = modemsmem_write_socinfo(info);
	if (ret)
		return dev_err_probe(dev, ret, "failed to write the SoC information\n");

	return 0;
}

static const struct of_device_id modemsmem_of_match[] = {
	{ .compatible = "google,modemsmem" },
	{ }
};
MODULE_DEVICE_TABLE(of, modemsmem_of_match);

static struct platform_driver modemsmem_driver = {
	.probe = modemsmem_probe,
	.driver = {
		.name = "modemsmem",
		.of_match_table = modemsmem_of_match,
	},
};
module_platform_driver(modemsmem_driver);

MODULE_AUTHOR("Richard Acayan <mailingradian@gmail.com>");
MODULE_DESCRIPTION("SoC information for the modem in shared memory");
MODULE_LICENSE("GPL");
