/* SPDX-License-Identifier: GPL-2.0+ */

#include <common.h>
#include <dm.h>
#include <log.h>
#include <dm/pinctrl.h>

#include "pinctrl-msm.h"

static struct msm_pinctrl_soc_data sdm660_pinctrl_data = {
	//
};

static int sdm660_pinctrl_probe(struct udevice *dev)
{
	return msm_pinctrl_probe(dev, &sdm660_pinctrl_data);
}

static const struct udevice_id sdm660_pinctrl_match[] = {
	{
		.compatible = "qcom,sdm630-pinctrl",
		.data = (ulong)&sdm660_pinctrl_data
	},
	{
		.compatible = "qcom,sdm660-pinctrl",
		.data = (ulong)&sdm660_pinctrl_data
	},
	{ }
};

U_BOOT_DRIVER(pinctrl_sdm660) = {
	.name		= "qcom_sdm660_pinctrl",
	.id		= UCLASS_PINCTRL,
	.of_match	= sdm660_pinctrl_match,
	.priv_auto	= sizeof(struct msm_pinctrl_soc_data),
	.ops		= &msm_pinctrl_ops,
#if CONFIG_IS_ENABLED(OF_REAL)
	.bind		= dm_scan_fdt_dev,
#endif
	.probe		= sdm660_pinctrl_probe,
	.remove		= msm_pinctrl_remove
};
