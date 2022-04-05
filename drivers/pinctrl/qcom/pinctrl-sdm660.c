/* SPDX-License-Identifier: GPL-2.0+ */

#include <common.h>
#include <dm.h>
#include <log.h>
#include <dm/pinctrl.h>

#include "pinctrl-msm.h"

static struct msm_pinctrl_soc_data sdm660_pinctrl_data = {
	//
};

static int msm_pinctrl_bind(struct udevice *dev)
{
	struct msm_pinctrl *priv = dev_get_priv(dev);
	int ret;

	// bind the gpio part of this driver to the same node
	ret = device_bind_driver_to_node(dev, "msm-gpio", dev->name,
					  dev_ofnode(dev), &priv->gpio);
	// share the priv
	dev_set_priv(priv->gpio, priv);

	return ret;
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
	.of_match	= of_match_ptr(sdm660_pinctrl_match),
	.priv_auto	= sizeof(struct msm_pinctrl_soc_data),
	.ops		= &msm_pinctrl_ops,
	.bind		= msm_pinctrl_bind,
	.probe		= sdm660_pinctrl_probe,
};
