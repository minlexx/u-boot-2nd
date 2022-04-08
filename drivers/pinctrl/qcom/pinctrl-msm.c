// SPDX-License-Identifier: GPL-2.0
/*
 * Common functions for all qcom pinctrl drivers
 * Ported from Linux
 */

#define DEBUG

#include <asm/global_data.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <common.h>
#include <dm.h>
#include <dm/device.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <dm/devres.h>
#include <dm/lists.h>
#include <dm/of_access.h>
#include <dm/pinctrl.h>
#include <dm/read.h>
#include <linux/bitops.h>
#include <linux/bitmap.h>
#include <linux/errno.h>
#include <linux/log2.h>
#include <log.h>
#include <regmap.h>
#include <syscon.h>

#include "pinctrl-msm.h"

#define MAX_NR_GPIO	300
#define MAX_NR_TILES	4

/**
 * struct msm_pinctrl - state for a pinctrl-msm device
 * @dev:            device handle.
 * @pctrl:          pinctrl handle.
 * @gpio:           gpio device.
 * @soc:            Reference to soc_data of platform specific data.
 * @regs:           Base addresses for the TLMM tiles.
 * @phys_base:      Physical base address
 * @reserved_gpios: bitmap or reserved GPIOs. can not touch them
 */
struct msm_pinctrl {
	struct udevice *dev;
	struct pinctrl_dev *pctrl;
	struct udevice *gpio;

	const struct msm_pinctrl_soc_data *soc;
	void __iomem *regs[MAX_NR_TILES];
	u32 phys_base[MAX_NR_TILES];

	/* in Linux this is handled by gpiolib core */
	DECLARE_BITMAP(reserved_gpios, MAX_NR_GPIO);
};

#define MSM_ACCESSOR(name) \
static u32 msm_readl_##name(struct msm_pinctrl *pctrl, \
			    const struct msm_pingroup *g) \
{ \
	return readl(pctrl->regs[g->tile] + g->name##_reg); \
} \
static void msm_writel_##name(u32 val, struct msm_pinctrl *pctrl, \
			      const struct msm_pingroup *g) \
{ \
	writel(val, pctrl->regs[g->tile] + g->name##_reg); \
}

MSM_ACCESSOR(ctl)
MSM_ACCESSOR(io)
/* this is undefined, because U-Boot does not support IRQs */
#ifdef IRQ_STUFF_IS_INTENTIONALLY_DISABLED
MSM_ACCESSOR(intr_cfg)
MSM_ACCESSOR(intr_status)
MSM_ACCESSOR(intr_target)
#endif

#ifdef IRQ_STUFF_IS_INTENTIONALLY_DISABLED
static void msm_ack_intr_status(struct msm_pinctrl *pctrl,
				const struct msm_pingroup *g)
{
	u32 val = g->intr_ack_high ? BIT(g->intr_status_bit) : 0;

	msm_writel_intr_status(val, pctrl, g);
}

static void msm_gpio_irq_mask(struct msm_pinctrl *pctrl, int selector)
{
	const struct msm_pingroup *g = &pctrl->soc->groups[selector];
	u32 val;

	val = msm_readl_intr_cfg(pctrl, g);
	val &= ~BIT(g->intr_raw_status_bit);
	val &= ~BIT(g->intr_enable_bit);
	msm_writel_intr_cfg(val, pctrl, g);
}
#endif

/* value stored in pull-bits, configures bias-disable/pull-up/pull-down */
enum msm_pin_pull_status {
	MSM_NO_PULL = 0,
	MSM_PULL_DOWN = 1,
	MSM_KEEPER = 2, /* PIN_CONFIG_BIAS_BUS_HOLD */
	MSM_PULL_UP_NO_KEEPER = 2,
	MSM_PULL_UP = 3
};

/* convert value stored in hw drive-strength bits to human (mA) */
static unsigned msm_pinctrl_regval_to_drive(u32 val)
{
	return (val + 1) * 2;
}

/* convert drive-strength value from dts to hw register value */
static unsigned int msm_pinctrl_drive_to_regval(u32 drive)
{
	return drive / 2 - 1;
}

static bool msm_is_reserved_pin(const struct msm_pinctrl *pctrl, unsigned pin)
{
	return test_bit(pin, pctrl->reserved_gpios);
}

/**
 * This method is used to "calculate" the mask and bit-shift inside the
 * ctl reg, used to read/write specified pin electrical property.
 * This method contains basically the list of all pin properties supported
 * by MSM pin controller block (TLMM).
 *
 * @group: pointer to pin group description, has all register offsets
 * @param: one of enum pin_config_param values
 * @mask: output, use this mask to read the param from hw
 * @bit: output, bit shift to read the param from hw
 */
static int calc_ctl_bit_mask(const struct msm_pingroup *group,
			     unsigned param,
			     unsigned *mask,
			     unsigned *bit)
{
	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_BUS_HOLD:
	case PIN_CONFIG_BIAS_PULL_UP:
		*bit = group->pull_bit;
		*mask = 3;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		*bit = group->od_bit;
		*mask = 1;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		*bit = group->drv_bit;
		*mask = 7;
		break;
	case PIN_CONFIG_OUTPUT:
	case PIN_CONFIG_INPUT_ENABLE:
		*bit = group->oe_bit;
		*mask = 1;
		break;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

static int msm_pinctrl_get_groups_count(struct udevice *dev)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);

	return pctrl->soc->ngroups;
}

static const char *msm_pinctrl_get_group_name(struct udevice *dev,
				      unsigned group)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);

	return pctrl->soc->groups[group].name;
}

static int msm_pinctrl_get_pins_count(struct udevice *dev)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	return pctrl->soc->npins;
}

static char pin_name[PINNAME_SIZE];
static const char *msm_pinctrl_get_pin_name(struct udevice *dev,
					    unsigned int selector)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	const struct msm_pinctrl_soc_data *soc = pctrl->soc;
	pin_name[0] = 0;
	if (selector >=0 && selector < soc->npins) {
		memset(pin_name, 0, PINNAME_SIZE);
		strncpy(pin_name, soc->groups[selector].name, PINNAME_SIZE);
	}
	pin_name[PINNAME_SIZE - 1] = '\0';
	return pin_name;
}

static int msm_pinctrl_get_funcs_count(struct udevice *dev)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);

	return pctrl->soc->nfunctions;
}

static const char *msm_pinctrl_get_func_name(struct udevice *dev,
					     unsigned function)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);

	return pctrl->soc->functions[function].name;
}

static inline bool is_sdcard_pin(const struct msm_pingroup *g)
{
	/* sdcard pins have no other functions */
	return !!(g->mux_bit >= 31);
}

/* This is used for debugging (cmd pinmux status) */
static int msm_pinctrl_get_pin_muxing(struct udevice *dev,
				      unsigned int selector,
				      char *buf,
				      int size)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	const struct msm_pingroup *gr = &pctrl->soc->groups[selector];
	int value;
	u32 ctl;
	unsigned mask, bit;
	char dsbuf[16];

	/* if pin is reserved, we can't read its registers */
	if (msm_is_reserved_pin(pctrl, selector)) {
		strncpy(buf, "(reserved)", size);
		return 0;
	}

	/* read ctl register */
	ctl = msm_readl_ctl(pctrl, gr);

	memset(buf, 0, size);

	/* SD card pins are special and don't have some props */
	if (!is_sdcard_pin(gr)) {
		/* if output-enable bit is set, pin is output */
		value = (ctl & BIT(gr->oe_bit) ? GPIOF_OUTPUT : GPIOF_INPUT);
		switch (value) {
		case GPIOF_INPUT:  strncat(buf, "in  ", size); break;
		case GPIOF_OUTPUT: strncat(buf, "out ", size); break;
		}
	}

	/* parse bias/pull */
	calc_ctl_bit_mask(gr, PIN_CONFIG_BIAS_DISABLE, &mask, &bit);
	value = (ctl >> bit) & mask;

	switch (value)
	{
	case MSM_NO_PULL:
		strncat(buf, "no-pull ", size);
		break;
	case MSM_PULL_DOWN:
		strncat(buf, "pull-dn ", size);
		break;
	case MSM_PULL_UP_NO_KEEPER:
		/* If the SoC does not support keeper bias, this is pull-up */
		if (pctrl->soc->pull_no_keeper)
			strncat(buf, "pull-up ", size);
		else
			strncat(buf, "keeper  ", size);
		break;
	case MSM_PULL_UP:
		strncat(buf, "pull-up ", size);
		break;
	}

	/* parse open drain */
	if (!is_sdcard_pin(gr)) {
		calc_ctl_bit_mask(gr, PIN_CONFIG_DRIVE_OPEN_DRAIN, &mask, &bit);
		value = (ctl >> bit) & mask;
		if (value)
			strncat(buf, "open-drain ", size);
		else
			strncat(buf, "           ", size);
	}

	/* parse drive-strength */
	calc_ctl_bit_mask(gr, PIN_CONFIG_DRIVE_STRENGTH, &mask, &bit);
	value = (ctl >> bit) & mask;
	value = msm_pinctrl_regval_to_drive(value);
	memset(dsbuf, 0, sizeof(dsbuf));
	snprintf(dsbuf, sizeof(dsbuf), "%2umA ", value);
	strncat(buf, dsbuf, size);

	/* parse mux (function) */
	if (!is_sdcard_pin(gr)) {
		int func;
		value = (ctl >> gr->mux_bit) & 7;

		if (value >= 0 || value < gr->nfuncs) {
			func = gr->funcs[value];
			strncat(buf, " f:", size);
			strncat(buf, pctrl->soc->functions[func].name, size);
		} else {
			strncat(buf, "f:unknown, (value: ", size);
			memset(dsbuf, 0, sizeof(dsbuf));
			snprintf(dsbuf, sizeof(dsbuf) - 1, "%u)", value);
			strncat(buf, dsbuf, size);
		}
	}

	buf[size - 1] = '\0';
	return 0;
}

#if CONFIG_IS_ENABLED(PINCONF)
const struct pinconf_param msm_pinconf_params[] = {
	{ "bias-disable", PIN_CONFIG_BIAS_DISABLE, 0 },
	{ "bias-pull-up", PIN_CONFIG_BIAS_PULL_UP, 0 },
	{ "bias-pull-down", PIN_CONFIG_BIAS_PULL_DOWN, 0 },
	{ "bias-bus-hold", PIN_CONFIG_BIAS_BUS_HOLD, 0 },
	{ "drive-open-drain", PIN_CONFIG_DRIVE_OPEN_DRAIN, 0 },
	{ "drive-strength", PIN_CONFIG_DRIVE_STRENGTH, 2 },
	{ "output-high", PIN_CONFIG_OUTPUT, 1 },
	{ "output-low", PIN_CONFIG_OUTPUT, 0 },
	{ "input-enable", PIN_CONFIG_INPUT_ENABLE, 0},
};

/**
* pinconf_set callback: Configure an individual pin with a parameter
* @dev: Pinctrl device to use
* @pin: The pin selector
* @param: An enum pin_config_param from @pinconf_params
* @argument: The argument to this param from the device tree, or
*            @pinconf_params.default_value
*
* This function is necessary for pin configuration against a single
* pin.
*
* @Return: 0 if OK, or negative error code on failure
*/
static int msm_pinctrl_pinconf_set(struct udevice *dev, unsigned int pin,
				   unsigned int param, unsigned int arg)
{
	int ret = 0;
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	const struct msm_pingroup *gr = &pctrl->soc->groups[pin];
	u32 val;
	unsigned mask, bit;

	ret = calc_ctl_bit_mask(gr, param, &mask, &bit);
	if (ret < 0)
		return ret;

	/* Convert pinconf values to register values */
	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		arg = MSM_NO_PULL;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		arg = MSM_PULL_DOWN;
		break;
	case PIN_CONFIG_BIAS_BUS_HOLD:
		if (pctrl->soc->pull_no_keeper)
			return -ENOTSUPP;

		arg = MSM_KEEPER;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (pctrl->soc->pull_no_keeper)
			arg = MSM_PULL_UP_NO_KEEPER;
		else
			arg = MSM_PULL_UP;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		/* Check for invalid values */
		if (arg > 16 || arg < 2 || (arg % 2) != 0)
			arg = -1;
		else
			arg = msm_pinctrl_drive_to_regval(arg);
		break;
	case PIN_CONFIG_OUTPUT:
		/* set output value */
		val = msm_readl_io(pctrl, gr);
		if (arg)
			val |= BIT(gr->out_bit);
		else
			val &= ~BIT(gr->out_bit);
		msm_writel_io(val, pctrl, gr);

		/* enable output */
		arg = 1;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		/* disable output */
		arg = 0;
		break;
	default:
		dev_err(pctrl->dev, "Unsupported config parameter: %x\n",
			param);
		return -EINVAL;
	}

	/* Range-check user-supplied value */
	if (arg & ~mask) {
		dev_err(pctrl->dev, "config %x: %x is invalid\n", param, arg);
		return -EINVAL;
	}

	val = msm_readl_ctl(pctrl, gr);
	val &= ~(mask << bit);
	val |= arg << bit;
	msm_writel_ctl(val, pctrl, gr);

	return ret;
}

#endif /* CONFIG_IS_ENABLED(PINCONF) */

static int msm_pinctrl_set_pinmux(struct udevice *dev,
				  unsigned pin_selector,
				  unsigned func_selector)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	// unsigned int gpio_func = pctrl->soc->gpio_func;
	unsigned int egpio_func = pctrl->soc->egpio_func;
	const struct msm_pingroup *g;
	u32 val, mask;
	int i;

	g = &pctrl->soc->groups[pin_selector];
	mask = GENMASK(g->mux_bit + order_base_2(g->nfuncs) - 1, g->mux_bit);

	for (i = 0; i < g->nfuncs; i++) {
		if (g->funcs[i] == func_selector)
			break;
	}

	if (i == g->nfuncs)
		return -EINVAL;

	val = msm_readl_ctl(pctrl, g);

	if (egpio_func && i == egpio_func) {
		if (val & BIT(g->egpio_present))
			val &= ~BIT(g->egpio_enable);
	} else {
		val &= ~mask;
		val |= i << g->mux_bit;
		/* Claim ownership of pin if egpio capable */
		if (egpio_func && val & BIT(g->egpio_present))
			val |= BIT(g->egpio_enable);
	}

	msm_writel_ctl(val, pctrl, g);

	return 0;
}

const struct pinctrl_ops msm_pinctrl_ops_dm = {
	.get_pins_count		= msm_pinctrl_get_pins_count,
	.get_pin_name		= msm_pinctrl_get_pin_name,
	.get_groups_count	= msm_pinctrl_get_groups_count,
	.get_group_name		= msm_pinctrl_get_group_name,
	.get_functions_count	= msm_pinctrl_get_funcs_count,
	.get_function_name	= msm_pinctrl_get_func_name,
	.set_state		= pinctrl_generic_set_state,
#if CONFIG_IS_ENABLED(PINCONF)
	.pinconf_num_params	= ARRAY_SIZE(msm_pinconf_params),
	.pinconf_params		= msm_pinconf_params,
	.pinconf_set		= msm_pinctrl_pinconf_set,
#endif
	.pinmux_set		= msm_pinctrl_set_pinmux,
	.get_pin_muxing 	= msm_pinctrl_get_pin_muxing,
};

static int msm_gpio_get_direction(struct udevice *dev, unsigned int offset)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	const struct msm_pingroup *g;
	u32 val;

	if (msm_is_reserved_pin(pctrl, offset))
		return GPIOF_UNKNOWN;

	g = &pctrl->soc->groups[offset];

	val = msm_readl_ctl(pctrl, g);

	return val & BIT(g->oe_bit) ? GPIOF_OUTPUT : GPIOF_INPUT;
}

static int msm_gpio_direction_input(struct udevice *dev, unsigned offset)
{
	const struct msm_pingroup *g;
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	u32 val;

	if (msm_is_reserved_pin(pctrl, offset))
		return -EACCES;

	g = &pctrl->soc->groups[offset];

	val = msm_readl_ctl(pctrl, g);
	/* clear "output-enable" bit */
	val &= ~BIT(g->oe_bit);
	msm_writel_ctl(val, pctrl, g);

	return 0;
}

static int msm_gpio_direction_output(struct udevice *dev, unsigned offset, int value)
{
	const struct msm_pingroup *g;
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	u32 val;

	if (msm_is_reserved_pin(pctrl, offset))
		return -EACCES;

	g = &pctrl->soc->groups[offset];

	val = msm_readl_io(pctrl, g);
	if (value)
		val |= BIT(g->out_bit);
	else
		val &= ~BIT(g->out_bit);
	msm_writel_io(val, pctrl, g);

	val = msm_readl_ctl(pctrl, g);
	val |= BIT(g->oe_bit);
	msm_writel_ctl(val, pctrl, g);

	return 0;
}

static int msm_gpio_get(struct udevice *dev, unsigned offset)
{
	const struct msm_pingroup *g;
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	u32 val;

	if (msm_is_reserved_pin(pctrl, offset))
		return 0;

	g = &pctrl->soc->groups[offset];

	val = msm_readl_io(pctrl, g);
	return !!(val & BIT(g->in_bit));
}

static int msm_gpio_set(struct udevice *dev, unsigned offset, int value)
{
	const struct msm_pingroup *g;
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	u32 val;

	if (msm_is_reserved_pin(pctrl, offset))
		return -EACCES;

	g = &pctrl->soc->groups[offset];

	val = msm_readl_io(pctrl, g);
	if (value)
		val |= BIT(g->out_bit);
	else
		val &= ~BIT(g->out_bit);
	msm_writel_io(val, pctrl, g);

	return 0;
}

static int msm_gpio_init(struct udevice *pindev)
{
	struct msm_pinctrl *pctrl = dev_get_priv(pindev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(pindev);
	unsigned int ngpio = pctrl->soc->ngpios;
	ofnode node = dev_ofnode(pindev);
	struct ofnode_phandle_args args;
	int prop_size;

	if (ngpio > MAX_NR_GPIO) {
		dev_warn(pindev, "Maximum number of GPIOs exceeded in"
			" pinctrl-msm driver!\n");
		return -EINVAL;
	}

	/*
	 * we could use pctrl->dev->name as bank name, but then it looks
	 * a bit ugly in "gpio status" output:
	 *  Bank pinctrl@3100000:
	 *  pinctrl@31000000 - hard to know that the last "0" is gpio number
	 * let's call it tlmm, then it will look like "tlmm0", "tlmm1", ...
	 */
	uc_priv->bank_name = "tlmm";
	uc_priv->gpio_count = ngpio;

	if (!dev_read_phandle_with_args(pindev, "gpio-ranges",
					NULL, 3, 0, &args)) {
		uc_priv->gpio_count = args.args[2];
	}

	/* parse reserved gpios */
	memset(pctrl->reserved_gpios, 0, sizeof(pctrl->reserved_gpios));
	prop_size = ofnode_read_size(node, "gpio-reserved-ranges");
	if (prop_size != -EINVAL) {
		int array_len = prop_size / sizeof(u32);
		int i;
		u32 start, count;
		for (i = 0; i < array_len; i += 2) {
			ofnode_read_u32_index(node, "gpio-reserved-ranges",
				i, &start);
			ofnode_read_u32_index(node, "gpio-reserved-ranges",
				i + 1, &count);

			if ((start >= ngpio) || (start + count >= ngpio))
				continue;

			bitmap_set(pctrl->reserved_gpios, start, count);
		}
	}

	return 0;
}

static int msm_gpio_probe(struct udevice *dev)
{
	return msm_gpio_init(dev);
}

static const struct dm_gpio_ops msm_gpio_ops = {
	.set_value		= msm_gpio_set,
	.get_value		= msm_gpio_get,
	.get_function		= msm_gpio_get_direction,
	.direction_input	= msm_gpio_direction_input,
	.direction_output	= msm_gpio_direction_output,
};

U_BOOT_DRIVER(msm_gpio) = {
	.name		= "msm-gpio",
	.id		= UCLASS_GPIO,
	.priv_auto	= 0,
	.probe		= msm_gpio_probe,
	.ops		= &msm_gpio_ops,
};

int msm_pinctrl_probe2(struct udevice *dev, const struct msm_pinctrl_soc_data *soc_data)
{
	struct msm_pinctrl *pctrl;
	int ret;
	int i;

	pctrl = devm_kzalloc(dev, sizeof(struct msm_pinctrl), GFP_KERNEL);
	if (!pctrl) {
		log_err("msm_pinctrl_probe2: Failed to allocate memory!\n");
		return -ENOMEM;
	}

	log_debug("msm_pinctrl_probe2: pctrl = %p\n", pctrl);

	pctrl->dev = dev;
	pctrl->soc = soc_data;

	dev_set_priv(dev, pctrl);

	if (soc_data->tiles) {
		/* read base address for each tile */
		for (i = 0; i < soc_data->ntiles; i++) {
			pctrl->regs[i] = dev_remap_addr_name(dev, soc_data->tiles[i]);
		}
	} else {
		/* No tiles, single reg */
		pctrl->regs[0] = dev_read_addr_ptr(dev);
		pctrl->phys_base[0] = dev_read_addr(dev);
	}

	/* bind the gpio part of this driver to the same node */
	ret = device_bind_driver_to_node(dev, "msm-gpio", dev->name,
					dev_ofnode(dev), &pctrl->gpio);

	/* share the priv */
	dev_set_priv(pctrl->gpio, pctrl);

	device_probe(pctrl->gpio);

	dev_dbg(dev, "Probed Qualcomm pinctrl driver\n");
	return 0;
}

int msm_pinctrl_bind(struct udevice *dev)
{
	/* call the default binder, probably not needed at all */
	return dm_scan_fdt_dev(dev);
};
