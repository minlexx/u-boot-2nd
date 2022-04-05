// TODO use git blame and get everyone to allow relicensing as SPDX-License-Identifier: GPL-2.0+
/*
 * Common functions for all qcom pinctrl drivers
 * Ported from Linux
 */

#include <asm/gpio.h>
#include <common.h>
#include <dm.h>
#include <log.h>
#include <dm/pinctrl.h>
#include <regmap.h>
#include <syscon.h>
#include <fdtdec.h>
#include <linux/bitops.h>
#include <linux/libfdt.h>
#include <asm/global_data.h>
#include <asm/io.h>

#include "pinctrl-msm.h"

#define MAX_NR_GPIO 300
#define MAX_NR_TILES 4
#define PS_HOLD_OFFSET 0x820

/**
 * struct msm_pinctrl - state for a pinctrl-msm device
 * @dev:            device handle.
 * @pctrl:          pinctrl handle.
 * @chip:           gpiochip handle.
 * @desc:           pin controller descriptor
 * @restart_nb:     restart notifier block.
 * @irq_chip:       irq chip information
 * @irq:            parent irq for the TLMM irq_chip.
 * @intr_target_use_scm: route irq to application cpu using scm calls
 * @lock:           Spinlock to protect register resources as well
 *                  as msm_pinctrl data structures.
 * @enabled_irqs:   Bitmap of currently enabled irqs.
 * @dual_edge_irqs: Bitmap of irqs that need sw emulated dual edge
 *                  detection.
 * @skip_wake_irqs: Skip IRQs that are handled by wakeup interrupt controller
 * @disabled_for_mux: These IRQs were disabled because we muxed away.
 * @soc:            Reference to soc_data of platform specific data.
 * @regs:           Base addresses for the TLMM tiles.
 * @phys_base:      Physical base address
 */
struct msm_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctrl;
//	struct gpio_chip chip; // no gpio_chip in u-boot
	struct udevice *gpio; // u-boot gpio device
//	struct pinctrl_desc desc; no pinctrl_desc in u-boot

//	DECLARE_BITMAP(disabled_for_mux, MAX_NR_GPIO); // we ignore IRQ stuff for now

	const struct msm_pinctrl_soc_data *soc;
	void /*__iomem*/ *regs[MAX_NR_TILES];
	u32 phys_base[MAX_NR_TILES];
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
MSM_ACCESSOR(intr_cfg)
MSM_ACCESSOR(intr_status)
MSM_ACCESSOR(intr_target)

static void msm_ack_intr_status(struct msm_pinctrl *pctrl,
				const struct msm_pingroup *g)
{
	u32 val = g->intr_ack_high ? BIT(g->intr_status_bit) : 0;

	msm_writel_intr_status(val, pctrl, g);
}

static int msm_get_groups_count(struct udevice *dev)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);

	return pctrl->soc->ngroups;
}

static const char *msm_get_group_name(struct udevice *dev,
				      unsigned group)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);

	return pctrl->soc->groups[group].name;
}

static int msm_get_group_pins(struct udevice *dev,
			      unsigned group,
			      const unsigned **pins,
			      unsigned *num_pins)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);

	*pins = pctrl->soc->groups[group].pins;
	*num_pins = pctrl->soc->groups[group].npins;
	return 0;
}

static int msm_pinctrl_set_state(struct udevice *dev,
				struct udevice *config)
{
	struct msm_pinctrl *priv = dev_get_priv(dev);
	return 0;
}

/*static const struct pinctrl_ops msm_pinctrl_ops = {
	.get_groups_count	= msm_get_groups_count,
	.get_group_name		= msm_get_group_name,
	.get_group_pins		= msm_get_group_pins,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_group,
	.dt_free_map		= pinctrl_utils_free_map,
};*/
/* U-Boot version of ops */
const struct pinctrl_ops msm_pinctrl_ops = {
	.set_state			= msm_pinctrl_set_state,
	//.get_gpio_mux			= rockchip_pinctrl_get_gpio_mux,
};

static int msm_pinmux_request(struct udevice *dev, unsigned offset)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);

	return gpiochip_line_is_valid(chip, offset) ? 0 : -EINVAL;
}

static int msm_get_functions_count(struct udevice *dev)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);

	return pctrl->soc->nfunctions;
}

static const char *msm_get_function_name(struct udevice *dev,
					 unsigned function)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);

	return pctrl->soc->functions[function].name;
}

static int msm_get_function_groups(struct udevice *dev,
				   unsigned function,
				   const char * const **groups,
				   unsigned * const num_groups)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);

	*groups = pctrl->soc->functions[function].groups;
	*num_groups = pctrl->soc->functions[function].ngroups;
	return 0;
}

static int msm_pinmux_set_mux(struct udevice *dev,
			      unsigned function,
			      unsigned group)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	struct gpio_chip *gc = &pctrl->gpio;
	unsigned int irq = irq_find_mapping(gc->irq.domain, group);
	struct irq_data *d = irq_get_irq_data(irq);
	unsigned int gpio_func = pctrl->soc->gpio_func;
	unsigned int egpio_func = pctrl->soc->egpio_func;
	const struct msm_pingroup *g;
	unsigned long flags;
	u32 val, mask;
	int i;

	g = &pctrl->soc->groups[group];
	mask = GENMASK(g->mux_bit + order_base_2(g->nfuncs) - 1, g->mux_bit);

	for (i = 0; i < g->nfuncs; i++) {
		if (g->funcs[i] == function)
			break;
	}

	if (WARN_ON(i == g->nfuncs))
		return -EINVAL;

	/*
	 * If an GPIO interrupt is setup on this pin then we need special
	 * handling.  Specifically interrupt detection logic will still see
	 * the pin twiddle even when we're muxed away.
	 *
	 * When we see a pin with an interrupt setup on it then we'll disable
	 * (mask) interrupts on it when we mux away until we mux back.  Note
	 * that disable_irq() refcounts and interrupts are disabled as long as
	 * at least one disable_irq() has been called.
	 */
	if (d && i != gpio_func &&
	    !test_and_set_bit(d->hwirq, pctrl->disabled_for_mux))
		disable_irq(irq);

//	raw_spin_lock_irqsave(&pctrl->lock, flags);

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

//	raw_spin_unlock_irqrestore(&pctrl->lock, flags);

	if (d && i == gpio_func &&
	    test_and_clear_bit(d->hwirq, pctrl->disabled_for_mux)) {
		/*
		 * Clear interrupts detected while not GPIO since we only
		 * masked things.
		 */
		if (d->parent_data && test_bit(d->hwirq, pctrl->skip_wake_irqs))
			irq_chip_set_parent_state(d, IRQCHIP_STATE_PENDING, false);
		else
			msm_ack_intr_status(pctrl, g);

		enable_irq(irq);
	}

	return 0;
}

static int msm_pinmux_request_gpio(struct udevice *dev,
				   struct pinctrl_gpio_range *range,
				   unsigned offset)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	const struct msm_pingroup *g = &pctrl->soc->groups[offset];

	/* No funcs? Probably ACPI so can't do anything here */
	if (!g->nfuncs)
		return 0;

	return msm_pinmux_set_mux(pctldev, g->funcs[pctrl->soc->gpio_func], offset);
}
/*
static const struct pinmux_ops msm_pinmux_ops = {
	.request		= msm_pinmux_request,
	.get_functions_count	= msm_get_functions_count,
	.get_function_name	= msm_get_function_name,
	.get_function_groups	= msm_get_function_groups,
	.gpio_request_enable	= msm_pinmux_request_gpio,
	.set_mux		= msm_pinmux_set_mux,
};
*/
static int msm_config_reg(struct msm_pinctrl *pctrl,
			  const struct msm_pingroup *g,
			  unsigned param,
			  unsigned *mask,
			  unsigned *bit)
{
	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_BUS_HOLD:
	case PIN_CONFIG_BIAS_PULL_UP:
		*bit = g->pull_bit;
		*mask = 3;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		*bit = g->od_bit;
		*mask = 1;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		*bit = g->drv_bit;
		*mask = 7;
		break;
	case PIN_CONFIG_OUTPUT:
	case PIN_CONFIG_INPUT_ENABLE:
		*bit = g->oe_bit;
		*mask = 1;
		break;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

#define MSM_NO_PULL		0
#define MSM_PULL_DOWN		1
#define MSM_KEEPER		2
#define MSM_PULL_UP_NO_KEEPER	2
#define MSM_PULL_UP		3

static unsigned msm_regval_to_drive(u32 val)
{
	return (val + 1) * 2;
}

static int msm_config_group_get(struct udevice *dev,
				unsigned int group,
				unsigned long *config)
{
	const struct msm_pingroup *g;
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	unsigned param = pinconf_to_config_param(*config);
	unsigned mask;
	unsigned arg;
	unsigned bit;
	int ret;
	u32 val;

	g = &pctrl->soc->groups[group];

	ret = msm_config_reg(pctrl, g, param, &mask, &bit);
	if (ret < 0)
		return ret;

	val = msm_readl_ctl(pctrl, g);
	arg = (val >> bit) & mask;

	/* Convert register value to pinconf value */
	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		if (arg != MSM_NO_PULL)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (arg != MSM_PULL_DOWN)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_BUS_HOLD:
		if (pctrl->soc->pull_no_keeper)
			return -ENOTSUPP;

		if (arg != MSM_KEEPER)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (pctrl->soc->pull_no_keeper)
			arg = arg == MSM_PULL_UP_NO_KEEPER;
		else
			arg = arg == MSM_PULL_UP;
		if (!arg)
			return -EINVAL;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		/* Pin is not open-drain */
		if (!arg)
			return -EINVAL;
		arg = 1;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		arg = msm_regval_to_drive(arg);
		break;
	case PIN_CONFIG_OUTPUT:
		/* Pin is not output */
		if (!arg)
			return -EINVAL;

		val = msm_readl_io(pctrl, g);
		arg = !!(val & BIT(g->in_bit));
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		/* Pin is output */
		if (arg)
			return -EINVAL;
		arg = 1;
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int msm_config_group_set(struct udevice *dev,
				unsigned group,
				unsigned long *configs,
				unsigned num_configs)
{
	const struct msm_pingroup *g;
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	unsigned long flags;
	unsigned param;
	unsigned mask;
	unsigned arg;
	unsigned bit;
	int ret;
	u32 val;
	int i;

	g = &pctrl->soc->groups[group];

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		ret = msm_config_reg(pctrl, g, param, &mask, &bit);
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
				arg = (arg / 2) - 1;
			break;
		case PIN_CONFIG_OUTPUT:
			/* set output value */
//			raw_spin_lock_irqsave(&pctrl->lock, flags);
			val = msm_readl_io(pctrl, g);
			if (arg)
				val |= BIT(g->out_bit);
			else
				val &= ~BIT(g->out_bit);
			msm_writel_io(val, pctrl, g);
//			raw_spin_unlock_irqrestore(&pctrl->lock, flags);

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

//		raw_spin_lock_irqsave(&pctrl->lock, flags);
		val = msm_readl_ctl(pctrl, g);
		val &= ~(mask << bit);
		val |= arg << bit;
		msm_writel_ctl(val, pctrl, g);
//		raw_spin_unlock_irqrestore(&pctrl->lock, flags);
	}

	return 0;
}

/*static const struct pinconf_ops msm_pinconf_ops = {
	.is_generic		= true,
	.pin_config_group_get	= msm_config_group_get,
	.pin_config_group_set	= msm_config_group_set,
};*/

static int msm_gpio_direction_input(struct udevice *dev, unsigned offset)
{
	const struct msm_pingroup *g;
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	unsigned long flags;
	u32 val;

	g = &pctrl->soc->groups[offset];

//	raw_spin_lock_irqsave(&pctrl->lock, flags);

	val = msm_readl_ctl(pctrl, g);
	val &= ~BIT(g->oe_bit);
	msm_writel_ctl(val, pctrl, g);

//	raw_spin_unlock_irqrestore(&pctrl->lock, flags);

	return 0;
}

static int msm_gpio_direction_output(struct udevice *dev, unsigned offset, int value)
{
	const struct msm_pingroup *g;
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	unsigned long flags;
	u32 val;

	g = &pctrl->soc->groups[offset];

//	raw_spin_lock_irqsave(&pctrl->lock, flags);

	val = msm_readl_io(pctrl, g);
	if (value)
		val |= BIT(g->out_bit);
	else
		val &= ~BIT(g->out_bit);
	msm_writel_io(val, pctrl, g);

	val = msm_readl_ctl(pctrl, g);
	val |= BIT(g->oe_bit);
	msm_writel_ctl(val, pctrl, g);

//	raw_spin_unlock_irqrestore(&pctrl->lock, flags);

	return 0;
}

static int msm_gpio_get_direction(struct udevice *dev, unsigned int offset)
{
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	const struct msm_pingroup *g;
	u32 val;

	g = &pctrl->soc->groups[offset];

	val = msm_readl_ctl(pctrl, g);

	return val & BIT(g->oe_bit) ? GPIO_LINE_DIRECTION_OUT :
				      GPIO_LINE_DIRECTION_IN;
}

static int msm_gpio_get(struct udevice *dev, unsigned offset)
{
	const struct msm_pingroup *g;
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	u32 val;

	g = &pctrl->soc->groups[offset];

	val = msm_readl_io(pctrl, g);
	return !!(val & BIT(g->in_bit));
}

static void msm_gpio_set(struct udevice *dev, unsigned offset, int value)
{
	const struct msm_pingroup *g;
	struct msm_pinctrl *pctrl = dev_get_priv(dev);
	unsigned long flags;
	u32 val;

	g = &pctrl->soc->groups[offset];

//	raw_spin_lock_irqsave(&pctrl->lock, flags);

	val = msm_readl_io(pctrl, g);
	if (value)
		val |= BIT(g->out_bit);
	else
		val &= ~BIT(g->out_bit);
	msm_writel_io(val, pctrl, g);

//	raw_spin_unlock_irqrestore(&pctrl->lock, flags);
}

// TODO add back IRQ controller support
static int msm_gpio_init(struct msm_pinctrl *pctrl)
{
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	int ret;
	unsigned int ngpio = pctrl->soc->ngpios;
	struct ofnode_phandle_args args;

	/*if (WARN_ON(ngpio > MAX_NR_GPIO))
		return -EINVAL;*/

	uc_priv->bank_name = pctrl->dev->name;
	uc_priv->gpio_count = ngpio;


	if (!dev_read_phandle_with_args(dev, "gpio-ranges",
					NULL, 3, 0, &args)) {
		uc_priv->gpio_count = args.args[2];
	}

	return 0;
}

int msm_pinctrl_probe(struct udevice *dev, const struct msm_pinctrl_soc_data *soc_data)
{
	struct msm_pinctrl *pctrl;
	struct resource *res;
	int ret;
	int i;

	pctrl->dev = dev;
	pctrl->soc = soc_data;

//	raw_spin_lock_init(&pctrl->lock);

	if (soc_data->tiles) {
		for (i = 0; i < soc_data->ntiles; i++) {
			pctrl->regs[i] = dev_remap_addr_name(dev, soc_data->tiles[i]);
		}
	} else {
		pctrl->regs[0] = dev_read_addr_ptr(dev);
		pctrl->phys_base[0] = dev_read_addr(dev);
	}
/*
	pctrl->pctrl = devm_pinctrl_register(&pdev->dev, &pctrl->desc, pctrl);
	if (IS_ERR(pctrl->pctrl)) {
		dev_err(&pdev->dev, "Couldn't register pinctrl driver\n");
		return PTR_ERR(pctrl->pctrl);
	}
*/
	ret = msm_gpio_init(pctrl);
	if (ret)
		return ret;

	device_probe(pctrl->gpio);

	dev_dbg(&dev, "Probed Qualcomm pinctrl driver\n");

	return 0;
}
/*
int msm_pinctrl_remove(struct udevice *udev)
{
	struct msm_pinctrl *pctrl = platform_get_drvdata(pdev);
	// gpiochip_remove(&pctrl->gpio);
	return 0;
}
*/

//TODO: test these, adapt msm_config_group_get/set
static const struct dm_gpio_ops msm_gpio_ops = {
	.set_value = msm_gpio_set,
	.get_value = msm_gpio_get,
//	.get_function = msm_gpio_get_function,
	.direction_input = msm_gpio_direction_input,
	.direction_output = msm_gpio_direction_output,
//	.set_flags = msm_gpio_set_flags, //msm_config_group_set
//	.get_flags = msm_gpio_get_flags, //msm_config_group_get
};

U_BOOT_DRIVER(msm_gpio) = {
	.name	= "msm-gpio",
	.id	= UCLASS_GPIO,
	.ops	= &msm_gpio_ops,
};
