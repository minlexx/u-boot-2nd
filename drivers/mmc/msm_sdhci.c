// SPDX-License-Identifier: GPL-2.0+
/*
 * Qualcomm SDHCI driver - SD/eMMC controller
 *
 * (C) Copyright 2015 Mateusz Kulikowski <mateusz.kulikowski@gmail.com>
 *
 * Based on Linux driver
 */

#include "linux/errno.h"
#define DEBUG

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <malloc.h>
#include <sdhci.h>
#include <wait_bit.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <log.h>

DECLARE_GLOBAL_DATA_PTR;

/* Non-standard registers needed for SDHCI startup */
#define SDCC_MCI_POWER			0x0
#define SDCC_MCI_POWER_SW_RST		BIT(7)

/* Used to parse version info from core_mci_version register */
#define CORE_VERSION_MAJOR_SHIFT	28
#define CORE_VERSION_MAJOR_MASK		(0xf << CORE_VERSION_MAJOR_SHIFT)
#define CORE_VERSION_MINOR_MASK		0xff

#define SDCC_MCI_STATUS2		0x6C
#define SDCC_MCI_STATUS2_MCI_ACT	0x1
#define SDCC_MCI_HC_MODE		0x78

#define CORE_VENDOR_SPEC_POR_VAL	0xa9c  /* mainline linux value */
#define CORE_VENDOR_SPEC_POR_VAL_DS	0xa1c  /* the value used in downstream driver */

/* values used in PWRCTL regs */
#define CORE_PWRCTL_BUS_OFF		0x1
#define CORE_PWRCTL_BUS_ON		BIT(1)
#define CORE_PWRCTL_IO_LOW		BIT(2)
#define CORE_PWRCTL_IO_HIGH		BIT(3)

#define CORE_PWRCTL_BUS_SUCCESS		0x1
#define CORE_PWRCTL_BUS_FAIL		BIT(1)
#define CORE_PWRCTL_IO_SUCCESS		BIT(2)
#define CORE_PWRCTL_IO_FAIL		BIT(3)

/*
 * From V5, register spaces have changed. Wrap this info in a structure
 * and choose the data_structure based on version info mentioned in DT.
 */

struct sdhci_msm_offset {
	u32 core_hc_mode;
	u32 core_mci_data_cnt;
	u32 core_mci_status;
	u32 core_mci_fifo_cnt;
	u32 core_mci_version;
	u32 core_generics;
	u32 core_testbus_config;
	u32 core_testbus_sel2_bit;
	u32 core_testbus_ena;
	u32 core_testbus_sel2;
	u32 core_pwrctl_status;
	u32 core_pwrctl_mask;
	u32 core_pwrctl_clear;
	u32 core_pwrctl_ctl;
	u32 core_sdcc_debug_reg;
	u32 core_dll_config;
	u32 core_dll_status;
	u32 core_vendor_spec;
	u32 core_vendor_spec_adma_err_addr0;
	u32 core_vendor_spec_adma_err_addr1;
	u32 core_vendor_spec_func2;
	u32 core_vendor_spec_capabilities0;
	u32 core_ddr_200_cfg;
	u32 core_vendor_spec3;
	u32 core_dll_config_2;
	u32 core_dll_config_3;
	u32 core_ddr_config_old; /* Applicable to sdcc minor ver < 0x49 */
	u32 core_ddr_config;
	u32 core_dll_usr_ctl; /* Present on SDCC5.1 onwards */
};

static const struct sdhci_msm_offset sdhci_msm_v5_offset = {
	.core_mci_data_cnt = 0x35c,
	.core_mci_status = 0x324,
	.core_mci_fifo_cnt = 0x308,
	.core_mci_version = 0x318,
	.core_generics = 0x320,
	.core_testbus_config = 0x32c,
	.core_testbus_sel2_bit = 3,
	.core_testbus_ena = (1 << 31),
	.core_testbus_sel2 = (1 << 3),
	.core_pwrctl_status = 0x240,
	.core_pwrctl_mask = 0x244,
	.core_pwrctl_clear = 0x248,
	.core_pwrctl_ctl = 0x24c,
	.core_sdcc_debug_reg = 0x358,
	.core_dll_config = 0x200,
	.core_dll_status = 0x208,
	.core_vendor_spec = 0x20c,
	.core_vendor_spec_adma_err_addr0 = 0x214,
	.core_vendor_spec_adma_err_addr1 = 0x218,
	.core_vendor_spec_func2 = 0x210,
	.core_vendor_spec_capabilities0 = 0x21c,
	.core_ddr_200_cfg = 0x224,
	.core_vendor_spec3 = 0x250,
	.core_dll_config_2 = 0x254,
	.core_dll_config_3 = 0x258,
	.core_ddr_config = 0x25c,
	.core_dll_usr_ctl = 0x388,
};

static const struct sdhci_msm_offset sdhci_msm_mci_offset = {
	.core_hc_mode = 0x78,
	.core_mci_data_cnt = 0x30,
	.core_mci_status = 0x34,
	.core_mci_fifo_cnt = 0x44,
	.core_mci_version = 0x50,
	.core_generics = 0x70,
	.core_testbus_config = 0x0cc,
	.core_testbus_sel2_bit = 4,
	.core_testbus_ena = (1 << 3),
	.core_testbus_sel2 = (1 << 4),
	.core_pwrctl_status = 0xdc,
	.core_pwrctl_mask = 0xe0,
	.core_pwrctl_clear = 0xe4,
	.core_pwrctl_ctl = 0xe8,
	.core_sdcc_debug_reg = 0x124,
	.core_dll_config = 0x100,
	.core_dll_status = 0x108,
	.core_vendor_spec = 0x10c,
	.core_vendor_spec_adma_err_addr0 = 0x114,
	.core_vendor_spec_adma_err_addr1 = 0x118,
	.core_vendor_spec_func2 = 0x110,
	.core_vendor_spec_capabilities0 = 0x11c,
	.core_ddr_200_cfg = 0x184,
	.core_vendor_spec3 = 0x1b0,
	.core_dll_config_2 = 0x1b4,
	.core_ddr_config_old = 0x1b8,
	.core_ddr_config = 0x1bc,
};

struct msm_sdhc;

struct sdhci_msm_variant_ops {
	u32 (*msm_readl_relaxed)(struct msm_sdhc *host, u32 offset);
	void (*msm_writel_relaxed)(struct msm_sdhc *host, u32 val, u32 offset);
};

struct sdhci_msm_variant_info {
	bool mci_removed;
	bool restore_dll_config;
	const struct sdhci_msm_variant_ops *var_ops;
	const struct sdhci_msm_offset *offset;
};

struct msm_sdhc_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct msm_sdhc {
	struct sdhci_host host;
	void *core_mem;

	bool mci_removed;
	bool restore_dll_config;
	const struct sdhci_msm_variant_ops *var_ops;
	const struct sdhci_msm_offset *offset;
};

inline struct msm_sdhc *to_msm_sdhc(struct sdhci_host *ptr)
{
	return container_of(ptr, struct msm_sdhc, host);
}

/*
 * APIs to read/write to vendor specific registers which were there in the
 * core_mem region before MCI was removed.
 */
static u32 sdhci_msm_mci_readl_relaxed(struct msm_sdhc *msmhost, u32 offset)
{
	return readl_relaxed(msmhost->core_mem + offset);
}

static u32 sdhci_msm_v5_readl_relaxed(struct msm_sdhc *msmhost, u32 offset)
{
	return readl_relaxed(msmhost->host.ioaddr + offset);
}

static void sdhci_msm_mci_writel_relaxed(struct msm_sdhc *msmhost, u32 val, u32 offset)
{
	writel_relaxed(val, msmhost->core_mem + offset);
}

static void sdhci_msm_v5_writel_relaxed(struct msm_sdhc *msmhost, u32 val, u32 offset)
{
	writel_relaxed(val, msmhost->host.ioaddr + offset);
}

static u32 msm_host_readl(struct msm_sdhc *msmhost, u32 offset)
{
	return msmhost->var_ops->msm_readl_relaxed(msmhost, offset);
}

static void msm_host_writel(struct msm_sdhc *msmhost, u32 val, u32 offset)
{
	msmhost->var_ops->msm_writel_relaxed(msmhost, val, offset);
}

static void sdhci_msm_dump_pwr_ctrl_regs(struct msm_sdhc *msmhost)
{
	const struct sdhci_msm_offset *regs = msmhost->offset;

	pr_err("PWRCTL: STATUS: 0x%08x | MASK: 0x%08x | CTL: 0x%08x\n",
		msm_host_readl(msmhost, regs->core_pwrctl_status),
		msm_host_readl(msmhost, regs->core_pwrctl_mask),
		msm_host_readl(msmhost, regs->core_pwrctl_ctl));
	// on start:
	// PWRCTL: STATUS: 0x00000000 | MASK: 0x00000000 | CTL: 0x00000001
}

static int msm_sdc_clk_init(struct udevice *dev)
{
	int node = dev_of_offset(dev);
	uint clk_rate = fdtdec_get_uint(gd->fdt_blob, node, "clock-frequency",
					400000);
	uint clkd[2]; /* clk_id and clk_no */
	int clk_offset;
	struct udevice *clk_dev;
	struct clk clk;
	int ret;

	ret = fdtdec_get_int_array(gd->fdt_blob, node, "clock", clkd, 2);
	if (ret) {
		if (ret == -FDT_ERR_NOTFOUND) {
			log_warning("%s: no clock property in fdt!\n", __func__);
			return 0;
		}
		return ret;
	}

	clk_offset = fdt_node_offset_by_phandle(gd->fdt_blob, clkd[0]);
	if (clk_offset < 0)
		return clk_offset;

	ret = uclass_get_device_by_of_offset(UCLASS_CLK, clk_offset, &clk_dev);
	if (ret)
		return ret;

	clk.id = clkd[1];
	ret = clk_request(clk_dev, &clk);
	if (ret < 0)
		return ret;

	ret = clk_set_rate(&clk, clk_rate);
	clk_free(&clk);
	if (ret < 0)
		return ret;

	return 0;
}

#define dump_gcc_reg(reg) \
	tmp = readl(SDM660_GCC_BASE + reg); \
	log_debug(" - %s: %x\n", #reg, tmp);

static void dump_sdcc1_clk_regs(void)
{
	u32 tmp;
	dump_gcc_reg(GCC_SDCC1_BCR);
	dump_gcc_reg(GCC_SDCC1_APPS_CBCR);
	dump_gcc_reg(GCC_SDCC1_AHB_CBCR);
	dump_gcc_reg(GCC_SDCC1_APPS_CMD_RCGR);
	dump_gcc_reg(GCC_SDCC1_APPS_CFG_RCGR);
	dump_gcc_reg(GCC_SDCC1_APPS_M);
	dump_gcc_reg(GCC_SDCC1_APPS_N);
	dump_gcc_reg(GCC_SDCC1_APPS_D);
}

static void hacky_sdcc_clocks_init(void)
{
	log_debug("hacky_sdcc_clocks_init start\n");
	
	dump_sdcc1_clk_regs();
/*- GCC_SDCC1_APPS_CBCR: 4221
  - GCC_SDCC1_AHB_CBCR: 20008001
  - GCC_SDCC1_CMD_RCGR: 80000000
  - GCC_SDCC1_CFG_RCGR: 0
  - GCC_SDCC1_M: 0
  - GCC_SDCC1_N: 0
  - GCC_SDCC1_D: 0 */

	log_debug("hacky_sdcc_clocks_init: doing HW reset\n");

	/*
	 * Reset GCC_SDCC_BCR register before every fresh initilazation.
	 * This will reset whole SDHC-msm controller, clears the previous
	 * power control states and avoids software reset timeout issues.
	 */
	writel(1, SDM660_GCC_BASE + GCC_SDCC1_BCR);
	/*
	 * The hardware requirement for delay between assert/deassert
	 * is at least 3-4 sleep clock (32.7KHz) cycles, which comes to
	 * ~125us (4/32768). To be on the safe side add 200us delay.
	 */
	udelay(200);
	//readl(SDM660_GCC_BASE + GCC_SDCC1_BCR);
	/* deassert */
	writel(0, SDM660_GCC_BASE + GCC_SDCC1_BCR);

	log_debug("hacky_sdcc_clocks_init after HW reset\n");
	
	dump_sdcc1_clk_regs();

	log_debug("hacky_sdcc_clocks_init end\n");
}

static int msm_sdc_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct msm_sdhc_plat *plat = dev_get_plat(dev);
	struct msm_sdhc *prv = dev_get_priv(dev);
	const struct sdhci_msm_offset *regs = prv->offset;
	struct sdhci_host *host = &prv->host;
	u16 host_version;
	u32 core_version, core_minor, core_major, caps, tmp;
	int ret;

	host->quirks = SDHCI_QUIRK_WAIT_SEND_CMD | SDHCI_QUIRK_BROKEN_R1B;

	host->max_clk = 0;

	/* Init clocks */
	ret = msm_sdc_clk_init(dev);
	if (ret) {
		log_warning("msm_sdhci: failed to init clocks: %d\n", ret);
		return ret;
	}

	hacky_sdcc_clocks_init();

	if (!prv->mci_removed) {
		/* Old v4 code path */
		/* Reset the core and Enable SDHC mode */
		writel(readl(prv->core_mem + SDCC_MCI_POWER) | SDCC_MCI_POWER_SW_RST,
			prv->core_mem + SDCC_MCI_POWER);


		/* Wait for reset to be written to register */
		if (wait_for_bit_le32(prv->core_mem + SDCC_MCI_STATUS2,
				SDCC_MCI_STATUS2_MCI_ACT, false, 10, false)) {
			printf("msm_sdhci: reset request failed\n");
			return -EIO;
		}

		/* SW reset can take upto 10HCLK + 15MCLK cycles. (min 40us) */
		if (wait_for_bit_le32(prv->core_mem + SDCC_MCI_POWER,
				SDCC_MCI_POWER_SW_RST, false, 2, false)) {
			printf("msm_sdhci: stuck in reset\n");
			return -ETIMEDOUT;
		}

		/* Enable host-controller mode */
		writel(1, prv->core_mem + SDCC_MCI_HC_MODE);
	} else {
		log_debug("msm_sdhci: new sdhci-msm-v5 mode\n");
		log_debug("msm_sdhci: ioaddr = %p\n", host->ioaddr);

		tmp = msm_host_readl(prv, regs->core_vendor_spec);
		log_debug("msm_sdhci: CORE_VENDOR_SPEC value from bootloader = %x\n", tmp);
		// msm_sdhci: CORE_VENDOR_SPEC value = 240a0e

		/* Reset the vendor spec register to power on reset state */
		//msm_host_writel(prv, CORE_VENDOR_SPEC_POR_VAL, regs->core_vendor_spec);
		msm_host_writel(prv, CORE_VENDOR_SPEC_POR_VAL_DS, regs->core_vendor_spec);
		tmp = msm_host_readl(prv, regs->core_vendor_spec);
		log_debug("msm_sdhci: CORE_VENDOR_SPEC new value = %x\n", tmp);
	}

	host_version = readw_relaxed((host->ioaddr + SDHCI_HOST_VERSION));
	log_debug("msm_sdhci: Host Version: 0x%x; Vendor Version: 0x%x\n",
		host_version, ((host_version & SDHCI_VENDOR_VER_MASK) >>
		SDHCI_VENDOR_VER_SHIFT));

	core_version = msm_host_readl(prv, regs->core_mci_version);
	core_major = (core_version & CORE_VERSION_MAJOR_MASK) >>
			CORE_VERSION_MAJOR_SHIFT;
	core_minor = core_version & CORE_VERSION_MINOR_MASK;
	log_debug("msm_sdhci: MCI Version: 0x%08x, major: 0x%04x, minor: 0x%02x\n",
		core_version, core_major, core_minor);

	// this commented below came from linux driver
	// if (core_major == 1 && core_minor >= 0x42)
	// 	prv->use_14lpp_dll_reset = true;

	// /*
	//  * SDCC 5 controller with major version 1, minor version 0x34 and later
	//  * with HS 400 mode support will use CM DLL instead of CDC LP 533 DLL.
	//  */
	// if (core_major == 1 && core_minor < 0x34)
	// 	prv->use_cdclp533 = true;

	/*
	 * Support for some capabilities is not advertised by newer
	 * controller versions and must be explicitly enabled.
	 */
	if (core_major >= 1 && core_minor != 0x11 && core_minor != 0x12) {
		caps = readl(host->ioaddr + SDHCI_CAPABILITIES);
		caps |= SDHCI_CAN_VDD_300 | SDHCI_CAN_DO_8BIT;

		log_debug("msm_sdhci: writing %x to VENDOR_SPEC_CAPABILITIES0\n", caps);
		writel_relaxed(caps, host->ioaddr +
				regs->core_vendor_spec_capabilities0);
		// msm_sdhci: writing 762dc8b2 to VENDOR_SPEC_CAPABILITIES0
		// sdhci_setup_cfg, caps: 0x762dc8b2
	}

	sdhci_msm_dump_pwr_ctrl_regs(prv);

	/*
	 * From downstream sdhci-msm driver:
	 * Set the PAD_PWR_SWTICH_EN bit so that the PAD_PWR_SWITCH bit can
	 * be used as required later on.
	 */
	//writel_relaxed((readl_relaxed(host->ioaddr +
	//		msm_host_offset->CORE_VENDOR_SPEC) |
	//		CORE_IO_PAD_PWR_SWITCH_EN), host->ioaddr +
	//		msm_host_offset->CORE_VENDOR_SPEC);

	ret = mmc_of_parse(dev, &plat->cfg);
	if (ret)
		return ret;

	host->mmc = &plat->mmc;
	host->mmc->dev = dev;
	ret = sdhci_setup_cfg(&plat->cfg, host, 0, 0);
	if (ret)
		return ret;
	host->mmc->priv = &prv->host;
	upriv->mmc = host->mmc;

	return sdhci_probe(dev);
}

static int msm_sdc_remove(struct udevice *dev)
{
	struct msm_sdhc *priv = dev_get_priv(dev);

	 /* Disable host-controller mode */
	writel(0, priv->core_mem + SDCC_MCI_HC_MODE);

	return 0;
}

static int msm_of_to_plat(struct udevice *dev)
{
	struct udevice *parent = dev->parent;
	struct msm_sdhc *priv = dev_get_priv(dev);
	struct sdhci_host *host = &priv->host;
	const struct sdhci_msm_variant_info *match_data =
		(const struct sdhci_msm_variant_info *)dev_get_driver_data(dev);
	int node = dev_of_offset(dev);

	priv->mci_removed = match_data->mci_removed;
	priv->restore_dll_config = match_data->restore_dll_config;
	priv->var_ops = match_data->var_ops;
	priv->offset = match_data->offset;

	host->name = strdup(dev->name);
	host->ioaddr = dev_read_addr_ptr(dev);
	host->bus_width = fdtdec_get_int(gd->fdt_blob, node, "bus-width", 4);
	host->index = fdtdec_get_uint(gd->fdt_blob, node, "index", 0);

	/* Read 2nd value from reg - core_mem. This is valid only for v4 */
	if (!priv->mci_removed) {
		priv->core_mem = (void *)fdtdec_get_addr_size_auto_parent(gd->fdt_blob,
				dev_of_offset(parent), node, "reg", 1, NULL, false);
		if (priv->core_mem == (void *)FDT_ADDR_T_NONE) {
			dev_warn(dev, "No core_mem reg for MSM SDHCI host!\n");
			return -EINVAL;
		}
	}

	if (host->ioaddr == (void *)FDT_ADDR_T_NONE) {
		dev_warn(dev, "No ioaddr reg for SDHCI host!\n");
		return -EINVAL;
	}

	return 0;
}

static int msm_sdc_bind(struct udevice *dev)
{
	struct msm_sdhc_plat *plat = dev_get_plat(dev);

	return sdhci_bind(dev, &plat->mmc, &plat->cfg);
}

static const struct sdhci_msm_variant_ops mci_var_ops = {
	.msm_readl_relaxed = sdhci_msm_mci_readl_relaxed,
	.msm_writel_relaxed = sdhci_msm_mci_writel_relaxed,
};

static const struct sdhci_msm_variant_ops v5_var_ops = {
	.msm_readl_relaxed = sdhci_msm_v5_readl_relaxed,
	.msm_writel_relaxed = sdhci_msm_v5_writel_relaxed,
};

static const struct sdhci_msm_variant_info sdhci_msm_mci_var = {
	.var_ops = &mci_var_ops,
	.offset = &sdhci_msm_mci_offset,
};

static const struct sdhci_msm_variant_info sdhci_msm_v5_var = {
	.mci_removed = true,
	.var_ops = &v5_var_ops,
	.offset = &sdhci_msm_v5_offset,
};

static const struct udevice_id sdhci_msm_dt_match[] = {
	{.compatible = "qcom,sdhci-msm-v4", .data = (ulong)&sdhci_msm_mci_var},
	{.compatible = "qcom,sdhci-msm-v5", .data = (ulong)&sdhci_msm_v5_var},
	{},
};

U_BOOT_DRIVER(msm_sdc_drv) = {
	.name		= "msm_sdc",
	.id		= UCLASS_MMC,
	.of_match	= sdhci_msm_dt_match,
	.of_to_plat	= msm_of_to_plat,
	.ops		= &sdhci_ops,
	.bind		= msm_sdc_bind,
	.probe		= msm_sdc_probe,
	.remove		= msm_sdc_remove,
	.priv_auto	= sizeof(struct msm_sdhc),
	.plat_auto	= sizeof(struct msm_sdhc_plat),
};
