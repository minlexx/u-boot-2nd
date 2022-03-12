// SPDX-License-Identifier: BSD-3-Clause
/*
 * Clock drivers for Qualcomm SDM660
 *
 * (C) Copyright 2017 Jorge Ramirez Ortiz <jorge.ramirez-ortiz@linaro.org>
 * (C) Copyright 2022 Alexey Minnekhanov <alexeymin@postmarketos.org>
 *
 * Based on Little Kernel driver, simplified
 */

#include <common.h>
#include <clk-uclass.h>
#include <dm.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include "clock-snapdragon.h"
#include "log.h"

#include <dt-bindings/clock/qcom,gcc-sdm660.h>

//enum {
//	P_XO = 0,
//	P_SLEEP_CLK = 0,
//	P_GPLL0 = 1,
//	P_GPLL1 = 2,
//	P_GPLL4 = 5,
//	P_GPLL0_EARLY_DIV = 6,
//	P_GPLL1_EARLY_DIV = 6,
//};

//struct parent_map {
//	u8 src;
//	u8 cfg;
//};
//
//static const struct parent_map gcc_parent_map_xo_gpll0_gpll4_gpll0_early_div[] = {
//	{ P_XO, 0 },
//	{ P_GPLL0, 1 },
//	{ P_GPLL4, 5 },
//	{ P_GPLL0_EARLY_DIV, 6 },
//};

//struct freq_tbl {
//	uint freq;
//	uint src;
//	u8 pre_div;
//	u16 m;
//	u16 n;
//};
//
//#define FREQ(freq, src, h, m, n) { (freq), (src), (2 * (h) - 1), (m), (n) }
//
//static const struct freq_tbl ftbl_sdcc1_apps_clk_src[] = {
//	FREQ(144000, P_XO, 16, 3, 25),
//	FREQ(400000, P_XO, 12, 1, 4),
//	FREQ(20000000, P_GPLL0_EARLY_DIV, 5, 1, 3),
//	FREQ(25000000, P_GPLL0_EARLY_DIV, 6, 1, 2),
//	FREQ(50000000, P_GPLL0_EARLY_DIV, 6, 0, 0),
//	FREQ(100000000, P_GPLL0, 6, 0, 0),
//	FREQ(192000000, P_GPLL4, 8, 0, 0),
//	FREQ(384000000, P_GPLL4, 4, 0, 0),
//	{ }
//};

//const struct freq_tbl *qcom_find_freq(const struct freq_tbl *f, uint rate)
//{
//	if (!f)
//		return NULL;
//
//	if (!f->freq)
//		return f;
//
//	for (; f->freq; f++)
//		if (rate <= f->freq)
//			return f;
//
//	/* Default to our fastest rate */
//	return f - 1;
//}

static const struct bcr_regs sdc1_regs = {
	.cfg_rcgr = GCC_SDCC1_APPS_CFG_RCGR,
	.cmd_rcgr = GCC_SDCC1_APPS_CMD_RCGR,
	.M = GCC_SDCC1_APPS_M,
	.N = GCC_SDCC1_APPS_N,
	.D = GCC_SDCC1_APPS_D,
};

static const struct bcr_regs sdc2_regs = {
	.cfg_rcgr = GCC_SDCC2_APPS_CFG_RCGR,
	.cmd_rcgr = GCC_SDCC2_APPS_CMD_RCGR,
	.M = GCC_SDCC2_APPS_M,
	.N = GCC_SDCC2_APPS_N,
	.D = GCC_SDCC2_APPS_D,
};

static int clk_init_uart(struct msm_clk_priv *priv, uint rate)
{
	/* NOOP for now, we hope that 1st BL already did that */
	/* TODO: implement */
	(void)priv;
	(void)rate;
	return 0;
}

static int clk_init_sdc1(struct msm_clk_priv *priv, uint rate)
{
#if 0
	/*
	 * From Linux's gcc-sdm660.c:
	 *  ...  F(400000, P_XO, 12, 1, 4), // P_XO maps to value 0
	 *  ...  F(192000000, P_GPLL4, 8, 0, 0), // P_GPLL4 maps to value 5
	 *  ...  F(384000000, P_GPLL4, 4, 0, 0), // linux sets this rate!
	 *   #define F(f, s, h, m, n) { (f), (s), (2 * (h) - 1), (m), (n) }
	 */
#define P_GPLL4 5
	clk_enable_cbc(priv->base + GCC_SDCC1_AHB_CBCR);
	clk_rcg_set_rate_mnd(priv->base, &sdc1_regs, 8, 0, 0, (P_GPLL4 << 8));
	clk_enable_cbc(priv->base + GCC_SDCC1_APPS_CBCR);

#endif
	log_debug("sdm660: clk_init_sdc1: set rate %u\n", rate);
	return rate;
}


static int clk_init_sdc2(struct msm_clk_priv *priv, uint rate)
{
	const int div = 3;

	clk_enable_cbc(priv->base + GCC_SDCC2_AHB_CBCR);
	clk_rcg_set_rate_mnd(priv->base, &sdc2_regs, div, 0, 0, CFG_CLK_SRC_GPLL0);
	// clk_enable_gpll0(priv->base, &gpll0_vote_clk);
	clk_enable_cbc(priv->base + GCC_SDCC2_APPS_CBCR);

	return rate;
}

/* this function is required to exist, called from clock-snapdragon.c */
ulong msm_set_rate(struct clk *clk, ulong rate)
{
	struct msm_clk_priv *priv = dev_get_priv(clk->dev);

	switch (clk->id) {
	case GCC_BLSP1_UART2_APPS_CLK:
	case GCC_BLSP1_AHB_CLK:
		return clk_init_uart(priv, rate);
	case GCC_SDCC1_AHB_CLK:
	case GCC_SDCC1_APPS_CLK:
		return clk_init_sdc1(priv, rate);
	case GCC_SDCC2_AHB_CLK:
	case GCC_SDCC2_APPS_CLK:
		return clk_init_sdc2(priv, rate);
	default:
		return 0;
	}
	return 0;
}

/* this function is required to exist, called from clock-snapdragon.c */
int msm_enable(struct clk *clk)
{
	(void)clk; /* unreferenced parameter */
	return 0;
}
