// SPDX-License-Identifier: GPL-2.0+
/*
 * Board init file for Xiaomi Redmi Note 7 (lavender)
 *
 * (C) Copyright 2022 Alexey Minnekhanov <alexeymin@postmarketos.org>
 */

#include <common.h>
#include <dm.h>
#include <env.h>
#include <fdtdec.h>
#include <init.h>

#include <asm/gpio.h> // includes linux/bitops.h already
#include <asm/io.h> // writel

#include <linux/delay.h> // udelay

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_DEBUG_UART_BOARD_INIT
static void board_debug_uart_clk_init(void)
{
	/* 
	 * we need to enable a couple of uart clocks:
	 *  - core:  BLSP1_UART2_APPS_CLK
	 *    its parent: BLSP1_UART2_APPS_CLK_SRC
	 * - iface: BLSP1_AHB_CLK
	 */
/* sdm660 global clock controller base addr */
#define GCC_BASE				0x100000
/* aka BLSP1_UART2_APPS_CBCR */
#define BLSP1_UART2_APPS_CLK_ENABLE_REG		0x1c004
#define BLSP1_UART2_APPS_CLK_ENABLE_MASK	BIT(0)

/* aka BLSP1_UART2_APPS_CMD_RCGR */
#define BLSP1_UART2_APPS_CLK_CMD_REG		0x1c00c /* .mnd_width = 16, .hid_width = 5, */
#define BLSP1_UART2_APPS_CLK_CFG_REG		0x1c010
#define BLSP1_UART2_APPS_CLK_M			0x1c014 /* bits 0..15 */
#define BLSP1_UART2_APPS_CLK_N			0x1c018 /* bits 0..15 */ /* ~(N - M) */
#define BLSP1_UART2_APPS_CLK_D			0x1c01c /* bits 0..15 */ /* ~(N) */

#define BLSP1_AHB_CLK_ENABLE_REG		0x52004
#define BLSP1_AHB_CLK_ENABLE_MASK		BIT(17)

#define MND_MASK 		0xf
#define CFG_SRC_DIV_SHIFT	0
#define CFG_SRC_SEL_SHIFT	8
#define CFG_SRC_SEL_MASK	(0x7 << CFG_SRC_SEL_SHIFT)
#define CFG_MODE_SHIFT		12
#define CFG_MODE_MASK		(0x3 << CFG_MODE_SHIFT)
#define CFG_MODE_DUAL_EDGE	(0x2 << CFG_MODE_SHIFT)
#define CFG_HW_CLK_CTRL_MASK	BIT(20)
#define CFG_MASK ((BIT(5) - 1) | CFG_SRC_SEL_MASK | CFG_MODE_MASK | CFG_HW_CLK_CTRL_MASK)
#define CMD_UPDATE		BIT(0)

	/* enable core clk: BLSP1_UART2_APPS_CLK */
	writel(BLSP1_UART2_APPS_CLK_ENABLE_MASK, GCC_BASE + BLSP1_UART2_APPS_CLK_ENABLE_REG);

	/* unfortunately it has a parent that is NOT simple branch (on/off) clock */
	/* it's RCG (Root Clock Generator) that has list of frequencies */
	/* and M/N/D divider setup */
/*
[    0.208682] msm_serial c170000.serial: msm_serial: detected port #0
[    0.208867] msm_serial c170000.serial: uartclk = 1843200
*/
/*
The way the hardware works is that an input clk rate comes from
the clk controller into the uart hw block. That rate is typically
1843200 or 3686400 Hz. That rate can then be divided by an
internal divider in the hw block to achieve a particular baud on
the serial wire.
*/
	/* Input clock freqency for msm_uartdm driver should be baudrate * 16,
	 * so e.g for baudrate 115'200 is is 1'843'200. Freqency table for
	 * BLSP1_UART2_APPS_CLK_SRC has lowest frequency as 3'686'400.
	 * It's fre_tbl line is F(3686400, P_GPLL0, 1,          96,  15625),
	 * which is               (f),     (s),    (2 * (h)-1), (m), (n)
	 *                        3686400, P_GPLL0, 1,          96,  15625
	 *                        freq     src,     pre_div,     m,   n;
	 */
	/* program M, N, D regs */
#define F_M 96
#define F_N 15625
	writel(F_M,          GCC_BASE + BLSP1_UART2_APPS_CLK_M); /* just M */
	writel(~(F_N - F_M), GCC_BASE + BLSP1_UART2_APPS_CLK_N); /* ~(N - M) */
	writel(~F_N,         GCC_BASE + BLSP1_UART2_APPS_CLK_D); /* ~(N) */

	/* source selection and pre_div */
	cfg = (1 << CFG_SRC_DIV_SHIFT) | (1 << CFG_SRC_SEL_SHIFT) | CFG_MODE_DUAL_EDGE;
	// if (F_M != F_N)
	// cfg |= CFG_MODE_DUAL_EDGE;
	writel(cfg & CFG_MASK, GCC_BASE + BLSP1_UART2_APPS_CLK_CFG_REG);
	
	/* update/apply clk_rcg2 config */
	writel(CMD_UPDATE, GCC_BASE + BLSP1_UART2_APPS_CLK_CMD_REG);

	/* enable iface clk: BLSP1_AHB_CLK */
	writel(BLSP1_AHB_CLK_ENABLE_MASK, GCC_BASE + BLSP1_AHB_CLK_ENABLE_REG);

	/* and configure a couple of pins */
	/*
	pins = "gpio4", "gpio5";
	drive-strength = <2>;
	bias-disable;
	*/
}

void board_debug_uart_putc(char ch)
{
#define UARTDM_CR			0x10 /* command register */
#define UARTDM_NCF_TX			0x40 /* register of number of characters to transmit */
#define UARTDM_TF			0x70 /* transtmitter FIFO register */
#define UART_CR_CMD_RESET_TX_READY	(3 << 8)
	// prepare for TX
	// msm_wait_for_xmitr(port);
	writel(UART_CR_CMD_RESET_TX_READY, CONFIG_DEBUG_UART_BASE + UART_CR);
	writel(1, CONFIG_DEBUG_UART_BASE + UARTDM_NCF_TX); /* 1 char to transfer */
	// TX !
	writel(ch, CONFIG_DEBUG_UART_BASE + UARTDM_TF);
}

static uint32_t tmp = 0;
#define MSM_PSHOLD	0x10ac00
#define UARTDM_CSR	0xA0

static inline void board_reset_pshold()
{
	writel(MSM_PSHOLD, 0);
}

struct msm_baud_map {
	u16	divisor;
	u8	code;
	u8	rxstale;
};
static const struct msm_baud_map btable[] = {
	{    1, 0xff, 31 },
	{    2, 0xee, 16 },
	{    3, 0xdd,  8 },
	{    4, 0xcc,  6 },
	{    6, 0xbb,  6 },
	{    8, 0xaa,  6 },
	{   12, 0x99,  6 },
	{   16, 0x88,  1 },
	{   24, 0x77,  1 },
	{   32, 0x66,  1 },
	{   48, 0x55,  1 },
	{   96, 0x44,  1 },
	{  192, 0x33,  1 },
	{  384, 0x22,  1 },
	{  768, 0x11,  1 },
	{ 1536, 0x00,  1 },
};

void board_debug_uart_test(void)
{
	for (tmp=0; tmp < ARRAY_SIZE(btable); tmp++) {
		writel(btable[tmp].code, CONFIG_DEBUG_UART_BASE + UARTDM_CSR);
		//debug_uart_putc('a' + tmp);
		board_debug_uart_putc('a' + tmp);
	}
}

void board_debug_uart_init(void)
{
	//board_debug_uart_clk_init();
	//board_debug_uart_test();
}
#endif /* CONFIG_DEBUG_UART_BOARD_INIT */

#define dump_gcc_reg(reg) \
	tmp = readl(SDM660_GCC_BASE + reg); \
	log_debug(" - %s: %x\n", #reg, tmp);

void lavender_dump_sdcc1_clk_regs(void)
{
	u32 tmp, m, n, d, cfg, src, div;
	dump_gcc_reg(GCC_SDCC1_BCR);
	dump_gcc_reg(GCC_SDCC1_APPS_CBCR);
	dump_gcc_reg(GCC_SDCC1_AHB_CBCR);
	dump_gcc_reg(GCC_SDCC1_APPS_CMD_RCGR);
	dump_gcc_reg(GCC_SDCC1_APPS_CFG_RCGR);
	dump_gcc_reg(GCC_SDCC1_APPS_M);
	dump_gcc_reg(GCC_SDCC1_APPS_N);
	dump_gcc_reg(GCC_SDCC1_APPS_D);

	m = readl(SDM660_GCC_BASE + GCC_SDCC1_APPS_M) & 0xff;
	n = readl(SDM660_GCC_BASE + GCC_SDCC1_APPS_N) & 0xff;
	d = readl(SDM660_GCC_BASE + GCC_SDCC1_APPS_D) & 0xff;

	/* calc n: NOT(N-M) value for MND divider. */
	/* written as: ~((n) - (m)) * !!(n); */
	if (n > 0) {
		n = ((~n) & 0xff) + m;
	}
	/* sanity check */
	if (d != ((~n) & 0xff)) {
		log_debug("Some error in MND decoding! (m: %u, n: %u, d: %u)\n", m, n, d);
		return;
	}

	/* to get the divider and source we need to read different reg */
#define CFG_MASK 0x3fff
#define CFG_CLK_SRC_MASK (0x7 << 8)
#define CFG_DIVIDER_MASK 0x1f
	cfg  = readl(SDM660_GCC_BASE + GCC_SDCC1_APPS_CFG_RCGR) & CFG_MASK;
	src = (cfg >> 8) & 0x7;
	div = cfg & CFG_DIVIDER_MASK;
	/* div written as  (2 * div - 1) & CFG_DIVIDER_MASK; */
	div = (div + 1) / 2;

	log_debug("=> Decoded clock definition: m=%u, n=%u, src=%u, div=%u\n",
		m, n, src, div);
}

/*static*/ void lavender_hacky_sdcc_clocks_init(void)
{
	log_debug("hacky_sdcc_clocks_init start\n");

	lavender_dump_sdcc1_clk_regs();
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

	lavender_dump_sdcc1_clk_regs();

	log_debug("hacky_sdcc_clocks_init end\n");
}

int board_init(void)
{
	//lavender_hacky_sdcc_clocks_init();
	return 0;
}

/* Check for vol- and power buttons */
__weak int misc_init_r(void)
{
	struct udevice *pon;
	struct gpio_desc resin;
	int node, ret;

	log_debug("*** misc_init_r ***\n");

	ret = uclass_get_device_by_name(UCLASS_GPIO, "pm660_pon@800", &pon);
	if (ret < 0) {
		printf("Failed to find PMIC pon node. Check device tree\n");
		return 0;
	}

	node = fdt_subnode_offset(gd->fdt_blob, dev_of_offset(pon),
				  "key_vol_down");
	if (node < 0) {
		printf("Failed to find key_vol_down node. Check device tree\n");
		return 0;
	}
	if (gpio_request_by_name_nodev(offset_to_ofnode(node), "gpios", 0,
				       &resin, 0)) {
		printf("Failed to request key_vol_down button.\n");
		return 0;
	}
	if (dm_gpio_get_value(&resin)) {
		env_set("key_vol_down", "1");
		printf("Volume down button pressed\n");
	} else {
		env_set("key_vol_down", "0");
	}

	node = fdt_subnode_offset(gd->fdt_blob, dev_of_offset(pon),
				  "key_power");
	if (node < 0) {
		printf("Failed to find key_power node. Check device tree\n");
		return 0;
	}
	if (gpio_request_by_name_nodev(offset_to_ofnode(node), "gpios", 0,
				       &resin, 0)) {
		printf("Failed to request key_power button.\n");
		return 0;
	}
	if (dm_gpio_get_value(&resin)) {
		env_set("key_power", "1");
		printf("Power button pressed\n");
	} else {
		env_set("key_power", "0");
	}

	return 0;
}