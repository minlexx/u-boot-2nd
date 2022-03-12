// SPDX-License-Identifier: GPL-2.0+
/*
 * Common init part for boards based on SDM845
 *
 * (C) Copyright 2022 Alexey Minnekhanov <alexeymin@postmarketos.org>
 */

#include <asm/io.h>
#include <fdtdec.h>
#include <init.h>
#include <mach/sysmap-sdm660.h>

DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	return fdtdec_setup_mem_size_base();
}

/* works on all SDM630/636/660 SoCs */
__weak void reset_cpu(void)
{
	/* psci_system_reset(); */
	writel(MSM_PSHOLD_BASE, 0);
}

__weak int print_cpuinfo(void)
{
	puts("Qualcomm Kryo 260 (based on Cortex-A53)\n");
	return 0;
}

__weak int board_init(void)
{
	return 0;
}

__weak int misc_init_r(void)
{
	return 0;
}
