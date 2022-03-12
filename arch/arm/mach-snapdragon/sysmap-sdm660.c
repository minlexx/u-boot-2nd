// SPDX-License-Identifier: GPL-2.0+
/*
 * Qualcomm SDM660 memory map
 *
 * (C) Copyright 2022 Alexey Minnekhanov <alexeymin@postmarketos.org>
 */

#include <common.h>
#include <asm/armv8/mmu.h>

/* From stock bootloader output:
* Add Base: 0x0000000040000000 Available Length: 0x0000000060000000 
* Add Base: 0x00000000A0000000 Available Length: 0x000000005EAC0000
*/

static struct mm_region sdm660_mem_map[] = {
	{
		.virt = 0x0UL, /* Peripheral block */
		.phys = 0x0UL, /* Peripheral block */
		.size = 0x10000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		/* 1st DDR bank ~1.5 Gb */
		.virt = 0x40000000UL, /* DDR */
		.phys = 0x40000000UL, /* DDR */
		.size = 0x60000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		/* 2nd DDR bank ~1.5 Gb */
		.virt = 0xA0000000UL, /* DDR */
		.phys = 0xA0000000UL, /* DDR */
		.size = 0x5EAC0000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = sdm660_mem_map;
