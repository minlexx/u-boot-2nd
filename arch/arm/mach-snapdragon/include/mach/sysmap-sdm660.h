/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Qualcomm SDM660 sysmap
 *
 * (C) Copyright 2022 Alexey Minnekhanov <alexeymin@postmarketos.org>
 */
#ifndef _MACH_SYSMAP_SDM660_H
#define _MACH_SYSMAP_SDM660_H

/* Writing 0 to this register will cause board to reset */
#define MSM_PSHOLD_BASE			0x10ac000

/*
 * Qualcomm TLMM (Top Level Mode Multiplexer) is a gpio/pin controller
 * with several register ranges called "tiles". There are 3 tiles in
 * sdm630/sdm660 TLMM: north, center, south. Each tile has the same size.
 */
#define TLMM_TILE_SIZE			0x400000
#define TLMM_BASE_NORTH			0x3100000
#define TLMM_BASE_CENTER		0x3500000
#define TLMM_BASE_SOUTH			0x3900000

/* Global Clock Controller (GCC) base addr */
#define SDM660_GCC_BASE			0x100000

/*
 * Clock registers offsets (from Global Clock Controller base)
 * for eMMC slot
 */
#define GCC_SDCC1_BCR			(0x16000) /* block reset */
#define GCC_SDCC1_APPS_CBCR		(0x16004) /* branch control */
#define GCC_SDCC1_AHB_CBCR		(0x16008)
//#define GCC_SDCC1_ICE_CORE_CBCR	(0x1600C) /* Inline Crypto Engine */
//#define GCC_SDCC1_ICE_CORE_CMD_RCGR	(0x16010)
//#define GCC_SDCC1_ICE_CORE_CFG_RCGR	(0x16014)
#define GCC_SDCC1_APPS_CMD_RCGR		(0x1602C)
#define GCC_SDCC1_APPS_CFG_RCGR		(0x16030)
#define GCC_SDCC1_APPS_M		(0x16034)
#define GCC_SDCC1_APPS_N		(0x16038)
#define GCC_SDCC1_APPS_D		(0x1603C)
/* ... for external SD card slot */
#define GCC_SDCC2_BCR			(0x14000) /* block reset */
#define GCC_SDCC2_APPS_CBCR		(0x14004) /* branch control */
#define GCC_SDCC2_AHB_CBCR		(0x14008)
#define GCC_SDCC2_APPS_CMD_RCGR		(0x14010)
#define GCC_SDCC2_APPS_CFG_RCGR		(0x14014)
#define GCC_SDCC2_APPS_M		(0x14018)
#define GCC_SDCC2_APPS_N		(0x1401C)
#define GCC_SDCC2_APPS_D		(0x14020)

#endif
