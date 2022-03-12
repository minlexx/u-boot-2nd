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

#endif
