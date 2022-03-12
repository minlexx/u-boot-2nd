/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration file for Xiaomi Redmi Note 7 based on Qualcomm SDM660 chip
 *
 * (C) Copyright 2022 Alexey Minnekhanov <alexeymin@postmarketos.org>
 */

#ifndef __CONFIGS_SDM660_XIAOMI_LAVENDER_H
#define __CONFIGS_SDM660_XIAOMI_LAVENDER_H

#include <linux/sizes.h>
#include <asm/arch/sysmap-sdm660.h>

#define CONFIG_SYS_BAUDRATE_TABLE	{ 115200 }

/* Generic Timer Definitions */
#define COUNTER_FREQUENCY	18000000

/* Size of malloc() pool */
#define CONFIG_SYS_BOOTM_LEN	SZ_64M

/* Monitor Command Prompt */
#define CONFIG_SYS_CBSIZE	512
#define CONFIG_SYS_MAXARGS	64

/* which place is correct? board/xiaomi/lavender/lavender.env or here? */
/*
In file included from ../env/common.c:32:
../include/env_default.h:113:3: error: #error "Your board uses a text-file environment,
	so must not define CONFIG_EXTRA_ENV_SETTINGS" */

/*#define CONFIG_EXTRA_ENV_SETTINGS \
	"stdout=serial,vidconsole" \
	"stderr=serial,vidconsole"
*/

#endif
