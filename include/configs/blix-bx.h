/*
 * Copyright (C) 2018 Christ Electronic Systems GmbH
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Christ BLIX board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QDLPIXI_CX_CONFIG_H
#define __MX6QDLPIXI_CX_CONFIG_H

#ifdef CONFIG_SPL
#include "imx6_spl.h"
#endif

#define CONFIG_MACH_TYPE	3980
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONSOLE_DEV		"ttymxc0"
#define CONFIG_MMCROOT			"/dev/mmcblk1p2"

#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */

/* Board specific network config */
#define CONFIG_FEC_XCV_TYPE		RMII
#define CONFIG_FEC_MXC_PHYADDR		1

#include "mx6ces_common.h"

/* Falcon Mode */
#define CONFIG_SPL_FS_LOAD_ARGS_NAME   "args"
#define CONFIG_SPL_FS_LOAD_KERNEL_NAME "uImage"
#define CONFIG_SYS_SPL_ARGS_ADDR       0x18000000

/* Falcon Mode - MMC support: args@1MB kernel@2MB */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR  0x800   /* 1MB */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS (CONFIG_CMD_SPL_WRITE_SIZE / 512)
#define CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR        0x1000  /* 2MB */

#define CONFIG_SYS_FSL_USDHC_NUM	2
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		1	/* SDHC4 */
#define CONFIG_SYS_MMC_ENV_PART		1
#endif

/* I2C Board Specific Configs */
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2 /* Enabled USB controller number */
#endif

#endif                         /* __MX6QDLPIXI_CX_CONFIG_H */
