/*
 * Copyright (C) 2018 Christ Electronic Systems GmbH
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Christ PIXI board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QDLCES_COMMON_CONFIG_H
#define __MX6QDLCES_COMMON_CONFIG_H

#include "mx6_common.h"

#define CONFIG_IMX_THERMAL

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_MXC_UART

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

/* Network */
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_ETHPRIME			"FEC"

#define CONFIG_TFTP_TSIZE
#define CONFIG_IP_DEFRAG
#define CONFIG_TFTP_BLOCKSIZE           16384

#ifdef CONFIG_CMD_SF
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		20000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#endif

#define CONFIG_CRC32_VERIFY
#define CONFIG_CRC32_ADD

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.img\0" \
	"image=zImage\0" \
	"fdt_addr=0x18000000\0" \
	"curt_file=curt.itb\0" \
	"curt_addr=0x20000000\0" \
	"ip_dyn=yes\0" \
	"console=" CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"ddr_addr=0x10000000\0" \
	"run_update=0\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"usbdev=0\0" \
	"usbpart=1\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot}; " \
		"setenv boottype mmc; " \
		"setenv bootdev ${mmcdev}; " \
		"setenv bootpart ${mmcpart}\0" \
	"curtargs=setenv bootargs console=${console},${baudrate} " \
		"rdinit=/linuxrc enable_wait_mode=off; " \
		"setenv boottype usb; " \
		"setenv bootdev ${usbdev}; " \
		"setenv bootpart ${usbpart}\0" \
	"loadbootscript=" \
		"fatload ${boottype} ${bootdev}:${bootpart} ${loadaddr} ${script}\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source ${loadaddr}\0" \
	"loadimage=fatload mmc ${bootdev}:${bootpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${bootdev}:${bootpart} ${fdt_addr} ${fdt_file} && fdt addr ${fdt_addr}\0" \
	"updatefdt=fdt addr ${fdt_addr} && fdt resize 100 && fdt memory ${ddr_addr} ${ddr_size}\0" \
	"setdisplay=run cmd_lvds; run cmd_hdmi; run cmd_lcd\0" \
	"mmcboot=echo Booting from ${boottype} ...; " \
		"run mmcargs; " \
		"if run loadfdt; then " \
			"run updatefdt; " \
			"run setdisplay; " \
			"bootz ${loadaddr} - ${fdt_addr}; " \
		"else " \
			"echo WARN: Cannot load the DT; " \
		"fi\0" \
	"loadcurt=fatload usb ${bootdev}:${bootpart} ${curt_addr} ${curt_file}\0" \
	"curtboot=echo Booting CURT ...; " \
		"bootm start ${curt_addr}#config@${curt_config}; " \
		"bootm loados; " \
		"bootm ramdisk; " \
		"bootm fdt; " \
		"run updatefdt; " \
		"run setdisplay; " \
		"bootm prep; " \
		"bootm go\0" \

#define CONFIG_BOOTCOMMAND \
	"setenv recovery 0; " \
	"if test ${last_bootcheck} -eq 0; then " \
		"setenv recovery 1; " \
	"fi; " \
	"if test ${run_update} -eq 1; then " \
		"setenv recovery 1; " \
	"fi; " \
	"if test ${recovery} -eq 1; then " \
		"run curtargs; " \
		"usb start; " \
		"if run loadbootscript; then " \
			"run bootscript; " \
		"else " \
			"if run loadcurt; then " \
				"run curtboot; " \
			"fi; " \
		"fi; " \
	"fi; " \
	"run mmcargs; " \
	"mmc dev ${bootdev}; " \
	"if mmc rescan; then " \
		"if run loadbootscript; then " \
		"run bootscript; " \
		"else " \
			"if run loadimage; then " \
				"run mmcboot; " \
			"else reset; " \
			"fi; " \
		"fi; " \
	"else reset; fi"

#define CONFIG_ARP_TIMEOUT     200UL

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE			(8 * 1024)

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(3072 * 1024)
#endif

#define CONFIG_SYS_QE_FMAN_FW_LENGTH	0x10000
#define CONFIG_SYS_FDT_PAD		(0x3000 + CONFIG_SYS_QE_FMAN_FW_LENGTH)

/* I2C & EEPROM Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_SYS_I2C_EEPROM_ADDR	0x52
#define EEPROM_ADDR_ENUM		0x18
#define EEPROM_ADDR_REV			0x22
#define EEPROM_ADDR_SERIAL		0x2C
#define EEPROM_ADDR_RES			0x298
#define EEPROM_ADDR_END			0x338

/* Framebuffer */
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_HIDE_LOGO_VERSION
#define CONFIG_IMX_HDMI
#define CONFIG_CMD_FBPANEL
#endif

#define CONFIG_FAT_WRITE

#ifndef CONFIG_SPL
#define CONFIG_USBD_HS

#define CONFIG_USB_FUNCTION_MASS_STORAGE
#endif

#endif                         /* __MX6QDLCES_COMMON_CONFIG_H */
