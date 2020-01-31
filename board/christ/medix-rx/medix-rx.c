/*
 * Copyright (C) 2016-2020 Christ Electronic Systems GmbH
 * Author: Peter Fink <pfink@christ-es.de>
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <linux/errno.h>
#include <linux/ctype.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/fbpanel.h>
#include <environment.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <input.h>
#include <usb.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define PWM_PAD_CTRL (PAD_CTL_PUS_100K_DOWN |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

#define GPIO_PD_PAD_CTRL (PAD_CTL_PUS_100K_DOWN |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define GPIO_PU_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

/* I2C3, EEPROM, RTC */
static struct i2c_pads_info mx6dl_i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6DL_PAD_EIM_D17__I2C3_SCL | I2C_PAD,
		.gpio_mode = MX6DL_PAD_EIM_D17__GPIO3_IO17 | I2C_PAD,
		.gp = IMX_GPIO_NR(3, 17)
	},
	.sda = {
		.i2c_mode = MX6DL_PAD_EIM_D18__I2C3_SDA | I2C_PAD,
		.gpio_mode = MX6DL_PAD_EIM_D18__GPIO3_IO18 | I2C_PAD,
		.gp = IMX_GPIO_NR(3, 18)
	}
};

static struct i2c_pads_info mx6q_i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_EIM_D17__I2C3_SCL | I2C_PAD,
		.gpio_mode = MX6Q_PAD_EIM_D17__GPIO3_IO17 | I2C_PAD,
		.gp = IMX_GPIO_NR(3, 17)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_EIM_D18__I2C3_SDA | I2C_PAD,
		.gpio_mode = MX6Q_PAD_EIM_D18__GPIO3_IO18 | I2C_PAD,
		.gp = IMX_GPIO_NR(3, 18)
	}
};

static iomux_v3_cfg_t const enet_pads[] = {
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
	/* KSZ9031RN PHY Reset */
	IOMUX_PADS(PAD_ENET_CRS_DV__GPIO1_IO25    | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads);

	/* Reset KSZ9031RN PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	mdelay(10);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
	udelay(100);
}

static iomux_v3_cfg_t const usdhc4_pads[] = {
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_ALE__SD4_RESET | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static iomux_v3_cfg_t const disp0_bkl_pads[] = {
	IOMUX_PADS(PAD_ENET_TXD0__GPIO1_IO30	| MUX_PAD_CTRL(GPIO_PD_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA13__GPIO3_IO13	| MUX_PAD_CTRL(GPIO_PD_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_9__GPIO1_IO09	| MUX_PAD_CTRL(GPIO_PD_PAD_CTRL)),
};


static void setup_iomux_bkl(void)
{
	SETUP_IOMUX_PADS(disp0_bkl_pads);

	/* Set DISP_ON, DISP_PWM and BKL_ON to initially LOW => OFF */
	gpio_direction_output(IMX_GPIO_NR(1, 30), 0); /* DISP_ON */
	gpio_direction_output(IMX_GPIO_NR(3, 13) , 0); /* BKL_ON */
	gpio_direction_output(IMX_GPIO_NR(1, 9) , 0); /* DISP_PWM */
}

void board_enable_lvds(const struct display_info_t *di, int enable)
{
	if (enable == 1) {
		gpio_direction_output(IMX_GPIO_NR(1, 30) , enable); /* DISP_ON */
		mdelay(50);
		gpio_direction_output(IMX_GPIO_NR(1, 9) , enable); /* DISP_PWM */
		gpio_direction_output(IMX_GPIO_NR(3, 13) , enable); /* BKL_ON */
	}
	else {
		gpio_direction_output(IMX_GPIO_NR(3, 13) , enable); /* BKL_ON */
		gpio_direction_output(IMX_GPIO_NR(1, 9) , enable); /* DISP_PWM */
		gpio_direction_output(IMX_GPIO_NR(1, 30) , enable); /* DISP_ON */
	}
}

static iomux_v3_cfg_t const status_led_pads[] = {
	IOMUX_PADS(PAD_EIM_DA6__GPIO3_IO06	| MUX_PAD_CTRL(GPIO_PU_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA7__GPIO3_IO07	| MUX_PAD_CTRL(GPIO_PU_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA8__GPIO3_IO08	| MUX_PAD_CTRL(GPIO_PU_PAD_CTRL)),
};

void setup_iomux_status_led(void)
{
	SETUP_IOMUX_PADS(status_led_pads);
	gpio_direction_output(IMX_GPIO_NR(3, 6), 0);
	gpio_direction_output(IMX_GPIO_NR(3, 7), 0);
	gpio_direction_output(IMX_GPIO_NR(3, 8), 0);
}

static iomux_v3_cfg_t const amp_shutdown_pads[] = {
	IOMUX_PADS(PAD_CSI0_DAT12__GPIO5_IO30	| MUX_PAD_CTRL(GPIO_PD_PAD_CTRL)),
};

void setup_iomux_amp_shutdown(void)
{
	SETUP_IOMUX_PADS(amp_shutdown_pads);
	gpio_direction_output(IMX_GPIO_NR(5, 30), 0);
}

static iomux_v3_cfg_t const pwm_pads[] = {
	IOMUX_PADS(PAD_SD1_DAT2__GPIO1_IO19	| MUX_PAD_CTRL(PWM_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_DAT1__GPIO1_IO17	| MUX_PAD_CTRL(PWM_PAD_CTRL)),
	IOMUX_PADS(PAD_SD1_CMD__GPIO1_IO18	| MUX_PAD_CTRL(PWM_PAD_CTRL)),
};

static void setup_iomux_pwm(void)
{
	SETUP_IOMUX_PADS(pwm_pads);

	/* set pwm2, pwm3, pwm4 simply to low as it's function is not used here
	 */
	gpio_direction_output(IMX_GPIO_NR(1, 19), 0); /* PWM2 */
	gpio_direction_output(IMX_GPIO_NR(1, 17), 0); /* PWM3 */
	gpio_direction_output(IMX_GPIO_NR(1, 18), 0); /* PWM4 */
}

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC4_BASE_ADDR},
};

int board_mmc_get_env_dev(int devno)
{
	/* devno from BOOT_CFG2[4:3] is mapped to mmcX here */
	/* 11 (uSDHC4) -> mmc1 */
	switch (devno) {
	case 3: return 0;
		break;
	default:
		printf("Warning: illegal mmc device number.\n");
	}
	return -EINVAL;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int ret;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc1                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			SETUP_IOMUX_PADS(usdhc4_pads);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
#else
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD1
	 * 0x2                  SD2
	 * 0x3                  SD4
	 */

	switch (reg & 0x3) {
	case 0x3:
		SETUP_IOMUX_PADS(usdhc4_pads);
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}
#endif

int board_phy_config(struct phy_device *phydev)
{
/*
 * optimized pad skew values depends on CPU variant on the medix board:
 * i.MX6Q or i.MX6DL
 */
#define CES_KSZ9031_CLK_SKEW  0x03ff

	/* rx/tx clk skew */
	ksz9031_phy_extended_write(phydev, 2,
					MII_KSZ9031_EXT_RGMII_CLOCK_SKEW,
					MII_KSZ9031_MOD_DATA_NO_POST_INC,
					CES_KSZ9031_CLK_SKEW);

	phydev->drv->config(phydev);

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)
static const struct display_info_t displays[] = {

	/* LVDS */
	VD_C_WVGA(LVDS, NULL, 0, 0x00),
	VD_C_SVGA(LVDS, NULL, 0, 0x00),
	VD_C_XGA(LVDS, NULL, 0, 0x00),
	VD_C_XGA_INNOLUX(LVDS, NULL, 0, 0x00),
	VD_C_WXGA(LVDS, NULL, 0, 0x00),
	VD_C_SXGA(LVDS, NULL, 0, 0x00),
	VD_C_FWXGA(LVDS, NULL, 0, 0x00),
	VD_C_FULLHD(LVDS, NULL, 0, 0x00),

	/* LVDS 2 */
	VD_C_WVGA(LVDS2, NULL, 0, 0x00),
	VD_C_SVGA(LVDS2, NULL, 0, 0x00),
	VD_C_XGA(LVDS2, NULL, 0, 0x00),
	VD_C_XGA_INNOLUX(LVDS2, NULL, 0, 0x00),
	VD_C_WXGA(LVDS2, NULL, 0, 0x00),
	VD_C_SXGA(LVDS2, NULL, 0, 0x00),
	VD_C_FWXGA(LVDS2, NULL, 0, 0x00),
	VD_C_FULLHD(LVDS2, NULL, 0, 0x00)
};

#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();

	return cpu_eth_init(bis);
}

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	IOMUX_PADS(PAD_ENET_RX_ER__USB_OTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const usb_hc1_pads[] = {
	IOMUX_PADS(PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void setup_usb(void)
{
	SETUP_IOMUX_PADS(usb_otg_pads);

	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 */
	imx_iomux_set_gpr_register(1, 13, 1, 0);

	SETUP_IOMUX_PADS(usb_hc1_pads);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	case 1:
		if (on)
			gpio_direction_output(IMX_GPIO_NR(1, 5), 1);
		else
			gpio_direction_output(IMX_GPIO_NR(1, 5), 0);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif

int board_early_init_f(void)
{
#ifdef CONFIG_VIDEO_IPUV3
	setup_iomux_bkl();
#endif

	setup_iomux_amp_shutdown();
	setup_iomux_status_led();

	setup_iomux_uart();
	setup_iomux_pwm();
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	if (is_mx6dq() || is_mx6dqp())
		setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c_pad_info0);
	else
		setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6dl_i2c_pad_info0);

#ifdef CONFIG_CMD_FBPANEL
	fbp_setup_display(displays, ARRAY_SIZE(displays));
#endif
#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

struct eeprom_device_info {
	char e_num[10];
	char rev[10];
	char serial[20];
	char eeprom_env[160];
};


/* code taken from lib/hashtable.c [himport_r] */
int insert_env_from_string(char * input, int size)
{
	char sep = ';';
	char *sp, *dp, *name, *value;
	int envset = 0;

	dp = input;

	/* Parse environment; allow for '\0' and 'sep' as separators */
	do {
		/* skip leading white space and non-printable chars */
		while ((isblank(*dp) || !isprint(*dp)) && (dp < input + size))
			++dp;

		if (dp >= input + size)
			break;

		/* parse name */
		for (name = dp; *dp != '=' && *dp && *dp != sep; ++dp)
			;

		/* deal with "name" and "name=" entries
				without values and ignore them */
		if (*dp == '\0' || *(dp + 1) == '\0' ||
		    *dp == sep || *(dp + 1) == sep) {
			if (*dp == '=')
				*dp++ = '\0';
			*dp++ = '\0';   /* terminate name */
			continue;
		}
		*dp++ = '\0';   /* terminate name */

		/* parse value; escapes are not allowed here */
		for (value = sp = dp; *dp && (*dp != sep); ++dp)
			*sp++ = *dp;
		*sp++ = '\0';   /* terminate value */
		++dp;

		if (*name == 0) {
			debug("INSERT: unable to use an empty key\n");
			continue;
		}

		if (env_get_yesno(name) == -1){
			env_set(name, value);
			envset = 1;
		}

	} while ((dp < input + size) && *dp);  /* size check needed for text */
					       /* without '\0' termination */
	return envset;
}

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	char *cpu;
	char fdt_file[32];
	char curt_config[32];
	unsigned long ddr_size = imx_ddr_size();
	int ddr_model = 0;
	char board[10] = {0,};
	char env_cpy[161];
	uint8_t reload_display = 0;

	struct eeprom_device_info eedi;

	if (is_cpu_type(MXC_CPU_MX6Q))
		cpu = "q";
	else if (is_cpu_type(MXC_CPU_MX6DL))
		cpu = "dl";
	else
		cpu = "x";

	env_set_hex("ddr_size", ddr_size);

	if (ddr_size == 0xEFFFFC00)
		ddr_model = 4;
	else
		ddr_model = (int) (ddr_size >> 30);

	sprintf(board, "medix-r%s%1x00", cpu, ddr_model);

	printf("Board: %s\n", board);
	env_set("board_name", board);

	if (env_get_yesno("fdt_file") == -1){
		sprintf(fdt_file, "imx6%s-medix-r%sx00.dtb", cpu, cpu);
		env_set("fdt_file", fdt_file);
	}
	if (env_get_yesno("curt_config") == -1){
		sprintf(curt_config, "medix-r%sx00", cpu);
		env_set("curt_config", curt_config);
	}

	/* get board info from eeprom */

	if(i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, EEPROM_ADDR_ENUM, 2,
			(uint8_t *)&eedi,
			sizeof(eedi) - sizeof(eedi.eeprom_env))){

		printf("EEPROM: Error! [E_ACCESS]\n");

	}
	else {
		printf("Device Identifier: ");
		for (int i=0; i < strlen(&eedi.e_num[0]) &&
					isprint(toascii(eedi.e_num[i])); i++)
			putc(toascii(eedi.e_num[i]));
		printf("\nDevice Revision: ");
		for (int i=0; i < strlen(&eedi.rev[0]) &&
					isprint(toascii(eedi.rev[i])); i++)
			putc(toascii(eedi.rev[i]));
		printf("\nDevice Serial No: ");
		for (int i=0; i < strlen(&eedi.serial[0]) &&
					isprint(toascii(eedi.serial[i])); i++)
			putc(toascii(eedi.serial[i]));
		putc('\n');
	}

	if(i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, EEPROM_ADDR_RES,
			2, (uint8_t *)&eedi.eeprom_env,
			sizeof(eedi.eeprom_env))){

		printf("EEPROM: Error! [E_ACCESS]\n");

	}
	else {
		memcpy(env_cpy, eedi.eeprom_env, 160);
		env_cpy[160] = '\0';
		if (insert_env_from_string(eedi.eeprom_env,
					sizeof(eedi.eeprom_env))){
			printf("Updating ENV from EEPROM: %s\n", env_cpy);
			reload_display = 1;
		}
	}

	if (env_get_yesno("fb_hdmi") == -1 ||
			env_get_yesno("fb_lcd") == -1 ||
			env_get_yesno("fb_lvds") == -1 ||
			env_get_yesno("fb_lvds2") == -1){
		env_set("fb_hdmi", "off");
		env_set("fb_lcd", "*off");
		env_set("fb_lvds", "c-wxga");
		env_set("fb_lvds2", "off");
		reload_display = 1;
	}

#endif
	/* christ: reset bootcheck to 0 if set to 1 and set last_bootcheck accordingly */
	char *last_bootcheck = env_get("bootcheck");

	if (last_bootcheck != NULL)
		env_set("last_bootcheck", last_bootcheck);
	else
		env_set("last_bootcheck", "0");
	env_set("bootcheck", "0");

#ifndef CONFIG_ENV_IS_NOWHERE
	env_save();
#endif

#ifdef CONFIG_CMD_FBPANEL
	if (reload_display == 1) {
		board_enable_lvds(displays, 0);
		board_video_skip();
	}
#endif

	return 0;
}

int checkboard(void)
{
	return 0;
}

