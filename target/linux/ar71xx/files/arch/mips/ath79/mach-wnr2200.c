/*
 *  NETGEAR WNR2200 board support
 *
 *  Copyright (C) 2013 Aidan Kissane <aidankissane at googlemail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <linux/gpio.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/mach-ath79/ath79.h>
#include <asm/mach-ath79/ar71xx_regs.h>

#include "common.h"
#include "dev-ap9x-pci.h"
#include "dev-eth.h"
#include "dev-gpio-buttons.h"
#include "dev-leds-gpio.h"
#include "dev-m25p80.h"
#include "dev-usb.h"
#include "machtypes.h"

//AR9287 GPIO LED
#define WNR2200_GPIO_LED_WLAN			0
#define WNR2200_GPIO_LED_PWR_AMBER		1
#define WNR2200_GPIO_LED_PWR_GREEN		2
#define WNR2200_GPIO_USB_5V			4
//AR9287 GPIO BUTTON
#define WNR2200_GPIO_BUTTON_WIFI			3
#define WNR2200_GPIO_BUTTON_WPS			5
#define WNR2200_GPIO_BUTTON_RESET		6

//AR7241
#define WNR2200_GPIO_LED_WPS			7
#define WNR2200_GPIO_LED_USB			8

#define WNR2200_GPIO_LED_LAN1_AMBER		6
#define WNR2200_GPIO_LED_LAN2_AMBER		0
#define WNR2200_GPIO_LED_LAN4_AMBER		1
#define WNR2200_GPIO_LED_LAN3_AMBER		11
#define WNR2200_GPIO_LED_WAN_AMBER		12

#define WNR2200_MAC0_OFFSET			0
#define WNR2200_MAC1_OFFSET			6
#define WNR2200_PCIE_CALDATA_OFFSET		0x1000

static struct gpio_led wnr2200_leds_gpio[] __initdata = {
	{
		.name		= "netgear:amber:lan1",
		.gpio		= WNR2200_GPIO_LED_LAN1_AMBER,
		.active_low	= 1,
	}, {
		.name		= "netgear:amber:lan2",
		.gpio		= WNR2200_GPIO_LED_LAN2_AMBER,
		.active_low	= 1,
	}, {
		.name		= "netgear:amber:lan3",
		.gpio		= WNR2200_GPIO_LED_LAN3_AMBER,
		.active_low	= 1,
	}, {
		.name		= "netgear:amber:lan4",
		.gpio		= WNR2200_GPIO_LED_LAN4_AMBER,
		.active_low	= 1,
	}, {
		.name		= "netgear:green:wps",
		.gpio		= WNR2200_GPIO_LED_WPS,
		.active_low	= 1,
	}, {
		.name		= "netgear:green:usb",
		.gpio		= WNR2200_GPIO_LED_USB,
		.active_low	= 1,
	}, {
		.name		= "netgear:amber:wan",
		.gpio		= WNR2200_GPIO_LED_WAN_AMBER,
		.active_low	= 1,
	}
};

static void __init wnr2200_setup(void)
{
	u8 *art = (u8 *) KSEG1ADDR(0x1fff0000);

	ath79_gpio_function_setup(AR724X_GPIO_FUNC_JTAG_DISABLE, 0);

	ath79_register_mdio(0, 0x0);

	ath79_init_mac(ath79_eth0_data.mac_addr, art+WNR2200_MAC0_OFFSET, 0);
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_RMII;
	ath79_eth0_data.speed = SPEED_100;
	ath79_eth0_data.duplex = DUPLEX_FULL;

	ath79_init_mac(ath79_eth1_data.mac_addr, art+WNR2200_MAC1_OFFSET, 0);
	ath79_eth1_data.phy_if_mode = PHY_INTERFACE_MODE_RMII;
	ath79_eth1_data.phy_mask = 0x10;

	ath79_register_eth(0);
	ath79_register_eth(1);

	ath79_register_m25p80(NULL);
	ap91_pci_init(art + WNR2200_PCIE_CALDATA_OFFSET, NULL);
	ap9x_pci_setup_wmac_led_pin(0, WNR2200_GPIO_LED_WLAN);
	
	ath79_register_leds_gpio(-1, ARRAY_SIZE(wnr2200_leds_gpio),
					wnr2200_leds_gpio);

	/* enable power for the USB port */
	ap9x_pci_setup_wmac_gpio(0,
			BIT(WNR2200_GPIO_USB_5V) | BIT(WNR2200_GPIO_LED_PWR_AMBER) | BIT(WNR2200_GPIO_LED_PWR_GREEN),
			BIT(WNR2200_GPIO_USB_5V) | BIT(WNR2200_GPIO_LED_PWR_AMBER));

	ath79_register_usb();
}

MIPS_MACHINE(ATH79_MACH_WNR2200, "WNR2200", "NETGEAR WNR2200", wnr2200_setup);
 
