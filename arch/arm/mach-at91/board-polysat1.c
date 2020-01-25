/*
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2008 Atmel
 *  Copyright (C) 2009 Rob Emanuele
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/at73c213.h>
#include <linux/clk.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/pca953x.h>
#include <linux/spi/xra1405.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/board.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <mach/at91sam9_smc.h>
#include <linux/rtc/ds3234.h>

#include "sam9_smc.h"
#include "generic.h"
#include "../../../drivers/spi/atmel_spi.h"

// Moved to arch/arm/include/asm/system.h
/*
#define LIGHTSAIL_ENG_MODEL 0x2107
#define LIGHTSAIL_BOARD (system_serial_high == 0 && (LIGHTSAIL_ENG_MODEL == system_serial_low) )
#define REV3_ENG_MODEL 0x03
#define REV3_BOARD (system_serial_high == 0 && (REV3_ENG_MODEL == (system_serial_low >> 12)) )
*/

static void __init ek_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9260_initialize(18432000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* USART0 on ttyS1. (Rx, Tx, CTS, RTS, DTR, DSR, DCD, RI) */
   // Modified for PolySat
   /*
	at91_register_uart(AT91SAM9260_ID_US0, 1, ATMEL_UART_CTS | ATMEL_UART_RTS
			   | ATMEL_UART_DTR | ATMEL_UART_DSR | ATMEL_UART_DCD
			   | ATMEL_UART_RI); */
	// at91_register_uart(AT91SAM9260_ID_US0, 1, ATMEL_UART_CTS | ATMEL_UART_RTS);

   // Initialize all UART pins to GPIO driven low
   at91_set_gpio_output(AT91_PIN_PB4, 0);
   at91_set_gpio_output(AT91_PIN_PB5, 0);
   // at91_set_gpio_output(AT91_PIN_PB6, 0);
   at91_set_gpio_output(AT91_PIN_PB7, 0);
   at91_set_gpio_output(AT91_PIN_PB8, 0);
   at91_set_gpio_output(AT91_PIN_PB9, 0);
   // at91_set_gpio_output(AT91_PIN_PA4, 0);
   at91_set_gpio_output(AT91_PIN_PA5, 0);
   at91_set_gpio_output(AT91_PIN_PB12, 0);
   at91_set_gpio_output(AT91_PIN_PB13, 0);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init ek_init_irq(void)
{
	at91sam9260_init_interrupts(NULL);
}


/*
 * USB Host port
 */
static struct at91_usbh_data __initdata ek_usbh_data = {
	.ports		= 2,
};

/*
 * USB Device port
 */
static struct at91_udc_data __initdata ek_udc_data = {
	// .vbus_pin	= AT91_PIN_PC5,
	.vbus_pin	= 0,   // Flight board lacks vbus sense input
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};


// SPI devices for Demux
// Demux is setup as A0 = SPI1_NPCS1, A1 = SPI1_NPCS0, A2 = SPI1_NPCS2, A3 = SPI1_NPCS3
// Active-low input.
static struct AtmelSPIMux spi1_cs0_dev[] = {
   // SPI1_CS0
   { AT91_PIN_PC5, 1 },
   { AT91_PIN_PB3, 1 },
   { AT91_PIN_PC4, 1 },
   { AT91_PIN_PC3, 1 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs1_dev[] = {
   // SPI1_CS1
   { AT91_PIN_PC5, 0 },
   { AT91_PIN_PB3, 1 },
   { AT91_PIN_PC4, 1 },
   { AT91_PIN_PC3, 1 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs2_dev[] = {
   // SPI1_CS2
   { AT91_PIN_PC5, 1 },
   { AT91_PIN_PB3, 0 },
   { AT91_PIN_PC4, 1 },
   { AT91_PIN_PC3, 1 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs3_dev[] = {
   // SPI1_CS3
   { AT91_PIN_PC5, 0 },
   { AT91_PIN_PB3, 0 },
   { AT91_PIN_PC4, 1 },
   { AT91_PIN_PC3, 1 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs4_dev[] = {
   // SPI1_CS4
   { AT91_PIN_PC5, 1 },
   { AT91_PIN_PB3, 1 },
   { AT91_PIN_PC4, 0 },
   { AT91_PIN_PC3, 1 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs5_dev[] = {
   // SPI1_CS5
   { AT91_PIN_PC5, 0 },
   { AT91_PIN_PB3, 1 },
   { AT91_PIN_PC4, 0 },
   { AT91_PIN_PC3, 1 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs6_dev[] = {
   // SPI1_CS6
   { AT91_PIN_PC5, 1 },
   { AT91_PIN_PB3, 0 },
   { AT91_PIN_PC4, 0 },
   { AT91_PIN_PC3, 1 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs7_dev[] = {
   // SPI1_CS7 - transceiver
   { AT91_PIN_PC5, 0 },
   { AT91_PIN_PB3, 0 },
   { AT91_PIN_PC4, 0 },
   { AT91_PIN_PC3, 1 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs8_dev[] = {
   // SPI1_CS8
   { AT91_PIN_PC5, 1 },
   { AT91_PIN_PB3, 1 },
   { AT91_PIN_PC4, 1 },
   { AT91_PIN_PC3, 0 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs9_dev[] = {
   // SPI1_CS9
   { AT91_PIN_PC5, 0 },
   { AT91_PIN_PB3, 1 },
   { AT91_PIN_PC4, 1 },
   { AT91_PIN_PC3, 0 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs10_dev[] = {
   // SPI1_CS10
   { AT91_PIN_PC5, 1 },
   { AT91_PIN_PB3, 0 },
   { AT91_PIN_PC4, 1 },
   { AT91_PIN_PC3, 0 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs11_dev[] = {
   // SPI1_CS11
   { AT91_PIN_PC5, 0 },
   { AT91_PIN_PB3, 0 },
   { AT91_PIN_PC4, 1 },
   { AT91_PIN_PC3, 0 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs12_dev[] = {
   // SPI1_CS12
   { AT91_PIN_PC5, 1 },
   { AT91_PIN_PB3, 1 },
   { AT91_PIN_PC4, 0 },
   { AT91_PIN_PC3, 0 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs13_dev[] = {
   // SPI1_CS13
   { AT91_PIN_PC5, 0 },
   { AT91_PIN_PB3, 1 },
   { AT91_PIN_PC4, 0 },
   { AT91_PIN_PC3, 0 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs14_dev[] = {
   // SPI1_CS14
   { AT91_PIN_PC5, 1 },
   { AT91_PIN_PB3, 0 },
   { AT91_PIN_PC4, 0 },
   { AT91_PIN_PC3, 0 },
   { -1, 0},
};

static struct AtmelSPIMux spi1_cs15_dev[] = {
   // SPI1_CS14
   { AT91_PIN_PC5, 0 },
   { AT91_PIN_PB3, 0 },
   { AT91_PIN_PC4, 0 },
   { AT91_PIN_PC3, 0 },
   { -1, 0},
};


// SPI CS enable struct
// Pin, active state, disabled?  Disabled by default.
static struct AtmelSPIMuxEnable spi_en = {
   AT91_PIN_PA22, 0, 0
};

#if 0
// Ready pin, asserted ready pin state, internal pullup enabled or disabled
// TODO EXAMPLE ONLY: Change for development or flight configurations as needed
static struct AtmelSPIMuxDevReady spi1_dev_rdy = {
   AT91_PIN_PA25,  // Atmel pin
               1,  // Ready state for slave is high
               1   // Enable internal pullup
};
#endif

/*
static struct AtmelSPIMuxData spi1_cs1_data = {
   &spi_en, spi1_cs1_dev, NULL
};
 */

static struct AtmelSPIMuxData spi1_cs0_data = {
   &spi_en, spi1_cs0_dev, NULL, 0
};

static struct AtmelSPIMuxData spi1_cs1_data = {
   &spi_en, spi1_cs1_dev, NULL, 0
};

static struct AtmelSPIMuxData spi1_cs2_data = {
   &spi_en, spi1_cs2_dev, NULL, 0
};

static struct AtmelSPIMuxData spi1_cs3_data = {
   &spi_en, spi1_cs3_dev, NULL, 0
};

static struct AtmelSPIMuxData spi1_cs4_data = {
   &spi_en, spi1_cs4_dev, NULL, 0
};

static struct AtmelSPIMuxData spi1_cs5_data = {
   &spi_en, spi1_cs5_dev, NULL, 0
};

static struct AtmelSPIMuxData spi1_cs6_data = {
   &spi_en, spi1_cs6_dev, NULL, 0
};

static struct AtmelSPIMuxData spi1_cs7_data = {
   &spi_en, spi1_cs7_dev, NULL, 0
};

static struct AtmelSPIMuxData spi1_cs8_data = {
   &spi_en, spi1_cs8_dev, NULL, 0
};

static struct AtmelSPIMuxData spi1_cs9_data = {
   &spi_en, spi1_cs9_dev, NULL, 0
};

static struct AtmelSPIMuxData spi1_cs10_data = {
   &spi_en, spi1_cs10_dev, NULL, 5
};

static struct AtmelSPIMuxData spi1_cs11_data = {
   &spi_en, spi1_cs11_dev, NULL, 5
};

static struct AtmelSPIMuxData spi1_cs12_data = {
   &spi_en, spi1_cs12_dev, NULL, 5
};

static struct AtmelSPIMuxData spi1_cs13_data = {
   &spi_en, spi1_cs13_dev, NULL, 5
};

static struct AtmelSPIMuxData spi1_cs14_data = {
   &spi_en, spi1_cs14_dev, NULL, 5
};

static struct AtmelSPIMuxData spi1_cs15_data = {
   &spi_en, spi1_cs15_dev, NULL, 5
};


static struct {
   //int regToggleGPIO, regReadIRQ;
   int dbaRegToggleGPIO, regRdyIRQ, txRxToggleGPIO;
} plat_data_axsem = { AT91_PIN_PC8, AT91_PIN_PC10, AT91_PIN_PB19 };

static struct timeval xra_shared_irq_time;
static struct xra1405_platform_data plat_data_xra1405 = { 160, &xra_shared_irq_time };
static struct DS3234PlatData plat_data_ds3234 = { 168, &xra_shared_irq_time };

/*
 * SPI devices.
 */

static struct spi_board_info r9_rtc_spidev = {
		.modalias	= "ds3234",
		.chip_select	= 8,
      .controller_data = &spi1_cs8_data,
                .platform_data = &plat_data_ds3234,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 1,
};

static struct spi_board_info r9_xra_spidev = {
		.modalias	= "xra1405",
		.chip_select	= 3,
                .platform_data = &plat_data_xra1405,
                .controller_data = &spi1_cs3_data,
		.max_speed_hz	= 16 * 1000 * 1000,
		.bus_num	= 1,
                .irq = 29, // Handled w/physical IRQ line, IRQ0
};

#define SPI0_3_IDX 3
#define SPI0_8_IDX 8

static struct spi_board_info ek_spi_devices[] = {
	{	//default device 1.0
		.modalias	= "spidev",
		.chip_select	= 0,
      .controller_data = &spi1_cs0_data,
		.max_speed_hz	= 2 * 1000 * 1000,
		.bus_num	= 1,
      .mode = SPI_MODE_0,
	},
	{	//default device 1.1
		.modalias	= "spidev",
		.chip_select	= 1,
      .controller_data = &spi1_cs1_data,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 1,
		.mode = SPI_MODE_3,
	},
	{	//default device 1.2
		.modalias	= "spidev",
		.chip_select	= 2,
      .controller_data = &spi1_cs2_data,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 1,
	},
	{	//default device 1.3
		.modalias	= "spidev",
		.chip_select	= 3,
      .controller_data = &spi1_cs3_data,
		.max_speed_hz	= 2 * 1000 * 1000,
		.bus_num	= 1,
	},
	{	//default device 1.4
		.modalias	= "spidev",
		.chip_select	= 4,
      .controller_data = &spi1_cs4_data,
		.max_speed_hz	= 2 * 1000 * 1000,
		.bus_num	= 1,
	},
	{	//default device 1.5
		.modalias	= "spidev",
		.chip_select	= 5,
      .controller_data = &spi1_cs5_data,
		.max_speed_hz	= 2 * 1000 * 1000,
		.bus_num	= 1,
	},
	{	//default device 1.6
		.modalias	= "spidev",
		.chip_select	= 6,
      .controller_data = &spi1_cs6_data,
		.max_speed_hz	= 2 * 1000 * 1000,
		.bus_num	= 1,
	},
	{	// Transceiver
		.modalias	= "AX5042",
		.chip_select	= 7,
      .controller_data = &spi1_cs7_data,
      .platform_data = &plat_data_axsem,
		.max_speed_hz	= 2 * 1000 * 1000,
		.bus_num	= 1,
      .irq = 30, // Handled w/physical IRQ line
	},
	{	//default device 1.0
		.modalias	= "ds3234",
		.chip_select	= 8,
      .controller_data = &spi1_cs8_data,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 1,
	},
	{	//default device 1.0
		.modalias	= "spidev",
		.chip_select	= 9,
      .controller_data = &spi1_cs9_data,
		.max_speed_hz	= 2 * 1000 * 1000,
		.bus_num	= 1,
	},
	{	//default device 1.0
		.modalias	= "spidev",
		.chip_select	= 10,
      .controller_data = &spi1_cs10_data,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 1,
	},
	{	//default device 1.0
		.modalias	= "spidev",
		.chip_select	= 11,
      .controller_data = &spi1_cs11_data,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 1,
	},
	{	//default device 1.0
		.modalias	= "spidev",
		.chip_select	= 12,
      .controller_data = &spi1_cs12_data,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 1,
	},
	{	//default device 1.0
		.modalias	= "spidev",
		.chip_select	= 13,
      .controller_data = &spi1_cs13_data,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 1,
	},
	{	//default device 1.0
		.modalias	= "spidev",
		.chip_select	= 14,
      .controller_data = &spi1_cs14_data,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 1,
	},
	{	//default device 1.0
		.modalias	= "spidev",
		.chip_select	= 15,
      .controller_data = &spi1_cs15_data,
		.max_speed_hz	= 1 * 1000 * 1000,
		.bus_num	= 1,
	},
//#if !(defined(CONFIG_MMC_ATMELMCI) || defined(CONFIG_MMC_AT91))
	{	// Numonyx PCM chip primary, mtd6
		// .modalias	= "mtd_dataflash",
		.modalias	= "spidev",
		.chip_select	= 0,
      .controller_data = NULL,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 0,
		.mode = SPI_MODE_0,
	},
	{	// Numonyx PCM chip secondary, mtd7
		// .modalias	= "mtd_dataflash",
		.modalias	= "spidev",
		.chip_select	= 1,
      .controller_data = NULL,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 0,
		.mode = SPI_MODE_0,
	},
	/*
#if defined(CONFIG_MTD_AT91_DATAFLASH_CARD)
	{	DataFlash card
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
	},
#endif */
//#endif
};


/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PB0,
	.is_rmii	= 1,
};


/*
 * NAND flash
 */
static struct mtd_partition __initdata ek_nand_partition[] = {
	{
		.name   = "Bootstrap",
		.offset = 0,
		.size   = 4 * SZ_1M,
	},
	{
		.name	= "Partition 1",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 60 * SZ_1M,
	},
	{
		.name	= "Partition 2",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(ek_nand_partition);
	return ek_nand_partition;
}

/* det_pin is not connected */
static struct atmel_nand_data __initdata ek_nand_data = {
	.ale		= 21,
	.cle		= 22,
	.rdy_pin	= AT91_PIN_PC13,
        // irq2 pin
	// .enable_pin	= AT91_PIN_PC14,
        // non-irq2 pin
	.enable_pin	= AT91_PIN_PC10,
	.partition_info	= nand_partitions,
#if defined(CONFIG_MTD_NAND_ATMEL_BUSWIDTH_16)
	.bus_width_16	= 1,
#else
	.bus_width_16	= 0,
#endif
};

static struct sam9_smc_config __initdata ek_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 2,
	.ncs_write_setup	= 0,
	.nwe_setup		= 2,

	.ncs_read_pulse		= 4,
	.nrd_pulse		= 4,
	.ncs_write_pulse	= 4,
	.nwe_pulse		= 4,

	.read_cycle		= 7,
	.write_cycle		= 7,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 3,
};

static int __init nandcs_setup(char *str)
{
    printk("NAND parameter is %s\n", str);
    if (0 == strcmp("PC14", str))
       ek_nand_data.enable_pin = AT91_PIN_PC14;
    else if (0 == strcmp("PC10", str))
       ek_nand_data.enable_pin = AT91_PIN_PC10;
    return 1;
}
__setup("nandcs=", nandcs_setup);

static void __init ek_add_device_nand(void)
{
   // Some older boards have complete NAND failure.  This check doesn't
   // register the NAND device on those boards, in the hope they can still
   // be partially useful.
   if (system_serial_high == 0 && (system_serial_low == 0x2108 ||
                                   system_serial_low == 0x5109)) {
      printk("**** Skipping NAND device!\n");
      return;
   }

	/* setup bus-width (8 or 16) */
	if (ek_nand_data.bus_width_16)
		ek_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		ek_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &ek_nand_smc_config);

   printk("Register NAND on %d\n", ek_nand_data.enable_pin);
	at91_add_device_nand(&ek_nand_data);
}

/*
 * MCI (SD/MMC)
 * wp_pin is not connected
 */
#if defined(CONFIG_MMC_ATMELMCI) || defined(CONFIG_MMC_ATMELMCI_MODULE)
static struct mci_platform_data __initdata ek_mmc_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin		= -ENODEV,
		// .detect_pin	= AT91_PIN_PC2,
		.wp_pin		= -ENODEV,
	},
	.slot[1] = {
		// .bus_width	= 4,
		.bus_width	= 0, // Disable mmc1
		.detect_pin		= -ENODEV,
		// .detect_pin	= AT91_PIN_PC9,
		.wp_pin		= -ENODEV,
	},

};
#else
static struct at91_mmc_data __initdata ek_mmc_data = {
	.slot_b		= 0,	/* Only one slot so use slot B */
	.wire4		= 1,
	// 	.det_pin		= -ENODEV,
	// .det_pin	= AT91_PIN_PC9,
};
#endif

/*
 * LEDs
 */
/*
static struct gpio_led ek_leds[] = {
	{	// "bottom" led, green, userled1 to be defined
		.name			= "ds5",
		.gpio			= AT91_PIN_PB8,
		.active_low		= 1,
		.default_trigger	= "none",
	},
	{	//"power" led, yellow
		.name			= "ds1",
		.gpio			= AT91_PIN_PB9,
		.default_trigger	= "heartbeat",
	}
}; */

#if 0
/*
 * GPIO Buttons
 */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button ek_buttons[] = {
	{
		.gpio		= AT91_PIN_PA30,
		.code		= BTN_3,
		.desc		= "Button 3",
		.active_low	= 1,
		.wakeup		= 1,
	},
	{
		.gpio		= AT91_PIN_PA31,
		.code		= BTN_4,
		.desc		= "Button 4",
		.active_low	= 1,
		.wakeup		= 1,
	}
};

static struct gpio_keys_platform_data ek_button_data = {
	.buttons	= ek_buttons,
	.nbuttons	= ARRAY_SIZE(ek_buttons),
};

static struct platform_device ek_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &ek_button_data,
	}
};
#endif

#if 0
static void __init ek_add_device_buttons(void)
{
	at91_set_gpio_input(AT91_PIN_PA30, 1);	/* btn3 */
	at91_set_deglitch(AT91_PIN_PA30, 1);
	at91_set_gpio_input(AT91_PIN_PA31, 1);	/* btn4 */
	at91_set_deglitch(AT91_PIN_PA31, 1);

	platform_device_register(&ek_button_device);
}
#endif
#else
#if 0
static void __init ek_add_device_buttons(void) {}
#endif
#endif

/*
 * I2C devices
 */
static struct at24_platform_data at24c1024 = {
	.byte_len	= (SZ_1M)/ 8,
	.page_size	= 256,
	.flags		= AT24_FLAG_ADDR16,
};

static struct pca953x_platform_data pca9535 = {
	.gpio_base	= 200,
	.invert		= 0,
};

#if 0
static int ax5042_setup_nandcs(struct i2c_client *client,
    unsigned gpio, unsigned ngpio, void *context)
{
   unsigned num = gpio + (uintptr_t)context;

   if (ek_nand_data.enable_pin == AT91_PIN_PC10) {
      at91_set_GPIO_periph(AT91_PIN_PC10, 0);
      at91_set_B_periph(AT91_PIN_PC14, 0);
      // at91_set_GPIO_periph(AT91_PIN_PC14, 0);
      if (gpio_request(num, "NAND CS Switch") >= 0) {
         gpio_direction_output(num, 0);
         printk("NAND Switched to alternate CS line\n");
      }
   }
   return 0;
}
#endif

static struct pca953x_platform_data ax5042_pca9535 = {
	.gpio_base	= 220,
	.invert		= 0,
//	.context	= (void*)8,
//	.setup		= &ax5042_setup_nandcs,
};

static struct i2c_board_info __initdata ek_i2c_devices[] = {
/*	{
		I2C_BOARD_INFO("wm8731", 0x1b),
	},*/
	{ /* 1Mbit EEPROM device,
	   * modified version of at24c512 as found in board-sam9260ek.c
      */
		I2C_BOARD_INFO("at24c1024", 0x50),
		.platform_data = &at24c1024
      //I2C_BOARD_INFO("i2c-3", 0x00),
	},
	{   /* I2C-0 device.  See pins in at91sam9260_devices.c*/
    	 I2C_BOARD_INFO("i2c-sb", 0x00),
       .platform_data = NULL
   },
   /*
    *  GPIO expander 1
    */
	{
	    I2C_BOARD_INFO("pca9535", 0x77),
		.platform_data = &pca9535
	},
	/* Now mission specific{
	    I2C_BOARD_INFO("ov3642", 0x3C),
       .platform_data = NULL
	},*/
};


static struct i2c_board_info __initdata ek_i2c_devices_rev3[] = {
/*	{
		I2C_BOARD_INFO("wm8731", 0x1b),
	},*/
	{ /* 1Mbit EEPROM device,
	   * modified version of at24c512 as found in board-sam9260ek.c
      */
		I2C_BOARD_INFO("at24c1024", 0x50),
		.platform_data = &at24c1024
      //I2C_BOARD_INFO("i2c-3", 0x00),
	},
	{   /* I2C-0 device.  See pins in at91sam9260_devices.c*/
    	 I2C_BOARD_INFO("i2c-0", 0x00),
       .platform_data = NULL
   },
   /*
    *  GPIO expander 1
    */
	{
	    I2C_BOARD_INFO("pca9535", 0x77),
		.platform_data = &pca9535
	},
   /*
    *  AX5042 GPIO expander (regulator check)
    */
	{
	    I2C_BOARD_INFO("pca9555", 0x20),
		.platform_data = &ax5042_pca9535
	},
   /*
    *  PCA9550 LED driver on top panel for magnetorquer
    */
   /*
	{
	    I2C_BOARD_INFO("pca9550", 0x60),
		.platform_data = NULL
	},
   */
	/* Now mission specific {
	    I2C_BOARD_INFO("ov3642", 0x3C),
       .platform_data = NULL
	}, */
};


#if 0
// Additional PolySat I2C devices.  Cannot be in the same ek_i2c_dev above.
static struct i2c_board_info __initdata ek_i2c_devices1[] = {
	{   /* I2C-1 device.  See pins in at91sam9260_devices.c*/
       I2C_BOARD_INFO("i2c-1", 0x00),
   },
};
#endif

// Additional PolySat I2C devices.  Cannot be in the same ek_i2c_dev above.
static struct i2c_board_info __initdata ek_i2c_devices_pl[] = {
	{   /* I2C-2 device.  See pins in at91sam9260_devices.c*/
       I2C_BOARD_INFO("i2c-pl", 0x00),
   },
};

static void __init ek_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* USB Host */
	at91_add_device_usbh(&ek_usbh_data);
	/* USB Device */
	at91_add_device_udc(&ek_udc_data);
	/* NAND */
	//ek_add_device_nand();
	/* Ethernet */
	at91_add_device_eth(&ek_macb_data);
	/* MMC */
   //  Currently not needed for polysat proto. Commented
#if defined(CONFIG_MMC_ATMELMCI) || defined(CONFIG_MMC_ATMELMCI_MODULE)
	at91_add_device_mci(0, &ek_mmc_data);
#else
	at91_add_device_mmc(0, &ek_mmc_data);
#endif
	/* I2C */
   // Extra struct for transceiver GPIO expander?
   if (BOARD_REV_NUM <= 2) {
	   at91_add_device_i2c(ek_i2c_devices, ARRAY_SIZE(ek_i2c_devices));
      printk("\n\n *** Not Rev 3/4 board init.\n\r");
   }
   else {
      //if (LIGHTSAIL_BOARD) {
         // Enable 3v3 PL regulator for LS
         // Should be in a kernel module now!
         // at91_set_gpio_output(AT91_PIN_PC6, 1);
         // at91_set_gpio_output(AT91_PIN_PC7, 1);
         printk("\n *** 3v3 and 5V0 PL disabled\n\r");
      //}
	   at91_add_device_i2c(ek_i2c_devices_rev3, ARRAY_SIZE(ek_i2c_devices_rev3));

      // COMM Daughter A uses diff regulator-ready pins b/w Revs
      plat_data_axsem.regRdyIRQ = ax5042_pca9535.gpio_base + 5;
      printk("\n\n *** Rev 3 or greater board init.\n\r");
   }

	/* SPI */
   if (BOARD_REV_NUM >= 9) {
      ek_spi_devices[SPI0_3_IDX] = r9_xra_spidev;
      ek_spi_devices[SPI0_8_IDX] = r9_rtc_spidev;
   }

   at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));

   // No LEDs or Push buttons needed for PolySat
	/* LEDs */
	//at91_gpio_leds(ek_leds, ARRAY_SIZE(ek_leds));
	/* Push Buttons */
	//ek_add_device_buttons();

   // No WM8731 functionality needed for PolySat
	/* PCK0 provides MCLK to the WM8731 */
	//at91_set_B_periph(AT91_PIN_PC1, 0);
	/* SSC (for WM8731) */
	//at91_add_device_ssc(AT91SAM9260_ID_SSC, ATMEL_SSC_TX);


   // Custom initialization for GPIOs needed by PolySat-1
   at91_set_GPIO_periph(AT91_PIN_PC9, 1);
   gpio_set_value(AT91_PIN_PC9, 0);

   at91_set_GPIO_periph(AT91_PIN_PC8, 1);
   at91_set_deglitch(AT91_PIN_PC8, 1);
   gpio_set_value(AT91_PIN_PC8, 0);
   at91_set_GPIO_periph(AT91_PIN_PC10, 1);
   at91_set_deglitch(AT91_PIN_PC10, 1);
   at91_set_GPIO_periph(AT91_PIN_PB19, 1);
   at91_set_deglitch(AT91_PIN_PB19, 1);
   gpio_set_value(AT91_PIN_PB19, 0);

   // Select GPIO for NAND CS switch
   if (BOARD_REV_NUM > 6) {
      // printk("*** NAND CS on default\n");
      // at91_set_A_periph(AT91_PIN_PC14, 0);
      // at91_set_GPIO_periph(AT91_PIN_PC14, 1);
      // at91_set_deglitch(AT91_PIN_PC14, 1);
#if 0
      if (gpio_request(ax5042_pca9535.gpio_base + 8, "NAND CS test")) {
         printk("*** Error requesting NAND CS swap pin 2\n");
      }
      else {
         printk("*** NAND CS moved off IRQ2 pin\n");
         gpio_direction_output(ax5042_pca9535.gpio_base + 8, 0);
         gpio_set_value(ax5042_pca9535.gpio_base + 8, 0);
         ek_nand_data.enable_pin = AT91_PIN_PC10;
         // gpio_set_value(AT91_PIN_PC10, 0);
      }
#endif
   }

   // IRQ line for transceiver
   at91_set_gpio_input(AT91_PIN_PC2, 0);
   at91_set_gpio_input(AT91_PIN_PC12, 0);
   at91_set_B_periph(AT91_PIN_PC15, 0);

   /* NAND */
   at91_set_GPIO_periph(ek_nand_data.enable_pin, 0);
   if (ek_nand_data.enable_pin == AT91_PIN_PC10)
      at91_set_B_periph(AT91_PIN_PC14, 0);
   ek_add_device_nand();

#if 0
   if (LIGHTSAIL_BOARD) {
      // GPIO for BW Antenna EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA4, 1);
      // GPIO for BW Panels 1 EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA5, 1);
      // GPIO for Camera 1 5V0 EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA26, 1);
      // GPIO for Camera 1 Heater EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA27, 1);
      // GPIO for Camera 3 5V0 EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA28, 1);
      // GPIO for Camera 3 Heater EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA29, 1);

      // GPIO for BW deployer EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA30, 1);
      // GPIO for BW Panels 2 EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA31, 1);

      printk("LIGHTSAIL CONFIGURATION\n");
      // GPIOs for Mag R1 and Mag R2 - LS
      at91_set_GPIO_periph(AT91_PIN_PB12, 1);
      at91_set_GPIO_periph(AT91_PIN_PB13, 1);

      // GPIOs for Mag R3 and Mag R4 - LS
      at91_set_GPIO_periph(AT91_PIN_PB29, 1);
      at91_set_GPIO_periph(AT91_PIN_PB30, 1);

	   // at91_add_device_i2c(ek_i2c_devices1, ARRAY_SIZE(ek_i2c_devices1));
   }
   else if (REV4_TEST) {
      printk("*** Rev 4 testing\n\r");

      // Typically these are RTS and CTS for ttyS1
      at91_set_GPIO_periph(AT91_PIN_PA4, 1);
      at91_set_GPIO_periph(AT91_PIN_PA5, 1);

      // GPIO for Camera 1 5V0 EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA26, 1);
      // GPIO for Camera 1 Heater EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA27, 1);
      // GPIO for Camera 3 5V0 EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA28, 1);
      // GPIO for Camera 3 Heater EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA29, 1);

      // GPIO for BW deployer EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA30, 1);
      // GPIO for BW Panels 2 EN - LS
      at91_set_GPIO_periph(AT91_PIN_PA31, 1);

      // Typically these are ISI lines
      at91_set_GPIO_periph(AT91_PIN_PB10, 1);
      at91_set_GPIO_periph(AT91_PIN_PB11, 1);
      at91_set_GPIO_periph(AT91_PIN_PB12, 1);
      at91_set_GPIO_periph(AT91_PIN_PB13, 1);
      at91_set_GPIO_periph(AT91_PIN_PB20, 1);
      at91_set_GPIO_periph(AT91_PIN_PB21, 1);
      at91_set_GPIO_periph(AT91_PIN_PB22, 1);
      at91_set_GPIO_periph(AT91_PIN_PB23, 1);
      at91_set_GPIO_periph(AT91_PIN_PB24, 1);
      at91_set_GPIO_periph(AT91_PIN_PB25, 1);
      at91_set_GPIO_periph(AT91_PIN_PB26, 1);
      at91_set_GPIO_periph(AT91_PIN_PB27, 1);
      at91_set_GPIO_periph(AT91_PIN_PB28, 1);
      at91_set_GPIO_periph(AT91_PIN_PB29, 1);
      at91_set_GPIO_periph(AT91_PIN_PB30, 1);
      at91_set_GPIO_periph(AT91_PIN_PB31, 1);

      at91_set_GPIO_periph(AT91_PIN_PB4, 1);
      at91_set_GPIO_periph(AT91_PIN_PB5, 1);
      at91_set_GPIO_periph(AT91_PIN_PB6, 1);
      at91_set_GPIO_periph(AT91_PIN_PB7, 1);
      at91_set_GPIO_periph(AT91_PIN_PB8, 1);
      printk("PB8 to GPIO\n");
      at91_set_GPIO_periph(AT91_PIN_PB9, 1);
      at91_set_GPIO_periph(AT91_PIN_PB16, 1);
      at91_set_GPIO_periph(AT91_PIN_PB19, 1);

      at91_set_GPIO_periph(AT91_PIN_PC10, 1);
      at91_set_GPIO_periph(AT91_PIN_PC12, 1);
      at91_set_GPIO_periph(AT91_PIN_PC13, 1);
      at91_set_GPIO_periph(AT91_PIN_PC14, 1);
      at91_set_GPIO_periph(AT91_PIN_PC15, 1);
   }
   else {
      // Now configured in separate kernel module
	   // at91_add_device_i2c(ek_i2c_devices1, ARRAY_SIZE(ek_i2c_devices1));
   }
   #endif

   // Set GPIO pins into GPIO and known state.
   at91_set_gpio_output(AT91_PIN_PA29, 0);
   at91_set_gpio_output(AT91_PIN_PB16, 0);
   at91_set_gpio_output(AT91_PIN_PC6, 0);
   at91_set_gpio_output(AT91_PIN_PC7, 0);

   // Payload I2C device
	at91_add_device_i2c(ek_i2c_devices_pl, ARRAY_SIZE(ek_i2c_devices_pl));
}

MACHINE_START(POLYSAT1, "PolySat Avionics Board, version 1")
	/* Maintainer: Rob Emanuele */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= ek_map_io,
	.init_irq	= ek_init_irq,
	.init_machine	= ek_board_init,
MACHINE_END
