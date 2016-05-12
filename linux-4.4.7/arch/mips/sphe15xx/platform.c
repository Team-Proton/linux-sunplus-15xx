/*
 * Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/platform_data/gpio-sphe15xx.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/serial.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/string.h>
#include <linux/etherdevice.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/mm.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>

#include <asm/addrspace.h>
#include <asm/cacheflush.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <asm/io.h>

#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/sphe15xx.h>
#include <asm/mach-sphe15xx/gpio.h>
#include <asm/mach-sphe15xx/prom.h>
#include <asm/mach-sphe15xx/irq.h>
#include <asm/mach-sphe15xx/spi.h>
#include <asm/mach-sphe15xx/platform.h>

#ifdef __CDT_PARSER__
#define __init
#endif


static struct resource sunplus_uart_resources[] = {
	{
		.name	= "uart0-reg",
		.start	= UART0_DATA,
		.end	= UART0_RX_RESIDUE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "uart0-irq",
		.start	= CYGNUM_HAL_INTERRUPT_UART0,
		.end	= CYGNUM_HAL_INTERRUPT_UART0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device sphe_uart_device = {
	.name		= "sphe15xx-uart",
	.id			= -1,
	.resource	= sunplus_uart_resources,
	.num_resources	= ARRAY_SIZE(sunplus_uart_resources),
};

/*
 0x000000000000-0x000000060000 : "boot"
 0x000000060000-0x000000800000 : "app"
 0x000000800000-0x000000a00000 : "appfs"
 0x000000a00000-0x000000d00000 : "kernel"
 0x000000d00000-0x000001000000 : "rootfs"  			// squashfs
 0x000001000000-0x000001fe0000 : "rootfs_data"		// jffs2
 0x000001fe0000-0x000001ff0000 : "redboot_config"
 0x000001ff0000-0x000002000000 : "fis_dir"
 0x000000a00000-0x000001fe0000 : "firmware"
 0x000000000000-0x000002000000 : "all"
 *
 */

#define REDBOOT_SIZE		0x00060000
#define ECOS_APP_SIZE		0x007A0000
#define ECOS_APP_FS_SIZE	0x00200000
#define LINUX_KERNEL_SIZE	0x00300000
#define LINUX_ROOTFS_SIZE	0x00300000

#define REDBOOT_CFG_SIZE   	(2 * 0x10000)
#define ECOS_PARTS_SIZE   	(REDBOOT_SIZE + ECOS_APP_SIZE + ECOS_APP_FS_SIZE)
#define LINUX_FIRMWARE_SIZE	(0x02000000 - ECOS_PARTS_SIZE - REDBOOT_CFG_SIZE)

#define KERNEL_OFFSET		ECOS_PARTS_SIZE

static struct mtd_partition spiflash_parts[] = {
	{
		.name	= "RedBoot",
		.offset = 0,
		.size	= REDBOOT_SIZE,
	},
	{
		.name	= "app",
		.offset = MTDPART_OFS_APPEND,
		.size	= ECOS_APP_SIZE,
	},
	{
		.name	= "appfs",
		.offset = MTDPART_OFS_APPEND,
		.size	= ECOS_APP_FS_SIZE,
	},
	{
		.name	= "linux",
		.offset = MTDPART_OFS_APPEND,
		.size	= LINUX_FIRMWARE_SIZE,
	},
	{
		.name	= "redboot_config",
		.offset = MTDPART_OFS_APPEND,
		.size	= 0x00010000,
	},
	{
		.name	= "fis_dir",
		.offset = MTDPART_OFS_APPEND,
		.size	= 0x00010000,
	},
	{
		.name	= "all",
		.offset = 0,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data spiflash_data = {
	.name		= "m25pxx",
	.parts		= spiflash_parts,
	.nr_parts	= ARRAY_SIZE(spiflash_parts),
	.type		= "en25qh256",
};

static struct spi_board_info sphe_spi_devs[] __initdata = {
	{
		.modalias	   = "m25pxx",
		.max_speed_hz  = 25000000,
		.bus_num	   = 0,
		.chip_select   = 0,
		.mode		   = 0,
		.platform_data = &spiflash_data,
	},
};

static struct sphe15xx_spi_platform_data sphe_spi_data = {
	.bus_num	= 0,
	.num_chipselect = 1,
};

static struct resource sphe_spi_resources[] = {
	{
		.name	= "spi-reg",
		.start	= SPI_CONTROL,
		.end	= SPI_SCRAMBLE3 + 4,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device sunplus_spi_device = {
	.name		= "sunplus-spi",
	.id			= -1,
	.resource	= sphe_spi_resources,
	.num_resources	= ARRAY_SIZE(sphe_spi_resources),
	.dev = {
		.platform_data	= &sphe_spi_data
	},
};

static void __init sphe15xx_register_wdt(void)
{
	struct resource res;

	memset(&res, 0, sizeof(res));

	res.flags = IORESOURCE_MEM;
	res.start = TIMERW_CTRL;
	res.end = TIMERW_CNT + 4;

	platform_device_register_simple("sphe15xx-wdt", -1, &res, 1);
}

static struct sphe15xx_gpio_platform_data sphe15xx_gpio_pdata = {
		.ngpios = SPHE15XX_GPIO_COUNT,
		.oe_inverted = 0,
};

static struct resource sphe15xx_gpio_resources[] = {
	{
		.flags = IORESOURCE_MEM,
		.start = SPHE15XX_GPIO_BASE,
		.end = SPHE15XX_GPIO_BASE + SPHE15XX_GPIO_REG_OUT + 4,
	},
};

static struct platform_device sphe15xx_gpio_device = {
	.name		= "sphe15xx-gpio",
	.id		= -1,
	.resource	= sphe15xx_gpio_resources,
	.num_resources	= ARRAY_SIZE(sphe15xx_gpio_resources),
	.dev = {
		.platform_data	= &sphe15xx_gpio_pdata
	},
};

static struct resource sphe15xx_mdio_resources[] = {
	{
		.name	= "mdio_base",
		.flags	= IORESOURCE_MEM,
		.start	= MAC_GLB_PHY_CMD,
		.end	= MAC_GLB_PHY_STATUS + 4,
	}
};

struct sphe15xx_mdio_platform_data sphe15xx_mdio_data = {
		.phy_mask = ~1,
};

struct platform_device sphe15xx_mdio_device = {
	.name		= "sphe15xx-mdio",
	.id			= 0,
	.resource	= sphe15xx_mdio_resources,
	.num_resources	= ARRAY_SIZE(sphe15xx_mdio_resources),
	.dev = {
		.platform_data = &sphe15xx_mdio_data,
	},
};

static struct resource sphe15xx_emac_resources[] = {
	{
		.name	= "mac_base",
		.flags	= IORESOURCE_MEM,
		.start	= MAC_GLB_INT_STATUS,
		.end	= G157_RESERVED,
	}, {
		.name	= "mac_irq",
		.flags	= IORESOURCE_IRQ,
		.start	= CYGNUM_HAL_INTERRUPT_MAC,
		.end	= CYGNUM_HAL_INTERRUPT_MAC,
	},
};

struct sphe15xx_emac_platform_data sphe15xx_emac_data = {
	.reset_bit	= 0,
	.mdio_data  = &sphe15xx_mdio_data,
};

struct platform_device sphe15xx_emac_device = {
	.name		= "sphe15xx-emac",
	.id		= 0,
	.resource	= sphe15xx_emac_resources,
	.num_resources	= ARRAY_SIZE(sphe15xx_emac_resources),
	.dev = {
		.platform_data = &sphe15xx_emac_data,
	},
};

static int __init sphe15xx_register_devices(void)
{
	int res;

	sphe15xx_register_wdt();

	res = platform_device_register(&sphe15xx_gpio_device);
	if (res)
		pr_warning("unable to register gpio: %d\n", res);

	res = platform_device_register(&sphe_uart_device);
	if (res)
		pr_warning("unable to register uart: %d\n", res);

	res = spi_register_board_info(sphe_spi_devs, ARRAY_SIZE(sphe_spi_devs));
	if (res)
		pr_warning("unable to register spi board info: %d\n", res);

	res = platform_device_register(&sunplus_spi_device);
	if (res)
		pr_warning("unable to register sunplus spi device: %d\n", res);

	res = platform_device_register(&sphe15xx_mdio_device);
	if (res)
		pr_warning("unable to register sunplus mdio device: %d\n", res);

	res = platform_device_register(&sphe15xx_emac_device);
	if (res)
		pr_warning("unable to register sunplus eth0 device: %d\n", res);

    printk("STC Time: %u\n", (HWREG_R(STC_31_16)  << 16) | HWREG_R(STC_15_0));

	return 0;
}
device_initcall(sphe15xx_register_devices);
