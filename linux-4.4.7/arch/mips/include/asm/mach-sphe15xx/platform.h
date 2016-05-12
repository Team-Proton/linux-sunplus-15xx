/*
 *  Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 *  Sunplus SPHE15XX SoC specific platform data definitions
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#ifndef __ASM_MACH_SPHE15XX_PLATFORM_H
#define __ASM_MACH_SPHE15XX_PLATFORM_H

#include <linux/if_ether.h>
#include <linux/skbuff.h>
#include <linux/phy.h>
#include <linux/spi/spi.h>

struct sphe15xx_mdio_platform_data {
	u32		phy_mask;
	u32		phy_addr;
	u32		phy_id;
	struct mii_bus	*mii_bus;
	unsigned long	mdio_clock;
	unsigned long	ref_clock;

	void		(*reset)(struct mii_bus *bus);
};

struct sphe15xx_mdio_priv {
	struct mii_bus		*mii_bus;
	struct sphe15xx_mdio_platform_data *pdata;

	void __iomem		*membase;
	unsigned int		base;

	int (*read_reg)(struct mii_bus *bus, int page, int reg);
	int (*write_reg)(struct mii_bus *bus, int page, int reg, unsigned short val);
};

struct sphe15xx_emac_platform_data {
	phy_interface_t	phy_if_mode;
	u32		phy_mask;
	int		speed;
	int		duplex;
	u32		reset_bit;
	u8		mac_addr[ETH_ALEN];
	struct device	*mii_bus_dev;
	struct sphe15xx_mdio_platform_data *mdio_data;

	void		(*set_speed)(int speed);

	unsigned int	max_frame_len;
	unsigned int	desc_pktlen_mask;
};

#endif /* __ASM_MACH_SPHE15XX_PLATFORM_H */
