/*
 * Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>

#include <asm/mach-sphe15xx/gpio.h>
#include <asm/mach-sphe15xx/platform.h>

struct sp483_priv {
	int	chip_id;
	int	chip_rev;
};

static int sp483_config_init(struct phy_device *phydev)
{
	return genphy_config_init(phydev);
}

static int sp483_config_aneg(struct phy_device *phydev)
{
	return genphy_config_aneg(phydev);
}

static int sp483_aneg_done(struct phy_device *phydev)
{
	return genphy_aneg_done(phydev);
}

static int sp483_read_status(struct phy_device *phydev)
{
	return genphy_read_status(phydev);
}

static int sp483_soft_reset(struct phy_device *phydev)
{
	/* Do nothing for now */
	return 0;
}

static int sp483_suspend(struct phy_device *phydev)
{
	return genphy_suspend(phydev);
}

static int sp483_resume(struct phy_device *phydev)
{
	return genphy_resume(phydev);
}

static int sp483_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->dev;
	struct sp483_priv *priv;
	struct sphe15xx_mdio_priv *mdio;

	u32 val;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phydev->priv = priv;


	/*  Set EPHY LED pin  */
	#ifdef EPHY_LED_PINOUT1
		val = HWREG_R(SFT_CFG_1) & ~(0x7<<22);
		HWREG_W(SFT_CFG_1, val | (0x2 <<22) );
	#else
		val = HWREG_R(SFT_CFG_1) & ~(0x7<<22);
		HWREG_W(SFT_CFG_1, val | (0x3<<22));
	#endif

	/* set SMI Freq Div */
	HWREG_W(SMI_FREQ_DIV, 0x50);

	HWREG_W(EPHY_CTRL_0, 0x0E8C);
	HWREG_W(EPHY_CTRL_1, 0x0BD4);

	mdio = phydev->bus->priv;

#if 0
	mdio->write_reg(mdio->mii_bus, 1, 0x15, 0x1704);
	mdio->write_reg(mdio->mii_bus, 1, 0x14, 0x60E0);
	mdio->write_reg(mdio->mii_bus, 2, 0x10, 0x01F5);
	mdio->write_reg(mdio->mii_bus, 3, 0x15, 0x0010);
	mdio->write_reg(mdio->mii_bus, 3, 0x18, 0x20A4);
	mdio->write_reg(mdio->mii_bus, 1, 0x1A, 0x0111);

	mdio->write_reg(mdio->mii_bus, 3, 0x17, 0x18A5);
	mdio->write_reg(mdio->mii_bus, 0, 0x13, 0x003F);
	mdio->write_reg(mdio->mii_bus, 1, 0x1B, 0x6B00);

	mdio->write_reg(mdio->mii_bus, 1, 0x1D, 0x8584);
	mdio->write_reg(mdio->mii_bus, 1, 0x1C, 0x0655);
	mdio->write_reg(mdio->mii_bus, 1, 0x1E, 0x0300);
#endif

	return 0;
}

static void sp483_remove(struct phy_device *phydev)
{
	struct device *dev = &phydev->dev;
	struct sp483_priv *priv = phydev->priv;

	if (priv)
		devm_kfree(dev, priv);
}

static struct phy_driver sp483_phy_driver[] = {
{
	.phy_id			= 0x02228002,
	.phy_id_mask	= 0xfffffff0,
	.name			= "SP483",
	.features		= PHY_BASIC_FEATURES | SUPPORTED_MII,
	.flags			= PHY_IS_INTERNAL,

	.soft_reset		= sp483_soft_reset,

	.config_init	= sp483_config_init,
	.config_aneg	= sp483_config_aneg,
	.aneg_done		= sp483_aneg_done,
	.read_status	= sp483_read_status,

	.resume			= sp483_resume,
	.suspend		= sp483_suspend,

	.probe			= sp483_probe,
	.remove			= sp483_remove,

	.driver	= { .owner = THIS_MODULE, }
} };

module_phy_driver(sp483_phy_driver);

static struct mdio_device_id __maybe_unused sp483_tbl[] = {
	{ 0x02228002, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, sp483_tbl);

MODULE_AUTHOR("Team-Proton <dev.team.proton@gmail.com>");
MODULE_DESCRIPTION("Sunplus SP483 PHY driver");
MODULE_LICENSE("GPL");
