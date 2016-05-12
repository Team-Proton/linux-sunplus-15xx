/*
 * Sunplus SPHE15XX EMAC MDIO bus controller driver
 *
 * Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>

#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/platform.h>

//define MDIO command bit
#define MDIO_WT_DATA_MASK		0xffff0000
#define MDIO_RD_CMD				0x00004000
#define MDIO_WT_CMD				0x00002000
#define MDIO_REG_MASK			0x00001f00
#define MDIO_ADR_MASK			0x0000001f

//define MDIO status bit
#define MDIO_RD_DATA_MASK		0xffff0000
#define MDIO_RD_RDY				2
#define MDIO_WT_DONE			1

// registers
#define MDIO_REG_CMD			0x00
#define MDIO_REG_STATUS			0x04

#define MAC_MDIO_ADDR			0x00    /*define by hardware*/
#define MDIO_TIMEOUT			(msecs_to_jiffies(100))

static int sphe15xx_mdio_read(struct mii_bus *bus, int addr, int reg)
{
	unsigned int value;
	unsigned long timeout_jiffies;
	struct sphe15xx_mdio_priv *priv = bus->priv;

	printk("@@@ --> mdio_read(>)\n");

	HWREG_W(priv->base + MDIO_REG_CMD, (((reg<<8) & MDIO_REG_MASK) | MDIO_RD_CMD | MAC_MDIO_ADDR));

	/* Wait read complete */
	while (((value = HWREG_R(priv->base + MDIO_REG_STATUS)) & MDIO_RD_RDY) == 0) {
		if (time_is_before_jiffies(timeout_jiffies))
			return -ETIMEDOUT;
		msleep(1);
	}

	printk("@@@ --> mdio_read(<) addr: 0x%x, reg: 0x%x, value: 0x%04x\n", addr, reg, value >> 16);

	return (value >> 16);
}

static int sphe15xx_mdio_write(struct mii_bus *bus, int addr, int reg, unsigned short value)
{
	unsigned long timeout_jiffies;
	struct sphe15xx_mdio_priv *priv = bus->priv;

	printk("@@@ --> mdio_write() addr: 0x%x, reg: 0x%x, value: 0x%04x\n", addr, reg, value);

	HWREG_W(priv->base + MDIO_REG_CMD, (((value << 16) & MDIO_WT_DATA_MASK) | MDIO_WT_CMD | ((reg<<8) & MDIO_REG_MASK) | MAC_MDIO_ADDR));

	/* Wait write complete */
	timeout_jiffies = jiffies + MDIO_TIMEOUT;
	while (((HWREG_R(priv->base + MDIO_REG_STATUS)) & MDIO_WT_DONE) == 0) {
		if (time_is_before_jiffies(timeout_jiffies))
			return -ETIMEDOUT;
		msleep(1);
	}

	return 0;
}

static int sphe15xx_mdio_read_reg(struct mii_bus *bus, int page, int reg)
{
	unsigned int reg_val;
	unsigned int page_ctrl = 0;
	
	/*  Set page  */
	page_ctrl = sphe15xx_mdio_read(bus, 0, 31) & 0x3fff;
	sphe15xx_mdio_write(bus, 0, 31, page_ctrl | (page & 0x3) << 14);
	
	/*  Read data  */
	reg_val = sphe15xx_mdio_read(bus, 0, reg);

	/*  Switch to page 0 */
	sphe15xx_mdio_write(bus, 0, 31, page_ctrl);
	return reg_val;
}

static int sphe15xx_mdio_write_reg(struct mii_bus *bus, int page, int reg, unsigned short val)
{
	unsigned int page_ctrl = 0;
	
	/*  Set page  */
	page_ctrl = sphe15xx_mdio_read(bus, 0, 0x1f) & 0x3fff;
	sphe15xx_mdio_write(bus, 0, 0x1f, page_ctrl | (page & 0x3) << 14);
	
	/*  Write data  */
	sphe15xx_mdio_write(bus, 0, reg, val);

	if (reg == 0x1f)
		page_ctrl = val & 0x3fff;

	/*  Switch to page 0 */
	sphe15xx_mdio_write(bus, 0, 0x1f, page_ctrl);

	return 0;
}

static int sphe15xx_mdio_probe(struct platform_device *pdev)
{
	struct sphe15xx_mdio_priv *priv;
	struct sphe15xx_mdio_platform_data *pdata;
	struct device_node *np;
	struct mii_bus *bus;
	struct resource *res;
	int ret;

	np = pdev->dev.of_node;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "no platform data specified\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(res)) {
		dev_err(&pdev->dev, "failed to get mdio resource\n");
		return -ENOENT;
	}

	priv->membase = devm_ioremap_nocache(&pdev->dev, res->start, resource_size(res));
	if (!priv->membase) {
		dev_err(&pdev->dev, "failed to remap mdio register\n");
		return -ENOMEM;
	}

	priv->base = res->start;
	priv->pdata = pdata;
	priv->mii_bus = mdiobus_alloc();
	if (!priv->mii_bus)
		return -ENOMEM;

	bus = priv->mii_bus;
	bus->priv = priv;
	bus->name = "sphe15xx_mii_bus";
	bus->parent = &pdev->dev;
	bus->phy_mask = pdata->phy_mask;
	bus->read = sphe15xx_mdio_read;
	bus->write = sphe15xx_mdio_write;
	priv->read_reg = sphe15xx_mdio_read_reg;
	priv->write_reg = sphe15xx_mdio_write_reg;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s", pdev->name);

	bus->irq = kcalloc(PHY_MAX_ADDR, sizeof(int), GFP_KERNEL);
	if (!bus->irq) {
		ret = -ENOMEM;
		goto out_mdio_free;
	}

	ret = mdiobus_register(bus);
	if (ret) {
		dev_err(&pdev->dev, "MDIO bus registration failed\n");
		goto out_mdio_irq;
	}

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "Sunplus EMAC MDIO bus at 0x%p\n", priv->membase);

	pdata->mii_bus = bus;
	return 0;

out_mdio_irq:
	kfree(bus->irq);
out_mdio_free:
	mdiobus_free(bus);
	return ret;
}

static int sphe15xx_mdio_remove(struct platform_device *pdev)
{
	struct sphe15xx_mdio_priv *priv = platform_get_drvdata(pdev);

	mdiobus_unregister(priv->mii_bus);
	kfree(priv->mii_bus->irq);
	mdiobus_free(priv->mii_bus);

	return 0;
}

static const struct of_device_id sphe15xx_mdio_ids[] = {
	{ .compatible = "sunplus,sphe1502-mdio", },
	{ .compatible = "sunplus,sphe1506-mdio", },
	{ .compatible = "sunplus,sphe1512-mdio", },
	{ .compatible = "sunplus,sphe15xx-mdio", },
	{  },
};
MODULE_DEVICE_TABLE(of, sphe15xx_mdio_ids);

static struct platform_driver sphe15xx_mdio_driver = {
	.driver = {
		.name = "sphe15xx-mdio",
		.of_match_table = sphe15xx_mdio_ids,
	},
	.probe	= sphe15xx_mdio_probe,
	.remove	= sphe15xx_mdio_remove,
};
module_platform_driver(sphe15xx_mdio_driver);

MODULE_AUTHOR("Team-Proton <dev.team.proton@gmail.com>");
MODULE_DESCRIPTION("Sunplus EMAC MDIO bus controller");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sphe15xx-mdio");
