/*
 * SPI controller driver for the Sunplus SPHE15XX SoCs
 *
 * Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 * This driver has been based on the spi-gpio.c:
 *	Copyright (C) 2006,2008 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/sphe15xx.h>
#include <asm/mach-sphe15xx/spi.h>

#define DRV_NAME	"sunplus-spi"

#ifdef __CDT_PARSER__
#define __init
#define __exit
#endif

#define SPHE15XX_SPI_RRW_DELAY_FACTOR	12000
#define MHZ				(1000 * 1000)

struct sphe15xx_spi {
	struct spi_bitbang	bitbang;
	u32			oneBit;
	u32			readCmd;
	u32			reg_ctrl;
	u32			base;
	struct clk	*clk;
	unsigned	rrw_delay;
	unsigned char __iomem	*membase;		/* read/write[bwl] */
};

static inline u32 sphe15xx_spi_rr(struct sphe15xx_spi *sp, unsigned reg)
{
	return HWREG_R(sp->base + reg);
}

static inline void sphe15xx_spi_wr(struct sphe15xx_spi *sp, unsigned reg, u32 val)
{
	HWREG_W(sp->base + reg, val);
}

static inline struct sphe15xx_spi *sphe15xx_spidev_to_sp(struct spi_device *spi)
{
	return spi_master_get_devdata(spi->master);
}

static inline void sphe15xx_spi_delay(struct sphe15xx_spi *sp, unsigned nsecs)
{
	if (nsecs > sp->rrw_delay)
		ndelay(nsecs - sp->rrw_delay);
}

static void sphe15xx_spi_chipselect(struct spi_device *spi, int is_active)
{
#if 0
	struct sphe15xx_spi *sp = sphe15xx_spidev_to_sp(spi);
	int cs_high = (spi->mode & SPI_CS_HIGH) ? is_active : !is_active;

	if (is_active) {
		/* set initial clock polarity */
		if (spi->mode & SPI_CPOL)
			sp->ioc_base |= AR71XX_SPI_IOC_CLK;
		else
			sp->ioc_base &= ~AR71XX_SPI_IOC_CLK;

		sphe15xx_spi_wr(sp, AR71XX_SPI_REG_IOC, sp->ioc_base);
	}

	if (spi->chip_select) {
		struct sphe15xx_spi_controller_data *cdata = spi->controller_data;

		/* SPI is normally active-low */
		gpio_set_value(cdata->gpio, cs_high);
	} else {
		if (cs_high)
			sp->ioc_base |= AR71XX_SPI_IOC_CS0;
		else
			sp->ioc_base &= ~AR71XX_SPI_IOC_CS0;

		sphe15xx_spi_wr(sp, AR71XX_SPI_REG_IOC, sp->ioc_base);
	}
#endif
}

static void sphe15xx_spi_enable(struct sphe15xx_spi *sp)
{
	/* enable GPIO mode */
	//sphe15xx_spi_wr(sp, AR71XX_SPI_REG_FS, AR71XX_SPI_FS_GPIO);

	/* save CTRL register */
	//sp->reg_ctrl = sphe15xx_spi_rr(sp, SPI_CONTROL);
	//sp->ioc_base = sphe15xx_spi_rr(sp, AR71XX_SPI_REG_IOC);

	/* TODO: setup speed? */
	//sphe15xx_spi_wr(sp, AR71XX_SPI_REG_CTRL, 0x43);


	sp->oneBit = 1;
    RF_SPI_CFG3 |= 1;
    RF_SPI_WAIT = 0x0105;
}

static void sphe15xx_spi_disable(struct sphe15xx_spi *sp)
{
	/* restore CTRL register */
	//sphe15xx_spi_wr(sp, SPI_CONTROL, sp->reg_ctrl);
	/* disable GPIO mode */
	//sphe15xx_spi_wr(sp, AR71XX_SPI_REG_FS, 0);
}

static int sphe15xx_spi_setup_cs(struct spi_device *spi)
{
	int status = 0;
#if 0
	struct sphe15xx_spi_controller_data *cdata;

	cdata = spi->controller_data;
	if (spi->chip_select && !cdata)
		return -EINVAL;

	status = 0;
	if (spi->chip_select) {
		unsigned long flags;

		flags = GPIOF_DIR_OUT;
		if (spi->mode & SPI_CS_HIGH)
			flags |= GPIOF_INIT_LOW;
		else
			flags |= GPIOF_INIT_HIGH;

		status = gpio_request_one(cdata->gpio, flags,
					  dev_name(&spi->dev));
	}
#endif
	return status;
}

static void sphe15xx_spi_cleanup_cs(struct spi_device *spi)
{
	if (spi->chip_select) {
		struct sphe15xx_spi_controller_data *cdata = spi->controller_data;
		gpio_free(cdata->gpio);
	}
}

static int sphe15xx_spi_setup(struct spi_device *spi)
{
	int status = 0;

	if (!spi->controller_state) {
		status = sphe15xx_spi_setup_cs(spi);
		if (status)
			return status;
	}

	status = spi_bitbang_setup(spi);
	if (status && !spi->controller_state)
		sphe15xx_spi_cleanup_cs(spi);

	return status;
}

static void sphe15xx_spi_cleanup(struct spi_device *spi)
{
	sphe15xx_spi_cleanup_cs(spi);
	spi_bitbang_cleanup(spi);
}

static u32 sphe15xx_spi_txrx_mode0(struct spi_device *spi, unsigned nsecs,
			       u32 word, u8 bits)
{
	return 0;
}

static int sphe15xx_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct sphe15xx_spi *sp;
	struct sphe15xx_spi_platform_data *pdata;
	struct resource	*r;
	unsigned long rate;
	int ret;

	master = spi_alloc_master(&pdev->dev, sizeof(*sp));
	if (master == NULL) {
		dev_err(&pdev->dev, "failed to allocate spi master\n");
		return -ENOMEM;
	}

	sp = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, sp);

	pdata = dev_get_platdata(&pdev->dev);

	/* the mode bits supported by this driver */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST;
	master->bits_per_word_mask = SPI_BPW_MASK(32) | SPI_BPW_MASK(16) | SPI_BPW_MASK(8);
	master->setup = sphe15xx_spi_setup;
	master->cleanup = sphe15xx_spi_cleanup;
	if (pdata) {
		master->bus_num = pdata->bus_num;
		master->num_chipselect = pdata->num_chipselect;
	}

	sp->bitbang.master = master;
	sp->bitbang.chipselect = sphe15xx_spi_chipselect;
	sp->bitbang.txrx_word[SPI_MODE_0] = sphe15xx_spi_txrx_mode0;
	sp->bitbang.setup_transfer = spi_bitbang_setup_transfer;
	sp->bitbang.flags = SPI_CS_HIGH;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sp->membase = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(sp->membase)) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "failed to allocate spi memory\n");
		goto err_put_master;
	}

	sp->base = (u32)r->start;
	if (!sp->base) {
		ret = -ENXIO;
		dev_err(&pdev->dev, "failed to allocate spi reg base\n");
		goto err_put_master;
	}

	sp->clk = devm_clk_get(&pdev->dev, "spi");
	if (IS_ERR(sp->clk)) {
		ret = PTR_ERR(sp->clk);
		dev_err(&pdev->dev, "failed to get spi clk\n");
		goto err_put_master;
	}

	ret = clk_enable(sp->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable spi clk\n");
		goto err_put_master;
	}

	rate = DIV_ROUND_UP(clk_get_rate(sp->clk), MHZ);
	if (!rate) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "failed to round up spi clk\n");
		goto err_clk_disable;
	}

	sp->rrw_delay = SPHE15XX_SPI_RRW_DELAY_FACTOR / rate;
	dev_dbg(&pdev->dev, "register read/write delay is %u nsecs\n",
		sp->rrw_delay);

	sphe15xx_spi_enable(sp);
	ret = spi_bitbang_start(&sp->bitbang);
	if (ret) {
		dev_err(&pdev->dev, "failed to start spi\n");
		goto err_disable;
	}

	return 0;

err_disable:
	sphe15xx_spi_disable(sp);
err_clk_disable:
	clk_disable(sp->clk);
err_put_master:
	spi_master_put(sp->bitbang.master);

	return ret;
}

static int sphe15xx_spi_remove(struct platform_device *pdev)
{
	struct sphe15xx_spi *sp = platform_get_drvdata(pdev);

	spi_bitbang_stop(&sp->bitbang);
	sphe15xx_spi_disable(sp);
	clk_disable(sp->clk);
	spi_master_put(sp->bitbang.master);

	return 0;
}

static void sphe15xx_spi_shutdown(struct platform_device *pdev)
{
	sphe15xx_spi_remove(pdev);
}

static struct platform_driver sphe15xx_spi_driver = {
	.probe		= sphe15xx_spi_probe,
	.remove		= sphe15xx_spi_remove,
	.shutdown	= sphe15xx_spi_shutdown,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};
module_platform_driver(sphe15xx_spi_driver);

MODULE_DESCRIPTION("SPI controller driver for Sunplus sphe15xx");
MODULE_AUTHOR("Team-Proton <dev.team.proton@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
