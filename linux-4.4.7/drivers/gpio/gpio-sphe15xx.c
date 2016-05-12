/*
 *  Sunplus SPHE15XX GPIO API support
 *
 *  Copyright (C) 2016 Team-Proton
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <linux/gpio/driver.h>
#include <linux/platform_data/gpio-sphe15xx.h>
#include <linux/of_device.h>

#include <asm/mach-sphe15xx/gpio.h>

struct sphe15xx_gpio_ctrl {
	struct gpio_chip chip;
	void __iomem *membase;
	unsigned int base;
	spinlock_t lock;
};

#define to_sphe15xx_gpio_ctrl(c) container_of(c, struct sphe15xx_gpio_ctrl, chip)

static void sphe15xx_gpio_set_value(struct gpio_chip *chip, unsigned gpio, int value)
{
	struct sphe15xx_gpio_ctrl *ctrl = to_sphe15xx_gpio_ctrl(chip);

	if (value) {
		HWREG_W(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_OUT,(gpio/32)),
		HWREG_R(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_OUT,(gpio/32))) | (1<<(gpio%32)));
	} else {
		HWREG_W(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_OUT,(gpio/32)),
		HWREG_R(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_OUT,(gpio/32))) & ~(1<<(gpio%32)));
	}
}

static int sphe15xx_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	struct sphe15xx_gpio_ctrl *ctrl = to_sphe15xx_gpio_ctrl(chip);

	return ((HWREG_R(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_IN,(gpio/32))) >> (gpio%32)) & 0x01);
}

static void sphe15xx_gpio_set_first(struct gpio_chip *chip, unsigned gpio, int value)
{
	struct sphe15xx_gpio_ctrl *ctrl = to_sphe15xx_gpio_ctrl(chip);

	if (value) {
		HWREG_W(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_FIRST,(gpio/32)),
		HWREG_R(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_FIRST,(gpio/32))) | (1<<(gpio%32)));
	} else {
		HWREG_W(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_FIRST,(gpio/32)),
		HWREG_R(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_FIRST,(gpio/32))) & ~(1<<(gpio%32)));
	}
}

static void sphe15xx_gpio_set_master(struct gpio_chip *chip, unsigned gpio, int value)
{
	struct sphe15xx_gpio_ctrl *ctrl = to_sphe15xx_gpio_ctrl(chip);

	if (value) {
		HWREG_W(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_MASTER,(gpio/32)),
		HWREG_R(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_MASTER,(gpio/32))) | (1<<(gpio%32)));
	} else {
		HWREG_W(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_MASTER,(gpio/32)),
		HWREG_R(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_MASTER,(gpio/32))) & ~(1<<(gpio%32)));
	}
}

static void sphe15xx_gpio_out_enable(struct gpio_chip *chip, unsigned gpio, int value)
{
	struct sphe15xx_gpio_ctrl *ctrl = to_sphe15xx_gpio_ctrl(chip);

#if (IC_VER == QCE491)
	if (gpio == 15) {
		if (value) {
			HWREG_W(ctrl->base + SPHE15XX_GPIO_REG_CTRL,
			HWREG_R(SPHE15XX_GPIO_REG_CTRL) & ~(1<<22));
		} else {
			HWREG_W(ctrl->base + SPHE15XX_GPIO_REG_CTRL,
			HWREG_R(SPHE15XX_GPIO_REG_CTRL) | (1<<22));

			HWREG_W(ctrl->base + SPHE15XX_GPIO_REG_CTRL,
			HWREG_R(SPHE15XX_GPIO_REG_CTRL) | 0xf);
		}
	}
#endif

	if (value) {
		HWREG_W(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_OE,(gpio/32)),
		HWREG_R(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_OE,(gpio/32))) | (1<<(gpio%32)));
	} else {
		HWREG_W(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_OE,(gpio/32)),
		HWREG_R(ctrl->base + ADDR_OF(SPHE15XX_GPIO_REG_OE,(gpio/32))) & ~(1<<(gpio%32)));
	}
}

static int sphe15xx_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	struct sphe15xx_gpio_ctrl *ctrl = to_sphe15xx_gpio_ctrl(chip);
	unsigned long flags;

	spin_lock_irqsave(&ctrl->lock, flags);

	sphe15xx_gpio_set_first (chip, gpio, 1);
	sphe15xx_gpio_set_master(chip, gpio, 1);

	spin_unlock_irqrestore(&ctrl->lock, flags);

	return 0;
}

static int sphe15xx_gpio_direction_output(struct gpio_chip *chip, unsigned gpio, int value)
{
	struct sphe15xx_gpio_ctrl *ctrl = to_sphe15xx_gpio_ctrl(chip);
	unsigned long flags;

	spin_lock_irqsave(&ctrl->lock, flags);

	sphe15xx_gpio_set_first (chip, gpio, 1);
	sphe15xx_gpio_set_master(chip, gpio, 1);
	sphe15xx_gpio_out_enable(chip, gpio, 1);
	sphe15xx_gpio_set_value (chip, gpio, value);

	spin_unlock_irqrestore(&ctrl->lock, flags);

	return 0;
}

static const struct gpio_chip sphe15xx_gpio_chip = {
	.label			= "sphe15xx",
	.get			= sphe15xx_gpio_get_value,
	.set			= sphe15xx_gpio_set_value,
	.direction_input	= sphe15xx_gpio_direction_input,
	.direction_output	= sphe15xx_gpio_direction_output,
	.base			= 0,
};

static int sphe15xx_gpio_probe(struct platform_device *pdev)
{
	struct sphe15xx_gpio_platform_data *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	struct sphe15xx_gpio_ctrl *ctrl;
	struct resource *res;
	u32 sphe15xx_gpio_count;
	bool oe_inverted;
	int err;

	ctrl = devm_kzalloc(&pdev->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	if (np) {
		err = of_property_read_u32(np, "ngpios", &sphe15xx_gpio_count);
		if (err) {
			dev_err(&pdev->dev, "ngpios property is not valid\n");
			return err;
		}
		if (sphe15xx_gpio_count >= 216) {
			dev_err(&pdev->dev, "ngpios must be less than 216\n");
			return -EINVAL;
		}
		oe_inverted = of_device_is_compatible(np, "sphe,sphe15xx-gpio");
	} else if (pdata) {
		sphe15xx_gpio_count = pdata->ngpios;
		oe_inverted = pdata->oe_inverted;
	} else {
		dev_err(&pdev->dev, "No DT node or platform data found\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctrl->membase = devm_ioremap_nocache(&pdev->dev, res->start, resource_size(res));
	if (!ctrl->membase) {
		dev_err(&pdev->dev,
				"Error allocate gpio memory resources!\n");
		return -ENOMEM;
	}

	ctrl->base = res->start;

	spin_lock_init(&ctrl->lock);

	memcpy(&ctrl->chip, &sphe15xx_gpio_chip, sizeof(ctrl->chip));
	ctrl->chip.dev = &pdev->dev;
	ctrl->chip.ngpio = sphe15xx_gpio_count;

	err = gpiochip_add(&ctrl->chip);
	if (err) {
		dev_err(&pdev->dev,
			"cannot add SPHE15XX GPIO chip, error=%d", err);
		return err;
	}

	printk(KERN_INFO "%s: count = %d pin\n",
			dev_name(&pdev->dev), sphe15xx_gpio_count);

	return 0;
}

static int sphe15xx_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id sphe15xx_gpio_of_match[] = {
	{ .compatible = "sphe,sphe1506-gpio" },
	{ .compatible = "sphe,sphe1512-gpio" },
	{ .compatible = "sphe,sphe15xx-gpio" },
	{},
};

MODULE_DEVICE_TABLE(of, sphe15xx_gpio_of_match);

static struct platform_driver sphe15xx_gpio_driver = {
	.driver = {
		.name = "sphe15xx-gpio",
		.of_match_table	= sphe15xx_gpio_of_match,
	},
	.probe = sphe15xx_gpio_probe,
	.remove = sphe15xx_gpio_remove,
};

module_platform_driver(sphe15xx_gpio_driver);

MODULE_AUTHOR("Team-Proton");
MODULE_DESCRIPTION("Driver for Sunplus SPHE15XX SoC GPIO");
MODULE_LICENSE("GPL v2");
