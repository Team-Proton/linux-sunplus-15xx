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

#include <linux/module.h>
#include <linux/gpio.h>

#include <asm/mach-sphe15xx/sphe15xx.h>

struct sphe_gpio_chip {
	void __iomem		*regs;
	struct gpio_chip	chip;
};

int __init sphe_gpio_init(void)
{
	int ret;
	//printk(KERN_INFO "XX: registered nn GPIOs\n");

	ret = 0;
	return ret;
}
