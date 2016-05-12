/*
 *  Sunplus SPHE15XX GPIO controller platform data
 *
 * Copyright (C) 2016 Team-Proton
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_PLATFORM_DATA_GPIO_SPHE15XX_H
#define __LINUX_PLATFORM_DATA_GPIO_SPHE15XX_H

struct sphe15xx_gpio_platform_data {
	unsigned ngpios;
	bool oe_inverted;
};

#endif
