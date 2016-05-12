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

#ifndef __SPHE15XX_GPIO_H__
#define __SPHE15XX_GPIO_H__

#include <asm/mach-sphe15xx/board.h>
#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/irq.h>
#include <asm/mach-sphe15xx/gpio_pin_map_1502.h>
#include <asm/mach-sphe15xx/brxx_pinmux.h>

#define SPHE15XX_GPIO_MAX 	216
#define NR_BUILTIN_GPIO 	SPHE15XX_GPIO_MAX

#define gpio_to_irq(gpio)	-1

#define gpio_get_value __gpio_get_value
#define gpio_set_value __gpio_set_value

#define gpio_cansleep __gpio_cansleep

#define SPHE15XX_GPIO_COUNT				216
#define SPHE15XX_GPIO_BASE				0xBFFE0200

#define SPHE15XX_GPIO_REG_CTRL			0x00 // PAD_CTRL
#define SPHE15XX_GPIO_REG_FIRST			0x6C
#define SPHE15XX_GPIO_REG_MASTER		0x80
#define SPHE15XX_GPIO_REG_OE			0xA0
#define SPHE15XX_GPIO_REG_IN			0xC0
#define SPHE15XX_GPIO_REG_OUT			0xE0

/********************************************************************
	Parameters for GPIO operations
********************************************************************/
#define GPIO_ON			1
#define GPIO_OFF		0

#define GPIO_OUTPUT		1
#define GPIO_INPUT		0

#define GPIO_RISC_CTL	1
#define GPIO_IOP_CTL	0

// GPIO macro
#define GPIO_F_SET(a,d) do { \
								if (d) { \
									HWREG_W(ADDR_OF(GPIO_FIRST,(a/32)), HWREG_R(ADDR_OF(GPIO_FIRST,(a/32)))|(1<<(a%32))); \
								} else { \
									HWREG_W(ADDR_OF(GPIO_FIRST,(a/32)), HWREG_R(ADDR_OF(GPIO_FIRST,(a/32)))& ~(1<<(a%32))); \
								} \
						} while (0)

#define GPIO_M_SET(a,d) do { \
								if (d) { \
									HWREG_W(ADDR_OF(GPIO_MASTER,(a/32)), HWREG_R(ADDR_OF(GPIO_MASTER,(a/32)))|(1<<(a%32))); \
								} else { \
									HWREG_W(ADDR_OF(GPIO_MASTER,(a/32)), HWREG_R(ADDR_OF(GPIO_MASTER,(a/32)))&~(1<<(a%32))); \
								} \
						} while (0)

#if (IC_VER==QCE491)
#define GPIO_E_SET(a,d) do { \
								if(a==15) { \
									if(d==0) { \
										HWREG_W(PAD_CTRL, (HWREG_R(PAD_CTRL)|(1<<22))); \
										HWREG_W(PAD_CTRL, (HWREG_R(PAD_CTRL)|0xf)); \
									} \
									else\
										HWREG_W(PAD_CTRL, (HWREG_R(PAD_CTRL)&~(1<<22))); \
								} \
								if (d) {\
									HWREG_W(ADDR_OF(GPIO_OE, (a/32)), HWREG_R(ADDR_OF(GPIO_OE, (a/32)))|(1<<(a%32))); \
                            	} else  {\
                            		HWREG_W(ADDR_OF(GPIO_OE, (a/32)), HWREG_R(ADDR_OF(GPIO_OE, (a/32)))&~(1<<(a%32))); \
                            	}\
						} while (0)
#else
#define GPIO_E_SET(a,d) do {
								if (d) { \
									HWREG_W(ADDR_OF(GPIO_OE, (a/32)), HWREG_R(ADDR_OF(GPIO_OE, (a/32)))|(1<<(a%32))); \
								} else {
									HWREG_W(ADDR_OF(GPIO_OE, (a/32)), HWREG_R(ADDR_OF(GPIO_OE, (a/32)))&~(1<<(a%32))); \
								} \
						} while (0)
#endif

#define GPIO_O_SET(a,d) do { \
								if (d) { \
									HWREG_W(ADDR_OF(GPIO_OUT,(a/32)), HWREG_R(ADDR_OF(GPIO_OUT,(a/32)))|(1<<(a%32))); \
								} else { \
									HWREG_W(ADDR_OF(GPIO_OUT,(a/32)), HWREG_R(ADDR_OF(GPIO_OUT,(a/32)))&~(1<<(a%32))); \
								} \
						} while (0)

#define GPIO_I_GET(a)   ((HWREG_R(ADDR_OF(GPIO_IN,(a/32))) >> (a%32)) & 0x01)


/* Board specific GPIO functions */
int sphe_gpio_enable(unsigned gpio);
int sphe_gpio_disable(unsigned gpio);

#include <asm-generic/gpio.h>

#endif
