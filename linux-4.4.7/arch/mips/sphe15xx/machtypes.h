/*
 *  Sunplus SPHE15XX machine type definitions
 *
 * Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#ifndef _SPHE15XX_MACHTYPE_H
#define _SPHE15XX_MACHTYPE_H

#include <asm/mips_machine.h>

enum sphe15xx_mach_type {
	SPHE15XX_MACH_GENERIC_OF = -1,	/* Device tree board */
	SPHE15XX_MACH_GENERIC = 0,
	SPHE15XX_MACH_BRAVE_BR10,			/* Sunplus 1512 brave br10 led reference board */
	SPHE15XX_MACH_BRAVE_BR11,			/* Sunplus 1512 brave br11 vfd reference board */
};

#endif /* _SPHE15XX_MACHTYPE_H */
