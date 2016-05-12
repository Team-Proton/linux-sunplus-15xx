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
#include <linux/bootmem.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pfn.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/swap.h>

#include <asm/bootinfo.h>
#include <asm/page.h>
#include <asm/sections.h>

#include <asm/mach-sphe15xx/sphe15xx.h>

#ifdef __CDT_PARSER__
#define __init
#endif

static int __init memsize(void)
{
	return SPHE_MEM_SIZE;
}

void __init prom_meminit(void)
{
	unsigned long pages;

	pages = memsize() >> PAGE_SHIFT;
	add_memory_region(0, pages << PAGE_SHIFT, BOOT_MEM_RAM);
}

void __init prom_free_prom_memory(void)
{
}
