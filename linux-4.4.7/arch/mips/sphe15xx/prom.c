/*
 * Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * Putting things on the screen/serial line using YAMONs facilities.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/serial_reg.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <asm/bootinfo.h>
#include <asm/mach-sphe15xx/sphe15xx.h>
#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/uart.h>

#ifdef __CDT_PARSER__
#define __init
#endif

int __init sphe_gpio_init(void);
int __init get_sys_bus_freq(void);
void sphe_sys_init(void);

int prom_putchar(char c);

static char ppbuf[1024];

asmlinkage void prom_printf(char *fmt, ...)
{
	va_list args;
	char ch, *bptr;
	int i;

	va_start(args, fmt);

#ifdef CONFIG_KGDB
	ppbuf[0] = 'O';
	i = vsprintf(ppbuf + 1, fmt, args) + 1;
#else
	i = vsprintf(ppbuf, fmt, args);
#endif

	bptr = ppbuf;

#ifdef CONFIG_KGDB
	if (kgdb_initialized) {
		printk("kgdb_initialized = %d\n", kgdb_initialized);
		putpacket(bptr, 1);
	} else
#else
	while((ch = *(bptr++)) != 0) {
		if(ch == '\n')
			prom_putchar('\r');

		prom_putchar(ch);
	}
#endif
	va_end(args);
	return;
}
EXPORT_SYMBOL(prom_printf);

static void  __init sphe_init_cmdline(int argc, char *argv[])
{
	int i;

	for (i = 1; i < argc; i++) {
		strlcat(arcs_cmdline, argv[i], COMMAND_LINE_SIZE);
		if (i < (argc - 1))
			strlcat(arcs_cmdline, " ", COMMAND_LINE_SIZE);
	}
}

static void __init sphe_uart_init(void)
{
	int baud = BAUD(115200);
	UART0_set_baudrate(baud);
}

void __init prom_init(void)
{
	sphe_init_cmdline(fw_arg0, (char **)fw_arg1);

	sphe_uart_init();
	sphe_sys_init();
	sphe_gpio_init();
}

int prom_getchar(void)
{
	while(!UART0_rx_rdy())
		;
	return UART0_getc();
}

int prom_putchar(char c)
{
	UART0_putc(c);
	return 1;
}
