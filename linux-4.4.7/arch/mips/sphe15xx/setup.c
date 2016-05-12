/*
 * Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 * Copyright (C) 2000 MIPS Technologies, Inc.  All rights reserved.
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
 */
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/pm.h>
#include <linux/time.h>
#include <linux/libfdt.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>

#include <asm/traps.h>
#include <asm/setup.h>
#include <asm/reboot.h>
#include <asm/mipsregs.h>
#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/irq.h>
#include <asm/mach-sphe15xx/prom.h>
#include <asm/mach-sphe15xx/gpio.h>
#include <asm/mach-sphe15xx/clock.h>

#include "machtypes.h"

#ifdef __CDT_PARSER__
#define __init
#endif

unsigned int cyg_hal_clock_period = 0;

static void sphe_machine_restart(char *command)
{
	HWREG_W(RESET, 1);
	while(1);
}

static void sphe_machine_halt(void)
{
	while (1)
		;
}

static void sphe_machine_power_off(void)
{
	HWREG_W(IOP_DATA, HWREG_R(IOP_DATA) |(1<<11));
	sphe_machine_halt();
}

#if 0
static void sphe_base_setup(void)
{
	ebase = SPHE_MEM_BASE;
	write_c0_hwrena(0);
	write_c0_intctl(0x20);

	printk("$$ base setup done.\n");
}
#endif

const char *get_system_type(void)
{
	return "Sunplus SPHE15XX";
}

u8 sphe_chip_id(void)
{
	return 0x15;
}

u8 sphe_chip_rev(void)
{
	return 0x12;
}

static int __init sphe_init_console(void)
{
	return 0;
}
console_initcall(sphe_init_console);

/*
 * Initializes basic routines and structures pointers, memory size (as
 * given by the bios and saves the command line.
 */
void __init plat_mem_setup(void)
{
	unsigned long io_base = 0;

	_machine_restart = sphe_machine_restart;
	_machine_halt = sphe_machine_halt;
	pm_power_off = sphe_machine_power_off;

	io_base = (unsigned long)ioremap(SPHE_REGS_BASE, 0x10000);

	if (!io_base)
		panic("Can't remap IO base!");
	set_io_port_base(io_base);

	//board_ebase_setup = &sphe_base_setup;

	prom_meminit();

	printk(KERN_INFO "SoC: %s (%02X%02X)\n",
			get_system_type(), sphe_chip_id(), sphe_chip_rev());
}


#define TEMP_DELAY()	asm volatile("nop;")

static const unsigned int l2c_size_config[8] = {128, 64, 32, 16, 8, 4, 2, 1};

static void GL_L2cInvalidateA(void)
{
	HWREG_W(L2C0_RGST_ACC_TAG, HWREG_R(L2C0_RGST_ACC_TAG) | (1 << 12));
	while (((HWREG_R(L2C0_RGST_ACC_TAG) & (1 << 13)) >> 13) != 1) {
		continue;
	}
	HWREG_W(L2C0_RGST_ACC_TAG, HWREG_R(L2C0_RGST_ACC_TAG) & ~(1 << 12));
}

static void GL_L2cSizeA(unsigned int size)
{
	unsigned int setting = 0;
	unsigned int wdog_ctl_l2c;

	if (size == 0 || size > 128) {
		size = 128;
	}
	size = (128 / size) - 1;
	while (size != 0) {
		size >>= 1;
		setting += 1;
	}

	/* Disable L2C.  */
	wdog_ctl_l2c = HWREG_R(CBUS_WDOG_CTL) & 0x0300;
	HWREG_W(CBUS_WDOG_CTL, HWREG_R(CBUS_WDOG_CTL) & ~(0x03 << 8));

	GL_L2cInvalidateA();

	/* Change L2C size.  */
	HWREG_W(L2C0_TYPE_CONFIG, (HWREG_R(L2C0_TYPE_CONFIG) & ~0x00F0) | ((setting & 0x07) << 4));

	/* Resotre L2C status.  */
	HWREG_W(CBUS_WDOG_CTL, HWREG_R(CBUS_WDOG_CTL) | wdog_ctl_l2c);
}

#define L2C_RAM_A_SIZE 64
#define L2C_RAM_B_SIZE 64

static void PLF_Init(void)
{
	// Disable Picture
	sphe15xx_clk_off(PNG);
	sphe15xx_clk_off(JR);
	sphe15xx_clk_off(JPG_IQT);
	sphe15xx_clk_off(HUFF);
	// Disable GPU
	sphe15xx_clk_off(GPU);

	sphe15xx_clk_off(DUMMY_MASTER);

	// default disable demux hw, the application should use demux init to
	// enable the clock
	sphe15xx_clk_off_ex(2, DEMUX, DEMUX2);

	// Note:
	// Set the l2c size for dram A and dram B
	GL_L2cSizeA(L2C_RAM_A_SIZE);
}

enum {
	WATCHDOG_CMD_CNT_WR_UNLOCK	= 0xAB00,
	WATCHDOG_CMD_CNT_WR_LOCK	= 0xAB01,
	WATCHDOG_CMD_CNT_WR_MAX		= 0xDEAF,
	WATCHDOG_CMD_PAUSE			= 0x3877,
	WATCHDOG_CMD_RESUME			= 0x4A4B,
	WATCHDOG_CMD_INTR_CLR		= 0x7482,
};

static int PLF_WatchdogInit(unsigned int count)
{
	if (~0 == count) {
		count = 0xFFFF;
	} else {
		/* Tranform count into STC clock.  */
		count = ((count & 0xFFFF) * 90) >> 4;
	}
	HWREG_W(TIMERW_CTRL, WATCHDOG_CMD_CNT_WR_UNLOCK);
	HWREG_W(TIMERW_CNT, count);

	return 0;
}

static void PLF_WatchdogResume(void)
{
	HWREG_W(TIMERW_CTRL, WATCHDOG_CMD_RESUME);
}

static void PLF_WatchdogSuspend(void)
{
	HWREG_W(TIMERW_CTRL, WATCHDOG_CMD_PAUSE);
}

int get_sys_bus_freq(void);
void sphe_init_clocks(void);

void sphe_sys_init(void)
{
	int temp_a=0;

	sphe_init_clocks();

	temp_a=HWREG_R(SD0_MON4_START_ADDR);

	PLF_Init();
	PLF_WatchdogSuspend();
	PLF_WatchdogInit(~0);
	PLF_WatchdogResume();

	/* Enable Watchdog Reset Counter in STC module. Enable this to trigger system reset. */
	HWREG_W(SFT_CFG_8, HWREG_R(SFT_CFG_8) | (1 << 12));

	HWREG_W(ADDR_OF(PAD_CTRL,12), 0x14000000);

	HWREG_R(STC_DIVISOR) = 0x8095;
	HWREG_R(STC_CONFIG) = 0;
    HWREG_R(STC_32) = 0; HWREG_R(STC_31_16) = 0; HWREG_R(STC_15_0) = 0;
}

void __init device_tree_init(void)
{
	unflatten_and_copy_device_tree();
}

static void __init sphe15xx_generic_init(void)
{
	/* Nothing to do */
}

MIPS_MACHINE(SPHE15XX_MACH_GENERIC,
	     "Generic",
	     "Generic SPHE15XX based board",
	     sphe15xx_generic_init);
