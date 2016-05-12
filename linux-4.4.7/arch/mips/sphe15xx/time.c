/*
 * Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 * Copyright (C) 1999,2000 MIPS Technologies, Inc.  All rights reserved.
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
 * Setting up the clock on the MIPS boards.
 */

#include <linux/init.h>
#include <linux/time.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <asm/time.h>
#include <asm/setup.h>
#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>

#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/irq.h>
#include <asm/mach-sphe15xx/uart.h>

#ifdef __CDT_PARSER__
#define __init
#endif

static cycle_t sphe_stc_read(struct clocksource *cs)
{
	return((u64)HWREG_R(STC_32)  << 32) | (HWREG_R(STC_31_16)  << 16) | HWREG_R(STC_15_0);
}

static struct clocksource clocksource_sphe15xx = {
	.name	= "STC",
	.rating	= 320,
	.read	= sphe_stc_read,
	.mask	= CLOCKSOURCE_MASK(32),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

int __init init_clocksource(void)
{
	u32 hz = 100000000;
	clocksource_register_hz(&clocksource_sphe15xx, hz);

	/* Set MIPS counter frequency for fixed_rate_gettimeoffset() */
	mips_hpt_frequency = hz;
	return 0;
}

static irqreturn_t sphe_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *cd = dev_id;

	HAL_INTERRUPT_ACKNOWLEDGE(irq);

	cd->event_handler(cd);
	return IRQ_HANDLED;
}

struct clock_event_device sphe_clockevent = {
	.name		= "sphe15xx",
	.features	= CLOCK_EVT_FEAT_PERIODIC,
	.rating		= 310,
	.irq		= SPHE15XX_TIMER_IRQ,
	.cpumask	= cpu_all_mask,
};

static struct irqaction sphe_timer_irqaction = {
	.handler	= sphe_timer_interrupt,
	.flags		= IRQF_PERCPU | IRQF_TIMER,
	.name		= "timer",
	.dev_id		= &sphe_clockevent,
};

int __init init_clockevents(void)
{
	struct clock_event_device *cd = &sphe_clockevent;
	struct irqaction *action = &sphe_timer_irqaction;
	unsigned int cpu = smp_processor_id();

	cd->cpumask	= cpumask_of(cpu);
	clockevents_register_device(cd);
	action->dev_id = cd;

	setup_irq(SPHE15XX_TIMER_IRQ, action);

	return 0;
}

void __init plat_time_init(void)
{
	//init_clocksource();
	//init_clockevents();

	//cp0_compare_irq = 7;

	HWREG_R(STC_32) = 0; HWREG_R(STC_31_16) = 0; HWREG_R(STC_15_0) = 0;
	/* to generate the first CPU timer interrupt */
	//write_c0_compare(read_c0_count() + mips_hpt_frequency / HZ);
	//local_irq_enable();
}
