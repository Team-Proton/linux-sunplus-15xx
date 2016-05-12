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

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <asm/setup.h>
#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/irq.h>
#include <asm/mach-sphe15xx/uart.h>

#ifdef __CDT_PARSER__
#define __init
#endif

#define L0_BIT(int_vec) (1 << ((int_vec) - CYGNUM_HAL_INTERRUPT_LEVEL0_BASE))
#define L1_BIT(int_vec) (1 << ((int_vec) - CYGNUM_HAL_INTERRUPT_LEVEL1_BASE))


CYG_WORD32 hal_level0_mask[IRQ_L0_COUNT] = {

	RISC0_INTR5_MASK,

	RISC0_INTR3_MASK, RISC0_INTR3_MASK, RISC0_INTR3_MASK, RISC0_INTR3_MASK,
	RISC0_INTR3_MASK, RISC0_INTR3_MASK, RISC0_INTR3_MASK, RISC0_INTR3_MASK,
	RISC0_INTR3_MASK, RISC0_INTR3_MASK, RISC0_INTR3_MASK, RISC0_INTR3_MASK,
	RISC0_INTR3_MASK, RISC0_INTR3_MASK, RISC0_INTR3_MASK, RISC0_INTR3_MASK,

	RISC0_INTR4_MASK, RISC0_INTR4_MASK, RISC0_INTR4_MASK,
	RISC0_INTR4_MASK, RISC0_INTR4_MASK, RISC0_INTR4_MASK, RISC0_INTR4_MASK,
	RISC0_INTR4_MASK, RISC0_INTR4_MASK, RISC0_INTR4_MASK, RISC0_INTR4_MASK,
	RISC0_INTR4_MASK, RISC0_INTR4_MASK, RISC0_INTR4_MASK, RISC0_INTR4_MASK,
};

CYG_WORD32 hal_level1_mask[IRQ_L1_COUNT] = {

	RISC0_INTR0_MASK, RISC0_INTR0_MASK, RISC0_INTR0_MASK, RISC0_INTR0_MASK,
	RISC0_INTR0_MASK, RISC0_INTR0_MASK, RISC0_INTR0_MASK, RISC0_INTR0_MASK,

	RISC0_INTR1_MASK, RISC0_INTR1_MASK, RISC0_INTR1_MASK, RISC0_INTR1_MASK,
	RISC0_INTR1_MASK, RISC0_INTR1_MASK, RISC0_INTR1_MASK, RISC0_INTR1_MASK,
	RISC0_INTR1_MASK, RISC0_INTR1_MASK, RISC0_INTR1_MASK, RISC0_INTR1_MASK,
	RISC0_INTR1_MASK, RISC0_INTR1_MASK, RISC0_INTR1_MASK, RISC0_INTR1_MASK,
	RISC0_INTR1_MASK, RISC0_INTR1_MASK, RISC0_INTR1_MASK, RISC0_INTR1_MASK,

	RISC0_INTR2_MASK, RISC0_INTR2_MASK, RISC0_INTR2_MASK, RISC0_INTR2_MASK,
};

static void sphe_irq_ack(struct irq_data *d)
{
	HAL_INTERRUPT_ACKNOWLEDGE(d->irq);
}

static void sphe_irq_mask(struct irq_data *d)
{
	HAL_INTERRUPT_MASK(d->irq);
}

static void sphe_irq_unmask(struct irq_data *d)
{
	HAL_INTERRUPT_UNMASK(d->irq);
}

/*
 irq: 0x20, ipnum: 7, mask_addr: 0xBFFE04B0
 irq: 0x21, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x22, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x23, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x24, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x25, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x26, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x27, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x28, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x29, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x2a, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x2b, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x2c, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x2d, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x2e, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x2f, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x30, ipnum: 5, mask_addr: 0xBFFE04A8
 irq: 0x31, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x32, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x33, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x34, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x35, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x36, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x37, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x38, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x39, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x3a, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x3b, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x3c, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x3d, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x3e, ipnum: 6, mask_addr: 0xBFFE04AC
 irq: 0x3f, ipnum: 6, mask_addr: 0xBFFE04AC

 irq: 0x40, ipnum: 2, mask_addr: 0xBFFE049C
 irq: 0x41, ipnum: 2, mask_addr: 0xBFFE049C
 irq: 0x42, ipnum: 2, mask_addr: 0xBFFE049C
 irq: 0x43, ipnum: 2, mask_addr: 0xBFFE049C
 irq: 0x44, ipnum: 2, mask_addr: 0xBFFE049C
 irq: 0x45, ipnum: 2, mask_addr: 0xBFFE049C
 irq: 0x46, ipnum: 2, mask_addr: 0xBFFE049C
 irq: 0x47, ipnum: 2, mask_addr: 0xBFFE049C
 irq: 0x48, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x49, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x4a, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x4b, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x4c, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x4d, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x4e, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x4f, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x50, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x51, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x52, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x53, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x54, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x55, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x56, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x57, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x58, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x59, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x5a, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x5b, ipnum: 3, mask_addr: 0xBFFE04A0
 irq: 0x5c, ipnum: 4, mask_addr: 0xBFFE04A4
 irq: 0x5d, ipnum: 4, mask_addr: 0xBFFE04A4
 irq: 0x5e, ipnum: 4, mask_addr: 0xBFFE04A4
 irq: 0x5f, ipnum: 4, mask_addr: 0xBFFE04A4

 */

__maybe_unused
static unsigned int sphe_irq_get_cpu_ipnum(unsigned int irq)
{
	unsigned int ipnum = irq;
	u32 shift, mask_addr;

	if ((irq) <= CYGNUM_HAL_INTERRUPT_IP7) {
		ipnum = irq;
	} else if (irq < CYGNUM_HAL_INTERRUPT_LEVEL1_BASE) {
		shift = irq - CYGNUM_HAL_INTERRUPT_LEVEL0_BASE;
		mask_addr = hal_level0_mask[shift];
		ipnum = ((mask_addr - HAL_INTR0_MASK) >> 2) + 2;
	} else if (irq < CYGNUM_HAL_INTERRUPT_LEVEL1_BASE + 32) {
		shift = irq - CYGNUM_HAL_INTERRUPT_LEVEL1_BASE;
		mask_addr = hal_level1_mask[shift];
		ipnum = ((mask_addr - HAL_INTR0_MASK) >> 2) + 2;
	}

	if (irq > CYGNUM_HAL_INTERRUPT_IP7)
	{
		//printk("@@@@ irq: 0x%02x, ipnum: %d, mask_addr: 0x%08X\n", irq, ipnum, mask_addr);
	}

	return ipnum;
}

static struct irq_chip sphe_irq_chip = { 0, };

static void __init sphe_irq_init(void)
{
	int i;
	unsigned int irq;

	sphe_irq_chip.name = "SPHE15XX";
	sphe_irq_chip.irq_ack = sphe_irq_ack;
	sphe_irq_chip.irq_mask = sphe_irq_mask;
	sphe_irq_chip.irq_unmask = sphe_irq_unmask;

	for (i = 0; i < IRQ_L0_COUNT; i++)
	{
		irq = i + CYGNUM_HAL_INTERRUPT_LEVEL0_BASE;
		irq_set_chip_and_handler(irq, &sphe_irq_chip, handle_level_irq);
		sphe_irq_get_cpu_ipnum(irq);
	}

	for (i = 0; i < IRQ_L1_COUNT; i++)
	{
		irq = i + CYGNUM_HAL_INTERRUPT_LEVEL1_BASE;
		irq_set_chip_and_handler(irq, &sphe_irq_chip, handle_level_irq);
		sphe_irq_get_cpu_ipnum(irq);
	}
}

unsigned int get_c0_compare_int(void)
{
	return SPHE15XX_TIMER_IRQ;
}

int get_c0_perfcount_int(void)
{
	return SPHE15XX_PERF_IRQ;
}
EXPORT_SYMBOL_GPL(get_c0_perfcount_int);

void __init arch_init_irq(void)
{
	HWREG_W(RISC0_INTR_L0_POLARITY, 0xFFFFFFFF);
	HWREG_W(INTR_L1_POLARITY, 0xFFFFFFFF);

	HWREG_W(RISC0_INTR_L0_EDGE,
		L0_BIT(CYGNUM_HAL_INTERRUPT_TIMER0)
		| L0_BIT(CYGNUM_HAL_INTERRUPT_TIMER1)
		| L0_BIT(CYGNUM_HAL_INTERRUPT_TIMER2)
		| L0_BIT(CYGNUM_HAL_INTERRUPT_TIMER3)
		| L0_BIT(CYGNUM_HAL_INTERRUPT_IOP)
		| L0_BIT(CYGNUM_HAL_INTERRUPT_WATCHDOG_TIMER));

	HWREG_W(INTR_L1_EDGE,
		  L1_BIT(CYGNUM_HAL_INTERRUPT_VLD_MP)
		| L1_BIT(CYGNUM_HAL_INTERRUPT_DEC_END)
		| L1_BIT(CYGNUM_HAL_INTERRUPT_SLICE_END)
		| L1_BIT(CYGNUM_HAL_INTERRUPT_MC2SD)
		| L1_BIT(CYGNUM_HAL_INTERRUPT_MJPEG)
		| L1_BIT(CYGNUM_HAL_INTERRUPT_TGEN_FLD_START)
		| L1_BIT(CYGNUM_HAL_INTERRUPT_TGEN_FLD_END)
		| L1_BIT(CYGNUM_HAL_INTERRUPT_TGEN_USER1)
		| L1_BIT(CYGNUM_HAL_INTERRUPT_TGEN_USER2)
		| L1_BIT(CYGNUM_HAL_INTERRUPT_AFRC));

	/* RISC0 */
	HWREG_W(RISC0_INTR0_MASK, 0x0);
	HWREG_W(RISC0_INTR1_MASK, 0x0);
	HWREG_W(RISC0_INTR2_MASK, 0x0);
	HWREG_W(RISC0_INTR3_MASK, 0x0);
	HWREG_W(RISC0_INTR4_MASK, 0x0);
	HWREG_W(RISC0_INTR5_MASK, 0x0);

	/* 0 0 0 1 1 1 */
	HWREG_W(RISC0_INTR_LX_SEL, 0X0007);

	/* USE REAL INTERRUPT SOURCE */
	HWREG_W(INTL0_TEST_MODE, 0x00000000);
	HWREG_W(INTL1_TEST_MODE, 0x00000000);
	HWREG_W(INTR_SRC_L0, 0x00000000);
	HWREG_W(INTR_SRC_L1, 0x00000000);

	HWREG_W(RISC0_INTR_L0_FLAG, 0xFFFFFFFF);
	HWREG_W(INTR_L1_FLAG, 0xFFFFFFFF);

	mips_cpu_irq_init();
	sphe_irq_init();
}

static void sphe_level0_handler(unsigned int mask)
{
	unsigned int i, irq;

	for (i=0; i < IRQ_L0_COUNT; i++)
	{
		if (mask & (1 << i))
		{
			irq = i + CYGNUM_HAL_INTERRUPT_LEVEL0_BASE;
			do_IRQ(irq);
		}
	}
}

static void sphe_level1_handler(unsigned int mask)
{
	unsigned int i, irq;

	for (i=0; i < IRQ_L1_COUNT; i++)
	{
		if (mask & (1 << i))
		{
			irq = i + CYGNUM_HAL_INTERRUPT_LEVEL1_BASE;
			do_IRQ(irq);
		}
	}
}

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending = read_c0_status() & read_c0_cause() & ST0_IM;

	if (pending & STATUSF_IP7)		/* cpu timer */
		do_IRQ(SPHE15XX_TIMER_IRQ);

	else if (pending & STATUSF_IP2)
		sphe_level1_handler(HWREG_R(RISC0_INTR0_MASK));

	else if (pending & STATUSF_IP3)
		sphe_level1_handler(HWREG_R(RISC0_INTR1_MASK));

	else if (pending & STATUSF_IP4)
		sphe_level1_handler(HWREG_R(RISC0_INTR2_MASK));

	else if (pending & STATUSF_IP5)
		sphe_level0_handler(HWREG_R(RISC0_INTR3_MASK));

	else if (pending & STATUSF_IP6)
		sphe_level0_handler(HWREG_R(RISC0_INTR4_MASK));

	else
		spurious_interrupt();
}
