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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/time.h>
#include <asm/addrspace.h>
#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/irq.h>
#include <asm/mach-sphe15xx/clock.h>

#ifdef __CDT_PARSER__
#define __init
#endif

struct clk {
	unsigned long rate;
	moduleList m;
};

static struct clk bus_clk;
static struct clk cpu_clk;
static struct clk dsp_clk;
static struct clk spi_clk;
static struct clk wdt_clk;
static struct clk vbus_clk;
static struct clk uart_clk;
static struct clk emac_clk;

static int init_clock_do_once = 0;

int __init get_sys_bus_freq(void)
{
	unsigned int NS;
	const unsigned int double_freq_step = 27;	// freq_step = 13.5

	NS = HWREG_R(PLLSYS_CFG) & 0x3F;

	/*
	 * The conventional equation of system clock frequency is:
	 *    SysClk = FreqStep * (NS + 1) / 2
	 *
	 * Due to integer limitation, here we use:
	 *        SysClk = FreqStep * (NS + 1) / 2
	 *               = DoubleFreqStep * (NS + 1) / 4
	 * 1000000: MHz
	 */
	return double_freq_step * (NS + 1) * (1000000 / 4);
}

static void __init sphe15xx_init_clocks(void)
{

	if (0 == init_clock_do_once)
	{
		init_clock_do_once = 1;

		HWREG_R(STC_32) = 0; HWREG_R(STC_31_16) = 0; HWREG_R(STC_15_0) = 0;

		HWREG_W(ADDR_OF(CLK_SEL,1), HWREG_R(ADDR_OF(CLK_SEL,1))&0xfff9ffff ); // bit 17:18 select dsp clock source
		HWREG_W(ADDR_OF(CLK_SEL,1), HWREG_R(ADDR_OF(CLK_SEL,1))|(1<<19) ); // bit 19 for PLLH clock divider output

		HWREG_W(SFT_CFG_24, HWREG_R(SFT_CFG_24)&0xfff9ffff ); // DSP: 540M
		HWREG_W(SFT_CFG_24, HWREG_R(SFT_CFG_24)|(1<<8) ); // set BWM to 1, cytest 0905
		HWREG_W(SFT_CFG_24, HWREG_R(SFT_CFG_24)|(1<<11) ); // enable bit 11 for DSPCLK
		HWREG_W(SFT_CFG_24, HWREG_R(SFT_CFG_24)|(1<<26) ); // enable bit 26 for CK2160M
		HWREG_W(SFT_CFG_24, HWREG_R(SFT_CFG_24)&~(1<<7) ); // set bit 7 to 0
		//
		// init_pbus()
		{
			HWREG_W(PERI_PAGE, 0);
			HWREG_W(PERI_M0_SETTING, 1);		// AUD real time type
			HWREG_W(PERI_M1_SETTING, 1);		// APP real time type
			HWREG_W(PERI_PAGE, 1);
			HWREG_W(PERI_M0_SETTING, 0x1F);		// AUD bandwidth
			HWREG_W(PERI_M1_SETTING, 0x1F);		// APP bandwidth
		}
	}
}

static signed char clken_uart1 = 1;

/*
 * Barrier
 */
void SP_LumaBarrier(void)
{
	register u32 t0;

	asm volatile ("lui		%0, 0xA000;"
				  "lw		%0, 0(%0);"
				  "sync"
				  :"=r"(t0));
}

void SP_ChromaBarrier(void)
{
	register u32 t0;

	asm volatile ("lui		%0, 0xB000;"
				  "lw		%0, 0(%0);"
				  "sync"
				  :"=r"(t0));
}

void SP_RegisterFileBarrier(void)
{
	register u32 t0;

	asm volatile ("lui		%0, 0xBFFE;"
				  "lw		%0, 0(%0);"
				  "sync"
				  :"=r"(t0));
}

int sphe15xx_clk_reset_on(moduleList m)
{
	int orig;

	if (m == DUMMY) // not in module list
		return 1;

	orig = HWREG_R(ADDR_OF(RESET,(m / 32)));
	HWREG_W(ADDR_OF(RESET,(m / 32)), orig | (1 << (m & 0x1F)));

	SP_RegisterFileBarrier();
	return (orig & (1 << (m & 0x1F))) != 0;
}

int sphe15xx_clk_reset_off(moduleList m)
{
	int orig;

	if (m == DUMMY) // not in module list
		return 1;

	orig = HWREG_R(ADDR_OF(RESET,(m / 32)));
	HWREG_W(ADDR_OF(RESET,(m / 32)), orig & ~(1 << (m & 0x1F)));

	SP_RegisterFileBarrier();
	return (orig & (1 << (m & 0x1F))) != 0;
}

int sphe15xx_clk_on(moduleList m)
{
	int orig;

	if (m == DUMMY) // not in module list
		return 1;

	orig = HWREG_R(ADDR_OF(CLKEN,(m)/32));
	switch (m) {
	case UART1:
		SAFE_CLKEN_ADD_ON(m, orig, clken_uart1);
		break;
	default:
		HWREG_W(ADDR_OF(CLKEN,(m)/32), orig | (1 << (m & 0x1F)));
		break;
	}

	SP_RegisterFileBarrier();
	return (orig & (1 << (m & 0x1F))) != 0;
}
EXPORT_SYMBOL(sphe15xx_clk_on);

int sphe15xx_clk_off(moduleList m)
{
	int orig;

	if (m == DUMMY) // not in module list
		return 1;

	orig = HWREG_R(ADDR_OF(CLKEN,(m)/32));
	switch (m) {
	case UART1:
		SAFE_CLKEN_SUB_OFF(m, orig, clken_uart1);
		break;
	default:
		HWREG_W(ADDR_OF(CLKEN,(m)/32), orig & ~(1 << (m & 0x1F)));
		break;
	}
	SP_RegisterFileBarrier();
	return (orig & (1 << (m & 0x1F))) != 0;
}
EXPORT_SYMBOL(sphe15xx_clk_off);

void sphe15xx_clk_off_ex(int n, ...)
{
	va_list ap;
	int i;
	moduleList t;
	unsigned int value[4] = { 0 };

	va_start(ap, n);
	for (i = 0; i < n; i++) {
		t = va_arg(ap, moduleList);
		value[t / 32] |= 1 << (t & 0x1F);
	}

	for (i = 0; i < 4; i++) {
		if (value[i]) {
			HWREG_0(ADDR_OF(CLKEN,i), ~value[i]);
		}
	}
	va_end(ap);
	SP_RegisterFileBarrier();
}
EXPORT_SYMBOL(sphe15xx_clk_off_ex);

/*
 * Linux clock API
 */
int clk_enable(struct clk *clk)
{
	int ret = sphe15xx_clk_on(clk->m);
	//printk("@@@@ clk_enable for module: %d, ret %d\n", clk->m, ret);
	return (ret == 0);
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	sphe15xx_clk_off(clk->m);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	return clk->rate;
}
EXPORT_SYMBOL(clk_get_rate);

struct clk *clk_get(struct device *dev, const char *id)
{
	if (!strcmp(id, "bus"))
		return &bus_clk;
	if (!strcmp(id, "cpu"))
		return &cpu_clk;
	if (!strcmp(id, "dsp"))
		return &dsp_clk;
	if (!strcmp(id, "spi"))
		return &spi_clk;
	if (!strcmp(id, "wdt"))
		return &wdt_clk;
	if (!strcmp(id, "vbus"))
		return &vbus_clk;
	if (!strcmp(id, "uart"))
		return &uart_clk;
	if (!strcmp(id, "emac"))
		return &emac_clk;
	return ERR_PTR(-ENOENT);
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_put);

void __init sphe_init_clocks(void)
{
	unsigned long rate;
	//register unsigned int *__rf asm("$22");
	//printk("############ __rf = 0x%08X\n", (u32)__rf);

	sphe15xx_init_clocks();

	rate = get_sys_bus_freq();

	/* adjust vbus clock rate */
	// Default SPI_clk rate = system_clock_rate/32

	bus_clk.rate  = rate;
	bus_clk.m = SYSTEM;

	cpu_clk.rate  = rate;
	cpu_clk.m = RISC0;

	spi_clk.rate  = rate / 32;
	spi_clk.m = SPI_M;

	wdt_clk.rate  = rate / 32;
	wdt_clk.m = DUMMY;

	vbus_clk.rate = rate / 2;
	vbus_clk.m = DUMMY;

	uart_clk.rate = rate / 32;
	uart_clk.m = UART0; // already enabled on boot

	emac_clk.rate = rate / 32;
	emac_clk.m = MAC;

	mips_hpt_frequency = rate;

	pr_info("CPU clk: %lu Mhz\n", rate / 1000000);

#if 0
    printk("$$ read_c0_conf:     0x%08x\n", read_c0_conf());
    printk("$$ read_c0_info:     0x%08x\n", read_c0_info());
    printk("$$ read_c0_prid:     0x%08x\n", read_c0_prid());
    printk("$$ read_c0_index:    0x%08x\n", read_c0_index());
    printk("$$ read_c0_ebase:    0x%08x\n", read_c0_ebase());
    printk("$$ read_c0_count:    0x%08x\n", read_c0_count());
    printk("$$ read_c0_status:   0x%08x\n", read_c0_status());
    printk("$$ read_c0_intctl:   0x%08x\n", read_c0_intctl());
    printk("$$ read_c0_hwrena:   0x%08x\n", read_c0_hwrena());
    printk("$$ read_c0_srsctl:   0x%08x\n", read_c0_srsctl());
    printk("$$ read_c0_srsmap:   0x%08x\n", read_c0_srsmap());
    printk("$$ read_c0_compare:  0x%08x\n", read_c0_compare());
    printk("$$ read_c0_compare2: 0x%08x\n", read_c0_compare2());
    printk("$$ read_c0_compare3: 0x%08x\n", read_c0_compare3());
    printk("$$ read_c0_config:   0x%08x\n", read_c0_config());
    printk("$$ read_c0_config1:  0x%08x\n", read_c0_config1());
    printk("$$ read_c0_config2:  0x%08x\n", read_c0_config2());
    printk("$$ read_c0_config3:  0x%08x\n", read_c0_config3());
    printk("$$ read_c0_config4:  0x%08x\n", read_c0_config4());
    printk("$$ read_c0_config5:  0x%08x\n", read_c0_config5());
    printk("$$ read_c0_config6:  0x%08x\n", read_c0_config6());
    printk("$$ read_c0_config7:  0x%08x\n", read_c0_config7());
#endif

}
