/*
 *  Sunplus SPHE15XX SoC built-in UART driver
 *
 *  Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/kthread.h> /* kthread_run, kthread_stop */
#include <linux/completion.h>

#include <asm/div64.h>

#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/uart.h>
#include <asm/mach-sphe15xx/irq.h>

#define DRIVER_NAME "sphe15xx-uart"

#define SPHE15XX_UART_MAX_SCALE	0xff
#define SPHE15XX_UART_MAX_STEP	0xffff

#define SPHE15XX_UART_MIN_BAUD	300
#define SPHE15XX_UART_MAX_BAUD	3000000

#define SPHE15XX_DUMMY_STATUS_RD	0x01

static struct uart_driver sphe15xx_uart_driver;

struct sphe15xx_uart_port {
	struct uart_port	port;
	unsigned int		ier;		/* shadow Interrupt Enable Register */
	unsigned int		min_baud;
	unsigned int		max_baud;
	struct clk			*clk;

	struct task_struct *rx_task; /* task struct for kthread. */
};

static void sphe15xx_uart_tx_chars(struct sphe15xx_uart_port *up);

static inline bool sphe15xx_uart_console_enabled(void)
{
	return config_enabled(CONFIG_SERIAL_SPHE15XX_CONSOLE);
}

static inline unsigned int sphe15xx_uart_read(struct sphe15xx_uart_port *up,
					    int offset)
{
	return HWREG_R(up->port.mapbase + offset);
}

static inline void sphe15xx_uart_write(struct sphe15xx_uart_port *up,
				     int offset, unsigned int value)
{
	HWREG_W(up->port.mapbase + offset, value);
}

static inline void sphe15xx_uart_rmw(struct sphe15xx_uart_port *up,
				  unsigned int offset,
				  unsigned int mask,
				  unsigned int val)
{
	unsigned int t;

	t = sphe15xx_uart_read(up, offset);
	t &= ~mask;
	t |= val;
	sphe15xx_uart_write(up, offset, t);
}

static inline void sphe15xx_uart_rmw_set(struct sphe15xx_uart_port *up,
				       unsigned int offset,
				       unsigned int val)
{
	sphe15xx_uart_rmw(up, offset, 0, val);
}

static inline void sphe15xx_uart_rmw_clear(struct sphe15xx_uart_port *up,
					 unsigned int offset,
					 unsigned int val)
{
	sphe15xx_uart_rmw(up, offset, val, 0);
}

static inline void sphe15xx_uart_start_tx_interrupt(struct sphe15xx_uart_port *up)
{
	up->ier = (sphe15xx_uart_read(up, UART_ISC_REG) & 0xffef)|(1<<4);
	sphe15xx_uart_write(up, UART_ISC_REG, up->ier);
}

static inline void sphe15xx_uart_stop_tx_interrupt(struct sphe15xx_uart_port *up)
{
	up->ier = sphe15xx_uart_read(up, UART_ISC_REG) & 0xffef;
	sphe15xx_uart_write(up, UART_ISC_REG, up->ier);
}

static inline void sphe15xx_uart_putc(struct sphe15xx_uart_port *up, int ch)
{
	sphe15xx_uart_write(up, UART_DATA_REG, ch);
}

static unsigned int sphe15xx_uart_tx_empty(struct uart_port *port)
{
	struct sphe15xx_uart_port *up = (struct sphe15xx_uart_port *) port;
	unsigned long flags;
	unsigned int rdata;

	spin_lock_irqsave(&up->port.lock, flags);
	rdata = sphe15xx_uart_read(up, UART_LSR_REG);
	spin_unlock_irqrestore(&up->port.lock, flags);

	return (rdata & UART_TX_EMPTY) ? 0 : TIOCSER_TEMT;
}

static unsigned int sphe15xx_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR;
}

static void sphe15xx_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static void sphe15xx_uart_start_tx(struct uart_port *port)
{
	struct sphe15xx_uart_port *up = (struct sphe15xx_uart_port *) port;

	sphe15xx_uart_start_tx_interrupt(up);
}

static void sphe15xx_uart_stop_tx(struct uart_port *port)
{
	struct sphe15xx_uart_port *up = (struct sphe15xx_uart_port *) port;

	sphe15xx_uart_stop_tx_interrupt(up);
}

static void sphe15xx_uart_stop_rx(struct uart_port *port)
{
#if 0
	struct sphe15xx_uart_port *up = (struct sphe15xx_uart_port *) port;

	up->ier &= ~SPHE15XX_UART_INT_RX_VALID;
	sphe15xx_uart_write(up, SPHE15XX_UART_INT_EN_REG, up->ier);
#endif
}

static void sphe15xx_uart_break_ctl(struct uart_port *port, int break_state)
{
	struct sphe15xx_uart_port *up = (struct sphe15xx_uart_port *) port;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);

#if 0
	if (break_state == -1)
		sphe15xx_uart_rmw_set(up, SPHE15XX_UART_CS_REG,
				    SPHE15XX_UART_CS_TX_BREAK);
	else
		sphe15xx_uart_rmw_clear(up, SPHE15XX_UART_CS_REG,
				      SPHE15XX_UART_CS_TX_BREAK);
#endif

	spin_unlock_irqrestore(&up->port.lock, flags);
}

/*
 * baudrate = (clk / (scale + 1)) * (step * (1 / 2^17))
 */
static unsigned long sphe15xx_uart_get_baud(unsigned int clk,
					  unsigned int scale,
					  unsigned int step)
{
	u64 t;
	u32 div;

	div = (2 << 16) * (scale + 1);
	t = clk;
	t *= step;
	t += (div / 2);
	do_div(t, div);

	return t;
}

__maybe_unused
static void sphe15xx_uart_get_scale_step(unsigned int clk,
				       unsigned int baud,
				       unsigned int *scale,
				       unsigned int *step)
{
	unsigned int tscale;
	long min_diff;

	*scale = 0;
	*step = 0;

	min_diff = baud;
	for (tscale = 0; tscale < SPHE15XX_UART_MAX_SCALE; tscale++) {
		u64 tstep;
		int diff;

		tstep = baud * (tscale + 1);
		tstep *= (2 << 16);
		do_div(tstep, clk);

		if (tstep > SPHE15XX_UART_MAX_STEP)
			break;

		diff = abs(sphe15xx_uart_get_baud(clk, tscale, tstep) - baud);
		if (diff < min_diff) {
			min_diff = diff;
			*scale = tscale;
			*step = tstep;
		}
	}
}

static void sphe15xx_uart_set_termios(struct uart_port *port,
				    struct ktermios *neW,
				    struct ktermios *old)
{

# if 0
	struct sphe15xx_uart_port *up = (struct sphe15xx_uart_port *) port;
	unsigned int cs;
	unsigned long flags;
	unsigned int baud, scale, step;

	/* Only CS8 is supported */
	neW->c_cflag &= ~CSIZE;
	neW->c_cflag |= CS8;

	/* Only one stop bit is supported */
	neW->c_cflag &= ~CSTOPB;

	cs = 0;
	if (neW->c_cflag & PARENB) {
		if (!(neW->c_cflag & PARODD))
			cs |= UART_CS_PARITY_EVEN;
		else
			cs |= UART_CS_PARITY_ODD;
	} else {
		cs |= UART_CS_PARITY_NONE;
	}

	/* Mark/space parity is not supported */
	neW->c_cflag &= ~CMSPAR;

	baud = uart_get_baud_rate(port, neW, old, up->min_baud, up->max_baud);
	sphe15xx_uart_get_scale_step(port->uartclk, baud, &scale, &step);

	/*
	 * Ok, we're now changing the port state. Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	/* disable the UART */
	//sphe15xx_uart_rmw_clear(up, UART_CS_REG, UART_CS_IF_MODE_M << UART_CS_IF_MODE_S);

	/* Update the per-port timeout. */
	uart_update_timeout(port, neW->c_cflag, baud);

	up->port.ignore_status_mask = 0;

	/* ignore all characters if CREAD is not set */
	if ((neW->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= SPHE15XX_DUMMY_STATUS_RD;

	//sphe15xx_uart_write(up, UART_CLOCK_REG, scale << UART_CLOCK_SCALE_S | step);

	/* setup configuration register */
	//sphe15xx_uart_rmw(up, UART_CS_REG, UART_CS_PARITY_M, cs);

	/* enable host interrupt */
	//sphe15xx_uart_rmw_set(up, ART_CS_REG, UART_CS_HOST_INT_EN);
#if 0
	/* reenable the UART */
	sphe15xx_uart_rmw(up, UART_CS_REG,
			UART_CS_IF_MODE_M << UART_CS_IF_MODE_S,
			UART_CS_IF_MODE_DCE << UART_CS_IF_MODE_S);
#endif

	spin_unlock_irqrestore(&up->port.lock, flags);

	if (tty_termios_baud_rate(neW))
		tty_termios_encode_baud_rate(neW, baud, baud);
#endif
}

static void sphe15xx_uart_rx_chars(struct sphe15xx_uart_port *up)
{
	struct tty_port *port = &up->port.state->port;
	int max_count = 256;

	do {
		unsigned int rdata;
		unsigned char ch;

		rdata = sphe15xx_uart_read(up, UART_LSR_REG);
		if ((rdata & UART_LSR_RX_RDY) == 0)
			break;

		rdata = sphe15xx_uart_read(up, UART_DATA_REG);

		up->port.icount.rx++;
		ch = rdata & UART_DATA_TX_RX_MASK;

		if (uart_handle_sysrq_char(&up->port, ch))
			continue;

		if ((up->port.ignore_status_mask & SPHE15XX_DUMMY_STATUS_RD) == 0)
			tty_insert_flip_char(port, ch, TTY_NORMAL);
	} while (max_count-- > 0);

	spin_unlock(&up->port.lock);
	tty_flip_buffer_push(port);
	spin_lock(&up->port.lock);
}

static void sphe15xx_uart_tx_chars(struct sphe15xx_uart_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count;

	if (uart_tx_stopped(&up->port))
		return;

	count = up->port.fifosize;
	do {
		unsigned int rdata;

		rdata = sphe15xx_uart_read(up, UART_LSR_REG);
		if ((rdata & UART_LSR_TX_RDY) == 0)
			break;

		if (up->port.x_char) {
			sphe15xx_uart_putc(up, up->port.x_char);
			up->port.icount.tx++;
			up->port.x_char = 0;
			continue;
		}

		if (uart_circ_empty(xmit))
			break;

		sphe15xx_uart_putc(up, xmit->buf[xmit->tail]);

		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (!uart_circ_empty(xmit))
		sphe15xx_uart_start_tx_interrupt(up);
}

static irqreturn_t sphe15xx_uart_interrupt(int irq, void *dev_id)
{
	struct sphe15xx_uart_port *up = dev_id;
	unsigned int status;

	spin_lock(&up->port.lock);

	status = sphe15xx_uart_read(up, UART_LSR_REG);

	if (status & UART_LSR_RX_RDY) {
		sphe15xx_uart_rx_chars(up);
	}

	if (status & UART_TX_EMPTY) {
		sphe15xx_uart_stop_tx_interrupt(up);
		sphe15xx_uart_tx_chars(up);
	}

	spin_unlock(&up->port.lock);

	return IRQ_HANDLED;
}

static int sphe15xx_uart_startup(struct uart_port *port)
{
	struct sphe15xx_uart_port *up = (struct sphe15xx_uart_port *) port;
	unsigned long flags;
	int ret;

	ret = request_irq(up->port.irq, sphe15xx_uart_interrupt,
			  up->port.irqflags, dev_name(up->port.dev), up);

	if (ret) {
		dev_err(port->dev, "unable to get alias id, err=%d\n",ret);
		return ret;
	}

	spin_lock_irqsave(&up->port.lock, flags);

	/* Enable RX interrupts */
	up->ier = (sphe15xx_uart_read(up, UART_ISC_REG) & 0xffdf)|(1<<5);
	sphe15xx_uart_write(up, UART_ISC_REG, up->ier);

	spin_unlock_irqrestore(&up->port.lock, flags);

	return 0;
}

static void sphe15xx_uart_shutdown(struct uart_port *port)
{
	struct sphe15xx_uart_port *up = (struct sphe15xx_uart_port *) port;

	/* Disable all interrupts */
	up->ier = 0;
	sphe15xx_uart_write(up, UART_ISC_REG, up->ier);
	free_irq(up->port.irq, up);
}

static const char *sphe15xx_uart_type(struct uart_port *port)
{
	return (port->type == PORT_SPHE15XX) ? "SPHE15XX UART" : NULL;
}

static void sphe15xx_uart_release_port(struct uart_port *port)
{
	/* Nothing to release ... */
}

static int sphe15xx_uart_request_port(struct uart_port *port)
{
	/* UARTs always present */
	return 0;
}

static void sphe15xx_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_SPHE15XX;
}

static int sphe15xx_uart_verify_port(struct uart_port *port,
				   struct serial_struct *ser)
{
	struct sphe15xx_uart_port *up = (struct sphe15xx_uart_port *) port;

	if (ser->type != PORT_UNKNOWN &&
	    ser->type != PORT_SPHE15XX)
		return -EINVAL;

	if (ser->irq < 0 || ser->irq >= NR_IRQS)
		return -EINVAL;

	if (ser->baud_base < up->min_baud ||
	    ser->baud_base > up->max_baud)
		return -EINVAL;

	return 0;
}

static struct uart_ops sphe15xx_uart_ops = {
	.tx_empty	= sphe15xx_uart_tx_empty,
	.set_mctrl	= sphe15xx_uart_set_mctrl,
	.get_mctrl	= sphe15xx_uart_get_mctrl,
	.stop_tx	= sphe15xx_uart_stop_tx,
	.start_tx	= sphe15xx_uart_start_tx,
	.stop_rx	= sphe15xx_uart_stop_rx,
	.break_ctl	= sphe15xx_uart_break_ctl,
	.startup	= sphe15xx_uart_startup,
	.shutdown	= sphe15xx_uart_shutdown,
	.set_termios	= sphe15xx_uart_set_termios,
	.type		= sphe15xx_uart_type,
	.release_port	= sphe15xx_uart_release_port,
	.request_port	= sphe15xx_uart_request_port,
	.config_port	= sphe15xx_uart_config_port,
	.verify_port	= sphe15xx_uart_verify_port,
};

static struct sphe15xx_uart_port *
sphe15xx_console_ports[CONFIG_SERIAL_SPHE15XX_NR_UARTS];

static void sphe15xx_uart_wait_xmitr(struct sphe15xx_uart_port *up)
{
	unsigned int status;
	do {
		status = sphe15xx_uart_read(up, UART_LSR_REG);
		udelay(1);
	} while ((status & UART_LSR_TX_RDY) == 0);
}

static void sphe15xx_uart_console_putchar(struct uart_port *port, int ch)
{
	struct sphe15xx_uart_port *up = (struct sphe15xx_uart_port *) port;

	sphe15xx_uart_wait_xmitr(up);
	sphe15xx_uart_putc(up, ch);
}

static void sphe15xx_uart_console_write(struct console *co, const char *s,
				      unsigned int count)
{
	struct sphe15xx_uart_port *up = sphe15xx_console_ports[co->index];
	unsigned long flags;
	int locked = 1;

	local_irq_save(flags);

	if (up->port.sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock(&up->port.lock);
	else
		spin_lock(&up->port.lock);

	uart_console_write(&up->port, s, count, sphe15xx_uart_console_putchar);

	/*
	 * Finally, wait for transmitter to become empty
	 * and restore the IER
	 */
	sphe15xx_uart_wait_xmitr(up);

	if (locked)
		spin_unlock(&up->port.lock);

	local_irq_restore(flags);
}

static int sphe15xx_uart_console_setup(struct console *co, char *options)
{
	struct sphe15xx_uart_port *up;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index >= CONFIG_SERIAL_SPHE15XX_NR_UARTS)
		return -EINVAL;

	up = sphe15xx_console_ports[co->index];
	if (!up)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&up->port, co, baud, parity, bits, flow);
}

static struct console sphe15xx_uart_console = {
	.name		= "ttyS",
	.write		= sphe15xx_uart_console_write,
	.device		= uart_console_device,
	.setup		= sphe15xx_uart_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &sphe15xx_uart_driver,
};

static void sphe15xx_uart_add_console_port(struct sphe15xx_uart_port *up)
{
	if (!sphe15xx_uart_console_enabled())
		return;

	sphe15xx_console_ports[up->port.line] = up;
}

static struct uart_driver sphe15xx_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= "ttyS",
	.nr		= CONFIG_SERIAL_SPHE15XX_NR_UARTS,
	.cons		= NULL, /* filled in runtime */
};

static int sphe15xx_uart_probe(struct platform_device *pdev)
{
	struct sphe15xx_uart_port *up;
	struct uart_port *port;
	struct resource *mem_res;
	struct resource *irq_res;
	struct device_node *np;
	unsigned int baud;
	int id;
	int ret;

	np = pdev->dev.of_node;
	if (config_enabled(CONFIG_OF) && np) {
		id = of_alias_get_id(np, "serial");
		if (id < 0) {
			dev_err(&pdev->dev, "unable to get alias id, err=%d\n",
				id);
			return id;
		}
	} else {
		id = pdev->id;
		if (id == -1)
			id = 0;
	}

	if (id > CONFIG_SERIAL_SPHE15XX_NR_UARTS)
		return -EINVAL;

	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq_res) {
		dev_err(&pdev->dev, "no IRQ resource\n");
		return -EINVAL;
	}

	up = devm_kzalloc(&pdev->dev, sizeof(struct sphe15xx_uart_port),
			  GFP_KERNEL);
	if (!up)
		return -ENOMEM;

	up->clk = devm_clk_get(&pdev->dev, "uart");
	if (IS_ERR(up->clk)) {
		dev_err(&pdev->dev, "unable to get UART clock\n");
		return PTR_ERR(up->clk);
	}


	port = &up->port;

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	port->membase = devm_ioremap_resource(&pdev->dev, mem_res);
	if (IS_ERR(port->membase)) {
		dev_err(&pdev->dev, "unable to get UART memory!\n");
		return PTR_ERR(port->membase);
	}

	ret = clk_prepare_enable(up->clk);
	if (ret)
		return ret;

	port->uartclk = clk_get_rate(up->clk);
	if (!port->uartclk) {
		ret = -EINVAL;
		goto err_disable_clk;
	}

	port->mapbase = mem_res->start;
	port->line = id;
	port->irq = irq_res->start;
	port->dev = &pdev->dev;
	port->type = PORT_SPHE15XX;
	port->iotype = UPIO_MEM32;

	port->regshift = 2;
	port->fifosize = UART_FIFO_SIZE;
	port->ops = &sphe15xx_uart_ops;

	baud = sphe15xx_uart_get_baud(port->uartclk, SPHE15XX_UART_MAX_SCALE, 1);
	up->min_baud = max_t(unsigned int, baud, SPHE15XX_UART_MIN_BAUD);

	baud = sphe15xx_uart_get_baud(port->uartclk, 0, SPHE15XX_UART_MAX_STEP);
	up->max_baud = min_t(unsigned int, baud, SPHE15XX_UART_MAX_BAUD);

	sphe15xx_uart_add_console_port(up);

	ret = uart_add_one_port(&sphe15xx_uart_driver, &up->port);
	if (ret)
		goto err_disable_clk;

	platform_set_drvdata(pdev, up);
	return 0;

err_disable_clk:
	clk_disable_unprepare(up->clk);
	return ret;
}

static int sphe15xx_uart_remove(struct platform_device *pdev)
{
	struct sphe15xx_uart_port *up;

	up = platform_get_drvdata(pdev);

	if (up) {
		uart_remove_one_port(&sphe15xx_uart_driver, &up->port);
		clk_disable_unprepare(up->clk);
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sphe15xx_uart_of_ids[] = {
	{ .compatible = "qca,sphe15xx-uart" },
	{},
};
MODULE_DEVICE_TABLE(of, sphe15xx_uart_of_ids);
#endif

static struct platform_driver sphe15xx_uart_platform_driver = {
	.probe		= sphe15xx_uart_probe,
	.remove		= sphe15xx_uart_remove,
	.driver		= {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(sphe15xx_uart_of_ids),
	},
};

static int __init sphe15xx_uart_init(void)
{
	int ret;

	if (sphe15xx_uart_console_enabled())
		sphe15xx_uart_driver.cons = &sphe15xx_uart_console;

	ret = uart_register_driver(&sphe15xx_uart_driver);
	if (ret)
		goto err_out;

	ret = platform_driver_register(&sphe15xx_uart_platform_driver);
	if (ret)
		goto err_unregister_uart_driver;

	return 0;

err_unregister_uart_driver:
	uart_unregister_driver(&sphe15xx_uart_driver);
err_out:
	return ret;
}

static void __exit sphe15xx_uart_exit(void)
{
	platform_driver_unregister(&sphe15xx_uart_platform_driver);
	uart_unregister_driver(&sphe15xx_uart_driver);
}

module_init(sphe15xx_uart_init);
module_exit(sphe15xx_uart_exit);

MODULE_DESCRIPTION("Sunplus SPHE15XX UART driver");
MODULE_AUTHOR("Team-Proton <dev.team.proton@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
