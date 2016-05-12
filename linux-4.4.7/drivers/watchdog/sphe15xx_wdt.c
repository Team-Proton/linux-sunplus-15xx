/*
 * Sunplus SPHE15XX built-in hardware watchdog timer.
 *
 * Copyright (C) 2016 Team-Proton
 *
 * This driver was based on: drivers/watchdog/ixp4xx_wdt.c
 *	Author: Deepak Saxena <dsaxena@plexity.net>
 *	Copyright 2004 (c) MontaVista, Software, Inc.
 *
 * which again was based on sa1100 driver,
 *	Copyright (C) 2000 Oleg Drokin <green@crimea.edu>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/sphe15xx.h>

#define DRIVER_NAME	"sphe15xx-wdt"

#define WDT_TIMEOUT			15	/* seconds */

#define WDOG_REG_CTRL		0x00
#define WDOG_REG_CNT		0x04

#define WDOG_CTRL_LAST_RESET	BIT(31)
#define WDOG_CTRL_ACTION_MASK	3
#define WDOG_CTRL_ACTION_NONE	0	/* no action */
#define WDOG_CTRL_ACTION_GPI	1	/* general purpose interrupt */
#define WDOG_CTRL_ACTION_NMI	2	/* NMI */
#define WDOG_CTRL_ACTION_FCR	3	/* full chip reset */

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
			   "(default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static int timeout = WDT_TIMEOUT;
module_param(timeout, int, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds "
			  "(default=" __MODULE_STRING(WDT_TIMEOUT) "s)");

static unsigned long wdt_flags;

#define WDT_FLAGS_BUSY		0
#define WDT_FLAGS_EXPECT_CLOSE	1

enum {
	WATCHDOG_CMD_CNT_WR_UNLOCK	= 0xAB00,
	WATCHDOG_CMD_CNT_WR_LOCK	= 0xAB01,
	WATCHDOG_CMD_CNT_WR_MAX		= 0xDEAF,
	WATCHDOG_CMD_PAUSE			= 0x3877,
	WATCHDOG_CMD_RESUME			= 0x4A4B,
	WATCHDOG_CMD_INTR_CLR		= 0x7482,
};

static struct clk *wdt_clk;
static unsigned long wdt_freq;
static int boot_status;
static int max_timeout;
static void __iomem *mem_base;
static unsigned int wdt_base;

static inline void sphe15xx_wdt_wr(unsigned reg, unsigned int val)
{
	HWREG_W(wdt_base + reg, val);
}

static inline u32 sphe15xx_wdt_rr(unsigned reg)
{
	return HWREG_R(wdt_base + reg);
}

/*
 * 65,756 - 37,390 = 28,366 = 5 sec
 * 5,673 = 1 sec
 */
static inline void sphe15xx_wdt_keepalive(void)
{
	unsigned int count = wdt_freq * timeout;

	count = ~0;  // fixme

	if (~0 == count) {
		count = 0xFFFF;
	} else {
		/* Tranform count into STC clock.  */
		count = ((count & 0xFFFF) * 90) >> 4;
	}

	sphe15xx_wdt_wr(WDOG_REG_CTRL, WATCHDOG_CMD_CNT_WR_UNLOCK);
	sphe15xx_wdt_wr(WDOG_REG_CNT, count);
	/* flush write */
	sphe15xx_wdt_rr(WDOG_REG_CNT);
}

static inline void sphe15xx_wdt_disable(void)
{
	sphe15xx_wdt_wr(WDOG_REG_CTRL, WATCHDOG_CMD_PAUSE);
}

static inline void sphe15xx_wdt_enable(void)
{
	sphe15xx_wdt_disable();
	sphe15xx_wdt_keepalive();
	sphe15xx_wdt_wr(WDOG_REG_CTRL, WATCHDOG_CMD_RESUME);

	/* Enable Watchdog Reset Counter in STC module. Enable this to trigger system reset. */
	HWREG_W(SFT_CFG_8, HWREG_R(SFT_CFG_8) | (1 << 12));
}

static int sphe15xx_wdt_set_timeout(int val)
{
	if (val < 1 || val > max_timeout)
		return -EINVAL;

	timeout = val;
	sphe15xx_wdt_keepalive();

	return 0;
}

static int sphe15xx_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(WDT_FLAGS_BUSY, &wdt_flags))
		return -EBUSY;

	clear_bit(WDT_FLAGS_EXPECT_CLOSE, &wdt_flags);
	sphe15xx_wdt_enable();

	return nonseekable_open(inode, file);
}

static int sphe15xx_wdt_release(struct inode *inode, struct file *file)
{
	if (test_bit(WDT_FLAGS_EXPECT_CLOSE, &wdt_flags))
		sphe15xx_wdt_disable();
	else {
		pr_crit("device closed unexpectedly, watchdog timer will not stop!\n");
		sphe15xx_wdt_keepalive();
	}

	clear_bit(WDT_FLAGS_BUSY, &wdt_flags);
	clear_bit(WDT_FLAGS_EXPECT_CLOSE, &wdt_flags);

	return 0;
}

static ssize_t sphe15xx_wdt_write(struct file *file, const char *data,
				size_t len, loff_t *ppos)
{
	if (len) {
		if (!nowayout) {
			size_t i;

			clear_bit(WDT_FLAGS_EXPECT_CLOSE, &wdt_flags);

			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i))
					return -EFAULT;

				if (c == 'V')
					set_bit(WDT_FLAGS_EXPECT_CLOSE,
						&wdt_flags);
			}
		}

		sphe15xx_wdt_keepalive();
	}

	return len;
}

static const struct watchdog_info sphe15xx_wdt_info = {
	.options		= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
				  WDIOF_MAGICCLOSE | WDIOF_CARDRESET,
	.firmware_version	= 0,
	.identity		= "SPHE15XX watchdog",
};

static long sphe15xx_wdt_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int err;
	int t;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		err = copy_to_user(argp, &sphe15xx_wdt_info,
				   sizeof(sphe15xx_wdt_info)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
		err = put_user(0, p);
		break;

	case WDIOC_GETBOOTSTATUS:
		err = put_user(boot_status, p);
		break;

	case WDIOC_KEEPALIVE:
		sphe15xx_wdt_keepalive();
		err = 0;
		break;

	case WDIOC_SETTIMEOUT:
		err = get_user(t, p);
		if (err)
			break;

		err = sphe15xx_wdt_set_timeout(t);
		if (err)
			break;

		/* fallthrough */
	case WDIOC_GETTIMEOUT:
		err = put_user(timeout, p);
		break;

	default:
		err = -ENOTTY;
		break;
	}

	return err;
}

static const struct file_operations sphe15xx_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= sphe15xx_wdt_write,
	.unlocked_ioctl	= sphe15xx_wdt_ioctl,
	.open		= sphe15xx_wdt_open,
	.release	= sphe15xx_wdt_release,
};

static struct miscdevice sphe15xx_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &sphe15xx_wdt_fops,
};

static int sphe15xx_wdt_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err;

	if (mem_base) {
		dev_err(&pdev->dev, "device busy!\n");
		return -EBUSY;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mem_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mem_base)) {
		dev_err(&pdev->dev, "error remap watchdog memory!\n");
		return PTR_ERR(mem_base);
	}

	wdt_base = (u32)res->start;

	wdt_clk = devm_clk_get(&pdev->dev, "wdt");
	if (IS_ERR(wdt_clk)) {
		dev_err(&pdev->dev, "error get watchdog clk!\n");
		return PTR_ERR(wdt_clk);
	}

	err = clk_prepare_enable(wdt_clk);
	if (err) {
		dev_err(&pdev->dev, "error prepare watchdog clk!\n");
		return err;
	}

	wdt_freq = clk_get_rate(wdt_clk);
	if (!wdt_freq) {
		err = -EINVAL;
		dev_err(&pdev->dev, "error get watchdog freq!\n");
		goto err_clk_disable;
	}

	max_timeout = (0xfffffffful / wdt_freq);
	if (timeout < 1 || timeout > max_timeout) {
		timeout = max_timeout;
		dev_info(&pdev->dev,
			"timeout value must be 0 < timeout < %d, using %d\n",
			max_timeout, timeout);
	}

	boot_status = sphe15xx_wdt_rr(WDOG_REG_CTRL);

	err = misc_register(&sphe15xx_wdt_miscdev);
	if (err) {
		dev_err(&pdev->dev,
			"unable to register misc device, err=%d\n", err);
		goto err_clk_disable;
	}

	return 0;

err_clk_disable:
	clk_disable_unprepare(wdt_clk);
	return err;
}

static int sphe15xx_wdt_remove(struct platform_device *pdev)
{
	misc_deregister(&sphe15xx_wdt_miscdev);
	clk_disable_unprepare(wdt_clk);
	return 0;
}

static void sphe15xx_wdt_shutdown(struct platform_device *pdev)
{
	sphe15xx_wdt_disable();
}

#ifdef CONFIG_OF
static const struct of_device_id sphe15xx_wdt_match[] = {
	{ .compatible = "sphe,sphe15xx-wdt" },
	{},
};
MODULE_DEVICE_TABLE(of, sphe15xx_wdt_match);
#endif

static struct platform_driver sphe15xx_wdt_driver = {
	.probe		= sphe15xx_wdt_probe,
	.remove		= sphe15xx_wdt_remove,
	.shutdown	= sphe15xx_wdt_shutdown,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(sphe15xx_wdt_match),
	},
};

module_platform_driver(sphe15xx_wdt_driver);

MODULE_DESCRIPTION("Sunplus SPHE15XX hardware watchdog driver");
MODULE_AUTHOR("Team-Proton");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
