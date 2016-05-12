/*
 * Sunplus SPHE15XX EMAC Fast Ethernet driver for Linux.
 *
 * Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include "sphe15xx-emac.h"

#include <linux/clk.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/phy.h>

#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/platform.h>
#include <asm/mach-sphe15xx/irq.h>


#define DRV_NAME		"sphe15xx-emac"
#define DRV_VERSION		"1.01"

#define EMAC_MAX_FRAME_LEN	0x0600

#if 1
#define dbg(fmt, arg...)  printk("@@@ --> %s()[%d] " fmt,__FUNCTION__,__LINE__,##arg)
#else
#define dbg(...)
#endif

/* Transmit timeout, default 5 seconds. */
static int watchdog = 5000;
module_param(watchdog, int, 0400);
MODULE_PARM_DESC(watchdog, "transmit timeout in milliseconds");

/* EMAC register address locking.
 *
 * The EMAC uses an address register to control where data written
 * to the data register goes. This means that the address register
 * must be preserved over interrupts or similar calls.
 *
 * During interrupt and other critical calls, a spinlock is used to
 * protect the system, but the calls themselves save the address
 * in the address register in case they are interrupting another
 * access to the device.
 *
 * For general accesses a lock is provided so that calls which are
 * allowed to sleep are serialised so that the address register does
 * not need to be saved. This lock also serves to serialise access
 * to the EEPROM and PHY access registers which are shared between
 * these two devices.
 */

/* The driver supports the original EMACE, and now the two newer
 * devices, EMACA and EMACB.
 */

#define RXTX_BUFF_SIZE 		2080

struct emac_board_info {
	struct clk		*clk;
	struct device		*dev;
	struct platform_device	*pdev;
	spinlock_t		lock;
	void __iomem		*membase;
	unsigned int		base;
	u32			msg_enable;
	struct net_device	*ndev;
	struct sk_buff		*skb_last;
	u16			tx_fifo_stat;

	int			emacrx_completed_flag;

	struct phy_device	*phydev;
	struct device_node	*phy_node;
	unsigned int		link;
	unsigned int		speed;
	unsigned int		duplex;

	unsigned int		init_status;
	unsigned int		recv_error;

	unsigned char		rx_buff[RXTX_BUFF_SIZE];
	unsigned char		tx_buff[RXTX_BUFF_SIZE];

	phy_interface_t		phy_interface;
};

#define MAC_ADDRESS_1502_START 457

static void emac_otp_read_one_addr(unsigned short addr, unsigned int *RDATA)
{
    u32 read_data;
    HWREG_W(OTP_ADDR, addr);
    HWREG_W(OTP_CMD, 0x0020);
    read_data = HWREG_R(OTP_CMD_SET);
    while(((read_data>>10)&0x0001) == 0x0){
		read_data = HWREG_R(OTP_CMD_SET);
    }
    HWREG_W(OTP_CMD_SET, 0x0);
    RDATA[0] = HWREG_R(OTP_READ_DATA_0);
    return;
}

void emac_otp_read_mac(unsigned char *mac_addr)
{
	unsigned short addr;
	unsigned int RDATA[1];

  	HWREG_W(OTP_CTRL, ((HWREG_R(OTP_CTRL) & 0xfffc0000) | (0<<17) | (0<<16) | (0x0)));
	for(addr=MAC_ADDRESS_1502_START; addr<MAC_ADDRESS_1502_START+6; addr++)
	{
		emac_otp_read_one_addr((addr<<1), RDATA);
		mac_addr[5-(addr-MAC_ADDRESS_1502_START)] = RDATA[0]&0xFF;
	}
}

static void emac_set_ethr_address(struct net_device *ndev)
{
	emac_otp_read_mac(ndev->dev_addr);
}

static void emac_init_hw(struct net_device *ndev)
{
	struct emac_board_info *db = netdev_priv(ndev);

    unsigned char *mac = ndev->dev_addr;
    unsigned int m0 = 0x10CE1F00;//0x10000 * mac[1] | 0x1000000 * mac[0] | mac[2] | 256 * mac[3];
    unsigned int m1 = 0x000009C0;//mac[5] | 256 * mac[4];

    // reset to default values */

    HWREG_W(db->base + EMAC_TX_TRIGGER, 0);
    HWREG_W(db->base + EMAC_PORT_CFGSTA, 0x1F);

    // 00:01:00:00:2E:B5
    // 0x10000 * 00 | 0x1000000 * 1 | 0 | 256 * 0
    // 10CE1F00 --> EMAC_MACADDR0 value

    // 0x2E * 256 | 0xB5		XXX fixme i don't know how to convert mac value
    // 000009C0 --> EMAC_MACADDR1 value

    /* set mac addr */
    HWREG_W(db->base + EMAC_MACADDR0, m0);
    HWREG_W(db->base + EMAC_MACADDR1, m1);

    HWREG_W(db->base + EMAC_TX_CFG, 0x002C1842);
    HWREG_W(db->base + EMAC_RX_CFG, 0x0381F81C);

    /* set default multicast */
    HWREG_W(db->base + EMAC_RX_MULTABLE0, 0); // OFF = 0, ON = 08000000
    HWREG_W(db->base + EMAC_RX_MULTABLE1, 0); // OFF = 0, ON = 00001000

    /* set default unicast */
    HWREG_W(db->base + EMAC_RX_UNICASTABLE0, 0xffffffff);
    HWREG_W(db->base + EMAC_RX_UNICASTABLE1, 0xffffffff);

    HWREG_W(db->base + EMAC_TX_DATABUFFBASEADDR, (u32)db->tx_buff & 0x1fffffff);
    HWREG_W(db->base + EMAC_RX_QUEUE0_DESCBASEADDR, (u32)db->rx_buff & 0x1fffffff);

    HWREG_W(db->base + EMAC_SYS_CFGCMD, 0x41654206);
    HWREG_W(db->base + EMAC_PAUSE_CFG, 0x6ffff);
}

static void emac_update_speed(struct net_device *dev)
{
#if 0
	struct emac_board_info *db = netdev_priv(dev);
	unsigned int reg_val;

	/* set EMAC SPEED, depend on PHY  */
	reg_val = readl(db->membase + EMAC_MAC_SUPP_REG);
	reg_val &= ~(0x1 << 8);
	if (db->speed == SPEED_100)
		reg_val |= 1 << 8;
	writel(reg_val, db->membase + EMAC_MAC_SUPP_REG);
#endif
}

static void emac_update_duplex(struct net_device *dev)
{
#if 0
	struct emac_board_info *db = netdev_priv(dev);
	unsigned int reg_val;

	/* set duplex depend on phy */
	reg_val = readl(db->membase + EMAC_MAC_CTL1_REG);
	reg_val &= ~EMAC_MAC_CTL1_DUPLEX_EN;
	if (db->duplex)
		reg_val |= EMAC_MAC_CTL1_DUPLEX_EN;
	writel(reg_val, db->membase + EMAC_MAC_CTL1_REG);
#endif
}

static void emac_handle_link_change(struct net_device *dev)
{
	struct emac_board_info *db = netdev_priv(dev);
	struct phy_device *phydev = db->phydev;
	unsigned long flags;
	int status_change = 0;

	if (phydev->link) {
		if (db->speed != phydev->speed) {
			spin_lock_irqsave(&db->lock, flags);
			db->speed = phydev->speed;
			emac_update_speed(dev);
			spin_unlock_irqrestore(&db->lock, flags);
			status_change = 1;
		}

		if (db->duplex != phydev->duplex) {
			spin_lock_irqsave(&db->lock, flags);
			db->duplex = phydev->duplex;
			emac_update_duplex(dev);
			spin_unlock_irqrestore(&db->lock, flags);
			status_change = 1;
		}
	}

	if (phydev->link != db->link) {
		if (!phydev->link) {
			db->speed = 0;
			db->duplex = -1;
		}
		db->link = phydev->link;

		status_change = 1;
	}

	if (status_change)
		phy_print_status(phydev);
}

static int emac_mdio_probe(struct net_device *dev)
{
	struct emac_board_info *db = netdev_priv(dev);
	struct sphe15xx_emac_platform_data *pdata;

	pdata = db->pdev->dev.platform_data;

	db->phydev = phy_find_first(pdata->mdio_data->mii_bus);
	if (!db->phydev) {
		dev_err(&db->pdev->dev, "no PHY found\n");
		return -ENODEV;
	}

	/* attach the mac to the phy */
	db->phydev = phy_connect(db->ndev, dev_name(&db->phydev->dev),
			     &emac_handle_link_change, db->phy_interface);

	if (IS_ERR(db->phydev)) {
		netdev_err(db->ndev, "could not find the PHY\n");
		return -ENODEV;
	}

	/* mask with MAC supported features */
	db->phydev->supported &= PHY_BASIC_FEATURES;
	db->phydev->advertising = db->phydev->supported;

	db->link = 0;
	db->speed = 0;
	db->duplex = -1;

	if (db->phydev->phy_id == 0x02228002) {
		//db->phydev->drv->name 		= "SP483";
		//db->phydev->drv->features	= PHY_BASIC_FEATURES;
		//db->phydev->drv->flags		= PHY_IS_INTERNAL;
	}

	dev_info(&db->pdev->dev, "attached PHY [%s] at address (%s), phy id: 0x%08x\n",
			db->phydev->drv->name,
			dev_name(&db->phydev->dev),
			db->phydev->phy_id);

	return 0;
}

static void emac_mdio_remove(struct net_device *dev)
{
	struct emac_board_info *db = netdev_priv(dev);

	phy_disconnect(db->phydev);
	db->phydev = NULL;
}

static void emac_reset(struct emac_board_info *db)
{
	dev_dbg(db->dev, "resetting device\n");

	/* RESET device */
    HWREG_W(db->base + EMAC_SYS_CFGCMD, HWREG_R(db->base + EMAC_SYS_CFGCMD) | 1);
	udelay(200);
    HWREG_W(db->base + EMAC_SYS_CFGCMD, 0x41654200);

	udelay(200);
}

static void emac_outblk_32bit(void __iomem *reg, void *data, int count)
{
	//writesl(reg, data, round_up(count, 4) / 4);
}

static void emac_inblk_32bit(void __iomem *reg, void *data, int count)
{
	//readsl(reg, data, round_up(count, 4) / 4);
}

static int emac_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct emac_board_info *dm = netdev_priv(dev);
	struct phy_device *phydev = dm->phydev;

	dbg(">\n");

	if (!netif_running(dev))
		return -EINVAL;

	if (!phydev)
		return -ENODEV;

	dbg("<\n");

	return phy_mii_ioctl(phydev, rq, cmd);
}

/* ethtool ops */
static void emac_get_drvinfo(struct net_device *dev,
			      struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(DRV_NAME));
	strlcpy(info->version, DRV_VERSION, sizeof(DRV_VERSION));
	strlcpy(info->bus_info, dev_name(&dev->dev), sizeof(info->bus_info));
}

static int emac_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct emac_board_info *dm = netdev_priv(dev);
	struct phy_device *phydev = dm->phydev;

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_gset(phydev, cmd);
}

static int emac_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct emac_board_info *dm = netdev_priv(dev);
	struct phy_device *phydev = dm->phydev;

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_sset(phydev, cmd);
}

static const struct ethtool_ops emac_ethtool_ops = {
	.get_drvinfo	= emac_get_drvinfo,
	.get_settings	= emac_get_settings,
	.set_settings	= emac_set_settings,
	.get_link	= ethtool_op_get_link,
};

static unsigned int emac_setup(struct net_device *ndev)
{
#if 0
	struct emac_board_info *db = netdev_priv(ndev);
	unsigned int reg_val;

	emac_init_hw();

	/* set up TX */
	reg_val = readl(db->membase + EMAC_TX_MODE_REG);

	writel(reg_val | EMAC_TX_MODE_ABORTED_FRAME_EN,
		db->membase + EMAC_TX_MODE_REG);

	/* set MAC */
	/* set MAC CTL0 */
	reg_val = readl(db->membase + EMAC_MAC_CTL0_REG);
	writel(reg_val | EMAC_MAC_CTL0_RX_FLOW_CTL_EN |
		EMAC_MAC_CTL0_TX_FLOW_CTL_EN,
		db->membase + EMAC_MAC_CTL0_REG);

	/* set MAC CTL1 */
	reg_val = readl(db->membase + EMAC_MAC_CTL1_REG);
	reg_val |= EMAC_MAC_CTL1_LEN_CHECK_EN;
	reg_val |= EMAC_MAC_CTL1_CRC_EN;
	reg_val |= EMAC_MAC_CTL1_PAD_EN;
	writel(reg_val, db->membase + EMAC_MAC_CTL1_REG);

	/* set up IPGT */
	writel(EMAC_MAC_IPGT_FULL_DUPLEX, db->membase + EMAC_MAC_IPGT_REG);

	/* set up IPGR */
	writel((EMAC_MAC_IPGR_IPG1 << 8) | EMAC_MAC_IPGR_IPG2,
		db->membase + EMAC_MAC_IPGR_REG);

	/* set up Collison window */
	writel((EMAC_MAC_CLRT_COLLISION_WINDOW << 8) | EMAC_MAC_CLRT_RM,
		db->membase + EMAC_MAC_CLRT_REG);

	/* set up Max Frame Length */
	writel(EMAC_MAX_FRAME_LEN,
		db->membase + EMAC_MAC_MAXF_REG);
#endif

	return 0;
}

static void emac_set_rx_mode(struct net_device *ndev)
{
#if 0
	struct emac_board_info *db = netdev_priv(ndev);
	unsigned int reg_val;

	/* set up RX */
	reg_val = readl(db->membase + EMAC_RX_CTL_REG);

	if (ndev->flags & IFF_PROMISC)
		reg_val |= EMAC_RX_CTL_PASS_ALL_EN;
	else
		reg_val &= ~EMAC_RX_CTL_PASS_ALL_EN;

	writel(reg_val | EMAC_RX_CTL_PASS_LEN_OOR_EN |
		EMAC_RX_CTL_ACCEPT_UNICAST_EN | EMAC_RX_CTL_DA_FILTER_EN |
		EMAC_RX_CTL_ACCEPT_MULTICAST_EN |
		EMAC_RX_CTL_ACCEPT_BROADCAST_EN,
		db->membase + EMAC_RX_CTL_REG);
#endif
}

static unsigned int emac_powerup(struct net_device *ndev)
{
#if 0
	struct emac_board_info *db = netdev_priv(ndev);
	unsigned int reg_val;

	/* initial EMAC */
	/* flush RX FIFO */
	reg_val = readl(db->membase + EMAC_RX_CTL_REG);
	reg_val |= 0x8;
	writel(reg_val, db->membase + EMAC_RX_CTL_REG);
	udelay(1);

	/* initial MAC */
	/* soft reset MAC */
	reg_val = readl(db->membase + EMAC_MAC_CTL0_REG);
	reg_val &= ~EMAC_MAC_CTL0_SOFT_RESET;
	writel(reg_val, db->membase + EMAC_MAC_CTL0_REG);

	/* set MII clock */
	reg_val = readl(db->membase + EMAC_MAC_MCFG_REG);
	reg_val &= (~(0xf << 2));
	reg_val |= (0xD << 2);
	writel(reg_val, db->membase + EMAC_MAC_MCFG_REG);

	/* clear RX counter */
	writel(0x0, db->membase + EMAC_RX_FBC_REG);

	/* disable all interrupt and clear interrupt status */
	writel(0, db->membase + EMAC_INT_CTL_REG);
	reg_val = readl(db->membase + EMAC_INT_STA_REG);
	writel(reg_val, db->membase + EMAC_INT_STA_REG);

	udelay(1);

	/* set up EMAC */
	emac_setup(ndev);

	/* set mac_address to chip */
	writel(ndev->dev_addr[0] << 16 | ndev->dev_addr[1] << 8 | ndev->
	       dev_addr[2], db->membase + EMAC_MAC_A1_REG);
	writel(ndev->dev_addr[3] << 16 | ndev->dev_addr[4] << 8 | ndev->
	       dev_addr[5], db->membase + EMAC_MAC_A0_REG);

	mdelay(1);
#endif

	return 0;
}

static int emac_set_mac_address(struct net_device *dev, void *p)
{
	struct sockaddr *addr = p;
	struct emac_board_info *db = netdev_priv(dev);

	if (netif_running(dev))
		return -EBUSY;

	memcpy(dev->dev_addr, addr->sa_data, ETH_ALEN);

#if 0
	writel(dev->dev_addr[0] << 16 | dev->dev_addr[1] << 8 | dev->
	       dev_addr[2], db->membase + EMAC_MAC_A1_REG);
	writel(dev->dev_addr[3] << 16 | dev->dev_addr[4] << 8 | dev->
	       dev_addr[5], db->membase + EMAC_MAC_A0_REG);
#endif

	return 0;
}

/* Initialize emac board */
static void emac_init_device(struct net_device *dev)
{
	struct emac_board_info *db = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&db->lock, flags);

	/* enable mac clock */
	clk_prepare_enable(db->clk);
	udelay(1000);

	emac_update_speed(dev);
	emac_update_duplex(dev);

	if (db->init_status & 0x100)
	{
		HWREG_W(db->base + EMAC_RX_MULTABLE0, 0xFFFFFFFF);
	    HWREG_W(db->base + EMAC_RX_MULTABLE1, 0xFFFFFFFF);
	    HWREG_W(db->base + EMAC_RX_UNICASTABLE0, 0xFFFFFFFF);
	    HWREG_W(db->base + EMAC_RX_UNICASTABLE1, 0xFFFFFFFF);
	    HWREG_W(db->base + EMAC_RX_CFG, HWREG_R(db->base + EMAC_RX_CFG) & 0xFFFFF8FF);
	}

	HWREG_W(db->base + EMAC_SYS_CFGCMD, HWREG_R(db->base + EMAC_SYS_CFGCMD) & 0xFFFFFFF9);
	/* enable interrupts */
	HWREG_W(db->base + EMAC_INT_MASK, 0);

	db->init_status = 0;
	udelay(1000);
	dbg("rx cfg:  0x%08X\n", HWREG_R(db->base + EMAC_RX_CFG));
	dbg("sys cfg: 0x%08X\n", HWREG_R(db->base + EMAC_SYS_CFGCMD));
	dbg(" <\n");
	spin_unlock_irqrestore(&db->lock, flags);
}

/* Our watchdog timed out. Called by the networking layer */
static void emac_timeout(struct net_device *dev)
{
	struct emac_board_info *db = netdev_priv(dev);
	unsigned long flags;

	if (netif_msg_timer(db))
		dev_err(db->dev, "tx time out.\n");

	/* Save previous register address */
	spin_lock_irqsave(&db->lock, flags);

	netif_stop_queue(dev);
	emac_reset(db);
	emac_init_device(dev);
	/* We can accept TX packets again */
	dev->trans_start = jiffies;
	netif_wake_queue(dev);

	/* Restore previous register address */
	spin_unlock_irqrestore(&db->lock, flags);
}

/* Hardware start transmission.
 * Send a packet to media from the upper layer.
 */
static int emac_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct emac_board_info *db = netdev_priv(dev);
	unsigned long channel;
	unsigned long flags;

	dbg(" >\n");

	channel = db->tx_fifo_stat & 3;
	if (channel == 3)
		return 1;

	channel = (channel == 1 ? 1 : 0);

	spin_lock_irqsave(&db->lock, flags);

#if 0
	writel(channel, db->membase + EMAC_TX_INS_REG);

	emac_outblk_32bit(db->membase + EMAC_TX_IO_DATA_REG,
			skb->data, skb->len);
	dev->stats.tx_bytes += skb->len;

	db->tx_fifo_stat |= 1 << channel;
	/* TX control: First packet immediately send, second packet queue */
	if (channel == 0) {
		/* set TX len */
		writel(skb->len, db->membase + EMAC_TX_PL0_REG);
		/* start translate from fifo to phy */
		writel(readl(db->membase + EMAC_TX_CTL0_REG) | 1,
		       db->membase + EMAC_TX_CTL0_REG);

		/* save the time stamp */
		dev->trans_start = jiffies;
	} else if (channel == 1) {
		/* set TX len */
		writel(skb->len, db->membase + EMAC_TX_PL1_REG);
		/* start translate from fifo to phy */
		writel(readl(db->membase + EMAC_TX_CTL1_REG) | 1,
		       db->membase + EMAC_TX_CTL1_REG);

		/* save the time stamp */
		dev->trans_start = jiffies;
	}

	if ((db->tx_fifo_stat & 3) == 3) {
		/* Second packet */
		netif_stop_queue(dev);
	}
#endif

	spin_unlock_irqrestore(&db->lock, flags);

	/* free this SKB */
	dev_consume_skb_any(skb);

	return NETDEV_TX_OK;
}

/* EMAC interrupt handler
 * receive the packet to upper layer, free the transmitted packet
 */
static void emac_tx_done(struct net_device *dev, struct emac_board_info *db,
			  unsigned int tx_status)
{
	dbg(" >\n");
	/* One packet sent complete */
	db->tx_fifo_stat &= ~(tx_status & 3);
	if (3 == (tx_status & 3))
		dev->stats.tx_packets += 2;
	else
		dev->stats.tx_packets++;

	if (netif_msg_tx_done(db))
		dev_dbg(db->dev, "tx done, NSR %02x\n", tx_status);

	netif_wake_queue(dev);
	dbg(" <\n");
}

static void hexdump(const unsigned char *buf, unsigned short len)
{
	print_hex_dump("", "", DUMP_PREFIX_OFFSET, 16, 1, buf, len, true);
}

/* Received a packet and pass to upper layer
 */
static void emac_rx(struct net_device *dev)
{
	struct emac_board_info *db = netdev_priv(dev);
	struct sk_buff *skb;
	u8 *rdptr;
	bool good_packet;
	static int rxlen_last;
	unsigned int reg_val;
	u32 rxhdr, rxstatus, rxcount, rxlen;

	dbg(" >\n");

	if (db->recv_error == 0)
	{
		reg_val = HWREG_R(MAC_RX_ERRORCNT0);
		reg_val = HWREG_R(MAC_RX_ERRORCNT1);
		reg_val = HWREG_R(MAC_RX_ERRORCNT2);
		db->emacrx_completed_flag = 1;
	}

	hexdump(db->rx_buff, 256);
#if 0

	/* Check packet ready or not */
	while (1) {
		/* race warning: the first packet might arrive with
		 * the interrupts disabled, but the second will fix
		 * it
		 */
		rxcount = readl(db->membase + EMAC_RX_FBC_REG);

		if (netif_msg_rx_status(db))
			dev_dbg(db->dev, "RXCount: %x\n", rxcount);

		if ((db->skb_last != NULL) && (rxlen_last > 0)) {
			dev->stats.rx_bytes += rxlen_last;

			/* Pass to upper layer */
			db->skb_last->protocol = eth_type_trans(db->skb_last,
								dev);
			netif_rx(db->skb_last);
			dev->stats.rx_packets++;
			db->skb_last = NULL;
			rxlen_last = 0;

			reg_val = readl(db->membase + EMAC_RX_CTL_REG);
			reg_val &= ~EMAC_RX_CTL_DMA_EN;
			writel(reg_val, db->membase + EMAC_RX_CTL_REG);
		}

		if (!rxcount) {
			db->emacrx_completed_flag = 1;
			reg_val = readl(db->membase + EMAC_INT_CTL_REG);
			reg_val |= (0xf << 0) | (0x01 << 8);
			writel(reg_val, db->membase + EMAC_INT_CTL_REG);

			/* had one stuck? */
			rxcount = readl(db->membase + EMAC_RX_FBC_REG);
			if (!rxcount)
				return;
		}

		reg_val = readl(db->membase + EMAC_RX_IO_DATA_REG);
		if (netif_msg_rx_status(db))
			dev_dbg(db->dev, "receive header: %x\n", reg_val);
		if (reg_val != EMAC_UNDOCUMENTED_MAGIC) {
			/* disable RX */
			reg_val = readl(db->membase + EMAC_CTL_REG);
			writel(reg_val & ~EMAC_CTL_RX_EN,
			       db->membase + EMAC_CTL_REG);

			/* Flush RX FIFO */
			reg_val = readl(db->membase + EMAC_RX_CTL_REG);
			writel(reg_val | (1 << 3),
			       db->membase + EMAC_RX_CTL_REG);

			do {
				reg_val = readl(db->membase + EMAC_RX_CTL_REG);
			} while (reg_val & (1 << 3));

			/* enable RX */
			reg_val = readl(db->membase + EMAC_CTL_REG);
			writel(reg_val | EMAC_CTL_RX_EN,
			       db->membase + EMAC_CTL_REG);
			reg_val = readl(db->membase + EMAC_INT_CTL_REG);
			reg_val |= (0xf << 0) | (0x01 << 8);
			writel(reg_val, db->membase + EMAC_INT_CTL_REG);

			db->emacrx_completed_flag = 1;

			return;
		}

		/* A packet ready now  & Get status/length */
		good_packet = true;

		emac_inblk_32bit(db->membase + EMAC_RX_IO_DATA_REG,
				&rxhdr, sizeof(rxhdr));

		if (netif_msg_rx_status(db))
			dev_dbg(db->dev, "rxhdr: %x\n", *((int *)(&rxhdr)));

		rxlen = EMAC_RX_IO_DATA_LEN(rxhdr);
		rxstatus = EMAC_RX_IO_DATA_STATUS(rxhdr);

		if (netif_msg_rx_status(db))
			dev_dbg(db->dev, "RX: status %02x, length %04x\n",
				rxstatus, rxlen);

		/* Packet Status check */
		if (rxlen < 0x40) {
			good_packet = false;
			if (netif_msg_rx_err(db))
				dev_dbg(db->dev, "RX: Bad Packet (runt)\n");
		}

		if (unlikely(!(rxstatus & EMAC_RX_IO_DATA_STATUS_OK))) {
			good_packet = false;

			if (rxstatus & EMAC_RX_IO_DATA_STATUS_CRC_ERR) {
				if (netif_msg_rx_err(db))
					dev_dbg(db->dev, "crc error\n");
				dev->stats.rx_crc_errors++;
			}

			if (rxstatus & EMAC_RX_IO_DATA_STATUS_LEN_ERR) {
				if (netif_msg_rx_err(db))
					dev_dbg(db->dev, "length error\n");
				dev->stats.rx_length_errors++;
			}
		}

		/* Move data from EMAC */
		if (good_packet) {
			skb = netdev_alloc_skb(dev, rxlen + 4);
			if (!skb)
				continue;
			skb_reserve(skb, 2);
			rdptr = (u8 *) skb_put(skb, rxlen - 4);

			/* Read received packet from RX SRAM */
			if (netif_msg_rx_status(db))
				dev_dbg(db->dev, "RxLen %x\n", rxlen);

			emac_inblk_32bit(db->membase + EMAC_RX_IO_DATA_REG,
					rdptr, rxlen);
			dev->stats.rx_bytes += rxlen;

			/* Pass to upper layer */
			skb->protocol = eth_type_trans(skb, dev);
			netif_rx(skb);
			dev->stats.rx_packets++;
		}
	}
#endif
	dbg(" <\n");
}

static irqreturn_t emac_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct emac_board_info *db = netdev_priv(dev);
	unsigned int int_mask;
	unsigned int int_status;
	unsigned long flags;
	unsigned int reg_val;

	/* A real interrupt coming */

	/* holders of db->lock must always block IRQs */
	spin_lock_irqsave(&db->lock, flags);

	/* Disable all interrupts */
	// mask: 0x0003ff0f, status: 0x00800000
	int_mask = HWREG_R(db->base + EMAC_INT_MASK);
	HWREG_W(db->base + EMAC_INT_MASK, ~0);

	/* Got EMAC interrupt status */
	/* Got ISR */
	int_status = HWREG_R(db->base + EMAC_INT_STATUS);

	if (netif_msg_intr(db))
		dev_dbg(db->dev, "emac interrupt %08x\n", int_status);

	dbg("mask: 0x%08x, status: 0x%08X\n", int_mask, int_status);

	/* all done ? */
	if (int_status == 0) {
		/* Re-enable interrupt mask */
		HWREG_W(db->base + EMAC_INT_MASK, 0);
		dbg("all done\n");
		goto done; // fixme
	}

	reg_val = int_status % 0x10;

    if (int_status & ETH_ISR_RCVDN) {
        HWREG_W(db->base + EMAC_INT_STATUS, int_status & 0x3c00);

    	/* Received the coming packet */
    	if ((int_status & ETH_ISR_RXRDY) && (db->emacrx_completed_flag == 1)) {
    		/* start rx */
    		db->emacrx_completed_flag = 0;
    		dbg("start rx...\n");
    		emac_rx(dev);
    	}
    	/* disable rx interrupt */
        HWREG_W(db->base + EMAC_INT_STATUS, int_status & 0x3c200);
    }

    if (reg_val != 0) {
    	dbg(" reg_val = 0x%08X\n", reg_val);

#if 0
        int32_t v5 = *(int32_t *)(v2 + 4); // 0x6b4c
        int32_t v6 = 16 * *(int32_t *)(v5 + 116) + *(int32_t *)(v5 + 100); // 0x6b5c
        unsigned char v7 = *(char *)v6; // 0x6b60
        unsigned char v8 = *(char *)(v6 + 1); // 0x6b64
        unsigned char v9 = *(char *)(v6 + 2); // 0x6b68
        unsigned char v10 = *(char *)(v6 + 3); // 0x6b6c
        if ((0x10000 * (int32_t)v9 || 256 * (int32_t)v8 || (int32_t)v7 || 0x1000000 * (int32_t)v10) >= 0) {
            // 0x6b98
            return unknown_380();
        }
        // 0x6dbc
#endif
        HWREG_W(db->base + EMAC_INT_STATUS, reg_val);
        //return unknown_a8();
    }

    //0x00800000 == (1 << 23)
    if ((int_status & (1 << 23)) == 0) {
    	dbg("\n");
       // int_status ==  0x1000000
        if (int_status & (1 << 24)) {
            // 0x69e4
            HWREG_W(db->base + EMAC_INT_STATUS, (1 << 24));
            // branch -> 0x69ec
        }
		/* Re-enable interrupt mask */
        HWREG_W(db->base + EMAC_INT_MASK, 0);
        //return 0x1000000;
    }

    reg_val = HWREG_R(db->base + EMAC_PORT_CFGSTA);
    if (reg_val & 0x20) {
    	dbg(" reg_val = 0x%08X\n", reg_val);
       // 0x6ed8
        //*(int32_t *)-0x3480 = 0x800000;
        HWREG_W(db->base + EMAC_INT_STATUS, 1 << 23);
        //return unknown_b0();
    }

	/* Transmit Interrupt check */ // FIXME add it
	//if (int_status & (ETH_ISR_XMTDN | ETH_ISR_TBDR))
		//emac_tx_done(dev, db, int_status);


	//if (int_status & (0x04 | 0x08))
		//netdev_info(dev, " ab : %x\n", int_status);

done:
	/* Re-enable interrupt mask */
#if 0
	if (db->emacrx_completed_flag == 1) {
		reg_val = readl(db->base + EMAC_INT_CTL_REG);
		reg_val |= (0xf << 0) | (0x01 << 8);
		writel(reg_val, db->base + EMAC_INT_CTL_REG);
	}
#else
	//HWREG_W(db->base + EMAC_INT_MASK, 0);
	HWREG_W(db->base + EMAC_INT_STATUS, 0x800000);
#endif

	spin_unlock_irqrestore(&db->lock, flags);

	return IRQ_HANDLED;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Used by netconsole
 */
static void emac_poll_controller(struct net_device *dev)
{
	disable_irq(dev->irq);
	emac_interrupt(dev->irq, dev);
	enable_irq(dev->irq);
}
#endif

/*  Open the interface.
 *  The interface is opened whenever "ifconfig" actives it.
 */
static int emac_open(struct net_device *dev)
{
	struct emac_board_info *db = netdev_priv(dev);

	dbg(">\n");

	if (netif_msg_ifup(db))
		dev_dbg(db->dev, "enabling %s\n", dev->name);

	if (request_irq(dev->irq, &emac_interrupt, 0, dev->name, dev))
		return -EAGAIN;

	/* Initialize EMAC board */
	emac_reset(db);
	emac_init_device(dev);

	phy_start(db->phydev);
	netif_start_queue(dev);
	dbg("<\n");

	return 0;
}

static void emac_shutdown(struct net_device *dev)
{
	struct emac_board_info *db = netdev_priv(dev);
	dbg(">\n");

#if 0
	/* Disable all interrupt */
	HWREG_W(db->base + EMAC_INT_MASK, ~0);
	/* clear interrupt status */
	HWREG_W(db->base + EMAC_INT_STATUS, 0);
#endif

    HWREG_W(db->base + EMAC_INT_MASK, 0xFE7FFFFF);
    HWREG_W(db->base + EMAC_INT_STATUS, 0xFE7FFFFF);
    HWREG_W(db->base + EMAC_SYS_CFGCMD, HWREG_R(db->base + EMAC_SYS_CFGCMD) | 6);

    dbg("MAC_SYS_CFGCMD = 0x%08X\n", HWREG_R(db->base + EMAC_SYS_CFGCMD));
	/* Disable RX/TX */
}

/* Stop the interface.
 * The interface is stopped when it is brought.
 */
static int emac_stop(struct net_device *ndev)
{
	struct emac_board_info *db = netdev_priv(ndev);
	dbg(">\n");

	if (netif_msg_ifdown(db))
		dev_dbg(db->dev, "shutting down %s\n", ndev->name);

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);

	phy_stop(db->phydev);

	emac_shutdown(ndev);

	free_irq(ndev->irq, ndev);
	dbg("<\n");

	return 0;
}

static const struct net_device_ops emac_netdev_ops = {
	.ndo_open			= emac_open,
	.ndo_stop			= emac_stop,
	.ndo_start_xmit		= emac_start_xmit,
	.ndo_tx_timeout		= emac_timeout,
	.ndo_set_rx_mode	= emac_set_rx_mode,
	.ndo_do_ioctl		= emac_ioctl,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= emac_set_mac_address,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= emac_poll_controller,
#endif
};

/* Search EMAC board, allocate space and register it
 */
static int sphe15xx_emac_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct emac_board_info *db;
	struct net_device *ndev;
	struct resource	*res;
	struct resource *irq_res;
	int ret = 0;

	ndev = alloc_etherdev(sizeof(struct emac_board_info));
	if (!ndev) {
		dev_err(&pdev->dev, "could not allocate device.\n");
		return -ENOMEM;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);

	db = netdev_priv(ndev);
	memset(db, 0, sizeof(*db));

	db->dev = &pdev->dev;
	db->ndev = ndev;
	db->pdev = pdev;

	spin_lock_init(&db->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(res)) {
		dev_err(&pdev->dev, "failed to get emac resource\n");
		ret = -ENOENT;
		goto out;
	}

	db->membase = devm_ioremap_nocache(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(db->membase)) {
		dev_err(&pdev->dev, "failed to remap registers\n");
		ret = -ENOMEM;
		goto out;
	}

	db->base = res->start;
	/* fill in parameters for net-dev structure */
	ndev->base_addr = (unsigned long)db->membase;

	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (IS_ERR(irq_res)) {
		dev_err(&pdev->dev, "no IRQ resource\n");
		ret = -EINVAL;
		goto out_iounmap;
	}

	ndev->irq = irq_res->start;

	db->clk = devm_clk_get(&pdev->dev, "emac");
	if (IS_ERR(db->clk)) {
		dev_err(&pdev->dev, "Error couldn't get emac clock (%d)\n", ret);
		ret = PTR_ERR(db->clk);
		goto out_iounmap;
	}

	ret = clk_prepare_enable(db->clk);
	if (ret) {
		dev_err(&pdev->dev, "Error couldn't enable clock (%d)\n", ret);
		goto out_iounmap;
	}

	db->phy_interface = PHY_INTERFACE_MODE_RMII;

	/* Read MAC-address from OTP */ // use otp or crypto ic
	emac_set_ethr_address(ndev);

	/* Check if the MAC address is valid, if not get a random one */
	if (!is_valid_ether_addr(ndev->dev_addr)) {
		eth_hw_addr_random(ndev);
		dev_warn(&pdev->dev, "using random MAC address %pM\n",
			 ndev->dev_addr);
	}

	db->init_status = 0x100;
	db->emacrx_completed_flag = 1;
	emac_init_hw(ndev);
	emac_powerup(ndev);
	emac_reset(db);

	ndev->netdev_ops = &emac_netdev_ops;
	ndev->watchdog_timeo = msecs_to_jiffies(watchdog);
	ndev->ethtool_ops = &emac_ethtool_ops;

	platform_set_drvdata(pdev, ndev);

	/* Carrier starts down, phylib will bring it up */
	netif_carrier_off(ndev);

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "Registering netdev failed!\n");
		ret = -ENODEV;
		goto out_release;
	}

	dev_info(&pdev->dev, "%s: at %p, IRQ %d, mode: RMII, MAC: %pM\n",
		 ndev->name, db->membase, ndev->irq, ndev->dev_addr);

	ret = emac_mdio_probe(ndev);
	if (ret < 0) {
		netdev_err(&pdev->dev, "cannot probe MDIO bus\n");
		goto out_release;
	}

	return 0;

out_release:
	clk_disable_unprepare(db->clk);
out_iounmap:
	iounmap(db->membase);
out:
	dev_err(db->dev, "not found (%d).\n", ret);

	free_netdev(ndev);

	return ret;
}

static int sphe15xx_emac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct emac_board_info *db = netdev_priv(ndev);

	unregister_netdev(ndev);
	clk_disable_unprepare(db->clk);

	emac_mdio_remove(ndev);

	iounmap(db->membase);
	free_netdev(ndev);

	dev_dbg(&pdev->dev, "released and freed device\n");
	return 0;
}

static int sphe15xx_emac_suspend(struct platform_device *dev, pm_message_t state)
{
	struct net_device *ndev = platform_get_drvdata(dev);

	netif_carrier_off(ndev);
	netif_device_detach(ndev);
	emac_shutdown(ndev);

	return 0;
}

static int sphe15xx_emac_resume(struct platform_device *dev)
{
	struct net_device *ndev = platform_get_drvdata(dev);
	struct emac_board_info *db = netdev_priv(ndev);

	emac_reset(db);
	emac_init_device(ndev);
	netif_device_attach(ndev);

	return 0;
}

static const struct of_device_id sphe15xx_emac_of_match[] = {
	{.compatible = "sunplus,sphe15xx-emac",},
	{},
};

MODULE_DEVICE_TABLE(of, sphe15xx_emac_of_match);

static struct platform_driver sphe15xx_emac_driver = {
	.driver = {
		.name = "sphe15xx-emac",
		.of_match_table = sphe15xx_emac_of_match,
	},
	.probe = sphe15xx_emac_probe,
	.remove = sphe15xx_emac_remove,
	.suspend = sphe15xx_emac_suspend,
	.resume = sphe15xx_emac_resume,
};

module_platform_driver(sphe15xx_emac_driver);

MODULE_AUTHOR("Team-Proton <dev.team.proton@gmail.com>");
MODULE_DESCRIPTION("Sunplus SPHE15XX emac network driver");
MODULE_LICENSE("GPL");
