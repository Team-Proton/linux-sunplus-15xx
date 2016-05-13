/*
 * MTD SPI driver for ST M25Pxx (and similar) serial flash chips
 *
 * Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 * Some parts are based on lart.c by Abraham Van Der Merwe
 *
 * Cleaned up and generalized based on mtd_dataflash.c
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/crc32.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/spi-nor.h>
#include <asm/mach-sphe15xx/regmap.h>
#include <asm/mach-sphe15xx/sphe15xx.h>
#include <asm/mach-sphe15xx/spi.h>
#include <asm/mach-sphe15xx/irq.h>

#define	MAX_CMD_SIZE	6
#define SPI_CMD_BE      (0x20A3)
#define SPI_CMD_SE      (0xD8A3)
#define SPI_CMD_CE      (0xC7A0)

#define EON_QH256 		0x19701C0611LL

enum { OP_READ=0, OP_WRITE };

typedef enum
{
	MODE_LOW_FREQ 	= 0x0393, // ok
	MODE_HIGH_FREQ	= 0x0B93,
	MODE_DUAL_IO	= 0xBB93, // ok
	MODE_DUAL_SPI	= 0x3B93,
	MODE_QUAD_SPI	= 0x6b93,

} RW_SPEED;

typedef unsigned long long	u_int64;

struct m25p {
	struct spi_device	*spi;
	struct spi_nor		spi_nor;
	struct mutex		lock;

	u_char		buf[4096];
	u_char 		oneBit;
	u_int		readCmd;
	u_int64		flash_info;
};

__maybe_unused
static void SPI_Reg_Write(u_char reg, u_char val)
{
	u_int cmd = (reg << 8) | val;
    RF_SPI_CUST_CMD = cmd;
    while(RF_SPI_CUST_CMD & 0x80);
}

static u_int SPI_Reg_Read(u_char reg, u_char n)
{
	u_int data = 0;
	u_int cmd = (reg << 8) | (0x80 + 4*n);

    RF_SPI_DATA_HIGH = 0;
    RF_SPI_DATA_LOW  = 0;
    RF_SPI_ADDR_HIGH = 0;
    RF_SPI_ADDR_LOW  = 0;
    RF_SPI_CUST_CMD  = cmd;
    while(RF_SPI_CUST_CMD & 0x80);

    data  = (RF_SPI_DATA_LOW  & 0xFFFF);
    data |= (RF_SPI_DATA_HIGH & 0xFFFF) << 16;

    data = 	( data>>24) |
    		((data<<8) & 0x00FF0000) |
			((data>>8) & 0x0000FF00) |
			( data<<24);

	return (data >> 8);
}

static u_char SPI_Read_Byte(u_int cmd, u_int addr)
{
    SPI_API_4B_ADDR_ON();

    RF_SPI_ADDR_LOW  = (addr & 0xFFFF);
    RF_SPI_ADDR_HIGH = (addr>>16) & 0xFFFF;
    RF_SPI_DATA_HIGH = 0;
    RF_SPI_DATA_LOW  = 0;
    RF_SPI_CUST_CMD  = (cmd);
    while(RF_SPI_CUST_CMD & 0x80);

 	return (RF_SPI_DATA_LOW & 0xFF);
}

static u_char SPI_CMD_RDSR(void)
{
    u_char data;

    RF_SPI_CUST_CMD = 0x0584;
    while(RF_SPI_CUST_CMD & 0x80);

	data = RF_SPI_STATUS & 0xFF;
    return data;
}

static void SPI_CMD_WREN(void)
{
    RF_SPI_CUST_CMD = 0x06A0;
    while(RF_SPI_CUST_CMD & 0x80);
}

static void SPI_CMD_SP2(int mode)
{
    if (mode)   // SP2 mode
        RF_SPI_CUST_CMD = 0x0AA0;
    else        // SPI mode
        RF_SPI_CUST_CMD = 0x04A0;

    while(RF_SPI_CUST_CMD & 0x80);
}

static void SPI_Set_Speed(struct m25p *flash, u_int speed)
{
	if (flash->oneBit)
		return;

    SPI_API_PREFETCH_OFF();

    switch(speed)
    {
    default:
    case MODE_LOW_FREQ:
        RF_SPI_CFG4 &= ~(0x7<<8);
        RF_SPI_CFG5 &= ~(0x7<<8);
        SPI_CMD_SP2(0);
        RF_SPI_CTRL = 0x0117;
        RF_SPI_CFG3 &= ~(1<<6);
        flash->readCmd = MODE_LOW_FREQ;
    	break;

    case MODE_HIGH_FREQ:
    case MODE_DUAL_SPI:
        RF_SPI_CTRL = 0x1133;
        RF_SPI_CFG1 |= ((0x3B<<8) | (1<<4));
        RF_SPI_CFG4 = 0x0425;
        RF_SPI_CFG5 = 0x0425;
        RF_SPI_CFG6 = 0x0025;
        RF_SPI_CFG7 = 0x0025;
        flash->readCmd = speed;
        break;

    case MODE_QUAD_SPI:
    case MODE_DUAL_IO:
        RF_SPI_CTRL = 0x1134;
        RF_SPI_CFG1 |= ((0xBB<<8) | (1<<4));
        RF_SPI_CFG4 = 0x0229;
        RF_SPI_CFG5 = 0x0229;
        RF_SPI_CFG6 = 0x0001;
        RF_SPI_CFG7 = 0x0001;
        flash->readCmd = MODE_DUAL_IO;
        break;
   }

    SPI_API_PREFETCH_ON();
}

static void SPI_CMD_ERASE(u_int cmd, u_int addr)
{
    SPI_CMD_WREN();
    SPI_API_4B_ADDR_ON();

    RF_SPI_ADDR_LOW = addr&0xFFFF;
    RF_SPI_ADDR_HIGH = (addr>>16)&0xFFFF;
    RF_SPI_CUST_CMD = cmd;
    while(RF_SPI_CUST_CMD & 0x80);
    while((SPI_CMD_RDSR()&0x01));
}

static void SPI_doErase(struct m25p *flash, u_int cmd, u_int addr)
{
    SPI_Set_Speed(flash, MODE_LOW_FREQ);
    SPI_CMD_ERASE(cmd, addr);
}

static u_int SPI_CMD_ReadID(struct m25p *flash, u_int cmd)
{
    u32 flash_id = 0;

    SPI_Set_Speed(flash, MODE_LOW_FREQ);

    RF_SPI_DATA_HIGH = 0;
    RF_SPI_DATA_LOW = 0;
    RF_SPI_ADDR_HIGH = 0;
    RF_SPI_ADDR_LOW = 0;
    RF_SPI_CUST_CMD = cmd;
    while(RF_SPI_CUST_CMD & 0x80);

    flash_id = (RF_SPI_DATA_LOW & 0xFFFF);
    flash_id |= (RF_SPI_DATA_HIGH&0xFFFF)<<16;

    return flash_id;
}

__maybe_unused
static u_int SPI_ReadID(struct m25p *sp, u_int cmd)
{
	u_int id = SPI_CMD_ReadID(sp, cmd);

	id = (id>>24) |
         ((id<<8) & 0x00FF0000) |
         ((id>>8) & 0x0000FF00) |
         (id<<24);

	return (id >> 8);
}

__maybe_unused
static u_int SPI_CMD_READ(u_int cmd, u_int addr)
{
	u_int data = 0;

    SPI_API_4B_ADDR_ON();

    RF_SPI_ADDR_LOW  = (addr & 0xFFFF);
    RF_SPI_ADDR_HIGH = ((addr>>16) & 0xFFFF);
    RF_SPI_DATA_LOW  = 0;
    RF_SPI_DATA_HIGH = 0;
    RF_SPI_CUST_CMD = (cmd);
    while(RF_SPI_CUST_CMD & 0x80);

    data = ((RF_SPI_DATA_HIGH << 16) & 0xFFFF0000);
    data |= (RF_SPI_DATA_LOW & 0xFFFF);

    return data;
}

static int SPI_doRead(struct m25p *flash, u_int addr, u_char *buf, u_int len)
{
	u_int cnt = len;

    while(cnt > 0)
    {
    	if (cnt > 3) { // 4 bytes read
    		*(u_int*)buf = SPI_CMD_READ(flash->readCmd, addr);

            addr += 4;
            buf += 4;
            cnt -= 4;
    	}
    	else {
    		*buf++ = SPI_Read_Byte(flash->readCmd, addr++);
    		cnt--;
     	}
    }
    return len;
}

static void SPI_DATA64_WRITE( unsigned int addr, unsigned int *data, unsigned int write_cnt)
{
   unsigned int w_cnt;
   w_cnt = write_cnt & 0x1ff;


   SPI_CMD_WREN();
   SPI_API_4B_ADDR_ON();

   RF_SPI_ADDR_LOW = addr & 0xFFFF;
   RF_SPI_ADDR_HIGH = (addr>>16) & 0xFFFF;

   RF_SPI_CUST_CMD2 = 0x0400|w_cnt; // set use SPI_DATA64 with 256 byte count for SPI data
   RF_SPI_CUST_CMD  = 0x02a3;  		// write to flash,0 byte data xfer, 3byte address

   for(w_cnt=0;w_cnt<(write_cnt/4);w_cnt++){
    	RF_SPI_DATA_64  = *data & 0xFFFF;
    	RF_SPI_DATA_64  = (*data>>16) & 0xFFFF;
    	data++;
    }

    while(RF_SPI_CUST_CMD & 0x80);
    while((SPI_CMD_RDSR() & 0x01));
    RF_SPI_CUST_CMD2 = 0x0;	// reset
}

#if 0
static void SPI_WRITE_DWORD(unsigned int addr, unsigned int data)
{
    SPI_CMD_WREN();
    SPI_API_4B_ADDR_ON();

    RF_SPI_ADDR_LOW = addr & 0xFFFF;
    RF_SPI_ADDR_HIGH = (addr>>16) & 0xFFFF;
    RF_SPI_DATA_LOW = data & 0xFFFF;
    RF_SPI_DATA_HIGH = (data>>16) & 0xFFFF;
    RF_SPI_CUST_CMD = 0x02B3;
    while(RF_SPI_CUST_CMD & 0x80);
    while((SPI_CMD_RDSR() & 0x01));
}

static void SPI_WRITE_BYTE(unsigned int addr, unsigned int data)
{
    SPI_CMD_WREN();
    SPI_API_4B_ADDR_ON();

    RF_SPI_ADDR_LOW = addr & 0xFFFF;
    RF_SPI_ADDR_HIGH = (addr>>16) & 0xFFFF;
    RF_SPI_DATA_LOW =  data & 0xFF;
    RF_SPI_DATA_HIGH = 0;
    RF_SPI_CUST_CMD = 0x02B3;
    while(RF_SPI_CUST_CMD & 0x80);
    while((SPI_CMD_RDSR() & 0x01));
}

static int SPI_Write(struct m25p *flash, u_int addr, u_char *buf, u_int len)
{
	u_int cnt = len;

    while(cnt > 0)
    {
    	if (cnt > 3) { // 4 bytes write
    		SPI_WRITE_DWORD(addr, *(u_int*)buf);

            addr += 4;
            buf += 4;
            cnt -= 4;
    	}
    	else {
    		SPI_WRITE_BYTE(addr++, *buf++);
    		cnt--;
     	}
    }
    return len;
}
#endif

u_int MAX_SPI_page_write_cnt = 64;

static u_int SPI_doWrite(struct m25p *flash, u_int addr, const u_char *buf, u_int len)
{
    u_int *ptr;
    u_int totalLen;
    u_int targetAddr;
    u_int page_write_cnt;
    u_char *pBuf = NULL;

    addr &= 0x01FFFFFF;

    if (((addr&(0x03)) == 0) && ((len&(0x03)) == 0) && (((u32)buf&(0x03)) == 0))
    {
        targetAddr = addr;
        totalLen = len;
        ptr = (u32*)buf;
    }
    else
    {
        targetAddr = addr&(~(0x03));
        totalLen = (len + (addr%4) + 3)&(~(0x03));
        pBuf = kmalloc(totalLen, GFP_KERNEL);
        if (pBuf == NULL)
        {
            printk("[m25pxx] no more free memory for writing!!\n");
            return 0;
        }
        ptr = (u32*)pBuf;
        memset(pBuf, 0xFF, totalLen);
        memcpy((pBuf+(addr%4)), buf, len);
    }

    while(totalLen)
    {
		page_write_cnt = totalLen;
		if( page_write_cnt >= MAX_SPI_page_write_cnt) page_write_cnt= MAX_SPI_page_write_cnt;
		SPI_DATA64_WRITE(targetAddr, ptr, page_write_cnt);
        targetAddr += page_write_cnt;
        ptr+=(page_write_cnt>>2);
        totalLen -= page_write_cnt;
    }

    if (pBuf)
    {
        kfree(pBuf);
    }

    return len;
}

static u_int SPI_ReadWrite(struct m25p *flash, u_int rw, u_int speed,
		u_int addr, u_char *rBuf, const u_char *pBuf, u_int len)
{
	int ret = 0;
	int size = sizeof(flash->buf);
	u_int offset = addr & 0x1FFF000;

	mutex_lock(&flash->lock);

    SPI_Set_Speed(flash, speed);

    if (rw == OP_READ)
    	ret = SPI_doRead(flash, addr, rBuf, len);
    else
    {
    	if (len <= size && size == 4096) {
    		SPI_doRead(flash, offset, flash->buf, size);
    		SPI_doErase(flash, SPI_CMD_BE, offset);
    		memcpy(&flash->buf[addr-offset], pBuf, len);
    		ret = SPI_doWrite(flash, offset, flash->buf, size);
    		if (ret == size)
    			ret = len;
    	}
    }

	mutex_unlock(&flash->lock);

	return ret;
}

static u_char *i2b_buf(int n, u_int i, u_char *b)
{
  switch(n)
  {
    case 1:
      b[0]=(i    ) & 0xff;
      break;

    case 2:
      b[0]=(i>> 8) & 0xff;
      b[1]=(i    ) & 0xff;
      break;

    case 3:
      b[0]=(i>>16) & 0xff;
      b[1]=(i>> 8) & 0xff;
      b[2]=(i    ) & 0xff;
      break;

    case 4:
      b[0]=(i>>24) & 0xff;
      b[1]=(i>>16) & 0xff;
      b[2]=(i>> 8) & 0xff;
      b[3]=(i    ) & 0xff;
      break;

    case 5:
    case 6:
      b[5]=0;
      b[4]=(i>>28) & 0xff;
      b[3]=(i>>24) & 0xff;
      b[0]=(i>>16) & 0xff;
      b[1]=(i>> 8) & 0xff;
      b[2]=(i    ) & 0xff;
      break;
  }

  return(b);
}

__maybe_unused
static u_int b2i(int n, u_char *b)
{
	u_int v=0;
	switch(n)
	{
    case 1:
      v = (b[0]); break;
    case 2:
      v = (b[0]<<8) | b[1]; break;
    case 3:
      v = (b[0]<<16) | (b[1]<<8) | b[2]; break;
    case 4:
      v = (b[0]<<24) | (b[1]<<16) | (b[2]<<8) | b[3]; break;
	}
	return v;
}

static int m25pxx_read_reg(struct spi_nor *nor, u_char opcode, u_char *buf, int len)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	int ret = 0;
	u_int val = 0;

	SPI_Set_Speed(flash, MODE_LOW_FREQ);

	switch (opcode)
	{
	case SPINOR_OP_RDSR:
		buf[0] = 0;//SPI_CMD_RDSR();
		//printk("m25pxx spi0.0: status: %02x\n", buf[0]);
		break;

	default:
		val = SPI_Reg_Read(opcode, (len > 4)? 4:len);
		i2b_buf(len, val, buf);
	}

	if (opcode == SPINOR_OP_RDID)
		printk("m25pxx spi0.0: jedec id: %08x\n", val);

	if (ret < 0)
		dev_err(&spi->dev, "read reg 0x%02x error (%d)\n", opcode, ret);

	return ret;
}

static int m25pxx_write_reg(struct spi_nor *nor, u_char opcode, u_char *buf, int len)
{
	int ret = 0;
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;

	SPI_Set_Speed(flash, MODE_LOW_FREQ);

	switch(opcode)
	{
	case SPINOR_OP_WREN:
		//SPI_CMD_WREN();
		break;

	case SPINOR_OP_CHIP_ERASE:
		SPI_doErase(flash, SPI_CMD_CE, 0);
		break;

	case 0x0A:
	case SPINOR_OP_WRDI:
		break;

	default:
		ret = -1;// not supported
#if 0
		if (len && buf)
			SPI_Reg_Write(opcode, buf[0]);
		else
			SPI_Reg_Write(opcode, 0xA0);
#endif
		break;
	}

	if (ret < 0)
		dev_err(&spi->dev, "write reg 0x%02x error (%d)\n", opcode, ret);

	return ret;
}

static void m25pxx_write(struct spi_nor *nor, loff_t to, size_t len,
			size_t *retlen, const u_char *buf)
{
	struct m25p *flash = nor->priv;
	u_int speed = MODE_LOW_FREQ;
	u_char *rBuf = NULL;

	switch (nor->program_opcode)
	{
	default:
		break;
	}
/*
	printk("---> program() opcode: 0x%02X, addr: 0x%08X, len: 0x%X\n",
			nor->program_opcode, (u_int)to, (u_int)len);
*/
	*retlen += SPI_ReadWrite(flash, OP_WRITE, speed, (u_int)to, rBuf, buf, (u_int)len);
}

#if 0
static inline unsigned int m25pxx_rx_nbits(struct spi_nor *nor)
{
	switch (nor->flash_read) {
	case SPI_NOR_DUAL:
		return 2;
	case SPI_NOR_QUAD:
		return 4;
	default:
		return 0;
	}
}
#endif

/*
 * Read an address range from the nor chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int m25pxx_read(struct spi_nor *nor, loff_t from, size_t len,
			size_t *retlen, u_char *buf)
{
	u32 speed = MODE_LOW_FREQ;
	struct m25p *flash = nor->priv;

	switch (nor->read_opcode)
	{
	case SPINOR_OP_READ:			// low frequency
		speed = MODE_LOW_FREQ;
		break;

	case SPINOR_OP_READ_FAST:		// high frequency
		speed = MODE_HIGH_FREQ;
		break;

	case SPINOR_OP_READ_1_1_2:		// Dual SPI
		speed = MODE_DUAL_SPI;
		break;

	case SPINOR_OP_READ_1_1_4:		// Quad SPI
		speed = MODE_QUAD_SPI;
		break;
	}

	*retlen = SPI_ReadWrite(flash, OP_READ, speed, (u32)from, buf, NULL, (u32)len);
	return 0;
}

static int m25pxx_erase(struct spi_nor *nor, loff_t offset)
{
	struct m25p *flash = nor->priv;
	int ret;

	dev_dbg(nor->dev, "%dKiB at 0x%08x\n",
		flash->spi_nor.mtd.erasesize / 1024, (u_int)offset);

	/* Send write enable, then erase commands. */
	ret = nor->write_reg(nor, SPINOR_OP_WREN, NULL, 0);
	if (ret)
		return ret;

	//printk("---> erase() opcode: 0x%02X, addr: 0x%08X\n", nor->erase_opcode, (u_int)offset);

	switch (nor->erase_opcode)
	{
	case SPINOR_OP_SE: 			// 64k
		SPI_doErase(flash, SPI_CMD_SE, (u_int)offset);
		break;
	case SPINOR_OP_BE_4K: 		// 4k
		SPI_doErase(flash, SPI_CMD_BE, (u_int)offset);
		break;
	case SPINOR_OP_CHIP_ERASE:	// full chip erase
		SPI_doErase(flash, SPI_CMD_CE, (u_int)offset);
		break;
	default:
		return -1;
	}

	return 0;
}

static void SPI_Init(void)
{
	HWREG_W(SPI_CONTROL, 0x114);
	HWREG_W(SPI_WAIT_CYC, 0x12A);
	HWREG_W(SPI_CUSTOM_CMD, 0x13);
	HWREG_W(SPI_ADDR_L, 0x0);
	HWREG_W(SPI_ADDR_H, 0x0);
	HWREG_W(SPI_DATA_L, 0x0);
	HWREG_W(SPI_DATA_H, 0x0);
	HWREG_W(ADDR_OF(SPI_CFG,0), 0x0);
	HWREG_W(ADDR_OF(SPI_CFG,1), 0x3B40);
	HWREG_W(ADDR_OF(SPI_CFG,2), 0x1200);
	HWREG_W(ADDR_OF(SPI_CFG,3), 0x9);
	HWREG_W(ADDR_OF(SPI_CFG,4), 0x15);
	HWREG_W(ADDR_OF(SPI_CFG,5), 0x15);
	HWREG_W(ADDR_OF(SPI_CFG,6), 0x95);
	HWREG_W(ADDR_OF(SPI_CFG,7), 0x95);
	HWREG_W(ADDR_OF(SPI_CFG,8), 0x0);
	HWREG_W(SPI_CUST_CMD_2, 0x4);

    // SETUP DPLL0 CLOCK
    HWREG_W(DPLL0_REMAINDER, 110);   // 190MHZ: 110
    HWREG_W(DPLL0_DENOMINATOR, 276); // 190MHZ: 276
    HWREG_W(DPLL0_DIVIDER, 0X1);     // 190MHZ: 1

    // SETUP DPLL1 CLOCK
    HWREG_W(DPLL1_REMAINDER, 264);   // 138MHZ: 264
    HWREG_W(DPLL1_DENOMINATOR, 276); // 138MHZ: 276
    HWREG_W(DPLL1_DIVIDER, 0X1);     // 138MHZ: 1

    HWREG_W(DPLL1_CTRL, 0X1);
    HWREG_W(DPLL0_CTRL, 0X1);

    HWREG_W(SFT_CFG_1, HWREG_R(SFT_CFG_1) | (1<<7));
    HWREG_W(SFT_CFG_1, HWREG_R(SFT_CFG_1) | (1<<4));
    HWREG_W(SFT_CFG_1, HWREG_R(SFT_CFG_1) | (1<<3));

    CLR_BIT(SFT_CFG_8, 3); // SFT_CFG_8[3]=0
    SET_BIT(SFT_CFG_8, 11);
    SET_BIT(SFT_CFG_8, 3); // SFT_CFG_8[3]=1, FINISH RESET PROCESS

    udelay(1000);

    HWREG_W(ADDR_OF(PAD_CTRL,0), 0XFFFFFFFF);
    HWREG_W(ADDR_OF(PAD_CTRL,2), 0X00200004);

    SET_BIT(SFT_CFG_1, 0); // ENABLE SERVO INTERFACE OF QAE377A

#if 0
    HWREG_W(SERVO_ADDR, RISC_WR_EN);
    HWREG_W(SERVO_WDATA, 0x01);

    // Set SPI clock DIV=16
    HWREG_W(SERVO_ADDR, SPI_CLK_DIV);
    HWREG_W(SERVO_WDATA, 0x03);
#endif

    RF_SPI_CFG3 |= 1;
    RF_SPI_WAIT = 0x0105;
}

/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int m25pxx_probe(struct spi_device *spi)
{
	struct mtd_part_parser_data	ppdata;
	struct flash_platform_data	*data;
	struct m25p *flash;
	struct spi_nor *nor;
	enum read_mode mode = SPI_NOR_NORMAL;
	char *flash_name = NULL;
	int ret;

	data = dev_get_platdata(&spi->dev);

	flash = devm_kzalloc(&spi->dev, sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	nor = &flash->spi_nor;

	/* install the hooks */
	nor->read = m25pxx_read;
	nor->write = m25pxx_write;
	nor->erase = m25pxx_erase;
	nor->write_reg = m25pxx_write_reg;
	nor->read_reg = m25pxx_read_reg;

	nor->dev = &spi->dev;
	nor->flash_node = spi->dev.of_node;
	nor->priv = flash;

	spi_set_drvdata(spi, flash);
	flash->spi = spi;

	if (spi->mode & SPI_RX_QUAD)
		mode = SPI_NOR_QUAD;
	else if (spi->mode & SPI_RX_DUAL)
		mode = SPI_NOR_DUAL;

	if (data && data->name)
		nor->mtd.name = data->name;

	mutex_init(&flash->lock);

	/* For some (historical?) reason many platforms provide two different
	 * names in flash_platform_data: "name" and "type". Quite often name is
	 * set to "m25pxx" and then "type" provides a real chip name.
	 * If that's the case, respect "type" and ignore a "name".
	 */
	if (data && data->type)
		flash_name = data->type;
	else
		flash_name = spi->modalias;

	SPI_Init();

    flash->flash_info = EON_QH256;

    flash->oneBit = 0;
    SPI_Set_Speed(flash, MODE_LOW_FREQ);
    flash->oneBit = 1;

	ret = spi_nor_scan(nor, flash_name, mode);
	if (ret)
		return ret;

	ppdata.of_node = spi->dev.of_node;

	return mtd_device_parse_register(&nor->mtd, NULL, &ppdata,
			data ? data->parts : NULL,
			data ? data->nr_parts : 0);
}

static int m25pxx_remove(struct spi_device *spi)
{
	struct m25p	*flash = spi_get_drvdata(spi);

	if (mutex_is_locked(&flash->lock))
		mutex_unlock(&flash->lock);

	mutex_destroy(&flash->lock);

	/* Clean up MTD stuff. */
	return mtd_device_unregister(&flash->spi_nor.mtd);
}

/*
 * XXX This needs to be kept in sync with spi_nor_ids.  We can't share
 * it with spi-nor, because if this is built as a module then modpost
 * won't be able to read it and add appropriate aliases.
 */
static const struct spi_device_id m25pxx_ids[] = {
	{"at25fs010"},	{"at25fs040"},	{"at25df041a"},	{"at25df321a"},
	{"at25df641"},	{"at26f004"},	{"at26df081a"},	{"at26df161a"},
	{"at26df321"},	{"at45db081d"},
	{"en25f32"},	{"en25p32"},	{"en25q32b"},	{"en25p64"},
	{"en25q64"},	{"en25qh128"},	{"en25qh256"},
	{"f25l32pa"},
	{"mr25h256"},	{"mr25h10"},
	{"gd25q32"},	{"gd25q64"},
	{"160s33b"},	{"320s33b"},	{"640s33b"},
	{"mx25l2005a"},	{"mx25l4005a"},	{"mx25l8005"},	{"mx25l1606e"},
	{"mx25l3205d"},	{"mx25l3255e"},	{"mx25l6405d"},	{"mx25l12805d"},
	{"mx25l12855e"},{"mx25l25635e"},{"mx25l25655e"},{"mx66l51235l"},
	{"mx66l1g55g"},
	{"n25q064"},	{"n25q128a11"},	{"n25q128a13"},	{"n25q256a"},
	{"n25q512a"},	{"n25q512ax3"},	{"n25q00"},
	{"pm25lv512"},	{"pm25lv010"},	{"pm25lq032"},
	{"s25sl032p"},	{"s25sl064p"},	{"s25fl256s0"},	{"s25fl256s1"},
	{"s25fl512s"},	{"s70fl01gs"},	{"s25sl12800"},	{"s25sl12801"},
	{"s25fl129p0"},	{"s25fl129p1"},	{"s25sl004a"},	{"s25sl008a"},
	{"s25sl016a"},	{"s25sl032a"},	{"s25sl064a"},	{"s25fl008k"},
	{"s25fl016k"},	{"s25fl064k"},
	{"sst25vf040b"},{"sst25vf080b"},{"sst25vf016b"},{"sst25vf032b"},
	{"sst25vf064c"},{"sst25wf512"},	{"sst25wf010"},	{"sst25wf020"},
	{"sst25wf040"},
	{"m25pxx"},	{"m25p10"},	{"m25p20"},	{"m25p40"},
	{"m25p80"},	{"m25p16"},	{"m25p32"},	{"m25p64"},
	{"m25p128"},	{"n25q032"},
	{"m25p05-nonjedec"},	{"m25p10-nonjedec"},	{"m25p20-nonjedec"},
	{"m25p40-nonjedec"},	{"m25p80-nonjedec"},	{"m25p16-nonjedec"},
	{"m25p32-nonjedec"},	{"m25p64-nonjedec"},	{"m25p128-nonjedec"},
	{"m45pe10"},	{"m45pe80"},	{"m45pe16"},
	{"m25pe20"},	{"m25pe80"},	{"m25pe16"},
	{"m25px16"},	{"m25px32"},	{"m25px32-s0"},	{"m25px32-s1"},
	{"m25px64"},	{"m25px80"},
	{"w25x10"},	{"w25x20"},	{"w25x40"},	{"w25x80"},
	{"w25x16"},	{"w25x32"},	{"w25q32"},	{"w25q32dw"},
	{"w25x64"},	{"w25q64"},	{"w25q80"},	{"w25q80bl"},
	{"w25q128"},	{"w25q256"},	{"cat25c11"},
	{"cat25c03"},	{"cat25c09"},	{"cat25c17"},	{"cat25128"},
	{ },
};
MODULE_DEVICE_TABLE(spi, m25pxx_ids);

static const struct of_device_id m25pxx_of_table[] = {
	/*
	 * Generic compatibility for SPI NOR that can be identified by the
	 * JEDEC READ ID opcode (0x9F). Use this, if possible.
	 */
	{ .compatible = "jedec,spi-nor" },
	{}
};
MODULE_DEVICE_TABLE(of, m25pxx_of_table);

static struct spi_driver m25pxx_driver = {
	.driver = {
		.name	= "m25pxx",
		.of_match_table = m25pxx_of_table,
	},
	.id_table	= m25pxx_ids,
	.probe	= m25pxx_probe,
	.remove	= m25pxx_remove,

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};

module_spi_driver(m25pxx_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Team-Proton <dev.team.proton@gmail.com>");
MODULE_DESCRIPTION("MTD SPI driver for ST M25Pxx flash chips");
