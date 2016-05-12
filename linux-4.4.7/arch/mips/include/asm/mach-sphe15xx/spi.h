/*
 *  Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 *  Platform data definition for Sunplus SPHE15XX SPI controller
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#ifndef _SPHE15XX_SPI_PLATFORM_H
#define _SPHE15XX_SPI_PLATFORM_H

struct sphe15xx_spi_platform_data {
	unsigned	bus_num;
	unsigned	num_chipselect;
};

struct sphe15xx_spi_controller_data {
	unsigned	gpio;
};

#define RISC_WR_EN              0x44DD
#define AFE_PAGE_SEL            0x7F
#define SPI_ADDR            	0x489A
#define SPI_DATA            	0x489B
#define SPI_CLK_DIV         	0x489C
//#define SPI_STATUS          	0x489D
#define TST_PIN_SIP_CLK_CTRL 	0x48ab

#define CLR_BIT(x,y)   			HWREG_W(x, HWREG_R(x) & ~(1<<y))
#define SET_BIT(x,y)    		HWREG_W(x, HWREG_R(x) | (1<<y))

// Register
#define RF_SPI_CTRL        		(HWREG_R(SPI_CONTROL))			// 0
#define RF_SPI_WAIT        		(HWREG_R(SPI_WAIT_CYC))      	// 1
#define RF_SPI_CUST_CMD    		(HWREG_R(SPI_CUSTOM_CMD))   	// 2
#define RF_SPI_ADDR_LOW    		(HWREG_R(SPI_ADDR_L))        	// 3
#define RF_SPI_ADDR_HIGH   		(HWREG_R(SPI_ADDR_H))        	// 4
#define RF_SPI_DATA_LOW    		(HWREG_R(SPI_DATA_L))        	// 5
#define RF_SPI_DATA_HIGH  		(HWREG_R(SPI_DATA_H))        	// 6
#define RF_SPI_STATUS      		(HWREG_R(SPI_STATUS))        	// 7
#define RF_SPI_CFG0        		(HWREG_R(ADDR_OF(SPI_CFG,0)))  	// 8
#define RF_SPI_CFG1        		(HWREG_R(ADDR_OF(SPI_CFG,1)))	// 9
#define RF_SPI_CFG2        		(HWREG_R(ADDR_OF(SPI_CFG,2)))   // 10
#define RF_SPI_CFG3        		(HWREG_R(ADDR_OF(SPI_CFG,3)))   // 11
#define RF_SPI_CFG4        		(HWREG_R(ADDR_OF(SPI_CFG,4)))   // 12
#define RF_SPI_CFG5        		(HWREG_R(ADDR_OF(SPI_CFG,5)))   // 13
#define RF_SPI_CFG6        		(HWREG_R(ADDR_OF(SPI_CFG,6)))   // 14
#define RF_SPI_CFG7        		(HWREG_R(ADDR_OF(SPI_CFG,7)))   // 15
#define RF_SPI_CFG8        		(HWREG_R(ADDR_OF(SPI_CFG,8)))   // 16
#define RF_SPI_CUST_CMD2   		(HWREG_R(SPI_CUST_CMD_2))   	// 17
#define RF_SPI_DATA_64     		(HWREG_R(SPI_DATA_64))       	// 18
#define RF_SPI_BUFFER_ADDR 		(HWREG_R(SPI_BUF_ADDR))      	// 19
#define RF_SPI_STATUS_2    		(HWREG_R(SPI_STATUS_2))      	// 20

#define PREFETCH   				(1<<6)
#define SPI_API_PREFETCH_ON()   (RF_SPI_CFG1 |= PREFETCH)
#define SPI_API_PREFETCH_OFF()  (RF_SPI_CFG1 &= ~PREFETCH)
#define SPI_API_4B_ADDR_ON()    (RF_SPI_CTRL |= (1<<10))

#define SPI_WRITE_PAGE_DATA64

#endif /* _SPHE15XX_SPI_PLATFORM_H */
