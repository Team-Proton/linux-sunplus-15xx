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

#ifndef _SPHE15XX_CLOCK_H_
#define _SPHE15XX_CLOCK_H_


typedef enum {
	DPLL			= 31,
	DEMUX2			= 30,
	DEMUX			= 29,
	SDIO			= 28,
	NAND			= 27,
	CARD			= 26,
	CPS				= 25,
	HOST			= 24,
	APP				= 23,
	AUD				= 22,
	PERI			= 21,
	PCMCIA			= 20,
	OTP_I2C			= 19,
	UART1			= 18,
	UART0			= 17,
	STC				= 16,
	SDCTRL1			= 15,
	MBUS1			= 14,
	SDCTRL0			= 13,
	MBUS0			= 12,
	// reserved		= 11,
	SPI_M			= 10,
	CB_DMA			=  9,
	RGST			=  8,
	BOOTROM			=  7,
	CBSW			=  6,
	// reserved		=  5,
	IOP				=  4,
	DSP				=  3,
	RISC1			=  2,
	RISC0			=  1,
	SYSTEM			=  0,

	DUMMY			= -1,

	// reserved		= 31 + 32,
	// reserved		= 30 + 32,
	LFRV			= 29 + 32,
	MPRV			= 28 + 32,
	LF264			= 27 + 32,
	MR264			= 26 + 32,
	QT264			= 25 + 32,
	MP264			= 24 + 32,
	LFVC1			= 23 + 32,
	MPVC1			= 22 + 32,
	MR9421			= 21 + 32,
	QT9421			= 20 + 32,
	VLD_AGDC		= 19 + 32,
	PICPARM			= 18 + 32,
	RG				= 17 + 32,
	DMX				= 16 + 32,
	MC_HD2SD		= 15 + 32,
	// reserved		= 14 + 32,
	PNG				= 13 + 32,
	JR				= 12 + 32,
	JPG_IQT			= 11 + 32,
	HUFF			= 10 + 32,
	GPU				=  9 + 32,
	FONT			=  8 + 32,
	// reserved		=  7 + 32,
	// reserved		=  6 + 32,
	M2S				=  5 + 32,
	SRV				=  4 + 32,
	// reserved		=  3 + 32,
	MAC				=  2 + 32,
	USB_SHCI		=  1 + 32,
	USB_EHCI		=  0 + 32,

	// reserved		= 31 + 64,
	// reserved		= 30 + 64,
	// reserved		= 29 + 64,
	// reserved		= 28 + 64,
	// reserved		= 27 + 64,
	// reserved		= 26 + 64,
	// reserved		= 25 + 64,
	// reserved		= 24 + 64,
	// reserved		= 23 + 64,
	// reserved		= 22 + 64,
	// reserved		= 21 + 64,
	// reserved		= 20 + 64,
	// reserved		= 19 + 64,
	// reserved		= 18 + 64,
	// reserved		= 17 + 64,
	DUMMY_MASTER	= 16 + 64,
	CAS				= 15 + 64,
	V656_IN			= 14 + 64,
	HDMI_TX			= 13 + 64,
	TTX_P2S			= 12 + 64,
	TV_DVD2			= 11 + 64,
	AFRC			= 10 + 64,
	HD2SD			=  9 + 64,
	DMIX			=  8 + 64,
	DSCL			=  7 + 64,
	AMIX			=  6 + 64,
	OSD				=  5 + 64,
	G2D				=  4 + 64,
	PG				=  3 + 64,
	VPP2			=  2 + 64,
	VPP				=  1 + 64,
	TGEN			=  0 + 64
} moduleList;

#define SAFE_CLKEN_ADD_ON(m, ori, cnt)								\
	do {															\
		unsigned int flag;											\
		HAL_DISABLE_INTERRUPTS(flag);								\
		(ori) = HWREG_R(ADDR_OF(CLKEN,(m)/32));							\
		if (0 == (cnt)) {											\
			HWREG_W(ADDR_OF(CLKEN,(m)/32), (ori) | (1 << ((m) & 0x1F)));	\
		}															\
		++(cnt);													\
		HAL_RESTORE_INTERRUPTS(flag);								\
	} while (0)

#define SAFE_CLKEN_SUB_OFF(m, ori, cnt)								\
	do {															\
		unsigned int flag;											\
		HAL_DISABLE_INTERRUPTS(flag);								\
		(ori) = HWREG_R(ADDR_OF(CLKEN,(m)/32));							\
		if (1 == (cnt)) {											\
			HWREG_W(ADDR_OF(CLKEN,(m)/32), (ori) & ~(1 << ((m) & 0x1F)));	\
		}															\
		--(cnt);													\
		HAL_RESTORE_INTERRUPTS(flag);								\
	} while (0)

int sphe15xx_clk_on(moduleList m);
int sphe15xx_clk_off(moduleList m);
void sphe15xx_clk_off_ex(int n, ...);

#endif /* _SPHE15XX_CLOCK_H_ */
