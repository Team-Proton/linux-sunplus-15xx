/*
 *  Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
 *
 *  Board data definition for Sunplus SPHE15XX
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#ifndef _SPHE15XX_BOARD_H
#define _SPHE15XX_BOARD_H

#define DVBS_216PIN_SZ11089_PINMUX  30

#define FTL_SUPPORT				0
#define PRIMARY_LIVE_INPUT		1
#define SECOND_LIVE_INPUT		0

#define	SUPPORT_3IN1_CARD
#define	SUPPORT_3IN1_CARD_TWO_SENSE_PIN

#if (FTL_SUPPORT == 1)
	#define	SUPPORT_NAND_FLASH
#endif

#define	SUPPORT_SPDIF_IN

#define SUPPORT_SPDIF_OUT

#define	SUPPORT_SERIAL_TS

#if defined(SMARTCARD_USE_UART1) || defined(SUPPORT_DUAL_SMARTCARD)
#define	SUPPORT_SMART_CARD
#endif
#if defined(SMARTCARD_USE_UART2) || defined(SUPPORT_DUAL_SMARTCARD)
#define	SUPPORT_SMART_CARD1
#endif

#define SUPPORT_HDMI

#define SUPPORT_SCART_TV
#ifdef SUPPORT_SCART_TV
#define SUPPORT_SCART_AUTO_LOOP
#endif

#define SUPPORT_GPIO_MUTE

#define STB_GPIO_MUTE

#define SUPPORT_IR

#define EPHY_LED_PINOUT1

#define SUPPORT_VFD

//#define SUPPORT_UART_NORMAL

#define SUPPORT_DVBS_NIM

#endif /* _SPHE15XX_BOARD_H */
