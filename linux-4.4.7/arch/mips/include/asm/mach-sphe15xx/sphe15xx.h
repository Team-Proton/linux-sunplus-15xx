/*
 * Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
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

#ifndef __SPHE15XX_H__
#define __SPHE15XX_H__

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/errno.h>

#include <asm/addrspace.h>
#include <asm/mach-sphe15xx/cyg_type.h>

#define SPHE_MEM_BASE					0x80000000
#define SPHE_MEM_END					0x88000000
#define SPHE_MEM_SIZE					(SPHE_MEM_END - SPHE_MEM_BASE)

#define HAL_REG(x)              		(volatile unsigned int *)(x)
#define HAL_REG8(x)             		(volatile unsigned char *)(x)
#define HAL_REG16(x)            		(volatile unsigned short *)(x)

#define HAL_REGISTER_BASE 	 			0xBFFE8000

#define HAL_PCI_PHYSICAL_MEMORY_BASE    0xa0000000
#define HAL_PCI_PHYSICAL_IO_BASE        0xa0000000

#define SP_PCI_CFG_REG_START            0xbc010000
#define SP_PCI_CFG_REG_PHY_START        0x1c010000

#define SP_PCI_CFG_REG_ADDR             0xbc010cf8
#define SP_PCI_CFG_REG_DATA             0xbc010cfc

#define rSP_PCI_CFG_REG_ADDR            *((volatile cyg_uint32 *)SP_PCI_CFG_REG_ADDR)
#define rSP_PCI_CFG_REG_DATA            *((volatile cyg_uint32 *)SP_PCI_CFG_REG_DATA)

/*
 * Registers
 */
#define	RF_GRP(group, reg)				(4 * (((group) - 256) * 32 + (reg)))
#define RF_ADDR(group,reg)				(RF_GRP(group, reg) + HAL_REGISTER_BASE)

/* LED Register */
#define HAL_STAMP						RF_ADDR(0, 0)

#define HAL_LEVEL1_FLAG					RF_ADDR(9, 0)
#define HAL_LEVEL1_POLARITY				RF_ADDR(9, 1)
#define HAL_LEVEL1_EDGE					RF_ADDR(9, 2)
#define HAL_RISC0_LEVEL1_SELECT			RF_ADDR(9, 3)

#define HAL_RISC0_LEVEL0_FLAG			RF_ADDR(9, 4)
#define HAL_RISC0_LEVEL0_POLARITY		RF_ADDR(9, 5)
#define HAL_RISC0_LEVEL0_EDGE			RF_ADDR(9, 6)

#define HAL_RISC1_LEVEL0_FLAG			RF_ADDR(10, 19)
#define HAL_RISC1_LEVEL0_POLARITY		RF_ADDR(10, 20)
#define HAL_RISC1_LEVEL0_EDGE			RF_ADDR(10, 21)

#define HAL_RISC0_INTR0_MASK			RF_ADDR(9, 7)
#define HAL_RISC0_INTR1_MASK			RF_ADDR(9, 8)
#define HAL_RISC0_INTR2_MASK			RF_ADDR(9, 9)
#define HAL_RISC0_INTR3_MASK			RF_ADDR(9, 10)
#define HAL_RISC0_INTR4_MASK			RF_ADDR(9, 11)
#define HAL_RISC0_INTR5_MASK			RF_ADDR(9, 12)

#define HAL_RISC1_INTR0_MASK			RF_ADDR(9, 13)
#define HAL_RISC1_INTR1_MASK			RF_ADDR(9, 14)
#define HAL_RISC1_INTR2_MASK			RF_ADDR(9, 15)
#define HAL_RISC1_INTR3_MASK			RF_ADDR(9, 16)
#define HAL_RISC1_INTR4_MASK			RF_ADDR(9, 17)
#define HAL_RISC1_INTR5_MASK			RF_ADDR(9, 18)

#define HAL_RISC0_INTR0_MASKED			RF_ADDR(9, 19)
#define HAL_RISC0_INTR1_MASKED			RF_ADDR(9, 20)
#define HAL_RISC0_INTR2_MASKED			RF_ADDR(9, 21)
#define HAL_RISC0_INTR3_MASKED			RF_ADDR(9, 22)
#define HAL_RISC0_INTR4_MASKED			RF_ADDR(9, 23)
#define HAL_RISC0_INTR5_MASKED			RF_ADDR(9, 24)

#define HAL_RISC1_INTR0_MASKED			RF_ADDR(9, 25)
#define HAL_RISC1_INTR1_MASKED			RF_ADDR(9, 26)
#define HAL_RISC1_INTR2_MASKED			RF_ADDR(9, 27)
#define HAL_RISC1_INTR3_MASKED			RF_ADDR(9, 28)
#define HAL_RISC1_INTR4_MASKED			RF_ADDR(9, 29)
#define HAL_RISC1_INTR5_MASKED			RF_ADDR(9, 30)

#define HAL_RISC1_LEVEL1_SELECT			RF_ADDR(9, 31)

/*
 * Name used by interrupt decode code
 */

#define	HAL_INTR0_MASK					HAL_RISC0_INTR0_MASK
#define	HAL_INTR1_MASK					HAL_RISC0_INTR1_MASK
#define	HAL_INTR2_MASK					HAL_RISC0_INTR2_MASK
#define	HAL_INTR3_MASK					HAL_RISC0_INTR3_MASK
#define	HAL_INTR4_MASK					HAL_RISC0_INTR4_MASK
#define	HAL_INTR5_MASK					HAL_RISC0_INTR5_MASK
#define	HAL_INTR0_MASKED				HAL_RISC0_INTR0_MASKED
#define	HAL_INTR1_MASKED				HAL_RISC0_INTR1_MASKED
#define	HAL_INTR2_MASKED				HAL_RISC0_INTR2_MASKED
#define	HAL_INTR3_MASKED				HAL_RISC0_INTR3_MASKED
#define	HAL_INTR4_MASKED				HAL_RISC0_INTR4_MASKED
#define	HAL_INTR5_MASKED				HAL_RISC0_INTR5_MASKED

#define HAL_LEVEL0_FLAG					HAL_RISC0_LEVEL0_FLAG
#define HAL_LEVEL0_POLARITY				HAL_RISC0_LEVEL0_POLARITY
#define HAL_LEVEL0_EDGE					HAL_RISC0_LEVEL0_EDGE

#define LSR_TX_RDY						(1<<0)
#define LSR_RX_RDY						(1<<1)
#define TX_EMPTY						(1<<6)
#define RX_EMPTY						(1<<7)

/* UART Registers*/
#define HAL_UART0_BASE					RF_ADDR(18, 0)
#define HAL_UART1_BASE					RF_ADDR(19, 10)

#define HAL_UART0_DATA					RF_ADDR(18, 0)
#define HAL_UART0_LSR					RF_ADDR(18, 1)
#define HAL_UART0_MSR					RF_ADDR(18, 2)
#define HAL_UART0_LCR					RF_ADDR(18, 3)
#define HAL_UART0_MCR					RF_ADDR(18, 4)
#define HAL_UART0_DIV_L					RF_ADDR(18, 5)
#define HAL_UART0_DIV_H					RF_ADDR(18, 6)
#define HAL_UART0_ISC					RF_ADDR(18, 7)

#define HAL_UART1_DATA					RF_ADDR(19, 10)
#define HAL_UART1_LSR					RF_ADDR(19, 11)
#define HAL_UART1_LCR					RF_ADDR(19, 13)
#define HAL_UART1_DIV_L					RF_ADDR(19, 15)
#define HAL_UART1_DIV_H					RF_ADDR(19, 16)
#define HAL_UART1_ISC					RF_ADDR(19, 17)


//-----------------------------------------------------------------------------
// IO Register address.
// This type is for recording the address of an IO register.

typedef volatile CYG_ADDRWORD HAL_IO_REGISTER;

//-----------------------------------------------------------------------------
// HAL IO macros.
#ifndef HAL_IO_MACROS_DEFINED

//-----------------------------------------------------------------------------
// BYTE Register access.
// Individual and vectorized access to 8 bit registers.

#define HAL_READ_UINT8( _register_, _value_ ) \
        ((_value_) = *((volatile CYG_BYTE *)(_register_)))

#define HAL_WRITE_UINT8( _register_, _value_ ) \
        (*((volatile CYG_BYTE *)(_register_)) = (_value_))

#define HAL_READ_UINT8_VECTOR( _register_, _buf_, _count_, _step_ )     \
{                                                                       \
    cyg_count32 _i_,_j_;                                                \
    for( _i_ = 0, _j_ = 0; _i_ < (_count_); _i_++, _j_ += (_step_))     \
        (_buf_)[_i_] = ((volatile CYG_BYTE *)(_register_))[_j_];        \
}

#define HAL_WRITE_UINT8_VECTOR( _register_, _buf_, _count_, _step_ )    \
{                                                                       \
    cyg_count32 _i_,_j_;                                                \
    for( _i_ = 0, _j_ = 0; _i_ < (_count_); _i_++, _j_ += (_step_))     \
        ((volatile CYG_BYTE *)(_register_))[_j_] = (_buf_)[_i_];        \
}


//-----------------------------------------------------------------------------
// 16 bit access.
// Individual and vectorized access to 16 bit registers.

#define HAL_READ_UINT16( _register_, _value_ ) \
        ((_value_) = *((volatile CYG_WORD16 *)(_register_)))

#define HAL_WRITE_UINT16( _register_, _value_ ) \
        (*((volatile CYG_WORD16 *)(_register_)) = (_value_))

#define HAL_READ_UINT16_VECTOR( _register_, _buf_, _count_, _step_ )    \
{                                                                       \
    cyg_count32 _i_,_j_;                                                \
    for( _i_ = 0, _j_ = 0; _i_ < (_count_); _i_++, _j_ += (_step_))     \
        (_buf_)[_i_] = ((volatile CYG_WORD16 *)(_register_))[_j_];      \
}

#define HAL_WRITE_UINT16_VECTOR( _register_, _buf_, _count_, _step_ )   \
{                                                                       \
    cyg_count32 _i_,_j_;                                                \
    for( _i_ = 0, _j_ = 0; _i_ < (_count_); _i_++, _j_ += (_step_))     \
        ((volatile CYG_WORD16 *)(_register_))[_j_] = (_buf_)[_i_];      \
}

//-----------------------------------------------------------------------------
// 32 bit access.
// Individual and vectorized access to 32 bit registers.

#define HAL_READ_UINT32( _register_, _value_ ) \
        ((_value_) = *((volatile CYG_WORD32 *)(_register_)))

#define HAL_WRITE_UINT32( _register_, _value_ ) \
        (*((volatile CYG_WORD32 *)(_register_)) = (_value_))

#define HAL_READ_UINT32_VECTOR( _register_, _buf_, _count_, _step_ )    \
{                                                                       \
    cyg_count32 _i_,_j_;                                                \
    for( _i_ = 0, _j_ = 0; _i_ < (_count_); _i_++, _j_ += (_step_))     \
        (_buf_)[_i_] = ((volatile CYG_WORD32 *)(_register_))[_j_];      \
}

#define HAL_WRITE_UINT32_VECTOR( _register_, _buf_, _count_, _step_ )   \
{                                                                       \
    cyg_count32 _i_,_j_;                                                \
    for( _i_ = 0, _j_ = 0; _i_ < (_count_); _i_++, _j_ += (_step_))     \
        ((volatile CYG_WORD32 *)(_register_))[_j_] = (_buf_)[_i_];      \
}

#define HAL_IO_MACROS_DEFINED

#endif

#endif /* __SPHE15XX_H__ */
