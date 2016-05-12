/*
 *  Copyright (C) 2016 Team-Proton <dev.team.proton@gmail.com>
**
** Description
** UART debugging support function
**
*/
#ifndef __DRV_UART_H__
#define __DRV_UART_H__

#define UART_FIFO_SIZE			0x10
#define UART_REGS_SIZE			0x28

#define UART_DATA_REG			0x00  	// data register
#define UART_LSR_REG			0x04  	// line status register
#define UART_MSR_REG			0x08	// modem status register
#define UART_LCR_REG			0x0c  	// line control register
#define UART_MCR_REG			0x10	// modem control register
#define UART_DIV_L_REG			0x14	// clock low register
#define UART_DIV_H_REG			0x18	// clock high register
#define UART_ISC_REG			0x1C  	// interrupt status register
#define UART_TX_RESIDUE_REG     0x20
#define UART_RX_RESIDUE_REG     0x24

#define UART_DATA_TX_RX_MASK	0xff
#define UART_DATA_RX_CSR		BIT(8)
#define UART_DATA_TX_CSR		BIT(9)

#define UART_CS_PARITY_S		0
#define UART_CS_PARITY_M		0x3
#define	UART_CS_PARITY_NONE		0
#define	UART_CS_PARITY_ODD		1
#define	UART_CS_PARITY_EVEN		2
#define UART_CS_IF_MODE_S		2
#define UART_CS_IF_MODE_M		0x3
#define	UART_CS_IF_MODE_NONE	0
#define	UART_CS_IF_MODE_DTE		1
#define	UART_CS_IF_MODE_DCE		2
#define UART_CS_FLOW_CTRL_S		4
#define UART_CS_FLOW_CTRL_M		0x3
#define UART_CS_DMA_EN			BIT(6)
#define UART_CS_TX_READY_ORIDE	BIT(7)
#define UART_CS_RX_READY_ORIDE	BIT(8)
#define UART_CS_TX_READY		BIT(9)
#define UART_CS_RX_BREAK		BIT(10)
#define UART_CS_TX_BREAK		BIT(11)
#define UART_CS_HOST_INT		BIT(12)
#define UART_CS_HOST_INT_EN		BIT(13)
#define UART_CS_TX_BUSY			BIT(14)
#define UART_CS_RX_BUSY			BIT(15)

#define UART_CLOCK_STEP_M		0xffff
#define UART_CLOCK_SCALE_M		0xfff
#define UART_CLOCK_SCALE_S		16
#define UART_CLOCK_STEP_M		0xffff

#define UART_INT_RX_VALID		BIT(0)
#define UART_INT_TX_READY		BIT(1)
#define UART_INT_RX_FRAMING_ERR	BIT(2)
#define UART_INT_RX_OFLOW_ERR	BIT(3)
#define UART_INT_TX_OFLOW_ERR	BIT(4)
#define UART_INT_RX_PARITY_ERR	BIT(5)
#define UART_INT_RX_BREAK_ON	BIT(6)
#define UART_INT_RX_BREAK_OFF	BIT(7)
#define UART_INT_RX_FULL		BIT(8)
#define UART_INT_TX_EMPTY		BIT(9)
#define UART_INT_ALLINTS		0x3ff

/* LSR 0: tx-rdy 1:tx-fifo not-full
       1: rx-rdy 1:rx-fifo not-empty
*/
#define UART_LSR_TX_RDY				(1<<0)
#define UART_LSR_RX_RDY				(1<<1)
#define UART_TX_EMPTY				(1<<6)
#define UART_RX_EMPTY				(1<<7)

#define UART0_tx_rdy()				(HWREG_R(UART0_LSR) & UART_LSR_TX_RDY)
#define UART0_rx_rdy()				(HWREG_R(UART0_LSR) & UART_LSR_RX_RDY)
#define UART0_tx_empty()			(HWREG_R(UART0_LSR) & UART_TX_EMPTY)
#define UART0_rx_empty()			(HWREG_R(UART0_LSR) & UART_RX_EMPTY)
#define UART0_putc_nw(c)			HWREG_W(UART0_DATA , (c))
#define IsEPPRxEmpty()				(!UART0_rx_rdy())

#define UART1_tx_rdy()		(HWREG_R(UART1_LSR) & UART_LSR_TX_RDY)
#define UART1_rx_rdy()		(HWREG_R(UART1_LSR) & UART_LSR_RX_RDY)
#define UART1_tx_empty()	(HWREG_R(UART1_LSR) & UART_TX_EMPTY)
#define UART1_putc_nw(c)	HWREG_W(UART1_DATA , (c))

/* UART0 */
#define UART0_wait()		do { while (!UART0_tx_rdy()) ; } while (0)
#define UART0_flush()		do { while (!UART0_tx_empty()) ; } while (0)
#define UART0_putc(c)		do { UART0_wait(); UART0_putc_nw(c); } while (0)
#define UART0_putc_nl(c)	do { UART0_putc(c); if (c==0x0a) UART0_putc(0x0d); } while (0)
#define UART0_puts(s)		do { int __c;										\
								 const char *__s = (const char *)(s);			\
								 while ((__c=*__s++)) { UART0_putc(__c);	}	\
							} while (0)
#define UART0_getc()		(HWREG_R(UART0_DATA))

/* UART1 */
#define UART1_wait()		do { while (!UART1_tx_rdy()) ; } while (0)
#define UART1_flush()		do { while (!UART1_tx_empty()) ; } while (0)
#define UART1_putc(c)		do { UART1_wait(); UART1_putc_nw(c); } while (0)
#define UART1_putc_nl(c)	do { UART1_putc(c); if (c==0x0a) UART1_putc(0x0d); } while (0)
#define UART1_puts(s)		do { int __c;										\
								 const char *__s = (const char *)(s);			\
								 while ((__c=*__s++)) { UART1_putc_nl(__c);	}	\
							} while (0)
#define UART1_getc()		(HWREG_R(UART1_DATA))



#define BAUDX(f,c)			((unsigned int)((2*(c)+16*(f)) / (32*(f)))-1)
#define BAUD(f)				BAUDX(f,get_sys_bus_freq())

#define UART0_set_baudrate(x)	do {							\
								HWREG_W(UART0_DIV_H , (x>>8));	\
								HWREG_W(UART0_DIV_L , (x));		\
								} while (0)						\

#define UART1_set_baudrate(x)	do {							\
									HWREG_W(UART1_DIV_H , (x>>8));	\
									HWREG_W(UART1_DIV_L , (x));		\
								} while (0)						\


#define UART0_Interrupt_Status() 	(HWREG_R(UART0_ISC))

#define UART0_Disabe_Interrupt() 	do{\
										HWREG_W(UART0_ISC, 0x0000); \
									}while(0)

#define UART0_Soft_Reset() 			do{\
										HWREG_W(UART0_LSR, (HWREG_R(UART0_LSR) & 0xff7f)|(1<<7)); \
									}while(0)

#define UART0_LS_Interrupt_On() 	do{\
										HWREG_W(UART0_ISC, (HWREG_R(UART0_ISC) & 0xffbf)|(1<<6)); \
									}while(0)

#define UART0_LS_Interrupt_Off() 	do{\
										HWREG_W(UART0_ISC, (HWREG_R(UART0_ISC) & 0xffbf)); \
									}while(0)

#define UART0_RX_Interrupt_On() 	do{\
										HWREG_W(UART0_ISC, (HWREG_R(UART0_ISC) & 0xffdf)|(1<<5)); \
									}while(0)

#define UART0_RX_Interrupt_Off() 	do{\
										HWREG_W(UART0_ISC, (HWREG_R(UART0_ISC) & 0xffdf)); \
									}while(0)

#define UART0_TX_Interrupt_On() 	do{\
										HWREG_W(UART0_ISC, (HWREG_R(UART0_ISC) & 0xffef)|(1<<4)); \
									}while(0)

#define UART0_TX_Interrupt_Off() 	do{\
										HWREG_W(UART0_ISC, (HWREG_R(UART0_ISC) & 0xffef)); \
									}while(0)


#define UART1_Interrupt_Status() 	(HWREG_R(UART1_ISC))

#define UART1_LS_Interrupt_On() 	do{\
										HWREG_W(UART1_ISC, (HWREG_R(UART1_ISC) & 0xffbf)|(1<<6)); \
									}while(0)


#define UART1_Disabe_Interrupt() 	do{\
										HWREG_W(UART1_ISC, 0x0000); \
									}while(0)

#endif /* __DRV_UART_H__ */
