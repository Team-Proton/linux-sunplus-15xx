#ifndef __REGMAP_H__
#define __REGMAP_H__

#include "regfile.h"

#define HWREG_W(reg, value)	\
	do {					\
		(*(volatile unsigned int *)(reg)) = (value);	\
	} while (0)

#define HWREG_R(reg) (*(volatile unsigned int *)(reg))

/* Get the address of a HWREG.  */
#define HWREG_ADDR(reg)	(reg)
#define ADDR_OF(reg, offs)	(reg +  offs*4)

/* Set HWREG bits.  */
#define HWREG_S(reg, mask, pattern);	\
	do {								\
	unsigned int x;					\
		x = HWREG_R(reg);				\
		x = x & (mask);					\
		x = x | (pattern);				\
		HWREG_W(reg, x);				\
	} while (0)

#define HWREG_1(reg, pattern)	\
	do {						\
	unsigned int x;			\
		x = HWREG_R(reg);		\
		x = x | (pattern);		\
		HWREG_W(reg, x);		\
	} while (0)

#define HWREG_0(reg, pattern)	\
	do {						\
	unsigned int x;			\
		x = HWREG_R(reg);		\
		x = x & (pattern);		\
		HWREG_W(reg, x);		\
	} while (0)

#define	REGS0(reg, value)	 \
		.set	push		 \
		.set	noat		 \
		li		$at, value	 \
		sw		$at, reg(s6) \
		.set	pop			 \


#endif /*__REGMAP_H__*/
