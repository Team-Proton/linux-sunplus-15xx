#ifndef SPHE15XX_IOREMAP_H_
#define SPHE15XX_IOREMAP_H_

#include <linux/types.h>

static inline phys_addr_t fixup_bigphys_addr(phys_addr_t phys_addr, phys_addr_t size)
{
	return phys_addr;
}

static inline int sphe15xx_internal_registers(phys_addr_t offset)
{
	if (offset >= 0xb0000000 && offset < 0xbfffffff)
		return 1;
	return 0;
}

static inline void __iomem *plat_ioremap(phys_addr_t offset, unsigned long size,
					 unsigned long flags)
{
	if (sphe15xx_internal_registers(offset))
		return (void __iomem *)offset;
	return NULL;
}

static inline int plat_iounmap(const volatile void __iomem *addr)
{
	return sphe15xx_internal_registers((unsigned long)addr);
}

#endif /* SPHE15XX_IOREMAP_H_ */
