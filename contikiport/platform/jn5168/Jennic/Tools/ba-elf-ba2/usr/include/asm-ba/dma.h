/* $Id: dma.h,v 1.1 2008/02/15 15:10:16 dcook Exp $ */

#ifndef _ASM_DMA_H
#define _ASM_DMA_H

#define MAX_DMA_ADDRESS PAGE_OFFSET

#ifdef CONFIG_PCI
extern int isa_dma_bridge_buggy;
#else
#define isa_dma_bridge_buggy    (0)
#endif

#endif /* _ASM_DMA_H */
