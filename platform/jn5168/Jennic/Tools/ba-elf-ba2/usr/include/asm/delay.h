#ifndef _OR32_DELAY_H
#define _OR32_DELAY_H

#include <linux/delay.h>
#include <linux/jiffies.h>

static inline void __delay(int loops)
{
	__asm__ __volatile__ (
			      "l.srli %0,%0,1;"
			      "1: l.sfeqi %0,0;"
			      "l.bnf 1b;"
			      "l.addi %0,%0,-1;"
			      : "=r" (loops): "0" (loops));
}

extern void __udelay(unsigned long usecs);

static inline void udelay(unsigned long usecs)
{
        __udelay(usecs);
}

#endif 



