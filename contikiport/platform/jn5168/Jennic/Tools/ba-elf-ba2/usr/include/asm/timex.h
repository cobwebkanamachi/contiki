/*
 * linux/include/asm-or32/timex.h
 *
 * Based on:
 * include/asm-cris/timex.h
 */
#ifndef _OR32_TIMEX_H
#define _OR32_TIMEX_H

#include <asm/param.h>

#define SYS_TICK_PER    ((CONFIG_OR32_SYS_CLK*1000000)/HZ)

#define CLOCK_TICK_RATE	(CONFIG_OR32_SYS_CLK*1000000 / HZ) /* Underlying HZ */

/*
 * We don't have a cycle-counter.. but we do not support SMP anyway where this is
 * used so it does not matter.
 */

typedef unsigned int cycles_t;

static inline cycles_t get_cycles(void)
{
        return 0;
}

#endif
