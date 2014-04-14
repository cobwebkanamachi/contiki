#ifdef __KERNEL__
#ifndef __OR32_IRQ_H__
#define __OR32_IRQ_H__

#define	NR_IRQS		32

#include <linux/irq.h>

static __inline__ int irq_canonicalize(int irq)
{
	return(irq);
}

/*
 * "Generic" interrupt sources
 */

#define IRQ_UART_0            (2)       /* interrupt source for UART dvice 0 */
#define IRQ_ETH_0             (4)       /* interrupt source for Ethernet dvice 0 */
#define IRQ_PS2_0             (5)       /* interrupt source for ps2 dvice 0 */
#define IRQ_SCHED_TIMER       (0)       /* interrupt source for scheduling timer */

#endif /* __OR32_IRQ_H__ */
#endif /* __KERNEL__ */
