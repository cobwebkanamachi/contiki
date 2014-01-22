/* asm/bitops.h for Linux/or32
 *
 * __PHX__ TODO: asm versions
 *
 */

#ifdef __KERNEL__
#ifndef _OR32_BITOPS_H
#define _OR32_BITOPS_H

#include <asm/system.h>
#include <asm-generic/bitops.h>

#define smp_mb__before_clear_bit()      barrier()
#define smp_mb__after_clear_bit()       barrier()

#endif /* _OR32_BITOPS_H */
#endif /* __KERNEL__ */
