#ifndef __ASM_OR32_ATOMIC__
#define __ASM_OR32_ATOMIC__

#include <asm/system.h>

/*
 * Atomic operations that C can't guarantee us.  Useful for
 * resource counting etc..
 */

/*
 * Make sure gcc doesn't try to be clever and move things around
 * on us. We need to use _exactly_ the address the user gave us,
 * not some alias that contains the same information.
 */

#define __atomic_fool_gcc(x) (*(struct { int a[100]; } *)x)

typedef struct { int counter; } atomic_t;

#define ATOMIC_INIT(i)  { (i) }

#define atomic_read(v) ((v)->counter)
#define atomic_set(v,i) (((v)->counter) = (i))

/* These should be written in asm but we do it in C for now. */

extern __inline__ void atomic_add(int i, volatile atomic_t *v)
{
	unsigned long flags;
	local_irq_save(flags);
	v->counter += i;
	local_irq_restore(flags);
}

extern __inline__ void atomic_sub(int i, volatile atomic_t *v)
{
	unsigned long flags;
	local_irq_save(flags);
	v->counter -= i;
	local_irq_restore(flags);
}

extern __inline__ int atomic_add_return(int i, volatile atomic_t *v)
{
	unsigned long flags;
	int retval;
	local_irq_save(flags);
	retval = (v->counter += i);
	local_irq_restore(flags);
	return retval;
}

extern __inline__ int atomic_sub_return(int i, volatile atomic_t *v)
{
	unsigned long flags;
	int retval;
	local_irq_save(flags);
	retval = (v->counter -= i);
	local_irq_restore(flags);
	return retval;
}

static __inline__ int atomic_sub_if_positive(int i, atomic_t * v)
{
	unsigned int result;
	unsigned long flags;

	local_irq_save(flags);
	result = v->counter;
	result -= i;
	if (result >= 0)
		v->counter = result;
	local_irq_restore(flags);

	return result;
}

#define atomic_xchg(v, new) (xchg(&((v)->counter), new))

static inline int atomic_add_unless(atomic_t *v, int a, int u)
{
	int ret;
	unsigned long flags;

	local_irq_save(flags);
	ret = v->counter;
	if (ret != u)
		v->counter += a;
	local_irq_restore(flags);
	return ret != u;
}
#define atomic_inc_not_zero(v) atomic_add_unless((v), 1, 0)

#define atomic_dec_return(v) atomic_sub_return(1,(v))
#define atomic_inc_return(v) atomic_add_return(1,(v))

#define atomic_sub_and_test(i,v) (atomic_sub_return((i), (v)) == 0)
#define atomic_inc_and_test(v) (atomic_inc_return(v) == 0)

#define atomic_dec_and_test(v) (atomic_sub_return(1, (v)) == 0)
#define atomic_dec_if_positive(v)	atomic_sub_if_positive(1, v)

#define atomic_inc(v) atomic_add(1,(v))
#define atomic_dec(v) atomic_sub(1,(v))

#define atomic_add_negative(a, v)	(atomic_add_return((a), (v)) < 0)

/* Atomic operations are already serializing */
#define smp_mb__before_atomic_dec()    barrier()
#define smp_mb__after_atomic_dec()     barrier()
#define smp_mb__before_atomic_inc()    barrier()
#define smp_mb__after_atomic_inc()     barrier()

/* Get the generic long-atomic varients */
#include <asm-generic/atomic.h>

#endif /* __ASM_OR32_ATOMIC__ */
