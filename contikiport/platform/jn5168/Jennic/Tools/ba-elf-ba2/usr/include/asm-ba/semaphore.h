/*
 * Based on:
 * include/asm-cris/semaphore.h
 */

#ifndef _OR32_SEMAPHORE_H
#define _OR32_SEMAPHORE_H

#define RW_LOCK_BIAS             0x01000000

#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>

#include <asm/system.h>
#include <asm/atomic.h>

int printk(const char *fmt, ...);

struct semaphore {
	atomic_t count;
	int sleepers;
	wait_queue_head_t wait;
};

extern spinlock_t semaphore_wake_lock;

#define __SEMAPHORE_INITIALIZER(name, n)				\
{									\
	.count		= ATOMIC_INIT(n),				\
	.sleepers	= 0,						\
	.wait		= __WAIT_QUEUE_HEAD_INITIALIZER((name).wait)    \
}

#define __DECLARE_SEMAPHORE_GENERIC(name,count) \
        struct semaphore name = __SEMAPHORE_INITIALIZER(name,count)

#define DECLARE_MUTEX(name) __DECLARE_SEMAPHORE_GENERIC(name,1)
#define DECLARE_MUTEX_LOCKED(name) __DECLARE_SEMAPHORE_GENERIC(name,0)

extern inline void sema_init(struct semaphore *sem, int val)
{
	*sem = (struct semaphore)__SEMAPHORE_INITIALIZER((*sem),val);
}

extern inline void init_MUTEX (struct semaphore *sem)
{
        sema_init(sem, 1);
}

extern inline void init_MUTEX_LOCKED (struct semaphore *sem)
{
        sema_init(sem, 0);
}

extern void __down(struct semaphore * sem);
extern int __down_interruptible(struct semaphore * sem);
extern int __down_trylock(struct semaphore * sem);
extern void __up(struct semaphore * sem);

/* notice - we probably can do cli/sti here instead of saving */

extern inline void down(struct semaphore * sem)
{
	might_sleep();
	/*
	 * Try to get the semaphore, take the slow path if we fail.
	 */
	if (unlikely(atomic_dec_return(&sem->count) < 0))
		__down(sem);
}

/*
 * This version waits in interruptible state so that the waiting
 * process can be killed.  The down_interruptible routine
 * returns negative for signalled and zero for semaphore acquired.
 */

extern inline int down_interruptible(struct semaphore * sem)
{
	int ret = 0;

	might_sleep();

	if (unlikely(atomic_dec_return(&sem->count) < 0))
		ret = __down_interruptible(sem);
	return ret;
}

extern inline int down_trylock(struct semaphore * sem)
{
	int ret = 0;

	if (atomic_dec_return(&sem->count) < 0)
		ret = __down_trylock(sem);
	return ret;
}

/*
 * Note! This is subtle. We jump to wake people up only if
 * the semaphore was negative (== somebody was waiting on it).
 * The default case (no contention) will result in NO
 * jumps for both down() and up().
 */
extern inline void up(struct semaphore * sem)
{  
	if (unlikely(atomic_inc_return(&sem->count) <= 0))
		__up(sem);
}


#endif /* _OR32_SEMAPHORE_H */
