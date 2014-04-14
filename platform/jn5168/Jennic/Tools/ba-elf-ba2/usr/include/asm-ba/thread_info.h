/* thread_info.h: BA low-level thread information
 *
 * Copyright (C) 2002  David Howells (dhowells@redhat.com)
 * - Incorporating suggestions made by Linus Torvalds and Dave Miller
 * 
 * BA port by Matjaz Breskvar (phoenix@bsemi.com)
 */

#ifndef _ASM_THREAD_INFO_H
#define _ASM_THREAD_INFO_H

#ifdef __KERNEL__

#define PREEMPT_ACTIVE		0x4000000

#ifndef __ASSEMBLY__

#include <asm/types.h>
#include <asm/processor.h>

/*
 * low level task data that entry.S needs immediate access to
 * - this struct should fit entirely inside of one cache line
 * - this struct shares the supervisor stack pages
 * - if the contents of this structure are changed, the assembly constants must also be changed
 */
struct thread_info {
	struct task_struct	*task;		/* main task structure */
	struct exec_domain	*exec_domain;	/* execution domain */
	unsigned long		flags;		/* low level flags */
	__u32			cpu;		/* current CPU */
	__s32			preempt_count; /* 0 => preemptable, <0 => BUG */

	mm_segment_t		addr_limit;	/* thread address space:
					 	   0-0x7FFFFFFF for user-thead
						   0-0xFFFFFFFF for kernel-thread
						*/
	struct restart_block    restart_block;
	__u8			supervisor_stack[0];

	/* saved context data */
	unsigned long           ksp;

};

/*
 * macros/functions for gaining access to the thread information structure
 *
 * preempt_count needs to be 1 initially, until the scheduler is functional.
 */
#define INIT_THREAD_INFO(tsk)				\
{							\
	.task		= &tsk,				\
	.exec_domain	= &default_exec_domain,		\
	.flags		= 0,				\
	.cpu		= 0,				\
	.preempt_count	= 1,				\
	.addr_limit	= KERNEL_DS,			\
	.restart_block  = {				\
		        .fn = do_no_restart_syscall,	\
	},						\
        .ksp            = 0,                            \
}

#define init_thread_info	(init_thread_union.thread_info)

/* how to get the thread information struct from C */
register struct thread_info *current_thread_info_reg asm("r29");
#define current_thread_info()   (current_thread_info_reg)

/* thread information allocation */
#define alloc_thread_info(tsk) ((struct thread_info *) __get_free_pages(GFP_KERNEL,1))
#define free_thread_info(ti) free_pages((unsigned long) (ti), 1)
#define get_thread_info(ti) get_task_struct((ti)->task)
#define put_thread_info(ti) put_task_struct((ti)->task)

#else

#include <asm/asm-offsets.h>

#endif /* !__ASSEMBLY__ */

/*
 * thread information flags
 * - these are process state flags that various assembly files may need to access
 * - pending work-to-be-done flags are in LSW
 * - other flags in MSW
 */
#define TIF_SYSCALL_TRACE	0	/* syscall trace active */
#define TIF_NOTIFY_RESUME	1	/* resumption notification requested */
#define TIF_SIGPENDING		2	/* signal pending */
#define TIF_NEED_RESCHED	3	/* rescheduling necessary */
#define TIF_SINGLESTEP		4	/* restore singlestep on return to user mode */
#define TIF_POLLING_NRFLAG	16	/* true if poll_idle() is polling TIF_NEED_RESCHED */
#define TIF_MEMDIE              17

#define _TIF_SYSCALL_TRACE	(1<<TIF_SYSCALL_TRACE)
#define _TIF_NOTIFY_RESUME	(1<<TIF_NOTIFY_RESUME)
#define _TIF_SIGPENDING		(1<<TIF_SIGPENDING)
#define _TIF_NEED_RESCHED	(1<<TIF_NEED_RESCHED)
#define _TIF_SINGLESTEP		(1<<TIF_SINGLESTEP)
#define _TIF_POLLING_NRFLAG	(1<<TIF_POLLING_NRFLAG)

/* work to do on interrupt/exception return */
#define _TIF_WORK_MASK \
  (0x0000FFFE & ~(_TIF_SYSCALL_TRACE|_TIF_SINGLESTEP))
/* work to do on any return to u-space */
#define _TIF_ALLWORK_MASK	0x0000FFFF

#endif /* __KERNEL__ */

#endif /* _ASM_THREAD_INFO_H */
