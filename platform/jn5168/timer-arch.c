/*
 * Copyright (c) 2014, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         RTIMER for NXP jn5168
 * \author
 *         Beshr Al Nahas <beshr@sics.se>
 */

#include "sys/rtimer.h"
#include "sys/clock.h"
#include <AppHardwareApi.h>
#include <PeripheralRegs.h>
#include "dev/watchdog.h"
#include "sys/energest.h"

//#define RTIMER_TIMER 					E_AHI_TICK_TIMER
#define RTIMER_TIMER_ISR_DEV  E_AHI_DEVICE_TICK_TIMER

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static volatile uint32 s_u32CompareTime;
static volatile uint32 s_u32LastExpiredTime;

void
rtimer_arch_run_next(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
	if(u32DeviceId != RTIMER_TIMER_ISR_DEV) {
		return;
	}

  ENERGEST_ON(ENERGEST_TYPE_IRQ);
	vAHI_TickTimerIntPendClr();
	vAHI_TickTimerIntEnable(false);
  /*
   * compare register is only 28bits wide so make sure the upper 4bits match
   * the set compare point
   */
  uint32 u32Delta = u32AHI_TickTimerRead() - s_u32CompareTime;
  if (0 == (u32Delta >> 28))
  {
      uint32 u32Temp = s_u32CompareTime;

      //run scheduled
      watchdog_start();

    	rtimer_run_next();

    //  if(process_nevents() > 0) {
    //    LPM4_EXIT;
    //  }

      watchdog_stop();
      //---
      s_u32LastExpiredTime = u32Temp;
  }
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
//  vAHI_TimerEnable(RTIMER_TIMER, RTIMER_PRESCALE, true, false, false);
//  vAHI_TimerClockSelect(RTIMER_TIMER, false, false);
//
//  vAHI_TimerConfigureOutputs(RTIMER_TIMER, false, true);
//  vAHI_TimerDIOControl(RTIMER_TIMER, false);
//
//#if (RTIMER_TIMER==E_AHI_TIMER_0)
//  vAHI_Timer0RegisterCallback(rtimer_arch_run_next);
//#else (RTIMER_TIMER==E_AHI_TIMER_1)
//  vAHI_Timer1RegisterCallback(rtimer_arch_run_next);
//#endif

	/* Initialise tick timer to run continuously */
	vAHI_TickTimerIntEnable(false);
	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_DISABLE);
	s_u32LastExpiredTime = s_u32CompareTime = 0;
	vAHI_TickTimerWrite(0);
	vAHI_TickTimerRegisterCallback(rtimer_arch_run_next);
	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_CONT);

}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
rtimer_arch_now(void)
{
  /* Disable interrupts */
  //spl_t s = splhigh();

//  rtimer_clock_t t1, t2;
//  do {
//    t1 = u32AHI_TickTimerRead();
//    t2 = u32AHI_TickTimerRead();
//  } while(t1 != t2);
  return u32AHI_TickTimerRead();
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
	PRINTF("rtimer_arch_schedule time %lu\n", t);
	vAHI_TickTimerIntPendClr();
	vAHI_TickTimerIntEnable(true);
	vAHI_TickTimerInterval(t);
	s_u32CompareTime = t;
}
/*---------------------------------------------------------------------------*/
