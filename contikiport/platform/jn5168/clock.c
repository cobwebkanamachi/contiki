/*
 * Copyright (c) 2005, Swedish Institute of Computer Science.
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
 *         Clock implementation for Unix.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include <AppHardwareApi.h>
#include <PeripheralRegs.h>
#include "contiki.h"
#include "sys/energest.h"
#include "sys/clock.h"
#include "sys/etimer.h"
#include "rtimer-arch.h"
#include "dev/watchdog.h"
/* Timer */
//#define TICK_PERIOD_ms                  1UL
//#define CPU_CLOCK_FREQ_HZ               16000000UL
//#define TICK_TIMER_TICK_PERIOD_COUNT    (16000UL * TICK_PERIOD_ms)
/* Timer clocked at (16MHz / (2^TIMER_PRESCALE)) */
#define TIMER_PRESCALE                  0UL
/* 1 ms tick timer, CLOCK_SECOND = 1000 ==> 16000UL */
#define INTERVAL (RTIMER_ARCH_SECOND / CLOCK_SECOND)

#define MAX_TICKS (INTERVAL)

static volatile unsigned long seconds;
static volatile uint8_t ticking = FALSE;
static volatile clock_time_t count = 0;
/* last_tar is used for calculating clock_fine */
static volatile uint32_t last_tick = 0, ticks = 0;
/*---------------------------------------------------------------------------*/
void
vTimerISR(uint32 u32Device, uint32 u32ItemBitmap)
{
	ENERGEST_ON(ENERGEST_TYPE_IRQ);
	watchdog_start();
	ticks += INTERVAL - last_tick;
	last_tick = 0;
	rtimer_run_next();
	//do
	{
		++count;

		/* Make sure the CLOCK_CONF_SECOND is a power of two, to ensure
		 that the modulo operation below becomes a logical and and not
		 an expensive divide. Algorithm from Wikipedia:
		 http://en.wikipedia.org/wiki/Power_of_two */
		//#if (CLOCK_CONF_SECOND & (CLOCK_CONF_SECOND - 1)) != 0
		//#error CLOCK_CONF_SECOND must be a power of two (i.e., 1, 2, 4, 8, 16, 32, 64, ...).
		//#error Change CLOCK_CONF_SECOND in contiki-conf.h.
		//#endif
//		if (count % CLOCK_CONF_SECOND == 0) {
//			++seconds;
//			energest_flush();
//		}
	} //while((TACCR1 - u32AHI_TickTimerRead()) > INTERVAL);

	if (etimer_pending() && (etimer_next_expiration_time() - count - 1)
			> MAX_TICKS) {
		etimer_request_poll();
		//LPM4_EXIT;
	}
	/*  if(process_nevents() >= 0) {
	 LPM4_EXIT;
	 }*/

	watchdog_stop();

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
void
clock_init(void)
{
	//gMAC_u8MaxBuffers = 2;
#ifdef JENNIC_CHIP_FAMILY_JN516x
	/* Turn off debugger */
	*(volatile uint32 *) 0x020000a0 = 0;
#endif
	/* system controller interrupts callback is disabled
	 * -- Only wake Interrupts --
	 */
	vAHI_SysCtrlRegisterCallback(0);

	/* Initialise tick timer to give periodic interrupt */
	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_DISABLE);
	vAHI_TickTimerWrite(0);
	vAHI_TickTimerRegisterCallback(vTimerISR);
	vAHI_TickTimerIntEnable(1);
	vAHI_TickTimerInterval(INTERVAL);
	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_RESTART);
	ticking = TRUE;
	(void) u32AHI_Init();
//#ifdef JENNIC_CHIP_FAMILY_JN516x
	bAHI_SetClockRate(E_AHI_XTAL_32MHZ);
	vAHI_OptimiseWaitStates();
	/* Turn on SPI master */
	vREG_SysWrite(REG_SYS_PWR_CTRL, u32REG_SysRead(REG_SYS_PWR_CTRL)
			| REG_SYSCTRL_PWRCTRL_SPIMEN_MASK);
//#endif

	if (bAHI_WatchdogResetEvent()) {
		//vPrintf("APP: Watchdog timer has reset device!\r\n");
	}
	vAHI_WatchdogStop();
	vAHI_TickTimerIntEnable(TRUE);

}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  clock_time_t t1, t2;
  do {
    t1 = count;
    t2 = count;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
void
clock_set(clock_time_t clock, clock_time_t fclock)
{
	//vAHI_TickTimerIntEnable(0);
	vAHI_TickTimerWrite(fclock);
  //vAHI_TickTimerRegisterCallback(vTimerISR);
  count = clock;
  //vAHI_TickTimerIntEnable(1);
  vAHI_TickTimerInterval(INTERVAL);
  //vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_RESTART);
}
/*---------------------------------------------------------------------------*/
int
clock_fine_max(void)
{
  return INTERVAL;
}
/*---------------------------------------------------------------------------*/
unsigned short
clock_fine(void)
{
  unsigned short t;
  /* Assign last_tar to local varible that can not be changed by interrupt */
  t = last_tick;
  /* perform calc based on t, TAR will not be changed during interrupt */
  return (unsigned short) (u32AHI_TickTimerRead() - t);
}
/*---------------------------------------------------------------------------*/
/**
 * Delay the CPU for a multiple of 1 us.
 */
void
clock_delay(unsigned int i)
{
	volatile uint32_t t = u32AHI_TickTimerRead();
	//XXX beware of wrapping
	if(MAX_TICKS-t < i) {
		while(u32AHI_TickTimerRead() < MAX_TICKS && u32AHI_TickTimerRead() !=0);
		i-=MAX_TICKS-t;
		t=0;
	}
	while(u32AHI_TickTimerRead()-t < i);
}
/*---------------------------------------------------------------------------*/
/**
 * Delay the CPU for a multiple of 1 us.
 */
void
clock_delay_usec(uint16_t i)
{
	volatile uint32_t t = u32AHI_TickTimerRead();
	//XXX beware of wrapping
	if(MAX_TICKS-t < i) {
		while(u32AHI_TickTimerRead() < MAX_TICKS && u32AHI_TickTimerRead() !=0);
		i-=MAX_TICKS-t;
		t=0;
	}
	while(u32AHI_TickTimerRead()-t < i);
}
/*---------------------------------------------------------------------------*/
/**
 * Wait for a multiple of 1 ms.
 *
 */
void
clock_wait(clock_time_t i)
{
  clock_time_t start;

  start = clock_time();
  while(clock_time() - start < (clock_time_t)i);
}
/*---------------------------------------------------------------------------*/
void
clock_set_seconds(unsigned long sec)
{
  seconds = sec;
}
/*---------------------------------------------------------------------------*/
unsigned long
clock_seconds(void)
{
  unsigned long t1, t2;
  do {
    t1 = seconds;
    t2 = seconds;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
clock_counter(void)
{
  rtimer_clock_t t1, t2;
  do {
    t1 = u32AHI_TickTimerRead();
    t2 = u32AHI_TickTimerRead();
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/

