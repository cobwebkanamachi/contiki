/*
 * Copyright (c) 2008-2010
 * Telecooperation Office (TecO), Universitaet Karlsruhe (TH), Germany.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. Neither the name of the Universitaet Karlsruhe (TH) nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author(s): Philipp Scholl <scholl@teco.edu>
 */

#include "sys/rtimer.h"
#include "sys/clock.h"
#include <AppHardwareApi.h>
#include <PeripheralRegs.h>

#define RTIMER_TIMER E_AHI_TIMER_0

void
rtimer_arch_run_next(uint32_t dev, uint32_t mask)
{
  rtimer_run_next();
}

void
rtimer_arch_init(void)
{
  /* prescale to 16mHz/2**9, which spans ~2sec
   * be sure to change RTIMER_ARCH_SECOND when changing prescale */
//  vAHI_TimerClockSelect(RTIMER_TIMER, false, false);
//  vAHI_TimerEnable(RTIMER_TIMER, 9, true, false, false);
//  vAHI_TimerDIOControl(RTIMER_TIMER, false);
//
//#if (RTIMER_TIMER==E_AHI_TIMER_0)
//  vAHI_Timer0RegisterCallback(rtimer_arch_run_next);
//#else
//  vAHI_Timer1RegisterCallback(rtimer_arch_run_next);
//#endif
}

void
rtimer_arch_schedule(rtimer_clock_t t)
{
  vAHI_TimerStartSingleShot(RTIMER_TIMER, t, 0);
}
