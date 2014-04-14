/*
 * Copyright (c) 2009
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
 *
 * Driver for a buzzer found on the landmarke devices.
 */

#include "dev/buzzer.h"
#include <AppHardwareApi.h>
#include <stdbool.h>
#include <stdint.h>
#include "gdb2.h"

#ifndef JENNIC_CONF_BUZZER_TIMER
# define BUZZER_TIMER E_AHI_TIMER_1
#else
# define BUZZER_TIMER JENNIC_CONF_BUZZER_TIMER
#endif

#ifndef JENNIC_CONF_BUZZER_POWER_PIN
# define POWER_PIN JENNIC_CONF_BUZZER_POWER_PIN
#else
# define POWER_PIN  E_AHI_DIO17_INT
#endif

#ifndef JENNIC_CONF_BUZZER_PIN
# define BUZZER_PIN JENNIC_CONF_BUZZER_PIN
#elif
# define BUZZER_PIN E_AHI_DIO10_INT
#endif

static void
buzz(uint32_t a, uint32_t b)
{
  static bool state = false;

  if (state) vAHI_DioSetOutput( BUZZER_PIN, 0);
  else       vAHI_DioSetOutput( 0, BUZZER_PIN);
  state = !state;
}


void
buzzer_off()
{
  vAHI_TimerStop(BUZZER_TIMER);
  vAHI_DioSetOutput(0, BUZZER_PIN);
}

#define TIMER_SECOND 31250

void
buzzer_init()
{
  vAHI_DioSetDirection(0, BUZZER_PIN|POWER_PIN);

  /* prescale to 16mHz/2**9, which spans ~2sec
   * be sure to change BUZZER_ARCH_SECOND when changing prescale */
  vAHI_TimerClockSelect(BUZZER_TIMER, false, false);
  vAHI_TimerEnable(BUZZER_TIMER, 9, false, true, false);
  vAHI_TimerDIOControl(BUZZER_TIMER, false);

#if (BUZZER_TIMER==E_AHI_TIMER_0)
  vAHI_Timer0RegisterCallback(buzz);
#else
  vAHI_Timer1RegisterCallback(buzz);
#endif
}

void
buzzer_tone(uint16_t freq)
  /* keep in mind that the highest freq equals BUZZER_SECOND */
{
  if (freq == 0)
    freq = 2500;

  vAHI_TimerStartRepeat(BUZZER_TIMER, 0, TIMER_SECOND/freq);
}
