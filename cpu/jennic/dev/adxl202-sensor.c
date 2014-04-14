/*
 * Copyright (c) 2010
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
 * Driver for ADXL202 Acceleration sensor.
 */

#include "dev/irq.h"
#include "dev/acc-sensor.h"
#include "dev/hrclock.h"
#include <AppHardwareApi.h>

static void
sample(u32_t pin, s32_t *high, s32_t *low)
{
  s32_t h=0,l=0;

  /* wait until one cycle is completed */
  while (u32AHI_DioReadInput() & pin)
    ; /* busy wait for low */

  while (!(u32AHI_DioReadInput() & pin))
    ; /* busy wait for rising edge */

  /* now sample this cycle */
  while (u32AHI_DioReadInput() & pin)
    h++; /* busy wait for falling edge */

  while (!(u32AHI_DioReadInput() & pin))
    l++; /* busy wait for rising edge */

  *high = h;
  *low  = l;

}

static int
value(int type)
{
  /* range is +- 2g, i.e. 50% duty cycle at 0g, 25% at -2g, 75% at * 2g */
  static const s32_t max = 836;   /* empirically, should be auto-calibrated */
  static const s32_t ratio = max/8; /* 12.5% duty cycle per g, also given by high/(high+low) */
  s32_t high=0, low=0;

  switch(type) {
  case ACC_VALUE_X:
    do { /* do sampling, until no irq occured while doing so*/
      sample(JENNIC_CONF_ADXL202_XPIN, &high, &low);
    } while(high+low != max);

    /* return value in mg, around 0mg*/
    return (high*1000 - max/2*1000)/ratio;
  case ACC_VALUE_Y:
    do { /* do sampling, until no irq occured while doing so*/
      sample(JENNIC_CONF_ADXL202_YPIN, &high, &low);
    } while(high+low != max);

    /* return value in mg, around 0mg*/
    return (high*1000 - max/2*1000)/ratio;
  }

  return 0;
}

static int
configure(int type, int v)
{
  switch (type) {
  case SENSORS_HW_INIT:
    vAHI_DioSetDirection(JENNIC_CONF_ADXL202_YPIN|
                         JENNIC_CONF_ADXL202_XPIN,
                         JENNIC_CONF_ADXL202_PWRPIN);

    return 1;
  case SENSORS_ACTIVE:
    if (v) vAHI_DioSetOutput(JENNIC_CONF_ADXL202_PWRPIN, 0x00);
    else   vAHI_DioSetOutput(0x00, JENNIC_CONF_ADXL202_PWRPIN);
    return 1;
  }

  return 0;
}

static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return 1;
  }

  return 0;
}

SENSORS_SENSOR(acc_sensor, ACC_SENSOR,
               value, configure, status);
