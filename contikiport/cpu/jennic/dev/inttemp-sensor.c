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
 */
#include "dev/irq.h"
#include "dev/inttemp-sensor.h"
#include <AppHardwareApi.h>
#include <stdbool.h>

const struct sensors_sensor inttemp_sensor;
static volatile int _value = 0;

#define ADC_MAX     4095
#define TEMP_MIN    (-40000)
#define TEMP_MAX    (80000)
#define TEMP_RANGE  ((-1)*TEMP_MIN + TEMP_MAX)
#define TEMP_FACTOR (TEMP_RANGE/ADC_MAX)

static void
irq(irq_t s)
{
  _value = (u16AHI_AdcRead() * TEMP_FACTOR) + TEMP_MIN;
  sensors_changed(&inttemp_sensor);
}

static int
configure(int type, int value)
{
  static irq_handle_t handle = {.callback=irq, .irqsrc=IRQ_ADC_TEMP, 
                                .adc_input_range=ADC_INPUT_RANGE_1};

  switch(type) {
  case SENSORS_HW_INIT:
    _value = 0;
    return 1;
  case SENSORS_ACTIVE:
    if (value) irq_add(&handle);
    else       irq_remove(&handle);
    return 1;
  }

  return 0;
}

static void *
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    break;
  }

  return NULL;
}

static int
value(int type)
{
  return _value;
}

SENSORS_SENSOR(inttemp_sensor, INTTEMP_SENSOR,
               value, configure, status);
