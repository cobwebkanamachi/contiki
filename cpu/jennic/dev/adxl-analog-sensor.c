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
 * Driver for ADXL sensor with analog outputs
 */
#include "dev/irq.h"
#include "dev/acc-sensor.h"
#include <AppHardwareApi.h>
#include <stdbool.h>

static volatile uint16_t x,y,z;
const struct sensors_sensor acc_sensor;

static void
irq(irq_t s)
{
  switch(s) {
  case IRQ_ADC1:
    x = u16AHI_AdcRead();
    break;
  case IRQ_ADC2:
    y = u16AHI_AdcRead();
    break;
  case IRQ_ADC3:
    z = u16AHI_AdcRead();
    break;
  default:
    break;
  }

  sensors_changed(&acc_sensor);
}

static int
configure(int type, int value)
{
  static irq_handle_t _x = {.callback=irq, .irqsrc=IRQ_ADC1,
                            .adc_input_range=ADC_INPUT_RANGE_2},
                      _y = {.callback=irq, .irqsrc=IRQ_ADC2,
                            .adc_input_range=ADC_INPUT_RANGE_2},
                      _z = {.callback=irq, .irqsrc=IRQ_ADC3,
                            .adc_input_range=ADC_INPUT_RANGE_2};

  switch (type) {
  case SENSORS_HW_INIT:
    x = y = z = 0;
    return 1;
  case SENSORS_ACTIVE:
    if (value) { irq_add(&_x); irq_add(&_y); irq_add(&_z); }
    else       { irq_remove(&_x); irq_remove(&_y); irq_remove(&_z); }
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
    break;
  }

  return 0;
}

static int
value(int type)
{
  switch(type) {
  case ACC_VALUE_X:
    return x;
  case ACC_VALUE_Y:
    return y;
  case ACC_VALUE_Z:
    return z;
  default:
    return 0;
  }
}

SENSORS_SENSOR(acc_sensor, ACC_SENSOR,
               value, configure, status);
