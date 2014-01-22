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
 * This is a driver for general smoke sensors ripped from smoke detectors.
 */
#include "dev/irq.h"
#include "dev/smoke-sensor.h"
#include <AppHardwareApi.h>
#include <stdbool.h>

const struct sensors_sensor smoke_sensor;

static bool              _active = false;
static volatile uint16_t _value = 0;

#ifndef JENNIC_CONF_SMOKE_SENSOR_ADCPIN
# define ADC_SRC IRQ_ADC_SRC1
#elif
# define ADC_SRC JENNIC_CONF_SMOKE_SENSOR_ADCPIN
#endif

/*---------------------------------------------------------------------------*/
static int
irq()
{
  _value = u16AHI_AdcRead();
  sensors_changed(&smoke_sensor);
  return 1;
}

/*---------------------------------------------------------------------------*/
static void
init(void)
{
}

/*---------------------------------------------------------------------------*/
static void
activate(void)
{
  sensors_add_irq(&smoke_sensor, ADC_SRC);
  adc_enable(ADC_SRC);
  _active = true;
}

/*---------------------------------------------------------------------------*/
static void
deactivate(void)
{
  sensors_remove_irq(&smoke_sensor, ADC_SRC);
  adc_disable(ADC_SRC);
  _active = false;
}

/*---------------------------------------------------------------------------*/
static int
active(void)
{
  return _active;
}

/*---------------------------------------------------------------------------*/
static unsigned int
value(int type)
{
  return _value;
}

/*---------------------------------------------------------------------------*/
static int
configure(int type, void *c)
{
  return 0;
}

/*---------------------------------------------------------------------------*/
static void *
status(int type)
{
  return NULL;
}

/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(smoke_sensor, SMOKE_SENSOR,
               init, irq, activate, deactivate, active,
               value, configure, status);
