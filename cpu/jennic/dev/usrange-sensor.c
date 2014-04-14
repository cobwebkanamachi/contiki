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
 * Driver for the SRF04 ultrasonic range finder.
 */
#include "lib/sensors.h"
#include "dev/usrange-sensor.h"
#include "net/rime/ctimer.h"
#include "dev/i2c.h"
#include <stdbool.h>

const struct sensors_sensor usrange_sensor;
static struct ctimer        interval;
static bool                 _active = false;
static uint16_t             _value = 0;

#define SRF_ADDR            (0xE0)
#define SRF_CMD_REG         (0x00)
#define SRF_RES_REG         (0x02)
#define SRF_REAL_RANGING_CM (0x51)
#define SRF_FAKE_RANGING_CM (0x57)

static u8_t cmd[] = {SRF_CMD_REG, SRF_FAKE_RANGING_CM};

/*---------------------------------------------------------------------------*/
static void
sequence(void *ptr)
{
  u8_t buf[3] = {c,0,0};

  /* read and prepare next cycle */
  buf[0]=SRF_RES_REG;
  i2cb(SRF_ADDR,1,2,buf);
  _value = (buf[1]<<8)|buf[0];
  I2CW(SRF_ADDR,cmd);

  sensors_changed(&usrange_sensor);
}

/*---------------------------------------------------------------------------*/
static void
init(void)
{
  // XXX: make sure the read interval is not below 100ms for the SRF02
  // ultrasonic ranger, thats the time needed to acquire a new sample
  // and give the signal enough time to fade away.
  ctimer_set(&interval, CLOCK_SECOND/10, sequence, NULL);
  ctimer_stop(&interval);
}

/*---------------------------------------------------------------------------*/
static void
activate(void)
{
  _active = true;

  I2CW(SRF_ADDR, cmd);
  ctimer_reset(&interval);
}

/*---------------------------------------------------------------------------*/
static void
deactivate(void)
{
  ctimer_stop(&interval);
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
SENSORS_SENSOR(usrange_sensor, USRANGE_SENSOR,
               init, NULL, activate, deactivate, active,
               value, configure, status);
