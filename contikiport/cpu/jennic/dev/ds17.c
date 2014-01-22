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
 * Driver for the Ds1721 temperature sensors.
 */
#include "lib/sensors.h"
#include "dev/ds17.h"
#include "dev/i2c.h"
#include "net/rime/ctimer.h"
#include <stdbool.h>
#include "gdb2.h"

const struct sensors_sensor ds17_sensor;
static struct ctimer        interval;
static uint16_t             _value=0;
static bool                 _active;

#define DS17_ADDR     0x9e
#define DS17_REG_TEMP 0xaa
#define DS17_REG_CONF 0xac
#define DS17_START    0x51
#define DS17_STOP     0x22

struct conf_reg {
  uint8_t done:1,
          reserved:2,
          undefined:1,
          resolution:2,
          polarity:1,
          oneshot:1;
};

static void
read(void *ptr)
{
  uint8_t buf[] = {DS17_REG_TEMP,0,0};
  i2cb(DS17_ADDR,1,2,buf);
  _value = buf[1]<<8|buf[0];
  sensors_changed(&ds17_sensor);
  ctimer_reset(&interval);
}

static int
configure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
    _value = 0;
    {
      I2W(DS17_ADDR,DS17_REG_CONF, 0x00);
    }
    return 1;
  case SENSORS_ACTIVE:
    if (value) {
      I2W(DS17_ADDR,DS17_START);
      ctimer_set(&interval, CLOCK_SECOND, read, NULL);
    }
    else {
      I2W(DS17_ADDR,DS17_STOP);
      ctimer_stop(&interval);
    }
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

static void
value(int type)
{
  return _value;
}

SENSORS_SENSOR(ds17_sensor, DS17_SENSOR,
               value, configure, status);
