/*
 * Copyright (c) 2011
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

#include "dev/tpa-sensor.h"
#include "dev/i2c.h"
#include <AppHardwareApi.h>

#define TPA81_ADDRESS 0xd0
#define REG_SOFT      0x00
#define REG_AMBIENT   0x01

static uint8_t ADDR = TPA81_ADDRESS;

static int
value(int type)
{
  uint8_t x = ADDR==TPA81_ADDRESS?8:0,
          y = ADDR==TPA81_ADDRESS?1:0;

  switch(type) {
  case TPA_X_DIMENSION:
    return x;
  case TPA_Y_DIMENSION:
    return y;
  default:
    break;
  }

  if (type >= TPA_AMBIENT_TEMPERATURE &&
      type < TPA_AMBIENT_TEMPERATURE + (x*y)+1)
  {
    uint8_t buf[] = {REG_AMBIENT + (type-TPA_AMBIENT_TEMPERATURE),0};

    i2cb(ADDR,1,1,&buf);
    return (int) buf[1];
  }

  return 0;
}

static int
configure(int type, int v)
{
  uint8_t buf[2] = {REG_SOFT};

  switch (type) {
  case SENSORS_HW_INIT:
    return 1;
  case SENSORS_ACTIVE:
    /* try and read softare revision register to see if device is present. */
    return i2cb(TPA81_ADDRESS,1,1,buf);
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

SENSORS_SENSOR(tpa_sensor, TPA_SENSOR,
               value, configure, status);
