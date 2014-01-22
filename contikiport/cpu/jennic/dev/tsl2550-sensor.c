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
 * Driver for TAOS TSL2550.
 * Conversion to lux by Application Note from TAOS.
 */
#include "lib/sensors.h"
#include "dev/lightlevel-sensor.h"
#include "dev/i2c.h"
#include <stdbool.h>
#include <AppHardwareApi.h>

#ifndef JENNIC_CONF_TSL2550_CONVERT
# define CONVERT_LUX 1
#else
# define CONVERT_LUX JENNIC_CONF_TSL2550_CONVERT
#endif

#define TSL_ADDR        0x72
#define TSL_PWR_UP      0x03
#define TSL_PWR_DWN     0x00
#define TSL_STD_RANGE   0x18
#define TSL_EXT_RANGE   0x1D
#define TSL_READ_ADC0   0x43
#define TSL_READ_ADC1   0x83
#define TSL_READ_CMDREG 0x03

#if CONVERT_LUX
static const u8_t ratio[129] = {
  100,100,100,100,100,100,100,100,
  100,100,100,100,100,100,99,99,
  99,99,99,99,99,99,99,99,
  99,99,99,98,98,98,98,98,
  98,98,97,97,97,97,97,96,
  96,96,96,95,95,95,94,94,
  93,93,93,92,92,91,91,90,
  89,89,88,87,87,86,85,84,
  83,82,81,80,79,78,77,75,
  74,73,71,69,68,66,64,62,
  60,58,56,54,52,49,47,44,
  42,41,40,40,39,39,38,38,
  37,37,37,36,36,36,35,35,
  35,35,34,34,34,34,33,33,
  33,33,32,32,32,32,32,31,
  31,31,31,31,30,30,30,30,
  30
};

static const u16_t count[128] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18, 20, 22, 24, 26,
  28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 49, 53, 57, 61, 65, 69, 73, 77, 81,
  85, 89, 93, 97, 101, 105, 109, 115, 123, 131, 139, 147, 155, 163, 171, 179,
  187, 195, 203, 211, 219, 227, 235, 247, 263, 279, 295, 311, 327, 343, 359,
  375, 391, 407, 423, 439, 455, 471, 487, 511, 543, 575, 607, 639, 671, 703,
  735, 767, 799, 831, 863, 895, 927, 959, 991,
  1039,1103,1167,1231,1295,1359,1423,1487,
  1551,1615,1679,1743,1807,1871,1935,1999,
  2095,2223,2351,2479,2607,2735,2863,2991,
  3119,3247,3375,3503,3631,3759,3887,4015
};
#endif

#define IS_VALID (1<<7)

static int
value(int type)
{
  static const u16_t max = 1846;
  u8_t buf[2] = {0,0};
  u8_t ch0=0x00, ch1=0x00, r;
  u32_t lux;

  switch(type)
  {
    case LIGHT_VALUE_VISIBLE_CENTILUX:
      while (!(ch0 & IS_VALID)) {
        buf[0] = TSL_READ_ADC0; i2cb(TSL_ADDR,1,1,buf);
        ch0 = buf[1];
      }

      while (!(ch1 & IS_VALID)) {
        buf[0] = TSL_READ_ADC1; i2cb(TSL_ADDR,1,1,buf);
        ch1 = buf[1];
      }

      /* strip valid bits */
      ch0 &= 0xff - IS_VALID;
      ch1 &= 0xff - IS_VALID;

#if CONVERT_LUX
      if (count[ch0] && count[ch1] <= count[ch0])
        r = count[ch1] * 128 / count[ch0];
      else
        r = 128;

      lux = ((count[ch0]-count[ch1]) * ratio[r]) / 256;
      if (lux>max) lux = max;

      return lux*100;
#else
      return (ch0<<8)|ch1;
#endif
    case TSL_VALUE_CHANNEL0:
      while (!(ch0 & IS_VALID)) {
        buf[0] = TSL_READ_ADC0; i2cb(TSL_ADDR,1,1,buf);
      }

      return buf[1];
    case TSL_VALUE_CHANNEL1:
      while (!(ch1 & IS_VALID)) {
        buf[0] = TSL_READ_ADC1; i2cb(TSL_ADDR,1,1,buf);
      }

      return buf[1];
  }

  return 0;
}

static int
configure(int type, int value)
{
  static u8_t operating_mode = TSL_STD_RANGE;
  u8_t c;

  switch(type) {
  case SENSORS_HW_INIT:
#ifdef JENNIC_CONF_TSL2550_PWRPIN
    vAHI_DioSetDirection(0x00, JENNIC_CONF_TSL2550_PWRPIN);
    vAHI_DioSetOutput(JENNIC_CONF_TSL2550_PWRPIN, 0x00);
#endif
    return 1;

  case SENSORS_ACTIVE:
    if (!I2CW(TSL_ADDR,(c = value ? TSL_PWR_UP : TSL_PWR_DWN)))
    {
      return false;
    }
    else
    {
      configure(TSL_CONFIGURE_RANGE, operating_mode);
      return 1;
    }

  case TSL_CONFIGURE_RANGE:
    return I2CW(TSL_ADDR,(TSL_CONFIGURE_RANGE_EXT ? TSL_EXT_RANGE : TSL_STD_RANGE));
  }

  return 0;
}

static int*
status(int type)
{
  u8_t buf[2] = {0,0};
  u8_t res = false;

  switch(type) {
  case SENSORS_ACTIVE:
    buf[0] = TSL_READ_CMDREG;
    i2cb(TSL_ADDR,1,1,buf);
    return (int*) (buf[1]&TSL_PWR_UP);
  case SENSORS_READY:
    buf[0] = TSL_READ_ADC0;
    i2cb(TSL_ADDR,1,1,buf);
    res = buf[1] & (1<<7); // read valid bit

    buf[0] = TSL_READ_ADC1;
    i2cb(TSL_ADDR,1,1,buf);
    res &= buf[1] & (1<<7); // read valid bit

    return (int*) res;
    break;
  }

  return NULL;
}

SENSORS_SENSOR(lightlevel_sensor, LIGHTLEVEL_SENSOR,
               value, configure, status);
