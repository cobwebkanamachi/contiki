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
 * Driver for ADXL345 Acceleration sensor.
 */

#include "dev/i2c.h"
#include "dev/acc-sensor.h"
#include "dev/mag-sensor.h"
#include <contiki-net.h>
#include <AppHardwareApi.h>

/* ------- Register names ------- */
#define LSM303_ADDR_A            0x30
#define LSM303_ADDR_M            0x3c
#define LSM303_CTRL_REG1         0x20
#define LSM303_CTRL_REG2         0x21
#define LSM303_CTRL_REG3         0x22
#define LSM303_CTRL_REG4         0x23
#define LSM303_CTRL_REG5         0x24
#define LSM303_REF               0x25
#define LSM303_STATUS            0x27
#define LSM303_OUT_X_A           0x28
#define LSM303_OUT_Y_A           0x30
#define LSM303_OUT_Z_A           0x32
#define LSM303_CRA_REG_M         0x00
#define LSM303_CRB_REG_M         0x01
#define LSM303_MR_REG_M          0x02
#define LSM303_OUT_X_M           0x03
#define LSM303_OUT_Z_M           0x05
#define LSM303_OUT_Y_M           0x07
#define LSM303_SR_REG_M          0x09
#define LSM303_IRA_REG_M         0x0A
#define LSM303_IRB_REG_M         0x0B
#define LSM303_IRC_REG_M         0x0C
#define LSM303_WHO_AM_I          0x0F

#define X_READ (1<<0)
#define Y_READ (1<<1)
#define Z_READ (1<<2)

static i2c_t acc_transaction;
static bool acc_active = false;

static void
acc_startsample(bool status)
{ /* this callback is *not* called in irq-context */
  if (acc_active) i2c(&acc_transaction);
  sensors_changed(&acc_sensor);
}

/* weird implementation on the accelerometer, all values have to be read at
 * once to start a new sample!
 * So resample whenever the same value is read twice!
 * And set MSB on the register address to enable multiple bytes read. */
static i2c_t acc_transaction = {
  .addr  = LSM303_ADDR_A,
  .cb    = acc_startsample,
  .wrlen = 1,
  .rdlen = 6,
  .buf   = {LSM303_OUT_X_A|(1<<7),0,0,0,0,0,0}
};

static int
avalue(int type)
{
  switch(type) {
  case ACC_VALUE_X:
    return (int16_t) (acc_transaction.buf[2]<<8|
                      acc_transaction.buf[1])>>4;

  case ACC_VALUE_Y:
    return (int16_t) (acc_transaction.buf[4]<<8|
                      acc_transaction.buf[3])>>4;

  case ACC_VALUE_Z:
    return (int16_t) (acc_transaction.buf[6]<<8|
                      acc_transaction.buf[5])>>4;

  default:
    return 0;
  }

  return 0;
}

static int
aconfigure(int type, int v)
{
  uint8_t buf[2];

  switch (type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    if (v) {
      /* normal mode, 100Hz output rate, enable all axes */
      if (!I2CW(LSM303_ADDR_A,LSM303_CTRL_REG1,0x2f))
        return false; /* see if we get an ack */

      /* +-2g, block update enabled */
      I2CW(LSM303_ADDR_A,LSM303_CTRL_REG4,0x80);

      acc_active = true;
      acc_startsample(true);
      return true;
    }
    else {
      acc_active = false;
      return I2CW(LSM303_ADDR_A,LSM303_CTRL_REG1,0x00);
    }
  case ACC_LSM303_RANGE:
    I2CW(LSM303_ADDR_A,LSM303_CTRL_REG4,(1<<7)|((v&3)<<4));
    break;

  case ACC_LSM303_DRATE:
    if (v<ACC_LSM303VAL_0_5HZ || v>ACC_LSM303VAL_1KHZ)
      return 1;
    else if (v<ACC_LSM303VAL_50HZ)  /* low-power mode */
      I2CW(LSM303_ADDR_A,LSM303_CTRL_REG1,(v<<5)|0x07);
    else if (v>=ACC_LSM303VAL_50HZ) /* normal mode    */
      I2CW(LSM303_ADDR_A,LSM303_CTRL_REG1,(1<<5)|((v-ACC_LSM303VAL_50HZ)<<3)|0x07);
    break;

  case ACC_LSM303_HIGHPASS:
    buf[0]=LSM303_CTRL_REG2;
    i2cb(LSM303_ADDR_A,1,1,buf);
    buf[1]&=~(0x02);
    buf[1]|=v|(1<<5);
    i2cb(LSM303_ADDR_A,2,0,buf);
    break;

  case ACC_LSM303_FILTER:
    buf[0]=LSM303_CTRL_REG2;
    i2cb(LSM303_ADDR_A,1,1,buf);

    if (v==ACC_LSM303VAL_NONE)
      buf[1]&=~(1<<4); // unset FDS bit
    else if (v==ACC_LSM303VAL_BOTH)
      buf[1]|=(1<<4);  // set FDS bit

    i2cb(LSM303_ADDR_A,2,0,buf);
    break;
  }

  return 0;
}

static int
astatus(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
    return 1;
  case SENSORS_READY:
    return 1;
  }

  return 0;
}

static i2c_t mag_transaction;
static bool mag_active = false;

static
mag_startsample(bool status)
{ /* this callback is *not* called in irq-context */
  if (mag_active) i2c(&mag_transaction);
  sensors_changed(&mag_sensor);
}

static i2c_t mag_transaction = {
  .addr  = LSM303_ADDR_M,
  .cb    = mag_startsample,
  .wrlen = 1,
  .rdlen = 6,
  .buf   = {LSM303_OUT_X_M,0,0,0,0,0,0}
};

static int
mvalue(int type)
{
  switch(type) {
  case MAG_VALUE_X:
    return (int16_t) (mag_transaction.buf[1]|(mag_transaction.buf[2]<<8));

  case MAG_VALUE_Y:
    return (int16_t) (mag_transaction.buf[5]|(mag_transaction.buf[6]<<8));

  case MAG_VALUE_Z:
    return (int16_t) (mag_transaction.buf[3]|(mag_transaction.buf[4]<<8));

  default:
    return 0;
  }

  return 0;
}

static int
mconfigure(int type, int v)
{
  switch (type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    if (v)
    {
      if (!I2CW(LSM303_ADDR_M,LSM303_CRA_REG_M,0x1c))
        return false; /* output rate: 220Hz */
      //I2CW(LSM303_ADDR_M,LSM303_CRB_REG_M,0xe0);  /* gain +8.1Gauss  */
      I2CW(LSM303_ADDR_M,LSM303_CRB_REG_M,0x20);  /* gain +1.3Gauss  */
      I2CW(LSM303_ADDR_M,LSM303_MR_REG_M,0x00);   /* continuous mode */

      mag_active = true;
      mag_startsample(true);
      return true;
    }
    else
    {
      mag_active = false;
      return I2CW(LSM303_ADDR_M,LSM303_MR_REG_M,0x03); /* sleep mode */
    }
    break;
  }

  return 0;
}

static int
mstatus(int type)
{
  // TODO

  switch(type) {
  case SENSORS_ACTIVE:
    return 1;
  case SENSORS_READY:
    return 1;
  }

  return 0;
}

SENSORS_SENSOR(acc_sensor, ACC_SENSOR, avalue, aconfigure, astatus);
SENSORS_SENSOR(mag_sensor, MAG_SENSOR, mvalue, mconfigure, mstatus);
