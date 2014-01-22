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

#include "dev/utils.h"
#include "dev/acc-sensor.h"
#include "dev/hrclock.h"
#include <contiki-net.h>
#include <AppHardwareApi.h>

/* ------- Register names ------- */
#define ADXL345_ADDR 0xA6
#define ADXL345_DEVID 0x00
#define ADXL345_RESERVED1 0x01
#define ADXL345_THRESH_TAP 0x1d
#define ADXL345_OFSX 0x1e
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_THRESH_ACT 0x24
#define ADXL345_THRESH_INACT 0x25
#define ADXL345_TIME_INACT 0x26
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_THRESH_FF 0x28
#define ADXL345_TIME_FF 0x29
#define ADXL345_TAP_AXES 0x2a
#define ADXL345_ACT_TAP_STATUS 0x2b
#define ADXL345_BW_RATE 0x2c
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_FIFO_CTL 0x38
#define ADXL345_FIFO_STATUS 0x39

#ifndef JENNIC_CONF_ADXL345_INT0_PIN
# define INT0_PIN E_AHI_DIO16_INT
#else
# define INT0_PIN JENNIC_CONF_ADXL345_INT0_PIN
#endif

static bool
wreg(uint8_t r, uint8_t v, uint8_t mask)
{
  uint8_t buf[2] = {r};

  /* preserve last register value */
  if (!i2cb(ADXL345_ADDR, 1,1,buf))
    return false;

  buf[1] &= mask;
  buf[1] |= v;

  return i2cb(ADXL345_ADDR, 2,0,buf);
}

static int
value(int type)
{
  uint8_t buf[3], v;

  switch(type) {
  case ACC_VALUE_X:
    buf[0] =  ADXL345_DATAX0;
    i2cb(ADXL345_ADDR, 1,2,buf);
    return (int16_t) ((buf[1]<<8)|buf[0]);

  case ACC_VALUE_Y:
    buf[0] = ADXL345_DATAY0;
    i2cb(ADXL345_ADDR, 1,2,buf);
    return (int16_t) ((buf[1]<<8)|buf[0]);

  case ACC_VALUE_Z:
    buf[0] = ADXL345_DATAZ0;
    i2cb(ADXL345_ADDR, 1,2,buf);
    return (int16_t) ((buf[1]<<8)|buf[0]);

  case ACC_VALUE_INTSOURCE:
    buf[0] = ADXL345_INT_SOURCE;
    i2cb(ADXL345_ADDR, 1,1,buf);
    return buf[0];

  case ACC_VALUE_TAPSTATUS:
    buf[0] = ADXL345_ACT_TAP_STATUS;
    i2cb(ADXL345_ADDR, 1,1,buf);
    return buf[0];
  }

  return 0;
}

static void
irq(irq_t i)
{
  sensors_changed(&acc_sensor);
}

static const irq_handle_t irqh = {.callback=irq, .irqsrc=INT0_PIN};

static int
configure(int type, int v)
{
  bool r = true;

  switch (type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    return wreg(ADXL345_POWER_CTL, v?(1<<3):0, (1<<3));

  case ACC_SENSOR_RATE:
    return wreg(ADXL345_BW_RATE, v&0x0f, 0x0f);

  case ACC_SENSOR_FULLRES:
    return wreg(ADXL345_DATA_FORMAT, v?(1<<3):0x00, ~(1<<3));

  case ACC_SENSOR_RANGE:
    return wreg(ADXL345_DATA_FORMAT, v?0x03:0, ~0x03);

  case ACC_SENSOR_TAPENABLE:
    vAHI_DioSetDirection(INT0_PIN, 0x00);
    vAHI_DioInterruptEdge(INT0_PIN, 0x00);
    vAHI_DioInterruptEnable(INT0_PIN, 0x00);

    if (v) irq_add(&irqh);
    else   irq_remove(&irqh);

    r &= wreg(ADXL345_TAP_AXES, 0x07, 0x0f); /* all tap axes, no suppression */
    r &= wreg(ADXL345_INT_MAP, 0, ~((1<<6)|1<<5)); /* map to int0 */
    r &= wreg(ADXL345_INT_ENABLE, v?((1<<6)|(1<<5)):0, ~((1<<6)|1<<5));
    return r;

  case ACC_SENSOR_TAPTHRESH:
    return wreg(ADXL345_THRESH_TAP, v&0xff, 0xff);

  case ACC_SENSOR_TAPDUR:
    return wreg(ADXL345_DUR, v&0xff, 0xff);

  case ACC_SENSOR_TAPLATENT:
    return wreg(ADXL345_LATENT, v&0xff, 0xff);

  case ACC_SENSOR_TAPWINDOW:
    return wreg(ADXL345_WINDOW, v&0xff, 0xff);
  }

  return 0;
}

static int
status(int type)
{
  uint8_t buf[] = {ADXL345_POWER_CTL,0};

  switch(type) {
  case SENSORS_ACTIVE:
    i2cb(ADXL345_ADDR, 1,1,buf);
    return buf[0] & (1<<3);
  case SENSORS_READY:
    return 1;
  }

  return 0;
}

SENSORS_SENSOR(acc_sensor, ACC_SENSOR, value, configure, status);
