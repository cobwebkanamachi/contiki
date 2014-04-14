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

#include "dev/infrared-sensor.h"
#include <AppHardwareApi.h>
#include <stdbool.h>
#include <stdint.h>

static int
configure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
    vAHI_UartEnable(E_AHI_UART_0);
    vAHI_UartReset(E_AHI_UART_0, true, true);
    vAHI_UartReset(E_AHI_UART_0, false, false);

    // set baudrate to 2400, divisor is derived from 1mHz clock
    vAHI_UartSetBaudDivisor(E_AHI_UART_0,417);

    vAHI_DioSetDirection(0x00, JENNIC_CONF_TSOP362_PWRPIN);

    return 1;
  case SENSORS_ACTIVE:
#ifdef JENNIC_CONF_TSOP362_PWRPIN
    if (value)
      vAHI_DioSetOutput(JENNIC_CONF_TSOP362_PWRPIN, 0x00);
    /* pin will not be switched to avoid effect on mcp9800 */
#endif

    return 1;
  }

  return 0;
}

static void*
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
#ifdef JENNIC_CONF_TSOP362_PWRPIN
    return u32AHI_DioReadInput()&JENNIC_CONF_TSOP362_PWRPIN;
#else
    return 1;
#endif
  case SENSORS_READY:
    return (u8AHI_UartReadLineStatus(E_AHI_UART_0) & E_AHI_UART_LS_DR);
  }

  return 0;
}

static int
value(int type)
  /* loop at max 255 times, flushing the input buffer and returning the latest
   * byte in the buffer. return 0 if there is nothing to read. */
{
  uint8_t c=0, i=1;

  while (u8AHI_UartReadLineStatus(E_AHI_UART_0) & E_AHI_UART_LS_DR && i++)
    c = u8AHI_UartReadData(E_AHI_UART_0);

  return c;
}

SENSORS_SENSOR(infrared_sensor, INFRARED_SENSOR,
               value, configure, status);
