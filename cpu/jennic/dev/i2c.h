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
 */

#ifndef __I2C_H__
#define __I2C_H__
#include "contiki.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct i2c_handle {
  struct i2c_handle *next;
  void  (*cb)(bool res);
  u8_t    addr,wrlen,rdlen,buf[];
} i2c_t;

/* execute one i2c transaction,
 * writing n chars from wr and afterwards reading m chars to rd.
 *
 * callback can either be NULL, in which case the functions busy loops until the
 * transaction is complete and returns the status. Or callback will be called
 * with the tranaction status once it has completed in interrupt context.
 *
 * If a callback is given, multiple calls will be queued and execute in the
 * order of the original calls to i2c().  */
void i2c(i2c_t *t);
bool i2cb(u8_t addr, u8_t wrlen, u8_t rdlen, u8_t buf[]);
#define I2CW(addr,args...) i2cb(addr,sizeof((u8_t[]){args}),0,(u8_t[]){args})

#endif
