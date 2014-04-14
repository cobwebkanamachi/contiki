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
 * Driver for TLC59116 led driver connected to two 7seg displays.
 */
#include "dev/sseg.h"
#include "dev/i2c.h"
#include "net/rime/ctimer.h"

#include <stdbool.h>
#include <stdint.h>

#include "gdb2.h"

/* address configured by A3-A0 to high */
#define ADDR      0xde

/* register addresses */
#define MODE1     0x00
#define PWM_REG0  0x02
#define PWM_REG15 0x11
#define GRPPWM    0x12
#define GRPFREQ   0x13
#define LEDOUT0   0x14
#define LEDOUT1   0x15
#define LEDOUT2   0x16
#define LEDOUT3   0x17

/* command modes */
#define AUTO_INC            0x80
#define AUTO_INC_BRIGHT     0xa0
#define AUTO_INC_GLBL_CTRL  0xc0
#define AUTO_INC_CTRL       0xe0

/* led modes */
#define LED_OFF  0x00
#define LED_ON   0x01
#define LED_PWM  0x02
#define LED_GRP  0x04

/* this struct describes the layout of the 7-seg display connected
 * via the LED controller in terms of the LEDSOUTx register structure
 * found in the datasheet:
 *
 *         -- a --    -- h --
 *        |       |  |       |
 *        f       b  m       i
 *        |       |  |       |
 *         -- g --    -- n --
 *        |       |  |       |
 *        e       c  l       j
 *        |       |  |       |
 *         -- d --x   -- k --y
 *
 * NOTE: the ORDER of the elements in the following struct define the pin
 *       connection between led driver and 7-seg display!
 */
struct sseg_layout
{
  uint8_t b:2,
          a:2,
          f:2,
          g:2,
          i:2,
          h:2,
          m:2,
          n:2,
          l:2,
          k:2,
          j:2,
          y:2,
          e:2,
          d:2,
          c:2,
          x:2;
} __attribute((__packed__));

struct sseg_mode1
{
  uint8_t autoinc:3,
          sleep:1,
          sub:3,
          all:1;
} __attribute((__packed__));

struct sseg_cmd
{
  uint8_t byte;
  union {
    struct sseg_layout layout;
    struct sseg_mode1  mode1;
  };
} __attribute((__packed__));

#include "gdb2.h"

void
sseg_init()
{
  uint8_t reset[] = { 0xa5, 0x5a };
  struct sseg_cmd init = {.byte  = MODE1,
                         {.mode1 = { .sleep = 1, .sub = 0, .all = 1 }}};
  I2CW(0xd6,0xa5,0x5a); /* reset */
  I2CW(ADDR,MODE1,(sseg_mode1) {.sleep=1,.sub=0,.all=1});
}

static void
load_digit(uint8_t d1, uint8_t d2, struct sseg_cmd *cmd)
{
  struct sseg_layout *x = &cmd->layout;
  uint8_t on = LED_ON;

  if(d1 != SSEG_OFF) {
  if (d1 >= 10) x->x = on;
  switch(d1%10) {
  case 0:
    x->a = x->b = x->c= x->d = x->e = x->f = on;
    break;
  case 1:
    x->b = x->c = on;
    break;
  case 2:
    x->a = x->b = x->g = x->e = x->d = on;
    break;
  case 3:
    x->a = x->b = x->g = x->c = x->d = on;
    break;
  case 4:
    x->f = x->b = x->g = x->c = on;
    break;
  case 5:
    x->a = x->f = x->g = x->c = x->d = on;
    break;
  case 6:
    x->a = x->f = x->g = x->e = x->c = x->d = on;
    break;
  case 7:
    x->a = x->b = x->c = on;
    break;
  case 8:
    x->a = x->b = x->c = x->d = x->e = x->f = x->g = on;
    break;
  case 9:
    x->a = x->f = x->b = x->g = x->c = x->d = on;
    break;
  case 10:
    x->x = on;
    break;
  default:
    break;
  }
  }


  if (d2!=SSEG_OFF) {
  if (d2 >= 10) x->y = on;
  switch(d2%10) {
  case 0:
    x->h = x->i = x->j= x->k = x->l = x->m = on;
    break;
  case 1:
    x->i = x->j = on;
    break;
  case 2:
    x->h = x->i = x->n = x->l = x->k = on;
    break;
  case 3:
    x->h = x->i = x->n = x->j = x->k = on;
    break;
  case 4:
    x->m = x->i = x->n = x->j = on;
    break;
  case 5:
    x->h = x->m = x->n = x->j = x->k = on;
    break;
  case 6:
    x->h = x->m = x->n = x->l = x->j = x->k = on;
    break;
  case 7:
    x->h = x->i = x->j = on;
    break;
  case 8:
    x->h = x->i = x->j = x->k = x->l = x->m = x->n = on;
    break;
  case 9:
    x->h = x->m = x->i = x->n = x->j = x->k = on;
    break;
  case 10:
    x->y = on;
    break;
  default:
    break;
  }
  }
}

static uint8_t sseg1=SSEG_OFF,sseg2=SSEG_OFF;

static void
set(char c, uint8_t to)
{
  struct sseg_cmd cmd = { LEDOUT0|AUTO_INC, {{0x0000}} };
  load_digit(sseg1, sseg2, &cmd);

  switch (c) {
  case 'a':
    cmd.layout.a = to;
    break;
  case 'b':
    cmd.layout.b = to;
    break;
  case 'c':
    cmd.layout.c = to;
    break;
  case 'd':
    cmd.layout.d = to;
    break;
  case 'e':
    cmd.layout.e = to;
    break;
  case 'f':
    cmd.layout.f = to;
    break;
  case 'g':
    cmd.layout.g = to;
    break;
  case 'h':
    cmd.layout.h = to;
    break;
  case 'i':
    cmd.layout.i = to;
    break;
  case 'j':
    cmd.layout.j = to;
    break;
  case 'k':
    cmd.layout.k = to;
    break;
  case 'l':
    cmd.layout.l = to;
    break;
  case 'm':
    cmd.layout.m = to;
    break;
  case 'n':
    cmd.layout.n = to;
    break;
  case 'x':
    cmd.layout.x = to;
    break;
  case 'y':
    cmd.layout.y = to;
    break;
  }

  i2cb(ADDR, sizeof(cmd), 0, (uint8_t*) &cmd);
}

void
sseg_set(char c)
{
  set(c, LED_ON);
}

void
sseg_unset(char c)
{
  set(c, LED_OFF);
}

static void
show()
{
  struct sseg_cmd cmd = { LEDOUT0|AUTO_INC, {{0x0000}} };
  load_digit(sseg1, sseg2, &cmd);
  i2cb(ADDR, sizeof(cmd), (uint8_t*) &cmd);
}

void
sseg_set1(uint8_t c)
{
  sseg1 = c;
  show();
}

void
sseg_set2(uint8_t c)
{
  sseg2 = c;
  show();
}

void sseg_set12(uint8_t c, uint8_t d)
{
  sseg1 = c;
  sseg2 = d;
  show();
}
