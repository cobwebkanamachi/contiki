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
 * Driver for the rgb leds found on the landmarke device.
 */

#include "rgb_leds.h"
#include "contiki.h"
#include <AppHardwareApi.h>
#include <stdbool.h>
#include <stdint.h>

#include "gdb2.h"

#define STEP0_DIR E_AHI_DIO1_INT
#define STEP1_DIR E_AHI_DIO3_INT
#define STEP0_INC E_AHI_DIO0_INT
#define STEP1_INC E_AHI_DIO2_INT
#define RED_LED   E_AHI_DIO20_INT
#define GREEN_LED E_AHI_DIO19_INT
#define BLUE_LED  E_AHI_DIO18_INT
#define LED_ENB   E_AHI_DIO13_INT
#define TIMER     E_AHI_TIMER_0

#define DEFAULT_BLINK_TIME 1000

PROCESS(rgb_leds, "rgb leds process");

void
rgb_leds_init()
{
  vAHI_DioSetDirection(0, STEP0_DIR| STEP1_DIR|
                          STEP0_INC| STEP1_INC|
                          RED_LED | GREEN_LED | BLUE_LED|
                          LED_ENB);
  vAHI_DioSetOutput(LED_ENB, 0);
  rgb_leds_brightness(0);
  rgb_leds_color(0,0,0);
  rgb_leds_blink(0);
}

/* brightness control */
void /* 32 step voltage regulator */
rgb_leds_brightness(uint8_t n)
{
  /* startup setting of regulator is 0, maximum is 32 */
  static int current=0;

  int steps = n - current;

  if (steps < 0) { /* direction down */
    vAHI_DioSetOutput(0, STEP0_DIR|STEP1_DIR);
    steps *= -1;
  } else           /* direction up */
    vAHI_DioSetOutput(STEP0_DIR|STEP1_DIR, 0);

  if (steps > 32)
    steps=32;

  /* pulse steps */
  while(steps--) {
    vAHI_DioSetOutput(0, STEP0_INC|STEP1_INC);
    vAHI_DioSetOutput(STEP0_INC|STEP1_INC, 0);
  }

  current=n;
}

/* color pwm mixing + blinking support */
static struct {
  uint8_t  r,g,b;
  clock_time_t  blink;
}rgb;

static void
switch_led(uint8_t pulsecount, uint8_t pulseson, uint32_t led)
{
  if (pulsecount < pulseson)
    vAHI_DioSetOutput(led, 0);
  else
    vAHI_DioSetOutput(0, led);
}

void
rgb_leds_color(uint8_t r, uint8_t g, uint8_t b)
{
  rgb.r = r;
  rgb.g = g;
  rgb.b = b;

  process_exit(&rgb_leds);
  process_start(&rgb_leds, NULL);
}

void
rgb_leds_blink(clock_time_t t)
{
  if (t==0)
    rgb.blink = DEFAULT_BLINK_TIME;
  else
    rgb.blink = t;

  process_exit(&rgb_leds);
  process_start(&rgb_leds, NULL);
}

PROCESS_THREAD(rgb_leds, ev, data)
{
  static struct etimer et;
  static uint8_t cycle = 0;
  PROCESS_BEGIN();

  while(1)
  {
    /* pwm cycle leds */
    etimer_set(&et, rgb.blink);
    cycle = 0;
    do
    {
      cycle++;
      switch_led(cycle, rgb.r, RED_LED);
      switch_led(cycle, rgb.g, GREEN_LED);
      switch_led(cycle, rgb.b, BLUE_LED);
      PROCESS_PAUSE();
    } while(ev!=PROCESS_EVENT_TIMER);

    /* turn off leds */
    switch_led(0, 0, RED_LED);
    switch_led(0, 0, GREEN_LED);
    switch_led(0, 0, BLUE_LED);
    etimer_set(&et, rgb.blink);
    PROCESS_YIELD_UNTIL(ev==PROCESS_EVENT_TIMER);
  }

  PROCESS_END();
}
