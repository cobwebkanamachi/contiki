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

#ifndef __JN_IRQ_H__
#define __JN_IRQ_H__

#include <stdbool.h>
#include <stdint.h>
#include "lib/list.h"
#include "AppHardwareApi.h"

typedef enum {
  IRQ_DIO0 =              0x00000001, /* same as E_AHI_DIO*_INT defs ... */
  IRQ_DIO1 =              0x00000002,
  IRQ_DIO2 =              0x00000004,
  IRQ_DIO3 =              0x00000008,
  IRQ_DIO4 =              0x00000010,
  IRQ_DIO5 =              0x00000020,
  IRQ_DIO6 =              0x00000040,
  IRQ_DIO7 =              0x00000080,
  IRQ_DIO8 =              0x00000100,
  IRQ_DIO9 =              0x00000200,
  IRQ_DIO10 =             0x00000400,
  IRQ_DIO11 =             0x00000800,
  IRQ_DIO12 =             0x00001000,
  IRQ_DIO13 =             0x00002000,
  IRQ_DIO14 =             0x00004000,
  IRQ_DIO15 =             0x00008000,
  IRQ_DIO16 =             0x00010000,
  IRQ_DIO17 =             0x00020000,
  IRQ_DIO18 =             0x00040000,
  IRQ_DIO19 =             0x00080000,
  IRQ_DIO20 =             0x00100000,

  IRQ_ADC1  =             0x00200000, /* ... until here */
  IRQ_ADC2  =             0x00400000,
  IRQ_ADC3  =             0x00800000,
  IRQ_ADC4  =             0x01000000,
  IRQ_ADC_TEMP  =         0x02000000,
  IRQ_ADC_VOLT  =         0x04000000,
} __attribute((__packed__)) irq_t;

typedef enum
{
  ADC_INPUT_RANGE_1 = E_AHI_AP_INPUT_RANGE_1, /* single vRef */
  ADC_INPUT_RANGE_2 = E_AHI_AP_INPUT_RANGE_2  /* double vRef */
} __attribute((__packed__)) adc_t;

typedef struct irq_handle {
  struct irq_handle *next;
  void             (*callback)(irq_t);
  irq_t              irqsrc;
  adc_t              adc_input_range; /* optional */
} irq_handle_t;

void irq_init();
void irq_add(const struct irq_handle *);
void irq_remove(const struct irq_handle *);

#endif
