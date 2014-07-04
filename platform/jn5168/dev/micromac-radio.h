/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         MICROMAC_RADIO driver header file
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */

#ifndef MICROMAC_RADIO_H_
#define MICROMAC_RADIO_H_

#include "contiki.h"
#include "dev/radio.h"
#include <MMAC.h>
#include "net/rime/rimeaddr.h"

void copy_to_rimeaddress(rimeaddr_t* addr, tuAddr* tu_addr);
void copy_from_rimeaddress(tuAddr*, rimeaddr_t* );

#define MICROMAC_HEADER_LEN (28)

int micromac_radio_init(void);

#define MICROMAC_RADIO_MAX_PACKET_LEN      127

int micromac_radio_set_channel(int channel);
int micromac_radio_get_channel(void);

void micromac_radio_set_pan_addr(unsigned pan,
                                unsigned addr,
                                const uint8_t *ieee_addr);

//extern signed char micromac_radio_last_rssi;
//extern uint8_t micromac_radio_last_correlation;

int micromac_radio_rssi(void);

extern const struct radio_driver micromac_radio_driver;

/**
 * \param power Between 0 and 3.
 */
void micromac_radio_set_txpower(uint8_t power);
int micromac_radio_get_txpower(void);
#define MICROMAC_RADIO_TXPOWER_MAX  3
#define MICROMAC_RADIO_TXPOWER_MIN  0

///* XXX hack: these will be made as Chameleon packet attributes */
//extern rtimer_clock_t micromac_radio_time_of_arrival,
//  micromac_radio_time_of_departure;

int micromac_radio_on(void);
int micromac_radio_off(void);

void micromac_radio_set_cca_threshold(int value);
void micromac_get_hw_mac_address(tsExtAddr *psExtAddress);

uint32_t* micromac_get_hw_mac_address_location(void);


#endif /* MICROMAC_RADIO_H_ */
