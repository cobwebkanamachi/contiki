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
 *         Beshr Al Nahas <beshr@sics.se>
 */

#ifndef MICROMAC_RADIO_H_
#define MICROMAC_RADIO_H_

#include "contiki.h"
#include "dev/radio.h"
#include "rimeaddr.h"
#include <MMAC.h>

#define RADIO_TO_RTIMER(X) 											((rtimer_clock_t)( (uint32_t)(X) << (uint32_t)9 ))
#define RTIMER_TO_RADIO(X) 											((uint32_t)( (uint32_t)(X) >> (uint32_t)9 ))
#define MICROMAC_RADIO_MAX_PACKET_LEN      (127)
#define ACK_LEN (3)
#define MICROMAC_HEADER_LEN (28)

#define MAX_PACKET_DURATION RADIO_TO_RTIMER((MICROMAC_RADIO_MAX_PACKET_LEN+1))

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

struct received_frame_radio_s {
  uint8_t* buf;
  uint8_t len;
  rimeaddr_t source_address;
  rtimer_clock_t sfd_timestamp;
};

typedef void(*softack_make_callback_f)(uint8_t **ackbuf, uint8_t seqno, rtimer_clock_t last_packet_timestamp, uint8_t nack);
typedef void(*softack_interrupt_exit_callback_f)(uint8_t need_ack, struct received_frame_radio_s * last_rf);

void copy_from_rimeaddress(tuAddr*, rimeaddr_t* );
void copy_to_rimeaddress(rimeaddr_t* addr, tuAddr* tu_addr);

/* Subscribe with two callbacks called from FIFOP interrupt */
void micromac_radio_softack_subscribe(softack_make_callback_f softack_make, softack_interrupt_exit_callback_f interrupt_exit);

int micromac_radio_init(void);

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

void micromac_radio_sfd_sync(void);
rtimer_clock_t micromac_radio_read_sfd_timer(void);
int micromac_radio_get_delayed_transmit_status(void);
void micromac_radio_send_ack_delayed(uint32 u32_delay_time);
void micromac_radio_send_ack(void);
void micromac_radio_transmit_delayed(uint32 u32_delay_time);
void micromac_radio_start_rx_delayed(uint32 u32_delay_time, uint32 u32_on_duration);
int micromac_radio_read_ack(void *buf, int alen);
void micromac_radio_interrupt(uint32 mac_event);

#define NETSTACK_RADIO_tx_duration(X) 					RADIO_TO_RTIMER(X+1)
#define NETSTACK_RADIO_start_rx_delayed(u32_delay_time, u32_on_duration) micromac_radio_start_rx_delayed(u32_delay_time, u32_on_duration)
#define NETSTACK_RADIO_get_delayed_transmit_status()		micromac_radio_get_delayed_transmit_status()
#define NETSTACK_RADIO_softack_subscribe(A,E) 	micromac_radio_softack_subscribe(A,E)
#define NETSTACK_RADIO_get_rx_end_time() 				micromac_radio_get_rx_end_time()
#define NETSTACK_RADIO_send_ack() 							micromac_radio_send_ack()
#define NETSTACK_RADIO_send_ack_delayed(D) 			micromac_radio_send_ack_delayed(D)
#define NETSTACK_RADIO_read_ack(B,I)						micromac_radio_read_ack(B,I)
#define NETSTACK_RADIO_address_decode(E) 				do{ }while(0)
#define NETSTACK_RADIO_sfd_sync(S,E) 						micromac_radio_sfd_sync()
#define NETSTACK_RADIO_read_sfd_timer() 				micromac_radio_read_sfd_timer()
#define NETSTACK_RADIO_set_channel(C)						micromac_radio_set_channel(C)
#define NETSTACK_RADIO_process_packet(X)				micromac_radio_interrupt( X )
#define NETSTACK_RADIO_pending_irq()						u32MMAC_PollInterruptSource(E_MMAC_INT_RX_HEADER|E_MMAC_INT_RX_COMPLETE)
#define NETSTACK_RADIO_transmit_delayed(D) 			micromac_radio_transmit_delayed(D)
#define NETSTACK_RADIO_get_time() 							u32MMAC_GetTime()
#define NETSTACK_RADIO_get_radio_rx_end_time() 	micromac_radio_get_radio_rx_end_time()

#endif /* MICROMAC_RADIO_H_ */
