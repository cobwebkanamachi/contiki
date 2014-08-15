/*
 * Copyright (c) 2014, Swedish Institute of Computer Science.
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
 *         IEEE 802.15.4 TSCH MAC implementation. Must be used with nullmac as NETSTACK_CONF_MAC
 * \author
 *         Beshr Al Nahas <beshr@sics.se>
 */

#include "contiki.h"
#include "dev/leds.h"
#include "net/nbr-table.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/mac/tsch.h"
#include "net/mac/tsch-queue.h"
#include "net/mac/tsch-private.h"
#include "net/mac/tsch-packet.h"
#include "net/mac/frame802154.h"
#include "sys/process.h"
#include "sys/rtimer.h"
/* TODO: remove dependencies to RPL */
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include <string.h>

#define DEBUG DEBUG_NONE
#include "net/uip-debug.h"

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif /* MIN */

#ifdef TSCH_CONF_WITHOUT_CHANNEL_HOPPING
#define TSCH_WITHOUT_CHANNEL_HOPPING TSCH_CONF_WITHOUT_CHANNEL_HOPPING
#else
#define TSCH_WITHOUT_CHANNEL_HOPPING 0
#endif /* TSCH_CONF_WITHOUT_CHANNEL_HOPPING */

#ifdef TSCH_CONF_ADDRESS_FILTER
#define TSCH_ADDRESS_FILTER TSCH_CONF_ADDRESS_FILTER
#else
#define TSCH_ADDRESS_FILTER 0
#endif /* TSCH_CONF_ADDRESS_FILTER */

#ifndef TSCH_802154_DUPLICATE_DETECTION
#ifdef TSCH_CONF_802154_DUPLICATE_DETECTION
#define TSCH_802154_DUPLICATE_DETECTION TSCH_CONF_802154_DUPLICATE_DETECTION
#else
#define TSCH_802154_DUPLICATE_DETECTION 1
#endif /* TSCH_CONF_802154_AUTOACK */
#endif /* TSCH_802154_AUTOACK */

#if TSCH_802154_DUPLICATE_DETECTION
struct seqno {
  rimeaddr_t sender;
  uint8_t seqno;
};

#ifdef NETSTACK_CONF_MAC_SEQNO_HISTORY
#define MAX_SEQNOS NETSTACK_CONF_MAC_SEQNO_HISTORY
#else /* NETSTACK_CONF_MAC_SEQNO_HISTORY */
#define MAX_SEQNOS 8
#endif /* NETSTACK_CONF_MAC_SEQNO_HISTORY */

static struct seqno received_seqnos[MAX_SEQNOS];
#endif /* TSCH_802154_DUPLICATE_DETECTION */

/* TODO: const? */
/* Schedule: addresses */
static rimeaddr_t broadcast_cell_address = { { 0, 0, 0, 0, 0, 0, 0, 0 } };
static rimeaddr_t eb_cell_address = { { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff } };
static rimeaddr_t cell_address1 = { { 0x00, 0x12, 0x74, 01, 00, 01, 01, 01 } };
static rimeaddr_t cell_address2 = { { 0x00, 0x12, 0x74, 02, 00, 02, 02, 02 } };
static rimeaddr_t cell_address3 = { { 0x00, 0x12, 0x74, 03, 00, 03, 03, 03 } };

/* Schedule: cells */
static const cell_t generic_shared_cell = { 0xffff, 0,
                                            LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED,
                                            LINK_TYPE_NORMAL, &broadcast_cell_address };
static const cell_t generic_eb_cell = { 0, 0,
                                        LINK_OPTION_TX,
                                        LINK_TYPE_ADVERTISING, &eb_cell_address };
static const cell_t cell_to_1 = { 1, 0,
                                  LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING,
                                  LINK_TYPE_NORMAL, &cell_address1 };
static const cell_t cell_to_2 = { 2, 0,
                                  LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED,
                                  LINK_TYPE_NORMAL, &cell_address2 };
static const cell_t cell_to_3 = { 3, 0,
                                  LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED,
                                  LINK_TYPE_NORMAL, &cell_address3 };
static const cell_t cell_3_to_2 = { 4, 0,
                                    LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED,
                                    LINK_TYPE_NORMAL, &cell_address2 };

/* Static schedule definition */
static const cell_t *minimum_cells[6] = {
  &generic_eb_cell, &generic_shared_cell, &generic_shared_cell,
  &generic_shared_cell, &generic_shared_cell, &generic_shared_cell
};
static const cell_t *links_list[] = { &generic_eb_cell, &generic_shared_cell,
                                      &cell_to_1, &cell_to_2, &cell_to_3, &cell_3_to_2 };
static slotframe_t minimum_slotframe = { 0, 101, 6, (cell_t **)minimum_cells };
#define TOTAL_LINKS (sizeof(links_list) / sizeof(cell_t *))

/* Other function prototypes */
static int powercycle(struct rtimer *t, void *ptr);
void tsch_make_sync_ack(uint8_t **buf, uint8_t seqno, rtimer_clock_t last_packet_timestamp, uint8_t nack);
static void tsch_init(void);
static void tsch_resynchronize(struct rtimer *, void *);
static void tsch_init_variables(void);

/* A global variable telling whether we are coordinator of the TSCH network
 * TODO: have a function to set this */
int tsch_is_coordinator = 0;

volatile ieee154e_vars_t ieee154e_vars;

enum tsch_state_e {
  TSCH_OFF, TSCH_ASSOCIATED, TSCH_SEARCHING, TSCH_TIMEOUT,
};

enum tsch_state_e tsch_state;

/* Debug timing */
static rtimer_clock_t t0prepare = 0, t0tx = 0, t0txack = 0, t0post_tx = 0, t0rx = 0, t0rxack = 0, tq = 0, tn = 0;

/* Contiki processes */
PROCESS(tsch_tx_callback_process, "tsch_tx_callback_process");
PROCESS(tsch_associate, "tsch_associate");
PROCESS(tsch_process, "TSCH process");

/**
 *  A pseudo-random generator with better properties than msp430-libc's default
 **/

static uint32_t tsch_random_seed;

static void
tsch_random_init(uint32_t x)
{
  tsch_random_seed = x;
}
static uint8_t
tsch_random_byte(uint8_t window)
{
  tsch_random_seed = tsch_random_seed * 1103515245 + 12345;
  return ((uint32_t)(tsch_random_seed / 65536) % 32768) & window;
}
/*---------------------------------------------------------------------------*/
static void
on(void)
{
  NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
  NETSTACK_RADIO.off();
}
/*---------------------------------------------------------------------------*/
static void
turn_on(void)
{
  PRINTF("TSCH: turn_on not supported\n");
}
/*---------------------------------------------------------------------------*/
static void
turn_off(int keep_radio_on)
{
  PRINTF("TSCH: turn_off not supported\n");
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  /* TSCH does not support send_list. Must be used with nullmac, not csma. */
  PRINTF("TSCH: send_list not supported\n");
  mac_call_sent_callback(sent, ptr, MAC_TX_ERR_FATAL, 1);
  return;
}
/*
 * Timing
 */

/* Checks if the current time has past a ref time + duration. Assumes
 * a single overflow and ref time prior to now. */
static uint8_t
check_timer_miss(rtimer_clock_t ref_time, rtimer_clock_t duration, rtimer_clock_t now)
{
  rtimer_clock_t target = ref_time + duration;
  int now_has_overflowed = now < ref_time;
  int target_has_overflowed = target < ref_time;

  if(now_has_overflowed == target_has_overflowed) {
    /* Both or none have overflowed, just compare now to the target */
    return target <= now;
  } else {
    /* Either now or target of overflowed.
     * If it is now, then it has passed the target.
     * If it is target, then we haven't reached it yet.
     *  */
    return now_has_overflowed;
  }
}
/* Wait for a condition with timeout t0+duration. */
#define BUSYWAIT_UNTIL_ABS(cond, t0, duration) \
  do { \
    rtimer_clock_t now = RTIMER_NOW(); \
    if(!check_timer_miss((t0), (duration), now)) { \
      while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), (t0) + (duration))) ; \
    } \
  } while(0)

/*
 * Channel hopping
 */

/* Return channel from ASN and channel offset */
static uint8_t
calculate_channel(uint8_t offset)
{
#if TSCH_WITHOUT_CHANNEL_HOPPING
  return RF_CHANNEL;
#else
  /* TODO: compute this with 5-byte ASN */
  return 11 + (offset + ieee154e_vars.asn.asn_4lsb) % 16;
#endif /* TSCH_WITHOUT_CHANNEL_HOPPING */
}
/* Select the current channel from ASN and channel offset, hop to it */
static uint8_t
hop_channel(uint8_t offset)
{
  return NETSTACK_RADIO_set_channel(calculate_channel(offset));
}
/*
 * Schedule
 */

/* Return the cell for a given timeslot */
static cell_t *
get_cell(uint16_t timeslot)
{
  return (timeslot >= ieee154e_vars.current_slotframe->on_size) ?
         NULL : ieee154e_vars.current_slotframe->cells[timeslot];
}
/* Return the first active (not OFF) timeslot */
static uint16_t
get_first_active_timeslot()
{
  return 0;
}
/* Return the next active (not OFF) timeslot after a given timeslot */
static uint16_t
get_next_active_timeslot(uint16_t timeslot)
{
  return (timeslot >= ieee154e_vars.current_slotframe->on_size - 1) ? 0 : timeslot + 1;
}
/*---------------------------------------------------------------------------*/
/* Function send for TSCH-MAC, puts the packet in packetbuf in the MAC queue */
static int
send_packet(mac_callback_t sent, void *ptr)
{
  PRINTF("TSCH send_packet\n");
  int ret = MAC_TX_DEFERRED;
  uint16_t seqno;
  struct tsch_neighbor *n;

  const rimeaddr_t *addr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  /* Ask for ACK if we are sending anything other than broadcast */
  if(!rimeaddr_cmp(addr, &rimeaddr_null)) {
    packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
    /* PACKETBUF_ATTR_MAC_SEQNO cannot be zero, due to a peculiarity
       in framer-802154.c. */
  }
  seqno = (++ieee154e_vars.dsn) ? ieee154e_vars.dsn : ++ieee154e_vars.dsn;
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, seqno);
  if(NETSTACK_FRAMER.create() < 0) {
    PRINTF("tsch: can't send packet due to framer error\n");
    ret = MAC_TX_ERR;
  } else {
    /* Look for the neighbor entry */
    n = tsch_queue_get_nbr(addr);
    if(n == NULL) {
      /* add new neighbor to list of neighbors */
      if(!tsch_queue_add_nbr(addr)) {
        PRINTF("tsch: can't send packet !tsch_queue_add_nbr(addr)\n");
        ret = MAC_TX_ERR;
      } else if(!tsch_queue_add_packet(addr, sent, ptr)) {  /* add new packet to neighbor list */
        PRINTF("tsch: can't send packet !tsch_queue_add_packet 1\n");
        ret = MAC_TX_ERR;
      }
    } else
    /* add new packet to neighbor list */
    if(!tsch_queue_add_packet(addr, sent, ptr)) {
      PRINTF("tsch: can't send packet !tsch_queue_add_packet 2\n");
      ret = MAC_TX_ERR;
    }
  }
  if(ret != MAC_TX_DEFERRED) {
    mac_call_sent_callback(sent, ptr, ret, 1);
  }
  return ret;
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
  PRINTF("tsch packet_input begin\n");

#ifdef NETSTACK_DECRYPT
  NETSTACK_DECRYPT();
#endif /* NETSTACK_DECRYPT */

  if(NETSTACK_FRAMER.parse() < 0) {
    PRINTF("tsch: failed to parse %u\n", packetbuf_datalen());
#if TSCH_ADDRESS_FILTER
  } else if(!rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                          &rimeaddr_node_addr)
            && !rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                             &rimeaddr_null)) {
    PRINTF("tsch: not for us\n");
#endif /* TSCH_ADDRESS_FILTER */
  } else {
    int duplicate = 0;

#if TSCH_802154_DUPLICATE_DETECTION
    /* Check for duplicate packet by comparing the sequence number
       of the incoming packet with the last few ones we saw. */
    int i;
    for(i = 0; i < MAX_SEQNOS; ++i) {
      if(packetbuf_attr(PACKETBUF_ATTR_PACKET_ID) == received_seqnos[i].seqno &&
         rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_SENDER),
                      &received_seqnos[i].sender)) {
        /* Drop the packet. */
        COOJA_DEBUG_STR("tsch: drop duplicate link layer packet");
        PRINTF("tsch: drop duplicate link layer packet %u\n",
               packetbuf_attr(PACKETBUF_ATTR_PACKET_ID));
        duplicate = 1;
      }
    }
    if(!duplicate) {
      for(i = MAX_SEQNOS - 1; i > 0; --i) {
        memcpy(&received_seqnos[i], &received_seqnos[i - 1],
               sizeof(struct seqno));
      }
      received_seqnos[0].seqno = packetbuf_attr(PACKETBUF_ATTR_PACKET_ID);
      rimeaddr_copy(&received_seqnos[0].sender,
                    packetbuf_addr(PACKETBUF_ADDR_SENDER));
    }
#endif /* TSCH_802154_DUPLICATE_DETECTION */

    if(!duplicate) {
      NETSTACK_MAC.input();
      PRINTF("tsch packet_input, Not duplicate\n");
    }
  }
  /*	COOJA_DEBUG_STR("tsch packet_input end\n"); */
}
/* Schedule a wakeup from a reference time for a specific duration.
 * Provides basic protection against missed deadlines and timer overflows
 * A non-zero return value signals to powercycle a missed deadline */
static uint8_t
schedule_fixed(struct rtimer *tm, rtimer_clock_t ref_time,
               rtimer_clock_t duration)
{
  int r, status = 0;
  rtimer_clock_t now = RTIMER_NOW();

  /*	RTIMER_CLOCK_LT(ref_time - now > duration+5) */
  /*	!RTIMER_CLOCK_LT(now,ref_time) */
  if(check_timer_miss(ref_time, duration, now)) {
    COOJA_DEBUG_STR("schedule_fixed: missed deadline");
    /*		PRINTF("schedule_fixed: missed deadline: %u, %u, %u, %u\n", ref_time, now, now - ref_time, duration ); */
    ref_time = now + TRIVIAL_DELAY;
  } else {
    ref_time += duration;
  } r = rtimer_set(tm, ref_time, 1, (void
                                     (*)(struct rtimer *, void *))powercycle, NULL /*(void*)&status*/);
  if(r != RTIMER_OK) {
    COOJA_DEBUG_STR("schedule_fixed: could not set rtimer\n");
    status = 2;
  }
  return status;
}
/* Schedule only if deadline not passed.
 * A non-zero return value signals to powercycle a missed deadline */
static uint8_t
schedule_strict(struct rtimer *tm, rtimer_clock_t ref_time,
                rtimer_clock_t duration)
{
  int r, status = 0;
  rtimer_clock_t now = RTIMER_NOW();

  if(check_timer_miss(ref_time, duration, now)) {
    COOJA_DEBUG_STR("schedule_strict: missed deadline");
    extern rtimer_clock_t start_of_powercycle;
    PRINTF("schedule_strict: missed deadline: %u, %u, %u (%u)\n", ref_time, duration, now, start_of_powercycle);
    PRINTF("tq: %u, t0prepare %u, t0tx %u, t0txack %u, t0post_tx %u, t0rx %u, t0rxack %u, tn %u\n", tq, t0prepare, t0tx, t0txack, t0post_tx, t0rx, t0rxack, tn);
    PUTCHAR('!');
    status = 1;
  } else {
    r = rtimer_set(tm, ref_time + duration, 1, (void
                                                (*)(struct rtimer *, void *))powercycle, NULL /*(void*)&status*/);
    if(r != RTIMER_OK) {
      COOJA_DEBUG_STR("schedule_strict: could not set rtimer\n");
      status = 2;
    }
    PRINTF("schedule_strict: scheduled: %u, %u, %u (%u)\n", ref_time, duration, now, ref_time + duration);
  }

  /* COOJA_DEBUG_PRINTF("tq: %u, t0prepare %u, t0tx %u, t0txack %u, t0post_tx %u, t0rx %u, t0rxack %u, tn %u\n", tq, t0prepare, t0tx, t0txack, t0post_tx, t0rx, t0rxack, tn ); */
  tn = RTIMER_NOW();
  return status;
}
/*---------------------------------------------------------------------------*/
void
tsch_resume_powercycle(uint8_t need_ack_irq, struct received_frame_radio_s *last_rf_irq)
{
  ieee154e_vars.need_ack = need_ack_irq;
  ieee154e_vars.last_rf = last_rf_irq;
  leds_off(LEDS_RED);
}
rtimer_clock_t start_of_powercycle;
/*---------------------------------------------------------------------------*/
static int
powercycle(struct rtimer *t, void *ptr)
{
  /* if timeslot for tx, and we have a packet, call timeslot_tx
   * else if timeslot for rx, call timeslot_rx
   * otherwise, schedule next wakeup
   */
  COOJA_DEBUG_PRINTF("hi %u %u\n", ieee154e_vars.start, RTIMER_NOW());
  start_of_powercycle = RTIMER_NOW();
  PT_BEGIN(&ieee154e_vars.mpt);
  rtimer_clock_t duration;
  uint16_t dt, next_timeslot;
  uint16_t ack_status = 0;
  static uint8_t is_broadcast = 0, cca_status = 1, len = 0, seqno = 0, ret = 0;
  /* to record the duration of packet tx */
  static rtimer_clock_t tx_time;
  uint8_t success = 0, window = 0;
#if CONTIKI_TARGET_JN5168
  static uint32_t cycle_start_radio_clock = 0;
  uint32_t tx_offset = 0;
#endif /* CONTIKI_TARGET_JN5168 */

  while(tsch_state != TSCH_OFF) {
    ieee154e_vars.timeslot = 0;
    ieee154e_vars.drift_correction = 0;
    ieee154e_vars.drift = 0; /* estimated ieee154e_vars.drift to all time source neighbors */
    ieee154e_vars.drift_counter = 0; /* number of received drift corrections source neighbors */
    ieee154e_vars.cell_decison = 0;
    ieee154e_vars.cell = NULL;
    ieee154e_vars.p = NULL;
    ieee154e_vars.n = NULL;
    ieee154e_vars.payload = NULL;
    ieee154e_vars.payload_len = 0;
    is_broadcast = 0;
    len = 0;
    seqno = 0;
    ret = 0;

    PRINTF("Resync\n");
    /*		PT_YIELD_UNTIL(&ieee154e_vars.mpt, tsch_state == TSCH_ASSOCIATED); */

    while(tsch_state != TSCH_ASSOCIATED) {
      leds_on(LEDS_RED);
      PT_YIELD(&ieee154e_vars.mpt);
      leds_off(LEDS_RED);
    }
    ieee154e_vars.timeslot = ieee154e_vars.asn.asn_4lsb % ieee154e_vars.current_slotframe->length;
    if(ieee154e_vars.join_priority == 0) {
      ieee154e_vars.start = RTIMER_NOW();
      /* while MAC-RDC is not disabled, and while its synchronized */
    }
    while(tsch_state == TSCH_ASSOCIATED) {

      tq = RTIMER_NOW();
#if CONTIKI_TARGET_JN5168
      cycle_start_radio_clock = NETSTACK_RADIO_get_time();
#endif /* CONTIKI_TARGET_JN5168 */
      /* sync with cycle start and enable capturing start & end sfd*/
      NETSTACK_RADIO_sfd_sync(1, 1);
      leds_on(LEDS_RED);

      ieee154e_vars.cell = get_cell(ieee154e_vars.timeslot);

      if(ieee154e_vars.cell == NULL || tsch_queue_is_locked()) {
        COOJA_DEBUG_STR("Off CELL\n");
        /* off cell */
        off();
        ieee154e_vars.cell_decison = CELL_OFF;
      } else {
        hop_channel(ieee154e_vars.cell->channel_offset);
        ieee154e_vars.payload = NULL;
        ieee154e_vars.payload_len = 0;
        ieee154e_vars.p = NULL;
        ieee154e_vars.n = NULL;
        ieee154e_vars.registered_drift = 0;
        ieee154e_vars.last_rf = NULL;
        ieee154e_vars.need_ack = 0;
        /* is there a packet to send? if not check if this slot is RX too */
        if(ieee154e_vars.cell->link_options & LINK_OPTION_TX) {
          /* is it for ADV/EB? */
          if(ieee154e_vars.cell->link_type == LINK_TYPE_ADVERTISING) {
            /* TODO fetch adv/EB packets */
            ieee154e_vars.n = tsch_queue_get_nbr(&eb_cell_address);
            /* XXX limit EB rate */
            if(tsch_random_byte(8 - 1) >= 4) {
              ieee154e_vars.payload_len = make_eb((uint8_t *)ieee154e_vars.eb_buf, TSCH_MAX_PACKET_LEN + 1); /* last byte in eb buf is for length */
              ieee154e_vars.payload = (void *)ieee154e_vars.eb_buf;
            } else {
              COOJA_DEBUG_STR("Do not send EB");
            }
          } else { /* NORMAL link */
            /* pick a packet from the neighbors queue who is associated with this cell */
            ieee154e_vars.n = tsch_queue_get_nbr(ieee154e_vars.cell->node_address);
            if(ieee154e_vars.n != NULL) {
              ieee154e_vars.p = tsch_queue_get_packet_from_dest_addr(ieee154e_vars.cell->node_address);
              /* if there it is a shared broadcast slot and there were no broadcast packets, pick any unicast packet */
            }
            if(ieee154e_vars.p == NULL
               && rimeaddr_cmp(ieee154e_vars.cell->node_address, &broadcast_cell_address)
               && (ieee154e_vars.cell->link_options & LINK_OPTION_SHARED)) {
              ieee154e_vars.p = tsch_queue_get_any_packet(&ieee154e_vars.n);
            }
            if(ieee154e_vars.p != NULL) {
              if(ieee154e_vars.p->pkt != NULL) {
                ieee154e_vars.payload = queuebuf_dataptr(ieee154e_vars.p->pkt);
                ieee154e_vars.payload_len = queuebuf_datalen(ieee154e_vars.p->pkt);
              }
            }
          }
        }

        /* Decide whether it is a TX/RX/IDLE or OFF ieee154e_vars.cell */
        if(ieee154e_vars.cell->link_options & LINK_OPTION_TX) {
          if(ieee154e_vars.payload != NULL && ieee154e_vars.payload_len > 0) {
            /* if dedicated slot or shared slot and BW_value=0, we transmit the packet */
            if(!(ieee154e_vars.cell->link_options & LINK_OPTION_SHARED)
               || (ieee154e_vars.n != NULL && ieee154e_vars.n->BW_value == 0) || (ieee154e_vars.cell->link_type == LINK_TYPE_ADVERTISING)) {
              ieee154e_vars.cell_decison = CELL_TX;
              if(ieee154e_vars.cell->link_type == LINK_TYPE_ADVERTISING || ieee154e_vars.p == NULL) {
                is_broadcast = 1;
              } else if(ieee154e_vars.p != NULL) {
                is_broadcast = rimeaddr_cmp(queuebuf_addr(ieee154e_vars.p->pkt, PACKETBUF_ADDR_RECEIVER), &rimeaddr_null);
              }
            } else if(ieee154e_vars.n != NULL && ieee154e_vars.n->BW_value != 0) {
              /* packet to transmit but we cannot use shared slot due to backoff counter */
              ieee154e_vars.n->BW_value--;
              ieee154e_vars.cell_decison = CELL_TX_BACKOFF;
            }
          } else if(ieee154e_vars.cell->link_type == LINK_TYPE_ADVERTISING) {
            ieee154e_vars.cell_decison = CELL_RX;
          } else {
            ieee154e_vars.cell_decison = CELL_TX_IDLE;
          }
        }
        if((ieee154e_vars.cell->link_options & LINK_OPTION_RX) && ieee154e_vars.cell_decison != CELL_TX) {
          ieee154e_vars.cell_decison = CELL_RX;
        }
        tq = RTIMER_NOW() - tq;

#if 0 && DEBUG
        switch(ieee154e_vars.cell_decison) {
        case CELL_TX:
          if(ieee154e_vars.cell->link_type == LINK_TYPE_ADVERTISING) {
            PUTCHAR('E');
          } else if(is_broadcast) {
            PUTCHAR('B');
          } else {
            PUTCHAR('T');
          } break;
        case CELL_TX_BACKOFF:
          PUTCHAR('F');
          break;
        case CELL_TX_IDLE:
          PUTCHAR('I');
          break;
        case CELL_RX:
          PUTCHAR('R');
          break;
        case CELL_OFF:
        default:
          PUTCHAR('O');
          break;
        }
#endif /* DEBUG */

        /* execute the cell */
        if(ieee154e_vars.cell_decison != CELL_TX && ieee154e_vars.cell_decison != CELL_RX) {
          /* Nothing to do --> Off cell */
          COOJA_DEBUG_STR("Nothing to TX or RX --> off CELL\n");
          off();
        } else if(ieee154e_vars.cell_decison == CELL_TX) {
          /**
           * TX cell:
           * 1. Copy packet to radio buffer
           * 2. Perform CCA if enabled
           * 3. Sleep until it is time to transmit
           * 4. Wait for ACK if it is a unicast packet
           * 5. Extract drift if we received an E-ACK from a time source neighbor
           * 6. Update CSMA parameters according to TX status
           * 7. Schedule mac_call_sent_callback
           **/
          /* SEND_METHOD: */
          t0prepare = RTIMER_NOW();
          COOJA_DEBUG_STR("CELL_TX");
          /* TODO There are small timing variations visible in cooja, which needs tuning */

          /* queuebuf bug, trying to debug */
          if(ieee154e_vars.payload == 2) {
            PRINTF("pkt@%x", ieee154e_vars.p->pkt);
          }
          /* read seqno from payload! */
          seqno = ((uint8_t *)(ieee154e_vars.payload))[2];
          /* prepare packet to send */
          success = !NETSTACK_RADIO.prepare(ieee154e_vars.payload, ieee154e_vars.payload_len);
          t0prepare = RTIMER_NOW() - t0prepare;
          cca_status = 1;
#if CCA_ENABLED
          /* delay before CCA */
          schedule_fixed(t, ieee154e_vars.start, TsCCAOffset);
          PT_YIELD(&ieee154e_vars.mpt);
          on();
          /* CCA */
          BUSYWAIT_UNTIL_ABS(!(cca_status |= NETSTACK_RADIO.channel_clear()),
                             ieee154e_vars.start, TsCCAOffset + TsCCA);
          /* there is not enough time to turn radio off */
          off();
#endif /* CCA_ENABLED */
          if(cca_status == 0 || !success) {
            success = RADIO_TX_COLLISION;
          } else {
            /* do not capture start SFD, we are only interested in SFD end which equals TX end time */
            /* I do it in transmit function now */
            /* delay before TX */
#if ENABLE_DELAYED_RF && CONTIKI_TARGET_JN5168
            t0tx = RTIMER_NOW();
            /* XXX fix potential wrap */
            /*						tx_offset = RTIMER_TO_RADIO((rtimer_clock_t)(TsTxOffset + ieee154e_vars.start - RTIMER_NOW())); */
            tx_offset = RTIMER_TO_RADIO((rtimer_clock_t)(TsTxOffset - tq - t0prepare));
            NETSTACK_RADIO_transmit_delayed(tx_offset - 1);
            tx_time = NETSTACK_RADIO_tx_duration(ieee154e_vars.payload_len + 1);
            schedule_fixed(t, ieee154e_vars.start, TsTxOffset + tx_time + delayTx);
            PT_YIELD(&ieee154e_vars.mpt);
            success = NETSTACK_RADIO_get_delayed_transmit_status();
#else /* Leave CC2420 as default */
            schedule_fixed(t, ieee154e_vars.start, TsTxOffset - delayTx);
            PT_YIELD(&ieee154e_vars.mpt);
            t0tx = RTIMER_NOW();
            tx_time = RTIMER_NOW();
            /* send packet already in radio tx buffer */
            success = NETSTACK_RADIO.transmit(ieee154e_vars.payload_len);
            /* tx_time = NETSTACK_RADIO_read_sfd_timer() - tx_time; */
            /* XXX check */
            tx_time = RADIO_TO_RTIMER((ieee154e_vars.payload_len + 1) * 2);
            /* limit tx_time in case of something wrong */
            tx_time = MIN(tx_time, wdDataDuration);
            off();
#endif /* CONTIKI_TARGET */
            t0tx = RTIMER_NOW() - t0tx;
            t0txack = RTIMER_NOW();
            if(success == RADIO_TX_OK) {
              if(!is_broadcast) {
                /* wait for ack: after tx */
                COOJA_DEBUG_STR("wait for ACK\n");
#if ENABLE_DELAYED_RF && CONTIKI_TARGET_JN5168
                tx_offset = RTIMER_TO_RADIO(TsTxOffset + tx_time + TsTxAckDelay - TsShortGT - (RTIMER_NOW() - ieee154e_vars.start)); /* - (uint32_t)(NETSTACK_RADIO_get_time() - cycle_start_radio_clock); */
                NETSTACK_RADIO_start_rx_delayed(tx_offset, RTIMER_TO_RADIO(TsShortGT * 2 + wdAckDuration));
                schedule_fixed(t, ieee154e_vars.start,
                               TsTxOffset + tx_time + TsTxAckDelay + wdAckDuration + TsShortGT);
                PT_YIELD(&ieee154e_vars.mpt);
#else
                /* disable capturing sfd */
                schedule_fixed(t, ieee154e_vars.start,
                               TsTxOffset + tx_time + TsTxAckDelay - TsShortGT - delayRx);
                /*								COOJA_DEBUG_PRINTF("NOW: %u, start: %u, tx_time: %u, scheduled %u, payload_len %u RADIO_TO_RTIMER %u\n", RTIMER_NOW(), ieee154e_vars.start, tx_time, ieee154e_vars.start + TsTxOffset + tx_time + TsTxAckDelay - TsShortGT - delayRx); */

                /* Disabling address decoding so the radio accepts the enhanced ACK */
                NETSTACK_RADIO_address_decode(0);
                PT_YIELD(&ieee154e_vars.mpt);
                cca_status = 0;
                on();
                BUSYWAIT_UNTIL_ABS(NETSTACK_RADIO.receiving_packet(),
                                   ieee154e_vars.start, TsTxOffset + tx_time + TsTxAckDelay + TsShortGT - delayRx);
                BUSYWAIT_UNTIL_ABS(!NETSTACK_RADIO.receiving_packet(),
                                   ieee154e_vars.start, TsTxOffset + tx_time + TsTxAckDelay + TsShortGT + wdAckDuration);
                /* Enabling address decoding again so the radio filter data packets */
                NETSTACK_RADIO_address_decode(1);
#endif /* CONTIKI_TARGET_JN5168 */
                len = NETSTACK_RADIO_read_ack((void *)ieee154e_vars.ackbuf, STD_ACK_LEN + SYNC_IE_LEN);
                if(2 == ieee154e_vars.ackbuf[0] && len >= STD_ACK_LEN && seqno == ieee154e_vars.ackbuf[2]) {
                  success = RADIO_TX_OK;
                  ack_status = 0;
                  /* IE-list present? */
                  if(ieee154e_vars.ackbuf[1] & 2) {
                    if(len == STD_ACK_LEN + SYNC_IE_LEN) {
                      if(ieee154e_vars.ackbuf[3] == 0x02
                         && ieee154e_vars.ackbuf[4] == 0x1e) {
                        ack_status = ieee154e_vars.ackbuf[5];
                        ack_status |= ieee154e_vars.ackbuf[6] << 8;
                        /* If the originator was a time source neighbor, the receiver adjusts its own clock by incorporating the
                         *  difference into an average of the drift to all its time source neighbors. The averaging method is
                         *  implementation dependent. If the receiver is not a clock source, the time correction is ignored.
                         */
                        if(ieee154e_vars.n != NULL) {
                          if(ieee154e_vars.n->is_time_source) {
                            /* extract time correction */
                            int16_t d = 0;
                            /*is it a negative correction?*/
                            if(ack_status & 0x0800) {
                              d = -(ack_status & 0x0fff & ~0x0800);
                            } else {
                              d = ack_status & 0x0fff;
                            } ieee154e_vars.drift += d;
                            ieee154e_vars.drift_counter++;
                            PUTCHAR('+');
                          }
                        }
                        if(ack_status & NACK_FLAG) {
                          /* TODO return NACK status to upper layer */
                          PUTCHAR('/');
                          COOJA_DEBUG_STR("ACK NACK_FLAG\n");
                        }
                      }
                    }
                  }
                  PUTCHAR('*');
                  COOJA_DEBUG_STR("ACK ok\n");
                } else {
                  success = RADIO_TX_NOACK;
                  PUTCHAR('@');
                  COOJA_DEBUG_STR("ACK not ok!\n");
                }
              }
              off();
              COOJA_DEBUG_STR("end tx slot\n");
            }
          }
          t0txack = RTIMER_NOW() - t0txack;
          t0post_tx = RTIMER_NOW();
          /* post TX: Update CSMA control variables */
          /* if it was EB then p is null and anyway there is no need to update anything */
          if(ieee154e_vars.p != NULL && ieee154e_vars.n != NULL) {
            if(success == RADIO_TX_NOACK) {
              ieee154e_vars.p->transmissions++;
              if(ieee154e_vars.p->transmissions >= MAC_MAC_FRAME_RETRIES) {
                tsch_queue_remove_packet_from_dest_addr(queuebuf_addr(ieee154e_vars.p->pkt, PACKETBUF_ADDR_RECEIVER));
                ieee154e_vars.n->BE_value = MAC_MIN_BE;
                ieee154e_vars.n->BW_value = 0;
              } else if((ieee154e_vars.cell->link_options & LINK_OPTION_SHARED) && !is_broadcast) {
                window = 1 << ieee154e_vars.n->BE_value;
                ieee154e_vars.n->BW_value = tsch_random_byte(window - 1);
                ieee154e_vars.n->BE_value++;
                if(ieee154e_vars.n->BE_value > MAC_MAC_BE) {
                  ieee154e_vars.n->BE_value = MAC_MAC_BE;
                }
              }
              ret = MAC_TX_NOACK;
            } else if(success == RADIO_TX_OK) {
              tsch_queue_remove_packet_from_dest_addr(queuebuf_addr(ieee154e_vars.p->pkt, PACKETBUF_ADDR_RECEIVER));
              if(!tsch_queue_get_packet_from_dest_addr(ieee154e_vars.cell->node_address)) {
                /* if no more packets in the queue */
                ieee154e_vars.n->BW_value = 0;
                ieee154e_vars.n->BE_value = MAC_MIN_BE;
              } else {
                /* if queue is not empty */
                ieee154e_vars.n->BW_value = 0;
              } ret = MAC_TX_OK;
            } else if(success == RADIO_TX_COLLISION) {
              ieee154e_vars.p->transmissions++;
              if(ieee154e_vars.p->transmissions >= MAC_MAC_FRAME_RETRIES) {
                tsch_queue_remove_packet_from_dest_addr(queuebuf_addr(ieee154e_vars.p->pkt, PACKETBUF_ADDR_RECEIVER));
                ieee154e_vars.n->BE_value = MAC_MIN_BE;
                ieee154e_vars.n->BW_value = 0;
              } else if((ieee154e_vars.cell->link_options & LINK_OPTION_SHARED) && !is_broadcast) {
                window = 1 << ieee154e_vars.n->BE_value;
                ieee154e_vars.n->BW_value = tsch_random_byte(window - 1);
                ieee154e_vars.n->BE_value++;
                if(ieee154e_vars.n->BE_value > MAC_MAC_BE) {
                  ieee154e_vars.n->BE_value = MAC_MAC_BE;
                }
              }
              ret = MAC_TX_COLLISION;
            } else if(success == RADIO_TX_ERR) {
              ieee154e_vars.p->transmissions++;
              if(ieee154e_vars.p->transmissions >= MAC_MAC_FRAME_RETRIES) {
                tsch_queue_remove_packet_from_dest_addr(queuebuf_addr(ieee154e_vars.p->pkt, PACKETBUF_ADDR_RECEIVER));
                ieee154e_vars.n->BE_value = MAC_MIN_BE;
                ieee154e_vars.n->BW_value = 0;
              } else if((ieee154e_vars.cell->link_options & LINK_OPTION_SHARED) && !is_broadcast) {
                window = 1 << ieee154e_vars.n->BE_value;
                ieee154e_vars.n->BW_value = tsch_random_byte(window - 1);
                ieee154e_vars.n->BE_value++;
                if(ieee154e_vars.n->BE_value > MAC_MAC_BE) {
                  ieee154e_vars.n->BE_value = MAC_MAC_BE;
                }
              }
              ret = MAC_TX_ERR;
            } else {
              /* successful transmission */
              tsch_queue_remove_packet_from_dest_addr(queuebuf_addr(ieee154e_vars.p->pkt, PACKETBUF_ADDR_RECEIVER));
              if(!tsch_queue_get_packet_from_dest_addr(ieee154e_vars.cell->node_address)) {
                /* if no more packets in the queue */
                ieee154e_vars.n->BW_value = 0;
                ieee154e_vars.n->BE_value = MAC_MIN_BE;
              } else {
                /* if queue is not empty */
                ieee154e_vars.n->BW_value = 0;
              } ret = MAC_TX_OK;
            }
            /* poll MAC TX callback */
            ieee154e_vars.p->ret = ret;
            while(PROCESS_ERR_OK != process_post(&tsch_tx_callback_process, PROCESS_EVENT_POLL, ieee154e_vars.p)) {
              PRINTF("PROCESS_ERR_OK != process_post(&tsch_tx_callback_process\n");
            }
          }
          t0post_tx = RTIMER_NOW() - t0post_tx;
        } else if(ieee154e_vars.cell_decison == CELL_RX) {
          /**
           * RX cell:
           * 1. Check if it is used for TIME_KEEPING
           * 2. Sleep and wake up just before expected RX time (with a guard time: TsLongGT)
           * 3. Check for radio activity for the guard time: TsLongGT
           * 4. Prepare and send ACK if needed
           * 5. Drift calculated in the ACK callback registered with the radio driver. Use it if receiving from a time source neighbor.
           **/

          /* this cell is RX ieee154e_vars.cell. Check if it is used for keep alive as well*/
          if((ieee154e_vars.cell->link_options & LINK_OPTION_TIME_KEEPING) &&
             (ieee154e_vars.join_priority != 0
              && ieee154e_vars.sync_timeout > KEEPALIVE_TIMEOUT)) {
            /* TODO */
            /* Check if we need to send keep-alive request */
            /* TODO fetch adv/EB packets */
            /*						n = tsch_queue_get_nbr(&eb_cell_address); */
            /*						{ */
            /*							payload_len = make_eb(ieee154e_vars.eb_buf, TSCH_MAX_PACKET_LEN+1); //last byte in eb buf is for length */
            /*							payload = ieee154e_vars.eb_buf; */
            /*						} */
            /*						PUTCHAR('K'); */
            /*						//XXX send keep-alive */
            /*						goto SEND_METHOD; */
            /* prepare keep-alive msg to be sent later... */
          }
          {
            t0rx = RTIMER_NOW();
            cca_status = 0;
#if ENABLE_DELAYED_RF && CONTIKI_TARGET_JN5168
            /*						tx_offset = RTIMER_TO_RADIO((rtimer_clock_t)(ieee154e_vars.start+TsTxOffset-TsLongGT-RTIMER_NOW())); */
            tx_offset = RTIMER_TO_RADIO((rtimer_clock_t)(TsTxOffset - tq - t0prepare - TsLongGT));

            NETSTACK_RADIO_start_rx_delayed(tx_offset, RTIMER_TO_RADIO(TsLongGT * 2 + wdDataDuration) + 1);
            schedule_fixed(t, ieee154e_vars.start, TsTxOffset + TsLongGT + delayRx);
            PT_YIELD(&ieee154e_vars.mpt);
#else
            /* wait before RX */
            schedule_fixed(t, ieee154e_vars.start, TsTxOffset - TsLongGT - delayRx);
            COOJA_DEBUG_STR("schedule RX on guard time - TsLongGT");
            NETSTACK_RADIO_sfd_sync(1, 1);
            PT_YIELD(&ieee154e_vars.mpt);
            /* Start radio for at least guard time */
            on();
            /* Check if receiving within guard time */
            BUSYWAIT_UNTIL_ABS(NETSTACK_RADIO.receiving_packet(),
                               ieee154e_vars.start, TsTxOffset + TsLongGT);
#endif /* CONTIKI_TARGET_JN5168 */
            uint32_t irq_status = NETSTACK_RADIO_pending_irq();
            if(!NETSTACK_RADIO.receiving_packet() && !irq_status) {
              off();
              t0rx = RTIMER_NOW() - t0rx;
              /* no packets on air */
              ret = 0;
            } else {
#if CONTIKI_TARGET_JN5168
              if(!irq_status) {
                /* Check if receiving within guard time */
                /*								BUSYWAIT_UNTIL_ABS(!NETSTACK_RADIO.receiving_packet(), */
                /*										ieee154e_vars.start, TsTxOffset + TsLongGT+wdDataDuration); */
                BUSYWAIT_UNTIL_ABS((irq_status = NETSTACK_RADIO_pending_irq()),
                                   ieee154e_vars.start, TsTxOffset + TsLongGT + wdDataDuration + delayRx);
              }
#endif
              NETSTACK_RADIO_process_packet(irq_status);
              /*							PUTCHAR('K'); */
              off();
              t0rx = RTIMER_NOW() - t0rx;
              t0rxack = RTIMER_NOW();
              if(ieee154e_vars.last_rf != NULL) {
                /* wait until ack time */
                if(ieee154e_vars.need_ack) {
#if ENABLE_DELAYED_RF && CONTIKI_TARGET_JN5168
                  tx_offset = RTIMER_TO_RADIO(TsTxAckDelay);
                  NETSTACK_RADIO_send_ack_delayed(tx_offset);
#else
                  schedule_fixed(t, NETSTACK_RADIO_get_rx_end_time(), TsTxAckDelay - delayTx);
                  /*									schedule_fixed(t, ieee154e_vars.last_rf->sfd_timestamp, rx_duration + TsTxAckDelay - delayTx); */
                  PT_YIELD(&ieee154e_vars.mpt);
                  NETSTACK_RADIO_send_ack();
#endif
                }
                /* If the originator was a time source neighbor, the receiver adjusts its own clock by incorporating the
                 *  difference into an average of the drift to all its time source neighbors. The averaging method is
                 *  implementation dependent. If the receiver is not a clock source, the time correction is ignored.
                 */
                /* drift calculated in radio_interrupt */
                if(ieee154e_vars.registered_drift) {
                  COOJA_DEBUG_PRINTF("ieee154e_vars.drift seen %d\n", ieee154e_vars.registered_drift);
                  /* check the source address for potential time-source match */
                  ieee154e_vars.n = tsch_queue_get_nbr(&ieee154e_vars.last_rf->source_address);
                  if(ieee154e_vars.n != NULL && ieee154e_vars.n->is_time_source) {
                    /* should be the average of drifts to all time sources */
                    ieee154e_vars.drift_correction -= ieee154e_vars.registered_drift;
                    ++ieee154e_vars.drift_counter;
                    COOJA_DEBUG_STR("ieee154e_vars.drift recorded");
                  }
                }
                /* TODO return length instead? or status? or something? */
                ret = 1;
              }
              t0rxack = RTIMER_NOW() - t0rxack;
            }
          }
        }
      }

      tn = RTIMER_NOW();
      next_timeslot = get_next_active_timeslot(ieee154e_vars.timeslot);

      dt =
        next_timeslot ? next_timeslot - ieee154e_vars.timeslot :
        ieee154e_vars.current_slotframe->length - ieee154e_vars.timeslot;
      duration = dt * TsSlotDuration;

      /*			COOJA_DEBUG_PRINTF("ASN %lu TS %u NTS %u duration %lu", ieee154e_vars.asn.asn_4lsb, ieee154e_vars.timeslot, next_timeslot, duration); */
      /* increase the timeout counter because we will reset it in case of successful TX or RX */
      ieee154e_vars.sync_timeout += dt;
      ieee154e_vars.timeslot = next_timeslot;
      /* increase asn */
      ieee154e_vars.asn.asn_4lsb += dt;
      if(!ieee154e_vars.asn.asn_4lsb) {
        ieee154e_vars.asn.asn_msb++;
      }
      leds_off(LEDS_RED);

      /* apply drift correction */
      if(ieee154e_vars.sync_timeout > DRIFT_CORRECTION_TIMEOUT && ieee154e_vars.drift_counter) {
        /* convert from microseconds to rtimer ticks and take average */
        ieee154e_vars.drift_correction += CONVERT_DRIFT_US_TO_RTIMER(
            ieee154e_vars.drift, ieee154e_vars.drift_counter);
        duration += (int16_t)ieee154e_vars.drift_correction;
        if(ieee154e_vars.drift_correction) {
          COOJA_DEBUG_PRINTF("New slot frame: drift_correction %d", ieee154e_vars.drift_correction);
        } else {
          COOJA_DEBUG_STR("New slot frame");
        } ieee154e_vars.drift_correction = 0;
        ieee154e_vars.drift = 0;
        ieee154e_vars.drift_counter = 0;
        ieee154e_vars.sync_timeout = 0;
        PUTCHAR('D');
      }

      /* Check if we need to resynchronize */
      if(ieee154e_vars.join_priority != 0
         && ieee154e_vars.sync_timeout > DESYNC_THRESHOLD) {
        tsch_state = TSCH_SEARCHING;
        ieee154e_vars.timeslot = 0;
        /* schedule init function to run again */
        int r = RTIMER_OK, i = 1;
        do {
          r = rtimer_set(t, RTIMER_NOW() + (i++) * (RTIMER_SECOND / 8), 1, (void
                                                                            (*)(struct rtimer *, void *))tsch_resynchronize, NULL);
        } while(r != RTIMER_OK);
        tn = RTIMER_NOW() - tn;
        PUTCHAR('S');
      } else { /* Schedule wakeup for next slot */

        /* skip slot if missed the deadline */
        while(schedule_strict(t, ieee154e_vars.start, duration)) {
          /*					watchdog_periodic(); */
          ieee154e_vars.start += duration;
          next_timeslot = get_next_active_timeslot(ieee154e_vars.timeslot);
          dt =
            next_timeslot ? next_timeslot - ieee154e_vars.timeslot :
            ieee154e_vars.current_slotframe->length - ieee154e_vars.timeslot;
          ieee154e_vars.timeslot = next_timeslot;
          duration = dt * TsSlotDuration;
          /* increase asn */
          ieee154e_vars.asn.asn_4lsb += dt;
          if(!ieee154e_vars.asn.asn_4lsb) {
            ieee154e_vars.asn.asn_msb++;
            /*					PUTCHAR('!'); */
          }
        }
        /*				PUTCHAR('N'); */
        ieee154e_vars.start += duration;

        PT_YIELD(&ieee154e_vars.mpt);
      }
    }
  }
  COOJA_DEBUG_STR("TSCH is OFF!!");
  PT_END(&ieee154e_vars.mpt);
}
/*---------------------------------------------------------------------------*/
void
tsch_wait_for_eb(uint8_t need_ack_irq, struct received_frame_radio_s *last_rf_irq)
{
  uint16_t dt = 0;
  rtimer_clock_t duration = 0;
  volatile softack_interrupt_exit_callback_f interrupt_exit = tsch_wait_for_eb;
  volatile softack_make_callback_f softack_make = NULL;
  ieee154e_vars.need_ack = need_ack_irq;
  ieee154e_vars.last_rf = last_rf_irq;
  COOJA_DEBUG_STR("tsch_wait_eb");
  NETSTACK_RADIO_sfd_sync(1, 1);
  if(ieee154e_vars.last_rf != NULL /*&& (NETSTACK_RADIO_get_rx_end_time() != 0)*/) {
    /*		 int i; */
    /*		 for(i=0; i<ieee154e_vars.last_rf->len; i++) { */
    /*			 PRINTF("%x", ieee154e_vars.last_rf->buf[i]); */
    /*		 } */
    /*		 PRINTF("\n"); */
    if(ieee154e_vars.last_rf->len >= 23 && (FRAME802154_BEACONFRAME == ((ieee154e_vars.last_rf)->buf[0] & 7))
       && (((ieee154e_vars.last_rf)->buf[1] & (2 | 32 | 128 | 64)) == (2 | 32 | 128 | 64))) {
      if(((ieee154e_vars.last_rf)->buf[16] & 0xfe) == 0x34) {  /* sync IE? (0x1a << 1) ==0 0x34 */
        ieee154e_vars.asn.asn_4lsb = (uint32_t)(ieee154e_vars.last_rf)->buf[17];
        ieee154e_vars.asn.asn_4lsb += ((uint32_t)(ieee154e_vars.last_rf)->buf[18] << (uint32_t)8);
        ieee154e_vars.asn.asn_4lsb += ((uint32_t)(ieee154e_vars.last_rf)->buf[19] << (uint32_t)16);
        ieee154e_vars.asn.asn_4lsb += ((uint32_t)(ieee154e_vars.last_rf)->buf[20] << (uint32_t)24);
        ieee154e_vars.asn.asn_msb = (ieee154e_vars.last_rf)->buf[21];
        ieee154e_vars.join_priority = (ieee154e_vars.last_rf)->buf[22] + 1;
        /* XXX to disable reading the packet */
        (ieee154e_vars.last_rf)->len = 0;
        /* sync for next slot */
        ieee154e_vars.start = (ieee154e_vars.last_rf)->sfd_timestamp - TsTxOffset;
        /*				PRINTF("tsch_wait_for_eb: sfd_timestamp %u, start %u, now %u\n", ieee154e_vars.last_rf->sfd_timestamp, ieee154e_vars.start, RTIMER_NOW()); */
        /* we are in sync */
        tsch_state = TSCH_ASSOCIATED;

        /* XXX schedule to run after sometime */
        /* check if missed deadline, try a new one */
        if(ieee154e_vars.current_slotframe) {
          dt = 20 + tsch_random_byte(16 - 1);
        } else {
          dt = 50 + tsch_random_byte(32 - 1);
        } duration = 0;
        do {
          ieee154e_vars.start += duration;
          duration = dt * TsSlotDuration;
          /* increase asn */
          ieee154e_vars.asn.asn_4lsb += dt;
          if(!ieee154e_vars.asn.asn_4lsb) {
            ieee154e_vars.asn.asn_msb++;
          }
          dt = 15 + tsch_random_byte(16 - 1);
          PRINTF("Debutg timing: %u %u %u\n", ieee154e_vars.start, duration, RTIMER_NOW());
        } while(schedule_strict(&ieee154e_vars.t, ieee154e_vars.start, duration));
        ieee154e_vars.start += duration;
        PRINTF("Debutg timing final: %u %u %u\n", ieee154e_vars.start, duration, RTIMER_NOW());
      }
    }
  }
  if(tsch_state == TSCH_ASSOCIATED) {
    off();
    interrupt_exit = tsch_resume_powercycle;
    softack_make = tsch_make_sync_ack;
    NETSTACK_RADIO_softack_subscribe(softack_make, interrupt_exit);

    /* process the schedule, to create queues and find time-sources (time-keeping) */
    /* wait until queue is free */
    /*		while(tsch_queue_is_locked()){} */
    if(!tsch_queue_is_locked()) {
      struct tsch_neighbor *n;
      uint8_t i = 0;
      for(i = 0; i < TOTAL_LINKS; i++) {
        /* add queues for neighbors with tx links and for time-sources */
        if((links_list[i]->link_options & LINK_OPTION_TIME_KEEPING)
           || (links_list[i]->link_options & LINK_OPTION_TX)) {
          rimeaddr_t *addr = links_list[i]->node_address;
          /* Look for the neighbor entry */
          n = tsch_queue_get_nbr(addr);
          if(n == NULL) {
            /* add new neighbor to list of neighbors */
            n = tsch_queue_add_nbr(addr);
          }
          if(n != NULL) {
            n->is_time_source = (links_list[i]->link_options & LINK_OPTION_TIME_KEEPING) ? 1 : n->is_time_source;
          }
        }
      }
    } else {
      PRINTF("XXX Could not create queues on association. Time source will not be set!\n");
      /* XXX HACK this should be set in sent schedule
       * --Set parent as timesource */
    }
    if((ieee154e_vars.last_rf)) {
      struct tsch_neighbor *n = tsch_queue_get_nbr(&ieee154e_vars.last_rf->source_address);
      if(n == NULL && !tsch_queue_is_locked()) {
        /* add new neighbor to list of neighbors */
        n = tsch_queue_add_nbr(&ieee154e_vars.last_rf->source_address);
      }
      if(n != NULL) {
        n->is_time_source = 1;
        PRINTF("Setting parent as time source : %x.%x\n", ieee154e_vars.last_rf->source_address.u8[RIMEADDR_SIZE - 2], ieee154e_vars.last_rf->source_address.u8[RIMEADDR_SIZE - 1]);
      }
    }
  } else {
    NETSTACK_RADIO_radio_raw_rx_on();
  }
  leds_off(LEDS_RED);
}
/*---------------------------------------------------------------------------*/
static void
tsch_resynchronize(struct rtimer *tm, void *ptr)
{
  off();
  COOJA_DEBUG_STR("tsch_resynchronize\n");
  tsch_init_variables();
  NETSTACK_RADIO_softack_subscribe(NULL, tsch_wait_for_eb);
  /* try to associate to a network or start one if setup as RPL root */
  while(PROCESS_ERR_OK != process_post(&tsch_associate, PROCESS_EVENT_POLL, NULL)) {
    PRINTF("PROCESS_ERR_OK != process_post(&tsch_associate\n");
  }
}
/* Synchronize
 * If we are a master, start right away
 * otherwise, wait for EBs to associate with a master
 */
PROCESS_THREAD(tsch_associate, ev, data)
{
  PROCESS_BEGIN();

  /* Trying to get the RPL DAG, and polling until it becomes available  */
  static rpl_dag_t *my_rpl_dag = NULL;
  static struct etimer periodic;
#if CONTIKI_TARGET_JN5168
  uint32_t irq_status = 0;
  rtimer_clock_t t0;
#endif

  while(tsch_state != TSCH_OFF) {
    COOJA_DEBUG_STR("tsch_associate\n");

    my_rpl_dag = NULL;
    /* setup radio functions for intercepting EB */
    NETSTACK_RADIO_softack_subscribe(NULL, tsch_wait_for_eb);
    etimer_set(&periodic, CLOCK_SECOND / 10);

    while(tsch_state == TSCH_SEARCHING) {

      PROCESS_YIELD();
      etimer_set(&periodic, CLOCK_SECOND / 100);
      NETSTACK_RADIO_sfd_sync(1, 1);
      /*If I am root (rank==1), then exit association process*/
      rpl_instance_t *rpl = rpl_get_instance(RPL_DEFAULT_INSTANCE);
      if(rpl != NULL) {
        my_rpl_dag = rpl->current_dag;
        if(my_rpl_dag != NULL) {
          COOJA_DEBUG_PRINTF("rank %d", my_rpl_dag->rank);

          ieee154e_vars.join_priority = (my_rpl_dag->rank) / RPL_DAG_MC_ETX_DIVISOR;
          /* to make sure that this is a root and not just a low ranked node */
          if(!tsch_is_coordinator) {
            /* TODO choose something else? */
            if(ieee154e_vars.first_associate /* || my_rpl_dag->rank == 256 ??*/) {
              ieee154e_vars.join_priority = 0xf0;
            }
          }
        }
      }

      if(tsch_is_coordinator) {
        ieee154e_vars.join_priority = 0;
      }
      /* XXX there should be a better way of handling rpl, and erasing rank, dag etc. on resync */
      ieee154e_vars.first_associate = 1;

      /* if this is root start now */
      if(tsch_is_coordinator) {
        PRINTF("rpl root\n");
        /* something other than 0 for now */
        tsch_state = TSCH_ASSOCIATED;
        /* make queues and data structures */
        tsch_wait_for_eb(0, NULL);
        NETSTACK_RADIO_softack_subscribe(tsch_make_sync_ack, tsch_resume_powercycle);
        ieee154e_vars.start = RTIMER_NOW();

        schedule_fixed(&ieee154e_vars.t, ieee154e_vars.start, 2 * TRIVIAL_DELAY);
        /*				ieee154e_vars.start += 2*TRIVIAL_DELAY; */
        PRINTF("associate done\n");
        /* XXX for debugging */
        ieee154e_vars.asn.asn_4lsb = 0;
      } else {
#if CONTIKI_TARGET_JN5168
        NETSTACK_RADIO_radio_raw_rx_on();
        t0 = RTIMER_NOW();
        BUSYWAIT_UNTIL_ABS(irq_status = NETSTACK_RADIO_pending_irq(), t0, RTIMER_SECOND / 10);
        NETSTACK_RADIO_process_packet(irq_status);
#else
        on();
#endif /* CONTIKI_TARGET_JN5168 */
        /* TODO hop channel after timeout */
        /* ... set_channel... */
      }
    }
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    PRINTF("tsch_associate polled\n");
    NETSTACK_RADIO_softack_subscribe(NULL, tsch_wait_for_eb);
  }
  COOJA_DEBUG_STR("tsch_associate exit");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/* a polled-process to invoke the MAC tx callback asynchronously */
PROCESS_THREAD(tsch_tx_callback_process, ev, data)
{
  PROCESS_BEGIN();
  PRINTF("tsch_tx_callback_process: started\n");
  COOJA_DEBUG_STR("tsch_tx_callback_process: started\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    /*		PRINTF("tsch_tx_callback_process: calling mac tx callback\n"); */
    COOJA_DEBUG_STR("tsch_tx_callback_process: calling mac tx callback\n");
    if(data != NULL) {
      struct tsch_packet *p = (struct tsch_packet *)data;
      /* XXX callback -- do we need to restore the packet to packetbuf? */
      mac_call_sent_callback(p->sent, p->ptr, p->ret, p->transmissions);
    }
  }
  PROCESS_END();
}














/* Get EB, broadcast or unicast packet to be sent, and target neighbor.
 * TODO: Update EB fields.
 *  */
static struct tsch_packet *
get_packet_and_neighbor_for_cell(cell_t *cell, struct tsch_neighbor **target_neighbor)
{
/* TODO */
  *target_neighbor = NULL;
  return NULL;
}

static int associated = 0;
static struct pt cell_operation_pt;

static asn_t current_asn;
static rtimer_clock_t current_cell_start;
static uint16_t current_timeslot;
slotframe_t *current_slotframe;

static PT_THREAD(tsch_cell_operation(struct rtimer *t, void *ptr));

static
PT_THREAD(tsch_tx_cell(struct pt *pt, struct rtimer *t))
{
  PT_BEGIN(pt);

  rtimer_set(t, current_cell_start + TRIVIAL_DELAY,
              0, (rtimer_callback_t)tsch_cell_operation, NULL);
  //printf("TSCH: before yield\n");
  PT_YIELD(pt);
  //printf("TSCH: after yield\n");

  PT_END(pt);
}

static
PT_THREAD(tsch_rx_cell(struct pt *pt, struct rtimer *t))
{
  PT_BEGIN(pt);

  rtimer_set(t, current_cell_start + TRIVIAL_DELAY,
              0, (rtimer_callback_t)tsch_cell_operation, NULL);
  //printf("TSCH: before yield\n");
  PT_YIELD(pt);
  //printf("TSCH: after yield\n");

  PT_END(pt);
}

static
PT_THREAD(tsch_cell_operation(struct rtimer *t, void *ptr))
{
  static asn_t last_sync_asn;
  static int32_t drift_correction;

  PT_BEGIN(&cell_operation_pt);

  /* Loop over all active cells */
  while(associated) {
      static struct tsch_packet *current_packet;
      static struct tsch_neighbor *current_neighbor;
      static cell_t *current_cell;

      current_cell = get_cell(current_timeslot);

      if(current_cell->link_options & LINK_OPTION_SHARED) {
/* TODO       update_backoff(all); */
      }

      if(current_cell->link_options & LINK_OPTION_TX) {
        current_packet = get_packet_and_neighbor_for_cell(current_cell, &current_neighbor);
      }

      /* Actual slot operation */
      if(current_packet != NULL) {
        /* We have something to transmit */
        static struct pt cell_tx_pt;
        PT_SPAWN(&cell_operation_pt, &cell_tx_pt, tsch_tx_cell(&cell_tx_pt, t));
        /* TODO
                  update_backoff_state(current_neighbor)
                  post tx callback */
      } else if(current_cell->link_options & LINK_OPTION_RX) {
        /* Listen */
        static struct pt cell_rx_pt;
        PT_SPAWN(&cell_operation_pt, &cell_rx_pt, tsch_rx_cell(&cell_rx_pt, t));
      }

      /* End of slot operation, schedule next slot */
      if(!tsch_is_coordinator && ASN_DIFF(current_asn, last_sync_asn) > DESYNC_THRESHOLD) {
        associated = 0;
        process_post(&tsch_process, PROCESS_EVENT_POLL, NULL);
      } else {
        /* Get next active timeslot */
        uint16_t next_timeslot = get_next_active_timeslot(current_timeslot);
        /* Calculate number of slots between current and next */
        uint16_t timeslot_diff = next_timeslot > current_timeslot ? next_timeslot - current_timeslot:
            current_slotframe->length - (current_timeslot - next_timeslot);
        /* Update ASN */
        ASN_INC(current_asn, timeslot_diff);

        /* Update timeslot and cell start, schedule next wakeup */
        current_timeslot = next_timeslot;
        current_cell_start +=  timeslot_diff * TsSlotDuration + drift_correction;
        rtimer_set(t, current_cell_start,
            0, (rtimer_callback_t)tsch_cell_operation, NULL);
        printf("TSCH: end of cell %u\n", timeslot_diff);
      }

      PT_YIELD(&cell_operation_pt);
  }

  PT_END(&cell_operation_pt);
}

static
PT_THREAD(tsch_associate_dummy(struct pt *pt))
{
  static struct etimer a_timer;

  PT_BEGIN(pt);

  printf("TSCH: associate\n");
  if(tsch_is_coordinator) {
    associated = 1;
    current_slotframe = &minimum_slotframe;
    current_timeslot = get_first_active_timeslot();
    ASN_SET(current_asn, current_timeslot);
    current_cell_start = RTIMER_NOW() + TRIVIAL_DELAY;
    printf("TSCH: associate [done as coordinator]\n");
  } else {
    /* TODO */
    etimer_set(&a_timer, CLOCK_SECOND);
    PT_WAIT_UNTIL(pt, etimer_expired(&a_timer));
  }

  PT_END(pt);
}

/* The main TSCH process */
PROCESS_THREAD(tsch_process, ev, data)
{
  static struct rtimer cell_operation_timer;
  static struct pt associate_pt;

  PROCESS_BEGIN();

  while(1) {

    /* Associate */
    while(!associated) {
      PROCESS_PT_SPAWN(&associate_pt, tsch_associate_dummy(&associate_pt));
    }

    /* Operate */
    printf("TSCH: starting cell operation\n");
    rtimer_set(&cell_operation_timer, current_cell_start,
        0, (rtimer_callback_t)tsch_cell_operation, NULL);

    PROCESS_WAIT_UNTIL(!associated);
  }

  PROCESS_END();
}









/*---------------------------------------------------------------------------*/
static void
tsch_init_variables(void)
{
  /* setting seed for the random generator */
  tsch_random_init(clock_time() * RTIMER_NOW());
  NETSTACK_RADIO_softack_subscribe(NULL, NULL);
  /* look for a root to sync with */
  ieee154e_vars.current_slotframe = &minimum_slotframe;
  ieee154e_vars.slot_template_id = 1;
  ieee154e_vars.hop_sequence_id = 1;
  ieee154e_vars.asn.asn_4lsb = 0;
  ieee154e_vars.asn.asn_msb = 0;
  /* start with a random sequence number */
  ieee154e_vars.dsn = tsch_random_byte(127);
  tsch_state = TSCH_SEARCHING;
  /* we need to sync */
  ieee154e_vars.sync_timeout = 0; /* 30sec/slotDuration - (asn-asn0)*slotDuration */
  ieee154e_vars.mac_ebsn = 0;
  ieee154e_vars.join_priority = 0xff; /* inherit from RPL - PAN coordinator: 0 -- lower is better */
  ieee154e_vars.need_ack = 0;
  ieee154e_vars.last_rf = NULL;
  ieee154e_vars.registered_drift = 0;
  ieee154e_vars.timeslot = 0;
  NETSTACK_RADIO_sfd_sync(1, 1);
  NETSTACK_RADIO_softack_subscribe(NULL, tsch_wait_for_eb);
}
/*---------------------------------------------------------------------------*/
static void
tsch_init(void)
{
  extern const struct mac_driver nullmac_driver;
  if(&NETSTACK_CONF_MAC != &nullmac_driver) {
    PRINTF("TSCH: must be used with nullmac.\n");
    return;
  }

  leds_blink();
  tsch_init_variables();
  /* on resynchronization, the node has already joined a RPL network and it is mistaking it with root
   * this flag is used to prevent this */
  ieee154e_vars.first_associate = 0;
  tsch_queue_init();
//  process_start(&tsch_tx_callback_process, NULL);
  /* try to associate to a network or start one if setup as RPL root */
//  process_start(&tsch_associate, NULL);
  /* XXX for debugging */
  hop_channel(0);
  /*	NETSTACK_RADIO.on(); */
//  powercycle(&ieee154e_vars.t, NULL);
  /* schedule_fixed(&ieee154e_vars.t, RTIMER_NOW(), 0); */

  process_start(&tsch_process, NULL);
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver tschrdc_driver = {
  "TSCH",
  tsch_init,
  send_packet,
  send_list,
  packet_input,
  turn_on,
  turn_off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/
