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
 *         Simon Duquennoy <simonduq@sics.se>
 *
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
#include "net/mac/tsch-schedule.h"
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

/* 802.15.4 broadcast MAC address */
const rimeaddr_t tsch_broadcast_address = { { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff } };
const rimeaddr_t tsch_eb_address = { { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff } };

/* TODO: const? */
/* Schedule: addresses */
static rimeaddr_t cell_address1 = { { 0x00, 0x12, 0x74, 01, 00, 01, 01, 01 } };
static rimeaddr_t cell_address2 = { { 0x00, 0x12, 0x74, 02, 00, 02, 02, 02 } };
static rimeaddr_t cell_address3 = { { 0x00, 0x12, 0x74, 03, 00, 03, 03, 03 } };

/* Schedule: links */
static const struct tsch_link generic_shared_cell = { 0xffff, 0,
                                            LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED,
                                            LINK_TYPE_NORMAL, &tsch_broadcast_address };
static const struct tsch_link generic_eb_cell = { 0, 0,
                                        LINK_OPTION_TX,
                                        LINK_TYPE_ADVERTISING, &tsch_eb_address };
static const struct tsch_link cell_to_1 = { 1, 0,
                                  LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING,
                                  LINK_TYPE_NORMAL, &cell_address1 };
static const struct tsch_link cell_to_2 = { 2, 0,
                                  LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED,
                                  LINK_TYPE_NORMAL, &cell_address2 };
static const struct tsch_link cell_to_3 = { 3, 0,
                                  LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED,
                                  LINK_TYPE_NORMAL, &cell_address3 };
static const struct tsch_link cell_3_to_2 = { 4, 0,
                                    LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED,
                                    LINK_TYPE_NORMAL, &cell_address2 };

/* Static schedule definition */
static const struct tsch_link *minimum_links[6] = {
  &generic_eb_cell, &generic_shared_cell, &generic_shared_cell,
  &generic_shared_cell, &generic_shared_cell, &generic_shared_cell
};
static const struct tsch_link *links_list[] = { &generic_eb_cell, &generic_shared_cell,
                                      &cell_to_1, &cell_to_2, &cell_to_3, &cell_3_to_2 };
static struct slotframe minimum_slotframe = { 0, 101, 6, (struct tsch_link **)minimum_links };
#define TOTAL_LINKS (sizeof(links_list) / sizeof(struct tsch_link *))

/* Other function prototypes */
static int powercycle(struct rtimer *t, void *ptr);
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
PROCESS(tsch_tx_callback_process, "tsch_tx");
PROCESS(tsch_rx_callback_process, "tsch_rx");
PROCESS(tsch_send_eb_process, "tsch_send_eb");
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
  return 11 + (offset + ieee154e_vars.asn) % 16;
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
static struct tsch_link *
get_cell(uint16_t timeslot)
{
  return (timeslot >= ieee154e_vars.current_slotframe->on_size) ?
         NULL : ieee154e_vars.current_slotframe->links[timeslot];
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
schedule_operation(struct rtimer *tm, rtimer_callback_t operation, rtimer_clock_t ref_time,
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
	}
  r = rtimer_set(tm, ref_time, 1, (void(*)(struct rtimer *, void *))operation, NULL /*(void*)&status*/);
	if(r != RTIMER_OK) {
		COOJA_DEBUG_STR("schedule_fixed: could not set rtimer\n");
		status = 2;
	}
	return status;
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
  }
//  r = rtimer_set(tm, ref_time, 1, (void
//                                     (*)(struct rtimer *, void *))powercycle, NULL /*(void*)&status*/);
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
//    r = rtimer_set(tm, ref_time + duration, 1, (void
//                                                (*)(struct rtimer *, void *))powercycle, NULL /*(void*)&status*/);
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
/*---------------------------------------------------------------------------*/
static void
tsch_resynchronize(struct rtimer *tm, void *ptr)
{
  off();
  COOJA_DEBUG_STR("tsch_resynchronize\n");
  tsch_init_variables();
//  NETSTACK_RADIO_softack_subscribe(NULL, NULL);
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
//    NETSTACK_RADIO_softack_subscribe(NULL, NULL);
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
//        tsch_wait_for_eb(0, NULL);
//        NETSTACK_RADIO_softack_subscribe(NULL/*tsch_packet_make_sync_ack*/, tsch_resume_powercycle);
        ieee154e_vars.start = RTIMER_NOW();

        schedule_fixed(&ieee154e_vars.t, ieee154e_vars.start, 2 * TRIVIAL_DELAY);
        /*				ieee154e_vars.start += 2*TRIVIAL_DELAY; */
        PRINTF("associate done\n");
        /* XXX for debugging */
        ieee154e_vars.asn = 0;
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
//    NETSTACK_RADIO_softack_subscribe(NULL, tsch_wait_for_eb);
  }
  COOJA_DEBUG_STR("tsch_associate exit");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/















/* Get EB, broadcast or unicast packet to be sent, and target neighbor.
 * TODO: Update EB fields.
 *  */
static struct tsch_packet *
get_packet_and_neighbor_for_cell(struct tsch_link *link, struct tsch_neighbor **target_neighbor)
{
	struct tsch_packet *p = NULL;
	struct tsch_neighbor *n = NULL;

  /* is there a packet to send? if not check if this slot is RX too */
  if(link->link_options & LINK_OPTION_TX) {
    /* is it for ADV/EB? */
    if(link->link_type == LINK_TYPE_ADVERTISING) {
      /* fetch adv/EB packets */
      n = tsch_queue_get_nbr(&tsch_eb_address);
      p = tsch_queue_get_packet_for_dest_addr(&tsch_eb_address, 0);
    }
    /* NORMAL link or no EB to send, pick a data packet */
    if(p == NULL) {
      /* pick a packet from the neighbors queue who is associated with this cell */
      n = tsch_queue_get_nbr(link->node_address);
      if(n != NULL) {
        p = tsch_queue_get_packet_for_nbr(n, 0);
      }
      /* if there it is a broadcast slot and there were no broadcast packets, pick any unicast packet */
      if(p == NULL
         && n->is_broadcast) {
        p = tsch_queue_get_packet_for_any(&n, link->link_options & LINK_OPTION_SHARED);
      }
    }
  }
  /* return nbr (by reference) */
  if(target_neighbor != NULL) {
  	*target_neighbor = n;
  }
  return p;
}

static int associated = 0;
static struct pt cell_operation_pt;

static asn_t current_asn;
static rtimer_clock_t current_cell_start;
static uint16_t current_timeslot;
struct slotframe *current_slotframe;
static struct tsch_link *current_cell;
static struct tsch_packet *current_packet;
static struct tsch_neighbor *current_neighbor;

/* Buffer to store received packet temporarily */
static uint8_t tsch_rx_buffer[TSCH_MAX_PACKET_LEN] = {0};

/* last estimated drift */
static int32_t drift = 0;

static uint8_t eb_buf[TSCH_MAX_PACKET_LEN] = { 0 };

static PT_THREAD(tsch_cell_operation(struct rtimer *t, void *ptr));

/* Reads ACK and process sync IE header for drift correction */
static uint8_t
tsch_read_and_process_ack(struct tsch_neighbor *n, uint8_t seqno) {
	uint8_t ackbuf[STD_ACK_LEN + SYNC_IE_LEN];
	uint8_t ack_len, /* ack_len*/
					success,
					ack_status;
	int16_t d;

	ack_len = NETSTACK_RADIO.read((void *)ackbuf, STD_ACK_LEN + SYNC_IE_LEN);
  if(2 == ackbuf[0] && ack_len >= STD_ACK_LEN && seqno == ackbuf[2]) {
    success = 1;
    ack_status = 0;
    /* IE-list present? */
    if(ackbuf[1] & 2) {
      if(ack_len >= STD_ACK_LEN + SYNC_IE_LEN) {
      	/* sync-IE present? */
        if(ackbuf[3] == 0x02 && ackbuf[4] == 0x1e) {
          ack_status = ackbuf[5];
          ack_status |= ackbuf[6] << 8;
          /* If the originator was a time source neighbor, the receiver adjusts its own clock by incorporating the
           *  difference into an average of the drift to all its time source neighbors. The averaging method is
           *  implementation dependent. If the receiver is not a clock source, the time correction is ignored.
           */
          if(n != NULL) {
            if(n->is_time_source) {
              /* extract time correction */
              d = 0;
              /*is it a negative correction?*/
              if(ack_status & 0x0800) {
                d = -(ack_status & 0x0fff & ~0x0800);
              } else {
                d = ack_status & 0x0fff;
              }
              drift += d;
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
    success = 0;
    PUTCHAR('@');
    COOJA_DEBUG_STR("ACK not ok!\n");
  }
  return success;
}

/* post TX: Update neighbor state after a transmission */
static void
update_neighbor_state(struct tsch_neighbor *n, struct tsch_packet * p,
    struct tsch_link *link, uint8_t mac_tx_status) {

  int is_shared_link = link->link_options & LINK_OPTION_SHARED;
  int is_unicast = !n->is_broadcast;

  t0post_tx = RTIMER_NOW();

	/* TODO fix this
	 * If this is a shared link,
	 * we need to decrease the backoff value for all neighbors waiting to send */
	if(is_shared_link) {
		tsch_decrement_backoff_counter_for_all_nbrs();
	}

	if(mac_tx_status == MAC_TX_OK) {
	  /* Successful transmission */
	  tsch_queue_remove_packet_from_queue(n);

	  /* Update CSMA state in the unicast case */
	  if(is_unicast) {
	    if(is_shared_link || tsch_queue_is_empty(n)) {
	      /* If this is a shared link, reset backoff on success.
	       * Otherwise, do so only is the queue is empty */
	      n->BW_value = 0;
	      n->BE_value = MAC_MIN_BE;
	    }
	  }
	} else {
	  /* Failed transmission */
	  p->transmissions++;
	  if(p->transmissions >= MAC_MAX_FRAME_RETRIES) {
	    /* Drop packet */
	    tsch_queue_remove_packet_from_queue(n);
	  }

	  /* Update CSMA state in the unicast case */
	  if(is_unicast) {
	    /* Failures on dedicated (== non-shared) leave the backoff
	     * window nor exponent unchanged */
	    if(is_shared_link) {
	      /* Shared link: increment backoff exponent, pick a new window */
	      n->BE_value = MIN(n->BE_value + 1, MAC_MAX_BE);
        n->BW_value = tsch_random_byte((1 << n->BE_value) - 1);
	    }
	  }
	}

	t0post_tx = RTIMER_NOW() - t0post_tx;
}

static
PT_THREAD(tsch_tx_cell(struct pt *pt, struct rtimer *t))
{
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
	COOJA_DEBUG_STR("CELL_TX");
  PT_BEGIN(pt);
  /* packet payload */
  static void *payload = NULL;
  /* packet payload length */
  static uint8_t payload_len = 0;
  /* packet seqno */
  static uint8_t seqno = 0;
  /* tx status */
  static uint8_t mac_tx_status = 0;
  /* is this a broadcast packet? (wait for ack?) */
  static uint8_t is_broadcast = 0;
  /* is channel clear? */
  static uint8_t cca_status = 0;
  /* tx duration */
  static rtimer_clock_t tx_time;

  /* TODO There are small timing variations visible in cooja, which needs tuning */

  t0prepare = RTIMER_NOW();

	if(current_packet != NULL && current_packet->pkt != NULL) {
		payload = queuebuf_dataptr(current_packet->pkt);
		payload_len = queuebuf_datalen(current_packet->pkt);
	}

  /* is this a broadcast packet? (wait for ack?) */
  is_broadcast = current_neighbor->is_broadcast;
  /* read seqno from payload! */
  seqno = ((uint8_t *)(payload))[2];
  /* prepare packet to send: copy to radio buffer */
  mac_tx_status = !NETSTACK_RADIO.prepare(payload, payload_len);
  t0prepare = RTIMER_NOW() - t0prepare;

  cca_status = 1;
#if CCA_ENABLED
  /* delay before CCA */
  schedule_operation(t, (rtimer_callback_t)tsch_tx_cell, current_cell_start, TsCCAOffset);
  PT_YIELD(pt);
  on();
  /* CCA */
  BUSYWAIT_UNTIL_ABS(!(cca_status |= NETSTACK_RADIO.channel_clear()),
  		current_cell_start, TsCCAOffset + TsCCA);
  /* there is not enough time to turn radio off */
//  off();
#endif /* CCA_ENABLED */
  if(cca_status == 0 || !mac_tx_status) {
    mac_tx_status = (!mac_tx_status) ? MAC_TX_ERR : MAC_TX_COLLISION;
  } else {
    /* delay before TX */
    schedule_operation(t, (rtimer_callback_t)tsch_tx_cell, current_cell_start, TsTxOffset - delayTx);
    PT_YIELD(pt);
    t0tx = RTIMER_NOW();
    tx_time = RTIMER_NOW();
    /* send packet already in radio tx buffer */
    mac_tx_status = NETSTACK_RADIO.transmit(payload_len);
    /* Calculate TX duration based on sent packet len */
    tx_time = PACKET_DURATION(payload_len);
    /* limit tx_time in case of something wrong */
    tx_time = MIN(tx_time, wdDataDuration);
    off();

    t0tx = RTIMER_NOW() - t0tx;
    t0txack = RTIMER_NOW();
    if(mac_tx_status == RADIO_TX_OK) {
      if(!is_broadcast) {
        /* wait for ack after tx: sleep until ack time */
        COOJA_DEBUG_STR("wait for ACK\n");
        schedule_operation(t, (rtimer_callback_t)tsch_tx_cell, current_cell_start,
                       TsTxOffset + tx_time + TsTxAckDelay - TsShortGT - delayRx);
        PT_YIELD(pt);
        cca_status = 0;
        /* Disabling address decoding so the radio accepts the enhanced ACK */
        NETSTACK_RADIO_address_decode(0);
        on();
        /* Wait for ACK to come */
        BUSYWAIT_UNTIL_ABS(NETSTACK_RADIO.receiving_packet(),
                           current_cell_start, TsTxOffset + tx_time + TsTxAckDelay + TsShortGT - delayRx);
        /* Wait for ACK to finish */
        BUSYWAIT_UNTIL_ABS(!NETSTACK_RADIO.receiving_packet(),
                           current_cell_start, TsTxOffset + tx_time + TsTxAckDelay + TsShortGT + wdAckDuration);
        /* Enabling address decoding again so the radio filter data packets */
        NETSTACK_RADIO_address_decode(1);

        /* read ack and extract drift correction */
        mac_tx_status = tsch_read_and_process_ack(current_neighbor, seqno) ? MAC_TX_OK : MAC_TX_NOACK;
        off();
      } else {
      	mac_tx_status = MAC_TX_OK;
      }
      COOJA_DEBUG_STR("end tx slot\n");
    } else {
    	mac_tx_status = MAC_TX_ERR;
    }
  }
  t0txack = RTIMER_NOW() - t0txack;

  /* post TX: Update neighbor state */
  update_neighbor_state(current_neighbor, current_packet, current_cell, mac_tx_status);

  /* Store TX status */
  current_packet->ret = mac_tx_status;

	/* poll MAC TX callback */
	while(PROCESS_ERR_OK != process_post(&tsch_tx_callback_process, PROCESS_EVENT_POLL, (void *)current_packet)) {
		PRINTF("PROCESS_ERR_OK != process_post(&tsch_tx_callback_process\n");
	}
  PT_END(pt);
}

static
PT_THREAD(tsch_rx_cell(struct pt *pt, struct rtimer *t))
{
  PT_BEGIN(pt);

  /* packet payload length */
  static uint8_t payload_len = 0;

	static uint8_t ack_len;

  /* E-ACK buffer */
  static uint8_t ack_buf[STD_ACK_LEN + SYNC_IE_LEN] = {0};

  /* Send a NACK due to RX-buffer underflow? */
	static uint8_t nack = 0;

	/* Estimated drift based on RX time */
	static int32_t estimated_drift = 0;

	uint8_t ack_needed = 0,
			ret = 0;
	/* RX-packet sequence number */
	uint8_t seqno;

	uint32_t irq_status = 0;
	rtimer_clock_t rx_start_time, rx_end_time;
	struct tsch_neighbor *n;
	rimeaddr_t *source_address;

  t0rx = RTIMER_NOW();
	/**
	 * RX cell:
	 * 1. Check if it is used for TIME_KEEPING
	 * 2. Sleep and wake up just before expected RX time (with a guard time: TsLongGT)
	 * 3. Check for radio activity for the guard time: TsLongGT
	 * 4. Prepare and send ACK if needed
	 * 5. Drift calculated in the ACK callback registered with the radio driver. Use it if receiving from a time source neighbor.
	 **/

	/* this cell is RX current_cell. Check if it is used for keep alive as well*/
	if((current_cell->link_options & LINK_OPTION_TIME_KEEPING)) {
		/* TODO LINK_OPTION_TIME_KEEPING ??? */
	}

	/* wait before RX */
	schedule_operation(t, (rtimer_callback_t)tsch_rx_cell, current_cell_start, TsTxOffset - TsLongGT - delayRx);
	COOJA_DEBUG_STR("schedule RX on guard time - TsLongGT");
	PT_YIELD(pt);
	/* Start radio for at least guard time */
	on();
	/* Check if receiving within guard time */
	BUSYWAIT_UNTIL_ABS(NETSTACK_RADIO.receiving_packet(),
										 current_cell_start, TsTxOffset + TsLongGT);
	/* register packet timestamp*/
	rx_start_time = NETSTACK_RADIO_read_sfd_timer() & 0xffffUL;
	/* in case of 16bit SFD timer and 32bit RTimer*/
  rx_start_time |= (uint32_t)RTIMER_NOW() & 0xffff0000UL;
	if(!NETSTACK_RADIO.receiving_packet() && !NETSTACK_RADIO.pending_packet()) {
		off();
		t0rx = RTIMER_NOW() - t0rx;
		/* no packets on air */
		ret = 0;
	} else {
#if CONTIKI_TARGET_JN5168
		if(!irq_status) {
			/* Check if receiving within guard time */
			BUSYWAIT_UNTIL_ABS((irq_status = NETSTACK_RADIO_pending_irq()),
												 current_cell_start, TsTxOffset + TsLongGT + wdDataDuration + delayRx);
		}
#endif

		/* Wait until packet is received */
		BUSYWAIT_UNTIL_ABS(!NETSTACK_RADIO.receiving_packet(),
											 current_cell_start, TsTxOffset + TsLongGT + wdDataDuration);
		off();

		/* Read packet */
    payload_len = NETSTACK_RADIO.read((void *)tsch_rx_buffer, TSCH_MAX_PACKET_LEN);

		/* register packet timestamp */
		rx_end_time = rx_start_time + PACKET_DURATION(payload_len);

		ack_needed = tsch_packet_is_ack_needed(tsch_rx_buffer, payload_len);

		t0rx = RTIMER_NOW() - t0rx;
		t0rxack = RTIMER_NOW();
		if(payload_len) {
			/* wait until ack time */
			if(ack_needed) {
			  /* read seqno from payload */
			  seqno = tsch_rx_buffer[2];
			  estimated_drift = (int32_t)current_cell_start + TsTxOffset - rx_start_time;

			  ack_len = tsch_packet_make_sync_ack(estimated_drift, ack_buf, seqno, nack);

			  /* Copy to radio buffer */
			  NETSTACK_RADIO.prepare((const void *)ack_buf, ack_len);

			  schedule_operation(t, (rtimer_callback_t)tsch_rx_cell, rx_end_time, TsTxAckDelay - delayTx);
				PT_YIELD(pt);
				NETSTACK_RADIO.transmit(ack_len);
			}

			source_address = tsch_packet_extract_sender_address(tsch_rx_buffer, payload_len);
			/* If the originator was a time source neighbor, the receiver adjusts its own clock by incorporating the
			 *  difference into an average of the drift to all its time source neighbors. The averaging method is
			 *  implementation dependent. If the receiver is not a clock source, the time correction is ignored.
			 */
			if(estimated_drift) {
				COOJA_DEBUG_PRINTF("estimated_drift seen %d\n", estimated_drift);
				/* check the source address for potential time-source match */
				n = tsch_queue_get_nbr(&source_address);
				if(n != NULL && n->is_time_source) {
					/* should be the average of drifts to all time sources */
					drift -= estimated_drift;
					COOJA_DEBUG_STR("drift recorded");
				}
			}

			/* poll RDC RX callback */
			while(PROCESS_ERR_OK != process_post(&tsch_rx_callback_process, PROCESS_EVENT_POLL, (void *)payload_len)) {
				PRINTF("PROCESS_ERR_OK != process_post(&tsch_rx_callback_process\n");
			}

			/* TODO return length instead? or status? or something? */
			ret = 1;
		}
	}
	t0rxack = RTIMER_NOW() - t0rxack;

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

		current_cell = get_cell(current_timeslot);

		if(current_cell->link_options & LINK_OPTION_TX) {
			/* Get a packet ready to be sent */
			current_packet = get_packet_and_neighbor_for_cell(current_cell, &current_neighbor);
		}

	  /* Decide whether it is a TX/RX/IDLE or OFF link */
		/* Actual slot operation */
		if(current_packet != NULL) {
			/* We have something to transmit, do the following:
			 * 1. send
			 * 2. update_backoff_state(current_neighbor)
			 * 3. post tx callback
			 **/
			static struct pt cell_tx_pt;
			PT_SPAWN(&cell_operation_pt, &cell_tx_pt, tsch_tx_cell(&cell_tx_pt, t));
		} else if(current_cell->link_options & LINK_OPTION_RX) {
			/* Listen */
			static struct pt cell_rx_pt;
			PT_SPAWN(&cell_operation_pt, &cell_rx_pt, tsch_rx_cell(&cell_rx_pt, t));
		}

		/* End of slot operation, schedule next slot */
		/* Do we need to resynchronize? i.e., wait for EB again */
		if(!tsch_is_coordinator && current_asn - last_sync_asn > (asn_t)DESYNC_THRESHOLD) {
			associated = 0;
			process_post(&tsch_process, PROCESS_EVENT_POLL, NULL);
		} else {
			/* Get next active timeslot */
			uint16_t next_timeslot = get_next_active_timeslot(current_timeslot);
			/* Calculate number of slots between current and next */
			uint16_t timeslot_diff = next_timeslot > current_timeslot ? next_timeslot - current_timeslot:
					current_slotframe->size - (current_timeslot - next_timeslot);
			/* Update ASN */
			current_asn += (asn_t)timeslot_diff;

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
    current_asn = (asn_t)current_timeslot;
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





/* a polled-process to invoke the tsch_packet_input callback asynchronously */
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

/* a polled-process to invoke the MAC tx callback asynchronously */
PROCESS_THREAD(tsch_rx_callback_process, ev, data)
{
  PROCESS_BEGIN();
  PRINTF("tsch_tx_callback_process: started\n");
  COOJA_DEBUG_STR("tsch_tx_callback_process: started\n");

  while(1) {
    uint8_t payload_len;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    /*		PRINTF("tsch_tx_callback_process: calling mac tx callback\n"); */
    COOJA_DEBUG_STR("tsch_rx_callback_process: calling input\n");
    payload_len = (uint8_t)data;
    if(payload_len > 0 && payload_len <= PACKETBUF_SIZE) {
      packetbuf_clear();
      memcpy(packetbuf_dataptr(), (void *)tsch_rx_buffer, payload_len);
      packetbuf_set_datalen(payload_len);
      NETSTACK_RDC.input();
    }
  }
  PROCESS_END();
}

/* a periodic process to send EBs */
PROCESS_THREAD(tsch_send_eb_process, ev, data)
{
  static struct etimer eb_timer;
  PROCESS_BEGIN();
	static struct tsch_neighbor *n;
  uint8_t eb_len = 0;

	/* yield the process until a neighbor for EBs is added */
  etimer_set(&eb_timer, CLOCK_SECOND/10);
  do {
		n = tsch_queue_add_nbr(&tsch_eb_address);
		if(n == NULL) {
			PROCESS_WAIT_EVENT();
		}
		etimer_reset(&eb_timer);
	}	while(n == NULL);

	/* Lock the EB nbr so it does not get deleted later on */
	tsch_queue_lock_nbr(n);

	PRINTF("tsch_send_eb_process: started\n");
  COOJA_DEBUG_STR("tsch_send_eb_process: started\n");
  etimer_set(&eb_timer, EB_PERIOD/2 + tsch_random_byte(0xff)*(EB_PERIOD/(2UL*0xff)));
  while(1) {
    PROCESS_WAIT_EVENT();
    if(ev == PROCESS_EVENT_TIMER) {
			if(data == &eb_timer) {
				etimer_reset(&eb_timer);
				/* Prepare the EB packet and schedule it to be sent */
				packetbuf_clear();
				eb_len = tsch_packet_make_eb(packetbuf_dataptr(), TSCH_MAX_PACKET_LEN);
				packetbuf_set_datalen(eb_len);
		    /* enqueue eb packet */
		    if(n != NULL && !tsch_queue_add_packet(&tsch_eb_address, NULL, NULL)) {
		      PRINTF("tsch: can't send EB packet\n");
		    }
			}
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static void
tsch_init_variables(void)
{
  /* setting seed for the random generator */
  tsch_random_init(clock_time() * RTIMER_NOW());
  /* look for a root to sync with */
  ieee154e_vars.current_slotframe = &minimum_slotframe;
  ieee154e_vars.slot_template_id = 1;
  ieee154e_vars.hop_sequence_id = 1;
  ieee154e_vars.asn = 0;
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

	/* save start sfd only */
	NETSTACK_RADIO_sfd_sync(1, 0);

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
  /* Disable radio interrupts so they do not interfere with RTIMER interrupts
   * Radio will be polled instead */
  NETSTACK_RADIO_set_interrupt_enable(0);

  leds_blink();
  tsch_init_variables();
  /* on resynchronization, the node has already joined a RPL network and it is mistaking it with root
   * this flag is used to prevent this */
  ieee154e_vars.first_associate = 0;
  tsch_queue_init();
  tsch_schedule_init();
  /* try to associate to a network or start one if setup as RPL root */
//  process_start(&tsch_associate, NULL);
  /* XXX for debugging */
  hop_channel(0);
  /*	NETSTACK_RADIO.on(); */
//  powercycle(&ieee154e_vars.t, NULL);
  process_start(&tsch_tx_callback_process, NULL);
  process_start(&tsch_send_eb_process, NULL);
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
