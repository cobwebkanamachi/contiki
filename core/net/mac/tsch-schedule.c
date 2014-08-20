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
 *         IEEE 802.15.4 TSCH MAC schedule manager.
 * \author
 *         Beshr Al Nahas <beshr@sics.se>
 *         Simon Duquennoy <simonduq@sics.se>
 */

#include "contiki.h"
#include "dev/leds.h"
#include "lib/memb.h"
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

/* Max number of TSCH slotframes */
#ifdef TSCH_CONF_MAX_SLOTFRAMES
#define TSCH_MAX_SLOTFRAMES TSCH_CONF_MAX_SLOTFRAMES
#else
#define TSCH_MAX_SLOTFRAMES 4
#endif

/* Max number of links */
#ifdef TSCH_CONF_MAX_LINKS
#define TSCH_MAX_LINKS TSCH_CONF_MAX_LINKS
#else
#define TSCH_MAX_LINKS 32
#endif

/* Pre-allocated space for links */
MEMB(link_memb, struct tsch_link_, TSCH_MAX_LINKS);
/* Pre-allocated space for slotframes */
MEMB(slotframe_memb, struct tsch_slotframe_, TSCH_MAX_SLOTFRAMES);
/* List of slotframes (each slotframe holds its own list of links) */
LIST(slotframe_list);

/* Adds and returns a slotframe (NULL if failure) */
struct tsch_slotframe_ *
tsch_schedule_add_slotframe(uint16_t handle, uint16_t size)
{
  if(tsch_schedule_get_slotframe_from_handle(handle)) {
    /* A slotframe with this handle already exists */
    return NULL;
  } else {
    struct tsch_slotframe_ *sf = memb_alloc(&slotframe_memb);
    if(sf != NULL) {
      /* Initialize the slotframe */
      sf->handle = handle;
      sf->size = size;
      LIST_STRUCT_INIT(sf, links_list);
      /* Add the slotframe to the global list */
      list_add(slotframe_list, sf);
    }
    return sf;
  }
}

/* Looks for a slotframe from a handle */
struct tsch_slotframe_ *
tsch_schedule_get_slotframe_from_handle(uint16_t handle)
{
  struct tsch_slotframe_ *sf = list_head(slotframe_list);
  while(sf != NULL) {
    if(sf->handle == handle) {
      return sf;
    }
    sf = list_item_next(sf);
  }
  return NULL;
}

/* Adds a link to a slotframe, return a pointer to it (NULL if failure) */
struct tsch_link_ *
tsch_schedule_add_link(struct tsch_slotframe_ *slotframe,
    uint8_t link_options, enum link_type link_type, const rimeaddr_t *address,
    uint16_t timeslot, uint16_t channel_offset)
{
  /* First check whether the timeslot is already occupied */
  if(slotframe != NULL && !tsch_schedule_get_link_from_timeslot(slotframe, timeslot)) {
    struct tsch_link_ *l = memb_alloc(&link_memb);
    if(l != NULL) {
      /* Initialize link */
      l->link_options = link_options;
      l->link_type = link_type;
      l->slotframe_handle = slotframe->handle;
      rimeaddr_copy(&l->addr, address);
      l->timeslot = timeslot;
      l->channel_offset = channel_offset;
      /* Add the link to the slotframe */
      list_add(slotframe->links_list, l);
    }
  }
  return NULL;
}

/* Looks within a slotframe for a link with a given timeslot */
struct tsch_link_ *
tsch_schedule_get_link_from_timeslot(struct tsch_slotframe_ *slotframe, uint16_t timeslot)
{
  if(slotframe != NULL) {
    struct tsch_link_ *l = list_head(slotframe->links_list);
    /* Loop over all items. Assume there is max one link per timeslot */
    while(l != NULL) {
      if(l->timeslot == timeslot) {
        return l;
      }
      l = list_item_next(l);
    }
    return l;
  } else {
    return NULL;
  }
}

/* Return the next active (not OFF) timeslot after a given timeslot */
struct tsch_link_ *
tsch_schedule_get_link_from_asn(asn_t asn)
{
  struct tsch_link_ *curr_best = NULL;
  struct tsch_slotframe_ *sf = list_head(slotframe_list);
  /* For each slotframe, looks for a link matching the asn.
   * Tx links have priority, then lower handle have priority. */
  while(sf != NULL) {
    /* Get timeslot from ASN, given the slotframe length */
    uint16_t timeslot = asn % sf->size;
    struct tsch_link_ *l = tsch_schedule_get_link_from_timeslot(sf, timeslot);
    /* We have a match */
    if(l != NULL) {
      if(curr_best == NULL) {
        curr_best = l;
      } else {
        /* We already have a current best,
         * we must check Tx flag and handle to find the highest priority link */
        if(!(curr_best->link_options & LINK_OPTION_TX) && (l->link_options & LINK_OPTION_TX)) {
          /* We have Tx link, the current best didn't */
          curr_best = l;
        } else if(curr_best->slotframe_handle > sf->handle) {
          /* We have a lower handle */
          curr_best = l;
        }
      }
    }
    sf = list_item_next(sf);
  }
  return curr_best;
}

/* Returns the number of slots until the first active link after a given ASN */
uint16_t
tsch_schedule_time_to_next_active_link(asn_t asn)
{
  uint16_t curr_earliest = 0;
  struct tsch_slotframe_ *sf = list_head(slotframe_list);
  /* For each slotframe, look for the earliest occurring link */
  while(sf != NULL) {
    /* Get timeslot from ASN, given the slotframe length */
    uint16_t timeslot = asn % sf->size;
    struct tsch_link_ *l = list_head(sf->links_list);
    while(l != NULL) {
      uint16_t time_to_timeslot =
          l->timeslot > timeslot ?
              l->timeslot - timeslot :
              sf->size + l->timeslot - timeslot;
      if(curr_earliest == 0 || time_to_timeslot < curr_earliest) {
        curr_earliest = time_to_timeslot;
      }
      l = list_item_next(l);
    }
    sf = list_item_next(sf);
  }
  return curr_earliest;
}

void tsch_schedule_print()
{
  struct tsch_slotframe_ *sf = list_head(slotframe_list);

  printf("Schedule: slotframe list\n");

  while(sf != NULL) {
    struct tsch_link_ *l = list_head(sf->links_list);

    printf("[Slotframe] Handle %u, size %u\n", sf->handle, sf->size);
    printf("List of links:\n");

    while(l != NULL) {
      printf("[Link] Options %02x, type %u, timeslot %u, channel offset %u, address %u\n",
          l->link_options, l->link_type, l->timeslot, l->channel_offset, l->addr.u8[7]);
      l = list_item_next(l);
    }

    sf = list_item_next(sf);
  }

  printf("Schedule: end of slotframe list\n");
}

void tsch_schedule_test()
{
  static rimeaddr_t link_broadcast_address = { { 0, 0, 0, 0, 0, 0, 0, 0 } };
  static rimeaddr_t address1 = { { 0x00, 0x12, 0x74, 01, 00, 01, 01, 01 } };
  static rimeaddr_t address2 = { { 0x00, 0x12, 0x74, 02, 00, 02, 02, 02 } };

  struct tsch_slotframe_ *sf1 = tsch_schedule_add_slotframe(20, 5);
  struct tsch_slotframe_ *sf2 = tsch_schedule_add_slotframe(21, 3);

  tsch_schedule_add_link(sf1,
      LINK_OPTION_RX | LINK_OPTION_TX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING,
      LINK_TYPE_ADVERTISING, &link_broadcast_address,
      0, 1);

  tsch_schedule_add_link(sf1,
      LINK_OPTION_RX,
      LINK_TYPE_NORMAL, &address1,
      1, 1);

  tsch_schedule_add_link(sf1,
      LINK_OPTION_RX,
      LINK_TYPE_NORMAL, &address1,
      4, 10);

  tsch_schedule_add_link(sf2,
      LINK_OPTION_TX,
      LINK_TYPE_NORMAL, &address2,
      0, 2);

  tsch_schedule_print();

  int asn;
  for(asn=0; asn<20; asn++) {
    struct tsch_link_ *l = tsch_schedule_get_link_from_asn((asn_t) asn);
    if(l != NULL) {
      printf("asn %u: timeslot %u, channel offset %u (schedule handle %u)\n",
          (unsigned)asn, l->timeslot, l->channel_offset, l->slotframe_handle);
    } else {
      printf("asn %u: no link\n", (unsigned)asn);
    }
    printf("time to next %u\n", tsch_schedule_time_to_next_active_link((asn_t) asn));
  }
}

/* Initialization */
void tsch_schedule_init()
{
  /* Build 6TiSCH minimal schedule.
   * We pick a slotframe length of 101. */
  struct tsch_slotframe_ *sf = tsch_schedule_add_slotframe(0, 101);
  /* Add a single Tx|Rx|Shared slot using broadcast address (i.e. usable for unicast and broadcast).
   * We set the link type to advertising, which is not compliant with 6TiSCH minimal schedule
   * but is required according to 802.15.4e if also used for EB transmission.
   * Timeslot: 0, channel offset: 0. */
  tsch_schedule_add_link(sf,
        LINK_OPTION_RX | LINK_OPTION_TX | LINK_OPTION_SHARED,
        LINK_TYPE_ADVERTISING, &tsch_broadcast_address,
        0, 0);

  /* Example of a dedicated Tx unicast link. Timeslot: 1, channel offset: 0. */
  /* static rimeaddr_t dest_addr = { { 0x00, 0x12, 0x74, 01, 00, 01, 01, 01 } }; */
  /* tsch_schedule_add_link(sf,
        LINK_OPTION_RX,
        LINK_TYPE_NORMAL, &dest_addr,
        1, 0); */
}
