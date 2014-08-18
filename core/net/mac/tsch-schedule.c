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
tsch_schedule_add_slotframe(uint16_t size)
{
  return NULL;
}

/* Adds a link to a slotframe, return a pointer to it (NULL if failure) */
struct tsch_link_ *
tsch_schedule_add_link(struct tsch_slotframe_ *slotframe,
    uint8_t link_options, enum link_type link_type, const rimeaddr_t *node_address,
    uint16_t timeslot, uint16_t channel_offset)
{

}

/* Return the next active (not OFF) timeslot after a given timeslot */
struct tsch_link_ *
tsch_schedule_get_link_from_asn(asn_t asn)
{

}

/* Initialization */
void tsch_schedule_init()
{
  printf("schedule: hi!\n");
}
