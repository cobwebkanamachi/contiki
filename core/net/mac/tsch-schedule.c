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

/* Initialization */
void tsch_schedule_init()
{
  printf("schedule: hi!\n");
}
