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

#ifndef __TSCH_SCHEDULE_H__
#define __TSCH_SCHEDULE_H__

#include "contiki.h"
#include "net/rime/rimeaddr.h"

enum link_options_enum {
  LINK_OPTION_TX = 1,
  LINK_OPTION_RX = 2,
  LINK_OPTION_SHARED = 4,
  LINK_OPTION_TIME_KEEPING = 8,
};

enum link_type_enum {
  LINK_TYPE_NORMAL = 0,
  LINK_TYPE_ADVERTISING = 1,
};

enum cell_decision_enum {
  CELL_OFF = 0,
  CELL_TX = 1,
  CELL_TX_IDLE = 2, /* No packet to transmit */
  CELL_TX_BACKOFF = 3, /* csma backoff */
  CELL_RX = 4,
};

struct tsch_link {
  /* Unique identifier (local to specified slotframe) for the link */
  uint16_t link_handle;
  /* Relative number of slot in slotframe */
  /* uint16_t timeslot; */
  /* maybe 0 to 15 */
  uint8_t channel_offset;
  /*b0 = Transmit, b1 = Receive, b2 = Shared, b3= Timekeeping, b4–b7 reserved.*/
  uint8_t link_options;
  /* Type of link. NORMAL = 0. ADVERTISING = 1, and indicates
     the link may be used to send an Enhanced beacon. */
  uint8_t link_type;
  /* short address of neighbor */
  rimeaddr_t *node_address;
};

struct slotframe {
  /* Unique identifier */
  uint16_t slotframe_handle;
  uint16_t length;
  uint16_t on_size;
  struct tsch_link **links;
};

/* Initialization */
void tsch_schedule_init();

#endif /* __TSCH_SCHEDULE_H__ */
