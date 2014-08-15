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
 *         Per-neighbor packet queues for TSCH MAC.
 *         The list of neighbor uses a lock, but per-neighbor packet array are lockfree.
 *				 Read-only operation on neighbor and packets are allowed from interrupts and outside of them.
 *				 *Other operations are allowed outside of interrupt only.*
 * \author
 *         Beshr Al Nahas <beshr@sics.se>
 */

#ifndef __TSCH_QUEUE_H__
#define __TSCH_QUEUE_H_

#include "contiki.h"
#include "net/rime/rimeaddr.h"

/* Per-neighbor queue size: must be power of two to enable atomic put operation */
#if (TSCH_NBR_BUFFER_CONF_SIZE && !(TSCH_NBR_BUFFER_CONF_SIZE & (TSCH_NBR_BUFFER_CONF_SIZE - 1)))
#define NBR_BUFFER_SIZE TSCH_NBR_BUFFER_CONF_SIZE
#else
#define NBR_BUFFER_SIZE 4
#endif

/* TSCH packet information */
struct tsch_packet {
  struct queuebuf *pkt;  /* pointer to the packet to be sent */
  mac_callback_t sent; /* callback for this packet */
  void *ptr; /* MAC callback parameter */
  uint8_t transmissions; /* #transmissions performed for this packet */
  uint8_t ret; /* status -- MAC return code */
};

/* TSCH neighbor information */
struct tsch_neighbor {
  /* Neighbors are stored as a list: "next" must be the first field */
  struct tsch_neighbor *next;
  /* TODO: have only one array for the whole system */
  struct tsch_packet buffer[NBR_BUFFER_SIZE]; /* circular buffer of packets.
                                                 Its size must be a power of two  to allow for atomic put */
  rimeaddr_t addr; /* link-layer address of the neighbor */
  uint8_t is_time_source; /* is this neighbor a time source? */
  uint8_t BE_value; /* current value of backoff exponent */
  uint8_t BW_value; /* current value of backoff counter */
  volatile uint8_t put_ptr,
                   get_ptr; /* pointers for circular buffer implementation */
};

/* Add a TSCH neighbor */
struct tsch_neighbor *tsch_queue_add_nbr(const rimeaddr_t *addr);
/* Get a TSCH neighbor */
struct tsch_neighbor *tsch_queue_get_nbr(const rimeaddr_t *addr);
/* Remove TSCH neighbor queue */
void tsch_queue_remove_nbr(struct tsch_neighbor *n);
/* Add packet to neighbor queue. Use same lockfree implementation as ringbuf.c (put is atomic) */
int tsch_queue_add_packet(const rimeaddr_t *addr, mac_callback_t sent, void *ptr);
/* Remove first packet from a neighbor queue */
int tsch_queue_remove_packet_from_dest_addr(const rimeaddr_t *addr);
/* Returns the first packet from a neighbor queue */
struct tsch_packet *tsch_queue_get_packet_from_neighbor(const struct tsch_neighbor *n);
/* Returns the head packet from a neighbor queue (from neighbor address) */
struct tsch_packet *tsch_queue_get_packet_from_dest_addr(const rimeaddr_t *addr);
/* Returns the head packet of any neighbor queue.
 * Writes pointer to the neighbor in *n */
struct tsch_packet *tsch_queue_get_any_packet(struct tsch_neighbor **n);
/* Initialize TSCH queue module */
void tsch_queue_init(void);

#endif /* __TSCH_QUEUE_H__ */
