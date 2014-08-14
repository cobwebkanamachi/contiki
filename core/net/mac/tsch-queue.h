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
 *         Per-neighbor packet queues for TSCH MAC
 * \author
 *         Beshr Al Nahas <beshr@sics.se>
 */

#ifndef __TSCH_QUEUE_H__
#define __TSCH_QUEUE_H_

#include "contiki.h"
#include "net/rime/rimeaddr.h"

/* TSCH queue size: must be power of two to enable atomic put operation */
#if ( TSCH_NBR_BUFFER_CONF_SIZE && !(TSCH_NBR_BUFFER_CONF_SIZE & (TSCH_NBR_BUFFER_CONF_SIZE-1)) )
#define NBR_BUFFER_SIZE TSCH_NBR_BUFFER_CONF_SIZE
#else
#define NBR_BUFFER_SIZE 4
#endif

/* TSCH packet information */
struct TSCH_packet
{
	struct queuebuf * pkt; /* pointer to the packet to be sent */
	mac_callback_t sent; /* callback for this packet */
	void *ptr; /* MAC callback parameter */
	uint8_t transmissions; /* #transmissions performed for this packet */
	uint8_t ret; /* status -- MAC return code */
};

/* TSCH neighbor information */
struct neighbor_queue
{
	struct TSCH_packet buffer[NBR_BUFFER_SIZE]; /* circular buffer of packets.
	Its size must be a power of two 	to allow for atomic put */
	uint8_t is_time_source; /* is this neighbor a time source? */
	uint8_t BE_value; /* current value of backoff exponent */
	uint8_t BW_value; /* current value of backoff counter */
	volatile uint8_t put_ptr,
					get_ptr; /* pointers for circular buffer implementation */
};

struct neighbor_queue *neighbor_queue_from_addr(const rimeaddr_t *addr);
struct neighbor_queue *add_queue(const rimeaddr_t *addr);
int remove_queue(const rimeaddr_t *addr);
int add_packet_to_queue(mac_callback_t sent, void* ptr, const rimeaddr_t *addr);
int remove_packet_from_queue(const rimeaddr_t *addr);
const struct TSCH_packet *read_packet_from_queue(const rimeaddr_t *addr);
/* get a packet to send in a shared slot, and put the neighbor reference in n */
const struct TSCH_packet *get_next_packet_for_shared_slot_tx(struct neighbor_queue **n);
/* returns the first packet in the queue of a neighbor */
const struct TSCH_packet* read_packet_from_neighbor_queue(const struct neighbor_queue *n);
void tsch_queue_init(void);

#endif /* __TSCH_QUEUE_H__ */
