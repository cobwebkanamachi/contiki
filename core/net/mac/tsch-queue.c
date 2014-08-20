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
 *         Simon Duquennoy <simonduq@sics.se>
 */

#include "contiki.h"
#include "net/queuebuf.h"
#include "net/nbr-table.h"
#include "net/mac/rdc.h"
#include "net/mac/tsch-private.h"
#include "net/mac/tsch-queue.h"
#include "net/mac/tsch-schedule.h"
#include <string.h>

#define DEBUG DEBUG_NONE
#include "net/uip-debug.h"

/* Is this neighbor queue empty? */
#define QUEUE_EMPTY(n) (!((((n)->put_ptr - (n)->get_ptr) & (NBR_BUFFER_SIZE - 1)) > 0))
/* Is this neighbor queue full? */
#define QUEUE_FULL(n) ((((((n)->put_ptr - (n)->get_ptr)) & (NBR_BUFFER_SIZE - 1)) == (NBR_BUFFER_SIZE - 1)))

/* We lock the whole module whenever manipulating the neighbor list.
 * Note that adding/removing packets from a neighbor queue is done using
 * a ringbuf with atomic insert, and does not require to take the lock. */
static int locked = 0;

/* Per-neighbor queue */
NBR_TABLE(struct tsch_neighbor, neighbor_queues);

/* Lock a TSCH neighbor, to prevent removal */
int
tsch_queue_lock_nbr(struct tsch_neighbor *n)
{
	if(n != NULL) {
		/* Lock neighbor entry */
		return nbr_table_lock(neighbor_queues, n);
	}
  return 0;
}

/* Get the address of a neighbor */
rimeaddr_t *
tsch_queue_get_nbr_address(struct tsch_neighbor *n)
{
	return nbr_table_get_lladdr(neighbor_queues, n);
}

/* Add a TSCH neighbor */
struct tsch_neighbor *
tsch_queue_add_nbr(const rimeaddr_t *addr)
{
  struct tsch_neighbor *n = NULL;

  if(!locked) {
    locked = 1;

    /* If we have an entry for this neighbor already, we simply update it */
    n = tsch_queue_get_nbr(addr);
    if(n != NULL) {
      /* Reset neighbor entry */
      memset(n, 0, sizeof(struct tsch_neighbor));
    } else {
      /* Add neighbor (will be zeroed initially) */
      n = nbr_table_add_lladdr(neighbor_queues, addr);
    }
    if(n != NULL) {
      /* Initialize neighbor entry */
      n->BE_value = MAC_MIN_BE;
      n->is_broadcast = rimeaddr_cmp(addr, &tsch_eb_address)
          || rimeaddr_cmp(addr, &tsch_broadcast_address);
    }
    locked = 0;
  }
  return n;
}

/* Get a TSCH neighbor */
struct tsch_neighbor *
tsch_queue_get_nbr(const rimeaddr_t *addr)
{
  if(!locked) {
    return nbr_table_get_from_lladdr(neighbor_queues, addr);
  }
  return NULL;
}

/* Remove TSCH neighbor queue */
void
tsch_queue_remove_nbr(struct tsch_neighbor *n)
{
  if(n != NULL && !locked) {
    int i;
    locked = 1;

    /* Free queuebufs */
    for(i = 0; i < NBR_BUFFER_SIZE; i++) {
      queuebuf_free(n->buffer[i].pkt);
    }
    nbr_table_remove(neighbor_queues, n);

    locked = 0;
  }
}

/* Add packet to neighbor queue. Use same lockfree implementation as ringbuf.c (put is atomic) */
int
tsch_queue_add_packet(const rimeaddr_t *addr, mac_callback_t sent, void *ptr)
{
  if(!locked) {
    struct tsch_neighbor *n = tsch_queue_get_nbr(addr);
    if(n == NULL) {
      n = tsch_queue_add_nbr(addr);
    }
    if(n != NULL && !QUEUE_FULL(n)) {
      /* Enqueue packet */
      n->buffer[n->put_ptr].pkt = queuebuf_new_from_packetbuf();
      if(n->buffer[n->put_ptr].pkt != NULL) {
        n->buffer[n->put_ptr].ptr = ptr;
        n->buffer[n->put_ptr].ret = MAC_TX_DEFERRED;
        n->buffer[n->put_ptr].transmissions = 0;
        PRINTF("TSCH-queue: packet enqueued 0%x p%d @%p\n", addr->u8[7], n->put_ptr, n->buffer[n->put_ptr].pkt);
        /* The following line must be atomic */
        n->put_ptr = (n->put_ptr + 1) & (NBR_BUFFER_SIZE - 1);
        return 1;
      }
    }
  }
  PRINTF("TSCH-queue: failed to enqueue packet\n");
  return 0;
}

/* Remove first packet from a neighbor queue */
int
tsch_queue_remove_packet_from_queue(struct tsch_neighbor *n)
{
  if(!locked) {
    if(n != NULL && !QUEUE_EMPTY(n)) {
      uint8_t prev_get_ptr = n->get_ptr;
      /* Actually remove form the ringbuf. Must be atomic. */
      n->get_ptr = (n->get_ptr + 1) & (NBR_BUFFER_SIZE - 1);
      /* Deallocate the queuebuf only now that it is not accessible anymore */
      queuebuf_free(n->buffer[prev_get_ptr].pkt);
      PRINTF("TSCH-queue: remove packet 0%x p%d @%p\n", addr->u8[7], prev_get_ptr, n->buffer[prev_get_ptr].pkt);
      return 1;
    }
  }
  return 0;
}

/* Is the neighbor queue empty? */
int
tsch_queue_is_empty(const struct tsch_neighbor *n)
{
  return !locked && QUEUE_EMPTY(n);
}

/* Returns the first packet from a neighbor queue */
struct tsch_packet *
tsch_queue_get_packet_for_nbr(const struct tsch_neighbor *n, int is_shared_link)
{
  if(!locked) {
    if(n != NULL && !QUEUE_EMPTY(n) &&
        !(is_shared_link && !tsch_queue_backoff_expired(n))) { /* If this is a shared link,
        make sure the backoff has expired */
      PRINTF("TSCH-queue: p%d @%p\n", n->get_ptr, n->buffer[n->get_ptr].pkt);
      return (struct tsch_packet *)&n->buffer[n->get_ptr];
    }
  }
  return NULL;
}

/* Returns the head packet from a neighbor queue (from neighbor address) */
struct tsch_packet *
tsch_queue_get_packet_for_dest_addr(const rimeaddr_t *addr, int is_shared_link)
{
  if(!locked) {
    return tsch_queue_get_packet_for_nbr(tsch_queue_get_nbr(addr), is_shared_link);
  }
  return NULL;
}

/* Returns the head packet of any neighbor queue with zero backoff counter.
 * Writes pointer to the neighbor in *n */
struct tsch_packet *
tsch_queue_get_packet_for_any(struct tsch_neighbor **n, int is_shared_link)
{
  if(!locked) {
    /* TODO: round-robin among neighbors */
    struct tsch_neighbor *curr_nbr = nbr_table_head(neighbor_queues);
    struct tsch_packet *p = NULL;

    while(curr_nbr != NULL) {
      p = tsch_queue_get_packet_for_nbr(curr_nbr, is_shared_link);
      if(p != NULL) {
        *n = curr_nbr;
        return p;
      }
      curr_nbr = nbr_table_next(neighbor_queues, curr_nbr);
    }
  }
  return NULL;
}

/* May the neighbor transmit over a share link? */
int
tsch_queue_backoff_expired(struct tsch_neighbor *n)
{
  return n->BW_next_asn <= current_asn;
}

/* Reset neighbor backoff */
void
tsch_queue_backoff_reset(struct tsch_neighbor *n)
{
  n->BW_next_asn = 0;
  n->BE_value = MAC_MIN_BE;
}

/* Increment backoff exponent, pick a new window */
void
tsch_queue_backoff_inc(struct tsch_neighbor *n)
{
  int16_t window;
  /* Increment exponent */
  n->BE_value = MIN(n->BE_value + 1, MAC_MAX_BE);
  /* Pick a window (number of shared slots to skip) */
  window = tsch_random_byte((1 << n->BE_value) - 1);
  /* Look for the next shared slot where we can transmit.
   * Iterate window+1 times, so that BW_next_asn points to the next usable share slot. */
  n->BW_next_asn = current_asn;
  while(window >= 0) {
    struct tsch_link_ *l;
    /* Jump to next active slot */
    n->BW_next_asn += tsch_schedule_time_to_next_active_link(n->BW_next_asn);
    /* Check if the link is a shared slot that can be used
     * to transmit to n */
    l = tsch_schedule_get_link_from_asn(n->BW_next_asn);
    if(l->link_options & LINK_OPTION_SHARED /* is shared */
        && l->link_options & LINK_OPTION_TX /* is tx */
        && (rimeaddr_cmp(&l->addr, &n->addr) /* is targetted at n */
            || rimeaddr_cmp(&l->addr, &tsch_broadcast_address)) /* or is broadcast */
        ) {
      --window;
    }
  }
}

/* Is the module locked? */
int
tsch_queue_is_locked(void)
{
  return locked;
}

/* Initialize TSCH queue module */
void
tsch_queue_init(void)
{
  nbr_table_register(neighbor_queues, (nbr_table_callback *)tsch_queue_remove_nbr);
}
