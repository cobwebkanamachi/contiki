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

#include "contiki.h"
#include "net/nbr-table.h"
#include "net/mac/rdc.h"
#include "net/mac/tsch-private.h"
#include "net/mac/tsch-queue.h"

#define DEBUG 0
#undef PUTCHAR
#if DEBUG
#define PUTCHAR(X) do { putchar(X); putchar('\n'); } while(0);
#if CONTIKI_TARGET_JN5168
int dbg_printf(const char *fmt, ...);
#define PRINTF(...) do {dbg_printf(__VA_ARGS__);} while(0)
#else
#define PRINTF printf
#define dbg_printf PRINTF
#endif /*CONTIKI_TARGET_JN5168*/
#else
#define PRINTF(...)
#define dbg_printf PRINTF
#define PUTCHAR(X)
#endif /* DEBUG */

/* Is this neighbor queue not empty? */
#define QUEUE_NOT_EMPTY(n) ((((((n)->put_ptr - (n)->get_ptr)) & (NBR_BUFFER_SIZE - 1)) > 0))
/* Is this neighbor queue full? */
#define QUEUE_FULL(n) ((((((n)->put_ptr - (n)->get_ptr)) & (NBR_BUFFER_SIZE - 1)) == (NBR_BUFFER_SIZE - 1)))

/* Per-neighbor queue */
NBR_TABLE(struct neighbor_queue, neighbor_queues);

/*---------------------------------------------------------------------------*/
// This function returns a pointer to the queue of neighbor whose address is equal to addr
inline struct neighbor_queue *
neighbor_queue_from_addr(const rimeaddr_t *addr)
{
	struct neighbor_queue *n = nbr_table_get_from_lladdr(neighbor_queues, addr);
	return n;
}
/*---------------------------------------------------------------------------*/
// This function adds one queue for neighbor whose address is equal to addr
// uses working_on_queue to protect data-structures from race conditions
// return 1 ok, 0 failed to allocate
struct neighbor_queue *
add_queue(const rimeaddr_t *addr)
{
	ieee154e_vars.working_on_queue = 1;
	struct neighbor_queue *n;
	/* If we have an entry for this neighbor already, we renew it. */
	n = neighbor_queue_from_addr(addr);
	if (n == NULL) {
		n = nbr_table_add_lladdr(neighbor_queues, addr);
	}
	//if n was actually allocated
	if (n) {
		/* Init neighbor entry */
		n->BE_value = MAC_MIN_BE;
		n->BW_value = 0;
		n->put_ptr = 0;
		n->get_ptr = 0;
		n->is_time_source = 0;
		uint8_t i;
		for (i = 0; i < NBR_BUFFER_SIZE; i++) {
			n->buffer[i].pkt = 0;
			n->buffer[i].transmissions = 0;
		}
		ieee154e_vars.working_on_queue = 0;
		return n;
	}
	ieee154e_vars.working_on_queue = 0;
	return n;
}
/*---------------------------------------------------------------------------*/
// This function remove the queue of neighbor whose address is equal to addr
// uses working_on_queue to protect data-structures from race conditions
// return 1 ok, 0 failed to find the queue
int
remove_queue(const rimeaddr_t *addr)
{
	ieee154e_vars.working_on_queue = 1;
	int i;
	struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
	if (n != NULL) {
		for (i = 0; i < NBR_BUFFER_SIZE; i++) {      // free packets of neighbor
			queuebuf_free(n->buffer[i].pkt);
			n->buffer[i].pkt = NULL;
		}
		nbr_table_remove(neighbor_queues, n);
		n = NULL;
		ieee154e_vars.working_on_queue = 0;
		return 1;
	}
	ieee154e_vars.working_on_queue = 0;
	return 0;
}
/*---------------------------------------------------------------------------*/
// This function adds one packet to the queue of neighbor whose address is addr
// return 1 ok, 0 failed to allocate
// the packet to be inserted is in packetbuf
int
add_packet_to_queue(mac_callback_t sent, void* ptr, const rimeaddr_t *addr)
{
  /* Check if buffer is full. If it is full, return 0 to indicate that
     the element was not inserted into the buffer.

     XXX: there is a potential risk for a race condition here, because
     the ->get_ptr field may be written concurrently by the
     ringbuf_get() function. To avoid this, access to ->get_ptr must
     be atomic. We use an uint8_t type, which makes access atomic on
     most platforms, but C does not guarantee this.
  */

	struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
	if (n != NULL) {
		//is queue full?
		if (QUEUE_FULL(n)) {
			PRINTF("queue full %x.%x.%x.%x.%x.%x.%x.%x\n", addr->u8[7],addr->u8[6],addr->u8[5],addr->u8[4],addr->u8[3],addr->u8[2],addr->u8[1],addr->u8[0]);
			/* send the callback to signal that tx failed */
			//XXX drop packet silently
//			mac_call_sent_callback(sent, ptr, MAC_TX_ERR_FATAL, 0);
			return 0;
		}
		n->buffer[n->put_ptr].pkt = queuebuf_new_from_packetbuf(); // create new packet from packetbuf
		n->buffer[n->put_ptr].ptr = ptr;
		n->buffer[n->put_ptr].ret = MAC_TX_DEFERRED;
		n->buffer[n->put_ptr].transmissions = 0;
		dbg_printf("qa0%x p%d @%x\n", addr->u8[7], n->put_ptr, n->buffer[n->put_ptr].pkt);
		if(n->buffer[n->put_ptr].pkt != NULL ) {
			n->put_ptr = (n->put_ptr + 1) & (NBR_BUFFER_SIZE - 1);
			return 1;
		}
		dbg_printf("qa failed!!\n");
	} else {
		n = add_queue(addr);
		if(n != NULL) {
			return add_packet_to_queue(sent, ptr, addr);
		}
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
// This function removes the head-packet of the queue of neighbor whose address is addr
// return 1 ok, 0 failed
// remove one packet from the queue
int
remove_packet_from_queue(const rimeaddr_t *addr)
{
	struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
	if (n != NULL) {
		if (QUEUE_NOT_EMPTY(n)) {
			uint8_t current_get_ptr = n->get_ptr;
			struct queuebuf * pkt = n->buffer[current_get_ptr].pkt;
			n->buffer[current_get_ptr].pkt = NULL;
			n->get_ptr = (n->get_ptr + 1) & (NBR_BUFFER_SIZE - 1);
			queuebuf_free(pkt);
			dbg_printf("qm0%x p%d @%x\n", addr->u8[7], current_get_ptr, pkt);
			return 1;
		}
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
/* returns the first packet in the queue of a neighbor */
const struct TSCH_packet*
read_packet_from_neighbor_queue(const struct neighbor_queue *n)
{
	if(n != NULL) {
		if (QUEUE_NOT_EMPTY(n)) {
			dbg_printf("qr p%d @%x\n", n->get_ptr, n->buffer[n->get_ptr].pkt);
			return (struct TSCH_packet*)&(n->buffer[n->get_ptr]);
		}
	}
	return NULL;
}
/*---------------------------------------------------------------------------*/
/* returns the first packet in the queue of neighbor */
const struct TSCH_packet*
read_packet_from_queue(const rimeaddr_t *addr)
{
	dbg_printf("qr0%x\n", addr->u8[7]);
	return read_packet_from_neighbor_queue( neighbor_queue_from_addr(addr) );
}
/*---------------------------------------------------------------------------*/
/* get a packet to send in a shared slot, and put the neighbor reference in n */
const struct TSCH_packet *
get_next_packet_for_shared_slot_tx(struct neighbor_queue **n)
{
	static struct neighbor_queue* last_neighbor_tx = NULL;
	if(last_neighbor_tx == NULL) {
		last_neighbor_tx = nbr_table_head(neighbor_queues);
		*n = last_neighbor_tx;
	}
	struct TSCH_packet * p = NULL;
	while(p==NULL && last_neighbor_tx != NULL) {
		*n = last_neighbor_tx;
		p = read_packet_from_neighbor_queue( last_neighbor_tx );
		last_neighbor_tx = nbr_table_next(neighbor_queues, last_neighbor_tx);
	}
	return p;
}

void
tsch_queue_init(void)
{
	nbr_table_register(neighbor_queues, NULL);
}
