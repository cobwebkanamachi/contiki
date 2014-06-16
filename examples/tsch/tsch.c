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
 *         TSCH.
 * \author
 *         Beshr Al Nahas <beshr@sics.se>
 */
#include "contiki.h"
#include "contiki-conf.h"
#include "tsch.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include "net/rime/rimestats.h"
#include <string.h>
#include "sys/rtimer.h"
#include "cooja-debug.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"
#include "dev/cc2420-tsch.h"
#include "net/mac/frame802154.h"
#include "dev/leds.h"
#include "sys/ctimer.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
//#define PUTCHAR(X) do { putchar(X); putchar('\n'); } while(0);
#else
#define PRINTF(...)
//#define PUTCHAR(X)
#endif

#ifndef True
#define True (1)
#endif

#ifndef False
#define False (0)
#endif



/* TSCH queue size: is it non-zero and a power of two? */
#if ( QUEUEBUF_CONF_NUM && !(QUEUEBUF_CONF_NUM & (QUEUEBUF_CONF_NUM-1)) )
#define NBR_BUFFER_SIZE QUEUEBUF_CONF_NUM // POWER OF 2 -- queue size
#else
#define NBR_BUFFER_SIZE 8
#endif /* !(QUEUEBUF_CONF_NUM & (QUEUEBUF_CONF_NUM-1)) */

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

static int
powercycle(struct rtimer *t, void *ptr);
#define macMinBE 1
#define macMaxFrameRetries 4
#define macMaxBE 4

// TSCH PACKET STRUCTURE
struct TSCH_packet
{
	struct queuebuf * pkt; // pointer to the packet to be sent
	uint8_t transmissions; // #transmissions performed for this packet
	mac_callback_t sent; // callback for this packet
	void *ptr; // parameters for MAC callback ... (usually NULL)
	uint8_t ret; //status -- MAC return code
};

struct neighbor_queue
{
	uint8_t is_time_source; //is this neighbor a time source?
	uint8_t BE_value; // current value of backoff exponent
	uint8_t BW_value; // current value of backoff counter
	struct TSCH_packet buffer[NBR_BUFFER_SIZE]; // circular buffer of packets. Its size should be a power of two
	uint8_t put_ptr, get_ptr; // pointers for circular buffer implementation
};
/*---------------------------------------------------------------------------*/
static const rimeaddr_t BROADCAST_CELL_ADDRESS = { { 0, 0, 0, 0, 0, 0, 0, 0 } };
static const rimeaddr_t EB_CELL_ADDRESS = { { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff} };

static const rimeaddr_t CELL_ADDRESS1 = { { 0x00, 0x12, 0x74, 01, 00, 01, 01, 01 } };
static const rimeaddr_t CELL_ADDRESS2 = { { 0x00, 0x12, 0x74, 02, 00, 02, 02, 02 } };
static const rimeaddr_t CELL_ADDRESS3 = { { 0x00, 0x12, 0x74, 03, 00, 03, 03, 03 } };
static const cell_t generic_shared_cell = { 0xffff, 0, LINK_OPTION_TX | LINK_OPTION_RX
		| LINK_OPTION_SHARED, LINK_TYPE_NORMAL, &BROADCAST_CELL_ADDRESS };
static const cell_t generic_eb_cell = { 0, 0, LINK_OPTION_TX, LINK_TYPE_ADVERTISING,
		&EB_CELL_ADDRESS };
static const cell_t cell_to_1 = { 1, 0, LINK_OPTION_TX | LINK_OPTION_RX
		| LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING, LINK_TYPE_NORMAL,
		&CELL_ADDRESS1 };
static const cell_t cell_to_2 = { 2, 0, LINK_OPTION_TX | LINK_OPTION_RX
		| LINK_OPTION_SHARED, LINK_TYPE_NORMAL, &CELL_ADDRESS2 };
static const cell_t cell_to_3 = { 3, 0, LINK_OPTION_TX | LINK_OPTION_RX
		| LINK_OPTION_SHARED, LINK_TYPE_NORMAL, &CELL_ADDRESS3 };
static const cell_t cell_3_to_2 = { 4, 0, LINK_OPTION_TX | LINK_OPTION_RX
		| LINK_OPTION_SHARED, LINK_TYPE_NORMAL, &CELL_ADDRESS2 };
#define TOTAL_LINKS 6
#define TSCH_MIN_SIZE 6
static const cell_t * minimum_cells[6] = {
		&generic_eb_cell, &generic_shared_cell,	&generic_shared_cell,
		&generic_shared_cell, &generic_shared_cell,	&generic_shared_cell	};
static const cell_t * links_list[TOTAL_LINKS] = { &generic_eb_cell, &generic_shared_cell,
		&cell_to_1, &cell_to_2, &cell_to_3, &cell_3_to_2 };
static const slotframe_t minimum_slotframe = { 0, 101, 6, minimum_cells };
/*---------------------------------------------------------------------------*/
/* variable to protect queue structure */
volatile uint8_t working_on_queue = 0;
static volatile ieee154e_vars_t ieee154e_vars;
static volatile uint8_t waiting_for_radio_interrupt = 0;
static volatile uint8_t need_ack = 0;
static volatile struct received_frame_s *last_rf = NULL;
static volatile int16_t last_drift = 0;
static volatile struct pt mpt;
static volatile struct rtimer t;
volatile unsigned char we_are_sending = 0;
static struct ctimer sync_ctimer;
/*---------------------------------------------------------------------------*/
/* NBR_TABLE_CONF_MAX_NEIGHBORS specifies the size of the table */
#include "net/nbr-table.h"
NBR_TABLE(struct neighbor_queue, neighbor_list);

static struct TSCH_packet *
get_next_packet_for_shared_tx(void);
struct neighbor_queue *
neighbor_queue_from_addr(const rimeaddr_t *addr);
struct neighbor_queue *
add_queue(const rimeaddr_t *addr);
int
remove_queue(const rimeaddr_t *addr);
int
add_packet_to_queue(mac_callback_t sent, void* ptr, const rimeaddr_t *addr);
int
remove_packet_from_queue(const rimeaddr_t *addr);
struct TSCH_packet*
read_packet_from_queue(const rimeaddr_t *addr);
/*---------------------------------------------------------------------------*/
static void tsch_timer(void *ptr);
static int make_eb(uint8_t * buf, uint8_t buf_size);
void tsch_make_sync_ack(uint8_t **buf, uint8_t seqno, rtimer_clock_t last_packet_timestamp, uint8_t nack);
static void tsch_init(void);
static void tsch_resynchronize(void);
/*---------------------------------------------------------------------------*/
/** This function takes the MSB of gcc generated random number
 * because the LSB alone has very bad random characteristics,
 * while the MSB appears more random.
 * window is the upper limit of the number. It should be a power of two - 1
 **/
#include "sys/node-id.h"

uint32_t seed_newg;
void srand_newg(uint32_t x){
     seed_newg=x;
}

static uint8_t generate_random_byte(uint8_t window) {
	// XXX this is not good enough // return (random_rand() >> 8) & window;
  seed_newg = seed_newg * 1103515245 + 12345;
  return ((uint32_t)(seed_newg / 65536) % 32768) & window;
}
/*---------------------------------------------------------------------------*/
// This function returns a pointer to the queue of neighbor whose address is equal to addr
inline struct neighbor_queue *
neighbor_queue_from_addr(const rimeaddr_t *addr)
{
	struct neighbor_queue *n = nbr_table_get_from_lladdr(neighbor_list, addr);
	return n;
}
/*---------------------------------------------------------------------------*/
// This function adds one queue for neighbor whose address is equal to addr
// uses working_on_queue to protect data-structures from race conditions
// return 1 ok, 0 failed to allocate
struct neighbor_queue *
add_queue(const rimeaddr_t *addr)
{
	working_on_queue = 1;
	struct neighbor_queue *n;
	/* If we have an entry for this neighbor already, we renew it. */
	n = neighbor_queue_from_addr(addr);
	if (n == NULL) {
		n = nbr_table_add_lladdr(neighbor_list, addr);
	}
	//if n was actually allocated
	if (n) {
		/* Init neighbor entry */
		n->BE_value = macMinBE;
		n->BW_value = 0;
		n->put_ptr = 0;
		n->get_ptr = 0;
		n->is_time_source = 0;
		uint8_t i;
		for (i = 0; i < NBR_BUFFER_SIZE; i++) {
			n->buffer[i].pkt = 0;
			n->buffer[i].transmissions = 0;
		}
		working_on_queue = 0;
		return n;
	}
	working_on_queue = 0;
	return n;
}
/*---------------------------------------------------------------------------*/
// This function remove the queue of neighbor whose address is equal to addr
// uses working_on_queue to protect data-structures from race conditions
// return 1 ok, 0 failed to find the queue
int
remove_queue(const rimeaddr_t *addr)
{
	working_on_queue = 1;
	int i;
	struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
	if (n != NULL) {
		for (i = 0; i < NBR_BUFFER_SIZE; i++) {      // free packets of neighbor
			queuebuf_free(n->buffer[i].pkt);
		}
		nbr_table_remove(neighbor_list, n);
		working_on_queue = 0;
		return 1;
	}
	working_on_queue = 0;
	return 0;
}
/*---------------------------------------------------------------------------*/
// This function adds one packet to the queue of neighbor whose address is addr
// return 1 ok, 0 failed to allocate
// the packet to be inserted is in packetbuf
int
add_packet_to_queue(mac_callback_t sent, void* ptr, const rimeaddr_t *addr)
{
	struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
	if (n != NULL) {
		//is queue full?
		if (((n->put_ptr - n->get_ptr) & (NBR_BUFFER_SIZE - 1)) == (NBR_BUFFER_SIZE - 1)) {
			return 0;
		}
		n->buffer[n->put_ptr].pkt = queuebuf_new_from_packetbuf(); // create new packet from packetbuf
		n->buffer[n->put_ptr].sent = sent;
		n->buffer[n->put_ptr].ptr = ptr;
		n->buffer[n->put_ptr].ret = MAC_TX_DEFERRED;
		n->buffer[n->put_ptr].transmissions = 0;
		n->put_ptr = (n->put_ptr + 1) & (NBR_BUFFER_SIZE - 1);
		return 1;
	} else {
		n= add_queue(addr);
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
		if (((n->put_ptr - n->get_ptr) & (NBR_BUFFER_SIZE - 1)) > 0) {
			queuebuf_free(n->buffer[n->get_ptr].pkt);
			n->get_ptr = (n->get_ptr + 1) & (NBR_BUFFER_SIZE - 1);
			return 1;
		}
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
/* returns the first packet in the queue of neighbor */
struct TSCH_packet*
read_packet_from_queue(const rimeaddr_t *addr)
{
	struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
	if (n != NULL) {
		if (((n->put_ptr - n->get_ptr) & (NBR_BUFFER_SIZE - 1)) > 0) {
			return &(n->buffer[n->get_ptr]);
		}
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
/* returns the first packet in the queue of a neighbor */
struct TSCH_packet*
read_packet_from_neighbor_queue(const struct neighbor_queue *n)
{
	if (n != NULL) {
		if (((n->put_ptr - n->get_ptr) & (NBR_BUFFER_SIZE - 1)) > 0) {
			return &(n->buffer[n->get_ptr]);
		} else {
			return 0;
		}
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
/* get a packet to send in a shared slot */
static struct TSCH_packet *
get_next_packet_for_shared_slot_tx(void) {
	static struct neighbor_queue* last_neighbor_tx = NULL;
	if(last_neighbor_tx == NULL) {
		last_neighbor_tx = nbr_table_head(neighbor_list);
	}
	struct TSCH_packet * p = NULL;
	while(p==NULL && last_neighbor_tx != NULL) {
		p = read_packet_from_neighbor_queue( last_neighbor_tx );
		last_neighbor_tx = nbr_table_next(neighbor_list, last_neighbor_tx);
	}
	return p;
}
/*---------------------------------------------------------------------------*/
// Function send for TSCH-MAC, puts the packet in packetbuf in the MAC queue
static int
send_one_packet(mac_callback_t sent, void *ptr)
{
	//send_one_packet(sent, ptr);
	COOJA_DEBUG_STR("TSCH send_one_packet\n");

	uint16_t seqno;
	const rimeaddr_t *addr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
	//Ask for ACK if we are sending anything other than broadcast
	if (!rimeaddr_cmp(addr, &rimeaddr_null)) {
		packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
	}
	/* PACKETBUF_ATTR_MAC_SEQNO cannot be zero, due to a peculiarity
	 in framer-802154.c. */
	seqno = (++ieee154e_vars.dsn) ? ieee154e_vars.dsn : ++ieee154e_vars.dsn;

	packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, seqno);
	if (NETSTACK_FRAMER.create() < 0) {
		return 0;
	}
	struct neighbor_queue *n;
	/* Look for the neighbor entry */
	n = neighbor_queue_from_addr(addr);
	if (n == NULL) {
		//add new neighbor to list of neighbors
		if (!add_queue(addr))
			return 0;
		//add new packet to neighbor list
		if (!add_packet_to_queue(sent, ptr, addr))
			return 0;
	} else {
		//add new packet to neighbor list
		if (!add_packet_to_queue(sent, ptr, addr))
			return 0;
	}
	return 1;
}
/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
	send_one_packet(sent, ptr);
}
/*---------------------------------------------------------------------------*/
static void
send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
	while (buf_list != NULL) {
		/* We backup the next pointer, as it may be nullified by
		 * mac_call_sent_callback() */
		struct rdc_buf_list *next = buf_list->next;
		int last_sent_ok;

		queuebuf_to_packetbuf(buf_list->buf);
		last_sent_ok = send_one_packet(sent, ptr);

		/* If packet transmission was not successful, we should back off and let
		 * upper layers retransmit, rather than potentially sending out-of-order
		 * packet fragments. */
		if (!last_sent_ok) {
			return;
		}
		buf_list = next;
	}
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
	COOJA_DEBUG_STR("tsch packet_input begin\n");

	int original_datalen;
	uint8_t *original_dataptr;

	original_datalen = packetbuf_datalen();
	original_dataptr = packetbuf_dataptr();
#ifdef NETSTACK_DECRYPT
	NETSTACK_DECRYPT();
#endif /* NETSTACK_DECRYPT */

	if (NETSTACK_FRAMER.parse() < 0) {
		PRINTF("tsch: failed to parse %u\n", packetbuf_datalen());
#if TSCH_ADDRESS_FILTER
	} else if (!rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
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

		if (!duplicate) {
			NETSTACK_MAC.input();
			COOJA_DEBUG_STR("tsch packet_input, Not duplicate\n");
		}
	}
//	COOJA_DEBUG_STR("tsch packet_input end\n");
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
	return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
volatile static int keep_radio_on = 0;
static int
off(int set_keep_radio_on)
{
	keep_radio_on = set_keep_radio_on;
	if (keep_radio_on) {
		return NETSTACK_RADIO.on();
	} else {
		return NETSTACK_RADIO.off();
	}
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
	return 0;
}
/*---------------------------------------------------------------------------*/
#define BUSYWAIT_UNTIL_ABS(cond, t0, duration)                     	    \
  do { rtimer_clock_t now = RTIMER_NOW(), t1=t0+duration;               \
  	if((rtimer_clock_t)(t1-now)>duration) break;												\
  	else {while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t1));}  												\
  } while(0)
/*---------------------------------------------------------------------------*/
#ifndef MIN
#define MIN(a, b) ((a) < (b)? (a) : (b))
#endif /* MIN */

/*---------------------------------------------------------------------------*/
static uint8_t
hop_channel(uint8_t offset)
{
	uint8_t channel = 11 + (offset + ieee154e_vars.asn.asn_4lsb) % 16;
	//XXX disabling channel hopping
	channel=20;
	if ( NETSTACK_RADIO_set_channel(channel)) {
		return channel;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
PROCESS(tsch_tx_callback_process, "tsch_tx_callback_process");
PROCESS(tsch_associate, "tsch_associate");
/*---------------------------------------------------------------------------*/
static cell_t *
get_cell(uint16_t timeslot)
{
	return (timeslot >= ieee154e_vars.current_slotframe->on_size) ?
	NULL : ieee154e_vars.current_slotframe->cells[timeslot];
}
/*---------------------------------------------------------------------------*/
static uint16_t
get_next_on_timeslot(uint16_t timeslot)
{
	return (timeslot >= ieee154e_vars.current_slotframe->on_size - 1) ? 0 : timeslot + 1;
}
/*---------------------------------------------------------------------------*/
/* Schedule a wakeup from a reference time for a specific duration.
 * Provides basic protection against missed deadlines and timer overflows */
static uint8_t
schedule_fixed(struct rtimer *tm, rtimer_clock_t ref_time,
		rtimer_clock_t duration)
{
	/* A non-zero status should signal to powercycle a missed deadline */
	int r, status = 0;
	rtimer_clock_t now = RTIMER_NOW() + 2;
	ref_time += duration;
	if (ref_time - now > duration+5) {
		COOJA_DEBUG_STR("schedule_fixed: missed deadline!\n");
		//XXX make it wakeup next slot instead... or add a check for missed deadlines in powercycle
		status = 1;
//		ref_time -= ref_time - now - duration;
		ref_time = now + duration + 5;
		if (ref_time - now > duration +5) {
			ref_time = now + 5;
		}
	}
	r = rtimer_set(tm, ref_time, 1, (void
	(*)(struct rtimer *, void *)) powercycle, status);
	if (r != RTIMER_OK) {
		COOJA_DEBUG_STR("schedule_fixed: could not set rtimer\n");
		status *= 2;
	}
	return status;
}

/* schedule only if deadline not passed */
static uint8_t
schedule_strict(struct rtimer *tm, rtimer_clock_t ref_time,
		rtimer_clock_t duration)
{
	/* A non-zero status should signal to powercycle a missed deadline */
	int r, status = 0;
	rtimer_clock_t now = RTIMER_NOW() + 1;
	ref_time += duration;
	if (ref_time - now > duration+5) {
		COOJA_DEBUG_STR("schedule_fixed: missed deadline!\n");
		//XXX make it wakeup next slot instead... or add a check for missed deadlines in powercycle
		status = 1;
		ref_time -= ref_time - now - duration;
		if (ref_time - now > duration+5) {
			ref_time = now + 5;
		}
	} else {
		r = rtimer_set(tm, ref_time, 1, (void
		(*)(struct rtimer *, void *)) powercycle, status);
		if (r != RTIMER_OK) {
			COOJA_DEBUG_STR("schedule_fixed: could not set rtimer\n");
			status *= 2;
		}
	}
	return status;
}

/*---------------------------------------------------------------------------*/
void
tsch_resume_powercycle(uint8_t is_ack, uint8_t need_ack_irq, struct received_frame_s * last_rf_irq)
{
	need_ack = need_ack_irq;
	last_rf = last_rf_irq;
	if (waiting_for_radio_interrupt || NETSTACK_RADIO_get_rx_end_time() != 0) {
		waiting_for_radio_interrupt = 0;
//		schedule_fixed(&t, RTIMER_NOW(), 5);
	}
	leds_off(LEDS_RED);
}
/*---------------------------------------------------------------------------*/
static int
powercycle(struct rtimer *t, void *ptr)
{
	/* if timeslot for tx, and we have a packet, call timeslot_tx
	 * else if timeslot for rx, call timeslot_rx
	 * otherwise, schedule next wakeup
	 */
	PT_BEGIN(&mpt);
	static volatile uint16_t timeslot = 0;
	static volatile int32_t drift_correction = 0;
	static volatile int32_t drift = 0; //estimated drift to all time source neighbors
	static volatile uint16_t drift_counter = 0; //number of received drift corrections source neighbors
	static uint8_t cell_decison = 0;
	static cell_t * cell = NULL;
	static struct TSCH_packet* p = NULL;
	static struct neighbor_queue *n = NULL;
	static void * payload = NULL;
	static unsigned short payload_len = 0;
	static uint8_t is_broadcast = 0, len=0, seqno=0, ret=0;
	static volatile uint8_t ackbuf[ACK_LEN + EXTRA_ACK_LEN +2];
	while (ieee154e_vars.state != TSCH_OFF) {
		timeslot = 0;
		drift_correction = 0;
		drift = 0; //estimated drift to all time source neighbors
		drift_counter = 0; //number of received drift corrections source neighbors
		cell_decison = 0;
		cell = NULL;
		p = NULL;
		n = NULL;
		payload = NULL;
		payload_len = 0;
		is_broadcast = 0; len=0; seqno=0; ret=0;

		COOJA_DEBUG_STR("Resync\n");
		PT_YIELD_UNTIL(&mpt, ieee154e_vars.state == TSCH_ASSOCIATED);
		timeslot = ieee154e_vars.asn.asn_4lsb % ieee154e_vars.current_slotframe->length;
		if(ieee154e_vars.join_priority==0) {
			ieee154e_vars.start = RTIMER_NOW();
		}
		//while MAC-RDC is not disabled, and while its synchronized
		while (ieee154e_vars.is_sync && ieee154e_vars.state == TSCH_ASSOCIATED) {
			/* sync with cycle start and enable capturing start & end sfd*/
			NETSTACK_RADIO_sfd_sync(1, 1);
			leds_on(LEDS_GREEN);
			cell = get_cell(timeslot);
			if (cell == NULL || working_on_queue) {
				COOJA_DEBUG_STR("Off CELL\n");
				//off cell
				off(keep_radio_on);
				cell_decison = CELL_OFF;
			} else {
				hop_channel(cell->channel_offset);
				payload = NULL;
				payload_len = 0;
				p = NULL;
				n = NULL;
				last_drift=0;
				last_rf = NULL;
				need_ack = 0;
				waiting_for_radio_interrupt = 0;
				//is there a packet to send? if not check if this slot is RX too
				if (cell->link_options & LINK_OPTION_TX) {
					//is it for ADV/EB?
					if (cell->link_type == LINK_TYPE_ADVERTISING) {
						//TODO fetch adv/EB packets
						n = neighbor_queue_from_addr(&EB_CELL_ADDRESS);
						if(generate_random_byte(8) >= 4) {
							payload_len = make_eb(ieee154e_vars.eb_buf, TSCH_MAX_PACKET_LEN+1); //last byte in eb buf is for length
							payload = ieee154e_vars.eb_buf;
						} else {
							COOJA_DEBUG_STR("Do not send EB");
						}
					} else { //NORMAL link
						//pick a packet from the neighbors queue who is associated with this cell
						n = neighbor_queue_from_addr(cell->node_address);
						if (n != NULL) {
							p = read_packet_from_neighbor_queue(n);
							//if there it is a shared broadcast slot and there were no broadcast packets, pick any unicast packet
							if(p==NULL
									&& rimeaddr_cmp(cell->node_address, &BROADCAST_CELL_ADDRESS)
									&& (cell->link_options & LINK_OPTION_SHARED)) {
								p = get_next_packet_for_shared_slot_tx();
							}
						}
						if(p!= NULL) {
							payload = queuebuf_dataptr(p->pkt);
							payload_len = queuebuf_datalen(p->pkt);
						}
					}
				}

				/* Decide whether it is a TX/RX/IDLE or OFF cell */
				if(cell->link_options & LINK_OPTION_TX) {
					if(payload != NULL) {
						// if dedicated slot or shared slot and BW_value=0, we transmit the packet
						if(!(cell->link_options & LINK_OPTION_SHARED)
							|| (n != NULL && n->BW_value == 0) || (cell->link_type == LINK_TYPE_ADVERTISING)) {
							cell_decison = CELL_TX;
							if (cell->link_type == LINK_TYPE_ADVERTISING || p == NULL) {
								is_broadcast = 1;
							} else if(p != NULL) {
								is_broadcast = rimeaddr_cmp(queuebuf_addr(p->pkt, PACKETBUF_ADDR_RECEIVER), &rimeaddr_null);
							}
						} else if(n != NULL && n->BW_value != 0) {
							// packet to transmit but we cannot use shared slot due to backoff counter
							n->BW_value--;
							cell_decison = CELL_TX_BACKOFF;
						}
					} else if (cell->link_type == LINK_TYPE_ADVERTISING) {
						cell_decison = CELL_RX;
					} else {
						cell_decison = CELL_TX_IDLE;
					}
				}
				if( (cell->link_options & LINK_OPTION_RX) && cell_decison != CELL_TX) {
					cell_decison = CELL_RX;
				}

//				switch (cell_decison) {
//				case CELL_TX:
//					if(cell->link_type == LINK_TYPE_ADVERTISING) {
////						PUTCHAR('E');
//					} else if(is_broadcast) {
////						PUTCHAR('B');
//					} else {
////						PUTCHAR('T');
//					}
//					break;
////				case CELL_TX_BACKOFF:
////					PUTCHAR('F');
////					break;
////				case CELL_TX_IDLE:
////					PUTCHAR('I');
////					break;
////				case CELL_RX:
////					PUTCHAR('R');
////					break;
////				case CELL_OFF:
////				default:
////					PUTCHAR('O');
////					break;
//				}

				/* execute the cell */
				if(cell_decison != CELL_TX && cell_decison != CELL_RX) {
					COOJA_DEBUG_STR("Nothing to TX or RX --> off CELL\n");
					off(keep_radio_on);
				} else if (cell_decison == CELL_TX) {
SEND_METHOD:
					COOJA_DEBUG_STR("CELL_TX");
					//TODO There are small timing variations visible in cooja, which needs tuning
					uint16_t ack_sfd_time = 0;
					rtimer_clock_t ack_sfd_rtime = 0;
					we_are_sending = 1;
					char* payload_ptr = payload;
					//read seqno from payload!
					seqno = payload_ptr[2];
					//prepare packet to send
					uint8_t success = !NETSTACK_RADIO.prepare(payload, payload_len);
					uint8_t cca_status = 1;
	#if CCA_ENABLED
					//delay before CCA
					schedule_fixed(t, ieee154e_vars.start, TsCCAOffset);
					PT_YIELD(&mpt);
					on();
					//CCA
					BUSYWAIT_UNTIL_ABS(!(cca_status |= NETSTACK_RADIO.channel_clear()),
							ieee154e_vars.start, TsCCAOffset + TsCCA);
					//there is not enough time to turn radio off
					off(keep_radio_on);
	#endif /* CCA_ENABLED */
					if (cca_status == 0) {
						success = RADIO_TX_COLLISION;
					} else {
						//delay before TX
						/* do not capture start SFD, we are only interested in SFD end which equals TX end time */
						NETSTACK_RADIO_sfd_sync(0, 1);
						schedule_fixed(t, ieee154e_vars.start, TsTxOffset - delayTx);
						PT_YIELD(&mpt);
						//to record the duration of packet tx
						static rtimer_clock_t tx_time;
						tx_time = RTIMER_NOW();
						//send packet already in radio tx buffer
						success = NETSTACK_RADIO.transmit(payload_len);
						tx_time = NETSTACK_RADIO_read_sfd_timer() - tx_time;
						//limit tx_time in case of something wrong
						tx_time = MIN(tx_time, wdDataDuration);
						off(keep_radio_on);
						if (success == RADIO_TX_OK) {
							if (!is_broadcast) {
								//wait for ack: after tx
								COOJA_DEBUG_STR("wait for ACK\n");
								/* disable capturing sfd */
								NETSTACK_RADIO_sfd_sync(0, 0);
								schedule_fixed(t, ieee154e_vars.start,
										TsTxOffset + tx_time + TsTxAckDelay - TsShortGT - delayTx);
								void
								cc2420_address_decode(uint8_t enable);
								cc2420_address_decode(0);
								PT_YIELD(&mpt);
//								BUSYWAIT_UNTIL_ABS(False,
//										ieee154e_vars.start, TsTxOffset + tx_time + TsTxAckDelay - TsShortGT - delayRx);
								waiting_for_radio_interrupt = 0;
								cca_status=0;
								on();
								BUSYWAIT_UNTIL_ABS(NETSTACK_RADIO.receiving_packet(),
										ieee154e_vars.start, TsTxOffset + tx_time + TsTxAckDelay + TsShortGT - delayRx);
								BUSYWAIT_UNTIL_ABS(!NETSTACK_RADIO.receiving_packet(),
										ieee154e_vars.start, TsTxOffset + tx_time + TsTxAckDelay + TsShortGT + wdAckDuration);
								len = NETSTACK_RADIO_read_ack(ackbuf, ACK_LEN + EXTRA_ACK_LEN);
								cc2420_address_decode(1);
								waiting_for_radio_interrupt = 0;
								if (2 == ackbuf[0] && len >= ACK_LEN && seqno == ackbuf[2]) {
									success = RADIO_TX_OK;
									ieee154e_vars.sync_timeout=0;
									uint16_t ack_status = 0;
									if (ackbuf[1] & 2) { //IE-list present?
										COOJA_DEBUG_STR("ACK IE-list present");
										if (len == ACK_LEN + EXTRA_ACK_LEN) {
											if (ackbuf[3] == 0x02 && ackbuf[4] == 0x1e) {
												ack_status = ackbuf[5];
												ack_status |= ackbuf[6] << 8;
												/* If the originator was a time source neighbor, the receiver adjusts its own clock by incorporating the
												 * 	difference into an average of the drift to all its time source neighbors. The averaging method is
												 * 	implementation dependent. If the receiver is not a clock source, the time correction is ignored.
												 */
												if (n->is_time_source) {
													COOJA_DEBUG_STR("ACK from time_source");
													/* extract time correction */
													int16_t d=0;
													//is it a negative correction?
													if(ack_status & 0x0800) {
														d = -(ack_status & 0x0fff & ~0x0800);
													} else {
														d = ack_status & 0x0fff;
													}
													drift += d;
													drift_counter++;
												}
												if (ack_status & NACK_FLAG) {
													//TODO return NACK status to upper layer
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
								waiting_for_radio_interrupt = 0;
							}
							we_are_sending = 0;
							off(keep_radio_on);
							COOJA_DEBUG_STR("end tx slot\n");
						}
					}
					/* post TX: Update CSMA control variables */
					//if it was EB then p is null and anyway there is no need to update anything
					if(p != NULL) {
						if (success == RADIO_TX_NOACK) {
							p->transmissions++;
							if (p->transmissions == macMaxFrameRetries) {
								remove_packet_from_queue(queuebuf_addr(p->pkt, PACKETBUF_ADDR_RECEIVER));
								n->BE_value = macMinBE;
								n->BW_value = 0;
							} else if ((cell->link_options & LINK_OPTION_SHARED) && !is_broadcast) {
								uint8_t window = 1 << n->BE_value;
								n->BW_value = generate_random_byte(window - 1);
								n->BE_value++;
								if (n->BE_value > macMaxBE) {
									n->BE_value = macMaxBE;
								}
							}
							ret = MAC_TX_NOACK;
						} else if (success == RADIO_TX_OK) {
							remove_packet_from_queue(queuebuf_addr(p->pkt, PACKETBUF_ADDR_RECEIVER));
							if (!read_packet_from_queue(cell->node_address)) {
								// if no more packets in the queue
								n->BW_value = 0;
								n->BE_value = macMinBE;
							} else {
								// if queue is not empty
								n->BW_value = 0;
							}
							ret = MAC_TX_OK;
						} else if (success == RADIO_TX_COLLISION) {
							p->transmissions++;
							if (p->transmissions == macMaxFrameRetries) {
								remove_packet_from_queue(queuebuf_addr(p->pkt, PACKETBUF_ADDR_RECEIVER));
								n->BE_value = macMinBE;
								n->BW_value = 0;
							} else if ((cell->link_options & LINK_OPTION_SHARED) && !is_broadcast) {
								uint8_t window = 1 << n->BE_value;
								n->BW_value = generate_random_byte(window - 1);
								n->BE_value++;
								if (n->BE_value > macMaxBE) {
									n->BE_value = macMaxBE;
								}
							}
							ret = MAC_TX_COLLISION;
						} else if (success == RADIO_TX_ERR) {
							p->transmissions++;
							if (p->transmissions == macMaxFrameRetries) {
								remove_packet_from_queue(queuebuf_addr(p->pkt, PACKETBUF_ADDR_RECEIVER));
								n->BE_value = macMinBE;
								n->BW_value = 0;
							}else if ((cell->link_options & LINK_OPTION_SHARED) && !is_broadcast) {
								uint8_t window = 1 << n->BE_value;
								n->BW_value = generate_random_byte(window - 1);
								n->BE_value++;
								if (n->BE_value > macMaxBE) {
									n->BE_value = macMaxBE;
								}
							}
							ret = MAC_TX_ERR;
						} else {
							// successful transmission
							remove_packet_from_queue(queuebuf_addr(p->pkt, PACKETBUF_ADDR_RECEIVER));
							if (!read_packet_from_queue(cell->node_address)) {
								// if no more packets in the queue
								n->BW_value = 0;
								n->BE_value = macMinBE;
							} else {
								// if queue is not empty
								n->BW_value = 0;
							}
							ret = MAC_TX_OK;
						}
						/* poll MAC TX callback */
						p->ret=ret;
						process_post(&tsch_tx_callback_process, PROCESS_EVENT_POLL, p);
					}
				} else if (cell_decison == CELL_RX) {
					/* this cell is RX cell. Check if it is used for keep alive as well*/
					if ((cell->link_options & LINK_OPTION_TIME_KEEPING) &&
							(	ieee154e_vars.join_priority != 0
								&& ieee154e_vars.sync_timeout > KEEPALIVE_TIMEOUT )) {
						// TODO
						/* Check if we need to send keep-alive request */
						//TODO fetch adv/EB packets
//						n = neighbor_queue_from_addr(&EB_CELL_ADDRESS);
//						{
//							payload_len = make_eb(ieee154e_vars.eb_buf, TSCH_MAX_PACKET_LEN+1); //last byte in eb buf is for length
//							payload = ieee154e_vars.eb_buf;
//						}
//						PUTCHAR('K');
//						//XXX send keep-alive
//						goto SEND_METHOD;
						//prepare keep-alive msg to be sent later...
					} {
						//TODO There are small timing variations visible in cooja, which needs tuning
						static uint8_t is_broadcast = 0, len, seqno, ret;
						uint16_t ack_sfd_time = 0;
						rtimer_clock_t ack_sfd_rtime = 0;
						is_broadcast = rimeaddr_cmp(cell->node_address, &rimeaddr_null);
						//wait before RX
						schedule_fixed(t, ieee154e_vars.start, TsTxOffset - TsLongGT - delayRx);
						COOJA_DEBUG_STR("schedule RX on guard time - TsLongGT");
						NETSTACK_RADIO_sfd_sync(1, 1);
						PT_YIELD(&mpt);
						//Start radio for at least guard time
						on();
						COOJA_DEBUG_STR("RX on -TsLongGT");
						uint8_t cca_status = 0;
						//Check if receiving within guard time
//						schedule_fixed(t, ieee154e_vars.start, TsTxOffset + TsLongGT);
//						PT_YIELD(&mpt);
//						COOJA_DEBUG_STR("RX on +TsLongGT");
//
//						if (!(NETSTACK_RADIO_get_rx_end_time() || cca_status || NETSTACK_RADIO.pending_packet()
//								|| !NETSTACK_RADIO.channel_clear()
//								|| NETSTACK_RADIO.receiving_packet())) {
//							off(keep_radio_on);
//							//no packets on air
//							ret = 0;
//						} else

						waiting_for_radio_interrupt = 1;

						BUSYWAIT_UNTIL_ABS(NETSTACK_RADIO.receiving_packet(),
								ieee154e_vars.start, TsTxOffset + TsLongGT);
//						BUSYWAIT_UNTIL_ABS(!NETSTACK_RADIO.receiving_packet(),
//								ieee154e_vars.start, TsTxOffset + TsLongGT + wdAckDuration);
//						off(keep_radio_on);
						int	cc2420_pending_irq(void);
						if(!NETSTACK_RADIO.receiving_packet() && !cc2420_pending_irq()) {
							off(keep_radio_on);
							//no packets on air
							ret = 0;
						} else {
							int cc2420_read_fifo(void);
							cc2420_read_fifo();
//							uint16_t expected_rx = ieee154e_vars.start + TsTxOffset;
							uint16_t rx_duration = NETSTACK_RADIO_get_rx_end_time() - last_rf->sfd_timestamp;
							off(keep_radio_on);
							/* wait until ack time */
							if (need_ack) {
//								schedule_fixed(t, NETSTACK_RADIO_get_rx_end_time(), TsTxAckDelay - delayTx);
								schedule_fixed(t, last_rf->sfd_timestamp, rx_duration + TsTxAckDelay - delayTx);
								PT_YIELD(&mpt);
								NETSTACK_RADIO_send_ack();
							}
							/* If the originator was a time source neighbor, the receiver adjusts its own clock by incorporating the
							 * 	difference into an average of the drift to all its time source neighbors. The averaging method is
							 * 	implementation dependent. If the receiver is not a clock source, the time correction is ignored.
							 */
							//drift calculated in radio_interrupt
							if (last_drift) {
								ieee154e_vars.sync_timeout=0;

								COOJA_DEBUG_PRINTF("drift seen %d\n", last_drift);
								// check the source address for potential time-source match
								n = neighbor_queue_from_addr(&last_rf->source_address);
								if(n) {COOJA_DEBUG_ADDR(&last_rf->source_address);}
								if(n != NULL && n->is_time_source) {
									// should be the average of drifts to all time sources
									drift_correction -= last_drift;
									++drift_counter;
									COOJA_DEBUG_STR("drift recorded");
								}
							}
//							if(last_rf) {
//								PUTCHAR('C');
//							}
							//XXX return length instead? or status? or something?
							ret = 1;
						}
					}
				}
			}
//			COOJA_DEBUG_PRINTF("ASN %lu TS %u", ieee154e_vars.asn.asn_4lsb, timeslot);

			uint16_t dt, duration, next_timeslot;
			next_timeslot = get_next_on_timeslot(timeslot);
			dt =
					next_timeslot ? next_timeslot - timeslot :
							ieee154e_vars.current_slotframe->length - timeslot;
			duration = dt * TsSlotDuration;
			/* increase the timeout counter because we will reset it in case of successful TX or RX */
			ieee154e_vars.sync_timeout += dt;

			/* apply sync correction on the ieee154e_vars.start of the new slotframe */
			if (!next_timeslot) {
				if(drift_counter) {
					/* convert from microseconds to rtimer ticks and take average */
					drift_correction += (drift*100)/(3051*drift_counter);
				}
				if(drift_correction) {
					COOJA_DEBUG_PRINTF("New slot frame: drift_correction %d", drift_correction);
				}	else {
					COOJA_DEBUG_STR("New slot frame");
				}
				duration += (int16_t)drift_correction;
				drift_correction = 0;
				drift=0;
				drift_counter=0;
			}

			timeslot = next_timeslot;
			//increase asn
			ieee154e_vars.asn.asn_4lsb += dt;
			if(!ieee154e_vars.asn.asn_4lsb) {
				ieee154e_vars.asn.asn_msb++;
			}
			ieee154e_vars.start += duration;

			/* check for missed deadline and skip slot accordingly in order not to corrupt the whole schedule */
			if (ieee154e_vars.start - RTIMER_NOW() > duration) {
				COOJA_DEBUG_STR("skipping slot because of missed deadline!\n");
				//go for next slot then
				next_timeslot = get_next_on_timeslot(timeslot);
				dt =
						next_timeslot ? next_timeslot - timeslot :
								ieee154e_vars.current_slotframe->length - timeslot;
				uint16_t duration2 = dt * TsSlotDuration;
				ieee154e_vars.sync_timeout += dt;
				timeslot = next_timeslot;
				//increase asn
				ieee154e_vars.asn.asn_4lsb += dt;
				if(!ieee154e_vars.asn.asn_4lsb) {
					ieee154e_vars.asn.asn_msb++;
				}
				ieee154e_vars.start -= duration;
				duration += duration2;
			} else {
				ieee154e_vars.start -= duration;
			}
			/* Check if we need to resynchronize */
			if(	ieee154e_vars.join_priority != 0
					&& ieee154e_vars.sync_timeout > RESYNCH_TIMEOUT ) {
				ieee154e_vars.is_sync = False;
				ieee154e_vars.state = TSCH_SEARCHING;
				timeslot = 0;
				/* schedule init function to run again */
				ctimer_set(&sync_ctimer, CLOCK_SECOND, &tsch_resynchronize, NULL);
				PUTCHAR('S');
			} else {
				/* skip slot if missed the deadline */
				while ( schedule_strict(t, ieee154e_vars.start, duration) ) {
					ieee154e_vars.start += duration;
					next_timeslot = get_next_on_timeslot(timeslot);
					dt =
							next_timeslot ? next_timeslot - timeslot :
									ieee154e_vars.current_slotframe->length - timeslot;
					timeslot = next_timeslot;
					duration = dt * TsSlotDuration;
					//increase asn
					ieee154e_vars.asn.asn_4lsb += dt;
					if(!ieee154e_vars.asn.asn_4lsb) {
						ieee154e_vars.asn.asn_msb++;
					}
					PUTCHAR('!');
				}
//				schedule_fixed(t, ieee154e_vars.start, duration);
				ieee154e_vars.start += duration;
				leds_off(LEDS_GREEN);
				PT_YIELD(&mpt);
			}
		}
	}
	COOJA_DEBUG_STR("TSCH is OFF!!");
	PT_END(&mpt);
}
/*---------------------------------------------------------------------------*/
/* This function adds the Sync IE from the beginning of the buffer and returns the reported drift in microseconds */
static int16_t
add_sync_IE(uint8_t* buf, int32_t time_difference_32, uint8_t nack) {
	int16_t time_difference;
	uint16_t ack_status = 0;
	uint8_t len=4;
	//do the math in 32bits to save precision
	time_difference = (time_difference_32 * 3051)/100;
	COOJA_DEBUG_PRINTF("ACK drift time_difference_32 %d, time_difference %d", time_difference_32, time_difference);
	if(time_difference >=0) {
		ack_status=time_difference & 0x07ff;
	} else {
		ack_status=((-time_difference) & 0x07ff) | 0x0800;
	}
	if(nack) {
		ack_status |= 0x8000;
	}
	buf[0] = 0x02;
	buf[1] = 0x1e;
	buf[2] = ack_status & 0xff;
	buf[3] = (ack_status >> 8) & 0xff;
	return time_difference;
}
/*---------------------------------------------------------------------------*/
/* Create an EB packet */
static int
make_eb(uint8_t * buf, uint8_t buf_size)
{
	/* XXX make sure this function does not overflow buf */
	uint16_t seqno, j=0;
	uint8_t nack = 0, len, sub_id, type, i=0, k=0;

	COOJA_DEBUG_STR("TSCH make EB");
	//fcf: 2Bytes
	//b0-2: Frame type=0 - b3: security - b4: pending - b5: AR - b6: PAN ID compression - b7: reserved
	buf[i++] = 0x00;
	//b8: seqno suppression - b9:IE-list-present=1 - b10-11: destination address mode - b12-b13:frame version=2 - b14-15: src address mode=3
	buf[i++] = 2 | 32 | 128 | 64;
	if(!ieee154e_vars.mac_ebsn) ieee154e_vars.mac_ebsn++;
	buf[i++] = ieee154e_vars.mac_ebsn++;

	/* copy src PAN ID and long src address */
  /* Source PAN ID */
	buf[i++] = IEEE802154_PANID & 0xff;
	buf[i++] = (IEEE802154_PANID >> 8) & 0xff;

  /* Source address */
	for(k = 0; k < 8; k++) {
    buf[i++] = rimeaddr_node_addr.u8[7-k];
  }
	k=0;
	/* XXX in 6top: EB length, group ID and type: leave 2 bytes for that */
	buf[i++] = 0x00;
	buf[i++] = ((1 << 1)|1)<<3; //b0-2:: left for length MSB, b3-6: GroupID, b7: Type

	/* Append TSCH sync IE */
	len = 6;
	sub_id = 0x1a;
	type = 0; //short type
	buf[i++] = len;
	buf[i++] = (sub_id << 1) | (type & 0x01);
	buf[i++] = ieee154e_vars.asn.asn_4lsb & 0xff;
	buf[i++] = ieee154e_vars.asn.asn_4lsb >> 8;
	buf[i++] = ieee154e_vars.asn.asn_4lsb >> 16;
	buf[i++] = ieee154e_vars.asn.asn_4lsb >> 24;
	buf[i++] = ieee154e_vars.asn.asn_msb;
	buf[i++] = ieee154e_vars.join_priority;

	/* Append timeslot template IE */
	len = 1;
	sub_id = 0x1c;
	type = 0; //short type
	buf[i++] = len;
	buf[i++] = (sub_id << 1) | (type & 0x01);
	buf[i++] = ieee154e_vars.slot_template_id;
	/* Optional: include the full timeslot template */
	//not today ...

	/* Append channel hopping IE */
	len = 1;
	sub_id = 0x09;
	type = 0; //short type
	buf[i++] = len;
	buf[i++] = (sub_id << 1) | (type & 0x01);
	buf[i++] = ieee154e_vars.hop_sequence_id;
	/* Optional: include the full hop sequence */
	//not today ...

	/* Append TSCH slotframe & link IE */
	if (ieee154e_vars.current_slotframe != NULL) {
			sub_id = 0x1b;
			type = 0; //short type
			len = i++; /* saving len index instead to calculate len at the end */
			// buf[i++] = 4;
			buf[i++] = (sub_id << 1) | (type & 0x01);
			buf[i++] = 1; /* number of slotframes described in EB */
			/* TSCH slotframe descriptor*/
			buf[i++] = ieee154e_vars.current_slotframe->slotframe_handle;
			buf[i++] = ieee154e_vars.current_slotframe->length;
			buf[i++] = ieee154e_vars.current_slotframe->length >> 8;
			//buf[i++] = ieee154e_vars.current_slotframe->on_size & 0xff; /* number of included cells */
			k=i++; //index of the element containing the number of included cells
			for(j=0; j < ieee154e_vars.current_slotframe->on_size && 0xff; j++) {
				/* Include cells I am listening on only */
				if(ieee154e_vars.current_slotframe->cells[j]->link_options & LINK_OPTION_RX) {
					/* increase the number of included cells */
					buf[k]++;
					/* XXX slotnumber may not be its index in the general case */
					buf[i++] = j;
					buf[i++] = j >> 8;
					/* XXX slotnumber end of comment */
					buf[i++] = ieee154e_vars.current_slotframe->cells[j]->channel_offset;
					buf[i++] = ieee154e_vars.current_slotframe->cells[j]->channel_offset >> 8;
					buf[i++] = ieee154e_vars.current_slotframe->cells[j]->link_options;
				}
			buf[len] = 4 + 5 * buf[k];
		}
	}
	buf[buf_size-1]=i;
	/* Append time correction IE */
	//Put IEs: sync, slotframe and link, timeslot and channel hopping sequence
//	if (reported_drift != 0) {
//		add_sync_IE(buf+i+3, reported_drift, nack);
//		i+=4;
//	}

	/* XXX only in 6top -- ignore ... EB length, group ID and type: leave 2 bytes for that */
	//FCF: in buf[0,1] - EBSN: in buf[2] - srcPAN: 3,4 - srcAddr: 5-12
	buf[13] = i;
	buf[14] = ((i>>8)&0x7)|(((1 << 1)|1)<<3); //b0-2:: left for length MSB, b3-6: GroupID, b7: Type
	return i;
}
/*---------------------------------------------------------------------------*/
void
tsch_wait_for_eb(uint8_t is_ack, uint8_t need_ack_irq, struct received_frame_s * last_rf_irq)
{
	uint16_t dt=0, next_timeslot=0;
	rtimer_clock_t duration=0;
	volatile softack_interrupt_exit_callback_f *interrupt_exit = tsch_wait_for_eb;
	volatile softack_make_callback_f *softack_make = NULL;
	need_ack = need_ack_irq;
	last_rf = last_rf_irq;
	COOJA_DEBUG_STR("tsch_wait_eb");
	NETSTACK_RADIO_sfd_sync(1, 1);

	if (!is_ack && last_rf_irq != NULL && (waiting_for_radio_interrupt || NETSTACK_RADIO_get_rx_end_time() != 0)) {
		if ( (FRAME802154_BEACONFRAME == (last_rf->buf[0]&7))
				&& ((last_rf->buf[1] & (2 | 32 | 128 | 64)) == (2 | 32 | 128 | 64))) {
			if ((last_rf->buf[16] & 0xfe) == 0x34) { //sync IE? (0x1a << 1) ==0 0x34
//				COOJA_DEBUG_STR("EB");
				ieee154e_vars.asn.asn_4lsb = (uint32_t)last_rf->buf[17];
				ieee154e_vars.asn.asn_4lsb += ((uint32_t)last_rf->buf[18] << (uint32_t)8);
				ieee154e_vars.asn.asn_4lsb += ((uint32_t)last_rf->buf[19] << (uint32_t)16);
				ieee154e_vars.asn.asn_4lsb += ((uint32_t)last_rf->buf[20] << (uint32_t)24);
				ieee154e_vars.asn.asn_msb = last_rf->buf[21];
				ieee154e_vars.join_priority = last_rf->buf[22]+1;
				//XXX to disable reading the packet
				last_rf->len = 0;
				//sync for next slot
				ieee154e_vars.start = last_rf->sfd_timestamp - TsTxOffset;
				/* in case of 16bit SFD timer and 32bit RTimer*/
				ieee154e_vars.start += RTIMER_NOW() & 0xffff0000;

				//we are in sync
				ieee154e_vars.is_sync = 1;
				ieee154e_vars.state = TSCH_ASSOCIATED;

				//XXX schedule to run after sometime
				/* check if missed deadline, try a new one */
				if(ieee154e_vars.current_slotframe) {
					dt=10;
				} else {
					dt = 50;
				}
				duration=0;
				do {
					ieee154e_vars.start += duration;
					duration += dt * TsSlotDuration;
					//increase asn
					ieee154e_vars.asn.asn_4lsb += dt;
					if(!ieee154e_vars.asn.asn_4lsb) {
						ieee154e_vars.asn.asn_msb++;
					}
					dt = 10;
				} while ( schedule_strict(&t, ieee154e_vars.start, duration) );
				ieee154e_vars.start += duration;
			}
		}
	}
	if(ieee154e_vars.state == TSCH_ASSOCIATED) {
		off(keep_radio_on);
		interrupt_exit = tsch_resume_powercycle;
		softack_make = tsch_make_sync_ack;
		NETSTACK_RADIO_softack_subscribe(softack_make, interrupt_exit);

		waiting_for_radio_interrupt = 0;
		we_are_sending = 0;
		//process the schedule, to create queues and find time-sources (time-keeping)
		if(!working_on_queue) {
			struct neighbor_queue *n;
			uint8_t i = 0;
			for(i=0; i<TOTAL_LINKS; i++) {
				//add queues for neighbors with tx links and for time-sources
				if( (links_list[i]->link_options & LINK_OPTION_TIME_KEEPING)
						|| (links_list[i]->link_options & LINK_OPTION_TX) ) {
					rimeaddr_t *addr = links_list[i]->node_address;
					/* Look for the neighbor entry */
					n = neighbor_queue_from_addr(addr);
					if (n == NULL) {
						//add new neighbor to list of neighbors
						n=add_queue(addr);
					}
					if( n!= NULL ) {
						n->is_time_source = (links_list[i]->link_options & LINK_OPTION_TIME_KEEPING) ? 1 : n->is_time_source;
					}
				}
			}
			/* XXX HACK this should be set in sent schedule
			 * --Set parent as timesource */
			if(last_rf) {
				struct neighbor_queue *n = neighbor_queue_from_addr(&last_rf->source_address);
				if (n == NULL) {
					//add new neighbor to list of neighbors
					n=add_queue(&last_rf->source_address);
				}
				if( n!= NULL ) {
					n->is_time_source = 1;
				}
			}
		}
	} else {
		on();
		waiting_for_radio_interrupt = 1;
	}

	leds_off(LEDS_RED);
}
/*---------------------------------------------------------------------------*/
#include "net/rpl/rpl.h"
/* on resynchronization, the node already has joined a rpl network and it is mistaking it with root */
static volatile uint8_t first_associate =0;
PROCESS_THREAD(tsch_associate, ev, data)
{
	PROCESS_BEGIN();
	/* TODO Synchronize
	 * If we are a master, start right away
	 * otherwise, wait for EBs to associate with a master
	 */
	/* Trying to get the RPL DAG, and polling until it becomes available  */
	static rpl_dag_t * my_rpl_dag = NULL;
	static uint8_t cca_status = 0;
  static struct etimer periodic;

  while (ieee154e_vars.state != TSCH_OFF) {
  	cca_status = 0;
  	my_rpl_dag = NULL;
		/* setup radio functions for intercepting EB */
		NETSTACK_RADIO_softack_subscribe(NULL, tsch_wait_for_eb);
		etimer_set(&periodic, CLOCK_SECOND/10);
		while(ieee154e_vars.state == TSCH_SEARCHING) {
			PROCESS_YIELD();
			timer_reset(&periodic);
			NETSTACK_RADIO_sfd_sync(1, 1);
//			COOJA_DEBUG_STR("waiting for RPL dag");

			/*If I am root (rank==1), then exit association process*/
			rpl_instance_t* rpl = rpl_get_instance(RPL_DEFAULT_INSTANCE);
			if(rpl != NULL) {
				my_rpl_dag = rpl->current_dag;
				if(my_rpl_dag != NULL) {
					COOJA_DEBUG_PRINTF("rank %d", my_rpl_dag->rank);

					ieee154e_vars.join_priority = (my_rpl_dag->rank) >> 16;
					/* to make sure that this is a root and not just a low ranked node */
					if(ieee154e_vars.join_priority == 0 && my_rpl_dag->rank > 1) {
						//XXX choose something else?
						if(!first_associate /* || my_rpl_dag->rank == 256 ??*/) {
							ieee154e_vars.join_priority = 0;
						} else {
							ieee154e_vars.join_priority = 0xf0;
						}
					}
				}
			}
			//XXX there should be a better way of handling rpl, and erasing rank, dag etc. on resync
			first_associate = 1;
			//if this is root start now
			if(ieee154e_vars.join_priority == 0) {
				COOJA_DEBUG_STR("rpl root");
				//for now assume we are in sync
				ieee154e_vars.is_sync = 1;
				//something other than 0 for now
				ieee154e_vars.state = TSCH_ASSOCIATED;
				//make queues and data structures
				tsch_wait_for_eb(0,0,NULL);
				NETSTACK_RADIO_softack_subscribe(tsch_make_sync_ack, tsch_resume_powercycle);
				ieee154e_vars.start = RTIMER_NOW();
				schedule_fixed(&t, ieee154e_vars.start, 5);
				COOJA_DEBUG_STR("associate done");
				// XXX for debugging
				ieee154e_vars.asn.asn_4lsb = 0;
			} else {
				waiting_for_radio_interrupt = 1;
				NETSTACK_RADIO_softack_subscribe(NULL, tsch_wait_for_eb);
				on();

				//TODO hop channel after timeout
				//... set_channel...
			}
		}
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
  }
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
volatile uint8_t ackbuf[1+ACK_LEN + EXTRA_ACK_LEN]={0};
void tsch_make_sync_ack(uint8_t **buf, uint8_t seqno, rtimer_clock_t last_packet_timestamp, uint8_t nack) {
	int32_t time_difference_32;
	COOJA_DEBUG_STR("tsch_make_sync_ack");
	*buf=ackbuf;
	/* calculating sync in rtimer ticks */
	time_difference_32 = (int32_t)ieee154e_vars.start + TsTxOffset - last_packet_timestamp;
	last_drift = time_difference_32;
	/* ackbuf[1+ACK_LEN + EXTRA_ACK_LEN] = {ACK_LEN + EXTRA_ACK_LEN + AUX_LEN, 0x02, 0x00, seqno, 0x02, 0x1e, ack_status_LSB, ack_status_MSB}; */
	ackbuf[1] = 0x02; /* ACK frame */
	ackbuf[3] = seqno;
	ackbuf[2] = 0x00; /* b9:IE-list-present=1 - b12-b13:frame version=2 */
	ackbuf[0] = 3; /*length*/
	/* Append IE timesync */
	ackbuf[2] = 0x22; /* b9:IE-list-present=1 - b12-b13:frame version=2 */
	add_sync_IE(&ackbuf[4], time_difference_32, nack);
	ackbuf[0] = 3 /*FCF 2B + SEQNO 1B*/ + 4 /* sync IE size */;
}
/*---------------------------------------------------------------------------*/
static void
tsch_init(void)
{
	COOJA_DEBUG_STR("tsch_init");
	//setting seed for the random generator
	srand_newg(node_id * node_id * node_id);
	NETSTACK_RADIO_softack_subscribe(NULL, NULL);
	//look for a root to sync with
	ieee154e_vars.current_slotframe = &minimum_slotframe;
	ieee154e_vars.slot_template_id = 1;
	ieee154e_vars.hop_sequence_id = 1;
	ieee154e_vars.asn.asn_4lsb = 0;
	ieee154e_vars.asn.asn_msb = 0;
	ieee154e_vars.captured_time = 0;
	ieee154e_vars.dsn = 0;
	ieee154e_vars.state = TSCH_SEARCHING;
	//we need to sync
	ieee154e_vars.is_sync = 0;
	ieee154e_vars.sync_timeout = 0; //30sec/slotDuration - (asn-asn0)*slotDuration
	ieee154e_vars.mac_ebsn = 0;
	ieee154e_vars.join_priority = 0xff; /* inherit from RPL - PAN coordinator: 0 -- lower is better */
	nbr_table_register(neighbor_list, NULL);
	working_on_queue = 0;
	process_start(&tsch_tx_callback_process, NULL);
	NETSTACK_RADIO_sfd_sync(True, True);
	//try to associate to a network or start one if setup as RPL root
	process_start(&tsch_associate, NULL);
	//XXX for debugging
//	NETSTACK_RADIO.on();
	hop_channel(0);

	powercycle(&t, NULL);
}
/*---------------------------------------------------------------------------*/
/* a polled-process to invoke the MAC tx callback asynchronously */
PROCESS_THREAD(tsch_tx_callback_process, ev, data)
{
	PROCESS_BEGIN();
	PRINTF("tsch_tx_callback_process: started\n");
	COOJA_DEBUG_STR("tsch_tx_callback_process: started\n");

	while (1) {
		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
		PRINTF("tsch_tx_callback_process: calling mac tx callback\n");
		COOJA_DEBUG_STR("tsch_tx_callback_process: calling mac tx callback\n");
		if(data != NULL) {
			struct TSCH_packet* p = (struct TSCH_packet*) data;
			/* XXX callback -- do we need to restore the packet to packetbuf? */
			mac_call_sent_callback(p->sent, p->ptr, p->ret, p->transmissions);
		}
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static void
tsch_resynchronize(void) {
	off(keep_radio_on);
	NETSTACK_RADIO_softack_subscribe(NULL, NULL);
	//look for a root to sync with
	ieee154e_vars.current_slotframe = &minimum_slotframe;
	ieee154e_vars.slot_template_id = 1;
	ieee154e_vars.hop_sequence_id = 1;
	ieee154e_vars.asn.asn_4lsb = 0;
	ieee154e_vars.asn.asn_msb = 0;
	ieee154e_vars.captured_time = 0;
	ieee154e_vars.dsn = 0;
	ieee154e_vars.state = TSCH_SEARCHING;
	//we need to sync
	ieee154e_vars.is_sync = 0;
	ieee154e_vars.sync_timeout = 0; //30sec/slotDuration - (asn-asn0)*slotDuration
	ieee154e_vars.mac_ebsn = 0;
	ieee154e_vars.join_priority = 0xff; /* inherit from RPL - PAN coordinator: 0 -- lower is better */
	working_on_queue = 0;
	NETSTACK_RADIO_sfd_sync(True, True);
	//try to associate to a network or start one if setup as RPL root
	process_post(&tsch_associate, PROCESS_EVENT_POLL, NULL);
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver tschrdc_driver = {
	"tschrdc",
	tsch_init,
	send_packet,
	send_list,
	packet_input,
	on,
	off,
	channel_check_interval,
};
/*---------------------------------------------------------------------------*/
