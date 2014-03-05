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

#include "contiki-conf.h"
#include "tsch.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include "net/rime/rimestats.h"
#include <string.h>
#include "sys/rtimer.h"

static volatile ieee154e_vars_t ieee154e_vars;

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef TSCH_CONF_ADDRESS_FILTER
#define TSCH_ADDRESS_FILTER TSCH_CONF_ADDRESS_FILTER
#else
#define TSCH_ADDRESS_FILTER 1
#endif /* TSCH_CONF_ADDRESS_FILTER */

#ifndef TSCH_802154_AUTOACK
#ifdef TSCH_CONF_802154_AUTOACK
#define TSCH_802154_AUTOACK TSCH_CONF_802154_AUTOACK
#else
#define TSCH_802154_AUTOACK 0
#endif /* TSCH_CONF_802154_AUTOACK */
#endif /* TSCH_802154_AUTOACK */

#ifndef TSCH_802154_AUTOACK_HW
#ifdef TSCH_CONF_802154_AUTOACK_HW
#define TSCH_802154_AUTOACK_HW TSCH_CONF_802154_AUTOACK_HW
#else
#define TSCH_802154_AUTOACK_HW 0
#endif /* TSCH_CONF_802154_AUTOACK_HW */
#endif /* TSCH_802154_AUTOACK_HW */

#if TSCH_802154_AUTOACK
#include "sys/rtimer.h"
#include "dev/watchdog.h"

#ifdef TSCH_CONF_ACK_WAIT_TIME
#define ACK_WAIT_TIME TSCH_CONF_ACK_WAIT_TIME
#else /* TSCH_CONF_ACK_WAIT_TIME */
#define ACK_WAIT_TIME                      RTIMER_SECOND / 2500
#endif /* TSCH_CONF_ACK_WAIT_TIME */
#ifdef TSCH_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#define AFTER_ACK_DETECTED_WAIT_TIME TSCH_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#else /* TSCH_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#define AFTER_ACK_DETECTED_WAIT_TIME       RTIMER_SECOND / 1500
#endif /* TSCH_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#endif /* TSCH_802154_AUTOACK */

#ifdef TSCH_CONF_SEND_802154_ACK
#define TSCH_SEND_802154_ACK TSCH_CONF_SEND_802154_ACK
#else /* TSCH_CONF_SEND_802154_ACK */
#define TSCH_SEND_802154_ACK 0
#endif /* TSCH_CONF_SEND_802154_ACK */

#if TSCH_SEND_802154_ACK
#include "net/mac/frame802154.h"
#endif /* TSCH_SEND_802154_ACK */

#define ACK_LEN 3

#if TSCH_802154_AUTOACK || TSCH_802154_AUTOACK_HW
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
#endif /* TSCH_802154_AUTOACK || TSCH_802154_AUTOACK_HW */

/*---------------------------------------------------------------------------*/
static int
send_one_packet(mac_callback_t sent, void *ptr)
{
  int ret;
  int last_sent_ok = 0;

  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);
#if TSCH_802154_AUTOACK || TSCH_802154_AUTOACK_HW
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
#endif /* TSCH_802154_AUTOACK || TSCH_802154_AUTOACK_HW */

  if(NETSTACK_FRAMER.create() < 0) {
    /* Failed to allocate space for headers */
    PRINTF("nullrdc: send failed, too large header\n");
    ret = MAC_TX_ERR_FATAL;
  } else {

#ifdef NETSTACK_ENCRYPT
    NETSTACK_ENCRYPT();
#endif /* NETSTACK_ENCRYPT */

#if TSCH_802154_AUTOACK
    int is_broadcast;
    uint8_t dsn;
    dsn = ((uint8_t *)packetbuf_hdrptr())[2] & 0xff;

    NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen());

    is_broadcast = rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                &rimeaddr_null);

    if(NETSTACK_RADIO.receiving_packet() ||
       (!is_broadcast && NETSTACK_RADIO.pending_packet())) {

      /* Currently receiving a packet over air or the radio has
         already received a packet that needs to be read before
         sending with auto ack. */
      ret = MAC_TX_COLLISION;
    } else {
      if(!is_broadcast) {
        RIMESTATS_ADD(reliabletx);
      }

      switch(NETSTACK_RADIO.transmit(packetbuf_totlen())) {
      case RADIO_TX_OK:
        if(is_broadcast) {
          ret = MAC_TX_OK;
        } else {
          rtimer_clock_t wt;

          /* Check for ack */
          wt = RTIMER_NOW();
          watchdog_periodic();
          while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + ACK_WAIT_TIME)) {
#if CONTIKI_TARGET_COOJA
            simProcessRunValue = 1;
            cooja_mt_yield();
#endif /* CONTIKI_TARGET_COOJA */
          }

          ret = MAC_TX_NOACK;
          if(NETSTACK_RADIO.receiving_packet() ||
             NETSTACK_RADIO.pending_packet() ||
             NETSTACK_RADIO.channel_clear() == 0) {
            int len;
            uint8_t ackbuf[ACK_LEN];

            if(AFTER_ACK_DETECTED_WAIT_TIME > 0) {
              wt = RTIMER_NOW();
              watchdog_periodic();
              while(RTIMER_CLOCK_LT(RTIMER_NOW(),
                                    wt + AFTER_ACK_DETECTED_WAIT_TIME)) {
      #if CONTIKI_TARGET_COOJA
                  simProcessRunValue = 1;
                  cooja_mt_yield();
      #endif /* CONTIKI_TARGET_COOJA */
              }
            }

            if(NETSTACK_RADIO.pending_packet()) {
              len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
              if(len == ACK_LEN && ackbuf[2] == dsn) {
                /* Ack received */
                RIMESTATS_ADD(ackrx);
                ret = MAC_TX_OK;
              } else {
                /* Not an ack or ack not for us: collision */
                ret = MAC_TX_COLLISION;
              }
            }
          } else {
	    PRINTF("nullrdc tx noack\n");
	  }
        }
        break;
      case RADIO_TX_COLLISION:
        ret = MAC_TX_COLLISION;
        break;
      default:
        ret = MAC_TX_ERR;
        break;
      }
    }

#else /* ! TSCH_802154_AUTOACK */

    switch(NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen())) {
    case RADIO_TX_OK:
      ret = MAC_TX_OK;
      break;
    case RADIO_TX_COLLISION:
      ret = MAC_TX_COLLISION;
      break;
    case RADIO_TX_NOACK:
      ret = MAC_TX_NOACK;
      break;
    default:
      ret = MAC_TX_ERR;
      break;
    }

#endif /* ! TSCH_802154_AUTOACK */
  }
  if(ret == MAC_TX_OK) {
    last_sent_ok = 1;
  }
  mac_call_sent_callback(sent, ptr, ret, 1);
  return last_sent_ok;
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
  while(buf_list != NULL) {
    /* We backup the next pointer, as it may be nullified by
     * mac_call_sent_callback() */
    struct rdc_buf_list *next = buf_list->next;
    int last_sent_ok;

    queuebuf_to_packetbuf(buf_list->buf);
    last_sent_ok = send_one_packet(sent, ptr);

    /* If packet transmission was not successful, we should back off and let
     * upper layers retransmit, rather than potentially sending out-of-order
     * packet fragments. */
    if(!last_sent_ok) {
      return;
    }
    buf_list = next;
  }
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
  int original_datalen;
  uint8_t *original_dataptr;

  original_datalen = packetbuf_datalen();
  original_dataptr = packetbuf_dataptr();
#ifdef NETSTACK_DECRYPT
    NETSTACK_DECRYPT();
#endif /* NETSTACK_DECRYPT */

#if TSCH_802154_AUTOACK
  if(packetbuf_datalen() == ACK_LEN) {
    /* Ignore ack packets */
    PRINTF("nullrdc: ignored ack\n");
  } else
#endif /* TSCH_802154_AUTOACK */
  if(NETSTACK_FRAMER.parse() < 0) {
    PRINTF("nullrdc: failed to parse %u\n", packetbuf_datalen());
#if TSCH_ADDRESS_FILTER
  } else if(!rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                         &rimeaddr_node_addr) &&
            !rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                          &rimeaddr_null)) {
    PRINTF("nullrdc: not for us\n");
#endif /* TSCH_ADDRESS_FILTER */
  } else {
    int duplicate = 0;

#if TSCH_802154_AUTOACK || TSCH_802154_AUTOACK_HW
    /* Check for duplicate packet by comparing the sequence number
       of the incoming packet with the last few ones we saw. */
    int i;
    for(i = 0; i < MAX_SEQNOS; ++i) {
      if(packetbuf_attr(PACKETBUF_ATTR_PACKET_ID) == received_seqnos[i].seqno &&
         rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_SENDER),
                      &received_seqnos[i].sender)) {
        /* Drop the packet. */
        PRINTF("nullrdc: drop duplicate link layer packet %u\n",
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
#endif /* TSCH_802154_AUTOACK */

#if TSCH_SEND_802154_ACK
    {
      frame802154_t info154;
      frame802154_parse(original_dataptr, original_datalen, &info154);
      if(info154.fcf.frame_type == FRAME802154_DATAFRAME &&
         info154.fcf.ack_required != 0 &&
         rimeaddr_cmp((rimeaddr_t *)&info154.dest_addr,
                      &rimeaddr_node_addr)) {
        uint8_t ackdata[ACK_LEN] = {0, 0, 0};

        ackdata[0] = FRAME802154_ACKFRAME;
        ackdata[1] = 0;
        ackdata[2] = info154.seq;
        NETSTACK_RADIO.send(ackdata, ACK_LEN);
      }
    }
#endif /* TSCH_SEND_ACK */
    if(!duplicate) {
      NETSTACK_MAC.input();
    }
  }
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int
off(int keep_radio_on)
{
  if(keep_radio_on) {
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
/**
MLME-SET-SLOTFRAME.request (
	slotframeHandle,
	operation,
	size
)

MLME-SET-SLOTFRAME.confirm (
	slotframeHandle,
	status
)
*/


/**
MLME-SET-LINK.request (
	operation,
	linkHandle,
	slotframeHandle,
	timeslot,
	channelOffset,
	linkOptions,
	linkType,
	nodeAddr
	)

MLME-SET-LINK.confirm (
	status,
	linkHandle,
	slotframeHandle
)
*/

/**
MLME-TSCH-MODE.request (
	TSCHModeOnOff
)

MLME-TSCH-MODE.confirm (
TSCHModeOnOff,
status
)
*/

/**
MLME-KEEP-ALIVE.request (
	dstAddr,
	keepAlivePeriod
)

MLME-KEEP-ALIVE.confirm (
	status
)
 */

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

#define BUSYWAIT_UNTIL_ABS(cond, t0)                              	    \
  do {                                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0));  							\
  } while(0)

static struct rtimer rt;

#ifndef MIN
#define MIN(a, b) ((a) < (b)? (a) : (b))
#endif /* MIN */
static int keep_radio_on=0;
int cc2420_set_channel(int c);
#define NETSTACK_RADIO_set_channel cc2420_set_channel

/* to get last packet timing information */
extern volatile uint16_t cc2420_sfd_start_time;

static uint16_t
get_sfd_start_time(void) {
	return cc2420_sfd_start_time;
}

static uint8_t
hop_channel(uint8_t offset) {
	uint8_t channel = 11 + (offset + ieee154e_vars.asn) % 16;
	if( NETSTACK_RADIO_set_channel(channel) ) {
		return channel;
	}
	return 0;
}

#define BROADCAST_CELL_ADDRESS ((uint16_t)(0xffff))

const cell_t generic_shared_cell = {
		BROADCAST_CELL_ADDRESS,
		0,
		LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED,
		LINK_TYPE_NORMAL,
		BROADCAST_CELL_ADDRESS
};

const cell_t generic_eb_cell = {
		0,
		0,
		LINK_OPTION_TX,
		LINK_TYPE_ADVERTISING,
		BROADCAST_CELL_ADDRESS
};

const cell_t * minimum_cells[6] = {
	&generic_eb_cell,
	&generic_shared_cell,
	&generic_shared_cell,
	&generic_shared_cell,
	&generic_shared_cell,
	&generic_shared_cell,
};

const slotframe_t minimum_slotframe = {
	0,
	101,
	6,
	minimum_cells
};

static slotframe_t * current_slotframe;

cell_t *
get_cell(uint16_t timeslot) {
	return (timeslot >= current_slotframe->on_size)	?
			NULL :
			current_slotframe->cells[timeslot];
}

volatile unsigned char we_are_sending = 0;

int
timeslot_tx(cell_t * cell, const void * payload, unsigned short payload_len)
{
	//TODO There are small timing variations visible in cooja, which needs tuning
	rtimer_clock_t start = RTIMER_NOW();
	uint8_t is_broadcast =0, len, seqno;
	uint16_t ack_sfd_time = 0;
	rtimer_clock_t ack_sfd_rtime = 0;

	if(cell == NULL) {
		//off cell
		off(keep_radio_on);
		return MAC_TX_DEFERRED;
	}
	we_are_sending=1;
	hop_channel(cell->channel_offset);

	//XXX read seqno from payload not packetbuf!!
	seqno = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
	ieee154e_vars.dsn=seqno;
	//prepare packet to send
	uint8_t success = !NETSTACK_RADIO.prepare(payload, payload_len);
	//delay before CCA
	while(RTIMER_CLOCK_LT(RTIMER_NOW(), start + TsCCAOffset));
	on();
	//CCA
	uint8_t cca_status=0;
	BUSYWAIT_UNTIL_ABS(!(cca_status |= NETSTACK_RADIO.channel_clear()), start + TsCCAOffset + TsCCA);
	off(keep_radio_on);
	if(cca_status == 0) {
		return MAC_TX_COLLISION;
  }
	//delay before TX
	while(RTIMER_CLOCK_LT(RTIMER_NOW(), start + TsTxOffset - delayTx));
	//on();
	//send
	success = NETSTACK_RADIO.transmit(payload_len);
	rtimer_clock_t tx_end_time = RTIMER_NOW();
	off(keep_radio_on);
	if(success != RADIO_TX_OK) {
		//failed

		return MAC_TX_COLLISION;
	}

	//delay wait for ack: after tx
	while(RTIMER_CLOCK_LT(RTIMER_NOW(), MIN(tx_end_time, start + TsTxOffset + wdDataDuration) + TsTxAckDelay - TsShortGT));
	on();
	//wait for detecting ACK
	while(RTIMER_CLOCK_LT(RTIMER_NOW(), MIN(tx_end_time, start + TsTxOffset + wdDataDuration) + TsTxAckDelay + TsShortGT)) {
		if(!is_broadcast && (NETSTACK_RADIO.receiving_packet() ||
												 NETSTACK_RADIO.pending_packet() ||
												 NETSTACK_RADIO.channel_clear() == 0)) {
			uint8_t ackbuf[ACK_LEN];
			ack_sfd_rtime = RTIMER_NOW();
			ack_sfd_time = get_sfd_start_time();
			while(RTIMER_CLOCK_LT(RTIMER_NOW(), ack_sfd_rtime + wdAckDuration)) { }

			len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
			if(len == ACK_LEN && seqno == ackbuf[2]) {
				success = RADIO_TX_OK;
				break;
			} else {
				success = RADIO_TX_NOACK;
				break;
			}
		}
  }
	we_are_sending=0;
	off(keep_radio_on);
	if(success == RADIO_TX_NOACK) {
		return MAC_TX_NOACK;
	} else if(success == RADIO_TX_OK) {
		//TODO synchronize using ack_sfd_rtime or ack_sfd_time
		return MAC_TX_OK;
	}
	return MAC_TX_OK;
}

//to initialize radio sfd counter and synchronize it with rtimer
void cc2420_arch_sfd_sync(void);

//extern volatile uint16_t expected_rx_time;
#include "cooja-debug.h"

int
timeslot_rx(cell_t * cell, const void * payload, unsigned short payload_len)
{
	//TODO There are small timing variations visible in cooja, which needs tuning
	rtimer_clock_t start = RTIMER_NOW();
	uint8_t is_broadcast =0, len, seqno;
	uint16_t ack_sfd_time = 0;
	rtimer_clock_t ack_sfd_rtime = 0;

	if(cell == NULL) {
		//off cell
		off(keep_radio_on);
		return 0;
	}
	cc2420_arch_sfd_sync();

	hop_channel(cell->channel_offset);
	//wait before RX
	while(RTIMER_CLOCK_LT(RTIMER_NOW(), start + TsTxOffset - TsLongGT));
	//Start radio for at least guard time
	on();
//	COOJA_DEBUG_STR("RX on -TsLongGT");
//	uint8_t cca_status=0;
//	//Check if receiving within guard time
//	BUSYWAIT_UNTIL_ABS(
//			(cca_status=(
//					NETSTACK_RADIO.channel_clear()
//					//|| NETSTACK_RADIO.pending_packet()
//					|| NETSTACK_RADIO.receiving_packet())
//				),
//				start + TsTxOffset + TsLongGT);
//	if(!cca_status) {
//		COOJA_DEBUG_STR("RX no packet in air\n");
//		off(keep_radio_on);
//		//no packets on air
//		return 0;
//	}
	//wait until rx finishes
//	BUSYWAIT_UNTIL_ABS(!NETSTACK_RADIO.receiving_packet(), start + TsTxOffset + wdDataDuration);
//	rtimer_clock_t rx_end_time = RTIMER_NOW();
	//prepare ack
	//XXX ... in interrupt?
	//off();
	//wait until ask time comes
	//BUSYWAIT_UNTIL_ABS(!NETSTACK_RADIO.receiving_packet(), rx_end_time + TsTxAckDelay);
	//send ack

	//off
	//while(RTIMER_CLOCK_LT(RTIMER_NOW(), start + TsTxOffset + wdDataDuration + TsTxAckDelay + wdAckDuration));
	//COOJA_DEBUG_STR("!RX TIME OUT");
//	off(keep_radio_on);

}

static char
powercycle(struct rtimer *t, void *ptr) {
/* if timeslot for tx, and we have a packet, call timeslot_tx
 * else if timeslot for rx, call timeslot_rx
 * otherwise, schedule next wakeup
 */

	uint16_t timeslot = ieee154e_vars.asn++ % current_slotframe->length;
	cell_t * cell = get_cell(timeslot);

}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
	current_slotframe = &minimum_slotframe;
	ieee154e_vars.asn=0;
	ieee154e_vars.capturedTime=0;
	ieee154e_vars.dsn=0;
	ieee154e_vars.isSync=0;
	ieee154e_vars.state = 0;
	ieee154e_vars.syncTimeout = 0; //30sec/slotDuration - (asn-asn0)*slotDuration
	//schedule next wakeup? or leave for higher layer to decide? i.e, scan, ...
  //on();
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver tschrdc_driver = {
  "tschrdc",
  init,
  send_packet,
  send_list,
  packet_input,
  on,
  off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/
