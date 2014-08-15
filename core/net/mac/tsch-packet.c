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
 *         TSCH packet format management
 * \author
 *         Beshr Al Nahas <beshr@sics.se>
 */

#include "contiki.h"
#include "net/mac/tsch.h"
#include "net/mac/tsch-packet.h"
#include "net/mac/tsch-private.h"
#include "net/mac/frame802154.h"
/* TODO: remove dependencies to RPL */
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"

/*---------------------------------------------------------------------------*/
/* This function adds the Sync IE from the beginning of the buffer and returns the reported drift in microseconds */
int16_t
add_sync_IE(uint8_t* buf, int32_t time_difference_32, uint8_t nack) {
	int16_t time_difference;
	uint16_t ack_status = 0;
	time_difference = RTIMER_TO_US(time_difference_32);
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
void tsch_make_sync_ack(uint8_t **buf, uint8_t seqno, rtimer_clock_t last_packet_timestamp, uint8_t nack) {
	int32_t time_difference_32;
	COOJA_DEBUG_STR("tsch_make_sync_ack");
	*buf=ieee154e_vars.ackbuf;
	/* calculating sync in rtimer ticks */
	time_difference_32 = (int32_t)ieee154e_vars.start + TsTxOffset - last_packet_timestamp;
	ieee154e_vars.registered_drift = time_difference_32;
	/* ackbuf[1+ACK_LEN + EXTRA_ACK_LEN] = {ACK_LEN + EXTRA_ACK_LEN + AUX_LEN, 0x02, 0x00, seqno, 0x02, 0x1e, ack_status_LSB, ack_status_MSB}; */
	ieee154e_vars.ackbuf[1] = 0x02; /* ACK frame */
	ieee154e_vars.ackbuf[3] = seqno;
	ieee154e_vars.ackbuf[2] = 0x00; /* b9:IE-list-present=1 - b12-b13:frame version=2 */
	ieee154e_vars.ackbuf[0] = 3; /*length*/
	/* Append IE timesync */
	add_sync_IE(&(ieee154e_vars.ackbuf[4]), time_difference_32, nack);
	ieee154e_vars.ackbuf[0] = 7; /* Len: FCF 2B + SEQNO 1B + sync IE 4B*/
	ieee154e_vars.ackbuf[2] = 0x22; /* b9:IE-list-present=1 - b12-b13:frame version=2 */
}
/*---------------------------------------------------------------------------*/
/* Create an EB packet */
int
make_eb(uint8_t * buf, uint8_t buf_size)
{
	/* XXX make sure this function does not overflow buf */
	uint16_t j=0;
	uint8_t len, sub_id, type, i=0, k=0;

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
	for(k = RIMEADDR_SIZE; k > 0; k--) {
    buf[i++] = rimeaddr_node_addr.u8[k-1];
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
	buf[i++] = ieee154e_vars.asn.asn_4lsb >> (uint16_t)16;
	buf[i++] = ieee154e_vars.asn.asn_4lsb >> (uint32_t)24;
	buf[i++] = ieee154e_vars.asn.asn_msb;
	/* update join_priority */
	rpl_instance_t* rpl = rpl_get_instance(RPL_DEFAULT_INSTANCE);
	static rpl_dag_t * my_rpl_dag = NULL;
	if (rpl != NULL) {
		my_rpl_dag = rpl->current_dag;
		if (my_rpl_dag != NULL) {
			ieee154e_vars.join_priority = (my_rpl_dag->rank) >> 8;
		}
	}

	if(tsch_is_coordinator) {
		ieee154e_vars.join_priority = 0;
	}


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
			for(j=0; j < (ieee154e_vars.current_slotframe->on_size & 0xff); j++) {
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
