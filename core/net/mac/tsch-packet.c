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
 *         Simon Duquennoy <simonduq@sics.se>
 */

#include "contiki.h"
#include "net/mac/tsch.h"
#include "net/mac/tsch-packet.h"
#include "net/mac/tsch-private.h"
#include "net/mac/tsch-schedule.h"
#include "net/mac/frame802154.h"
/* TODO: remove dependencies to RPL */
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"

/*---------------------------------------------------------------------------*/
/* This function adds the Sync IE from the beginning of the buffer and returns the reported drift in microseconds */
static int16_t
tsch_packet_add_sync_IE(uint8_t *buf, int32_t time_difference_32, uint8_t nack)
{
  int16_t time_difference;
  uint16_t ack_status = 0;
  time_difference = RTIMER_TO_US(time_difference_32);
  COOJA_DEBUG_PRINTF("ACK drift time_difference_32 %d, time_difference %d", time_difference_32, time_difference);
  if(time_difference >= 0) {
    ack_status = time_difference & 0x07ff;
  } else {
    ack_status = ((-time_difference) & 0x07ff) | 0x0800;
  } if(nack) {
    ack_status |= 0x8000;
  }
  buf[0] = 0x02;
  buf[1] = 0x1e;
  buf[2] = ack_status & 0xff;
  buf[3] = (ack_status >> 8) & 0xff;
  return time_difference;
}

/* Construct enhanced ACK packet and return ACK length */
uint8_t
tsch_packet_make_sync_ack(int32_t sync_time, uint8_t *ackbuf, uint8_t seqno, uint8_t nack)
{
  COOJA_DEBUG_STR("tsch_packet_make_sync_ack");
  /* calculating sync in rtimer ticks */
  /* ackbuf[1+ACK_LEN + EXTRA_ACK_LEN] = {ACK_LEN + EXTRA_ACK_LEN + AUX_LEN, 0x02, 0x00, seqno, 0x02, 0x1e, ack_status_LSB, ack_status_MSB}; */
  ackbuf[0] = 0x02; /* ACK frame */
  ackbuf[2] = seqno;
  ackbuf[1] = 0x22; /* b9:IE-list-present=1 - b12-b13:frame version=2 */

  /* Append IE timesync */
  tsch_packet_add_sync_IE(&(ackbuf[3]), sync_time, nack);

  return 7; /* Len: FCF 2B + SEQNO 1B + sync IE 4B*/
}
/*---------------------------------------------------------------------------*/
/* Create an EB packet */
int
tsch_packet_make_eb(uint8_t *buf, uint8_t buf_size)
{
  /* XXX make sure this function does not overflow buf */
  uint16_t j = 0;
  uint8_t len, sub_id, type, i = 0, k = 0;
  static uint8_t mac_eb_seqno;
  asn_t next_eb_asn;
  struct tsch_link *l;
  /* TODO */
  uint8_t tsch_slot_template_id = 1;
  uint8_t tsch_hop_sequence_id = 1;
  rpl_instance_t *rpl = rpl_get_instance(RPL_DEFAULT_INSTANCE);
  static rpl_dag_t *my_rpl_dag = NULL;
  struct tsch_slotframe *slotframe;

  COOJA_DEBUG_STR("TSCH make EB");
  /* fcf: 2Bytes */
  /* b0-2: Frame type=0 - b3: security - b4: pending - b5: AR - b6: PAN ID compression - b7: reserved */
  buf[i++] = 0x00;
  /* b8: seqno suppression - b9:IE-list-present=1 - b10-11: destination address mode - b12-b13:frame version=2 - b14-15: src address mode=3 */
  buf[i++] = 2 | 32 | 128 | 64;
  if(mac_eb_seqno == 0) {
    mac_eb_seqno++;
  }
  buf[i++] = mac_eb_seqno++;

  /* copy src PAN ID and long src address */
  /* Source PAN ID */
  buf[i++] = IEEE802154_PANID & 0xff;
  buf[i++] = (IEEE802154_PANID >> 8) & 0xff;

  /* Source address */
  for(k = RIMEADDR_SIZE; k > 0; k--) {
    buf[i++] = rimeaddr_node_addr.u8[k - 1];
  }
  k = 0;
  /* XXX in 6top: EB length, group ID and type: leave 2 bytes for that */
  buf[i++] = 0x00;
  buf[i++] = ((1 << 1) | 1) << 3; /* b0-2:: left for length MSB, b3-6: GroupID, b7: Type */

  /* Append TSCH sync IE */
  len = 6;
  sub_id = 0x1a;
  type = 0; /* short type */
  buf[i++] = len;
  buf[i++] = (sub_id << 1) | (type & 0x01);
  /* Find ASN of the slot where we will transmit the EB */
  next_eb_asn = current_asn;
  do {
    /* Jump to next active slot */
    next_eb_asn += tsch_schedule_time_to_next_active_link(next_eb_asn);
    /* Check if the link is usable for transmission of EB  */
    l = tsch_schedule_get_link_from_asn(next_eb_asn);
  } while(!(l && l->link_options & LINK_OPTION_TX /* is tx */
      && l->link_type == LINK_TYPE_ADVERTISING /* is ADVERTISING */
      && rimeaddr_cmp(&l->addr, &tsch_broadcast_address))); /* or is broadcast */

  buf[i++] = next_eb_asn;
  buf[i++] = next_eb_asn >> 8;
  buf[i++] = next_eb_asn >> 16;
  buf[i++] = next_eb_asn >> 24;
  buf[i++] = next_eb_asn >> 32;

  if(rpl != NULL) {
    my_rpl_dag = rpl->current_dag;
    if(my_rpl_dag != NULL) {
      tsch_join_priority = (my_rpl_dag->rank) >> 8;
    }
  }

  if(tsch_is_coordinator) {
    tsch_join_priority = 0;
  }

  buf[i++] = tsch_join_priority;

  /* Append timeslot template IE */
  len = 1;
  sub_id = 0x1c;
  type = 0; /* short type */
  buf[i++] = len;
  buf[i++] = (sub_id << 1) | (type & 0x01);
  buf[i++] = tsch_slot_template_id;
  /* Optional: include the full timeslot template */
  /* TODO not today ... */

  /* Append channel hopping IE */
  len = 1;
  sub_id = 0x09;
  type = 0; /* short type */
  buf[i++] = len;
  buf[i++] = (sub_id << 1) | (type & 0x01);
  buf[i++] = tsch_hop_sequence_id;
  /* Optional: include the full hop sequence */
  /* TODO not today ... */

  /* Append TSCH slotframe & link IE */
  slotframe = tsch_schedule_get_slotframe_from_handle(MINSCHEDULE_SLOTFRAME_HANDLE);
  if(slotframe != NULL) {
    sub_id = 0x1b;
    type = 0; /* short type */
    len = i++; /* saving len index instead to calculate len at the end */
    /* buf[i++] = 4; */
    buf[i++] = (sub_id << 1) | (type & 0x01);

    buf[i++] = 1; /* number of slotframes described in EB */
    /* TSCH slotframe descriptor*/
    buf[i++] = slotframe->handle;
    buf[i++] = slotframe->size;
    buf[i++] = slotframe->size >> 8;

    k = i++; /* index of the element containing the number of included links */
    struct tsch_link *l = list_head(slotframe->links_list);
    while(l != NULL) {
      /* Include cells I am listening on only */
      if(l->link_options & LINK_OPTION_RX) {
        /* increase the number of included cells */
        buf[k]++;
        buf[i++] = l->timeslot;
        buf[i++] = l->timeslot >> 8;
        buf[i++] = l->channel_offset;
        buf[i++] = l->channel_offset >> 8;
        buf[i++] = l->link_options;
      }
      /* Size of slotframe IE */
      buf[len] = 4 + 5 * buf[k];

      l = list_item_next(l);
    }

  }
  buf[buf_size - 1] = i;
  /* Append time correction IE */
  /* Put IEs: sync, slotframe and link, timeslot and channel hopping sequence */
  /*	if (reported_drift != 0) { */
  /*		tsch_packet_add_sync_IE(buf+i+3, reported_drift, nack); */
  /*		i+=4; */
  /*	} */

  /* XXX only in 6top -- ignore ... EB length, group ID and type: leave 2 bytes for that */
  /* FCF: in buf[0,1] - EBSN: in buf[2] - srcPAN: 3,4 - srcAddr: 5-12 */
  buf[13] = i;
  buf[14] = ((i >> 8) & 0x7) | (((1 << 1) | 1) << 3); /* b0-2:: left for length MSB, b3-6: GroupID, b7: Type */
  return i;
}

#define DO_ACK 2
#define IS_DATA 4
#define IS_ACK 8

static uint8_t
tsch_packet_parse_frame_type(uint8_t *data, uint8_t len)
{
	if(len < ACK_LEN) {
		return 0;
	}
	/* decode the FCF */
	uint8_t do_ack = ((data[0] >> 5) & 1) == 1 ? DO_ACK : 0;
	uint8_t is_data = (data[0] & 7) == FRAME802154_DATAFRAME ? IS_DATA : 0;
	uint8_t is_ack = (data[0] & 7) == FRAME802154_ACKFRAME ? IS_ACK : 0;
	return do_ack | is_data | is_ack;
}

uint8_t
tsch_packet_is_ack_needed(uint8_t *data, uint8_t len)
{
	uint8_t do_ack = 0;
	if(len > ACK_LEN) {
		/* decode the FCF */
		do_ack = ((data[0] >> 5) & 1) == 1 ? DO_ACK : 0;
	}
	return do_ack;
}

static rimeaddr_t source_address = { { 0, 0, 0, 0, 0, 0, 0, 0 } };
/* Extract sender address from raw packet */
rimeaddr_t *
tsch_packet_extract_sender_address(uint8_t *buf, uint8_t len)
{
	frame802154_fcf_t fcf;

	int c;

	if(len > ACK_LEN) {
		uint8_t * p = buf;

		/* decode the FCF */
		fcf.frame_type = p[0] & 7;
		fcf.security_enabled = (p[0] >> 3) & 1;
		fcf.frame_pending = (p[0] >> 4) & 1;
		fcf.ack_required = (p[0] >> 5) & 1;
		fcf.panid_compression = (p[0] >> 6) & 1;

		fcf.dest_addr_mode = (p[1] >> 2) & 3;
		fcf.frame_version = (p[1] >> 4) & 3;
		fcf.src_addr_mode = (p[1] >> 6) & 3;

		/* copy fcf and seqNum */
		p += 3;  /* Skip first three bytes */

		/* Destination address, if any */
		if(fcf.dest_addr_mode) {
			/* Destination PAN */
			p += 2;
			if(fcf.dest_addr_mode == FRAME802154_SHORTADDRMODE) {
				p += 2;
			} else if(fcf.dest_addr_mode == FRAME802154_LONGADDRMODE) {
				p += 8;
			}
		}

		/* Source address, if any */
		if(fcf.src_addr_mode) {
			/* Source PAN */
			if(!fcf.panid_compression) {
				p += 2;
			} else {
			}
			/* Source address */
			if(fcf.src_addr_mode == FRAME802154_SHORTADDRMODE) {
				source_address.u8[0] = p[1];
				source_address.u8[1] = p[0];
				p += 2;
			} else if(fcf.src_addr_mode == FRAME802154_LONGADDRMODE) {
				for(c = 0; c < 8; c++) {
					source_address.u8[c] = p[7-c];
				}
				p += 8;
			}
		} else {
			rimeaddr_copy(&source_address, &rimeaddr_null);
		}
	} else {
		return NULL;
	}
	return &source_address;
}

uint8_t
tsch_parse_eb(uint8_t* buf, uint8_t buf_len, asn_t *asn, uint8_t *join_priority)
{
	uint8_t sync = 0;
	if (buf_len >= 23 /* is long enough to contain Sync-IE? */
	/* is beacon? */
	&& (FRAME802154_BEACONFRAME == (buf[0] & 7))
	/* IE available? */
	&& ((buf[1] & (2 | 32 | 128 | 64)) == (2 | 32 | 128 | 64))
	/* sync IE? (0x1a << 1) ==0 0x34 */
	&& ((buf[16] & 0xfe) == 0x34)) {
		/* TODO: check this */
		*asn = (asn_t) buf[17];
		*asn |= (asn_t) buf[18] << 8;
		*asn |= (asn_t) buf[19] << 16;
		*asn |= (asn_t) buf[20] << 24;
		*asn |= (asn_t) buf[21] << 32;
		*join_priority = buf[22] + 1;

		/* we are in sync */
		sync = 1;
	}
  return sync;
}
