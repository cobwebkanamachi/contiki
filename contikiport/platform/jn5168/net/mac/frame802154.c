/*
 *
 *  Copyright (c) 2008, Swedish Institute of Computer Science
 *  All rights reserved.
 *
 *  Additional fixes for AVR contributed by:
 *
 *      Colin O'Flynn coflynn@newae.com
 *      Eric Gnoske egnoske@gmail.com
 *      Blake Leverett bleverett@gmail.com
 *      Mike Vidales mavida404@gmail.com
 *      Kevin Brown kbrown3@uccs.edu
 *      Nate Bohlmann nate@elfwerks.com
 *
 *  Additional fixes for MSP430 contributed by:
 *        Joakim Eriksson
 *        Niclas Finne
 *        Nicolas Tsiftes
 *
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of the copyright holders nor the names of
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
*/
/*
 *  \brief This file is where the main functions that relate to frame
 *  manipulation will reside.
*/
/**
 *   \addtogroup frame802154
 *   @{
*/
/**
 *  \file
 *  \brief 802.15.4 frame creation and parsing functions
 *
 *  This file converts to and from a structure to a packed 802.15.4
 *  frame.
 */

#include "sys/cc.h"
#include "net/mac/frame802154.h"
#include <string.h>
#include "dev/micromac-radio.h"
#include <MMAC.h>

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif
/**
 *  \brief Structure that contains the lengths of the various addressing and security fields
 *  in the 802.15.4 header.  This structure is used in \ref frame802154_create()
 */
typedef struct {
  uint8_t dest_pid_len;    /**<  Length (in bytes) of destination PAN ID field */
  uint8_t dest_addr_len;   /**<  Length (in bytes) of destination address field */
  uint8_t src_pid_len;     /**<  Length (in bytes) of source PAN ID field */
  uint8_t src_addr_len;    /**<  Length (in bytes) of source address field */
  uint8_t aux_sec_len;     /**<  Length (in bytes) of aux security header field */
} field_length_t;

/*----------------------------------------------------------------------------*/
CC_INLINE static uint8_t
addr_len(uint8_t mode)
{
  switch(mode) {
  case FRAME802154_SHORTADDRMODE:  /* 16-bit address */
    return 2;
  case FRAME802154_LONGADDRMODE:   /* 64-bit address */
    return 8;
  default:
    return 0;
  }
}
/*----------------------------------------------------------------------------*/
static void
field_len(frame802154_t *p, field_length_t *flen)
{
  /* init flen to zeros */
  memset(flen, 0, sizeof(field_length_t));

  /* Determine lengths of each field based on fcf and other args */
  if(p->fcf.dest_addr_mode & 3) {
    flen->dest_pid_len = 2;
  }
  if(p->fcf.src_addr_mode & 3) {
    flen->src_pid_len = 2;
  }

  /* Set PAN ID compression bit if src pan id matches dest pan id. */
  if(p->fcf.dest_addr_mode & 3 && p->fcf.src_addr_mode & 3 &&
     p->src_pid == p->dest_pid) {
    p->fcf.panid_compression = 1;

    /* compressed header, only do dest pid */
    flen->src_pid_len = 0;
  } else {
    p->fcf.panid_compression = 0;
  }

  /* determine address lengths */
  flen->dest_addr_len = addr_len(p->fcf.dest_addr_mode & 3);
  flen->src_addr_len = addr_len(p->fcf.src_addr_mode & 3);

  /* Aux security header */
  if(p->fcf.security_enabled & 1) {
    /* TODO Aux security header not yet implemented */
#if 0
    switch(p->aux_hdr.security_control.key_id_mode) {
    case 0:
      flen->aux_sec_len = 5; /* minimum value */
      break;
    case 1:
      flen->aux_sec_len = 6;
      break;
    case 2:
      flen->aux_sec_len = 10;
      break;
    case 3:
      flen->aux_sec_len = 14;
      break;
    default:
      break;
    }
#endif
  }
}
/*----------------------------------------------------------------------------*/
/**
 *   \brief Calculates the length of the frame header.  This function is
 *   meant to be called by a higher level function, that interfaces to a MAC.
 *
 *   \param p Pointer to frame802154_t_t struct, which specifies the
 *   frame to send.
 *
 *   \return The length of the frame header.
*/
int
frame802154_hdrlen(frame802154_t *p)
{
  field_length_t flen;
  field_len(p, &flen);
  return 3 + flen.dest_pid_len + flen.dest_addr_len +
    flen.src_pid_len + flen.src_addr_len + flen.aux_sec_len;
}
/*----------------------------------------------------------------------------*/
/**
 *   \brief Creates a frame for transmission over the air.  This function is
 *   meant to be called by a higher level function, that interfaces to a MAC.
 *
 *   \param p Pointer to frame802154_t struct, which specifies the
 *   frame to send.
 *
 *   \param buf Pointer to the buffer to use for the frame.
 *
 *   \param buf_len The length of the buffer to use for the frame.
 *
 *   \return The length of the frame header or 0 if there was
 *   insufficient space in the buffer for the frame headers.
*/
int
frame802154_create(frame802154_t *p, uint8_t *buf, int buf_len)
{
	PRINTF("frame802154_create: %d bytes\n", buf_len);
  int c;
  field_length_t flen;
  tsMacFrame *tx_frame_buffer;
  uint8_t pos;

  field_len(p, &flen);

  if(3 + flen.dest_pid_len + flen.dest_addr_len +
     flen.src_pid_len + flen.src_addr_len + flen.aux_sec_len > buf_len) {
    /* Too little space for headers. */
    return 0;
  }

  /* OK, now we have field lengths.  Time to actually construct */
  /* the outgoing frame, and store it in tx_frame_buffer */
  tx_frame_buffer = (tsMacFrame *)buf;

  pos = (uint8_t *) &(tx_frame_buffer->uPayload) - buf;

  /* sequence number */
  tx_frame_buffer->u8SequenceNum = p->seq;

  tx_frame_buffer->u16FCF = 0xff & ((p->fcf.frame_type & 7) |
    ((p->fcf.security_enabled & 1) << 3) |
    ((p->fcf.frame_pending & 1) << 4) |
    ((p->fcf.ack_required & 1) << 5) |
    ((p->fcf.panid_compression & 1) << 6));
  tx_frame_buffer->u16FCF |= 0xff00 & ((((p->fcf.dest_addr_mode & 3) << 2) |
    ((p->fcf.frame_version & 3) << 4) |
    ((p->fcf.src_addr_mode & 3) << 6))<<8);

  tx_frame_buffer->u8PayloadLength=3; //seqNo+FCF
  /* Destination PAN ID */
  if(flen.dest_pid_len == 2) {
    tx_frame_buffer->u16DestPAN = p->dest_pid;
    tx_frame_buffer->u8PayloadLength+=2;
  }

  /* Destination address */
//  if(p->fcf.dest_addr_mode == FRAME802154_LONGADDRMODE) {
//  	memcpy(&(tx_frame_buffer->uDestAddr.sExt), p->dest_addr, 8);
//  	tx_frame_buffer->u8PayloadLength+=8;
//  } else if(p->fcf.dest_addr_mode == FRAME802154_SHORTADDRMODE){
//    memcpy(&(tx_frame_buffer->uDestAddr.u16Short), p->dest_addr, 2);
//    tx_frame_buffer->u8PayloadLength+=2;
//  }
  //copy_from_rimeaddress(&(tx_frame_buffer->uDestAddr), &(p->dest_addr));
  copy_from_rimeaddress((tuAddr*)&(tx_frame_buffer->uDestAddr), (rimeaddr_t*)&(p->dest_addr));
  tx_frame_buffer->u8PayloadLength+=RIMEADDR_SIZE;

  /* Source PAN ID */
  if(flen.src_pid_len == 2) {
    tx_frame_buffer->u16SrcPAN = p->src_pid;
    tx_frame_buffer->u8PayloadLength+=2;
  }


  /* Source address */
//  if(p->fcf.src_addr_mode == FRAME802154_LONGADDRMODE) {
//  	tx_frame_buffer->u8PayloadLength+=8;
//  	//memcpy(&(tx_frame_buffer->uSrcAddr.sExt), p->src_addr, 8);
//  } else if(p->fcf.src_addr_mode == FRAME802154_SHORTADDRMODE) {
//    memcpy(&(tx_frame_buffer->uSrcAddr.u16Short), p->src_addr, 2);
//    tx_frame_buffer->u8PayloadLength+=2;
//  }
  copy_from_rimeaddress((tuAddr*)&(tx_frame_buffer->uSrcAddr), (rimeaddr_t*)&(p->src_addr));
  tx_frame_buffer->u8PayloadLength+=RIMEADDR_SIZE;

  /* Aux header */
  if(flen.aux_sec_len) {
    /* TODO Aux security header not yet implemented */
/*     pos += flen.aux_sec_len; */
  }
//unsigned char tmp[3]={1,2,3};
//  PRINTF("frame802154_create: u8PayloadLength %d, u8SequenceNum %d, \
//u16FCF 0x%02x, u16DestPAN 0x%02x, u16SrcPAN 0x%02x, uDestAddr 0x%04x, \
//uSrcAddr 0x%04x, fcf.src_addr_mode %d, flen.src_addr_len %d,\n \
//RIMEADDR_SIZE %d, sizeof(rimeaddr_t) %d, sizeof(uchar) %d, sizeof(tmp) %d, pos %d\n",
//  		tx_frame_buffer->u8PayloadLength,
//  		tx_frame_buffer->u8SequenceNum,
//  		tx_frame_buffer->u16FCF,
//  		tx_frame_buffer->u16DestPAN,
//  		tx_frame_buffer->u16SrcPAN,
//  		tx_frame_buffer->uDestAddr.u16Short,
//  		tx_frame_buffer->uSrcAddr.u16Short,
//  		p->fcf.src_addr_mode,
//  		flen.src_addr_len,
//  		RIMEADDR_SIZE,
//  		sizeof(rimeaddr_t), sizeof(unsigned char), sizeof(tmp),
//  		pos);

  return (int)pos;
}
/*----------------------------------------------------------------------------*/
/**
 *   \brief Parses an input frame.  Scans the input frame to find each
 *   section, and stores the information of each section in a
 *   frame802154_t structure.
 *
 *   \param data The input data from the radio chip.
 *   \param len The size of the input data
 *   \param pf The frame802154_t struct to store the parsed frame information.
 */
int
frame802154_parse(uint8_t *data, int len, frame802154_t *pf)
{
	tsMacFrame *p;
  frame802154_fcf_t fcf;
  int c=3,i;

  if(len < 3) {
    return 0;
  }

  p = (tsMacFrame *)data;

  /* decode the FCF */
  fcf.frame_type = p->u16FCF & 7;
  fcf.security_enabled = (p->u16FCF >> 3) & 1;
  fcf.frame_pending = (p->u16FCF >> 4) & 1;
  fcf.ack_required = (p->u16FCF >> 5) & 1;
  fcf.panid_compression = (p->u16FCF >> 6) & 1;

  uint8_t fcf_h = p->u16FCF >> 8;
  fcf.dest_addr_mode = (fcf_h >> 2) & 3;
  fcf.frame_version = (fcf_h >> 4) & 3;
  fcf.src_addr_mode = (fcf_h >> 6) & 3;

  /* copy fcf and seqNum */
  memcpy(&pf->fcf, &fcf, sizeof(frame802154_fcf_t));
  pf->seq =  p->u8SequenceNum;

  /* Destination address, if any */
  if(fcf.dest_addr_mode) {
    /* Destination PAN */
    pf->dest_pid = p->u16DestPAN;
    c+=2;
    /* Destination address */
    if(fcf.dest_addr_mode == FRAME802154_SHORTADDRMODE) {
      rimeaddr_copy((rimeaddr_t *)&(pf->dest_addr), &rimeaddr_null);
      pf->dest_addr[0] = p->uDestAddr.u16Short >>8;
      pf->dest_addr[1] = p->uDestAddr.u16Short & 0xff;
      c+=2;
    } else if(fcf.dest_addr_mode == FRAME802154_LONGADDRMODE) {
    	c+=8;
      for(i = 0; i < 4; i++) {
        pf->dest_addr[i] = p->uDestAddr.sExt.u32H >> ((3-i)*8);
        pf->dest_addr[i+4] = p->uDestAddr.sExt.u32L >> ((3-i)*8);
      }
    }
  } else {
    rimeaddr_copy((rimeaddr_t *)&(pf->dest_addr), &rimeaddr_null);
    pf->dest_pid = 0;
  }

  /* Source address, if any */
  if(fcf.src_addr_mode) {
    /* Source PAN */
    if(!fcf.panid_compression) {
      pf->src_pid = p->u16SrcPAN;
      c+=2;
    } else {
      pf->src_pid = pf->dest_pid;
    }

    /* Source address */
    if(fcf.src_addr_mode == FRAME802154_SHORTADDRMODE) {
    	c+=2;
      rimeaddr_copy((rimeaddr_t *)&(pf->src_addr), &rimeaddr_null);
      pf->src_addr[0] = p->uSrcAddr.u16Short >>8;
      pf->src_addr[1] = p->uSrcAddr.u16Short & 0xff;
    } else if(fcf.src_addr_mode == FRAME802154_LONGADDRMODE) {
    	c+=8;
      for(i = 0; i < 4; i++) {
        pf->src_addr[i] = p->uSrcAddr.sExt.u32H >> ((3-i)*8);
        pf->src_addr[i+4] = p->uSrcAddr.sExt.u32L >> ((3-i)*8);
      }
    }
  } else {
    rimeaddr_copy((rimeaddr_t *)&(pf->src_addr), &rimeaddr_null);
    pf->src_pid = 0;
  }

  if(fcf.security_enabled) {
    /* TODO aux security header, not yet implemented */
/*     return 0; */
  }

  /* header length = c*/

  /* payload length */
  pf->payload_len = (len - c);
  /* payload */
  pf->payload = p->uPayload.au8Byte;

  printf("frame802154_parse: u8PayloadLength %d, u8SequenceNum %d, \
u16FCF 0x%02x, u16DestPAN 0x%02x, u16SrcPAN 0x%02x, uDestAddr 0x%04x, \
uSrcAddr 0x%04x, len %d\n",
  		p->u8PayloadLength,
  		p->u8SequenceNum,
  		p->u16FCF,
  		p->u16DestPAN,
  		p->u16SrcPAN,
  		p->uDestAddr.u16Short,
  		p->uSrcAddr.u16Short,
  		len);
  /* return header length if successful */
  return (len < MICROMAC_HEADER_LEN) ? 0 : MICROMAC_HEADER_LEN; //c > len ? 0 : c;
}
/** \}   */
