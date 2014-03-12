/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 *         A null RDC implementation that uses framer for headers.
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Niclas Finne <nfi@sics.se>
 */

#include "net/mac/nullrdc.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include "net/rime/rimestats.h"
#include "sys/rtimer.h"
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "lib/list.h"
#include "lib/memb.h"
#include <node-id.h>

#if CONTIKI_TARGET_COOJA
#include "lib/simEnvChange.h"
#endif /* CONTIKI_TARGET_COOJA */

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef NULLRDC_CONF_ADDRESS_FILTER
#define NULLRDC_ADDRESS_FILTER NULLRDC_CONF_ADDRESS_FILTER
#else
#define NULLRDC_ADDRESS_FILTER 1
#endif /* NULLRDC_CONF_ADDRESS_FILTER */

#ifndef NULLRDC_802154_AUTOACK
#ifdef NULLRDC_CONF_802154_AUTOACK
#define NULLRDC_802154_AUTOACK NULLRDC_CONF_802154_AUTOACK
#else
#define NULLRDC_802154_AUTOACK 1
#endif /* NULLRDC_CONF_802154_AUTOACK */
#endif /* NULLRDC_802154_AUTOACK */

#ifndef NULLRDC_802154_AUTOACK_HW
#ifdef NULLRDC_CONF_802154_AUTOACK_HW
#define NULLRDC_802154_AUTOACK_HW NULLRDC_CONF_802154_AUTOACK_HW
#else
#define NULLRDC_802154_AUTOACK_HW 1
#endif /* NULLRDC_CONF_802154_AUTOACK_HW */
#endif /* NULLRDC_802154_AUTOACK_HW */

#if NULLRDC_802154_AUTOACK
#include "sys/rtimer.h"
#include "dev/watchdog.h"

#ifdef NULLRDC_CONF_ACK_WAIT_TIME
#define ACK_WAIT_TIME NULLRDC_CONF_ACK_WAIT_TIME
#else /* NULLRDC_CONF_ACK_WAIT_TIME */
#define ACK_WAIT_TIME                      RTIMER_SECOND / 2500
#endif /* NULLRDC_CONF_ACK_WAIT_TIME */
#ifdef NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#define AFTER_ACK_DETECTED_WAIT_TIME NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#else /* NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#define AFTER_ACK_DETECTED_WAIT_TIME       RTIMER_SECOND / 1500
#endif /* NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#endif /* NULLRDC_802154_AUTOACK */

#ifdef NULLRDC_CONF_SEND_802154_ACK
#define NULLRDC_SEND_802154_ACK NULLRDC_CONF_SEND_802154_ACK
#else /* NULLRDC_CONF_SEND_802154_ACK */
#define NULLRDC_SEND_802154_ACK 0
#endif /* NULLRDC_CONF_SEND_802154_ACK */

#if NULLRDC_SEND_802154_ACK
#include "net/mac/frame802154.h"
#endif /* NULLRDC_SEND_802154_ACK */

#define ACK_LEN 3

#if NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW
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
#endif /* NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW */


// variable for real timer tsch
static struct rtimer timer_tsch;

//variable for absolute slot number 
static int ASN;
// variable to protect queue structure
volatile uint8_t working_on_queue;

#define MAX_NUM_PKT 8 // POWER OF 2
#define MAX_NEIGHBOR 16
#define macMinBE 3
#define macMaxFrameRetries 4

// TSCH PACKET STRUCTURE
struct TSCH_packet{
struct queuebuf * pkt; // pointer to the packet to be sent
uint8_t transmissions; // #transmissions performed for this packet
mac_callback_t sent; // callback for this packet
};


// NEIGHBOR QUEUE STRUCTURE
struct neighbor_queue{
struct neighbor_queue *next; // pointer to next neighbor
rimeaddr_t addr; // neighbor address
uint8_t BE_value; // current value of backoff exponent for this neighbor 
struct TSCH_packet buffer[MAX_NUM_PKT]; // circular buffer of packets for this neighbor
uint8_t put_ptr, get_ptr, mask; // data-structures for circular buffer
};


// DECLARATION OF THE LIST OF NEIGHBORS
MEMB(neighbor_memb, struct neighbor_queue, MAX_NEIGHBOR);
LIST(neighbor_list);

// PROTOTYPES
struct neighbor_queue * neighbor_queue_from_addr(const rimeaddr_t *addr);
int add_queue(const rimeaddr_t *addr);
int remove_queue(const rimeaddr_t *addr);
int add_packet_to_queue(mac_callback_t sent, const rimeaddr_t *addr);
int remove_packet_from_queue(const rimeaddr_t *addr);
struct TSCH_packet* read_packet_from_queue(const rimeaddr_t *addr);
static void tsch_timer(void *ptr);

// This function returns a pointer to the queue of neighbor whose address is equal to addr


struct neighbor_queue *
neighbor_queue_from_addr(const rimeaddr_t *addr)
{
  struct neighbor_queue *n = list_head(neighbor_list);
  while(n != NULL) {
    if(rimeaddr_cmp(&n->addr, addr)) {
      return n;
    }
    n = list_item_next(n);
  }
  return NULL;
}


// This function adds one queue for neighbor whose address is equal to addr
// uses working_on_queue to protect data-structures from race conditions
// return 1 ok, 0 failed to allocate

int add_queue(const rimeaddr_t *addr){
working_on_queue=1; 
struct neighbor_queue *n;
int i;
n = memb_alloc(&neighbor_memb);
if(n != NULL) {
      /* Init neighbor entry */
      rimeaddr_copy(&n->addr, addr);
      n->BE_value = macMinBE;
      n->put_ptr=0;
      n->get_ptr=0;
      n->mask=MAX_NUM_PKT-1; 
      for(i=0; i < MAX_NUM_PKT; i++){
      n->buffer[i].pkt=0;
      n->buffer[i].transmissions = 0;
      }
      list_add(neighbor_list, n);
      working_on_queue=0;
      PRINTF("ADD QUEUE %d\n", addr);
      return 1;
    }
working_on_queue=0;
return 0;
}


// This function remove the queue of neighbor whose address is equal to addr
// uses working_on_queue to protect data-structures from race conditions
// return 1 ok, 0 failed to find the queue

int remove_queue(const rimeaddr_t *addr){
working_on_queue=1;
int i;
struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
if(n!=NULL){
for(i=0; i < MAX_NUM_PKT; i++){      // free packets of neighbor
      queuebuf_free(n->buffer[i].pkt); 
      }
list_remove(neighbor_list, n); // free queue of neighbor
memb_free(&neighbor_memb, n);
working_on_queue=0;
return 1;
}
working_on_queue=0;
return 0;
}


// This function adds one packet to the queue of neighbor whose address is addr
// return 1 ok, 0 failed to allocate
// the packet to be inserted is in packetbuf
int add_packet_to_queue(mac_callback_t sent, const rimeaddr_t *addr){
struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
if(n!=NULL){
             if(((n->put_ptr - n->get_ptr) & n->mask) == n->mask) {
                return 0;
             }
             n->buffer[n->put_ptr].pkt = queuebuf_new_from_packetbuf(); // create new packet from packetbuf
             n->buffer[n->put_ptr].sent=sent;
             n->put_ptr = (n->put_ptr + 1) & n->mask;
             return 1;
}
return 0;
}


// This function removes the head-packet of the queue of neighbor whose address is addr
// return 1 ok, 0 failed
// remove one packet from the queue
int remove_packet_from_queue(const rimeaddr_t *addr){
struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
if(n!=NULL){
  if(((n->put_ptr - n->get_ptr) & n->mask) > 0) {
    queuebuf_free(n->buffer[n->get_ptr].pkt);
    n->get_ptr = (n->get_ptr + 1) & n->mask;
    return 1;
  } else {
    return 0;
    }
}
return 0;
}


// This function returns the pointer to the first packet in the queue of neighbor whose address is addr
struct TSCH_packet* read_packet_from_queue(const rimeaddr_t *addr){
struct neighbor_queue *n = neighbor_queue_from_addr(addr); // retrieve the queue from address
if(n!=NULL){
   if(((n->put_ptr - n->get_ptr) & n->mask) > 0) {
       return n->buffer[n->get_ptr].pkt;
   }
   else{
   return 0;
   }  
}
return 0;
}




/*---------------------------------------------------------------------------*/
static int
send_one_packet(mac_callback_t sent, void *ptr)
{
  int ret;
  int last_sent_ok = 0;

  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);
#if NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
#endif /* NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW */

  if(NETSTACK_FRAMER.create() < 0) {
    /* Failed to allocate space for headers */
    PRINTF("nullrdc: send failed, too large header\n");
    ret = MAC_TX_ERR_FATAL;
  } else {

#ifdef NETSTACK_ENCRYPT
    NETSTACK_ENCRYPT();
#endif /* NETSTACK_ENCRYPT */

#if NULLRDC_802154_AUTOACK
    PRINTF("CASO AUTOACK\n");
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
                PRINTF("RICEVUTO AUTOACK %d\n", len);
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

#else /* ! NULLRDC_802154_AUTOACK */
    PRINTF("CASO NO-AUTOACK\n");
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

#endif /* ! NULLRDC_802154_AUTOACK */
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
  PRINTF("SEND RDC\n");
  send_TSCH(sent, ptr);
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
  PRINTF("RDC INPUT FUNCTION\n");
  int original_datalen;
  uint8_t *original_dataptr;

  original_datalen = packetbuf_datalen();
  original_dataptr = packetbuf_dataptr();
#ifdef NETSTACK_DECRYPT
    NETSTACK_DECRYPT();
#endif /* NETSTACK_DECRYPT */

#if NULLRDC_802154_AUTOACK
  PRINTF("SONO ENTRATO %d %d\n", ACK_LEN, packetbuf_datalen());
  if(packetbuf_datalen() == ACK_LEN) {
    /* Ignore ack packets */
    PRINTF("nullrdc: ignored ack\n"); 
  } else
#endif /* NULLRDC_802154_AUTOACK */
  if(NETSTACK_FRAMER.parse() < 0) {
    PRINTF("nullrdc: failed to parse %u\n", packetbuf_datalen());
#if NULLRDC_ADDRESS_FILTER
  } else if(!rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                         &rimeaddr_node_addr) &&
            !rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                          &rimeaddr_null)) {
    PRINTF("nullrdc: not for us\n");
#endif /* NULLRDC_ADDRESS_FILTER */
  } else {
    int duplicate = 0;

#if NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW
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
#endif /* NULLRDC_802154_AUTOACK */

#if NULLRDC_SEND_802154_ACK
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
#endif /* NULLRDC_SEND_ACK */
    if(!duplicate) {
      NETSTACK_MAC.input();
    }
  }
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  PRINTF("NULLRDC ON \n");
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
static void
init(void)
{
  memb_init(&neighbor_memb);
  working_on_queue=0;
  ASN = 0;
  rtimer_set(&timer_tsch, RTIMER_NOW() + RTIMER_SECOND/50, 1, (void (*) (void *)) tsch_timer, NULL);
}

static void
 tsch_timer(void *ptr)
 {
   rtimer_set(&timer_tsch, RTIMER_NOW() + RTIMER_SECOND/50, 1, (void (*) (void *)) tsch_timer, NULL);
   if(working_on_queue==0){
   if((ASN%101)>2){NETSTACK_RADIO.off();}
   else{
   // accendo la radio e setto il canale
   // se sono nel mio slot trasmetto altrimenti se devo ricevere ricevo 
   PRINTF("NODO %d ASN %d  TIMESLOT %d \n", node_id, ASN, ASN%101);
   NETSTACK_RADIO.on();
   //int choffset = 1;
   //NETSTACK_RADIO.cc2420_set_channel( ((ASN+choffset)%16) + 11); // set channel 
   if(((ASN%101)+1)==node_id){

            // cerco una coda piena
            struct neighbor_queue *n = list_head(neighbor_list);
            while(n != NULL) {
                             struct TSCH_packet* p = read_packet_from_queue(&n->addr);
                             if(!p){
                             n = list_item_next(n);
                             } else{
                                     queuebuf_to_packetbuf(p->pkt);
                                     send_one_packet(0,0); 
                                     PRINTF("MANDATO PACCHETTO %d\n", node_id);
                                     remove_packet_from_queue(&n->addr);
                                     break;
                             }
            }

 
            
   }
   }
   }
   ASN=ASN+1;
 }


// Function send for TSCH-MAC, puts the packet in packetbuf in the MAC queue
int send_TSCH(mac_callback_t sent, void *ptr){
struct neighbor_queue *n;
  static uint16_t seqno;
  const rimeaddr_t *addr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);

  if(seqno == 0) {
    /* PACKETBUF_ATTR_MAC_SEQNO cannot be zero, due to a pecuilarity
       in framer-802154.c. */
    seqno++;
  }

  packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, seqno++);

  /* Look for the neighbor entry */
  n = neighbor_queue_from_addr(addr);
  if(n == NULL) {  
                  //add new neighbor to list of neighbor
                  if(!add_queue(addr)) return 0;
                  //add new packet to neighbor list
                  if(!add_packet_to_queue(sent, addr)) return 0;
  }  
  else{
                  //add new packet to neighbor list
                  if(!add_packet_to_queue(sent, addr)) return 0;
  }

return 1;
}




/*---------------------------------------------------------------------------*/
const struct rdc_driver nullrdc_driver = {
  "nullrdc",
  init,
  send_packet,
  send_list,
  packet_input,
  on,
  off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/