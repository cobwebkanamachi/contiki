/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 *         A wrapper for NXP MicroMAC.
 * \author
 *         Beshr Al Nahas <beshr@sics.se>
 */
#include "MMAC.h"
#include "net/mac/micromac.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include <string.h>

//volatile static void
//(*MMAC_ISR_handler)(uint32 mac_event);
PROCESS(micromac_process, "micromac driver");
/*---------------------------------------------------------------------------*/
volatile static uint8_t tx_in_progress = 0;
/*---------------------------------------------------------------------------*/
volatile static uint8_t locked, lock_on, lock_off;
/*---------------------------------------------------------------------------*/
#define GET_LOCK() locked++
static void RELEASE_LOCK(void) {
//  if(locked == 1) {
//    if(lock_on) {
//      on();
//      lock_on = 0;
//    }
//    if(lock_off) {
//      off();
//      lock_off = 0;
//    }
//  }
  locked--;
}
/*---------------------------------------------------------------------------*/

static void
MMAC_ISR_handler(uint32 mac_event) {
	switch (mac_event) {
	case mac_event & E_MMAC_INT_TX_COMPLETE: /* Transmission attempt has finished */
		tx_in_progress = 0;
	break;;
	case mac_event & E_MMAC_INT_RX_HEADER: /* MAC header has been received */
		break;;
	case mac_event & E_MMAC_INT_RX_COMPLETE: /* Complete frame has been received */
		process_poll(&micromac_process);

//  last_packet_timestamp = cc2420_sfd_start_time;
//  pending++;
//  cc2420_packets_seen++
		break;;
	}
}
/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
	if(tx_in_progress) {
		mac_call_sent_callback(sent, ptr, MAC_TX_COLLISION, 1);
	}
	GET_LOCK();
	tx_in_progress=1;
  tsMacFrame psFrame;
  psFrame.u16DestPAN=packetbuf_addr(PACKETBUF_ATTR_NETWORK_ID);
  psFrame.u16SrcPAN=packetbuf_addr(PACKETBUF_ATTR_NETWORK_ID);
  psFrame.u8PayloadLength=packetbuf_totlen();
  psFrame.uDestAddr=packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  psFrame.uSrcAddr=packetbuf_addr(PACKETBUF_ADDR_SENDER);
  memcpy( psFrame.uPayload, packetbuf_hdrptr(), packetbuf_totlen() );

  vMMAC_StartMacTransmit(&psFrame, E_MMAC_TX_START_NOW | E_MMAC_TX_USE_AUTO_ACK | E_MMAC_TX_USE_CCA );
  // switched in ISR
  while(tx_in_progress);
  RELEASE_LOCK();
  uint32_t tx_error = u32MMAC_GetTxErrors();
  if( tx_error == 0) {
    ret = MAC_TX_OK;
  } else if(tx_error & E_MMAC_TXSTAT_CCA_BUSY){
    ret =  MAC_TX_COLLISION;
  } else if(tx_error & E_MMAC_TXSTAT_NO_ACK){
    ret =  MAC_TX_NOACK;
  } else if(tx_error & E_MMAC_TXSTAT_ABORTED){
    ret =  MAC_TX_ERR;
  }
  mac_call_sent_callback(sent, ptr, ret, 1);
}
/*---------------------------------------------------------------------------*/
static void
send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  if(buf_list != NULL) {
    queuebuf_to_packetbuf(buf_list->buf);
    send_packet(sent, ptr);
  }
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
  NETSTACK_MAC.input();
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return 0; //NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int
off(int keep_radio_on)
{
//  if(keep_radio_on) {
//    return NETSTACK_RADIO.on();
//  } else {
//    return NETSTACK_RADIO.off();
//  }
	return 0;
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
  vMMAC_Enable();
  vMMAC_EnableInterrupts(&MMAC_ISR_handler);

  vMMAC_ConfigureRadio();
  vMMAC_SetChannel(RF_CHANNEL);
  uint16_t u16PanId=IEEE802154_PANDID;
  uint16_t u16ShortAddress = rimeaddr_node_addr.u8[1]<<8 + rimeaddr_node_addr.u8[0];
  //XXX returns wrong address
  MAC_ExtAddr_s* psMacAddr = pvAppApiGetMacAddrLocation();
  vMMAC_SetRxAddress(u16PanId, u16ShortAddress, psMacAddr);
  /* these parameters should disable hardware backoff, but still enable autoACK processing and TX CCA */
  uint8_t u8TxAttempts=1, /* 1 transmission attempt without ACK */
  u8MinBE=1, /* min backoff exponent */
  u8MaxBE=1, /* max backoff exponent */
  u8MaxBackoffs=0; /* backoff before aborting */
  vMMAC_SetTxParameters(u8TxAttempts, u8MinBE, u8MaxBE, u8MaxBackoffs);
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver micromac_driver = {
  "micromac",
  init,
  send_packet,
  send_list,
  packet_input,
  on,
  off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(micromac_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  PRINTF("micromac_process: started\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    PRINTF("micromac_process: calling receiver callback\n");

    packetbuf_clear();
    packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
//    len = cc2420_read(packetbuf_dataptr(), PACKETBUF_SIZE);

    packetbuf_set_datalen(len);

    NETSTACK_RDC.input();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

