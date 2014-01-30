/*
 * Copyright (c) 2014, NXP & Swedish Institute of Computer Science
 * All rights reserved.
 */
/*
 * NXP MMAC and NXP confidential radio functions/implementation
 */

#include <string.h>

#include "contiki.h"

#include "dev/leds.h"
#include "dev/micromac-radio.h"
#include "MMAC.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"
#include "JPT.h"
#include <AppHardwareApi.h>
#include <AppApi.h>

#ifndef IEEE802154_PANID
#	ifdef IEEE802154_CONF_PANID
#		define IEEE802154_PANID           IEEE802154_CONF_PANID
#	else
#		define IEEE802154_PANID           0xABCD
#	endif
#endif

#ifndef RF_CHANNEL
#	define RF_CHANNEL 26
#endif

/* XXX JN5168_CONF_CCA_THRESH has an arbitrary value */
// an integer between 0 and 255
#ifndef JN5168_CONF_CCA_THRESH
#define JN5168_CONF_CCA_THRESH 75
#endif /* JN5168_CONF_CCA_THRESH */

#define WITH_SEND_CCA 1

#define FOOTER_LEN 2

#define AUX_LEN (CHECKSUM_LEN + FOOTER_LEN)

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

#define DEBUG_LEDS DEBUG
#undef LEDS_ON
#undef LEDS_OFF
#if DEBUG_LEDS
#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#else
#define LEDS_ON(x)
#define LEDS_OFF(x)
#endif

/* XXX hack: these will be made as Chameleon packet attributes */
rtimer_clock_t micromac_radio_time_of_arrival, micromac_radio_time_of_departure;

static uint8_t volatile pending, micromac_radio_packets_seen;

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

volatile uint8_t micromac_radio_sfd_counter = 0, micromac_packets_read = 0;
//volatile uint16_t micromac_radio_sfd_start_time;
//volatile uint16_t micromac_radio_sfd_end_time;

static volatile uint32_t last_packet_timestamp = 0;

//TODO: consider making a list
static volatile tsMacFrame tx_frame_buffer;
static volatile tsMacFrame rx_frame_buffer;
/*---------------------------------------------------------------------------*/
PROCESS(micromac_radio_process, "micromac_radio_driver");
/*---------------------------------------------------------------------------*/
static uint8_t receive_on;

volatile static uint8_t tx_in_progress = 0;

volatile static uint8_t rx_in_progress = 0;

static int channel;
/*---------------------------------------------------------------------------*/
static void
on(void)
{
	vMMAC_StartMacReceive(&rx_frame_buffer, E_MMAC_RX_START_NOW | E_MMAC_RX_USE_AUTO_ACK | E_MMAC_RX_ALLOW_MALFORMED | E_MMAC_RX_NO_FCS_ERROR | E_MMAC_RX_ADDRESS_MATCH);
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
	off();
}
/*---------------------------------------------------------------------------*/
static uint8_t locked, lock_on, lock_off;
/*---------------------------------------------------------------------------*/
#define GET_LOCK() locked++
static void RELEASE_LOCK(void) {
  if(locked == 1) {
    if(lock_on) {
      on();
      lock_on = 0;
    }
    if(lock_off) {
      off();
      lock_off = 0;
    }
  }
  locked--;
}
/*---------------------------------------------------------------------------*/
static void
micromac_radio_interrupt(uint32 mac_event)
{
	if (mac_event & E_MMAC_INT_TX_COMPLETE) { /* Transmission attempt has finished */
		tx_in_progress = 0;
	}	else if (mac_event & E_MMAC_INT_RX_HEADER) { /* MAC header has been received */
		micromac_radio_sfd_counter++;
	}	else if (mac_event & E_MMAC_INT_RX_COMPLETE) { /* Complete frame has been received */
		rx_in_progress = 0;
		process_poll(&micromac_radio_process);
		last_packet_timestamp = u32MMAC_GetRxTime();
		micromac_radio_time_of_arrival = last_packet_timestamp;
		pending++;
		micromac_radio_packets_seen++;
	}
}

volatile static uint16_t u16PanId = IEEE802154_PANID;
volatile static uint16_t u16ShortAddress;
/*---------------------------------------------------------------------------*/
int
micromac_radio_init(void)
{
	if(locked) {
		return 0;
	}
	GET_LOCK();
	tx_in_progress = 0;
	u32JPT_Init();
  vMMAC_Enable();
  vMMAC_EnableInterrupts(&micromac_radio_interrupt);

  vMMAC_ConfigureRadio();
  channel = RF_CHANNEL;
  vMMAC_SetChannel(channel);
  u16ShortAddress = (rimeaddr_node_addr.u8[1]<<8) + rimeaddr_node_addr.u8[0];
  tsExtAddr* psMacAddr = (tsExtAddr*) pvAppApiGetMacAddrLocation();
  vMMAC_SetRxAddress(u16PanId, u16ShortAddress, psMacAddr);
  /* these parameters should disable hardware backoff, but still enable autoACK processing and TX CCA */
  uint8_t u8TxAttempts=1, /* 1 transmission attempt without ACK */
  u8MinBE=1, /* min backoff exponent */
  u8MaxBE=1, /* max backoff exponent */
  u8MaxBackoffs=0; /* backoff before aborting */
  vMMAC_SetTxParameters(u8TxAttempts, u8MinBE, u8MaxBE, u8MaxBackoffs);
	RELEASE_LOCK();
  process_start(&micromac_radio_process, NULL);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_transmit(unsigned short payload_len)
{
	if(tx_in_progress) {
		return RADIO_TX_COLLISION;
	}
  uint8_t total_len;
  GET_LOCK();
	tx_in_progress=1;
  vMMAC_StartMacTransmit(&tx_frame_buffer, E_MMAC_TX_START_NOW | E_MMAC_TX_USE_AUTO_ACK | E_MMAC_TX_USE_CCA );
  // switched in ISR
  while(tx_in_progress);
  int ret = RADIO_TX_ERR;;
  uint32_t tx_error = u32MMAC_GetTxErrors();
  if( tx_error == 0) {
    ret = RADIO_TX_OK;
  } else if(tx_error & E_MMAC_TXSTAT_CCA_BUSY){
    ret =  RADIO_TX_COLLISION;
  } else if(tx_error & E_MMAC_TXSTAT_NO_ACK){
    ret =  RADIO_TX_NOACK;
  } else if(tx_error & E_MMAC_TXSTAT_ABORTED){
    ret = RADIO_TX_ERR;
  }
  RELEASE_LOCK();
  return ret;
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_prepare(const void *payload, unsigned short payload_len)
{
  GET_LOCK();

  PRINTF("micromac_radio: sending %d bytes\n", payload_len);

  RIMESTATS_ADD(lltx);

	if(tx_in_progress) {
		return 1;
	}
	GET_LOCK();
	/* copy payload to (soft) tx buffer */
	/* XXX use packetbuf_dataptr() or packetbuf_hdrptr(); also, packetbuf_datalen() or packetbuf_totallen()?? */
//  memcpy(&(tx_frame_buffer.u16DestPAN), packetbuf_addr(PACKETBUF_ATTR_NETWORK_ID), sizeof(tx_frame_buffer.u16DestPAN));
//  //rimeaddr_copy()
//	tx_frame_buffer.u16SrcPAN = tx_frame_buffer.u16DestPAN;
//	//tx_frame_buffer.u8PayloadLength = packetbuf_datalen();
//	//tx_frame_buffer.u16FCF = packetbuf_attr(PACKETBUF_ATTR_);
//	rimeaddr_copy(&(tx_frame_buffer.uDestAddr), packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
//  //memcpy(&(tx_frame_buffer.uDestAddr), packetbuf_addr(PACKETBUF_ADDR_RECEIVER), sizeof(tx_frame_buffer.uDestAddr));
// // memcpy(&(tx_frame_buffer.uSrcAddr), packetbuf_addr(PACKETBUF_ADDR_SENDER), sizeof(tx_frame_buffer.uSrcAddr));
//	rimeaddr_copy(&(tx_frame_buffer.uSrcAddr), packetbuf_addr(PACKETBUF_ADDR_SENDER));
//	memcpy(&(tx_frame_buffer.uPayload), packetbuf_dataptr(), packetbuf_datalen());
//	tx_frame_buffer.u8PayloadLength = packetbuf_totlen();
	//((tsMacFrame*)payload)->u8PayloadLength
	memcpy(&(tx_frame_buffer), payload, packetbuf_totlen());
	tx_frame_buffer.u8PayloadLength += packetbuf_datalen();
	tx_frame_buffer.u16Unused = packetbuf_datalen()%4;
//  tx_frame_buffer.u16FCF = payload[0]<<8;
//  tx_frame_buffer.u16FCF |= payload[1];
//	  /* sequence number */
//	  tx_frame_buffer.u8SequenceNum = payload[2];



//	  //tx_frame_buffer->u8PayloadLength=3; //seqNo+FCF
//	  /* Destination PAN ID */
//	  if(flen.dest_pid_len == 2) {
//	    tx_frame_buffer->u16DestPAN = p->dest_pid;
//	    tx_frame_buffer->u8PayloadLength+=2;
//	  }
//
//	  /* Destination address */
//	  rimeaddr_copy(&(tx_frame_buffer->uDestAddr), &(p->dest_addr));
//	  tx_frame_buffer->u8PayloadLength+=flen.dest_addr_len;
//
//	  /* Source PAN ID */
//	  if(flen.src_pid_len == 2) {
//	    tx_frame_buffer->u16SrcPAN = p->src_pid;
//	    tx_frame_buffer->u8PayloadLength+=flen.src_pid_len;
//	  }
//
//	  /* Source address */
//	  rimeaddr_copy(&(tx_frame_buffer->uSrcAddr), &(p->src_addr));
//	  tx_frame_buffer->u8PayloadLength+=flen.src_addr_len;
//
//	  /* Aux header */
//	  if(flen.aux_sec_len) {
//	    /* TODO Aux security header not yet implemented */
//	/*     pos += flen.aux_sec_len; */
//	  }
	RELEASE_LOCK();
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_send(const void *payload, unsigned short payload_len)
{
	micromac_radio_prepare(payload, payload_len);
  return micromac_radio_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
int
micromac_radio_off(void)
{
  GET_LOCK();
  off();
  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
micromac_radio_on(void)
{
  GET_LOCK();
  rx_in_progress=1;
  on();
	RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
micromac_radio_get_channel(void)
{
  return channel;
}
/*---------------------------------------------------------------------------*/
int
micromac_radio_set_channel(int c)
{
  GET_LOCK();
  channel = c;
  vMMAC_SetChannel(channel);
  RELEASE_LOCK();
  return 1;
}
/*---------------------------------------------------------------------------*/
void
micromac_radio_set_pan_addr(unsigned pan,
                    unsigned addr,
                    const uint8_t *ieee_addr)
{
  GET_LOCK();
  tsExtAddr* psMacAddr = (tsExtAddr*) ieee_addr;
  u16PanId = pan;
  u16ShortAddress = addr;
  if(ieee_addr == NULL) {
  	psMacAddr = pvAppApiGetMacAddrLocation();
  }
  vMMAC_SetRxAddress(u16PanId, u16ShortAddress, psMacAddr);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_read(void *buf, unsigned short bufsize)
{
	pending = 0;

	GET_LOCK();

	micromac_packets_read++;
	int len = rx_frame_buffer.u8PayloadLength;
	memcpy(packetbuf_hdrptr(), &(rx_frame_buffer.uPayload), len);
	packetbuf_set_addr(PACKETBUF_ATTR_NETWORK_ID, (rimeaddr_t *)&(rx_frame_buffer.u16DestPAN));
	packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, (rimeaddr_t *)&(rx_frame_buffer.uDestAddr));
	packetbuf_set_addr(PACKETBUF_ADDR_SENDER, (rimeaddr_t *)&(rx_frame_buffer.uSrcAddr));
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, rx_frame_buffer.u8SequenceNum);

	//  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, cc2420_last_rssi);
	//  packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, cc2420_last_correlation);
	RELEASE_LOCK();
	return len;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(micromac_radio_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  PRINTF("micromac_radio_process: started\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    PRINTF("micromac_radio_process: calling receiver callback\n");

    packetbuf_clear();
    /* XXX PACKETBUF_ATTR_TIMESTAMP is 16bits while last_packet_timestamp is 32bits*/
    packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, (uint16_t)last_packet_timestamp);
    len = micromac_radio_read(packetbuf_dataptr(), PACKETBUF_SIZE);
    packetbuf_set_datalen(len);
    NETSTACK_RDC.input();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
micromac_set_txpower(uint8_t power)
{
  GET_LOCK();
  vJPT_RadioSetPower(power);
  RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
int
micromac_get_txpower(void)
{
  int power=0;
  GET_LOCK();
  power = u8JPT_RadioGetPower();
  RELEASE_LOCK();
  return power;
}
/*---------------------------------------------------------------------------*/
//int
//micromac_rssi(void)
//{
//  int rssi=0;
//  int radio_was_off = 0;
//
//  if(locked) {
//    return 0;
//  }
//
//  GET_LOCK();
//
//
//  if(radio_was_off) {
//  	micromac_off();
//  }
//  RELEASE_LOCK();
//  return rssi;
//}
/*---------------------------------------------------------------------------*/
int
micromac_detected_energy(void)
{
	uint32 u32Samples = 8;
  return u8JPT_EnergyDetect(channel, u32Samples);
}
///*---------------------------------------------------------------------------*/
//int
//micromac_cca_valid(void)
//{
//  int valid=0;
//  if(locked) {
//    return 1;
//  }
//  GET_LOCK();
////  valid = !!(status() & BV(CC2420_RSSI_VALID));
//  RELEASE_LOCK();
//  return valid;
//}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_cca(void)
{
  int cca;
  /* If the radio is locked by an underlying thread (because we are
     being invoked through an interrupt), we pretend that the coast is
     clear (i.e., no packet is currently being transmitted by a
     neighbor). */
  if(locked) {
    return 1;
  }

  GET_LOCK();
  cca = bJPT_CCA(channel, E_JPT_CCA_MODE_CARRIER, JN5168_CONF_CCA_THRESH);
  RELEASE_LOCK();
  return cca;
}
/*---------------------------------------------------------------------------*/
int
micromac_radio_receiving_packet(void)
{
  return rx_in_progress;
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_pending_packet(void)
{
  return pending;
}
/*---------------------------------------------------------------------------*/
//void
//micromac_set_cca_threshold(int value)
//{
////  uint16_t shifted = value << 8;
//  GET_LOCK();
////  setreg(CC2420_RSSI, shifted);
//  RELEASE_LOCK();
//}
/*---------------------------------------------------------------------------*/
const struct radio_driver micromac_radio_driver =
{
	micromac_radio_init,
	micromac_radio_prepare,
	micromac_radio_transmit,
	micromac_radio_send,
	micromac_radio_read,
	/* micromac_set_channel, */
	/* micromac_detected_energy, */
	micromac_radio_cca,
	micromac_radio_receiving_packet,
	micromac_radio_pending_packet,
	micromac_radio_on,
	micromac_radio_off,
};
