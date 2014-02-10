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

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"
#include "JPT.h"
#include <AppHardwareApi.h>
#include <AppApi.h>
#include <MMAC.h>
#include "micromac-radio.h"

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
#define JN5168_CONF_CCA_THRESH 127
#endif /* JN5168_CONF_CCA_THRESH */

#define WITH_SEND_CCA 1

#define FOOTER_LEN 2

#define AUX_LEN (CHECKSUM_LEN + FOOTER_LEN)

#define DEBUG 0
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

static uint8_t volatile pending = 0, micromac_radio_packets_seen = 0,
		rx_ackneeded = 0, rx_noackneeded = 0, rx_aborted = 0, rx_complete,
		rx_malformed, rx_error, rx_overflow;

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
static volatile uint32_t micromac_radio_rx_garbage = 0,
		micromac_radio_tx_completed = 0, rx_state;

//TODO: consider making a list
#define RX_LIST_SIZE 10
static tsMacFrame __attribute__((aligned)) tx_frame_buffer;
static tsMacFrame __attribute__((aligned)) rx_frame_buffer[RX_LIST_SIZE];
static tsMacFrame * volatile rx_frame_buffer_write_ptr,
		* volatile rx_frame_buffer_read_ptr, * volatile rx_frame_buffer_recent_ptr;
static volatile uint32_t rx_frame_buffer_write_index = 0,
		rx_frame_buffer_read_index = 0;

static void
switch_rx_receive_buffer()
{
	rx_frame_buffer_recent_ptr = rx_frame_buffer_write_ptr;
	rx_frame_buffer_write_index = (rx_frame_buffer_write_index + 1)
			% RX_LIST_SIZE;
	rx_frame_buffer_write_ptr = &(rx_frame_buffer[rx_frame_buffer_write_index]);
	if (rx_frame_buffer_write_ptr == rx_frame_buffer_read_ptr) {
		rx_frame_buffer_write_ptr = rx_frame_buffer_recent_ptr;
	}
	//	rx_frame_buffer_read_ptr=rx_frame_buffer_write_ptr;
}
static void
switch_rx_read_buffer()
{
	rx_frame_buffer_read_index = (1 + rx_frame_buffer_read_index) % RX_LIST_SIZE;
	rx_frame_buffer_read_ptr = &(rx_frame_buffer[rx_frame_buffer_read_index]);
	//	rx_frame_buffer_read_ptr=rx_frame_buffer_write_ptr;
}
/*---------------------------------------------------------------------------*/
PROCESS(micromac_radio_process, "micromac_radio_driver")
;
/*---------------------------------------------------------------------------*/
static uint8_t receive_on;

volatile static uint8_t tx_in_progress = 0;

volatile static uint8_t rx_in_progress = 0;

static int channel;
/*---------------------------------------------------------------------------*/
static void
on(void)
{
	vMMAC_StartMacReceive(rx_frame_buffer_write_ptr, E_MMAC_RX_START_NOW
			| E_MMAC_RX_USE_AUTO_ACK | E_MMAC_RX_NO_MALFORMED
			| E_MMAC_RX_NO_FCS_ERROR | E_MMAC_RX_ADDRESS_MATCH);
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
	vMMAC_RadioOff();
}
/*---------------------------------------------------------------------------*/
static uint8_t locked, lock_on, lock_off;
/*---------------------------------------------------------------------------*/
#define GET_LOCK() locked++
static void
RELEASE_LOCK(void)
{
	if (locked == 1) {
		if (lock_on) {
			on();
			lock_on = 0;
		}
		if (lock_off) {
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
		micromac_radio_tx_completed++;
		//XXX: Always ON!!
		on();
	} else if (mac_event & E_MMAC_INT_RX_HEADER) { /* MAC header has been received */
		micromac_radio_sfd_counter++;
		rx_state = u32MMAC_GetRxErrors();
		if (rx_state & E_MMAC_RXSTAT_ABORTED) {
			rx_aborted++;
			RIMESTATS_ADD(badsynch);
		} else if (rx_state & E_MMAC_RXSTAT_ERROR) {
			rx_error++;
			RIMESTATS_ADD(badcrc);
		} else if (rx_state & E_MMAC_RXSTAT_MALFORMED) {
			rx_malformed++;
			RIMESTATS_ADD(toolong);
		} else if (!rx_state) {
			switch_rx_receive_buffer();
			uint8_t ack_needed = (rx_frame_buffer_recent_ptr->u16FCF >> 5) & 1;
			process_poll(&micromac_radio_process);
			last_packet_timestamp = u32MMAC_GetRxTime();
			micromac_radio_time_of_arrival = last_packet_timestamp;
			pending++;
			micromac_radio_packets_seen++;
			process_poll(&micromac_radio_process);
			//check if ACk is needed!
			if (ack_needed) {
				rx_ackneeded++;
			} else {
				rx_noackneeded++;
				//XXX: Always ON!!
				on();
			}
		}
	} else if (mac_event & E_MMAC_INT_RX_COMPLETE) { /* Complete frame has been received and ACKed*/
		//rx_in_progress = 0;
		rx_complete++;
		uint8_t ack_needed = (rx_frame_buffer_recent_ptr->u16FCF >> 5) & 1;
		//XXX: Always ON!!
		if (ack_needed) {
			on();
		}
	}
}

volatile static uint16_t u16PanId = IEEE802154_PANID;
volatile static uint16_t u16ShortAddress;
/*---------------------------------------------------------------------------*/
int
micromac_radio_init(void)
{
	if (locked) {
		return 0;
	}
	GET_LOCK();
	tx_in_progress = 0;
	rx_frame_buffer_write_ptr = &(rx_frame_buffer[0]);
	rx_frame_buffer_read_ptr = rx_frame_buffer_write_ptr;
	rx_frame_buffer_recent_ptr = rx_frame_buffer_write_ptr;
	uint32_t jpt_ver = u32JPT_Init();
	vMMAC_Enable();
	vMMAC_EnableInterrupts(&micromac_radio_interrupt);

	vMMAC_ConfigureRadio();
	channel = RF_CHANNEL;
	vMMAC_SetChannel(channel);
	u16ShortAddress = rimeaddr_node_addr.u8[1] + (rimeaddr_node_addr.u8[0] << 8);
	tsExtAddr psExtAddress;
	uint32 *pu32Mac = pvAppApiGetMacAddrLocation();
	psExtAddress.u32H = pu32Mac[0];
	psExtAddress.u32L = pu32Mac[1];
	vMMAC_SetRxAddress(u16PanId, u16ShortAddress, &psExtAddress);
	/* these parameters should disable hardware backoff, but still enable autoACK processing and TX CCA */
	uint8_t u8TxAttempts = 1, /* 1 transmission attempt without ACK */
	u8MinBE = 0, /* min backoff exponent */
	u8MaxBE = 1, /* max backoff exponent */
	u8MaxBackoffs = 1; /* backoff before aborting */
	vMMAC_SetTxParameters(u8TxAttempts, u8MinBE, u8MaxBE, u8MaxBackoffs);
	rx_in_progress = 1;
	on();
	RELEASE_LOCK();
	process_start(&micromac_radio_process, NULL);
	PRINTF("micromac_radio init: RXAddress %04x == %08x.%08x @ PAN %04x, channel: %d, u32JPT_Init: %d\n", u16ShortAddress, psExtAddress.u32H, psExtAddress.u32L, u16PanId, channel, jpt_ver);
	return 1;
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_transmit(unsigned short payload_len)
{
	if (tx_in_progress) {
		return RADIO_TX_COLLISION;
	}
	GET_LOCK();
	tx_in_progress = 1;
	vMMAC_StartMacTransmit(&tx_frame_buffer, E_MMAC_TX_START_NOW
			| E_MMAC_TX_USE_AUTO_ACK | E_MMAC_TX_USE_CCA);
	// switched in ISR
	while (tx_in_progress){}
	int ret = RADIO_TX_ERR;
	uint32_t tx_error = u32MMAC_GetTxErrors();
	if (tx_error == 0) {
		ret = RADIO_TX_OK;
		RIMESTATS_ADD(acktx);
	} else if (tx_error & E_MMAC_TXSTAT_CCA_BUSY) {
		ret = RADIO_TX_COLLISION;
		RIMESTATS_ADD(contentiondrop);
	} else if (tx_error & E_MMAC_TXSTAT_NO_ACK) {
		ret = RADIO_TX_NOACK;
		RIMESTATS_ADD(noacktx);
	} else if (tx_error & E_MMAC_TXSTAT_ABORTED) {
		ret = RADIO_TX_ERR;
		RIMESTATS_ADD(sendingdrop);
	}
	RELEASE_LOCK();
	return ret;
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_prepare(const void *payload, unsigned short payload_len)
{
	printf(
			"micromac_radio: sending payload_len %dB u8PayloadLength %u + packetbuf_datalen %u, rx_overflow %u, rx_ackneeded %u, rx_noackneeded %u, rx_state %u, rx_complete %d, rx_error %d, rx_malformed %d, rx_aborted %d, packets_seen %d, rx_garbage %u,\nsfd_counter %d, noacktx %u, acktx %u, tx_completed %u, contentiondrop %u, sendingdrop %u\n",
			payload_len, ((tsMacFrame*) payload)->u8PayloadLength,
			packetbuf_datalen(), rx_overflow, rx_ackneeded, rx_noackneeded, rx_state,
			rx_complete, rx_error, rx_malformed, rx_aborted,
			micromac_radio_packets_seen, micromac_radio_rx_garbage,
			micromac_radio_sfd_counter, RIMESTATS_GET(noacktx), RIMESTATS_GET(acktx),
			micromac_radio_tx_completed, RIMESTATS_GET(contentiondrop),
			RIMESTATS_GET(sendingdrop));
	int i;

	PRINTF("\n");
	RIMESTATS_ADD(lltx);

	if (tx_in_progress) {
		return 1;
	}
	GET_LOCK();
	/* copy payload to (soft) tx buffer */
	/* XXX use packetbuf_dataptr() or packetbuf_hdrptr(); also, packetbuf_datalen() or packetbuf_totallen()?? */
	memcpy(&(tx_frame_buffer), payload, payload_len);
	tx_frame_buffer.u16Unused = packetbuf_datalen()%4;
	tx_frame_buffer.u8PayloadLength += packetbuf_datalen() - tx_frame_buffer.u16Unused;
	printf(
			"micromac_radio: sending payload_len %dB u8PayloadLength %u + packetbuf_datalen %u, u16Unused %u, rx_overflow %u\n",
			payload_len, ((tsMacFrame*) payload)->u8PayloadLength,
			packetbuf_datalen(), tx_frame_buffer.u16Unused, rx_overflow);
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
	rx_in_progress = 0;
	RELEASE_LOCK();
	return 1;
}
/*---------------------------------------------------------------------------*/
int
micromac_radio_on(void)
{
	GET_LOCK();
	rx_in_progress = 1;
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
micromac_radio_set_pan_addr(unsigned pan, unsigned addr,
		const uint8_t *ieee_addr)
{
	GET_LOCK();
	tsExtAddr* psMacAddr = (tsExtAddr*) ieee_addr;
	u16PanId = pan;
	u16ShortAddress = addr;
	if (ieee_addr == NULL) {
		psMacAddr = pvAppApiGetMacAddrLocation();
	}
	vMMAC_SetRxAddress(u16PanId, u16ShortAddress, psMacAddr);
	RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_read(void *buf, unsigned short bufsize)
{
	//XXX disable printf
	uint8_t footer[2];
	int len, unused;
	GET_LOCK();
	if (--pending) {
		process_poll(&micromac_radio_process);
		if(pending>=RX_LIST_SIZE) {
			pending %= RX_LIST_SIZE;
			rx_overflow++;
		}
	}
	micromac_packets_read++;
	len = sizeof(tsMacFrame) - 32 * sizeof(uint32) + rx_frame_buffer_read_ptr->u8PayloadLength; //MICROMAC_HEADER_LEN
	//should copy the whole thing or else something wrong happens probably because of alignment
	memcpy(buf, rx_frame_buffer_read_ptr, sizeof(tsMacFrame));
	printf(
			"len: %u, u8PayloadLength %u, u16Unused %u\n",
			len, rx_frame_buffer_read_ptr->u8PayloadLength, ((tsMacFrame*)buf)->u16Unused);
	switch_rx_read_buffer();
	//  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, cc2420_last_rssi);
	//  packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, cc2420_last_correlation);
	RIMESTATS_ADD(llrx);
	RELEASE_LOCK();
	int i;
	for (i = 0; i < len; i++) {
		printf("%02x ", ((uint8_t*)buf)[i]);
	}
	PRINTF("micromac_radio: reading %d bytes\n", len);

	return len;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(micromac_radio_process, ev, data)
{

	PROCESS_BEGIN();
		static int len;
		PRINTF("micromac_radio_process: started\n");

		while(1) {
			PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

			PRINTF("micromac_radio_process: calling receiver callback\n");
			//while(pending)
			{
				packetbuf_clear();
				/* XXX PACKETBUF_ATTR_TIMESTAMP is 16bits while last_packet_timestamp is 32bits*/
				packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, (uint16_t)last_packet_timestamp);
				len = micromac_radio_read(packetbuf_hdrptr(), PACKETBUF_SIZE);
				packetbuf_set_datalen(len);
				NETSTACK_RDC.input();
			}
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
	int power = 0;
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
	if (locked) {
		return 1;
	}

	GET_LOCK();
	cca = bJPT_CCA(channel, E_JPT_CCA_MODE_CARRIER_OR_ENERGY,
			JN5168_CONF_CCA_THRESH);
	RELEASE_LOCK();
	return !cca;
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
void
copy_from_rimeaddress(tuAddr* tu_addr, rimeaddr_t* addr)
{
#if(RIMEADDR_SIZE==8)
	tu_addr->sExt.u32L=addr->u8[7] | addr->u8[6]<<8 | addr->u8[5]<<16 | addr->u8[4]<<24;
	tu_addr->sExt.u32H=addr->u8[3] | addr->u8[2]<<8 | addr->u8[1]<<16 | addr->u8[0]<<24;
#elif(RIMEADDR_SIZE==2)
	tu_addr->u16Short = addr->u8[1] | addr->u8[0] << 8;
#endif
}
/*---------------------------------------------------------------------------*/
const struct radio_driver micromac_radio_driver = {
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
	micromac_radio_on, micromac_radio_off
};
