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
#include "Utilities/include/JPT.h"
#include <HardwareApi/include/AppHardwareApi.h>
#include <AppApi/include/AppApi.h>
#include <MMAC/include/MMAC.h>
#include "micromac-radio.h"

#if MICROMAC_RADIO_CONF_NO_IRQ
#define MICROMAC_RADIO_NO_IRQ MICROMAC_RADIO_CONF_NO_IRQ
#pragma "MICROMAC_RADIO_NO_IRQ!"
#endif

#if MICROMAC_RADIO_CONF_ALWAYS_ON
#define MICROMAC_RADIO_ALWAYS_ON 1
#pragma "MICROMAC_RADIO_ALWAYS_ON!"
#else
#define MICROMAC_RADIO_ALWAYS_ON 0
#endif

#if MICROMAC_RADIO_CONF_AUTOACK
#define MMAC_RX_AUTO_ACK_CONF E_MMAC_RX_USE_AUTO_ACK
#define MMAC_TX_AUTO_ACK_CONF E_MMAC_TX_USE_AUTO_ACK
#pragma "MICROMAC_RADIO_CONF_AUTOACK enabled"
#else
#define MMAC_RX_AUTO_ACK_CONF E_MMAC_RX_NO_AUTO_ACK
#define MMAC_TX_AUTO_ACK_CONF E_MMAC_TX_NO_AUTO_ACK
#endif /* MICROMAC_RADIO_CONF_AUTOACK */

#ifndef IEEE802154_PANID
#	ifdef IEEE802154_CONF_PANID
#		define IEEE802154_PANID           IEEE802154_CONF_PANID
#	else
#		define IEEE802154_PANID           0xABCD
#	endif
#endif

#ifdef RF_CONF_CHANNEL
#undef RF_CHANNEL
#define RF_CHANNEL RF_CONF_CHANNEL
#else
#undef RF_CHANNEL
#define RF_CHANNEL (26)
#endif
//vMMAC_SetCutOffTimer(uint32 u32CutOffTime, bool_t bEnable);
/* XXX JN5168_CONF_CCA_THRESH has an arbitrary value */
// an integer between 0 and 255
#ifndef JN5168_CONF_CCA_THRESH
#define JN5168_CONF_CCA_THRESH 127
#endif /* JN5168_CONF_CCA_THRESH */

#define WITH_SEND_CCA 0

#define FOOTER_LEN 2
#define CHECKSUM_LEN 0
#define AUX_LEN (CHECKSUM_LEN + FOOTER_LEN)

#define DEBUG 0
#if DEBUG
int dbg_printf(const char *fmt, ...);
#define PRINTF(...) do {dbg_printf(__VA_ARGS__);} while(0)
#else
#define PRINTF(...) do {} while (0)
#endif

#define DEBUG_LEDS 1
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

static volatile uint32_t last_packet_timestamp = 0, last_packet_end_timestamp = 0;
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
PROCESS(micromac_radio_process, "micromac_radio_driver");
/*---------------------------------------------------------------------------*/


volatile static uint8_t tx_in_progress = 0;

volatile static uint8_t rx_in_progress = 0;

static int channel;
/*---------------------------------------------------------------------------*/
uint32_t* micromac_get_hw_mac_address_location(void) {
	//pvAppApiGetMacAddrLocation() does not return the hw mac address location but something else (something that points to 0xdeadbeef00080006)
	//the number is from http://www.nxp.com/documents/application_note/JN-AN-1003.pdf -- memory map
	return (uint32_t *)0x01001580;
}
void
micromac_get_hw_mac_address(tsExtAddr *psExtAddress)
{
	//WRONG: Does not respect field order. The CPU uses BIG_ENDIAN, and the struct has u32L first.
	//memcpy(psExtAddress, pvAppApiGetMacAddrLocation(), sizeof(tsExtAddr));
	// From jennic support:
	uint32_t *pu32Mac = micromac_get_hw_mac_address_location();
	psExtAddress->u32H = pu32Mac[0];
	psExtAddress->u32L = pu32Mac[1];
}
/*---------------------------------------------------------------------------*/
static volatile softack_make_callback_f softack_make_callback = NULL;
static volatile softack_interrupt_exit_callback_f interrupt_exit_callback = NULL;

/* Subscribe with two callbacks called from FIFOP interrupt */
void
micromac_radio_softack_subscribe(softack_make_callback_f softack_make, softack_interrupt_exit_callback_f interrupt_exit)
{
	softack_make_callback = softack_make;
  interrupt_exit_callback = interrupt_exit;
}
/*---------------------------------------------------------------------------*/
static volatile uint32_t radio_ref_time = 0;
static volatile rtimer_clock_t rtimer_ref_time = 0;

void micromac_radio_sfd_sync(void)
{
	radio_ref_time = u32MMAC_GetTime();
	rtimer_ref_time = RTIMER_NOW();
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t micromac_radio_read_sfd_timer(void)
{
	return RADIO_TO_RTIMER(u32MMAC_GetRxTime() - radio_ref_time) + rtimer_ref_time;
}
/*---------------------------------------------------------------------------*/
void
micromac_radio_start_rx_delayed(uint32 u32_delay_time, uint32 u32_on_duration)
{
	micromac_radio_sfd_sync();
	vMMAC_SetRxStartTime(u32MMAC_GetTime() + u32_delay_time);
	vMMAC_SetCutOffTimer(u32MMAC_GetTime() + u32_delay_time + u32_on_duration, TRUE);
	vMMAC_StartMacReceive(rx_frame_buffer_write_ptr,
			E_MMAC_RX_DELAY_START
			| MMAC_RX_AUTO_ACK_CONF
			| E_MMAC_RX_ALLOW_MALFORMED
			| E_MMAC_RX_NO_FCS_ERROR
			| E_MMAC_RX_ADDRESS_MATCH);
	vMMAC_SetCutOffTimer(0, FALSE);
}
/*---------------------------------------------------------------------------*/
static void
on(void)
{
	micromac_radio_sfd_sync();
	vMMAC_StartMacReceive(rx_frame_buffer_write_ptr,
			E_MMAC_RX_START_NOW
			| MMAC_RX_AUTO_ACK_CONF
			| E_MMAC_RX_ALLOW_MALFORMED
			| E_MMAC_RX_NO_FCS_ERROR
			| E_MMAC_RX_ADDRESS_MATCH);
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
	vMMAC_SetCutOffTimer(0, FALSE);
	vMMAC_RadioOff();
	//PROTOCOL shutdown... to shutdown the circuit instead.
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
static volatile rtimer_clock_t rx_end_time=0;
rtimer_clock_t micromac_radio_get_rx_end_time(void)
{
	return rx_end_time;
}

uint32_t micromac_radio_get_rx_radio_end_time(void)
{
	return last_packet_end_timestamp;
}
/*---------------------------------------------------------------------------*/
static uint8_t extrabuf[ACK_LEN]={0};
volatile struct received_frame_radio_s last_rf;
static volatile tsPhyFrame phy_ackbuf;
void
micromac_radio_interrupt(uint32 mac_event)
{
	uint8_t* ackbuf=NULL;
	uint8_t need_ack = 0;

	if (mac_event & E_MMAC_INT_TX_COMPLETE) { /* Transmission attempt has finished */
		tx_in_progress = 0;
		micromac_radio_tx_completed++;
		//XXX: Always ON!!
#if MICROMAC_RADIO_ALWAYS_ON
		on();
#endif
	} else if (mac_event & E_MMAC_INT_RX_HEADER) { /* MAC header has been received */
		micromac_radio_sfd_counter++;
		rx_state = u32MMAC_GetRxErrors();
		/* if rx is successful */
		if (!rx_state) {
			switch_rx_receive_buffer();
			need_ack = (rx_frame_buffer_recent_ptr->u16FCF >> 5) & 1;
			last_packet_timestamp = u32MMAC_GetRxTime();
			last_packet_end_timestamp = u32MMAC_GetRxTime() + rx_frame_buffer_recent_ptr->u8PayloadLength +1;
			//XXX fix wrapping bugs
			micromac_radio_time_of_arrival = RADIO_TO_RTIMER(last_packet_timestamp - radio_ref_time) + rtimer_ref_time;
			rx_end_time = micromac_radio_time_of_arrival + RADIO_TO_RTIMER(rx_frame_buffer_recent_ptr->u8PayloadLength +1);
			pending++;
			micromac_radio_packets_seen++;
			process_poll(&micromac_radio_process);
			if(need_ack) {
				rx_ackneeded++;
			 if(softack_make_callback != NULL) {
				 /* first byte in ackbuf defines frame length */
				 softack_make_callback(&ackbuf, rx_frame_buffer_recent_ptr->u8SequenceNum, micromac_radio_time_of_arrival, 0);
			 } else { /* construct standard ack */
				 ackbuf = extrabuf;
				 ackbuf[0] = 3;
				 ackbuf[1] = 0x02;
				 ackbuf[2] = 0;
				 ackbuf[3] = rx_frame_buffer_recent_ptr->u8SequenceNum;
			 }
			 /* copy ACK to phy_buf */
			 phy_ackbuf.u8PayloadLength = ackbuf[0];
			 int i;
			 for(i=0; i<phy_ackbuf.u8PayloadLength; i++) {
				 phy_ackbuf.uPayload.au8Byte[i]=ackbuf[i+1];
			 }
			} else {
				rx_noackneeded++;
				//XXX: Always ON!!
#if MICROMAC_RADIO_ALWAYS_ON
				on();
#endif
			}
		} else { /* if rx is not successful */
			if (rx_state & E_MMAC_RXSTAT_ABORTED) {
				rx_aborted++;
				RIMESTATS_ADD(badsynch);
			} else if (rx_state & E_MMAC_RXSTAT_ERROR) {
				rx_error++;
				RIMESTATS_ADD(badcrc);
			} else if (rx_state & E_MMAC_RXSTAT_MALFORMED) {
				rx_malformed++;
				RIMESTATS_ADD(toolong);
			}
			rx_end_time = 0;
		  if(interrupt_exit_callback != NULL) {
		  	interrupt_exit_callback(0, NULL);
		  }
		}
	} else if (mac_event & E_MMAC_INT_RX_COMPLETE) { /* Complete frame has been received and ACKed*/
		//rx_in_progress = 0;
		rx_complete++;
		need_ack = (rx_frame_buffer_recent_ptr->u16FCF >> 5) & 1;
		//XXX: Always ON!!
#if MICROMAC_RADIO_ALWAYS_ON
		if (need_ack) {
			on();
		}
#endif

		if(interrupt_exit_callback != NULL) {
		 last_rf.buf = rx_frame_buffer_recent_ptr->uPayload.au8Byte;
		 last_rf.len = rx_frame_buffer_recent_ptr->u8PayloadLength;
		 last_rf.sfd_timestamp = micromac_radio_time_of_arrival;
		 copy_to_rimeaddress(&(last_rf.source_address), &rx_frame_buffer_recent_ptr->uSrcAddr);
		 interrupt_exit_callback(need_ack, &last_rf);
		}
	}
}
/*---------------------------------------------------------------------------*/
int
micromac_radio_read_ack(void *buf, int alen) {
  uint8_t len=0, footer1=0, overflow=0;
	if (!locked) {
//		BUSYWAIT_UNTIL(!bMMAC_RxDetected(), delayRx);
		BUSYWAIT_UNTIL(u32MMAC_PollInterruptSource(E_MMAC_INT_TX_COMPLETE), 16*500); //TsShortGT
		len = sizeof(tsMacFrame) - 32 * sizeof(uint32) + rx_frame_buffer_read_ptr->u8PayloadLength; //MICROMAC_HEADER_LEN
//		len = rx_frame_buffer_read_ptr->u8PayloadLength; //MICROMAC_HEADER_LEN

		alen = (len > alen) ? alen : len;
		if (buf && len >= ACK_LEN) {
//			//should copy the whole thing or else something wrong happens probably because of alignment
//			//XXX check for possibility of overflow: rx_frame_buffer_read_ptr == rx_frame_buffer_write_ptr
//			memcpy(buf, rx_frame_buffer_read_ptr, sizeof(tsMacFrame));
			memcpy(buf, &rx_frame_buffer_write_ptr->uPayload.au8Byte, alen);
		}
		rx_frame_buffer_read_ptr->u8PayloadLength=0;
  }
  return rx_frame_buffer_read_ptr->u8PayloadLength;
}
/*---------------------------------------------------------------------------*/
void
micromac_radio_send_ack(void) {
  if(!locked) {
		GET_LOCK();
		tx_in_progress = 1;
		vMMAC_StartPhyTransmit(&phy_ackbuf,
				E_MMAC_TX_START_NOW
				| E_MMAC_TX_NO_AUTO_ACK
				| E_MMAC_TX_NO_CCA);
		tx_in_progress = 0;
		RELEASE_LOCK();
  }
  rx_end_time = 0;
}
/*---------------------------------------------------------------------------*/
void
micromac_radio_send_ack_delayed(uint32 u32_delay_time)
{
  if(!locked) {
		GET_LOCK();
		tx_in_progress = 1;
		vMMAC_SetTxStartTime(micromac_radio_get_rx_radio_end_time() + u32_delay_time);
		vMMAC_StartPhyTransmit(&phy_ackbuf,
				E_MMAC_TX_DELAY_START
				| E_MMAC_TX_NO_AUTO_ACK
				| E_MMAC_TX_NO_CCA);
		tx_in_progress = 0;
		RELEASE_LOCK();
  }
  rx_end_time = 0;
}
/*---------------------------------------------------------------------------*/
volatile static uint16_t u16PanId = IEEE802154_PANID;
volatile static uint16_t u16ShortAddress;
/*---------------------------------------------------------------------------*/
int
micromac_radio_init(void)
{
	uint8_t u8TxAttempts = 0, /* 1 transmission attempt without ACK */
	u8MinBE = 0, /* min backoff exponent */
	u8MaxBE = 0, /* max backoff exponent */
	u8MaxBackoffs = 0; /* backoff before aborting */
	uint32_t jpt_ver = 0;
	tsExtAddr psExtAddress;
	if (locked) {
		return 0;
	}
	GET_LOCK();
	tx_in_progress = 0;
	rx_frame_buffer_write_ptr = &(rx_frame_buffer[0]);
	rx_frame_buffer_read_ptr = rx_frame_buffer_write_ptr;
	rx_frame_buffer_recent_ptr = rx_frame_buffer_write_ptr;
	jpt_ver = u32JPT_Init();
	vMMAC_Enable();
#if MICROMAC_RADIO_NO_IRQ
	vMMAC_EnableInterrupts(NULL);
	vMMAC_ConfigureInterruptSources(0);
#else
	vMMAC_EnableInterrupts(&micromac_radio_interrupt);
#endif /* MICROMAC_RADIO_NO_IRQ */
	vMMAC_ConfigureRadio();
	channel = RF_CHANNEL;
	vMMAC_SetChannel(channel);
	u16ShortAddress = rimeaddr_node_addr.u8[1] + (rimeaddr_node_addr.u8[0] << 8);

	vMMAC_GetMacAddress((tsExtAddr *)&psExtAddress);
	vMMAC_SetRxAddress(u16PanId, u16ShortAddress, &psExtAddress);
	/* these parameters should disable hardware backoff, but still enable autoACK processing and TX CCA */

	vMMAC_SetTxParameters(u8TxAttempts, u8MinBE, u8MaxBE, u8MaxBackoffs);
	vMMAC_SetCutOffTimer(0, FALSE);

#if MICROMAC_RADIO_ALWAYS_ON
	rx_in_progress = 1;
	on();
#endif
	RELEASE_LOCK();
	process_start(&micromac_radio_process, NULL);
	PRINTF("micromac_radio init: RXAddress %04x == %08x.%08x @ PAN %04x, channel: %d, u32JPT_Init: %d\n", u16ShortAddress, psExtAddress.u32H, psExtAddress.u32L, u16PanId, channel, jpt_ver);
	return 1;
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_transmit_with_irq(unsigned short payload_len)
{
	if (tx_in_progress) {
		return RADIO_TX_COLLISION;
	}
	GET_LOCK();
	tx_in_progress = 1;
	vMMAC_StartMacTransmit(&tx_frame_buffer,
			E_MMAC_TX_START_NOW
			| MMAC_TX_AUTO_ACK_CONF
			| E_MMAC_TX_NO_CCA);
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
micromac_radio_transmit(unsigned short payload_len)
{
	if (tx_in_progress) {
		return RADIO_TX_COLLISION;
	}
	GET_LOCK();
	tx_in_progress = 1;
	vMMAC_StartMacTransmit(&tx_frame_buffer,
			E_MMAC_TX_START_NOW
			| MMAC_TX_AUTO_ACK_CONF
			| E_MMAC_TX_NO_CCA);
//	uint32 u32Mask = E_MMAC_INT_TX_COMPLETE;
	BUSYWAIT_UNTIL(u32MMAC_PollInterruptSource(E_MMAC_INT_TX_COMPLETE), MAX_PACKET_DURATION);
//	while (!u32MMAC_PollInterruptSource(E_MMAC_INT_TX_COMPLETE)){}
	tx_in_progress = 0;
	int ret = RADIO_TX_ERR;
	uint32_t tx_error = u32MMAC_GetTxErrors();
	if (tx_error == 0) {
		ret = RADIO_TX_OK;
		RIMESTATS_ADD(acktx);
	} else if (tx_error & E_MMAC_TXSTAT_ABORTED) {
		ret = RADIO_TX_ERR;
		RIMESTATS_ADD(sendingdrop);
	}	else if (tx_error & E_MMAC_TXSTAT_CCA_BUSY) {
		ret = RADIO_TX_COLLISION;
		RIMESTATS_ADD(contentiondrop);
	} else if (tx_error & E_MMAC_TXSTAT_NO_ACK) {
		ret = RADIO_TX_NOACK;
		RIMESTATS_ADD(noacktx);
	}
	RELEASE_LOCK();
	return ret;
}
/*---------------------------------------------------------------------------*/
void
micromac_radio_transmit_delayed(uint32 u32_delay_time)
{
	GET_LOCK();
	micromac_radio_sfd_sync();
	tx_in_progress = 1;
	vMMAC_SetTxStartTime(u32MMAC_GetTime() + u32_delay_time);
	vMMAC_StartMacTransmit(&tx_frame_buffer,
			E_MMAC_TX_DELAY_START
			| MMAC_TX_AUTO_ACK_CONF
			| E_MMAC_TX_NO_CCA);
	tx_in_progress = 0;
	RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
int micromac_radio_get_delayed_transmit_status(void)
{
	int ret = RADIO_TX_ERR;
	//XXX could stall longer!
	BUSYWAIT_UNTIL(u32MMAC_PollInterruptSource(E_MMAC_INT_TX_COMPLETE), MAX_PACKET_DURATION);
	uint32_t tx_error = u32MMAC_GetTxErrors();
	if (tx_error == 0) {
		ret = RADIO_TX_OK;
	} else if (tx_error & E_MMAC_TXSTAT_ABORTED) {
		ret = RADIO_TX_ERR;
	}	else if (tx_error & E_MMAC_TXSTAT_CCA_BUSY) {
		ret = RADIO_TX_COLLISION;
	} else if (tx_error & E_MMAC_TXSTAT_NO_ACK) {
		ret = RADIO_TX_NOACK;
	}
	return ret;
}
/*---------------------------------------------------------------------------*/
void
micromac_radio_raw_transmit_delayed(tsPhyFrame *psFrame, uint32 u32_delay_time)
{
	GET_LOCK();
	micromac_radio_sfd_sync();
	tx_in_progress = 1;
	vMMAC_SetTxStartTime(u32MMAC_GetTime() + u32_delay_time);
	vMMAC_StartPhyTransmit(psFrame,
			E_MMAC_TX_DELAY_START
			| MMAC_TX_AUTO_ACK_CONF
			| E_MMAC_TX_NO_CCA);
	tx_in_progress = 0;
	RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
int
micromac_radio_raw_transmit(tsPhyFrame *psFrame)
{
	if (tx_in_progress) {
		return RADIO_TX_COLLISION;
	}
	GET_LOCK();
	micromac_radio_sfd_sync();
	tx_in_progress = 1;
	vMMAC_StartPhyTransmit(psFrame,
			E_MMAC_TX_START_NOW
			| MMAC_TX_AUTO_ACK_CONF
			| E_MMAC_TX_NO_CCA);
//	uint32 u32Mask = E_MMAC_INT_TX_COMPLETE;
	BUSYWAIT_UNTIL(u32MMAC_PollInterruptSource(E_MMAC_INT_TX_COMPLETE), MAX_PACKET_DURATION);
//	while (!u32MMAC_PollInterruptSource(E_MMAC_INT_TX_COMPLETE)){}
	tx_in_progress = 0;
	int ret = RADIO_TX_ERR;
	uint32_t tx_error = u32MMAC_GetTxErrors();
	if (tx_error == 0) {
		ret = RADIO_TX_OK;
		RIMESTATS_ADD(acktx);
	} else if (tx_error & E_MMAC_TXSTAT_ABORTED) {
		ret = RADIO_TX_ERR;
		RIMESTATS_ADD(sendingdrop);
	}	else if (tx_error & E_MMAC_TXSTAT_CCA_BUSY) {
		ret = RADIO_TX_COLLISION;
		RIMESTATS_ADD(contentiondrop);
	} else if (tx_error & E_MMAC_TXSTAT_NO_ACK) {
		ret = RADIO_TX_NOACK;
		RIMESTATS_ADD(noacktx);
	}
	RELEASE_LOCK();
	return ret;
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_prepare(const void *payload, unsigned short payload_len)
{
	PRINTF(
			"micromac_radio: sending payload_len %dB u8PayloadLength %u + packetbuf_datalen %u, rx_overflow %u, rx_ackneeded %u, rx_noackneeded %u, rx_state %u, rx_complete %d, rx_error %d, rx_malformed %d, rx_aborted %d, packets_seen %d, rx_garbage %u,\nsfd_counter %d, noacktx %u, acktx %u, tx_completed %u, contentiondrop %u, sendingdrop %u\n",
			payload_len, ((tsMacFrame*) payload)->u8PayloadLength,
			packetbuf_datalen(), rx_overflow, rx_ackneeded, rx_noackneeded, rx_state,
			rx_complete, rx_error, rx_malformed, rx_aborted,
			micromac_radio_packets_seen, micromac_radio_rx_garbage,
			micromac_radio_sfd_counter, RIMESTATS_GET(noacktx), RIMESTATS_GET(acktx),
			micromac_radio_tx_completed, RIMESTATS_GET(contentiondrop),
			RIMESTATS_GET(sendingdrop));


	PRINTF("\n");
	RIMESTATS_ADD(lltx);

	if (tx_in_progress) {
		return 1;
	}
	GET_LOCK();
	/* copy payload to (soft) tx buffer */

	memcpy(&(tx_frame_buffer), payload, payload_len);
	tx_frame_buffer.u8PayloadLength = payload_len - MICROMAC_HEADER_LEN;
	tx_frame_buffer.u16Unused = tx_frame_buffer.u8PayloadLength % 4;

	PRINTF(
			"micromac_radio: sending packetbuf_totlen() %u payload_len %dB u8PayloadLength %u -> %u + packetbuf_datalen %u, u16Unused %u, rx_overflow %u\n",
			packetbuf_totlen(), payload_len, ((tsMacFrame*) payload)->u8PayloadLength, tx_frame_buffer.u8PayloadLength,
			packetbuf_datalen(), tx_frame_buffer.u16Unused, rx_overflow);
	RELEASE_LOCK();
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_send(const void *payload, unsigned short payload_len)
{
	micromac_radio_prepare(payload, payload_len);
#if MICROMAC_RADIO_NO_IRQ
	return micromac_radio_transmit(payload_len);
#else
	return micromac_radio_transmit_with_irq(payload_len);
#endif
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_send_delayed(const void *payload, unsigned short payload_len, uint32 u32_delay_time)
{
	micromac_radio_prepare(payload, payload_len);
	if (tx_in_progress) {
			return RADIO_TX_COLLISION;
		}
		GET_LOCK();
		vMMAC_SetTxStartTime(u32MMAC_GetTime() + u32_delay_time);
		tx_in_progress = 1;
		vMMAC_StartMacTransmit(&tx_frame_buffer, E_MMAC_TX_DELAY_START
				| E_MMAC_TX_NO_AUTO_ACK | E_MMAC_TX_NO_CCA);

		RELEASE_LOCK();
		return 1;
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_get_transmission_status(void)
{
//	uint32 u32Mask = E_MMAC_INT_TX_COMPLETE;
//	BUSYWAIT_UNTIL(u32MMAC_PollInterruptSource(E_MMAC_INT_TX_COMPLETE), MAX_PACKET_DURATION);
//	while (!u32MMAC_PollInterruptSource(E_MMAC_INT_TX_COMPLETE)){}
	int ret = RADIO_TX_ERR;
	uint32_t tx_error = u32MMAC_GetTxErrors();
	if (tx_error == 0) {
		ret = RADIO_TX_OK;
		RIMESTATS_ADD(acktx);
	} else if (tx_error & E_MMAC_TXSTAT_ABORTED) {
		ret = RADIO_TX_ERR;
		RIMESTATS_ADD(sendingdrop);
	}	else if (tx_error & E_MMAC_TXSTAT_CCA_BUSY) {
		ret = RADIO_TX_COLLISION;
		RIMESTATS_ADD(contentiondrop);
	} else if (tx_error & E_MMAC_TXSTAT_NO_ACK) {
		ret = RADIO_TX_NOACK;
		RIMESTATS_ADD(noacktx);
	}
	return ret;
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
//	uint8_t footer[2];
	int i, len=0;
	GET_LOCK();
	if (--pending) {
		process_poll(&micromac_radio_process);
		if(pending>=RX_LIST_SIZE) {
			pending %= RX_LIST_SIZE;
			rx_overflow++;
		}
	}
	if(rx_frame_buffer_read_ptr->u8PayloadLength) {
		micromac_packets_read++;
		len = sizeof(tsMacFrame) - 32 * sizeof(uint32) + rx_frame_buffer_read_ptr->u8PayloadLength; //MICROMAC_HEADER_LEN
		//should copy the whole thing or else something wrong happens probably because of alignment
		//XXX check for possibility of overflow: rx_frame_buffer_read_ptr == rx_frame_buffer_write_ptr
		memcpy(buf, rx_frame_buffer_read_ptr, sizeof(tsMacFrame));
		PRINTF(
				"len: %u, u8PayloadLength %u, u16Unused %u\n",
				len, rx_frame_buffer_read_ptr->u8PayloadLength, ((tsMacFrame*)buf)->u16Unused);
		rx_frame_buffer_read_ptr->u8PayloadLength=0;
	}
	switch_rx_read_buffer();
	//  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, cc2420_last_rssi);
	//  packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, cc2420_last_correlation);
	RIMESTATS_ADD(llrx);
	RELEASE_LOCK();
	for (i = 0; i < len; i++) {
		PRINTF("%02x ", ((uint8_t*)buf)[i]);
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
				packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, (uint16_t)micromac_radio_time_of_arrival);
				len = micromac_radio_read(packetbuf_hdrptr(), PACKETBUF_SIZE);
				packetbuf_set_datalen(len);
				NETSTACK_RDC.input();
			}
		}

		PROCESS_END();
	}
	/*---------------------------------------------------------------------------*/
void
micromac_radio_set_txpower(uint8_t power)
{
	GET_LOCK();
	vJPT_RadioSetPower(power);
	RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
int
micromac_radio_get_txpower(void)
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
micromac_radio_detected_energy(void)
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
int
micromac_radio_radio_cca(void)
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
	return bMMAC_RxDetected();
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
void
copy_to_rimeaddress(rimeaddr_t* addr, tuAddr* tu_addr)
{
#if(RIMEADDR_SIZE==8)
	addr->u8[7] = tu_addr->sExt.u32L;
	addr->u8[6] = tu_addr->sExt.u32L >> (uint32_t)8;
	addr->u8[5] = tu_addr->sExt.u32L >> (uint32_t)16;
	addr->u8[4] = tu_addr->sExt.u32L >> (uint32_t)24;
	addr->u8[3] = tu_addr->sExt.u32H;
	addr->u8[2] = tu_addr->sExt.u32H >> (uint32_t)8;
	addr->u8[1] = tu_addr->sExt.u32H >> (uint32_t)16;
	addr->u8[0] = tu_addr->sExt.u32H >> (uint32_t)24;
#elif(RIMEADDR_SIZE==2)
	addr->u8[1] = tu_addr->u16Short;
	addr->u8[0] = tu_addr->u16Short >> (uint16_t)8;
#endif
}
/*---------------------------------------------------------------------------*/
const struct radio_driver micromac_radio_driver = {
	micromac_radio_init,
	micromac_radio_prepare,
#if MICROMAC_RADIO_NO_IRQ
	micromac_radio_transmit,
#else
	micromac_radio_transmit_with_irq,
#endif
	micromac_radio_send,
	micromac_radio_read,
	/* micromac_set_channel, */
	/* micromac_detected_energy, */
	micromac_radio_cca,
	micromac_radio_receiving_packet,
	micromac_radio_pending_packet,
	micromac_radio_on, micromac_radio_off
};
