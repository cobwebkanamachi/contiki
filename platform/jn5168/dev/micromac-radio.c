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
#include "lib/crc16.h"
#include "net/mac/frame802154.h"
#include "lib/list.h"
#include "lib/memb.h"

#undef putchar
void uart0_writeb(unsigned char c);
#define putchar uart0_writeb
#define PUTCHAR(X) do { putchar(X); putchar('\n'); } while(0);

#define CHECKSUM_LEN 2

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

#ifndef RF_CHANNEL
#define RF_CHANNEL (26)
#endif
//#if RF_CHANNEL!= 15
//#error "!!"
//#endif
//vMMAC_SetCutOffTimer(uint32 u32CutOffTime, bool_t bEnable);
/* XXX JN5168_CONF_CCA_THRESH has an arbitrary value */
// an integer between 0 and 255
#ifndef JN5168_CONF_CCA_THRESH
#define JN5168_CONF_CCA_THRESH 127
#endif /* JN5168_CONF_CCA_THRESH */

#define WITH_SEND_CCA 0

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
volatile rtimer_clock_t micromac_radio_time_of_arrival, micromac_radio_time_of_departure;

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


static volatile uint8_t phy_rx_attempt = 0;
static tsPhyFrame tx_frame_buffer, phy_rx, phy_ackbuf;
static volatile uint8_t extrabuf[ACK_LEN]={0};
static volatile struct received_frame_radio_s last_rf;
#define RX_LIST_SIZE 4
/* Data structure used as the internal RX buffer */
struct rx_frame {
	struct rx_frame * next;
	tsMacFrame buffer;
};
typedef struct rx_frame rx_frame_t;
static volatile rx_frame_t *rf = NULL;
MEMB(rf_memb, rx_frame_t, RX_LIST_SIZE);
LIST(rf_list);
/*---------------------------------------------------------------------------*/
PROCESS(micromac_radio_process, "micromac_radio_driver");
/*---------------------------------------------------------------------------*/


volatile static uint8_t tx_in_progress = 0;

volatile static uint8_t rx_in_progress = 0;

static int channel;
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
//	vMMAC_SetCutOffTimer(u32MMAC_GetTime() + u32_delay_time + u32_on_duration, TRUE);
	vMMAC_StartMacReceive(&rf->buffer,
			E_MMAC_RX_DELAY_START
			| MMAC_RX_AUTO_ACK_CONF
			| E_MMAC_RX_ALLOW_MALFORMED
			| E_MMAC_RX_NO_FCS_ERROR
			| E_MMAC_RX_ADDRESS_MATCH);
//	vMMAC_SetCutOffTimer(0, FALSE);
	phy_rx_attempt = 0;
}
/*---------------------------------------------------------------------------*/
static void
on(void)
{
	micromac_radio_sfd_sync();
	vMMAC_StartMacReceive(&rf->buffer,
			E_MMAC_RX_START_NOW
			| MMAC_RX_AUTO_ACK_CONF
			| E_MMAC_RX_ALLOW_MALFORMED
			| E_MMAC_RX_NO_FCS_ERROR
			| E_MMAC_RX_ADDRESS_MATCH);
	phy_rx_attempt = 0;
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
	vMMAC_SetCutOffTimer(0, FALSE);
#if !MICROMAC_RADIO_ALWAYS_ON
	vMMAC_RadioOff();
#endif
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
int
micromac_radio_raw_rx_on(void)
{
	GET_LOCK();
	rx_in_progress = 1;
	phy_rx_attempt = 1;
	micromac_radio_sfd_sync();
	vMMAC_StartPhyReceive(&phy_rx,
			E_MMAC_RX_START_NOW
			| MMAC_RX_AUTO_ACK_CONF
			| E_MMAC_RX_ALLOW_MALFORMED
			| E_MMAC_RX_NO_FCS_ERROR
			| E_MMAC_RX_ADDRESS_MATCH);
	RELEASE_LOCK();
	return 1;
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
static void
extract_sender_address(struct received_frame_radio_s* frame) {
	frame802154_fcf_t fcf;
	int c;

	if(frame->len < 3) {
		return;
	}

	uint8_t * p = frame->buf;

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
			rimeaddr_copy(&frame->source_address, &rimeaddr_null);
			frame->source_address.u8[0] = p[1];
			frame->source_address.u8[1] = p[0];
			p += 2;
		} else if(fcf.src_addr_mode == FRAME802154_LONGADDRMODE) {
			for(c = 0; c < 8; c++) {
				frame->source_address.u8[c] = p[7-c];
			}
			p += 8;
		}
	} else {
		rimeaddr_copy(&frame->source_address, &rimeaddr_null);
	}
}
/*---------------------------------------------------------------------------*/
void
micromac_radio_interrupt(uint32 mac_event)
{
	uint8_t* ackbuf=NULL;
	uint8_t need_ack = 0, nack = 1, seqno=0;
	uint16_t checksum = 0;
	rx_frame_t *tmp_rf = NULL;
//	printf(
//					"micromac_radio_process_packet: rx_overflow %u, rx_ackneeded %u, rx_noackneeded %u, rx_state %u, rx_complete %d, rx_error %d, rx_malformed %d, rx_aborted %d, packets_seen %d, rx_garbage %u,\nsfd_counter %d, noacktx %u, acktx %u, tx_completed %u, contentiondrop %u, sendingdrop %u\n",
//				rx_overflow, rx_ackneeded, rx_noackneeded, rx_state,
//					rx_complete, rx_error, rx_malformed, rx_aborted,
//					micromac_radio_packets_seen, micromac_radio_rx_garbage,
//					micromac_radio_sfd_counter, RIMESTATS_GET(noacktx), RIMESTATS_GET(acktx),
//					micromac_radio_tx_completed, RIMESTATS_GET(contentiondrop),
//					RIMESTATS_GET(sendingdrop));
	if (mac_event & E_MMAC_INT_TX_COMPLETE) { /* Transmission attempt has finished */
		tx_in_progress = 0;
		micromac_radio_tx_completed++;
//		PUTCHAR('X');
		//XXX: Always ON!!
#if MICROMAC_RADIO_ALWAYS_ON
		on();
#endif
	} else if (mac_event & (E_MMAC_INT_RX_HEADER|E_MMAC_INT_RX_COMPLETE)) { /* MAC header has been received */
//		PUTCHAR('R');
		micromac_radio_sfd_counter++;
		last_packet_timestamp = u32MMAC_GetRxTime();
		rx_state = u32MMAC_GetRxErrors();
		/* we don't have free memory. */
		if(!rf) {
			rx_state =E_MMAC_RXSTAT_ERROR;
			PUTCHAR('@');
		}
		/* if rx is successful */
		if (!rx_state) {
			micromac_radio_packets_seen++;
			if(!phy_rx_attempt) {
				/* Allocate space to store the next received frame, and store the current frame in read list */
				tmp_rf=memb_alloc(&rf_memb);
				if(tmp_rf != NULL) {
				  	nack = 0;
				}
				need_ack = (rf->buffer.u16FCF >> 5) & 1;
				seqno = rf->buffer.u8SequenceNum;
				last_packet_end_timestamp = last_packet_timestamp + rf->buffer.u8PayloadLength +1;
			} else {
				need_ack = (phy_rx.uPayload.au8Byte[1] >> 5) & 1;
				seqno = phy_rx.uPayload.au8Byte[2];
				last_packet_end_timestamp = last_packet_timestamp + phy_rx.u8PayloadLength +1;
			}
			micromac_radio_time_of_arrival = RADIO_TO_RTIMER(last_packet_timestamp)-RADIO_TO_RTIMER(u32MMAC_GetTime())+RTIMER_NOW();
			rx_end_time = RADIO_TO_RTIMER(last_packet_end_timestamp)-RADIO_TO_RTIMER(u32MMAC_GetTime())+RTIMER_NOW();;

			if(need_ack) {
				rx_ackneeded++;
			 if(softack_make_callback != NULL) {
				 /* first byte in ackbuf defines frame length */
				 softack_make_callback(&ackbuf, seqno, micromac_radio_time_of_arrival, nack);
			 } else { /* construct standard ack */
				 ackbuf = extrabuf;
				 ackbuf[0] = 3;
				 ackbuf[1] = 0x02;
				 ackbuf[2] = 0;
				 ackbuf[3] = seqno;
			 }
			 /* copy ACK to phy_buf */
			 phy_ackbuf.u8PayloadLength = ackbuf[0];
			 int i;
			 for(i=0; i<phy_ackbuf.u8PayloadLength; i++) {
				 phy_ackbuf.uPayload.au8Byte[i]=ackbuf[i+1];
			 }
			 checksum = crc16_data(phy_ackbuf.uPayload.au8Byte, phy_ackbuf.u8PayloadLength, 0);
			 phy_ackbuf.uPayload.au8Byte[i++] = checksum;
			 phy_ackbuf.uPayload.au8Byte[i++] = ( checksum >> 8) & 0xff;
			 phy_ackbuf.u8PayloadLength += CHECKSUM_LEN;
			} else {
				rx_noackneeded++;
			}
			/* E_MMAC_INT_RX_COMPLETE */
			rx_complete++;

			if(interrupt_exit_callback != NULL) {
				if(!phy_rx_attempt) {
					 last_rf.buf = rf->buffer.uPayload.au8Byte;
					 last_rf.len = rf->buffer.u8PayloadLength;
					 copy_to_rimeaddress(&(last_rf.source_address), &rf->buffer.uSrcAddr);
				} else {
					 last_rf.buf = phy_rx.uPayload.au8Byte;
					 last_rf.len = phy_rx.u8PayloadLength;
					 extract_sender_address(&last_rf);
				}
			 last_rf.sfd_timestamp = micromac_radio_time_of_arrival;
			 interrupt_exit_callback(need_ack, &last_rf);
			}

			/* move rf to the recently allocated slot in rf memory block */
			if(!phy_rx_attempt && tmp_rf) {
		  	list_add(rf_list, rf);
				rf=tmp_rf;
				pending++;
				process_poll(&micromac_radio_process);
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
		rx_in_progress = 0;
		//XXX: Always ON!!
#if MICROMAC_RADIO_ALWAYS_ON
//		if (need_ack)
		{
			on();
		}
#endif
	}
//	else if (mac_event & E_MMAC_INT_RX_COMPLETE) { /* Complete frame has been received and ACKed*/
////		PUTCHAR('L');
//		//rx_in_progress = 0;
//
//	}
}
/*---------------------------------------------------------------------------*/
int
micromac_radio_read_ack(void *buf, int alen) {
	uint8_t len = 0, i = 0;
	BUSYWAIT_UNTIL(u32MMAC_PollInterruptSource(E_MMAC_INT_TX_COMPLETE), 16 * 500); //TsShortGT
	if(phy_rx_attempt) {
		len = phy_rx.u8PayloadLength;
		alen = (len > alen) ? alen : len;
		if (buf && len >= ACK_LEN) {
			//memcpy(buf, &phy_rx->uPayload.au8Byte, alen);
			for (i = 0; i < alen; i++) {
				((uint8_t*)buf)[i] = phy_rx.uPayload.au8Byte[i];
			}
		}
		phy_rx.u8PayloadLength=0;
	} else {
		len = rf->buffer.u8PayloadLength;
		alen = (len > alen) ? alen : len;
		if (buf && len >= ACK_LEN) {
			//memcpy(buf, &phy_rx->uPayload.au8Byte, alen);
			for (i = 0; i < alen; i++) {
				((uint8_t*)buf)[i] = rf->buffer.uPayload.au8Byte[i];
			}
		}
//		rf->u8PayloadLength=0;
	}
	return len;
}
/*---------------------------------------------------------------------------*/
void
micromac_radio_send_ack(void) {
//  if(!locked) {
//		GET_LOCK();
		tx_in_progress = 1;
		vMMAC_StartPhyTransmit(&phy_ackbuf,
				E_MMAC_TX_START_NOW
				| E_MMAC_TX_NO_AUTO_ACK
				| E_MMAC_TX_NO_CCA);
//		vMMAC_StartMacTransmit(&phy_ackbuf,
//				E_MMAC_TX_START_NOW
//				| E_MMAC_TX_NO_AUTO_ACK
//				| E_MMAC_TX_NO_CCA);
		tx_in_progress = 0;
//		RELEASE_LOCK();
//  }
  rx_end_time = 0;
}
/*---------------------------------------------------------------------------*/
void
micromac_radio_send_ack_delayed(uint32 u32_delay_time)
{
//  if(!locked) {
//		GET_LOCK();
		tx_in_progress = 1;
		vMMAC_SetTxStartTime(micromac_radio_get_rx_radio_end_time() + u32_delay_time);
		vMMAC_StartPhyTransmit(&phy_ackbuf,
				E_MMAC_TX_DELAY_START
				| E_MMAC_TX_NO_AUTO_ACK
				| E_MMAC_TX_NO_CCA);
//		vMMAC_StartMacTransmit(&phy_ackbuf,
//				E_MMAC_TX_DELAY_START
//				| E_MMAC_TX_NO_AUTO_ACK
//				| E_MMAC_TX_NO_CCA);
		tx_in_progress = 0;
//		RELEASE_LOCK();
//  }
  rx_end_time = 0;
}
/*---------------------------------------------------------------------------*/
volatile static uint16_t u16PanId = IEEE802154_PANID;
volatile static uint16_t u16ShortAddress;
/*---------------------------------------------------------------------------*/
int
micromac_radio_init(void)
{
	uint8_t u8TxAttempts = 1, /* 1 transmission attempt without ACK */
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
  memb_init(&rf_memb);
  list_init(rf_list);
	/* Allocate space to store the received frame */
	rf=memb_alloc(&rf_memb);
	process_start(&micromac_radio_process, NULL);
	PRINTF("micromac_radio init: RXAddress %04x == %08x.%08x @ PAN %04x, channel: %d, u32JPT_Init: %d\n", u16ShortAddress, psExtAddress.u32H, psExtAddress.u32L, u16PanId, channel, jpt_ver);
	return 1;
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_transmit_with_irq(unsigned short payload_len)
{
	if (tx_in_progress) {
		PUTCHAR('#');
		return RADIO_TX_COLLISION;
	}
	GET_LOCK();
	tx_in_progress = 1;
//	vMMAC_StartMacTransmit(&tx_frame_buffer,
//			E_MMAC_TX_START_NOW
//			| MMAC_TX_AUTO_ACK_CONF
//			| E_MMAC_TX_NO_CCA);
	vMMAC_StartPhyTransmit(&tx_frame_buffer,
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
		PUTCHAR('#');
		return RADIO_TX_COLLISION;
	}
	GET_LOCK();
	tx_in_progress = 1;
//	vMMAC_StartMacTransmit(&tx_frame_buffer,
//			E_MMAC_TX_START_NOW
//			| MMAC_TX_AUTO_ACK_CONF
//			| E_MMAC_TX_NO_CCA);
	vMMAC_StartPhyTransmit(&tx_frame_buffer,
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
//	vMMAC_StartMacTransmit(&tx_frame_buffer,
//			E_MMAC_TX_DELAY_START
//			| MMAC_TX_AUTO_ACK_CONF
//			| E_MMAC_TX_NO_CCA);
	vMMAC_StartPhyTransmit(&tx_frame_buffer,
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
		PUTCHAR('#');
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
	int i;
	uint16_t checksum;

	RIMESTATS_ADD(lltx);

	if (tx_in_progress) {
		PUTCHAR('$');
		return 1;
	}
	if(!payload) {
		PUTCHAR('=');
		while(1){};//XXX
		return 1;
	}
	GET_LOCK();
	/* copy payload to (soft) tx buffer */
//	memset(&tx_frame_buffer, 0, sizeof(tx_frame_buffer));
//	memcpy(&(tx_frame_buffer.uPayload.au8Byte), payload, payload_len);
	for(i=0;i<payload_len;i++) {

		tx_frame_buffer.uPayload.au8Byte[i] = ((uint8_t*)payload)[i];
	}
  checksum = crc16_data(payload, payload_len, 0);
  tx_frame_buffer.uPayload.au8Byte[i++] = checksum;
  tx_frame_buffer.uPayload.au8Byte[i++] = ( checksum >> 8) & 0xff;
	tx_frame_buffer.u8PayloadLength=payload_len+CHECKSUM_LEN;
//	tx_frame_buffer.u8PayloadLength = payload_len - MICROMAC_HEADER_LEN;
//	tx_frame_buffer.u16Unused = tx_frame_buffer.u8PayloadLength % 4;

	PRINTF(
				"micromac_radio: sending packetbuf_totlen() %u payload_len %dB u8PayloadLength %u + packetbuf_datalen %u, rx_overflow %u, rx_ackneeded %u, rx_noackneeded %u, rx_state %u, rx_complete %d, rx_error %d, rx_malformed %d, rx_aborted %d, packets_seen %d, rx_garbage %u,\nsfd_counter %d, noacktx %u, acktx %u, tx_completed %u, contentiondrop %u, sendingdrop %u\n",
				packetbuf_totlen(), payload_len, tx_frame_buffer.u8PayloadLength,
				packetbuf_datalen(), rx_overflow, rx_ackneeded, rx_noackneeded, rx_state,
				rx_complete, rx_error, rx_malformed, rx_aborted,
				micromac_radio_packets_seen, micromac_radio_rx_garbage,
				micromac_radio_sfd_counter, RIMESTATS_GET(noacktx), RIMESTATS_GET(acktx),
				micromac_radio_tx_completed, RIMESTATS_GET(contentiondrop),
				RIMESTATS_GET(sendingdrop));
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
		PUTCHAR('#');
			return RADIO_TX_COLLISION;
		}
		GET_LOCK();
		vMMAC_SetTxStartTime(u32MMAC_GetTime() + u32_delay_time);
		tx_in_progress = 1;
		vMMAC_StartPhyTransmit(&tx_frame_buffer,
				E_MMAC_TX_DELAY_START
				| MMAC_TX_AUTO_ACK_CONF
				| E_MMAC_TX_NO_CCA);
//		vMMAC_StartMacTransmit(&tx_frame_buffer, E_MMAC_TX_DELAY_START
//				| MMAC_TX_AUTO_ACK_CONF | E_MMAC_TX_NO_CCA);

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
		vMMAC_GetMacAddress( psMacAddr );
	}
	vMMAC_SetRxAddress(u16PanId, u16ShortAddress, psMacAddr);
	RELEASE_LOCK();
}
/*---------------------------------------------------------------------------*/
static int
micromac_radio_read(void *buf, unsigned short bufsize)
{
	//XXX disable printf
	GET_LOCK();
	int i, len=0;
	rx_frame_t * rx_frame_buffer_read_ptr = list_pop(rf_list);
  if(rx_frame_buffer_read_ptr == NULL) {
    RELEASE_LOCK();
    dbg_printf("micromac_radio_read rx_frame_buffer_read_ptr == NULL\n");
    return 0;
  } else {
    if(list_head(rf_list) != NULL) {
      /* If there are other packets pending, poll */
      process_poll(&micromac_radio_process);
    }

		if(pending>=RX_LIST_SIZE) {
			rx_overflow+=pending-RX_LIST_SIZE+1;
			pending = RX_LIST_SIZE-1;
		}
		len = rx_frame_buffer_read_ptr->buffer.u8PayloadLength;

		if(len) {
			micromac_packets_read++;
			len = sizeof(tsMacFrame) - 32 * sizeof(uint32) + rx_frame_buffer_read_ptr->buffer.u8PayloadLength; //MICROMAC_HEADER_LEN
			//should copy the whole thing or else something wrong happens probably because of alignment
			memcpy(buf, &rx_frame_buffer_read_ptr->buffer, sizeof(tsMacFrame));
		}

		//  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, cc2420_last_rssi);
		//  packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, cc2420_last_correlation);
		PRINTF("micromac_radio: reading %d bytes\n", len);

    memb_free(&rf_memb, rx_frame_buffer_read_ptr);
		RIMESTATS_ADD(llrx);
		RELEASE_LOCK();
  }
	return len;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(micromac_radio_process, ev, data)
{

	PROCESS_BEGIN();
		int len, frame_type;
		PRINTF("micromac_radio_process: started\n");

		while(1) {
			PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

			packetbuf_clear();
			/* XXX PACKETBUF_ATTR_TIMESTAMP is 16bits while last_packet_timestamp is 32bits*/
			packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, micromac_radio_time_of_arrival>>16);
			len = micromac_radio_read(packetbuf_hdrptr(), PACKETBUF_SIZE);
//	    frame_type = ((((tsMacFrame*)packetbuf_hdrptr())->u16FCS)>>8) & 7;
//	    if(frame_type == FRAME802154_ACKFRAME) {
//	      len = 0;
//	    }
			dbg_printf("micromac_radio_process: calling receiver callback %u\n", len);
			packetbuf_set_datalen(len);
			NETSTACK_RDC.input();
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
//	return pending;
	return list_head(rf_list) != NULL;
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
