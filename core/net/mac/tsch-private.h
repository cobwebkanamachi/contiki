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
 *         Private TSCH definitions
 *         (meant for use by TSCH implementation files only)
 * \author
 *         Beshr Al Nahas <beshr@sics.se>
 *         Simon Duquennoy <simonduq@sics.se>
 */

#ifndef __TSCH_PRIVATE_H__
#define __TSCH_PRIVATE_H__

#include "contiki.h"
#include "rimeaddr.h"
/* TODO: move platform-specific code away from core */
#if CONTIKI_TARGET_JN5168
#define CONVERT_DRIFT_US_TO_RTIMER(D, DC) ((uint32_t)(D) * 16UL) / ((uint32_t)(DC));
#define RTIMER_TO_US(T)   ((T) >> 4UL)
#define ENABLE_DELAYED_RF 1
#include "dev/micromac-radio.h"
#undef putchar
void uart0_writeb(unsigned char c);
#define putchar uart0_writeb
#else /* Leave CC2420 as default */
#define CONVERT_DRIFT_US_TO_RTIMER(D, DC) (((D) * 100UL) / (3051UL * (DC)));
/* Do the math in 32bits to save precision */
#define RTIMER_TO_US(T)   (((uint32_t)(T) * 3051UL) / (uint32_t)100UL)
#define RADIO_TO_RTIMER(X) 	((rtimer_clock_t)( (uint32_t)((uint32_t)(X)*524)/(uint32_t)1000 ))
#include "dev/cc2420.h"
#endif /* CONTIKI_TARGET */

/* Calculate packet tx/rc duration based on sent packet len assuming 802.15.4 250kbps data rate */
#define PACKET_DURATION(payload_len) (RADIO_TO_RTIMER(((payload_len) + 1) * 2))


#define NACK_FLAG (0x8000)
/* number of slots to wait before initiating resynchronization */
#define DESYNC_THRESHOLD 2048
/* number of slots to wait before activating keep-alive mechanism */
#define KEEPALIVE_TIMEOUT 256
/* number of slots to wait before applying drift correction */
#define DRIFT_CORRECTION_TIMEOUT  (3)
/* Period of EB */
#define EB_PERIOD (4*CLOCK_SECOND)


/* TODO: move platform-specific code away from core */
#if CONTIKI_TARGET_JN5168
#pragma __TSCH_PARAMETERS_H__ CONTIKI_TARGET_JN5168
#define TRIVIAL_DELAY (100 * 16UL) /* 50us */
/* Atomic durations */
/* expressed in 16MHz ticks: */
/*    - ticks = duration_in_seconds * 16000000 */
/*    - duration_in_seconds = ticks / 16000000 */

/* XXX check these numbers on real hw or cooja */
/* 164*3 as PORT_TsSlotDuration causes 147.9448us drift every slotframe ==4.51 ticks */
/* 15000us */
#define PORT_TsSlotDuration (16 * 15000UL)
/*   100us */
#define PORT_maxTxDataPrepare (1600)
/*   100us */
#define PORT_maxRxAckPrepare (1600)
/*   100us */
#define PORT_maxRxDataPrepare (1600)

#define PORT_maxTxAckPrepare (1600)

/* ~327us+129preample */
#define PORT_delayTx (16 * 260UL)
/* ~50us delay + 129preample + ?? */
#define PORT_delayRx (16 * 50UL)

/* time-slot related */
#define TsCCAOffset (16 * 1000UL) /* 1000us //98,										//3000us */
#define TsCCA (16 * 500UL)                        /* ~500us */
#define TsRxTx (16 * 500UL)                       /* 500us */
#define TsTxOffset (16 * 4000UL)                  /*  4000us */
#define TsLongGT (16 * 1300UL)                  /*  1300us */
#define TsTxAckDelay (16 * 4000UL)                  /*  4000us */
/*	TsTxAckDelay = 99,                  //  3000us */
#define TsShortGT (16 * 500UL)                  /*   500us */
/*	TsShortGT = 32,                  //  1000us */
#define TsSlotDuration (PORT_TsSlotDuration)  /* 15000us */
/* execution speed related */
#define maxTxDataPrepare (PORT_maxTxDataPrepare)
#define maxRxAckPrepare (PORT_maxRxAckPrepare)
#define maxRxDataPrepare (PORT_maxRxDataPrepare)
#define maxTxAckPrepare (PORT_maxTxAckPrepare)
/* radio speed related */
#define delayTx (PORT_delayTx)         /* between GO signal and SFD: radio fixed delay + 4Bytes preample + 1B SFD -- 1Byte time is 32us */
#define delayRx (PORT_delayRx)         /* between GO signal and start listening */
/* radio watchdog */
#define wdRadioTx (16 * 1000UL)                  /*  1000us (needs to be >delayTx) */
#define wdDataDuration (16 * 4300UL)            /*  4500us (measured 4280us with max payload) */
#define wdAckDuration (16 * 500UL)                  /*  600us (measured 1000us me: 440us) */

/* enum ieee154e_atomicdurations_enum { */
/*	// time-slot related */
/*	TsCCAOffset= 16*1000, //1000us //98,										//3000us */
/*	TsCCA=16*500,												//~500us */
/*	TsRxTx=16*500,												//500us */
/*	TsTxOffset = 16*4000,                  //  4000us */
/*	TsLongGT = 16*1300,                  //  1300us */
/*	TsTxAckDelay = 16*4000,                  //  4000us */
/* //	TsTxAckDelay = 99,                  //  3000us */
/*	TsShortGT = 16*500,                  //   500us */
/* //	TsShortGT = 32,                  //  1000us */
/*	TsSlotDuration = PORT_TsSlotDuration,  // 15000us */
/*	// execution speed related */
/*	maxTxDataPrepare = PORT_maxTxDataPrepare, */
/*	maxRxAckPrepare = PORT_maxRxAckPrepare, */
/*	maxRxDataPrepare = PORT_maxRxDataPrepare, */
/*	maxTxAckPrepare = PORT_maxTxAckPrepare, */
/*	// radio speed related */
/*	delayTx = PORT_delayTx,         // between GO signal and SFD: radio fixed delay + 4Bytes preample + 1B SFD -- 1Byte time is 32us */
/*	delayRx = PORT_delayRx,         // between GO signal and start listening */
/*	// radio watchdog */
/*	wdRadioTx = 16*1000,                  //  1000us (needs to be >delayTx) */
/*	wdDataDuration = (129<<8),            //  4500us (measured 4280us with max payload) */
/*	wdAckDuration = (9<<8),                  //  600us (measured 1000us me: 440us) */
/* }; */
#else
#define TRIVIAL_DELAY 5
/* Atomic durations */
/* expressed in 32kHz ticks: */
/*    - ticks = duration_in_seconds * 32768 */
/*    - duration_in_seconds = ticks / 32768 */

/* XXX check these numbers on real hw or cooja */
/* 164*3 as PORT_TsSlotDuration causes 147.9448us drift every slotframe ==4.51 ticks */
/* 15000us */
#define PORT_TsSlotDuration (164 * 3)
/*   1200us */
#define PORT_maxTxDataPrepare (38)
/*   600us */
#define PORT_maxRxAckPrepare (19)
/*   600us */
#define PORT_maxRxDataPrepare (19)

#define PORT_maxTxAckPrepare (21)

/* ~327us+129preample */
#define PORT_delayTx (15)
/* ~50us delay + 129preample + ?? */
#define PORT_delayRx (7)

enum ieee154e_atomicdurations_enum {
  /* time-slot related */
  TsCCAOffset = 33, /* 1000us //98,										//3000us */
  TsCCA = 14,                     /* ~500us */
  TsRxTx = 16,                      /* 500us */
  TsTxOffset = 131,                  /*  4000us */
  TsLongGT = 43,                  /*  1300us */
  TsTxAckDelay = 131,                  /*  4000us */
  /*	TsTxAckDelay = 99,                  //  3000us */
  TsShortGT = 16,                  /*   500us */
  /*	TsShortGT = 32,                  //  1000us */
  TsSlotDuration = PORT_TsSlotDuration,  /* 15000us */
  /* execution speed related */
  maxTxDataPrepare = PORT_maxTxDataPrepare,
  maxRxAckPrepare = PORT_maxRxAckPrepare,
  maxRxDataPrepare = PORT_maxRxDataPrepare,
  maxTxAckPrepare = PORT_maxTxAckPrepare,
  /* radio speed related */
  delayTx = PORT_delayTx,         /* between GO signal and SFD: radio fixed delay + 4Bytes preample + 1B SFD -- 1Byte time is 32us */
  delayRx = PORT_delayRx,         /* between GO signal and start listening */
  /* radio watchdog */
  wdRadioTx = 33,                  /*  1000us (needs to be >delayTx) */
  wdDataDuration = 148,            /*  4500us (measured 4280us with max payload) */
  wdAckDuration = 21,                  /*  600us (measured 1000us me: 440us) */
};
#endif

#define TSCH_MAX_PACKET_LEN 127

typedef uint64_t asn_t;

#define STD_ACK_LEN 3
#define SYNC_IE_LEN 4

/* TSCH MAC parameters */
#define MAC_MIN_BE 0
#define MAC_MAX_FRAME_RETRIES 4
#define MAC_MAX_BE 4

/* 802.15.4 broadcast MAC address */
const rimeaddr_t tsch_broadcast_address;
/* The address we use to identify EB queue */
const rimeaddr_t tsch_eb_address;

/* The current Absolute Slot Number (ASN) */
asn_t current_asn;
uint8_t tsch_join_priority;

/* A pseudo-random generator with better properties than msp430-libc's default */
void tsch_random_init(uint32_t x);
uint8_t tsch_random_byte(uint8_t window);

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif /* MIN */

#endif /* __TSCH_PRIVATE_H__ */
