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
 */

#ifndef __TSCH_PRIVATE_H__
#define __TSCH_PRIVATE_H__

#include "contiki.h"
#include "net/mac/tsch-schedule.h"

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
#include "dev/cc2420-tsch.h"
#endif /* CONTIKI_TARGET */

/* Calculate packet tx/rc duration based on sent packet len assuming 802.15.4 250kbps data rate */
#define PACKET_DURATION(payload_len) (RADIO_TO_RTIMER(((payload_len) + 1) * 2))

/* struct received_frame_radio_s { */
/*  uint8_t* buf; */
/*  uint8_t len; */
/*  rimeaddr_t source_address; */
/*  rtimer_clock_t sfd_timestamp; */
/* }; */

#define NACK_FLAG (0x8000)

/* number of slots to wait before initiating resynchronization */
#define DESYNC_THRESHOLD (ieee154e_vars.current_slotframe->size * 30)
/* number of slots to wait before activating keep-alive mechanism */
#define KEEPALIVE_TIMEOUT (ieee154e_vars.current_slotframe->size * 3)
/* number of slots to wait before applying drift correction */
#define DRIFT_CORRECTION_TIMEOUT  (3)
/* Period of EB */
#define EB_PERIOD (4*CLOCK_SECOND)


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
#define MAC_MIN_BE 1
#define MAC_MAC_FRAME_RETRIES 4
#define MAC_MAC_BE 4

/* IEEE 802.154e TSCH state */
typedef struct {
  volatile asn_t asn;                /* current absolute slot number */
  volatile uint8_t state;              /* state of the FSM */
  uint8_t dsn;                /* data sequence number */
  uint16_t sync_timeout;        /* how many slots left before loosing sync */
  uint8_t mac_ebsn;           /* EB sequence number */
  uint8_t join_priority;      /* inherit from RPL - for PAN coordinator: 0 -- lower is better */
  struct slotframe *current_slotframe;
  volatile rtimer_clock_t start; /* cell start time */
  uint8_t slot_template_id;
  uint8_t hop_sequence_id;
  volatile uint16_t timeslot;
  volatile int16_t registered_drift;
  volatile struct received_frame_radio_s *last_rf;
  volatile struct rtimer t;
  volatile struct pt mpt;
  volatile uint8_t need_ack;
  /* variable to protect queue structure */
  volatile uint8_t working_on_queue;
  uint8_t eb_buf[TSCH_MAX_PACKET_LEN + 1]; /* a buffer for EB packets, last byte for length */

  /* on resynchronization, the node has already joined a RPL network and it is mistaking it with root
   * this flag is used to prevent this */
  volatile uint8_t first_associate;
  volatile int32_t drift_correction;
  volatile int32_t drift; /* estimated drift to all time source neighbors */
  volatile uint16_t drift_counter; /* number of received drift corrections source neighbors */
  uint8_t cell_decison;
  struct tsch_link *cell;
  struct tsch_packet *p;
  struct tsch_neighbor *n;
  void *payload;
  unsigned short payload_len;
  /* 1 byte for length if needed as dictated by the radio driver */
  uint8_t ackbuf[STD_ACK_LEN + SYNC_IE_LEN + 1];
} ieee154e_vars_t;

extern volatile ieee154e_vars_t ieee154e_vars;

struct tsch_link {
  /* Unique identifier (local to specified slotframe) for the link */
  uint16_t link_handle;
  /* Relative number of slot in slotframe */
  /* uint16_t timeslot; */
  /* maybe 0 to 15 */
  uint8_t channel_offset;
  /*b0 = Transmit, b1 = Receive, b2 = Shared, b3= Timekeeping, b4–b7 reserved.*/
  uint8_t link_options;
  /* Type of link. NORMAL = 0. ADVERTISING = 1, and indicates
     the link may be used to send an Enhanced beacon. */
  uint8_t link_type;
  /* short address of neighbor */
  rimeaddr_t *node_address;
};

struct slotframe {
  /* Unique identifier */
  uint16_t slotframe_handle;
  uint16_t size;
  uint16_t on_size;
  struct tsch_link **links;
};

enum cell_decision_enum {
  CELL_OFF = 0,
  CELL_TX = 1,
  CELL_TX_IDLE = 2, /* No packet to transmit */
  CELL_TX_BACKOFF = 3, /* csma backoff */
  CELL_RX = 4,
};

#endif /* __TSCH_PRIVATE_H__ */
