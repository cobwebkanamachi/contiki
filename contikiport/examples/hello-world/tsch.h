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
 *         TSCH.
 * \author
 *         Beshr Al Nahas <beshr@sics.se>
 */

#ifndef __TSCH_H__
#define __TSCH_H__
#include "contiki-conf.h"

#include "net/mac/rdc.h"

typedef uint32_t asn_t;

typedef struct {
	asn_t asn;                // current absolute slot number
	uint8_t state;              // state of the FSM
	uint8_t dsn;                // data sequence number
	uint16_t capturedTime;       // last captures time
	uint16_t syncTimeout;        // how many slots left before looses sync
	uint8_t isSync;             // TRUE iff mote synchronized to network
//   OpenQueueEntry_t*  dataToSend;         // pointer to the data to send
//   OpenQueueEntry_t*  dataReceived;       // pointer to the data received
//   OpenQueueEntry_t*  ackToSend;          // pointer to the ack to send
//   OpenQueueEntry_t*  ackReceived;        // pointer to the ack received
} ieee154e_vars_t;

// Atomic durations
// expressed in 32kHz ticks:
//    - ticks = duration_in_seconds * 32768
//    - duration_in_seconds = ticks / 32768

//XXX check these numbers on real hw or cooja
// 15000us
#define PORT_TsSlotDuration (164*3)
//   500us
#define PORT_maxTxDataPrepare (16)
//   500us
#define PORT_maxRxAckPrepare (16)
//   500us
#define PORT_maxRxDataPrepare (16)

#define PORT_maxTxAckPrepare (16)

#define PORT_delayTx (16)

#define PORT_delayRx (16)

enum ieee154e_atomicdurations_enum {
	// time-slot related
	TsCCAOffset=100,										//>3000us
	TsCCA=16,												//500us
	TsRxTx=15,												//<500us
	TsTxOffset = 131,                  //  4000us
	TsLongGT = 43,                  //  1300us
	TsTxAckDelay = 131,                  //  4000us
	TsShortGT = 16,                  //   500us
	TsSlotDuration = PORT_TsSlotDuration,  // 15000us
	// execution speed related
	maxTxDataPrepare = PORT_maxTxDataPrepare,
	maxRxAckPrepare = PORT_maxRxAckPrepare,
	maxRxDataPrepare = PORT_maxRxDataPrepare,
	maxTxAckPrepare = PORT_maxTxAckPrepare,
	// radio speed related
	delayTx = PORT_delayTx,         // between GO signal and SFD
	delayRx = PORT_delayRx,         // between GO signal and start listening
	// radio watchdog
	wdRadioTx = 33,                  //  1000us (needs to be >delayTx)
	wdDataDuration = 164,            //  5000us (measured 4280us with max payload)
	wdAckDuration = 98,                  //  3000us (measured 1000us)
};
extern const struct rdc_driver tschrdc_driver;

#endif /* __TSCH_H__ */
