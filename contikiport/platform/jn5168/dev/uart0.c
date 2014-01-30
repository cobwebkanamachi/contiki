/*
 * Copyright (c) 2011
 * Telecooperation Office (TecO), Universitaet Karlsruhe (TH), Germany.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. Neither the name of the Universitaet Karlsruhe (TH) nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author(s): Philipp Scholl <scholl@teco.edu>
 */

#include <jendefs.h>
#include <AppHardwareApi.h>
#include <PeripheralRegs.h>
#include "dev/uart0.h"
#include "lib/ringbuf.h"
#include "dev/watchdog.h"

#define UART_MCR_OFFSET   0x10
#define UART_LCR_OFFSET   0x0C
#define UART_DLM_OFFSET   0x04
#define UART_EFR_OFFSET   0x20

#define TXBUFSIZE 256

#define u8Uart E_AHI_UART_0

static struct ringbuf txbuf;
static uint8_t txbuf_data[TXBUFSIZE];
static volatile uint8_t transmitting;

static bool_t bOpen = FALSE;

volatile static int (*uart0_input)(unsigned char c);

static void
vUartISR(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
	static uint8 u8UartL;
	static uint32 u32Bytes;

	switch(u32DeviceId){
	case E_AHI_DEVICE_UART0:
		u8UartL = E_AHI_UART_0;
		break;
	case E_AHI_DEVICE_UART1:
		u8UartL = E_AHI_UART_1;
		break;
	default:
		return;
	}

	if (u32ItemBitmap == E_AHI_UART_INT_RXDATA || u32ItemBitmap == E_AHI_UART_INT_TIMEOUT) {
		if (uart0_input)
			uart0_input(u8AHI_UartReadData(u8UartL));
		else
			u8AHI_UartReadData(u8UartL);
	} else if (u32ItemBitmap == E_AHI_UART_INT_TX) {
		u32Bytes = 0;
		/*
		 * if there is data in buffer waiting for tx and we've not filled the
		 * hardware fifo up
		 */
		transmitting = 1;
		while (u32Bytes++<TXBUFSIZE && ringbuf_elements(&txbuf)) {
//		if(ringbuf_elements(&txbuf)) {
			vAHI_UartWriteData(u8Uart, ringbuf_get(&txbuf)); /* write one byte to the UART */
		}
		transmitting = 0;
	}
}

void
uart0_set_input(int
(*input)(unsigned char c))
{
	uart0_input = input;
}

void
uart0_writeb(unsigned char c)
{
	//watchdog_periodic();
	/* push into ringbuf until there is space */
	while (!ringbuf_put(&txbuf, c));
	UART_vPTSChar(ringbuf_get(&txbuf));
//	while(u16AHI_UartReadTxFifoLevel(u8Uart) >= TXBUFSIZE -1);
//	vAHI_UartWriteData(u8Uart, c);
	//watchdog_periodic();
	/* push into ringbuf until there is space */
//	while (!ringbuf_put(&txbuf, c));
//	/*
//	 * if there is already a tx in progress, we can expect a TX interrupt
//	 * some time in the future, in which case the data we wrote to the tx
//	 * buffer will get sent in due course to the UART in the interrupt
//	 * service routine.
//	 * if there is no tx in progress, there won't be a tx interrupt, and the
//	 * byte won't get read from the buffer in the ISR, so we must write it
//	 * to the UART tx FIFO here.
//	 */
//	if ( !transmitting && (u8AHI_UartReadLineStatus(u8Uart)
//			& (E_AHI_UART_LS_THRE | E_AHI_UART_LS_TEMT)) == (E_AHI_UART_LS_THRE | E_AHI_UART_LS_TEMT)) {
//			transmitting = 1;
//			vAHI_UartWriteData(u8Uart, ringbuf_get(&txbuf));
//			transmitting = 0;
//	}
}

void
uart0_init(uint8_t br)
{
//	transmitting = 0;
	ringbuf_init(&txbuf, txbuf_data, TXBUFSIZE);
//	//ringbuf_init(&rxbuf, rxbuf_data, sizeof(rxbuf_data));
//
//	/* Disable use of CTS/RTS */
//	vAHI_UartSetRTSCTS(E_AHI_UART_0, false);
//	/* Enable UART */
//	//vAHI_UartEnable(u8Uart);
//	bAHI_UartEnable(u8Uart, txbuf_data, TXBUFSIZE, NULL, 0);
//	vAHI_UartReset(u8Uart, TRUE, TRUE);
//	vAHI_UartSetAutoFlowCtrl(u8Uart, E_AHI_UART_FIFO_LEVEL_14, true, true, true);
//	/* Set settings */
//	vAHI_UartSetControl( u8Uart, E_AHI_UART_EVEN_PARITY,
//			E_AHI_UART_PARITY_DISABLE, E_AHI_UART_WORD_LEN_8, E_AHI_UART_1_STOP_BIT,
//			E_AHI_UART_RTS_HIGH);
//	/* Set baud rate */
//	vAHI_UartSetClockDivisor(u8Uart, br);
//
//	/* install interrupt service callback */
//#if u8Uart == E_AHI_UART_0
//	vAHI_Uart0RegisterCallback((void*) vUartISR);
//#else
//	vAHI_Uart1RegisterCallback((void*)vUartISR);
//#endif
//	/* Enable TX Fifo empty and Rx data interrupts */
//	vAHI_UartSetInterrupt(u8Uart, FALSE, /* modem status         */
//	false, /* rx line error status */
//	false, /* tx fifo empty        */
//	false, /* rx data there        */
//	E_AHI_UART_FIFO_LEVEL_14);
//
//	vAHI_UartReset(u8Uart, false, false);
//	/* Note we opened the UART */
//	bOpen = true;
	/* Disable use of CTS/RTS */
	vAHI_UartSetRTSCTS(u8Uart, FALSE);
	/* Enable UART */
	vAHI_UartEnable(u8Uart);
	/* Set settings */
	vAHI_UartSetControl(u8Uart, E_AHI_UART_EVEN_PARITY, E_AHI_UART_PARITY_DISABLE, E_AHI_UART_WORD_LEN_8, E_AHI_UART_1_STOP_BIT, E_AHI_UART_RTS_HIGH);
	/* Set baud rate */
	vAHI_UartSetClockDivisor(u8Uart, br);
	/* Note we opened the UART */
	bOpen = TRUE;
}

void
UART_vPTSChar(char cChar)
{
	/* UART is open ? */
	if (bOpen) {
		vAHI_UartWriteData(u8Uart, cChar);
		while ((u8AHI_UartReadLineStatus(u8Uart) & E_AHI_UART_LS_THRE) == 0)
			;
		while ((u8AHI_UartReadLineStatus(u8Uart) & E_AHI_UART_LS_TEMT) == 0)
			;
	}
}

void
UART_vClose(void)
{
	/* UART is open ? */
	if (bOpen) {
		/* Wait for buffers to empty */
		while ((u8AHI_UartReadLineStatus(u8Uart) & E_AHI_UART_LS_THRE) == 0)
			;
		while ((u8AHI_UartReadLineStatus(u8Uart) & E_AHI_UART_LS_TEMT) == 0)
			;
		/* Disable UART */
		vAHI_UartDisable(u8Uart);
	}
}

//#include <stdio.h>
//#include <string.h>
//
//int puts(const char *str)
//{
//  dbg_send_bytes((unsigned char*)str, strlen(str));
//  dbg_putchar('\n');
//  return 0;
//}

