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

	switch(u32DeviceId){
	case E_AHI_DEVICE_UART0:
		u8UartL = E_AHI_UART_0;
		break;
	default:
		return;
	}

	if (u32ItemBitmap == E_AHI_UART_INT_RXDATA || u32ItemBitmap == E_AHI_UART_INT_TIMEOUT) {
		if (uart0_input) {
			uart0_input(u8AHI_UartReadData(u8UartL));
		} else {
			u8AHI_UartReadData(u8UartL);
		}
	} else if (u32ItemBitmap == E_AHI_UART_INT_TX) {
		if(ringbuf_elements(&txbuf)) {
			transmitting = 1;
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
  if (transmitting==0)
  {
    transmitting=1;
//    UART_vPTSChar(ringbuf_get(&txbuf));
    vAHI_UartWriteData(u8Uart, ringbuf_get(&txbuf));
  }
}

void
uart0_init(uint8_t br)
{
	transmitting = 0;
	ringbuf_init(&txbuf, txbuf_data, TXBUFSIZE);
	/* Disable use of CTS/RTS */
	vAHI_UartSetRTSCTS(u8Uart, FALSE);
	/* Enable UART */
	vAHI_UartEnable(u8Uart);

	vAHI_UartReset(u8Uart, true, true);

	/* Set settings */
	vAHI_UartSetControl(u8Uart, E_AHI_UART_EVEN_PARITY, E_AHI_UART_PARITY_DISABLE, E_AHI_UART_WORD_LEN_8, E_AHI_UART_1_STOP_BIT, E_AHI_UART_RTS_HIGH);

	/* Set interrupts */
	vAHI_Uart0RegisterCallback(vUartISR);

	vAHI_UartSetInterrupt(u8Uart, false,  /* modem status         */
															false,  /* rx line error status */
															true,   /* tx fifo empty        */
															true,   /* rx data there        */
															E_AHI_UART_FIFO_LEVEL_1);

	/* Set baud rate */
	vAHI_UartSetClockDivisor(u8Uart, br);

	vAHI_UartSetRTSCTS(u8Uart, false);
	vAHI_UartReset(u8Uart, false, false);

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
