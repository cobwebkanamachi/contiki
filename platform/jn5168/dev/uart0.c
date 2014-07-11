
#include <jendefs.h>
#include <AppHardwareApi.h>
#include <PeripheralRegs.h>
#include "contiki-conf.h"
#include "dev/uart0.h"
#include "uart_buffered.h"

#define u8Uart E_AHI_UART_0
#define TXBUFSIZE 256UL
static unsigned char txbuf_data[TXBUFSIZE];
volatile static int (*uart0_input)(unsigned char c);

void
uart0_set_input(int
(*input)(unsigned char c))
{
	uart0_input = input;
}

void
uart0_writeb(unsigned char c)
{
	vUartWrite(u8Uart, c);
}

void
uart0_init(uint8_t br)
{
	vUartInit(u8Uart, br, txbuf_data, TXBUFSIZE, uart0_input);
}

