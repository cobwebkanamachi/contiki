
#include <jendefs.h>
#include <AppHardwareApi.h>
#include <PeripheralRegs.h>
#include "contiki-conf.h"
#include "dev/uart1.h"
#include "uart_buffered.h"

#define u8Uart E_AHI_UART_1
#define TXBUFSIZE 256UL
static unsigned char txbuf_data[TXBUFSIZE];
volatile static int (*uart1_input)(unsigned char c);

void
uart1_set_input(int
(*input)(unsigned char c))
{
	uart1_input = input;
}

void
uart1_writeb(unsigned char c)
{
	vUartWrite(u8Uart, c);
}

void
uart1_init(uint8_t br)
{
	vUartInit(u8Uart, br, txbuf_data, TXBUFSIZE, uart1_input);
}

