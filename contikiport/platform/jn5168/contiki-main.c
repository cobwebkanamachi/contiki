/*
 * Copyright (c) 2002, Adam Dunkels.
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the Contiki OS
 *
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "dev/watchdog.h"
#include <AppHardwareApi.h>
#include "dev/uart0.h"

#include "contiki.h"
#include "net/netstack.h"

#include "dev/serial-line.h"

#include "net/uip.h"
#include "dev/leds.h"

//#include "dev/button-sensor.h"
//#include "dev/pir-sensor.h"
//#include "dev/vib-sensor.h"

#if WITH_UIP6
#include "net/uip-ds6.h"
#endif /* WITH_UIP6 */

#include "net/rime.h"

#ifdef SELECT_CONF_MAX
#define SELECT_MAX SELECT_CONF_MAX
#else
#define SELECT_MAX 8
#endif


//SENSORS(&pir_sensor, &vib_sensor, &button_sensor);

static uint8_t serial_id[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
static uint16_t node_id = 0x0102;
/*---------------------------------------------------------------------------*/
static void
set_rime_addr(void)
{
  rimeaddr_t addr;
  int i;

  memset(&addr, 0, sizeof(rimeaddr_t));
#if UIP_CONF_IPV6
  memcpy(addr.u8, serial_id, sizeof(addr.u8));
#else
  if(node_id == 0) {
    for(i = 0; i < sizeof(rimeaddr_t); ++i) {
      addr.u8[i] = serial_id[7 - i];
    }
  } else {
    addr.u8[0] = node_id & 0xff;
    addr.u8[1] = node_id >> 8;
  }
#endif
  rimeaddr_set_node_addr(&addr);
  printf("Rime started with address ");
  for(i = 0; i < sizeof(addr.u8) - 1; i++) {
    printf("%d.", addr.u8[i]);
  }
  printf("%d\n", addr.u8[i]);
}


/*---------------------------------------------------------------------------*/

int
main(void)
{
  clock_init();
  leds_arch_init();
  leds_arch_set(LEDS_ALL);
  process_init();
  ctimer_init();
  /* Wait for clock to stablise */
  while(bAHI_Clock32MHzStable() == FALSE);
  uart0_init(E_AHI_UART_RATE_115200); /* Must come before first printf */

  queuebuf_init();
  serial_line_init();

  process_start(&etimer_process, NULL);
  set_rime_addr();

  netstack_init();

  #if UIP_CONF_IPV6
#if UIP_CONF_IPV6_RPL
  printf(CONTIKI_VERSION_STRING " started with IPV6, RPL\n");
#else
  printf(CONTIKI_VERSION_STRING " started with IPV6\n");
#endif
#else
  printf(CONTIKI_VERSION_STRING " started\n");
#endif
  printf("MAC %s RDC %s NETWORK %s\n", NETSTACK_MAC.name, NETSTACK_RDC.name, NETSTACK_NETWORK.name);

#if WITH_UIP6
  memcpy(&uip_lladdr.addr, serial_id, sizeof(uip_lladdr.addr));

  process_start(&tcpip_process, NULL);
#ifdef __CYGWIN__
  process_start(&wpcap_process, NULL);
#endif
  printf("Tentative link-local IPv6 address ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:", lladdr->ipaddr.u8[i * 2],
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    /* make it hardcoded... */
    lladdr->state = ADDR_AUTOCONF;

    printf("%02x%02x\n", lladdr->ipaddr.u8[14], lladdr->ipaddr.u8[15]);
  }
#else
  process_start(&tcpip_process, NULL);
#endif

  watchdog_start();
  autostart_start(autostart_processes);

  while(1) {

    //etimer_request_poll();
    int r;
    do {
      /* Reset watchdog. */
      watchdog_periodic(); //   vAHI_WatchdogRestart();
      r = process_run();
    } while(r > 0);

    /*
     * Idle processing.
     */
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
void
log_message(char *m1, char *m2)
{
//  fprintf(stderr, "%s%s\n", m1, m2);
}
/*---------------------------------------------------------------------------*/
void
uip_log(char *m)
{
//  fprintf(stderr, "%s\n", m);
}
/*---------------------------------------------------------------------------*/
void
AppColdStart(void)
{
	main();

}

void
AppWarmStart(void)
{
	main();
}
