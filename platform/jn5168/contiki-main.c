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
#include <AppApi.h>
#include "dev/uart0.h"

#include "contiki.h"
#include "net/netstack.h"

#include "dev/serial-line.h"

#include "net/uip.h"
#include "dev/leds.h"

#include "dev/button-sensor.h"
#include "dev/micromac-radio.h"

//#include "dev/pir-sensor.h"
//#include "dev/vib-sensor.h"

#if WITH_UIP6
#include "net/uip-ds6.h"
#endif /* WITH_UIP6 */

#include "net/rime.h"
#include "MMAC.h"

#ifdef SELECT_CONF_MAX
#define SELECT_MAX SELECT_CONF_MAX
#else
#define SELECT_MAX 8
#endif

/*&pir_sensor, &vib_sensor*/
SENSORS(&button_sensor);

/*---------------------------------------------------------------------------*/
#define VERBOSE 1
#if VERBOSE
int dbg_printf(const char *fmt, ...);
#define PRINTF(...) do {dbg_printf(__VA_ARGS__);} while(0)
#else
#define PRINTF(...) do {} while (0)
#endif
/*---------------------------------------------------------------------------*/

//static void
//init_net(void)
//{
//  uip_ipaddr_t ipaddr;
//  uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0, 0, 0, 0);
//
//  /* load mac address */
//  memcpy(uip_lladdr.addr, micromac_get_hw_mac_address_location(), sizeof(uip_lladdr.addr));
//
//#if UIP_CONF_ROUTER
//  uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0, 0, 0, 0);
//#else /* UIP_CONF_ROUTER */
//  uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0);
//#endif /* UIP_CONF_ROUTER */
//  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
//  uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
//
//  PRINTF("Tentative link-local IPv6 address ");
//  {
//    int i, a;
//    for(a = 0; a < UIP_DS6_ADDR_NB; a++) {
//      if (uip_ds6_if.addr_list[a].isused) {
//        for(i = 0; i < 7; ++i) {
//          PRINTF("%02x%02x:",
//                 uip_ds6_if.addr_list[a].ipaddr.u8[i * 2],
//                 uip_ds6_if.addr_list[a].ipaddr.u8[i * 2 + 1]);
//        }
//        PRINTF("%02x%02x\n",
//               uip_ds6_if.addr_list[a].ipaddr.u8[14],
//               uip_ds6_if.addr_list[a].ipaddr.u8[15]);
//      }
//    }
//  }
//
//  netstack_init();
//}

static void
set_rime_addr(void)
{
  rimeaddr_t addr;
  int i;

  memset(&addr, 0, sizeof(addr));
//#if UIP_CONF_IPV6
//  memcpy(addr.u8, micromac_get_hw_mac_address_location(), sizeof(addr.u8));
//#else
//  memcpy(addr.u8, micromac_get_hw_mac_address_location(), sizeof(addr.u8));
//  //memcpy(addr.u8, micromac_get_hw_mac_address_location()+6, sizeof(addr.u8));
//#endif
  tsExtAddr psExtAddress;
  vMMAC_GetMacAddress(&psExtAddress);
  copy_to_rimeaddress(&addr, (tuAddr *)&psExtAddress);
  rimeaddr_set_node_addr(&addr);
  PRINTF("Rime started with address ");
  for(i = 0; i < sizeof(addr.u8) - 1; i++) {
    PRINTF("%02x.", addr.u8[i]);
  }
  PRINTF("%02x\n", addr.u8[i]);
  PRINTF("HW MAC tsExtAddr: %08x.%08x\n", psExtAddress.u32H, psExtAddress.u32L);
}


/*---------------------------------------------------------------------------*/

int
main(void)
{
  clock_init();
  leds_init();
  leds_on(LEDS_ALL);
  process_init();
  ctimer_init();

  queuebuf_init();
  serial_line_init();
  uart0_init(E_AHI_UART_RATE_115200); /* Must come before first PRINTF */
#if USE_SLIP_UART1
  uart1_init(E_AHI_UART_RATE_115200);
#endif /* USE_SLIP_UART1 */

  //check for reset source
  if (bAHI_WatchdogResetEvent()) {
		PRINTF("Init: Watchdog timer has reset device!\r\n");
	}
	vAHI_WatchdogStop();

  /* Wait for clock to stablise */
  while(bAHI_Clock32MHzStable() == FALSE);

  process_start(&etimer_process, NULL);
  set_rime_addr();
  netstack_init();

#if UIP_CONF_IPV6
#if UIP_CONF_IPV6_RPL
  PRINTF(CONTIKI_VERSION_STRING " started with IPV6, RPL\n");
#else
  PRINTF(CONTIKI_VERSION_STRING " started with IPV6\n");
#endif
#else
  PRINTF(CONTIKI_VERSION_STRING " started\n");
#endif
  NETSTACK_MAC.init();
  NETSTACK_RDC.init();
  NETSTACK_NETWORK.init();
  PRINTF("MAC %s RDC %s NETWORK %s\n", NETSTACK_MAC.name, NETSTACK_RDC.name, NETSTACK_NETWORK.name);

#if WITH_UIP6

  tsExtAddr psExtAddress;
  vMMAC_GetMacAddress(&psExtAddress);
  copy_to_rimeaddress((rimeaddr_t *)&uip_lladdr, (tuAddr *)&psExtAddress);
//  memcpy(&uip_lladdr.addr, micromac_get_hw_mac_address_location(), sizeof(uip_lladdr.addr));

  process_start(&tcpip_process, NULL);
  PRINTF("Tentative link-local IPv6 address ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      PRINTF("%02x%02x:", lladdr->ipaddr.u8[i * 2],
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    /* make it hardcoded... */
    lladdr->state = ADDR_AUTOCONF;

    PRINTF("%02x%02x\n", lladdr->ipaddr.u8[14], lladdr->ipaddr.u8[15]);
  }
#else
  process_start(&tcpip_process, NULL);
#endif

  watchdog_start();
  autostart_start(autostart_processes);
  leds_off(LEDS_ALL);

  while(1) {

    //etimer_request_poll();
    int r;
    do {
      /* Reset watchdog. */
    	watchdog_periodic();
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
  PRINTF("log_message: %s%s\n", m1, m2);
}
/*---------------------------------------------------------------------------*/
void
uip_log(char *m)
{
  PRINTF("uip_log: %s\n", m);
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
/*---------------------------------------------------------------------------*/
#if UIP_ARCH_ADD32
void
uip_add32(uint8_t *op32, uint16_t op16)
{
  uip_acc32[3] = op32[3] + (op16 & 0xff);
  uip_acc32[2] = op32[2] + (op16 >> 8);
  uip_acc32[1] = op32[1];
  uip_acc32[0] = op32[0];

  if(uip_acc32[2] < (op16 >> 8)) {
    ++uip_acc32[1];
    if(uip_acc32[1] == 0) {
      ++uip_acc32[0];
    }
  }


  if(uip_acc32[3] < (op16 & 0xff)) {
    ++uip_acc32[2];
    if(uip_acc32[2] == 0) {
      ++uip_acc32[1];
      if(uip_acc32[1] == 0) {
	++uip_acc32[0];
      }
    }
  }
}

#endif /* UIP_ARCH_ADD32 */

#if UIP_ARCH_CHKSUM
/*---------------------------------------------------------------------------*/
static uint16_t
chksum(uint16_t sum, const uint8_t *data, uint16_t len)
{
  uint16_t t;
  const uint8_t *dataptr;
  const uint8_t *last_byte;

  dataptr = data;
  last_byte = data + len - 1;

  while(dataptr < last_byte) {	/* At least two more bytes */
    t = (dataptr[0] << 8) + dataptr[1];
    sum += t;
    if(sum < t) {
      sum++;		/* carry */
    }
    dataptr += 2;
  }

  if(dataptr == last_byte) {
    t = (dataptr[0] << 8) + 0;
    sum += t;
    if(sum < t) {
      sum++;		/* carry */
    }
  }

  /* Return sum in host byte order. */
  return sum;
}
/*---------------------------------------------------------------------------*/
uint16_t
uip_chksum(uint16_t *data, uint16_t len)
{
  return uip_htons(chksum(0, (uint8_t *)data, len));
}
#endif  /*UIP_ARCH_CHKSUM*/
/*---------------------------------------------------------------------------*/
#ifdef UIP_ARCH_IPCHKSUM
uint16_t
uip_ipchksum(void)
{
  uint16_t sum;

  sum = chksum(0, &uip_buf[UIP_LLH_LEN], UIP_IPH_LEN);
  DEBUG_PRINTF("uip_ipchksum: sum 0x%04x\n", sum);
  return (sum == 0) ? 0xffff : uip_htons(sum);
}
#endif
