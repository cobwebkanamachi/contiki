#include "bootloader.h"
#include "gdb2.h"

#include "contiki.h"
#include "contiki-net.h"

#include "dev/temperature-sensor.h"
#include "dev/lightlevel-sensor.h"
#include "dev/infrared-sensor.h"
#include "dev/inttemp-sensor.h"
#include "dev/battery-sensor.h"
#include "dev/button-sensor.h"
#include "dev/acc-sensor.h"
#include "dev/leds.h"
#include "dev/i2c.h"
#include "dev/irq.h"
#include "jts.h"

#include "net/ieee802.h"
#include <AppHardwareApi.h>
#include <AppApi.h>
#include <string.h>

PROCESS_NAME(ledtest_process);
PROCINIT(&etimer_process, &sensors_process, &ledtest_process, &tcpip_process, &jennic_bootloader_process);
SENSORS(&lightlevel_sensor, &button_sensor, &acc_sensor, &inttemp_sensor, &infrared_sensor);
void
init_net(void)
{
  uip_ipaddr_t ipaddr;
  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);

  /* load the mac address */
  memcpy(uip_lladdr.addr, ieee_get_mac(), sizeof(uip_lladdr.addr));

#if UIP_CONF_ROUTER
  uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0, 0, 0, 0);
#else /* UIP_CONF_ROUTER */
  uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0);
#endif /* UIP_CONF_ROUTER */
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  netstack_init();
}

/* Jennic equivalent of main() */
void AppColdStart(void)
{
  /* default startup */
  init_hardware();
  process_init();
  init_net();

  procinit_init();
  autostart_start(autostart_processes);
  jts_init();

  /* default main loop */
  while(1)
  {
    process_run();
    etimer_request_poll();
  }
}

void AppWarmStart(void)
{
  AppColdStart();
}

void uip_log(char *msg)
{
  printf(msg);
}

u8_t leds[] = { LEDS_RED, LEDS_GREEN, LEDS_BLUE, LEDS_INNER, LEDS_OUTER };
PROCESS(ledtest_process, "led test process");
PROCESS_THREAD(ledtest_process, ev, data)
{
  static struct etimer timer;
  static u8_t i;
  PROCESS_BEGIN();

  for (i=0; i<sizeof(leds); i++) {
    etimer_set(&timer, 500);
    PROCESS_YIELD_UNTIL(ev==PROCESS_EVENT_TIMER);
    leds_on(leds[i]);
  }

  etimer_set(&timer, 500);
  PROCESS_YIELD_UNTIL(ev==PROCESS_EVENT_TIMER);
  leds_off(LEDS_ALL);
  PROCESS_END();
}
