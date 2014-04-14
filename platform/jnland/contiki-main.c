#include "contiki-net.h"
#include "bootloader.h"
#include "gdb2.h"
#include "jts.h"
#include "net/ieee802.h"
#include "init.h"
#include <string.h>
#include <AppApi.h>

#include "dev/ds17.h"
#include "dev/battery-sensor.h"
#include "dev/button-sensor.h"
#include "dev/inttemp-sensor.h"
#include "dev/acc-sensor.h"
#include "dev/rgb_leds.h"
#include "dev/buzzer.h"
#include "dev/sseg.h"
#include "dev/leds.h"

PROCINIT(&etimer_process, &sensors_process, &tcpip_process, &jennic_bootloader_process);
SENSORS(&battery_sensor, &button_sensor, &inttemp_sensor, &ds17_sensor, &acc_sensor);

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

void AppColdStart(void)
{
  /* default startup */
  init_hardware();

  buzzer_init();
  rgb_leds_init();
  sseg_init();

  process_init();
  init_net();

  /* start the main processes */
  procinit_init();
  autostart_start(autostart_processes);
  jts_init();

  /* enable watchdog on JN5148, there is none on JN5139 */
  watchdog_start();

  /* default main loop */
  while(1)
  {
    process_run();
    etimer_request_poll();
    watchdog_periodic();
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
