#ifndef PROJECT_H_
#define PROJECT_H_

#undef CC2420_CONF_CHANNEL
#define CC2420_CONF_CHANNEL 20

#undef IEEE802154_CONF_PANID
#define IEEE802154_CONF_PANID       0x0022

#undef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     nullmac_driver

#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     nullrdc_driver

#endif /* PROJECT_H_ */
