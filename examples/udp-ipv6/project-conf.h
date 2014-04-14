#ifndef PROJECT_H_
#define PROJECT_H_

#undef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     nullmac_driver

#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     nullrdc_driver

#endif /* PROJECT_H_ */
