#ifndef PROJECT_H_
#define PROJECT_H_

#define ENABLE_COOJA_DEBUG 0

#define RF_CONF_CHANNEL 15
#define MICROMAC_RADIO_CONF_NO_IRQ 1
#define MICROMAC_RADIO_CONF_AUTOACK 0
#define MICROMAC_RADIO_CONF_ALWAYS_ON 0
#if CONTIKI_TARGET_JN5168



#else /* Leave CC2420 as default */

#undef CC2420_CONF_TX_POWER
#define CC2420_CONF_TX_POWER 15

#undef CC2420_CONF_CHANNEL
#define CC2420_CONF_CHANNEL RF_CONF_CHANNEL

#undef CC2420_CONF_SFD_TIMESTAMPS
#define CC2420_CONF_SFD_TIMESTAMPS 1

#endif /* CONTIKI_TARGET */

//#undef WITH_UIP6
//#define WITH_UIP6 0

#undef UIP_CONF_LOGGING
#define UIP_CONF_LOGGING 0

#undef DCOSYNCH_CONF_ENABLED
#define DCOSYNCH_CONF_ENABLED 0

#undef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     nullmac_driver

#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     tschrdc_driver

//#if TMOTE_SKY
//#undef NETSTACK_CONF_RADIO
//#define NETSTACK_CONF_RADIO     tschrdc_driver
//#endif

#undef UIP_CONF_ND6_SEND_NA
#define UIP_CONF_ND6_SEND_NA 0

#undef UIP_CONF_ND6_SEND_RA
#define UIP_CONF_ND6_SEND_RA 0

#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM 4 /* should be a power of two */

#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS	20

#undef TSCH_CONF_802154_DUPLICATE_DETECTION
#define TSCH_CONF_802154_DUPLICATE_DETECTION 0

#undef RPL_CONF_DIO_INTERVAL_DOUBLINGS
#define RPL_CONF_DIO_INTERVAL_DOUBLINGS 8

#undef IEEE802154_CONF_PANID
#define IEEE802154_CONF_PANID     0xcdba

/* RPL and neighborhood information */

#undef RPL_CONF_INIT_LINK_METRIC
#define RPL_CONF_INIT_LINK_METRIC 2

#undef RPL_CONF_MIN_HOPRANKINC
#define RPL_CONF_MIN_HOPRANKINC 128

#define UIP_CONF_DS6_ADDR_NBU 1

#define RPL_CONF_MAX_INSTANCES    1 /* default 1 */
#define RPL_CONF_MAX_DAG_PER_INSTANCE 1 /* default 2 */

#undef RPL_CONF_DIO_INTERVAL_MIN
#define RPL_CONF_DIO_INTERVAL_MIN 10

/* Other system parameters */
#undef UIP_CONF_UDP_CONNS
#define UIP_CONF_UDP_CONNS       1

#undef UIP_CONF_FWCACHE_SIZE
#define UIP_CONF_FWCACHE_SIZE    4

#undef UIP_CONF_TCP
#define UIP_CONF_TCP          0

#undef UIP_CONF_UDP_CHECKSUMS
#define UIP_CONF_UDP_CHECKSUMS   0

#undef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG 0

#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM 4

#define ALL_NODES_ADDRESSABLE 1

/* Reject parents that have a higher link metric than the following. */
#define MAX_LINK_METRIC     10

#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES  2


#endif /* PROJECT_H_ */

