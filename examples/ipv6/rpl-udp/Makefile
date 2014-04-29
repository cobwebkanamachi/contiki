all: udp-client udp-server 
#test-button
APPS=servreg-hack
CONTIKI=../../..

WITH_UIP6=1
UIP_CONF_IPV6=1
CFLAGS += -lm -DPROJECT_CONF_H=\"project-conf.h\"
SMALL = 1
CFLAGS+= -DUIP_CONF_IPV6_RPL -DWITH_UIP6 -DUIP_CONF_IPV6 
CONTIKI_PROJECT = udp-server udp-client

ifdef WITH_COMPOWER
APPS+=powertrace
CFLAGS+= -DCONTIKIMAC_CONF_COMPOWER=1 -DWITH_COMPOWER=1 -DQUEUEBUF_CONF_NUM=4
endif

ifdef SERVER_REPLY
CFLAGS+=-DSERVER_REPLY=$(SERVER_REPLY)
endif
ifdef PERIOD
CFLAGS+=-DPERIOD=$(PERIOD)
endif

include $(CONTIKI)/Makefile.include
