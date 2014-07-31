TARGET ?= sky
#DEVICE_NAME ?= DR1198
#APPS=servreg-hack
 
ifeq ($(TARGET), sky) 
CONTIKI_SOURCEFILES += cc2420-tsch.c
endif
CONTIKI_SOURCEFILES += tsch.c

CONTIKI_PROJECT = udp-client udp-server rt-leds
UIP_CONF_IPV6=1
CFLAGS+= -DUIP_CONF_IPV6_RPL
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
SMALL = 1

all: $(CONTIKI_PROJECT)

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

sim: $(CONTIKI_PROJECT)
	java -jar $(CONTIKI)/tools/cooja/dist/cooja.jar -quickstart=$(CONTIKI_PROJECT).csc
	
	