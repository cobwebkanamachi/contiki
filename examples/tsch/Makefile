TARGET ?= sky
#APPS=servreg-hack
CONTIKIDIRS += ./dev 
ifeq ($(TARGET), sky) 
CONTIKI_SOURCEFILES += cc2420-tsch.c tsch.c
else
CONTIKI_SOURCEFILES += tsch.c
endif


CONTIKI_PROJECT = udp-client udp-server
UIP_CONF_IPV6=1
CFLAGS+= -DUIP_CONF_IPV6_RPL
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
SMALL = 1

all: $(CONTIKI_PROJECT)

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

sim: $(CONTIKI_PROJECT)
	java -jar $(CONTIKI)/tools/cooja/dist/cooja.jar -quickstart=$(CONTIKI_PROJECT).csc
	
	