TARGET ?= sky
CONTIKI_DIRS += dev
CONTIKI_SOURCEFILES += cc2420.c tsch.c
CONTIKI_PROJECT = hello-world
#WITH_UIP6=1
#UIP_CONF_IPV6=1
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
SMALL = 1
#CFLAGS+= -DUIP_CONF_IPV6_RPL -DWITH_UIP6 -DUIP_CONF_IPV6

all: $(CONTIKI_PROJECT)

#UIP_CONF_IPV6=1

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

sim: $(CONTIKI_PROJECT)
	java -jar $(CONTIKI)/tools/cooja/dist/cooja.jar -quickstart=$(CONTIKI_PROJECT).csc
	