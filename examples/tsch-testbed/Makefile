CONTIKI_PROJECT = app-rpl-collect-only app-rpl-any-to-any app-no-rpl-unicast app-no-rpl-broadcast
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
TARGET ?= sky

WITH_UIP6=1
UIP_CONF_IPV6=1
CFLAGS+= -DUIP_CONF_IPV6_RPL
PROJECTDIRS += tools
PROJECT_SOURCEFILES += cc2420.c deployment.c simple-energest.c rpl-log.c

all: $(CONTIKI_PROJECT)

CONTIKI=../..
include $(CONTIKI)/Makefile.include

%.exe: %.sky
	cp $< $@
