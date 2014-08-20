CONTIKI_PROJECT = app-rpl-collect-only app-rpl-any-to-any app-no-rpl-unicast app-no-rpl-broadcast
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
TARGET ?= sky

CONFIG_NULLRDC=1
CONFIG_CONTIKIMAC=2
CONFIG_TSCH=3

CONFIG=$(CONFIG_TSCH)

WITH_UIP6=1
UIP_CONF_IPV6=1
CFLAGS+= -DUIP_CONF_IPV6_RPL
CFLAGS+= -DCONFIG_NULLRDC=$(CONFIG_NULLRDC)
CFLAGS+= -DCONFIG_CONTIKIMAC=$(CONFIG_CONTIKIMAC)
CFLAGS+= -DCONFIG_TSCH=$(CONFIG_TSCH)
CFLAGS+= -DCONFIG=$(CONFIG)
PROJECTDIRS += tools
PROJECT_SOURCEFILES += deployment.c simple-energest.c rpl-log.c

all: $(CONTIKI_PROJECT)

CONTIKI=../..
include $(CONTIKI)/Makefile.include

%.exe: %.sky
	cp $< $@
