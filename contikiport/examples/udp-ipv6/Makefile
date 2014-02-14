APP_BASE ?= $(abspath .)
UIP_CONF_IPV6=1
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
SMALL = 1

CONTIKI_PROJECT = udp-server udp-client

all: udp-server udp-client test-button
CONTIKI = ../..
include $(CONTIKI)/Makefile.include
