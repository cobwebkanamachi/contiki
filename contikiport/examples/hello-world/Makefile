TARGET ?= jn5168
APP_BASE ?= $(abspath .)

#UIP_CONF_IPV6=1

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
SMALL = 1

CONTIKI_PROJECT = hello-world

all: hello-world

CONTIKI = ../..

include $(CONTIKI)/Makefile.include
