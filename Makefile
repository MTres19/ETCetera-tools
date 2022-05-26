# Copied with only minor changes from apps/system/nsh

include $(APPDIR)/Make.defs

MAINSRC = cantest_main.c dynohelper_main.c throttle_logdump_main.c drstest_main.c wsstest_main.c relaytest_main.c

PROGNAME = cantest dynohelper throttle_logdump drstest wsstest relaytest
PRIORITY = $(CONFIG_INDUSTRY_ETCETERA_TOOLS_PRIORITY)
STACKSIZE = $(CONFIG_INDUSTRY_ETCETERA_TOOLS_STACKSIZE)
MODULE = $(CONFIG_INDUSTRY_ETCETERA_TOOLS)

include $(APPDIR)/Application.mk
