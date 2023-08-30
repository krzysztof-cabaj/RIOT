# name of your application
APPLICATION = shell

# If no BOARD is found in the environment, use this default:
BOARD ?= native

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= /data/RIOT/RIOT

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

USEMODULE += shell
USEMODULE += xtimer

USEMODULE += shell_cmds_default

#for LWIP IPv4 net
USEMODULE += netdev_default
USEMODULE += lwip_netdev
USEMODULE += ipv4_addr
USEMODULE += lwip_arp
USEMODULE += lwip_ipv4
#USEMODULE += lwip_udp
#USEMODULE += lwip_sock
USEMODULE += lwip_sock_udp
#USEMODULE += sock_udp
#USEMODULE += lwip_ipv6

USEMODULE += sock_aux_local

USEMODULE += l2util
USEMODULE += inet_csum
USEMODULE += netdev_eth
USEMODULE += netdev_test

#other modules from tests/pkg/lwip_udp

CFLAGS += -DETHARP_SUPPORT_STATIC_ENTRIES=1
CFLAGS += -DSO_REUSE



CFLAGS +=-Wno-error=cast-align
CFLAGS +=-DTHREAD_STACKSIZE_DEFAULT=16384
#CFLAGS +=-DCHECKSUM_GEN_ICMP=0
#CFLAGS +=-DLWIP_CHECKSUM_CTRL_PER_NETIF=1

include $(RIOTBASE)/Makefile.include
