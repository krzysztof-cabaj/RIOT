APPLICATION = pio_blink
BOARD ?= rpi-pico
RIOTBASE ?= $(CURDIR)/../..

FEATURES_REQUIRED += periph_pio
USEMODULE += shell

DEVELHELP ?= 1
QUIET ?= 1

include $(RIOTBASE)/Makefile.include

DIRS += $(CPU)_pio_blink
USEMODULE += $(CPU)_pio_blink
