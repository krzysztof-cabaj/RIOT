PKG_NAME=pioasm
PKG_URL=https://github.com/raspberrypi/pico-sdk.git
PKG_VERSION=2062372d203b372849d573f252cf7c6dc2800c0a
PKG_LICENSE=BSD-3-Clause
PKG_PATCH_DIR=$(RIOTTOOLS)/pioasm/patches

include $(RIOTBASE)/pkg/pkg.mk

all: $(CURDIR)/pioasm

$(CURDIR)/pioasm:
	@env -i PATH="$(PATH)" cmake $(PKG_SOURCE_DIR)/tools/pioasm/CMakeLists.txt
	"$(MAKE)" -C "$(PKG_SOURCE_DIR)/tools/pioasm"
	cp $(PKG_SOURCE_DIR)/tools/pioasm/pioasm .
