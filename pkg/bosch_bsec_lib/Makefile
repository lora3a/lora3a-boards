PKG_NAME=bosch_bsec_lib
PKG_URL=https://github.com/boschsensortec/Bosch-BSEC2-Library/
PKG_VERSION=4a08fd082c17fc9d13313162d0a32eca0f5e6f35
PKG_LICENSE=BSD-3-Clause

include $(RIOTBASE)/pkg/pkg.mk

BOSCH_BSEC_CPU_DIR ?= cortex-m0plus

.PHONY: all

all:
	$(QQ)"$(MAKE)" -C $(PKG_SOURCE_DIR) -f $(RIOTBASE)/Makefile.base
	cp $(PKGDIRBASE)/$(PKG_NAME)/src/$(BOSCH_BSEC_CPU_DIR)/libalgobsec.a $(BINDIR)/
