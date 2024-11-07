PKG_NAME=soniclib
PKG_URL=https://github.com/tdk-invn-oss/ultrasonic.soniclib
# release v4.4.1
PKG_VERSION=a6514c1fe5b01ebe18f75fd2a627f08864bf757b
PKG_LICENSE=BSD-3-Clause

include $(RIOTBASE)/pkg/pkg.mk

.PHONY: ..cmake_version_supported

CMAKE_MINIMAL_VERSION = 3.13

SONICLIB_FIRMWARE ?= NONE
SONICLIB_DEBUG_LEVEL ?= DISABLE

TOOLCHAIN_FILE = $(PKG_SOURCE_DIR)/xcompile-toolchain.cmake

all: $(BINDIR)/libsoniclib.a

$(BINDIR)/libsoniclib.a: $(PKG_BUILD_DIR)/libsoniclib.a
	cp $< $@

$(PKG_BUILD_DIR)/libsoniclib.a: $(PKG_BUILD_DIR)/Makefile
	"$(MAKE)" -C $(PKG_BUILD_DIR)

$(PKG_BUILD_DIR)/Makefile: $(PKG_PREPARED) $(TOOLCHAIN_FILE) | ..cmake_version_supported
	cmake -B$(PKG_BUILD_DIR) -H$(PKG_SOURCE_DIR) \
	 -DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN_FILE) \
	 -DCMAKE_BUILD_TYPE=MinSizeRel \
	 -DCHIRP_MAIN_FW_TYPE:STRING=$(SONICLIB_FIRMWARE) \
	 -DCHIRP_INIT_FW_TYPE:STRING=FULL \
	 -DCHIRP_LOG_LEVEL:STRING=CH_LOG_LEVEL_$(SONICLIB_DEBUG_LEVEL)

$(TOOLCHAIN_FILE): FORCE
	$(RIOTTOOLS)/cmake/generate-xcompile-toolchain.sh > $(TOOLCHAIN_FILE)

..cmake_version_supported:
	@ # Remove '-rcX' from version as they are not well handled
	$(Q)\
	CMAKE_VERSION=$$(cmake --version | sed -n '1 {s/cmake version //;s/-rc.*//;p;}'); \
	$(RIOTTOOLS)/has_minimal_version/has_minimal_version.sh "$${CMAKE_VERSION}" "$(CMAKE_MINIMAL_VERSION)" cmake

clean::
	@rm -rf $(PKG_BUILD_DIR)

