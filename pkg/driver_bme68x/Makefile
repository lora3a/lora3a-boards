PKG_NAME=driver_bme68x
PKG_URL=https://github.com/BoschSensortec/BME68x-Sensor-API
PKG_VERSION=80ea120a8b8ac987d7d79eb68a9ed796736be845
PKG_LICENSE=BSD-3-Clause

include $(RIOTBASE)/pkg/pkg.mk

ifeq (,$(filter bme68x_fp,$(USEMODULE)))
    CFLAGS += -DBME68X_DO_NOT_USE_FPU
endif

.PHONY: all

all:
	$(QQ)"$(MAKE)" -C $(PKG_SOURCE_DIR) -f $(RIOTBASE)/Makefile.base
