APPLICATION = test-lis2dw12
BOARD ?= berta-h10

BASE ?= $(CURDIR)/../../..
LORA3ABASE ?= $(BASE)/lora3a-boards
RIOTBASE ?= $(BASE)/RIOT

EXTERNAL_BOARD_DIRS=$(LORA3ABASE)/boards
EXTERNAL_MODULE_DIRS+=$(LORA3ABASE)/modules
EXTERNAL_PKG_DIRS=$(LORA3ABASE)/pkg

DEVELHELP ?= 1
QUIET ?= 1
PORT ?= /dev/ttyUSB0

USEMODULE += printf_float
USEMODULE += saml21_backup_mode

# LIS2DW12
USEMODULE += lis2dw12

ACME_PORT ?= 0

ifeq ($(ACME_PORT), 0)
CFLAGS += -DACME0_BUS_MODE=MODE_I2C
CFLAGS += -DLIS2DW12_I2C_DEV=ACME0_I2C_DEV
CFLAGS += -DLIS2DW12_POWER_PIN=ACME0_POWER_PIN
endif

ifeq ($(ACME_PORT), 1)
CFLAGS += -DACME1_BUS_MODE=MODE_I2C
CFLAGS += -DLIS2DW12_I2C_DEV=ACME1_I2C_DEV
CFLAGS += -DLIS2DW12_POWER_PIN=ACME1_POWER_PIN
endif

ifeq ($(ACME_PORT), 2)
CFLAGS += -DACME2_BUS_MODE=MODE_I2C
CFLAGS += -DLIS2DW12_I2C_DEV=ACME2_I2C_DEV
CFLAGS += -DLIS2DW12_POWER_PIN=ACME2_POWER_PIN
endif

include $(RIOTBASE)/Makefile.include
include $(LORA3ABASE)/Makefile.include