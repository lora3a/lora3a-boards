APPLICATION = sensor-node
BOARD ?= lora3a-h10
BASE ?= $(CURDIR)/../../..
LORA3ABASE ?= $(BASE)/lora3a-boards
RIOTBASE ?= $(BASE)/RIOT
EXTERNAL_BOARD_DIRS=$(LORA3ABASE)/boards
EXTERNAL_MODULE_DIRS=$(LORA3ABASE)/modules
EXTERNAL_PKG_DIRS=$(LORA3ABASE)/pkg
DEVELHELP ?= 1
QUIET ?= 1
PORT ?= /dev/ttyUSB0

USEMODULE += hdc3020
USEMODULE += printf_float
USEMODULE += saml21_backup_mode

include $(RIOTBASE)/Makefile.include
include $(LORA3ABASE)/Makefile.include
