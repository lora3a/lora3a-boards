APPLICATION = h10-shell
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

USEMODULE += saul_default
USEMODULE += shell_cmds_default

USEMODULE += h10_adc
USEMODULE += saml21_cpu_debug
USEMODULE += shell_extra_commands

include $(RIOTBASE)/Makefile.include
include $(LORA3ABASE)/Makefile.include
