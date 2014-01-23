#
# Makefile for NaCl-based OpenRisc emulator
#

#
# GNU Make based build file.  For details on GNU Make see:
# http://www.gnu.org/software/make/manual/make.html
#

#
# Get pepper directory for toolchain and includes.
#
# If NACL_SDK_ROOT is not set, then assume it can be found three directories up.
#
THIS_MAKEFILE := $(abspath $(lastword $(MAKEFILE_LIST)))
NACL_SDK_ROOT ?= $(abspath $(dir $(THIS_MAKEFILE))../..)

#
# Project build options
#
LINUX_IMAGE ?= vmlinux.bin.bz2
HDD_IMAGE ?= hdd.bin.bz2

DIST_DIR ?= dist

#
# Project source files
#
SOURCES := \
	or_emulator.cpp \
	cpu.cpp \
	util.cpp \
	dummy_device.cpp \
	uart_device.cpp \
	ata_device.cpp \
	fb_device.cpp \
	kb_device.cpp \
	eth_device.cpp \
	lpc32_device.cpp

SRC_DIR := src
SRC_DIR_C := $(SRC_DIR)/c
SRC_DIR_JS := $(SRC_DIR)/js
SRC_DIR_CSS := $(SRC_DIR)/css
SRC_DIR_OTHER := $(SRC_DIR)
BIN_IMAGE_DIR := bin

#
# Project Build flags
#
#WARNINGS := -Wno-long-long -Wall -Wswitch-enum -pedantic -Werror
WARNINGS := -Wno-long-long -Wall -Wswitch-enum -pedantic
LIBS := -lppapi_cpp -lppapi -lnacl_io -lbz2

#
# Compute tool paths
#
GETOS := python $(NACL_SDK_ROOT)/tools/getos.py
OSHELPERS = python $(NACL_SDK_ROOT)/tools/oshelpers.py
OSNAME := $(shell $(GETOS))
CP := $(OSHELPERS) cp
RM := $(OSHELPERS) rm -fr
MKDIR := $(OSHELPERS) mkdir -p

PNACL_TC_PATH := $(abspath $(NACL_SDK_ROOT)/toolchain/$(OSNAME)_pnacl)
PNACL_C := $(PNACL_TC_PATH)/bin/pnacl-clang++
PNACL_FINALIZE := $(PNACL_TC_PATH)/bin/pnacl-finalize
CFLAGS := -std=gnu++11 -I$(NACL_SDK_ROOT)/include -I$(PNACL_TC_PATH)/usr/include $(WARNINGS) -pthread
LDFLAGS := -L$(NACL_SDK_ROOT)/lib/pnacl/Debug -L$(PNACL_TC_PATH)/usr/lib $(LIBS)

#
# Disable DOS PATH warning when using Cygwin based tools Windows
#
CYGWIN ?= nodosfilewarning
export CYGWIN


# Declare the ALL target first, to make the 'all' target the default build
.PHONY: all
all: $(SRC_DIR_C)/or_emulator.pexe
	$(MKDIR) $(DIST_DIR)
	$(MKDIR) $(DIST_DIR)/bin
	$(MKDIR) $(DIST_DIR)/js
	$(MKDIR) $(DIST_DIR)/css
	$(CP) $(SRC_DIR_C)/or_emulator.pexe $(DIST_DIR)
	$(CP) $(SRC_DIR_JS)/*.js $(DIST_DIR)/js
	$(CP) $(SRC_DIR_CSS)/*.css $(DIST_DIR)/css
	$(CP) $(SRC_DIR_OTHER)/*.html $(DIST_DIR)
	$(CP) $(SRC_DIR_OTHER)/*.nmf $(DIST_DIR)
	$(CP) $(BIN_IMAGE_DIR)/$(LINUX_IMAGE) $(DIST_DIR)/bin/vmlinux.bin.bz2
	$(CP) $(BIN_IMAGE_DIR)/$(HDD_IMAGE) $(DIST_DIR)/bin/hdd.bin.bz2

.PHONY: clean
clean:
	$(RM) $(DIST_DIR)
	$(RM) $(SRC_DIR_C)/*.bc $(SRC_DIR_C)/*.pexe

$(SRC_DIR_C)/or_emulator.bc: $(addprefix $(SRC_DIR_C)/,$(SOURCES))
	$(PNACL_C) -o $@ $^ -O4 $(CFLAGS) $(LDFLAGS)

$(SRC_DIR_C)/or_emulator.pexe: $(SRC_DIR_C)/or_emulator.bc
	$(PNACL_FINALIZE) -o $@ $<


#
# Makefile target to run the SDK's simple HTTP server and serve this example.
#
HTTPD_PY := python $(NACL_SDK_ROOT)/tools/httpd.py

.PHONY: serve
serve: all
	$(HTTPD_PY) -C $(DIST_DIR)
