# ======================================================================
# Makefile for USBtinyISP AVR programmer
#
# Copyright 2006-2010 Dick Streefland
#
# This is free software, licensed under the terms of the GNU General
# Public License as published by the Free Software Foundation.
# ======================================================================

USBTINY        = ./
TARGET_ARCH    = -mmcu=atmega88pa
OBJECTS        = main.o
FLASH_CMD    = avrdude -p atmega88pa -U flash:w:main.hex
FUSES_CMD    = avrdude -p atmega88pa -U hfuse:w:0xdf:m -U lfuse:w:0xef:m
STACK        = 512
FLASH        = 8192
SRAM         = 1024
SCHEM        = 

# avrdude: safemode: Fuses OK (E:FD, H:DE, L:FF)

include $(USBTINY)/common.mk
