# ======================================================================
# Makefile for USBtinyISP AVR programmer
#
# Copyright 2006-2010 Dick Streefland
#
# This is free software, licensed under the terms of the GNU General
# Public License as published by the Free Software Foundation.
# ======================================================================

USBTINY        = ./
TARGET_ARCH    = -mmcu=atmega328p
OBJECTS        = main.o
FLASH_CMD    = avrdude -p atmega328p -U flash:w:main.hex
FUSES_CMD    = avrdude -p atmega328p -U hfuse:w:0xdf:m -U lfuse:w:0xef:m
STACK        = 1024
FLASH        = 32000
SRAM         = 2048
SCHEM        = 

include $(USBTINY)/common.mk
