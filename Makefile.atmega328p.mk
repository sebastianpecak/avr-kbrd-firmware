TARGET_MCU     = atmega328p
MCU_FLASH_SIZE = 32768
MCU_SRAM_SIZE  = 2048
MCU_STACK_SIZE = 1024
# Required oscylator is 12MHz.
MCU_FREQ       = 12000000

include ./Makefile.main.mk
