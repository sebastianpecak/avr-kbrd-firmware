TARGET_MCU     = atmega88pa
MCU_FLASH_SIZE = 8192
MCU_SRAM_SIZE  = 1024
MCU_STACK_SIZE = 512
# Required oscylator is 12MHz.
MCU_FREQ       = 12000000

include ./Makefile.main.mk
