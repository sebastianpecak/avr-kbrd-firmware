OUTPUT_NAME=keyboard_firmware
# C compiler.
CC=$(shell which avr-gcc)
# C++ compiler.
CPPC=$(shell which avr-g++)
AVRDUDE=$(shell which avrdude)

C_SOURCES=$(shell ls *.c)
CPP_SOURCES=$(shell ls *.cpp)

TARGET_ARCH = -mmcu=$(TARGET_MCU)


check    = $(shell $(CC) $1 -c -xc /dev/null -o/dev/null 2>/dev/null && echo $1)

OBJECTS = main.o
#CC    = avr-gcc
OPTIM    = -Os -ffunction-sections $(call check,-fno-split-wide-types)
CFLAGS    = -g -Wall -I. -I./usbtiny $(OPTIM) -DF_CPU=$(MCU_FREQ) -std=c11
LDFLAGS    = -g -Wl,--relax,--gc-sections
MODULES = usbtiny/crc.o usbtiny/int.o usbtiny/usb.o asm_tools.o $(OBJECTS)
UTIL    = ./util

STACK        = 1024
FLASH        = 32000
SRAM        = 2048

main.hex:

all:        main.hex $(SCHEM)

clean:
	rm -f main.elf *.o tags *.sch~ gschem.log *.hex

clobber:    clean
	rm -f main.hex $(SCHEM)

main.elf:    $(MODULES)
	$(LINK.o) -o $@ $(MODULES)

main.hex:    main.elf $(UTIL)/check.py
	@python $(UTIL)/check.py main.elf $(STACK) $(FLASH) $(SRAM)
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

check:        main.elf $(UTIL)/check.py
	@python $(UTIL)/check.py main.elf $(STACK) $(FLASH) $(SRAM)

disasm:        main.elf
	avr-objdump -S main.elf

flash:        main.hex
	$(FLASH_CMD)

fuses:
	$(FUSES_CMD)

# crc.o:        $(USBTINY)/crc.S $(USBTINY)/def.h usbtiny.h
# 	$(COMPILE.c) $(USBTINY)/crc.S
# int.o:        $(USBTINY)/int.S $(USBTINY)/def.h usbtiny.h
# 	$(COMPILE.c) $(USBTINY)/int.S
# usb.o:        $(USBTINY)/usb.c $(USBTINY)/def.h $(USBTINY)/usb.h usbtiny.h
# 	$(COMPILE.c) $(USBTINY)/usb.c

asm_tools.o: asm_tools.S
	$(COMPILE.c) ./asm_tools.S

main.o:        usbtiny/usb.h

%.ps:        %.sch $(UTIL)/sch2ps
	$(UTIL)/sch2ps $<


debug:
	@echo "TARGET_MCU    : $(TARGET_MCU)"
	@echo "MCU_FLASH_SIZE: $(MCU_FLASH_SIZE)"
	@echo "MCU_SRAM_SIZE : $(MCU_SRAM_SIZE)"
	@echo "MCU_STACK_SIZE: $(MCU_STACK_SIZE)"
	@echo "MCU_FREQ      : $(MCU_FREQ)"
	@echo "CC            : $(CC)"
	@echo "CPPC          : $(CPPC)"
	@echo "AVRDUDE       : $(AVRDUDE)"
	@echo "C_SOURCES     : $(C_SOURCES)"
	@echo "CPP_SOURCES   : $(CPP_SOURCES)"
