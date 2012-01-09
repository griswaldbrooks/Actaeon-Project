###############################################################################
# Makefile for the project lbRTOS
###############################################################################

## General Flags
PROJECT = lbRTOS
MCU = atmega640
TARGET = lbRTOS.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99       -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=lbRTOS.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"C:\Robot\lbRTOS\Source\portable\GCC\ATMega323" -I"C:\Robot\lbRTOS\Source\include" -I"C:\Robot\lbRTOS\." -I"C:\Robot\lbRTOS\SoR" 

## Libraries
LIBS = -lm -lc 

## Objects that must be built in order to link
OBJECTS = lbRTOS.o list.o queue.o tasks.o port.o croutine.o heap_1.o timer640.o uart4.o buffer.o a2d.o rprintf.o i2c.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) lbRTOS.hex lbRTOS.eep lbRTOS.lss size

## Compile
lbRTOS.o: ../lbRTOS.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

list.o: ../Source/list.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

queue.o: ../Source/queue.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

tasks.o: ../Source/tasks.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

port.o: ../Source/portable/GCC/ATMega323/port.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

croutine.o: ../Source/croutine.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

heap_1.o: ../Source/portable/MemMang/heap_1.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

timer640.o: ../SoR/timer640.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

uart4.o: ../SoR/uart4.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

buffer.o: ../SoR/buffer.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

a2d.o: ../SoR/a2d.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

rprintf.o: ../SoR/rprintf.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

i2c.o: ../SoR/i2c.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) lbRTOS.elf dep/* lbRTOS.hex lbRTOS.eep lbRTOS.lss lbRTOS.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

