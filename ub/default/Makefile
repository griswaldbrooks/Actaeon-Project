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
INCLUDES = -I"C:\Robot\Actaeon-Project\ub\." -I"C:\Robot\Actaeon-Project\ub\SoR" -I"C:\Robot\Actaeon-Project\ub\obstacle_avoidance\_main_" 

## Libraries
LIBS = -lm -lc 

## Objects that must be built in order to link
OBJECTS = lbRTOS.o timer640.o uart4.o a2d.o rprintf.o i2c.o lds.o buffer.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) lbRTOS.hex lbRTOS.eep lbRTOS.lss## Compile
lbRTOS.o: ../lbRTOS.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

timer640.o: ../SoR/timer640.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

uart4.o: ../SoR/uart4.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

a2d.o: ../SoR/a2d.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

rprintf.o: ../SoR/rprintf.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

i2c.o: ../SoR/i2c.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

lds.o: ../lds.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

buffer.o: ../SoR/buffer.c
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

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) lbRTOS.elf dep/* lbRTOS.hex lbRTOS.eep lbRTOS.lss lbRTOS.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

