# Name: Makefile
# Project: hid-mouse example
# Author: Christian Starkjohann
# Creation Date: 2008-04-07
# Tabsize: 4
# Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
# License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)

DEVICE   = atmega8
F_CPU    = 12000000	# in Hz
FUSE_L   = 0xff
FUSE_H   = 0xc9
AVRDUDE  = avrdude -c usbasp -p $(DEVICE) # edit this line for your programmer

CFLAGS   = -Iusbdrv -I. -DDEBUG_LEVEL=0
OBJECTS  = usbdrv/usbdrv.o usbdrv/usbdrvasm.o usbdrv/oddebug.o
JOY_OBJ  = $(OBJECTS) joystick.o
CD32_OBJ = $(OBJECTS) cd32.o
SEGA_OBJ = $(OBJECTS) sega.o
NES_OBJ  = $(OBJECTS) nes.o
SNES_OBJ = $(OBJECTS) snes.o

COMPILE  = avr-gcc -Wall -Os -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE)

##############################################################################
# Fuse values for particular devices
##############################################################################
# If your device is not listed here, go to
# http://palmavr.sourceforge.net/cgi-bin/fc.cgi
# and choose options for external crystal clock and no clock divider
#
################################## ATMega8 ##################################
# ATMega8 FUSE_L (Fuse low byte):
# 0x9f = 1 0 0 1   1 1 1 1
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (external >8M crystal)
#        | |  +--------------- SUT 1..0 (crystal osc, BOD enabled)
#        | +------------------ BODEN (BrownOut Detector enabled)
#        +-------------------- BODLEVEL (2.7V)
# ATMega8 FUSE_H (Fuse high byte):
# 0xc9 = 1 1 0 0   1 0 0 1 <-- BOOTRST (boot reset vector at 0x0000)
#        ^ ^ ^ ^   ^ ^ ^------ BOOTSZ0
#        | | | |   | +-------- BOOTSZ1
#        | | | |   + --------- EESAVE (don't preserve EEPROM over chip erase)
#        | | | +-------------- CKOPT (full output swing)
#        | | +---------------- SPIEN (allow serial programming)
#        | +------------------ WDTON (WDT not always on)
#        +-------------------- RSTDISBL (reset pin is enabled)
#
############################## ATMega48/88/168 ##############################
# ATMega*8 FUSE_L (Fuse low byte):
# 0xdf = 1 1 0 1   1 1 1 1
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (external >8M crystal)
#        | |  +--------------- SUT 1..0 (crystal osc, BOD enabled)
#        | +------------------ CKOUT (if 0: Clock output enabled)
#        +-------------------- CKDIV8 (if 0: divide by 8)
# ATMega*8 FUSE_H (Fuse high byte):
# 0xde = 1 1 0 1   1 1 1 0
#        ^ ^ ^ ^   ^ \-+-/
#        | | | |   |   +------ BODLEVEL 0..2 (110 = 1.8 V)
#        | | | |   + --------- EESAVE (preserve EEPROM over chip erase)
#        | | | +-------------- WDTON (if 0: watchdog always on)
#        | | +---------------- SPIEN (allow serial programming)
#        | +------------------ DWEN (debug wire enable)
#        +-------------------- RSTDISBL (reset pin is enabled)
#
############################## ATTiny25/45/85 ###############################
# ATMega*5 FUSE_L (Fuse low byte):
# 0xef = 1 1 1 0   1 1 1 1
#        ^ ^ \+/   \--+--/
#        | |  |       +------- CKSEL 3..0 (clock selection -> crystal @ 12 MHz)
#        | |  +--------------- SUT 1..0 (BOD enabled, fast rising power)
#        | +------------------ CKOUT (clock output on CKOUT pin -> disabled)
#        +-------------------- CKDIV8 (divide clock by 8 -> don't divide)
# ATMega*5 FUSE_H (Fuse high byte):
# 0xdd = 1 1 0 1   1 1 0 1
#        ^ ^ ^ ^   ^ \-+-/ 
#        | | | |   |   +------ BODLEVEL 2..0 (brownout trigger level -> 2.7V)
#        | | | |   +---------- EESAVE (preserve EEPROM on Chip Erase -> not preserved)
#        | | | +-------------- WDTON (watchdog timer always on -> disable)
#        | | +---------------- SPIEN (enable serial programming -> enabled)
#        | +------------------ DWEN (debug wire enable)
#        +-------------------- RSTDISBL (disable external reset -> enabled)
#
################################ ATTiny2313 #################################
# ATTiny2313 FUSE_L (Fuse low byte):
# 0xef = 1 1 1 0   1 1 1 1
#        ^ ^ \+/   \--+--/
#        | |  |       +------- CKSEL 3..0 (clock selection -> crystal @ 12 MHz)
#        | |  +--------------- SUT 1..0 (BOD enabled, fast rising power)
#        | +------------------ CKOUT (clock output on CKOUT pin -> disabled)
#        +-------------------- CKDIV8 (divide clock by 8 -> don't divide)
# ATTiny2313 FUSE_H (Fuse high byte):
# 0xdb = 1 1 0 1   1 0 1 1
#        ^ ^ ^ ^   \-+-/ ^
#        | | | |     |   +---- RSTDISBL (disable external reset -> enabled)
#        | | | |     +-------- BODLEVEL 2..0 (brownout trigger level -> 2.7V)
#        | | | +-------------- WDTON (watchdog timer always on -> disable)
#        | | +---------------- SPIEN (enable serial programming -> enabled)
#        | +------------------ EESAVE (preserve EEPROM on Chip Erase -> not preserved)
#        +-------------------- DWEN (debug wire enable)


# symbolic targets:
help:
	@echo "This Makefile has no default rule. Use one of the following:"
	@echo "make joystick .. to build joystick.hex"
	@echo "make cd32 ...... to build cd32.hex"
	@echo "make sega ...... to build sega.hex"
	@echo "make nes ....... to build nes.hex"
	@echo "make snes ...... to build snes.hex"
	@echo "make all ....... to build joystick.hex, cd32.hex, sega.hex, nes.hex & snes.hex"
	@echo "=============================================================================="
	@echo "make fuse ...... to flash the fuses"
	@echo "make flashjoy .. to flash the Joystick firmware"
	@echo "make flashcd32 . to flash the CD32 firmware"
	@echo "make flashsega . to flash the Sega firmware"
	@echo "make flashnes .. to flash the NES firmware"
	@echo "make flashsnes . to flash the SNES firmware"
	@echo "make clean ..... to delete objects and hex file"

all: joystick.hex cd32.hex sega.hex nes.hex snes.hex

joystick: joystick.hex

cd32: cd32.hex

sega: sega.hex

nes: nes.hex

snes: snes.hex

# rule for programming fuse bits:
fuse:
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" ] || \
		{ echo "*** Edit Makefile and choose values for FUSE_L and FUSE_H!"; exit 1; }
	$(AVRDUDE) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m

# rule for uploading firmware:
flashjoy: joystick.hex
	$(AVRDUDE) -U flash:w:joystick.hex:i
   
flashcd32: cd32.hex
	$(AVRDUDE) -U flash:w:cd32.hex:i

flashsega: sega.hex
	$(AVRDUDE) -U flash:w:sega.hex:i

flashnes: nes.hex
	$(AVRDUDE) -U flash:w:nes.hex:i

flashnes: snes.hex
	$(AVRDUDE) -U flash:w:snes.hex:i

# rule for deleting dependent files (those which can be built by Make):
clean:
	rm -f *.hex *.lst *.obj *.cof *.list *.map *.eep.hex *.elf *.o usbdrv/*.o main.s usbdrv/oddebug.s usbdrv/usbdrv.s

# Generic rule for compiling C files:
.c.o:
	$(COMPILE) -c $< -o $@

# Generic rule for assembling Assembler source files:
.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

# Generic rule for compiling C to assembler, used for debugging only.
.c.s:
	$(COMPILE) -S $< -o $@

# file targets:

# Since we don't want to ship the driver multipe times, we copy it into this project:
usbdrv:
	cp -r ../../../usbdrv .

joystick.elf: usbdrv $(JOY_OBJ)
	$(COMPILE) -o joystick.elf $(JOY_OBJ)

cd32.elf: usbdrv $(CD32_OBJ)
	$(COMPILE) -o cd32.elf $(CD32_OBJ)

sega.elf: usbdrv $(SEGA_OBJ)
	$(COMPILE) -o sega.elf $(SEGA_OBJ)

nes.elf: usbdrv $(NES_OBJ)
	$(COMPILE) -o nes.elf $(NES_OBJ)

snes.elf: usbdrv $(SNES_OBJ)
	$(COMPILE) -o snes.elf $(SNES_OBJ)

joystick.hex: joystick.elf
	rm -f joystick.hex joystick.eep.hex
	avr-objcopy -j .text -j .data -O ihex joystick.elf joystick.hex
	avr-size joystick.hex

cd32.hex: cd32.elf
	rm -f cd32.hex cd32.eep.hex
	avr-objcopy -j .text -j .data -O ihex cd32.elf cd32.hex
	avr-size cd32.hex

sega.hex: sega.elf
	rm -f sega.hex sega.eep.hex
	avr-objcopy -j .text -j .data -O ihex sega.elf sega.hex
	avr-size sega.hex

nes.hex: nes.elf
	rm -f nes.hex nes.eep.hex
	avr-objcopy -j .text -j .data -O ihex nes.elf nes.hex
	avr-size nes.hex

snes.hex: snes.elf
	rm -f snes.hex snes.eep.hex
	avr-objcopy -j .text -j .data -O ihex snes.elf snes.hex
	avr-size snes.hex

cpp:
	$(COMPILE) -E joystick.c
	$(COMPILE) -E cd32.c
	$(COMPILE) -E sega.c
	$(COMPILE) -E nes.c
	$(COMPILE) -E snes.c
