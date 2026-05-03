DEVICE     	= attiny1604
CLOCK      	= 10000000
PORT	   	= /dev/tty.usbserial-B0010TZE
FILENAME   	= main
export FW_VERSION=$(shell git describe --long --dirty --tags)
COMPILE    	= avr-gcc -Wall -O3 -DF_CPU=$(CLOCK) -DFW_VERSION="$(FW_VERSION)" -mmcu=$(DEVICE) -DFW_VER_MAJOR=0 -DFW_VER_MINOR=2 -fstack-usage
AVR_GCC_DIR = avr
CSOURCES    = $(wildcard src/*.c )
ASMSOURCES  = $(wildcard src/*.S )
OBJECTS    	= $(subst src/,build/,$(subst .c,.o,$(CSOURCES))) # $(subst src/,build/,$(subst .S,.o,$(ASMSOURCES)))
# OBJECTS = build/main.o build/timers.o
export PATH := $(shell pwd)/$(AVR_GCC_DIR)/bin:$(PATH)

all: clean build erase flash

download_gcc:
	wget http://downloads.arduino.cc/tools/avr-gcc-7.3.0-atmel3.6.1-arduino7-x86_64-apple-darwin14.tar.bz2 -q -O- | bzcat | tar xv

   
build/%.o: src/%.c
	$(COMPILE) -c $< -o $@

build/%.o: src/%.S
	$(COMPILE) -c $< -o $@


prepare:
	mkdir -p build
	
build: prepare $(OBJECTS)
	$(COMPILE) -o build/$(FILENAME).elf $(OBJECTS)
	avr-objcopy -R .eeprom -R .lock -R .signature -R .user_signatures -O ihex build/$(FILENAME).elf build/$(FILENAME).hex
	avr-size --format=avr --mcu=$(DEVICE) build/$(FILENAME).elf

erase:
	pymcuprog -t uart -u $(PORT) -d $(DEVICE) erase

flash:
	pymcuprog -t uart -u $(PORT) -d $(DEVICE) write -f build/$(FILENAME).hex --verify

get_config: 
	pymcuprog -t uart -u ${PORT} -d $(DEVICE) -m user_row read

# `make configure` writes the three persisted-config values (slave addr, baud
# index, rescan interval) into USERROW.  Override any of these on the command
# line; defaults match the firmware's "fresh chip" defaults.
#   make configure                                # slave 1, 9600 baud, 64 s rescan
#   make configure SLAVE_ADDR=5 BAUD_INDEX=1      # slave 5, 19200, default rescan
#   make configure RESCAN_INTERVAL=120            # change just the rescan
#
# Encoding (matches the read formulas in registers.c):
#   byte 0 (slave):     SLAVE_ADDR=1 -> 0xFF (the post-erase default), else SLAVE_ADDR-2
#   byte 1 (baud):      BAUD_INDEX directly (0=9600, 1=19200, 2=38400)
#   bytes 2-3 (rescan): RESCAN_INTERVAL big-endian (high byte first)
# Bytes 4-31 are written 0xFF so the page is in a known state regardless of
# what was there before.
SLAVE_ADDR      ?= 1
BAUD_INDEX      ?= 0
RESCAN_INTERVAL ?= 64

USERROW_SLAVE_BYTE := $(if $(filter 1,$(SLAVE_ADDR)),0xFF,$(shell printf '0x%02X' $$(($(SLAVE_ADDR) - 2))))
USERROW_BAUD_BYTE  := $(shell printf '0x%02X' $(BAUD_INDEX))
USERROW_RESCAN_HI  := $(shell printf '0x%02X' $$(($(RESCAN_INTERVAL) >> 8)))
USERROW_RESCAN_LO  := $(shell printf '0x%02X' $$(($(RESCAN_INTERVAL) & 255)))

configure:
	pymcuprog -t uart -u $(PORT) -d $(DEVICE) -m user_row write -l 0x00 \
		$(USERROW_SLAVE_BYTE) $(USERROW_BAUD_BYTE) \
		$(USERROW_RESCAN_HI) $(USERROW_RESCAN_LO) \
		0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF \
		0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF

reset:
	pymcuprog -t uart -u ${PORT} -d $(DEVICE) reset


clean:
	rm -rf build/
	rm -f main.elf
	rm -f main.hex
