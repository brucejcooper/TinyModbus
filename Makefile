DEVICE     	= attiny806
CLOCK      	= 3333333
PORT	   	= /dev/tty.usbserial-B0010TZE
FILENAME   	= main
export FW_VERSION=$(shell git describe --long --dirty --tags)
COMPILE    	= avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -DFW_VERSION="$(FW_VERSION)" -mmcu=$(DEVICE)
AVR_GCC_DIR = avr
CSOURCES    = $(wildcard src/*.c )
ASMSOURCES  = $(wildcard src/*.S )
OBJECTS    	= $(subst src/,build/,$(subst .c,.o,$(CSOURCES))) # $(subst src/,build/,$(subst .S,.o,$(ASMSOURCES)))
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
	avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex build/$(FILENAME).elf build/$(FILENAME).hex
	avr-size --format=avr --mcu=$(DEVICE) build/$(FILENAME).elf

erase:
	pymcuprog -t uart -u $(PORT) -d $(DEVICE) erase

flash:
	pymcuprog -t uart -u $(PORT) -d $(DEVICE) write -f build/$(FILENAME).hex --verify

get_config: 
	pymcuprog -t uart -u ${PORT} -d $(DEVICE) -m user_row read

configure:
	pymcuprog -t uart -u ${PORT} -d $(DEVICE) -m user_row write -l 0x01    0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF

reset:
	pymcuprog -t uart -u ${PORT} -d $(DEVICE) reset


clean:
	rm -rf build/
	rm -f main.elf
	rm -f main.hex
