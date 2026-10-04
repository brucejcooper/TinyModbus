DEVICE     	= attiny1604

# Board revision select.  BOARD_5V=1 is the 5 V synchronous-buck board rev:
# 20 MHz core (osc undivided), OSC20ERR5V trim, BODLEVEL7 (~4.2 V).  BOARD_5V=0
# is the legacy 3.3 V / HT7533-LDO board: 20 MHz osc divided to 10 MHz,
# OSC20ERR3V trim, BODLEVEL2 (2.6 V).  The flag is passed into the build so
# main.c (clock + BOD fuse) and modbus.c (baud_reg_for) select the matching
# rail-dependent constants.  F_CPU is derived from it, never set by hand.
BOARD_5V   	?= 1
ifeq ($(BOARD_5V),1)
CLOCK      	= 20000000
else
CLOCK      	= 10000000
endif

PORT	   	= /dev/tty.usbserial-B0010TZE
FILENAME   	= main
export FW_VERSION=$(shell git describe --long --dirty --tags)
# FW_VER_MAJOR/MINOR are the numbers in the FC04 0x0200 version string
# ("DALI v<maj>.<min> <serial>"), and under queue v4 they are the QUEUE
# GENERATION MARKER the controller latches on (QUEUE-PROTOCOL.md §8).
#
#   0.x  pre-v4: FC23 slot queue, or the never-deployed draft FC65/66
#   1.0  queue v4: FC 0x41, response queue, ack-by-seq
#
# Bumped 0.2 -> 1.0 by task 08. v4 removes the fallback path, so controller and
# gateway are a matched set and the controller must be able to refuse a
# mismatched gateway loudly rather than limp; that needs an unambiguous,
# monotone boundary, and MAJOR >= 1 is it. Do not bump MAJOR again for anything
# short of another wire-breaking queue generation — the controller's
# MBQ_GW_MIN_VER_MAJOR is keyed to it.
# avr-gcc by FULL PATH: with the quoted FW_VERSION string literal below, GNU
# make 3.81 (the macOS default) looks avr-gcc up in its own PATH rather than the
# exported one, finds Homebrew's avr-gcc first, and that compiler has no
# ATtiny1604 device spec. FW_VERSION must be a real C string literal for the
# FC04 0x0210 build block.
COMPILE    	= $(CURDIR)/$(AVR_GCC_DIR)/bin/avr-gcc -Wall -O3 -DF_CPU=$(CLOCK) -DBOARD_5V=$(BOARD_5V) -DFW_VERSION='"$(FW_VERSION)"' -mmcu=$(DEVICE) -DFW_VER_MAJOR=1 -DFW_VER_MINOR=0 -fstack-usage -ffunction-sections -fdata-sections
LDFLAGS    	= -Wl,--gc-sections
AVR_GCC_DIR = avr
CSOURCES    = $(wildcard src/*.c )
ASMSOURCES  = $(wildcard src/*.S )
OBJECTS    	= $(subst src/,build/,$(subst .c,.o,$(CSOURCES))) # $(subst src/,build/,$(subst .S,.o,$(ASMSOURCES)))
# OBJECTS = build/main.o build/timers.o
export PATH := $(shell pwd)/$(AVR_GCC_DIR)/bin:$(PATH)

# None of these name a real file, and two of them — `build` and `test` — share a
# name with a DIRECTORY, so without this make treats those directories as the
# targets and does nothing while exiting 0.  `build` only escaped that today by
# accident, because its `prepare` prerequisite always reruns.
.PHONY: all download_gcc prepare build test erase flash get_config configure reset clean

all: clean build erase flash

download_gcc:
	wget http://downloads.arduino.cc/tools/avr-gcc-7.3.0-atmel3.6.1-arduino7-x86_64-apple-darwin14.tar.bz2 -q -O- | bzcat | tar xv

   
build/%.o: src/%.c
	$(COMPILE) -c $< -o $@

build/%.o: src/%.S
	$(COMPILE) -c $< -o $@


prepare:
	mkdir -p build

# Host unit tests.  NOT part of `build` or `all`: these are host binaries, built
# with the SYSTEM compiler, not the AVR toolchain.  `cc` is safe despite the PATH
# override above — avr/bin ships only avr-* prefixed binaries, nothing that
# shadows it.
#
# This target is gate V5's evidence, not a convenience.  Task 08 deliberately
# moved malformed-frame validation off the bench and into cq_test.c, because the
# rig has no independent RS-485 master to inject bad frames with, so "the host
# test passes" is the only proof that gate has.  It therefore has to be
# reproducible and it has to fail loudly: cc failure or a non-zero exit from the
# binary both fail the make.
#
# Note .PHONY below — without it this target is shadowed by the test/ DIRECTORY
# and `make test` reports "Nothing to be done" while exiting 0, which is a
# clean-looking pass having run nothing at all.
HOSTCC ?= cc
test: prepare
	$(HOSTCC) -Wall -Wextra -I src -o build/cq_test test/cq_test.c src/cq.c src/errors.c
	./build/cq_test
	$(HOSTCC) -Wall -Wextra -I src -o build/frame_pair_test test/frame_pair_test.c src/frame_pair.c
	./build/frame_pair_test

build: prepare $(OBJECTS)
	$(COMPILE) $(LDFLAGS) -o build/$(FILENAME).elf $(OBJECTS)
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
#
# Encoding (matches the read formulas in registers.c):
#   byte 0 (slave):     SLAVE_ADDR=1 -> 0xFF (the post-erase default), else SLAVE_ADDR-2
#   byte 1 (baud):      BAUD_INDEX directly (0=9600, 1=19200, 2=38400)
#   bytes 2-3 (rescan): RETIRED in task 05 — written as 0xFFFF (unused).
# Bytes 4-31 are written 0xFF so the page is in a known state regardless of
# what was there before.
SLAVE_ADDR      ?= 2
BAUD_INDEX      ?= 2
# Rescan interval retired in task 05 (the gateway does no polling of its own —
# the controller owns the background sweep via dali_sweep_interval_ms). The
# USERROW bytes are left written as 0xFFFF so the layout is unchanged.
RESCAN_INTERVAL ?= 65535

USERROW_SLAVE_BYTE := $(if $(filter 1,$(SLAVE_ADDR)),0xFF,$(shell printf '0x%02X' $$(($(SLAVE_ADDR) - 2))))
USERROW_BAUD_BYTE  := $(shell printf '0x%02X' $(BAUD_INDEX))
USERROW_RESCAN_HI  := $(shell printf '0x%02X' $$(($(RESCAN_INTERVAL) >> 8)))
USERROW_RESCAN_LO  := $(shell printf '0x%02X' $$(($(RESCAN_INTERVAL) & 255)))

configure:
	pymcuprog -t uart -u $(PORT) -d $(DEVICE) -m user_row write -l \
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
