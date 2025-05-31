# TinyModbus
This is a simple little ATTINY program intended to act as a MODBUS device, processing inputs for up to 8 relays.

The PCB design is in the `pcb` folder.  It is set up to allow 8 [PTF08A](https://www.aliexpress.com/w/wholesale-PTF08A.html?spm=a2g0o.home.search.0) housings with [Omrom LY2](https://www.omron.com.au/products/family/949/lineup.html) relays side by side.  The PCB automatically slots beneath the housings.

![PCB](pcb/TinyModbus.png)

## Device addressing
Each device can have its own Modbus Device Address programmed using byte 0 of the USERROW EEPROM value.  Whatever is written will have 2 added to it.  This means that the default value of 0xFF will translate to 1.  If you write 0 then it will have a deviceId of 2, and so on and so forth.

See the `configure` makefile target for an exmaple of how to program it

```sh
pymcuprog -t uart -u ${PORT} -d ${DEVICE} -m user_row write -l 0x01
```

## Supported Modbus Commands

1. Get Coils

## Building
This thing just needs a version of avr-gcc with support for my CPU (attiny806) and pymcuprog.  I installed pymcuprog using pip.  I tried using the version of avr-gcc that comes from homebrew (for macs) but it didn't have the CPU support built in.  So instead, the makefile has a command `download_gcc` that will download the version used by Arduino, and host it locally.

```sh
make download_gcc
# This will both build and flash the device
make
```

Programming is done via UPDI using the test point on the board.  Use a [Serial adapter board](https://github.com/SpenceKonde/AVR-Guidance/blob/master/UPDI/jtag2updi.md) and a standard FTDI<->USB thingy.