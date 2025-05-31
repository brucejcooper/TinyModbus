# TinyModbus
This is a simple little ATTINY program intended to act as a MODBUS device, processing inputs for up to 8 relays.

The PCB design is in the `pcb` folder.  It is set up to allow 8 PTF08A relay housings side by side.  The PCB automatically slots beneath the housings.

## Building
This thing just needs a version of avr-gcc with support for my CPU (attiny806) and pymcuprog.  I installed pymcuprog using pip.  I tried using the version of avr-gcc that comes from homebrew (for macs) but it didn't have the CPU support built in.  So instead, the makefile has a command `download_gcc` that will download the version used by Arduino, and host it locally.

```sh
make download_gcc
# This will both build and flash the device
make
```