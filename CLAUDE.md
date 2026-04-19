# TinyModbus Project Notes

## Toolchain

Always use the project-local AVR toolchain in `avr/bin/` (e.g. `avr/bin/avr-gcc`, `avr/bin/avr-nm`, `avr/bin/avr-size`). Never use system-installed AVR tools from `/opt/homebrew` or anywhere else — they are out of date and produce incorrect results for this project.

The Makefile already sets `PATH := $(shell pwd)/$(AVR_GCC_DIR)/bin:$(PATH)` so `make build` uses the correct tools automatically. For manual invocations, prefix with `avr/bin/` or set PATH accordingly.

## Linker script fix

The bundled avr-gcc 7.3.0 has an incorrect device spec for ATtiny1604: `crtattiny1604.o` defines `__DATA_REGION_LENGTH__ = 0x400` (1KB) and `__stack = 0x3FFF`, but the chip has 2KB of SRAM (0x3C00–0x43FF). The Makefile overrides these weak symbols at link time:

```
-Wl,--defsym=__DATA_REGION_LENGTH__=0x800,--defsym=__stack=0x43ff
```

Without this, the linker reports `.bss not within region data` even though the firmware fits comfortably within the actual 2KB SRAM.

## Diagnostics

Clang (macOS) will report false-positive errors for AVR-specific attributes (`__progmem__`, `__AVR_TINY_PM_BASE_ADDRESS__`, ISR attributes, EEPROM macros). These are harmless — the code targets avr-gcc, not clang.
