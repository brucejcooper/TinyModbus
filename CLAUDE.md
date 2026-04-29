# TinyModbus Project Notes

## Toolchain

Always use the project-local AVR toolchain in `avr/bin/` (e.g. `avr/bin/avr-gcc`, `avr/bin/avr-nm`, `avr/bin/avr-size`). Never use system-installed AVR tools from `/opt/homebrew` or anywhere else - they are out of date and produce incorrect results for this project.

The Makefile already sets `PATH := $(shell pwd)/$(AVR_GCC_DIR)/bin:$(PATH)` so `make build` uses the correct tools automatically. For manual invocations, prefix with `avr/bin/` or set PATH accordingly.

## RAM budget

ATtiny1604 has 1KB SRAM. Static data (BSS + data + COMMON) must stay well under 1KB to leave room for the stack (~150 bytes minimum). Key consumers:

- `g_regs` (registers.c): 702 bytes - the register map for 64 ballasts
- `g_cmd_queue` (cmd_queue.c): ~82 bytes - 8-entry ring buffer with packed enums
- `buf` (modbus.c): 72 bytes - Modbus frame buffer, limits max FC03/FC04 read to 32 registers

Enum fields in `cmd_entry_t` use `__attribute__((packed))` to force 1-byte storage (instead of default 2-byte int-sized enums on AVR). The Modbus version string (FC04 at 0x0200) is built on demand directly into `buf` rather than kept in a static buffer.

## Diagnostics

Clang (macOS) will report false-positive errors for AVR-specific attributes (`__progmem__`, `__AVR_TINY_PM_BASE_ADDRESS__`, ISR attributes, EEPROM macros). These are harmless - the code targets avr-gcc, not clang.
