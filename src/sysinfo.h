#ifndef SYSINFO_H
#define SYSINFO_H

#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
//                       BOOT / RUNTIME DIAGNOSTICS
//
// Uptime, reboot count and reset cause — the three values the FC04 sysinfo block
// (0x0310) serves. None of them has anything to do with DALI, Modbus, or the
// command queue; they are properties of THIS CHIP HAVING BOOTED, so they live in
// their own module rather than as guests of whichever subsystem happened to own
// the timer or the reset path.
//
// No EEPROM or flash is used, so none of this wears the part out even under a
// watchdog crash-loop: uptime is a RAM counter, boot_count lives in .noinit SRAM
// behind a magic guard, and reset_reason is the hardware-latched RSTCTRL.RSTFR.
// ─────────────────────────────────────────────────────────────────────────────

// Latch the reset cause and maintain the warm-reboot counter. Call FIRST in
// main(), before anything else reads or clears RSTCTRL.RSTFR and before the WDT
// re-arms — RSTFR flags are cumulative until cleared, so a late capture blends
// this boot's cause with the previous one's.
void sysinfo_boot_latch(void);

// Start the 1 Hz uptime tick (RTC PIT off the 1 kHz internal oscillator).
// Enables interrupts for it, so call before sei() or accept the first tick
// arriving whenever interrupts open.
void sysinfo_init(void);

// Seconds since boot. Fenced internally: the 32-bit counter is maintained by the
// PIT ISR and a plain read can tear mid-update on an 8-bit core. Keeping the
// fence in here means a caller cannot forget it.
uint32_t sysinfo_uptime_s(void);

// Reboots since the last cold power-on. Survives warm resets (watchdog, software
// reset, external RESET pin); restarts at 1 after a power-on or brownout, where
// SRAM content — and therefore the .noinit magic guard — is lost.
uint16_t sysinfo_boot_count(void);

// RSTCTRL.RSTFR as latched at boot: bit0 PORF, 1 BORF, 2 EXTRF, 3 WDRF, 4 SWRF,
// 5 UPDIRF. A master spots a reboot when uptime drops or boot_count changes, and
// reads this to learn why.
uint8_t sysinfo_reset_reason(void);

#endif
