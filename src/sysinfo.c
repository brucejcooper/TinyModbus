// sysinfo.c — uptime, reboot count and reset cause. See sysinfo.h.
//
// Not on any hot path: one ISR that increments a counter once a second, and
// three accessors read only when a master polls FC04 0x0310.

#pragma GCC optimize("Os")

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "sysinfo.h"

// g_boot_count / s_boot_magic live in .noinit SRAM: the C runtime does NOT clear
// .noinit, so they survive a WARM reset (watchdog, software RSTCTRL.SWRR, the
// external RESET pin) but are undefined after a true power-on / brownout where
// SRAM content is lost. The magic guard detects that (and PORF) and restarts the
// count, so boot_count means "reboots since the last cold power-on".
#define BOOT_MAGIC 0xB007u

static volatile uint32_t s_uptime_s;
static volatile uint16_t s_boot_count __attribute__((section(".noinit")));
static          uint16_t s_boot_magic __attribute__((section(".noinit")));
static volatile uint8_t  s_reset_reason;

void sysinfo_boot_latch(void) {
    // RSTFR is write-1-to-clear and cumulative until cleared, so capture then
    // clear to keep each boot's reason distinct.
    uint8_t rr = RSTCTRL.RSTFR;
    RSTCTRL.RSTFR  = rr;
    s_reset_reason = rr;
    // Cold boot (power-on/brownout) or corrupted .noinit magic => restart count.
    if (s_boot_magic != BOOT_MAGIC || (rr & RSTCTRL_PORF_bm)) {
        s_boot_count = 0;
        s_boot_magic = BOOT_MAGIC;
    }
    s_boot_count++;
}

void sysinfo_init(void) {
    // RTC PIT off the 1 kHz internal oscillator: 1024 cycles ≈ 1 s.
    // This timer exists ONLY for the uptime counter. It was set up in
    // dali_init() and described as driving a "periodic bus rescan" — that rescan
    // became the controller's job in task 05, leaving the PIT with no DALI
    // purpose and a comment that no longer described anything.
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;
    while (RTC.PITSTATUS & RTC_CTRLABUSY_bm) {}
    RTC.PITCTRLA   = RTC_PERIOD_CYC1024_gc | RTC_PITEN_bm;
    RTC.PITINTCTRL = RTC_PI_bm;
}

ISR(RTC_PIT_vect) {
    s_uptime_s++;
    RTC.PITINTFLAGS = RTC_PI_bm;
}

uint32_t sysinfo_uptime_s(void) {
    uint32_t v;
    // A 32-bit read is four byte loads on this core; the ISR can land between
    // them and produce a value that never existed.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = s_uptime_s; }
    return v;
}

uint16_t sysinfo_boot_count(void)  { return s_boot_count; }
uint8_t  sysinfo_reset_reason(void) { return s_reset_reason; }
