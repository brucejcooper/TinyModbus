#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/fuse.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <util/delay.h>

#include "registers.h"
#include "tick.h"
#include "dali.h"
#include "dali_sync.h"
#include "modbus.h"

// Fuse configuration.  Emits a `.fuse` section in the ELF; the Makefile's
// avr-objcopy invocation passes this through into main.hex so pymcuprog
// programs it on flash.  Brown-out detection is enabled at 2.6 V in active
// mode - this is the highest-leverage protection against the bus-stuck
// symptom seen when Vcc dips during DALI activity.  All other fuse bytes
// are left at their factory defaults.
FUSES = {
    .WDTCFG  = FUSE_WDTCFG_DEFAULT,
    .BODCFG  = ACTIVE_ENABLED_gc | LVL_BODLEVEL2_gc,
    .OSCCFG  = FUSE_OSCCFG_DEFAULT,
    .SYSCFG0 = FUSE_SYSCFG0_DEFAULT,
    .SYSCFG1 = FUSE_SYSCFG1_DEFAULT,
    .APPEND  = FUSE_APPEND_DEFAULT,
    .BOOTEND = FUSE_BOOTEND_DEFAULT,
};

int main(void) {
    // Drive DE low immediately - prevents floating pin from enabling RS-485 TX
    // during the startup window before modbus_init() configures the pin.
    PORTA.DIRSET = PIN4_bm;
    PORTA.OUTCLR = PIN4_bm;

    // PB3 = logic-probe channel D7, reserved as a general-purpose firmware timing
    // marker.  Configured as a driven-LOW output here but NOT toggled by any
    // current code — drop a PORTB.OUTSET/OUTCLR/OUTTGL PIN3_bm at any point of
    // interest to expose it on the scope.  (PB3 is otherwise unused — DALI is
    // PB0/PB1.)
    PORTB.DIRSET = PIN3_bm;
    PORTB.OUTCLR = PIN3_bm;

    // 10 MHz internal oscillator (safe at 3.3 V, per datasheet)
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;

    // Hold off bus traffic for 1 s after reset so DALI ballasts on the same
    // supply rail have time to power up and start responding before the
    // initial SYNC_READ_FULL sweep marks them absent.  Runs after the clock
    // switch so _delay_ms is calibrated against F_CPU = 10 MHz.
    _delay_ms(1000);

    // Initialise subsystems
    regs_init();
    dali_sync_init();                     // sync_flags[], current_op, group_write
    tick_init();                          // starts TCA0 - ticks begin here
    modbus_init(userrow_get_slave_addr());
    dali_init();

    set_sleep_mode(SLEEP_MODE_STANDBY);
    sleep_enable();

    // WDT: 8.2 s timeout.  WDT.CTRLA is CCP-protected; a plain store is silently
    // ignored.  _PROTECTED_WRITE emits the CCP unlock and the register store, which
    // must complete uninterrupted within 4 cycles - we run it before sei() so an ISR
    // cannot slip into the OUT/STS gap and blow past the protection window.
    _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8KCLK_gc);
    wdt_reset();

    sei();

    // Initial bus scan: arm SYNC_READ_FULL on every ballast.  The work
    // selector services them one at a time via the unified DALI
    // issuance path (which sets the present bit on first OK response).
    for (uint8_t i = 0; i < DALI_MAX_DEVICES; i++)
        sync_flags_set(i, SYNC_READ_FULL);

    for (;;) {
        modbus_process();
        // Kick the sync-flag-driven work selector when the bus is idle
        // and no sync op is currently in flight.
        if (dali_idle && !dali_busy)
            start_next_dali_work();
        wdt_reset();

        // Sleep left ENABLED — confirmed (again) NOT the cause of the deafness:
        // with sleep disabled and scanning disabled the gateway is still dead.
        if (dali_idle && modbus_idle && !dali_busy)
            sleep_cpu();
    }
}
