#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/fuse.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <util/delay.h>

#include "registers.h"
#include "sysinfo.h"
#include "tick.h"
#include "dali.h"
#include "dali_sync.h"
#include "modbus.h"

// Fuse configuration.  Emits a `.fuse` section in the ELF; the Makefile's
// avr-objcopy invocation passes this through into main.hex so pymcuprog
// programs it on flash.  Brown-out detection is enabled in active mode - the
// highest-leverage protection against the bus-stuck symptom seen when Vcc dips
// during DALI activity.  The level is rail-dependent: on the 5 V board rev
// (BOARD_5V) the core runs 20 MHz, which requires V_DD >= 4.5 V, so the BOD
// must catch a sagging rail at ~4.2 V (BODLEVEL7) or the MCU runs out-of-spec;
// on the legacy 3.3 V board 10 MHz is safe and the BOD sits at 2.6 V
// (BODLEVEL2).  All other fuse bytes are left at their factory defaults.
FUSES = {
    .WDTCFG  = FUSE_WDTCFG_DEFAULT,
#if BOARD_5V
    .BODCFG  = ACTIVE_ENABLED_gc | LVL_BODLEVEL7_gc,
#else
    .BODCFG  = ACTIVE_ENABLED_gc | LVL_BODLEVEL2_gc,
#endif
    .OSCCFG  = FUSE_OSCCFG_DEFAULT,
    .SYSCFG0 = FUSE_SYSCFG0_DEFAULT,
    .SYSCFG1 = FUSE_SYSCFG1_DEFAULT,
    .APPEND  = FUSE_APPEND_DEFAULT,
    .BOOTEND = FUSE_BOOTEND_DEFAULT,
};

int main(void) {
    // Reset cause + warm-reboot counter FIRST, before anything touches RSTFR
    // or the WDT re-arms.
    sysinfo_boot_latch();

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

    // Core clock from the 20 MHz internal oscillator.  On the 5 V board rev
    // (BOARD_5V) run it undivided at 20 MHz (V_DD >= 4.5 V, per datasheet);
    // otherwise divide by 2 to 10 MHz, which is the fastest safe rate at 3.3 V.
    CCP = CCP_IOREG_gc;
#if BOARD_5V
    CLKCTRL.MCLKCTRLB = 0;
#else
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;
#endif

    // Hold off bus traffic for 1 s after reset so DALI ballasts on the same
    // supply rail have time to power up before the controller's first queries
    // reach them (a clean no-answer is what it counts as absence).  Runs after
    // the clock
    // switch so _delay_ms is calibrated against F_CPU (20 MHz on the 5 V rev,
    // 10 MHz otherwise — both set by the Makefile to match the divider above).
    _delay_ms(1000);

    // Initialise subsystems
    regs_init();
    dali_sync_init();                     // queue slots + issuance state
    sysinfo_init();                       // 1 Hz uptime tick (RTC PIT)
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

    // No boot scan. TASK 05: discovering the bus is the controller's job — it
    // interrogates every short address through the command queue and holds the
    // only cache. The gateway starts with nothing to know and nothing to lose.

    for (;;) {
        modbus_process();
        // Launch new work only when we are parked on an idle wire AND own no
        // command — the one combination of the two layers that permits it.
        if (dali_line_idle && !dali_cmd_active)
            start_next_dali_work();
        wdt_reset();

        // Sleep left ENABLED — confirmed (again) NOT the cause of the deafness:
        // with sleep disabled and scanning disabled the gateway is still dead.
        if (dali_line_idle && modbus_idle && !dali_cmd_active)
            sleep_cpu();
    }
}
