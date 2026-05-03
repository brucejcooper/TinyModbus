#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <util/delay.h>

#include "registers.h"
#include "cmd_queue.h"
#include "tick.h"
#include "dali.h"
#include "modbus.h"
#include "bg_tasks.h"

int main(void) {
    // Drive DE low immediately - prevents floating pin from enabling RS-485 TX
    // during the startup window before modbus_init() configures the pin.
    PORTA.DIRSET = PIN4_bm;
    PORTA.OUTCLR = PIN4_bm;

    // 10 MHz internal oscillator (safe at 3.3 V, per datasheet)
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;

    // Initialise subsystems
    regs_init();
    cmdq_init();
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

    // Initial bus scan
    bgtasks_trigger_scan();
    bgtasks_run();

    for (;;) {
        modbus_process();
        bgtasks_run();
        wdt_reset();

        if (dali_idle && modbus_idle)
            sleep_cpu();
    }
}
