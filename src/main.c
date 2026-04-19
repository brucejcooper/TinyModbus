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

// Diagnostic: transmit a byte as a serial pulse-train on PB3 (scope/analyser only)
void pulse_led(uint8_t times) {
    int bits = 8;
    PORTB.OUTCLR = PIN3_bm;
    _delay_us(104);
    while (bits--) {
        if (times & 0x01) PORTB.OUTSET = PIN3_bm;
        else              PORTB.OUTCLR = PIN3_bm;
        _delay_us(103);
        times >>= 1;
    }
    PORTB.OUTSET = PIN3_bm;
    _delay_us(208);
}

int main(void) {
    // Drive DE low immediately — prevents floating pin from enabling RS-485 TX
    // during the startup window before modbus_init() configures the pin.
    PORTA.DIRSET = PIN4_bm;
    PORTA.OUTCLR = PIN4_bm;

    // 10 MHz internal oscillator (safe at 3.3 V, per datasheet)
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;

    // Diagnostic LED pin
    PORTB.DIRSET = PIN3_bm;
    PORTB.OUTSET = PIN3_bm;

    // PA7 used elsewhere (keep high-Z output as original)
    PORTA.DIRSET = PIN7_bm;
    PORTA.OUTSET = PIN7_bm;

    // Initialise subsystems
    regs_init();
    cmdq_init();
    tick_init();                          // starts TCA0 — ticks begin here
    modbus_init(g_regs.mb_slave_addr);
    dali_init();

    set_sleep_mode(SLEEP_MODE_STANDBY);
    sleep_enable();

    // WDT: 8.2 s timeout
    WDT.CTRLA = WDT_PERIOD_8KCLK_gc;
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
