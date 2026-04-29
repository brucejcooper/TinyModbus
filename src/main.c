#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "rs485.h"
#include "dali.h"
#include "timers.h"




int main(void) {

    // Drive RS485 DE low before anything else so the transceiver does not drive the
    // RS485 bus during the startup window after a WDT or power-on reset (the pin
    // floats high-Z on reset; an external pull-up or the chip's internal default
    // would otherwise assert the driver and block other devices on the bus).
    PORTA.DIRSET = PIN4_bm;
    PORTA.OUTCLR = PIN4_bm;

    // Set ourselves up to run at 10Mhz.  We will spend most of our time asleep, so going fast while in operation is fine.
    CCP = CCP_IOREG_gc;
    // 10Mhz is the best we can reliably do at 3.3V (according to the datasheet)
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;

    // LED pin
    PORTB.DIRSET = PIN3_bm;
    PORTB.OUTSET = PIN3_bm;


    PORTA.DIRSET = PIN7_bm;
    PORTA.OUTSET = PIN7_bm;


    // Device ID is stored in the first byte of the USERROW EEPROM, offset by 2 (so the factory
    // default 0xFF maps to ID 1; write 0x00 for ID 2, etc.). Modbus reserves IDs 0 and 248–255,
    // so we clamp anything that would map there back to the default ID 1:
    //   userrow ≤ 0xF5 → IDs 2..247  (valid)
    //   userrow 0xF6..0xFD → would map to reserved IDs 248..255 → clamp to 1
    //   userrow 0xFE → would wrap to 0 (broadcast) → clamp to 1
    //   userrow 0xFF (default) → wraps to 1 (default)
    timer_init();
    uint8_t userrow = USERROW.USERROW0;
    uint8_t deviceId = (userrow >= 0xF6 && userrow != 0xFF) ? 1 : (uint8_t)(userrow + 2);
    rs485_init(deviceId);
    dali_init();
    set_sleep_mode(SLEEP_MODE_STANDBY);

    // Set up a WDT, with a timeout of 8.2 seconds. This is ooodles more than needed.
    // WDT.CTRLA is Configuration Change Protected — _PROTECTED_WRITE emits the CCP unlock
    // followed by the register write, and must complete uninterrupted within 4 cycles.
    // We do this before sei() so an ISR can't slip in between the OUT and STS instructions.
    _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8KCLK_gc);
    wdt_reset();

    sei();  // Enable Interrupts


    sleep_enable();

    // Do an inital scan of the bus.
    dali_start_background_scan(0);

    for (;;) {
        rs485_process_command();
        wdt_reset();

        // We are idle, so go to sleep
        if (dali_idle &&  modbus_idle) {
            // PORTB.OUTCLR = PIN3_bm;
            sleep_cpu();
            // PORTB.OUTSET = PIN3_bm;
        }
    }
}

