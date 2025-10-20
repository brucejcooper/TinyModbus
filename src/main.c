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


void pulse_led() {
    PORTB.OUTTGL = PIN3_bm;
    PORTB.OUTTGL = PIN3_bm;
}


int main(void) {
    // Device ID is stored in the first byte of the USERRROW EEPROM.  It is offset by 2, so that when set to the
    // default value (0xFF), it comes out as Device no 1.  If you want device 2, write 0x00
    timer_init();
    rs485_init(USERROW.USERROW0 + 2);
    dali_init();
    set_sleep_mode(SLEEP_MODE_STANDBY);
    sei();  // Enable Interrupts

    // LED pin
    PORTB.DIRSET = PIN3_bm;
    sleep_enable();

    // Do an inital scan of the bus.
    dali_start_scan(0);

    PORTB.OUTSET = PIN3_bm;
    for (;;) {
        // We are idle, so go to sleep
        if (dali_idle && modbus_idle) {
            PORTB.OUTCLR = PIN3_bm;
            sleep_cpu();
            PORTB.OUTSET = PIN3_bm;
        }
    }
}

