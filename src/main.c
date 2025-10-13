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
    PORTB.OUTTGL = PIN3_bm;  // clear the LED.
    PORTB.OUTTGL = PIN3_bm;  // clear the LED.

}

int main(void) {
    // Device ID is stored in the first byte of the USERRROW EEPROM.  It is offset by 2, so that when set to the
    // default value (0xFF), it comes out as Device no 1.  If you want device 2, write 0x00
    timeout_init();
    rs485_init(USERROW.USERROW0 + 2);
    dali_init();
    // set_sleep_mode(SLEEP_MODE_STANDBY);
    sei();  // Enable Interrupts

    // LED pin
    PORTB.DIRSET = PIN3_bm;

    // Do an inital scal of the bus.
    dali_start_scan(0);


    for (;;) {
        while (rs485_process_command() || !dali_idle()) {
            ;
        }
        // pulse_led();
        // // We are idle, so go to sleep
        // sleep_mode();
    }
}

