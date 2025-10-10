#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "relays.h"
#include "soft_serial.h"
#include "rs485.h"



int main(void) {
    // Device ID is stored in the first byte of the USERRROW EEPROM.  It is offset by 2, so that when set to the
    // default value (0xFF), it comes out as Device no 1.  If you want device 2, write 0x00
    rs485_init(USERROW.USERROW0 + 2);
    set_sleep_mode(SLEEP_MODE_STANDBY);
    sei();  // Enable Interrupts


    for (;;) {
        // If we're not transmitting anything, then go to sleep
        if (!rs485_is_transmitting()) {
            sleep_mode();
        }
        rs485_poll();
    }
}

