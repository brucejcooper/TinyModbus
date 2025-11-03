#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <string.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "rs485.h"
#include "dali.h"
#include "timers.h"


void pulse_led(uint8_t times) {
    while (times--) {
        PORTB.OUTTGL = PIN3_bm;
        PORTB.OUTTGL = PIN3_bm;
    }
}
    


int main(void) {

    // Set ourselves up to run at 10Mhz.  We will spend most of our time asleep, so going fast while in operation is fine.
    CCP = CCP_IOREG_gc;
    // 10Mhz is the best we can reliably do at 3.3V (according to the datasheet)
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;


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
    dali_start_background_scan(0);
    // dali_set_level(0, 0, NULL);
    // dali_set_level(0, 254, NULL);
    // dali_set_level(0, 0, NULL);

    PORTB.OUTSET = PIN3_bm;
    for (;;) {
        rs485_process_command();
        // We are idle, so go to sleep
        if (dali_idle && modbus_idle) {
            // PORTB.OUTCLR = PIN3_bm;
            sleep_cpu();
            // PORTB.OUTSET = PIN3_bm;
        }
    }
}

