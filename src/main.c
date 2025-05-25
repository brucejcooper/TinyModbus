#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <stdbool.h>


#define DOUT_bm PORT_INT7_bm
#define DIN_bm PORT_INT2_bm
#define RELAY_bm PORT_INT1_bm


static void reset() {
    // Write a bit into the SWRR to reboot the device (to the bootloader).
    RSTCTRL.SWRR = RSTCTRL_SWRE_bm;
}




int main(void) {

    // Set up outputs
    PORTA.OUTCLR = RELAY_bm;
    PORTA.OUTSET = DOUT_bm;
    PORTA.DIRSET = DOUT_bm | RELAY_bm;

    sei();

    // Blinky
    while (1) {
        PORTA.OUTSET = RELAY_bm;
        _delay_ms(500);
        PORTA.OUTCLR = RELAY_bm;
        _delay_ms(500);
        // sleep_mode();
    }
}


ISR(RTC_PIT_vect) {
    RTC.PITINTFLAGS = RTC_PI_bm;
}