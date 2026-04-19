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




#define BLIP_PORT PORTB
#define BLIP_PIN PIN3_bm


void blip(uint8_t ch) {
    // Start bit
    cli();
    BLIP_PORT.OUTCLR = BLIP_PIN;
    _delay_us(8.1);
    if (ch & 0x01) { BLIP_PORT.OUTSET = BLIP_PIN; } else { BLIP_PORT.OUTCLR = BLIP_PIN;}
    _delay_us(8.1);
    if (ch & 0x02) { BLIP_PORT.OUTSET = BLIP_PIN; } else { BLIP_PORT.OUTCLR = BLIP_PIN;}
    _delay_us(8.1);
    if (ch & 0x04) { BLIP_PORT.OUTSET = BLIP_PIN; } else { BLIP_PORT.OUTCLR = BLIP_PIN;}
    _delay_us(8.1);
    if (ch & 0x08) { BLIP_PORT.OUTSET = BLIP_PIN; } else { BLIP_PORT.OUTCLR = BLIP_PIN;}
    _delay_us(8.1);
    if (ch & 0x10) { BLIP_PORT.OUTSET = BLIP_PIN; } else { BLIP_PORT.OUTCLR = BLIP_PIN;}
    _delay_us(8.1);
    if (ch & 0x20) { BLIP_PORT.OUTSET = BLIP_PIN; } else { BLIP_PORT.OUTCLR = BLIP_PIN;}
    _delay_us(8.1);
    if (ch & 0x40) { BLIP_PORT.OUTSET = BLIP_PIN; } else { BLIP_PORT.OUTCLR = BLIP_PIN;}
    _delay_us(8.1);
    if (ch & 0x80) { BLIP_PORT.OUTSET = BLIP_PIN; } else { BLIP_PORT.OUTCLR = BLIP_PIN;}
    _delay_us(8.1);
    BLIP_PORT.OUTSET = BLIP_PIN;
    _delay_us(8.1); // Stop bit
    sei();
}
    



void pulse_led(uint8_t times) {
    int bits = 8;
    // Start bit
    PORTB.OUTCLR = PIN3_bm;
    _delay_us(104);
    while (bits--) {
        if (times & 0x01) {
            PORTB.OUTSET = PIN3_bm;
        } else {
            PORTB.OUTCLR = PIN3_bm;
        }
        _delay_us(103);
        times >>=1;
    }
    PORTB.OUTSET = PIN3_bm;
    _delay_us(208); // Stop bit




    // while (times--) {
    //     PORTB.OUTTGL = PIN3_bm;
    //     PORTB.OUTTGL = PIN3_bm;
    // }
}
    


int main(void) {

    // Set ourselves up to run at 10Mhz.  We will spend most of our time asleep, so going fast while in operation is fine.
    CCP = CCP_IOREG_gc;
    // 10Mhz is the best we can reliably do at 3.3V (according to the datasheet)
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm;

    // LED pin
    PORTB.DIRSET = PIN3_bm;
    PORTB.OUTSET = PIN3_bm;


    PORTA.DIRSET = PIN7_bm;
    PORTA.OUTSET = PIN7_bm;


    // Device ID is stored in the first byte of the USERRROW EEPROM.  It is offset by 2, so that when set to the
    // default value (0xFF), it comes out as Device no 1.  If you want device 2, write 0x00
    timer_init();
    rs485_init(USERROW.USERROW0 + 2);
    dali_init();
    set_sleep_mode(SLEEP_MODE_STANDBY);
    sei();  // Enable Interrupts

    // Set up a WDT, with a timeout of 8.2 seconds. This is ooodles more than needed.
    WDT.CTRLA = WDT_PERIOD_8KCLK_gc;
    wdt_reset();


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

