#include <avr/interrupt.h>
#include <avr/io.h>


static uint8_t coilValues = 0;


extern void relay_init() {
    // Set up Relay outputs, which are split amongst PA and PB
    PORTA.OUTCLR = PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;
    PORTA.DIRSET = PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;
    PORTB.OUTCLR = PIN4_bm | PIN5_bm;
    PORTB.DIRSET = PIN4_bm | PIN5_bm;
}


extern void latchOutputs() {
    // Bit 0 -> PB5
    if (coilValues & 0x01) {
        PORTB.OUTSET = PIN5_bm;
    } else {
        PORTB.OUTCLR = PIN5_bm;
    }
    // Bit 1 -> PB4
    if (coilValues & 0x02) {
        PORTB.OUTSET = PIN4_bm;
    } else {
        PORTB.OUTCLR = PIN4_bm;
    }

    // Bit 2->7 match to PORTA directly, by pure chance.
    PORTA.OUT = coilValues & 0xFC;
}

extern void setCoil(uint8_t addr, uint8_t val) {
    uint8_t bm = 1 << addr;
    if (val == 0xFF) {
        // Set
        coilValues |= bm;
    } else if (val == 0x55) {
        // Toggle
        coilValues ^= bm;
    } else {
        // Clear
        coilValues &= ~bm;
    }
}


extern uint8_t getCoils(uint8_t addr) {
    return coilValues >> addr;
}