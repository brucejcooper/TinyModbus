#include <avr/interrupt.h>
#include <avr/io.h>
#include "soft_serial.h"

#define USEC_TO_TICKS(x) ((unsigned)(((float)F_CPU * (x) / (float)1000000) + 0.5))
#define SOFTSERIAL_PER USEC_TO_TICKS(104.2)
#define SOFTSERIAL_CMPVAL_ONE (SOFTSERIAL_PER + 1)

static volatile uint8_t *softserial_ptr;
static volatile uint8_t softserial_byte = 0;
static volatile uint8_t softserial_remaining_bytes = 0;
static volatile uint8_t softserial_remaining_bits = 0;


extern void softserial_init() {
        // Software Serial Setup, @9600 baud using TCA0
    PORTB.OUTSET = PIN1_bm;  // PB1 defaults to high
    PORTB.DIRSET = PIN1_bm;  // PB1 is an output
    TCA0.SINGLE.PER = SOFTSERIAL_PER;
    TCA0.SINGLE.CTRLC = TCA_SINGLE_CMP1OV_bm;  // Default to high
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP1EN_bm;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;  // Interrupt on OVF

}


/**
 * Transmits at most 256 bytes of data out via timer.
 */
extern void softserial_tx(uint8_t *data, uint8_t num) {
    // wait for any previous transmission to complete
    softserial_ptr = data;
    softserial_byte = *softserial_ptr++;
    softserial_remaining_bytes = num - 1;  // -1 because we've already popped one off.
    softserial_remaining_bits = 10;

    // transmissionFlags |= TXFLAG_SOFT;

    TCA0.SINGLE.CMP1 = SOFTSERIAL_CMPVAL_ONE;  // No toggling on first cycle
    TCA0.SINGLE.CNT = 4;                       // We've got some interrupt delay and setup instructions
                                               // which add latency, so we start a bit higher for the first bit
    TCA0.SINGLE.CTRLC = 0;                     // Drive output low
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV1_gc;        // Start Timer.
    TCA0.SINGLE.CMP1BUF = (softserial_byte & 0x01) ? SOFTSERIAL_CMPVAL_ONE : 0;  // Buffer next Period which will be the LSB
}



ISR(TCA0_OVF_vect) {
    switch (--softserial_remaining_bits) {
        case 2:
            // B7 has been set as CMP1. Set next bit to a 1 (Stop bit)
            TCA0.SINGLE.CMP1BUF = SOFTSERIAL_CMPVAL_ONE;
            break;
        case 1:
            // STOP bit has been set as CMP1.  Check to see if there's another byte still to send.
            // Set to 0 (a start bit) if there is, or 1 otherwise (leave it high).
            TCA0.SINGLE.CMP1BUF = softserial_remaining_bytes ? 0 : SOFTSERIAL_CMPVAL_ONE;
            break;
        case 0:
            // START bit has been set as CMP1. Set next bit to Bit 0 of the new byte
            if (softserial_remaining_bytes) {
                softserial_byte = *softserial_ptr++;
                softserial_remaining_bytes--;
                softserial_remaining_bits = 10;
                TCA0.SINGLE.CMP1BUF = softserial_byte & 0x01 ? SOFTSERIAL_CMPVAL_ONE : 0;  // Buffer next Period.
            } else {
                TCA0.SINGLE.CTRLA = 0;              // Stop the timer.
                // transmissionFlags &= ~TXFLAG_SOFT;  // Mark transmission as complete.
            }
            break;
        default:
            softserial_byte >>= 1;
            TCA0.SINGLE.CMP1BUF = softserial_byte & 0x01 ? SOFTSERIAL_CMPVAL_ONE : 0;  // Buffer next Period.
            break;
    }
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;  // Clear interrupt flag
}

