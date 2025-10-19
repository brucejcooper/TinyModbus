#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "dali.h"
#include "timers.h"

// Good documentation for Dali commands - https://www.nxp.com/files-static/microcontrollers/doc/ref_manual/DRM004.pdf

static volatile uint16_t shiftReg;
static volatile int8_t bitsLeft;  // Signed because we allow this to go negative.
// static volatile uint8_t lastBit;

static void finishedTransmission() {
    // Turn off timer.
    cancel_timeout();
    dali_wait_for_response_state_enter();
}

void halfBitToggle();

void sendOne() {
    DALI_PORT.OUTSET = DALI_TX_bm;
    update_timeout_callback(halfBitToggle);
}

void sendZero() {
    DALI_PORT.OUTCLR = DALI_TX_bm;
    update_timeout_callback(halfBitToggle);
}

void startStopBits() {
    DALI_PORT.OUTCLR = DALI_TX_bm;
    on_timeout(USEC_TO_TICKS(2*DALI_BIT_USECS), finishedTransmission);
}

/** 
 * Called in the middle of each output but.  As we are using manchester, it simply toggles the output, then works out what the next bit will be
 */
void halfBitToggle() {
    DALI_PORT.OUTTGL = DALI_TX_bm;
    // set the handler to What happens at the start of the next bit.
    if (--bitsLeft == 0) {
        update_timeout_callback(startStopBits);
    } else {
        update_timeout_callback(shiftReg & 0x8000 ? sendOne: sendZero);
        shiftReg <<= 1;
    }
}

extern void dali_transmitting_state_enter(dali_command_t *cmd) {
    // Before we start messing with stuff, make sure the timer is off
    TCA0.SINGLE.CTRLA = 0;
    shiftReg = cmd->cmd;
    bitsLeft = 17; // One extra for the stop bit. 
    on_timeout(USEC_TO_TICKS(DALI_HALF_BIT_USECS), halfBitToggle);
    TCA0.SINGLE.CNT = 38;  // Give it a bit of head start for the first bit, as there is latency in ISR latency and set up to consider.
    DALI_PORT.OUTSET = DALI_TX_bm; // Short dali line (start of start)
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm; // Start TCA0 for timeout.
}

