#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "dali.h"
#include "timers.h"

// Good documentation for Dali commands - https://www.nxp.com/files-static/microcontrollers/doc/ref_manual/DRM004.pdf

static volatile uint16_t shiftreg_out;
static volatile uint8_t bitsLeft;

static void finishedTransmission() {
    // Turn off timer.
    dali_wait_for_response_state_enter();
}

static void halfBitToggle();

static void sendOne() {
    DALI_PORT.OUTSET = DALI_TX_bm;
    daliTimerCallback = halfBitToggle;
    daliTimer = 1;
}

static void sendZero() {
    DALI_PORT.OUTCLR = DALI_TX_bm;
    daliTimerCallback = halfBitToggle;
    daliTimer = 1;
}

static void startStopBits() {
    DALI_PORT.OUTCLR = DALI_TX_bm;
    daliTimerCallback = finishedTransmission;
    daliTimer = 4;
}

/** 
 * Called in the middle of each output but.  As we are using manchester, it simply toggles the output, then works out what the next bit will be
 */
static void halfBitToggle() {
    DALI_PORT.OUTTGL = DALI_TX_bm;
    // set the handler to What happens at the start of the next bit.
    if (--bitsLeft == 0) {
        daliTimerCallback = startStopBits;
    } else {
        daliTimerCallback = shiftreg_out & 0x8000 ? sendOne: sendZero;
        shiftreg_out <<= 1;
    }
    daliTimer = 1;
}

extern void dali_transmitting_state_enter(dali_command_t *cmd) {
    // Before we start messing with stuff, make sure the timer is off
    shiftreg_out = cmd->cmd;
    bitsLeft = 17; // 1 start bit + 16 data bits; stop bits are emitted by startStopBits() afterwards.

    // We send a one as the start bit.
    // cli();
    daliTimerCallback = sendOne;
    daliTimer = 1;
    // sei();
}

