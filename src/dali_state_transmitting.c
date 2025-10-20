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
static volatile uint8_t bitsLeft;

static void finishedTransmission() {
    // Turn off timer.
    dali_wait_for_response_state_enter();
}

static void halfBitToggle();

static void sendOne() {
    DALI_PORT.OUTSET = DALI_TX_bm;
    daliTimer.callback = halfBitToggle;
    timer_extend(daliTimer, 1);
}

static void sendZero() {
    DALI_PORT.OUTCLR = DALI_TX_bm;
    daliTimer.callback = halfBitToggle;
    timer_extend(daliTimer, 1);
}

static void startStopBits() {
    DALI_PORT.OUTCLR = DALI_TX_bm;
    daliTimer.callback = finishedTransmission;
    timer_extend(daliTimer, 2);
}

/** 
 * Called in the middle of each output but.  As we are using manchester, it simply toggles the output, then works out what the next bit will be
 */
static void halfBitToggle() {
    DALI_PORT.OUTTGL = DALI_TX_bm;
    // set the handler to What happens at the start of the next bit.
    if (--bitsLeft == 0) {
        daliTimer.callback = startStopBits;
    } else {
        daliTimer.callback = shiftReg & 0x8000 ? sendOne: sendZero;
        shiftReg <<= 1;
    }
    timer_extend(daliTimer, 1);
}

extern void dali_transmitting_state_enter(dali_command_t *cmd) {
    // Before we start messing with stuff, make sure the timer is off
    shiftReg = cmd->cmd;
    bitsLeft = 17; // One extra for the stop bit. 

    // We send a one as the start bit.
    cli();
    daliTimer.callback = sendOne;
    daliTimer.next = timerCurrentTick + 1;
    daliTimer.active = true;
    sei();
}

