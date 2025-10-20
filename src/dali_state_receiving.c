#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "dali.h"
#include "timers.h"

#define DALI_READ_TOLERANCE_USECS USEC_TO_TICKS(20)
#define HALFBIT_MIN_TICKS USEC_TO_TICKS(DALI_HALF_BIT_USECS - DALI_READ_TOLERANCE_USECS)
#define HALFBIT_MAX_TICKS USEC_TO_TICKS(DALI_HALF_BIT_USECS + DALI_READ_TOLERANCE_USECS)
#define FULLBIT_MIN_TICKS USEC_TO_TICKS(DALI_BIT_USECS - DALI_READ_TOLERANCE_USECS)
#define FULLBIT_MAX_TICKS USEC_TO_TICKS(DALI_BIT_USECS + DALI_READ_TOLERANCE_USECS)

// Good documentation for Dali commands - https://www.nxp.com/files-static/microcontrollers/doc/ref_manual/DRM004.pdf

static volatile uint32_t shiftreg = 0;
static volatile uint8_t numBits = 0;

static volatile bool inStartBit = true;
static volatile bool atHalfBit;
static volatile uint8_t lastBit;

typedef void (*pulse_received_cb_t)(bool longpulse);
static volatile pulse_received_cb_t handler;

#define STATE_STARTBIT 0
#define STATE_START_0 1
#define STATE_START_1 2
#define STATE_MIDDLE_0 3
#define STATE_MIDDLE_1 4
#define STATE_ERROR 5

#define NOSHIFT 3
#define SHIFT_0 0
#define SHIFT_1 1

typedef struct {
    uint8_t shift;
    uint8_t next;
} reader_action_t;

typedef struct {
    reader_action_t shortpulse;
    reader_action_t longpulse;
} reader_state_t;

static const reader_state_t state_machine[STATE_ERROR] = {
    // START_BIT
    {.shortpulse = {.shift = NOSHIFT, .next = STATE_MIDDLE_1}, .longpulse = {.shift = NOSHIFT, .next = STATE_ERROR}},
    // START_0 - or it might be the end of sequence
    {.shortpulse = {.shift = SHIFT_0, .next = STATE_MIDDLE_0}, .longpulse = {.shift = NOSHIFT, .next = STATE_ERROR}},
    // START_1
    {.shortpulse = {.shift = SHIFT_1, .next = STATE_MIDDLE_1}, .longpulse = {.shift = NOSHIFT, .next = STATE_ERROR}},
    // MIDDLE_0
    {.shortpulse = {.shift = NOSHIFT, .next = STATE_START_0}, .longpulse = {.shift = SHIFT_1, .next = STATE_MIDDLE_1}},
    // MIDDLE_1 - but it might be the end of the sequence (after the last half)
    {.shortpulse = {.shift = NOSHIFT, .next = STATE_START_1}, .longpulse = {.shift = SHIFT_0, .next = STATE_MIDDLE_0}},
};
uint8_t state;

static inline void cleanup() {
    TCB0.CTRLA = 0;
    daliTimer.active = false;
    TCB0.EVCTRL = 0;   // Disable input capture event.
    TCB0.INTCTRL = 0;  // Turn off CAPT ISR.
}

void timerOverflow() {
    // This indicates that a timeout has occurred without a bit flip. This can either be stop bits or a timing error/
    // In either case, we will be leaving this state.  Shut down the timers.
    cleanup();

    if ((state == STATE_START_0 || state == STATE_MIDDLE_1) && numBits == 8) {
        // We have come to a valid end of sequence.  Process what we have read.
        send_command_response(DALI_RESPONSE_OK, shiftreg);
    } else {
        // Some timing error or collision occurred.
        send_command_response(numBits == 0 ? DALI_RESPONSE_NAK : DALI_RESPONSE_MALFORMED, 0);
    }
    dali_wait_for_idle_state_enter();

}


/**
 * We start the receiving state as a result of an interrupt.  To avoid wasting time executing setup task,
 * we pre-start them here.
 */
void dali_state_receiving_prepare() {
    state = STATE_STARTBIT;
    shiftreg = 0;
    numBits = 0;

    TCB0.CTRLA = 0;
    // We'll be using TCB0 to read pulse widths.
    TCB0.CNT = 73;                              // Account for startup latencies
    TCB0.EVCTRL = TCB_CAPTEI_bm | TCB_EDGE_bm;  // First ISR will happen on the falling edge at the middle of the start bit.
    TCB0.INTCTRL = TCB_CAPT_bm;                 // Turn CAPT ISR.
}

/**
 * @brief Start receiving a dali waveform.
 * This method is called after receiving the rising edge of a start bit, so we expect the bus to be
 * high (shorted) at this point
 *
 * We also expect dali_state_receiving_prepare to have been called before this one.
 */
void dali_state_receiving_enter() {
    // Start TCB0 for pulse width timing.  It will start counting upon a rising edge (start of start bit.)
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
    TCB0.INTFLAGS = TCB_CAPT_bm;  // Clear any pending ISR

    // We want to start TCA0 as soon as possible, to mirror the pulse widths being measured by TCB0.
    // However, in the wait_for_respone state, the timer would have been taken over with its own timeout.
    // As a result, we must take over the timer again.
    daliTimer.callback = timerOverflow;
    daliTimer.next = timerCurrentTick + 4;
    daliTimer.active = true;
}

ISR(TCB0_INT_vect) {
    // TCB0's CNT has been reset automatically, but TCA0's hasn't
    // We do this first so that it closely matches TCB0s as it can
    uint16_t pulseLengthInTicks = TCB0.CCMP;  // This will clear the interrupt flag.
    uint8_t evctrl = TCB0.EVCTRL;
    // Due to capacitance / collector behaviour, positive pulses are recorded shorter than they actually are.
    // We adjust the pulse lengths to be more representative of what the source device wished.
    if (evctrl & TCB_EDGE_bm) {
        // It was a falling edge (from high)
        pulseLengthInTicks += USEC_TO_TICKS(54);
    } else {
        // It was a rising edge (from low)
        pulseLengthInTicks -= USEC_TO_TICKS(52);
    }
    TCB0.EVCTRL = evctrl ^ TCB_EDGE_bm;  // Toggle the EDGE value to make it so that we capture the other edge

    // What action we take is dependent on the pulse length
    const reader_action_t *action;
    // Now process our pulse.
    if (pulseLengthInTicks < HALFBIT_MIN_TICKS) {
        // Pulse is too short
        goto error;
    } else if (pulseLengthInTicks <= HALFBIT_MAX_TICKS) {
        action = &state_machine[state].shortpulse;
    } else if (pulseLengthInTicks <= FULLBIT_MIN_TICKS) {
        // pulse width is too long for a half bit, and too short for a full bit
        goto error;
    } else if (pulseLengthInTicks <= FULLBIT_MAX_TICKS) {
        action = &state_machine[state].longpulse;
    } else {
        // pulse is too long. 
        goto error;
    }

    state = action->next;
    if (state == STATE_ERROR) {
        goto error;
    }
    if (action->shift != NOSHIFT) {
        shiftreg = shiftreg << 1 | action->shift;
        numBits++;
    }
    // We extend the timeout to be in 4 ticks time.  
    daliTimer.next = timerCurrentTick + 4;
    return;  // Avoid the error handler.
error:
    // We received an invalid pulse width.
    cleanup();
    send_command_response(DALI_RESPONSE_MALFORMED, 0);
    dali_wait_for_idle_state_enter();
}
