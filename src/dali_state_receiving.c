#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "dali.h"
#include "timers.h"

// 20% tolerance on bit timings.
#define DALI_READ_TOLERANCE_USECS (DALI_HALF_BIT_USECS/5)
#define HALFBIT_MIN_TICKS USEC_TO_TICKS(DALI_HALF_BIT_USECS - DALI_READ_TOLERANCE_USECS)
#define HALFBIT_MAX_TICKS USEC_TO_TICKS(DALI_HALF_BIT_USECS + DALI_READ_TOLERANCE_USECS)
#define FULLBIT_MIN_TICKS USEC_TO_TICKS(DALI_BIT_USECS - DALI_READ_TOLERANCE_USECS)
#define FULLBIT_MAX_TICKS USEC_TO_TICKS(DALI_BIT_USECS + DALI_READ_TOLERANCE_USECS)

// Good documentation for Dali commands - https://www.nxp.com/files-static/microcontrollers/doc/ref_manual/DRM004.pdf

static volatile uint8_t shiftreg_in = 0;
static volatile uint8_t numBits = 0;

#define STATE_STARTBIT 0
#define STATE_START_0 1
#define STATE_START_1 2
#define STATE_MIDDLE_0 3
#define STATE_MIDDLE_1 4
#define STATE_ERROR 5

#define STATE_BP 4

#define SHIFT_0 0
#define SHIFT_1 1
#define NOSHIFT 2


typedef struct {
    uint8_t shortpulse;
    uint8_t longpulse;
} reader_state_t;

static const reader_state_t state_machine[STATE_ERROR+1] = {
    // START_BIT
    {.shortpulse = NOSHIFT | (STATE_MIDDLE_1 << STATE_BP), .longpulse = STATE_ERROR << STATE_BP},
    // START_0 - or it might be the end of sequence
    {.shortpulse = 0 | (STATE_MIDDLE_0 << STATE_BP), .longpulse = (STATE_ERROR << STATE_BP)},
    // START_1
    {.shortpulse = 1 | (STATE_MIDDLE_1 << STATE_BP), .longpulse = (STATE_ERROR << STATE_BP)},
    // MIDDLE_0
    {.shortpulse = NOSHIFT | (STATE_START_0 << STATE_BP), .longpulse = 1 | (STATE_MIDDLE_1 << STATE_BP)},
    // MIDDLE_1- but it might be the end of the sequence (after the last half)
    {.shortpulse = NOSHIFT | (STATE_START_1 << STATE_BP), .longpulse = 0 | (STATE_MIDDLE_0 << STATE_BP)},

    // this isn't strictly necessary, but we're trying to fix a pernicious bug
    {.shortpulse = STATE_ERROR << STATE_BP, .longpulse = STATE_ERROR << STATE_BP}
};

static volatile uint8_t state;

static inline void cleanup() {
    TCB0.CTRLA = 0;
    daliTimer = 0;
    TCB0.EVCTRL = 0;   // Disable input capture event.
    TCB0.INTCTRL = 0;  // Turn off CAPT ISR.
}

static void timerOverflow() {
    // blip(2);

    // This indicates that a timeout has occurred without a bit flip. This can either be stop bits or a timing error/
    // In either case, we will be leaving this state.  Shut down the timers.
    cleanup();

    if ((state == STATE_START_0 || state == STATE_MIDDLE_1) && numBits == 8) {
        // We have come to a valid end of sequence.  Process what we have read.
        process_command_response(DALI_RESPONSE_OK, shiftreg_in);
    } else {
        // Some timing error or collision occurred.
        process_command_response(DALI_RESPONSE_MALFORMED, 0);
    }
    dali_wait_for_idle_state_enter();

}


/**
 * We start the receiving state as a result of an interrupt.  To avoid wasting time executing setup task,
 * we pre-start them here.
 */
void dali_state_receiving_prepare() {
    state = STATE_STARTBIT;
    shiftreg_in = 0;
    numBits = 0;

    // Timer is not running yet
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
    // blip(1);


    // Start TCB0 for pulse width timing.  It will start counting upon a rising edge (start of start bit.)
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
    TCB0.INTFLAGS = TCB_CAPT_bm;  // Clear any pending ISR

    // We want to start TCA0 as soon as possible, to mirror the pulse widths being measured by TCB0.
    // However, in the wait_for_respone state, the timer would have been taken over with its own timeout.
    // As a result, we must take over the timer again.
    //
    // We're called from PORTB_PORT_vect (LVL0) while daliTimer is still mid-countdown from
    // dali_wait_for_response_state_enter. daliTimerCallback is a 16-bit pointer and is written
    // with two non-atomic STS instructions; if TCA0 LVL1 fires between them and daliTimer hits
    // zero in the same cycle, the dispatched callback would be a torn pointer. Zeroing daliTimer
    // first suppresses any in-flight dispatch while we swap the pointer.
    daliTimer = 0;
    daliTimerCallback = timerOverflow;
    daliTimer = 5; // Anything between 2 and 2.5 bits
}

static inline void sendError(dali_response_type_t err) {
    cleanup();
    process_command_response(err, 0);
    dali_wait_for_idle_state_enter();
}

ISR(TCB0_INT_vect) {
    uint8_t action;

    // We extend the timeout to be in 4 half bits time.  
    daliTimer = 5;
    // blip(state);

    uint16_t pulseLengthInTicks = TCB0.CCMP;  // This will clear the interrupt flag.
    uint8_t evctrl = TCB0.EVCTRL;
    // Due to capacitance / collector behaviour, positive pulses are recorded shorter than they actually are.
    // We adjust the pulse lengths to be more representative of what the source device wished.
    // if (evctrl & TCB_EDGE_bm) {
    //     // It was a falling edge (from high)
    //     pulseLengthInTicks += USEC_TO_TICKS(54);
    // } else {
    //     // It was a rising edge (from low)
    //     pulseLengthInTicks -= USEC_TO_TICKS(52);
    // }
    TCB0.EVCTRL = evctrl ^ TCB_EDGE_bm;  // Toggle the EDGE value to make it so that we capture the other edge

    // What action we take is dependent on the pulse length
    // Now process our pulse.
    // blip(pulseLengthInTicks /100);
    if (pulseLengthInTicks < HALFBIT_MIN_TICKS) {
        // Pulse is too short
        sendError(DALI_RESPONSE_PULSE_TOO_SHORT);
        return;
    } else if (pulseLengthInTicks <= HALFBIT_MAX_TICKS) {
        action = state_machine[state].shortpulse;
    } else if (pulseLengthInTicks <= FULLBIT_MIN_TICKS) {
        // pulse width is too long for a half bit, and too short for a full bit
        sendError(DALI_RESPONSE_PULSE_IN_MIDDLE);
        return;
    } else if (pulseLengthInTicks <= FULLBIT_MAX_TICKS) {
        action = state_machine[state].longpulse;
    } else {
        // pulse is too long. 
        sendError(DALI_RESPONSE_PULSE_TOO_LONG);
        return;
    }
    // blip(action);


    // Change to next state.
    state = action >> STATE_BP;
    if (state == STATE_ERROR) {
        sendError(DALI_RESPONSE_MALFORMED);
        return;
    }
    // Take the action (from the old state.
    uint8_t bitSet = action & 0x03;
    if (bitSet != NOSHIFT) {
        shiftreg_in = (shiftreg_in << 1) | bitSet;
        numBits++;
    }
}
