
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <stdbool.h>
#include "dali.h"
#include "timers.h"

// TODO What is the correct value here?
#define NORESPONSE_TIMEOUT_TICKS 22

static inline void cleanup() {
    daliTimer.active = false;
}


/**
 * @brief If we got no response within period, we go back to idle. 
 */
static void timeoutWaitingForResponse() {
    // Turn everything off.
    cleanup();
    dali_cancel_edge_detect();
    send_command_response(DALI_RESPONSE_NAK, 0);
    dali_idle_state_enter();
}

static void startReceivingResponse() {
    // Normally, we would call cancel_timeout here, but we want the TCA timer to start ASAP after the event.
    // Instead, we trust that dali_state_receiving_enter() will fix the timer itself. 
    dali_state_receiving_enter();
}


extern void dali_wait_for_response_state_enter() {
    // For this, we will be reading the line, so we enable the rising line interrupt, and make it interrupt on any change.
    dali_state_receiving_prepare();
    dali_call_on_rising_once(startReceivingResponse);
    daliTimer.callback = timeoutWaitingForResponse;
    timer_extend(daliTimer, NORESPONSE_TIMEOUT_TICKS);
}