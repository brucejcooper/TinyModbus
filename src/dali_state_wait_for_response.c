
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <stdbool.h>
#include <stdlib.h>
#include "dali.h"
#include "timers.h"

// TODO What is the correct value here?
#define NORESPONSE_TIMEOUT USEC_TO_TICKS(10500)

static inline void cleanup() {
    cancel_timeout();
}


/**
 * @brief If we got no response within period, we go back to idle. 
 */
static void timeoutWaitingForResponse() {
    // Turn everything off.
    cleanup();
    dali_call_on_rising_once(NULL);
    send_command_response(DALI_RESPONSE_NAK, 0);
    dali_idle_state_enter();

}

static void startReceivingResponse() {
    // Normally, we would call cancel_timeout here, but we want the TCA timer to start ASAP after the event.
    // Instead, we trust that dali_state_receiving_enter() will fix the timer itself. 
    cancel_timeout();
    dali_state_receiving_enter();
}


extern void dali_wait_for_response_state_enter() {
    // log_info("wfr");
    // For this, we will be reading the line, so we enable the rising line interrupt, and make it interrupt on any change.
    dali_state_receiving_prepare();
    dali_call_on_rising_once(startReceivingResponse);
    on_timeout(NORESPONSE_TIMEOUT, timeoutWaitingForResponse);
    start_timeout();
}