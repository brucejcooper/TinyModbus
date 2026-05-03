#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "dali.h"
#include "tick.h"

#define NORESPONSE_TIMEOUT_TICKS 22u

static void cleanup(void) {
    dali_timer = 0;
}

static void timeout_waiting_for_response(void) {
    // Belt-and-braces: release the DALI TX line before yielding back to idle,
    // in case any prior path left it asserted.
    DALI_PORT.OUTCLR = DALI_TX_bm;
    cleanup();
    dali_cancel_edge_detect();
    cmdq_result_received(DALI_RESPONSE_NAK, 0);
}

static void start_receiving_response(void) {
    dali_state_receiving_enter();
}

void dali_wait_for_response_state_enter(void) {
    dali_state_receiving_prepare();
    dali_call_on_rising_once(start_receiving_response);
    dali_timer_cb = timeout_waiting_for_response;
    dali_timer    = NORESPONSE_TIMEOUT_TICKS;
}
