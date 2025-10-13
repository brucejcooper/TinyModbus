#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdlib.h>

#include "dali.h"
#include "timers.h"

// The DALi Specification says that the bus must be idle for 22 Te (half byte periods) between commands. 
#define IDLE_THRESHOLD USEC_TO_TICKS(22 * DALI_HALF_BIT_USECS)


/**
 * @brief Called when the timeout occurs (only enabled when line is released).
 * This is good. It means that the line has been idle for enough time.
 */
static void onTimeout() {
    dali_call_on_rising_once(NULL);
    cancel_timeout();
    dali_idle_state_enter();
}

static void startReceivingResponse() {
    if (dali_is_bus_shorted()) {
        // Disable the timer. We will test again next time we get a toggle ISR.
        cancel_timeout();

        // Wait until we receive a low level (not shorted), then start checking again.
        dali_call_on_low_once(startReceivingResponse);
    } else {
        // Set up a timer to go off if we don't receive a positive edge within the threshold period.
        dali_call_on_rising_once(NULL);
        restart_timeout();
    }
}

extern void dali_wait_for_idle_state_enter() {
    dali_call_on_rising_once(startReceivingResponse);
    on_timeout(IDLE_THRESHOLD, onTimeout);
    // Perform an initial inspection of the bus state.  If its shorted, we need to wait for it to go lower.
    startReceivingResponse();
}