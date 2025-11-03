#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "dali.h"
#include "timers.h"

// The DALi Specification says that the bus must be idle for 22 Te (half byte periods) between commands. 
#define IDLE_THRESHOLD_TE 22

static void onBusShorted();


/**
 * @brief Called when the timeout occurs (only enabled when line is released).
 * This is good. It means that the line has been idle for enough time.
 */
static void idlePeriodExprired() {
    dali_cancel_edge_detect();
    dali_idle_state_enter();
}

static void onIdleBus() {
    // cli();
    dali_call_on_rising_once(onBusShorted);
    daliTimerCallback = idlePeriodExprired;
    daliTimer = IDLE_THRESHOLD_TE;
    // sei();
}

static void onBusShorted() {

    // cli();
    daliTimer = 0;
    // Wait until we receive a low level (not shorted), then start checking again.
    dali_call_on_low_once(onIdleBus);
    // sei();
}

extern void dali_wait_for_idle_state_enter() {
    // Perform an initial inspection of the bus state.  If its shorted, we need to wait for it to go lower.
    if (dali_is_bus_shorted()) {
        onBusShorted();
    } else {
        onIdleBus();
    }
}