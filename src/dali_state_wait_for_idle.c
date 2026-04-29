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

// If the bus stays active longer than this, something is wrong (ballast fault, wiring short, etc.).
// Force the state machine out of WAIT_FOR_IDLE rather than hanging forever.
// 200 Te × 416.67 µs ≈ 83 ms — well beyond any legitimate DALI frame.
#define BUS_STUCK_TIMEOUT_TE 200

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
    dali_call_on_rising_once(onBusShorted);
    // When entered from PORT_vect (LVL0) the previous state's daliTimer is still mid-countdown.
    // Zero it before swapping the (16-bit, non-atomic) callback pointer so TCA0 LVL1 can't fire
    // a torn pointer if the timer happens to expire between the two STS instructions.
    daliTimer = 0;
    daliTimerCallback = idlePeriodExprired;
    daliTimer = IDLE_THRESHOLD_TE;
}

/**
 * @brief Bus has been stuck active (low) for too long — escape the wait.
 * Releases our TX pin defensively, cancels edge detection, and returns to the
 * idle state. The state machine will attempt to transmit or wait for a new
 * command. If the external fault persists it will re-enter this path and keep
 * retrying rather than hanging indefinitely.
 */
static void busStuckTimeout() {
    DALI_PORT.OUTCLR = DALI_TX_bm;
    dali_cancel_edge_detect();
    dali_idle_state_enter();
}

static void onBusShorted() {
    // Wait for the bus to return to idle (RX pin goes low).
    dali_call_on_low_once(onIdleBus);
    // Safety net: if the bus never recovers, escape after BUS_STUCK_TIMEOUT_TE.
    // Zero daliTimer first so TCA0 LVL1 can't fire a torn callback while the (16-bit,
    // non-atomic) pointer write is in flight — see the equivalent comment in onIdleBus.
    daliTimer = 0;
    daliTimerCallback = busStuckTimeout;
    daliTimer = BUS_STUCK_TIMEOUT_TE;
}

extern void dali_wait_for_idle_state_enter() {
    // Perform an initial inspection of the bus state.  If its shorted, we need to wait for it to go lower.
    if (dali_is_bus_shorted()) {
        onBusShorted();
    } else {
        onIdleBus();
    }
}