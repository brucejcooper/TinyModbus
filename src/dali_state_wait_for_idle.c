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
    // When entered from PORT_vect (LVL0) the previous state's daliTimer is still mid-countdown.
    // Zero it BEFORE the callback rewrites — we have two non-atomic 16-bit pointer writes coming
    // (dali_rx_callback inside dali_call_on_rising_once, then daliTimerCallback below). If the
    // old daliTimerCallback fires from TCA0 LVL1 mid-write, both pointers can end up torn.
    // Suppressing the dispatch up front closes both windows.
    daliTimer = 0;
    dali_call_on_rising_once(onBusShorted);
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
    // Suppress the previous state's daliTimerCallback dispatch up front so neither of the two
    // non-atomic 16-bit pointer writes that follow (dali_rx_callback inside dali_call_on_low_once,
    // then daliTimerCallback) can be torn by TCA0 LVL1 firing the old callback mid-write.
    daliTimer = 0;
    // Wait for the bus to return to idle (RX pin goes low).
    dali_call_on_low_once(onIdleBus);
    // Safety net: if the bus never recovers, escape after BUS_STUCK_TIMEOUT_TE.
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