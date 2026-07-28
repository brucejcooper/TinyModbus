#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>

#include "dali.h"
#include "dali_sync.h"
#include "tick.h"

#define IDLE_THRESHOLD_TE   22u   // minimum bus-idle period before next TX
#define BUS_STUCK_TIMEOUT_TE 200u  // escape hatch (~83 ms) if bus never goes idle

static void on_bus_shorted(void);

// The 22 TE quiet period has elapsed; the bus is clean - dispatch the next command.
static void idle_period_expired(void) {
    dali_cancel_edge_detect();
    dali_idle_state_enter();
}

// Bus just went low (idle): start the 22 TE countdown and watch for collisions.
static void on_idle_bus(void) {
    dali_call_on_rising_once(on_bus_shorted);
    dali_timer_cb = idle_period_expired;
    dali_timer    = IDLE_THRESHOLD_TE;
}

// Safety escape: bus held high for 200 TE without clearing.
// Force TX low (release any stuck open-drain drive) and recover.
//
// Without clearing dali_busy here, dali_idle_state_enter takes its
// "busy" branch and re-enters wait_for_idle on the still-shorted bus —
// indefinite ~83 ms/cycle loop, slave never sleeps.  Clear dali_busy
// so dali_idle_state_enter takes the idle-arm branch instead, and
// (for an in-flight commissioning run) force phase=COM_IDLE with a
// distinct ERR_BUS_STUCK status so pick_work doesn't immediately
// re-pick OP_COMMISSION on the next main pass and re-issue commands
// onto the still-shorted bus.  Non-commissioning ops are left to be
// re-picked by main; if the bus has cleared by then they retry, and
// if it hasn't they'll re-enter wait_for_idle (one cycle of slow
// retry rather than a tight loop).
static void bus_stuck_timeout(void) {
    DALI_PORT.OUTCLR = DALI_TX_bm;
    dali_cancel_edge_detect();
    // Fail the in-flight queue command with a bus-fault code. TASK 05: this
    // used to abort a gateway-side commissioning run; commissioning now lives on
    // the controller, and what must not happen is leaving a slot EXECUTING
    // forever — the controller would wait on a command that can never finish.
    dali_sync_abort_busfault();
    dali_idle_state_enter();
}

// Bus went high - another master is active, or a collision occurred.
// Wait for it to release the bus (go low), then begin the idle countdown.
static void on_bus_shorted(void) {
    dali_call_on_low_once(on_idle_bus);
    dali_timer_cb = bus_stuck_timeout;
    dali_timer    = BUS_STUCK_TIMEOUT_TE;
}

void dali_wait_for_idle_state_enter(void) {
    if (dali_is_bus_active())
        on_bus_shorted();
    else
        on_idle_bus();
}
