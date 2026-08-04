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
// This is the path that releases a command without the wire being ours again:
// the slot is failed and the handle cleared while recovery continues.
//
// Without releasing the command here, dali_idle_state_enter takes its
// "busy" branch and re-enters wait_for_idle on the still-shorted bus —
// indefinite ~83 ms/cycle loop, slave never sleeps.  Release it so
// dali_idle_state_enter takes the idle-arm branch instead, and complete the
// in-flight slot with a bus-fault code (see dali_sync_abort_busfault) so the
// master is not left waiting on a command that can never finish.
static void bus_stuck_timeout(void) {
    DALI_PORT.OUTCLR = DALI_TX_bm;
    dali_cancel_edge_detect();
    // Fail the in-flight queue command with a bus-fault code: what must not
    // never happen is leaving a slot EXECUTING forever — the controller would
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
