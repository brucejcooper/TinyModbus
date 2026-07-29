#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/atomic.h>

#include "dali.h"
#include "dali_sync.h"

volatile bool dali_bus_busy = true;

// ---- Internal helpers ---------------------------------------------------

// Rising edge while parked: SOMETHING asserted the bus and it was not us. Yield
// the wire, wait for it to settle, then re-enter to dispatch queued commands.
//
// Do not read this as multi-master-only code. The monitor cannot tell what
// pulled the line, and on a single-master bus the same edge is produced by a
// ballast answering late (a backward frame arriving after our response window
// closed), by noise or reflections on a long run, and by the bus being held
// asserted by a short or a sagging supply — the last of which is what the 200 TE
// escape in dali_state_wait_for_idle.c exists to recover from. A second master
// is one cause among several, and the least likely on this installation.
//
// This is the PARKED -> YIELDED transition, and the reason "the bus is busy" and
// "we are executing" are different questions: here the bus is very much busy and
// we own nothing.
static void external_message_started(void) {
    dali_bus_busy = true;
    dali_cancel_edge_detect();
    dali_wait_for_idle_state_enter();
}

// ---- dali_result_received - called from ISR context --------------------
//
// All DALI completions flow through dali_sync_completion.  This function
// is the entry point from the bit-level state-machine callers
// (dali_state_transmitting.c, dali_state_receiving.c,
// dali_state_wait_for_response.c).
//
// Two intercepts before dispatch:
//
//   1. dali_sync_repeat_send(): a send-twice command's second send. The
//      issuance layer decides — it owns the command and the step — and
//      re-fires immediately with no inter-frame gap here, because the 22 TE
//      inter-send window is provided by wait_for_response between the two
//      sends. This file no longer keeps a repeat counter of its own.
//
//   2. otherwise: dispatch to dali_sync_completion which advances the
//      logical step or releases the channel.

void dali_result_received(dali_response_type_t type, uint8_t value) {
    if (dali_sync_owns_channel()) {
        if (dali_sync_repeat_send()) return;
        dali_sync_completion(type, value);
        return;
    }
    dali_idle_state_enter();
}

// ---- dali_idle_state_enter ---------------------------------------------
//
// Two branches:
//   1. The queue still owns a command: we just finished the
//      inter-frame gap of a multi-step sequence. Dispatch its next step via
//      dali_sync_dispatch_current — and do NOT go PARKED, because the
//      wire is still ours.
//   2. Otherwise: arm the bus monitor and park, so main() may kick
//      start_next_dali_work.

void dali_idle_state_enter(void) {
    // Belt-and-braces: ensure the DALI TX line is released before entering idle.
    DALI_PORT.OUTCLR = DALI_TX_bm;

    if (dali_sync_owns_channel()) {
        // The queue still owns this command — we are between frames of a
        // multi-step sequence, not finished. Do not park; the wire stays ours.
        dali_cancel_edge_detect();
        if (dali_is_bus_active()) {
            dali_wait_for_idle_state_enter();
        } else {
            dali_sync_dispatch_current();
        }
        return;
    }

    dali_bus_busy = false;
    dali_state_receiving_prepare();
    dali_call_on_rising_once(external_message_started);
}
