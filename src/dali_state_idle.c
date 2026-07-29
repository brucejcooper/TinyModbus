#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/atomic.h>

#include "dali.h"
#include "dali_sync.h"

// ---- Internal helpers ---------------------------------------------------

// Rising edge while parked: another master has started a frame. Yield the wire,
// wait for it to settle, then re-enter to dispatch queued commands.
//
// This is the PARKED -> YIELDED transition, and the reason "the bus is busy" and
// "we are executing" are different questions: here the bus is very much busy and
// we own nothing.
static void external_message_started(void) {
    dali_chan = DALI_CHAN_YIELDED;
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
    if (dali_chan == DALI_CHAN_EXECUTING) {
        if (dali_sync_repeat_send()) return;
        dali_sync_completion(type, value);
        return;
    }
    dali_idle_state_enter();
}

// ---- dali_idle_state_enter ---------------------------------------------
//
// Two branches:
//   1. Still EXECUTING: we just finished the
//      inter-frame gap of a multi-step sequence. Dispatch its next step via
//      dali_sync_dispatch_current — and do NOT go PARKED, because the
//      wire is still ours.
//   2. Otherwise: arm the bus monitor and go PARKED, so main() may kick
//      start_next_dali_work.

void dali_idle_state_enter(void) {
    // Belt-and-braces: ensure the DALI TX line is released before entering idle.
    DALI_PORT.OUTCLR = DALI_TX_bm;

    if (dali_chan == DALI_CHAN_EXECUTING) {
        // Stay EXECUTING: the wire is still ours mid-sequence, so only the
        // rising-edge monitor needs disarming. (The old cleanup() also cleared
        // the idle flag here, which was already false by the invariant.)
        dali_cancel_edge_detect();
        if (dali_is_bus_active()) {
            dali_wait_for_idle_state_enter();
        } else {
            dali_sync_dispatch_current();
        }
        return;
    }

    dali_chan = DALI_CHAN_PARKED;
    dali_state_receiving_prepare();
    dali_call_on_rising_once(external_message_started);
}
