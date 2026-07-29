#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/atomic.h>

#include "dali.h"
#include "dali_sync.h"

volatile bool dali_line_idle = false;

// ---- Internal helpers ---------------------------------------------------

// Leave the parked state: we are about to drive the wire (or yield it to another
// master), so drop dali_line_idle and disarm the rising-edge monitor. Says
// nothing about command ownership — that is dali_cmd_executing's job.
static void cleanup(void) {
    dali_line_idle = false;
    dali_cancel_edge_detect();
}

// Rising edge detected while idle: another master has started a frame.
// Yield, wait for the bus to settle, then re-enter to dispatch queued commands.
static void external_message_started(void) {
    cleanup();
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
    if (dali_cmd_executing) {
        if (dali_sync_repeat_send()) return;
        dali_sync_completion(type, value);
        return;
    }
    dali_idle_state_enter();
}

// ---- dali_idle_state_enter ---------------------------------------------
//
// Two branches:
//   1. A command is still claimed (dali_cmd_executing): we just finished the
//      inter-frame gap of a multi-step sequence. Dispatch its next step via
//      dali_sync_dispatch_current — and do NOT set dali_line_idle, because the
//      wire is still ours. This branch is why the two flags are not inverses.
//   2. Otherwise: arm the bus monitor and park, setting dali_line_idle so
//      main() may kick start_next_dali_work.

void dali_idle_state_enter(void) {
    // Belt-and-braces: ensure the DALI TX line is released before entering idle.
    DALI_PORT.OUTCLR = DALI_TX_bm;

    if (dali_cmd_executing) {
        cleanup();
        if (dali_is_bus_active()) {
            dali_wait_for_idle_state_enter();
        } else {
            dali_sync_dispatch_current();
        }
        return;
    }

    dali_line_idle = true;
    dali_state_receiving_prepare();
    dali_call_on_rising_once(external_message_started);
}
