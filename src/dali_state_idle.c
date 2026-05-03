#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/atomic.h>

#include "dali.h"
#include "registers.h"
#include "dali_sync.h"

volatile bool dali_idle = false;

// ---- Internal helpers ---------------------------------------------------

// Clear dali_idle and disarm bus-monitor edge detection before transmitting.
static void cleanup(void) {
    dali_idle = false;
    dali_cancel_edge_detect();
}

// Rising edge detected while idle: another master has started a frame.
// Yield, wait for the bus to settle, then re-enter to dispatch queued commands.
static void external_message_started(void) {
    cleanup();
    dali_wait_for_idle_state_enter();
}

// ---- cmdq_result_received - called from ISR context --------------------
//
// All DALI completions in subtask 4+ flow through dali_sync_completion.
// The function name is preserved for compatibility with the bit-level
// state-machine callers (dali_state_transmitting.c,
// dali_state_receiving.c, dali_state_wait_for_response.c); subtask 5
// renames it.
//
// Two intercepts before dispatch:
//
//   1. s_active.repeat > 1: this is a SEND_TWICE (raw twice, group bit,
//      SET commands within multi-step writes).  Decrement the repeat
//      counter and re-fire dali_transmitting_state_enter immediately —
//      no inter-frame gap, matching the legacy queue mechanism.  This
//      keeps the bit-level state machine semantics identical to the
//      pre-refactor code (the 22 TE inter-send window is provided by
//      wait_for_response between the two sends).
//
//   2. otherwise: dispatch to dali_sync_completion which advances the
//      logical step or releases the channel.

void cmdq_result_received(dali_response_type_t type, uint8_t value) {
    if (dali_busy) {
        if (s_active.repeat > 1u) {
            s_active.repeat--;
            dali_transmitting_state_enter((const cmd_entry_t *)&s_active);
            return;
        }
        dali_sync_completion(type, value);
        return;
    }
    dali_idle_state_enter();
}

// ---- dali_idle_state_enter ---------------------------------------------
//
// Two branches:
//   1. dali_busy is true: a sync op is in flight and we just finished
//      the inter-frame gap.  Dispatch the next step via
//      dali_sync_dispatch_current (which calls dali_transmitting_state_enter).
//   2. Otherwise: arm the bus monitor and set dali_idle so main can
//      kick start_next_dali_work.

void dali_idle_state_enter(void) {
    // Belt-and-braces: ensure the DALI TX line is released before entering idle.
    DALI_PORT.OUTCLR = DALI_TX_bm;

    if (dali_busy) {
        cleanup();
        if (dali_is_bus_active()) {
            dali_wait_for_idle_state_enter();
        } else {
            dali_sync_dispatch_current();
        }
        return;
    }

    dali_idle = true;
    dali_state_receiving_prepare();
    dali_call_on_rising_once(external_message_started);
}
