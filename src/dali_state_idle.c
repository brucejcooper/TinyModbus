#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/atomic.h>

#include "dali.h"
#include "registers.h"
#include "cmd_queue.h"

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

// ---- cmdq_result_received — called from ISR context --------------------

// Processes the result of the head-of-queue command:
//   - updates the target register (if any)
//   - fires the optional callback
//   - for CMD_SEND_TWICE: decrements repeat and re-dispatches without popping
//   - otherwise: pops the queue and enters idle to dispatch the next command

void cmdq_result_received(dali_response_type_t type, uint8_t value) {
    if (cmdq_empty()) {
        dali_idle_state_enter();
        return;
    }

    cmd_entry_t cmd;
    cmdq_peek(&cmd);

    // Handle repeat: CMD_SEND_TWICE sends twice before processing the result
    if (cmd.repeat > 1) {
        // Write back the decremented repeat counter into the queue head
        g_cmd_queue.entries[g_cmd_queue.head].repeat--;
        // Re-dispatch the same command (second send)
        dali_transmitting_state_enter(&g_cmd_queue.entries[g_cmd_queue.head]);
        return;
    }

    // Apply result to register map
    if (cmd.reg_target != REG_NONE && type == DALI_RESPONSE_OK)
        regs_apply_result(cmd.ballast_idx, cmd.reg_target, value);

    // Fire optional callback (used by scan and fade tracking)
    if (cmd.on_result)
        cmd.on_result(type, cmd.ballast_idx, value);

    cmdq_pop();
    // After any completed command, wait for the bus to settle (22 TE) before
    // dispatching the next one.  When the queue is now empty, enter idle
    // immediately so we can listen for external traffic without the extra wait.
    if (cmdq_empty())
        dali_idle_state_enter();
    else
        dali_wait_for_idle_state_enter();
}

// ---- dali_idle_state_enter ---------------------------------------------

void dali_idle_state_enter(void) {
    if (!cmdq_empty()) {
        cleanup();
        if (dali_is_bus_active()) {
            // Bus became active between the idle check and now — wait for it to clear.
            dali_wait_for_idle_state_enter();
        } else {
            cmd_entry_t cmd;
            cmdq_peek(&cmd);
            dali_transmitting_state_enter(&cmd);
        }
    } else {
        dali_idle = true;
        dali_state_receiving_prepare();
        dali_call_on_rising_once(external_message_started);
    }
}
