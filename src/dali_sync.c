/*
 * dali_sync.c — DALI issuance for the command queue. See dali_sync.h.
 *
 * TASK 05 — STATELESS PIPE. What used to be ~1100 lines of gateway brain (work
 * selector, ballast cache, presence/dirty bitmaps, group masks, rescan cursor,
 * commissioning state machine) is now just the DALI adapter for the generic
 * slot queue. The controller owns all of that state and drives commissioning
 * itself; the gateway holds nothing a reboot could lose.
 *
 * Concurrency, unchanged from before: the queue's terminal write (mbq_complete —
 * response8 BEFORE status) runs in the LVL1 DALI completion handler, and
 * mbq_read_status fences each slot's read. start_next_dali_work commits its
 * launch under a brief ATOMIC re-check against the LVL0 idle-timer / PIT / edge
 * callbacks.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "dali_sync.h"
#include "dali.h"
#include "registers.h"
#include "mbq_slots.h"
#include "tick.h"

volatile bool              dali_busy;
volatile dali_current_op_t current_op;
volatile cmd_entry_t       s_active;
volatile uint32_t          g_uptime_s;

// The executing slot and its unpacked opaque command [addr, data, flags, dtr0].
static uint8_t s_raw_slot;
static mbq_cmd_t s_raw_cmd;

void dali_sync_init(void) {
    dali_busy    = false;
    current_op.op = OP_NONE;
    mbq_init();
}

void issue_dali_command(uint8_t addr_byte, uint8_t data_byte,
                        dali_kind_t kind, uint8_t repeat) {
    // No cache invalidation any more — there is no cache to invalidate. The
    // controller decides what to re-read after a command.
    cmd_type_t t;
    switch (kind) {
        case DALI_KIND_DAPC:  t = CMD_DIRECT_ARC; break;
        case DALI_KIND_CMD:   t = CMD_SEND;       break;
        case DALI_KIND_TWICE: t = CMD_SEND_TWICE; break;
        case DALI_KIND_QUERY: t = CMD_QUERY;      break;
        default:              t = CMD_SEND;       break;
    }
    s_active.type      = t;
    s_active.dali_addr = addr_byte;
    s_active.dali_cmd  = data_byte;
    s_active.repeat    = repeat;

    dali_transmitting_state_enter((const cmd_entry_t *)&s_active);
}

// ---- Dispatch ---------------------------------------------------------

void dali_sync_dispatch_current(void) {
    if (current_op.op != OP_RAW) return;

    // Unpack the executing slot's opaque command and issue it. A QF_DTR0
    // command is a two-step atomic sequence — step 0 preloads DTR0, step 1
    // sends the frame — back-to-back with nothing interleaved, because
    // dali_busy stays held across the pair (one slot's lifecycle).
    dali_frame_t f = dali_frame_of(&s_raw_cmd);

    if ((f.flags & QF_DTR0) && current_op.step == 0u) {
        issue_dali_command(DALI_ADDR_SET_DTR0, f.dtr0, DALI_KIND_CMD, 1u);
        return;
    }
    // Terminal frame. KIND from the flags byte:
    //   QF_TWICE  -> send twice (SET-class), no answer
    //   QF_ANSWER -> KIND_QUERY, open the backward-frame window
    //   else      -> KIND_CMD, completes as soon as the frame is out
    if (f.flags & QF_TWICE)       issue_dali_command(f.addr, f.data, DALI_KIND_TWICE, 2u);
    else if (f.flags & QF_ANSWER) issue_dali_command(f.addr, f.data, DALI_KIND_QUERY, 1u);
    else                          issue_dali_command(f.addr, f.data, DALI_KIND_CMD,   1u);
}

// ---- Work selection ---------------------------------------------------

bool start_next_dali_work(void) {
    bool launch = false;
    uint8_t old_tcb = TCB0.INTCTRL;
    TCB0.INTCTRL = 0;   // fence the LVL1 completion handler across the selection

    uint8_t slot = MBQ_NONE;
    if (dali_idle && !dali_busy) slot = mbq_next_pending();

    if (slot != MBQ_NONE) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            // Re-check under global-atomic: state may have moved during the
            // selection (LVL0 idle timer / PIT / edge callbacks).
            if (dali_idle && !dali_busy) {
                mbq_begin(slot, &s_raw_cmd);
                s_raw_slot      = slot;
                current_op.op   = OP_RAW;
                current_op.addr = 0xFFu;
                current_op.step = 0;
                dali_busy = true;
                dali_idle = false;
                dali_cancel_edge_detect();
                launch = true;
            }
        }
    }
    TCB0.INTCTRL = old_tcb;

    if (launch) {
        // Committed to a send. If another master is driving the bus, wait for
        // it to clear; the wait-for-idle -> idle-state-enter path re-enters and
        // dispatches via dali_sync_dispatch_current (dali_busy is true).
        if (dali_is_bus_active()) dali_wait_for_idle_state_enter();
        else                      dali_sync_dispatch_current();
    }
    return launch;
}

// ---- Completion handler (ISR context) ---------------------------------

void dali_sync_completion(dali_response_type_t type, uint8_t value) {
    bool more = false;

    if (current_op.op == OP_RAW) {
        dali_frame_t f = dali_frame_of(&s_raw_cmd);

        // DTR0 preload just finished (no backward frame, so OK/NAK both mean
        // "done"). Advance to the frame within the SAME slot lifecycle; a real
        // bus error falls through to terminal handling below.
        if ((f.flags & QF_DTR0) && current_op.step == 0u
            && (type == DALI_RESPONSE_OK || type == DALI_RESPONSE_NAK)) {
            current_op.step = 1u;
            more = true;
        } else {
            // Terminal. Map the DALI outcome to the slot's (status, response8):
            //   OK        -> COMPLETE, response8 = backward-frame answer (0 if none)
            //   NAK       -> FAILED,   DALI_RESP_NAK
            //   MALFORMED -> FAILED,   DALI_RESP_COLLISION
            //   other     -> FAILED,   DALI_RESP_BUSFAULT
            // mbq_complete writes response8 BEFORE status under the read fence,
            // so a fenced reader never sees a terminal status with a stale value.
            uint8_t st, r8;
            if      (type == DALI_RESPONSE_OK)        { st = MBQ_S_COMPLETE; r8 = value; }
            else if (type == DALI_RESPONSE_NAK)       { st = MBQ_S_FAILED;   r8 = DALI_RESP_NAK; }
            else if (type == DALI_RESPONSE_MALFORMED) { st = MBQ_S_FAILED;   r8 = DALI_RESP_COLLISION; }
            else                                      { st = MBQ_S_FAILED;   r8 = DALI_RESP_BUSFAULT; }
            mbq_complete(s_raw_slot, st, r8);
        }
    }

    if (!more) {
        dali_busy = false;
        current_op.op = OP_NONE;
    }

    // Always go through the wait-for-idle gap. When dali_busy stays true,
    // dali_idle_state_enter dispatches the next step via
    // dali_sync_dispatch_current; when false, it arms idle.
    dali_wait_for_idle_state_enter();
}

// Abort the in-flight command because the bus is unusable (stuck/shorted past
// the escape timeout). The slot MUST be completed, not just abandoned: an
// EXECUTING slot that never reaches a terminal leaves the controller waiting on
// a command that will never finish. Reports a bus fault so the controller can
// tell "the bus is broken" from "the ballast said no".
//
// Deliberately does NOT re-enter the wait-for-idle path — the caller is already
// in it and drives the transition itself.
void dali_sync_abort_busfault(void) {
    if (current_op.op == OP_RAW)
        mbq_complete(s_raw_slot, MBQ_S_FAILED, DALI_RESP_BUSFAULT);
    dali_busy     = false;
    current_op.op = OP_NONE;
}

// ---- 1 Hz tick --------------------------------------------------------
//
// The RTC PIT used to also drive a round-robin rescan cursor that set
// SYNC_READ_LEVEL on one ballast per interval. That is the controller's job
// now, so all this does is keep the uptime counter for the sysinfo block.

ISR(RTC_PIT_vect) {
    g_uptime_s++;
    RTC.PITINTFLAGS = RTC_PI_bm;
}
