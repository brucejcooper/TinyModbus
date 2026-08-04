/*
 * dali_sync.c — DALI issuance for the command queue. See dali_sync.h.
 *
 * STATELESS PIPE. This is only the DALI adapter for the generic slot queue: it
 * unpacks a slot's opaque command, puts the frames on the wire, and reports the
 * outcome back. The controller owns the bus state — ballast levels, presence,
 * group membership, commissioning — so the gateway holds nothing a reboot could
 * lose.
 *
 * Concurrency: the queue's terminal write (mbq_complete —
 * response8 BEFORE status) runs in the LVL1 DALI completion handler, and
 * mbq_read_status fences each slot's read. start_next_dali_work commits its
 * launch under a brief ATOMIC re-check against the LVL0 idle-timer / PIT / edge
 * callbacks.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "dali_sync.h"
#include "errors.h"
#include "dali.h"
#include "registers.h"
#include "mbq_slots.h"
#include "tick.h"


// The slot we are running and its unpacked command [addr, data, flags, dtr0].
// MBQ_NONE means we own nothing — so this handle, which we need anyway to call
// mbq_complete, IS the "is a command in flight" flag. No separate bool can
// disagree with it, and the queue's own MBQ_S_EXECUTING remains the truth.
static uint8_t s_raw_slot = MBQ_NONE;
static mbq_cmd_t s_raw_cmd;

// Where we are in the executing command. Meaningless unless we own the channel.
// Together
// with s_raw_cmd this is the WHOLE issuance state — see dali_sync.h.
static volatile dali_step_t s_step;

void dali_sync_init(void) {
    s_raw_slot = MBQ_NONE;
    mbq_init();
}

bool dali_sync_owns_channel(void) {
    return s_raw_slot != MBQ_NONE;
}

// ---- Dispatch ---------------------------------------------------------

// Put the current step on the wire. Both bytes and the window decision are
// DERIVED here from the command plus s_step — this is the only place that
// knows how a queued command maps onto sends.
static void dispatch_step(void) {
    dali_frame_t f = dali_frame_of(&s_raw_cmd);

    if (s_step == DALI_STEP_DTR0) {
        // The preload never awaits an answer, whatever the frame after it does.
        dali_transmitting_state_enter(DALI_ADDR_SET_DTR0, f.dtr0, false);
        return;
    }
    // A frame step holds the window open to receive a backward frame
    // (QF_ANSWER) or to provide the 22 TE inter-send gap (QF_TWICE — both of
    // its sends use it).
    dali_transmitting_state_enter(f.addr, f.data,
                                  (f.flags & (QF_ANSWER | QF_TWICE)) != 0u);
}

void dali_sync_dispatch_current(void) {
    if (!dali_sync_owns_channel()) {
        // The caller reaches here only with the channel claimed, so returning
        // quietly parks the DALI channel busy with nothing in flight — a stall
        // that clears only via the bus escape timeout.
        tm_fault(TM_SITE_DALI_ISSUE, TM_ERR_BAD_SLOT);
        return;
    }
    dispatch_step();
}

// The second send of a QF_TWICE command, re-fired from the completion path
// (ISR) before the slot is allowed to go terminal. Returns true if another send
// was issued. Replaces a mutable repeat counter: "which send is this" IS the
// step, so there is nothing to decrement and nothing that can disagree with the
// command's own flags.
bool dali_sync_repeat_send(void) {
    dali_frame_t f = dali_frame_of(&s_raw_cmd);
    if (s_step != DALI_STEP_FRAME || !(f.flags & QF_TWICE)) return false;
    s_step = DALI_STEP_FRAME_2;
    dispatch_step();
    return true;
}

// ---- Work selection ---------------------------------------------------

bool start_next_dali_work(void) {
    // The whole main-loop decision, in the order the questions actually matter:
    //
    //   1. A command is executing -> nothing to do but let it finish.
    //   2. Otherwise, a command is pending -> start it, if the bus is free.
    //   3. Neither -> nothing to do.
    //
    // All three answers come from the queue and the line, with no gateway-side
    // summary of either to fall out of step with them.
    //
    // The two early exits below are also why this can report "work outstanding"
    // for free: in both cases the answer is known WITHOUT scanning the slots.
    if (dali_sync_owns_channel()) return true;   // one of ours is in flight
    if (dali_bus_busy)            return true;   // settling, or someone else has it

    bool launch = false;
    uint8_t old_tcb = TCB0.INTCTRL;
    TCB0.INTCTRL = 0;   // fence the LVL1 completion handler across the selection

    uint8_t slot = mbq_next_pending();
    if (slot != MBQ_NONE) {
        // THE ONLY INTERRUPT FENCE LEFT IN THE QUEUE PATH, and it guards
        // dali_bus_busy, not the slots. mbq_slots.c needs none — its write
        // ordering carries the invariant (see that file's header) — but this is a
        // genuine test-and-set: between reading "bus free" and committing the
        // launch, an LVL0 callback (idle timer, PIT, or the rising-edge monitor
        // firing because someone else took the line) can set dali_bus_busy. Losing
        // that race means transmitting on top of another sender.
        //
        // Kept deliberately short for the reason mbq_slots.c has none at all: a
        // long cli here starves the USART RX FIFO. TCB0.INTCTRL above fences LVL1
        // separately, so this block only has to exclude LVL0.
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (!dali_bus_busy && !dali_sync_owns_channel()) {
                // If the queue cannot hand us the command, do NOT launch: a
                // committed launch with no command would put the previous frame
                // back on the bus. Record it — nothing above can act on it — and
                // leave the line parked for the next selection.
                if (mbq_begin(slot, &s_raw_cmd) != TM_OK) {
                    tm_fault(TM_SITE_MBQ_BEGIN, TM_ERR_BAD_SLOT);
                    goto no_launch;
                }
                s_raw_slot = slot;          // we now own the channel
                // A command without QF_DTR0 has no preload step to run.
                s_step = (dali_frame_of(&s_raw_cmd).flags & QF_DTR0)
                             ? DALI_STEP_DTR0 : DALI_STEP_FRAME;
                dali_bus_busy = true;
                dali_cancel_edge_detect();
                launch = true;
            }
no_launch: ;
        }
    }
    TCB0.INTCTRL = old_tcb;

    if (launch) {
        // Committed to a send. If something else is driving the bus, wait for it
        // to clear; the wait-for-idle -> idle-state-enter path re-enters and
        // dispatches via dali_sync_dispatch_current.
        if (dali_is_bus_active()) dali_wait_for_idle_state_enter();
        else                      dali_sync_dispatch_current();
    }
    // Work existed if a slot was pending, whether or not we managed to launch it
    // (the ATOMIC re-check can lose the bus, and mbq_begin can refuse). Either
    // way it is still queued, so the caller must not sleep on it.
    return slot != MBQ_NONE;
}

// ---- Completion handler (ISR context) ---------------------------------

void dali_sync_completion(dali_response_type_t type, uint8_t value) {
    bool more = false;

    if (dali_sync_owns_channel()) {
        // DTR0 preload just finished (no backward frame, so OK/NAK both mean
        // "done"). Advance to the frame within the SAME slot lifecycle; a real
        // bus error falls through to terminal handling below.
        if (s_step == DALI_STEP_DTR0
            && (type == DALI_RESPONSE_OK || type == DALI_RESPONSE_NAK)) {
            s_step = DALI_STEP_FRAME;
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
            // ISR context: no caller to return to, and a dropped completion
            // strands the slot EXECUTING until reboot. Record it so a stuck slot
            // is diagnosable over Modbus instead of a mystery.
            tm_fault(TM_SITE_MBQ_COMPLETE, mbq_complete(s_raw_slot, st, r8));
        }
    }

    if (!more) s_raw_slot = MBQ_NONE;   // released; wait-for-idle re-parks us

    // Always go through the wait-for-idle gap. When we stay EXECUTING,
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
    if (dali_sync_owns_channel())
        tm_fault(TM_SITE_MBQ_COMPLETE,
                 mbq_complete(s_raw_slot, MBQ_S_FAILED, DALI_RESP_BUSFAULT));
    s_raw_slot = MBQ_NONE;
}

