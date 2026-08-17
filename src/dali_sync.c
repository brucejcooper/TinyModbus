/*
 * dali_sync.c — DALI issuance for the command queue. See dali_sync.h.
 *
 * STATELESS PIPE. This is only the DALI adapter for the generic slot queue: it
 * unpacks a slot's opaque command, puts the frames on the wire, and reports the
 * outcome back. The controller owns the bus state — ballast levels, presence,
 * group membership, commissioning — so the gateway holds nothing a reboot could
 * lose.
 *
 * Concurrency: the queue's completion write (cq_complete — the result's bytes
 * BEFORE the counter that publishes them) is reached from TWO interrupt levels,
 * not one: LVL1 for a normal completion (dali_sync_completion) and LVL0 for the
 * bus-stuck escape (dali_sync_abort_busfault). Everything else here runs in main
 * context. start_next_dali_work commits its launch under a brief ATOMIC re-check
 * against the LVL0 idle-timer / PIT / edge callbacks. See cq.h for why the two
 * completion paths cannot nest, which is what keeps the queue's unfenced
 * single-writer discipline sound.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "dali_sync.h"
#include "errors.h"
#include "dali.h"
#include "registers.h"
#include "cq.h"
#include "tick.h"


// The command we are running: its queue index, its seq, and its unpacked bytes
// [addr, data, flags, dtr0]. CQ_NONE means we own nothing — so this handle, which
// we need anyway, IS the "is a command in flight" flag. No separate bool can
// disagree with it, and the queue's own CQ_EXECUTING remains the truth.
//
// The SEQ, not the index, is what completion is reported against: a seq names
// one command for its whole life, where an index names a reusable slot, so a
// completion can never be attributed to whatever was enqueued after it.
//
// VOLATILE: both are written from main (start_next_dali_work, inside the
// ATOMIC re-check) and from ISR context (dali_sync_completion at LVL1,
// dali_sync_abort_busfault at LVL0), then read back from main
// (dali_sync_owns_channel, and again inside that same ATOMIC re-check). The
// two sides never fight over a value — main only ever moves the handle
// CQ_NONE -> owned, an ISR only ever moves it owned -> CQ_NONE — but without
// volatile the compiler is free to cache a stale read across that boundary;
// only -O3 happening to reload it made this safe before.
static volatile uint8_t s_cq_idx = CQ_NONE;
static volatile uint8_t s_cq_seq = CQ_SEQ_NONE;
static cq_cmd_t s_raw_cmd;

// Where we are in the executing command. Meaningless unless we own the channel.
// Together with s_raw_cmd this is the WHOLE issuance state — see dali_sync.h.
static volatile dali_step_t s_step;

void dali_sync_init(void) {
    s_cq_idx = CQ_NONE;
    s_cq_seq = CQ_SEQ_NONE;
    cq_init();
}

bool dali_sync_owns_channel(void) {
    return s_cq_idx != CQ_NONE;
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
    // (QF_ANSWER) or to provide the 22 TE inter-send gap (QF_TWICE). Only the
    // FIRST send needs QF_TWICE for that gap — it's what re-fires the second
    // send with no dead time (dali_sync_repeat_send, called from within that
    // same wait). The SECOND send has nothing left to wait for: a send-twice
    // command can never carry QF_ANSWER (it's SET-class, no answer defined),
    // so gating on QF_TWICE too for FRAME_2 was holding the window open for a
    // response nothing could ever fill, burning a full NORESPONSE_TIMEOUT_
    // TICKS (~9.2 ms, dali_state_wait_for_response.c) before the settle that
    // was actually needed (dali_wait_for_idle_state_enter, unconditional at
    // the end of dali_sync_completion) even started. Same idea as CMD_SEND
    // skipping this window entirely (README.md's DALI state-machine section,
    // "TRANSMITTING") — just scoped to one step of a two-send command instead
    // of the whole command.
    bool want_window = (s_step == DALI_STEP_FRAME_2)
                            ? (f.flags & QF_ANSWER) != 0u
                            : (f.flags & (QF_ANSWER | QF_TWICE)) != 0u;
    dali_transmitting_state_enter(f.addr, f.data, want_window);
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

    // cq_next() also enforces the protocol's backpressure rule: it returns
    // CQ_NONE when the response queue is full even though a command is pending,
    // so a master that stops acking stalls DALI here rather than causing a result
    // to be dropped. From this function's point of view that is simply "no work
    // available", and the stall is counted in the queue's own diagnostics.
    uint8_t idx = cq_next();
    if (idx != CQ_NONE) {
        // THE ONLY INTERRUPT FENCE LEFT IN THE QUEUE PATH, and it guards
        // dali_bus_busy, not the queue. cq.c needs none — its single-writer rule
        // and write ordering carry the invariant (see that file's header) — but
        // this is a genuine test-and-set: between reading "bus free" and
        // committing the launch, an LVL0 callback (idle timer, PIT, or the
        // rising-edge monitor firing because someone else took the line) can set
        // dali_bus_busy. Losing that race means transmitting on top of another
        // sender.
        //
        // Kept deliberately short for the reason cq.c has none at all: a long cli
        // here starves the USART RX FIFO. TCB0.INTCTRL above fences LVL1
        // separately, so this block only has to exclude LVL0.
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (!dali_bus_busy && !dali_sync_owns_channel()) {
                // If the queue cannot hand us the command, do NOT launch: a
                // committed launch with no command would put the previous frame
                // back on the bus. Record it — nothing above can act on it — and
                // leave the line parked for the next selection.
                // cq_begin takes a plain uint8_t* — passing &s_cq_seq
                // directly would discard the volatile qualifier (and the
                // store inside cq.c would not be a volatile access). Land
                // the seq in a local, then store it through the volatile.
                uint8_t seq;
                if (cq_begin(idx, &s_raw_cmd, &seq) != TM_OK) {
                    tm_fault(TM_SITE_CQ_BEGIN, TM_ERR_BAD_SLOT);
                    goto no_launch;
                }
                s_cq_seq = seq;
                s_cq_idx = idx;             // we now own the channel
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
    // Work existed if the queue offered one, whether or not we managed to launch
    // it (the ATOMIC re-check can lose the bus, and cq_begin can refuse). Either
    // way it is still queued, so the caller must not sleep on it.
    return idx != CQ_NONE;
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
            // Done. Map the DALI outcome onto the result (hstatus, resp8):
            //   OK             -> H_OK,     resp8 = backward-frame answer
            //   NAK + QF_ANSWER-> H_NAK,    resp8 = 0
            //   NAK, no ANSWER -> H_OK,     resp8 = 0
            //   MALFORMED      -> H_FAILED, DALI_RESP_COLLISION
            //   other          -> H_FAILED, DALI_RESP_BUSFAULT
            //
            // The NAK split is the whole point. finished_transmission() reports
            // DALI_RESPONSE_NAK for a command that never opened a response
            // window at all, so "NAK" here covers two unrelated events:
            //
            //   a SET-class frame that completed normally, where no backward
            //   frame was ever EXPECTED — that is a success, not a NAK, and
            //   dali_sync.h's own definition of NAK says "where one was
            //   expected"; and
            //
            //   a query that asked for an answer and heard silence — a real
            //   NAK, reported as the neutral fact H_NAK.
            //
            // We deliberately do NOT decide what that silence MEANS. For a
            // yes/no query (COMPARE, QUERY LAMP FAILURE) silence is the answer
            // "no"; for a value query (QUERY ACTUAL LEVEL) it is evidence the
            // gear is absent. Only the issuer knows which it sent, so the
            // verdict belongs to the master and the fact belongs here.
            uint8_t st, r8;
            if      (type == DALI_RESPONSE_OK)        { st = H_OK;     r8 = value; }
            else if (type == DALI_RESPONSE_NAK)       {
                bool expected = (dali_frame_of(&s_raw_cmd).flags & QF_ANSWER) != 0u;
                st = expected ? H_NAK : H_OK;
                r8 = 0;
            }
            else if (type == DALI_RESPONSE_MALFORMED) { st = H_FAILED; r8 = DALI_RESP_COLLISION; }
            else                                      { st = H_FAILED; r8 = DALI_RESP_BUSFAULT; }
            // ISR context: no caller to return to, and a dropped completion
            // strands the command EXECUTING until reboot. Record it so a stuck
            // queue is diagnosable over Modbus instead of a mystery. The two
            // failures get different sites because they accuse different things:
            // BAD_SLOT is a lost completion, BUSY means the backpressure rule
            // did not hold and a result was thrown away.
            tm_err_t ce = cq_complete(s_cq_seq, st, r8);
            tm_fault(ce == TM_ERR_BUSY ? TM_SITE_CQ_RESPQ : TM_SITE_CQ_COMPLETE, ce);
        }
    }

    if (!more) s_cq_idx = CQ_NONE;   // released; wait-for-idle re-parks us

    // Always go through the wait-for-idle gap. When we stay EXECUTING,
    // dali_idle_state_enter dispatches the next step via
    // dali_sync_dispatch_current; when false, it arms idle.
    dali_wait_for_idle_state_enter();
}

// Abort the in-flight command because the bus is unusable (stuck/shorted past
// the escape timeout). The command MUST be completed, not just abandoned: an
// EXECUTING entry that never produces a result leaves the controller waiting on
// a command that will never finish — and under v4 it also pins the response
// queue's backpressure reservation, so it would stall every command behind it.
// Reports a bus fault so the controller can tell "the bus is broken" from "the
// ballast said no".
//
// Deliberately does NOT re-enter the wait-for-idle path — the caller is already
// in it and drives the transition itself.
//
// RUNS AT LVL0, not LVL1: the caller chain is TCA0_OVF -> dali_tick() ->
// bus_stuck_timeout(). This is the SECOND of cq_complete's two ISR entry points
// and the reason cq.h states its contract as "ISR context at either level"
// rather than naming the DALI completion handler alone.
void dali_sync_abort_busfault(void) {
    if (dali_sync_owns_channel()) {
        tm_err_t ce = cq_complete(s_cq_seq, H_FAILED, DALI_RESP_BUSFAULT);
        tm_fault(ce == TM_ERR_BUSY ? TM_SITE_CQ_RESPQ : TM_SITE_CQ_COMPLETE, ce);
    }
    s_cq_idx = CQ_NONE;
}

