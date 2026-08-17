#ifndef CQ_H
#define CQ_H

/*
 * cq.h — the v4 command queue and response queue.
 *
 * A DALI-AGNOSTIC pair of queues. Commands go in as an OPAQUE 4-byte blob and
 * results come out as an 8-bit value plus a 3-bit status; the meaning of both
 * (the command bytes, the answer byte, the failure codes) belongs entirely to
 * the caller — the DALI adapter in dali_sync.c. Do not add payload semantics
 * here. The protocol is QUEUE-PROTOCOL.md (v4), which is normative.
 *
 * TWO QUEUES, ONE IDENTITY:
 *
 *   cmdq   accepted-but-unfinished commands, executed strict FIFO in accept
 *          order. A command occupies a slot from cq_accept() until its result
 *          has been pushed; there are no terminal states, because a finished
 *          command is no longer a command — it is a response-queue entry.
 *   respq  results, held until the master explicitly acks them. Retention is
 *          what makes a lost reply survivable: no reply loss can destroy a
 *          result, so the master's recovery is a re-poll, never a re-execute.
 *
 * The seq (1..31, 0 reserved for "none") is assigned HERE, by the gateway, and
 * is the only identity a command has. The master learns it from the reply. That
 * is what makes a gateway reboot self-correcting — the counter simply restarts
 * and the master learns the new numbers; there is no shared ledger to reconcile,
 * which was v1's fatal flaw.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * CONCURRENCY: ONE WRITER PER BYTE, AND NO INTERRUPT FENCES. DELIBERATELY.
 *
 * cq_complete() runs in ISR CONTEXT AT EITHER OF TWO INTERRUPT LEVELS, and the
 * distinction matters enough to state precisely:
 *
 *   LVL1  the normal path — the DALI completion handler (TCB0 CAPT) via
 *         dali_sync_completion().
 *   LVL0  the bus-stuck escape — TCA0_OVF -> dali_tick() -> bus_stuck_timeout()
 *         -> dali_sync_abort_busfault(), which fails the in-flight command
 *         rather than leaving it EXECUTING forever.
 *
 * EVERYTHING else runs in main context (the Modbus handler and the work
 * arbiter). Two cq_complete() calls nesting would both read-modify-write
 * s_rq_pushed — the one race this design cannot absorb — so it is worth knowing
 * WHY that cannot happen: TCB0's CAPT interrupt is disabled outside the DALI
 * receive state (dali_state_receiving.c), so no LVL1 completion can fire during
 * the LVL0 escape path. THAT IS LOAD-BEARING; re-enabling TCB0 CAPT more widely
 * would need this reconsidered.
 *
 * The single-writer rules below survive either level regardless, because what
 * they actually partition is main against not-main.
 *
 * There are no fences in cq.c and adding one is a bug, not a hardening: a
 * global cli here blocks the
 * DALI bit-timing callbacks and the USART RX ISR, and an over-long ATOMIC_BLOCK
 * on this path is precisely what caused the 2026-06-05 RX starvation incident
 * (see CLAUDE.md) — a scan held interrupts off for hundreds of µs and overran
 * the 2-byte RX FIFO.
 *
 * What carries the invariants instead is a strict SINGLE-WRITER rule on every
 * shared byte, plus write ORDER:
 *
 *   cmdq[i].state    main writes FREE->PENDING->EXECUTING; the ISR writes only
 *                    EXECUTING->FREE. Never both at once: the ISR can only fire
 *                    after main launched the frame, and main does not touch the
 *                    byte again until it observes FREE.
 *   s_rq_pushed      ISR only.        s_rq_popped   main only.
 *   s_head, s_live   main only (the ISR frees slots; main reclaims them).
 *
 * A count is therefore never read-modify-written by two contexts — which is why
 * respq is two free-running 8-bit counters rather than the obvious head+count:
 * head+count would have main and the ISR both incrementing `count`, and that
 * race is not fixable by ordering.
 *
 * PUBLICATION ORDERING (load-bearing, the v2 lesson kept verbatim):
 *
 *   cq_complete   writes the entry's two bytes, THEN increments s_rq_pushed.
 *   cq_head       reads s_rq_pushed, THEN the entry.
 *   cq_accept     writes cmd[] and seq, THEN publishes state = PENDING.
 *   cq_next       reads state before trusting cmd[]/seq.
 *
 * So a reader can never see a published entry holding stale bytes. Each of those
 * publishing writes is a single AVR store and cannot tear; `volatile` is what
 * stops the compiler reordering the pair. Swapping any of those two lines
 * reintroduces exactly the race the order exists to prevent.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <stdint.h>
#include <stdbool.h>
#include "errors.h"

// Command-queue depth: "current + next" is the requirement. A BUILD-TIME
// constant, and nothing on the wire depends on it — the master learns its
// in-flight cap from this number being matched on its own side, not from any
// field in a frame. Moving it to 3 is this one line (the depth-3 soak in the
// task-08 validation gates decides whether it should be).
#define CMDQ_DEPTH   2u

// Response-queue depth. Steady-state occupancy is 2 (one result being collected,
// one arriving); 4 leaves room for the master to fall a transaction behind
// without stalling DALI. Must be a power of two <= 128, for two reasons the
// static asserts below spell out.
#define RESPQ_DEPTH  4u

#define CQ_CMD_BYTES 4u    // opaque command size (32-bit)

#define CQ_NONE      0xFFu // "no entry" sentinel from cq_next()
#define CQ_SEQ_NONE  0u    // reserved seq: no ack requested, none assigned, empty
#define CQ_SEQ_MAX   31u   // seq field is 5 bits; 0 is reserved, so 1..31

// Response-queue entry status, the top 3 bits of a head byte. Generic, not DALI.
// THREE outcomes, because "it answered" and "it did not" do not partition the
// space — silence is sometimes the answer:
//
//   H_OK      the command did what it was asked. The payload byte carries the
//             answer for a query, or 0 for a command that expects no reply.
//   H_NAK     a command that EXPECTED a reply got none. A neutral FACT, not a
//             verdict: this layer cannot know whether that means "the device is
//             absent" or "the device said no", because only the issuer knows
//             which kind of question it asked. Payload 0.
//   H_FAILED  something broke — a garbled reply, a bus fault. The payload byte
//             carries the failure code.
//
// H_NAK exists so H_FAILED means only breakage. Folding it into H_FAILED (as
// this protocol did until 2026-08-13) makes the failure counters count normal
// negative answers, and leaves "absent" and "said no" distinguishable only by
// the issuer remembering which flags it set.
#define H_OK         0u
#define H_FAILED     1u
#define H_NAK        2u

// An opaque command. Deliberately a byte blob that this layer carries and never
// interprets. The wrapper exists for TYPE SAFETY, not meaning — it stops a bare
// uint8_t* from being confused with a register pointer or a wrongly-sized
// buffer, which is precisely the mix-up that once made a submit declare twice
// its length and arm the following slot with adjacent memory.
typedef struct { uint8_t b[CQ_CMD_BYTES]; } cq_cmd_t;

// Command lifecycle. THREE states, and that is the whole set: a finished command
// is not a command in a terminal state, it is a response-queue entry and a freed
// slot. v2's COMPLETE/FAILED states are gone with the slot model that needed
// them (the master used to have to read a slot to learn it had finished, which
// is what forced the extra poll per DALI frame).
enum {
    CQ_FREE      = 0u,
    CQ_PENDING   = 1u,   // accepted, waiting its turn
    CQ_EXECUTING = 2u,   // handed to the DALI adapter, result not yet pushed
};

// Cold-path diagnostic counters, served over FC04 (see registers.h
// FC04_QDIAG_BLOCK). 16-bit and WRAPPING: these are rate/ratio evidence read a
// few times a session, not health flags, so a wrap is readable where a
// saturating counter would hide the recent past.
//
// `results` is the one counter incremented in ISR context, so a 16-bit read of
// it can tear across the increment and misreport one sample. That is accepted
// deliberately: fencing it would put the cli in this file that the header
// forbids, for a cold-path diagnostic whose next read is correct anyway.
// `full` and `stalls` are both NORMAL, expected counters, not health flags —
// registers.h spells this out for whoever finds them non-zero on a bench:
// `full` is the flow-control refusal share, the headroom gauge (~72% of
// transactions under one saturated gateway, by design — see
// QUEUE-PROTOCOL.md §5); `stalls` is the backpressure rule doing its job
// whenever the master stops acking. Neither means the gateway is broken; the
// one thing that IS still a fault is the invariant-break branch in
// cq_accept() (cq.c), which deliberately does NOT touch `full`.
typedef struct {
    uint16_t accepts;   // commands enqueued (A_ACCEPTED replies sent)
    uint16_t full;      // enqueue attempts refused, queue full (A_FULL replies)
    uint16_t executes;  // commands handed to the DALI adapter
    uint16_t results;   // results pushed onto the response queue  [ISR-written]
    uint16_t acks;      // response entries popped by a matching ack
    uint16_t stalls;    // cq_next() refusals caused by a full response queue
} cq_counters_t;

// Reset both queues and the sequence counter. Call once at boot. No state
// crosses a reboot — that is the point: the master detects the restart from the
// seq numbering (rules R2/R3) and reconstructs everything it needs.
void cq_init(void);

// ---- Modbus side (main context) --------------------------------------------

// Enqueue one opaque command. Returns the seq it was stamped with (1..31), or
// CQ_SEQ_NONE (0) if the command queue is full, which the caller reports in band
// as A_FULL rather than as an exception — an exception would throw away the head
// report in the same breath, and the head is exactly what the master needs to
// learn that a slot freed.
//
// A full command queue IS this protocol's flow control (the response queue's
// own backpressure rule, via cq_next, is a separate mechanism — see this
// file's top-of-file notes). The client re-offers on A_FULL rather than
// pacing against an estimate, so this is EXPECTED to happen often: under one
// saturated gateway ~72% of transactions are refusals, and that share is the
// headroom gauge (QUEUE-PROTOCOL.md §5). Count it — `full` is a rate/ratio
// diagnostic, not a health flag — but a non-zero counter here is not itself a
// fault to investigate.
uint8_t cq_accept(const uint8_t cmd[CQ_CMD_BYTES]);

// Ack: delete the head of the response queue IFF the head's seq equals `seq`.
// Returns whether it popped. A mismatch — or an ack for an already-popped entry
// — is a SILENT NO-OP by design, which is what makes ack-carrying frames safe to
// resend verbatim after a timeout: re-acking cannot destroy the wrong result,
// and the reply's head field shows the master the real head, which is all it
// needs to re-converge. `seq` == 0 means "no ack" and pops nothing.
bool cq_ack(uint8_t seq);

// Current head of the response queue as the two wire bytes:
//   out[0] = [hstatus:3 | hseq:5],  out[1] = resp8
// Both zero when the queue is empty — hseq 0 is the reserved "none", so an empty
// head is unambiguous and needs no separate flag.
void cq_head(uint8_t out[2]);

// ---- Arbiter side (main context) -------------------------------------------

// Index of the command to run next, or CQ_NONE if there is nothing to start.
//
// BACKPRESSURE (load-bearing, spec §0): this returns CQ_NONE when the response
// queue is full even though a command is pending. A full response queue stalls
// DALI execution rather than ever dropping a result. The master always has a way
// out, because the ack in a request is processed BEFORE the enqueue: one
// transaction frees an entry and refills the pipeline in the same frame.
//
// `stalls` counts EPISODES, not refusals — the rising edge only. The arbiter is
// polled thousands of times a second, so a per-call counter would wrap every few
// seconds and tell you nothing about how often backpressure actually engaged.
uint8_t cq_next(void);

// Take entry `idx` (from cq_next): copy its command into *out_cmd, its seq into
// *out_seq, and mark it EXECUTING.
//   TM_OK            *out_cmd and *out_seq hold the command
//   TM_ERR_BAD_SLOT  idx out of range, NOT the queue head, or not PENDING;
//                    *out_cmd is ZEROED and *out_seq set to CQ_SEQ_NONE
// Only the head is startable, and that is enforced here rather than assumed of
// the caller: it is what keeps execution strict FIFO, and what stops a caller
// that passed its own index from putting two commands in flight and quietly
// invalidating the response-queue reservation cq_next made.
// The zeroing matters: a caller that ignored the failure would otherwise re-fire
// whatever command the buffer still held — a repeat of the previous DALI frame,
// which for a non-idempotent UP/DOWN step is exactly the double-apply the
// protocol works to prevent.
tm_err_t cq_begin(uint8_t idx, cq_cmd_t *out_cmd, uint8_t *out_seq);

// Completion hook, called from the DALI completion ISR. Pushes {hstatus, resp8}
// onto the response queue under command `seq`, then frees its command slot.
//   TM_OK            result published, slot freed
//   TM_ERR_BAD_SLOT  no EXECUTING command carries that seq — nothing changed
//   TM_ERR_BUSY      response queue full: the slot is FREED and THE RESULT IS
//                    DROPPED
// TM_ERR_BUSY is unconstructible while the backpressure rule holds: cq_next
// reserved the entry before this command ever started, and only one command can
// be executing — which cq_begin ENFORCES (head only) rather than assuming of
// its caller. It is still handled rather than asserted, and it frees
// the slot on the way out, because the alternative failure mode is worse: a
// stranded EXECUTING slot wedges the queue until reboot, whereas a dropped
// result is a case the master already has a rule for (R4, the result deadline).
// Callers must hand both errors to tm_fault() — this runs where there is no
// caller to return to, and a dropped completion that is also unrecorded is how a
// stuck queue becomes a mystery.
tm_err_t cq_complete(uint8_t seq, uint8_t hstatus, uint8_t resp8);

// ---- Diagnostics ------------------------------------------------------------

// The counter block, for the FC04 diag read. Returns a pointer to live storage,
// not a snapshot: the caller serialises it directly into the response buffer.
const volatile cq_counters_t *cq_counters(void);

#endif /* CQ_H */
