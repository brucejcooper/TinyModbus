/*
 * cq_test.c — host-side test for src/cq.c, the v4 command/response queues,
 * and for src/fc41.h's request validation table.
 *
 * The last two groups stand in for hardware validation gate V5 ("malformed
 * frames are discarded, counted, no reply, no state change"). The bench has no
 * independent RS-485 master to inject bad frames with, so the length/flag table
 * is enumerated here instead — the whole input space rather than a sample of it.
 * The one part of V5 that CANNOT live here is the discard counter, which is in
 * modbus.c and does not compile on the host; that increment stays a code-review
 * item.
 *
 * Not part of the firmware build: cq.c is pure C whose only dependency is
 * errors.h (a type and three prototypes it never calls), so it compiles and
 * runs on the host exactly as it does on the ATtiny. What CANNOT be reproduced
 * here is the concurrency — cq_complete really runs in the DALI completion ISR
 * — so this file tests the STATE MACHINE and the protocol invariants, and the
 * single-writer/publication-order rules that make it safe against that ISR are
 * argued in cq.h rather than tested here. Calling cq_complete from main line
 * below is a sequential stand-in for that ISR, which is legitimate because the
 * AVR is single-core: every real interleaving is one of these orderings.
 *
 * Build & run (from this directory):
 *   cc -Wall -Wextra -I ../src -o /tmp/cq_test cq_test.c ../src/cq.c \
 *       && /tmp/cq_test
 *
 * Exits non-zero on any failure. Clang emits one expected warning building
 * cq.c — "unknown pragma ignored" for its #pragma GCC optimize("Os"), which is
 * an avr-gcc flash-budget directive with no host meaning. Do not remove the
 * pragma to silence it; it is doing real work in the firmware build.
 */
#include "../src/cq.h"
#include "../src/fc41.h"

#include <stdio.h>
#include <string.h>

static int fail = 0;
#define CHECK(cond) do { \
        if (!(cond)) { \
            printf("FAIL: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
            fail = 1; \
        } \
    } while (0)

// A distinguishable 4-byte command, so a mis-delivered payload is visible
// rather than passing as a zeroed one.
static void mkcmd(uint8_t out[CQ_CMD_BYTES], uint8_t tag) {
    out[0] = (uint8_t)(0xA0u + tag); out[1] = tag;
    out[2] = 0x04u;                  out[3] = (uint8_t)(0x50u + tag);
}

// accept -> next -> begin -> complete, i.e. one command all the way through the
// gateway. Returns the seq it was given (0 if the accept was refused).
static uint8_t run_one(uint8_t tag, uint8_t hstatus, uint8_t resp8) {
    uint8_t cmd[CQ_CMD_BYTES];
    mkcmd(cmd, tag);
    uint8_t seq = cq_accept(cmd);
    if (seq == CQ_SEQ_NONE) return CQ_SEQ_NONE;
    uint8_t idx = cq_next();
    if (idx == CQ_NONE) return CQ_SEQ_NONE;
    cq_cmd_t got; uint8_t bseq = 0;
    if (cq_begin(idx, &got, &bseq) != TM_OK) return CQ_SEQ_NONE;
    if (bseq != seq || memcmp(got.b, cmd, CQ_CMD_BYTES) != 0) return CQ_SEQ_NONE;
    if (cq_complete(seq, hstatus, resp8) != TM_OK) return CQ_SEQ_NONE;
    return seq;
}

// ── 1. A full cycle, and what each step reports ──────────────────────────────
static void test_full_cycle(void) {
    cq_init();

    uint8_t head[2];
    cq_head(head);
    CHECK(head[0] == 0 && head[1] == 0);        // empty head is {0,0}

    uint8_t cmd[CQ_CMD_BYTES];
    mkcmd(cmd, 1);
    uint8_t seq = cq_accept(cmd);
    CHECK(seq == 1u);                            // first seq is 1, never 0

    cq_head(head);
    CHECK(head[0] == 0 && head[1] == 0);        // accepting does not produce a result

    uint8_t idx = cq_next();
    CHECK(idx != CQ_NONE);

    cq_cmd_t got; uint8_t bseq = 0;
    CHECK(cq_begin(idx, &got, &bseq) == TM_OK);
    CHECK(bseq == seq);
    CHECK(memcmp(got.b, cmd, CQ_CMD_BYTES) == 0);   // the command survives the queue intact

    CHECK(cq_next() == CQ_NONE);                 // nothing else pending; one runs at a time

    CHECK(cq_complete(seq, H_OK, 0xC7u) == TM_OK);
    cq_head(head);
    CHECK(head[0] == (uint8_t)((H_OK << 5) | 1u));
    CHECK(head[1] == 0xC7u);

    CHECK(cq_ack(seq) == true);
    cq_head(head);
    CHECK(head[0] == 0 && head[1] == 0);        // acked, and back to empty

    // The whole cycle is one of each.
    const volatile cq_counters_t *c = cq_counters();
    CHECK(c->accepts == 1 && c->executes == 1 && c->results == 1 && c->acks == 1);
    CHECK(c->full == 0 && c->stalls == 0);
}

// ── 2. H_FAILED packing, and the command slot being reusable afterwards ──────
static void test_failed_result(void) {
    cq_init();
    uint8_t seq = run_one(2, H_FAILED, 0x02u);   // DALI_RESP_COLLISION
    CHECK(seq == 1u);

    uint8_t head[2];
    cq_head(head);
    CHECK(head[0] == (uint8_t)((H_FAILED << 5) | 1u));
    CHECK((head[0] >> 5) == H_FAILED);
    CHECK((head[0] & CQ_SEQ_MAX) == 1u);
    CHECK(head[1] == 0x02u);
    CHECK(cq_ack(1u) == true);
}

// ── 3. Sequence numbers: 1..31, and 0 SKIPPED on wrap ────────────────────────
//
// The hazard this exists for: 0 is the reserved "none" on the wire, so a
// counter that emitted it would make "no ack requested" and "ack command 32"
// the same frame.
static void test_seq_wrap(void) {
    cq_init();
    uint8_t expect = 1;
    for (int i = 0; i < 70; i++) {               // >2 full laps of the 31 values
        uint8_t seq = run_one((uint8_t)i, H_OK, 0);
        CHECK(seq == expect);
        CHECK(seq != CQ_SEQ_NONE);               // 0 is never assigned
        CHECK(seq <= CQ_SEQ_MAX);                // and it always fits the 5-bit field
        CHECK(cq_ack(seq) == true);              // keep the response queue drained
        expect = (uint8_t)((expect >= CQ_SEQ_MAX) ? 1u : expect + 1u);
    }
}

// ── 4. Command queue full ────────────────────────────────────────────────────
static void test_cmdq_full(void) {
    cq_init();
    uint8_t cmd[CQ_CMD_BYTES];

    for (uint8_t i = 0; i < CMDQ_DEPTH; i++) {
        mkcmd(cmd, i);
        CHECK(cq_accept(cmd) == (uint8_t)(i + 1u));
    }
    mkcmd(cmd, 99);
    CHECK(cq_accept(cmd) == CQ_SEQ_NONE);        // full: A_FULL, not an error
    CHECK(cq_counters()->full == 1);
    CHECK(cq_counters()->accepts == CMDQ_DEPTH);

    // A refused accept must not have burned a sequence number, or the master's
    // predicted-seq rule (R1) would drift by one for every A_FULL it ever saw.
    //
    // The accept at the end of this block is also the reclaim test: it is the
    // first main-context call after the completion, so it only succeeds if
    // cq_accept reclaims freed slots itself instead of waiting for the arbiter.
    // If it did wait, an enqueue arriving between a completion and the next
    // arbiter pass would be told A_FULL over a slot that is already free —
    // a whole transaction per DALI frame at saturation.
    uint8_t idx = cq_next();
    CHECK(idx != CQ_NONE);
    cq_cmd_t got; uint8_t bseq = 0;
    CHECK(cq_begin(idx, &got, &bseq) == TM_OK);
    CHECK(cq_complete(bseq, H_OK, 0) == TM_OK);
    mkcmd(cmd, 100);
    CHECK(cq_accept(cmd) == (uint8_t)(CMDQ_DEPTH + 1u));
}

// ── 5. Both queues are FIFO ──────────────────────────────────────────────────
static void test_fifo_order(void) {
    cq_init();
    uint8_t cmd[CQ_CMD_BYTES];

    // Fill the command queue, then drain it: execution must follow accept
    // order, not slot order.
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++) { mkcmd(cmd, i); CHECK(cq_accept(cmd) == (uint8_t)(i + 1u)); }
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++) {
        uint8_t idx = cq_next();
        CHECK(idx != CQ_NONE);
        cq_cmd_t got; uint8_t bseq = 0;
        CHECK(cq_begin(idx, &got, &bseq) == TM_OK);
        CHECK(bseq == (uint8_t)(i + 1u));                 // oldest first
        uint8_t want[CQ_CMD_BYTES]; mkcmd(want, i);
        CHECK(memcmp(got.b, want, CQ_CMD_BYTES) == 0);    // ...with ITS OWN payload
        CHECK(cq_complete(bseq, H_OK, (uint8_t)(0x10u + i)) == TM_OK);
    }

    // Results come back in the same order, and each holds its own value.
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++) {
        uint8_t head[2];
        cq_head(head);
        CHECK((head[0] & CQ_SEQ_MAX) == (uint8_t)(i + 1u));
        CHECK(head[1] == (uint8_t)(0x10u + i));
        CHECK(cq_ack((uint8_t)(i + 1u)) == true);
    }
    uint8_t head[2];
    cq_head(head);
    CHECK(head[0] == 0 && head[1] == 0);
}

// ── 6. Acks: matching pops, mismatching is a SILENT no-op ────────────────────
//
// The no-op is what makes an ack-carrying frame safe to resend verbatim after a
// timeout — re-acking cannot destroy the wrong result.
static void test_ack_semantics(void) {
    cq_init();
    CHECK(cq_ack(1u) == false);                  // ack against an empty queue
    CHECK(cq_ack(0u) == false);                  // seq 0 means "no ack"

    CHECK(run_one(1, H_OK, 0x11u) == 1u);
    CHECK(run_one(2, H_OK, 0x22u) == 2u);

    CHECK(cq_ack(2u) == false);                  // not the head: no-op, not a pop
    CHECK(cq_ack(31u) == false);                 // never issued
    CHECK(cq_ack(0u) == false);

    uint8_t head[2];
    cq_head(head);
    CHECK((head[0] & CQ_SEQ_MAX) == 1u);         // head is untouched by all three
    CHECK(head[1] == 0x11u);
    CHECK(cq_counters()->acks == 0);

    CHECK(cq_ack(1u) == true);
    CHECK(cq_ack(1u) == false);                  // re-acking the same seq is a no-op
    cq_head(head);
    CHECK((head[0] & CQ_SEQ_MAX) == 2u);
    CHECK(head[1] == 0x22u);
    CHECK(cq_ack(2u) == true);
    CHECK(cq_counters()->acks == 2);
}

// ── 7. BACKPRESSURE: a full response queue stalls DALI, and an ack frees it ──
//
// The load-bearing one. Retention is the guarantee the master's whole recovery
// design rests on, so a result must never be dropped to keep the bus moving.
static void test_backpressure(void) {
    cq_init();

    for (uint8_t i = 0; i < RESPQ_DEPTH; i++)
        CHECK(run_one(i, H_OK, (uint8_t)(0x80u + i)) == (uint8_t)(i + 1u));

    // Response queue is now full. A command can still be ACCEPTED — the Modbus
    // side keeps working, which is what lets the master ack its way out — but
    // the arbiter must refuse to START it.
    uint8_t cmd[CQ_CMD_BYTES];
    mkcmd(cmd, 200);
    uint8_t seq = cq_accept(cmd);
    CHECK(seq == (uint8_t)(RESPQ_DEPTH + 1u));
    CHECK(cq_next() == CQ_NONE);
    CHECK(cq_counters()->stalls == 1);
    CHECK(cq_next() == CQ_NONE);                 // still stalled, still counted
    CHECK(cq_counters()->stalls == 2);

    // The head is still readable while stalled: the master must be able to see
    // what to ack, or the stall would be unrecoverable.
    uint8_t head[2];
    cq_head(head);
    CHECK((head[0] & CQ_SEQ_MAX) == 1u);
    CHECK(head[1] == 0x80u);

    // One ack is enough to restart execution.
    CHECK(cq_ack(1u) == true);
    uint8_t idx = cq_next();
    CHECK(idx != CQ_NONE);
    CHECK(cq_counters()->stalls == 2);           // a successful selection is not a stall

    cq_cmd_t got; uint8_t bseq = 0;
    CHECK(cq_begin(idx, &got, &bseq) == TM_OK);
    CHECK(bseq == seq);
    CHECK(cq_complete(seq, H_OK, 0xEEu) == TM_OK);

    // Nothing was lost: the three unacked results plus the new one, in order.
    for (uint8_t i = 1; i < RESPQ_DEPTH; i++) {
        cq_head(head);
        CHECK((head[0] & CQ_SEQ_MAX) == (uint8_t)(i + 1u));
        CHECK(head[1] == (uint8_t)(0x80u + i));
        CHECK(cq_ack((uint8_t)(i + 1u)) == true);
    }
    cq_head(head);
    CHECK((head[0] & CQ_SEQ_MAX) == seq);
    CHECK(head[1] == 0xEEu);
    CHECK(cq_ack(seq) == true);
    cq_head(head);
    CHECK(head[0] == 0 && head[1] == 0);
    CHECK(cq_counters()->results == (uint16_t)(RESPQ_DEPTH + 1u));
}

// ── 8. Misuse of the arbiter hooks is refused, not obeyed ────────────────────
static void test_hook_guards(void) {
    cq_init();

    // cq_begin must ZERO its output on refusal: a caller that ignored the error
    // would otherwise re-fire whatever the buffer still held, which for a
    // non-idempotent UP/DOWN step is the double-apply the protocol prevents.
    cq_cmd_t got; uint8_t bseq = 0xFFu;
    memset(got.b, 0xAB, sizeof got.b);
    CHECK(cq_begin(CMDQ_DEPTH, &got, &bseq) == TM_ERR_BAD_SLOT);   // out of range
    for (uint8_t i = 0; i < CQ_CMD_BYTES; i++) CHECK(got.b[i] == 0);
    CHECK(bseq == CQ_SEQ_NONE);

    memset(got.b, 0xAB, sizeof got.b);
    bseq = 0xFFu;
    CHECK(cq_begin(0, &got, &bseq) == TM_ERR_BAD_SLOT);            // in range, but FREE
    for (uint8_t i = 0; i < CQ_CMD_BYTES; i++) CHECK(got.b[i] == 0);
    CHECK(bseq == CQ_SEQ_NONE);

    // Completing a seq that is not executing changes nothing.
    CHECK(cq_complete(7u, H_OK, 0x33u) == TM_ERR_BAD_SLOT);
    uint8_t head[2];
    cq_head(head);
    CHECK(head[0] == 0 && head[1] == 0);
    CHECK(cq_counters()->results == 0);

    // A PENDING command is not an EXECUTING one: completing it before it began
    // must be refused too, or a stray completion could retire a command that
    // never went on the wire.
    uint8_t cmd[CQ_CMD_BYTES];
    mkcmd(cmd, 1);
    uint8_t seq = cq_accept(cmd);
    CHECK(cq_complete(seq, H_OK, 0) == TM_ERR_BAD_SLOT);
    CHECK(cq_counters()->results == 0);

    // Begin it, then complete the WRONG seq: also refused, and the real command
    // is still executing afterwards.
    uint8_t idx = cq_next();
    CHECK(cq_begin(idx, &got, &bseq) == TM_OK);
    CHECK(cq_complete((uint8_t)(seq + 1u), H_OK, 0) == TM_ERR_BAD_SLOT);
    CHECK(cq_complete(seq, H_OK, 0x44u) == TM_OK);
    cq_head(head);
    CHECK((head[0] & CQ_SEQ_MAX) == seq);
    CHECK(head[1] == 0x44u);
}

// ── 9. Reserved-value invariants, swept over a long run ──────────────────────
//
// Everything the wire's reserved values depend on, asserted continuously rather
// than at one lucky moment: seq 0 is never assigned, seq never exceeds the
// 5-bit field, and hdr == 0 means empty AND ONLY means empty.
static void test_reserved_invariants(void) {
    cq_init();
    for (int i = 0; i < 200; i++) {
        uint8_t cmd[CQ_CMD_BYTES];
        mkcmd(cmd, (uint8_t)i);
        uint8_t seq = cq_accept(cmd);
        if (seq != CQ_SEQ_NONE) {
            CHECK(seq >= 1u && seq <= CQ_SEQ_MAX);
        }
        uint8_t idx = cq_next();
        if (idx != CQ_NONE) {
            cq_cmd_t got; uint8_t bseq = 0;
            CHECK(cq_begin(idx, &got, &bseq) == TM_OK);
            CHECK(bseq >= 1u && bseq <= CQ_SEQ_MAX);
            // Alternate the outcome so both hstatus values get packed into a hdr.
            CHECK(cq_complete(bseq, (i & 1) ? H_FAILED : H_OK, (uint8_t)i) == TM_OK);
        }
        uint8_t head[2];
        cq_head(head);
        if (head[0] != 0) {
            CHECK((head[0] & CQ_SEQ_MAX) != 0u);   // a non-empty head never carries seq 0
            // Ack every third pass, so the response queue spends time full,
            // partly full and empty across the sweep.
            if (i % 3 == 0) CHECK(cq_ack((uint8_t)(head[0] & CQ_SEQ_MAX)) == true);
        }
    }
    // Drain, and confirm the empty head is exactly {0,0}.
    for (int i = 0; i < (int)RESPQ_DEPTH + 1; i++) {
        uint8_t head[2];
        cq_head(head);
        if (head[0] == 0) { CHECK(head[1] == 0); break; }
        CHECK(cq_ack((uint8_t)(head[0] & CQ_SEQ_MAX)) == true);
    }
    uint8_t head[2];
    cq_head(head);
    CHECK(head[0] == 0 && head[1] == 0);
}

// ── 10. FC 0x41 request validation, exhaustively ─────────────────────────────
//
// This stands in for validation gate V5. The bench has no independent RS-485
// master to inject malformed frames with (the only spare USB-serial is the UPDI
// programmer), so the length/flag table is exercised here instead — which is
// arguably the better place for it anyway, because a host test can enumerate
// the ENTIRE input space where a bench sender could only sample it.
//
// fc41_len_ok() is the real production predicate, shared verbatim with
// frame_is_valid(); it lives in fc41.h precisely so both can reach it.
static void test_fc41_validation(void) {
    // The spec's table, row by row, with the flags stated as literals rather
    // than derived — a derived expectation would just restate the code.
    CHECK(fc41_len_ok(5, 0x00) == true);    // poll/ack, no flags: the legal 5 B
    CHECK(fc41_len_ok(5, 0x20) == false);   // 5 B + QW_ANSWER
    CHECK(fc41_len_ok(5, 0x40) == false);   // 5 B + QW_DTR0
    CHECK(fc41_len_ok(5, 0x80) == false);   // 5 B + QW_TWICE
    CHECK(fc41_len_ok(5, 0xE0) == false);   // 5 B + all three

    CHECK(fc41_len_ok(7, 0x00) == true);    // enqueue, no preload
    CHECK(fc41_len_ok(7, 0x20) == true);    // ...expecting an answer
    CHECK(fc41_len_ok(7, 0x80) == true);    // ...send-twice
    CHECK(fc41_len_ok(7, 0xA0) == true);    // ...both
    CHECK(fc41_len_ok(7, 0x40) == false);   // QW_DTR0 set but no dtr0 byte present
    CHECK(fc41_len_ok(7, 0xE0) == false);

    CHECK(fc41_len_ok(8, 0x40) == true);    // enqueue with preload
    CHECK(fc41_len_ok(8, 0x60) == true);    // ...expecting an answer
    CHECK(fc41_len_ok(8, 0xC0) == true);    // ...send-twice
    CHECK(fc41_len_ok(8, 0xE0) == true);    // ...all three
    CHECK(fc41_len_ok(8, 0x00) == false);   // dtr0 byte present but QW_DTR0 clear
    CHECK(fc41_len_ok(8, 0x20) == false);
    CHECK(fc41_len_ok(8, 0x80) == false);

    // The two inconsistent cases above are the security-relevant ones, and they
    // are why the table rejects rather than guesses: QW_DTR0 is what decides
    // whether byte 5 is a payload byte or the first CRC byte. Getting it wrong
    // either feeds a CRC byte to a ballast as a DTR0 value, or reads the CRC as
    // payload and the payload as CRC.

    // Every length from 0 to 12 that is not 5, 7 or 8 must reject for EVERY
    // flag combination — including 4, 6 and 9, the near-misses a crafted or
    // truncated frame would most plausibly present.
    for (unsigned len = 0; len <= 12u; len++) {
        if (len == 5u || len == 7u || len == 8u) continue;
        for (unsigned f = 0; f < 8u; f++)
            CHECK(fc41_len_ok((uint8_t)len, (uint8_t)(f << 5)) == false);
    }

    // The ack sequence shares the byte with the flags, so the decision must
    // depend on bits 7-5 ONLY. Sweep all 256 byte values against the same byte
    // with its low 5 bits cleared: an ack value must never change the verdict.
    // (A predicate that read the whole byte would reject every real steady-state
    // frame, since those all carry a piggybacked ack.)
    for (unsigned len = 4u; len <= 9u; len++) {
        for (unsigned b = 0; b < 256u; b++) {
            bool with_ack = fc41_len_ok((uint8_t)len, (uint8_t)b);
            bool no_ack   = fc41_len_ok((uint8_t)len, (uint8_t)(b & 0xE0u));
            CHECK(with_ack == no_ack);
        }
    }
}

// ── 11. A discarded frame changes nothing ────────────────────────────────────
//
// The other half of gate V5. The guard below mirrors modbus.c's control flow —
// validate, and only then touch the queues — using the REAL predicate and the
// REAL queue calls. What it cannot include is the discard counter itself, which
// lives in modbus.c and does not compile on the host; that increment is a
// code-review item, noted in the report.
static void feed(uint8_t len, uint8_t flagsack, uint8_t addr, uint8_t data) {
    if (!fc41_len_ok(len, flagsack)) return;        // discarded: no reply, no state change
    (void)cq_ack((uint8_t)(flagsack & CQ_SEQ_MAX)); // ack first, always
    if (len != FCQ_REQ_POLL) {
        uint8_t cmd[CQ_CMD_BYTES] = { addr, data, 0, 0 };
        (void)cq_accept(cmd);
    }
}

static void test_discard_changes_nothing(void) {
    cq_init();

    // Put the queue into a non-trivial state first: an empty queue would pass a
    // "nothing changed" check even if the frame HAD been processed.
    CHECK(run_one(1, H_OK, 0x5Au) == 1u);
    uint8_t cmd[CQ_CMD_BYTES];
    mkcmd(cmd, 2);
    CHECK(cq_accept(cmd) == 2u);

    cq_counters_t before = *cq_counters();
    uint8_t head_before[2];
    cq_head(head_before);

    // Every malformed shape, each carrying an ack for the head that is really
    // there (seq 1) and a plausible command — so if one were wrongly accepted it
    // would visibly pop the head AND enqueue.
    feed(4, 0x01, 0xFE, 0x05);
    feed(6, 0x01, 0xFE, 0x05);
    feed(9, 0x41, 0xFE, 0x05);
    feed(5, 0xA1, 0xFE, 0x05);   // 5 B with flags set
    feed(5, 0x41, 0xFE, 0x05);
    feed(7, 0x41, 0xFE, 0x05);   // 7 B with QW_DTR0 set
    feed(8, 0x01, 0xFE, 0x05);   // 8 B with QW_DTR0 clear
    feed(0, 0x01, 0xFE, 0x05);

    cq_counters_t after = *cq_counters();
    uint8_t head_after[2];
    cq_head(head_after);
    CHECK(memcmp(&before, &after, sizeof before) == 0);
    CHECK(head_after[0] == head_before[0] && head_after[1] == head_before[1]);
    CHECK(head_after[0] != 0);   // the head really was there to be disturbed

    // ...and a WELL-formed frame through the same path does change things, so
    // the check above is not passing merely because feed() does nothing.
    feed(5, 0x01, 0, 0);         // ack seq 1, no enqueue
    cq_head(head_after);
    CHECK(head_after[0] == 0);                       // head 1 popped
    CHECK(cq_counters()->acks == before.acks + 1u);
    feed(8, 0x40, 0xFE, 0x05);   // enqueue with preload
    CHECK(cq_counters()->accepts == before.accepts + 1u);
}

int main(void) {
    test_full_cycle();
    test_failed_result();
    test_seq_wrap();
    test_cmdq_full();
    test_fifo_order();
    test_ack_semantics();
    test_backpressure();
    test_hook_guards();
    test_reserved_invariants();
    test_fc41_validation();
    test_discard_changes_nothing();

    if (!fail) printf("PASS: all cq checks passed\n");
    return fail;
}
