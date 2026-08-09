// cq.c — the v4 command queue and response queue. See cq.h for the model, the
// single-writer concurrency rule, and why there are no interrupt fences here.
//
// Size-optimise this module: it is not on any hot path (called a few times per
// ~28 ms DALI transaction, never in a bit-timing ISR), and the ATtiny1604 flash
// is tight — -Os here reclaims the -O3 code-gen bloat at no latency cost.
#pragma GCC optimize("Os")

#include "cq.h"

_Static_assert((RESPQ_DEPTH & (RESPQ_DEPTH - 1u)) == 0u && RESPQ_DEPTH <= 128u,
               "RESPQ_DEPTH must be a power of two <= 128: the index is a mask "
               "of a free-running 8-bit counter, and 256 must divide the depth "
               "or the counter's wrap would skip an entry");
_Static_assert(CMDQ_DEPTH >= 1u && CMDQ_DEPTH <= 8u,
               "CMDQ_DEPTH is the in-flight cap, not a buffer size");

typedef struct {
    uint8_t cmd[CQ_CMD_BYTES];
    uint8_t seq;      // 1..31, the command's whole identity
    uint8_t state;    // CQ_FREE / CQ_PENDING / CQ_EXECUTING
} cq_entry_t;

typedef struct {
    uint8_t hdr;      // [hstatus:3 | hseq:5]
    uint8_t resp8;    // answer byte on H_OK, failure code on H_FAILED
} rq_entry_t;

static volatile cq_entry_t s_cmdq[CMDQ_DEPTH];
static volatile rq_entry_t s_respq[RESPQ_DEPTH];

// Command queue ordering. Execution is strict FIFO in accept order, and it is
// expressed as a ring cursor rather than a lowest-seq scan so that seq wrap
// (31 -> 1) needs no wrap-aware compare — the one place an off-by-one would
// silently reorder two commands.
//
// Both are MAIN-ONLY. The ISR frees a slot; main is what reclaims it (cq_reap),
// so no count is ever incremented from two contexts.
static uint8_t s_head;   // oldest live command
static uint8_t s_live;   // live commands; main's view, so it can lag the ISR by
                         // one freed slot until the next cq_reap()

// Response queue. Two FREE-RUNNING 8-bit counters, each with exactly one writer;
// occupancy is their difference and the index is their low bits. See cq.h for
// why this shape and not head+count.
static volatile uint8_t s_rq_pushed;   // ISR writes
static uint8_t          s_rq_popped;   // main writes

static uint8_t s_seq;    // last assigned seq; main-only

// Rising-edge latch for the stall counter. Main-only.
//
// Without it, `stalls` counts POLLS, not stalls: the main loop calls
// start_next_dali_work() -> cq_next() free-running, and while a stall is in
// progress neither of that function's early exits fires (nothing is executing
// and the bus is idle), so every loop pass lands on the refusal — measured at
// roughly 18 000 increments per second, wrapping the 16-bit counter about every
// 3.6 s. The counter would be non-zero, so the backpressure validation gate
// would appear to pass, while the value itself said nothing: you could not tell
// one stall episode from fifty, and two reads a second apart would disagree.
// That is the task-07 failure shape exactly — correct behaviour, useless
// evidence, hidden behind a check that only asks "is it non-zero".
static bool s_stalled;

static volatile cq_counters_t s_ctr;

static inline uint8_t ring_next(uint8_t i, uint8_t depth) {
    return (uint8_t)((i + 1u == depth) ? 0u : i + 1u);
}

// Live entries in the response queue. Reads the ISR-written counter once; a
// concurrent push makes the answer stale-LOW, which is the safe direction for
// every caller (an entry looks absent for one transaction, never present-but-
// unwritten).
static inline uint8_t rq_count(void) {
    return (uint8_t)(s_rq_pushed - s_rq_popped);
}

void cq_init(void) {
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++) {
        for (uint8_t b = 0; b < CQ_CMD_BYTES; b++) s_cmdq[i].cmd[b] = 0;
        s_cmdq[i].seq   = CQ_SEQ_NONE;
        s_cmdq[i].state = CQ_FREE;
    }
    for (uint8_t i = 0; i < RESPQ_DEPTH; i++) { s_respq[i].hdr = 0; s_respq[i].resp8 = 0; }
    s_head = 0; s_live = 0;
    s_rq_pushed = 0; s_rq_popped = 0;
    s_seq = CQ_SEQ_NONE;
    s_stalled = false;
    s_ctr.accepts = 0; s_ctr.full = 0; s_ctr.executes = 0;
    s_ctr.results = 0; s_ctr.acks = 0; s_ctr.stalls = 0;
}

// Reclaim command slots the completion ISR has freed. Main-context only, and
// called from BOTH main-side entry points (accept and next) rather than just the
// arbiter: an enqueue that arrives before the arbiter has run again would
// otherwise be told A_FULL over a slot that is already free, costing a
// transaction per DALI frame at saturation — the exact cost this protocol exists
// to remove.
//
// Only the head can be EXECUTING (one command runs at a time), so a FREE head
// means that command completed and the next one becomes the head. The loop is
// bounded by s_live and terminates on the first still-live entry.
static void cq_reap(void) {
    while (s_live && s_cmdq[s_head].state == CQ_FREE) {
        s_head = ring_next(s_head, CMDQ_DEPTH);
        s_live--;
    }
}

// Next sequence, 1..31, SKIPPING 0 on wrap. 0 is the reserved "none" on the
// wire, so an off-by-one that let it through would make "no ack requested" and
// "ack command 32" the same frame.
static uint8_t next_seq(void) {
    s_seq = (uint8_t)((s_seq >= CQ_SEQ_MAX) ? 1u : s_seq + 1u);
    return s_seq;
}

uint8_t cq_accept(const uint8_t cmd[CQ_CMD_BYTES]) {
    cq_reap();
    if (s_live >= CMDQ_DEPTH) { s_ctr.full++; return CQ_SEQ_NONE; }

    uint8_t idx = s_head;
    for (uint8_t i = 0; i < s_live; i++) idx = ring_next(idx, CMDQ_DEPTH);
    // The ring cursor says this slot is free; the state byte is the truth. If
    // they disagree an invariant has broken, and reporting A_FULL is the honest
    // answer — the master paces and retries, which is survivable, where arming a
    // slot that still holds a live command is not.
    if (s_cmdq[idx].state != CQ_FREE) { s_ctr.full++; return CQ_SEQ_NONE; }

    uint8_t seq = next_seq();
    // PUBLICATION ORDER: the payload and the identity first, the state that
    // publishes them to the arbiter last. Reversing these two lets the arbiter
    // launch a slot whose command bytes are still the previous occupant's.
    for (uint8_t b = 0; b < CQ_CMD_BYTES; b++) s_cmdq[idx].cmd[b] = cmd[b];
    s_cmdq[idx].seq   = seq;
    s_cmdq[idx].state = CQ_PENDING;
    s_live++;
    s_ctr.accepts++;
    return seq;
}

bool cq_ack(uint8_t seq) {
    if (seq == CQ_SEQ_NONE || rq_count() == 0u) return false;
    if ((uint8_t)(s_respq[s_rq_popped & (RESPQ_DEPTH - 1u)].hdr & CQ_SEQ_MAX) != seq)
        return false;
    s_rq_popped++;
    s_ctr.acks++;
    return true;
}

void cq_head(uint8_t out[2]) {
    // Read the count BEFORE the entry — the other half of the completion path's
    // bytes-then-publish. An entry observed as present was fully written before
    // the counter that revealed it.
    if (rq_count() == 0u) { out[0] = 0; out[1] = 0; return; }
    uint8_t i = (uint8_t)(s_rq_popped & (RESPQ_DEPTH - 1u));
    out[0] = s_respq[i].hdr;
    out[1] = s_respq[i].resp8;
}

uint8_t cq_next(void) {
    cq_reap();
    if (s_live == 0u || s_cmdq[s_head].state != CQ_PENDING) { s_stalled = false; return CQ_NONE; }
    // BACKPRESSURE. Stall DALI rather than start work whose result would have
    // nowhere to go — retention is the guarantee the whole recovery design rests
    // on, so a dropped result is the one outcome not worth any throughput.
    //
    // The counter takes the RISING EDGE ONLY, so it reads as "times backpressure
    // engaged" rather than "times we were asked while stalled" — see s_stalled.
    // Every non-stall outcome clears the latch, including "nothing pending", so
    // a later episode is counted as its own even if the queue drained in between.
    if (rq_count() >= RESPQ_DEPTH) {
        if (!s_stalled) { s_stalled = true; s_ctr.stalls++; }
        return CQ_NONE;
    }
    s_stalled = false;
    return s_head;
}

tm_err_t cq_begin(uint8_t idx, cq_cmd_t *out_cmd, uint8_t *out_seq) {
    // `idx != s_head` is what makes "only one command executes at a time" an
    // invariant of THIS MODULE rather than an assumption about its caller — and
    // that invariant is load-bearing twice over: it is what makes execution
    // strict FIFO in accept order, and it is what makes cq_complete's
    // TM_ERR_BUSY unconstructible (one executing command means the response
    // entry cq_next reserved is still the only one that can be consumed). Test
    // it before the PENDING check so a caller passing its own index is refused
    // even when that slot happens to be legitimately pending.
    if (idx >= CMDQ_DEPTH || idx != s_head || s_cmdq[idx].state != CQ_PENDING) {
        for (uint8_t b = 0; b < CQ_CMD_BYTES; b++) out_cmd->b[b] = 0;
        *out_seq = CQ_SEQ_NONE;
        return TM_ERR_BAD_SLOT;
    }
    for (uint8_t b = 0; b < CQ_CMD_BYTES; b++) out_cmd->b[b] = s_cmdq[idx].cmd[b];
    *out_seq = s_cmdq[idx].seq;
    s_cmdq[idx].state = CQ_EXECUTING;   // single-byte store, atomic
    s_ctr.executes++;
    return TM_OK;
}

tm_err_t cq_complete(uint8_t seq, uint8_t hstatus, uint8_t resp8) {
    // Find the command by identity, not by index: the caller (an ISR) holds the
    // seq it was given at cq_begin, and matching on it means a completion can
    // never be attributed to a slot that has since been recycled.
    uint8_t idx = CQ_NONE;
    for (uint8_t i = 0; i < CMDQ_DEPTH; i++)
        if (s_cmdq[i].state == CQ_EXECUTING && s_cmdq[i].seq == seq) { idx = i; break; }
    if (idx == CQ_NONE) return TM_ERR_BAD_SLOT;

    if (rq_count() >= RESPQ_DEPTH) {
        // Unconstructible while backpressure holds (see cq.h). Free the slot
        // anyway: losing this result costs the master one deadline expiry, where
        // a slot stuck EXECUTING costs it the queue until the gateway reboots.
        s_cmdq[idx].state = CQ_FREE;
        return TM_ERR_BUSY;
    }

    // PUBLICATION ORDER: both bytes of the entry, THEN the counter that reveals
    // it. cq_head reads the counter first, so it can never serialise an entry
    // whose bytes are still the previous occupant's.
    uint8_t w = (uint8_t)(s_rq_pushed & (RESPQ_DEPTH - 1u));
    s_respq[w].hdr   = (uint8_t)(((hstatus & 0x07u) << 5) | (seq & CQ_SEQ_MAX));
    s_respq[w].resp8 = resp8;
    s_rq_pushed++;
    s_ctr.results++;

    // Free the command slot only after the result is published, so the two are
    // never both invisible: main reclaims the slot on its next reap and the
    // result is already collectable when it does.
    s_cmdq[idx].state = CQ_FREE;
    return TM_OK;
}

const volatile cq_counters_t *cq_counters(void) { return &s_ctr; }
