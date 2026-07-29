// mbq_slots.c — generic self-describing slot command queue (task 06, queue v2).
//
// A DALI-AGNOSTIC Modbus command-queue: 8 slots, each self-describing its own
// lifecycle. The queue carries an OPAQUE 4-byte command in and reports an 8-bit
// response + lifecycle status out. It knows nothing about DALI — the payload's
// meaning (the 4 command bytes, the response8 values, the failure codes) lives
// entirely in the caller (the DALI issuance adapter in dali_sync.c). See
// controller/docs/modbus-command-queue.md for the protocol.
//
// Threading: mbq_submit / mbq_read_status / mbq_next_pending / mbq_begin all run
// in main (Modbus + work-arbiter) context. The ONLY concurrent writer is
// mbq_complete, in the LVL1 DALI completion ISR, and it touches only the one
// EXECUTING slot.
//
// THERE ARE NO INTERRUPT FENCES IN THIS FILE, deliberately. The write ORDER is
// what carries the invariant:
//
//   mbq_complete writes response8, THEN status.
//   mbq_read_status reads status,  THEN response8.
//
// So if a reader observes a terminal status, the ISR had already written status —
// which happens after response8 — and the response8 it goes on to read is
// therefore fresh. The other interleaving gives a stale non-terminal status with
// a fresh response8, which is harmless: response8 is defined only when the status
// is terminal. Each byte is a single AVR store/load, so neither can tear, and
// `volatile` stops the compiler reordering the pair.
//
// Cross-slot tears are benign too — every slot is self-describing and reconciles
// independently — so a multi-slot read needs no snapshot either.
//
// mbq_submit needs none for a different reason: mbq_complete can only move the
// executing slot EXECUTING -> terminal, which can never make a slot that read as
// free become active. It cannot invalidate an all-or-nothing "none active" check.
//
// This is not a micro-optimisation. A global cli here blocks the DALI bit-timing
// callbacks and the USART RX ISR, and an over-long ATOMIC_BLOCK on a hot path is
// precisely what caused the 2026-06-05 RX starvation bug (see CLAUDE.md): a scan
// held interrupts off for hundreds of µs and overran the 2-byte RX FIFO. Do not
// add a fence here without a concrete race the write order does not already
// cover.
//
// State is an array of mbq_slot_t — one struct per queue entry. It was four
// parallel byte arrays, to avoid a multiply on every access from a 7-byte
// stride; padding the struct to 8 bytes buys the same cheap indexing while
// letting a queue entry be an actual type. Measured: no flash cost.

// Size-optimise this module: it is not on any hot path (called a few times per
// ~28 ms DALI transaction, never in a bit-timing ISR), and the ATtiny1604 flash
// is tight — -Os here reclaims the -O3 code-gen bloat without any latency cost.
#pragma GCC optimize("Os")

#include "mbq_slots.h"

static volatile mbq_slot_t s_slot[MBQ_SLOTS];
static volatile uint8_t    s_seq_ctr;   // monotonic accept-sequence counter

void mbq_init(void) {
    for (uint8_t i = 0; i < MBQ_SLOTS; i++) {
        for (uint8_t b = 0; b < MBQ_CMD_BYTES; b++) s_slot[i].cmd.b[b] = 0;
        s_slot[i].st.status    = MBQ_S_IDLE;
        s_slot[i].st.response8 = 0;
        s_slot[i].seq       = 0;
    }
    s_seq_ctr = 0;
}

static inline bool slot_active(uint8_t st) {
    return st == MBQ_S_PENDING || st == MBQ_S_EXECUTING;
}

tm_err_t mbq_submit(uint8_t first_slot, const uint8_t *regs, uint8_t n_slots) {
    if (n_slots == 0u || (uint16_t)first_slot + n_slots > MBQ_SLOTS) return TM_ERR_BAD_RANGE;
    // All-or-nothing: refuse the whole write if ANY targeted slot is active
    // (pending/executing) — a stale-cache write must not clobber in-flight work.
    // Unfenced: see the file header. The completion ISR can only retire the
    // executing slot, never make a free one active, so it cannot turn this
    // decision stale in the unsafe direction. The reverse — a slot retiring just
    // after we saw it active — costs one spurious DEVICE_BUSY that the master
    // retries, and a fence would not prevent that either, only narrow the window.
    for (uint8_t i = 0; i < n_slots; i++) {
        if (slot_active(s_slot[first_slot + i].st.status)) return TM_ERR_BUSY;
    }
    // Arm each targeted slot to PENDING with the opaque command and a fresh
    // accept sequence, stamped in ascending slot order so execution order ==
    // the master's intended submit order for a multi-slot write.
    for (uint8_t i = 0; i < n_slots; i++) {
        uint8_t s = (uint8_t)(first_slot + i);
        const uint8_t *r = regs + (uint16_t)i * MBQ_CMD_BYTES;  // 2 regs = 4 bytes
        for (uint8_t b = 0; b < MBQ_CMD_BYTES; b++) s_slot[s].cmd.b[b] = r[b];
        s_slot[s].st.response8 = 0;
        s_slot[s].seq          = s_seq_ctr++;
        s_slot[s].st.status    = MBQ_S_PENDING;
    }
    return TM_OK;
}

tm_err_t mbq_read_status(uint8_t sidx, uint8_t count, uint8_t *out) {
    // One 16-bit register per slot: [status:8 | response8:8].
    //
    // Unfenced, and the read order below is load-bearing: status FIRST, then
    // response8. That is what makes a terminal status imply a fresh response8
    // against mbq_complete's response8-then-status write. Swapping these two
    // lines would reintroduce exactly the stale-value race the ordering exists to
    // prevent. See the file header.
    tm_err_t err = TM_OK;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t s = (uint8_t)(sidx + i);
        uint8_t st = 0, r8 = 0;
        // Out-of-range entries are reported AND zeroed — the caller may still
        // put this buffer on the wire, and stale bytes would read as real
        // slot state.
        if (s < MBQ_SLOTS) { st = s_slot[s].st.status; r8 = s_slot[s].st.response8; }
        else               { err = TM_ERR_BAD_RANGE; }
        out[(uint16_t)i * 2u]      = st;   // hi byte = status
        out[(uint16_t)i * 2u + 1u] = r8;   // lo byte = response8
    }
    return err;
}

uint8_t mbq_next_pending(void) {
    // Lowest accept-sequence PENDING slot; wrap-aware (modular) compare over the
    // ≤8 live window. Only main mutates PENDING slots, so no fence needed (single-
    // byte reads are atomic; the ISR only touches the one EXECUTING slot).
    uint8_t best = MBQ_NONE;
    uint8_t best_seq = 0;
    for (uint8_t i = 0; i < MBQ_SLOTS; i++) {
        if (s_slot[i].st.status != MBQ_S_PENDING) continue;
        uint8_t sq = s_slot[i].seq;
        if (best == MBQ_NONE || (int8_t)(sq - best_seq) < 0) { best = i; best_seq = sq; }
    }
    return best;
}

tm_err_t mbq_begin(uint8_t slot, mbq_cmd_t *out) {
    if (slot >= MBQ_SLOTS) {
        // Zero the output rather than leaving it untouched: a caller that ignores
        // this error would otherwise re-fire the command still sitting in *out.
        for (uint8_t b = 0; b < MBQ_CMD_BYTES; b++) out->b[b] = 0;
        return TM_ERR_BAD_SLOT;
    }
    for (uint8_t b = 0; b < MBQ_CMD_BYTES; b++) out->b[b] = s_slot[slot].cmd.b[b];
    s_slot[slot].st.status = MBQ_S_EXECUTING;   // single-byte store, atomic
    return TM_OK;
}

tm_err_t mbq_complete(uint8_t slot, uint8_t status, uint8_t response8) {
    if (slot >= MBQ_SLOTS) return TM_ERR_BAD_SLOT;
    // ORDERING (load-bearing): publish the value BEFORE publishing that it is ready.
    // A fenced reader (mbq_read_status) therefore never sees a terminal status with
    // a stale response8.
    s_slot[slot].st.response8 = response8;
    s_slot[slot].st.status    = status;   // MBQ_S_COMPLETE or MBQ_S_FAILED
    return TM_OK;
}
