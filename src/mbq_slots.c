// mbq_slots.c — generic self-describing slot command queue (task 06, queue v2).
//
// A DALI-AGNOSTIC Modbus command-queue: 8 slots, each self-describing its own
// lifecycle. The queue carries an OPAQUE 4-byte command in and reports an 8-bit
// response + lifecycle status out. It knows nothing about DALI — the payload's
// meaning (the 4 command bytes, the response8 values, the failure codes) lives
// entirely in the caller (the DALI issuance adapter in dali_sync.c). See
// controller/docs/modbus-command-queue.md for the protocol.
//
// Threading: mbq_submit / mbq_read_status / mbq_next_pending / mbq_begin run in
// main (Modbus + work-arbiter) context; mbq_complete runs in the LVL1 DALI
// completion (ISR) context. The single load-bearing ordering is that a terminal
// slot's response8 is written BEFORE its status, and a status read is atomic
// against the completion writer — so a fenced reader never sees status=complete/
// failed with a stale response8 (the v2 analog of v1's ring-before-completed_id).
// Because each slot is one self-describing 16-bit register, cross-slot tears are
// benign (slots reconcile independently), so only the in-register order + a fenced
// read matter. Compound/main-context accesses use ATOMIC_BLOCK (a brief global
// cli, which also blocks the DALI completion ISR); single-byte status reads are
// naturally atomic on the AVR.
//
// State is kept as separate byte arrays (structure-of-arrays), not an array of
// structs: on the 8-bit AVR a byte-array index is a cheap add, whereas a 7-byte
// struct stride costs a multiply on every access — the same power-of-two
// discipline v1's 4-/2-byte records used.

// Size-optimise this module: it is not on any hot path (called a few times per
// ~28 ms DALI transaction, never in a bit-timing ISR), and the ATtiny1604 flash
// is tight — -Os here reclaims the -O3 code-gen bloat without any latency cost.
#pragma GCC optimize("Os")

#include "mbq_slots.h"
#include <util/atomic.h>

static volatile uint8_t s_cmd[MBQ_SLOTS][MBQ_CMD_BYTES];  // opaque command bytes, wire order
static volatile uint8_t s_status[MBQ_SLOTS];              // MBQ_S_* lifecycle
static volatile uint8_t s_resp[MBQ_SLOTS];               // value on COMPLETE / code on FAILED
static volatile uint8_t s_seq[MBQ_SLOTS];                // accept sequence (valid when PENDING)
static volatile uint8_t s_seq_ctr;                       // monotonic accept-sequence counter

void mbq_init(void) {
    for (uint8_t i = 0; i < MBQ_SLOTS; i++) {
        for (uint8_t b = 0; b < MBQ_CMD_BYTES; b++) s_cmd[i][b] = 0;
        s_status[i] = MBQ_S_IDLE;
        s_resp[i]   = 0;
        s_seq[i]    = 0;
    }
    s_seq_ctr = 0;
}

static inline bool slot_active(uint8_t st) {
    return st == MBQ_S_PENDING || st == MBQ_S_EXECUTING;
}

bool mbq_submit(uint8_t first_slot, const uint8_t *regs, uint8_t n_slots) {
    if (n_slots == 0u || (uint16_t)first_slot + n_slots > MBQ_SLOTS) return false;
    bool ok = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // All-or-nothing: refuse the whole write if ANY targeted slot is active
        // (pending/executing) — a stale-cache write must not clobber in-flight work.
        bool any_active = false;
        for (uint8_t i = 0; i < n_slots; i++) {
            if (slot_active(s_status[first_slot + i])) { any_active = true; break; }
        }
        if (!any_active) {
            // Arm each targeted slot to PENDING with the opaque command and a fresh
            // accept sequence, stamped in ascending slot order so execution order ==
            // the master's intended submit order for a multi-slot write.
            for (uint8_t i = 0; i < n_slots; i++) {
                uint8_t s = (uint8_t)(first_slot + i);
                const uint8_t *r = regs + (uint16_t)i * MBQ_CMD_BYTES;  // 2 regs = 4 bytes
                for (uint8_t b = 0; b < MBQ_CMD_BYTES; b++) s_cmd[s][b] = r[b];
                s_resp[s]   = 0;
                s_seq[s]    = s_seq_ctr++;
                s_status[s] = MBQ_S_PENDING;
            }
            ok = true;
        }
    }
    return ok;
}

void mbq_read_status(uint8_t sidx, uint8_t count, uint8_t *out) {
    // One 16-bit register per slot: [status:8 | response8:8]. Fenced so a single
    // slot's status+response8 are read atomically against the completion writer.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t i = 0; i < count; i++) {
            uint8_t s = (uint8_t)(sidx + i);
            uint8_t st = 0, r8 = 0;
            if (s < MBQ_SLOTS) { st = s_status[s]; r8 = s_resp[s]; }
            out[(uint16_t)i * 2u]      = st;   // hi byte = status
            out[(uint16_t)i * 2u + 1u] = r8;   // lo byte = response8
        }
    }
}

uint8_t mbq_next_pending(void) {
    // Lowest accept-sequence PENDING slot; wrap-aware (modular) compare over the
    // ≤8 live window. Only main mutates PENDING slots, so no fence needed (single-
    // byte reads are atomic; the ISR only touches the one EXECUTING slot).
    uint8_t best = MBQ_NONE;
    uint8_t best_seq = 0;
    for (uint8_t i = 0; i < MBQ_SLOTS; i++) {
        if (s_status[i] != MBQ_S_PENDING) continue;
        uint8_t sq = s_seq[i];
        if (best == MBQ_NONE || (int8_t)(sq - best_seq) < 0) { best = i; best_seq = sq; }
    }
    return best;
}

void mbq_begin(uint8_t slot, mbq_cmd_t *out) {
    if (slot >= MBQ_SLOTS) return;
    for (uint8_t b = 0; b < MBQ_CMD_BYTES; b++) out->b[b] = s_cmd[slot][b];
    s_status[slot] = MBQ_S_EXECUTING;   // single-byte store, atomic
}

void mbq_complete(uint8_t slot, uint8_t status, uint8_t response8) {
    if (slot >= MBQ_SLOTS) return;
    // ORDERING (load-bearing): publish the value BEFORE publishing that it is ready.
    // A fenced reader (mbq_read_status) therefore never sees a terminal status with
    // a stale response8.
    s_resp[slot]   = response8;
    s_status[slot] = status;   // MBQ_S_COMPLETE or MBQ_S_FAILED
}
