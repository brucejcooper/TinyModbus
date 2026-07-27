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

#include "mbq_slots.h"
#include <util/atomic.h>

// Per-slot state. cmd[] is the opaque wire-order command (never interpreted here).
// seq is the accept sequence, meaningful only while status == PENDING.
typedef struct {
    uint8_t cmd[MBQ_CMD_BYTES];  // opaque command bytes, wire order
    uint8_t status;              // MBQ_S_* lifecycle
    uint8_t response8;           // value on COMPLETE / code on FAILED
    uint8_t seq;                 // accept sequence (wrap-aware; valid when PENDING)
} mbq_slot_t;

static volatile mbq_slot_t s_slot[MBQ_SLOTS];
static volatile uint8_t    s_seq;   // monotonic accept-sequence counter

void mbq_init(void) {
    for (uint8_t i = 0; i < MBQ_SLOTS; i++) {
        for (uint8_t b = 0; b < MBQ_CMD_BYTES; b++) s_slot[i].cmd[b] = 0;
        s_slot[i].status    = MBQ_S_IDLE;
        s_slot[i].response8 = 0;
        s_slot[i].seq       = 0;
    }
    s_seq = 0;
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
            if (slot_active(s_slot[first_slot + i].status)) { any_active = true; break; }
        }
        if (!any_active) {
            // Arm each targeted slot to PENDING with the opaque command and a fresh
            // accept sequence, stamped in ascending slot order so execution order ==
            // the master's intended submit order for a multi-slot write.
            for (uint8_t i = 0; i < n_slots; i++) {
                volatile mbq_slot_t *sl = &s_slot[first_slot + i];
                const uint8_t *r = regs + (uint16_t)i * (MBQ_CMD_BYTES);  // 2 regs = 4 bytes
                for (uint8_t b = 0; b < MBQ_CMD_BYTES; b++) sl->cmd[b] = r[b];
                sl->response8 = 0;
                sl->seq       = s_seq++;
                sl->status    = MBQ_S_PENDING;
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
            if (s < MBQ_SLOTS) { st = s_slot[s].status; r8 = s_slot[s].response8; }
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
        if (s_slot[i].status != MBQ_S_PENDING) continue;
        uint8_t sq = s_slot[i].seq;
        if (best == MBQ_NONE || (int8_t)(sq - best_seq) < 0) { best = i; best_seq = sq; }
    }
    return best;
}

void mbq_begin(uint8_t slot, uint8_t *out4) {
    if (slot >= MBQ_SLOTS) return;
    for (uint8_t b = 0; b < MBQ_CMD_BYTES; b++) out4[b] = s_slot[slot].cmd[b];
    s_slot[slot].status = MBQ_S_EXECUTING;   // single-byte store, atomic
}

void mbq_complete(uint8_t slot, uint8_t status, uint8_t response8) {
    if (slot >= MBQ_SLOTS) return;
    // ORDERING (load-bearing): publish the value BEFORE publishing that it is ready.
    // A fenced reader (mbq_read_status) therefore never sees a terminal status with
    // a stale response8.
    s_slot[slot].response8 = response8;
    s_slot[slot].status    = status;   // MBQ_S_COMPLETE or MBQ_S_FAILED
}
