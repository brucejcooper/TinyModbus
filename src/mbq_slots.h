#pragma once
// mbq_slots.h — generic self-describing slot command queue (task 06, queue v2).
//
// A DALI-AGNOSTIC Modbus command-queue. It carries OPAQUE fixed-size commands to
// the master and reports an 8-bit response + a lifecycle status per slot. It has
// NO knowledge of the payload: the meaning of the command bytes, of the response8
// values, and of the failure codes belongs to the caller (the DALI issuance
// adapter). Do not add DALI (or any payload) semantics to this module — the
// generic pattern is documented in controller/docs/modbus-command-queue.md.
//
// Wire ABI (see registers.h for the register addresses):
//   Command block (master-write): 0x1000 + 2*slot, one 32-bit opaque command per
//     slot (2 Modbus registers, wire order = MBQ_CMD_BYTES bytes).
//   Status block (gateway-write, FC03+FC04): 0x1010 + slot, one register per slot
//     = [status:8 | response8:8].

#include <stdint.h>
#include "errors.h"
#include <stdbool.h>

#define MBQ_SLOTS      8u   // number of self-describing slots
#define MBQ_CMD_BYTES  4u   // opaque command size (32-bit)
#define MBQ_CMD_REGS   2u   // ...expressed in Modbus REGISTERS

// A slot's opaque command. Deliberately a byte blob: this layer carries it and
// does not interpret it. The wrapper exists for TYPE SAFETY, not meaning — it
// stops a bare uint8_t* from being confused with a register pointer or a
// wrongly-sized buffer, which is precisely the mix-up that once made a submit
// declare twice its length and arm the following slot with adjacent memory.
typedef struct { uint8_t b[MBQ_CMD_BYTES]; } mbq_cmd_t;

// A slot's status register exactly as it appears on the wire:
// [status:8 | response8:8]. This is the ONE definition of that pair — mbq_slot_t
// embeds it rather than repeating the fields, so the in-memory layout and the
// wire layout cannot drift apart.
typedef struct {
    uint8_t status;      // MBQ_S_*
    uint8_t response8;   // value on COMPLETE, code on FAILED, else 0
} mbq_slot_status_t;

// ONE QUEUE ENTRY. Everything the queue knows about a command lives here.
//
// `seq` is the load-bearing field and is deliberately NOT part of the command:
// it is stamped by the GATEWAY when a slot goes idle->pending, and execution
// order is lowest-seq-first, wrap-aware. That is why slot INDEX does not affect
// ordering — writing slots 6, 7, 0 executes 6, 7, 0. Keeping the sequence
// gateway-side is what removed v1's controller-managed id ledger, and with it
// the whole class of reboot/glitch recovery-corruption bugs.
//
// Padded to 8 bytes so indexing is a shift rather than a multiply on the 8-bit
// AVR — the reason this was previously four parallel arrays. The static assert
// keeps that property from being lost by a future field.
typedef struct {
    mbq_cmd_t         cmd;   // opaque payload; the queue never interprets it
    mbq_slot_status_t st;    // exactly what a status read returns
    uint8_t           seq;   // accept sequence, valid while PENDING
    uint8_t           _pad;  // keep sizeof == 8 (power-of-two stride)
} mbq_slot_t;
_Static_assert(sizeof(mbq_slot_t) == 8u,
               "mbq_slot_t must stay a power-of-two stride for cheap AVR indexing");

// Lifecycle status (the hi byte of a slot's status register).
enum {
    MBQ_S_IDLE      = 0u,   // free; writable
    MBQ_S_PENDING   = 1u,   // accepted, waiting to execute
    MBQ_S_EXECUTING = 2u,   // in flight
    MBQ_S_COMPLETE  = 3u,   // terminal; response8 = value
    MBQ_S_FAILED    = 4u,   // terminal; response8 = code
};

#define MBQ_NONE  0xFFu   // "no slot" sentinel from mbq_next_pending

// Reset all slots to idle and the accept sequence to 0. Call once at boot; a
// reboot naturally leaves every slot idle (the "idle means gone" signal to the
// master). No cross-reboot state is kept.
void mbq_init(void);

// Modbus submit: arm `n_slots` contiguous slots starting at `first_slot` from
// `regs` (n_slots * 2 big-endian registers, wire order). ALL-OR-NOTHING: if ANY
// targeted slot is active (pending/executing) nothing changes and this returns
// false (the caller raises SLAVE_DEVICE_BUSY 0x06). Otherwise each targeted slot
// is armed to PENDING with the opaque command and a fresh accept sequence stamped
// in ascending slot order. Fenced.
//   TM_OK           armed
//   TM_ERR_BAD_RANGE  n_slots == 0, or the window runs past the last slot
//   TM_ERR_BUSY       a targeted slot is active — nothing was changed
// The two failures are distinct on purpose: BUSY is a retry-later, BAD_RANGE is a
// malformed request. Collapsing both to "false" made a malformed submit report
// DEVICE_BUSY, inviting the master to retry a request that could never succeed.
tm_err_t mbq_submit(uint8_t first_slot, const uint8_t *regs, uint8_t n_slots);

// Serve `count` status registers starting at slot index `sidx` into `out`
// (count * 2 big-endian bytes, [status:8 | response8:8] per slot). Fenced so each
// slot's status+response8 pair is atomic against the completion writer.
//   TM_OK             all `count` slots served
//   TM_ERR_BAD_RANGE  the window overran the slot array
// On BAD_RANGE the out-of-range entries are still written as zero, so `out` is
// never left holding whatever the caller's buffer had before — a partially built
// response must not carry stale bytes onto the wire.
tm_err_t mbq_read_status(uint8_t sidx, uint8_t count, uint8_t *out);

// Arbiter hook: the lowest accept-sequence PENDING slot (0..MBQ_SLOTS-1), or
// MBQ_NONE if none is pending. Wrap-aware over the small live window.
uint8_t mbq_next_pending(void);

// Arbiter hook: copy `slot`'s opaque command (MBQ_CMD_BYTES) into `out` and mark
// the slot EXECUTING. The caller (payload adapter) interprets the bytes.
//   TM_OK            *out holds the command
//   TM_ERR_BAD_SLOT  slot >= MBQ_SLOTS; *out is ZEROED, never left untouched
// The zeroing matters: this used to return silently without writing *out, so a
// caller that ignored the failure would re-fire whatever command the buffer still
// held — a repeat of the previous DALI frame, which for a non-idempotent UP/DOWN
// step is exactly the double-apply the protocol works to prevent.
tm_err_t mbq_begin(uint8_t slot, mbq_cmd_t *out);

// Completion hook (called from the payload adapter's completion path): set the
// slot terminal. `status` is MBQ_S_COMPLETE or MBQ_S_FAILED; `response8` is the
// value or code. Writes response8 before status (the load-bearing ordering).
//   TM_OK            slot is terminal
//   TM_ERR_BAD_SLOT  slot >= MBQ_SLOTS — the slot stays EXECUTING FOREVER
// That failure is unrecoverable from here (this runs in the DALI completion ISR)
// and strands a queue slot until reboot, so callers should hand it to tm_fault()
// rather than drop it. Dropping it is how a stuck slot becomes a mystery.
tm_err_t mbq_complete(uint8_t slot, uint8_t status, uint8_t response8);
