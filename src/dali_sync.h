#ifndef DALI_SYNC_H
#define DALI_SYNC_H

/*
 * dali_sync.h — DALI issuance for the command queue.
 *
 * STATELESS PIPE. This file is the DALI adapter for the generic slot queue: a
 * sync-flag work selector, a per-ballast arc/status cache, presence and dirty
 * bitmaps, group masks with ADD/REMOVE fan-out, a periodic rescan cursor, and
 * the IEC 62386-102 commissioning state machine. ALL OF IT IS GONE.
 *
 * The controller owns every one of those concerns now: it learns the bus by
 * querying the ballasts, holds the single authoritative cache, and drives
 * commissioning itself as ordinary queued frames. What remains here is the DALI
 * ADAPTER for the generic slot queue — take the next pending slot, unpack its
 * opaque 4-byte command into DALI frames, put them on the wire, map the outcome
 * back to (status, response8).
 *
 * The gateway therefore holds NO state a reboot could lose, which is what makes
 * gateway reboots consequence-free and made the original group-toggle failure
 * structurally impossible rather than merely mitigated.
 */

#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>

#include "dali.h"
#include "cq.h"

// ---- DALI payload mapping (the ONLY DALI-specific part of the queue) -------
//
// How the queue's opaque 32-bit command is interpreted. This is the INTERNAL
// command form, not the wire form: the FC 0x41 request packs its flags into the
// top 3 bits of a byte shared with the ack sequence, and modbus.c normalises
// them into the QF_* bits below before the command ever reaches the queue. That
// boundary is deliberate — see queue_flags_to_qf() — and it is why nothing in
// this file changed when the wire packing did.
//
//   byte 0  addr   DALI address byte — short address, group, broadcast (0xFE),
//                  or a special command opcode (e.g. 0xA3 SET DTR0, or the
//                  commissioning specials the controller drives)
//   byte 1  data   DALI data byte
//   byte 2  flags  QF_* below
//   byte 3  dtr0   value preloaded into DTR0 when QF_DTR0 is set
typedef struct {
    uint8_t addr;
    uint8_t data;
    uint8_t flags;
    uint8_t dtr0;
} dali_frame_t;

#define QF_TWICE   0x01u   // send the forward frame twice (SET-class), no answer
#define QF_DTR0    0x02u   // emit 0xA3<dtr0> back-to-back BEFORE the frame
#define QF_ANSWER  0x04u   // expects a backward frame; capture the answer byte

// Unpack a queued command's opaque bytes into the DALI frame they describe.
static inline dali_frame_t dali_frame_of(const cq_cmd_t *c) {
    return (dali_frame_t){ .addr  = c->b[0], .data = c->b[1],
                           .flags = c->b[2], .dtr0 = c->b[3] };
}

// ---- DALI failure codes carried in an H_FAILED result's resp8 --------------
// (an H_OK result's resp8 is the backward-frame answer instead; H_NAK's is 0)
//
// 0x01 was DALI_RESP_NAK, retired 2026-08-13. "A query got silence" is now the
// STATUS H_NAK, not a failure code, so keeping a failure code that says the
// same thing would reintroduce exactly the ambiguity H_NAK removes — two
// encodings for one outcome, differing only in whether the reader checks the
// status or the payload. The value stays reserved rather than reused so a
// mismatched old master decodes an unknown code instead of a wrong one.
#define DALI_RESP_COLLISION  0x02u  // garbled backward frame (>=1 gear answered)
#define DALI_RESP_BUSFAULT   0x03u  // stuck/shorted bus (dali_sync_abort_busfault); a garbled reply of any kind is COLLISION

// True while this gateway is running a queued command — i.e. one entry is
// CQ_EXECUTING and we hold the wire for it, across the whole multi-frame
// sequence including the quiet inter-frame gaps.
//
// Derived from the slot handle we need anyway, not a separate flag, so it cannot
// disagree with the queue. The bit-level machine asks this to decide "dispatch
// the next step" versus "park" — the one fact about us that "is the bus busy"
// cannot answer, since the bus is quiet between frames of a send-twice.
bool dali_sync_owns_channel(void);

// Where the issuance state lives: the executing command's 4 bytes and its seq,
// plus a step counter inside dali_sync.c. Whether a command is in flight is
// answered by dali_sync_owns_channel() rather than by a flag that could disagree
// with it.

// Reset issuance state. Call once at boot.
void dali_sync_init(void);

// The main loop's entire DALI service call. Starts the next queued command when
// there is one and the bus is free, and returns whether any DALI work is
// OUTSTANDING — a command in flight, a command queued, or a bus that is not yet
// free. Returns false, silently, only when there is genuinely nothing to do.
//
// That return is the caller's sleep interlock, and it costs nothing extra: the
// two conditions that mean "outstanding" without any queued command — we own one
// already, or the bus is mid-settle — are the same two early exits that skip the
// slot scan. So one pass answers both questions where the previous shape scanned
// the slot array twice per loop iteration — once here and again in a separate
// mbq_has_work() query, since deleted.
//
// Sleeping while this returns true would be wrong for two independent reasons:
// queued work would sit unstarted, and TCA0 — which has no RUNSTDBY bit and so
// stops in standby — may have the 22 TE settle countdown outstanding.
bool start_next_dali_work(void);

// WHERE THE ISSUANCE STATE LIVES: the executing slot's 4-byte command, plus
// this. Nothing else.
//
// Nothing holds a parallel copy of the command. A struct of {kind, addr, data,
// repeat} — populated from the command on every send. Every field of it was
// derivable from the command and the step:
//   addr/data  DTR0 preload sends (0xA3, dtr0); any frame step sends the
//              command's own addr/data bytes.
//   kind       QF_TWICE / QF_ANSWER / neither, and a preload is never a query
//              regardless of what the frame after it is.
//   repeat     a send-twice is two sends by definition; the countdown is just
//              "which of the two is this", i.e. the step.
// Keeping that copy meant the same facts existed twice and could disagree.
// The command is the single source of truth; this says where we are in it.
typedef enum __attribute__((packed)) {
    DALI_STEP_DTR0 = 0,   // 0xA3<dtr0> preload — only for a QF_DTR0 command
    DALI_STEP_FRAME,      // the command frame itself (first send)
    DALI_STEP_FRAME_2,    // its second send — only for a QF_TWICE command
} dali_step_t;

// Re-fire the second send of a QF_TWICE command. Called from
// dali_result_received (ISR) BEFORE the slot is allowed to go terminal; returns
// true if another send went out, false if this command is done sending.
bool dali_sync_repeat_send(void);

// Completion handler — called from dali_result_received (dali_state_idle.c)
// while we own the channel. Runs in ISR context.
void dali_sync_completion(dali_response_type_t type, uint8_t value);

// Dispatch the next step of the in-flight op, called when the bus goes idle.
void dali_sync_dispatch_current(void);

// Fail the in-flight command because the bus is unusable (stuck/shorted). The
// slot is completed with a bus-fault code rather than abandoned, so the
// controller is never left waiting on a command that can never finish.
void dali_sync_abort_busfault(void);

#endif /* DALI_SYNC_H */
