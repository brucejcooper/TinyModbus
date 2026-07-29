#ifndef DALI_SYNC_H
#define DALI_SYNC_H

/*
 * dali_sync.h — DALI issuance for the command queue.
 *
 * TASK 05 — STATELESS PIPE. This file used to be the gateway's brain: a
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
#include "mbq_slots.h"

// ---- DALI payload mapping (the ONLY DALI-specific part of the queue) -------
//
// How the queue's opaque 32-bit command is interpreted. This mapping is the
// contract with the controller's DALI adapter (controller/src/dali_queue.c) and
// MUST match it byte for byte.
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

// Unpack a queue slot's opaque bytes into the DALI frame they describe.
static inline dali_frame_t dali_frame_of(const mbq_cmd_t *c) {
    return (dali_frame_t){ .addr  = c->b[0], .data = c->b[1],
                           .flags = c->b[2], .dtr0 = c->b[3] };
}

// ---- DALI failure codes carried in a FAILED slot's response8 ---------------
// (a COMPLETE slot's response8 is the backward-frame answer instead)
#define DALI_RESP_NAK        0x01u  // no/short backward frame where one was expected
#define DALI_RESP_COLLISION  0x02u  // garbled backward frame (>=1 gear answered)
#define DALI_RESP_BUSFAULT   0x03u  // transport-level fault (short pulse, stuck bus)

// The channel's state (dali_chan / DALI_CHAN_*) is declared in dali.h — it spans
// both layers, so it lives in the lower one that this header already includes.

// The "operation in flight" struct is gone. It held three fields, of which:
//   op    duplicated the busy flag exactly — both were set together and cleared
//         together at every site, so keeping them in agreement across an ISR
//         boundary was a standing hazard with no benefit. That flag has since
//         become DALI_CHAN_EXECUTING (dali.h); the enum here had a single
//         meaningful value (OP_RAW) once the background work selector went in
//         task 05.
//   addr  was written 0xFF at launch and never read by anything.
//   step  is genuinely live and now stands alone inside dali_sync.c (nothing
//         outside that file ever touched it).

// Reset issuance state. Call once at boot.
void dali_sync_init(void);

// If the bus is idle and a slot is pending, begin executing it. Returns true if
// a command was launched. Called from the main loop.
bool start_next_dali_work(void);

// WHERE THE ISSUANCE STATE LIVES: the executing slot's 4-byte command, plus
// this. Nothing else.
//
// There used to be a parallel copy — a struct holding {kind, addr, data,
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
// while the channel is DALI_CHAN_EXECUTING. Runs in ISR context.
void dali_sync_completion(dali_response_type_t type, uint8_t value);

// Dispatch the next step of the in-flight op, called when the bus goes idle.
void dali_sync_dispatch_current(void);

// Fail the in-flight command because the bus is unusable (stuck/shorted). The
// slot is completed with a bus-fault code rather than abandoned, so the
// controller is never left waiting on a command that can never finish.
void dali_sync_abort_busfault(void);

#endif /* DALI_SYNC_H */
