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
#include "types.h"

// dali_busy: true while a queued command is executing. Held across a multi-frame
// atomic sequence (DTR0 preload + frame) so nothing can interleave.
extern volatile bool dali_busy;

// The operation in flight. Only one kind remains now that the background work
// selector is gone: a command from the queue.
typedef enum __attribute__((packed)) {
    OP_NONE = 0,
    OP_RAW,          // a slot from the generic command queue (mbq_slots.c)
} dali_op_t;

typedef struct {
    dali_op_t op;
    uint8_t   addr;   // unused for OP_RAW — the command carries its own address
    uint8_t   step;   // 0 = DTR0 preload, 1 = the frame (QF_DTR0 pairs)
} dali_current_op_t;
extern volatile dali_current_op_t current_op;

// Seconds since boot, incremented by the RTC PIT. A pure diagnostic now (FC04
// sysinfo) — nothing schedules off it since the rescan cursor was removed.
extern volatile uint32_t g_uptime_s;

// Reset issuance state. Call once at boot.
void dali_sync_init(void);

// If the bus is idle and a slot is pending, begin executing it. Returns true if
// a command was launched. Called from the main loop.
bool start_next_dali_work(void);

// How a forward frame is transmitted. repeat selects how many hardware sends
// comprise one logical command (1 one-shot, 2 for SET-class / send-twice).
typedef enum __attribute__((packed)) {
    DALI_KIND_DAPC = 0,        // CMD_DIRECT_ARC; bit0 of addr clear
    DALI_KIND_CMD,             // CMD_SEND; no response
    DALI_KIND_TWICE,           // CMD_SEND_TWICE; no response, but 22 TE wait
    DALI_KIND_QUERY,           // CMD_QUERY; backward frame expected
} dali_kind_t;
void issue_dali_command(uint8_t addr_byte, uint8_t data_byte,
                        dali_kind_t kind, uint8_t repeat);

// In-flight command stash, consumed by the bit-level state machine.
extern volatile cmd_entry_t s_active;

// Completion handler — called from dali_result_received (dali_state_idle.c)
// when dali_busy is true. Runs in ISR context.
void dali_sync_completion(dali_response_type_t type, uint8_t value);

// Dispatch the next step of the in-flight op, called when the bus goes idle.
void dali_sync_dispatch_current(void);

// Fail the in-flight command because the bus is unusable (stuck/shorted). The
// slot is completed with a bus-fault code rather than abandoned, so the
// controller is never left waiting on a command that can never finish.
void dali_sync_abort_busfault(void);

#endif /* DALI_SYNC_H */
