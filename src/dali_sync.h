#ifndef DALI_SYNC_H
#define DALI_SYNC_H

#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "dali.h"
#include "types.h"

// ---- Sync flag bits (one byte per ballast in sync_flags[]) -----------
//
// Read bits track "the cache is stale, need fresh DALI read."
// Write bits track "the cache has a value the master set; push it to the ballast."
// Group writes do NOT use these flags - they go through the group_write
// pending-work struct (see below).
//
// The retry-pending bit gates the absent-on-NAK transition: a single NAK
// keeps the read flag set, sets SYNC_RETRY_PENDING, and the work loop
// retries.  A second consecutive NAK transitions the ballast to absent.
#define SYNC_READ_LEVEL    (1u << 0)  // QUERY_STATUS + GET_LEVEL (the live arc level)
#define SYNC_READ_SETTINGS (1u << 1)  // re-fetch group membership (QUERY_GROUPS_0_7/8_15).
                                      // Since the config-bank retirement this is group
                                      // membership only — min/max/power/fail/fade are no
                                      // longer cached on the gateway.  Does NOT read the
                                      // live arc level; pair with SYNC_READ_LEVEL for both.
#define SYNC_WRITE_LEVEL   (1u << 2)  // DAPC (one DALI command)
// bits 3-6 spare (SYNC_WRITE_MINMAX/POWER/FADE retired with the config banks)
#define SYNC_RETRY_PENDING (1u << 7)  // first NAK seen on a level read; one retry before marking absent

// ---- Per-ballast sync flag table -------------------------------------
//
// One byte per ballast, indexed by short address 0..63.
// Both main and ISR contexts touch this table - all read-modify-write
// updates MUST go through sync_flags_set/sync_flags_clear (or be wrapped
// in ATOMIC_BLOCK if the caller needs more complex semantics).
extern volatile uint8_t sync_flags[DALI_MAX_DEVICES];

// ---- Async DALI work tracking ----------------------------------------
//
// dali_busy: true while a DALI command (or multi-step sequence) is in
// flight.  Set by start_next_dali_work, cleared by the completion handler
// when a sequence finishes.
extern volatile bool dali_busy;

// Op classifier for the in-flight DALI work item.  This is a SEPARATE encoding
// from the SYNC_* bits in sync_flags[]: those drive priority selection, this
// drives completion-handler dispatch.  The two map onto each other but are
// distinct (e.g. OP_GROUP_BIT and OP_RAW have no SYNC_* counterpart).
typedef enum __attribute__((packed)) {
    OP_NONE = 0,
    OP_LEVEL_READ,    // SYNC_READ_LEVEL: QUERY_STATUS -> QUERY_ACTUAL_LEVEL
    OP_FULL_SCAN,     // SYNC_READ_SETTINGS: 2 group-membership queries, yields between steps
    OP_WRITE_LEVEL,   // SYNC_WRITE_LEVEL: single DAPC
    OP_GROUP_BIT,     // single ADD_TO_GROUP / REMOVE_FROM_GROUP (twice)
    OP_RAW,           // the executing slot of the generic queue (mbq_slots)
    OP_COMMISSION,    // IEC 62386-102 commissioning state machine (commission_state)
} dali_op_t;

// Phase tracker for the commissioning state machine.  Each value names
// one DALI command; completion advances to the next phase.  Lives in a
// packed 1-byte field of commission_state.
typedef enum __attribute__((packed)) {
    COM_IDLE = 0,
    COM_INITIALISE,                            // INITIALISE x2 (DALI_KIND_TWICE)
    COM_RANDOMISE,                             // RANDOMISE x2  (DALI_KIND_TWICE)
    COM_SEARCH_H, COM_SEARCH_M, COM_SEARCH_L,  // SEARCHADDRH/M/L = search[2/1/0]
    COM_COMPARE,                               // COMPARE; bisect_bit==24 = "any ballast?" probe
    COM_PROGRAM_ADDR,                          // PROGRAM_SHORT_ADDRESS = (next_short_addr<<1)|1
    COM_VERIFY,                                // VERIFY_SHORT_ADDRESS, expects backward 0xFF
    COM_WITHDRAW,                              // WITHDRAW (remove just-isolated gear)
    COM_TERMINATE                              // TERMINATE (final cleanup)
} commission_phase_t;

// What DALI is currently doing, so the completion handler knows how to
// process the response and what (if anything) to start next.
typedef struct {
    uint8_t   addr;  // ballast short address being processed (0xFF for raw / broadcast)
    dali_op_t op;    // see dali_op_t above
    uint8_t   step;  // step within a multi-step sequence (0-based)
} dali_current_op_t;

extern volatile dali_current_op_t current_op;

// ---- Group register write pending state ------------------------------
//
// One Modbus group-register write (0x0110-0x014F) can change up to 16
// membership bits, requiring up to 16 separate DALI commands.  This
// struct tracks the in-progress group write; reg_addr = 0xFFFF means no
// pending write.  Only one group write can be in flight at a time;
// concurrent writes return SLAVE_DEVICE_BUSY.
typedef struct {
    uint16_t reg_addr;          // Modbus register address being processed; 0xFFFF = idle
    uint16_t pending_adds;      // bits where membership add is pending (DALI not sent yet)
    uint16_t pending_removes;   // bits where membership remove is pending (DALI not sent yet)
} dali_group_write_t;

extern volatile dali_group_write_t group_write;

// ---- Commissioning state ---------------------------------------------
//
// Single global state machine driven by the OP_COMMISSION work-selector
// slot (highest priority — see pick_work).  Master writes to MBREG_COMMISSION
// (0x1014) arm or abort the run; reads the same register to poll status.
//
// Layout (12 bytes; packed enum keeps phase to 1 byte):
//   search[0..2]     : current 24-bit SEARCHADDR — search[0]=L, [1]=M, [2]=H.
//                      Indexed by `bit >> 3` so com_{set,clear}_bit are
//                      branch-free.  Initialised to 0xFFFFFF at the start
//                      of every bisect cycle (each ballast).
//   next_short_addr  : short address to assign on the next PROGRAM step.
//                      Full mode advances 0..N-1; incremental scans the
//                      presence bitmap for the lowest 0-bit.
//   bisect_bit       : current bit of the bisect (23..0).  Sentinel 24
//                      means "haven't started bisecting yet — first COMPARE
//                      at 0xFFFFFF probes for any responder at all".
//   count            : ballasts successfully programmed in this run.
//                      Mirrored into the low byte of status while running.
//   err_streak       : consecutive VERIFY-fails.  >3 trips status 0x0303.
//   phase            : commission_phase_t — drives dispatch + completion.
//   mode             : 1 = full (INITIALISE 0x00), 2 = incremental (0xFF).
//   abort_requested  : set by master writing 0x0000.  Picked up at the
//                      next dispatch boundary; transitions phase to
//                      COM_TERMINATE with status 0x0301.
//   resend_pending   : set by COM_COMPARE when a bb==0 NO outcome restores
//                      bit 0 of `search` locally; SEARCH_L's completion
//                      handler clears the flag and routes directly to
//                      COM_PROGRAM_ADDR (skipping the wasted COMPARE round).
//                      Without this, the gear-side SEARCHADDR registers
//                      lag the local search and PROGRAM is issued against
//                      a stale value -> verify-fail.
//   status           : exposed verbatim via FC03 0x1014.
//                        0x0000      idle (post-read or never run)
//                        0x01xx      running, xx = count so far
//                        0x02xx      complete, xx = total programmed (read-clears)
//                        0x03xx      error/aborted, xx = subtype (read-clears)
//
// Threading: written from main only inside dali_sync_arm_commission and
// regs_read_holding (terminal-state clear) under TCB0+ATOMIC.  All other
// updates run from ISR context (dali_sync_completion / dispatch_current);
// 16-bit `status` writes from ISR are LVL1, fenced against main reads
// by main's TCB0=0 around its ATOMIC blocks.
typedef struct {
    uint8_t            search[3];        // [0]=L, [1]=M, [2]=H — indexed by bit/8
    uint8_t            next_short_addr;
    uint8_t            bisect_bit;       // 24 = "first COMPARE" sentinel; 23..0 = active bit
    uint8_t            count;
    uint8_t            err_streak;
    commission_phase_t phase;
    uint8_t            mode;             // 1 = full, 2 = incremental, 3 = re-provision one addr
    uint8_t            reprov_addr;      // mode 3: short address whose gear to re-provision
    bool               abort_requested;
    bool               resend_pending;   // bb==0 NO -> resync SEARCHADDR before PROGRAM
    uint16_t           status;
} commission_state_t;

extern volatile commission_state_t commission_state;

// Commission status word encoding (mirrors the FC03 0x1014 wire format).
#define COMM_STATUS_IDLE             0x0000u
#define COMM_STATUS_RUNNING(c)       ((uint16_t)(0x0100u | (uint8_t)(c)))
#define COMM_STATUS_COMPLETE(c)      ((uint16_t)(0x0200u | (uint8_t)(c)))
#define COMM_STATUS_ERR_ABORTED      0x0301u
#define COMM_STATUS_ERR_NO_BALLASTS  0x0302u
#define COMM_STATUS_ERR_VERIFY_FAIL  0x0303u
#define COMM_STATUS_ERR_ADDR_EXHAUST 0x0304u
#define COMM_STATUS_ERR_BUS_STUCK    0x0305u  // 200 TE bus-shorted timeout aborted the run

// Note: there's no dali_sync_commission_busy() helper analogous to
// dali_sync_group_busy() — DEVICE_BUSY for 0x1014 writes is signalled
// by dali_sync_arm_commission returning false (which regs_write_holding
// also returns).  process_fc06 / process_fc10 distinguish ILLEGAL_VALUE
// from DEVICE_BUSY by re-checking val > 2.

// ---- Raw command queue (task 06, queue v2) — DALI adapter contract ----
//
// The generic slot queue is in mbq_slots.c (opaque 32-bit command in,
// [status:8|response8:8] out — DALI-agnostic).  These are the DALI ADAPTER's
// payload contract for it: the meaning of the opaque command bytes and of the
// response8 codes.  The adapter itself is the OP_RAW arms in dali_sync.c.
//
// Command (32-bit, wire order = 2 Modbus regs):
//   byte0 addr | byte1 data | byte2 flags | byte3 dtr0
//   flags: bit0 send-twice, bit1 send-DTR0-first (0xA3<dtr0>), bit2 expects-answer;
//   bits 3-7 reserved 0 (room for future DT8 multi-frame flags).  No id field —
//   the slot is the identity and the gateway assigns the accept sequence.
#define QF_TWICE    0x01u  // send the forward frame twice (no answer)
#define QF_DTR0     0x02u  // emit 0xA3<dtr0> back-to-back immediately before the frame
#define QF_ANSWER   0x04u  // expects a backward frame; response8 = the 8-bit answer

// response8 on a FAILED slot (a COMPLETE slot's response8 is the backward-frame
// answer, or 0 for a no-answer command).  The any-on probe needs NAK (silence →
// "none on") DISTINCT from COLLISION (garbled backward frame → "≥1 on"); a genuine
// bus fault is a third code so it can't be misread as a collision.
#define DALI_RESP_NAK        0x01u  // no / short backward frame where one was expected
#define DALI_RESP_COLLISION  0x02u  // garbled/malformed backward frame (≥1 gear answered)
#define DALI_RESP_BUSFAULT   0x03u  // transport-level fault (short pulse etc.)

// ---- Full-scan tracking ----------------------------------------------
//
// scan_full_addr: which ballast's full scan is in progress (0xFF = none).
// scan_full_step: which of the 7 config-field queries we're up to.
extern volatile uint8_t scan_full_addr;
extern volatile uint8_t scan_full_step;

// Seconds since boot (RTC PIT, ~1 Hz).  Exposed via the FC04 sysinfo block.
extern volatile uint32_t g_uptime_s;

// ---- Public API ------------------------------------------------------

// Zero sync_flags[], dali_busy, current_op; arm group_write to idle
// (reg_addr = 0xFFFF).  Call once at boot before sei().
void dali_sync_init(void);

// Picks the highest-priority pending work and starts it.  Returns true
// if a command was initiated (dali_busy is now true).  Returns false if
// there is nothing to do, or if dali_busy was already true.  Called from
// main-loop context; safe against ISR-driven state changes via internal
// ATOMIC_BLOCK + TCB0 disable.
bool start_next_dali_work(void);

// Issue a single DALI command.  Internal to dali_sync; called by
// start_next_dali_work and by dali_sync_completion when advancing to
// the next logical step.  Performs cache-invalidation as a property of
// the command bytes (address-byte -> affected ballasts; cmd-byte ->
// INV_LEVEL/CONFIG/BOTH/NONE).  kind selects cmd_type_t.  repeat
// selects how many hardware sends comprise this logical command (1
// for one-shot, 2 for SET / ADD_TO_GROUP / REMOVE_FROM_GROUP / raw
// twice).  The bit-level state machine handles the second send via
// dali_result_received's repeat-decrement re-fire (matching the
// legacy queue mechanism, kept for inter-send timing fidelity).
typedef enum __attribute__((packed)) {
    DALI_KIND_DAPC = 0,        // CMD_DIRECT_ARC; bit0 of addr clear
    DALI_KIND_CMD,             // CMD_SEND; no response
    DALI_KIND_TWICE,           // CMD_SEND_TWICE; no response, but 22 TE wait
    DALI_KIND_QUERY,           // CMD_QUERY; backward frame expected
} dali_kind_t;
void issue_dali_command(uint8_t addr_byte, uint8_t data_byte,
                         dali_kind_t kind, uint8_t repeat);

// In-flight command stash.  Populated by issue_dali_command and consumed
// by dali_transmitting_state_enter (for the bytes), by finished_transmission
// (for the type, to decide whether to arm a backward-frame window), and
// by dali_result_received in dali_state_idle.c (for the repeat-decrement
// re-fire that handles SEND_TWICE without an extra inter-frame gap).
extern volatile cmd_entry_t s_active;

// Completion handler — called from dali_result_received (in dali_state_idle.c)
// when dali_busy is true.  Dispatches on current_op.op to advance the
// in-flight sequence or release dali_busy.  Runs in ISR context.
void dali_sync_completion(dali_response_type_t type, uint8_t value);

// Dispatch the next step of an in-flight sync op.  Called from
// dali_idle_state_enter when dali_busy is true and the bus has just
// become idle (i.e. the 22 TE inter-frame gap has elapsed).
void dali_sync_dispatch_current(void);

// Arm a commissioning run.  mode = 1 for full (INITIALISE_ALL, wipe cache,
// assign 0..N-1) or 2 for incremental (INITIALISE without short addr,
// assign lowest unused).  Returns false if a run is already in progress
// (master should re-poll 0x1014 first).  Called from main context via
// regs_write_holding for 0x1014 writes; the actual phase progression then
// runs out of the OP_COMMISSION work-selector slot.
// mode 1 = full, 2 = incremental, 3 = re-provision the gear at short address
// `target` (only used for mode 3) onto the lowest free short addresses.
bool dali_sync_arm_commission(uint8_t mode, uint8_t target);

// Arm a group register write (called from modbus.c for 0x0110-0x014F).
// reg_addr in [0x0110,0x014F]; pending_adds/pending_removes are bit
// masks of which bits in the cached register need ADD_TO_GROUP /
// REMOVE_FROM_GROUP DALI commands.  Returns false if a previous group
// write is still in flight.
bool dali_sync_arm_group_write(uint16_t reg_addr,
                                uint16_t pending_adds,
                                uint16_t pending_removes);

// True if a group write is currently in flight (caller uses this to
// decide whether to return SLAVE_DEVICE_BUSY before starting a new one).
// reg_addr is uint16_t = 2 bytes = non-atomic read on AVR; the
// completion ISR (dali_result_received -> dali_sync_completion's
// OP_GROUP_BIT path) can write 0xFFFF between the high-byte and low-
// byte read.  TCB0.INTCTRL=0 fences out LVL1; ATOMIC_BLOCK fences out
// LVL0 — same pattern as start_next_dali_work / process_fc04 raw-result clearing.
static inline bool dali_sync_group_busy(void) {
    bool busy;
    uint8_t old_tcb = TCB0.INTCTRL;
    TCB0.INTCTRL = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        busy = (group_write.reg_addr != 0xFFFFu);
    }
    TCB0.INTCTRL = old_tcb;
    return busy;
}

// ---- Atomic helpers --------------------------------------------------
//
// sync_flags[i] is read-modify-written from both main and ISR contexts;
// AVR has no atomic byte RMW so every update must run with interrupts
// disabled.  ATOMIC_RESTORESTATE preserves SREG so these helpers are
// safe to call from any context (including ISRs).
static inline void sync_flags_set(uint8_t idx, uint8_t mask) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sync_flags[idx] |= mask;
    }
}

static inline void sync_flags_clear(uint8_t idx, uint8_t mask) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        sync_flags[idx] &= (uint8_t)~mask;
    }
}

#endif
