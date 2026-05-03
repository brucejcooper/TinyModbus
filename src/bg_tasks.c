#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include "bg_tasks.h"
#include "registers.h"
#include "cmd_queue.h"
#include "dali.h"
#include "tick.h"

// ---- RTC / periodic rescan ----------------------------------------
//
// SUBTASK 3 NOTE: RTC_PIT_vect has moved to dali_sync.c with new
// per-ballast round-robin semantics.  scan_trigger is now only set by
// main.c's initial boot scan; MBREG_SCAN_TRIGGER (0x1010) writes go
// through dali_sync's sync_flags[] (see modbus.c dispatch_action_reg).
// The bg_tasks scan path is dormant after the boot pass until subtask 5
// deletes it entirely.  rescan_counter is no longer touched by anything
// — left in place as temporary dead code.
static volatile uint16_t rescan_counter  __attribute__((unused)) = 64u;
static volatile bool     scan_trigger    = false;

// ---- Fade chain state ---------------------------------------------
// One bit per ballast (64 ballasts = 4 x uint16_t).
// Set while a self-sustaining STATUS → BRIGHTNESS → STATUS chain
// is active for that ballast.  Prevents duplicate chains.

static volatile uint16_t fade_chain_active[4];

// ---- Bus scan state ------------------------------------------------

static volatile bool    scan_active      = false;
static volatile uint8_t scan_err_count   = 0;
static volatile uint8_t scan_query_step  = 0;  // index into scan_queries[] during data reads

static void enqueue_scan_for_addr(uint8_t addr);  // forward decl

// ---- Per-ballast data query sequence (PROGMEM) ---------------------
// After confirming a ballast is present, these queries are issued one
// at a time via scan_next_query, allowing the fade poller to interleave.

typedef struct { uint8_t dali_cmd; reg_target_t reg_target; } scan_query_t;

static const scan_query_t scan_queries[] PROGMEM = {
    { DALI_CMD_QUERY_ACTUAL_LEVEL,         REG_BRIGHTNESS },
    { DALI_CMD_QUERY_MIN_LEVEL,            REG_MIN        },
    { DALI_CMD_QUERY_MAX_LEVEL,            REG_MAX        },
    { DALI_CMD_QUERY_POWER_ON_LEVEL,       REG_POWER_ON   },
    { DALI_CMD_QUERY_FAIL_LEVEL,           REG_SYS_FAIL   },
    // One DALI query refreshes both fade_time and fade_rate (regs_apply_result
    // splits the response byte's nibbles into block3.fade_time/fade_rate).
    { DALI_CMD_QUERY_FADE_TIME_FADE_RATE,  REG_FADE_RATE  },
    { DALI_CMD_QUERY_GROUP_0_7,            REG_GROUP_0_7  },
    { DALI_CMD_QUERY_GROUP_8_15,           REG_GROUP_8_15 },
};
#define NUM_SCAN_QUERIES (sizeof(scan_queries) / sizeof(scan_queries[0]))

// ---- Fade chain callbacks (run in ISR context) ---------------------

// ISR callback: if FADE_RUNNING is still set, re-queue BRIGHTNESS + STATUS
// to continue tracking; otherwise clear the active flag and let the chain expire.
// Queuing one pair at a time bounds the queue depth to 2 per active chain.
static void fade_poll_status_cb(dali_response_type_t type, uint8_t ballast_idx, uint8_t response) {
    uint8_t  word = ballast_idx >> 4;
    uint16_t bit  = (uint16_t)1u << (ballast_idx & 0x0Fu);

    if (type != DALI_RESPONSE_OK || !(response & DALI_STATUS_FADE_RUNNING)) {
        fade_chain_active[word] &= ~bit;
        return;
    }

    uint8_t dali_addr = DALI_SHORT_ADDR(ballast_idx) | DALI_CMD_FLAG;
    // Both pushes must succeed: if the brightness query fails, skip the status push
    // too (no orphan commands) and let the chain expire.
    if (cmdq_push_query(dali_addr, DALI_CMD_QUERY_ACTUAL_LEVEL, ballast_idx, REG_BRIGHTNESS) != SYS_OK ||
        cmdq_push_query_cb(dali_addr, DALI_CMD_QUERY_STATUS, ballast_idx, REG_STATUS,
                           fade_poll_status_cb) != SYS_OK)
        fade_chain_active[word] &= ~bit;  // queue full; chain expires gracefully
}

// ---- Scan callbacks (run in ISR context) ---------------------------

// ISR callback: advance the bus scan to ballast_idx+1, or mark scan complete at addr 63.
static void scan_advance(dali_response_type_t type, uint8_t ballast_idx, uint8_t response) {
    (void)type; (void)response;
    uint8_t next = ballast_idx + 1u;
    if (next < DALI_MAX_DEVICES) {
        enqueue_scan_for_addr(next);
    } else {
        g_regs.scan_status = SCAN_COMPLETE;
        scan_active = false;
    }
}

// ISR callback: push the next query in the per-ballast data sequence, one per invocation.
// When all queries are done, calls scan_advance to move to the next ballast address.
// Queuing one command at a time keeps the queue from overflowing and lets the fade
// poller interleave between scan commands.
static void scan_next_query(dali_response_type_t type, uint8_t ballast_idx, uint8_t response) {
    (void)type; (void)response;
    uint8_t step = scan_query_step;
    if (step >= NUM_SCAN_QUERIES) {
        scan_advance(DALI_RESPONSE_OK, ballast_idx, 0);
        return;
    }
    scan_query_step = step + 1u;
    uint8_t      dali_cmd  = pgm_read_byte(&scan_queries[step].dali_cmd);
    reg_target_t reg_tgt   = (reg_target_t)pgm_read_byte(&scan_queries[step].reg_target);
    uint8_t      dali_addr = DALI_SHORT_ADDR(ballast_idx) | DALI_CMD_FLAG;
    if (cmdq_push_query_cb(dali_addr, dali_cmd, ballast_idx, reg_tgt, scan_next_query) != SYS_OK) {
        scan_active = false;
        g_regs.scan_status = SCAN_COMPLETE;
    }
}

// ISR callback: handle the QUERY_STATUS result for one ballast during a bus scan.
// OK  → mark present, reset query step, kick off sequential data reads.
// NAK → mark absent and advance to next ballast.
// Other → retry up to 3 times, then treat as absent.
static void scan_status_received(dali_response_type_t type, uint8_t ballast_idx, uint8_t response) {
    if (type == DALI_RESPONSE_OK) {
        scan_err_count  = 0;
        scan_query_step = 0;
        regs_set_present(ballast_idx, true);
        scan_next_query(type, ballast_idx, response);
    } else if (type == DALI_RESPONSE_NAK) {
        scan_err_count = 0;
        regs_clear_ballast(ballast_idx);
        scan_advance(type, ballast_idx, 0);
    } else {
        // Transient error: retry up to 3 times, then treat as absent
        if (++scan_err_count < 3) {
            uint8_t dali_addr_byte = DALI_SHORT_ADDR(ballast_idx) | DALI_CMD_FLAG;
            if (cmdq_push_query_cb(dali_addr_byte, DALI_CMD_QUERY_STATUS, ballast_idx,
                                   REG_STATUS, scan_status_received) != SYS_OK) {
                scan_err_count = 0;
                regs_clear_ballast(ballast_idx);
                scan_advance(type, ballast_idx, 0);
            }
        } else {
            scan_err_count = 0;
            regs_clear_ballast(ballast_idx);
            scan_advance(type, ballast_idx, 0);
        }
    }
}

// Push QUERY_STATUS for one ballast address and kick the DALI state machine.
static void enqueue_scan_for_addr(uint8_t addr) {
    uint8_t dali_addr_byte = DALI_SHORT_ADDR(addr) | DALI_CMD_FLAG;
    if (cmdq_push_query_cb(dali_addr_byte, DALI_CMD_QUERY_STATUS, addr, REG_STATUS,
                           scan_status_received) != SYS_OK) {
        scan_active        = false;
        g_regs.scan_status = SCAN_COMPLETE;
        return;
    }
    cmdq_kick();
}

// ---- Public API ----------------------------------------------------

void bgtasks_trigger_scan(void) {
    scan_trigger = true;
}

void bgtasks_start_fade_poll(uint8_t ballast_idx) {
    uint8_t  word = ballast_idx >> 4;
    uint16_t bit  = (uint16_t)1u << (ballast_idx & 0x0Fu);
    // Atomic check-and-set: prevents a concurrent ISR clearing the bit between
    // our check and our set, which would leave a dead chain marked active.
    bool was_active;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        was_active = (fade_chain_active[word] & bit) != 0;
        if (!was_active) fade_chain_active[word] |= bit;
    }
    if (was_active) return;
    uint8_t dali_addr = DALI_SHORT_ADDR(ballast_idx) | DALI_CMD_FLAG;
    if (cmdq_push_query_cb(dali_addr, DALI_CMD_QUERY_STATUS, ballast_idx, REG_STATUS,
                           fade_poll_status_cb) != SYS_OK) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { fade_chain_active[word] &= ~bit; }
    }
    cmdq_kick();
}

void bgtasks_tick(void) {
    // No timer-based work remains; kept for API compatibility with tick.c ISR.
}

void bgtasks_run(void) {
    // Bus scan
    if (scan_trigger) {
        scan_trigger = false;
        if (!scan_active) {
            scan_active = true;
            scan_err_count = 0;
            g_regs.scan_status = SCAN_SCANNING;
            enqueue_scan_for_addr(0);
        }
    }
}

// ---- RTC PIT - periodic rescan ------------------------------------
//
// SUBTASK 3: RTC_PIT_vect moved to dali_sync.c with new per-ballast
// round-robin semantics.  MBREG_SCAN_TRIGGER (0x1010) now drives
// dali_sync's sync_flags[] directly from modbus.c (no longer routed
// through bgtasks_trigger_scan).  The bg_tasks scan path is dormant
// after main.c's initial boot scan and gets deleted in subtask 5.
