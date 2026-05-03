// dali_sync.c — sync-flag table, async DALI work tracking, PIT cursor.
//
// Threading model (subtask 3 scope):
//   - sync_flags[]: read/written by main (set on Modbus writes / SCAN_TRIGGER)
//     and by RTC_PIT_vect (sets SYNC_READ_LEVEL).  All RMW updates go through
//     sync_flags_set / sync_flags_clear (ATOMIC_BLOCK).
//   - scan_cursor + tick: written only inside RTC_PIT_vect; not read elsewhere
//     in this subtask.
//   - dali_busy / current_op / group_write: declared here, exercised in
//     subtask 4 when start_next_dali_work() is filled in.
//
// SUBTASK 3 ONLY: start_next_dali_work() is a stub that always returns false.
// The cmd_queue / bg_tasks scan path is still wired up via main.c, so the
// queue keeps running.  The PIT now sets SYNC_READ_LEVEL bits that nothing
// consumes - that becomes live in subtask 4.

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/atomic.h>

#include "dali.h"
#include "dali_sync.h"
#include "registers.h"

// ---- Globals ---------------------------------------------------------

volatile uint8_t          sync_flags[DALI_MAX_DEVICES];
volatile bool             dali_busy;
volatile dali_current_op_t current_op;
volatile dali_group_write_t group_write;

// PIT round-robin cursor — advances at most one slot per fired tick.
static volatile uint8_t scan_cursor;

// ---- Init ------------------------------------------------------------

void dali_sync_init(void) {
    for (uint8_t i = 0; i < DALI_MAX_DEVICES; i++) {
        sync_flags[i] = 0;
    }
    dali_busy        = false;
    current_op.addr  = 0xFF;
    current_op.op    = 0;
    current_op.step  = 0;
    group_write.reg_addr        = 0xFFFFu;  // idle
    group_write.pending_adds    = 0;
    group_write.pending_removes = 0;
    scan_cursor = 0;
}

// ---- Work selector (stub for subtask 3) ------------------------------

bool start_next_dali_work(void) {
    // Subtask 4 will implement the real selection priority (raw cmd >
    // group bit-change > SYNC_WRITE_* > SYNC_READ_LEVEL > full-scan
    // continuation > new full-scan).  For now the queue/bg_tasks path
    // is still in charge, so this always returns false.
    return false;
}

// ---- Periodic re-poll (PIT) — round-robin ----------------------------
//
// The RTC PIT fires every ~1 s (configured by dali_init via CYC1024 on
// the 1 kHz internal oscillator).  Each fire we increment a tick counter;
// once it reaches MBREG_RESCAN_INTERVAL, we set SYNC_READ_LEVEL on the
// ballast at scan_cursor and advance the cursor (mod 64).
//
// SEMANTIC CHANGE from the original (queue-based) implementation:
// MBREG_RESCAN_INTERVAL (0x1011) used to mean "seconds between full
// scans of all 64 ballasts."  It now means "seconds between single-
// ballast level polls."  With the recommended default of 1, that gives
// a complete cycle every 64 s — the same per-ballast cadence as the
// previous 64 s default, just spread evenly instead of bursting.
//
// rescan_interval == 0 disables the cursor entirely (no automatic polls).
ISR(RTC_PIT_vect) {
    static uint8_t tick = 0;

    uint16_t interval = userrow_get_rescan_interval();
    if (interval > 0) {
        tick++;
        if (tick >= interval) {
            tick = 0;
            // ISR-context: sync_flags[i] |= ... is non-atomic on AVR (LDS/OR/STS)
            // but we are already at the highest-priority context that touches this
            // byte; only main can race with us, and main's writes are wrapped in
            // ATOMIC_BLOCK via sync_flags_set.  Disabling interrupts here would be
            // redundant.
            sync_flags[scan_cursor] |= SYNC_READ_LEVEL;
            scan_cursor++;
            if (scan_cursor >= DALI_MAX_DEVICES) scan_cursor = 0;
        }
    } else {
        // Disabled: keep tick at 0 so a later non-zero interval doesn't
        // immediately fire on a stale count.
        tick = 0;
    }

    RTC.PITINTFLAGS = RTC_PI_bm;
}
