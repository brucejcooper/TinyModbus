#ifndef DALI_SYNC_H
#define DALI_SYNC_H

#include <stdbool.h>
#include <stdint.h>
#include <util/atomic.h>

#include "dali.h"

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
#define SYNC_READ_LEVEL    (1u << 0)  // QUERY_STATUS + GET_LEVEL
#define SYNC_READ_FULL     (1u << 1)  // re-fetch all 7 config fields (incl. group memberships)
#define SYNC_WRITE_LEVEL   (1u << 2)  // DAPC (one DALI command)
#define SYNC_WRITE_MINMAX  (1u << 3)  // DTR0+SET_MIN(x2)+DTR0+SET_MAX(x2) - 6 sends
#define SYNC_WRITE_POWER   (1u << 4)  // DTR0+SET_POWER_ON(x2)+DTR0+SET_SYS_FAIL(x2)
#define SYNC_WRITE_FADE    (1u << 5)  // DTR0+SET_FADE_RATE(x2)+DTR0+SET_FADE_TIME(x2)
// bit 6 spare
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

// What DALI is currently doing, so the completion handler knows how to
// process the response and what (if anything) to start next.
typedef struct {
    uint8_t addr;  // ballast short address being processed (0xFF for raw / broadcast)
    uint8_t op;    // SYNC_* bit being serviced, or a marker for raw / full-scan
    uint8_t step;  // step within a multi-step sequence (0-based)
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

// ---- Public API ------------------------------------------------------

// Zero sync_flags[], dali_busy, current_op; arm group_write to idle
// (reg_addr = 0xFFFF).  Call once at boot before sei().
void dali_sync_init(void);

// Picks the highest-priority pending work and starts it.  Returns true
// if a command was initiated (dali_busy is now true).  Returns false if
// there is nothing to do, or if dali_busy was already true.
//
// SUBTASK 3: stub implementation, always returns false.  Subtask 4
// fills in the real selection priority.
bool start_next_dali_work(void);

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
