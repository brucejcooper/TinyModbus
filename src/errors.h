#ifndef ERRORS_H
#define ERRORS_H

#include <stdint.h>
#include <stdbool.h>

// ─────────────────────────────────────────────────────────────────────────────
//                        ONE ERROR VOCABULARY, TWO RULES
//
// RULE 1 — if a function can fail, it returns tm_err_t. Not bool, not void.
// A bool collapses distinct causes into one value: regs_write_holding used to
// answer "false" for a bad baud index, an out-of-range slave address, a wrong
// reset magic, an NVM write failure AND an unmapped address, so the master was
// told "no such register" for four faults that were nothing of the sort.
//
// RULE 2 — an error that cannot be returned must be RECORDED. Some failures
// happen where no caller can act: mbq_complete runs in the DALI completion ISR,
// and a bad slot index there means a queue slot is stuck EXECUTING forever. Such
// a site calls tm_fault() so the fault becomes visible over Modbus (FC04 0x0300,
// registers 13-14) instead of vanishing. "Silently does nothing" is the one
// outcome this header exists to make impossible.
//
// tm_err_t is INTERNAL and deliberately not the Modbus exception codes. Those
// values are wire bytes; making the queue and DALI layers speak them would couple
// them to a protocol they otherwise know nothing about. tm_err_to_modbus() is the
// single translation point, at the boundary where the wire actually begins.
// ─────────────────────────────────────────────────────────────────────────────

typedef enum __attribute__((packed)) {
    TM_OK = 0,           // success — the only non-error value
    TM_ERR_NO_SUCH_REG,  // address is not in the register map
    TM_ERR_BAD_VALUE,    // value out of range for an otherwise valid register
    TM_ERR_BAD_RANGE,    // count / alignment / window invalid
    TM_ERR_BUSY,         // resource in use (slot active, DALI channel busy)
    TM_ERR_BAD_SLOT,     // slot index out of range — an internal invariant break
    TM_ERR_HW_FAULT,     // NVM/USERROW write did not take
} tm_err_t;

// Where a recorded fault came from. Kept tiny — one byte on the wire — but enough
// to tell which invariant broke without a debugger on a live bus.
typedef enum __attribute__((packed)) {
    TM_SITE_NONE = 0,
    TM_SITE_MBQ_BEGIN,     // mbq_begin got a slot index >= MBQ_SLOTS
    TM_SITE_MBQ_COMPLETE,  // mbq_complete got a slot index >= MBQ_SLOTS
    TM_SITE_MBQ_STATUS,    // mbq_read_status range overran the slot array
    TM_SITE_DALI_ISSUE,    // the DALI adapter could not launch a selected command
} tm_fault_site_t;

// Record a fault that cannot be returned to a caller. Safe from ISR context: a
// saturating counter plus two byte stores, no fence needed (the reader fences).
// Passing TM_OK is a no-op, so `tm_fault(site, some_call())` is safe to write
// unconditionally.
void tm_fault(tm_fault_site_t site, tm_err_t err);

// Accessors for the FC04 diagnostic block. count saturates at 0xFFFF rather than
// wrapping — a stuck-at-max counter still reads as "this is broken", whereas a
// wrapped one can read as healthy.
uint16_t tm_fault_count(void);
tm_err_t tm_fault_last_err(void);
tm_fault_site_t tm_fault_last_site(void);

#endif
