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
// happen where no caller can act: cq_complete runs in the DALI completion ISR,
// and an unmatched seq there means a queue slot is stuck EXECUTING forever. Such
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
// These values go on the wire (FC04 0x0300 reg14), so the numbering is kept
// stable across the v4 rework: 1, 2 and 4 still mean what they meant under the
// v2 slot model. Only 3 was repurposed — mbq_read_status has no successor (the
// master no longer reads slot state at all), and the response queue overflowing
// is the new failure that needs a name.
typedef enum __attribute__((packed)) {
    TM_SITE_NONE = 0,
    TM_SITE_CQ_BEGIN,      // cq_begin was handed an index that was not PENDING
    TM_SITE_CQ_COMPLETE,   // cq_complete found no EXECUTING command with that seq
    TM_SITE_CQ_RESPQ,      // response queue full at completion — backpressure broke
    TM_SITE_DALI_ISSUE,    // the DALI adapter could not launch a selected command
    TM_SITE_CQ_ACCEPT,     // cq_accept: ring cursor and slot state disagree
} tm_fault_site_t;

// Record a fault that cannot be returned to a caller. Passing TM_OK is a no-op,
// so `tm_fault(site, some_call())` is safe to write unconditionally.
//
// CALLABLE FROM ANY CONTEXT, AND UNFENCED — but "safe" here means the CONTRACT
// survives, not that the count is exact, and the difference is worth stating
// because the callers really do span contexts:
//
//   main  cq_accept via the Modbus handler; dali_sync_dispatch_current via
//         start_next_dali_work's launch, which is OUTSIDE its ATOMIC_BLOCK.
//         (TM_SITE_CQ_BEGIN is the one main-context site that is fenced, and
//         only incidentally — it sits inside that block.)
//   LVL1  cq_complete from the DALI completion ISR.
//   LVL0  cq_complete via the bus-stuck escape, and dali_sync_dispatch_current
//         via the idle/wait-for-idle timer callbacks.
//
// The count is a 16-bit read-modify-write, so a main/ISR interleave can LOSE an
// increment, and a reader can tear it. Neither breaks what this module actually
// promises: a lost increment can never take a non-zero count back to zero, so
// "reg13 != 0 means an internal invariant broke" holds regardless. Treat the
// value as a repeat indicator, not a measurement.
//
// The err/site pair is first-write-wins and each is a single byte, so neither
// can tear. The only residual pathology is two SIMULTANEOUS distinct faults
// pairing one's err with the other's site — which needs two invariant breaks
// racing on an already-broken gateway, and would mislead about which of two real
// bugs fired first rather than inventing one.
//
// Not fixed because both fixes are worse than the defect: a fence here would sit
// on paths this project deliberately keeps fence-free (see cq.h and the
// 2026-06-05 RX starvation incident), and a byte-wise counter would trade an
// exact count for a much smaller ceiling.
void tm_fault(tm_fault_site_t site, tm_err_t err);

// Accessors for the FC04 diagnostic block. count saturates at 0xFFFF rather than
// wrapping — a stuck-at-max counter still reads as "this is broken", whereas a
// wrapped one can read as healthy.
uint16_t tm_fault_count(void);
tm_err_t tm_fault_last_err(void);
tm_fault_site_t tm_fault_last_site(void);

#endif
