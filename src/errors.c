// errors.c — storage for faults that have no caller to return them to.
// See errors.h for the two rules. This module is deliberately tiny and
// allocation-free; it costs 4 bytes of SRAM.

#pragma GCC optimize("Os")

#include "errors.h"

static volatile uint16_t        s_count;
static volatile tm_err_t        s_last_err  = TM_OK;
static volatile tm_fault_site_t s_last_site = TM_SITE_NONE;

void tm_fault(tm_fault_site_t site, tm_err_t err) {
    if (err == TM_OK) return;
    // Saturate: a counter pinned at 0xFFFF still reads as broken, a wrapped one
    // can read as healthy.
    if (s_count != 0xFFFFu) s_count++;
    // Keep the FIRST fault's identity, not the last: the first break is the one
    // that explains the state, and a repeating fault would otherwise overwrite
    // the original cause with its own aftermath. The counter still shows repeats.
    if (s_last_err == TM_OK) {
        s_last_err  = err;
        s_last_site = site;
    }
}

uint16_t tm_fault_count(void)          { return s_count; }
tm_err_t tm_fault_last_err(void)       { return s_last_err; }
tm_fault_site_t tm_fault_last_site(void) { return s_last_site; }
