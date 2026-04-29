#ifndef BG_TASKS_H
#define BG_TASKS_H

#include <stdint.h>

// Called from TCA0_OVF ISR - no-op, kept for API compatibility.
void bgtasks_tick(void);

// Called from main loop - starts a bus scan when one is pending.
void bgtasks_run(void);

// Trigger a full bus scan (addr 0-63).  Safe to call from main loop or ISR.
void bgtasks_trigger_scan(void);

// Start a reactive fade-tracking chain for a single short-address ballast.
// Sends QUERY_STATUS; if FADE_RUNNING is set, re-queues BRIGHTNESS + STATUS
// and repeats until fading stops.  No-op if a chain is already active.
// Call after issuing any DAPC or arc command to a specific ballast.
void bgtasks_start_fade_poll(uint8_t ballast_idx);

#endif
