#ifndef BG_TASKS_H
#define BG_TASKS_H

#include <stdint.h>

// Called from TCA0_OVF ISR — short, just decrements timers and sets flags.
void bgtasks_tick(void);

// Called from main loop — enqueues DALI work when timers are due.
void bgtasks_run(void);

// Trigger a full bus scan (addr 0–63).  Safe to call from main loop or ISR.
void bgtasks_trigger_scan(void);

#endif
