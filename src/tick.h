#ifndef TICK_H
#define TICK_H

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>

// TCA0 runs at F_CPU with no prescaler.
// PER = USEC_TO_TICKS(DALI_HALF_BIT_USECS) ≈ 4167 ticks at 10 MHz → 416.7 µs per tick.
#define USEC_TO_TICKS(u) ((uint16_t)(((float)(u)) * (F_CPU / 1000000.0) + 0.5))

// Ticks per 100 ms at DALI 1200-baud half-bit period (416.67 µs/tick):
//   100000 µs / 416.67 µs = 240 ticks
#define TICKS_PER_100MS  240u

typedef void (*tick_cb_t)(void);

// One-shot countdown timer used by the DALI state machine.
// Set dali_timer = N, dali_timer_cb = func; callback fires after N ticks.
extern volatile uint8_t  dali_timer;
extern volatile tick_cb_t dali_timer_cb;

void tick_init(void);

// Called from TCA0_OVF ISR — fires dali_timer callback when countdown hits 0.
void dali_tick(void);

// Prototypes for the other ISR-dispatched tick functions (implemented in their modules).
void modbus_tick(void);
void bgtasks_tick(void);

#endif
