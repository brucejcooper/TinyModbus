/*
  DALI master, useable as a multi-switch controller.  Programmed (via UPDI, or UART?) to respond to button presses
  by sending out DALI commands to dim
*/

#ifndef __TIMERS_H__
#define __TIMERS_H__

#include <inttypes.h>
#include <stdbool.h>

// Converts microseconds to TCA0/TCB0 ticks at F_CPU. The result is uint16_t, so the largest
// value that can be expressed is ~6553 µs at F_CPU = 10 MHz (uint16_t max / ticks-per-µs).
#define USEC_TO_TICKS(u) ((uint16_t)(((float)u) * (F_CPU / 1000000.0) + 0.5))

typedef void (*timer_callback_t)();

#define TIMER_DALI 0
#define TIMER_RS485 1
#define TIMER_MAX 2


extern volatile uint8_t daliTimer;
extern timer_callback_t daliTimerCallback;
extern volatile uint8_t rs485Timer;

extern void timer_init();

#endif