/*
  DALI master, useable as a multi-switch controller.  Programmed (via UPDI, or UART?) to respond to button presses
  by sending out DALI commands to dim
*/

#ifndef __TIMERS_H__
#define __TIMERS_H__

#include <inttypes.h>
#include <stdbool.h>

// 19.86 ms is the highest value that this will support.
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