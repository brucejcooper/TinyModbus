/*
  DALI master, useable as a multi-switch controller.  Programmed (via UPDI, or UART?) to respond to button presses
  by sending out DALI commands to dim
*/

#ifndef __TIMERS_H__
#define __TIMERS_H__

#include <inttypes.h>
#include <stdbool.h>

#define TICKS_TO_USEC(t) (t * 3 / 10)
// 19.86 ms is the highest value that this will support.
#define USEC_TO_TICKS(u) ((uint16_t)(((float)u) * (F_CPU / 1000000.0) + 0.5))

typedef void (*pulsewidth_handler_t)(uint16_t pulse_width);

void timeout_init();
void update_timeout(uint16_t to);
void on_timeout(uint16_t timeout, void (*callback)());
void start_timeout();
void cancel_timeout();
void reset_timeout();
void restart_timeout();
void update_timeout_callback(void (*callback)());

#endif