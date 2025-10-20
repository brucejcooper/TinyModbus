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



typedef void (*timer_callback_t)();

#define TIMER_DALI 0
#define TIMER_RS485 1
#define TIMER_MAX 2

typedef struct {
    uint8_t next;
    bool active;
    timer_callback_t callback;
} timer_t;


extern timer_t daliTimer;
extern timer_t rs485Timer;
extern volatile uint8_t timerCurrentTick;

extern void timer_init();
// extern void timer_set_cb(uint8_t idx, timer_callback_t cb);
// extern void timer_exec_in(uint8_t idx, uint8_t ticks);
// extern void timer_stop(uint8_t idx);
#define timer_extend(timer, ticks) { timer.next += (ticks); timer.active = true;}

#endif