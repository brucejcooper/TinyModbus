#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "tick.h"
#include "dali.h"
#include "modbus.h"
#include "bg_tasks.h"

volatile uint8_t   dali_timer    = 0;
volatile tick_cb_t dali_timer_cb = NULL;

// Configure TCA0 as the system timebase: overflow period = one DALI half-bit (~416 µs).
// Assigns TCB0 to CPU interrupt level 1 so it pre-empts TCA0 during RX capture.
void tick_init(void) {
    TCA0.SINGLE.CTRLA    = 0;
    TCA0.SINGLE.INTCTRL  = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.PER      = USEC_TO_TICKS(DALI_HALF_BIT_USECS);
    TCA0.SINGLE.CTRLB    = TCA_SINGLE_WGMODE_NORMAL_gc;
    CPUINT.LVL1VEC       = TCB0_INT_vect_num;
    TCA0.SINGLE.CTRLA    = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
}

// Decrement dali_timer and fire dali_timer_cb when it reaches zero.
// Called from the TCA0_OVF ISR every ~416 µs.
//
// dali_timer_cb is a 16-bit pointer that TCB0 (LVL1) can rewrite via cmdq_result_received
// → dali_idle_state_enter / dali_wait_for_idle_state_enter.  Snapshot the pointer inside
// ATOMIC_BLOCK so a TCB0 preemption between the two LDS instructions cannot tear the load.
void dali_tick(void) {
    if (dali_timer) {
        dali_timer--;       // volatile write to memory
        if (!dali_timer) {  // fresh volatile read - sees TCB0's update if it preempted
            tick_cb_t cb;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                cb = dali_timer_cb;
            }
            cb();
        }
    }
}

ISR(TCA0_OVF_vect) {
    // Clear OVF at the start so a callback that runs longer than one PER (~417 µs) does not
    // clobber a freshly raised flag - at most one tick is lost rather than all in-progress fires.
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
    dali_tick();
    modbus_tick();
    bgtasks_tick();
}
