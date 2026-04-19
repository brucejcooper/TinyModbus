#include <avr/io.h>
#include <avr/interrupt.h>
#include "tick.h"
#include "dali.h"
#include "modbus.h"
#include "bg_tasks.h"

volatile uint8_t   dali_timer    = 0;
volatile tick_cb_t dali_timer_cb = NULL;

void tick_init(void) {
    TCA0.SINGLE.CTRLA    = 0;
    TCA0.SINGLE.INTCTRL  = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.PER      = USEC_TO_TICKS(DALI_HALF_BIT_USECS);
    TCA0.SINGLE.CTRLB    = TCA_SINGLE_WGMODE_NORMAL_gc;
    CPUINT.LVL1VEC       = TCA0_OVF_vect_num;
    TCA0.SINGLE.CTRLA    = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
}

void dali_tick(void) {
    if (dali_timer && (--dali_timer == 0))
        dali_timer_cb();
}

ISR(TCA0_OVF_vect) {
    dali_tick();
    modbus_tick();
    bgtasks_tick();
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}
