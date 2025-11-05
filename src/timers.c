
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <util/atomic.h>
#include <stdbool.h>
#include "dali.h"
#include "rs485.h"
#include "timers.h"


timer_callback_t daliTimerCallback = NULL;
volatile uint8_t daliTimer = 0;

volatile uint8_t rs485Timer = 0;



void timer_init() {
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm; // Enable OVF interrupt.
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; 
    TCA0.SINGLE.PER = USEC_TO_TICKS(DALI_HALF_BIT_USECS); // Send ISR when clock reaches PER.
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc; // normal counter mode, no output.

    // Make TCA0.OVF the highest priority interrupt, to ensure it executes first. 
    CPUINT.LVL1VEC = TCA0_OVF_vect_num;
    // Turn on the timer. It always runs.
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
}


ISR(TCA0_OVF_vect) {
    if (daliTimer && (--daliTimer == 0)) {
        daliTimerCallback();
    }

    if (rs485Timer && (--rs485Timer == 0)) {
        rs485Timer_timeout();
    }
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; // Clear interrupt flag
}
