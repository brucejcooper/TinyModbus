
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <util/atomic.h>
#include <stdbool.h>
#include "timers.h"


typedef void (*callback_t)();
callback_t timeoutCallback = NULL;




void update_timeout(uint16_t to) {
    TCA0.SINGLE.PER = to;
}

void update_timeout_callback(callback_t callback) {
    cli();
    timeoutCallback = callback;
    sei();
}


void timeout_init() {
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm; // Enable OVF interrupt.
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; 
    TCA0.SINGLE.PER = 0xFFFF; // Send ISR when clock reaches PER.
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc; // normal counter mode, no output.
    TCA0.SINGLE.CNT = 0;  // We try to always reset timer counters, but make sure. 

    // Make TCA0.OVF the highest priority interrupt, to ensure it executes first. 
    CPUINT.LVL1VEC = TCA0_OVF_vect_num;
}


void on_timeout(uint16_t timeout, callback_t callback) {
    cli();
    TCA0.SINGLE.PER = timeout; // Send ISR when clock reaches PER.
    timeoutCallback = callback;
    sei();
}

inline void start_timeout() {
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm; // Start TCA0 for timeout.
}


inline void cancel_timeout() {
    TCA0.SINGLE.CTRLA = 0;
}


inline void reset_timeout() {
    TCA0.SINGLE.CNT = 0;
}

inline void restart_timeout() {
    reset_timeout();
    start_timeout();

}


ISR(TCA0_OVF_vect) {
    if (timeoutCallback) {
        timeoutCallback();
    }
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; // Clear interrupt flag
}
