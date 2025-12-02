
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <stdio.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "dali.h"

// Good documentation for Dali commands - https://www.nxp.com/files-static/microcontrollers/doc/ref_manual/DRM004.pdf

/**
 * @brief We may want to do different things with the DALI rising signal interrupt.  This stores a pointer to the current thing.
 * May be NULL to do nothing.
 *
 */
void (*dali_rx_rising_callback)() = NULL;

// By default, we go off once per 64 seconds (8 x 1 second periods)
#define RESCAN_PERIOD 64


static volatile uint8_t rescan_counter = RESCAN_PERIOD;


void dali_init() {
    // Transmission set up
    DALI_PORT.OUTCLR = DALI_TX_bm;
    DALI_PORT.DIRSET = DALI_TX_bm;

    // Receiving set up
    DALI_PORT.OUTCLR = DALI_RX_bm;
    DALI_PORT.DIRCLR = DALI_RX_bm;

    // We use TCB0 to process inputs, using events
    TCB0.CTRLB = TCB_CNTMODE_FRQ_gc;  // Put it in Capture Frequency Measurement mode.
    // Set up event systemt to route DALI RX change events to channel 0.
    EVSYS.ASYNCCH1 = EVSYS_ASYNCCH1_PORTB_PIN1_gc;    // Make BP2 send asyncch1 event.
    EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH1_gc;  // Set TCB0 to use ASYNCHCH1

    dali_state_init();  // Initialise the state machine.

    // Set up the PIT to go off occasionally to scan the dali devices and feed the WDT

    // Run off internal 1Khz clock;
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;

    // Turn the PIT on, to execute every 8ish seconds.
    while (RTC.PITSTATUS & RTC_CTRLABUSY_bm) {};
    RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc | RTC_PITEN_bm;
    RTC.PITINTCTRL = RTC_PI_bm; // Enable PIT interrupts. 

    // We start in the reset state (which we should exit in short order).
    dali_wait_for_idle_state_enter();
}

/**
 * @brief Reads the current DALI bus state.
 *
 * @return uint8_t 1 for shorted, 0 for released...
 */
uint8_t dali_is_bus_shorted() {
    return DALI_PORT.IN & DALI_RX_bm ? 1 : 0;
    // return  AC0.STATUS & AC_STATE_bm ? 0 : 1;
}

void dali_call_on_rising_once(void (*callback)()) {
    dali_rx_rising_callback = callback;
    DALI_PORT.INTFLAGS = DALI_RX_bm;                 // Clear any pending interrupts.
    DALI_PORT.DALI_RX_PINCTRL = PORT_ISC_RISING_gc;  // Enable ISR 
}

void dali_call_on_low_once(void (*callback)()) {
    dali_rx_rising_callback = callback;
    DALI_PORT.INTFLAGS = DALI_RX_bm;                // Clear any pending interrupts.
    DALI_PORT.DALI_RX_PINCTRL = PORT_ISC_LEVEL_gc;  // Enable ISR 
}

void dali_cancel_edge_detect() {
    DALI_PORT.DALI_RX_PINCTRL = 0;
    dali_rx_rising_callback = NULL;
}

// This controls the one-shot event on rising for detecting the start of data.
ISR(PORTB_PORT_vect) {
    if (DALI_PORT.INTFLAGS & DALI_RX_bm) {
        DALI_PORT.DALI_RX_PINCTRL = 0;  // Because we only want the call to happen once, disable the ISC
        if (dali_rx_rising_callback) {
            dali_rx_rising_callback();
        }
        DALI_PORT.INTFLAGS = DALI_RX_bm;  // Clear the Interrupt flag.
    }
}


ISR(RTC_PIT_vect) {
    if (--rescan_counter == 0) {
        // pulse_led(2);
        dali_start_background_scan(0);
        rescan_counter = RESCAN_PERIOD;
    }
    RTC.PITINTFLAGS = RTC_PI_bm; // Clear the flag.
}