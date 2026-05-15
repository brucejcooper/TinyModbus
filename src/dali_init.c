#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/atomic.h>

#include "dali.h"
#include "tick.h"

// One-shot callback pointer armed by dali_call_on_rising_once / dali_call_on_low_once.
// Cleared by the ISR immediately before firing so it cannot fire twice.
//
// PORTB_PORT_vect runs at LVL0 and reads this pointer with two LDS instructions; TCB0
// (LVL1) can preempt during the read and rewrite the pointer through the
// dali_result_received chain (which calls dali_idle_state_enter / dali_call_on_rising_once).
// Both reads and LVL0 writes are wrapped in ATOMIC_BLOCK to keep the 16-bit access
// uninterruptible.
void (*dali_rx_callback)(void) = NULL;

void dali_init(void) {
    // TX: configured as output, driven LOW (released bus).  DIR is held set
    // for the lifetime of the firmware; OUT is what the bit-level state
    // machine toggles - OUTSET asserts the bus (PB0 HIGH); OUTCLR releases
    // (PB0 LOW).  The external transistor inverts polarity between MCU pin
    // and bus line, so PB0 LOW = bus released = idle/safe.
    //
    // PB0 returns to high-Z (DIR=0, OUT=0) on every reset source per
    // ATtiny1604 datasheet § 16.5.4.  This is the WDT-during-TX recovery
    // mechanism: if the WDT fires while PB0 is OUTSET (bus asserted), the
    // reset clears OUT and DIR atomically, the external transistor base
    // pull-down keeps the bus released, then this dali_init runs and
    // re-establishes PB0 = driven LOW (released).  The OUTCLR-then-DIRSET
    // sequence below transitions PB0 high-Z -> driven LOW without ever
    // passing through driven-HIGH.
    DALI_PORT.OUTCLR = DALI_TX_bm;
    DALI_PORT.DIRSET = DALI_TX_bm;

    // RX: input, no pull
    DALI_PORT.OUTCLR = DALI_RX_bm;
    DALI_PORT.DIRCLR = DALI_RX_bm;

    // TCB0 in Frequency Measurement mode for RX edge capture
    TCB0.CTRLB = TCB_CNTMODE_FRQ_gc;
    EVSYS.ASYNCCH1    = EVSYS_ASYNCCH1_PORTB_PIN1_gc;
    EVSYS.ASYNCUSER0  = EVSYS_ASYNCUSER0_ASYNCCH1_gc;

    // RTC PIT for periodic bus rescan (1 kHz internal osc, fires every ~1 s)
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;
    while (RTC.PITSTATUS & RTC_CTRLABUSY_bm) {}
    RTC.PITCTRLA   = RTC_PERIOD_CYC1024_gc | RTC_PITEN_bm;
    RTC.PITINTCTRL = RTC_PI_bm;

    // Start in wait-for-idle to let the bus settle after reset
    dali_wait_for_idle_state_enter();
}

uint8_t dali_is_bus_active(void) {
    return (DALI_PORT.IN & DALI_RX_bm) ? 1u : 0u;
}

void dali_call_on_rising_once(void (*callback)(void)) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        dali_rx_callback = callback;
    }
    DALI_PORT.INTFLAGS       = DALI_RX_bm;
    DALI_PORT.DALI_RX_PINCTRL = PORT_ISC_RISING_gc;
}

void dali_call_on_low_once(void (*callback)(void)) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        dali_rx_callback = callback;
    }
    DALI_PORT.INTFLAGS       = DALI_RX_bm;
    DALI_PORT.DALI_RX_PINCTRL = PORT_ISC_LEVEL_gc;
}

void dali_cancel_edge_detect(void) {
    DALI_PORT.DALI_RX_PINCTRL = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        dali_rx_callback = NULL;
    }
}

ISR(PORTB_PORT_vect) {
    // Clear INTFLAGS at the start of the ISR so an edge that arrives after the callback
    // re-arms PINCTRL (e.g. start_receiving_response → dali_state_receiving_prepare)
    // is not clobbered by a trailing flag-clear at the end of this handler.
    if (DALI_PORT.INTFLAGS & DALI_RX_bm) {
        DALI_PORT.INTFLAGS = DALI_RX_bm;
        DALI_PORT.DALI_RX_PINCTRL = 0;

        // Snapshot the 16-bit pointer atomically: TCB0 (LVL1) can preempt this LVL0 ISR
        // and rewrite dali_rx_callback through the dali_result_received chain.
        void (*cb)(void);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            cb = dali_rx_callback;
        }
        if (cb)
            cb();
    }
}
