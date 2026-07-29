#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>

#include "dali.h"
#include "dali_sync.h"
#include "tick.h"

static volatile uint16_t shift_reg;
static volatile uint8_t  bits_left;

// Whether the send in flight keeps the response window open after its last stop
// bit. Set by dali_transmitting_state_enter; a property of the ONE send on the
// wire, so it lives here rather than being re-derived from the command.
static volatile bool s_hold_window;

// Last stop bit has been sent. Either arm the backward-frame window (a query
// awaiting an answer, or a send-twice using it as the 22 TE inter-send gap), or
// complete immediately so the caller can advance state before the 22 TE idle gap.
static void finished_transmission(void) {
    if (s_hold_window) dali_wait_for_response_state_enter();
    else               dali_result_received(DALI_RESPONSE_NAK, 0);
}

static void half_bit_toggle(void);

// First half of a Manchester '1': drive TX high, schedule the midpoint toggle.
static void send_one(void) {
    DALI_PORT.OUTSET     = DALI_TX_bm;
    dali_timer_cb = half_bit_toggle;
    dali_timer    = 1;
}

// First half of a Manchester '0': drive TX low, schedule the midpoint toggle.
static void send_zero(void) {
    DALI_PORT.OUTCLR     = DALI_TX_bm;
    dali_timer_cb = half_bit_toggle;
    dali_timer    = 1;
}

// Drive TX low and hold for 2 TE (the DALI stop period), then signal completion.
static void start_stop_bits(void) {
    DALI_PORT.OUTCLR     = DALI_TX_bm;
    dali_timer_cb = finished_transmission;
    dali_timer    = 4;
}

// Midpoint of a Manchester bit: toggle TX, then load the next bit or begin stop period.
static void half_bit_toggle(void) {
    DALI_PORT.OUTTGL = DALI_TX_bm;
    if (--bits_left == 0) {
        dali_timer_cb = start_stop_bits;
    } else {
        dali_timer_cb = (shift_reg & 0x8000u) ? send_one : send_zero;
        shift_reg <<= 1;
    }
    dali_timer = 1;
}

void dali_transmitting_state_enter(uint8_t addr, uint8_t data, bool hold_window) {
    s_hold_window = hold_window;
    shift_reg = ((uint16_t)addr << 8) | data;
    bits_left = 17;  // 16 data bits + 1 extra for the stop-bit transition
    dali_timer_cb = send_one;
    dali_timer    = 1;
}
