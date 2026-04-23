#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>

#include "cmd_queue.h"
#include "dali.h"
#include "tick.h"

static volatile uint16_t shift_reg;
static volatile uint8_t  bits_left;

// Last stop bit has been sent.
// CMD_QUERY: arm the backward frame window (response or 22 TE timeout).
// CMD_SEND_TWICE: must hold the 22 TE inter-send gap, so also goes through the response window.
// CMD_SEND / CMD_DIRECT_ARC: no response expected; call cmdq_result_received directly so the
// entry is popped before the mandatory 22 TE idle gap (saves one full response-window wait).
static void finished_transmission(void) {
    cmd_entry_t cmd;
    if (cmdq_peek(&cmd) && (cmd.type == CMD_QUERY || cmd.type == CMD_SEND_TWICE))
        dali_wait_for_response_state_enter();
    else
        cmdq_result_received(DALI_RESPONSE_NAK, 0);
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

void dali_transmitting_state_enter(const cmd_entry_t *cmd) {
    // Build the 16-bit DALI forward frame from the queue entry
    shift_reg = ((uint16_t)cmd->dali_addr << 8) | cmd->dali_cmd;
    bits_left = 17;  // 16 data bits + 1 extra for the stop-bit transition
    dali_timer_cb = send_one;
    dali_timer    = 1;
}
