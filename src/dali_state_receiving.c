#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/atomic.h>

#include "dali.h"
#include "tick.h"

#define DALI_READ_TOLERANCE_USECS (DALI_HALF_BIT_USECS / 5.0)
#define HALFBIT_MIN_TICKS  USEC_TO_TICKS(DALI_HALF_BIT_USECS - DALI_READ_TOLERANCE_USECS)
#define HALFBIT_MAX_TICKS  USEC_TO_TICKS(DALI_HALF_BIT_USECS + DALI_READ_TOLERANCE_USECS)
#define FULLBIT_MIN_TICKS  USEC_TO_TICKS(DALI_BIT_USECS       - DALI_READ_TOLERANCE_USECS)
#define FULLBIT_MAX_TICKS  USEC_TO_TICKS(DALI_BIT_USECS       + DALI_READ_TOLERANCE_USECS)

static volatile uint32_t shift_reg = 0;
static volatile uint8_t  num_bits  = 0;

#define STATE_STARTBIT  0
#define STATE_START_0   1
#define STATE_START_1   2
#define STATE_MIDDLE_0  3
#define STATE_MIDDLE_1  4
#define STATE_ERROR     5

#define STATE_BP  4
#define SHIFT_0   0
#define SHIFT_1   1
#define NOSHIFT   2

typedef struct { uint8_t shortpulse; uint8_t longpulse; } rx_state_t;

static const rx_state_t state_machine[STATE_ERROR + 1] = {
    /* STARTBIT */ { NOSHIFT | (STATE_MIDDLE_1 << STATE_BP), STATE_ERROR << STATE_BP },
    /* START_0  */ { SHIFT_0 | (STATE_MIDDLE_0 << STATE_BP), STATE_ERROR << STATE_BP },
    /* START_1  */ { SHIFT_1 | (STATE_MIDDLE_1 << STATE_BP), STATE_ERROR << STATE_BP },
    /* MIDDLE_0 */ { NOSHIFT | (STATE_START_0  << STATE_BP), SHIFT_1 | (STATE_MIDDLE_1 << STATE_BP) },
    /* MIDDLE_1 */ { NOSHIFT | (STATE_START_1  << STATE_BP), SHIFT_0 | (STATE_MIDDLE_0 << STATE_BP) },
    /* ERROR    */ { STATE_ERROR << STATE_BP,                 STATE_ERROR << STATE_BP },
};

static volatile uint8_t rx_state;

static void rx_cleanup(void) {
    TCB0.CTRLA   = 0;
    dali_timer   = 0;
    TCB0.EVCTRL  = 0;
    TCB0.INTCTRL = 0;
}

static void timer_overflow(void) {
    rx_cleanup();
    if ((rx_state == STATE_START_0 || rx_state == STATE_MIDDLE_1) && num_bits == 8)
        cmdq_result_received(DALI_RESPONSE_OK, (uint8_t)shift_reg);
    else
        cmdq_result_received(DALI_RESPONSE_MALFORMED, 0);
    dali_wait_for_idle_state_enter();
}

static void send_rx_error(dali_response_type_t err) {
    rx_cleanup();
    cmdq_result_received(err, 0);
    dali_wait_for_idle_state_enter();
}

void dali_state_receiving_prepare(void) {
    rx_state  = STATE_STARTBIT;
    shift_reg = 0;
    num_bits  = 0;
    TCB0.CTRLA   = 0;
    TCB0.CNT     = 73;
    TCB0.EVCTRL  = TCB_CAPTEI_bm | TCB_EDGE_bm;
    TCB0.INTCTRL = TCB_CAPT_bm;
}

void dali_state_receiving_enter(void) {
    TCB0.CTRLA   = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
    TCB0.INTFLAGS = TCB_CAPT_bm;
    dali_timer_cb = timer_overflow;
    dali_timer    = 5;
}

ISR(TCB0_INT_vect) {
    dali_timer = 5;  // extend timeout on each edge

    uint16_t pulse = TCB0.CCMP;
    TCB0.EVCTRL ^= TCB_EDGE_bm;  // toggle to capture opposite edge

    uint8_t action;
    if (pulse < HALFBIT_MIN_TICKS) {
        send_rx_error(DALI_RESPONSE_PULSE_TOO_SHORT); return;
    } else if (pulse <= HALFBIT_MAX_TICKS) {
        action = state_machine[rx_state].shortpulse;
    } else if (pulse < FULLBIT_MIN_TICKS) {
        send_rx_error(DALI_RESPONSE_PULSE_IN_MIDDLE); return;
    } else if (pulse <= FULLBIT_MAX_TICKS) {
        action = state_machine[rx_state].longpulse;
    } else {
        send_rx_error(DALI_RESPONSE_PULSE_TOO_LONG); return;
    }

    rx_state = action >> STATE_BP;
    if (rx_state == STATE_ERROR) {
        send_rx_error(DALI_RESPONSE_MALFORMED); return;
    }

    uint8_t bit = action & 0x03u;
    if (bit != NOSHIFT) {
        shift_reg = (shift_reg << 1) | bit;
        num_bits++;
    }
}
