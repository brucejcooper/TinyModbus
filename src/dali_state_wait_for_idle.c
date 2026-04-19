#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>

#include "dali.h"
#include "tick.h"

#define IDLE_THRESHOLD_TE   22u   // minimum bus-idle period before next TX
#define BUS_STUCK_TIMEOUT_TE 200u  // escape hatch (~83 ms) if bus never goes idle

static void on_bus_shorted(void);

static void idle_period_expired(void) {
    dali_cancel_edge_detect();
    dali_idle_state_enter();
}

static void on_idle_bus(void) {
    dali_call_on_rising_once(on_bus_shorted);
    dali_timer_cb = idle_period_expired;
    dali_timer    = IDLE_THRESHOLD_TE;
}

static void bus_stuck_timeout(void) {
    DALI_PORT.OUTCLR = DALI_TX_bm;
    dali_cancel_edge_detect();
    dali_idle_state_enter();
}

static void on_bus_shorted(void) {
    dali_call_on_low_once(on_idle_bus);
    dali_timer_cb = bus_stuck_timeout;
    dali_timer    = BUS_STUCK_TIMEOUT_TE;
}

void dali_wait_for_idle_state_enter(void) {
    if (dali_is_bus_shorted())
        on_bus_shorted();
    else
        on_idle_bus();
}
