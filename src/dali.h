#ifndef DALI_H
#define DALI_H

#include <inttypes.h>
#include <stdbool.h>
#include <avr/io.h>
#include "types.h"
#include "tick.h"

// ---- Protocol constants ------------------------------------------------

#define DALI_BAUD             1200
#define DALI_BIT_USECS        (1000000.0 / DALI_BAUD)
#define DALI_HALF_BIT_USECS   (DALI_BIT_USECS / 2.0)
#define DALI_MAX_DEVICES      64

// ---- Address encoding --------------------------------------------------

#define DALI_SHORT_ADDR(n)  ((uint8_t)(((n) & 0x3Fu) << 1u))
#define DALI_GROUP_ADDR(n)  ((uint8_t)(0x80u | (((n) & 0x0Fu) << 1u)))
#define DALI_BROADCAST      0xFEu
#define DALI_CMD_FLAG       0x01u   // set bit 0 for command (vs DAPC)

// ---- DALI commands (low byte of 16-bit frame) --------------------------

#define DALI_CMD_OFF                  0x00u
#define DALI_CMD_UP                   0x01u
#define DALI_CMD_DOWN                 0x02u
#define DALI_CMD_STEP_UP              0x03u
#define DALI_CMD_STEP_DOWN            0x04u
#define DALI_CMD_RECALL_MAX           0x05u
#define DALI_CMD_RECALL_MIN           0x06u
#define DALI_CMD_GOTO_LAST_ACTIVE     0x0Au
#define DALI_CMD_QUERY_STATUS         0x90u
#define DALI_CMD_QUERY_ACTUAL_LEVEL   0xA0u
#define DALI_CMD_QUERY_MIN_LEVEL      0xA1u
#define DALI_CMD_QUERY_MAX_LEVEL      0xA2u
#define DALI_CMD_QUERY_POWER_ON_LEVEL 0xA3u
#define DALI_CMD_QUERY_FAIL_LEVEL     0xA4u
// QUERY_FADE_TIME_FADE_RATE returns one byte: high nibble = fade time
// (0..15), low nibble = fade rate (0..15).  Both fields refresh from this
// single query.  (DALI opcode 0xA6 is reserved by IEC 62386-102 - never
// issue it.)
#define DALI_CMD_QUERY_FADE_TIME_FADE_RATE  0xA5u
#define DALI_CMD_QUERY_GROUP_0_7      0xC0u
#define DALI_CMD_QUERY_GROUP_8_15     0xC1u

// Configuration commands - require SEND_TWICE; DTR0 loaded with new value first
#define DALI_CMD_SET_MAX_LEVEL        0x2Au
#define DALI_CMD_SET_MIN_LEVEL        0x2Bu
#define DALI_CMD_SET_SYS_FAIL_LEVEL   0x2Cu
#define DALI_CMD_SET_POWER_ON_LEVEL   0x2Du
#define DALI_CMD_SET_FADE_TIME        0x2Eu
#define DALI_CMD_SET_FADE_RATE        0x2Fu

// DTR set commands: addr byte = opcode, cmd byte = value to load
#define DALI_ADDR_SET_DTR0  0xA3u
#define DALI_ADDR_SET_DTR1  0xC3u
#define DALI_ADDR_SET_DTR2  0xC5u

// ---- Hardware pins -----------------------------------------------------

#define DALI_PORT         PORTB
#define DALI_TX_bm        PIN0_bm
#define DALI_RX_bm        PIN1_bm
#define DALI_RX_PINCTRL   PIN1CTRL

// ---- Public interface --------------------------------------------------

// True while the state machine is idle and able to accept new commands.
extern volatile bool dali_idle;

// Initialise TX/RX pins, TCB0 edge-capture, RTC PIT for periodic rescan,
// and enter wait-for-idle to let the bus settle after reset.
void    dali_init(void);

// Returns non-zero when another master is driving the bus (RX pin HIGH after optocoupler inversion).
uint8_t dali_is_bus_active(void);

// Arm a one-shot PORTB rising-edge interrupt; fires callback once then self-disarms.
void dali_call_on_rising_once(void (*callback)(void));

// Arm a one-shot PORTB low-level interrupt; fires callback once then self-disarms.
void dali_call_on_low_once(void (*callback)(void));

// Disarm the PORTB edge/level interrupt and clear the pending callback pointer.
void dali_cancel_edge_detect(void);

// If the queue is non-empty, begin transmitting the head command immediately.
// Otherwise arm the bus-monitor to detect external traffic.
void dali_idle_state_enter(void);

// Begin transmitting cmd as a DALI forward frame (Manchester-encoded, 1200 baud).
// Returns immediately; the timer ISR drives the rest of the transmission.
void dali_transmitting_state_enter(const cmd_entry_t *cmd);

// Enforce the mandatory 22 TE (~9 ms) inter-frame gap before the next TX.
// Handles bus-shorted (another master active) by waiting for it to release first.
// Has a 200 TE safety escape in case the bus never becomes idle.
void dali_wait_for_idle_state_enter(void);

// Arm TCB0 and a no-response timeout for the backward frame window that follows TX.
// Must be called immediately after the last stop bit is sent.
void dali_wait_for_response_state_enter(void);

// Configure TCB0 and the Manchester decoder without enabling the timer.
// Call this before the rising edge that starts the backward frame.
void dali_state_receiving_prepare(void);

// Enable TCB0 and begin decoding the backward frame that has already started.
// Assumes dali_state_receiving_prepare() was called first.
void dali_state_receiving_enter(void);

// Single authority for post-command state transitions - called from ISR context.
// Updates the target register (if any), fires the optional callback, decrements
// repeat count for CMD_SEND_TWICE, pops the queue, and enters the next state.
void cmdq_result_received(dali_response_type_t type, uint8_t value);

#endif
