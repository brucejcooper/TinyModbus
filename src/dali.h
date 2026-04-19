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
#define DALI_CMD_QUERY_FADE_RATE      0xA5u
#define DALI_CMD_QUERY_FADE_TIME      0xA6u
#define DALI_CMD_QUERY_GROUP_0_7      0xC0u
#define DALI_CMD_QUERY_GROUP_8_15     0xC1u

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

extern volatile bool dali_idle;

void    dali_init(void);
uint8_t dali_is_bus_shorted(void);

// Edge-detect one-shots (used by state files)
void dali_call_on_rising_once(void (*callback)(void));
void dali_call_on_low_once(void (*callback)(void));
void dali_cancel_edge_detect(void);

// State machine entry points (each lives in its own .c file)
void dali_idle_state_enter(void);
void dali_transmitting_state_enter(const cmd_entry_t *cmd);
void dali_wait_for_idle_state_enter(void);
void dali_wait_for_response_state_enter(void);

// RX state setup (called before the rising edge that starts a frame)
void dali_state_receiving_prepare(void);
void dali_state_receiving_enter(void);

// Called by the receive state when a backward frame completes or times out.
// Runs in ISR context — updates register, decrements repeat, pops queue.
void cmdq_result_received(dali_response_type_t type, uint8_t value);

// LED diagnostic (kept from original)
void pulse_led(uint8_t times);

#endif
