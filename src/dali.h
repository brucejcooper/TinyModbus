/* 
  DALI master, useable as a multi-switch controller.  Programmed (via UPDI, or UART?) to respond to button presses
  by sending out DALI commands to dim 
*/

#ifndef __DALI_H__
#define __DALI_H__

#include <inttypes.h>
#include <stdbool.h>
#include <avr/io.h>

typedef enum { 
    DALI_OK = 0, 
    DALI_ERR_COLLISION_DETECTED,
    DALI_ERR_INVALID_STATE,
    DALI_ERR_NO_START_BIT,
    DALI_ERR_NOT_FOR_ME,
    DALI_ERR_CORRUPT_READ,
    DALI_ERR_QUEUEFULL,
} dali_result_t;

#define DALI_CMD_GET_LEVEL 0x1a0
#define DALI_CMD_QUERY_STATUS 0x190
#define DALI_CMD_GOTO_LAST_ACTIVE 0x10a
#define DALI_CMD_GET_MIN 0x1a2
#define DALI_CMD_SET_MIN 0x12b
#define DALI_CMD_GET_MAX 0x1a1
#define DALI_CMD_SET_MAX 0x12a
#define DALI_CMD_GET_FADE 0x1a5
#define DALI_CMD_SET_FADE_TIME 0x12e
#define DALI_CMD_GET_EXT_FADE 0x1a8
#define DALI_CMD_SET_FADE_RATE 0x12f
#define DALI_CMD_GET_POWER_ON_LEVEL 0x1a3
#define DALI_CMD_SET_POWER_ON_LEVEL 0x12d
#define DALI_CMD_GET_POWER_FAIL_LEVEL 0x1a4
#define DALI_CMD_SET_POWER_FAIL_LEVEL 0x12c
#define DALI_CMD_GET_GROUPS0_7 0x1c0
#define DALI_CMD_GET_GROUPS8_15 0x1c1
#define DALI_CMD_ADD_TO_GROUP 0x160
#define DALI_CMD_REMOVE_FROM_GROUP 0x170
#define DALI_CMD_OFF 0x100
#define DALI_CMD_SET_DTR0 0xa300



#define DALI_BAUD                   (1200)
#define DALI_BIT_USECS              (1000000.0/DALI_BAUD)
#define DALI_HALF_BIT_USECS         (DALI_BIT_USECS/2.0)
#define DALI_MAX_DEVICES            (64)
#define DALI_DECODE_ADDR(cmd)       (((cmd) >> 9) & 0x3F)


// The DALI configuration - PB3 for transmit - PB2 for receive.
#define DALI_PORT PORTB
#define DALI_TX_bm PIN0_bm
#define DALI_RX_bm PIN1_bm
#define DALI_RX_PINCTRL PIN1CTRL

typedef enum {
  DALI_RESPONSE_OK = 0,
  DALI_RESPONSE_NAK = 404,
  DALI_RESPONSE_COLLISION = 409,
  DALI_RESPONSE_MALFORMED = 500,
  DALI_RESPONSE_PULSE_TOO_SHORT,
  DALI_RESPONSE_PULSE_IN_MIDDLE,
  DALI_RESPONSE_PULSE_TOO_LONG,
  DALI_RESPONSE_SHORT = 600,
  DALI_RESPONSE_PENDING = 255,
} dali_response_type_t;


typedef union {
    uint16_t shortVal;
    struct {
        // ATMEL is little endian
        uint8_t low;
        uint8_t high;
    };
} modbus_register_t;

#define DALI_STATUS_BANK 0
#define DALI_MINMAX_BANK 1
#define DALI_FADE_BANK 2
#define DALI_POWER_BANK 3
#define DALI_GROUPS_BANK 4
#define DALI_NUM_REG_BANKS 5

typedef struct {
    modbus_register_t banks[DALI_NUM_REG_BANKS][DALI_MAX_DEVICES];
} dali_all_holding_registers_t;

struct dali_command_t;

typedef void (*dali_callback_t)(dali_response_type_t responseType, uint16_t cmd, uint8_t lastResponse);
typedef bool (*dali_callback_next_t)(struct dali_command_t *cmd, dali_response_type_t responseType, uint8_t response);


typedef struct dali_command_t {
    uint16_t cmd;
    dali_callback_t callback;
    uint8_t repeat;
} dali_command_t;



extern dali_all_holding_registers_t dali_holding_regs;
#define DALI_MAX_HOLDING_REGISTER (sizeof(dali_holding_regs) / sizeof(uint16_t))

extern void pulse_led(uint8_t times);

extern void dali_init();
uint8_t dali_is_bus_shorted();


// State management - each of these functions lives in its own source file. 
void dali_idle_state_enter();
extern void dali_transmitting_state_enter(dali_command_t *cmd);
void dali_wait_for_idle_state_enter();

void dali_call_on_rising_once(void (*callback)());
void dali_call_on_low_once(void (*callback)());
void dali_cancel_edge_detect();


// Receiving state.
void dali_state_receiving_prepare();
void dali_state_receiving_enter();


void dali_wait_for_response_state_enter();
extern volatile bool dali_idle;
void process_command_response(dali_response_type_t err, uint8_t res);
void dali_state_init();


// Commmands
extern void dali_start_background_scan(uint8_t addr);
extern void dali_goto_last_active(uint8_t addr, dali_callback_t callback);
extern void dali_off(uint8_t addr, dali_callback_t callback);
extern void dali_toggle(uint8_t addr, dali_callback_t callback);
extern void dali_set_level(uint8_t addr, uint8_t level, dali_callback_t callback);
extern void dali_read_level_and_status(uint8_t addr, dali_callback_t callback);
extern void dali_custom_command(uint16_t cmd, dali_callback_t responseCallback, uint8_t repeat);



extern void dali_set_min_level(uint8_t addr, uint8_t val, dali_callback_t callback);
extern void dali_set_max_level(uint8_t addr, uint8_t val, dali_callback_t callback);
extern void dali_set_fade_rate(uint8_t addr, uint8_t val, dali_callback_t callback);
extern void dali_set_fade_time(uint8_t addr, uint8_t val, dali_callback_t callback);
extern void dali_set_power_on_level(uint8_t addr, uint8_t val, dali_callback_t callback);
extern void dali_set_power_fail_level(uint8_t addr, uint8_t val, dali_callback_t callback);
extern void dali_add_to_group(uint8_t addr, uint8_t group, dali_callback_t callback);

extern void dali_remove_from_group(uint8_t addr, uint8_t group, dali_callback_t callback);

#endif
